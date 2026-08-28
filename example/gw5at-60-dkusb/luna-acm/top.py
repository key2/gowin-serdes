"""LUNA SuperSpeed CDC-ACM echo device on the gw_usb3 PHY (DK_USB, GW5AT-60).

The first DATA-carrying device on the fully open Gen1 stack: everything
luna-enum has (LUNA LTSSM/link/protocol on the gw_usb3 PIPE PHY over
gowin-serdes, 10G boot + boot-time rate switch to 5G), plus a CDC-ACM
function whose bulk OUT endpoint is looped back into its bulk IN
endpoint.  The host sees a /dev/ttyACM* serial port; every byte written
is echoed back -- validating the complete bulk datapath in BOTH
directions (DP reception w/ CRC-32 + sequence/ACK generation on OUT,
DP transmission + IN-token handling on IN) through link, protocol and
endpoint layers.

    ep0     control + CDC class requests ({GET,SET}_LINE_CODING,
            SET_CONTROL_LINE_STATE)
    ep1 OUT -> ep1 IN   bulk 1024B echo loopback
    ep2 IN  interrupt notification endpoint (idle; NRDYs politely)

The SS OUT endpoint gateware does not exist upstream in LUNA; it lives
here as ss_stream_out.py.

Build & program:

    python top.py                # build only (build/)
    python top.py serdes         # regenerate serdes.toml/csr only
    python top.py flash          # flash the existing bitstream
    python top.py program        # build + flash

Test:

    dmesg | grep ttyACM          # cdc_acm binds -> /dev/ttyACM0
    stty -F /dev/ttyACM0 raw -echo
    cat /dev/ttyACM0 &           # reader
    printf 'hello superspeed' > /dev/ttyACM0
    # -> 'hello superspeed' comes back

Debug UARTs are identical to luna-enum (ttyUSB4 link probe / ttyUSB5
clock+symbol probe); LED = link trained (U0).
"""

import importlib.util
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXAMPLE_ROOT = HERE.parent                      # gw5at-60-dkusb/
WORKSPACE = HERE.parents[3]                     # GW_USB3/
LUNA_ROOT = WORKSPACE / "luna"

sys.path.insert(0, str(HERE))                   # ss_stream_out
sys.path.insert(0, str(EXAMPLE_ROOT))           # dk_usb_gw5at60
sys.path.insert(0, str(WORKSPACE / "gowin-serdes"))
sys.path.insert(0, str(WORKSPACE))              # gw_usb3
sys.path.insert(0, str(LUNA_ROOT))              # luna

from amaranth.hdl import *
from amaranth.lib.cdc import FFSynchronizer

from dk_usb_gw5at60 import DKUSBGW5AT60Platform, add_serdes_refclk_forward

from gowin_serdes import GowinDevice, make_usb3_serdes, usb3_boot_writes
from gowin_serdes.config import RefClkSource
from gowin_serdes.usb3 import attach_usb3_phy

from luna.gateware.interface.serdes_phy.gowin_gtr12 import GowinGTR12PIPE
from luna.gateware.usb.usb3.device import USBSuperSpeedDevice
from luna.gateware.usb.usb3.endpoints.stream import SuperSpeedStreamInEndpoint
from luna.gateware.usb.usb3.application.request import SuperSpeedRequestHandler

from usb_protocol.types import USBRequestType
from usb_protocol.emitters import SuperSpeedDeviceDescriptorCollection
from usb_protocol.emitters.descriptors.cdc import (
    ACMFunctionalDescriptorEmitter,
    CallManagementFunctionalDescriptorEmitter,
    HeaderDescriptorEmitter,
    UnionFunctionalDescriptorEmitter,
)

from ss_stream_out import SuperSpeedStreamOutEndpoint

# Reuse the debug probe from the (vendor-stack) enum example.
_spec = importlib.util.spec_from_file_location(
    "usb31_enum_top", EXAMPLE_ROOT / "usb31-enum" / "top.py")
_enum_top = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_enum_top)
ClockFreqProbe = _enum_top.ClockFreqProbe

# ── Configuration ─────────────────────────────────────────────────────
QUAD, LANE = 0, 1
REF_CLK_SOURCE = RefClkSource.Q0_REFCLK1
REF_CLK_FREQ = "200M"
DBG_FREQ = 24_000_000
BAUD_RATE = 115_200
BOOT_RATE = "10G"           # proven: 10G boot + adapter rate switch to 5G

BULK_EP = 1                 # data endpoints (OUT 0x01 / IN 0x81)
NOTIF_EP = 2                # notification endpoint (IN 0x82)


def make_serdes():
    return make_usb3_serdes(GowinDevice.GW5AT_60, QUAD, LANE,
                            REF_CLK_SOURCE, REF_CLK_FREQ,
                            boot_rate=BOOT_RATE)


def create_descriptors():
    """CDC-ACM (single function): comm interface 0 + data interface 1."""
    descriptors = SuperSpeedDeviceDescriptorCollection()

    with descriptors.DeviceDescriptor() as d:
        d.bDeviceClass       = 0x02         # Communications
        d.idVendor           = 0x1209       # pid.codes
        d.idProduct          = 0x0001       # test PID
        d.bcdUSB             = 3.2
        d.bMaxPacketSize0    = 9            # 2**9 = 512
        d.iManufacturer      = "LUNA + gw_usb3"
        d.iProduct           = "GTR12 SuperSpeed ACM echo"
        d.iSerialNumber      = "DK60"
        d.bNumConfigurations = 1

    with descriptors.ConfigurationDescriptor() as c:
        c.bMaxPower = 50

        # Communications (control) interface, with the CDC functional
        # descriptors that let cdc_acm find its data interface.
        with c.InterfaceDescriptor() as i:
            i.bInterfaceNumber   = 0
            i.bInterfaceClass    = 0x02     # CDC
            i.bInterfaceSubclass = 0x02     # ACM
            i.bInterfaceProtocol = 0x01     # AT commands (conventional)

            header = HeaderDescriptorEmitter()
            i.add_subordinate_descriptor(header)

            call_mgmt = CallManagementFunctionalDescriptorEmitter()
            call_mgmt.bDataInterface = 1
            i.add_subordinate_descriptor(call_mgmt)

            acm = ACMFunctionalDescriptorEmitter()
            acm.bmCapabilities = 0b0010     # line coding + control line state
            i.add_subordinate_descriptor(acm)

            union = UnionFunctionalDescriptorEmitter()
            union.bControlInterface      = 0
            union.bSubordinateInterface0 = 1
            i.add_subordinate_descriptor(union)

            with i.EndpointDescriptor(add_default_superspeed=True) as e:
                e.bEndpointAddress = 0x80 | NOTIF_EP
                e.bmAttributes     = 0x03   # interrupt
                e.wMaxPacketSize   = 64
                e.bInterval        = 9      # 2**(9-1) * 125 us = 32 ms

        # Data interface: the bulk echo pair.
        with c.InterfaceDescriptor() as i:
            i.bInterfaceNumber   = 1
            i.bInterfaceClass    = 0x0A     # CDC data
            i.bInterfaceSubclass = 0x00
            i.bInterfaceProtocol = 0x00

            with i.EndpointDescriptor(add_default_superspeed=True) as e:
                e.bEndpointAddress = 0x80 | BULK_EP
                e.bmAttributes     = 0x02   # bulk
                e.wMaxPacketSize   = 1024

            with i.EndpointDescriptor(add_default_superspeed=True) as e:
                e.bEndpointAddress = BULK_EP
                e.bmAttributes     = 0x02   # bulk
                e.wMaxPacketSize   = 1024

    return descriptors


class ACMRequestHandler(SuperSpeedRequestHandler):
    """Minimal CDC-ACM class-request handler.

    Follows the request-handler pattern of
    luna/examples/usb/superspeed/vendor_request.py: everything is gated
    on ``setup.type == CLASS`` so it composes with the standard handler.

    Handles:
      SET_LINE_CODING (0x20)        -- 7-byte OUT data stage, stored;
      GET_LINE_CODING (0x21)        -- 7-byte IN data stage, returned;
      SET_CONTROL_LINE_STATE (0x22) -- no-data, acknowledged (DTR/RTS
                                        exposed on :attr:`line_state`);
      anything else CLASS-typed     -- STALLed.
    """

    SET_LINE_CODING        = 0x20
    GET_LINE_CODING        = 0x21
    SET_CONTROL_LINE_STATE = 0x22

    def __init__(self):
        super().__init__()
        self.line_state = Signal(2)     # {RTS, DTR}

    def elaborate(self, platform):
        m = Module()

        interface = self.interface
        setup     = interface.setup
        handshakes = interface.handshakes_out

        # Line coding storage: dwDTERate / bCharFormat / bParityType /
        # bDataBits.  Default: 115200 8N1.
        coding_word0 = Signal(32, init=115_200)          # dwDTERate (LE)
        coding_word1 = Signal(24, init=0x08_00_00)       # stop/parity/bits

        # GET_LINE_CODING send-side state: which of the two words is next.
        tx_word  = Signal()
        tx_valid = Signal(4)

        with m.If(setup.type == USBRequestType.CLASS):

            with m.Switch(setup.request):

                # ── SET_LINE_CODING: accept + store 7 bytes ─────────────
                with m.Case(self.SET_LINE_CODING):

                    with m.If(interface.rx.valid.any()):
                        with m.If(interface.rx.first):
                            m.d.ss += coding_word0.eq(interface.rx.payload)
                        with m.Else():
                            m.d.ss += coding_word1.eq(
                                interface.rx.payload[0:24])

                    with m.If(interface.rx_complete | interface.rx_invalid):
                        m.d.comb += [
                            handshakes.retry_required
                                .eq(interface.rx_invalid),
                            handshakes.next_sequence.eq(1),
                            handshakes.send_ack.eq(1),
                        ]

                    with m.If(interface.status_requested):
                        m.d.comb += [
                            handshakes.next_sequence.eq(1),
                            handshakes.send_ack.eq(1),
                        ]

                # ── GET_LINE_CODING: return the 7 bytes ─────────────────
                with m.Case(self.GET_LINE_CODING):

                    m.d.comb += [
                        interface.tx.valid  .eq(tx_valid),
                        interface.tx.first  .eq(~tx_word),
                        interface.tx.last   .eq(tx_word),
                        interface.tx.payload.eq(Mux(
                            tx_word, coding_word1, coding_word0)),
                        interface.tx_length .eq(7),
                    ]

                    with m.If(interface.data_requested):
                        m.d.ss += [tx_word.eq(0), tx_valid.eq(0b1111)]

                    with m.If(tx_valid.any() & interface.tx.ready):
                        with m.If(~tx_word):                 # word 0 taken
                            m.d.ss += [tx_word.eq(1),
                                       tx_valid.eq(0b0111)]  # 3 bytes left
                        with m.Else():                       # word 1 taken
                            m.d.ss += tx_valid.eq(0)

                    with m.If(interface.status_requested):
                        m.d.comb += [
                            handshakes.next_sequence.eq(1),
                            handshakes.send_ack.eq(1),
                        ]

                # ── SET_CONTROL_LINE_STATE: latch DTR/RTS, ack ──────────
                with m.Case(self.SET_CONTROL_LINE_STATE):

                    with m.If(interface.status_requested):
                        m.d.ss   += self.line_state.eq(setup.value[0:2])
                        m.d.comb += [
                            handshakes.next_sequence.eq(1),
                            handshakes.send_ack.eq(1),
                        ]

                # ── anything else class-typed: STALL ────────────────────
                with m.Default():
                    stall_opportunity = (
                        interface.rx_complete
                        | interface.rx_invalid
                        | interface.data_requested
                        | interface.status_requested
                    )
                    with m.If(stall_opportunity):
                        m.d.comb += handshakes.send_stall.eq(1)

        return m


class LunaAcmTop(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        led = platform.request("led", 0)
        usb_pwr_en = platform.request("usb_pwr_en", 0)
        m.d.comb += usb_pwr_en.o.eq(0)     # device mode: never source VBUS

        # Quiet the USB2 pins (floating pins = phantom LS device).
        usb2 = platform.request("usb2_softphy", 0)
        m.d.comb += [
            usb2.pullup_en.o.eq(0),
            usb2.term_dp.o.eq(0),
            usb2.term_dn.o.eq(0),
            usb2.tx_dp.o.eq(0),
            usb2.tx_dn.o.eq(0),
            usb2.dp.oe.eq(0),
            usb2.dp_b.oe.eq(0),
            usb2.dn.oe.eq(0),
            usb2.dn_b.oe.eq(0),
        ]

        # ==============================================================
        # Clocks & power-on reset (vendor-proven ordering)
        # ==============================================================
        sys_clk = add_serdes_refclk_forward(m, platform)
        m.domains += ClockDomain("cfg", reset_less=True)
        m.d.comb += ClockSignal("cfg").eq(sys_clk)

        clk24 = platform.request("clk_24m", 0)
        m.domains += ClockDomain("dbg", reset_less=True)
        m.d.comb += ClockSignal("dbg").eq(clk24.i)

        por_cnt = Signal(18)
        por_n = Signal()
        luna_go = Signal()
        with m.If(~por_cnt.all()):
            m.d.cfg += por_cnt.eq(por_cnt + 1)

        # Board key = full POR replay (LTSSM restart without reflash).
        key = platform.request("key", 0)
        key_s = Signal(2)
        m.d.cfg += key_s.eq(Cat(key.i, key_s[0]))
        with m.If(key_s[1]):
            m.d.cfg += por_cnt.eq(0)

        m.d.cfg += [
            por_n.eq(por_cnt > 66_000),
            luna_go.eq(por_cnt.all()),
        ]

        # ==============================================================
        # SerDes + PHY + PIPE adapter
        # ==============================================================
        serdes, group = make_serdes()
        m.submodules.serdes = serdes
        lane = group.lanes[0]
        drp = getattr(serdes, group.drp_name)
        m.d.comb += serdes.por_n.eq(por_n)

        adapter = GowinGTR12PIPE(boot_rate_switch=(BOOT_RATE == "10G"),
                                 boot_domain="ss_raw")
        m.submodules.adapter = adapter
        attach_usb3_phy(m, adapter.phy, lane, drp)

        m.domains += ClockDomain("ss_raw", reset_less=True)
        m.d.comb += ClockSignal("ss_raw").eq(lane.tx.pcs_clkout)
        rstn_r0 = Signal()
        rstn_r1 = Signal()
        # LUNA's ss/sync reset is released only once the adapter reports
        # the boot rate switch COMPLETE (phy_ready): no LUNA state ever
        # clocks at the 156.25 MHz boot rate or through the rate-change
        # pclk retune (boot-window corruption class; HANDOVER 10k).  The
        # adapter sequences the bring-up itself in the reset-free ss_raw
        # domain, started by the POR chain.
        m.d.comb += adapter.boot_start.eq(luna_go)
        m.d.ss_raw += [rstn_r0.eq(luna_go & adapter.phy_ready),
                       rstn_r1.eq(rstn_r0)]
        for dom in ("ss", "sync"):
            m.domains += ClockDomain(dom)
            m.d.comb += [
                ClockSignal(dom).eq(ClockSignal("ss_raw")),
                ResetSignal(dom).eq(~rstn_r1),
            ]

        # ==============================================================
        # LUNA SuperSpeed device: CDC-ACM with bulk echo
        # ==============================================================
        m.submodules.usb = usb = USBSuperSpeedDevice(
            phy=adapter, sync_frequency=125e6)

        control_ep = usb.add_standard_control_endpoint(create_descriptors())
        control_ep.add_request_handler(ACMRequestHandler())

        # Bulk echo: OUT endpoint drains straight into the IN endpoint.
        out_ep = SuperSpeedStreamOutEndpoint(
            endpoint_number=BULK_EP, max_packet_size=1024)
        usb.add_endpoint(out_ep)

        # generate_zlps=False: the echo delimits every received packet with
        # ``last`` (packet-sized "transfers"), and a tty byte stream needs no
        # transfer framing -- ZLP follow-ups after every max-size packet
        # would only cost bus turnarounds.
        in_ep = SuperSpeedStreamInEndpoint(
            endpoint_number=BULK_EP, max_packet_size=1024,
            generate_zlps=False)
        usb.add_endpoint(in_ep)

        m.d.comb += in_ep.stream.stream_eq(out_ep.stream)

        # Notification endpoint: declared in the descriptors, never has
        # data -- the stream-in endpoint NRDYs IN tokens politely.
        notif_ep = SuperSpeedStreamInEndpoint(
            endpoint_number=NOTIF_EP, max_packet_size=64)
        usb.add_endpoint(notif_ep)
        m.d.comb += notif_ep.stream.valid.eq(0)

        m.d.comb += led.o.eq(usb.link_trained)

        # ==============================================================
        # Debug UARTs (identical layout to luna-enum)
        # ==============================================================
        rx_com = Signal()
        rx_d102 = Signal()
        tx_d102 = Signal()
        m.d.comb += [
            rx_com.eq(adapter.rx_datavalid & adapter.rx_datak[0]
                      & (adapter.rx_data[0:8] == 0xBC)),
            rx_d102.eq(adapter.rx_datavalid & (adapter.rx_datak[0:4] == 0)
                       & (adapter.rx_data[0:16] == 0x4A4A)),
            tx_d102.eq(~adapter.tx_elec_idle & (adapter.tx_datak[0:4] == 0)
                       & (adapter.tx_data[0:16] == 0x4A4A)),
        ]

        # uart 0: link-event probe.
        # C <ss-freq> <lfps_det> <ts1_det> <ts2_det> <rx_com> <flag>
        uart0 = platform.request("uart", 0)
        m.submodules.linkprobe = linkprobe = DomainRenamer({"cfg": "dbg"})(
            ClockFreqProbe(clk_freq=DBG_FREQ, baud=BAUD_RATE, channels=(
                ("ss", None),
                ("ss", usb.debug_lfps_polling_detected),
                ("ss", usb.debug_ts1_detected),
                ("ss", usb.debug_ts2_detected),
                ("ss", rx_com),
            )))
        link_flags = Signal(4)
        m.submodules += FFSynchronizer(
            Cat(usb.link_trained, usb.link_in_reset,
                usb.debug_phy_ready, usb.debug_engage_terminations),
            link_flags, o_domain="dbg")
        m.d.comb += [
            linkprobe.flags.eq(link_flags),
            uart0.tx.o.eq(linkprobe.tx_o),
        ]
        rx0_unused = Signal()
        m.submodules += FFSynchronizer(uart0.rx.i, rx0_unused,
                                       o_domain="dbg")

        # uart 1: IN-endpoint transaction-ladder probe.
        # C <in-tokens> <nrdy> <erdy> <ep-tx-words> <echo-words> <flag>
        # flag = {erdy_done_seen, in_token_seen, ep_tx_seen, link_trained}
        # All probe taps are registered first so the debug logic never adds
        # combinational load to protocol-critical cones.
        in_if = in_ep.interface
        in_token = Signal()
        ep_tx_word = Signal()
        echo_word = Signal()
        erdy_done = Signal()
        m.d.ss += [
            in_token.eq(
                in_if.handshakes_in.ack_received
                & (in_if.handshakes_in.endpoint_number == BULK_EP)
                & (in_if.handshakes_in.number_of_packets != 0)),
            ep_tx_word.eq(in_if.tx.valid.any() & in_if.tx.ready),
            echo_word.eq(out_ep.stream.valid.any() & out_ep.stream.ready),
            erdy_done.eq(in_if.handshakes_out.send_erdy
                         & in_if.handshakes_out.done),
        ]

        sticky_bits = Signal(3)
        with m.If(erdy_done):
            m.d.ss += sticky_bits[0].eq(1)
        with m.If(in_token):
            m.d.ss += sticky_bits[1].eq(1)
        with m.If(ep_tx_word):
            m.d.ss += sticky_bits[2].eq(1)
        sticky_src = Cat(sticky_bits, usb.link_trained)

        uart1 = platform.request("uart", 1)
        m.submodules.clkprobe = clkprobe = DomainRenamer({"cfg": "dbg"})(
            ClockFreqProbe(clk_freq=DBG_FREQ, baud=BAUD_RATE, channels=(
                ("ss", in_token),
                ("ss", in_if.handshakes_out.send_nrdy),
                ("ss", in_if.handshakes_out.send_erdy),
                ("ss", ep_tx_word),
                ("ss", echo_word),
            )))
        dbg_flags = Signal(4)
        m.submodules += FFSynchronizer(sticky_src, dbg_flags, o_domain="dbg")
        m.d.comb += [
            clkprobe.flags.eq(dbg_flags),
            uart1.tx.o.eq(clkprobe.tx_o),
        ]
        rx1_unused = Signal()
        m.submodules += FFSynchronizer(uart1.rx.i, rx1_unused,
                                       o_domain="dbg")

        return m


# ======================================================================
# Build entry point
# ======================================================================

def generate_serdes_files():
    serdes, _ = make_serdes()
    toml_path = HERE / "serdes.toml"
    csr_path = HERE / "serdes.csr"
    serdes.generate_csr(output_path=str(csr_path), toml_path=str(toml_path),
                        extra_writes=usb3_boot_writes(QUAD, LANE))
    print(f"Generated {toml_path.name} / {csr_path.name} ({BOOT_RATE} boot)")


def _setup_gowin_env(platform):
    import os
    os.environ.setdefault("LD_PRELOAD",
                          "/usr/lib/x86_64-linux-gnu/libfreetype.so.6")
    os.environ.setdefault("LD_LIBRARY_PATH",
                          str(Path(platform.gowin_path) / "IDE" / "lib"))
    gowin_bin = str(Path(platform.gowin_path) / "IDE" / "bin")
    if gowin_bin not in os.environ.get("PATH", ""):
        os.environ["PATH"] = gowin_bin + os.pathsep + os.environ["PATH"]


def build(do_program=False):
    generate_serdes_files()
    platform = DKUSBGW5AT60Platform()
    _setup_gowin_env(platform)
    platform.add_file("serdes.csr", (HERE / "serdes.csr").read_text())
    platform.build(LunaAcmTop(), name="luna_acm", build_dir="build",
                   do_program=do_program)


def flash():
    bitstream = HERE / "build" / "luna_acm.fs"
    if not bitstream.exists():
        sys.exit(f"no bitstream at {bitstream}; run `python top.py` first")
    cmd = ["openFPGALoader", "-c", "ft232", str(bitstream)]
    try:
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError:
        subprocess.check_call(["sudo", "-n"] + cmd)


if __name__ == "__main__":
    if "serdes" in sys.argv:
        generate_serdes_files()
    elif "flash" in sys.argv:
        flash()
    else:
        build(do_program="program" in sys.argv)
