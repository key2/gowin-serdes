"""LUNA SuperSpeed multi-endpoint bulk loopback on the gw_usb3 PHY (DK_USB).

Concurrency vehicle for the open Gen1 stack: a vendor-specific device
with THREE independent bulk OUT -> bulk IN loopback pairs (EP1..EP3),
each buffered by a 16 KiB BSRAM elastic FIFO.  Exercises the shared
protocol machinery under simultaneous multi-endpoint traffic: the
endpoint-mux TX packet lock, the kind-granular handshake arbiter, the
per-endpoint NRDY/ERDY flow control, and interleaved per-EP sequence
spaces (HANDOVER 10j).

Validated in simulation first: sim_link_loopback.py (luna-loopback/)
runs the same endpoint pairs against a raw-wire link-partner host model
with NUM_EPS=2..4, including LBAD/bad-header fault injection.

Build & program:

    python top.py                # build only (build/)
    python top.py serdes         # regenerate serdes.toml/csr only
    python top.py flash          # flash the existing bitstream
    python top.py program        # build + flash

Host test (needs pyusb):

    sudo python multiep_test.py [MiB-per-EP]

Debug UARTs as in luna-loopback: ttyUSB4 link probe, ttyUSB5 EP1
IN-ladder probe.  LED = link trained (U0).
"""

import importlib.util
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXAMPLE_ROOT = HERE.parent                      # gw5at-60-dkusb/
WORKSPACE = HERE.parents[3]                     # GW_USB3/
LUNA_ROOT = WORKSPACE / "luna"

sys.path.insert(0, str(HERE))
sys.path.insert(0, str(EXAMPLE_ROOT / "luna-acm"))   # ss_stream_out
sys.path.insert(0, str(EXAMPLE_ROOT))           # dk_usb_gw5at60
sys.path.insert(0, str(WORKSPACE / "gowin-serdes"))
sys.path.insert(0, str(WORKSPACE))              # gw_usb3
sys.path.insert(0, str(LUNA_ROOT))              # luna

from amaranth.hdl import *
from amaranth.lib.cdc import FFSynchronizer
from amaranth.lib.fifo import SyncFIFOBuffered

from dk_usb_gw5at60 import DKUSBGW5AT60Platform, add_serdes_refclk_forward

from gowin_serdes import GowinDevice, make_usb3_serdes, usb3_boot_writes
from gowin_serdes.config import RefClkSource
from gowin_serdes.usb3 import attach_usb3_phy

from luna.gateware.interface.serdes_phy.gowin_gtr12 import GowinGTR12PIPE
from luna.gateware.usb.usb3.device import USBSuperSpeedDevice
from luna.gateware.usb.usb3.endpoints.stream import SuperSpeedStreamInEndpoint
from luna.gateware.usb.usb3.protocol.layer import TxDataSkidBuffer
from usb_protocol.emitters import SuperSpeedDeviceDescriptorCollection

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

# Loopback endpoint pairs (OUT 0x0n / IN 0x8n).  Two pairs are verified on
# hardware (16 MiB/EP simultaneous, 69.5 MB/s per direction each).
#
# THREE pairs: the SET_ADDRESS enumeration failure of HANDOVER 10j is
# FIXED (GowinSynthesis all-ones-sentinel constant-fold in the endpoint
# mux, HANDOVER 10k) and a 3-pair build now enumerates cleanly and moves
# 114 MB/s aggregate across all three pipes -- but SUSTAINED 3-pipe
# traffic hits a NEW intermittent failure (-71 EPROTO on two IN pipes
# within ~µs of each other, link stays in U0, ~50% of 1 MiB/EP runs;
# open item #23, HANDOVER 10k).  Flip to (1, 2, 3) when working on it.
BULK_EPS = (1, 2)
FIFO_WORDS = 4096           # per-pair elastic buffer: 16 KiB


def make_serdes():
    return make_usb3_serdes(GowinDevice.GW5AT_60, QUAD, LANE,
                            REF_CLK_SOURCE, REF_CLK_FREQ,
                            boot_rate=BOOT_RATE)


def create_descriptors():
    """Vendor-specific device: one interface, three bulk OUT/IN pairs."""
    descriptors = SuperSpeedDeviceDescriptorCollection()

    with descriptors.DeviceDescriptor() as d:
        d.bDeviceClass       = 0xFF        # vendor specific
        d.idVendor           = 0x1209      # pid.codes
        d.idProduct          = 0x0001      # test PID
        d.bcdUSB             = 3.2
        d.bMaxPacketSize0    = 9           # 2**9 = 512
        d.iManufacturer      = "LUNA + gw_usb3"
        d.iProduct           = "GTR12 SuperSpeed multi-EP loopback"
        d.iSerialNumber      = "DK60"
        d.bNumConfigurations = 1

    with descriptors.ConfigurationDescriptor() as c:
        c.bMaxPower = 50

        with c.InterfaceDescriptor() as i:
            i.bInterfaceNumber   = 0
            i.bInterfaceClass    = 0xFF    # vendor specific
            i.bInterfaceSubclass = 0x00
            i.bInterfaceProtocol = 0x00

            for ep in BULK_EPS:
                with i.EndpointDescriptor(add_default_superspeed=True) as e:
                    e.bEndpointAddress = 0x80 | ep
                    e.bmAttributes     = 0x02  # bulk
                    e.wMaxPacketSize   = 1024

                with i.EndpointDescriptor(add_default_superspeed=True) as e:
                    e.bEndpointAddress = ep
                    e.bmAttributes     = 0x02  # bulk
                    e.wMaxPacketSize   = 1024

    return descriptors


class LunaMultiEpTop(Elaboratable):
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
        # LUNA's ss/sync reset is released only once the adapter reports
        # the boot rate switch COMPLETE (phy_ready): no LUNA state ever
        # clocks at the 156.25 MHz boot rate or through the rate-change
        # pclk retune -- the boot window corrupted idle-but-clocking LUNA
        # registers (dead TP generator path at 3 endpoint pairs; HANDOVER
        # 10k).  The adapter sequences the bring-up itself in the
        # reset-free ss_raw domain, started by the POR chain.
        m.d.comb += adapter.boot_start.eq(luna_go)
        rstn_r0 = Signal()
        rstn_r1 = Signal()
        m.d.ss_raw += [rstn_r0.eq(luna_go & adapter.phy_ready),
                       rstn_r1.eq(rstn_r0)]
        for dom in ("ss", "sync"):
            m.domains += ClockDomain(dom)
            m.d.comb += [
                ClockSignal(dom).eq(ClockSignal("ss_raw")),
                ResetSignal(dom).eq(~rstn_r1),
            ]

        # ==============================================================
        # LUNA SuperSpeed device: three bulk loopback pairs
        # ==============================================================
        m.submodules.usb = usb = USBSuperSpeedDevice(
            phy=adapter, sync_frequency=125e6)

        control_ep = usb.add_standard_control_endpoint(create_descriptors())

        in_eps = {}
        out_eps = {}
        for ep in BULK_EPS:
            out_ep = SuperSpeedStreamOutEndpoint(
                endpoint_number=ep, max_packet_size=1024)
            usb.add_endpoint(out_ep)
            out_eps[ep] = out_ep

            in_ep = SuperSpeedStreamInEndpoint(
                endpoint_number=ep, max_packet_size=1024,
                generate_zlps=False)
            usb.add_endpoint(in_ep)
            in_eps[ep] = in_ep

            # Per-pair elastic loopback buffer (see luna-loopback/top.py for
            # the rationale; 16 KiB each keeps three pairs affordable).
            fifo = SyncFIFOBuffered(width=32 + 4 + 1, depth=FIFO_WORDS)
            m.submodules[f"loop_fifo{ep}"] = DomainRenamer({"sync": "ss"})(fifo)

            out_ep.packet_space = Signal(name=f"packet_space{ep}")
            m.d.comb += [
                # OUT endpoint -> FIFO
                fifo.w_data.eq(Cat(out_ep.stream.payload,
                                   out_ep.stream.valid,
                                   out_ep.stream.last)),
                fifo.w_en.eq(out_ep.stream.valid.any() & fifo.w_rdy),
                out_ep.stream.ready.eq(fifo.w_rdy),

                # FIFO -> IN endpoint
                in_ep.stream.payload.eq(fifo.r_data[0:32]),
                in_ep.stream.valid.eq(Mux(fifo.r_rdy, fifo.r_data[32:36], 0)),
                in_ep.stream.last.eq(fifo.r_data[36]),
                fifo.r_en.eq(fifo.r_rdy & in_ep.stream.ready),

            ]
            # Accept a data packet only when a whole max-size packet fits.
            # Registered: the wide level comparison otherwise sits in the
            # endpoint's accept/handshake cone (the margin absorbs staleness).
            m.d.ss += out_ep.packet_space.eq((FIFO_WORDS - fifo.level) >= 260)

        m.d.comb += led.o.eq(usb.link_trained)

        # ==============================================================
        # Debug UARTs (identical layout to luna-loopback)
        # ==============================================================
        rx_com = Signal()
        m.d.comb += [
            rx_com.eq(adapter.rx_datavalid & adapter.rx_datak[0]
                      & (adapter.rx_data[0:8] == 0xBC)),
        ]

        # uart 0: link-event probe.
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

        # uart 1: control-transfer (SET_ADDRESS) path probe.
        #
        # Localizes where the status ACK dies during the failing 3-pair
        # enumeration.  Per second (0.35 s gate), a healthy SET_ADDRESS
        # attempt contributes: ch0 +2 (SETUP ack + STATUS ack strobed at
        # the control endpoint), ch1 +1 (STATUS TP received), ch2 +2
        # (generator accepted both ACKs), ch3 +2 (both TPs entered the
        # link header queue).  ch4 counts ss cycles in which the control
        # interface has *latched* (fast-path-demoting) pending requests:
        # any nonzero value means the same-cycle SET_ADDRESS contract is
        # at risk; a huge value means a stuck pending bit.
        # uart 1: control/handshake-path health probe.  Per gate window a
        # healthy control transfer contributes: ch0 (any handshake
        # dispatched) and ch2 (ACK dispatched) counts, ch1 counts received
        # STATUS TPs, ch3 counts TPs entering the link queue; ch4 counts
        # cycles the control interface has latched (fast-path-demoting)
        # pending requests -- large values indicate the HANDOVER-10k
        # arbiter wedge.
        any_dispatch = Signal()
        status_rx = Signal()
        ack_dispatch = Signal()
        tp_accepted = Signal()
        pending0 = Signal()
        recovery = Signal()
        wrong_addr_ack = Signal()
        m.d.ss += [
            any_dispatch.eq(usb.debug_hsk_any_dispatch),
            status_rx.eq(usb.debug_status_received),
            ack_dispatch.eq(usb.debug_hsk_ack_dispatch),
            tp_accepted.eq(usb.debug_tp_hdr_accepted),
            pending0.eq(usb.debug_hsk_pending0),
            recovery.eq(usb.debug_recovery),
            wrong_addr_ack.eq(usb.debug_hsk_ep0_dispatch
                              & usb.debug_address_nonzero),
        ]

        sticky_bits = Signal(3)
        with m.If(pending0):
            m.d.ss += sticky_bits[0].eq(1)
        with m.If(recovery):
            m.d.ss += sticky_bits[1].eq(1)
        with m.If(wrong_addr_ack):
            m.d.ss += sticky_bits[2].eq(1)
        sticky_src = Cat(sticky_bits, usb.link_trained)

        uart1 = platform.request("uart", 1)
        m.submodules.clkprobe = clkprobe = DomainRenamer({"cfg": "dbg"})(
            ClockFreqProbe(clk_freq=DBG_FREQ, baud=BAUD_RATE, channels=(
                ("ss", any_dispatch),
                ("ss", status_rx),
                ("ss", ack_dispatch),
                ("ss", tp_accepted),
                ("ss", pending0),
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
    platform.build(LunaMultiEpTop(), name="luna_multiep", build_dir="build",
                   do_program=do_program)


def flash():
    bitstream = HERE / "build" / "luna_multiep.fs"
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
