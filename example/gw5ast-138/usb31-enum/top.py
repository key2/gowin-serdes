"""USB 3.1 SuperSpeed enumeration bring-up — Tang Mega 138K Pro (GW5AST-138).

Minimal stack to validate the Amaranth USB3 PHY port on real hardware:

    [gowin-serdes]   GTR12 quad, Q0 lane 0, CPLL, 125 MHz refclk (Q1 REFPAD0,
                     inter-quad routing), boots in the Gen2 trim
                     (10G / 16-bit / 1:4) exactly like the Gowin reference
                     design; rate changes happen at runtime through the
                     PHY's CSR sequencer.
    [gw_usb3]        Usb31Phy — the Amaranth port of Gowin's USB3.1 PHY
                     (8b10b, alignment, elastic buffer, LFPS, PIPE, upar CSR).
    [vendor stack]   usb3 pipe/ltssm/link netlists + protocol/EP0 RTL +
                     reference UVC descriptors (via usb31_enum_core.sv shim).
    [debug]          UART reporter (status word + every SETUP request) and
                     LED status.

Rate strategy: the vendor LTSSM advertises Gen2 (SCD) during Polling.LFPS
and falls back to Gen1 per the USB 3.1 spec when the host does not respond
or Gen2 training fails.  Whenever the negotiated rate differs from the
current PHY rate, the controller toggles ``phy_rate`` and the PHY
reconfigures the SerDes (10G<->5G) over the UPAR bus.  Because the SerDes
boots in the Gen2 trim, the 10G->5G reconfiguration runs on *every* Gen1
link-up — watch bit 5 of the UART status word for the active rate.  Do NOT
assume the link mounts at 10G: start bring-up on a USB 3.0 (5 Gbit) host
port or through a 5 Gbit hub.

Board wiring assumptions (Tang Mega 138K Pro):
    * USB-C SuperSpeed pairs on SerDes Quad 1 lane 0;
    * 125 MHz SerDes reference clock on Q1 REFPAD0 (Q1 REFPAD1 carries
      100 MHz -- also supported by the vendor CSR tables if preferred);
    * UART on the J3 FT4232 header (A19 tx / A18 rx), 115200 8N1;
    * VBUS is assumed present (no Type-C controller handling here).

Build & program:

    python top.py                # build only (build/)
    python top.py program        # build + program via openFPGALoader
"""

import re
import sys
from pathlib import Path

from amaranth.hdl import *
from amaranth.lib.cdc import FFSynchronizer

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent.parent.parent          # GW_USB3 checkout root
REFDESIGN = REPO / "Gowin_USB3.1_UVC_BULK_RefDesign" / "prj" / "src"

sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent.parent.parent))   # gowin-serdes/
sys.path.insert(0, str(REPO))                        # gw_usb3 package

from gw5ast_dvk import GW5ASTDVKPlatform
from usb_debug import UsbDebugReporter

from gowin_serdes import GowinDevice, make_usb3_serdes, usb3_boot_writes
from gowin_serdes.config import RefClkSource
from gowin_serdes.usb3 import attach_usb3_phy

from gw_usb3 import Usb31Phy, UparCsrConfig, PllType, RefClk

# ── Board / SerDes configuration ─────────────────────────────────────
UPAR_FREQ = 62_500_000
BAUD_RATE = 115_200
DIVISOR = UPAR_FREQ // BAUD_RATE

QUAD = 1
LANE = 0          # USB-C SuperSpeed pairs on Q1 lane 0

# The PHY's CSR sequencer address map must match the chosen quad/lane.
PHY_CSR = UparCsrConfig(pll=PllType.CPLL, quad=QUAD, lane=LANE,
                        refclk=RefClk.F125M)

# Include the Gen2 (10G, 128b/132b) datapath in the PHY.  False = Gen1-only
# bring-up: smaller and faster; Gen2 hosts fall back to 5 Gbit per spec.
GEN2_DATAPATH = False

# SerDes: the USB3 recipe from gowin_serdes.usb3 -- Gen2 boot trim
# (10G/16-bit/1:4) like the reference design; 125 MHz reference on Q1
# REFPAD0 (this board; REFPAD1 carries 100 MHz, also supported by the
# vendor CSR tables).
REF_CLK_SOURCE = RefClkSource.Q0_REFCLK0
REF_CLK_FREQ = "125M"

VENDOR_CTRL = REFDESIGN / "usb3_2_device_controller"
VENDOR_USER = REFDESIGN / "usb3_2_user_layer"

CTRL_FILES = [
    "usb3_macro_define.v",
    "usb3_const.vh",
    "usb3_2_device_controller.sv",
    "usb3_protocol.sv",
    "usb3_ep0.v",
    "usb3_ep.v",
    "usb3_ep0in_ram.v",
    "transfer_in_mem.v",
    "transfer_in_mem_ping_pong.v",
    "transfer_out_mem.v",
    "transfer_out_mem_ping_pong.v",
    "usb3_pipe.vg",
    "usb3_ltssm_device.vg",
    "usb3_link.vg",
]
DESCRIPTORS = [
    "device_descriptor.dat", "configuration_descriptor.dat",
    "bos_descriptor_GEN1.dat", "bos_descriptor_GEN2.dat",
    "string0_descriptor.dat", "string1_descriptor.dat",
    "string2_descriptor.dat", "stringMSFT_descriptor.dat",
    "usb_descrip_define.v",
]


def make_serdes():
    return make_usb3_serdes(GowinDevice.GW5AST_138, QUAD, LANE,
                            REF_CLK_SOURCE, REF_CLK_FREQ, boot_rate="10G")


class Usb31EnumTop(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        clk50 = platform.request("clk50")
        uart = platform.request("uart", 0)
        leds = [platform.request("led", i) for i in range(6)]

        # ==============================================================
        # Power-on sequencing (raw 50 MHz oscillator domain, no reset)
        # -- mirrors the reference top.sv: release the controller reset
        # first (~5 us), then the SerDes POR (~1.3 ms later).
        # ==============================================================
        m.domains += ClockDomain("cfg", reset_less=True)
        m.d.comb += ClockSignal("cfg").eq(clk50.i)

        por_cnt = Signal(18)
        phy_resetn = Signal()      # controller + PHY reset (active low)
        por_n = Signal()           # SerDes quad power-on reset (active low)
        with m.If(~por_cnt.all()):
            m.d.cfg += por_cnt.eq(por_cnt + 1)
        m.d.cfg += [
            phy_resetn.eq(por_cnt > 255),
            por_n.eq(por_cnt.all()),
        ]

        # ==============================================================
        # SerDes (gowin-serdes, unproven — treat with suspicion)
        # ==============================================================
        serdes, group = make_serdes()
        m.submodules.serdes = serdes
        lane = group.lanes[0]
        drp = getattr(serdes, group.drp_name)
        m.d.comb += serdes.por_n.eq(por_n)

        # ==============================================================
        # Amaranth USB3.1 PHY (gw_usb3)
        # ==============================================================
        # Gen1-only PHY: the 128b/132b datapath is omitted (GEN2_DATAPATH
        # switch below).  Gen2 SCD negotiation and the 10G<->5G SerDes rate
        # change still work (they live in the LTSSM / upar_csr); only Gen2
        # *training/data* framing is unavailable, so a Gen2-capable host
        # falls back to Gen1 per the USB 3.1 spec -- which is this
        # example's target.  Saves ~3.2k LUTs and the associated timing
        # pressure at the 156.25 MHz Gen2-boot pclk.
        phy = Usb31Phy(csr_config=PHY_CSR, gen2=GEN2_DATAPATH)
        m.submodules.phy = phy

        # complete PHY <-> lane <-> DRP wiring (incl. the deliberately
        # crossed fabric clocks), from the verified gowin_serdes.usb3 table
        attach_usb3_phy(m, phy, lane, drp)

        # pclk fabric domain (frequency changes with the USB rate!)
        m.domains += ClockDomain("pclk", reset_less=True)
        m.d.comb += ClockSignal("pclk").eq(lane.tx.pcs_clkout)

        # ==============================================================
        # Vendor USB3.2 device controller + user layer (SV shim)
        # ==============================================================
        ctrl_phy_reset_n = Signal()
        pipe_tx_data = Signal(64)
        pipe_tx_datak = Signal(4)
        pipe_tx_synchead = Signal(4)
        pipe_tx_startblock = Signal()
        pipe_tx_valid = Signal()
        tx_detrx = Signal()
        tx_elecidle = Signal()
        power_down = Signal(2)
        rx_polarity = Signal()
        rx_termination = Signal()
        rate = Signal()
        elas_buf_mode = Signal()
        ltssm_training = Signal()
        attached = Signal()
        itp_received = Signal()
        warm_hot_reset = Signal()
        request_active = Signal()
        bm_request_type = Signal(8)
        b_request = Signal(8)
        w_value = Signal(16)
        w_index = Signal(16)
        w_length = Signal(16)

        m.submodules.core = Instance(
            "usb31_enum_core",
            i_pclk=ClockSignal("pclk"),
            i_reset_n=phy_resetn,

            i_phy_pipe_rx_data_k=phy.PipeRxDataK,
            i_phy_pipe_rx_data=phy.PipeRxData,
            i_phy_pipe_rx_sync_head=phy.PipeRxSyncHead,
            i_phy_pipe_rx_start_block=phy.PipeRxStartBlock,
            i_phy_pipe_rx_valid=phy.PipeRxDataValid,
            o_phy_pipe_tx_data_k=pipe_tx_datak,
            o_phy_pipe_tx_data=pipe_tx_data,
            o_phy_pipe_tx_sync_head=pipe_tx_synchead,
            o_phy_pipe_tx_start_block=pipe_tx_startblock,
            o_phy_pipe_tx_valid=pipe_tx_valid,

            o_phy_reset_n=ctrl_phy_reset_n,
            o_phy_tx_detrx_lpbk=tx_detrx,
            o_phy_tx_elecidle=tx_elecidle,
            i_phy_rx_elecidle=phy.RxElecIdle,
            i_phy_rx_status=phy.RxStatus,
            o_phy_power_down=power_down,
            i_phy_phy_status=phy.PhyStatus,
            i_phy_pwrpresent=C(1, 1),     # bus-powered board: VBUS present
            o_phy_tx_deemph=Signal(2),
            o_phy_tx_margin=Signal(3),
            o_phy_tx_swing=Signal(),
            o_phy_rx_polarity=rx_polarity,
            o_phy_rx_termination=rx_termination,
            o_phy_rate=rate,
            o_phy_elas_buf_mode=elas_buf_mode,
            i_phy_tx_fifo_wrnum=phy.TxFifoWrNum,
            i_phy_serdes_pll_lock=phy.serdes_pll_lock,
            o_phy_ltssm_is_training=ltssm_training,

            o_attached=attached,
            o_itp_received=itp_received,
            o_warm_or_hot_reset=warm_hot_reset,
            o_request_active=request_active,
            o_bmRequestType=bm_request_type,
            o_bRequest=b_request,
            o_wValue=w_value,
            o_wIndex=w_index,
            o_wLength=w_length,
        )

        m.d.comb += [
            phy.phy_resetn.eq(ctrl_phy_reset_n),
            phy.PipeTxDataK.eq(pipe_tx_datak),
            phy.PipeTxData.eq(pipe_tx_data),
            phy.PipeTxSyncHead.eq(pipe_tx_synchead),
            phy.PipeTxStartBlock.eq(pipe_tx_startblock),
            phy.PipeTxDataValid.eq(pipe_tx_valid),
            phy.TxDetectRx_loopback.eq(tx_detrx),
            phy.TxElecIdle.eq(tx_elecidle),
            phy.PowerDown.eq(power_down),
            phy.RxPolarity.eq(rx_polarity),
            phy.RxTermination.eq(rx_termination),
            phy.Rate.eq(rate),
            phy.ElasticityBufferMode.eq(elas_buf_mode),
            phy.LTSSM_is_Training.eq(ltssm_training),
        ]

        # ==============================================================
        # UART debug reporter ("upar" domain from the SerDes life clock)
        # ==============================================================
        dbg = UsbDebugReporter(divisor=DIVISOR)
        m.submodules.dbg = dbg

        m.d.comb += dbg.mon_status_bits.eq(Cat(
            lane.status.pll_lock,        # 0
            lane.status.ready,           # 1
            lane.status.rx_cdr_lock,     # 2
            lane.status.signal_detect,   # 3
            phy.RxElecIdle,              # 4
            rate,                        # 5
            tx_elecidle,                 # 6
            rx_termination,              # 7
            power_down,                  # 9:8
            tx_detrx,                    # 10
            ltssm_training,              # 11
            attached,                    # 12
            itp_received,                # 13
            warm_hot_reset,              # 14
            phy.PhyStatus,               # 15
        ))
        m.d.comb += [
            dbg.mon_request_stb.eq(request_active),
            dbg.mon_bmRequestType.eq(bm_request_type),
            dbg.mon_bRequest.eq(b_request),
            dbg.mon_wValue.eq(w_value),
            dbg.mon_wIndex.eq(w_index),
            dbg.mon_wLength.eq(w_length),
            uart.tx.o.eq(dbg.tx_o),
        ]

        # UART RX unused (reporter is transmit-only); keep the input synced.
        rx_unused = Signal()
        m.submodules += FFSynchronizer(uart.rx.i, rx_unused, o_domain="upar")

        # ==============================================================
        # LED status (active-low handled by PinsN in the platform)
        # ==============================================================
        heartbeat = Signal(25)
        m.d.upar += heartbeat.eq(heartbeat + 1)
        m.d.comb += [
            leds[0].o.eq(lane.status.pll_lock),
            leds[1].o.eq(lane.status.rx_cdr_lock),
            leds[2].o.eq(ltssm_training),
            leds[3].o.eq(attached),
            leds[4].o.eq(rate),            # ON = Gen2 (10G) active
            leds[5].o.eq(heartbeat[-1]),   # ~2 Hz blink: design alive
        ]

        return m


# ======================================================================
# Build entry point
# ======================================================================

def _read(path: Path) -> str:
    return path.read_text()


def build(do_program=False):
    import os
    platform = GW5ASTDVKPlatform()

    # Gowin-on-Linux quirks: gw_sh links a bundled freetype that clashes
    # with the system fontconfig; preloading the system freetype fixes the
    # missing FT_Done_MM_Var symbol.  The IDE lib dir must win the library
    # search so gw_sh uses its own Qt.
    os.environ.setdefault("LD_PRELOAD",
                          "/usr/lib/x86_64-linux-gnu/libfreetype.so.6")
    os.environ.setdefault("LD_LIBRARY_PATH",
                          str(Path(platform.gowin_path) / "IDE" / "lib"))
    gowin_bin = str(Path(platform.gowin_path) / "IDE" / "bin")
    if gowin_bin not in os.environ.get("PATH", ""):
        os.environ["PATH"] = gowin_bin + os.pathsep + os.environ["PATH"]

    # ---- SerDes CSR (generated from the Python config) ----------------
    # The reference design flow appends two boot writes after TOML->CSR
    # conversion (proven by byte-diffing its shipped serdes.csr):
    #   * a lane TX-AFE tuning register (0x8082f8 + lane*0x100 = 0x00000A02)
    #   * TX electrical idle asserted (the PHY's CSR sequencer boots with
    #     eidle synchronisers at 1 and only rewrites the register on the
    #     first 1->0 transition -- boot state must match).
    serdes, _ = make_serdes()
    csr_path = HERE / "serdes.csr"
    toml_path = HERE / "serdes.toml"
    try:
        serdes.generate_csr(output_path=str(csr_path), toml_path=str(toml_path),
                            extra_writes=usb3_boot_writes(QUAD, LANE))
        print(f"Generated {csr_path.name} / {toml_path.name}")
    except FileNotFoundError as exc:
        if not csr_path.exists():
            raise FileNotFoundError(
                f"serdes.csr missing and Gowin CSR tool unavailable: {exc}"
            ) from exc
        print(f"Gowin tool not found; using existing {csr_path}")
    platform.add_file("serdes.csr", csr_path.read_bytes())

    # ---- local shim ----------------------------------------------------
    platform.add_file("usb31_enum_core.sv", _read(HERE / "usb31_enum_core.sv"))
    platform.add_file("ram16sdp_lutram.v", _read(HERE / "ram16sdp_lutram.v"))

    # ---- vendor controller + user layer -------------------------------
    # The .vg netlists were synthesized for the GW5AT-60 and instantiate
    # RAM16SDP* distributed-RAM primitives; the GW5AST-138 die has no SSRAM,
    # so those instances are renamed to the register-based equivalents in
    # ram16sdp_lutram.v.
    for name in CTRL_FILES:
        content = _read(VENDOR_CTRL / name)
        if name.endswith(".vg"):
            content = re.sub(r"\bRAM16SDP([124])\b", r"usb3_ram16sdp\1",
                             content)
        platform.add_file(name, content)
    platform.add_file("UVCDefine.v", _read(REFDESIGN / "UVCDefine.v"))
    platform.add_file("UserLayer_top.sv",
                      _read(VENDOR_USER / "UserLayer_top.sv"))

    # ep2_IN.v / ControlTransfer.v: retarget the UVCDefine include and the
    # $readmemh descriptor paths from the reference layout to the build dir.
    ep2 = _read(VENDOR_USER / "ep2_IN.v")
    ep2 = ep2.replace('"../UVCDefine.v"', '"UVCDefine.v"')
    platform.add_file("ep2_IN.v", ep2)

    ct = _read(VENDOR_USER / "ControlTransfer.v")
    ct = ct.replace('"../UVCDefine.v"', '"UVCDefine.v"')
    ct = ct.replace("./../../prj/src/usb3_2_user_layer/descriptors/",
                    "descriptors/")
    # The GW5AST-138 die has no SSRAM (distributed LUT-RAM); the reference
    # design forces its tiny descriptor ROMs into distributed_ram (fine on
    # the GW5AT-60 it targeted).  All ROM reads are registered, so mapping
    # them to plain registers is timing-equivalent.
    ct = ct.replace('syn_ramstyle="distributed_ram"',
                    'syn_ramstyle="registers"')
    platform.add_file("ControlTransfer.v", ct)

    for name in DESCRIPTORS:
        platform.add_file(f"descriptors/{name}",
                          _read(VENDOR_USER / "descriptors" / name))

    platform.build(
        Usb31EnumTop(),
        name="usb31_enum",
        build_dir="build",
        do_program=do_program,
    )


if __name__ == "__main__":
    build(do_program="program" in sys.argv)
