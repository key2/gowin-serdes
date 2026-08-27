"""USB 3.1 SuperSpeed enumeration bring-up — Gowin DK_USB (GW5AT-LV60UG225).

Same stack as the Tang Mega 138K example, retargeted to the board the Gowin
USB3.1 reference design actually ships for — with a properly wired Type-C
connector:

    [gowin-serdes]   GTR12 Quad 0 lane 1, CPLL, 200 MHz refclk on Q0
                     REFPAD1 (fabric-forwarded from the 200 MHz LVDS
                     oscillator, exactly like the reference top.sv),
                     10G / 16-bit / 1:4 boot trim.
    [gw_usb3]        Usb31Phy with its *default* CSR configuration --
                     UparCsrConfig() == CPLL / Q0 / LN1 / 200M, the very
                     configuration every table is pinned against.
    [vendor stack]   usb3 pipe/ltssm/link netlists + protocol/EP0 RTL +
                     reference UVC descriptors (usb31_enum_core.sv shim).
    [debug]          UART reporter on the wired J18 MIPI debug UART
                     ("uart" 0 = /dev/ttyUSB4, 115200 8N1) + the board
                     LED showing "attached".

This board's SerDes configuration is the reference design's own: the
generated ``serdes.toml`` must match ``serdes_tmp.toml`` of the reference
project and the generated ``serdes.csr`` must match its shipped ``serdes.csr``
byte-for-byte (both are enforced by the test suite and re-checked at build
time here).

Rate strategy: full Gen2 PHY datapath (``GEN2_DATAPATH = True``), like the
vendor design.  VERIFIED ON HARDWARE 2026-08-26: enumerates as
``030a:0301 Gowin UVC`` at **SuperSpeed Plus Gen 2x1 (10 Gbit)** on a
Gen2 host port.  (A Gen1-only datapath wedges in Polling.PortMatch
against Gen2 hosts -- see the GEN2_DATAPATH comment.)

Build & program:

    python top.py                # build only (build/)
    python top.py program        # + openFPGALoader -c ft232 (Mini USB-B port)
    python top.py flash          # flash the existing bitstream only

Watch the link: ``stty -F /dev/ttyUSB4 115200 raw -echo; cat /dev/ttyUSB4``
(U-line bits [21:16] = LTSSM state: 10=Polling.LFPS, 12=PortMatch,
16=Polling.Active, 19=U0/attached, 23=Compliance).
"""

import sys
from pathlib import Path

from amaranth.hdl import *
from amaranth.lib.cdc import FFSynchronizer

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent.parent.parent          # GW_USB3 checkout root
REFDESIGN = REPO / "Gowin_USB3.1_UVC_BULK_RefDesign" / "prj" / "src"

sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))                 # platform file
sys.path.insert(0, str(HERE.parent.parent.parent))   # gowin-serdes/
sys.path.insert(0, str(REPO))                        # gw_usb3 package

from dk_usb_gw5at60 import DKUSBGW5AT60Platform, add_serdes_refclk_forward
from uart import AsyncSerialTX
from usb_debug import UsbDebugReporter

from gowin_serdes import GowinDevice, make_usb3_serdes, usb3_boot_writes
from gowin_serdes.config import RefClkSource
from gowin_serdes.usb3 import attach_usb3_phy

from gw_usb3 import Usb31Phy, UparCsrConfig

# ── Board / SerDes configuration ─────────────────────────────────────
# Debug UARTs are clocked from the 24 MHz oscillator ("dbg" domain --
# proven by the uart-hello example, and trivial to close timing on).
# MEASURED ON HARDWARE (ClockFreqProbe, 2026-08-26): the GTR12 "life
# clock" (upar/DRP/CK_AHB) is a free-running ring oscillator wandering
# ~56..118 MHz -- it can clock handshake logic but never a fixed-baud
# UART.  (The vendor top.sdc's 10 ns constraint is a bound, not a rate.)
DBG_FREQ = 24_000_000
BAUD_RATE = 115_200
DIVISOR = DBG_FREQ // BAUD_RATE

QUAD = 0
LANE = 1          # Type-C straight-orientation pair = Q0 lane 1

# The ported PHY's default CSR configuration IS this board's configuration
# (CPLL, quad 0, lane 1, 200 MHz refclk) -- every handshake address and
# rate-change value is pinned against the vendor sources for it.
PHY_CSR = UparCsrConfig()
assert PHY_CSR.quad == QUAD and PHY_CSR.lane == LANE

# Include the Gen2 (10G, 128b/132b) datapath in the PHY, as in the vendor
# design.  The reduced gen2=False build wedges in Polling.PortMatch (LTSSM
# state 12) against a Gen2 host: the LTSSM netlist always advertises Gen2
# and the capability negotiation needs datapath-side signals (observed on
# hardware 2026-08-26; our gen2=True PHY transplanted into the vendor
# project enumerated at Gen2 SuperSpeed+ on the same port).
GEN2_DATAPATH = True

# Debug UART: "uart" 0 = hand-wired FT4232H channel C on the J18 MIPI
# GPIOs = /dev/ttyUSB4 on the bench (verified with the uart-hello example);
# alternative: "uart_j21" = TX on the J21 header SCL pin (excludes i2c).
UART_RESOURCE = "uart"

# SerDes: the USB3 recipe from gowin_serdes.usb3 -- boots the lane in the
# Gen2 trim (10G/16-bit/1:4) exactly like the reference design, 200 MHz on
# Q0 REFPAD1.  The generated serdes.toml/csr are byte-identical to the
# reference design's (verified at build time below and by the test suite).
REF_CLK_SOURCE = RefClkSource.Q0_REFCLK1
REF_CLK_FREQ = "200M"

VENDOR_CTRL = REFDESIGN / "usb3_2_device_controller"
VENDOR_USER = REFDESIGN / "usb3_2_user_layer"
VENDOR_SERDES = REFDESIGN / "serdes"

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
    # NOTE: the RAM16SDP*/distributed-ram usage inside these files needs
    # the undocumented `set_option -enable_dsrm 1` (distributed SRAM);
    # see the platform add_options.
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
    return make_usb3_serdes(GowinDevice.GW5AT_60, QUAD, LANE,
                            REF_CLK_SOURCE, REF_CLK_FREQ, boot_rate="10G")


class ClockFreqProbe(Elaboratable):
    """Frequency counter for SerDes-derived clocks, reported over UART.

    Elaborates in a domain called ``cfg``; rename with ``DomainRenamer``
    to place it on any stable reference clock (the enum top runs it on
    the 24 MHz oscillator).  Each measured clock drives a free-running
    counter snapshot via a request/ack handshake CDC; every
    2^gate_bits reference cycles the deltas are printed as
    ``C <pclk> <rxclk> <upar> <flags>\\r\\n`` (7 hex digits each).
    f[Hz] = delta * f_ref / 2^gate_bits.

    Rationale: the upar/DRP "life clock" of the GTR12 was measured to be
    a free-running ring oscillator (~56..118 MHz); this probe measured it
    (and shows the pclk 156.25 -> 125 MHz rate switch live).
    """

    WIDTH = 28

    def __init__(self, clk_freq, baud=115_200, gate_bits=23,
                 channels=(("pclk", None), ("rxprobe", None),
                           ("upar", None))):
        """`channels`: (domain, event) pairs.  event=None counts clock
        cycles (frequency); an event Signal counts its assertions in that
        domain (event rate)."""
        self._divisor = clk_freq // baud
        self._gate_bits = gate_bits
        self._channels = list(channels)
        self.flags = Signal(4)          # cfg domain; appended as hex digit
        self.tx_o = Signal(init=1)

    def elaborate(self, platform):
        m = Module()

        tx = AsyncSerialTX(divisor=self._divisor)
        m.submodules.tx = DomainRenamer("cfg")(tx)
        m.d.comb += self.tx_o.eq(tx.o)

        # -- per-clock counters, capture-and-hold handshake CDC ----------
        # A request toggle (cfg) makes each source domain latch its
        # free-running counter into a holding register and answer with an
        # ack toggle; cfg then reads the *stable* snapshot.  No decode
        # logic runs at speed and no gray-skew assumptions are needed.
        deltas = []
        gate = Signal(self._gate_bits)
        m.d.cfg += gate.eq(gate + 1)
        snapshot = gate == 0            # ~0.35 s at 24 MHz / 23 bits
        req = Signal()
        with m.If(snapshot):
            m.d.cfg += req.eq(~req)

        for ci, (dom, event) in enumerate(self._channels):
            tag = f"ch{ci}_{dom}"
            cnt = Signal(self.WIDTH, name=f"cnt_{tag}")
            if event is None:
                m.d[dom] += cnt.eq(cnt + 1)
            else:
                with m.If(event):
                    m.d[dom] += cnt.eq(cnt + 1)

            req_s = Signal(name=f"req_{tag}_s")
            m.submodules += FFSynchronizer(req, req_s, o_domain=dom)
            req_d = Signal(name=f"req_{tag}_d")
            snap = Signal(self.WIDTH, name=f"snap_{tag}")
            ack = Signal(name=f"ack_{tag}")
            m.d[dom] += req_d.eq(req_s)
            with m.If(req_s != req_d):
                m.d[dom] += [snap.eq(cnt), ack.eq(~ack)]

            ack_s = Signal(name=f"ack_{tag}_s")
            m.submodules += FFSynchronizer(ack, ack_s, o_domain="cfg")
            ack_d = Signal(name=f"ack_{tag}_d")
            m.d.cfg += ack_d.eq(ack_s)
            last = Signal(self.WIDTH, name=f"last_{tag}")
            delta = Signal(self.WIDTH, name=f"delta_{tag}")
            pending = Signal(name=f"pending_{tag}")
            with m.If(snapshot):
                m.d.cfg += pending.eq(1)
                with m.If(pending):     # no ack since last request:
                    m.d.cfg += delta.eq(0)   # the clock is dead -> read 0
            with m.If(ack_s != ack_d):
                # snap has been stable for >= 2 cfg cycles (synchronizer
                # latency); safe multi-bit read.
                m.d.cfg += [last.eq(snap), delta.eq(snap - last),
                            pending.eq(0)]
            deltas.append(delta)

        # -- line formatter: 'C' + one 7-digit hex group per clock,
        #    then ' ' + one flags digit -------------------------------
        flags_l = Signal(4)
        with m.If(snapshot):
            m.d.cfg += flags_l.eq(self.flags)

        DIGITS = self.WIDTH // 4
        n = len(deltas)
        per = 1 + DIGITS                      # ' ' + digits
        total = 1 + n * per + 2 + 2           # 'C' + groups + ' '+flag + CR LF
        idx = Signal(range(total))
        active = Signal()

        char = Signal(8)
        with m.Switch(idx):
            with m.Case(0):
                m.d.comb += char.eq(ord("C"))
            for gi, delta in enumerate(deltas):
                base = 1 + gi * per
                with m.Case(base):
                    m.d.comb += char.eq(ord(" "))
                for j in range(DIGITS):
                    nib = delta[(DIGITS - 1 - j) * 4:(DIGITS - j) * 4]
                    with m.Case(base + 1 + j):
                        m.d.comb += char.eq(
                            Mux(nib < 10, ord("0") + nib,
                                ord("a") - 10 + nib))
            with m.Case(1 + n * per):
                m.d.comb += char.eq(ord(" "))
            with m.Case(1 + n * per + 1):
                m.d.comb += char.eq(
                    Mux(flags_l < 10, ord("0") + flags_l,
                        ord("a") - 10 + flags_l))
            with m.Case(total - 2):
                m.d.comb += char.eq(ord("\r"))
            with m.Case(total - 1):
                m.d.comb += char.eq(ord("\n"))

        with m.If(active):
            m.d.comb += [tx.data.eq(char), tx.ack.eq(1)]
            with m.If(tx.rdy):
                with m.If(idx == total - 1):
                    m.d.cfg += active.eq(0)
                with m.Else():
                    m.d.cfg += idx.eq(idx + 1)
        with m.Elif(snapshot):
            m.d.cfg += [active.eq(1), idx.eq(0)]

        return m


class Usb31EnumTop(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        uart = platform.request(UART_RESOURCE, 0)
        led = platform.request("led", 0)
        usb_pwr_en = platform.request("usb_pwr_en", 0)
        m.d.comb += usb_pwr_en.o.eq(0)   # device mode: never source VBUS

        # Quiet the USB2 pins on the shared Type-C connector.  Left
        # floating, the pull-up switch / termination FETs let the host see
        # a phantom (broken) low-speed device; it then endlessly
        # power-cycles the port, disrupting SuperSpeed training
        # (observed: "usb 3-2.1 new low-speed device" + "attempt power
        # cycle" loops in the host kernel log).
        usb2 = platform.request("usb2_softphy", 0)
        m.d.comb += [
            usb2.pullup_en.o.eq(0),      # no 1.5k on D+ -> no USB2 device
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
        # Clocking: forward the 200 MHz oscillator to Q0 REFPAD1 exactly
        # like the reference top.sv, and run the power-on sequencing
        # counters on the same 200 MHz fabric clock it returns.
        # ==============================================================
        sys_clk = add_serdes_refclk_forward(m, platform)
        m.domains += ClockDomain("cfg", reset_less=True)
        m.d.comb += ClockSignal("cfg").eq(sys_clk)

        # Debug clock domain: 24 MHz oscillator (uarts, reporters).
        clk24 = platform.request("clk_24m", 0)
        m.domains += ClockDomain("dbg", reset_less=True)
        m.d.comb += ClockSignal("dbg").eq(clk24.i)

        # Reset ordering (matches the vendor top.sv, which is hardware-
        # proven -- and is the OPPOSITE of what this file used to do):
        #   1. ~1.3 us   base reset releases (vendor cnt0 == 255);
        #   2. ~330 us   SerDes quad POR (por_n) releases (vendor
        #                cnt1 == 65535);
        #   3. ~1.3 ms   controller/PHY reset (phy_resetn) releases LAST
        #                (vendor gates it on pll_init lock; a fixed delay
        #                long after por_n serves the same purpose here).
        # Releasing the PHY *before* the quad leaves POR silently loses
        # the CSR sequencer's one-shot init writes (CDR/FFE/TX trims):
        # rx-detect and rate changes still work (runtime writes), but the
        # host never recognizes our LFPS/TS -- exactly the failure mode
        # this design exhibited before the fix.
        por_cnt = Signal(18)
        phy_resetn = Signal()      # controller + PHY reset (active low)
        por_n = Signal()           # SerDes quad power-on reset (active low)
        with m.If(~por_cnt.all()):
            m.d.cfg += por_cnt.eq(por_cnt + 1)

        # Board key = full power-on-reset replay (SerDes POR + controller
        # + PHY).  Recovery without reflashing: the LTSSM was observed
        # wedged in Loopback after the host abandoned a marginal link.
        # Key is PinsN: .i reads 1 while pressed; holding it pins the POR
        # counter at 0, releasing it replays the whole reset sequence.
        key = platform.request("key", 0)
        key_s = Signal(2)
        m.d.cfg += key_s.eq(Cat(key.i, key_s[0]))
        with m.If(key_s[1]):
            m.d.cfg += por_cnt.eq(0)

        m.d.cfg += [
            por_n.eq(por_cnt > 66_000),      # ~330 us: quad POR release
            phy_resetn.eq(por_cnt.all()),    # ~1.3 ms: PHY comes up LAST
        ]

        # ==============================================================
        # SerDes (gowin-serdes; TOML/CSR proven against the reference)
        # ==============================================================
        serdes, group = make_serdes()
        m.submodules.serdes = serdes
        lane = group.lanes[0]
        drp = getattr(serdes, group.drp_name)
        m.d.comb += serdes.por_n.eq(por_n)

        # ==============================================================
        # Amaranth USB3.1 PHY (gw_usb3)
        # ==============================================================
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
            i_phy_pwrpresent=C(1, 1),     # VBUS assumed present (device)
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
            o_dbg_ltssm_state=(ltssm_state := Signal(6)),

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
        # Debug: UART reporter ("dbg" 24 MHz domain -- see DBG_FREQ
        # note) + LED = attached
        # ==============================================================
        # The reporter was written for the 138K with its FSM in a domain
        # called "upar"; rename that onto the stable 24 MHz debug clock
        # (the real upar life clock is an unstable ring oscillator).
        dbg = DomainRenamer({"upar": "dbg"})(UsbDebugReporter(divisor=DIVISOR))
        m.submodules.dbg = dbg

        # uart 1 (/dev/ttyUSB5): clock-frequency probe -- measures the
        # SerDes-derived clocks against the 200 MHz oscillator.  The
        # flags digit carries the Type-C CC pin levels: the host's Rp
        # pull-up sits on the CC line of the *plugged* orientation, so
        # flag bit0 (CC1) = straight (our Q0 lane 1 pair), bit1 (CC2) =
        # flipped (wired to the unsupported TX2/RX2 pair).
        uart1 = platform.request("uart", 1)
        m.domains += ClockDomain("rxprobe", reset_less=True)
        m.d.comb += ClockSignal("rxprobe").eq(lane.rx.pcs_clkout)

        # PIPE-level ordered-set counters (pclk domain, Gen1 16-bit view:
        # symbols in data[15:0] / K[1:0]).  rx_com counts K28.5 (0xBC)
        # comma symbols leaving our PHY RX (8b/10b decode + alignment
        # proof); rx_d102 / tx_d102 count D10.2-pair words (0x4a4a: TS1
        # body OR CP1 compliance pattern -- COM-less runs mean CP1).
        rx_com = Signal()
        rx_d102 = Signal()
        tx_d102 = Signal()
        m.d.comb += [
            rx_com.eq(phy.PipeRxDataValid[0]
                      & phy.PipeRxDataK[0]
                      & (phy.PipeRxData[0:8] == 0xBC)),
            rx_d102.eq(phy.PipeRxDataValid[0]
                       & (phy.PipeRxDataK[0:2] == 0)
                       & (phy.PipeRxData[0:16] == 0x4A4A)),
            tx_d102.eq(pipe_tx_valid
                       & (pipe_tx_datak[0:2] == 0)
                       & (pipe_tx_data[0:16] == 0x4A4A)),
        ]

        # LFPS-handshake instrumentation (pclk domain edge counters):
        #  tx_lfps    -- falling edges of the controller's TxElecIdle =
        #                LFPS bursts we emit;
        #  rxei_ctrl  -- falling edges of the PHY's RxElecIdle output =
        #                LFPS bursts the *controller* gets to see (the
        #                post-mux signal gating Polling.LFPS!);
        #  rxei_lane  -- falling edges of the lane-level rx_elecidle
        #                (raw SerDes signal-detect view).
        lane_rxei_p = Signal(2)
        m.d.pclk += lane_rxei_p.eq(
            Cat(lane.status.rx_elecidle, lane_rxei_p[0]))
        tx_ei_d = Signal()
        rxei_d = Signal()
        m.d.pclk += [tx_ei_d.eq(tx_elecidle), rxei_d.eq(phy.RxElecIdle)]
        tx_lfps = Signal()
        rxei_ctrl = Signal()
        rxei_lane = Signal()
        m.d.comb += [
            tx_lfps.eq(tx_ei_d & ~tx_elecidle),
            rxei_ctrl.eq(rxei_d & ~phy.RxElecIdle),
            rxei_lane.eq(lane_rxei_p[1] & ~lane_rxei_p[0]),
        ]

        # Line: C <pclk> <rx_com> <rx_d102> <tx_d102> <tx_lfps>
        #         <rxei_ctrl> <rxei_lane> <flags>
        m.submodules.clkprobe = clkprobe = DomainRenamer({"cfg": "dbg"})(
            ClockFreqProbe(clk_freq=DBG_FREQ, baud=BAUD_RATE, channels=(
                ("pclk", None),
                ("pclk", rx_com),
                ("pclk", rx_d102),
                ("pclk", tx_d102),
                ("pclk", tx_lfps),
                ("pclk", rxei_ctrl),
                ("pclk", rxei_lane),
            )))
        pd_sync = Signal(2)
        m.submodules += FFSynchronizer(power_down, pd_sync, o_domain="dbg")
        txei_sync = Signal()
        m.submodules += FFSynchronizer(tx_elecidle, txei_sync,
                                       o_domain="dbg")
        rxei_sync = Signal()
        m.submodules += FFSynchronizer(phy.RxElecIdle, rxei_sync,
                                       o_domain="dbg")
        m.d.comb += [
            clkprobe.flags.eq(Cat(pd_sync, txei_sync, rxei_sync)),
            uart1.tx.o.eq(clkprobe.tx_o),
        ]
        rx1_unused = Signal()
        m.submodules += FFSynchronizer(uart1.rx.i, rx1_unused,
                                       o_domain="dbg")

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
            dbg.mon_ltssm_state.eq(ltssm_state),
            dbg.mon_request_stb.eq(request_active),
            dbg.mon_bmRequestType.eq(bm_request_type),
            dbg.mon_bRequest.eq(b_request),
            dbg.mon_wValue.eq(w_value),
            dbg.mon_wIndex.eq(w_index),
            dbg.mon_wLength.eq(w_length),
            uart.tx.o.eq(dbg.tx_o),
            led.o.eq(attached),          # LED on = LTSSM reached U0
        ]
        if hasattr(uart, "rx"):
            rx_unused = Signal()
            m.submodules += FFSynchronizer(uart.rx.i, rx_unused,
                                           o_domain="dbg")

        return m


# ======================================================================
# Build entry point
# ======================================================================

def _read(path: Path) -> str:
    return path.read_text()


def generate_serdes_files(check_against_reference=True):
    """Generate serdes.toml / serdes.csr and verify against the reference.

    The DK_USB board runs the exact SerDes configuration of the Gowin
    reference design, so the generated TOML must equal its
    ``serdes_tmp.toml`` and the CSR (with the two vendor boot writes) must
    equal its shipped ``serdes.csr`` byte-for-byte.
    """
    serdes, _ = make_serdes()
    toml_path = HERE / "serdes.toml"
    csr_path = HERE / "serdes.csr"

    serdes.generate_csr(output_path=str(csr_path), toml_path=str(toml_path),
                        extra_writes=usb3_boot_writes(QUAD, LANE))
    print(f"Generated {toml_path.name} / {csr_path.name}")

    if check_against_reference:
        ref_toml = (VENDOR_SERDES / "serdes_tmp.toml").read_text()
        ref_csr = (VENDOR_SERDES / "serdes.csr").read_text()
        ours_toml = toml_path.read_text()
        ours_csr = csr_path.read_text()

        norm = lambda s: s.replace("\r\n", "\n").rstrip("\n")  # noqa: E731
        if norm(ours_toml) == norm(ref_toml):
            print("serdes.toml == reference serdes_tmp.toml (exact match)")
        else:
            import difflib
            diff = list(difflib.unified_diff(
                norm(ref_toml).splitlines(), norm(ours_toml).splitlines(),
                "reference/serdes_tmp.toml", "generated/serdes.toml",
                lineterm=""))
            raise AssertionError(
                "generated TOML differs from the reference design:\n"
                + "\n".join(diff[:80]))
        if norm(ours_csr) == norm(ref_csr):
            print("serdes.csr  == reference serdes.csr      (exact match)")
        else:
            raise AssertionError("generated CSR differs from the reference")

    return toml_path, csr_path


def build(do_program=False):
    import os
    platform = DKUSBGW5AT60Platform()

    # Gowin-on-Linux quirks (see the 138K example).
    os.environ.setdefault("LD_PRELOAD",
                          "/usr/lib/x86_64-linux-gnu/libfreetype.so.6")
    os.environ.setdefault("LD_LIBRARY_PATH",
                          str(Path(platform.gowin_path) / "IDE" / "lib"))
    gowin_bin = str(Path(platform.gowin_path) / "IDE" / "bin")
    if gowin_bin not in os.environ.get("PATH", ""):
        os.environ["PATH"] = gowin_bin + os.pathsep + os.environ["PATH"]

    # ---- SerDes CSR/TOML, verified against the reference design --------
    try:
        _, csr_path = generate_serdes_files()
    except FileNotFoundError as exc:
        csr_path = HERE / "serdes.csr"
        if not csr_path.exists():
            raise
        print(f"Gowin tool not found ({exc}); using existing {csr_path.name}")
    platform.add_file("serdes.csr", csr_path.read_bytes())

    # ---- local shim ----------------------------------------------------
    platform.add_file("usb31_enum_core.sv", _read(HERE / "usb31_enum_core.sv"))

    # ---- vendor controller + user layer --------------------------------
    # The RAM16SDP*/distributed-ram usage needs `set_option -enable_dsrm 1`
    # (distributed SRAM); the platform sets it.  Without it the IDE
    # reports RP0007 "no SSRAM resource in current device" -- which had
    # been misread as a die limitation and worked around with register
    # equivalents (destroying pclk timing).
    for name in CTRL_FILES:
        content = _read(VENDOR_CTRL / name)
        if name == "usb3_2_device_controller.sv":
            # Debug tap: bring the (syn_keep) binary-coded LTSSM state out
            # as a port so the UART reporter can show the exact substate.
            content = content.replace(
                ",output wire 			phy_ltssm_is_training",
                ",output wire 			phy_ltssm_is_training\n"
                ",output wire [5:0]		dbg_ltssm_state", 1)
            content = content.replace(
                "endmodule",
                "assign dbg_ltssm_state = ltssm_state;\nendmodule", 1)
            assert "dbg_ltssm_state" in content
        platform.add_file(name, content)
    platform.add_file("UVCDefine.v", _read(REFDESIGN / "UVCDefine.v"))
    platform.add_file("UserLayer_top.sv",
                      _read(VENDOR_USER / "UserLayer_top.sv"))

    ep2 = _read(VENDOR_USER / "ep2_IN.v")
    ep2 = ep2.replace('"../UVCDefine.v"', '"UVCDefine.v"')
    platform.add_file("ep2_IN.v", ep2)

    ct = _read(VENDOR_USER / "ControlTransfer.v")
    ct = ct.replace('"../UVCDefine.v"', '"UVCDefine.v"')
    ct = ct.replace("./../../prj/src/usb3_2_user_layer/descriptors/",
                    "descriptors/")
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


def flash():
    import subprocess
    bitstream = HERE / "build" / "usb31_enum.fs"
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
