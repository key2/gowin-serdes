"""SerDes DRP Bridge — read/write SerDes UPAR registers over UART.

Targets the **GW5AST-138** (Sipeed Tang Mega 138K Pro) board.

Uses the gowin_serdes pure-Amaranth implementation (no Gowin encrypted IP).
The entire design runs on the UPAR life-clock (~62.5 MHz) from the GTR12
hard macro, so there is no clock domain crossing.

The 125 MHz SerDes PLL reference clock enters on Q1's dedicated REFPAD0
and is propagated to Q0 via inter-quad clock routing (refimux0_sel=2).
The 50 MHz board oscillator on P16 is NOT used by fabric logic.

Binary protocol (all multi-byte values MSB-first, 115200 baud):

  Read:   'R' A2 A1 A0                      -> D3 D2 D1 D0 STATUS
  Write:  'W' A2 A1 A0 D3 D2 D1 D0          -> STATUS
  Status: 'S'                                -> 1 byte (io_hash)
  Ping:   'T'                                -> 0x55

  STATUS: 0x00 = OK, 0x01 = DRP resp error, 0xFF = timeout

Anti-sweep strategy (from DRP_NOTES.md / upar/ project):
  1. All lane inputs driven from fabric (held in reset)
  2. All lane + DRP outputs consumed by an io_hash XOR tree
  3. PCS clock outputs looped back to clock inputs
  4. drp_ready_o handshake bypassed in the FSM

Build & program:

    python top.py                # build only
    python top.py program        # build + program

The TOML and CSR files are generated automatically from the
GowinSerDes configuration with GW5AST-138-specific inter-quad clock
routing overrides.  If the Gowin IDE tools are not installed, place
a pre-generated ``serdes.csr`` next to this file.
"""

import sys
from pathlib import Path

from amaranth import *
from amaranth.lib.cdc import FFSynchronizer

# ── Local imports ──────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).parent))
from uart import AsyncSerialRX, AsyncSerialTX
from gw5ast_dvk import GW5ASTDVKPlatform

# ── gowin_serdes package ──────────────────────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent.parent))
from gowin_serdes import (
    GearRate,
    GowinDevice,
    GowinSerDes,
    GowinSerDesGroup,
    LaneConfig,
    OperationMode,
)

# ── Constants ─────────────────────────────────────────────────
UPAR_FREQ = 62_500_000  # UPAR life-clock frequency (Hz)
BAUD_RATE = 115_200
DIVISOR = UPAR_FREQ // BAUD_RATE  # 542

# SerDes configuration — single source of truth used by both the
# hardware elaboration (SerDesDRPBridge) and TOML/CSR generation.
DEVICE = GowinDevice.GW5AST_138
LANE_CFG = LaneConfig(
    operation_mode=OperationMode.TX_RX,
    tx_data_rate="5G",
    rx_data_rate="5G",
    tx_gear_rate=GearRate.G1_2,
    rx_gear_rate=GearRate.G1_2,
    width_mode=20,
)


def make_serdes():
    """Build a GowinSerDes from the shared configuration constants."""
    group = GowinSerDesGroup(quad=0, first_lane=0, lane_configs=[LANE_CFG])
    return GowinSerDes(device=DEVICE, groups=[group]), group


class SerDesDRPBridge(Elaboratable):
    """Top-level design: GowinSerDes + UART <-> DRP FSM.

    GowinSerDes internally creates the "upar" clock domain from the
    GTR12 life-clock.  All fabric logic (UART, FSM) runs on that domain
    so there is no CDC.
    """

    def elaborate(self, platform):
        m = Module()

        # ==============================================================
        # Platform resources
        # ==============================================================
        uart = platform.request("uart")

        # NOTE: No clk_en here — the GW5AST-138 Tang Mega 138K Pro
        # receives its 125 MHz SerDes PLL reference clock on Q1's
        # dedicated REFPAD0, not through a fabric-controlled oscillator.

        # ==============================================================
        # SerDes instantiation — single lane, raw mode, DRP-only use
        # ==============================================================
        serdes, group = make_serdes()
        m.submodules.serdes = serdes

        # POR_N: assert high so the QUAD / PLL can start.
        m.d.comb += serdes.por_n.eq(1)

        # ==============================================================
        # Lane 0 — hold in reset, drive all inputs, consume all outputs
        # ==============================================================
        lane0 = group.lanes[0]

        # -- Drive reset: match USB3 PHY (resets deasserted) --
        # The USB3 PHY hardwires: pcs_rx_rst=0, pcs_tx_rst=0, fabric_rstn=1
        # PMA must be out of reset for the TX analog driver to power up.
        m.d.comb += [
            lane0.reset.pma_rstn.eq(1),  # PMA out of reset (active-low)
            lane0.reset.pcs_rx_rst.eq(0),  # PCS RX out of reset (active-high)
            lane0.reset.pcs_tx_rst.eq(0),  # PCS TX out of reset (active-high)
        ]

        # -- Drive RX inputs (idle, loop PCS clock back) --
        m.d.comb += [
            lane0.rx.fifo_rden.eq(0),
            lane0.rx.clk.eq(lane0.rx.pcs_clkout),
        ]

        # -- Drive TX inputs (idle, loop PCS clock back) --
        m.d.comb += [
            lane0.tx.data.eq(0),
            lane0.tx.fifo_wren.eq(0),
            lane0.tx.clk.eq(lane0.tx.pcs_clkout),
        ]

        # -- Consume all outputs: XOR tree into 8-bit io_hash --
        # Every output bit contributes to io_hash, making them all
        # observable (and therefore unsweepable by the synthesizer).
        drp = serdes.drp_q0_ln0

        io_hash = Signal(8, name="io_hash")
        m.d.comb += io_hash.eq(
            # Lane 0 status bits (8 bits)
            Cat(
                lane0.status.ready,
                lane0.status.pll_lock,
                lane0.status.rx_cdr_lock,
                lane0.status.signal_detect,
                lane0.status.refclk,
                lane0.rx.valid,
                lane0.rx.fifo_empty,
                lane0.rx.fifo_aempty,
            )
            # RX FIFO occupancy
            ^ lane0.rx.fifo_rdusewd[0:5].xor()
            # RX data (88 bits, folded 8 bits at a time)
            ^ lane0.rx.data[0:8]
            ^ lane0.rx.data[8:16]
            ^ lane0.rx.data[16:24]
            ^ lane0.rx.data[24:32]
            ^ lane0.rx.data[32:40]
            ^ lane0.rx.data[40:48]
            ^ lane0.rx.data[48:56]
            ^ lane0.rx.data[56:64]
            ^ lane0.rx.data[64:72]
            ^ lane0.rx.data[72:80]
            ^ lane0.rx.data[80:88]
            # RX PCS clock
            ^ lane0.rx.pcs_clkout
            # TX status
            ^ lane0.tx.fifo_full
            ^ lane0.tx.fifo_afull
            ^ lane0.tx.fifo_wrusewd[0:5].xor()
            ^ lane0.tx.pcs_clkout
            # DRP status
            ^ drp.ready
            ^ drp.rdvld
            ^ drp.resp
            # DRP read data (32 bits, folded 8 bits at a time)
            ^ drp.rddata[0:8]
            ^ drp.rddata[8:16]
            ^ drp.rddata[16:24]
            ^ drp.rddata[24:32]
            # Arbiter debug state
            ^ serdes.dbg_arb_state
        )

        # ==============================================================
        # UART — running in the "upar" clock domain
        # ==============================================================
        # GowinSerDes.elaborate() creates the "upar" domain from life_clk.
        # We DomainRename our UART to use it.
        uart_rx = AsyncSerialRX(divisor=DIVISOR)
        uart_tx = AsyncSerialTX(divisor=DIVISOR)
        m.submodules.uart_rx = DomainRenamer("upar")(uart_rx)
        m.submodules.uart_tx = DomainRenamer("upar")(uart_tx)

        # RX pin → 2-FF synchronizer → uart_rx.i (into upar domain)
        rx_sync = Signal(init=1)
        m.submodules += FFSynchronizer(uart.rx.i, rx_sync, o_domain="upar", init=1)
        m.d.comb += uart_rx.i.eq(rx_sync)

        # TX pin ← uart_tx.o
        m.d.comb += uart.tx.o.eq(uart_tx.o)

        # ==============================================================
        # DRP command FSM  (runs in "upar" domain)
        # ==============================================================
        is_write = Signal()
        addr = Signal(24)
        wrdata = Signal(32)
        rddata = Signal(32)
        status = Signal(8)
        byte_cnt = Signal(3)
        timeout = Signal(26)  # MSB set ≈ 1 s @ 62.5 MHz

        with m.FSM(domain="upar"):
            # ----------------------------------------------------------
            with m.State("IDLE"):
                m.d.comb += uart_rx.ack.eq(1)
                with m.If(uart_rx.rdy):
                    with m.If((uart_rx.data == ord("R")) | (uart_rx.data == ord("W"))):
                        m.d.upar += [
                            is_write.eq(uart_rx.data == ord("W")),
                            byte_cnt.eq(0),
                        ]
                        m.next = "RX_ADDR"
                    with m.Elif(uart_rx.data == ord("S")):
                        m.d.upar += status.eq(io_hash)
                        m.next = "TX_STATUS"
                    with m.Elif(uart_rx.data == ord("T")):
                        m.d.upar += status.eq(0x55)
                        m.next = "TX_STATUS"

            # ----------------------------------------------------------
            with m.State("RX_ADDR"):
                m.d.comb += uart_rx.ack.eq(1)
                with m.If(uart_rx.rdy):
                    m.d.upar += addr.eq(Cat(uart_rx.data, addr[:16]))
                    with m.If(byte_cnt == 2):
                        with m.If(is_write):
                            m.d.upar += byte_cnt.eq(0)
                            m.next = "RX_DATA"
                        with m.Else():
                            m.d.upar += timeout.eq(0)
                            m.next = "DRP_READY"
                    with m.Else():
                        m.d.upar += byte_cnt.eq(byte_cnt + 1)

            # ----------------------------------------------------------
            with m.State("RX_DATA"):
                m.d.comb += uart_rx.ack.eq(1)
                with m.If(uart_rx.rdy):
                    m.d.upar += wrdata.eq(Cat(uart_rx.data, wrdata[:24]))
                    with m.If(byte_cnt == 3):
                        m.d.upar += timeout.eq(0)
                        m.next = "DRP_READY"
                    with m.Else():
                        m.d.upar += byte_cnt.eq(byte_cnt + 1)

            # ----------------------------------------------------------
            with m.State("DRP_READY"):
                # Skip drp_ready_o check — go directly to DRP_OP.
                # The arbiter's drp_strb_i may be optimized away in
                # single-port mode, leaving drp_ready_o stuck at 0.
                # DRP transactions complete without the ready handshake.
                m.d.upar += timeout.eq(0)
                m.next = "DRP_OP"

            # ----------------------------------------------------------
            with m.State("DRP_OP"):
                m.d.comb += drp.addr.eq(addr)
                with m.If(is_write):
                    m.d.comb += [
                        drp.wren.eq(1),
                        drp.wrdata.eq(wrdata),
                        drp.strb.eq(0xFF),
                    ]
                with m.Else():
                    m.d.comb += drp.rden.eq(1)
                m.d.upar += timeout.eq(0)
                m.next = "DRP_WAIT"

            # ----------------------------------------------------------
            with m.State("DRP_WAIT"):
                m.d.upar += timeout.eq(timeout + 1)
                m.d.comb += drp.addr.eq(addr)

                # Keep wren/rden asserted so the arbiter can latch
                # them during the JUDG_ADDR phase (3 cycles).
                with m.If(is_write):
                    m.d.comb += [
                        drp.wren.eq(1),
                        drp.wrdata.eq(wrdata),
                        drp.strb.eq(0xFF),
                    ]
                with m.Else():
                    m.d.comb += drp.rden.eq(1)

                with m.If(is_write):
                    # Write: wait for ready or timeout
                    with m.If(drp.ready & (timeout > 1)):
                        m.d.upar += status.eq(0x00)
                        m.next = "TX_STATUS"
                    with m.Elif(timeout[-1]):
                        m.d.upar += status.eq(0xFF)
                        m.next = "TX_STATUS"
                with m.Else():
                    # Read: wait for rdvld or timeout
                    with m.If(drp.rdvld):
                        m.d.upar += [
                            rddata.eq(drp.rddata),
                            status.eq(Mux(drp.resp, 0x01, 0x00)),
                            byte_cnt.eq(0),
                        ]
                        m.next = "TX_DATA"
                    with m.Elif(timeout[-1]):
                        m.d.upar += [
                            rddata.eq(0),
                            status.eq(0xFF),
                            byte_cnt.eq(0),
                        ]
                        m.next = "TX_DATA"

            # ----------------------------------------------------------
            with m.State("TX_DATA"):
                m.d.comb += uart_tx.data.eq(rddata[24:32])
                with m.If(uart_tx.rdy):
                    m.d.comb += uart_tx.ack.eq(1)
                    with m.If(byte_cnt == 3):
                        m.next = "TX_STATUS"
                    with m.Else():
                        m.d.upar += [
                            rddata.eq(rddata << 8),
                            byte_cnt.eq(byte_cnt + 1),
                        ]

            # ----------------------------------------------------------
            with m.State("TX_STATUS"):
                m.d.comb += uart_tx.data.eq(status)
                with m.If(uart_tx.rdy):
                    m.d.comb += uart_tx.ack.eq(1)
                    m.next = "IDLE"

        return m


# ======================================================================
# Build entry-point
# ======================================================================


def build(do_program=False):
    platform = GW5ASTDVKPlatform()

    # Generate the SerDes CSR directly from the shared configuration.
    # Falls back to a pre-existing file if the Gowin tool is not installed.
    serdes, _ = make_serdes()
    csr_path = Path(__file__).parent / "serdes.csr"
    try:
        serdes.generate_csr(
            output_path=str(csr_path),
            toml_path=str(csr_path.with_suffix(".toml")),  # keep for inspection
        )
        print(f"Generated {csr_path} (and .toml) from GowinSerDes config")
    except FileNotFoundError as e:
        # Gowin tool not installed — fall back to pre-existing CSR
        if not csr_path.exists():
            csr_path = Path(__file__).parent / "build" / "serdes.csr"
        if not csr_path.exists():
            raise FileNotFoundError(
                f"serdes.csr not found and Gowin tool unavailable:\n  {e}\n"
                "Either install the Gowin IDE tools or place a pre-generated "
                "serdes.csr next to top.py."
            ) from e
        print(f"Gowin tool not found, using existing {csr_path}")

    platform.add_file("serdes.csr", csr_path.read_bytes())

    # SDC: The design runs only on the UPAR life-clock domain.
    sdc = (
        "# UPAR life-clock only design — no fabric clock constraints.\n"
        "# The UART and DRP FSM run in the upar domain from the\n"
        "# GTR12 hard macro life_clk (~62.5 MHz).\n"
    )

    platform.build(
        SerDesDRPBridge(),
        name="serdes_drp_uart",
        build_dir="build",
        do_program=do_program,
        add_constraints=sdc,
    )


if __name__ == "__main__":
    build(do_program="program" in sys.argv)
