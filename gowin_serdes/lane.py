"""Per-lane SerDes wiring component.

Each GowinSerDesLane is a Component exposing user-facing TX/RX/status/reset
ports. Internally it holds "quad-side" signals that GowinSerDes.elaborate()
connects to the GTR12 hard macro.

The lane contains zero logic — it is pure wiring (combinatorial assigns)
that replaces Gowin's 2000+ lines of ifdef'd Verilog.
"""

from amaranth.hdl import Signal, Module
from amaranth.lib.wiring import Component, In, Out

from .config import LaneConfig
from .signature import (
    LaneTXSignature,
    LaneRXSignature,
    LaneStatusSignature,
    LaneResetSignature,
)


class GowinSerDesLane(Component):
    """One SerDes lane with user-facing interface.

    Parameters
    ----------
    config : LaneConfig
        Per-lane configuration (encoding, width, features).
    """

    def __init__(self, config: LaneConfig):
        self.config = config

        members = {}
        if config.has_tx:
            members["tx"] = Out(
                LaneTXSignature(
                    config.tx_data_width,
                    has_64b66b=config.has_64b66b,
                    has_64b67b=config.has_64b67b,
                )
            )
        if config.has_rx:
            members["rx"] = Out(
                LaneRXSignature(
                    config.rx_data_width,
                    has_64b66b=config.has_64b66b,
                    has_64b67b=config.has_64b67b,
                )
            )

        members["status"] = Out(
            LaneStatusSignature(
                has_8b10b=config.has_8b10b,
                has_64b=config.has_64b,
                has_64b67b=config.has_64b67b,
            )
        )
        members["reset"] = In(LaneResetSignature())

        if config.ctc_enable:
            members["cc_clk"] = In(1)

        super().__init__(members)

        # Quad-side signals — populated by GowinSerDes.elaborate()
        # TX side (outputs to QUAD)
        self._quad_txdata = Signal(80, name="quad_txdata")
        self._quad_tx_vld = Signal(name="quad_tx_vld")
        self._quad_tx_clk = Signal(name="quad_fabric_tx_clk")
        self._quad_rstn = Signal(name="quad_rstn")
        self._quad_pcs_rx_rst = Signal(name="quad_pcs_rx_rst")
        self._quad_pcs_tx_rst = Signal(name="quad_pcs_tx_rst")
        self._quad_rx_clk = Signal(name="quad_fabric_rx_clk")
        self._quad_fifo_rden = Signal(name="quad_rx_fifo_rden")
        self._quad_chbond_start = Signal(name="quad_chbond_start")
        self._quad_c2i_clk = Signal(name="quad_c2i_clk")
        self._quad_tx_disparity = Signal(8, name="quad_tx_disparity")

        # QUADB-only 64B66B signals (outputs from QUAD)
        self._quad_64b66b_tx_invld_blk = Signal(name="quad_64b66b_tx_invld_blk")
        self._quad_64b66b_tx_fetch = Signal(name="quad_64b66b_tx_fetch")
        self._quad_64b66b_rx_valid = Signal(name="quad_64b66b_rx_valid")

        # RX side (inputs from QUAD)
        self._quad_rxdata = Signal(88, name="quad_rxdata")
        self._quad_rx_vld = Signal(name="quad_rx_vld")
        self._quad_rx_pcs_clk = Signal(name="quad_rx_pcs_clk")
        self._quad_tx_pcs_clk = Signal(name="quad_tx_pcs_clk")
        self._quad_rx_fifo_rdusewd = Signal(5, name="quad_rx_fifo_rdusewd")
        self._quad_rx_fifo_aempty = Signal(name="quad_rx_fifo_aempty")
        self._quad_rx_fifo_empty = Signal(name="quad_rx_fifo_empty")
        self._quad_tx_fifo_wrusewd = Signal(5, name="quad_tx_fifo_wrusewd")
        self._quad_tx_fifo_afull = Signal(name="quad_tx_fifo_afull")
        self._quad_tx_fifo_full = Signal(name="quad_tx_fifo_full")
        self._quad_stat = Signal(13, name="quad_stat")
        self._quad_astat = Signal(6, name="quad_astat")
        self._quad_pma_rx_lock = Signal(name="quad_pma_rx_lock")
        self._quad_cmu_ok = Signal(name="quad_cmu_ok")
        self._quad_cmu_ck_ref = Signal(name="quad_cmu_ck_ref")
        self._quad_align_link = Signal(name="quad_align_link")
        self._quad_k_lock = Signal(name="quad_k_lock")

    def elaborate(self, platform):
        m = Module()
        cfg = self.config

        # ── TX wiring ──────────────────────────────────────────
        if cfg.has_tx:
            if cfg.has_64b66b:
                # 64B66B TX: pack txd[63:0], txc[7:0], tx_ctrl[2:0] into 80-bit bus
                m.d.comb += [
                    self._quad_txdata[:64].eq(self.tx.txd),
                    self._quad_txdata[64:72].eq(self.tx.txc),
                    self._quad_txdata[72:75].eq(self.tx.tx_ctrl),
                    # TX_VLD_IN = ~TX_IF_FIFO_AFULL in 64B66B mode
                    self._quad_tx_vld.eq(~self._quad_tx_fifo_afull),
                    self.tx.tx_fetch.eq(self._quad_64b66b_tx_fetch),
                ]
            elif cfg.has_64b67b:
                # 64B67B TX: pack tx_data[63:0], {6'b0, tx_header[1:0]}, tx_ctrl[2:0]
                m.d.comb += [
                    self._quad_txdata[:64].eq(self.tx.tx_data),
                    self._quad_txdata[64:72].eq(self.tx.tx_header),
                    self._quad_txdata[72:75].eq(self.tx.tx_ctrl),
                    self._quad_tx_vld.eq(~self._quad_tx_fifo_afull),
                    self.tx.tx_fetch.eq(self._quad_64b66b_tx_fetch),
                ]
            else:
                # General / 8B10B: direct passthrough
                m.d.comb += [
                    self._quad_txdata.eq(self.tx.data),
                    self._quad_tx_vld.eq(self.tx.fifo_wren),
                ]

            # Common TX signals
            m.d.comb += [
                self._quad_tx_clk.eq(self.tx.clk),
                self.tx.pcs_clkout.eq(self._quad_tx_pcs_clk),
                self.tx.fifo_wrusewd.eq(self._quad_tx_fifo_wrusewd),
                self.tx.fifo_afull.eq(self._quad_tx_fifo_afull),
                self.tx.fifo_full.eq(self._quad_tx_fifo_full),
            ]

        # ── RX wiring ──────────────────────────────────────────
        if cfg.has_rx:
            if cfg.has_64b66b:
                # 64B66B RX: extract rxd[63:0], rxc[7:0] from 88-bit bus
                m.d.comb += [
                    self.rx.rxd.eq(self._quad_rxdata[:64]),
                    self.rx.rxc.eq(self._quad_rxdata[64:72]),
                    # In 64B66B mode, valid comes from _64B66B_RX_VALID (QUADB)
                    # or RX_VLD_OUT (QUADA)
                    self.rx.valid.eq(self._quad_64b66b_rx_valid | self._quad_rx_vld),
                ]
            elif cfg.has_64b67b:
                m.d.comb += [
                    self.rx.rx_data.eq(self._quad_rxdata[:64]),
                    self.rx.rx_header.eq(self._quad_rxdata[64:66]),
                    self.rx.valid.eq(self._quad_64b66b_rx_valid | self._quad_rx_vld),
                ]
            else:
                m.d.comb += [
                    self.rx.valid.eq(self._quad_rx_vld),
                ]

            # Common RX — raw data always available
            m.d.comb += [
                self.rx.data.eq(self._quad_rxdata),
                self.rx.pcs_clkout.eq(self._quad_rx_pcs_clk),
                self._quad_rx_clk.eq(self.rx.clk),
                self._quad_fifo_rden.eq(self.rx.fifo_rden),
                self.rx.fifo_rdusewd.eq(self._quad_rx_fifo_rdusewd),
                self.rx.fifo_aempty.eq(self._quad_rx_fifo_aempty),
                self.rx.fifo_empty.eq(self._quad_rx_fifo_empty),
            ]

        # ── Status wiring ──────────────────────────────────────
        m.d.comb += [
            self.status.ready.eq(self._quad_stat[12]),
            self.status.signal_detect.eq(self._quad_astat[5]),
            self.status.rx_cdr_lock.eq(self._quad_pma_rx_lock),
            self.status.pll_lock.eq(self._quad_cmu_ok),
            self.status.refclk.eq(self._quad_cmu_ck_ref),
        ]
        if cfg.has_8b10b:
            m.d.comb += [
                self.status.k_lock.eq(self._quad_k_lock),
                self.status.word_align_link.eq(self._quad_align_link),
            ]
        if cfg.has_64b:
            m.d.comb += [
                self.status.tx_invld_blk.eq(self._quad_64b66b_tx_invld_blk),
                self.status.rx_blk_lock.eq(self._quad_rxdata[72]),
                self.status.rx_dec_err.eq(self._quad_rxdata[73]),
            ]
        if cfg.has_64b67b:
            m.d.comb += [
                self.status.rx_dscr_err.eq(self._quad_rxdata[74]),
            ]

        # NOTE: Duplicate RX and Status wiring blocks that existed here
        # (lines 203-228 in the original) have been removed. The first
        # wiring blocks above (with has_64b66b/has_64b67b dispatch for
        # RX, and has_8b10b/has_64b for Status) are the canonical ones.

        # ── Reset wiring ───────────────────────────────────────
        m.d.comb += [
            self._quad_rstn.eq(self.reset.pma_rstn),
            self._quad_pcs_rx_rst.eq(self.reset.pcs_rx_rst),
            self._quad_pcs_tx_rst.eq(self.reset.pcs_tx_rst),
        ]

        # ── Channel bonding ────────────────────────────────────
        # Set by group.py based on bonding config; defaults to 0 (GND)

        # ── CTC clock ──────────────────────────────────────────
        if cfg.ctc_enable:
            m.d.comb += self._quad_c2i_clk.eq(self.cc_clk)
        else:
            m.d.comb += self._quad_c2i_clk.eq(0)

        return m
