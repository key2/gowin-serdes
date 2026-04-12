"""Wiring signatures for Gowin SerDes interfaces.

These define the typed port interfaces used to connect components.
Direction (In/Out) is from the perspective of the component that *owns*
the interface; connecting components see flipped directions.
"""

from amaranth.lib.wiring import Signature, In, Out


class DRPSignature(Signature):
    """Dynamic Reconfiguration Port — one client's view of the arbiter."""

    def __init__(self):
        super().__init__(
            {
                "clk": In(1),
                "addr": Out(24),
                "wren": Out(1),
                "wrdata": Out(32),
                "strb": Out(8),
                "ready": In(1),
                "rden": Out(1),
                "rdvld": In(1),
                "rddata": In(32),
                "resp": In(1),
            }
        )


class UPARSignature(Signature):
    """UPAR register bus — master side facing GTR12_UPARA."""

    def __init__(self):
        super().__init__(
            {
                "clk": In(1),
                "rst": Out(1),
                "addr": Out(24),
                "wren": Out(1),
                "wrdata": Out(32),
                "strb": Out(8),
                "ready": In(1),
                "rden": Out(1),
                "bus_width": Out(1),
                "rdvld": In(1),
                "rddata": In(32),
            }
        )


class LaneStatusSignature(Signature):
    """Per-lane status outputs."""

    def __init__(self, *, has_8b10b=False, has_64b=False, has_64b67b=False):
        members = {
            "ready": Out(1),
            "signal_detect": Out(1),
            "rx_cdr_lock": Out(1),
            "pll_lock": Out(1),
            "refclk": Out(1),
        }
        if has_8b10b:
            members["k_lock"] = Out(1)
            members["word_align_link"] = Out(1)
        if has_64b:
            members["tx_invld_blk"] = Out(1)
            members["rx_blk_lock"] = Out(1)
            members["rx_dec_err"] = Out(1)
        if has_64b67b:
            members["rx_dscr_err"] = Out(1)
        super().__init__(members)


class LaneTXSignature(Signature):
    """Per-lane TX interface."""

    def __init__(self, data_width=80, *, has_64b66b=False, has_64b67b=False):
        members = {
            "pcs_clkout": Out(1),
            "clk": In(1),
            "data": In(data_width),
            "fifo_wren": In(1),
            "fifo_wrusewd": Out(5),
            "fifo_afull": Out(1),
            "fifo_full": Out(1),
        }
        if has_64b66b:
            members["txc"] = In(8)
            members["txd"] = In(64)
            members["tx_ctrl"] = In(3)
            members["tx_fetch"] = Out(1)
        if has_64b67b:
            members["tx_header"] = In(2)
            members["tx_data"] = In(64)
            members["tx_ctrl"] = In(3)
            members["tx_fetch"] = Out(1)
        super().__init__(members)


class LaneRXSignature(Signature):
    """Per-lane RX interface."""

    def __init__(self, data_width=88, *, has_64b66b=False, has_64b67b=False):
        members = {
            "pcs_clkout": Out(1),
            "clk": In(1),
            "data": Out(data_width),
            "fifo_rden": In(1),
            "fifo_rdusewd": Out(5),
            "fifo_aempty": Out(1),
            "fifo_empty": Out(1),
            "valid": Out(1),
        }
        if has_64b66b:
            members["rxc"] = Out(8)
            members["rxd"] = Out(64)
        if has_64b67b:
            members["rx_header"] = Out(2)
            members["rx_data"] = Out(64)
        super().__init__(members)


class LaneResetSignature(Signature):
    """Per-lane reset controls."""

    def __init__(self):
        super().__init__(
            {
                "pma_rstn": In(1),
                "pcs_rx_rst": In(1),
                "pcs_tx_rst": In(1),
            }
        )
