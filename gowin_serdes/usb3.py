"""USB 3.1 SerDes recipe for the Gowin USB3.1 PHY (``gw_usb3``).

Single source of truth for everything needed to run a GTR12 lane under the
USB3.1 PHY, extracted from (and verified against) the Gowin USB3.1 UVC
reference design:

* :func:`usb3_lane_config` -- the lane settings.  The generated TOML for
  the reference configuration (GW5AT-60, Q0 lane 1, 200 MHz on REFPAD1) is
  **byte-identical** to the reference design's IDE-generated
  ``serdes_tmp.toml`` (enforced by ``tests/test_gowin_serdes.py``).
* :data:`USB3_QUAD_OVERRIDES` -- quad-level deviations from the generator
  defaults (fabric-controlled resets, POR toggle, RX equalizer bias).
* :func:`usb3_boot_writes` -- the two CSR writes the reference flow appends
  after TOML->CSR conversion: a lane TX-AFE tuning register and *TX
  electrical idle asserted* (the PHY's CSR sequencer boots with its EIDLE
  synchronisers at 1 and only rewrites the register on the first 1->0
  transition, so the boot state must already match).  With these appended,
  the generated ``.csr`` is byte-identical to the reference ``serdes.csr``.
* :func:`make_usb3_serdes` -- one-call construction of the
  :class:`~gowin_serdes.GowinSerDes` + group for a USB3 lane.
* :func:`attach_usb3_phy` -- the complete PHY<->lane<->DRP wiring,
  table-driven by :data:`PHY_LANE_WIRING` (which is itself cross-checked
  against the reference ``SerDes_Top`` netlist by
  ``tests/test_serdes_wiring.py``).  Note the deliberately *crossed* fabric
  clocks: the PHY's ``serdes_fabric_tx_clk_o`` (a copy of the RX PCS clock)
  drives the lane's FABRIC **RX** CLK and vice versa -- this reproduces the
  reference design wiring exactly.

Boot-rate note: the reference design boots the lane in the Gen2 trim
(10G / 16-bit / 1:4); the PHY's ``upar_csr`` sequencer then reconfigures
10G<->5G at runtime on LTSSM rate changes, so its rate synchronisers reset
to 1 (``rate_init=1``, the ``gw_usb3.Usb31Phy`` default).  A 5G boot trim
(5G / 20-bit / 1:2) is also expressible for experiments -- pair it with
``rate_init=0``.
"""

from typing import Optional, Tuple

from .config import (
    GearRate,
    GowinDevice,
    LaneConfig,
    OperationMode,
    PLLSelection,
    RefClkSource,
)
from .group import GowinSerDesGroup
from .serdes import GowinSerDes


# ── Reference-design deviations from the generator defaults ────────────

#: Quad-level TOML overrides of the USB3.1 reference design.
USB3_QUAD_OVERRIDES = {
    "cmu0_reset_by_fabric": True,
    "cmu1_reset_by_fabric": True,
    "por_toggle_by_fabric": True,
    "rx_eq_bias": 15,
}

#: Lane-level TOML overrides of the USB3.1 reference design.  CTC and
#: channel bonding are disabled; their fields are still mirrored so the
#: CSR blob matches the proven configuration bit for bit.
USB3_LANE_OVERRIDES = {
    "rx_if_cfg_rd_start_depth": 3,
    "tx_if_cfg_rd_start_depth": 3,
    "cpll_reset_by_fabric": True,
    "chbond_cfg_rd_start_depth": 8,
    "ctc_rd_start_depth": "8",
    "ctc_skipa_pattern": 28,
    "ctc_skipb_pattern": 28,
}

#: Boot trims: rate -> (tx/rx data rate, gearing, fabric width).
_BOOT_TRIMS = {
    "10G": ("10G", GearRate.G1_4, 16),   # Gen2 boot, reference design
    "5G": ("5G", GearRate.G1_2, 20),     # Gen1 boot (pair with rate_init=0)
}


def usb3_lane_config(
    ref_clk_source: RefClkSource,
    ref_clk_freq: str,
    boot_rate: str = "10G",
) -> LaneConfig:
    """Lane configuration for the Gowin USB3.1 PHY.

    Parameters
    ----------
    ref_clk_source, ref_clk_freq:
        Reference clock routing; the PHY's ``UparCsrConfig.refclk`` must be
        set to the matching frequency (100/125/200 MHz are supported by the
        vendor rate-change tables).
    boot_rate:
        ``"10G"`` (reference design Gen2 trim, default) or ``"5G"``.
    """
    rate, gear, width = _BOOT_TRIMS[boot_rate]
    return LaneConfig(
        operation_mode=OperationMode.TX_RX,
        tx_data_rate=rate,
        rx_data_rate=rate,
        tx_gear_rate=gear,
        rx_gear_rate=gear,
        width_mode=width,
        pll=PLLSelection.CPLL,
        ref_clk_source=ref_clk_source,
        ref_clk_freq=ref_clk_freq,
        toml_lane_overrides=dict(USB3_LANE_OVERRIDES),
    )


def usb3_boot_writes(quad: int, lane: int) -> list:
    """The two post-conversion CSR boot writes of the reference flow.

    ``(address, data)`` tuples for :meth:`GowinSerDes.generate_csr`'s
    ``extra_writes``: lane TX-AFE tuning (0x8082F8 region) and TX
    electrical idle asserted (0x8003A4 region, the state the PHY's CSR
    sequencer assumes at reset).
    """
    base = quad * 0x100000
    return [
        (0x8082F8 + base + lane * 0x100, 0x00000A02),
        (0x8003A4 + base + lane * 0x200, 0x00000001),
    ]


def make_usb3_serdes(
    device: GowinDevice,
    quad: int,
    lane: int,
    ref_clk_source: RefClkSource,
    ref_clk_freq: str,
    boot_rate: str = "10G",
) -> Tuple[GowinSerDes, GowinSerDesGroup]:
    """Construct the SerDes for one USB3 lane.

    Returns ``(serdes, group)``; the lane is ``group.lanes[0]`` and the
    PHY's DRP port is ``getattr(serdes, group.drp_name)``.
    """
    group = GowinSerDesGroup(
        quad=quad,
        first_lane=lane,
        lane_configs=[usb3_lane_config(ref_clk_source, ref_clk_freq,
                                       boot_rate)],
        toml_quad_overrides=dict(USB3_QUAD_OVERRIDES),
    )
    return GowinSerDes(device=device, groups=[group]), group


# ── PHY <-> lane wiring ─────────────────────────────────────────────────

#: (phy port, lane signature member, direction) -- ``direction`` is
#: "to_phy" (lane drives PHY input) or "to_lane" (PHY output drives lane).
#: The FABRIC_*_CLK entries are crossed on purpose (see module docstring).
PHY_LANE_WIRING = (
    # clocks & resets
    ("serdes_pcs_tx_clk_i",      "tx.pcs_clkout",       "to_phy"),
    ("serdes_pcs_rx_clk_i",      "rx.pcs_clkout",       "to_phy"),
    ("serdes_fabric_tx_clk_o",   "rx.clk",              "to_lane"),
    ("serdes_fabric_rx_clk_o",   "tx.clk",              "to_lane"),
    ("serdes_fabric_rstn_o",     "reset.pma_rstn",      "to_lane"),
    ("serdes_pcs_rx_rst_o",      "reset.pcs_rx_rst",    "to_lane"),
    ("serdes_pcs_tx_rst_o",      "reset.pcs_tx_rst",    "to_lane"),
    # RX data path
    ("serdes_rxdata_i",          "rx.data",             "to_phy"),
    ("serdes_rx_vld_i",          "rx.valid",            "to_phy"),
    ("serdes_rxfifo_rd_en_o",    "rx.fifo_rden",        "to_lane"),
    ("serdes_rxfifo_aempty_i",   "rx.fifo_aempty",      "to_phy"),
    ("serdes_rx_fifo_rdusewd_i", "rx.fifo_rdusewd",     "to_phy"),
    # TX data path
    ("serdes_txdata_o",          "tx.data",             "to_lane"),
    ("serdes_fabric_tx_vld_o",   "tx.fifo_wren",        "to_lane"),
    ("serdes_tx_fifo_wrusewd_i", "tx.fifo_wrusewd",     "to_phy"),
    # status
    ("serdes_cpll_ok_i",         "status.pll_lock",     "to_phy"),
    ("serdes_pma_rx_lock_i",     "status.rx_cdr_lock",  "to_phy"),
    ("serdes_astat_i",           "status.astat",        "to_phy"),
    ("serdes_rxelecidle_i",      "status.rx_elecidle",  "to_phy"),
)


def _lane_member(lane, path: str):
    obj = lane
    for part in path.split("."):
        obj = getattr(obj, part)
    return obj


def attach_usb3_phy(m, phy, lane, drp):
    """Wire a ``gw_usb3.Usb31Phy`` to a SerDes lane and its DRP port.

    Adds all combinational connections between *phy*, *lane* and *drp*
    (duck-typed: this module does not import ``gw_usb3``).  Quad PLL locks
    and the unused ``ref_clk`` input are tied off; CPLL lock comes from the
    lane.  The caller still owns ``phy.phy_resetn``, the PIPE side, and
    ``serdes.por_n``.
    """
    for phy_port, lane_path, direction in PHY_LANE_WIRING:
        phy_sig = getattr(phy, phy_port)
        lane_sig = _lane_member(lane, lane_path)
        if direction == "to_phy":
            m.d.comb += phy_sig.eq(lane_sig)
        else:
            m.d.comb += lane_sig.eq(phy_sig)

    m.d.comb += [
        phy.serdes_q0_qpll0_ok_i.eq(0),
        phy.serdes_q0_qpll1_ok_i.eq(0),
        phy.serdes_q1_qpll0_ok_i.eq(0),
        phy.serdes_q1_qpll1_ok_i.eq(0),
        phy.ref_clk.eq(0),

        # UPAR / DRP
        phy.serdes_upar_clk_i.eq(drp.clk),
        drp.addr.eq(phy.serdes_upar_addr_o),
        drp.wren.eq(phy.serdes_upar_wren_o),
        drp.wrdata.eq(phy.serdes_upar_wrdata_o),
        drp.strb.eq(phy.serdes_upar_strb_o),
        drp.rden.eq(phy.serdes_upar_rden_o),
        phy.serdes_upar_ready_i.eq(drp.ready),
        phy.serdes_upar_rdvld_i.eq(drp.rdvld),
        phy.serdes_upar_rddata_i.eq(drp.rddata),
        phy.serdes_upar_resp_i.eq(drp.resp),
    ]
