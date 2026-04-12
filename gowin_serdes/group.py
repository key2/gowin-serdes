"""SerDes lane group — 1 to 4 lanes sharing a single DRP port.

Maps to exactly one Gowin ``Customized_PHY_Top`` instance.
The group owns the lanes and channel bonding config.

DRP access is NOT on the group — it lives on the GowinSerDes Component,
one port per group, so that Amaranth signal ownership is correct.
"""

from typing import List, Optional

from amaranth.hdl import Module
from amaranth.lib.wiring import Component, In, Out

from .config import LaneConfig, ChannelBondingDepth
from .lane import GowinSerDesLane


class GowinSerDesGroup(Component):
    """A group of 1-4 lanes sharing a single DRP arbiter slot.

    Parameters
    ----------
    quad : int
        Quad index (0 or 1).
    first_lane : int
        Index of the first lane in this group within the quad (0-3).
    lane_configs : list of LaneConfig
        Configuration for each lane. Length 1-4.
    chbond_master : int or None
        Index (within this group) of the channel bonding master lane.
        None means no bonding.
    chbond_depth : ChannelBondingDepth
        Bonding alignment depth.
    """

    def __init__(
        self,
        quad: int,
        first_lane: int,
        lane_configs: List[LaneConfig],
        chbond_master: Optional[int] = None,
        chbond_depth: ChannelBondingDepth = ChannelBondingDepth.NONE,
    ):
        assert 1 <= len(lane_configs) <= 4
        assert 0 <= first_lane <= 3
        assert first_lane + len(lane_configs) <= 4

        self.quad = quad
        self.first_lane = first_lane
        self.chbond_master = chbond_master
        self.chbond_depth = chbond_depth
        self.lane_configs = lane_configs

        # Create lane components
        self.lanes = [GowinSerDesLane(cfg) for cfg in lane_configs]

        # Arbiter slot = quad * 4 + first_lane
        self.arbiter_slot = quad * 4 + first_lane

        # Build component signature
        members = {}
        if chbond_master is not None:
            members["cb_start"] = In(1)

        super().__init__(members)

    def elaborate(self, platform):
        m = Module()

        for i, lane in enumerate(self.lanes):
            m.submodules[f"lane{i}"] = lane

        if self.chbond_master is not None:
            for lane in self.lanes:
                m.d.comb += lane._quad_chbond_start.eq(self.cb_start)
        else:
            for lane in self.lanes:
                m.d.comb += lane._quad_chbond_start.eq(0)

        return m

    @property
    def num_lanes(self) -> int:
        return len(self.lanes)

    @property
    def drp_name(self) -> str:
        """Canonical name for this group's DRP port on GowinSerDes."""
        return f"drp_q{self.quad}_ln{self.first_lane}"
