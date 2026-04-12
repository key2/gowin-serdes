"""GTR12 QUAD and UPAR hard macro Instance builders.

GowinSerDesQuadInstance builds ONLY the QUAD Instance. GowinUPARInstance
builds the single UPAR Instance. The separation ensures exactly one UPAR
per device (not one per quad).
"""

from amaranth.hdl import Signal, Module, Instance, Const, Elaboratable

from .config import GowinDevice, device_quad_primitive, device_upar_primitive
from .primitives import get_quad_ports, get_upar_ports


class GowinSerDesQuadInstance(Elaboratable):
    """Instantiates one GTR12_QUADB/QUADA hard macro.

    Parameters
    ----------
    device : GowinDevice
    quad_idx : int
    lane_signals : dict of lane_idx -> {port_name: Signal}
    por_n : Signal
    life_clk : Signal
        Life clock output from QUAD (FABRIC_CM_LIFE_CLK_O).
    ahb_rstn : Signal
    test_dec_en : Signal
    inet_signals : dict (for GW5AT-138 INET buses)
    """

    def __init__(
        self,
        device,
        quad_idx,
        lane_signals,
        por_n,
        life_clk,
        ahb_rstn,
        test_dec_en,
        inet_signals=None,
    ):
        self.device = device
        self.quad_idx = quad_idx
        self.lane_signals = lane_signals
        self.por_n = por_n
        self.life_clk = life_clk
        self.ahb_rstn = ahb_rstn
        self.test_dec_en = test_dec_en
        self.inet_signals = inet_signals or {}

    def elaborate(self, platform):
        m = Module()

        quad_prim = device_quad_primitive(self.device)
        quad_ports = get_quad_ports(self.device)
        inst_args = {}

        active_lanes = self.lane_signals
        quad_conns = self._get_quad_level_connections()

        for port_name, direction, width in quad_ports:
            lane_idx = _extract_lane_index(port_name)

            if lane_idx is not None and lane_idx in active_lanes:
                sig = active_lanes[lane_idx].get(port_name)
                if sig is not None:
                    prefix = "i_" if direction == "i" else "o_"
                    inst_args[prefix + port_name] = sig
                elif port_name in quad_conns:
                    # Per-lane port not in lane dict but in quad-level
                    # (e.g. CPLL resets which apply to all lanes)
                    sig = quad_conns[port_name]
                    prefix = "i_" if direction == "i" else "o_"
                    inst_args[prefix + port_name] = sig
                else:
                    _add_default_port(inst_args, port_name, direction, width)
            elif port_name in quad_conns:
                sig = quad_conns[port_name]
                prefix = "i_" if direction == "i" else "o_"
                inst_args[prefix + port_name] = sig
            elif port_name in self.inet_signals:
                sig = self.inet_signals[port_name]
                prefix = "i_" if direction == "i" else "o_"
                inst_args[prefix + port_name] = sig
            else:
                _add_default_port(inst_args, port_name, direction, width)

        # POSITION defparam for multi-quad devices
        if self.device == GowinDevice.GW5AT_138:
            inst_args[f"p_POSITION"] = f"Q{self.quad_idx}"

        m.submodules.quad = Instance(quad_prim, **inst_args)

        return m

    def _get_quad_level_connections(self):
        conns = {
            "FABRIC_POR_N_I": self.por_n,
            "FABRIC_CMU0_RESETN_I": self.por_n,
            "FABRIC_CMU1_RESETN_I": self.por_n,
            "CK_AHB_I": self.life_clk,
            "AHB_RSTN": self.ahb_rstn,
            "TEST_DEC_EN": self.test_dec_en,
            "FABRIC_CM_LIFE_CLK_O": self.life_clk,
        }
        # ALL lane CPLL resets must be tied to por_n, not just active lanes.
        # Leaving unused CPLL in permanent reset can block the entire PLL tree.
        for i in range(4):
            conns[f"FABRIC_LN{i}_CPLL_RESETN_I"] = self.por_n
        return conns


class GowinUPARInstance(Elaboratable):
    """Instantiates one GTR12_UPARA/UPAR hard macro (singleton per device).

    Parameters
    ----------
    device : GowinDevice
    upar_signals : dict of UPAR bus signal name -> Signal
    life_clk : Signal
    ahb_clk : Signal
    ahb_rstn : Signal
    test_dec_en : Signal
    inet_signals : dict (for GW5AT-138 INET buses)
    """

    def __init__(
        self,
        device,
        upar_signals,
        life_clk,
        ahb_clk,
        ahb_rstn,
        test_dec_en,
        inet_signals=None,
    ):
        self.device = device
        self.upar_signals = upar_signals
        self.life_clk = life_clk
        self.ahb_clk = ahb_clk
        self.ahb_rstn = ahb_rstn
        self.test_dec_en = test_dec_en
        self.inet_signals = inet_signals or {}

    def elaborate(self, platform):
        m = Module()

        upar_prim = device_upar_primitive(self.device)
        upar_ports = get_upar_ports(self.device)
        upar_args = {}

        upar_conns = self._get_upar_connections()

        for port_name, direction, width in upar_ports:
            if port_name in upar_conns:
                sig = upar_conns[port_name]
                prefix = "i_" if direction == "i" else "o_"
                upar_args[prefix + port_name] = sig
            elif port_name in self.inet_signals:
                sig = self.inet_signals[port_name]
                prefix = "i_" if direction == "i" else "o_"
                upar_args[prefix + port_name] = sig
            else:
                _add_default_port(upar_args, port_name, direction, width)

        m.submodules.upar = Instance(upar_prim, **upar_args)

        return m

    def _get_upar_connections(self):
        us = self.upar_signals
        return {
            "UPAR_CLK": self.life_clk,
            "UPAR_RST": us["rst"],
            "UPAR_WREN_S": us["wren"],
            "UPAR_ADDR_S": us["addr"],
            "UPAR_WRDATA_S": us["wrdata"],
            "UPAR_RDEN_S": us["rden"],
            "UPAR_STRB_S": us["strb"],
            "UPAR_BUS_WIDTH_S": us["bus_width"],
            "UPAR_RDDATA_S": us["rddata"],
            "UPAR_RDVLD_S": us["rdvld"],
            "UPAR_READY_S": us["ready"],
            "AHB_CLK_O": self.ahb_clk,
            "AHB_RSTN_O": self.ahb_rstn,
            "QUAD_CFG_TEST_DEC_EN": self.test_dec_en,
            "CSR_MODE": Const(0b10100, 5),
        }


# ── Helpers ────────────────────────────────────────────────────


def _extract_lane_index(port_name):
    """Extract lane index from port name, or None if quad-level.

    Handles both LN{i}_ and LANE{i}_ patterns. Avoids false positives
    from quad-level ports like FABRIC_CMU1_CK_REF_O by requiring
    the lane pattern at specific positions.
    """
    import re

    # Match FABRIC_LN{i}_ or _LN{i}_ (preceded by underscore)
    m = re.search(r"(?:FABRIC_LN|_LN)(\d)_", port_name)
    if m:
        return int(m.group(1))
    # Match LANE{i}_ at start or after underscore
    m = re.search(r"(?:^|_)LANE(\d)_", port_name)
    if m:
        return int(m.group(1))
    # Match LN{i}_TXM, LN{i}_TXP, LN{i}_RXM, LN{i}_RXP
    m = re.match(r"^LN(\d)_", port_name)
    if m:
        return int(m.group(1))
    return None


def _add_default_port(inst_args, port_name, direction, width):
    """Tie input to GND, output to a dummy signal."""
    if direction == "i":
        inst_args[f"i_{port_name}"] = Const(0, width)
    else:
        inst_args[f"o_{port_name}"] = Signal(width, name=f"_unused_{port_name}")
