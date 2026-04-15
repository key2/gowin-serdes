"""SerDes CSR register map and TOML-based configuration for Gowin GTR12.

Models the complete TOML parameter space used by Gowin SerDes IP.
The gearbox is inside the GTR12 hard macro — configured via TOML
settings (tx_gear_rate, rx_gear_rate, width_mode) and static CSR
writes. The fabric sees different pin widths depending on the gear ratio.
"""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional


class Quad(IntEnum):
    """Quad selector — top nibble of address space."""

    Q0 = 0x80
    Q1 = 0x90


class UPARPrefix(IntEnum):
    """UPAR bus prefix for quad selection."""

    Q0 = 0xC0
    Q1 = 0xC1


class CSRBlock(IntEnum):
    """Major CSR block base addresses."""

    GLOBAL = 0xB00000
    UPAR_CTRL = 0xC00000
    QUAD_CTRL = 0x808000
    QPLL0 = 0x808100
    AFE_BASE = 0x808200
    CDR_BASE = 0x800200
    PCS_BASE = 0x809000


class LaneCSR:
    """Per-lane CSR address calculator.

    Computes register addresses based on the quad and lane number,
    matching the Verilog ifdef patterns exactly.

    Parameters
    ----------
    quad : Quad
        Quad selector (Q0 or Q1).
    lane : int
        Lane number within quad (0-3).
    """

    def __init__(self, quad=Quad.Q0, lane=0):
        assert 0 <= lane <= 3, f"Lane must be 0-3, got {lane}"
        self.quad = quad
        self.lane = lane

        q = int(quad)

        # AFE registers: base at quad<<16 | (0x8200 + lane*0x100)
        self.afe_base = (q << 16) | (0x8200 + lane * 0x100)

        # CDR registers: base at quad<<16 | (0x0200 + lane*0x200)
        self.cdr_base = (q << 16) | (0x0200 + lane * 0x200)

        # PCS registers: base at quad<<16 | (0x9000 + lane*0x200)
        self.pcs_base = (q << 16) | (0x9000 + lane * 0x200)

        # Per-lane control block: quad<<16 | (0x0300 + lane*0x200)
        self.ln_ctrl_base = (q << 16) | (0x0300 + lane * 0x200)

    # ---- TX FFE addresses (AFE block) ----

    @property
    def tx_ffe_0(self):
        """CSR_TX_FFE_0: AFE base + 0x34."""
        return self.afe_base | 0x34

    @property
    def tx_ffe_1(self):
        """CSR_TX_FFE_1: AFE base + 0x38."""
        return self.afe_base | 0x38

    @property
    def tx_ffe_2(self):
        """CSR_TX_FFE_2: AFE base + 0xD8."""
        return self.afe_base | 0xD8

    # ---- Eidle / Pulse / RxDet (lane AFE/CDR) ----

    @property
    def eidle_addr(self):
        """CSR_WRITE_EIDLE: quad<<16 | (0x03A4 + lane*0x200)."""
        return (int(self.quad) << 16) | (0x03A4 + self.lane * 0x200)

    @property
    def pulse_addr(self):
        """CSR_WRITE_PLUSE: quad<<16 | (0x033F + lane*0x200)."""
        return (int(self.quad) << 16) | (0x033F + self.lane * 0x200)

    @property
    def rxdet_addr(self):
        """CSR_READ_RXDET: quad<<16 | (0x8B34 + lane*0x100)."""
        return (int(self.quad) << 16) | (0x8B34 + self.lane * 0x100)

    # ---- PCS registers ----

    @property
    def pcs_8b10b_addr(self):
        """CSR_WRITE_8B10B: PCS base + 0x68."""
        return self.pcs_base | 0x68

    @property
    def rx_polarity_addr(self):
        """CSR_WRITE_RX_POLARITY: PCS base + 0x08."""
        return self.pcs_base | 0x08

    # ---- CDR config ----

    @property
    def cdr_cfg_0(self):
        """CSR_WRITE_CDR_CFG_0: CDR base + 0x53."""
        return self.cdr_base | 0x53

    @property
    def cdr_cfg_1(self):
        """CSR_WRITE_CDR_CFG_1: CDR base + 0x5E."""
        return self.cdr_base | 0x5E

    @property
    def cdr_cfg_2(self):
        """CSR_WRITE_CDR_CFG_2: CDR base + 0x5F."""
        return self.cdr_base | 0x5F

    @property
    def cdr_cfg_3(self):
        """CSR_WRITE_CDR_CFG_3: CDR base + 0x54."""
        return self.cdr_base | 0x54

    @property
    def cdr_cfg_4(self):
        """CSR_WRITE_CDR_CFG_4: CDR base + 0x60."""
        return self.cdr_base | 0x60

    @property
    def cdr_cfg_5(self):
        """CSR_WRITE_CDR_CFG_5: CDR base + 0x61."""
        return self.cdr_base | 0x61

    # ---- Loopback ----

    @property
    def loopback_addr(self):
        """CSR_LOOPBACK_MODE: CDR base + 0x56."""
        return self.cdr_base | 0x56

    # ---- Shared quad-level registers ----

    @property
    def cdr_cfg_shared(self):
        """CSR_WRITE_CDR_CFG: quad<<16 | 0x83F8 (shared per-quad)."""
        return (int(self.quad) << 16) | 0x83F8

    @property
    def ln_ctrl_shared(self):
        """CSR_WRITE_LN_CTRL: quad<<16 | 0x8830 (shared per-quad)."""
        return (int(self.quad) << 16) | 0x8830


# ---------------------------------------------------------------------------
# TOML-based configuration dataclasses
# ---------------------------------------------------------------------------


@dataclass
class LaneConfig:
    """Per-lane SerDes configuration matching the Gowin TOML schema.

    The gearbox is inside the GTR12 hard macro.  The ``fabric_tx_width``
    and ``fabric_rx_width`` properties compute the data width that the
    *fabric* sees after the internal gearbox has been applied.
    """

    enable: bool = True
    tx_data_rate: str = "5.0G"  # "1.25G", "2.5G", "5.0G", "10.3125G"
    rx_data_rate: str = "5.0G"
    width_mode: int = 10  # 8, 10, 16, 20, 32, 40, 64
    rx_separated_width_mode: int = 10
    encode_mode: str = "8b10b"  # "OFF", "8b10b", "64b66b", "128b130b"
    decode_mode: str = "8b10b"
    tx_gear_rate: str = "1:1"  # "1:1", "1:2", "1:4"
    rx_gear_rate: str = "1:1"
    word_align_enable: bool = True
    comma: str = "K28.5"
    comma_mask: str = "1111111111"
    tx_pol_invert: bool = False
    rx_pol_invert: bool = True
    loopback: str = "OFF"  # "OFF", "PCS", "PMA", "NEAREND"
    ctc_enable: bool = True  # Clock tolerance compensation (SKP)
    # TX equalization
    txlev: int = 15
    ffe_manual: bool = False
    ffe_cm: int = 0
    ffe_c1: int = 0
    ffe_c0: int = 40
    # RX equalization
    eq_manual: bool = False
    dr_rx_att: int = 7
    dr_rx_boost: int = 9
    sr_sd_thsel: int = 3  # Signal detect threshold
    rx_coupling: str = "AC"  # "AC", "DC"
    # CDR
    cdr_calib_clk_src: str = "REFMUX0"
    cdr_gc_counter: int = 250
    # CPLL
    cpll_reset_by_fabric: bool = True
    cpll_ref_sel: int = 0
    # PCS resets
    pcs_rx_reset_by_fabric: bool = True
    pcs_tx_reset_by_fabric: bool = True
    pcs_tx_clk_src: int = 0

    @property
    def fabric_tx_width(self) -> int:
        """Compute the fabric-facing TX data width from width_mode and gear_rate."""
        base = self.width_mode
        gear = {"1:1": 1, "1:2": 2, "1:4": 4}[self.tx_gear_rate]
        return base * gear

    @property
    def fabric_rx_width(self) -> int:
        """Compute the fabric-facing RX data width from width_mode and gear_rate."""
        base = (
            int(self.rx_separated_width_mode)
            if isinstance(self.rx_separated_width_mode, str)
            else self.rx_separated_width_mode
        )
        gear = {"1:1": 1, "1:2": 2, "1:4": 4}[self.rx_gear_rate]
        return base * gear


@dataclass
class QuadConfig:
    """Per-quad SerDes configuration."""

    enable: bool = True
    ref_pad0_freq: str = "125M"
    ref_pad1_freq: str = "0M"
    rx_eq_bias: int = 7
    cmu0_reset_by_fabric: bool = True
    cmu1_reset_by_fabric: bool = True
    # clock mux
    quad_clk_to_mac_sel: str = "CM0"
    mac_quad_clk_sel: str = "Q0"
    rx_quad_clk_internal_sel: str = "LN0_PMA_RX_CLK"
    tx_quad_clk_internal_sel: str = "CM0"
    # lanes
    lanes: list = field(default_factory=lambda: [LaneConfig() for _ in range(4)])


# ---------------------------------------------------------------------------
# Top-level config — bridges TOML world and CSR/hardware world
# ---------------------------------------------------------------------------


class SerDesConfig:
    """Top-level SerDes configuration matching the Gowin TOML schema.

    Bridges between the TOML parameter space and the hardware CSR map.
    For backward-compatible shorthand, ``quad`` / ``lane`` select the
    active quad and lane whose ``LaneCSR`` is exposed.

    Parameters
    ----------
    device : str
        Target device (e.g. "GW5AT-15").
    quads : list of QuadConfig, optional
        Per-quad configurations.  Defaults to one quad with default lanes.
    quad : Quad, optional
        Active quad selector for CSR address computation.
    lane : int, optional
        Active lane index (0-3) for CSR address computation.
    """

    def __init__(self, device="GW5AT-15", quads=None, quad=Quad.Q0, lane=0):
        self.device = device
        self.quads = quads or [QuadConfig()]
        self.quad = quad
        self.lane = lane
        self.lane_csr = LaneCSR(quad, lane)

    # ------------------------------------------------------------------
    # Convenience accessors
    # ------------------------------------------------------------------

    @property
    def active_lane(self) -> LaneConfig:
        """Return the LaneConfig for the active quad/lane."""
        qi = 0 if self.quad == Quad.Q0 else 1
        return self.quads[qi].lanes[self.lane]

    @property
    def fabric_tx_width(self) -> int:
        """Fabric-facing TX width for the active lane."""
        return self.active_lane.fabric_tx_width

    @property
    def fabric_rx_width(self) -> int:
        """Fabric-facing RX width for the active lane."""
        return self.active_lane.fabric_rx_width

    # ------------------------------------------------------------------
    # Init / FFE sequences (unchanged from original)
    # ------------------------------------------------------------------

    def init_sequence(self):
        """Return list of (addr, data) tuples for the init FSM.

        Matches the 11-entry FSM_INIT sequence from the Verilog upar_csr:
            cnt0=0:  TX_FFE_0       = 0x0000_F000
            cnt0=1:  TX_FFE_1       = 0x0000_0000
            cnt0=2:  TX_FFE_2       = 0x0000_0110
            cnt0=3:  CDR_CFG_shared = 0x0003_8002
            cnt0=4:  LN_CTRL_shared = 0xFFFF_F9FF
            cnt0=5:  CDR_CFG_0      = 0x7F00_0000
            cnt0=6:  CDR_CFG_1      = 0x007F_0000
            cnt0=7:  CDR_CFG_2      = 0x7F00_0000
            cnt0=8:  CDR_CFG_3      = 0x0000_004F
            cnt0=9:  CDR_CFG_4      = 0x0000_004F
            cnt0=10: CDR_CFG_5      = 0x0000_4F00
        """
        lc = self.lane_csr
        seq = [
            (lc.tx_ffe_0, 0x0000_F000),
            (lc.tx_ffe_1, 0x0000_0000),
            (lc.tx_ffe_2, 0x0000_0110),
            (lc.cdr_cfg_shared, 0x0003_8002),
            (lc.ln_ctrl_shared, 0xFFFF_F9FF),
            (lc.cdr_cfg_0, 0x7F00_0000),
            (lc.cdr_cfg_1, 0x007F_0000),
            (lc.cdr_cfg_2, 0x7F00_0000),
            (lc.cdr_cfg_3, 0x0000_004F),
            (lc.cdr_cfg_4, 0x0000_004F),
            (lc.cdr_cfg_5, 0x0000_4F00),
        ]
        return seq

    def rate_switch_5g_sequence(self):
        """Return list of (addr, data) tuples to switch from 10G to 5G.

        These are the 7 rate-dependent CSR registers that differ between
        5 Gbps (USB 3.0 Gen1) and 10 Gbps (USB 3.1 Gen2) operation.
        Writing these values converts a 10G-configured SerDes to 5G mode.

        The addresses are lane-independent (they live in the PLL/CDR/PCS
        shared register space).  Values from USB30_vs_USB31_CSR_COMPARISON.md.
        """
        return [
            (0x808120, 0x0000001A),  # PLL bandwidth (5G)
            (0x8082A0, 0x00003150),  # RX EQ bias (5G)
            (0x8082B8, 0x00000020),  # CDR loop config (5G)
            (0x8082C0, 0x00000210),  # TX serializer divider (5G)
            (0x808600, 0x0000011A),  # TX clock mux (5G)
            (0x808620, 0x00000016),  # RX clock mux (5G)
            (0x809000, 0x00000511),  # PCS width/gearbox (5G: 20-bit, 1:2)
        ]

    def rate_switch_10g_sequence(self):
        """Return list of (addr, data) tuples to switch from 5G to 10G.

        Inverse of :meth:`rate_switch_5g_sequence`.
        """
        return [
            (0x808120, 0x00000014),  # PLL bandwidth (10G)
            (0x8082A0, 0x00001170),  # RX EQ bias (10G)
            (0x8082B8, 0x00000030),  # CDR loop config (10G)
            (0x8082C0, 0x00000310),  # TX serializer divider (10G)
            (0x808600, 0x0000021A),  # TX clock mux (10G)
            (0x808620, 0x00000026),  # RX clock mux (10G)
            (0x809000, 0x00000F11),  # PCS width/gearbox (10G: 16-bit, 1:4)
        ]

    def init_sequence_with_rate_switch(self, target_rate="5G"):
        """Return init sequence extended with rate-switch writes.

        When the static CSR is configured at 10G and we want to start at
        5G (for USB 3.0 link training), the init FSM must write the 7
        rate-switch registers after the standard 11-step CDR/FFE init.

        Parameters
        ----------
        target_rate : str
            ``"5G"`` to downshift to 5 Gbps, ``"10G"`` to stay at 10 Gbps.
        """
        base = self.init_sequence()
        if target_rate == "5G":
            return base + self.rate_switch_5g_sequence()
        elif target_rate == "10G":
            return base + self.rate_switch_10g_sequence()
        else:
            return base

    def ffe_sequence(self, enable):
        """Return list of (addr, data) for FFE toggle.

        Parameters
        ----------
        enable : bool
            True to enable FFE (ffe_en low in Verilog = active), False to disable.
        """
        lc = self.lane_csr
        if enable:
            return [
                (lc.tx_ffe_1, 0x0000_0000),
                (lc.tx_ffe_2, 0x0000_0000),
                (lc.tx_ffe_2, 0x0000_0110),
            ]
        else:
            return [
                (lc.tx_ffe_1, 0x0000_0805),
                (lc.tx_ffe_2, 0x0000_0000),
                (lc.tx_ffe_2, 0x0000_0110),
            ]

    # ------------------------------------------------------------------
    # TOML I/O
    # ------------------------------------------------------------------

    @classmethod
    def from_toml(cls, path):
        """Load configuration from a Gowin SerDes TOML file.

        Parameters
        ----------
        path : str or pathlib.Path
            Path to the TOML file (e.g. ``serdes_tmp.toml``).
        """
        import tomllib

        with open(path, "rb") as f:
            data = tomllib.load(f)

        device = data.get("device", "GW5AT-15")
        quads = []

        for qi in range(2):
            qkey = f"q{qi}"
            if qkey not in data:
                continue
            qd = data[qkey]
            lanes = []
            for li in range(4):
                lkey = f"ln{li}"
                if lkey not in qd:
                    lanes.append(LaneConfig(enable=False))
                    continue
                ld = qd[lkey]
                lc = LaneConfig(
                    enable=ld.get("enable", False),
                    tx_data_rate=ld.get("tx_data_rate", "1.25G"),
                    rx_data_rate=ld.get("rx_data_rate", "1.25G"),
                    width_mode=ld.get("width_mode", 10),
                    rx_separated_width_mode=_to_int(
                        ld.get(
                            "rx_seperated_width_mode",
                            ld.get("rx_separated_width_mode", 10),
                        )
                    ),
                    encode_mode=ld.get("encode_mode", "OFF"),
                    decode_mode=ld.get("decode_mode", "OFF"),
                    tx_gear_rate=ld.get("tx_gear_rate", "1:1"),
                    rx_gear_rate=ld.get("rx_gear_rate", "1:1"),
                    word_align_enable=ld.get("word_align_enable", False),
                    comma=ld.get("comma", "K28.5"),
                    comma_mask=ld.get("comma_mask", "1111111111"),
                    tx_pol_invert=ld.get("tx_pol_invert", False),
                    rx_pol_invert=ld.get("rx_pol_invert", False),
                    loopback=ld.get("loopBack", ld.get("loopback", "OFF")),
                    ctc_enable=ld.get("ctc_enable", False),
                    txlev=ld.get("txlev", 15),
                    ffe_manual=ld.get("ffe_manual", False),
                    ffe_cm=ld.get("ffe_cm", 0),
                    ffe_c1=ld.get("ffe_c1", 0),
                    ffe_c0=ld.get("ffe_c0", 40),
                    eq_manual=ld.get("eq_manual", False),
                    dr_rx_att=ld.get("dr_rx_att", 7),
                    dr_rx_boost=ld.get("dr_rx_boost", 9),
                    sr_sd_thsel=ld.get("sr_sd_thsel", 3),
                    rx_coupling=ld.get("rx_coupling", "AC"),
                    cdr_calib_clk_src=ld.get("cdr_calib_clk_src", "REFMUX0"),
                    cdr_gc_counter=ld.get("cdr_gc_counter", 250),
                    cpll_reset_by_fabric=ld.get("cpll_reset_by_fabric", True),
                    cpll_ref_sel=ld.get("cpll_ref_sel", 0),
                    pcs_rx_reset_by_fabric=ld.get("pcs_rx_reset_by_fabric", True),
                    pcs_tx_reset_by_fabric=ld.get("pcs_tx_reset_by_fabric", True),
                    pcs_tx_clk_src=ld.get("pcs_tx_clk_src", 0),
                )
                lanes.append(lc)

            qc = QuadConfig(
                enable=qd.get("enable", True),
                ref_pad0_freq=qd.get("ref_pad0_freq", "125M"),
                ref_pad1_freq=qd.get("ref_pad1_freq", "0M"),
                rx_eq_bias=qd.get("rx_eq_bias", 7),
                cmu0_reset_by_fabric=qd.get("cmu0_reset_by_fabric", True),
                cmu1_reset_by_fabric=qd.get("cmu1_reset_by_fabric", True),
                quad_clk_to_mac_sel=qd.get("quad_clk_to_mac_sel", "CM0"),
                mac_quad_clk_sel=qd.get("mac_quad_clk_sel", "Q0"),
                rx_quad_clk_internal_sel=qd.get(
                    "rx_quad_clk_internal_sel", "LN0_PMA_RX_CLK"
                ),
                tx_quad_clk_internal_sel=qd.get("tx_quad_clk_internal_sel", "CM0"),
                lanes=lanes,
            )
            quads.append(qc)

        if not quads:
            quads = [QuadConfig()]

        # Determine active quad/lane: first enabled lane
        active_quad = Quad.Q0
        active_lane = 0
        for qi, qc in enumerate(quads):
            for li, lc in enumerate(qc.lanes):
                if lc.enable:
                    active_quad = Quad.Q0 if qi == 0 else Quad.Q1
                    active_lane = li
                    break
            else:
                continue
            break

        return cls(
            device=device,
            quads=quads,
            quad=active_quad,
            lane=active_lane,
        )

    def to_toml(self, path):
        """Write configuration to a TOML file compatible with Gowin tools.

        Parameters
        ----------
        path : str or pathlib.Path
            Output path.
        """
        lines = []
        lines.append(f'device = "{self.device}"')
        lines.append("")
        lines.append("")
        lines.append("[regulator]")
        lines.append("regulator_enable = false")
        lines.append("")

        for qi, qc in enumerate(self.quads):
            qkey = f"q{qi}"
            lines.append("")
            lines.append(f"[{qkey}]")
            lines.append(f"enable = {_bool_toml(qc.enable)}")
            lines.append(f'ref_pad0_freq = "{qc.ref_pad0_freq}"')
            lines.append(f'ref_pad1_freq = "{qc.ref_pad1_freq}"')
            lines.append(f"rx_eq_bias = {qc.rx_eq_bias}")
            lines.append(
                f"cmu0_reset_by_fabric = {_bool_toml(qc.cmu0_reset_by_fabric)}"
            )
            lines.append(
                f"cmu1_reset_by_fabric = {_bool_toml(qc.cmu1_reset_by_fabric)}"
            )
            lines.append(f'quad_clk_to_mac_sel = "{qc.quad_clk_to_mac_sel}"')
            lines.append(f'mac_quad_clk_sel = "{qc.mac_quad_clk_sel}"')
            lines.append(f'rx_quad_clk_internal_sel = "{qc.rx_quad_clk_internal_sel}"')
            lines.append(f'tx_quad_clk_internal_sel = "{qc.tx_quad_clk_internal_sel}"')
            lines.append("")

            for li, lc in enumerate(qc.lanes):
                lkey = f"ln{li}"
                lines.append("")
                lines.append(f"[{qkey}.{lkey}]")
                lines.append(f"enable = {_bool_toml(lc.enable)}")
                lines.append(f'tx_data_rate = "{lc.tx_data_rate}"')
                lines.append(f'rx_data_rate = "{lc.rx_data_rate}"')
                lines.append(f"width_mode = {lc.width_mode}")
                lines.append(
                    f'rx_seperated_width_mode = "{lc.rx_separated_width_mode}"'
                )
                lines.append(f'encode_mode = "{lc.encode_mode}"')
                lines.append(f'decode_mode = "{lc.decode_mode}"')
                lines.append(f'tx_gear_rate = "{lc.tx_gear_rate}"')
                lines.append(f'rx_gear_rate = "{lc.rx_gear_rate}"')
                lines.append(f"word_align_enable = {_bool_toml(lc.word_align_enable)}")
                lines.append(f'comma = "{lc.comma}"')
                lines.append(f'comma_mask = "{lc.comma_mask}"')
                lines.append(f"tx_pol_invert = {_bool_toml(lc.tx_pol_invert)}")
                lines.append(f"rx_pol_invert = {_bool_toml(lc.rx_pol_invert)}")
                lines.append(f'loopBack = "{lc.loopback}"')
                lines.append(f"ctc_enable = {_bool_toml(lc.ctc_enable)}")
                lines.append(f"txlev = {lc.txlev}")
                lines.append(f"ffe_manual = {_bool_toml(lc.ffe_manual)}")
                lines.append(f"ffe_cm = {lc.ffe_cm}")
                lines.append(f"ffe_c1 = {lc.ffe_c1}")
                lines.append(f"ffe_c0 = {lc.ffe_c0}")
                lines.append(f"eq_manual = {_bool_toml(lc.eq_manual)}")
                lines.append(f"dr_rx_att = {lc.dr_rx_att}")
                lines.append(f"dr_rx_boost = {lc.dr_rx_boost}")
                lines.append(f"sr_sd_thsel = {lc.sr_sd_thsel}")
                lines.append(f'rx_coupling = "{lc.rx_coupling}"')
                lines.append(f'cdr_calib_clk_src = "{lc.cdr_calib_clk_src}"')
                lines.append(f"cdr_gc_counter = {lc.cdr_gc_counter}")
                lines.append(
                    f"cpll_reset_by_fabric = {_bool_toml(lc.cpll_reset_by_fabric)}"
                )
                lines.append(f"cpll_ref_sel = {lc.cpll_ref_sel}")
                lines.append(
                    f"pcs_rx_reset_by_fabric = {_bool_toml(lc.pcs_rx_reset_by_fabric)}"
                )
                lines.append(
                    f"pcs_tx_reset_by_fabric = {_bool_toml(lc.pcs_tx_reset_by_fabric)}"
                )
                lines.append(f"pcs_tx_clk_src = {lc.pcs_tx_clk_src}")

        with open(path, "w") as f:
            f.write("\n".join(lines))
            f.write("\n")

    # ------------------------------------------------------------------
    # Convenience constructors
    # ------------------------------------------------------------------

    @classmethod
    def usb30_5g(cls, quad=Quad.Q0, lane=0):
        """Convenience: USB 3.0 Gen1, 5Gbps, 8b10b, 125MHz CPLL.

        width_mode=10, gear=1:1 → fabric sees 10-bit TX/RX.
        """
        lanes = [LaneConfig(enable=False) for _ in range(4)]
        lanes[lane] = LaneConfig(
            enable=True,
            tx_data_rate="5.0G",
            rx_data_rate="5.0G",
            width_mode=10,
            rx_separated_width_mode=10,
            encode_mode="8b10b",
            decode_mode="8b10b",
            tx_gear_rate="1:1",
            rx_gear_rate="1:1",
            word_align_enable=True,
            comma="K28.5",
            rx_pol_invert=True,
            ctc_enable=True,
            cdr_calib_clk_src="REFMUX0",
            cpll_reset_by_fabric=True,
        )
        qc = QuadConfig(enable=True, lanes=lanes)
        return cls(device="GW5AT-15", quads=[qc], quad=quad, lane=lane)

    @classmethod
    def usb31_10g(cls, quad=Quad.Q0, lane=0):
        """Convenience: USB 3.1 Gen2, 10.3125Gbps, 128b130b, 125MHz CPLL.

        width_mode=32, gear=1:2 → fabric sees 64-bit TX/RX.
        """
        lanes = [LaneConfig(enable=False) for _ in range(4)]
        lanes[lane] = LaneConfig(
            enable=True,
            tx_data_rate="10.3125G",
            rx_data_rate="10.3125G",
            width_mode=32,
            rx_separated_width_mode=32,
            encode_mode="128b130b",
            decode_mode="128b130b",
            tx_gear_rate="1:2",
            rx_gear_rate="1:2",
            word_align_enable=False,
            ctc_enable=False,
            cdr_calib_clk_src="REFMUX0",
            cpll_reset_by_fabric=True,
        )
        qc = QuadConfig(enable=True, lanes=lanes)
        return cls(device="GW5AT-15", quads=[qc], quad=quad, lane=lane)

    @classmethod
    def raw_1_25g(cls, quad=Quad.Q0, lane=0, width=10):
        """Convenience: Raw 1.25Gbps, no encoding, configurable width.

        Parameters
        ----------
        width : int
            Base width_mode (8, 10, 16, 20, etc.).  Gear is 1:1 so
            fabric width equals width_mode.
        """
        lanes = [LaneConfig(enable=False) for _ in range(4)]
        lanes[lane] = LaneConfig(
            enable=True,
            tx_data_rate="1.25G",
            rx_data_rate="1.25G",
            width_mode=width,
            rx_separated_width_mode=width,
            encode_mode="OFF",
            decode_mode="OFF",
            tx_gear_rate="1:1",
            rx_gear_rate="1:1",
            word_align_enable=False,
            ctc_enable=False,
        )
        qc = QuadConfig(enable=True, lanes=lanes)
        return cls(device="GW5AT-15", quads=[qc], quad=quad, lane=lane)

    # ------------------------------------------------------------------
    # repr
    # ------------------------------------------------------------------

    def __repr__(self):
        al = self.active_lane
        return (
            f"SerDesConfig(device={self.device!r}, "
            f"quad={self.quad.name}, lane={self.lane}, "
            f"rate={al.tx_data_rate}, encode={al.encode_mode}, "
            f"fabric_tx={self.fabric_tx_width}, fabric_rx={self.fabric_rx_width})"
        )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _bool_toml(v):
    """Convert Python bool to TOML-style lowercase string."""
    return "true" if v else "false"


def _to_int(v):
    """Coerce a string or int to int."""
    if isinstance(v, str):
        return int(v)
    return int(v)


# ═══════════════════════════════════════════════════════════════════════════
#  PIPE adapter interface — CSR enum, address helper, constants, and tables
# ═══════════════════════════════════════════════════════════════════════════


class CSR(IntEnum):
    """Logical CSR register selector used by the PIPE adapter layer.

    Each member maps to a ``LaneCSR`` property via :func:`csr_addr`.
    """

    EIDLE = 0  # Electrical-idle control
    RXDET_PULSE = 1  # Receiver-detection pulse trigger
    RXDET_RESULT = 2  # Receiver-detection result readback
    RX_POLARITY = 3  # PCS RX polarity inversion
    PCS_8B10B = 4  # PCS 8b10b encode/decode bypass
    TX_FFE_C0 = 5  # TX FFE coefficient 0
    TX_FFE_C1 = 6  # TX FFE coefficient 1
    TX_FFE_VDDT = 7  # TX FFE voltage / driver trim


def csr_addr(csr: CSR, quad: int = 0, lane: int = 0) -> int:
    """Compute the 24-bit DRP address for a logical CSR register.

    Parameters
    ----------
    csr : CSR
        Logical register selector.
    quad : int
        Quad index (0 or 1).
    lane : int
        Lane index within quad (0-3).

    Returns
    -------
    int
        24-bit DRP address suitable for ``Signal(24)``.
    """
    q = Quad.Q0 if quad == 0 else Quad.Q1
    lc = LaneCSR(quad=q, lane=lane)

    _map = {
        CSR.EIDLE: lc.eidle_addr,
        CSR.RXDET_PULSE: lc.pulse_addr,
        CSR.RXDET_RESULT: lc.rxdet_addr,
        CSR.RX_POLARITY: lc.rx_polarity_addr,
        CSR.PCS_8B10B: lc.pcs_8b10b_addr,
        CSR.TX_FFE_C0: lc.tx_ffe_0,
        CSR.TX_FFE_C1: lc.tx_ffe_1,
        CSR.TX_FFE_VDDT: lc.tx_ffe_2,
    }
    return _map[csr]


# ── DRP write-data constants ──────────────────────────────────────────────

EIDLE_ON = 0x00000001  # Assert electrical idle
EIDLE_OFF = 0x00000007  # Deassert electrical idle

RXDET_START = 0x03000000  # Start receiver-detection pulse
RXDET_END = 0x00000000  # End receiver-detection pulse

RX_POLARITY_NORMAL = 0x00000000  # RX polarity normal
RX_POLARITY_INVERT = 0x00000001  # RX polarity inverted

BYPASS_8B10B = 0x00000001  # Bypass 8b10b encode/decode


# ── Table generators ──────────────────────────────────────────────────────


def lfps_ffe_regs(quad: int = 0, lane: int = 0) -> list:
    """Return FFE register (addr, normal_val, lfps_val, desc) tuples.

    During LFPS signaling the TX driver must be reconfigured to produce
    low-frequency square-wave signaling.  These three registers hold the
    normal (high-speed) and LFPS (low-swing) FFE settings.

    Parameters
    ----------
    quad, lane : int
        Quad and lane indices.

    Returns
    -------
    list[tuple[int, int, int, str]]
        ``[(address, normal_value, lfps_value, description), ...]``
    """
    q = Quad.Q0 if quad == 0 else Quad.Q1
    lc = LaneCSR(quad=q, lane=lane)
    return [
        (lc.tx_ffe_0, 0x0000F000, 0x00000040, "TX FFE_0"),
        (lc.tx_ffe_1, 0x00000000, 0x00000001, "TX FFE_1"),
        (lc.tx_ffe_2, 0x00000110, 0x00000003, "TX FFE_2"),
    ]


def rate_change_regs(quad: int = 0, lane: int = 0) -> list:
    """Return rate-change register (addr, gen1_val, gen2_val, desc) tuples.

    These registers must be rewritten atomically (under DRP lock) when
    switching between Gen1 (2.5 Gbps) and Gen2 (5.0 Gbps) line rates.

    Parameters
    ----------
    quad, lane : int
        Quad and lane indices.

    Returns
    -------
    list[tuple[int, int, int, str]]
        ``[(address, gen1_value, gen2_value, description), ...]``
    """
    q = Quad.Q0 if quad == 0 else Quad.Q1
    lc = LaneCSR(quad=q, lane=lane)

    # CPLL divider is in a per-lane register at quad<<16 | (0xA020 + lane*0x200)
    cpll_addr = (int(q) << 16) | (0xA020 + lane * 0x200)
    # TX/RX clock source at PCS base offsets
    tx_clk_addr = lc.pcs_base | 0x600  # Approximate — TX clock source
    rx_clk_addr = lc.pcs_base | 0x620  # Approximate — RX clock source
    pcs_rate_addr = lc.pcs_base  # PCS rate mode

    return [
        (cpll_addr, 0x0000001A, 0x00000014, "CPLL divider ratio"),
        (lc.afe_base | 0xA0, 0x00003150, 0x00001170, "RX AFE gain/attenuation"),
        (lc.afe_base | 0xB8, 0x00000020, 0x00000040, "RX AFE bias current"),
        (lc.afe_base | 0xC0, 0x00000210, 0x00000310, "RX AFE boost"),
        (tx_clk_addr, 0x0000011A, 0x0000021A, "TX clock source select"),
        (rx_clk_addr, 0x00000016, 0x00000026, "RX clock source select"),
        (pcs_rate_addr, 0x00000001, 0x00000003, "PCS rate mode"),
    ]


def csr_init_table(quad: int = 0, lane: int = 0) -> list:
    """Return the power-on CSR initialization sequence.

    These 11 DRP writes configure the TX driver (FFE), CDR loop filter,
    and lane control registers.  They must be performed in order before
    any PIPE operations.

    Parameters
    ----------
    quad, lane : int
        Quad and lane indices.

    Returns
    -------
    list[tuple[str, int, int]]
        ``[(name, address, data), ...]``
    """
    q = Quad.Q0 if quad == 0 else Quad.Q1
    lc = LaneCSR(quad=q, lane=lane)
    return [
        ("TX_FFE_0", lc.tx_ffe_0, 0x0000F000),
        ("TX_FFE_1", lc.tx_ffe_1, 0x00000000),
        ("TX_FFE_2", lc.tx_ffe_2, 0x00000110),
        ("CDR_CFG_shared", lc.cdr_cfg_shared, 0x00038002),
        ("LN_CTRL_shared", lc.ln_ctrl_shared, 0xFFFFF9FF),
        ("CDR_CFG_0", lc.cdr_cfg_0, 0x7F000000),
        ("CDR_CFG_1", lc.cdr_cfg_1, 0x007F0000),
        ("CDR_CFG_2", lc.cdr_cfg_2, 0x7F000000),
        ("CDR_CFG_3", lc.cdr_cfg_3, 0x0000004F),
        ("CDR_CFG_4", lc.cdr_cfg_4, 0x0000004F),
        ("CDR_CFG_5", lc.cdr_cfg_5, 0x00004F00),
    ]


def runtime_addrs(quad: int = 0, lane: int = 0) -> dict:
    """Return a dict of register-name → 24-bit DRP address.

    Provides a convenient lookup for all CSR addresses relevant at
    runtime (after init).

    Parameters
    ----------
    quad, lane : int
        Quad and lane indices.

    Returns
    -------
    dict[str, int]
    """
    q = Quad.Q0 if quad == 0 else Quad.Q1
    lc = LaneCSR(quad=q, lane=lane)
    return {
        "eidle": lc.eidle_addr,
        "rxdet_pulse": lc.pulse_addr,
        "rxdet_result": lc.rxdet_addr,
        "rx_polarity": lc.rx_polarity_addr,
        "pcs_8b10b": lc.pcs_8b10b_addr,
        "tx_ffe_0": lc.tx_ffe_0,
        "tx_ffe_1": lc.tx_ffe_1,
        "tx_ffe_2": lc.tx_ffe_2,
        "cdr_cfg_shared": lc.cdr_cfg_shared,
        "ln_ctrl_shared": lc.ln_ctrl_shared,
        "loopback": lc.loopback_addr,
    }
