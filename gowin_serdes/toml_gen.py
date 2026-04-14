"""TOML configuration generator for Gowin SerDes.

Generates TOML files compatible with Gowin's ``serdes_toml_to_csr_*k.bin``
tools directly from the live GowinSerDes / GowinSerDesGroup / LaneConfig
object graph.  No external script required.

The generated TOML is structurally identical to what the Gowin IDE produces
(see ``gen/gw138-2/serdes/serdes_tmp.toml`` for a reference).
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List, Optional

from .config import (
    DEVICE_META,
    ChannelBondingDepth,
    EncodingMode,
    GearRate,
    GowinDevice,
    LaneConfig,
    _is_138,
    device_num_quads,
)

if TYPE_CHECKING:
    from .group import GowinSerDesGroup


# ═══════════════════════════════════════════════════════════════════════════════
# TOML FORMATTING
# ═══════════════════════════════════════════════════════════════════════════════


def _fmt(v: Any) -> str:
    """Format a Python value as a TOML literal."""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return str(v)
    if isinstance(v, str):
        return f'"{v}"'
    return str(v)


def _write_toml(config: Dict, path: str) -> None:
    """Serialize *config* dict to a Gowin-compatible TOML file at *path*."""
    lines: List[str] = []

    # Root keys
    if "device" in config:
        lines.append(f'device = "{config["device"]}"')
        lines.append("")

    # Regulator section
    if "regulator" in config:
        lines.append("")
        lines.append("[regulator]")
        for k, v in config["regulator"].items():
            lines.append(f"{k} = {_fmt(v)}")
        lines.append("")

    # Quad and lane sections
    for qi in range(2):
        qkey = f"q{qi}"
        if qkey not in config:
            continue
        lines.append("")
        lines.append(f"[{qkey}]")
        for k, v in config[qkey].items():
            if isinstance(v, dict):
                continue  # lane sub-sections handled below
            lines.append(f"{k} = {_fmt(v)}")
        lines.append("")

        # Lane sections
        for li in range(4):
            full_key = f"{qkey}.ln{li}"
            lane_data: Optional[Dict] = None
            if full_key in config:
                lane_data = config[full_key]
            elif f"ln{li}" in config.get(qkey, {}):
                lane_data = config[qkey][f"ln{li}"]
            if lane_data is None:
                continue

            lines.append("")
            lines.append(f"[{full_key}]")
            for k, v in lane_data.items():
                lines.append(f"{k} = {_fmt(v)}")
            lines.append("")

    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


# ═══════════════════════════════════════════════════════════════════════════════
# DEFAULT BUILDERS (same semantics as serdes_toml_gen.py)
# ═══════════════════════════════════════════════════════════════════════════════


def _default_quad_config(
    quad_id: int,
    enabled: bool,
    has_extra_pads: bool,
    num_quads: int = 1,
    device: GowinDevice = GowinDevice.GW5AT_15,
) -> Dict[str, Any]:
    """Quad-level TOML defaults.

    Matches the Gowin IDE output for single-quad (15K) and
    multi-quad (138K) devices.

    For GW5AST-138 the 125 MHz reference clock enters on Q1's REFPAD0
    and is propagated to Q0 via inter-quad routing:
    - Q0: ``ref_pad0_freq="0M"``, ``refimux0_sel=2``, ``ref_prop_dir=2``
    - Q1: ``ref_pad0_freq="125M"``, ``ref_prop_dir=2``
    """
    is_multi = num_quads > 1
    is_gw5ast = device == GowinDevice.GW5AST_138

    cfg: Dict[str, Any] = {
        "enable": enabled,
        "cmu0_reset_by_fabric": False,
        "cmu1_reset_by_fabric": False,
        "quad_clk_to_mac_sel": "CM0",
        "mac_quad_clk_sel": "Q0",
        "pd_toggle_by_fabric": False,
        "lane_reset_by_fabric": True,
        "por_toggle_by_fabric": False,
        "ref_pad1_freq": "0M",
        "rx_eq_bias": 7,
        "rx_quad_clk_internal_sel": "LN0_PMA_RX_CLK",
        "rx_quad_clk_sel": "Internal",
        "tx_quad_clk_internal_sel": "CM0",
        "tx_quad_clk_sel": "Internal",
        "refimux0_sel": 0,
        "refimux1_sel": 0,
        "qpll0_ref_sel": 0,
        "qpll1_ref_sel": 0,
        "refmux_scheme": "USER_DEFINED",
    }

    # Single-quad devices include mclk/gpio fields
    if not is_multi:
        cfg["mclk_freq"] = "0M"
        cfg["gpio_freq"] = "0M"

    if is_gw5ast:
        # GW5AST-138 inter-quad clock routing: Q1 has the 125 MHz ref
        # pad, Q0 receives it via refprop.
        if quad_id == 0:
            cfg["ref_pad0_freq"] = "0M"
            cfg["refimux0_sel"] = 2
        else:
            cfg["ref_pad0_freq"] = "125M"
        cfg["ref_prop_dir"] = 2
        cfg["refomux0_sel"] = 0
    elif is_multi:
        # Generic multi-quad (GW5AT-138): default routing
        cfg["ref_pad0_freq"] = "125M"
        cfg["ref_prop_dir"] = 1
        cfg["refomux0_sel"] = 0
    else:
        cfg["ref_pad0_freq"] = "125M"

    if has_extra_pads:
        cfg["ref_pad2_freq"] = "0M"
        cfg["ref_pad3_freq"] = "0M"
        cfg["gpio0_freq"] = "0M"
        cfg["gpio1_freq"] = "0M"
        cfg["gpio2_freq"] = "0M"
        cfg["gpio3_freq"] = "0M"
        cfg["refimux2_sel"] = 0
        cfg["refimux3_sel"] = 0
    return cfg


def _default_lane_config(
    quad_id: int, lane_id: int, device: GowinDevice = GowinDevice.GW5AT_15
) -> Dict[str, Any]:
    """Lane-level TOML defaults (disabled lane).

    Key ordering matches the Gowin IDE output so that a diff against
    the reference ``serdes_tmp.toml`` is clean.
    """
    qln = f"q{quad_id}.ln{lane_id}"
    cfg: Dict[str, Any] = {
        "cpll_reset_by_fabric": False,
        "chbond_align_pattern1": 124,
        "chbond_align_pattern1_is_kcode": False,
        "chbond_align_pattern2": 124,
        "chbond_align_pattern2_is_kcode": False,
        "chbond_align_pattern3": 124,
        "chbond_align_pattern3_is_kcode": False,
        "decode_mode": "OFF",
        "encode_mode": "OFF",
        "preamEn": False,
        "rxBistInv": False,
        "rxPattern": "PRBS31",
        "txBistInv": False,
        "txPattern": "PRBS31",
        "cdr_gc_counter": 250,
        "cdr_calib_clk_src": "AUTO",
        "chbond_enable": False,
        "chbond_align_length": 1,
        "chbond_align_pattern0": 124,
        "chbond_mst_sel": qln,
        "chbond_max_skew": 8,
        "ctc_enable": False,
        "ctc_skipb_pattern": 28,
        "ctc_skipb_pattern_is_kcode": False,
        "ctc_clk_src": "fabric_c2i_clk",
        "ctc_mst_sel": qln,
        "ctc_skipa_pattern": 28,
        "ctc_rd_start_depth": "8",
        "enable": False,
        "loopBack": "OFF",
        "pcs_rx_reset_by_fabric": True,
        "pcs_tx_reset_by_fabric": True,
        "pcs_tx_clk_src": 0,
        "width_mode": 10,
        "dr_rx_att": 7,
        "dr_rx_att_boost": 0,
        "rx_bit_invert": False,
        "chbond_clk_src": "lane",
        "chbond_cfg_rd_start_depth": 8,
        "dr_rx_boost": 9,
        "rx_byte_invert": False,
        "rx_coupling": "AC",
        "rx_data_manipulation_enable": False,
        "eq_manual": False,
        "rx_gear_rate": "1:1",
        "rx_if_cfg_rd_start_depth": 8,
        "locked_from_fabric": False,
        "rx_ovs_mode": "OFF",
        "rx_ovs_pll_src": "N/A",
        "rx_ovs_ratio": "N/A",
        "rx_pol_invert": False,
        "rx_data_rate": "1.25G",
        "sr_sd_thsel": 3,
        "rx_slip_distance": 8,
        "idle_high_filter": 0,
        "idle_low_filter": 0,
        "chbond_trigger_by_fabric": True,
        "ctc_skipb_pattern_enable": False,
        "tx_bit_invert": False,
        "tx_byte_invert": False,
        "tx_data_manipulation_enable": False,
        "txlev": 15,
        "ffe_c1": 0,
        "ffe_cm": 0,
        "ffe_manual": False,
        "tx_gear_rate": "1:1",
        "tx_if_cfg_mst_sel": qln,
        "tx_if_cfg_rd_start_depth": 8,
        "tx_ovs_mode": "OFF",
        "tx_ovs_ratio": "N/A",
        "tx_pol_invert": False,
        "tx_data_rate": "1.25G",
        "tx_slip_distance": 8,
        "vddt": 900.0,
        "word_align_enable": False,
        "comma": "K28.5",
        "comma_mask": "1111111111",
        "ffe_c0": 40,
        "cpll_ref_sel": 0,
    }
    # GW5AT-15 includes extra fields not present in the 138K reference
    if not _is_138(device):
        cfg["rx_seperated_width_mode"] = "10"
        cfg["10GBASE-R"] = False
        cfg["ilk_metaframe_len"] = "32"
        cfg["rxsd_use_dlogic"] = False
    return cfg


# ═══════════════════════════════════════════════════════════════════════════════
# LaneConfig → TOML lane dict
# ═══════════════════════════════════════════════════════════════════════════════

_ENCODING_MAP = {
    EncodingMode.OFF: "OFF",
    EncodingMode.B8B10B: "8B10B",
    EncodingMode.B64B66B: "64B66B",
    EncodingMode.B64B67B: "64B67B",
}

_GEAR_MAP = {
    GearRate.G1_1: "1:1",
    GearRate.G1_2: "1:2",
    GearRate.G1_4: "1:4",
}


def _pcs_tx_clk_src(lane_cfg: LaneConfig) -> int:
    """Derive pcs_tx_clk_src from the TX data rate.

    0 = internal (≤1.5 Gbps), 1 = from PLL (>1.5 Gbps).
    """
    rate_ghz = float(lane_cfg.tx_data_rate.rstrip("G"))
    return 1 if rate_ghz > 1.5 else 0


def _build_enabled_lane(
    lane_cfg: LaneConfig,
    quad_id: int,
    lane_id: int,
    group: "GowinSerDesGroup",
    device: GowinDevice = GowinDevice.GW5AT_15,
) -> Dict[str, Any]:
    """Build a complete TOML lane dict for an *enabled* lane.

    Key ordering matches the Gowin IDE output so that a diff against the
    reference ``serdes_tmp.toml`` is clean.
    """
    qln = f"q{quad_id}.ln{lane_id}"
    master_lane_id = group.first_lane
    master_qln = f"q{quad_id}.ln{master_lane_id}"
    has_bonding = group.chbond_master is not None and group.num_lanes > 1

    cfg: Dict[str, Any] = {
        "tx_data_rate": lane_cfg.tx_data_rate,
        "rx_data_rate": lane_cfg.rx_data_rate,
        "pcs_tx_clk_src": _pcs_tx_clk_src(lane_cfg),
        "loopBack": "OFF",
        "enable": True,
        "width_mode": lane_cfg.width_mode,
        "tx_gear_rate": _GEAR_MAP[lane_cfg.tx_gear_rate],
        "rx_gear_rate": _GEAR_MAP[lane_cfg.rx_gear_rate],
        "cpll_reset_by_fabric": False,
        "chbond_align_pattern1": 124,
        "chbond_align_pattern1_is_kcode": False,
        "chbond_align_pattern2": 124,
        "chbond_align_pattern2_is_kcode": False,
        "chbond_align_pattern3": 124,
        "chbond_align_pattern3_is_kcode": False,
        "decode_mode": _ENCODING_MAP[lane_cfg.rx_encoding],
        "encode_mode": _ENCODING_MAP[lane_cfg.tx_encoding],
        "preamEn": False,
        "rxBistInv": False,
        "rxPattern": "PRBS31",
        "txBistInv": False,
        "txPattern": "PRBS31",
        "cdr_gc_counter": 250,
        "cdr_calib_clk_src": "AUTO",
        "chbond_enable": has_bonding,
        "chbond_align_length": group.num_lanes if has_bonding else 1,
        "chbond_align_pattern0": 124,
        "chbond_mst_sel": master_qln if has_bonding else qln,
        "chbond_max_skew": 8,
        "ctc_enable": lane_cfg.ctc_enable,
        "ctc_skipb_pattern": 28,
        "ctc_skipb_pattern_is_kcode": False,
        "ctc_clk_src": "fabric_c2i_clk",
        "ctc_mst_sel": qln,
        "ctc_skipa_pattern": 28,
        "ctc_rd_start_depth": "8",
        "pcs_rx_reset_by_fabric": True,
        "pcs_tx_reset_by_fabric": True,
        "dr_rx_att": 7,
        "dr_rx_att_boost": 0,
        "rx_bit_invert": False,
        "chbond_clk_src": "lane",
        "chbond_cfg_rd_start_depth": 8,
        "dr_rx_boost": 9,
        "rx_byte_invert": False,
        "rx_coupling": "AC",
        "rx_data_manipulation_enable": False,
        "eq_manual": False,
        "rx_if_cfg_rd_start_depth": 8,
        "locked_from_fabric": False,
        "rx_ovs_mode": "OFF",
        "rx_ovs_pll_src": "N/A",
        "rx_ovs_ratio": "N/A",
        "rx_pol_invert": False,
        "sr_sd_thsel": 3,
        "rx_slip_distance": 8,
        "idle_high_filter": 0,
        "idle_low_filter": 0,
        "chbond_trigger_by_fabric": True,
        "ctc_skipb_pattern_enable": False,
        "tx_bit_invert": False,
        "tx_byte_invert": False,
        "tx_data_manipulation_enable": False,
        "txlev": 15,
        "ffe_c1": 0,
        "ffe_cm": 0,
        "ffe_manual": False,
        "tx_if_cfg_mst_sel": master_qln if has_bonding else qln,
        "tx_if_cfg_rd_start_depth": 8,
        "tx_ovs_mode": "OFF",
        "tx_ovs_ratio": "N/A",
        "tx_pol_invert": False,
        "tx_slip_distance": 8,
        "vddt": 900.0,
        "word_align_enable": lane_cfg.word_align,
        "comma": "K28.5",
        "comma_mask": "1111111111",
        "ffe_c0": 40,
        "cpll_ref_sel": 0,
    }
    # GW5AT-15 includes extra fields not present in the 138K reference
    if not _is_138(device):
        cfg["rx_seperated_width_mode"] = str(lane_cfg.width_mode)
        cfg["10GBASE-R"] = False
        cfg["ilk_metaframe_len"] = "32"
        cfg["rxsd_use_dlogic"] = False
    return cfg


# ═══════════════════════════════════════════════════════════════════════════════
# PUBLIC API
# ═══════════════════════════════════════════════════════════════════════════════


def build_toml_config(
    device: GowinDevice,
    groups: List["GowinSerDesGroup"],
) -> Dict:
    """Build the full TOML config dict from a device + group list.

    This is the pure-data equivalent of ``serdes_toml_gen.py``'s
    ``build_config()``, but driven entirely by the GowinSerDes object graph
    instead of CLI arguments.
    """
    meta = DEVICE_META[device]
    toml_device_name, _, num_quads, has_extra_pads = meta

    # Determine which quads are in use
    used_quads = {g.quad for g in groups}

    config: Dict[str, Any] = {
        "device": toml_device_name,
        "regulator": {"regulator_enable": False},
    }

    # Build per-quad and per-lane sections
    for qi in range(num_quads):
        qkey = f"q{qi}"
        enabled = qi in used_quads
        qcfg = _default_quad_config(
            qi, enabled, has_extra_pads, num_quads, device=device
        )

        # If any lane in this quad is enabled, use its ref_clk_freq
        # (but not for GW5AST-138 where clock routing is handled by
        # _default_quad_config based on quad_id)
        if enabled and device != GowinDevice.GW5AST_138:
            for g in groups:
                if g.quad == qi and g.lane_configs:
                    qcfg["ref_pad0_freq"] = g.lane_configs[0].ref_clk_freq
                    break

        config[qkey] = qcfg

        # Create all 4 lane slots for this quad
        for li in range(4):
            # Check if this lane belongs to a group
            lane_group = None
            lane_offset = -1
            for g in groups:
                if g.quad != qi:
                    continue
                off = li - g.first_lane
                if 0 <= off < g.num_lanes:
                    lane_group = g
                    lane_offset = off
                    break

            if lane_group is not None:
                lcfg_toml = _build_enabled_lane(
                    lane_group.lane_configs[lane_offset],
                    qi,
                    li,
                    lane_group,
                    device=device,
                )
            else:
                lcfg_toml = _default_lane_config(qi, li, device=device)

            config[f"{qkey}.ln{li}"] = lcfg_toml

    return config


def generate_toml(
    device: GowinDevice,
    groups: List["GowinSerDesGroup"],
    output_path: str,
) -> str:
    """Generate a Gowin-compatible TOML file and return the output path.

    Parameters
    ----------
    device : GowinDevice
        Target FPGA device.
    groups : list of GowinSerDesGroup
        The groups exactly as passed to GowinSerDes().
    output_path : str
        Where to write the TOML file.

    Returns
    -------
    str
        The *output_path* (for chaining convenience).
    """
    config = build_toml_config(device, groups)
    _write_toml(config, output_path)
    return output_path


def generate_csr(
    device: GowinDevice,
    groups: List["GowinSerDesGroup"],
    output_path: str,
    toml_path: Optional[str] = None,
    gowin_bin_dir: Optional[str] = None,
) -> str:
    """Generate a ``.csr`` file from the live object graph.

    1. Writes a temporary (or caller-specified) TOML file.
    2. Invokes ``serdes_toml_to_csr_*k.bin <toml> -o <csr>``.
    3. Returns the CSR output path.

    Parameters
    ----------
    device : GowinDevice
        Target FPGA device.
    groups : list of GowinSerDesGroup
        Groups exactly as passed to GowinSerDes().
    output_path : str
        Where to write the ``.csr`` file.
    toml_path : str or None
        If given, write the intermediate TOML here (and keep it).
        If None, a temp file is used and cleaned up.
    gowin_bin_dir : str or None
        Path to Gowin IDE ``bin/`` directory.  If None the tool binary
        is looked up on ``$PATH`` and then ``$GOWIN_IDE/bin/``.

    Returns
    -------
    str
        The *output_path* (for chaining convenience).

    Raises
    ------
    FileNotFoundError
        If the Gowin CSR conversion tool is not found.
    subprocess.CalledProcessError
        If the tool exits with a non-zero return code.
    """
    meta = DEVICE_META[device]
    _, csr_suffix, _, _ = meta
    tool_name = f"serdes_toml_to_csr_{csr_suffix}.bin"

    # Resolve tool binary
    tool_path = _find_tool(tool_name, gowin_bin_dir)

    # Generate TOML
    cleanup_toml = toml_path is None
    if toml_path is None:
        fd, toml_path = tempfile.mkstemp(suffix=".toml", prefix="serdes_")
        os.close(fd)

    try:
        generate_toml(device, groups, toml_path)

        # Invoke the Gowin binary
        cmd = [tool_path, toml_path, "-o", output_path]
        subprocess.run(cmd, check=True, capture_output=True, text=True)
    finally:
        if cleanup_toml and os.path.exists(toml_path):
            os.remove(toml_path)

    return output_path


def _find_tool(tool_name: str, gowin_bin_dir: Optional[str] = None) -> str:
    """Locate a Gowin tool binary.

    Search order:
    1. *gowin_bin_dir* (explicit override)
    2. ``$PATH``
    3. ``$GOWIN_IDE/bin/`` (and ``bin/serdes_toml_to_csr.dist/``)
    4. Common install locations
    """
    # Explicit dir
    if gowin_bin_dir:
        candidate = os.path.join(gowin_bin_dir, tool_name)
        if os.path.isfile(candidate):
            return candidate
        raise FileNotFoundError(f"{tool_name} not found in {gowin_bin_dir}")

    # $PATH
    found = shutil.which(tool_name)
    if found:
        return found

    # $GOWIN_IDE/bin/  (Gowin packages the binary inside a .dist subfolder)
    gowin_ide = os.environ.get("GOWIN_IDE")
    if gowin_ide:
        for subdir in ["bin", "bin/serdes_toml_to_csr.dist"]:
            candidate = os.path.join(gowin_ide, subdir, tool_name)
            if os.path.isfile(candidate):
                return candidate

    # Common locations (check both bin/ and bin/serdes_toml_to_csr.dist/)
    for base in [
        Path.home() / "Downloads" / "gowin" / "IDE",
        Path("/opt/gowin/IDE"),
        Path("/usr/local/gowin/IDE"),
    ]:
        for subdir in ["bin", "bin/serdes_toml_to_csr.dist"]:
            candidate = str(base / subdir / tool_name)
            if os.path.isfile(candidate):
                return candidate

    raise FileNotFoundError(
        f"Gowin tool '{tool_name}' not found.  Either:\n"
        f"  1. Add it to $PATH\n"
        f"  2. Set $GOWIN_IDE to your Gowin IDE installation directory\n"
        f"  3. Pass gowin_bin_dir= explicitly"
    )
