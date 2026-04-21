#!/usr/bin/env python3
"""Compare gowin_serdes TOML generation against Gowin IDE reference files.

Covers all three devices: GW5AT-15 (gw15), GW5AT-60 (gw60), GW5AST-138 (gw138).
"""

import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from gowin_serdes.config import (
    GowinDevice,
    LaneConfig,
    PLLSelection,
    RefClkSource,
)
from gowin_serdes.group import GowinSerDesGroup
from gowin_serdes.toml_gen import build_toml_config

# ── Device mapping ──────────────────────────────────────────────
DEVICE_MAP = {
    "gw15": GowinDevice.GW5AT_15,
    "gw60": GowinDevice.GW5AT_60,
    "gw138": GowinDevice.GW5AST_138,
}

REFCLK_MAP = {
    "q0_refclk0": RefClkSource.Q0_REFCLK0,
    "q0_refclk1": RefClkSource.Q0_REFCLK1,
    "q0_refclk2": RefClkSource.Q0_REFCLK2,
    "q0_refclk3": RefClkSource.Q0_REFCLK3,
    "q1_refclk0": RefClkSource.Q1_REFCLK0,
    "q1_refclk1": RefClkSource.Q1_REFCLK1,
    "q0_refin": RefClkSource.Q0_REFIN,
    "q0_refin0": RefClkSource.Q0_REFIN0,
    "q0_refin1": RefClkSource.Q0_REFIN1,
    "mclk": RefClkSource.MCLK,
}

PLL_MAP = {
    "cpll": PLLSelection.CPLL,
    "cpll0": PLLSelection.CPLL,  # directory alias
    "qpll0": PLLSelection.QPLL0,
    "qpll1": PLLSelection.QPLL1,
}


def parse_toml_simple(path):
    """Simple TOML parser returning section -> key -> value."""
    result = {}
    current_section = "__root__"
    result[current_section] = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("[") and line.endswith("]"):
                current_section = line[1:-1]
                result[current_section] = {}
                continue
            if "=" in line:
                key, _, val = line.partition("=")
                key = key.strip()
                val = val.strip()
                if val == "true":
                    val = True
                elif val == "false":
                    val = False
                elif val.startswith('"') and val.endswith('"'):
                    val = val[1:-1]
                else:
                    try:
                        val = int(val)
                    except ValueError:
                        try:
                            val = float(val)
                        except ValueError:
                            pass
                result[current_section][key] = val
    return result


def discover_configs():
    """Walk /tmp/gwsr and discover all single-lane reference configs."""
    configs = []
    base = "/tmp/gwsr"

    for dev_name, dev_enum in DEVICE_MAP.items():
        dev_dir = os.path.join(base, dev_name)
        if not os.path.isdir(dev_dir):
            continue

        for qi_name in sorted(os.listdir(dev_dir)):
            if qi_name == "mix1":
                continue
            qi_dir = os.path.join(dev_dir, qi_name)
            if not os.path.isdir(qi_dir) or not qi_name.startswith("q"):
                continue
            qi = int(qi_name[1:])

            for li_name in sorted(os.listdir(qi_dir)):
                li_dir = os.path.join(qi_dir, li_name)
                if not os.path.isdir(li_dir) or not li_name.startswith("l"):
                    continue
                li = int(li_name[1:])

                for rname in sorted(os.listdir(li_dir)):
                    r_dir = os.path.join(li_dir, rname)
                    if not os.path.isdir(r_dir):
                        continue
                    rsrc = REFCLK_MAP.get(rname)
                    if rsrc is None:
                        print(f"WARNING: unknown refclk '{rname}' in {r_dir}")
                        continue

                    for pname in sorted(os.listdir(r_dir)):
                        p_dir = os.path.join(r_dir, pname)
                        toml_path = os.path.join(p_dir, "serdes_tmp.toml")
                        if not os.path.isfile(toml_path):
                            continue
                        psel = PLL_MAP.get(pname)
                        if psel is None:
                            print(f"WARNING: unknown PLL '{pname}' in {p_dir}")
                            continue

                        label = f"{dev_name}/{qi_name}/{li_name}/{rname}/{pname}"
                        configs.append((toml_path, dev_enum, qi, li, rsrc, psel, label))

    return configs


def build_config_for(device, quad, lane, ref_src, pll_sel):
    """Build a gowin_serdes config for a single lane."""
    lc = LaneConfig(pll=pll_sel, ref_clk_source=ref_src, ref_clk_freq="125M")
    group = GowinSerDesGroup(quad=quad, first_lane=lane, lane_configs=[lc])
    return build_toml_config(device, [group])


def build_mix1_config():
    """Build the gw138 mix1: Q0.L0+L1, Q1.L1+L3, QPLL0, Q1_REFCLK1."""
    lc = lambda: LaneConfig(
        pll=PLLSelection.QPLL0,
        ref_clk_source=RefClkSource.Q1_REFCLK1,
        ref_clk_freq="125M",
    )
    groups = [
        GowinSerDesGroup(quad=0, first_lane=0, lane_configs=[lc(), lc()]),
        GowinSerDesGroup(quad=1, first_lane=1, lane_configs=[lc()]),
        GowinSerDesGroup(quad=1, first_lane=3, lane_configs=[lc()]),
    ]
    return build_toml_config(GowinDevice.GW5AST_138, groups)


def compare_configs(ref_path, gen_cfg, label):
    """Compare reference and generated configs, return mismatches."""
    ref = parse_toml_simple(ref_path)
    mismatches = []

    # Determine quad count from sections
    ref_sections = set(ref.keys()) - {"__root__", "regulator"}
    gen_sections = set()
    for k in gen_cfg:
        if k not in ("device", "regulator"):
            gen_sections.add(k)

    # Compare quad-level fields
    for qkey in sorted(s for s in ref_sections if s.startswith("q") and "." not in s):
        ref_q = ref.get(qkey, {})
        gen_q = gen_cfg.get(qkey, {})
        all_keys = set(ref_q.keys()) | set(gen_q.keys())
        for field in sorted(all_keys):
            ref_val = ref_q.get(field)
            gen_val = gen_q.get(field)
            if ref_val != gen_val:
                mismatches.append(f"  [{qkey}].{field}: ref={ref_val} gen={gen_val}")

    # Compare lane-level fields
    for lkey in sorted(s for s in ref_sections if "." in s):
        ref_l = ref.get(lkey, {})
        gen_l = gen_cfg.get(lkey, {})
        all_keys = set(ref_l.keys()) | set(gen_l.keys())
        for field in sorted(all_keys):
            ref_val = ref_l.get(field)
            gen_val = gen_l.get(field)
            if ref_val != gen_val:
                mismatches.append(f"  [{lkey}].{field}: ref={ref_val} gen={gen_val}")

    return mismatches


def main():
    total = 0
    passed = 0
    failed = 0
    all_mismatches = {}

    print("=" * 70)
    print("TOML Generation Comparison Test (all devices)")
    print("=" * 70)
    print()

    # Single-lane configs
    configs = discover_configs()
    for toml_path, dev, qi, li, rsrc, psel, label in configs:
        total += 1
        gen_cfg = build_config_for(dev, qi, li, rsrc, psel)
        mismatches = compare_configs(toml_path, gen_cfg, label)
        if mismatches:
            print(f"FAIL: {label}")
            for m in mismatches:
                print(m)
            print()
            failed += 1
            all_mismatches[label] = mismatches
        else:
            print(f"PASS: {label}")
            passed += 1

    # gw138 mix1
    mix1_path = "/tmp/gwsr/gw138/mix1/serdes_tmp.toml"
    if os.path.isfile(mix1_path):
        total += 1
        gen_cfg = build_mix1_config()
        mismatches = compare_configs(mix1_path, gen_cfg, "gw138/mix1")
        if mismatches:
            print(f"FAIL: gw138/mix1")
            for m in mismatches:
                print(m)
            print()
            failed += 1
            all_mismatches["gw138/mix1"] = mismatches
        else:
            print(f"PASS: gw138/mix1")
            passed += 1

    print()
    print("=" * 70)
    print(f"Results: {passed}/{total} passed, {failed} failed")
    print("=" * 70)

    if failed:
        print(f"\nSUMMARY OF FAILURES ({failed}):")
        for label, mm in all_mismatches.items():
            print(f"\n  {label}:")
            for m in mm[:10]:
                print(f"    {m}")
            if len(mm) > 10:
                print(f"    ... ({len(mm) - 10} more)")

    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
