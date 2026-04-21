#!/usr/bin/env python3
"""Generate TOML → CSR for all configs and compare against reference .csr files."""

import sys
import os
import tempfile
import subprocess

sys.path.insert(0, os.path.dirname(__file__))

from gowin_serdes.config import (
    DEVICE_META,
    GowinDevice,
    LaneConfig,
    PLLSelection,
    RefClkSource,
)
from gowin_serdes.group import GowinSerDesGroup
from gowin_serdes.toml_gen import generate_toml

GOWIN_BIN = "/home/key2/Downloads/gowin/IDE/bin/serdes_toml_to_csr.dist"

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
    "cpll0": PLLSelection.CPLL,
    "qpll0": PLLSelection.QPLL0,
    "qpll1": PLLSelection.QPLL1,
}


def discover_configs():
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
                        continue
                    for pname in sorted(os.listdir(r_dir)):
                        p_dir = os.path.join(r_dir, pname)
                        csr_path = os.path.join(p_dir, "serdes.csr")
                        if not os.path.isfile(csr_path):
                            continue
                        psel = PLL_MAP.get(pname)
                        if psel is None:
                            continue
                        label = f"{dev_name}/{qi_name}/{li_name}/{rname}/{pname}"
                        configs.append((csr_path, dev_enum, qi, li, rsrc, psel, label))
    return configs


def get_csr_tool(device):
    suffix = DEVICE_META[device][1]
    tool = os.path.join(GOWIN_BIN, f"serdes_toml_to_csr_{suffix}.bin")
    if os.path.isfile(tool):
        return tool
    return None


def generate_and_convert(device, quad, lane, ref_src, pll_sel):
    lc = LaneConfig(pll=pll_sel, ref_clk_source=ref_src, ref_clk_freq="125M")
    group = GowinSerDesGroup(quad=quad, first_lane=lane, lane_configs=[lc])
    tool = get_csr_tool(device)
    if not tool:
        return None, f"CSR tool not found for {device}"
    with tempfile.TemporaryDirectory() as tmpdir:
        toml_path = os.path.join(tmpdir, "serdes_tmp.toml")
        csr_path = os.path.join(tmpdir, "serdes.csr")
        generate_toml(device, [group], toml_path)
        try:
            subprocess.run(
                [tool, toml_path, "-o", csr_path],
                capture_output=True,
                text=True,
                check=True,
            )
        except subprocess.CalledProcessError as e:
            return None, f"CSR tool failed: {e.stderr[:200]}"
        if os.path.isfile(csr_path):
            with open(csr_path) as f:
                return f.read(), None
        return None, "CSR file not produced"


def gen_mix1():
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
    tool = get_csr_tool(GowinDevice.GW5AST_138)
    if not tool:
        return None, "CSR tool not found"
    with tempfile.TemporaryDirectory() as tmpdir:
        toml_path = os.path.join(tmpdir, "serdes_tmp.toml")
        csr_path = os.path.join(tmpdir, "serdes.csr")
        generate_toml(GowinDevice.GW5AST_138, groups, toml_path)
        try:
            subprocess.run(
                [tool, toml_path, "-o", csr_path],
                capture_output=True,
                text=True,
                check=True,
            )
        except subprocess.CalledProcessError as e:
            return None, f"CSR tool failed: {e.stderr[:200]}"
        if os.path.isfile(csr_path):
            with open(csr_path) as f:
                return f.read(), None
        return None, "CSR file not produced"


def compare_csr(ref_path, gen_content):
    with open(ref_path) as f:
        ref_content = f.read()
    ref_lines = [l.strip() for l in ref_content.strip().splitlines() if l.strip()]
    gen_lines = [l.strip() for l in gen_content.strip().splitlines() if l.strip()]
    mismatches = []
    if len(ref_lines) != len(gen_lines):
        mismatches.append(f"  Line count: ref={len(ref_lines)} gen={len(gen_lines)}")
    for i, (rl, gl) in enumerate(zip(ref_lines, gen_lines)):
        if rl != gl:
            mismatches.append(f"  L{i + 1}: ref={rl}")
            mismatches.append(f"        gen={gl}")
            if len(mismatches) > 20:
                mismatches.append("  ... (truncated)")
                break
    return mismatches


def main():
    total = passed = failed = errors = 0
    print("=" * 70)
    print("CSR Generation Comparison Test (all devices)")
    print("=" * 70)
    print()

    for csr_path, dev, qi, li, rsrc, psel, label in discover_configs():
        total += 1
        gen, err = generate_and_convert(dev, qi, li, rsrc, psel)
        if err:
            print(f"ERROR: {label} - {err}")
            errors += 1
            continue
        mm = compare_csr(csr_path, gen)
        if mm:
            print(f"FAIL: {label}")
            for m in mm:
                print(m)
            print()
            failed += 1
        else:
            print(f"PASS: {label}")
            passed += 1

    # mix1
    mix1_csr = "/tmp/gwsr/gw138/mix1/serdes.csr"
    if os.path.isfile(mix1_csr):
        total += 1
        gen, err = gen_mix1()
        if err:
            print(f"ERROR: gw138/mix1 - {err}")
            errors += 1
        else:
            mm = compare_csr(mix1_csr, gen)
            if mm:
                print(f"FAIL: gw138/mix1")
                for m in mm:
                    print(m)
                failed += 1
            else:
                print(f"PASS: gw138/mix1")
                passed += 1

    print()
    print("=" * 70)
    print(f"Results: {passed}/{total} passed, {failed} failed, {errors} errors")
    print("=" * 70)
    return 0 if (failed == 0 and errors == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
