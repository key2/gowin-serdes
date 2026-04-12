# Gowin SerDes -- Pure Amaranth Reimplementation

## Goal

Replace Gowin's encrypted, IDE-generated SerDes IP with a **pure-Amaranth**
Python library that:

- Needs **no generated Verilog** from the Gowin IDE
- Instantiates the same hard macros (`GTR12_QUADB`, `GTR12_QUADA`,
  `GTR12_UPARA`, `GTR12_UPAR`) with the correct parameters
- Exposes clean, typed Python interfaces using `amaranth.lib.wiring`
- Is configured entirely through Python dataclasses (`LaneConfig`)
- Generates the TOML configuration and `.csr` register blob directly from
  the Python object graph -- no external scripts required

---

## What Gowin Generates vs. What We Replace

| Gowin artefact | Lines | What it actually is | Our replacement |
|---|---|---|---|
| `customized_phy_wrapper.v` | 1608 | `ifdef` mux selecting ports per encoding mode | Python conditionals in `GowinSerDesLane` |
| `customized_phy.v` (encrypted) | 2044 | More `ifdef` + pure `assign` wiring | Same -- zero logic, all wiring |
| `upar_arbiter.v` (encrypted) | 371 | Round-robin FSM arbiter | `GowinUPARArbiter` Elaboratable |
| `upar_arbiter_wrap.v` | 250 | Flat-bus to array unpacking | Absorbed into arbiter (Python loops) |
| `serdes.v` (top-level) | 272-1409 | Instantiate macros + glue | `GowinSerDes` top-level Component |
| `define.vh` + `parameter.vh` | per-variant | Compile-time config | `LaneConfig` dataclass |
| `serdes_toml_gen.py` (standalone) | 705 | CLI TOML generator | `GowinSerDes.generate_toml()` / `generate_csr()` |
| TOML + `.csr` binary | N/A | Register blob for hard macro | Generated from `GowinSerDes` via `toml_gen.py` module |

**Total Verilog replaced: ~5000+ lines -> ~1500 lines of Python.**

---

## Hardware Reality

There is **one** hard macro per quad on silicon. Each `GTR12_QUADB/QUADA`
exposes 4 lanes of raw signals (TX data, RX data, clocks, FIFOs, status,
resets). The hard macro has a fixed port list regardless of how many lanes
are used. A single `GTR12_UPARA/UPAR` primitive provides register access to
the entire SerDes subsystem.

The user's cores (USB3, XAUI, raw link, etc.) each need some subset of lane
signals. The UPAR arbiter multiplexes register access from multiple
requestors onto the single UPAR bus.

---

## Architecture: Bottom-Up, Group-Centric

The user builds **groups** (each containing 1-4 lanes with their config),
then passes them to `GowinSerDes` which owns the hard macros, generates
the arbiter, and wires everything together.

### Module Hierarchy

```
GowinSerDes(wiring.Component)              <-- user instantiates this
|
+-- groups: [GowinSerDesGroup, ...]        <-- user creates these FIRST
|   |
|   +-- lanes: [GowinSerDesLane, ...]      <-- 1 to 4 lanes per group
|       +-- config: LaneConfig             <-- per-lane settings
|       +-- tx: LaneTXSignature            <-- user connects TX here
|       +-- rx: LaneRXSignature            <-- user connects RX here
|       +-- status: LaneStatusSignature    <-- ready, CDR lock, etc.
|       +-- reset: LaneResetSignature      <-- PMA/PCS resets
|
+-- GowinSerDesQuadInstance(internal)      <-- 1 or 2, created by GowinSerDes
|   +-- Instance("GTR12_QUADB/QUADA")      <-- hard macro
|
+-- GowinUPARInstance(internal)            <-- 1 per device
|   +-- Instance("GTR12_UPARA/UPAR")       <-- UPAR register access
|
+-- GowinUPARArbiter(internal)             <-- 1 per device
|   +-- DRP_NUM client ports (4 or 8)      <-- fixed by device
|   +-- active slots = one per group       <-- slot = quad*4 + first_lane
|   +-- unused slots: no request = no grant
|   +-- 1 x UPAR master port              <-- to GTR12_UPARA
|
+-- wiring logic (internal)
    +-- each group's lanes <-> quad primitive
    +-- unused lane slots: tie to GND
    +-- group DRP <-> arbiter slot
    +-- arbiter UPAR <-> GTR12_UPARA
```

### System Diagram

```
                     USER CODE
                  +------+--------+
                  | USB3 | Raw    |
                  | PHY  | Link   |
                  +--+---+---+----+
                     |       |
               +-----+       +--------+
               |                      |
     GowinSerDesGroup          GowinSerDesGroup
     (q0, first_lane=0,       (q1, first_lane=0,
      lane_configs=[cfg])      lane_configs=[cfg])
      --> 1 lane interface     --> 1 lane interface
               |                      |
               +-------+    +---------+
                       |    |
                 GowinSerDes(device, groups=[...])
                       |
             +---------+---------+
             |         |         |
        GTR12_QUADA  GTR12    GowinUPAR
        (per quad)   _UPARA   Arbiter
             |         |      (DRP_NUM=8,
        4 lanes      UPAR     2 active
        per quad     reg      slots)
             |       bus
        unused lanes
        tied to GND
```

---

## File Layout

```
gowin_serdes/
+-- ARCHITECTURE.md              <-- this file
+-- gowin_serdes/
|   +-- __init__.py              <-- public exports
|   +-- config.py                <-- LaneConfig, GowinDevice, enums, DEVICE_META
|   +-- signature.py             <-- DRP, UPAR, TX, RX, Status, Reset signatures
|   +-- lane.py                  <-- GowinSerDesLane(Component) -- per-lane wiring
|   +-- group.py                 <-- GowinSerDesGroup(Component) -- 1-4 lanes + bonding
|   +-- upar_arbiter.py          <-- GowinUPARArbiter(Elaboratable) -- round-robin FSM
|   +-- quad.py                  <-- GowinSerDesQuadInstance, GowinUPARInstance builders
|   +-- primitives.py            <-- GTR12 port name tables per device family
|   +-- serdes.py                <-- GowinSerDes(Component) -- top-level assembly
|   +-- toml_gen.py              <-- TOML/CSR generation from the live object graph
+-- tests/
|   +-- test_arbiter.py          <-- arbiter simulation (POR, write, read, round-robin)
|   +-- test_lane_group.py       <-- lane + group wiring verification
|   +-- test_serdes.py           <-- integration: elaborate + port checking
+-- example/
|   +-- gw5at-15/
|       +-- eidle/               <-- DRP bridge over UART (working example)
|       +-- csr_rw/              <-- CSR read/write example
+-- gw/                          <-- reference: decrypted Gowin IP sources
+-- gen/                         <-- reference: Gowin IDE-generated variants
```

---

## Module Details

### `config.py` -- Configuration Model

All compile-time decisions are driven by Python dataclasses and enums.

**`GowinDevice`** -- Target device:

| Enum | Device | Quads | Primitive | DRP_NUM |
|------|--------|-------|-----------|---------|
| `GW5AT_15` | GW5AT-15 | 1 | GTR12_QUADB | 4 |
| `GW5AT_60` | GW5AT-60 | 1 | GTR12_QUADA | 4 |
| `GW5AT_138` | GW5AT-138 | 2 | GTR12_QUADA | 8 |

**`DEVICE_META`** -- Per-device metadata for TOML/CSR generation:

```python
DEVICE_META = {
    GowinDevice.GW5AT_15:  ("GW5AT-15",  "15k",  1, False),
    GowinDevice.GW5AT_60:  ("GW5AT-60",  "60k",  1, True),
    GowinDevice.GW5AT_138: ("GW5AT-138", "138k", 2, False),
}
# (toml_device_name, csr_binary_suffix, num_quads, has_extra_pads)
```

**`LaneConfig`** -- Per-lane configuration dataclass:

```python
@dataclass
class LaneConfig:
    operation_mode: OperationMode = OperationMode.TX_RX
    tx_data_rate:   str           = "1.25G"
    rx_data_rate:   str           = "1.25G"
    tx_gear_rate:   GearRate      = GearRate.G1_1
    rx_gear_rate:   GearRate      = GearRate.G1_1
    pll:            PLLSelection  = PLLSelection.CPLL
    ref_clk_source: RefClkSource  = RefClkSource.Q0_REFCLK0
    ref_clk_freq:   str           = "125M"
    width_mode:     int           = 10
    tx_encoding:    EncodingMode  = EncodingMode.OFF
    rx_encoding:    EncodingMode  = EncodingMode.OFF
    word_align:     bool          = False
    ctc_enable:     bool          = False
    # ... plus low-rate TX, 64B66B/67B, alignment pattern fields
```

Key properties: `has_tx`, `has_rx`, `has_8b10b`, `has_64b66b`, `has_64b67b`,
`tx_data_width` (80), `rx_data_width` (88).

### `signature.py` -- Wiring Signatures

All interfaces use `amaranth.lib.wiring.Signature` with typed `In`/`Out` members.

| Signature | Purpose | Key members |
|-----------|---------|-------------|
| `DRPSignature` | Per-client DRP bus | `addr[24]`, `wren`, `wrdata[32]`, `strb[8]`, `rden`, `ready`, `rdvld`, `rddata[32]` |
| `UPARSignature` | Master UPAR bus | Same as DRP + `rst`, `bus_width` |
| `LaneTXSignature` | Per-lane TX | `data[80]`, `clk`, `pcs_clkout`, FIFO signals; optional `txc/txd` for 64B66B |
| `LaneRXSignature` | Per-lane RX | `data[88]`, `clk`, `pcs_clkout`, `valid`, FIFO signals; optional `rxc/rxd` |
| `LaneStatusSignature` | Per-lane status | `ready`, `signal_detect`, `rx_cdr_lock`, `pll_lock`, `refclk`; conditional 8B10B/64B fields |
| `LaneResetSignature` | Per-lane resets | `pma_rstn` (active-low), `pcs_rx_rst` (active-high), `pcs_tx_rst` (active-high) |

### `lane.py` -- GowinSerDesLane

A `wiring.Component` exposing user-facing ports and holding "quad-side"
internal signals. Contains **zero logic** -- only combinatorial `assign`
wiring that replaces Gowin's 2000+ lines of `ifdef`-guarded Verilog.

Port selection is driven by `LaneConfig`:
- TX/RX ports conditional on `has_tx` / `has_rx`
- 64B66B/67B extra ports conditional on encoding mode
- Status ports conditional on encoding mode (8B10B adds `k_lock`, `word_align_link`)
- CTC clock input conditional on `ctc_enable`

Key wiring in `elaborate()`:
- **TX**: General mode passes `data[80]` directly to QUAD; 64B66B packs
  `txd[64]+txc[8]+tx_ctrl[3]`; 64B67B packs `tx_data[64]+tx_header[2]+tx_ctrl[3]`
- **RX**: General mode passes `data[88]` from QUAD; 64B66B extracts `rxd[64]+rxc[8]`
- **Status**: `ready=STAT_O[12]`, `signal_detect=ASTAT_O[5]`, `rx_cdr_lock=PMA_RX_LOCK_O`,
  `pll_lock=CMU_OK_O`, `refclk=CMU_CK_REF_O`
- **Reset**: `pma_rstn -> RSTN_I`, `pcs_rx_rst -> PCS_RX_RST`, `pcs_tx_rst -> PCS_TX_RST`

### `group.py` -- GowinSerDesGroup

A `wiring.Component` wrapping 1-4 lanes that share a single DRP arbiter slot.
Maps to exactly one Gowin `Customized_PHY_Top` instance.

```python
GowinSerDesGroup(
    quad=0,             # quad index (0 or 1)
    first_lane=0,       # first lane in quad (0-3)
    lane_configs=[...], # 1-4 LaneConfig objects
    chbond_master=0,    # bonding master index (None = no bonding)
)
```

- `arbiter_slot = quad * 4 + first_lane`
- `drp_name = f"drp_q{quad}_ln{first_lane}"`
- Channel bonding: if `chbond_master is not None`, `cb_start` is propagated
  to all lanes; otherwise each lane's `CHBOND_START` is tied to GND.

### `upar_arbiter.py` -- GowinUPARArbiter

A plain `Elaboratable` (not a Component) implementing a masked round-robin
arbiter between `DRP_NUM` clients and one UPAR bus. Uses raw `Signal` ports
to avoid direction conflicts across Component boundaries.

**Algorithm**: masked round-robin with prefix-OR scan.

**FSM** (4 states):
```
IDLE --(any req)--> JUDG_ADDR --(3 cycles)--> UPAR_EN --(ack)--> WAIT --> IDLE
```

- `IDLE`: Sample requests, run arbiter, latch winner
- `JUDG_ADDR`: 3-cycle pipeline to latch address/data from winner
- `UPAR_EN`: Assert write or read on UPAR bus. Wait for ready/rdvld.
  Timeout after 2^20 cycles prevents permanent lockup.
- `WAIT`: 1 cycle, route response back to winning DRP client

Constants: `upar_rst = 0`, `upar_bus_width = 0` (always 32-bit).
Clock distribution: `drp_clk[j] = upar_clk` for all clients.
Power-on reset: 16-cycle counter before arbitration begins.

### `quad.py` -- Hard Macro Instance Builders

**`GowinSerDesQuadInstance`** builds one `Instance("GTR12_QUADB")` or
`Instance("GTR12_QUADA")` with programmatically-generated port mappings.

- Active lanes get their signals wired from the lane's `_quad_*` signals
- Unused lanes are tied to GND (inputs) or dummy signals (outputs)
- Quad-level connections: `POR_N`, `CMU0/CMU1_RESETN`, `CK_AHB`, `LIFE_CLK`
- All 4 CPLL resets tied to `por_n` (even unused lanes -- prevents PLL tree lockup)
- GW5AT-138 gets `p_POSITION = "Q0"/"Q1"` defparam

**`GowinUPARInstance`** builds one `Instance("GTR12_UPARA")` or
`Instance("GTR12_UPAR")` (singleton per device).

- UPAR bus signals wired from the arbiter
- `CSR_MODE = 0b10100` (5-bit constant)
- AHB clock/reset outputs captured for internal use

### `primitives.py` -- Port Name Tables

Complete port tables extracted from Gowin-generated `_tmp.v` files.
Organized as `(name_template, direction, width)` tuples with `{ln}` placeholders
for per-lane expansion.

| Table | Device | Ports |
|-------|--------|-------|
| `QUAD_LANE_PORTS_COMMON` | All | TX/RX data, status, clocks, FIFOs, resets, CPLL |
| `QUAD_LANE_PORTS_QUADB_ONLY` | GW5AT-15 | 64B66B signals, TX disparity |
| `QUAD_COMMON_PORTS` | All | Refclk, CMU/PLL, misc control |
| `QUAD_PORTS_QUADB_ONLY` | GW5AT-15 | MIPI, extra refclk detect |
| `QUAD_PORTS_QUADA_60_ONLY` | GW5AT-60 | Extra refclk pads |
| `QUAD_PORTS_QUAD_ONLY` | GW5AT-138 | INET buses, refclk OE |
| `UPAR_COMMON_PORTS` | All | UPAR bus, JTAG/CSR, SPI, DFT |
| `UPAR_PORTS_138_ONLY` | GW5AT-138 | INET UPAR buses |

### `serdes.py` -- GowinSerDes (Top-Level)

The top-level `wiring.Component` that assembles everything.

**Signature** (dynamically built):
- `por_n : In(1)` -- device-level power-on reset (active-low)
- `dbg_arb_state : Out(2)` -- arbiter FSM state for debug
- `drp_q{Q}_ln{L} : In(DRPSignature())` -- one per group

**`__init__`**:
1. Validates: no overlapping (quad, lane) assignments, quad in device range
2. Groups indexed `_groups_by_quad` for quad instantiation
3. Computes `drp_num` and `num_quads` from device

**`elaborate()`**:
1. Creates `"upar"` clock domain from `FABRIC_CM_LIFE_CLK_O`
2. Instantiates `GowinUPARArbiter(drp_num, domain="upar")`
3. Per-quad: `GowinSerDesQuadInstance` with lane signal maps
4. Singleton: `GowinUPARInstance` with arbiter UPAR signals
5. Wires each group's Component DRP port to its arbiter slot
6. GW5AT-138: creates INET signal buses between quads and UPAR

**TOML/CSR generation methods**:
- `serdes.toml_config()` -- returns the raw TOML config dict
- `serdes.generate_toml(path)` -- writes a Gowin-compatible `.toml` file
- `serdes.generate_csr(path)` -- writes `.toml` + invokes `serdes_toml_to_csr_*k.bin`

### `toml_gen.py` -- TOML / CSR Generation

Generates TOML configuration files compatible with Gowin's
`serdes_toml_to_csr_*k.bin` tools, directly from the live `GowinSerDes` /
`GowinSerDesGroup` / `LaneConfig` object graph.

The generated TOML is structurally identical to what the Gowin IDE produces.

**Key functions**:

| Function | Purpose |
|----------|---------|
| `build_toml_config(device, groups)` | Walk object graph, produce complete TOML dict |
| `generate_toml(device, groups, path)` | Write TOML file to disk |
| `generate_csr(device, groups, path)` | Write TOML, invoke Gowin tool, produce `.csr` |

**LaneConfig to TOML mapping**:
- `EncodingMode` enums map to TOML strings (`B8B10B -> "8B10B"`)
- `GearRate` enums map to TOML strings (`G1_2 -> "1:2"`)
- `pcs_tx_clk_src` derived from TX data rate (0 if <=1.5G, 1 otherwise)
- Channel bonding settings derived from group's `chbond_master` / `num_lanes`
- Multi-quad fields (`ref_prop_dir`, `refomux0_sel`) only emitted for 138K

**Tool discovery** (`_find_tool`):
1. Explicit `gowin_bin_dir` parameter
2. `$PATH`
3. `$GOWIN_IDE/bin/` and `$GOWIN_IDE/bin/serdes_toml_to_csr.dist/`
4. Common locations (`~/Downloads/gowin/IDE/bin/`, `/opt/gowin/IDE/bin/`)

---

## TOML/CSR Pipeline

```
LaneConfig (Python dataclass)
    |
    v
GowinSerDes.generate_toml()        --> serdes.toml
    |
    v
serdes_toml_to_csr_*k.bin          --> serdes.csr
    |
    v
platform.add_file("serdes.csr")    --> set_csr in TCL
    |
    v
gw_sh (synthesis + PnR)            --> .fs bitstream
```

The entire pipeline is driven from `GowinSerDes.generate_csr()`:

```python
serdes = GowinSerDes(device=GowinDevice.GW5AT_15, groups=[group])
serdes.generate_csr("serdes.csr", toml_path="serdes.toml")
```

Or just the TOML for inspection:

```python
serdes.generate_toml("serdes.toml")
```

---

## Usage

### Creating a SerDes instance

```python
from gowin_serdes import *

# 1. Define lane configuration
lane_cfg = LaneConfig(
    operation_mode=OperationMode.TX_RX,
    tx_data_rate="5G",
    rx_data_rate="5G",
    tx_gear_rate=GearRate.G1_2,
    rx_gear_rate=GearRate.G1_2,
    width_mode=20,
)

# 2. Create a group (1-4 lanes sharing one DRP slot)
group = GowinSerDesGroup(
    quad=0,
    first_lane=0,
    lane_configs=[lane_cfg],
)

# 3. Assemble the top-level
serdes = GowinSerDes(
    device=GowinDevice.GW5AT_15,
    groups=[group],
)
```

### Wiring in a design

```python
class MyTop(Elaboratable):
    def elaborate(self, platform):
        m = Module()
        m.submodules.serdes = serdes

        # POR_N: assert high so the QUAD / PLL can start
        m.d.comb += serdes.por_n.eq(1)

        # Access lane and DRP
        lane0 = group.lanes[0]
        drp = serdes.drp_q0_ln0

        # Drive resets
        m.d.comb += [
            lane0.reset.pma_rstn.eq(1),
            lane0.reset.pcs_rx_rst.eq(0),
            lane0.reset.pcs_tx_rst.eq(0),
        ]

        # TX: loop PCS clock back, drive data
        m.d.comb += [
            lane0.tx.clk.eq(lane0.tx.pcs_clkout),
            lane0.tx.data.eq(my_tx_data),
        ]

        # RX: loop PCS clock back, read data
        m.d.comb += [
            lane0.rx.clk.eq(lane0.rx.pcs_clkout),
            lane0.rx.fifo_rden.eq(1),
        ]

        return m
```

### Generating CSR at build time

```python
def build():
    platform = MyPlatform()

    serdes.generate_csr("serdes.csr", toml_path="serdes.toml")
    platform.add_file("serdes.csr", open("serdes.csr", "rb").read())

    platform.build(MyTop(), name="design")
```

### Multi-quad bonded example (GW5AT-138)

```python
# 4-lane bonded XAUI on Q0
xaui_cfg = LaneConfig(
    tx_data_rate="3.125G", width_mode=10,
    tx_encoding=EncodingMode.B8B10B, rx_encoding=EncodingMode.B8B10B,
    word_align=True,
)
group_xaui = GowinSerDesGroup(
    quad=0, first_lane=0,
    lane_configs=[xaui_cfg] * 4,
    chbond_master=0,
)

# 1 independent raw lane on Q1
group_raw = GowinSerDesGroup(
    quad=1, first_lane=0,
    lane_configs=[LaneConfig(tx_data_rate="1.25G", width_mode=10)],
)

serdes = GowinSerDes(
    device=GowinDevice.GW5AT_138,
    groups=[group_xaui, group_raw],
)

# Result:
#   2x GTR12_QUADA (one per quad)
#   1x GTR12_UPAR
#   1x GowinUPARArbiter(DRP_NUM=8) -- slots 0 and 4 active
#   Q0: 4 bonded lanes (CHBOND_START connected)
#   Q1: lane 0 wired, lanes 1-3 GND
```

---

## Tests

All tests live in `tests/` and run with `pytest`:

```
pytest tests/ -v
```

| Test file | Tests | What it covers |
|-----------|-------|----------------|
| `test_arbiter.py` | 5 | POR reset, single write, single read, round-robin fairness, elaboration |
| `test_lane_group.py` | 8 | Lane elaboration, TX/RX passthrough, status extraction, reset wiring, group bonding, arbiter slot calculation |
| `test_serdes.py` | 8 | GW5AT-15 single lane, GW5AT-138 variants (2 independent, 4-lane bonded, mixed, all 8 lanes), overlap/quad-range rejection, DRP write-through, port names |

---

## Reference Material

### `gw/` -- Decrypted Gowin IP

- `CUSTOMIZED/customized_phy_decrypted.v` -- 2044 lines of `ifdef`-guarded wiring
- `Upar_Arbiter/upar_arbiter_decrypted.v` -- 371-line round-robin FSM
- `p1735_decryptor-master/` -- IEEE P1735 decryptor used to decrypt the IP

### `gen/` -- Gowin IDE-Generated Variants

- `gw138/` -- GW5AT-138 with 8 independent single-lane PHY instances
- `gw138-2/` -- GW5AT-138 with 4-lane bonded + 1 independent
  - Contains the reference `serdes_tmp.toml` (713 lines)
  - `gw_trace.log` shows exact Gowin tool invocations

### Gowin Tool Invocation (from `gw_trace.log`)

```
serdes_toml_to_csr_138k.bin serdes_tmp.toml -o serdes.csr
GowinSynthesis -prj CUSTOMIZED.prj
GowinModGen -do serdes.mod
```
