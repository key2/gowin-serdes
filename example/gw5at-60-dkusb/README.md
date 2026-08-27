# DK_USB_GW5AT-LV60UG225_V1.0 — Amaranth platform

Platform file for the **Gowin DK_USB development board** (GW5AT-LV60UG225,
C2/I1 speed grade) — the board the Gowin USB3.1 UVC reference design ships
for, with a properly wired USB Type-C connector.

`dk_usb_gw5at60.py` provides `DKUSBGW5AT60Platform` plus:

* `SERDES_PINS` — documentation of the SerDes hard-macro pads (USB3
  SuperSpeed pairs, SerDes refclk pads, SDI pairs).  These are bonded to
  the GTR12 quad and must never be IO_LOC'd.
* `add_serdes_refclk_forward(m, platform)` — reproduces the reference
  design's clocking: 200 MHz LVDS oscillator → `TLVDS_IBUF` →
  `ELVDS_OBUF` → L9/K8 → PCB trace → **Q0 REFPAD1** (D10/C10).  Returns
  the 200 MHz fabric clock.  The SerDes TOML then selects `Q0_REFCLK1`
  at `200M` — the exact configuration whose CSR boot blob the test suite
  reproduces **byte-identically** from the vendor's shipped `serdes.csr`.

## Pin provenance

Every pin comes from two cross-checked sources, enforced by
`tests/test_dkusb_platform.py`:

1. the board user guide (`doc/DBUG1280-1.0.3E_...pdf`, tables 3-1…3-11);
2. the vendor USB3.1 reference design's working `top.cst` (authoritative
   for IO standards — e.g. `clk_24m` is LVCMOS33/PULL_UP there although
   the guide lists the bank as 1.8 V).

## USB3 on this board

| Item | Value |
|---|---|
| SuperSpeed pairs | TX1/RX1 = D6/C6, B7/A7 (straight plug orientation, vendor **Q0 lane 1**); TX2/RX2 = D4/C4, B3/A3 (flipped, unsupported by the vendor IP) |
| SerDes refclk | 200 MHz on Q0 REFPAD1 via the fabric forward (see above); Q0 REFPAD0 carries 148.5 MHz (SDI) |
| PHY CSR config | `UparCsrConfig(pll=CPLL, quad=0, lane=1, refclk=RefClk.F200M)` — the *shipped default* of the ported PHY; all addresses and rate-change values are pinned against the vendor source |
| CC controller | FUSB302B on the shared I2C bus (`i2c` 0, G11/F11), interrupt `fusb_int` (G15); the guide warns the CC circuit is **not validated** — tie `phy_pwrpresent = 1` for bring-up like the reference design effectively does |
| VBUS | `usb_pwr_en` (J11) |

## Debug UART options

The board has no dedicated UART header.  The bench setup hand-wires an
**FT4232H** (enumerates as "FTDI Quad RS232-HS") to the 80-pin J18 MIPI
connector GPIOs — two channels, both **verified on hardware** (115200 8N1,
banner + byte-exact echo, `uart-hello/` example):

| Resource | FTDI channel | Host device | FPGA RX | FPGA TX |
|---|---|---|---|---|
| `uart` 0 | C | `/dev/ttyUSB4` | G5 (MIPI_GPIO1, J18.52) | H12 (MIPI_GPIO2, J18.54) |
| `uart` 1 | D | `/dev/ttyUSB5` | H15 (MIPI_GPIO3, J18.56) | J15 (MIPI_GPIO4, J18.58) |

(Directions are FPGA-side; polarity was determined with the auto-detect
probe `TangMegaPro/luna_softphy_example/uart_probe_dk60.py`.)

* `uart_j21` 0 — TX-only on the 2.54 mm J21 header **SCL pin (G11)**.
  J21 exposes the FUSB302B/INA3221 I2C bus; transmitting on SCL while
  never driving SDA cannot form an I2C START condition, so the on-board
  slaves stay idle.  Mutually exclusive with the `i2c` 0 resource (same
  physical pins — the platform enforces this at `request()` time).

## uart-hello smoke test (verified working)

`uart-hello/top.py` — minimal end-to-end check of the debug setup: LED
heartbeat + both UARTs sending a `DK60 uartN #xxxx` banner every ~1.4 s and
echoing every received byte.  Build ~1 min, timing trivially met
(233 MHz fmax vs 24 MHz required):

```console
cd uart-hello
python top.py                 # → build/uart_hello.fs
python top.py flash           # openFPGALoader -c ft232 (FT232H download port)
stty -F /dev/ttyUSB4 115200 raw -echo && cat /dev/ttyUSB4   # banners + echo
```

This was the first design to go through full synthesis/PnR/bitstream and
onto the actual board with this platform file.

## Reusing the usb31-enum example

The enumeration example (`../gw5ast-138/usb31-enum/`) ports to this board
with: `QUAD = 0`, `LANE = 1`, `RefClk.F200M` / `ref_clk_freq="200M"` /
`RefClkSource.Q0_REFCLK1`, `GowinDevice.GW5AT_60`, this platform class,
`add_serdes_refclk_forward()` for the refclk, the POR counters on
`clk_24m`, and the boot-CSR extra writes at the Q0_LN1 addresses
(`0x8083F8 = 0xA02`, `eidle = 1`) — which are then *exactly* the two writes
the vendor ships.  On this C2/I1 grade the vendor closes the same netlists
at 160 MHz, so the Gen2-boot timing shortfall seen on the 138K board should
shrink or vanish; the `GEN2_DATAPATH` switch can stay off for first
bring-up regardless (Gen1/5 Gbit target, per-spec fallback from Gen2).

Programming: the board's Mini USB-B download port is an **FT232H**
("FTDI Single RS232-HS", `/dev/ttyUSB1` on the bench) —
`openFPGALoader -c ft232 <bitstream.fs>`, verified working
(`programmer_cable = "ft232"`).
