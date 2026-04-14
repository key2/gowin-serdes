# EIDLE — SerDes TX Electrical Idle Toggle Example

Toggle TX Electrical Idle on Gowin GW5AT-15 SerDes Q0 Lane 0 via UART-to-DRP
bridge, using the `gowin_serdes` pure-Amaranth PHY.

## How `top.py` is wired

### Clock

The entire fabric design runs on the **UPAR life-clock (~62.5 MHz)** from the
GTR12 hard macro. The 125 MHz board oscillator is enabled (`clk_en=1`) as the
SerDes PLL reference clock but is NOT used for fabric logic. This avoids all
clock domain crossing.

### SerDes configuration

```python
LaneConfig(
    operation_mode = OperationMode.TX_RX,
    tx_data_rate   = "5G",
    rx_data_rate   = "5G",
    tx_gear_rate   = GearRate.G1_2,
    rx_gear_rate   = GearRate.G1_2,
    width_mode     = 20,
)
```

This matches the `serdes_tmp.toml` from `/tmp/myusb` (USB3-like parameters).
The `serdes.csr` blob in this directory was generated from that TOML.

### Reset signals

Resets are **deasserted** to match the Gowin USB3 PHY:

```python
lane0.reset.pma_rstn.eq(1)       # PMA out of reset (active-low)
lane0.reset.pcs_rx_rst.eq(0)     # PCS RX out of reset (active-high)
lane0.reset.pcs_tx_rst.eq(0)     # PCS TX out of reset (active-high)
```

PMA must be out of reset for the TX analog driver to power up. With PMA in
reset, CSR reads/writes still work (the UPAR path is independent) but nothing
appears on the TX differential pair.

### TX data path

```python
lane0.tx.data.eq(0)
lane0.tx.fifo_wren.eq(0)
lane0.tx.clk.eq(lane0.tx.pcs_clkout)
```

TX FIFO write-enable is held low and data is zero. The TX output is controlled
exclusively via the EIDLE CSR register through DRP writes. When EIDLE is
deasserted (`0x07`), the PMA driver activates briefly (visible as a spike on
the scope); when asserted (`0x01`), the line returns to common mode.

### Anti-sweep strategy

All lane outputs are consumed by an XOR tree (`io_hash`) to prevent the
synthesizer from optimizing away the SerDes hard macros. PCS clock outputs are
looped back to clock inputs.

### UART protocol

115200 baud on `/dev/ttyUSB3` (pins F14/E14):

| Command | TX bytes | RX bytes |
|---------|----------|----------|
| Ping    | `T`      | `0x55`   |
| Read    | `R` A2 A1 A0 | D3 D2 D1 D0 STATUS |
| Write   | `W` A2 A1 A0 D3 D2 D1 D0 | STATUS |
| Status  | `S`      | io_hash (1 byte) |

STATUS: `0x00` = OK, `0x01` = DRP resp error, `0xFF` = timeout.

## How to run

### Prerequisites

- Gowin IDE toolchain in PATH
- `pyserial` installed (`pip install pyserial`)
- GW5AT DVK board connected (JTAG + UART on `/dev/ttyUSB3`)

### Step 1 — Build and program

```
python top.py program
```

Or build only, then program with openFPGALoader:

```
python top.py
openFPGALoader --cable ft4232 --bitstream build/serdes_drp_uart.fs
```

### Step 2 — Run FSM_INIT

This writes the 11 initialization registers extracted from the Gowin USB3 PHY
(`usb3_0_phy_decrypted.v`, `FSM_INIT` state). Must be run once after each
FPGA programming:

```
python fsm_init.py
```

Expected output — all 11 writes return OK:

```
=== FSM_INIT writes ===
  [ 0] CSR_TX_FFE_0              0x808234 <- 0x0000F000  (OK)
  [ 1] CSR_TX_FFE_1              0x808238 <- 0x00000000  (OK)
  ...
  [10] CSR_WRITE_CDR_CFG_5       0x800261 <- 0x00004F00  (OK)

CSR_READ_RXDET  0x808B34 = 0x02000002  (OK)
```

### Step 3 — Toggle EIDLE

Toggle TX Electrical Idle on and off. Each transition is visible as a spike
on the TX differential pair:

```
python eidle_toggle.py --loop 5 --delay 1
```

Other modes:

```
python eidle_toggle.py --on            # assert EIDLE (TX quiet)
python eidle_toggle.py --off           # deassert EIDLE (TX active)
python eidle_toggle.py --loop 100      # 100 toggles, 0.1s apart
python eidle_toggle.py --pulse         # single LFPS Rx.Detect pulse
python eidle_toggle.py --pulse --loop 1000   # 1000 Rx.Detect pulses
```

### EIDLE CSR values (Q0 Lane 0)

| Register | Address | Value | Effect |
|----------|---------|-------|--------|
| `CSR_WRITE_EIDLE` | `0x8003A4` | `0x00000001` | TX electrical idle ON (common mode) |
| `CSR_WRITE_EIDLE` | `0x8003A4` | `0x00000007` | TX electrical idle OFF (driver active) |
| `CSR_WRITE_PLUSE` | `0x80033F` | `0x03000000` | Assert Rx.Detect pulse |
| `CSR_WRITE_PLUSE` | `0x80033F` | `0x00000000` | Deassert Rx.Detect pulse |

## Files

| File | Purpose |
|------|---------|
| `top.py` | Amaranth design: SerDes + UART-to-DRP bridge |
| `fsm_init.py` | Replay the USB3 PHY FSM_INIT register writes |
| `eidle_toggle.py` | Toggle TX Electrical Idle / send Rx.Detect pulses |
| `gw5at_dvk.py` | GW5AT DVK platform definition |
| `uart.py` | Async serial RX/TX components |
| `serdes.csr` | SerDes configuration blob (5G, width=20, gear=1:2) |
