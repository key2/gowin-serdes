# USB 3.1 SuperSpeed enumeration — Tang Mega 138K Pro

Basic bring-up design whose only job is to **enumerate as a SuperSpeed
device**, validating the Amaranth USB3.1 PHY (`gw_usb3`) together with the
Python-generated SerDes (`gowin_serdes`) on real hardware.

```
USB-C ── GTR12 Quad 1 Lane 0 ── gowin-serdes ── gw_usb3.Usb31Phy ── vendor
         (10G boot trim,         (Amaranth,      (Amaranth port,     pipe/ltssm/link
          CSR from Python)        arbiter＋CSR    equivalence-        netlists + EP0
                                  generated)      proven)             + UVC descriptors
```

## What is Python-generated vs vendor

| Piece | Source | Proof |
|---|---|---|
| GTR12 quad + UPAR + arbiter | `gowin_serdes` (Amaranth) | arbiter cycle-exact vs `Upar_Arbiter/upar_arbiter.v` (cocotb); lane↔quad wiring cross-checked against the reference `serdes.v` (`tests/test_serdes_wiring.py`) |
| SerDes boot CSR | `gowin_serdes` TOML → Gowin tool | reproduces the reference design's `serdes.csr` **byte-exact** for its configuration (`tests/test_gowin_serdes.py`) |
| USB3.1 PHY | `gw_usb3` (Amaranth) | bit/cycle-exact vs decrypted vendor RTL, 20 cocotb equivalence suites |
| PIPE/LTSSM/link | vendor `.vg` netlists (frozen) | as shipped in the reference design |
| Protocol/EP0/descriptors | vendor RTL (plaintext) | as shipped (UVC camera descriptor set, VID 0x030A) |

The PHY is the **only** UPAR/CSR bus master at runtime; the boot blob
additionally carries the two hand-appended writes the reference flow adds
(TX AFE tuning + *TX electrical idle asserted*, which the PHY's CSR
sequencer assumes as reset state — discovered by byte-diffing the golden
CSR).

## Board configuration

* **SerDes**: Quad 1, Lane 0 — USB-C SuperSpeed pairs.
* **Reference clock**: 125 MHz on Q1 REFPAD0 (REFPAD1 = 100 MHz would also
  work: the vendor CSR tables include both; change `RefClk`/`ref_clk_freq`).
* **Boot trim**: 10G / 16-bit / 1:4 — exactly like the reference design.
  The PHY reconfigures the SerDes 10G↔5G over UPAR on LTSSM rate changes,
  so the 10G→5G sequence runs on **every Gen1 link-up**.
* **UART debug**: J3 FT4232 header, A19 (TX) / A18 (RX), 115200 8N1.
* **VBUS**: assumed present (`phy_pwrpresent = 1`); no Type-C controller.

## Rate strategy — do not expect 10 Gbit

`GEN2_DATAPATH = False` (top.py): the PHY is built **Gen1-only**.  The
LTSSM still advertises Gen2 via SCD during Polling.LFPS (that logic is in
the frozen vendor netlists); on a Gen2 host the Gen2 training then fails
and the LTSSM falls back to **Gen1 (5 Gbit)** per USB 3.1 §7.5.4 — which is
the point of this example.  Start bring-up on a USB 3.0 (5 Gbit) port or
behind a 5 Gbit hub to skip that detour entirely.

Set `GEN2_DATAPATH = True` to include the full 128b/132b datapath
(equivalence-proven too, +3.2k LUTs) once Gen1 is solid.

## Build, program, observe

```console
$ python top.py            # TOML + CSR generation, synthesis, PnR, .fs
$ python top.py program    # + openFPGALoader
$ python ../../../../gowin_timing_report.py build   # post-route timing
```

LEDs: 0 = CPLL lock, 1 = CDR lock, 2 = LTSSM training, 3 = **attached
(U0 reached)**, 4 = Gen2 rate active, 5 = heartbeat.

UART lines (115200):

```
U 0000f0b3          status word, on change + ~0.5 s heartbeat
R 80 06 0100 0000 0012   every SETUP: bmRequestType bRequest wValue wIndex wLength
```

Status word bits: [0] cpll_ok, [1] lane ready, [2] cdr lock, [3] signal
detect, [4] RxElecIdle, [5] rate (1 = Gen2), [6] TxElecIdle, [7] RxTerm,
[9:8] PowerDown, [10] rx-detect/loopback, [11] LTSSM training,
[12] attached, [13] ITP received, [14] warm/hot reset, [15] PhyStatus,
[23:16] SETUP count, [31:24] line counter.

A healthy enumeration shows: cpll_ok → rx-detect pulses → training → rate
drops to 0 (10G→5G CSR sequence done) → attached=1 → a burst of `R` lines
(`GET_DESCRIPTOR` 80 06, `SET_ADDRESS` 00 05, `SET_CONFIGURATION` 00 09 …)
and the host sees a UVC camera (VID 0x030A, no video data — EP2 is tied
off).

## Timing status (C1/I0, slow corner)

| Clock | Required | Achieved |
|---|---|---|
| pclk (Gen1 operation) | 125 MHz | **125.7 MHz** ✔ |
| rxclk (Gen1 operation) | 125 MHz | **131.2 MHz** ✔ |
| upar life clock | 62.5 MHz | 119 MHz ✔ |
| pclk/rxclk during the 10G **boot window** | 156.25 MHz | ~126/131 MHz ✘ |

The 156.25 MHz shortfall applies only while the link is still in the Gen2
boot trim (Polling/LFPS + the rate-change window).  The worst offenders are
vendor link-layer reset-release fan-outs and header-latch enables that are
architecturally idle in that window, plus the (then statically-held) Gen1
encoder path; the vendor's own project constrains the same netlists at
160 MHz on a faster GW5AT-60 grade.  Treat Gen2-boot behaviour as
best-effort: if Polling misbehaves, first try a 5 Gbit host port.

## Known limitations

* GW5AST-138 die has no SSRAM: the vendor link netlist's `RAM16SDP*`
  distributed-RAM cells are transparently renamed to register-based
  equivalents (`ram16sdp_lutram.v`), and the descriptor ROMs are mapped to
  registers.
* EP2 (bulk video) is tied off — the device enumerates but streams nothing.
* No Type-C orientation/CC handling: flip the connector if the SS pairs
  land on the unused lane pair.
* Timing at the 10G boot trim is not closed (table above).
```
