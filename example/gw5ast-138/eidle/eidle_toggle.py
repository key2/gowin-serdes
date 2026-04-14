#!/usr/bin/env python3
"""Toggle TX Electrical Idle on SerDes Q0 Lane 0 via the serial DRP bridge.

Extracted from /tmp/usb3_0_phy_decrypted.v — the Gowin USB3 PHY controls
EIDLE through two CSR registers:

  CSR_WRITE_EIDLE  (0x8003A4):
      0x00000001  → assert electrical idle  (TX lines go to common mode)
      0x00000007  → deassert electrical idle (TX lines active)

  CSR_WRITE_PLUSE  (0x80033F):
      0x03000000  → assert LFPS pulse
      0x00000000  → deassert LFPS pulse  (PHY waits 250 clocks before this)

Usage:
    python eidle_toggle.py                   # toggle EIDLE on/off once
    python eidle_toggle.py --loop 10         # toggle 10 times
    python eidle_toggle.py --on              # assert EIDLE and leave it
    python eidle_toggle.py --off             # deassert EIDLE and leave it
    python eidle_toggle.py --pulse           # LFPS pulse sequence
    python eidle_toggle.py --delay 0.5       # 500ms between toggles

Requires: pyserial
"""

import argparse
import serial
import struct
import sys
import time

# ── DRP Bridge Protocol ──────────────────────────────────────────
PORT = "/dev/ttyUSB5"
BAUD = 115_200
TIMEOUT = 2


def drp_read(ser, addr):
    ser.reset_input_buffer()
    ser.write(b"R" + struct.pack(">I", addr)[1:])
    r = ser.read(5)
    if len(r) != 5:
        return None, 0xFF
    return struct.unpack(">I", r[:4])[0], r[4]


def drp_write(ser, addr, data):
    ser.reset_input_buffer()
    ser.write(b"W" + struct.pack(">I", addr)[1:] + struct.pack(">I", data))
    r = ser.read(1)
    if len(r) != 1:
        return 0xFF
    return r[0]


def drp_ping(ser):
    ser.reset_input_buffer()
    ser.write(b"T")
    return ser.read(1) == b"\x55"


# ── Q0 LN0 CSR Addresses ────────────────────────────────────────
CSR_WRITE_EIDLE = 0x8003A4
CSR_WRITE_PLUSE = 0x80033F
CSR_READ_RXDET  = 0x808b34


# EIDLE values (from FSM_WRITE_EIDLE_1)
EIDLE_ON = 0x00000001  # assert electrical idle (eidle_en_d1=1)
EIDLE_OFF = 0x00000007  # deassert electrical idle (eidle_en_d1=0)

# PLUSE values (from FSM_WRITE_PLUSE_1 / PLUSE_2)
PLUSE_ON = 0x03000000  # assert LFPS pulse
PLUSE_OFF = 0x00000000  # deassert LFPS pulse

STATUS_MAP = {0x00: "OK", 0x01: "RESP_ERR", 0xFF: "TIMEOUT"}


def st(s):
    return STATUS_MAP.get(s, f"0x{s:02X}")


def eidle_assert(ser):
    """Assert electrical idle: write 0x01 to CSR_WRITE_EIDLE."""
    ws = drp_write(ser, CSR_WRITE_EIDLE, EIDLE_ON)
    print(f"  EIDLE ON   0x{CSR_WRITE_EIDLE:06X} <- 0x{EIDLE_ON:08X}  ({st(ws)})")
    return ws


def eidle_deassert(ser):
    """Deassert electrical idle: write 0x07 to CSR_WRITE_EIDLE."""
    ws = drp_write(ser, CSR_WRITE_EIDLE, EIDLE_OFF)
    print(f"  EIDLE OFF  0x{CSR_WRITE_EIDLE:06X} <- 0x{EIDLE_OFF:08X}  ({st(ws)})")
    return ws


def eidle_read(ser):
    """Read current EIDLE register value."""
    val, s = drp_read(ser, CSR_READ_RXDET)
    if val is not None:
        state = (
            "ON (idle)"
            if val == EIDLE_ON
            else "OFF (active)"
            if val == EIDLE_OFF
            else "unknown"
        )
        print(f"  EIDLE read 0x{CSR_WRITE_EIDLE:06X} = 0x{val:08X}  ({state})")
    else:
        print(f"  EIDLE read 0x{CSR_WRITE_EIDLE:06X} = TIMEOUT")
    return val


def lfps_pulse(ser, hold_s=0.004):
    """LFPS pulse: assert PLUSE, wait, deassert.

    The PHY waits 250 clocks (~4us at 62.5MHz) before deasserting.
    Over serial we can't be that fast, so hold_s is the minimum hold
    time in seconds (default 4ms, well above 4us).
    """
    ws = drp_write(ser, CSR_WRITE_PLUSE, PLUSE_ON)
    print(f"  PLUSE ON   0x{CSR_WRITE_PLUSE:06X} <- 0x{PLUSE_ON:08X}  ({st(ws)})")
    time.sleep(hold_s)
    ws = drp_write(ser, CSR_WRITE_PLUSE, PLUSE_OFF)
    print(f"  PLUSE OFF  0x{CSR_WRITE_PLUSE:06X} <- 0x{PLUSE_OFF:08X}  ({st(ws)})")
    return ws


def main():
    ap = argparse.ArgumentParser(description="Toggle TX EIDLE on Q0 LN0 via DRP")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--on", action="store_true", help="Assert EIDLE and exit")
    ap.add_argument("--off", action="store_true", help="Deassert EIDLE and exit")
    ap.add_argument(
        "--pulse",
        action="store_true",
        help="Send LFPS pulse (combine with --loop N for N pulses)",
    )
    ap.add_argument(
        "--loop",
        type=int,
        default=0,
        help="Repeat N times (applies to toggle or --pulse)",
    )
    ap.add_argument(
        "--delay",
        type=float,
        default=0.1,
        help="Delay in seconds between toggles (default 0.1)",
    )
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=TIMEOUT)
    time.sleep(0.1)
    ser.reset_input_buffer()

    if not drp_ping(ser):
        print("ERROR: DRP bridge not responding")
        ser.close()
        sys.exit(1)
    print(f"Connected to {args.port} @ {args.baud}\n")

    # Read current state
    print("=== Current state ===")
    eidle_read(ser)
    print()

    if args.on:
        print("=== Assert EIDLE ===")
        eidle_assert(ser)

    elif args.off:
        print("=== Deassert EIDLE ===")
        eidle_deassert(ser)

    elif args.pulse:
        n = max(args.loop, 1)
        print(f"=== LFPS Pulse x{n} (delay={args.delay}s) ===")
        eidle_deassert(ser)
        time.sleep(0.01)
        for i in range(n):
            if n > 1:
                print(f"\n--- pulse {i + 1}/{n} ---")
            lfps_pulse(ser)
            if i < n - 1:
                time.sleep(args.delay)

    elif args.loop > 0:
        print(f"=== Toggle EIDLE x{args.loop} (delay={args.delay}s) ===")
        for i in range(args.loop):
            print(f"\n--- cycle {i + 1}/{args.loop} ---")
            eidle_assert(ser)
            time.sleep(args.delay)
            eidle_deassert(ser)
            time.sleep(args.delay)

    else:
        # Default: single toggle
        print("=== Toggle EIDLE ===")
        eidle_assert(ser)
        time.sleep(args.delay)
        eidle_deassert(ser)

    # Read final state
    print()
    print("=== Final state ===")
    eidle_read(ser)

    ser.close()


if __name__ == "__main__":
    main()
