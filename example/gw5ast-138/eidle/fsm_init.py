#!/usr/bin/env python3
"""Replay the USB3 PHY FSM_INIT sequence over the serial DRP bridge.

Extracted from /tmp/usb3_0_phy_decrypted.v — the Gowin USB3 PHY's
``upar_csr`` module performs 11 sequential DRP writes at power-on
before entering FSM_IDLE.

This script writes all 11 FSM_INIT registers, then reads CSR_READ_RXDET.

Usage:
    python fsm_init.py
    python fsm_init.py --port /dev/ttyUSB3

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


def drp_ping(ser):
    ser.reset_input_buffer()
    ser.write(b"T")
    r = ser.read(1)
    return r == b"\x55"


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


# ── FSM_INIT Register Sequence ───────────────────────────────────
# From usb3_0_phy_decrypted.v, `ifdef Q0_LN0 addresses.
# The FSM walks cnt0 = 0..10, writing each (addr, data) pair.
# CSR_WRITE_CDR_CFG (0x9083f8) and CSR_WRITE_LN_CTRL (0x908830)
# are common/hard-macro registers (not lane-specific).

FSM_INIT_WRITES = [
    # cnt0, name,                addr,       data
    (0, "CSR_TX_FFE_0", 0x808234, 0x0000_F000),
    (1, "CSR_TX_FFE_1", 0x808238, 0x0000_0000),
    (2, "CSR_TX_FFE_2", 0x8082D8, 0x0000_0110),
    (3, "CSR_WRITE_CDR_CFG", 0x9083F8, 0x0003_8002),
    (4, "CSR_WRITE_LN_CTRL", 0x908830, 0xFFFF_F9FF),
    (5, "CSR_WRITE_CDR_CFG_0", 0x800253, 0x7F00_0000),
    (6, "CSR_WRITE_CDR_CFG_1", 0x80025E, 0x007F_0000),
    (7, "CSR_WRITE_CDR_CFG_2", 0x80025F, 0x7F00_0000),
    (8, "CSR_WRITE_CDR_CFG_3", 0x800254, 0x0000_004F),
    (9, "CSR_WRITE_CDR_CFG_4", 0x800260, 0x0000_004F),
    (10, "CSR_WRITE_CDR_CFG_5", 0x800261, 0x0000_4F00),
]

CSR_READ_RXDET = 0x808B34

STATUS_MAP = {0x00: "OK", 0x01: "RESP_ERR", 0xFF: "TIMEOUT"}


def status_str(s):
    return STATUS_MAP.get(s, f"0x{s:02X}")


def main():
    ap = argparse.ArgumentParser(description="Replay USB3 PHY FSM_INIT via DRP bridge")
    ap.add_argument("--port", default=PORT, help=f"Serial port (default: {PORT})")
    ap.add_argument("--baud", type=int, default=BAUD)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=TIMEOUT)
    time.sleep(0.1)
    ser.reset_input_buffer()

    # ── Ping ──────────────────────────────────────────────────
    if not drp_ping(ser):
        print("ERROR: DRP bridge not responding (ping failed)")
        ser.close()
        sys.exit(1)
    print(f"Connected to {args.port} @ {args.baud}  (ping OK)\n")

    # ── Write FSM_INIT sequence ───────────────────────────────
    print("=== FSM_INIT writes ===")
    for cnt, name, addr, data in FSM_INIT_WRITES:
        ws = drp_write(ser, addr, data)
        print(
            f"  [{cnt:2d}] {name:<24}  0x{addr:06X} <- 0x{data:08X}  ({status_str(ws)})"
        )

    # ── Read RXDET ────────────────────────────────────────────
    print()
    val, st = drp_read(ser, CSR_READ_RXDET)
    if val is not None:
        print(
            f"CSR_READ_RXDET  0x{CSR_READ_RXDET:06X} = 0x{val:08X}  ({status_str(st)})"
        )
    else:
        print(f"CSR_READ_RXDET  0x{CSR_READ_RXDET:06X} = TIMEOUT")

    ser.close()


if __name__ == "__main__":
    main()
