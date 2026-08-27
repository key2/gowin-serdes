#!/usr/bin/env python3
"""Windowed loopback: at most WINDOW bytes in flight (bounded concurrency)."""
import os, sys, time
import usb.core, usb.util

TOTAL  = int(sys.argv[1]) * 1024 if len(sys.argv) > 1 else 1024 * 1024
WINDOW = int(sys.argv[2]) * 1024 if len(sys.argv) > 2 else 2 * 1024

dev = usb.core.find(idVendor=0x1209, idProduct=0x0001)
usb.util.claim_interface(dev, 0)

payload = os.urandom(TOTAL)
rx = bytearray()
sent = 0
t0 = time.time()
try:
    while len(rx) < TOTAL:
        # top up the window
        while sent < TOTAL and sent - len(rx) < WINDOW:
            n = min(1024, TOTAL - sent, WINDOW - (sent - len(rx)))
            dev.write(0x01, payload[sent:sent+n], timeout=3000)
            sent += n
        rx.extend(dev.read(0x81, 1024, timeout=3000))
except Exception as e:
    print(f"failed at sent={sent} rx={len(rx)}: {e}")
dt = time.time() - t0
ok = bytes(rx) == payload[:len(rx)]
print(f"window={WINDOW//1024}KiB: echoed {len(rx)}/{TOTAL} in {dt:.2f}s "
      f"-> {len(rx)/dt/1e6:.1f} MB/s  integrity={'OK' if ok else 'FAIL'}")
