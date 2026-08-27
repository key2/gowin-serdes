#!/usr/bin/env python3
"""Windowed loopback with 2-packet URBs (device buffer depth = 2)."""
import os, sys, time
import usb.core, usb.util
TOTAL = int(sys.argv[1]) * 1024 if len(sys.argv) > 1 else 2048 * 1024
dev = usb.core.find(idVendor=0x1209, idProduct=0x0001)
usb.util.claim_interface(dev, 0)
payload = os.urandom(TOTAL)
rx = bytearray()
t0 = time.time()
try:
    for off in range(0, TOTAL, 2048):
        dev.write(0x01, payload[off:off+2048], timeout=3000)
        rx.extend(dev.read(0x81, 2048, timeout=3000))
except Exception as e:
    print(f"failed at rx={len(rx)}: {e}")
dt = time.time() - t0
ok = bytes(rx) == payload[:len(rx)]
print(f"2KiB URBs: echoed {len(rx)}/{TOTAL} in {dt:.2f}s "
      f"-> {len(rx)/dt/1e6:.2f} MB/s each direction  "
      f"integrity={'OK' if ok else 'FAIL'}")
