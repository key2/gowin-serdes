#!/usr/bin/env python3
"""Bulk loopback bandwidth + integrity test for the luna-loopback device.

Writes a random payload to EP1 OUT while concurrently reading the echo
from EP1 IN, verifies byte-exactness, and reports the achieved
throughput.  The loopback is full-duplex at the USB level, so the
reported rate is simultaneously the OUT and the IN rate.

    sudo python bandwidth_test.py [MiB] [--chunk KiB]

Requires pyusb (libusb).  The device is vendor-class; no kernel driver
needs detaching.
"""

import argparse
import hashlib
import sys
import threading
import time

import usb.core
import usb.util

VID, PID = 0x1209, 0x0001
EP_OUT, EP_IN = 0x01, 0x81


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mib", nargs="?", type=float, default=16.0,
                    help="total payload size in MiB (default 16)")
    ap.add_argument("--chunk", type=int, default=256,
                    help="write chunk size in KiB (default 256)")
    args = ap.parse_args()

    total = int(args.mib * 1024 * 1024)
    chunk = args.chunk * 1024

    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit("device 1209:0001 not found")
    if dev.speed is not None:
        speeds = {3: "high", 4: "super", 5: "super+"}
        print(f"device speed: {speeds.get(dev.speed, dev.speed)}")

    for intf in (0,):
        try:
            if dev.is_kernel_driver_active(intf):
                dev.detach_kernel_driver(intf)
        except Exception:
            pass
    usb.util.claim_interface(dev, 0)

    # warmup / pipe-clear: one small exchange
    dev.write(EP_OUT, b"ping", timeout=2000)
    got = bytes(dev.read(EP_IN, 1024, timeout=2000))
    assert got == b"ping", f"warmup echo mismatch: {got!r}"
    print("warmup echo OK")

    payload = bytes(bytearray(__import__("os").urandom(total)))

    rx = bytearray()
    errors = []

    def writer():
        try:
            off = 0
            while off < total:
                n = dev.write(EP_OUT, payload[off:off + chunk], timeout=10000)
                off += n
        except Exception as e:
            errors.append(f"writer: {e}")

    def reader():
        try:
            while len(rx) < total:
                data = dev.read(EP_IN, 1024 * 1024, timeout=10000)
                rx.extend(data)
        except Exception as e:
            errors.append(f"reader: {e}")

    t0 = time.time()
    wt = threading.Thread(target=writer)
    rt = threading.Thread(target=reader)
    wt.start(); rt.start()
    wt.join(timeout=max(60, total / 1e6))
    rt.join(timeout=max(60, total / 1e6))
    dt = time.time() - t0

    ok = bytes(rx) == payload
    mb = len(rx) / 1e6
    print(f"echoed {len(rx)}/{total} bytes in {dt:.2f} s "
          f"-> {mb / dt:.1f} MB/s ({mb / dt * 8:.2f} Mbit/s x2 directions)")
    print(f"sha256 tx {hashlib.sha256(payload).hexdigest()[:16]} "
          f"rx {hashlib.sha256(bytes(rx)).hexdigest()[:16]}")
    for e in errors:
        print("ERROR:", e)
    print("BANDWIDTH TEST", "PASS" if ok and not errors else "FAIL")
    return 0 if ok and not errors else 1


if __name__ == "__main__":
    sys.exit(main())
