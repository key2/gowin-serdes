#!/usr/bin/env python3
"""Simultaneous full-duplex loopback across all three endpoint pairs.

For every bulk pair (EP1..EP3), a writer thread streams a distinct
random payload to OUT while a reader thread concurrently collects the
echo from IN; all six threads run at once.  Verifies each pipe's echo
byte-exactly and reports per-pipe and aggregate throughput.

    sudo python multiep_test.py [MiB-per-EP] [--eps 1,2,3]
"""

import argparse
import hashlib
import os
import sys
import threading
import time

import usb.core
import usb.util

VID, PID = 0x1209, 0x0001
CHUNK = 16384


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mib", nargs="?", type=float, default=4.0,
                    help="MiB to echo per endpoint pair (default 4)")
    ap.add_argument("--eps", default="1,2,3",
                    help="comma-separated endpoint numbers (default 1,2,3; the\n"
                         "shipping build has three pairs -- see top.py)")
    ap.add_argument("--no-warmup", action="store_true",
                    help="skip the serial 1 KiB warmup echo (bug-#34 probe "
                         "runs: keeps the event capture for the burst phase)")
    args = ap.parse_args()

    eps = [int(e) for e in args.eps.split(",")]
    total = int(args.mib * 1024 * 1024)

    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit("device 1209:0001 not found")
    usb.util.claim_interface(dev, 0)
    print(f"device speed: {dev.speed} "
          f"({ {3: 'high', 4: 'super', 5: 'super+'}.get(dev.speed, '?')})")

    payloads = {ep: os.urandom(total) for ep in eps}
    rx = {ep: bytearray() for ep in eps}
    errors = []

    def writer(ep):
        try:
            data = payloads[ep]
            off = 0
            while off < total:
                n = dev.write(ep, data[off:off + CHUNK], timeout=5000)
                off += n
        except Exception as e:
            errors.append(f"ep{ep} writer: {e!r}")

    def reader(ep):
        try:
            buf = rx[ep]
            while len(buf) < total:
                buf.extend(dev.read(0x80 | ep, CHUNK, timeout=5000))
        except Exception as e:
            errors.append(f"ep{ep} reader: {e!r} (rx={len(rx[ep])})")

    # warmup: one packet through each pipe, serially
    if not args.no_warmup:
        for ep in eps:
            probe = os.urandom(1024)
            dev.write(ep, probe, timeout=3000)
            back = bytes(dev.read(0x80 | ep, 1024, timeout=3000))
            assert back == probe, f"ep{ep} warmup mismatch"
        print("warmup echo OK on all pipes")

    threads = [threading.Thread(target=fn, args=(ep,), daemon=True)
               for ep in eps for fn in (writer, reader)]
    t0 = time.time()
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=120)
    dt = time.time() - t0

    ok = True
    for e in errors:
        print("ERROR:", e)
        ok = False
    for ep in eps:
        good = bytes(rx[ep]) == payloads[ep]
        ok &= good
        print(f"ep{ep}: echoed {len(rx[ep])}/{total} bytes "
              f"-> {len(rx[ep]) / dt / 1e6:.1f} MB/s per direction  "
              f"sha256 tx {hashlib.sha256(payloads[ep]).hexdigest()[:16]} "
              f"rx {hashlib.sha256(bytes(rx[ep])).hexdigest()[:16]}  "
              f"integrity={'OK' if good else 'FAIL'}")
    agg = sum(len(rx[ep]) for ep in eps) / dt / 1e6
    print(f"aggregate: {agg:.1f} MB/s per direction across {len(eps)} pipes "
          f"in {dt:.2f} s")
    print("MULTI-EP TEST PASS" if ok else "MULTI-EP TEST FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
