#!/usr/bin/env python3
"""CDC-ACM echo validator: writes N random bytes, reads them back, compares.

Usage: sudo python3 acm_echo_test.py [/dev/ttyACM0] [bytes]
"""
import os, sys, termios, threading, time, hashlib

DEV = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
N   = int(sys.argv[2]) if len(sys.argv) > 2 else 65536
CHUNK = 4096

fd = os.open(DEV, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
attrs = termios.tcgetattr(fd)
attrs[0] = attrs[1] = attrs[3] = 0                # iflag/oflag/lflag: fully raw
attrs[2] = termios.CREAD | termios.CLOCAL | termios.CS8
termios.tcsetattr(fd, termios.TCSANOW, attrs)
termios.tcflush(fd, termios.TCIOFLUSH)

payload = os.urandom(N)
rx = bytearray()
t0 = time.time()

def writer():
    import select as _select
    off = 0
    while off < N:
        try:
            off += os.write(fd, payload[off:off+CHUNK])
        except BlockingIOError:
            _select.select([], [fd], [], 0.1)

w = threading.Thread(target=writer, daemon=True)
w.start()

import select
deadline = time.time() + 15
last_progress = time.time()
while len(rx) < N and time.time() < deadline:
    r, _, _ = select.select([fd], [], [], 0.5)
    if r:
        try:
            chunk = os.read(fd, 65536)
        except (OSError, BlockingIOError):
            chunk = b''
        if chunk:
            rx.extend(chunk)
            last_progress = time.time()
    if time.time() - last_progress > 3:
        print(f"stalled at rx={len(rx)}")
        break

dt = time.time() - t0
ok = bytes(rx) == payload
print(f"sent {N} bytes, received {len(rx)} in {dt:.2f}s "
      f"({len(rx)/dt/1e6:.2f} MB/s)")
print(f"sha256 tx {hashlib.sha256(payload).hexdigest()[:16]} "
      f"rx {hashlib.sha256(bytes(rx)).hexdigest()[:16]}")
print("ECHO PASS" if ok else "ECHO FAIL")
sys.exit(0 if ok else 1)
