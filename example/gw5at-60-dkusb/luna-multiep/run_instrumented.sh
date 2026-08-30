#!/bin/bash
# Instrumented 3-pair run: usbmon + uarts + test, aligned timestamps.
#
#   sudo ./run_instrumented.sh <tag> [MiB] [outdir]
#
# Writes usbmon_<tag>.txt, uart_<tag>_both.txt, t0_<tag>.txt, test_<tag>.txt
# into <outdir> (default /tmp/kilo).
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../../.." && pwd)"          # GW_USB3 workspace root
TAG=${1:?usage: run_instrumented.sh <tag> [MiB] [outdir]}
MIB=${2:-1}
OUT=${3:-/tmp/kilo}
mkdir -p "$OUT"
cd "$ROOT"
sudo -n cat /sys/kernel/debug/usb/usbmon/4u > "$OUT/usbmon_${TAG}.txt" &
MON=$!
sudo -n .venv/bin/python "$HERE/uart_capture.py" 40 "$OUT/uart_${TAG}" &
UART=$!
sleep 2
date +%s.%N > "$OUT/t0_${TAG}.txt"
sudo -n .venv/bin/python "$HERE/multiep_test.py" "$MIB" --eps 1,2,3 \
    > "$OUT/test_${TAG}.txt" 2>&1
date +%s.%N >> "$OUT/t0_${TAG}.txt"
wait $UART
sudo -n kill $MON 2>/dev/null
tail -6 "$OUT/test_${TAG}.txt"
