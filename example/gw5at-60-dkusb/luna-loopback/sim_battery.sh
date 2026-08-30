#!/bin/bash
# Full regression battery (HANDOVER 10j set, extended in sessions 8+).
# Run from anywhere: paths resolve relative to this script.
# Logs land in /tmp/kilo (created if missing); failures are copied to
# /tmp/kilo/batt_FAIL_<tag>.log for inspection.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../../.." && pwd)"
cd "$ROOT"
LL=gowin-serdes/example/gw5at-60-dkusb/luna-loopback
mkdir -p /tmp/kilo
run() {
    local tag="$1"; shift
    if env "$@" pdm run python "$LL/sim_link_loopback.py" \
        > /tmp/kilo/batt_last.log 2>&1; then
        echo "PASS $tag"
    else
        echo "FAIL $tag <<<<<<<<"
        cp /tmp/kilo/batt_last.log "/tmp/kilo/batt_FAIL_${tag// /_}.log"
    fi
}
run "default"
run "eps2" NUM_EPS=2
run "eps3" NUM_EPS=3
run "eps4" NUM_EPS=4
run "eps3+ctrl" NUM_EPS=3 WITH_CONTROL=1
run "ctrl" WITH_CONTROL=1
run "bytes4" LOOPBACK_BYTES=4
run "bytes1028" LOOPBACK_BYTES=1028
run "lbad5" LBAD_EVERY=5 LOOPBACK_BYTES=8192
run "badhdr7" BADHDR_EVERY=7 LOOPBACK_BYTES=8192
run "bubbles3" HOST_BUBBLES=3 LOOPBACK_BYTES=8192
run "window4" WINDOW_KIB=4 LOOPBACK_BYTES=16384
run "eps3+ctrl+itp" NUM_EPS=3 WITH_CONTROL=1 ITP_EVERY=211 HOST_LDN_EVERY=97
# link-recovery conformance (session 9): retrain from U0 mid-traffic;
# unacked headers must be retransmitted per the advertisement, not flushed
run "rec-every" RECOVERY_EVERY=1200 NUM_EPS=3
run "rec+lbad" RECOVERY_EVERY=2000 NUM_EPS=2 LBAD_EVERY=5
run "rec+badhdr" RECOVERY_EVERY=2000 NUM_EPS=2 BADHDR_EVERY=7
run "rec+ctrl" RECOVERY_AT=1500 WITH_CONTROL=1 NUM_EPS=3 LOOPBACK_BYTES=8192
run "rec+urb" RECOVERY_EVERY=3000 NUM_EPS=3 URB_PACKETS=4 URB_GAP=800 HOST_LATENCY=20 LC_LATENCY=12 REORDER=1
# bMaxBurst>1 engines (session 9): multi-packet bursts per token, per-packet
# sequence advance, cumulative ACKs, EOB, rewind-on-retry, OUT window
# advertisement.  (Recovery below ~2600-cycle periods with 3 bursting pipes
# is a starvation regime -- recovery overhead plus the spec-correct
# EOB/terminating-ACK/ERDY round trip per burst (session 10) eats the
# service windows -- not a correctness bound; see HANDOVER 10m/10n.)
run "burst" BURST=4 NUMP=4 NUM_EPS=3
run "burst+sweep" BURST=4 NUMP=4 NUMP_SWEEP=1 NUM_EPS=2
run "burst+lbad" BURST=4 NUMP=4 LBAD_EVERY=5
run "burst+badhdr" BURST=4 NUMP=4 BADHDR_EVERY=7
run "burst+short" BURST=4 NUMP=4 LOOPBACK_BYTES=1028
run "burst+rec" BURST=4 NUMP=4 RECOVERY_EVERY=2600 NUM_EPS=3
run "burst+urb" BURST=4 NUMP=4 NUM_EPS=3 URB_PACKETS=4 URB_GAP=800 HOST_LATENCY=20 LC_LATENCY=12 REORDER=1
run "burst+ctrl" BURST=2 NUMP=2 WITH_CONTROL=1 NUM_EPS=3 LOOPBACK_BYTES=8192
# bug-#35 regression (session 10): the bench xHC pipelines its whole
# scheduling window at transfer start (16 back-to-back OUT DPs before any
# device ACK); with the hardware's elastic loopback FIFO behind the OUT
# engine, the ring can become non-full MID-PACKET and the un-fixed engine
# commits partial tails / phantom ZLPs (the BURST=2 2048-byte bench wedge).
run "burst+blast" BURST=2 NUMP=2 OUT_WINDOW0=16 WITH_FIFO=4096 NUM_EPS=1 LOOPBACK_BYTES=65536 RETRY_TIMEOUT=1500
run "burst+blast3" BURST=2 NUMP=2 OUT_WINDOW0=16 WITH_FIFO=4096 NUM_EPS=3 LOOPBACK_BYTES=32768 RETRY_TIMEOUT=2500
run "blast+fifo1" BURST=1 NUMP=1 OUT_WINDOW0=16 WITH_FIFO=4096 NUM_EPS=1 LOOPBACK_BYTES=65536 RETRY_TIMEOUT=1500
if pdm run python "$LL/sim_loopback.py" > /tmp/kilo/batt_ep.log 2>&1; then
    echo "PASS endpoint-sim"
else
    echo "FAIL endpoint-sim <<<<<<<<"
fi
if pdm run pytest tests/ -q > /tmp/kilo/batt_pytest.log 2>&1; then
    echo "PASS pytest ($(grep -Eo '[0-9]+ passed' /tmp/kilo/batt_pytest.log | head -1))"
else
    echo "FAIL pytest <<<<<<<<"
fi
echo BATTERY-DONE
# training sims (added session 8)
for s in gowin_gtr12_sim gowin_gtr12_training_sim; do
    if pdm run python luna/examples/usb/superspeed/$s.py > /tmp/kilo/batt_$s.log 2>&1; then
        echo "PASS $s"
    else
        echo "FAIL $s <<<<<<<<"
    fi
done
echo BATTERY2-DONE
# stale-ACK race regression (bug #26, session 8)
if pdm run python "$LL/sim_stale_ack.py" > /tmp/kilo/batt_staleack.log 2>&1; then
    echo "PASS stale-ack"
else
    echo "FAIL stale-ack <<<<<<<<"
fi
# adversarial TX-chain fuzz (found #24/#25, session 8; in-repo session 9)
if pdm run python "$LL/tx_fuzz.py" > /tmp/kilo/batt_txfuzz.log 2>&1; then
    echo "PASS tx-fuzz"
else
    echo "FAIL tx-fuzz <<<<<<<<"
fi
echo BATTERY3-DONE
