#!/usr/bin/env python3
"""Deterministic reproduction of the #23 per-pipe IN wedge (RTL level).

Scenario (hardware forensics, HANDOVER 10l): an IN endpoint in
WAIT_FOR_ACK receives a STALE acknowledgement (e.g. a link-level DL
replay of the previous ACK after an LBAD, or any duplicate) -- the
``retry_requested | ~sequence_advancing`` arm correctly starts a
protocol resend.  The GENUINE ACK+token then arrives while the FSM is
in SEND_PACKET / FINISH_LAST_WORD, where handshake strobes are ignored
-- the single-cycle event is dropped.  Device: parked in WAIT_FOR_ACK
forever.  Host: waiting for a data packet it already granted credit
for; times out and halts the pipe (-71 EPROTO).

Run: pdm run python gowin-serdes/example/gw5at-60-dkusb/luna-loopback/sim_stale_ack.py
Expected: STALE-ACK TEST PASS (fails with 'WEDGED' on a tree without
the pending-acknowledgement latch in SuperSpeedStreamInEndpoint).
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(HERE / "luna"))
sys.path.insert(0, str(HERE))

from amaranth import *
from amaranth.sim import Simulator

from luna.gateware.usb.usb3.endpoints.stream import SuperSpeedStreamInEndpoint


class Bench(Elaboratable):
    def __init__(self):
        self.ep = SuperSpeedStreamInEndpoint(endpoint_number=1,
                                             max_packet_size=1024,
                                             generate_zlps=False)

    def elaborate(self, platform):
        m = Module()
        m.domains += ClockDomain("ss")
        m.submodules.ep = self.ep
        return m


def main():
    bench = Bench()
    ep = bench.ep
    iface = ep.interface
    sim = Simulator(bench)
    sim.add_clock(8e-9, domain="ss")

    async def tb(ctx):
        hs = iface.handshakes_in
        tx = iface.tx

        async def tick(n=1):
            for _ in range(n):
                await ctx.tick("ss")

        async def feed_packet():
            """Feed 1024 bytes into the endpoint's stream."""
            ctx.set(ep.stream.valid, 0b1111)
            for w in range(256):
                ctx.set(ep.stream.payload, 0x01010101 * (w & 0xFF))
                ctx.set(ep.stream.first, 1 if w == 0 else 0)
                ctx.set(ep.stream.last, 1 if w == 255 else 0)
                await ctx.tick("ss").until(ep.stream.ready)
            ctx.set(ep.stream.valid, 0)
            ctx.set(ep.stream.last, 0)

        async def pulse_ack(nseq, nump=1, retry=0):
            ctx.set(hs.endpoint_number, 1)
            ctx.set(hs.next_sequence, nseq)
            ctx.set(hs.number_of_packets, nump)
            ctx.set(hs.retry_required, retry)
            ctx.set(hs.ack_received, 1)
            await tick()
            ctx.set(hs.ack_received, 0)
            ctx.set(hs.number_of_packets, 0)

        async def drain_packet(ready_gap=0):
            """Consume one packet from interface.tx; returns word count.
            ``ready_gap`` throttles to stretch SEND_PACKET (models mux
            contention with other endpoints)."""
            words = 0
            idle = 0
            while True:
                ctx.set(tx.ready, 1)
                _, _, v, last = await ctx.tick("ss").sample(tx.valid, tx.last)
                if v:
                    idle = 0
                    words += 1
                    if last:
                        ctx.set(tx.ready, 0)
                        return words
                    if ready_gap:
                        ctx.set(tx.ready, 0)
                        await tick(ready_gap)
                else:
                    idle += 1
                    if idle > 4000:
                        return words

        # -- bring-up: two packets fed, first token, packet 0 exchange ----
        await tick(4)
        await feed_packet()          # packet seq 0 (fills buffer A)
        await feed_packet()          # packet seq 1 (fills buffer B)
        await pulse_ack(nseq=0, nump=1)          # IN token for seq 0
        n = await drain_packet()
        assert n == 256, f"packet 0 words {n}"
        print(f"packet seq0 sent ({n} words), fsm={ctx.get(ep.debug_fsm)}")

        # ACK seq0 + token: endpoint sends packet seq 1.
        await pulse_ack(nseq=1, nump=1)
        n = await drain_packet()
        assert n == 256, f"packet 1 words {n}"
        await tick(4)
        print(f"packet seq1 sent, fsm={ctx.get(ep.debug_fsm)} (5=WAIT_FOR_ACK)")
        assert ctx.get(ep.debug_fsm) == 5

        # -- the race ------------------------------------------------------
        # STALE ACK: nseq=1 (a DL-replayed copy of the previous ACK).
        # Not advancing -> the endpoint starts a protocol resend of seq 1.
        await pulse_ack(nseq=1, nump=1)
        await tick(4)
        print(f"stale ACK delivered, fsm={ctx.get(ep.debug_fsm)} "
              f"(3=SEND_PACKET: resending)")

        # GENUINE ACK for seq1 arrives while the resend is in flight
        # (SEND_PACKET, stretched by mux contention).
        await pulse_ack(nseq=2, nump=1)
        print(f"genuine ACK delivered mid-resend, fsm={ctx.get(ep.debug_fsm)}")

        # Drain the resent packet; then the host is silent (it has no
        # reason to speak: it granted a token in the genuine ACK).
        n = await drain_packet()
        print(f"resent packet drained ({n} words)")

        # Feed more data so WAIT_FOR_DATA is not the parking reason.
        await feed_packet()

        # Wait and observe; service the endpoint's flow-control handshakes
        # like a live host/arbiter would (NRDY/ERDY dispatched; an ERDY is
        # answered with a fresh token for the current expectation).
        hso = iface.handshakes_out
        erdy_seen = False
        for _ in range(3000):
            await tick()
            if ctx.get(tx.valid):
                # endpoint is transmitting the next packet: the token
                # (or the ERDY-recovered flow) was honored.
                n = await drain_packet()
                print(f"endpoint recovered and sent next packet ({n} words)"
                      f"{' via NRDY/ERDY' if erdy_seen else ''}")
                print("STALE-ACK TEST PASS")
                return
            if ctx.get(hso.send_nrdy):
                ctx.set(hso.done, 1)
                await tick()
                ctx.set(hso.done, 0)
            elif ctx.get(hso.send_erdy):
                erdy_seen = True
                ctx.set(hso.done, 1)
                await tick()
                ctx.set(hso.done, 0)
                await tick(20)          # host turnaround
                await pulse_ack(nseq=2, nump=1)
        fsm = ctx.get(ep.debug_fsm)
        print(f"endpoint parked in fsm={fsm} "
              f"({'WAIT_FOR_ACK' if fsm == 5 else fsm}) with data queued "
              f"and a granted token unanswered")
        print("WEDGED (race reproduced) -- host would EPROTO this pipe")
        raise SystemExit(1)

    sim.add_testbench(tb)
    sim.run()


if __name__ == "__main__":
    main()
