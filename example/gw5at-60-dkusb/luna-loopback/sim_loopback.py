#!/usr/bin/env python3
"""Endpoint-layer loopback simulation with a USB3 host model.

Exercises the exact device-side flow-control machinery of the bulk echo
(SuperSpeedStreamOutEndpoint -> SuperSpeedStreamInEndpoint through the
endpoint multiplexer and the real TransactionPacketGenerator) against a
host model that speaks the endpoint-interface dialect:

  * bulk OUT: streams DP payloads into the shared rx interface (header +
    payload + rx_complete), honours NumP=1 (one DP in flight until the
    device ACKs), retransmits on rty/unexpected-sequence ACKs;
  * bulk IN: issues IN tokens (registered handshakes_in), consumes DPs
    from the shared tx interface, ACKs them with advancing sequence
    numbers, parks on NRDY and resumes on ERDY -- like an xHC;
  * checks the echoed byte stream for integrity and watchdogs any
    stall, dumping an event trace.

Every flow-control bug found during the hardware bring-up (HANDOVER
10g bugs 1-2, 6-7) reproduces in this bench; run it after touching any
of the involved gateware:

    pdm run python gowin-serdes/example/gw5at-60-dkusb/luna-loopback/sim_loopback.py
"""

import os
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXAMPLE_ROOT = HERE.parent
WORKSPACE = HERE.parents[3]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(EXAMPLE_ROOT / "luna-acm"))    # ss_stream_out
sys.path.insert(0, str(WORKSPACE / "luna"))
sys.path.insert(0, str(WORKSPACE))

from amaranth import *
from amaranth.sim import Simulator

from luna.gateware.usb.usb3.endpoints.stream import SuperSpeedStreamInEndpoint
from luna.gateware.usb.usb3.protocol.endpoint import SuperSpeedEndpointMultiplexer
from luna.gateware.usb.usb3.protocol.transaction import TransactionPacketGenerator

from ss_stream_out import SuperSpeedStreamOutEndpoint

TOTAL_BYTES = int(os.environ.get("LOOPBACK_BYTES", 16384))
MPS = 1024
SEED = int(os.environ.get("LOOPBACK_SEED", 1))


class LoopbackBench(Elaboratable):
    def __init__(self):
        self.out_ep = SuperSpeedStreamOutEndpoint(
            endpoint_number=1, max_packet_size=MPS)
        self.in_ep = SuperSpeedStreamInEndpoint(
            endpoint_number=1, max_packet_size=MPS, generate_zlps=False)
        self.mux = SuperSpeedEndpointMultiplexer()
        self.gen = TransactionPacketGenerator()

    def elaborate(self, platform):
        m = Module()
        m.domains += ClockDomain("ss")

        m.submodules.out_ep = self.out_ep
        m.submodules.in_ep = self.in_ep
        m.submodules.mux = self.mux
        m.submodules.gen = self.gen

        self.mux.add_interface(self.out_ep.interface)
        self.mux.add_interface(self.in_ep.interface)

        # echo: OUT drains straight into IN
        m.d.comb += self.in_ep.stream.stream_eq(self.out_ep.stream)

        # generator exactly as wired by protocol/layer.py
        m.d.comb += self.gen.interface.connect(self.mux.shared.handshakes_out)
        m.d.comb += self.gen.address.eq(5)

        return m


def main():
    bench = LoopbackBench()
    sim = Simulator(bench)
    sim.add_clock(8e-9, domain="ss")

    rng = random.Random(SEED)
    payload = bytes(rng.getrandbits(8) for _ in range(TOTAL_BYTES))
    packets = [payload[i:i + MPS] for i in range(0, len(payload), MPS)]

    shared = bench.mux.shared
    hq = bench.gen.header_source

    events = []

    hs_events = []                   # (cycle, kind, ep, nseq, rty)
    dp_events = []                   # (cycle, seq, bytes)
    clk = [0]

    async def monitor(ctx):
        """Every-cycle monitor: consumes generator headers, records
        handshake dispatches, and collects the device's tx stream into
        completed data packets (all of these are single-cycle events
        the main coroutine would otherwise miss)."""
        ctx.set(shared.tx.ready, 1)
        word_buf = bytearray()
        while True:
            await ctx.tick("ss")
            clk[0] += 1
            # consume generator headers like the real link would
            ctx.set(hq.ready, 1 if ctx.get(hq.valid) else 0)
            if ctx.get(shared.handshakes_out.ready):
                for kind in ("send_ack", "send_nrdy", "send_erdy"):
                    if ctx.get(getattr(shared.handshakes_out, kind)):
                        ep = ctx.get(shared.handshakes_out.endpoint_number)
                        nseq = ctx.get(shared.handshakes_out.next_sequence)
                        rty = ctx.get(shared.handshakes_out.retry_required)
                        events.append((clk[0], kind, ep, nseq, rty))
                        st = state
                        if kind == "send_ack" and ep == 1:
                            if rty:
                                st["out_seq"] = nseq
                                st["out_wait_ack"] = False
                            elif st["out_wait_ack"] and \
                                    nseq == ((st["out_seq"] + 1) & 0x1F):
                                st["out_seq"] = nseq
                                st["out_idx"] += 1
                                st["out_wait_ack"] = False
                                st["last_progress"] = clk[0]
                        elif kind == "send_nrdy":
                            st["in_parked"] = True
                            st["token_outstanding"] = False
                        elif kind == "send_erdy":
                            st["in_parked"] = False
                            st["token_outstanding"] = False
            # windowed every-cycle trace (env: TRACE_LO..TRACE_HI)
            tlo = int(os.environ.get("TRACE_LO", 0))
            thi = int(os.environ.get("TRACE_HI", 0))
            if tlo <= clk[0] < thi:
                print((clk[0], "trace",
                    f"ofill={ctx.get(bench.out_ep.debug_fill)} "
                    f"ototal={ctx.get(bench.out_ep.debug_total)} "
                    f"opos={ctx.get(bench.out_ep.debug_pos)} "
                    f"s.val={ctx.get(bench.out_ep.stream.valid):04b} "
                    f"s.rdy={ctx.get(bench.out_ep.stream.ready)} "
                    f"s.fst={ctx.get(bench.out_ep.stream.first)} "
                    f"s.lst={ctx.get(bench.out_ep.stream.last)} "
                    f"wfill={ctx.get(bench.in_ep.debug_write_fill)} "
                    f"rfill={ctx.get(bench.in_ep.debug_read_fill)} "
                    f"ifsm={ctx.get(bench.in_ep.debug_fsm)} "
                    f"erq={ctx.get(bench.in_ep.debug_erdyreq)} "
                    f"ps={ctx.get(bench.mux.debug_pass_sel)} "
                    f"gr={ctx.get(bench.mux.debug_grant)} "
                    f"p0={ctx.get(bench.mux.debug_pending[0]):04b} "
                    f"p1={ctx.get(bench.mux.debug_pending[1]):04b} "
                    f"pd={ctx.get(bench.mux.debug_pdisp)} "
                    f"gdone={ctx.get(bench.gen.interface.done)} "
                    f"grdy={ctx.get(bench.gen.interface.ready)} "
                    f"edone={ctx.get(bench.in_ep.interface.handshakes_out.done)} "
                    f"tok={ctx.get(shared.handshakes_in.ack_received)}"))
            # debug: trace IN-EP buffer bookkeeping on change
            dbg = (ctx.get(bench.in_ep.debug_write_fill),
                   ctx.get(bench.in_ep.debug_read_fill),
                   ctx.get(bench.in_ep.debug_pingpong),
                   ctx.get(bench.in_ep.debug_ready))
            if False and dbg != getattr(monitor, "_dbg", None):
                monitor._dbg = dbg
                events.append((clk[0], "in-ep",
                               f"wfill={dbg[0]} rfill={dbg[1]} "
                               f"pp={dbg[2]} rdy={dbg[3]}"))
            # collect tx words
            lanes = ctx.get(shared.tx.valid)
            if lanes:
                word = ctx.get(shared.tx.payload)
                nbytes = bin(lanes).count("1")
                word_buf += word.to_bytes(4, "little")[:nbytes]
                if ctx.get(shared.tx.last):
                    dp_events.append((clk[0],
                                      ctx.get(shared.tx_sequence_number),
                                      bytes(word_buf)))
                    word_buf.clear()

    state = {
        "rx_bytes": bytearray(),
        "out_seq": 0, "out_idx": 0, "out_wait_ack": False,
        "in_seq_expect": 0, "in_parked": False, "token_outstanding": False,
        "last_progress": 0, "done": False,
    }
    hs_rd = [0]
    dp_rd = [0]

    async def out_driver(ctx):
        """Streams OUT DPs; NumP=1 discipline (wait for the device ACK)."""
        st = state
        while st["out_idx"] < len(packets):
            if not st["out_wait_ack"] and st["out_idx"] < len(packets):
                idx = st["out_idx"]
                data = packets[idx]
                events.append((clk[0], "host-OUT", idx, st["out_seq"]))
                ctx.set(shared.rx_header.endpoint_number, 1)
                ctx.set(shared.rx_header.direction, 0)
                ctx.set(shared.rx_header.data_sequence, st["out_seq"])
                ctx.set(shared.rx_header.data_length, len(data))
                words = [data[k:k + 4] for k in range(0, len(data), 4)]
                for wi, w in enumerate(words):
                    ctx.set(shared.rx.payload, int.from_bytes(w, "little"))
                    ctx.set(shared.rx.valid, (1 << len(w)) - 1)
                    ctx.set(shared.rx.first, 1 if wi == 0 else 0)
                    ctx.set(shared.rx.last, 1 if wi == len(words) - 1 else 0)
                    await ctx.tick("ss")
                ctx.set(shared.rx.valid, 0)
                ctx.set(shared.rx.first, 0)
                ctx.set(shared.rx.last, 0)
                ctx.set(shared.rx_complete, 1)
                await ctx.tick("ss")
                ctx.set(shared.rx_complete, 0)
                st["out_wait_ack"] = True
            else:
                await ctx.tick("ss")

    async def in_driver(ctx):
        """Keeps an IN token pending; ACKs received DPs (like an xHC)."""
        st = state

        async def pulse_token(seq):
            ctx.set(shared.handshakes_in.endpoint_number, 1)
            ctx.set(shared.handshakes_in.number_of_packets, 1)
            ctx.set(shared.handshakes_in.next_sequence, seq)
            ctx.set(shared.handshakes_in.retry_required, 0)
            ctx.set(shared.handshakes_in.ack_received, 1)
            await ctx.tick("ss")
            ctx.set(shared.handshakes_in.ack_received, 0)
            ctx.set(shared.handshakes_in.number_of_packets, 0)

        while len(state["rx_bytes"]) < TOTAL_BYTES:
            # drain handshake events that affect IN polling
            # (out_driver owns hs_rd; mirror parked/token here read-only)
            # deadlock watchdog
            if clk[0] - st["last_progress"] > 60_000:
                print(f"DEADLOCK at cycle {clk[0]}: "
                      f"echoed={len(st['rx_bytes'])}/{TOTAL_BYTES} "
                      f"out_idx={st['out_idx']} "
                      f"out_wait_ack={st['out_wait_ack']} "
                      f"in_parked={st['in_parked']} "
                      f"token_out={st['token_outstanding']}")
                for e in events[-40:]:
                    print("   ", e)
                raise SystemExit(1)

            if dp_rd[0] < len(dp_events):
                ec, seq, data = dp_events[dp_rd[0]]
                dp_rd[0] += 1
                events.append((ec, "dev-DP", seq, len(data)))
                if seq == st["in_seq_expect"]:
                    st["rx_bytes"].extend(data)
                    st["in_seq_expect"] = (st["in_seq_expect"] + 1) & 0x1F
                    st["last_progress"] = clk[0]
                st["token_outstanding"] = False
                # ACK (advancing); doubles as the next IN token (NumP=1)
                await pulse_token(st["in_seq_expect"])
                st["token_outstanding"] = True
                continue

            if not st["in_parked"] and not st["token_outstanding"]:
                events.append((clk[0], "host-IN-token", st["in_seq_expect"]))
                await pulse_token(st["in_seq_expect"])
                st["token_outstanding"] = True
                continue

            await ctx.tick("ss")

        cyc = clk[0]
        print(f"echoed {len(st['rx_bytes'])} bytes in {cyc} cycles "
              f"({len(st['rx_bytes']) / (cyc * 8e-9) / 1e6:.1f} MB/s at the "
              f"endpoint layer)")
        assert bytes(st["rx_bytes"]) == payload, "DATA CORRUPTION"
        print("LOOPBACK SIM PASS")
        state["done"] = True

    sim.add_testbench(monitor, background=True)
    sim.add_testbench(out_driver, background=True)
    sim.add_testbench(in_driver)
    with sim.write_vcd("/tmp/kilo/loopback_sim.vcd"):
        sim.run()


if __name__ == "__main__":
    main()
