#!/usr/bin/env python3
"""Adversarial fuzz of the shared IN-transmit chain (open item #23).

Instantiates the REAL modules of the device's data transmit path:

    3x endpoint TX interfaces -> SuperSpeedEndpointMultiplexer (packet-
    granular grant lock) -> TxDataSkidBuffer -> DataPacketTransmitter ->
    HeaderQueueArbiter (+ a scripted TP header producer, as the real
    protocol layer interleaves TP headers) -> PacketTransmitter

and drives it with:

  * three concurrent scripted "endpoints" that obey the endpoint
    contract (valid held for the whole packet; parameters driven
    combinationally while sending; packets of randomized lengths),
  * scripted link credits (LGOOD/LCRD via the real link-command wire
    format into transmitter.sink) with randomized return latency, so
    credit exhaustion happens constantly,
  * per-cycle randomized stall patterns on transmitter.source.ready
    (models arbiter preemption/SKP insertion/backpressure at every
    possible phase against packet handoffs),
  * interleaved TP headers (models the OUT-ACK/NRDY/ERDY flows that
    share the header buffer with data headers).

Asserts on the wire output:
  * every DPH is followed immediately by SDP + exactly data_length
    payload bytes + CRC32 + END framing,
  * the payload bytes are the per-endpoint expected stream (catches
    cross-endpoint splices and duplicated/stale words),
  * data sequence numbers per endpoint are continuous,
  * TP headers only ever appear between packets,
  * no deadlock (progress watchdog).

Env: SEED, PACKETS (per EP), MAXLEN, STALL_PCT, CREDIT_LAT, TP_EVERY
"""

import os
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(HERE / "luna"))
sys.path.insert(0, str(HERE))

from amaranth import *
from amaranth.sim import Simulator

from luna.gateware.usb.stream import USBRawSuperSpeedStream
from luna.gateware.usb.usb3.protocol.endpoint import (
    SuperSpeedEndpointMultiplexer, SuperSpeedEndpointInterface)
from luna.gateware.usb.usb3.protocol.layer import TxDataSkidBuffer
from luna.gateware.usb.usb3.link.data import DataPacketTransmitter
from luna.gateware.usb.usb3.link.header import HeaderQueue, HeaderQueueArbiter
from luna.gateware.usb.usb3.link.transmitter import PacketTransmitter

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_link_loopback import (crc16_header, crc5, crc32_payload,
                               build_link_command, LC_LGOOD, LC_LCRD,
                               W_HPSTART, W_SDP, W_END, W_IDLE)

SEED       = int(os.environ.get("SEED", 1))
PACKETS    = int(os.environ.get("PACKETS", 120))
MAXLEN     = int(os.environ.get("MAXLEN", 96))
STALL_PCT  = int(os.environ.get("STALL_PCT", 30))
CREDIT_LAT = int(os.environ.get("CREDIT_LAT", 12))
TP_EVERY   = int(os.environ.get("TP_EVERY", 3))
NUM_EPS    = 3

HP_DP, HP_TP = 8, 4


class Bench(Elaboratable):
    def __init__(self):
        self.ifaces = [SuperSpeedEndpointInterface() for _ in range(NUM_EPS)]
        self.mux = SuperSpeedEndpointMultiplexer()
        for i in self.ifaces:
            self.mux.add_interface(i)
        self.tx_skid = TxDataSkidBuffer()
        self.data_tx = DataPacketTransmitter()
        self.hp_mux = HeaderQueueArbiter()
        self.tp_queue = HeaderQueue()          # scripted TP header producer
        self.transmitter = PacketTransmitter()

    def elaborate(self, platform):
        m = Module()
        m.domains += ClockDomain("ss")
        m.submodules.mux = self.mux
        m.submodules.tx_skid = self.tx_skid
        m.submodules.data_tx = self.data_tx
        m.submodules.hp_mux = self.hp_mux
        m.submodules.transmitter = self.transmitter

        shared = self.mux.shared
        # Exactly the protocol-layer wiring (protocol/layer.py).
        m.d.comb += [
            self.tx_skid.sink            .stream_eq(shared.tx),
            self.data_tx.data_sink       .stream_eq(self.tx_skid.source),
            self.data_tx.send_zlp        .eq(shared.tx_zlp),
            self.data_tx.sequence_number .eq(shared.tx_sequence_number),
            self.data_tx.endpoint_number .eq(shared.tx_endpoint_number),
            self.data_tx.data_length     .eq(shared.tx_length),
            self.data_tx.direction       .eq(shared.tx_direction),
            self.data_tx.end_of_burst    .eq(shared.tx_eob),
            self.data_tx.address         .eq(5),
            shared.tx_parameters_consumed.eq(self.data_tx.parameters_consumed),
        ]
        # Header queue arbiter: TP producer first (as the protocol layer's
        # tp_generator is added before... order: hp_mux in protocol layer
        # adds lmp, tp_generator; link layer hp_mux adds header_sink
        # (protocol headers) THEN data_tx.header_source.  Model: tp_queue
        # then data_tx.
        self.hp_mux.add_producer(self.tp_queue)
        self.hp_mux.add_producer(self.data_tx.header_source)
        m.d.comb += [
            self.transmitter.queue       .header_eq(self.hp_mux.source),
            self.transmitter.data_sink   .stream_eq(self.data_tx.data_source),
            self.transmitter.enable      .eq(1),
        ]
        return m


def main():
    rng = random.Random(SEED)
    bench = Bench()
    sim = Simulator(bench)
    sim.add_clock(8e-9, domain="ss")

    # Per-EP packet plans: (length, payload bytes) with a per-EP LCG
    # pattern so misrouted words are attributable.
    plans = []
    for e in range(NUM_EPS):
        plan = []
        for k in range(PACKETS):
            if os.environ.get("BIG_ONLY") or rng.random() < 0.2:
                length = 1024                     # some max-size packets
            else:
                length = 4 * rng.randint(1, MAXLEN // 4)
            payload = bytes(((e + 1) * 37 + k * 11 + j) & 0xFF
                            for j in range(length))
            plan.append(payload)
        plans.append(plan)

    state = {"done": [False] * NUM_EPS, "progress": 0, "fail": None,
             "tp_sent": 0, "tp_seen": 0, "dps_seen": 0}
    exp_seq = [0] * NUM_EPS      # 5-bit data seq we assign per EP
    seen_seq = [0] * NUM_EPS     # next expected on the wire per EP
    sent_idx = [0] * NUM_EPS     # packets fully handed to the mux
    wire_idx = [0] * NUM_EPS     # packets verified on the wire

    def fail(cycle, msg):
        state["fail"] = f"cycle {cycle}: {msg}"
        for line in state.get("ring", [])[-55:]:
            print(" ", line)
        raise SystemExit(1)

    async def endpoint(ctx, e):
        """Obeys the endpoint TX contract, like SuperSpeedStreamInEndpoint:
        drives valid for the whole packet, params comb while sending."""
        iface = bench.ifaces[e]
        tx = iface.tx
        for k, payload in enumerate(plans[e]):
            # random inter-packet pause (token pacing)
            for _ in range(rng.randint(2, 14)):
                await ctx.tick("ss")
            words = [payload[i:i+4] for i in range(0, len(payload), 4)]
            ctx.set(iface.tx_length, len(payload))
            ctx.set(iface.tx_endpoint_number, e + 1)
            ctx.set(iface.tx_sequence_number, exp_seq[e])
            ctx.set(iface.tx_direction, 1)
            for wi, w in enumerate(words):
                ctx.set(tx.payload, int.from_bytes(w.ljust(4, b'\0'), "little"))
                ctx.set(tx.valid, (1 << len(w)) - 1)
                ctx.set(tx.first, 1 if wi == 0 else 0)
                ctx.set(tx.last, 1 if wi == len(words) - 1 else 0)
                await ctx.tick("ss").until(tx.ready)
            ctx.set(tx.valid, 0)
            ctx.set(tx.first, 0)
            ctx.set(tx.last, 0)
            # params drop when not sending (endpoints drive them comb
            # only in their SEND states)
            ctx.set(iface.tx_length, 0)
            ctx.set(iface.tx_endpoint_number, 0)
            ctx.set(iface.tx_sequence_number, 0)
            exp_seq[e] = (exp_seq[e] + 1) & 0x1F
            sent_idx[e] = k + 1
        state["done"][e] = True

    async def tp_producer(ctx):
        """Interleaves TP headers, as OUT-ACK/NRDY/ERDY flows do."""
        q = bench.tp_queue
        n = 0
        while not all(state["done"]) and TP_EVERY:
            for _ in range(rng.randint(4, 40)):
                await ctx.tick("ss")
            n += 1
            ctx.set(q.header.dw0, HP_TP | (5 << 25))
            ctx.set(q.header.dw1, 1 | (1 << 7) | ((n & 0xF) << 8)
                    | (1 << 16) | ((n & 0x1F) << 21))
            ctx.set(q.header.dw2, 0)
            ctx.set(q.valid, 1)
            await ctx.tick("ss").until(q.ready)
            ctx.set(q.valid, 0)
            state["tp_sent"] += 1
        state["tp_done"] = True

    async def host(ctx):
        """Consumes the wire with stall injection; returns credits with
        latency; parses and asserts framing/content."""
        src = bench.transmitter.source
        snk = bench.transmitter.sink

        # link bringup: advertisement + 4 credits
        adv = build_link_command(LC_LGOOD, 7)
        for i in range(4):
            adv += build_link_command(LC_LCRD, i)
        lc_fifo = [(0, adv)]
        lcrd_next = 0
        lgood_next = 0
        pending_credits = []

        pstate = "IDLE"
        hdr = []
        dpp = None
        cycle = 0
        hdr_count = 0

        def parse(data, ctrl):
            nonlocal pstate, hdr, dpp, hdr_count
            word = (data, ctrl)
            if pstate == "IDLE":
                if word == W_IDLE or (data == 0 and ctrl == 0):
                    return
                if word == W_HPSTART:
                    hdr = []
                    pstate = "HDR"
                    return
                fail(cycle, f"unexpected top-level word {data:08x}/{ctrl:04b}")
            elif pstate == "HDR":
                if ctrl != 0:
                    fail(cycle, f"K inside header {data:08x}")
                hdr.append(data)
                if len(hdr) == 4:
                    _check_header()
                return
            elif pstate == "SDP":
                if word != W_SDP:
                    fail(cycle, f"DPH not followed by SDP: {data:08x}/{ctrl:04b} "
                                f"(ep={dpp['ep']} dseq={dpp['dseq']} "
                                f"len={dpp['len']})")
                pstate = "DPP"
                return
            elif pstate == "DPP":
                for i in range(4):
                    b = (data >> (8*i)) & 0xFF
                    k = (ctrl >> i) & 1
                    need = dpp["len"] - len(dpp["pl"])
                    if need > 0:
                        if k:
                            fail(cycle, f"K in payload ep={dpp['ep']}")
                        dpp["pl"].append(b)
                    elif len(dpp["crc"]) < 4:
                        if k:
                            fail(cycle, f"K in CRC ep={dpp['ep']}")
                        dpp["crc"].append(b)
                    elif len(dpp["end"]) < 4:
                        exp = [(0xFD,1),(0xFD,1),(0xFD,1),(0xF7,1)][len(dpp["end"])]
                        if (b, k) != exp:
                            fail(cycle, f"bad END byte ep={dpp['ep']} "
                                        f"dseq={dpp['dseq']} len={dpp['len']} "
                                        f"got {b:02x}/{k}")
                        dpp["end"].append(b)
                    else:
                        if b or k:
                            fail(cycle, f"non-idle pad {b:02x}/{k}")
                if len(dpp["end"]) == 4:
                    _finish_dp()
                return

        def _check_header():
            nonlocal pstate, dpp, hdr_count
            dw0, dw1, dw2, dw3 = hdr
            if (dw3 & 0xFFFF) != crc16_header([dw0, dw1, dw2]):
                fail(cycle, "header CRC16 mismatch")
            hdr_count += 1
            pending_credits.append(cycle + CREDIT_LAT +
                                   rng.randint(0, CREDIT_LAT))
            ptype = dw0 & 0x1F
            if ptype == HP_DP:
                ep = (dw1 >> 8) & 0xF
                dseq = dw1 & 0x1F
                ln = (dw1 >> 16) & 0xFFFF
                dpp = {"ep": ep, "dseq": dseq, "len": ln,
                       "pl": bytearray(), "crc": bytearray(), "end": []}
                pstate = "SDP"
            elif ptype == HP_TP:
                state["tp_seen"] += 1
                pstate = "IDLE"
            else:
                fail(cycle, f"unexpected header type {ptype}")

        def _finish_dp():
            nonlocal pstate, dpp
            e = dpp["ep"] - 1
            if not (0 <= e < NUM_EPS):
                fail(cycle, f"DP for bad ep {dpp['ep']}")
            if dpp["dseq"] != seen_seq[e]:
                fail(cycle, f"ep{e+1} wire dseq {dpp['dseq']} expected "
                            f"{seen_seq[e]}")
            expect = plans[e][wire_idx[e]]
            if dpp["len"] != len(expect):
                fail(cycle, f"ep{e+1} pkt{wire_idx[e]} DPH len {dpp['len']} "
                            f"!= plan {len(expect)}")
            if bytes(dpp["pl"]) != expect:
                diff = next(i for i, (a, b) in
                            enumerate(zip(dpp["pl"], expect)) if a != b)
                fail(cycle, f"ep{e+1} pkt{wire_idx[e]} payload mismatch at "
                            f"byte {diff}: got {dpp['pl'][diff]:02x} want "
                            f"{expect[diff]:02x}")
            crc = int.from_bytes(bytes(dpp["crc"]), "little")
            if crc != crc32_payload(bytes(dpp["pl"])):
                fail(cycle, f"ep{e+1} pkt{wire_idx[e]} CRC32 mismatch")
            seen_seq[e] = (seen_seq[e] + 1) & 0x1F
            wire_idx[e] += 1
            state["dps_seen"] += 1
            state["progress"] = cycle
            pstate = "IDLE"
            dpp = None

        ring = state["ring"] = []
        stall = False
        while True:
            # stall injection on the wire consumer
            stall = rng.randrange(100) < STALL_PCT
            ctx.set(src.ready, 0 if stall else 1)

            # credit returns with latency
            if pending_credits and pending_credits[0] <= cycle:
                pending_credits.pop(0)
                words = build_link_command(LC_LGOOD, lgood_next) \
                    + build_link_command(LC_LCRD, lcrd_next)
                lgood_next = (lgood_next + 1) & 0x7
                lcrd_next = (lcrd_next + 1) & 0x3
                lc_fifo.append((cycle, words))

            if lc_fifo:
                t, words = lc_fifo[0]
                if words:
                    w = words.pop(0)
                    ctx.set(snk.valid, 1)
                    ctx.set(snk.data, w[0])
                    ctx.set(snk.ctrl, w[1])
                if not words:
                    lc_fifo.pop(0)
            else:
                ctx.set(snk.valid, 0)

            shared = bench.mux.shared
            (_, _, v, d, c,
             mx_v, mx_r, mx_l,
             sk_v, sk_r, sk_l,
             dt_fsm, dt_hv, dt_hr,
             tx_dsv, tx_dsr) = await ctx.tick("ss").sample(
                src.valid, src.data, src.ctrl,
                shared.tx.valid, shared.tx.ready, shared.tx.last,
                bench.tx_skid.source.valid, bench.tx_skid.source.ready,
                bench.tx_skid.source.last,
                bench.data_tx.debug_fsm, bench.data_tx.header_source.valid,
                bench.data_tx.header_source.ready,
                bench.transmitter.data_sink.valid,
                bench.transmitter.data_sink.ready)
            cycle += 1
            ring.append(f"{cycle:>7} mux v={mx_v:04b} r={mx_r} l={mx_l} | "
                        f"skidout v={sk_v:04b} r={sk_r} l={sk_l} | "
                        f"dtx fsm={dt_fsm} hv={dt_hv} hr={dt_hr} | "
                        f"ptx dsv={tx_dsv:04b} dsr={tx_dsr} | "
                        f"wire v={v} {d:08x}/{c:04b}")
            if len(ring) > 60:
                ring.pop(0)

            # Compression detector: across a mux-invalid gap, the skid's
            # output never went invalid while data_tx stayed in
            # SEND_PAYLOAD -- the packet boundary was annihilated.
            if not state.get("comp_found"):
                if mx_v == 0:
                    state.setdefault("gap_cycles", []).append(
                        (sk_v != 0, dt_fsm))
                else:
                    gap = state.pop("gap_cycles", [])
                    if gap and all(skv for skv, _ in gap) \
                            and all(f == 2 for _, f in gap) and dt_fsm == 2:
                        state["comp_found"] = True
                        print(f"---- COMPRESSION at cycle {cycle} "
                              f"(gap of {len(gap)} cycles) ----")
                        for line in ring:
                            print(" ", line)

            if not stall and v:
                parse(d, c)

            # Completion also requires the TP producer to have finished:
            # if the host returns while a final TP is still awaiting
            # queue-ready, nobody services the wire/credits anymore, the
            # TP never dispatches, and sim.run() never returns (the
            # buffered PASS is then lost when the hung process is killed
            # -- phantom "timeouts").
            if all(state["done"]) and state.get("tp_done", not TP_EVERY) \
                    and all(wire_idx[e] >= len(plans[e])
                            for e in range(NUM_EPS)):
                total = sum(len(p) for plan in plans for p in plan)
                print(f"TX-FUZZ PASS seed={SEED}: {state['dps_seen']} DPs "
                      f"({total} bytes), {state['tp_seen']} TPs, "
                      f"{cycle} cycles")
                return

            if cycle - state["progress"] > 20000:
                print("---- ring buffer (onset window) ----")
                for line in ring:
                    print(" ", line)
                print("---- deadlock trace ----")
                for _ in range(16):
                    ctx.set(src.ready, 1)
                    _, _, tv, td, tc = await ctx.tick("ss").sample(
                        src.valid, src.data, src.ctrl)
                    print(f"  v={tv} {td:08x}/{tc:04b} "
                          f"tosend={ctx.get(bench.transmitter.packets_to_send)} "
                          f"cred={ctx.get(bench.transmitter.credits_available)} "
                          f"dtxfsm={ctx.get(bench.data_tx.debug_fsm)} "
                          f"qv={ctx.get(bench.hp_mux.source.valid)} "
                          f"qr={ctx.get(bench.hp_mux.source.ready)}")
                diag = (f"credits={ctx.get(bench.transmitter.credits_available)} "
                        f"to_send={ctx.get(bench.transmitter.packets_to_send)} "
                        f"dtx_fsm={ctx.get(bench.data_tx.debug_fsm)} "
                        f"dtx_hdr_v={ctx.get(bench.data_tx.header_source.valid)} "
                        f"dtx_src_v={ctx.get(bench.data_tx.data_source.valid)} "
                        f"skid_src_v={ctx.get(bench.tx_skid.source.valid)} "
                        f"mux_tx_v={ctx.get(bench.mux.shared.tx.valid)} "
                        f"tpq_v={ctx.get(bench.tp_queue.valid)} "
                        f"src_v={ctx.get(src.valid)} "
                        f"hdr={[hex(h) for h in hdr]}")
                fail(cycle, f"DEADLOCK: wire_idx={wire_idx} "
                            f"sent_idx={sent_idx} tp={state['tp_seen']}"
                            f"/{state['tp_sent']} pstate={pstate} {diag}")
            if cycle > 4_000_000:
                fail(cycle, "cycle limit")

    def make_ep(e):
        async def tb(ctx):
            await endpoint(ctx, e)
        return tb
    for e in range(NUM_EPS):
        sim.add_testbench(make_ep(e))
    sim.add_testbench(tp_producer)
    sim.add_testbench(host)
    try:
        sim.run()
    except SystemExit:
        print(f"TX-FUZZ FAIL seed={SEED}: {state['fail']}")
        raise


if __name__ == "__main__":
    main()
