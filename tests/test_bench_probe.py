# Copyright (c) 2026 key2
# SPDX-License-Identifier: BSD-3-Clause

"""Wire-level UART equivalence and domain isolation for ClockFreqProbe."""

import re

import pytest
from amaranth import ClockDomain, Module, Signal
from amaranth.back import rtlil
from amaranth.hdl import Fragment
from amaranth.sim import Simulator

from gowin_serdes.bench import ClockFreqProbe


class _UARTReceiver:
    """Sample 8N1 bit centers, including start/stop validation, without resync."""

    def __init__(self, divisor):
        self.divisor = divisor
        self.data = bytearray()
        self.starts = []
        self.bit = None
        self.sample_at = 0
        self.value = 0

    def sample(self, cycle, level):
        if self.bit is None:
            if level == 0:
                self.starts.append(cycle)
                self.sample_at = cycle + self.divisor // 2
                self.bit = -1
                self.value = 0
        elif cycle == self.sample_at:
            if self.bit == -1:
                assert level == 0, f"bad UART start at cycle {cycle}"
            elif self.bit < 8:
                self.value |= level << self.bit
            else:
                assert level == 1, f"bad UART stop at cycle {cycle}"
                self.data.append(self.value)
                self.bit = None
                return
            self.bit += 1
            self.sample_at += self.divisor


@pytest.mark.parametrize("channel_count,gate_bits", [(1, 10), (5, 11), (29, 14)])
def test_compact_matches_legacy_uart(channel_count, gate_bits):
    divisor = 4
    gate_cycles = 1 << gate_bits
    line_bytes = 1 + channel_count * 8 + 4
    byte_cycles = 10 * divisor + 1
    flag_script = (0, 9, 10, 15)
    assert line_bytes * byte_cycles + 16 < gate_cycles

    m = Module()
    m.domains += [ClockDomain("cfg"), ClockDomain("src_a"), ClockDomain("src_b")]
    phases = {domain: Signal(6, name=f"phase_{domain}")
              for domain in ("src_a", "src_b")}
    for domain, phase in phases.items():
        m.d[domain] += phase.eq(phase + 1)
    pulse_width = Signal(4, init=1)
    flags = Signal(4)
    channels = []
    for ci in range(channel_count):
        domain = "src_a" if ci % 2 == 0 else "src_b"
        if ci % 5 == 4:
            event = None  # Mix free-running clock counters with event counters.
        else:
            event = Signal(name=f"pulse_{ci}")
            m.d.comb += event.eq(
                ((phases[domain] + 3 * ci) & 63) < (pulse_width + ci))
        channels.append((domain, event))

    # Exercise the omitted default as well as explicit compact=False.
    legacy_options = {} if channel_count == 1 else {"compact": False}
    legacy = ClockFreqProbe(1_000_000, baud=250_000, gate_bits=gate_bits,
                            channels=channels, **legacy_options)
    compact = ClockFreqProbe(1_000_000, baud=250_000, gate_bits=gate_bits,
                             channels=channels, compact=True)
    m.submodules.legacy = legacy
    m.submodules.compact = compact
    m.d.comb += [legacy.flags.eq(flags), compact.flags.eq(flags)]

    receivers = [_UARTReceiver(divisor), _UARTReceiver(divisor)]
    probes = [legacy, compact]
    sim = Simulator(m)
    sim.add_clock(1e-6, domain="cfg")
    # Both DUTs see the same independently phased, non-reference clocks.
    sim.add_clock(0.7e-6, phase=0.11e-6, domain="src_a")
    sim.add_clock(1.3e-6, phase=0.37e-6, domain="src_b")

    async def bench(ctx):
        assert [ctx.get(probe.tx_o) for probe in probes] == [1, 1]
        for cycle in range(len(flag_script) * gate_cycles):
            window, offset = divmod(cycle, gate_cycles)
            if offset == 0:
                ctx.set(flags, flag_script[window])
                ctx.set(pulse_width, 1 + 4 * window)
            elif offset == 16:
                # Change flags while the line is still sending, after latching.
                ctx.set(flags, flag_script[window] ^ 15)
            await ctx.tick("cfg")
            levels = [ctx.get(probe.tx_o) for probe in probes]
            assert levels[0] == levels[1], f"wire mismatch at cfg cycle {cycle}"
            for receiver, level in zip(receivers, levels):
                receiver.sample(cycle, level)
        assert levels == [1, 1]

    sim.add_testbench(bench)
    sim.run()

    # Check the entire stream, including startup and the idle tail. No filtering
    # on 'C', newline splitting, ignored decode errors, or partial-frame drops.
    for receiver in receivers:
        assert receiver.bit is None
        assert len(receiver.data) == len(flag_script) * line_bytes
        assert len(receiver.starts) == len(receiver.data)
        assert receiver.starts == [
            receiver.starts[0] + frame * gate_cycles + byte * byte_cycles
            for frame in range(len(flag_script)) for byte in range(line_bytes)
        ]
    assert receivers[0].starts == receivers[1].starts
    assert receivers[0].data == receivers[1].data

    frames = [bytes(receivers[0].data[start:start + line_bytes])
              for start in range(0, len(receivers[0].data), line_bytes)]
    values = []
    pattern = rb"C(?: [0-9a-f]{7}){%d} [0-9a-f]\r\n" % channel_count
    for frame, expected_flags in zip(frames, flag_script):
        assert re.fullmatch(pattern, frame), repr(frame)
        assert frame[-4:] == f" {expected_flags:x}\r\n".encode("ascii")
        values.append([int(frame[2 + 8 * ci:9 + 8 * ci], 16)
                       for ci in range(channel_count)])
    for ci, (_, event) in enumerate(channels):
        counts = [frame[ci] for frame in values]
        if event is None:
            assert counts[0] < min(counts[1:])
            assert max(counts[1:]) - min(counts[1:]) <= 1
        else:
            assert all(before < after for before, after in zip(counts, counts[1:]))
    print(f"{channel_count} channels: {len(frames)} frames, {line_bytes} bytes/frame, "
          f"{len(receivers[0].data)} identical wire bytes per stream; flags=0,9,a,f")


def _fragments(fragment):
    yield fragment
    for child, _, _ in fragment.subfragments:
        yield from _fragments(child)


@pytest.mark.parametrize("channel_count", [1, 5, 29])
def test_compact_preserves_inputs_counters_and_cdc(channel_count):
    events = [Signal(name=f"pulse_{ci}") for ci in range(channel_count)]
    channels = [("src_a" if ci % 2 == 0 else "src_b",
                 event if ci % 5 != 4 else None)
                for ci, event in enumerate(events)]
    probes = [ClockFreqProbe(1_000_000, baud=250_000, gate_bits=14,
                             channels=channels, compact=compact)
              for compact in (False, True)]
    legacy, compact = [Fragment.get(probe, None) for probe in probes]
    assert legacy.statements.keys() == compact.statements.keys()
    for domain in legacy.statements:
        if domain not in ("comb", "cfg"):
            assert repr(legacy.statements[domain]) == repr(compact.statements[domain])

    # The only added cfg-domain sequential statement writes the word latch.
    added = [statement for statement in compact.statements["cfg"]
             if any(signal.name == "selected_delta"
                    for signal in statement._lhs_signals())]
    assert len(added) == 1
    assert [(signal.name, len(signal)) for signal in added[0]._lhs_signals()] == [
        ("selected_delta", 28)
    ]
    assert [repr(statement) for statement in legacy.statements["cfg"]] == [
        repr(statement) for statement in compact.statements["cfg"]
        if statement is not added[0]
    ]

    # Include each complete synchronizer (both directions), and the unchanged TX.
    assert [(name, len(list(_fragments(child))))
            for child, name, _ in legacy.subfragments] == [
        (name, len(list(_fragments(child)))) for child, name, _ in compact.subfragments
    ]
    assert [{domain: repr(statements) for domain, statements in child.statements.items()}
            for child in list(_fragments(legacy))[1:]] == [
        {domain: repr(statements) for domain, statements in child.statements.items()}
        for child in list(_fragments(compact))[1:]
    ]
    for probe, fragment in zip(probes, (legacy, compact)):
        driven = {id(signal) for child in _fragments(fragment)
                  for statements in child.statements.values()
                  for statement in statements for signal in statement._lhs_signals()}
        assert all(id(signal) not in driven for signal in [probe.flags, *events])


def test_default_formatter_elaborates_as_explicit_legacy():
    default = ClockFreqProbe(24_000_000)
    legacy = ClockFreqProbe(24_000_000, compact=False)
    assert default._compact is False
    assert rtlil.convert(default, ports=[default.flags, default.tx_o], emit_src=False) == (
        rtlil.convert(legacy, ports=[legacy.flags, legacy.tx_o], emit_src=False))
