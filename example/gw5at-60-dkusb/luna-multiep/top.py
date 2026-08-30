"""LUNA SuperSpeed multi-endpoint bulk loopback on the gw_usb3 PHY (DK_USB).

Concurrency vehicle for the open Gen1 stack: a vendor-specific device
with THREE independent bulk OUT -> bulk IN loopback pairs (EP1..EP3),
each buffered by a 16 KiB BSRAM elastic FIFO.  Exercises the shared
protocol machinery under simultaneous multi-endpoint traffic: the
endpoint-mux TX packet lock, the kind-granular handshake arbiter, the
per-endpoint NRDY/ERDY flow control, and interleaved per-EP sequence
spaces (HANDOVER 10j).

Validated in simulation first: sim_link_loopback.py (luna-loopback/)
runs the same endpoint pairs against a raw-wire link-partner host model
with NUM_EPS=2..4, including LBAD/bad-header fault injection.

Build & program:

    python top.py                # build only (build/)
    python top.py serdes         # regenerate serdes.toml/csr only
    python top.py flash          # flash the existing bitstream
    python top.py program        # build + flash

Host test (needs pyusb):

    sudo python multiep_test.py [MiB-per-EP]

Debug UARTs as in luna-loopback: ttyUSB4 link probe, ttyUSB5 EP1
IN-ladder probe.  LED = link trained (U0).
"""

import functools
import importlib.util
import operator
import os
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXAMPLE_ROOT = HERE.parent                      # gw5at-60-dkusb/
WORKSPACE = HERE.parents[3]                     # GW_USB3/
LUNA_ROOT = WORKSPACE / "luna"

sys.path.insert(0, str(HERE))
sys.path.insert(0, str(EXAMPLE_ROOT / "luna-acm"))   # ss_stream_out
sys.path.insert(0, str(EXAMPLE_ROOT))           # dk_usb_gw5at60
sys.path.insert(0, str(WORKSPACE / "gowin-serdes"))
sys.path.insert(0, str(WORKSPACE))              # gw_usb3
sys.path.insert(0, str(LUNA_ROOT))              # luna

from amaranth.hdl import *
from amaranth.lib.cdc import FFSynchronizer
from amaranth.lib.fifo import SyncFIFOBuffered
from amaranth.lib.memory import Memory

from dk_usb_gw5at60 import DKUSBGW5AT60Platform, add_serdes_refclk_forward

from gowin_serdes import GowinDevice, make_usb3_serdes, usb3_boot_writes
from gowin_serdes.config import RefClkSource
from gowin_serdes.usb3 import attach_usb3_phy

from luna.gateware.interface.serdes_phy.gowin_gtr12 import GowinGTR12PIPE
from luna.gateware.usb.usb3.device import USBSuperSpeedDevice
from luna.gateware.usb.usb3.endpoints.stream import SuperSpeedStreamInEndpoint
from luna.gateware.usb.usb3.protocol.layer import TxDataSkidBuffer
from usb_protocol.emitters import SuperSpeedDeviceDescriptorCollection

from ss_stream_out import SuperSpeedStreamOutEndpoint

# Reuse the debug probe from the (vendor-stack) enum example.
_spec = importlib.util.spec_from_file_location(
    "usb31_enum_top", EXAMPLE_ROOT / "usb31-enum" / "top.py")
_enum_top = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_enum_top)
ClockFreqProbe = _enum_top.ClockFreqProbe
from uart import AsyncSerialTX      # usb31-enum dir (on sys.path via exec above)

# ── Configuration ─────────────────────────────────────────────────────
QUAD, LANE = 0, 1
REF_CLK_SOURCE = RefClkSource.Q0_REFCLK1
REF_CLK_FREQ = "200M"
DBG_FREQ = 24_000_000
BAUD_RATE = 115_200
BOOT_RATE = "10G"           # proven: 10G boot + adapter rate switch to 5G

# Loopback endpoint pairs (OUT 0x0n / IN 0x8n).  Two pairs are verified on
# hardware (16 MiB/EP simultaneous, 69.5 MB/s per direction each).
#
# THREE pairs: the SET_ADDRESS enumeration failure of HANDOVER 10j is
# FIXED (GowinSynthesis all-ones-sentinel constant-fold in the endpoint
# mux, HANDOVER 10k) and a 3-pair build now enumerates cleanly and moves
# 114 MB/s aggregate across all three pipes -- but SUSTAINED 3-pipe
# traffic hits a NEW intermittent failure (-71 EPROTO on two IN pipes
# within ~µs of each other, link stays in U0, ~50% of 1 MiB/EP runs;
# open item #23, HANDOVER 10k/10l).  Session-8 build under test: three
# pairs + the data_tx packet-boundary fix (bug #24).
BULK_EPS = (1, 2, 3)
FIFO_WORDS = 4096           # per-pair elastic buffer: 16 KiB

# Burst depth (packets in flight per pipe; descriptors advertise
# bMaxBurst = max(BURST_IN, BURST_OUT) - 1).  1 elaborates the
# historical single-packet engines (the session-9 shipping default).
# Session 10 closed the burst hardware bring-up (bug #34: post-EOB
# grant discipline + ERDY-on-NumP=0 in the IN engine; bug #35 in the
# OUT engine's capture gating): the full BURST=2 hardware ladder is
# green -- 1 MiB x10, 16/64 MiB x3 pipes, 64 MiB x3 x10 soak, 94 MB/s
# per direction per pipe (283 MB/s aggregate), retry_flagged=0
# throughout -- so 2 is the shipping default now.  Set MULTIEP_BURST=1
# to rebuild the historical single-packet image.
BURST     = int(os.environ.get("MULTIEP_BURST", 2))
BURST_IN  = int(os.environ.get("MULTIEP_BURST_IN", BURST))
BURST_OUT = int(os.environ.get("MULTIEP_BURST_OUT", BURST))

# Bug-#34 field probe: when set, uart0 carries the BurstEventCapture dump
# (the first 64 bulk-endpoint header events, raw DW1 words) instead of
# the link probe.  uart1 (wire checker; retry_flagged) is unaffected.
ACKPROBE  = int(os.environ.get("MULTIEP_ACKPROBE", 0))


def make_serdes():
    return make_usb3_serdes(GowinDevice.GW5AT_60, QUAD, LANE,
                            REF_CLK_SOURCE, REF_CLK_FREQ,
                            boot_rate=BOOT_RATE)


def create_descriptors():
    """Vendor-specific device: one interface, three bulk OUT/IN pairs."""
    descriptors = SuperSpeedDeviceDescriptorCollection()

    with descriptors.DeviceDescriptor() as d:
        d.bDeviceClass       = 0xFF        # vendor specific
        d.idVendor           = 0x1209      # pid.codes
        d.idProduct          = 0x0001      # test PID
        d.bcdUSB             = 3.2
        d.bMaxPacketSize0    = 9           # 2**9 = 512
        d.iManufacturer      = "LUNA + gw_usb3"
        d.iProduct           = "GTR12 SuperSpeed multi-EP loopback"
        d.iSerialNumber      = "DK60"
        d.bNumConfigurations = 1

    with descriptors.ConfigurationDescriptor() as c:
        c.bMaxPower = 50

        with c.InterfaceDescriptor() as i:
            i.bInterfaceNumber   = 0
            i.bInterfaceClass    = 0xFF    # vendor specific
            i.bInterfaceSubclass = 0x00
            i.bInterfaceProtocol = 0x00

            for ep in BULK_EPS:
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x80 | ep
                    e.bmAttributes     = 0x02  # bulk
                    e.wMaxPacketSize   = 1024
                    with e.SuperSpeedCompanion() as c:
                        c.bMaxBurst = BURST_IN - 1

                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = ep
                    e.bmAttributes     = 0x02  # bulk
                    e.wMaxPacketSize   = 1024
                    with e.SuperSpeedCompanion() as c:
                        c.bMaxBurst = BURST_OUT - 1

    return descriptors


class TxWireChecker(Elaboratable):
    """Monitors the (pre-scrambler) transmit word stream and verifies data
    packet framing on the wire: every DP header must be followed by SDP,
    exactly ``data_length`` payload bytes (only length%4==0 checked -- all
    bulk packets here are 1024B), one CRC-32 word, and END framing.  For
    aligned packets the CRC-32 itself is recomputed and compared.  Also
    counts emitted DPs per endpoint.  Open-item-#23 probe: discriminates
    "device emitted a malformed DP" (H2) from "device never emitted /
    host-side loss" (H1), and link-layer CRC splice errors from
    post-scrambler physical damage.

    Inputs are the device.debug_wire_tx_* taps.  Everything registered.
    """

    def __init__(self, eps=(1, 2, 3)):
        self._eps = eps
        self.data   = Signal(32)
        self.ctrl   = Signal(4)
        self.strobe = Signal()
        self.enable = Signal()

        self.dp_seen  = {ep: Signal(name=f"dp_seen{ep}") for ep in eps}
        self.dp_other = Signal()      # DP from an endpoint not in eps (EP0 excluded)
        self.err      = Signal()      # framing/length violation strobe
        self.crc_err  = Signal()      # DPP CRC-32 mismatch (aligned DPs)

        # Bug-#34 field capture: DW1 of every transmitted DP / transaction
        # header whose endpoint field is a bulk endpoint (EP0 excluded)
        # -- plus, once ``armed`` is high, EP0 *data* headers as well
        # (a ghost DPH with stale/reset parameters would carry ep=0 and
        # is invisible to every other counter; arming on the first bulk
        # event keeps enumeration's legitimate EP0 traffic out).
        # Strobed with the word for one cycle; ``cap_tp``=1 marks a TP
        # (subtype self-describing in dw1[0:4]), 0 a data packet header.
        self.armed    = Signal()
        self.cap_stb  = Signal()
        self.cap_tp   = Signal()
        self.cap_dw1  = Signal(32)

    def elaborate(self, platform):
        m = Module()

        W_HPSTART = 0xF7FBFBFB
        W_LCSTART = 0xF7FEFEFE
        W_SDP     = 0xF75C5C5C
        W_END     = 0xF7FDFDFD
        K1111     = 0b1111

        # Registered input stage.
        d = Signal(32)
        c = Signal(4)
        v = Signal()
        m.d.ss += [d.eq(self.data), c.eq(self.ctrl),
                   v.eq(self.strobe & self.enable)]

        dw_index  = Signal(range(5))
        is_dp     = Signal()
        is_tp     = Signal()
        dp_ep     = Signal(4)
        dp_len    = Signal(16)
        words_left = Signal(range(1024 // 4 + 2))
        aligned   = Signal()

        # Reference CRC-32 over the (aligned) payload words, using the very
        # CRC module the link layer uses.
        from luna.gateware.usb.usb3.link.crc import DataPacketPayloadCRC
        m.submodules.crc32 = crc32 = DataPacketPayloadCRC()
        crc_advance = Signal()
        crc_clear   = Signal()
        m.d.comb += [
            crc32.data_input  .eq(d),
            crc32.advance_word.eq(crc_advance),
            crc32.clear       .eq(crc_clear),
        ]

        with m.FSM(domain="ss"):
            with m.State("IDLE"):
                with m.If(v & (d == W_HPSTART) & (c == K1111)):
                    m.d.ss += dw_index.eq(0)
                    m.next = "HDR"
                with m.Elif(v & (d == W_LCSTART) & (c == K1111)):
                    m.next = "LC"

            with m.State("LC"):
                with m.If(v):
                    m.next = "IDLE"

            with m.State("HDR"):
                with m.If(v):
                    m.d.ss += dw_index.eq(dw_index + 1)
                    with m.If(dw_index == 0):
                        m.d.ss += [
                            is_dp.eq(d[0:5] == 8),   # HP_DP
                            is_tp.eq(d[0:5] == 4),   # HP_TRANSACTION
                        ]
                    with m.If(dw_index == 1):
                        m.d.ss += [
                            dp_ep .eq(d[8:12]),
                            dp_len.eq(d[16:32]),
                        ]
                        # Bug-#34 field capture: both DPH and TP dw1 carry
                        # the endpoint number at [8:12]; EP0 (enumeration)
                        # traffic is excluded -- except EP0 DPHs once the
                        # capture is armed (ghost-DPH hunt).
                        with m.If(((is_dp | is_tp) & (d[8:12] != 0))
                                  | (is_dp & self.armed)):
                            m.d.comb += [
                                self.cap_stb.eq(1),
                                self.cap_tp .eq(is_tp),
                                self.cap_dw1.eq(d),
                            ]
                    with m.If(dw_index == 3):
                        with m.If(is_dp):
                            m.next = "EXPECT_SDP"
                        with m.Else():
                            m.next = "IDLE"

            with m.State("EXPECT_SDP"):
                with m.If(v):
                    # Tally the DP by endpoint.
                    for ep in self._eps:
                        with m.If(dp_ep == ep):
                            m.d.comb += self.dp_seen[ep].eq(1)
                    if self._eps:
                        # EP0 is excluded: control-transfer IN data stages
                        # are legitimate DPs from endpoint 0 (they fire
                        # during every enumeration), and flagging them
                        # made the sticky meaningless.  ``dp_other`` now
                        # means "DP from an endpoint that cannot send
                        # DPs" -- a real mux/parameter corruption.
                        with m.If(~functools.reduce(
                                operator.or_,
                                [dp_ep == ep for ep in self._eps])
                                  & (dp_ep != 0)):
                            m.d.comb += self.dp_other.eq(1)

                    with m.If((d == W_SDP) & (c == K1111)):
                        m.d.ss += [
                            words_left.eq(dp_len[2:] + 1),   # payload + CRC
                            aligned   .eq(dp_len[0:2] == 0),
                        ]
                        m.d.comb += crc_clear.eq(1)
                        with m.If(dp_len == 0):
                            # ZLP: SDP, CRC, END.
                            m.d.ss += [words_left.eq(1), aligned.eq(1)]
                        m.next = "DPP"
                    with m.Else():
                        # A DP header not followed by its payload framing.
                        m.d.comb += self.err.eq(1)
                        m.next = "IDLE"

            with m.State("DPP"):
                with m.If(v):
                    with m.If(~aligned):
                        # Unaligned tail: not length-checked (not used by
                        # the bulk pipes); resynchronize at next END.
                        with m.If((d == W_END) & (c == K1111)):
                            m.next = "IDLE"
                    with m.Elif(words_left != 1):
                        # Payload words: feed the reference CRC.
                        m.d.ss += words_left.eq(words_left - 1)
                        m.d.comb += crc_advance.eq(1)
                        with m.If(c != 0):
                            # K symbols inside payload/CRC region: the DPP
                            # ended early (truncated vs its header).
                            m.d.comb += self.err.eq(1)
                            m.next = "IDLE"
                    with m.Elif(words_left == 1):
                        # The CRC-32 word itself: compare with reference.
                        m.d.ss += words_left.eq(0)
                        with m.If(c != 0):
                            m.d.comb += self.err.eq(1)
                            m.next = "IDLE"
                        with m.Elif(d != crc32.crc):
                            m.d.comb += self.crc_err.eq(1)
                            m.next = "RESYNC"
                        with m.Else():
                            m.next = "EXPECT_END"

            with m.State("EXPECT_END"):
                with m.If(v):
                    with m.If((d == W_END) & (c == K1111)):
                        m.next = "IDLE"
                    with m.Else():
                        m.d.comb += self.err.eq(1)
                        m.next = "IDLE"

            with m.State("RESYNC"):
                # After a CRC mismatch, skip to the END framing.
                with m.If(v & (d == W_END) & (c == K1111)):
                    m.next = "IDLE"

        return m


class BurstEventCapture(Elaboratable):
    """Bug-#34 field probe: captures the first ``depth`` bulk-endpoint
    protocol events after training, in arrival order, and dumps them
    continuously over UART.

    Event sources (``ss`` domain):
      * the TxWireChecker's header-field capture (transmitted DPH / TP
        dw1 words, straight off the pre-scrambler wire);
      * the device's RX header tap (every ACK TP / DPH the link layer
        delivered to the protocol layer, raw dw1).

    Capture is write-once: it stops when the ring is full, preserving
    the FIRST mid-burst acknowledgement exchange -- exactly where the
    BURST=2 images die on the bench.  The dump side (``dbg`` domain)
    reads entries through a request/ack handshake CDC (ClockFreqProbe
    pattern; no gray-code assumptions) and cycles forever:

        #NN F\\r\\n            -- sweep header: NN = events captured (hex),
                                 F = 1 if an event was lost to a skid
                                 collision (three same-cycle events)
        II cXXXXXXXX\\r\\n     -- entry II: c = D (TX DPH), T (TX TP),
                                 A (RX TP), R (RX DPH), . (empty);
                                 XXXXXXXX = the header's DW1, hex

    Decode: DPH dw1 = dseq[0:5] eob[6] dir[7] ep[8:12] len[16:32];
    ACK TP dw1 = subtype[0:4] rty[6] dir[7] ep[8:12] herr[15]
    nump[16:21] nseq[21:26] (subtype 1=ACK 2=NRDY 3=ERDY).
    """

    DEPTH = 64

    def __init__(self, clk_freq, baud=115_200):
        self._divisor = clk_freq // baud

        # Event port A: TX header capture (from TxWireChecker).
        self.a_stb = Signal()
        self.a_tp  = Signal()          # 1 = transaction packet, 0 = DPH
        self.a_dw1 = Signal(32)
        # Event port B: RX header capture (from the device tap).
        self.b_stb = Signal()
        self.b_tag = Signal(3)         # 3 = RX TP, 4 = RX DPH
        self.b_dw1 = Signal(32)

        self.tx_o  = Signal(init=1)

    def elaborate(self, platform):
        m = Module()
        depth = self.DEPTH

        # ── capture ring (ss domain, write-once) ─────────────────────
        m.submodules.ring = ring = Memory(shape=35, depth=depth, init=[])
        wr = ring.write_port(domain="ss")
        rd = ring.read_port(domain="ss")

        wptr = Signal(range(depth + 1))
        full = wptr == depth
        lost = Signal()

        # One-deep skid for a same-cycle A+B collision: A is written
        # first, B waits one cycle.  A third event during the skid is
        # lost (flagged); at header rates that window is negligible.
        skid_v   = Signal()
        skid_tag = Signal(3)
        skid_dw1 = Signal(32)

        a_tag = Mux(self.a_tp, 2, 1)

        m.d.comb += wr.en.eq(0)
        with m.If(~full):
            with m.If(self.a_stb):
                m.d.comb += [
                    wr.en  .eq(1),
                    wr.addr.eq(wptr),
                    wr.data.eq(Cat(self.a_dw1, a_tag)),
                ]
                m.d.ss += wptr.eq(wptr + 1)
                with m.If(self.b_stb):
                    with m.If(skid_v):
                        m.d.ss += lost.eq(1)
                    with m.Else():
                        m.d.ss += [
                            skid_v  .eq(1),
                            skid_tag.eq(self.b_tag),
                            skid_dw1.eq(self.b_dw1),
                        ]
            with m.Elif(skid_v):
                m.d.comb += [
                    wr.en  .eq(1),
                    wr.addr.eq(wptr),
                    wr.data.eq(Cat(skid_dw1, skid_tag)),
                ]
                m.d.ss += [wptr.eq(wptr + 1), skid_v.eq(0)]
                with m.If(self.b_stb):
                    m.d.ss += [
                        skid_v  .eq(1),
                        skid_tag.eq(self.b_tag),
                        skid_dw1.eq(self.b_dw1),
                    ]
            with m.Elif(self.b_stb):
                m.d.comb += [
                    wr.en  .eq(1),
                    wr.addr.eq(wptr),
                    wr.data.eq(Cat(self.b_dw1, self.b_tag)),
                ]
                m.d.ss += wptr.eq(wptr + 1)

        # ── entry readout: request/ack handshake CDC ─────────────────
        # dbg sets ``idx`` then toggles ``req``; ss latches the entry
        # into a holding register and answers with ``ack``.  The
        # multi-bit values are only sampled when their producers have
        # been stable for at least one synchronizer delay.
        idx   = Signal(range(depth))    # dbg domain
        req   = Signal()                # dbg domain toggle
        hold  = Signal(35)              # ss domain, read by dbg when stable

        req_s = Signal()
        m.submodules += FFSynchronizer(req, req_s, o_domain="ss")
        req_d = Signal()
        m.d.ss += req_d.eq(req_s)

        ack       = Signal()            # ss domain toggle
        rd_wait   = Signal()
        m.d.comb += rd.addr.eq(idx)     # stable long before req edge arrives
        with m.If(req_s != req_d):
            m.d.ss += rd_wait.eq(1)     # address settled; data next cycle
        with m.Elif(rd_wait):
            m.d.ss += [
                hold   .eq(rd.data),
                rd_wait.eq(0),
                ack    .eq(~ack),
            ]

        ack_s = Signal()
        m.submodules += FFSynchronizer(ack, ack_s, o_domain="dbg")
        ack_d = Signal()
        m.d.dbg += ack_d.eq(ack_s)
        entry_ready = ack_s != ack_d    # hold stable >= 2 dbg cycles ago

        # Status snapshot (quasi-static once capture stops; a torn read
        # during the microseconds of active capture is acceptable).
        wcount_s = Signal.like(wptr)
        lost_s   = Signal()
        m.submodules += FFSynchronizer(wptr, wcount_s, o_domain="dbg")
        m.submodules += FFSynchronizer(lost, lost_s, o_domain="dbg")

        # ── UART line formatter (dbg domain) ─────────────────────────
        tx = AsyncSerialTX(divisor=self._divisor)
        m.submodules.tx = DomainRenamer("dbg")(tx)
        m.d.comb += self.tx_o.eq(tx.o)

        def hexchar(nib):
            return Mux(nib < 10, ord("0") + nib, ord("a") - 10 + nib)

        tag  = hold[32:35]
        word = hold[0:32]
        tag_char = Signal(8)
        with m.Switch(tag):
            with m.Case(1):
                m.d.comb += tag_char.eq(ord("D"))
            with m.Case(2):
                m.d.comb += tag_char.eq(ord("T"))
            with m.Case(3):
                m.d.comb += tag_char.eq(ord("A"))
            with m.Case(4):
                m.d.comb += tag_char.eq(ord("R"))
            with m.Default():
                m.d.comb += tag_char.eq(ord("."))

        # Entry line: II cXXXXXXXX\r\n (14 chars).
        ENTRY_LEN = 14
        entry_char = Signal(8)
        cpos = Signal(range(ENTRY_LEN))
        with m.Switch(cpos):
            with m.Case(0):
                m.d.comb += entry_char.eq(hexchar(Cat(idx, C(0, 2))[4:8]))
            with m.Case(1):
                m.d.comb += entry_char.eq(hexchar(idx[0:4]))
            with m.Case(2):
                m.d.comb += entry_char.eq(ord(" "))
            with m.Case(3):
                m.d.comb += entry_char.eq(tag_char)
            for j in range(8):
                with m.Case(4 + j):
                    m.d.comb += entry_char.eq(
                        hexchar(word[(7 - j) * 4:(8 - j) * 4]))
            with m.Case(ENTRY_LEN - 2):
                m.d.comb += entry_char.eq(ord("\r"))
            with m.Case(ENTRY_LEN - 1):
                m.d.comb += entry_char.eq(ord("\n"))

        # Status line: #NN F\r\n (7 chars).
        STATUS_LEN = 7
        status_char = Signal(8)
        wcount8 = Signal(8)
        m.d.comb += wcount8.eq(wcount_s)
        with m.Switch(cpos):
            with m.Case(0):
                m.d.comb += status_char.eq(ord("#"))
            with m.Case(1):
                m.d.comb += status_char.eq(hexchar(wcount8[4:8]))
            with m.Case(2):
                m.d.comb += status_char.eq(hexchar(wcount8[0:4]))
            with m.Case(3):
                m.d.comb += status_char.eq(ord(" "))
            with m.Case(4):
                m.d.comb += status_char.eq(ord("0") + lost_s)
            with m.Case(5):
                m.d.comb += status_char.eq(ord("\r"))
            with m.Case(6):
                m.d.comb += status_char.eq(ord("\n"))

        with m.FSM(domain="dbg"):

            with m.State("STATUS"):
                m.d.comb += [tx.data.eq(status_char), tx.ack.eq(1)]
                with m.If(tx.rdy):
                    with m.If(cpos == STATUS_LEN - 1):
                        m.d.dbg += [cpos.eq(0), idx.eq(0), req.eq(~req)]
                        m.next = "FETCH"
                    with m.Else():
                        m.d.dbg += cpos.eq(cpos + 1)

            with m.State("FETCH"):
                with m.If(entry_ready):
                    m.next = "ENTRY"

            with m.State("ENTRY"):
                m.d.comb += [tx.data.eq(entry_char), tx.ack.eq(1)]
                with m.If(tx.rdy):
                    with m.If(cpos == ENTRY_LEN - 1):
                        m.d.dbg += cpos.eq(0)
                        with m.If(idx == depth - 1):
                            m.next = "STATUS"
                        with m.Else():
                            m.d.dbg += [idx.eq(idx + 1), req.eq(~req)]
                            m.next = "FETCH"
                    with m.Else():
                        m.d.dbg += cpos.eq(cpos + 1)

        return m


class LunaMultiEpTop(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        led = platform.request("led", 0)
        usb_pwr_en = platform.request("usb_pwr_en", 0)
        m.d.comb += usb_pwr_en.o.eq(0)     # device mode: never source VBUS

        # Quiet the USB2 pins (floating pins = phantom LS device).
        usb2 = platform.request("usb2_softphy", 0)
        m.d.comb += [
            usb2.pullup_en.o.eq(0),
            usb2.term_dp.o.eq(0),
            usb2.term_dn.o.eq(0),
            usb2.tx_dp.o.eq(0),
            usb2.tx_dn.o.eq(0),
            usb2.dp.oe.eq(0),
            usb2.dp_b.oe.eq(0),
            usb2.dn.oe.eq(0),
            usb2.dn_b.oe.eq(0),
        ]

        # ==============================================================
        # Clocks & power-on reset (vendor-proven ordering)
        # ==============================================================
        sys_clk = add_serdes_refclk_forward(m, platform)
        m.domains += ClockDomain("cfg", reset_less=True)
        m.d.comb += ClockSignal("cfg").eq(sys_clk)

        clk24 = platform.request("clk_24m", 0)
        m.domains += ClockDomain("dbg", reset_less=True)
        m.d.comb += ClockSignal("dbg").eq(clk24.i)

        por_cnt = Signal(18)
        por_n = Signal()
        luna_go = Signal()
        with m.If(~por_cnt.all()):
            m.d.cfg += por_cnt.eq(por_cnt + 1)

        # Board key = full POR replay (LTSSM restart without reflash).
        key = platform.request("key", 0)
        key_s = Signal(2)
        m.d.cfg += key_s.eq(Cat(key.i, key_s[0]))
        with m.If(key_s[1]):
            m.d.cfg += por_cnt.eq(0)

        m.d.cfg += [
            # (threshold nudged 66_000 -> 66_002: semantically null, but a
            # content change reshuffles the deterministic PnR placement --
            # the previous roll came in at pclk Fmax 123.6 < the 125 MHz
            # operating gate.)
            por_n.eq(por_cnt > 66_009),
            luna_go.eq(por_cnt.all()),
        ]

        # ==============================================================
        # SerDes + PHY + PIPE adapter
        # ==============================================================
        serdes, group = make_serdes()
        m.submodules.serdes = serdes
        lane = group.lanes[0]
        drp = getattr(serdes, group.drp_name)
        m.d.comb += serdes.por_n.eq(por_n)

        adapter = GowinGTR12PIPE(boot_rate_switch=(BOOT_RATE == "10G"),
                                 boot_domain="ss_raw")
        m.submodules.adapter = adapter
        attach_usb3_phy(m, adapter.phy, lane, drp)

        m.domains += ClockDomain("ss_raw", reset_less=True)
        m.d.comb += ClockSignal("ss_raw").eq(lane.tx.pcs_clkout)
        # LUNA's ss/sync reset is released only once the adapter reports
        # the boot rate switch COMPLETE (phy_ready): no LUNA state ever
        # clocks at the 156.25 MHz boot rate or through the rate-change
        # pclk retune -- the boot window corrupted idle-but-clocking LUNA
        # registers (dead TP generator path at 3 endpoint pairs; HANDOVER
        # 10k).  The adapter sequences the bring-up itself in the
        # reset-free ss_raw domain, started by the POR chain.
        m.d.comb += adapter.boot_start.eq(luna_go)
        rstn_r0 = Signal()
        rstn_r1 = Signal()
        m.d.ss_raw += [rstn_r0.eq(luna_go & adapter.phy_ready),
                       rstn_r1.eq(rstn_r0)]
        for dom in ("ss", "sync"):
            m.domains += ClockDomain(dom)
            m.d.comb += [
                ClockSignal(dom).eq(ClockSignal("ss_raw")),
                ResetSignal(dom).eq(~rstn_r1),
            ]

        # ==============================================================
        # LUNA SuperSpeed device: three bulk loopback pairs
        # ==============================================================
        m.submodules.usb = usb = USBSuperSpeedDevice(
            phy=adapter, sync_frequency=125e6)

        control_ep = usb.add_standard_control_endpoint(create_descriptors())

        in_eps = {}
        out_eps = {}
        for ep in BULK_EPS:
            out_ep = SuperSpeedStreamOutEndpoint(
                endpoint_number=ep, max_packet_size=1024, max_burst=BURST_OUT)
            usb.add_endpoint(out_ep)
            out_eps[ep] = out_ep

            in_ep = SuperSpeedStreamInEndpoint(
                endpoint_number=ep, max_packet_size=1024,
                generate_zlps=False, max_burst=BURST_IN)
            usb.add_endpoint(in_ep)
            in_eps[ep] = in_ep

            # Per-pair elastic loopback buffer (see luna-loopback/top.py for
            # the rationale; 16 KiB each keeps three pairs affordable).
            fifo = SyncFIFOBuffered(width=32 + 4 + 1, depth=FIFO_WORDS)
            m.submodules[f"loop_fifo{ep}"] = DomainRenamer({"sync": "ss"})(fifo)

            out_ep.packet_space = Signal(name=f"packet_space{ep}")
            m.d.comb += [
                # OUT endpoint -> FIFO
                fifo.w_data.eq(Cat(out_ep.stream.payload,
                                   out_ep.stream.valid,
                                   out_ep.stream.last)),
                fifo.w_en.eq(out_ep.stream.valid.any() & fifo.w_rdy),
                out_ep.stream.ready.eq(fifo.w_rdy),

                # FIFO -> IN endpoint
                in_ep.stream.payload.eq(fifo.r_data[0:32]),
                in_ep.stream.valid.eq(Mux(fifo.r_rdy, fifo.r_data[32:36], 0)),
                in_ep.stream.last.eq(fifo.r_data[36]),
                fifo.r_en.eq(fifo.r_rdy & in_ep.stream.ready),

            ]
            # Accept a data packet only when a whole max-size packet fits.
            # Registered: the wide level comparison otherwise sits in the
            # endpoint's accept/handshake cone (the margin absorbs staleness).
            m.d.ss += out_ep.packet_space.eq((FIFO_WORDS - fifo.level) >= 260)

        m.d.comb += led.o.eq(usb.link_trained)

        # ==============================================================
        # Debug UARTs (identical layout to luna-loopback)
        # ==============================================================
        rx_com = Signal()
        m.d.comb += [
            rx_com.eq(adapter.rx_datavalid & adapter.rx_datak[0]
                      & (adapter.rx_data[0:8] == 0xBC)),
        ]

        # uart 0: link-event probe -- or, in an ACKPROBE build, the
        # bug-#34 header-field capture dump (see BurstEventCapture).
        uart0 = platform.request("uart", 0)
        if not ACKPROBE:
            m.submodules.linkprobe = linkprobe = DomainRenamer({"cfg": "dbg"})(
                ClockFreqProbe(clk_freq=DBG_FREQ, baud=BAUD_RATE, channels=(
                    ("ss", None),
                    ("ss", usb.debug_lfps_polling_detected),
                    ("ss", usb.debug_ts1_detected),
                    ("ss", usb.debug_ts2_detected),
                    ("ss", rx_com),
                )))
            link_flags = Signal(4)
            m.submodules += FFSynchronizer(
                Cat(usb.link_trained, usb.link_in_reset,
                    usb.debug_phy_ready, usb.debug_engage_terminations),
                link_flags, o_domain="dbg")
            m.d.comb += [
                linkprobe.flags.eq(link_flags),
                uart0.tx.o.eq(linkprobe.tx_o),
            ]
        rx0_unused = Signal()
        m.submodules += FFSynchronizer(uart0.rx.i, rx0_unused,
                                       o_domain="dbg")

        # uart 1: TX wire checker + RX ACK probe (open item #23, H1-vs-H2
        # discriminator).
        #   ch0: wire-framing violations (malformed DP on the wire);
        #   ch1/ch2/ch3: DPs emitted on the wire per endpoint 1/2/3;
        #   ch4: host ACK TPs received (broadcast strobe, all endpoints).
        # flags: bit0 = sticky wire-framing violation (H2 confirmed);
        #        bit1 = sticky payload underrun (transmitter consumed an
        #               invalid payload word -- the historical suspect #1);
        #        bit2 = sticky DP emitted with an endpoint number outside
        #               the configured set (misaddressed DPH);
        #        bit3 = link trained.
        m.submodules.txchk = txchk = TxWireChecker(eps=tuple(BULK_EPS))
        ack_rx = Signal()
        m.d.comb += [
            txchk.data  .eq(usb.debug_wire_tx_data),
            txchk.ctrl  .eq(usb.debug_wire_tx_ctrl),
            txchk.strobe.eq(usb.debug_wire_tx_strobe),
            txchk.enable.eq(usb.link_trained),
        ]
        m.d.ss += ack_rx.eq(usb.debug_ack_received)

        # Resend-cause mix across all IN endpoints (probe 9): host-flagged
        # retries (rty=1: it wants a damaged/missing DP again) vs stale
        # acknowledgements (rty=0 but non-advancing: duplicates/replays).
        retry_flagged = Signal()
        stale_ack = Signal()
        rx_dpp_invalid = Signal()
        rx_hdr_bad = Signal()
        m.d.ss += [
            retry_flagged.eq(functools.reduce(operator.or_,
                [in_eps[ep].debug_retry_flagged for ep in BULK_EPS])),
            stale_ack.eq(functools.reduce(operator.or_,
                [in_eps[ep].debug_stale_ack for ep in BULK_EPS])),
            rx_dpp_invalid.eq(usb.debug_rx_dpp_invalid),
            rx_hdr_bad.eq(usb.debug_rx_hdr_bad),
        ]

        sticky_bits = Signal(3)
        with m.If(txchk.err):
            m.d.ss += sticky_bits[0].eq(1)
        with m.If(usb.debug_payload_underrun):
            m.d.ss += sticky_bits[1].eq(1)
        with m.If(txchk.crc_err):
            m.d.ss += sticky_bits[2].eq(1)
        sticky_src = Cat(sticky_bits, usb.link_trained)

        uart1 = platform.request("uart", 1)
        _chan_eps = list(BULK_EPS)[:3] + [BULK_EPS[0]] * (3 - len(BULK_EPS))
        m.submodules.clkprobe = clkprobe = DomainRenamer({"cfg": "dbg"})(
            ClockFreqProbe(clk_freq=DBG_FREQ, baud=BAUD_RATE, channels=(
                ("ss", retry_flagged),
                ("ss", rx_dpp_invalid),
                ("ss", rx_hdr_bad),
                ("ss", txchk.dp_seen[_chan_eps[0]]),
                ("ss", ack_rx),
            )))
        dbg_flags = Signal(4)
        m.submodules += FFSynchronizer(sticky_src, dbg_flags, o_domain="dbg")
        m.d.comb += [
            clkprobe.flags.eq(dbg_flags),
            uart1.tx.o.eq(clkprobe.tx_o),
        ]
        rx1_unused = Signal()
        m.submodules += FFSynchronizer(uart1.rx.i, rx1_unused,
                                       o_domain="dbg")

        # Bug-#34 field probe (uart0, ACKPROBE builds only): first 64
        # bulk-EP header events -- TX DPH/TP dw1 straight off the wire,
        # RX TP/DPH dw1 as accepted by the protocol layer.
        if ACKPROBE:
            m.submodules.evcap = evcap = BurstEventCapture(
                clk_freq=DBG_FREQ, baud=BAUD_RATE)
            rx_is_tp = usb.debug_rx_hdr_type == 4
            rx_is_dp = usb.debug_rx_hdr_type == 8
            m.d.comb += [
                evcap.a_stb.eq(txchk.cap_stb),
                evcap.a_tp .eq(txchk.cap_tp),
                evcap.a_dw1.eq(txchk.cap_dw1),

                evcap.b_stb.eq(usb.debug_rx_hdr_stb
                               & (rx_is_tp | rx_is_dp)
                               & (usb.debug_rx_hdr_dw1[8:12] != 0)),
                evcap.b_tag.eq(Mux(rx_is_tp, 3, 4)),
                evcap.b_dw1.eq(usb.debug_rx_hdr_dw1),

                uart0.tx.o.eq(evcap.tx_o),
            ]
            # Ghost-DPH hunt: arm the EP0-DPH capture from the first bulk
            # event onward (keeps enumeration's legitimate EP0 data
            # stages out of the ring).
            cap_armed = Signal()
            with m.If(evcap.a_stb | evcap.b_stb):
                m.d.ss += cap_armed.eq(1)
            m.d.comb += txchk.armed.eq(cap_armed)

        return m


# ======================================================================
# Build entry point
# ======================================================================

def generate_serdes_files():
    serdes, _ = make_serdes()
    toml_path = HERE / "serdes.toml"
    csr_path = HERE / "serdes.csr"
    serdes.generate_csr(output_path=str(csr_path), toml_path=str(toml_path),
                        extra_writes=usb3_boot_writes(QUAD, LANE))
    print(f"Generated {toml_path.name} / {csr_path.name} ({BOOT_RATE} boot)")


def _setup_gowin_env(platform):
    import os
    os.environ.setdefault("LD_PRELOAD",
                          "/usr/lib/x86_64-linux-gnu/libfreetype.so.6")
    os.environ.setdefault("LD_LIBRARY_PATH",
                          str(Path(platform.gowin_path) / "IDE" / "lib"))
    gowin_bin = str(Path(platform.gowin_path) / "IDE" / "bin")
    if gowin_bin not in os.environ.get("PATH", ""):
        os.environ["PATH"] = gowin_bin + os.pathsep + os.environ["PATH"]


def build(do_program=False):
    generate_serdes_files()
    platform = DKUSBGW5AT60Platform()
    _setup_gowin_env(platform)
    platform.add_file("serdes.csr", (HERE / "serdes.csr").read_text())
    platform.build(LunaMultiEpTop(), name="luna_multiep", build_dir="build",
                   do_program=do_program)


def flash():
    bitstream = HERE / "build" / "luna_multiep.fs"
    if not bitstream.exists():
        sys.exit(f"no bitstream at {bitstream}; run `python top.py` first")
    cmd = ["openFPGALoader", "-c", "ft232", str(bitstream)]
    try:
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError:
        subprocess.check_call(["sudo", "-n"] + cmd)


if __name__ == "__main__":
    if "serdes" in sys.argv:
        generate_serdes_files()
    elif "flash" in sys.argv:
        flash()
    else:
        build(do_program="program" in sys.argv)
