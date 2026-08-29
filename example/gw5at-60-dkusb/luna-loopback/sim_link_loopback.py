#!/usr/bin/env python3
"""Link-partner loopback simulation: the full USB3 link + protocol stack
against a host model that speaks the raw wire dialect.

This is the layer below ``sim_loopback.py`` (endpoint-level host model):
here the DUT is ``USB3LinkLayer`` + ``USB3ProtocolLayer`` + the endpoint
multiplexer + the real bulk endpoint pair -- i.e. ``USBSuperSpeedDevice``
minus the physical/PIPE layer.  The physical layer is replaced by a fake
boundary object of raw 32-bit streams, and the device TX additionally
runs through the *real* ``CTCSkipInserter`` (as in
``physical/layer.py``), so SKP scheduling behaves exactly as on
hardware.

The host model:

  * scripts the LTSSM through Polling (LFPS handshake, TS1/TS2 feed,
    idle handshake) to U0;
  * performs the link bringup handshake (LGOOD_7 advertisement + 4x
    LCRD), then LMP port capability/configuration;
  * generates host->device wire traffic with correct CRC-16/CRC-5/CRC-32
    (ported bit-exactly from the gateware equations): OUT DPH+DPP, ACK
    TPs (IN tokens), LGOOD/LCRD credit returns for every device header;
  * parses the device's TX wire stream word-by-word and *asserts*:
      - DPH/DPP atomicity (nothing between a DPH and its DPP; nothing
        inside SDP..END but payload/CRC),
      - DPP length == the header's data_length, CRC-32 integrity,
      - header CRC-16/CRC-5 and 3-bit sequence continuity,
      - link-command well-formedness (dup halves + CRC-5),
      - LGOOD/LCRD ordering, 5-bit DP sequence continuity,
      - SKP ordered sets appear at least every MAX_SKP_GAP wire bytes
        (elastic-buffer requirement; the >2-packet hardware failure),
      - payload bytes echo the OUT stream byte-exactly;
  * runs the bulk OUT and bulk IN pipes concurrently with configurable
    aggressiveness, reproducing the >2-packets-in-flight concurrency of
    the hardware failure.

Run:
    pdm run python gowin-serdes/example/gw5at-60-dkusb/luna-loopback/sim_link_loopback.py

Env knobs:
    LOOPBACK_BYTES=65536   total bytes echoed per endpoint (default 16384)
    NUM_EPS=n              number of OUT->IN loopback endpoint pairs
                           (EP1..EPn, default 1); traffic runs on all of
                           them simultaneously
    LOOPBACK_SEED=1        payload PRNG seed
    HOST_GAP=0             idle ticks host inserts between its frames
    MAX_SKP_GAP=1800       wire bytes allowed between SKP ordered sets
    NO_SKP_CHECK=1         disable the SKP-interval assertion
    LBAD_EVERY=n           host LBADs every n-th device header (link-level
                           retry: LRTY + delayed resends + EDB-aborted DPP
                           + protocol-level endpoint retransmission)
    BADHDR_EVERY=n         host corrupts the CRC of every n-th of its own
                           headers (device-side LBAD/ignore/LRTY path)
    HOST_BUBBLES=n         1-cycle valid gap every n-th host word (models
                           the RX bubbles left by the SKP remover)
    WINDOW_KIB=n           mimic window_test.py: write n KiB before the
                           first IN token is granted, then keep at most
                           n KiB in flight (>2 backs up the device's
                           2 KiB of echo buffering -- the hardware
                           failure mode)
    ITP_EVERY=n            host sends an Isochronous Timestamp Packet
                           every n cycles once trained (a real xHC sends
                           one every 125 us = 15625 cycles; they consume
                           header sequence numbers and credits and
                           interleave with everything, incl. the
                           SETUP/STATUS exchange)
    HOST_LDN_EVERY=n       host interleaves an LDN keepalive link command
                           every n cycles (real links are never silent)
    CTRL_GAP=n             cycles the host waits between parsing the
                           SETUP ack and sending the STATUS TP (scans the
                           SETUP->STATUS hardware phase; default 0 = the
                           historical immediate turnaround)
    CTRL_PRE_GAP=n         cycles the host waits after link bringup
                           before sending the SETUP (scans the phase
                           against ITP/keepalive traffic)

  xHC-shaped stimulus (HANDOVER 10k open item #23; all default off --
  with every knob at 0 the host behaves exactly as before):

    URB_PACKETS=n          model the synchronous-pyusb URB rhythm of
                           multiep_test.py: per IN pipe, after every n
                           received DPs the closing ACK carries NumP=0
                           (the xHC has no buffer left -- it CANNOT grant
                           credit), then after URB_GAP(+jitter) cycles the
                           next URB's token repeats the same nseq with
                           NumP=1.  Per OUT pipe, a same-sized gap is
                           inserted after every n acknowledged DPs.
                           The historical host (URB_PACKETS=0) never sends
                           NumP=0 and re-tokens instantly -- a stimulus
                           class the hardware sees at EVERY 16 KiB URB
                           boundary.
    URB_GAP=n              base inter-URB gap in cycles (default 2500,
                           matching the observed 20-60 us resubmission
                           latency of the bench host)
    URB_JITTER=n           extra random 0..n cycles per URB gap
    HOST_LATENCY=n         cycles between parsing a device event and the
                           corresponding host response frame becoming
                           eligible to send (wire + xHC latency; the
                           historical host answers in the same cycle)
    HOST_JITTER=n          extra random 0..n cycles per response frame
    LC_LATENCY=n           same, for link-command responses (LGOOD/LCRD
                           credit returns -- nonzero values make credit
                           exhaustion and deep header queues reachable,
                           as on hardware)
    LC_JITTER=n            extra random 0..n cycles per link-command burst
    PIPE_PHASE=n           random initial 0..n cycle offset per pipe
                           before its first token/OUT (thread start skew)
    REORDER=1              allow eligible response frames of different
                           pipes to be sent in randomized order (per-pipe
                           order is always preserved)
    RETRY_TIMEOUT=n        xHC no-response behavior: if a granted IN token
                           has produced no DP within n cycles, the host
                           re-sends the ACK with the retry bit set (same
                           nseq); after 3 fruitless retries the pipe is
                           declared dead (the hardware -71 EPROTO).  The
                           historical host waits forever, so device-side
                           token-strobe loss is invisible to it.  Late
                           and duplicate DPs crossing a retry are
                           tolerated like an xHC tolerates them.
    TRACE_LO/TRACE_HI      cycle window: print every parsed TX word
    WRITE_VCD=1            dump /tmp/kilo/link_loopback.vcd
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

from luna.gateware.usb.stream import USBRawSuperSpeedStream
from luna.gateware.usb.usb3.link.layer import USB3LinkLayer
from luna.gateware.usb.usb3.physical.ctc import CTCSkipInserter, TxStreamSkidBuffer
from luna.gateware.usb.usb3.physical.scrambling import Scrambler, Descrambler
from luna.gateware.usb.usb3.endpoints.control import USB3ControlEndpoint
from usb_protocol.emitters import SuperSpeedDeviceDescriptorCollection
from luna.gateware.usb.usb3.protocol.layer import USB3ProtocolLayer
from luna.gateware.usb.usb3.protocol.endpoint import SuperSpeedEndpointMultiplexer
from luna.gateware.usb.usb3.endpoints.stream import SuperSpeedStreamInEndpoint

from ss_stream_out import SuperSpeedStreamOutEndpoint

import luna.gateware.usb.usb3.link.crc as crcmod

# ── Parameters ────────────────────────────────────────────────────────
TOTAL_BYTES  = int(os.environ.get("LOOPBACK_BYTES", 16384))
SEED         = int(os.environ.get("LOOPBACK_SEED", 1))
HOST_GAP     = int(os.environ.get("HOST_GAP", 0))
# Worst legal inter-SKP interval: up to (2*354 - 1) bytes of accumulated
# debt entering a packet boundary (the inserter sends pairs, so debt < 2
# ordered sets cannot fire), plus one maximum-size packet (DPH+DPP,
# ~1082 bytes) during which insertion is forbidden.  Anything beyond
# ~1790 bytes means a packet boundary passed without offering the
# inserter an opportunity -- the starvation that breaks the average
# SKP rate and overruns the link partner's elastic buffer.
MAX_SKP_GAP  = int(os.environ.get("MAX_SKP_GAP", 1800))
NO_SKP_CHECK = int(os.environ.get("NO_SKP_CHECK", 0))
WITH_CONTROL = int(os.environ.get("WITH_CONTROL", 0))
LBAD_EVERY   = int(os.environ.get("LBAD_EVERY", 0))
BADHDR_EVERY = int(os.environ.get("BADHDR_EVERY", 0))
HOST_BUBBLES = int(os.environ.get("HOST_BUBBLES", 0))
WINDOW_KIB   = int(os.environ.get("WINDOW_KIB", 0))
ITP_EVERY    = int(os.environ.get("ITP_EVERY", 0))
HOST_LDN_EVERY = int(os.environ.get("HOST_LDN_EVERY", 0))
CTRL_GAP     = int(os.environ.get("CTRL_GAP", 0))
CTRL_PRE_GAP = int(os.environ.get("CTRL_PRE_GAP", 0))
URB_PACKETS  = int(os.environ.get("URB_PACKETS", 0))
URB_GAP      = int(os.environ.get("URB_GAP", 2500))
URB_JITTER   = int(os.environ.get("URB_JITTER", 0))
HOST_LATENCY = int(os.environ.get("HOST_LATENCY", 0))
HOST_JITTER  = int(os.environ.get("HOST_JITTER", 0))
LC_LATENCY   = int(os.environ.get("LC_LATENCY", 0))
LC_JITTER    = int(os.environ.get("LC_JITTER", 0))
PIPE_PHASE   = int(os.environ.get("PIPE_PHASE", 0))
REORDER      = int(os.environ.get("REORDER", 0))
RETRY_TIMEOUT = int(os.environ.get("RETRY_TIMEOUT", 0))
TRACE_LO     = int(os.environ.get("TRACE_LO", 0))
TRACE_HI     = int(os.environ.get("TRACE_HI", 0))
MPS          = 1024
DEV_ADDRESS  = 5
NUM_EPS      = int(os.environ.get("NUM_EPS", 1))
EPS          = list(range(1, NUM_EPS + 1))

# Wire word constants (little-endian byte order, ctrl bit per byte).
W_HPSTART = (0xF7FBFBFB, 0b1111)   # SHP SHP SHP EPF
W_LCSTART = (0xF7FEFEFE, 0b1111)   # SLC SLC SLC EPF
W_SDP     = (0xF75C5C5C, 0b1111)   # SDP SDP SDP EPF
W_END     = (0xF7FDFDFD, 0b1111)   # END END END EPF
W_EDB     = (0xF77C7C7C, 0b1111)   # EDB EDB EDB EPF (aborted DPP)
W_SKP     = (0x3C3C3C3C, 0b1111)   # SKP x4 (two SKP ordered sets)
W_IDLE    = (0x00000000, 0b0000)

TS1_WORDS = [(0xBCBCBCBC, 0b1111), (0x4A4A0000, 0), (0x4A4A4A4A, 0), (0x4A4A4A4A, 0)]
TS2_WORDS = [(0xBCBCBCBC, 0b1111), (0x45450000, 0), (0x45454545, 0), (0x45454545, 0)]

# Link command codes [usb_protocol.types.superspeed.LinkCommand]
LC_LGOOD, LC_LCRD, LC_LRTY, LC_LBAD = 0, 1, 2, 3
LC_LGO_U, LC_LAU, LC_LXU, LC_LPMA = 4, 5, 6, 7
LC_LUP, LC_LDN = 8, 11
LC_NAMES = {0: "LGOOD", 1: "LCRD", 2: "LRTY", 3: "LBAD", 4: "LGO_U",
            5: "LAU", 6: "LXU", 7: "LPMA", 8: "LUP", 11: "LDN"}

# Header packet types
HP_LMP, HP_TP, HP_DP, HP_ITP = 0, 4, 8, 12
# TP subtypes
TP_ACK, TP_NRDY, TP_ERDY, TP_STATUS, TP_STALL = 1, 2, 3, 4, 5


# ── CRC helpers: bit-exact ports of the gateware equations ────────────
# The gateware CRC modules build pure-XOR expressions over indexed bits;
# by monkeypatching ``Cat`` and feeding an int-backed bit vector, the
# very same code computes the values in Python.  Host and device then
# agree by construction.

class _Bits:
    def __init__(self, val, width):
        self.val = val
        self.width = width

    def __getitem__(self, i):
        if isinstance(i, slice):
            lo, hi, st = i.indices(self.width)
            assert st == 1
            return _Bits((self.val >> lo) & ((1 << (hi - lo)) - 1), hi - lo)
        return (self.val >> i) & 1

    def __len__(self):
        return self.width


def _fake_cat(*bits):
    out = 0
    for i, b in enumerate(bits):
        out |= (b & 1) << i
    return out


def _bitrev(v, w):
    out = 0
    for i in range(w):
        if (v >> i) & 1:
            out |= 1 << (w - 1 - i)
    return out


_hp_crc = crcmod.HeaderPacketCRC()
_dp_crc = crcmod.DataPacketPayloadCRC()


def _with_fake_cat(fn):
    """Runs ``fn`` with the crc module's Cat swapped for the int version."""
    def wrapper(*args, **kwargs):
        saved = crcmod.Cat
        crcmod.Cat = _fake_cat
        try:
            return fn(*args, **kwargs)
        finally:
            crcmod.Cat = saved
    return wrapper


@_with_fake_cat
def crc16_header(dws):
    cur = 0xFFFF
    for w in dws:
        cur = _hp_crc._generate_next_crc(_Bits(cur, 16), _Bits(w, 32))
    return (~_bitrev(cur, 16)) & 0xFFFF


@_with_fake_cat
def crc5(v11):
    return crcmod.compute_usb_crc5(_Bits(v11, 11)) & 0x1F


@_with_fake_cat
def crc32_payload(data: bytes):
    cur = 0xFFFFFFFF
    n = len(data)
    i = 0
    while n - i >= 4:
        w = int.from_bytes(data[i:i + 4], "little")
        cur = _dp_crc._generate_next_full_crc(_Bits(cur, 32), _Bits(w, 32))
        i += 4
    rem = n - i
    if rem:
        w = int.from_bytes(data[i:], "little")
        if rem == 3:
            cur = _dp_crc._generate_next_3B_crc(_Bits(cur, 32), _Bits(w, 24))
        elif rem == 2:
            cur = _dp_crc._generate_next_2B_crc(_Bits(cur, 32), _Bits(w, 16))
        elif rem == 1:
            cur = _dp_crc._generate_next_1B_crc(_Bits(cur, 32), _Bits(w, 8))
    return (~_bitrev(cur, 32)) & 0xFFFFFFFF


# ── Wire frame builders (host -> device) ──────────────────────────────

def build_header(dw0, dw1, dw2, seq, delayed=0, deferred=0):
    """Returns the five (data, ctrl) words of a header packet."""
    c16 = crc16_header([dw0, dw1, dw2])
    link_control = (seq & 0x7) | (0 << 3) | (0 << 6) | ((delayed & 1) << 9) \
        | ((deferred & 1) << 10)
    dw3 = c16 | (link_control << 16) | (crc5(link_control) << 27)
    return [W_HPSTART, (dw0, 0), (dw1, 0), (dw2, 0), (dw3, 0)]


def build_link_command(command, subtype):
    lcw = (subtype & 0xF) | ((command & 0xF) << 7)
    lcw |= crc5(lcw & 0x7FF) << 11
    return [W_LCSTART, (lcw | (lcw << 16), 0)]


def frame_ack_tp(*, ep, nseq, nump, retry=0, direction=1):
    dw0 = HP_TP | (DEV_ADDRESS << 25)
    dw1 = (TP_ACK
           | ((retry & 1) << 6)
           | ((direction & 1) << 7)
           | ((ep & 0xF) << 8)
           | ((nump & 0x1F) << 16)
           | ((nseq & 0x1F) << 21))
    return {"dw0": dw0, "dw1": dw1, "dw2": 0, "payload": None,
            "tok_ep": ep, "tok_nump": nump,
            "kind": f"ACK(nseq={nseq},rty={retry},nump={nump})"}


def frame_out_dp(ep, data_seq, payload: bytes, *, setup=0, address=DEV_ADDRESS):
    dw0 = HP_DP | ((address & 0x7F) << 25)
    dw1 = (data_seq & 0x1F) | (0 << 7) | ((ep & 0xF) << 8) \
        | ((setup & 1) << 15) | (len(payload) << 16)
    return {"dw0": dw0, "dw1": dw1, "dw2": 0, "payload": payload,
            "kind": f"DP(ep={ep},dseq={data_seq}{',SETUP' if setup else ''})"}


def frame_status_tp(ep=0, *, address=0):
    dw0 = HP_TP | ((address & 0x7F) << 25)
    dw1 = TP_STATUS | (1 << 7) | ((ep & 0xF) << 8)
    return {"dw0": dw0, "dw1": dw1, "dw2": 0, "payload": None,
            "kind": f"STATUS(ep={ep})"}


def frame_lmp(subtype, dw0_extra=0, dw1=0):
    dw0 = HP_LMP | ((subtype & 0xF) << 5) | dw0_extra
    return {"dw0": dw0, "dw1": dw1, "dw2": 0, "payload": None,
            "kind": f"LMP({subtype})"}


def frame_itp(cycle):
    """Isochronous Timestamp Packet [USB3.2r1: 8.7]: broadcast, no
    response expected, but consumes a header sequence number and a
    credit like any header -- a real xHC sends one every bus interval,
    interleaved with whatever else is in flight."""
    bus_interval = (cycle // 15625) & 0x3FFF
    delta = cycle % 8192
    dw0 = HP_ITP | (bus_interval << 5) | (delta << 19)
    return {"dw0": dw0, "dw1": 0, "dw2": 0, "payload": None,
            "kind": f"ITP({bus_interval})"}


def frame_to_words(frame, seq, *, delayed=0, corrupt=False):
    """Encodes a frame dict into wire words; regenerable for retries."""
    words = build_header(frame["dw0"], frame["dw1"], frame["dw2"], seq,
                         delayed=delayed)
    if corrupt:
        # Flip one CRC-16 bit: the device must LBAD this header.
        words[4] = (words[4][0] ^ 1, 0)
    if frame["payload"] is not None:
        payload = frame["payload"]
        words.append(W_SDP)
        assert len(payload) % 4 == 0, "host encoder sends word-aligned payloads"
        for i in range(0, len(payload), 4):
            words.append((int.from_bytes(payload[i:i + 4], "little"), 0))
        words.append((crc32_payload(payload), 0))
        words.append(W_END)
    return words


# ── Fake physical layer boundary ──────────────────────────────────────

class FakePhysicalLayer:
    """Raw-stream stand-in for USB3PhysicalLayer, driven by the host model."""

    def __init__(self):
        self.sink                     = USBRawSuperSpeedStream()  # device TX
        self.source                   = USBRawSuperSpeedStream()  # device RX (descrambled view)
        self.raw_source               = USBRawSuperSpeedStream()  # device RX (raw view)

        self.ready                    = Signal()
        self.engage_terminations      = Signal()
        self.tx_deemph                = Signal(2)
        self.tx_electrical_idle       = Signal()
        self.tx_ones_zeros            = Signal()
        self.invert_rx_polarity       = Signal()
        self.train_equalizer          = Signal()
        self.vbus_present             = Signal()
        self.enable_scrambling        = Signal()

        self.perform_rx_detection     = Signal()
        self.link_partner_detected    = Signal()
        self.no_link_partner_detected = Signal()

        self.send_lfps_polling        = Signal()
        self.lfps_cycles_sent         = Signal(16)
        self.lfps_ping_detected       = Signal()
        self.lfps_polling_detected    = Signal()
        self.lfps_reset_detected      = Signal()

        self.can_send_skp             = Signal()


def _control_descriptors():
    d = SuperSpeedDeviceDescriptorCollection()
    with d.DeviceDescriptor() as dev:
        dev.bDeviceClass = 0xFF
        dev.idVendor = 0x1209
        dev.idProduct = 0x0001
        dev.bcdUSB = 3.2
        dev.bMaxPacketSize0 = 9
        dev.iManufacturer = "sim"
        dev.iProduct = "sim"
        dev.iSerialNumber = "0"
        dev.bNumConfigurations = 1
    with d.ConfigurationDescriptor() as c:
        c.bMaxPower = 50
        with c.InterfaceDescriptor() as i:
            i.bInterfaceNumber = 0
            i.bInterfaceClass = 0xFF
            for ep in EPS:
                with i.EndpointDescriptor(add_default_superspeed=True) as e:
                    e.bEndpointAddress = 0x80 | ep
                    e.bmAttributes = 0x02
                    e.wMaxPacketSize = 1024
                with i.EndpointDescriptor(add_default_superspeed=True) as e:
                    e.bEndpointAddress = ep
                    e.bmAttributes = 0x02
                    e.wMaxPacketSize = 1024
    return d


class LinkBench(Elaboratable):
    def __init__(self):
        self.phy = FakePhysicalLayer()
        self.link = USB3LinkLayer(physical_layer=self.phy, tseq_burst_length=32)
        self.protocol = USB3ProtocolLayer(link_layer=self.link)
        self.mux = SuperSpeedEndpointMultiplexer()
        self.control_ep = None
        if WITH_CONTROL:
            self.control_ep = USB3ControlEndpoint()
            self.control_ep.add_standard_request_handlers(
                _control_descriptors())
        self.out_eps = {ep: SuperSpeedStreamOutEndpoint(
            endpoint_number=ep, max_packet_size=MPS) for ep in EPS}
        self.in_eps = {ep: SuperSpeedStreamInEndpoint(
            endpoint_number=ep, max_packet_size=MPS, generate_zlps=False)
            for ep in EPS}
        # Real TX conditioning stages, exactly as in physical/layer.py --
        # INCLUDING the scrambler (paired with a real descrambler at the
        # host boundary).  The scrambler was originally omitted here as
        # "data-neutral", which hid its interaction with the skid stage:
        # its forced-valid sink turned skid bubbles into stale garbage
        # words on the hardware wire (bug #28, HANDOVER 10l).
        self.tx_skid = TxStreamSkidBuffer()
        self.tx_scrambler = Scrambler(initial_value=0xffff)
        self.tx_ctc = CTCSkipInserter()
        self.host_descrambler = Descrambler(initial_value=0xffff)

    def elaborate(self, platform):
        m = Module()
        m.domains += ClockDomain("ss")

        m.submodules.link = self.link
        m.submodules.protocol = self.protocol
        m.submodules.mux = self.mux
        for ep in EPS:
            m.submodules[f"out_ep{ep}"] = self.out_eps[ep]
            m.submodules[f"in_ep{ep}"] = self.in_eps[ep]
        m.submodules.tx_skid = self.tx_skid
        m.submodules.tx_scrambler = self.tx_scrambler
        m.submodules.tx_ctc = self.tx_ctc
        m.submodules.host_descrambler = self.host_descrambler

        # Endpoint wiring, exactly as USBSuperSpeedDevice does it.
        shared = self.mux.shared
        proto_ep = self.protocol.endpoint_interface
        m.d.comb += [
            shared.rx                 .tap(proto_ep.rx),
            shared.rx_header          .eq(proto_ep.rx_header),
            shared.rx_complete        .eq(proto_ep.rx_complete),
            shared.rx_invalid         .eq(proto_ep.rx_invalid),

            proto_ep.tx               .stream_eq(shared.tx),
            proto_ep.tx_zlp           .eq(shared.tx_zlp),
            proto_ep.tx_length        .eq(shared.tx_length),
            proto_ep.tx_endpoint_number  .eq(shared.tx_endpoint_number),
            proto_ep.tx_sequence_number  .eq(shared.tx_sequence_number),
            proto_ep.tx_direction     .eq(shared.tx_direction),

            proto_ep.handshakes_out   .connect(shared.handshakes_out),
            proto_ep.handshakes_in    .connect(shared.handshakes_in),
        ]

        # Device address handling, as in USBSuperSpeedDevice: starts at 0,
        # updated by the control endpoint on SET_ADDRESS.  Without a control
        # endpoint we emulate an already-addressed device.
        address = Signal(7, init=0 if WITH_CONTROL else DEV_ADDRESS)
        with m.If(shared.address_changed):
            m.d.ss += address.eq(shared.new_address)
        m.d.comb += [
            self.protocol.current_address.eq(address),
            self.link.current_address .eq(address),
        ]

        if self.control_ep is not None:
            m.submodules.control_ep = self.control_ep
            self.mux.add_interface(self.control_ep.interface)

        for ep in EPS:
            self.mux.add_interface(self.out_eps[ep].interface)
            self.mux.add_interface(self.in_eps[ep].interface)

            # Bulk echo per pair: OUT drains into IN.
            m.d.comb += self.in_eps[ep].stream.stream_eq(
                self.out_eps[ep].stream)

        # TX conditioning: link sink -> skid -> scrambler -> skip inserter,
        # wired exactly as the real physical layer (physical/layer.py),
        # including the scrambler's always-valid sink; then a real
        # descrambler standing in for the host's receiver, so any stale
        # word the conditioning chain fabricates reaches the host parser
        # exactly as it reaches the wire.
        m.d.comb += [
            self.tx_skid.sink              .stream_eq(self.phy.sink),

            self.tx_scrambler.enable       .eq(1),
            self.tx_scrambler.sink         .stream_eq(self.tx_skid.source,
                                                      omit={'valid', 'data',
                                                            'ctrl'}),
            self.tx_scrambler.sink.valid   .eq(1),
            # Bug-#28 fix, as in physical/layer.py: skid bubbles become
            # logical idle, never a stale word.
            self.tx_scrambler.sink.data    .eq(Mux(self.tx_skid.source.valid,
                                                   self.tx_skid.source.data,
                                                   0)),
            self.tx_scrambler.sink.ctrl    .eq(Mux(self.tx_skid.source.valid,
                                                   self.tx_skid.source.ctrl,
                                                   0)),

            self.tx_ctc.sink               .stream_eq(self.tx_scrambler.source),
            self.tx_ctc.can_send_skip      .eq(self.tx_skid.source.valid &
                                               self.tx_skid.source.first),
            self.tx_scrambler.hold         .eq(self.tx_ctc.sending_skip),

            # Host-side view: descramble everything except SKPs (the real
            # receiver's CTC removes them before descrambling; here the
            # host model skips SKP words itself, so hold the descrambler
            # LFSR while they pass).  The CTC's output is registered, so
            # the SKP word lags ``sending_skip`` by one cycle.
            self.host_descrambler.enable   .eq(1),
            self.host_descrambler.sink     .stream_eq(self.tx_ctc.source,
                                                      omit={'ready'}),
            self.tx_ctc.source.ready       .eq(self.host_descrambler.sink.ready),
        ]
        skp_on_wire = Signal()
        m.d.ss += skp_on_wire.eq(self.tx_ctc.sending_skip)
        m.d.comb += self.host_descrambler.hold.eq(skp_on_wire)

        return m


# ── Failure reporting ─────────────────────────────────────────────────

class SimViolation(Exception):
    pass


class EventLog:
    def __init__(self, depth=int(os.environ.get('LOG_DEPTH', 400))):
        self.depth = depth
        self.events = []

    def add(self, cycle, kind, detail=""):
        self.events.append((cycle, kind, detail))
        if len(self.events) > self.depth:
            self.events.pop(0)

    def dump(self):
        print("---- last events ----")
        for cyc, kind, detail in self.events:
            print(f"  {cyc:>8} {kind:<14} {detail}")


# ── Device TX wire parser ─────────────────────────────────────────────

class DevTxParser:
    """Parses (and asserts on) the device's transmitted wire stream."""

    def __init__(self, log, dispatch):
        self.log = log
        self.dispatch = dispatch          # callback(kind, info, cycle)
        self.state = "IDLE"
        self.hdr = []
        self.exp_hdr_seq = None           # set by advertisement tracking
        self.dpp = None                   # dict during payload reception
        self.enabled = False
        self.ignoring = False             # post-LBAD: drop headers until LRTY
        self.cycle = 0
        self.words_seen = 0

    def violation(self, msg):
        raise SimViolation(f"cycle {self.cycle}: TX-PARSE: {msg} "
                           f"(state={self.state})")

    def feed(self, data, ctrl, cycle):
        self.cycle = cycle
        if not self.enabled:
            return
        self.words_seen += 1
        word = (data, ctrl)

        if TRACE_LO <= cycle < TRACE_HI:
            print(f"  {cycle:>8} tx-word {data:08x}/{ctrl:04b} [{self.state}]")

        if self.state == "IDLE":
            if word == W_IDLE:
                return
            if word == W_HPSTART:
                self.hdr = []
                self.state = "HDR"
                return
            if word == W_LCSTART:
                self.state = "LC"
                return
            self.violation(f"unexpected word {data:08x}/{ctrl:04b} at top level")

        elif self.state == "HDR":
            if ctrl != 0:
                self.violation(f"K-symbols inside header: {data:08x}/{ctrl:04b}")
            self.hdr.append(data)
            if len(self.hdr) == 4:
                self._check_header()
            return

        elif self.state == "EXPECT_SDP":
            # THE atomicity assertion: a Data Packet Header must be
            # followed immediately by its Data Packet Payload.
            if word != W_SDP:
                self.violation(
                    f"DPH (seq {self.dpp['data_seq']}, len "
                    f"{self.dpp['length']}) not followed by DPP framing: "
                    f"got {data:08x}/{ctrl:04b}")
            self.state = "DPP"
            return

        elif self.state == "DPP":
            self._feed_dpp_word(data, ctrl)
            return

        elif self.state == "LC":
            if ctrl != 0:
                self.violation(f"K-symbols in link command word: {data:08x}")
            lo = data & 0xFFFF
            hi = (data >> 16) & 0xFFFF
            if lo != hi:
                self.violation(f"link command halves differ: {data:08x}")
            if (lo >> 11) & 0x1F != crc5(lo & 0x7FF):
                self.violation(f"link command CRC-5 mismatch: {data:08x}")
            cmd = (lo >> 7) & 0xF
            sub = lo & 0xF
            self.dispatch("lc", {"cmd": cmd, "sub": sub}, self.cycle)
            self.state = "IDLE"
            return

    # -- header handling ------------------------------------------------

    def _check_header(self):
        dw0, dw1, dw2, dw3 = self.hdr
        c16 = dw3 & 0xFFFF
        if c16 != crc16_header([dw0, dw1, dw2]):
            self.violation(f"header CRC-16 mismatch: dws "
                           f"{dw0:08x} {dw1:08x} {dw2:08x} {dw3:08x}")
        link_control = (dw3 >> 16) & 0x7FF
        if (dw3 >> 27) & 0x1F != crc5(link_control):
            self.violation(f"header CRC-5 mismatch: dw3 {dw3:08x}")
        seq = link_control & 0x7
        delayed = (dw3 >> 25) & 1
        if self.exp_hdr_seq is not None and not self.ignoring:
            if seq != self.exp_hdr_seq:
                self.violation(f"header sequence {seq}, expected "
                               f"{self.exp_hdr_seq}")
            self.exp_hdr_seq = (self.exp_hdr_seq + 1) & 0x7

        ptype = dw0 & 0x1F
        info = {"dw0": dw0, "dw1": dw1, "dw2": dw2, "seq": seq,
                "delayed": delayed, "type": ptype}
        if ptype == HP_DP:
            self.log.add(self.cycle, "dev-DPH",
                         f"seq={seq} ep={(dw1 >> 8) & 0xF} "
                         f"dseq={dw1 & 0x1F} len={(dw1 >> 16) & 0xFFFF} "
                         f"dl={delayed}")
            self.dpp = {
                "data_seq": dw1 & 0x1F,
                "direction": (dw1 >> 7) & 1,
                "ep": (dw1 >> 8) & 0xF,
                "length": (dw1 >> 16) & 0xFFFF,
                "payload": bytearray(),
                "crc": bytearray(),
                "end": [],
                "info": info,
            }
            self.state = "EXPECT_SDP"
        else:
            self.dispatch("hdr", info, self.cycle)
            self.state = "IDLE"

    # -- DPP handling (byte-wise: tails may be unaligned) ----------------

    def _feed_dpp_word(self, data, ctrl):
        d = self.dpp

        # A link-level retransmission of a data packet header aborts its
        # payload: the transmitter has no copy, so it sends EDB framing in
        # place of the DPP.  Only legal directly after the SDP framing and
        # only for delayed (DL=1) headers in this implementation.
        if (data, ctrl) == W_EDB and not d["payload"] and not d["crc"]:
            if not d["info"]["delayed"]:
                self.violation(f"EDB-aborted DPP on a non-delayed header "
                               f"[DP seq {d['data_seq']}]")
            info = dict(d["info"])
            info.update(data_seq=d["data_seq"], ep=d["ep"],
                        direction=d["direction"], aborted=True)
            self.dispatch("dp_aborted", info, self.cycle)
            self.dpp = None
            self.state = "IDLE"
            return

        for i in range(4):
            byte = (data >> (8 * i)) & 0xFF
            k = (ctrl >> i) & 1
            need_payload = d["length"] - len(d["payload"])
            if need_payload > 0:
                if k:
                    self.violation(f"K-symbol inside DPP payload "
                                   f"(byte {len(d['payload'])}): {byte:02x}")
                d["payload"].append(byte)
            elif len(d["crc"]) < 4:
                if k:
                    self.violation(f"K-symbol inside DPP CRC: {byte:02x}")
                d["crc"].append(byte)
            elif len(d["end"]) < 4:
                exp = [(0xFD, 1), (0xFD, 1), (0xFD, 1), (0xF7, 1)][len(d["end"])]
                if (byte, k) != exp:
                    self.violation(
                        f"bad DPP end framing byte {len(d['end'])}: "
                        f"{byte:02x}/{k} (expected {exp[0]:02x}/{exp[1]}) "
                        f"[DP seq {d['data_seq']} len {d['length']}]")
                d["end"].append(byte)
            else:
                # Padding bytes after EPF within the final word: logical idle.
                if byte != 0 or k:
                    self.violation(f"non-idle padding after DPP: {byte:02x}/{k}")

        if len(d["end"]) == 4:
            crc = int.from_bytes(bytes(d["crc"]), "little")
            expect = crc32_payload(bytes(d["payload"]))
            if crc != expect:
                self.violation(f"DPP CRC-32 mismatch: got {crc:08x}, "
                               f"expected {expect:08x} "
                               f"[DP seq {d['data_seq']} len {d['length']}]")
            info = dict(d["info"])
            info.update(data_seq=d["data_seq"], ep=d["ep"],
                        direction=d["direction"], payload=bytes(d["payload"]))
            self.dispatch("dp", info, self.cycle)
            self.dpp = None
            self.state = "IDLE"


# ── Host model / main simulation ──────────────────────────────────────

def main():
    bench = LinkBench()
    sim = Simulator(bench)
    sim.add_clock(8e-9, domain="ss")

    rng = random.Random(SEED)
    payloads = {ep: bytes(rng.getrandbits(8) for _ in range(TOTAL_BYTES))
                for ep in EPS}
    packets = {ep: [payloads[ep][i:i + MPS]
                    for i in range(0, TOTAL_BYTES, MPS)]
               for ep in EPS}

    log = EventLog()
    phy = bench.phy

    st = {
        # host link-layer state
        "trained_seen": False,
        "host_hdr_seq": 0,            # 3-bit seq of headers we transmit
        "host_tx_credits": 0,         # device rx-buffer credits we hold
        "dev_adv_seen": False,        # device sent its LGOOD advertisement
        "dev_lcrd_next": 0,           # expected LCRD subtype cycle
        "dev_lgood_next": 0,          # expected LGOOD sequence
        # control-transfer phase (WITH_CONTROL)
        "addressed": not WITH_CONTROL,
        "ctrl_step": 0,
        "ctrl_wait": 0,
        # fault injection
        "dev_hdr_count": 0,           # device headers seen (for LBAD_EVERY)
        "host_hdr_count": 0,          # host headers sent (for BADHDR_EVERY)
        "lbads_sent": 0, "lbads_taken": 0, "aborted_dps": 0,
        "lbad_seq": None,             # exp seq to restore after LRTY
        # bookkeeping
        "now": 0,
        "progress_cycle": 0,
        "skp_gap": 0, "max_skp_gap": 0,
        "lc_counts": {}, "hdr_counts": {},
        "nrdy": 0, "erdy": 0, "acks": 0, "dps": 0,
        "done": False,
    }

    # Per-endpoint-pair protocol state.
    eps = {ep: {
        "out_idx": 0, "out_seq": 0, "out_wait_ack": False,
        "out_retry": 0, "out_nrdy": False, "out_retrans": 0,
        "in_seq": 0, "in_parked": False, "in_token": False,
        "rx_bytes": bytearray(),
        # xHC URB-rhythm modeling (URB_PACKETS).
        "in_urb_left":  URB_PACKETS,
        "out_urb_left": URB_PACKETS,
        "in_resume_at":  0,
        "out_resume_at": 0,
        # xHC no-response retry modeling (RETRY_TIMEOUT).
        "tok_sent_at": None,          # cycle the outstanding token went out
        "tok_retries": 0,
    } for ep in EPS}

    # Frames the host still has to transmit.  Link commands preempt
    # header frames; header frames consume a device rx-buffer credit.
    # Retried frames (after a device LBAD) bypass the credit gate.
    #
    # Each hp_queue frame carries a ``flow`` key and a ``ready`` cycle:
    # per-flow order is FIFO (protocol requirement), but with REORDER=1
    # different flows' eligible heads may be sent in any order, and with
    # HOST_LATENCY/HOST_JITTER a frame only becomes eligible some cycles
    # after it was queued -- the xHC's response latency.  lc_queue
    # entries are (ready, words) bursts with the same eligibility rule.
    lc_queue = []      # list of (ready_cycle, flat word list)
    hp_queue = []      # list of frame dicts (unsent; credit-gated)
    hp_resend = []     # list of (seq, frame) to retransmit with DL=1
    unacked = []       # list of (seq, frame) sent, awaiting device LGOOD

    flow_last_ready = {}

    def _flow_ready(flow, base, latency, jitter):
        """Monotonic per-flow eligibility cycle (order is never inverted)."""
        t = base + latency + (rng.randint(0, jitter) if jitter else 0)
        t = max(t, flow_last_ready.get(flow, 0))
        flow_last_ready[flow] = t
        return t

    def lc_push(words):
        lc_queue.append((_flow_ready("lc", st["now"], LC_LATENCY,
                                     LC_JITTER), words))

    def hp_push(frame, flow, *, earliest=None):
        frame["flow"] = flow
        base = st["now"] if earliest is None else max(st["now"], earliest)
        frame["ready"] = _flow_ready(flow, base, HOST_LATENCY, HOST_JITTER)
        hp_queue.append(frame)

    def fail(msg, ctx=None):
        log.dump()
        print(f"\nHOST-STATE: {_state_summary()}")
        if ctx is not None:
            for ep in EPS:
                print(f"DEV ep{ep}: out_fsm={ctx.get(bench.out_eps[ep].debug_fsm)} "
                      f"out_fill={ctx.get(bench.out_eps[ep].debug_fill)} "
                      f"out_space={ctx.get(bench.out_eps[ep].stream.ready)} "
                      f"in_fsm={ctx.get(bench.in_eps[ep].debug_fsm)} "
                      f"in_wfill={ctx.get(bench.in_eps[ep].debug_write_fill)} "
                      f"in_rfill={ctx.get(bench.in_eps[ep].debug_read_fill)} "
                      f"in_rdy={ctx.get(bench.in_eps[ep].debug_ready)} "
                      f"in_erdyreq={ctx.get(bench.in_eps[ep].debug_erdyreq)}")
            print(f"DEV mux: pass_sel={ctx.get(bench.mux.debug_pass_sel)} "
                  f"grant={ctx.get(bench.mux.debug_grant)} "
                  f"pdisp={ctx.get(bench.mux.debug_pdisp)} "
                  f"pending={[f'{ctx.get(p):04b}' for p in bench.mux.debug_pending]}")
            print(f"DEV link: tx_credits={ctx.get(bench.link.debug_tx_credits)} "
                  f"tx_pending={ctx.get(bench.link.debug_tx_pending)} "
                  f"dtx_fsm={ctx.get(bench.link.debug_dtx_fsm)} "
                  f"dsink_v={ctx.get(bench.link.debug_dsink_valid)} "
                  f"dsink_r={ctx.get(bench.link.debug_dsink_ready)}")
        print(f"\nFAIL: {msg}")
        raise SystemExit(1)

    def _state_summary():
        per_ep = " ".join(
            f"ep{ep}[out={e['out_idx']}/{len(packets[ep])}"
            f" wa={int(e['out_wait_ack'])} nrdy={int(e['out_nrdy'])}"
            f" iseq={e['in_seq']} park={int(e['in_parked'])}"
            f" tok={int(e['in_token'])} rx={len(e['rx_bytes'])}"
            f" rty={e['out_retry']} rtx={e['out_retrans']}]"
            for ep, e in eps.items())
        return (f"{per_ep} "
                f"credits={st['host_tx_credits']} "
                f"unacked={[s for s, _ in unacked]} "
                f"lc={st['lc_counts']} hdr={st['hdr_counts']} "
                f"nrdy={st['nrdy']} erdy={st['erdy']} acks={st['acks']} "
                f"dps={st['dps']} "
                f"lbads_sent={st['lbads_sent']} lbads_taken={st['lbads_taken']} "
                f"aborted_dps={st['aborted_dps']} "
                f"tx_overlap={st.get('tx_overlap', 0)} "
                f"max_skp_gap={st['max_skp_gap']}")

    # -- host protocol reactions -----------------------------------------

    def grant_in_token(ep, nump=1, retry=0, earliest=None):
        e = eps[ep]
        hp_push(frame_ack_tp(ep=ep, nseq=e["in_seq"],
                             nump=nump, retry=retry, direction=1),
                ("in", ep), earliest=earliest)
        if nump:
            e["in_token"] = True

    def alloc_hdr_seq():
        seq = st["host_hdr_seq"]
        st["host_hdr_seq"] = (seq + 1) & 0x7
        return seq

    def out_dp_queued(ep):
        """Is a copy of this endpoint's OUT DP already queued for (re)send?"""
        tag = f"DP(ep={ep},"
        return any(f["kind"].startswith(tag) for f in hp_queue) or \
            any(f["kind"].startswith(tag) for _s, f in hp_resend)

    def send_next_out(ep, cycle):
        e = eps[ep]
        idx = e["out_idx"]
        data = packets[ep][idx]
        hp_push(frame_out_dp(ep, e["out_seq"], data), ("out", ep))
        e["out_wait_ack"] = True
        log.add(cycle, "host-OUT", f"ep={ep} idx={idx} dseq={e['out_seq']} "
                                   f"len={len(data)}")

    def _lbad_this_header(info, cycle):
        """Fault injection: reject a (structurally fine) device header with
        LBAD, exercising the device's LRTY + DL=1 retransmission path."""
        st["lbads_sent"] += 1
        st["lbad_seq"] = info["seq"]      # expected seq resumes here
        parser.ignoring = True
        lc_push(build_link_command(LC_LBAD, 0))
        log.add(cycle, "host-LBAD", f"rejected seq={info['seq']} "
                                    f"type={info['type']}")

    def _lbad_scheduled(info):
        if not LBAD_EVERY or parser.ignoring or info["delayed"]:
            return False
        st["dev_hdr_count"] += 1
        # Skip the early bringup headers; then hit every n-th.
        return st["dev_hdr_count"] > 4 and \
            st["dev_hdr_count"] % LBAD_EVERY == 0

    def dispatch(kind, info, cycle):
        if kind == "lc":
            cmd, sub = info["cmd"], info["sub"]
            name = LC_NAMES.get(cmd, f"LC{cmd}")
            st["lc_counts"][name] = st["lc_counts"].get(name, 0) + 1
            log.add(cycle, "dev-lc", f"{name}_{sub}")
            if cmd == LC_LGOOD:
                if not st["dev_adv_seen"]:
                    if sub != 7:
                        fail(f"device advertisement LGOOD_{sub}, expected 7")
                    st["dev_adv_seen"] = True
                    parser.exp_hdr_seq = 0
                else:
                    if sub != st["dev_lgood_next"]:
                        fail(f"device LGOOD_{sub}, expected "
                             f"LGOOD_{st['dev_lgood_next']}")
                    st["dev_lgood_next"] = (sub + 1) & 0x7
                    if not unacked:
                        fail(f"device LGOOD_{sub} with nothing outstanding")
                    exp, _frame = unacked.pop(0)
                    if exp != sub:
                        fail(f"device LGOOD_{sub} acks header seq {exp}")
            elif cmd == LC_LCRD:
                if sub != st["dev_lcrd_next"]:
                    fail(f"device LCRD_{sub}, expected LCRD_"
                         f"{st['dev_lcrd_next']}")
                st["dev_lcrd_next"] = (sub + 1) & 0x3
                st["host_tx_credits"] += 1
                if st["host_tx_credits"] > 4:
                    fail("device issued more than 4 credits")
            elif cmd == LC_LRTY:
                # Device answers our LBAD: resume accepting its headers,
                # from the sequence number we rejected.
                if not parser.ignoring:
                    fail("device LRTY without a pending LBAD")
                parser.ignoring = False
                parser.exp_hdr_seq = st["lbad_seq"]
                log.add(cycle, "host-resume", f"exp_seq={st['lbad_seq']}")
            elif cmd in (LC_LUP, LC_LDN):
                pass                       # keepalives: fine
            elif cmd == LC_LXU:
                pass                       # power-state rejection: fine
            elif cmd == LC_LBAD:
                # Device rejected one of our headers (BADHDR_EVERY):
                # send LRTY, then retransmit every unacknowledged header
                # with the DL bit set [USB3.2r1: 7.2.4.1.10].
                st["lbads_taken"] += 1
                lc_push(build_link_command(LC_LRTY, 0))
                hp_resend.extend(unacked)
                log.add(cycle, "host-LRTY",
                        f"resending {[s for s, _ in unacked]}")
            else:
                fail(f"unexpected link command {name}_{sub}")

        elif kind == "hdr":
            if parser.ignoring:
                return
            if _lbad_scheduled(info):
                _lbad_this_header(info, cycle)
                return
            ptype = info["type"]
            st["hdr_counts"][ptype] = st["hdr_counts"].get(ptype, 0) + 1
            # Host link layer: ack + credit for every good header.
            host_ack_header(info["seq"])
            if ptype == HP_TP:
                _handle_tp(info, cycle)
            elif ptype == HP_LMP:
                log.add(cycle, "dev-lmp", f"subtype={(info['dw0'] >> 5) & 0xF}")
            else:
                fail(f"unexpected header type {ptype}")

        elif kind == "dp":
            if parser.ignoring:
                return
            if _lbad_scheduled(info):
                _lbad_this_header(info, cycle)
                return
            host_ack_header(info["seq"])
            _handle_in_dp(info, cycle)

        elif kind == "dp_aborted":
            # Link-level retransmission aborted the payload (EDB): the
            # header is fine at the link level -- ack it -- but the data
            # never arrived; request a protocol-level retransmission.
            if parser.ignoring:
                return
            st["aborted_dps"] += 1
            host_ack_header(info["seq"])
            log.add(cycle, "dev-DP-abort",
                    f"ep={info['ep']} dseq={info['data_seq']}")
            grant_in_token(info["ep"], retry=1)
            st["progress_cycle"] = cycle

    def host_ack_header(seq):
        lc_push(build_link_command(LC_LGOOD, seq)
                + build_link_command(LC_LCRD, st["host_lcrd_next"]))
        st["host_lcrd_next"] = (st["host_lcrd_next"] + 1) & 0x3
    st["host_lcrd_next"] = 0

    def _handle_tp(info, cycle):
        dw1 = info["dw1"]
        subtype = dw1 & 0xF
        retry = (dw1 >> 6) & 1
        direction = (dw1 >> 7) & 1
        ep = (dw1 >> 8) & 0xF
        nump = (dw1 >> 16) & 0x1F
        nseq = (dw1 >> 21) & 0x1F

        if WITH_CONTROL and ep == 0:
            # SET_ADDRESS control transfer: SETUP ACK, then the status ACK.
            tp_addr = (info["dw0"] >> 25) & 0x7F
            if subtype != TP_ACK:
                fail(f"unexpected EP0 TP subtype {subtype}")
            if st["ctrl_step"] == 1:
                log.add(cycle, "ctrl", f"SETUP acked (addr={tp_addr})")
                st["ctrl_step"] = 2
            elif st["ctrl_step"] == 3:
                # THE SET_ADDRESS CONTRACT: the status-stage ACK must still
                # carry the *old* (zero) address [USB3.2r1: 8.5.1 / 9.4.6].
                if tp_addr != 0:
                    fail(f"SET_ADDRESS status ACK sent from the NEW address "
                         f"{tp_addr}; the host discards it (-71 'device not "
                         f"responding to setup address')")
                log.add(cycle, "ctrl", "STATUS acked from address 0")
                st["addressed"] = True
                st["progress_cycle"] = cycle
            else:
                fail(f"unexpected EP0 ACK at ctrl_step {st['ctrl_step']}")
            return

        if ep not in EPS:
            fail(f"TP for unexpected endpoint {ep}")
        e = eps[ep]

        if subtype == TP_ACK:
            st["acks"] += 1
            log.add(cycle, "dev-ACK",
                    f"ep={ep} nseq={nseq} rty={retry} nump={nump}")
            # Sequence-driven, like an xHC: the advertised next-expected
            # sequence decides; a retry bit or repeated sequence asks for a
            # (re)transmission of the current packet.  Duplicate responses
            # (possible when link-level retries overlap protocol retries)
            # are ignored.
            if nseq == ((e["out_seq"] + 1) & 0x1F):
                if e["out_wait_ack"]:
                    e["out_seq"] = nseq
                    e["out_idx"] += 1
                    e["out_wait_ack"] = False
                    st["progress_cycle"] = cycle
                    # URB rhythm: the writer's next 16 KiB URB reaches the
                    # xHC only after a resubmission gap.
                    if URB_PACKETS:
                        e["out_urb_left"] -= 1
                        if e["out_urb_left"] <= 0:
                            e["out_urb_left"] = URB_PACKETS
                            e["out_resume_at"] = cycle + URB_GAP + \
                                (rng.randint(0, URB_JITTER) if URB_JITTER
                                 else 0)
                # else: duplicate ACK of an already-completed packet.
            elif nseq == e["out_seq"]:
                # Device expects the current packet (again).
                if e["out_wait_ack"]:
                    e["out_retry"] += 1
                    log.add(cycle, "host-retry",
                            f"ep={ep} dseq={e['out_seq']}")
                    e["out_wait_ack"] = False
                # else: a (re)send is already queued; nothing to do.
            else:
                fail(f"device ACK ep={ep} with unexpected nseq {nseq} "
                     f"(out_seq={e['out_seq']})")
        elif subtype == TP_NRDY:
            st["nrdy"] += 1
            log.add(cycle, "dev-NRDY", f"ep={ep} dir={direction}")
            if direction == 1:
                # IN pipe: stop polling until ERDY.
                e["in_parked"] = True
                e["in_token"] = False
            else:
                # OUT pipe: the device discarded our unacknowledged DP;
                # hold it for retransmission after ERDY.  With link-level
                # retries in play this can also be the response to our own
                # DL-replay of an already-acknowledged DP -- stale, and
                # ignorable like an xHC ignores responses for completed
                # transfers (the next DP just gets flow-controlled again).
                if e["out_wait_ack"]:
                    e["out_nrdy"] = True
                else:
                    log.add(cycle, "host-note",
                            f"stale OUT NRDY ep={ep} ignored")
        elif subtype == TP_ERDY:
            st["erdy"] += 1
            log.add(cycle, "dev-ERDY", f"ep={ep} dir={direction}")
            if direction == 1:
                e["in_parked"] = False
            else:
                # OUT pipe reopened: retransmit the NRDY'd packet.
                if e["out_nrdy"]:
                    e["out_nrdy"] = False
                    e["out_retrans"] += 1
                    e["out_wait_ack"] = False   # main loop re-sends out_idx
                    st["progress_cycle"] = cycle
        else:
            fail(f"unexpected TP subtype {subtype}")

    def _handle_in_dp(info, cycle):
        st["dps"] += 1
        ep = info["ep"]
        if ep not in EPS or info["direction"] != 1:
            fail(f"DP from unexpected source ep={ep} "
                 f"dir={info['direction']}")
        e = eps[ep]
        dseq = info["data_seq"]
        log.add(cycle, "dev-DP",
                f"ep={ep} dseq={dseq} len={len(info['payload'])}")
        if RETRY_TIMEOUT and dseq == ((e["in_seq"] - 1) & 0x1F):
            # Duplicate of the previous packet (a late original crossing a
            # no-response retry, or a retry-induced resend): an xHC
            # discards it and re-issues the current expectation.
            log.add(cycle, "host-dup", f"ep={ep} dseq={dseq} ignored")
            e["in_token"] = False
            e["tok_sent_at"] = None
            if not any(f.get("flow") == ("in", ep) for f in hp_queue):
                grant_in_token(ep)
            return
        if dseq != e["in_seq"]:
            fail(f"device DP ep={ep} sequence {dseq}, "
                 f"expected {e['in_seq']}")
        pos = len(e["rx_bytes"])
        expect = payloads[ep][pos:pos + len(info["payload"])]
        if info["payload"] != expect:
            for i, (a, b) in enumerate(zip(info["payload"], expect)):
                if a != b:
                    break
            # Identify what was actually delivered: search all endpoint
            # payload streams for the received bytes.
            needle = bytes(info["payload"][:64])
            origin = "unknown"
            for oep in EPS:
                off = payloads[oep].find(needle)
                if off >= 0:
                    origin = (f"ep{oep} offset {off} "
                              f"(packet {off // MPS}, +{off % MPS})")
                    break
            fail(f"echo data corruption ep={ep} DP dseq={dseq}: first diff "
                 f"at byte {pos + i}: got {info['payload'][i]:02x}, "
                 f"expected {expect[i]:02x}; delivered bytes are {origin}")
        e["rx_bytes"].extend(info["payload"])
        e["in_seq"] = (e["in_seq"] + 1) & 0x1F
        e["in_token"] = False
        e["tok_sent_at"] = None
        e["tok_retries"] = 0
        st["progress_cycle"] = cycle
        if URB_PACKETS:
            e["in_urb_left"] -= 1
            if e["in_urb_left"] <= 0 and len(e["rx_bytes"]) < TOTAL_BYTES:
                # URB boundary: the xHC has no transfer ring buffer left,
                # so the closing ACK CANNOT grant credit -- NumP=0.  The
                # next URB's token then repeats the same nseq with NumP=1
                # after the resubmission gap.
                e["in_urb_left"] = URB_PACKETS
                grant_in_token(ep, nump=0)
                resume = cycle + URB_GAP + \
                    (rng.randint(0, URB_JITTER) if URB_JITTER else 0)
                e["in_resume_at"] = resume
                grant_in_token(ep, earliest=resume)
                return
        # ACK doubles as the next IN token, like an xHC.
        grant_in_token(ep)

    parser = DevTxParser(log, dispatch)

    # -- main host coroutine ---------------------------------------------

    async def host(ctx):
        ctc_src = bench.host_descrambler.source
        ctx.set(ctc_src.ready, 1)
        ctx.set(phy.vbus_present, 1)
        ctx.set(phy.ready, 1)

        cycle = 0
        feed = None                 # training pattern (list of words), looping
        feed_pos = 0
        lfps_div = 0
        train_phase = "DETECT"
        rx_words = []               # host->device words currently streaming
        rx_gap = 0
        advertised = False

        def refill_rx():
            nonlocal rx_words
            if lc_queue:
                # link commands preempt and are never credit-gated; they
                # are sent in FIFO order once their (latency-modeled)
                # eligibility cycle arrives.
                burst = []
                while lc_queue and lc_queue[0][0] <= cycle:
                    burst.extend(lc_queue.pop(0)[1])
                if burst:
                    rx_words = burst
                    log.add(cycle, "host-lc-burst", f"{len(burst)} words")
                    return
            if hp_resend:
                # Link-level retransmission after a device LBAD: original
                # sequence numbers, DL=1, no fresh credit consumed.
                seq, frame = hp_resend.pop(0)
                log.add(cycle, "host-resend", f"seq={seq} {frame['kind']}")
                rx_words = frame_to_words(frame, seq, delayed=1)
                return
            if hp_queue and st["host_tx_credits"] > 0:
                # Eligible = per-flow head frames whose ready cycle has
                # arrived.  Default: the first eligible frame in queue
                # order (exactly the historical FIFO when latencies are
                # 0); REORDER=1 picks randomly among eligible flows.
                seen_flows = set()
                eligible = []
                for i, f in enumerate(hp_queue):
                    flow = f.get("flow")
                    if flow in seen_flows:
                        continue
                    seen_flows.add(flow)
                    if f.get("ready", 0) <= cycle:
                        eligible.append(i)
                if not eligible:
                    return
                idx = rng.choice(eligible) if REORDER else eligible[0]
                st["host_tx_credits"] -= 1
                frame = hp_queue.pop(idx)
                seq = alloc_hdr_seq()
                unacked.append((seq, frame))
                if len(unacked) > 8:
                    # Sending is credit-gated (<=4 outstanding per the
                    # device's own buffer count); more than 8 un-LGOODed
                    # headers means the device is dropping LGOODs (a real
                    # host's 5 ms PENDING_HP_TIMER then forces Recovery).
                    fail(f"LGOOD starvation: {len(unacked)} headers "
                         f"unacknowledged: {[s for s, _ in unacked]}")
                st["host_hdr_count"] += 1
                # Stamp outstanding-token dispatch time (RETRY_TIMEOUT).
                if frame.get("tok_ep") in eps and frame.get("tok_nump"):
                    eps[frame["tok_ep"]]["tok_sent_at"] = cycle
                corrupt = bool(BADHDR_EVERY) and \
                    st["host_hdr_count"] > 4 and \
                    st["host_hdr_count"] % BADHDR_EVERY == 0
                log.add(cycle, "host-frame",
                        f"seq={seq} {frame['kind']}"
                        + (" CORRUPTED" if corrupt else ""))
                rx_words = frame_to_words(frame, seq, corrupt=corrupt)

        while not st["done"]:
            await ctx.tick("ss")
            cycle += 1
            st["now"] = cycle

            # ── consume device TX (post skip-inserter) ────────────────
            trained = ctx.get(bench.link.trained)
            if st["trained_seen"]:
                st["skp_gap"] += 4
            v = ctx.get(ctc_src.valid)
            if v:
                data = ctx.get(ctc_src.data)
                ctrl = ctx.get(ctc_src.ctrl)
                if (data, ctrl) == W_SKP:
                    if parser.enabled and parser.state not in ("IDLE",):
                        fail(f"cycle {cycle}: SKP inserted inside a packet "
                             f"(parser state {parser.state})")
                    st["max_skp_gap"] = max(st["max_skp_gap"], st["skp_gap"])
                    st["skp_gap"] = 0
                else:
                    try:
                        parser.feed(data, ctrl, cycle)
                    except SimViolation as e:
                        fail(str(e))
            if st["trained_seen"] and not NO_SKP_CHECK:
                if st["skp_gap"] > MAX_SKP_GAP:
                    fail(f"cycle {cycle}: SKP starvation: {st['skp_gap']} "
                         f"wire bytes without a SKP ordered set "
                         f"(limit {MAX_SKP_GAP}); parser state "
                         f"{parser.state}")
            st["max_skp_gap"] = max(st["max_skp_gap"], st["skp_gap"])

            # ── generator-dispatch tracing (multi-EP debug) ───────────
            if st["trained_seen"] and NUM_EPS > 1:
                sho = bench.mux.shared.handshakes_out
                if ctx.get(sho.ready):
                    kindbits = (ctx.get(sho.send_ack),
                                ctx.get(sho.send_nrdy),
                                ctx.get(sho.send_erdy),
                                ctx.get(sho.send_stall))
                    if any(kindbits):
                        log.add(cycle, "gen-dispatch",
                                f"a{kindbits[0]}n{kindbits[1]}"
                                f"e{kindbits[2]}s{kindbits[3]} "
                                f"ep={ctx.get(sho.endpoint_number)} "
                                f"dir={ctx.get(sho.direction)} "
                                f"psel={ctx.get(bench.mux.debug_pass_sel)} "
                                f"grant={ctx.get(bench.mux.debug_grant)} "
                                f"gkind={ctx.get(bench.mux.debug_grant_kind):04b}")

            # ── handshake-interface tracing (multi-EP debug) ──────────
            if st["trained_seen"] and NUM_EPS > 1:
                for ep in EPS:
                    for pfx, epi in (("o", bench.out_eps[ep]),
                                     ("i", bench.in_eps[ep])):
                        hso = epi.interface.handshakes_out
                        snap = (ctx.get(hso.send_ack), ctx.get(hso.send_nrdy),
                                ctx.get(hso.send_erdy), ctx.get(hso.done),
                                ctx.get(epi.debug_fsm))
                        key = f"_hs_{pfx}{ep}"
                        if snap != st.get(key):
                            st[key] = snap
                            log.add(cycle, f"hs-{pfx}{ep}",
                                    f"ack={snap[0]} nrdy={snap[1]} "
                                    f"erdy={snap[2]} done={snap[3]} "
                                    f"fsm={snap[4]}")

            # ── endpoint TX overlap diagnostics ───────────────────────
            if st["trained_seen"] and NUM_EPS > 1:
                n_valid = sum(1 for ep in EPS
                              if ctx.get(bench.in_eps[ep].stream.valid))
                # bench.in_eps[ep].stream is the *loopback* input; the TX
                # side is interface.tx:
                n_tx = sum(1 for ep in EPS
                           if ctx.get(bench.in_eps[ep].interface.tx.valid))
                if n_tx > 1:
                    st["tx_overlap"] = st.get("tx_overlap", 0) + 1

            # ── TX handoff-boundary monitor (DBG_HANDOFF=1) ───────────
            # Watches the endpoint-mux -> skid -> DataPacketTransmitter
            # chain across mux grant handoffs: the shared transmitter
            # delimits packets by a one-cycle valid gap, and the skid
            # stage can compress that gap away when it spans a frozen,
            # occupied buffer (open item #23 investigation).
            if os.environ.get("DBG_HANDOFF") and st["trained_seen"]:
                mux_v = ctx.get(bench.mux.shared.tx.valid) != 0
                skid_v = ctx.get(bench.link.data_sink.valid) != 0
                dtx = ctx.get(bench.link.debug_dtx_fsm)
                key = st.get("_ho")
                snap = (mux_v, skid_v, dtx)
                if snap != key:
                    st["_ho"] = snap
                    log.add(cycle, "handoff",
                            f"muxv={int(mux_v)} skidv={int(skid_v)} "
                            f"dtx={dtx}")
                # Invariant: data_tx only enters SEND_HEADER (1) from
                # WAIT_FOR_DATA (0) -- an entry from SEND_PAYLOAD would
                # mean a packet boundary was never observed.
                prev_dtx = st.get("_dtx_prev", 0)
                if dtx == 1 and prev_dtx not in (0, 1):
                    fail(f"cycle {cycle}: data_tx offered a header without "
                         f"passing WAIT_FOR_DATA since the previous packet "
                         f"(mux handoff boundary compressed; prev state "
                         f"{prev_dtx})")
                st["_dtx_prev"] = dtx

            # ── recovery-cause diagnostics ────────────────────────────
            if st["trained_seen"]:
                if ctx.get(bench.link.debug_payload_underrun):
                    fail(f"cycle {cycle}: transmitter consumed an INVALID "
                         f"payload word (gapless-feed contract violation)",
                         ctx)
                for name, sig in (("rec-timers", bench.link.debug_rec_timers),
                                  ("rec-rx", bench.link.debug_rec_rx),
                                  ("rec-tx", bench.link.debug_rec_tx),
                                  ("rx-bad-packet", bench.link.debug_rx_bad_packet)):
                    if ctx.get(sig):
                        log.add(cycle, "DEV-" + name,
                                f"credits={ctx.get(bench.link.debug_tx_credits)} "
                                f"pending={ctx.get(bench.link.debug_tx_pending)} "
                                f"rx_seq={ctx.get(bench.link.debug_rx_seq)} "
                                f"exp_seq={ctx.get(bench.link.debug_rx_expected_seq)}")

            # ── link training script ──────────────────────────────────
            if not trained:
                if st["trained_seen"]:
                    fail(f"cycle {cycle}: link dropped out of U0 "
                         f"(recovery triggered)")
                if train_phase == "DETECT":
                    if ctx.get(phy.perform_rx_detection):
                        ctx.set(phy.link_partner_detected, 1)
                        train_phase = "LFPS"
                elif train_phase == "LFPS":
                    ctx.set(phy.link_partner_detected, 0)
                    if ctx.get(phy.send_lfps_polling):
                        ctx.set(phy.lfps_polling_detected, 1)
                        lfps_div += 1
                        if lfps_div % 8 == 0:
                            ctx.set(phy.lfps_cycles_sent,
                                    ctx.get(phy.lfps_cycles_sent) + 1)
                    elif lfps_div > 0:
                        # LFPS handshake done; wait for training sets
                        ctx.set(phy.lfps_polling_detected, 0)
                        train_phase = "TS"
                        feed = TS1_WORDS
                elif train_phase == "TS":
                    # switch the feed reactively on what the device sends
                    if v and ctrl == 0 and data in (0x45450000,):
                        feed = TS2_WORDS
                    if v and feed is TS2_WORDS and (data, ctrl) == W_IDLE:
                        feed = [W_IDLE]
            else:
                if not st["trained_seen"]:
                    st["trained_seen"] = True
                    st["progress_cycle"] = cycle
                    parser.enabled = True
                    feed = None
                    log.add(cycle, "U0", "link trained")
                    print(f"link trained at cycle {cycle}")

            # ── drive device RX ───────────────────────────────────────
            if feed is not None:
                w = feed[feed_pos % len(feed)]
                feed_pos += 1
                for stream in (phy.source, phy.raw_source):
                    ctx.set(stream.valid, 1)
                    ctx.set(stream.data, w[0])
                    ctx.set(stream.ctrl, w[1])
            elif st["trained_seen"]:
                ctx.set(phy.raw_source.valid, 0)
                if not advertised:
                    # host link bringup: sequence advertisement + credits
                    advertised = True
                    adv = build_link_command(LC_LGOOD, 7)
                    for i in range(4):
                        adv += build_link_command(LC_LCRD, i)
                    lc_push(adv)
                    # LMP dance like a real host
                    hp_push(frame_lmp(4, dw0_extra=(1 << 9),
                                      dw1=(4 | (1 << 16))), ("mgmt",))
                    hp_push(frame_lmp(5, dw0_extra=(1 << 9)), ("mgmt",))
                    if not WITH_CONTROL:
                        # open the IN pipes (with the thread-start skew of
                        # the bench harness, when modeled)
                        for ep in EPS:
                            phase = rng.randint(0, PIPE_PHASE) \
                                if PIPE_PHASE else 0
                            grant_in_token(ep, earliest=cycle + phase)
                            eps[ep]["out_resume_at"] = cycle + \
                                (rng.randint(0, PIPE_PHASE)
                                 if PIPE_PHASE else 0)

                # Background traffic a real xHC generates around (and
                # during) the control exchange: periodic ITP headers and
                # LDN keepalive link commands.
                if ITP_EVERY and cycle % ITP_EVERY == 0:
                    hp_push(frame_itp(cycle), ("mgmt",))
                if HOST_LDN_EVERY and cycle % HOST_LDN_EVERY == 0:
                    lc_push(build_link_command(LC_LDN, 0))

                # SET_ADDRESS control transfer, before any bulk traffic
                # (mirrors real enumeration: it is the first transfer the
                # host issues, and the first thing that broke on hardware).
                # CTRL_PRE_GAP/CTRL_GAP scan the hardware phases: bringup->
                # SETUP and SETUP-ack->STATUS turnaround respectively.
                if WITH_CONTROL and not st["addressed"]:
                    if st["ctrl_step"] == 0:
                        if st["ctrl_wait"] < CTRL_PRE_GAP:
                            st["ctrl_wait"] += 1
                        else:
                            st["ctrl_wait"] = 0
                            setup = bytes([0x00, 0x05, DEV_ADDRESS, 0x00,
                                           0x00, 0x00, 0x00, 0x00])
                            hp_push(frame_out_dp(0, 0, setup,
                                                 setup=1, address=0),
                                    ("ctrl",))
                            log.add(cycle, "ctrl", "SETUP(SET_ADDRESS) sent")
                            st["ctrl_step"] = 1
                    elif st["ctrl_step"] == 2:
                        if st["ctrl_wait"] < CTRL_GAP:
                            st["ctrl_wait"] += 1
                        else:
                            st["ctrl_wait"] = 0
                            hp_push(frame_status_tp(0, address=0), ("ctrl",))
                            log.add(cycle, "ctrl", "STATUS sent")
                            st["ctrl_step"] = 3
                elif WITH_CONTROL and st["addressed"] and \
                        not st.get("pipes_open"):
                    st["pipes_open"] = True
                    for ep in EPS:
                        phase = rng.randint(0, PIPE_PHASE) \
                            if PIPE_PHASE else 0
                        grant_in_token(ep, earliest=cycle + phase)
                        eps[ep]["out_resume_at"] = cycle + \
                            (rng.randint(0, PIPE_PHASE) if PIPE_PHASE else 0)

                # protocol engines, per endpoint pair: keep the OUT pipes
                # saturated.  With WINDOW_KIB, mimic window_test.py: never
                # have more than WINDOW_KIB KiB unread in flight per pipe.
                for ep in (EPS if st["addressed"] else ()):
                    e = eps[ep]
                    window_ok = (not WINDOW_KIB or
                                 (e["out_idx"] + 1) * MPS
                                 - len(e["rx_bytes"])
                                 <= WINDOW_KIB * 1024)
                    if (not e["out_wait_ack"]
                            and e["out_idx"] < len(packets[ep])
                            and window_ok
                            and cycle >= e["out_resume_at"]
                            and not out_dp_queued(ep)
                            and len(hp_queue) < 1 + NUM_EPS
                                + (2 * NUM_EPS if URB_PACKETS else 0)):
                        send_next_out(ep, cycle)
                    # keep the IN pipe polled; with WINDOW_KIB the reader
                    # only starts once the initial window has been written
                    # -- or once the writer is flow-controlled (a concurrent
                    # reader, as in bandwidth_test.py or cdc-acm, is always
                    # running; only the single-threaded window_test.py truly
                    # serializes, and that case is covered on hardware by
                    # the deep loopback FIFO).
                    reads_started = (not WINDOW_KIB or
                                     e["out_idx"] * MPS >= WINDOW_KIB * 1024
                                     or e["out_nrdy"]
                                     or e["out_idx"] >= len(packets[ep]))
                    if (reads_started
                            and not e["in_parked"] and not e["in_token"]
                            and cycle >= e["in_resume_at"]
                            and not any(f.get("flow") == ("in", ep)
                                        for f in hp_queue)
                            and len(e["rx_bytes"]) < TOTAL_BYTES):
                        grant_in_token(ep)

                    # xHC no-response retry: a granted token with no DP
                    # for RETRY_TIMEOUT cycles is re-issued with the retry
                    # bit; three fruitless retries kill the pipe (-71).
                    if (RETRY_TIMEOUT and e["in_token"]
                            and e["tok_sent_at"] is not None
                            and cycle - e["tok_sent_at"] > RETRY_TIMEOUT):
                        e["tok_retries"] += 1
                        if e["tok_retries"] > 3:
                            fail(f"cycle {cycle}: ep{ep} IN pipe DEAD: no "
                                 f"DP after 3 no-response retries "
                                 f"(hardware -71 EPROTO signature)", None)
                        e["tok_sent_at"] = None
                        log.add(cycle, "host-tok-retry",
                                f"ep={ep} nseq={e['in_seq']} "
                                f"n={e['tok_retries']}")
                        hp_push(frame_ack_tp(ep=ep, nseq=e["in_seq"],
                                             nump=1, retry=1, direction=1),
                                ("in", ep))

                if not rx_words and rx_gap == 0:
                    refill_rx()
                    rx_gap = HOST_GAP if rx_words else 0
                words_streamed = getattr(host, "_wcount", 0)
                if rx_words and HOST_BUBBLES and \
                        words_streamed % HOST_BUBBLES == HOST_BUBBLES - 1:
                    # Model an RX bubble left behind by the SKP remover.
                    host._wcount = words_streamed + 1
                    ctx.set(phy.source.valid, 0)
                elif rx_words:
                    host._wcount = words_streamed + 1
                    w = rx_words.pop(0)
                    ctx.set(phy.source.valid, 1)
                    ctx.set(phy.source.data, w[0])
                    ctx.set(phy.source.ctrl, w[1])
                else:
                    if rx_gap > 0:
                        rx_gap -= 1
                    ctx.set(phy.source.valid, 0)
            else:
                ctx.set(phy.source.valid, 0)
                ctx.set(phy.raw_source.valid, 0)

            # ── completion / watchdogs ────────────────────────────────
            if all(len(eps[ep]["rx_bytes"]) >= TOTAL_BYTES
                   and eps[ep]["out_idx"] >= len(packets[ep])
                   for ep in EPS):
                st["done"] = True
                total = TOTAL_BYTES * NUM_EPS
                mbps = total / (cycle * 8e-9) / 1e6
                print(f"echoed {total} bytes across {NUM_EPS} endpoint "
                      f"pair(s) in {cycle} cycles "
                      f"({mbps:.1f} MB/s aggregate at the link layer)")
                print(f"stats: {_state_summary()}")
                for ep in EPS:
                    if bytes(eps[ep]["rx_bytes"]) != payloads[ep]:
                        fail(f"final echo comparison mismatch on ep{ep}")
                print("LINK LOOPBACK SIM PASS")
                return

            if cycle - st["progress_cycle"] > 80_000:
                fail(f"cycle {cycle}: DEADLOCK (no progress for 80k cycles)",
                     ctx)
            if cycle > 3_000_000:
                fail("global cycle limit exceeded", ctx)

    sim.add_testbench(host)
    if os.environ.get("WRITE_VCD"):
        os.makedirs("/tmp/kilo", exist_ok=True)
        with sim.write_vcd("/tmp/kilo/link_loopback.vcd"):
            sim.run()
    else:
        sim.run()


if __name__ == "__main__":
    main()
