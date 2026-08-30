"""SuperSpeed bulk OUT endpoint for LUNA (missing upstream).

LUNA's USB3 stack ships only ``SuperSpeedStreamInEndpoint``; this module
adds the receive direction so a device can accept host->device bulk
data.  Single-buffered, burst depth 1 (matches LUNA's ACK generator,
which always advertises NumP=1).

Protocol behaviour (USB 3.2r1 8.12.1.2, mirrored from the patterns in
``luna/gateware/usb/usb3/endpoints/{stream,control}.py``):

* data packets addressed to this endpoint are captured into a one-packet
  buffer as they stream in;
* on a CRC-valid packet with the expected 5-bit sequence number the
  payload is drained to :attr:`stream` FIRST and the ACK TP (with the
  advanced sequence number) is sent AFTER the drain -- with NumP=1 the
  host sends nothing further until that ACK, so the single buffer can
  never be overrun;
* a CRC-invalid packet, or one with an unexpected sequence number, is
  discarded and answered with ACK(rty=1, next=expected) so the host
  retransmits;
* zero-length packets are acknowledged without touching the stream;
* ``ep_reset`` (asserted by the device core on SET_CONFIGURATION)
  returns the sequence number and state machine to their defaults.
"""

from amaranth import *
from amaranth.lib.memory import Memory

from luna.gateware.usb.stream import SuperSpeedStreamInterface
from luna.gateware.usb.usb3.protocol.endpoint import SuperSpeedEndpointInterface


class SuperSpeedStreamOutEndpoint(Elaboratable):
    """ Endpoint interface that receives a bulk data stream from the host.

    Attributes
    ----------
    stream: SuperSpeedStreamInterface, output stream
        Received payload, one packet at a time (first/last delimited).
    interface: SuperSpeedEndpointInterface
        Communications link to our USB device.

    Parameters
    ----------
    endpoint_number: int
        The endpoint number (not address) this endpoint responds to.
    max_packet_size: int
        Maximum packet size; must match wMaxPacketSize (1024 for SS bulk).
    """

    SEQUENCE_NUMBER_BITS = 5

    def __init__(self, *, endpoint_number, max_packet_size=1024, max_burst=1):
        self._endpoint_number = endpoint_number
        self._max_packet_size = max_packet_size

        # Burst depth: the number of packet buffers -- and thus the
        # receive window (NumP) advertised to the host; the endpoint
        # descriptor's SuperSpeed companion should advertise
        # ``bMaxBurst = max_burst - 1``.  With the default of 1 the
        # historical single-buffered engine is used unchanged (ACK after
        # drain, generator-default NumP=1); larger values elaborate the
        # burst engine: packets are acknowledged IMMEDIATELY on
        # validation (with the remaining window in NumP), buffered in a
        # ring, and drained concurrently -- the host can keep up to
        # ``max_burst`` packets in flight.
        self._max_burst = max_burst

        # I/O port
        self.stream    = SuperSpeedStreamInterface()
        self.interface = SuperSpeedEndpointInterface()

        # Optional accept gate: a packet is only accepted (and drained)
        # when this is high at packet-completion time; otherwise it is
        # NRDY'd and the host retransmits after our ERDY.  When left
        # ``None`` the stream's own ``ready`` is used -- correct when the
        # sink is packet-buffered (SuperSpeedStreamInEndpoint), whose
        # ``ready`` implies a whole free packet buffer.  Drive it with a
        # "room for a full packet" level when the sink is a FIFO.
        self.packet_space = None

        # Debug taps (bring-up visibility; pruned when unused).
        self.debug_fill  = Signal(range(max_packet_size // 4 + 1))
        self.debug_total = Signal(range(max_packet_size // 4 + 1))
        self.debug_pos   = Signal(range(max_packet_size // 4 + 1))
        self.debug_fsm   = Signal(3)

    def elaborate(self, platform):
        if self._max_burst > 1:
            return self._elaborate_burst(platform)

        m = Module()

        interface      = self.interface
        stream         = self.stream
        rx             = interface.rx
        rx_header      = interface.rx_header
        handshakes_out = interface.handshakes_out

        # One packet buffer: 32-bit payload + 4 valid lanes per word.
        words = self._max_packet_size // 4
        m.submodules.buffer = buffer = Memory(shape=36, depth=words, init=[])
        wr = buffer.write_port(domain="ss")
        rd = buffer.read_port(domain="ss")

        # Expected data packet sequence number [USB3.2r1: 8.12.1.2].
        expected_seq = Signal(self.SEQUENCE_NUMBER_BITS)

        # Accept gate for completed packets (see ``packet_space`` above).
        packet_space = self.packet_space if self.packet_space is not None \
            else stream.ready

        # Is the packet currently on the rx interface for us?
        # (The DP header is fully parsed before payload words stream in.)
        is_our_packet = (
            (rx_header.endpoint_number == self._endpoint_number)
            & ~rx_header.direction                       # OUT = host->device
        )
        seq_matches = (rx_header.data_sequence == expected_seq)

        # Always tag our handshakes with our endpoint number -- and with the
        # OUT direction: NRDY/ERDY name the pipe they flow-control, and the
        # shared generator's default direction for them is IN.
        m.d.comb += [
            handshakes_out.endpoint_number .eq(self._endpoint_number),
            handshakes_out.direction       .eq(0),   # USBDirection.OUT
            handshakes_out.direction_valid .eq(1),
        ]

        # ── capture datapath ─────────────────────────────────────────────
        fill_count  = Signal(range(words + 1))      # words captured
        total_words = Signal(range(words + 1))      # committed packet size

        rx_word_valid = rx.valid.any() & is_our_packet
        m.d.comb += [
            wr.addr.eq(fill_count),
            wr.data.eq(Cat(rx.payload, rx.valid)),
            wr.en.eq(0),
        ]

        m.d.comb += [
            self.debug_fill.eq(fill_count),
            self.debug_total.eq(total_words),
        ]

        # ── drain sub-state ──────────────────────────────────────────────
        position = Signal(range(words + 1))
        fetched  = Signal()                          # rd.data matches position
        draining = Signal()                          # buffer busy being read
        m.d.comb += rd.addr.eq(position)

        m.d.comb += self.debug_pos.eq(position)
        last_word_of_drain = (position + 1 == total_words)

        # Capture runs whenever the buffer is not being drained -- NOT only
        # in the IDLE state: a packet's first word can arrive while the FSM
        # spends its single cycle in ACK, and a state-gated capture would
        # silently skip it (and then corrupt addressing via the stale fill
        # count).  A compliant host cannot send during DRAIN (it has no
        # credit until our ACK), so gating on ``draining`` alone is exact.
        with m.If(~draining & rx_word_valid):
            m.d.comb += wr.en.eq(1)
            with m.If(rx.first):
                m.d.comb += wr.addr.eq(0)
                m.d.ss   += fill_count.eq(1)
            with m.Else():
                m.d.ss   += fill_count.eq(fill_count + 1)

        with m.FSM(domain="ss") as rx_fsm:

            # IDLE -- adjudicate completed packets on the CRC strobes
            # (payload capture runs continuously outside the FSM).
            with m.State("IDLE"):

                # Packet concluded and CRC-valid.
                with m.If(interface.rx_complete & is_our_packet):

                    with m.If(seq_matches):

                        with m.If(fill_count != 0):
                            with m.If(packet_space):
                                # Sink can take the packet: hand the payload
                                # to the stream, then ACK.  (Paired with a
                                # packet-buffered sink like
                                # SuperSpeedStreamInEndpoint, ``ready`` here
                                # means a whole free packet buffer, so the
                                # drain below cannot stall mid-packet.)
                                m.d.ss += [
                                    expected_seq.eq(expected_seq + 1),
                                    total_words.eq(fill_count),
                                    position.eq(0),
                                    fetched.eq(0),
                                ]
                                m.next = "DRAIN"
                            with m.Else():
                                # Sink full (e.g. the echo path is backed up
                                # because the host hasn't read yet).  A DP
                                # must never be left unanswered -- the host
                                # retries it and errors the pipe out after
                                # ~35 ms (-EPROTO).  Flow-control instead:
                                # discard, answer NRDY, and reopen the pipe
                                # with ERDY once the sink drains; the host
                                # then retransmits this packet.
                                # [USB3.2r1: 8.10.1]
                                m.d.comb += handshakes_out.send_nrdy.eq(1)
                                m.d.ss   += fill_count.eq(0)
                                m.next = "AWAIT_SPACE"
                        with m.Else():
                            # Zero-length packet: just acknowledge it.
                            m.d.comb += [
                                handshakes_out.retry_required.eq(0),
                                handshakes_out.next_sequence
                                    .eq(expected_seq + 1),
                                handshakes_out.send_ack.eq(1),
                            ]
                            m.d.ss += [
                                expected_seq.eq(expected_seq + 1),
                                fill_count.eq(0),
                            ]

                    with m.Else():
                        # Unexpected sequence: discard, ask for the one we
                        # expect (host retransmits from there).
                        m.d.comb += [
                            handshakes_out.retry_required.eq(1),
                            handshakes_out.next_sequence.eq(expected_seq),
                            handshakes_out.send_ack.eq(1),
                        ]
                        m.d.ss += fill_count.eq(0)

                # Packet concluded with a bad CRC: discard and request retry.
                with m.If(interface.rx_invalid & is_our_packet):
                    m.d.comb += [
                        handshakes_out.retry_required.eq(1),
                        handshakes_out.next_sequence.eq(expected_seq),
                        handshakes_out.send_ack.eq(1),
                    ]
                    m.d.ss += fill_count.eq(0)

            # DRAIN -- stream the captured packet out of :attr:`stream`.
            # The host cannot send another packet yet (no ACK, NumP=1),
            # so the buffer is stable while we drain it.
            with m.State("DRAIN"):
                m.d.comb += draining.eq(1)

                # Wait one cycle for the (synchronous) read port.
                with m.If(~fetched):
                    m.d.ss += fetched.eq(1)

                with m.Else():
                    m.d.comb += [
                        stream.valid   .eq(rd.data[32:36]),
                        stream.payload .eq(rd.data[0:32]),
                        stream.first   .eq(position == 0),
                        stream.last    .eq(last_word_of_drain),
                    ]

                    # Word accepted: move to the next, or finish.
                    with m.If(stream.ready):
                        with m.If(last_word_of_drain):
                            m.d.ss += fill_count.eq(0)
                            m.next = "ACK"
                        with m.Else():
                            m.d.ss += [
                                position.eq(position + 1),
                                fetched.eq(0),
                            ]

            # ACK -- acknowledge the packet; this hands the host credit
            # for the next one.
            with m.State("ACK"):
                m.d.comb += [
                    handshakes_out.retry_required.eq(0),
                    handshakes_out.next_sequence.eq(expected_seq),
                    handshakes_out.send_ack.eq(1),
                ]
                m.next = "IDLE"

            # AWAIT_SPACE -- we NRDY'd a packet because our sink was full.
            # Wait for the sink to free a packet buffer, then reopen the
            # pipe: the host retransmits the discarded packet after ERDY.
            with m.State("AWAIT_SPACE"):
                with m.If(packet_space):
                    m.next = "SEND_ERDY"

            # SEND_ERDY -- request that the host resume this pipe.  Held
            # until the (shared) handshake generator confirms dispatch.
            with m.State("SEND_ERDY"):
                m.d.comb += handshakes_out.send_erdy.eq(1)
                with m.If(handshakes_out.done):
                    m.next = "IDLE"

        for _i, _name in enumerate(("IDLE", "DRAIN", "ACK", "AWAIT_SPACE",
                                    "SEND_ERDY")):
            with m.If(rx_fsm.ongoing(_name)):
                m.d.comb += self.debug_fsm.eq(_i)

        # Endpoint reset (SET_CONFIGURATION): back to sequence zero.
        # This is outside the FSM so it wins over any in-flight state.
        with m.If(interface.ep_reset):
            m.d.ss += [
                expected_seq.eq(0),
                fill_count.eq(0),
            ]

        return m


    def _elaborate_burst(self, platform):
        """ Burst-capable receive engine (``max_burst`` > 1).

        A ring of ``max_burst`` packet buffers decouples reception from
        draining:

          * CAPTURE runs continuously into ``buffer[wr_idx]`` (always a
            free buffer while the ring is not full);
          * on a CRC-valid, in-sequence packet the buffer is COMMITTED
            and the ACK goes out IMMEDIATELY -- before the drain -- with
            ``NumP`` advertising the remaining free buffers, so the host
            keeps sending without waiting for us to drain;
          * DRAIN streams committed buffers to :attr:`stream` in order,
            independently, freeing them as it goes;
          * an acknowledgement that reports no free buffers (NumP=0)
            parks the pipe; the ERDY that reopens it is sent as soon as
            a drain completes [USB3.2r1: 8.10.1].

        Retry semantics are unchanged: bad-CRC or out-of-sequence
        packets are discarded and answered with ACK(rty=1, expected).
        """
        m = Module()

        interface      = self.interface
        stream         = self.stream
        rx             = interface.rx
        rx_header      = interface.rx_header
        handshakes_out = interface.handshakes_out

        ring  = self._max_burst
        words = self._max_packet_size // 4

        buffers = [Memory(shape=36, depth=words, init=[]) for _ in range(ring)]
        wr_ports = Array(buf.write_port(domain="ss") for buf in buffers)
        rd_ports = Array(buf.read_port(domain="ss") for buf in buffers)
        for i, buf in enumerate(buffers):
            m.submodules[f"rx_buffer_{i}"] = buf

        total_words = Array(Signal(range(words + 1), name=f"total_words_{i}")
                            for i in range(ring))

        wr_idx   = Signal(range(ring))
        rd_idx   = Signal(range(ring))
        occupied = Signal(range(ring + 1))

        occ_inc, occ_dec = Signal(), Signal()
        with m.If(occ_inc & ~occ_dec):
            m.d.ss += occupied.eq(occupied + 1)
        with m.Elif(occ_dec & ~occ_inc):
            m.d.ss += occupied.eq(occupied - 1)

        def wrap_inc(sig):
            return Mux(sig == ring - 1, 0, sig + 1)

        # Free buffers AFTER a commit happening this cycle -- the value
        # advertised in that commit's ACK.
        free_after_commit = Signal(range(ring + 1))
        m.d.comb += free_after_commit.eq(ring - occupied - 1)

        # Expected data packet sequence number.
        expected_seq = Signal(self.SEQUENCE_NUMBER_BITS)

        # Set when we advertised a zero window (or NRDY'd): the host is
        # parked and must be re-opened with an ERDY once a drain frees a
        # buffer.
        erdy_needed = Signal()

        is_our_packet = (
            (rx_header.endpoint_number == self._endpoint_number)
            & ~rx_header.direction
        )
        seq_matches = (rx_header.data_sequence == expected_seq)

        # Handshake tagging: endpoint number, OUT direction, and our
        # advertised receive window on every ACK/ERDY.
        m.d.comb += [
            handshakes_out.endpoint_number          .eq(self._endpoint_number),
            handshakes_out.direction                .eq(0),
            handshakes_out.direction_valid          .eq(1),
            handshakes_out.number_of_packets_valid  .eq(1),
        ]

        # ── capture datapath ─────────────────────────────────────────
        fill_count = Signal(range(words + 1))
        can_capture = (occupied != ring)

        # A packet that lost ANY word to a full ring is unusable: the
        # ring can become non-full MID-PACKET (a drain completing frees
        # a buffer while the DP is still streaming in), after which a
        # naive occupancy check at completion time would commit the
        # captured TAIL as a complete packet -- or, for a packet dropped
        # entirely, treat ``fill_count == 0`` as a phantom ZLP and
        # advance the sequence number over 1024 vanished bytes (bug #35,
        # reproduced by the OUT_WINDOW0 blast stimulus: the bench xHC
        # pipelines its whole scheduling window at transfer start).
        # Latch the loss; adjudication below then takes the same
        # NRDY-and-park path as the ring-full case, and the host
        # retransmits after our ERDY.
        packet_lost = Signal()

        rx_word_valid = rx.valid.any() & is_our_packet
        # Whether THIS word may be captured: a first word restarts the
        # packet's fate regardless of a stale loss latch (which
        # adjudication normally consumes, but a DPP truncated by a
        # retrain never completes and would leave it set).
        word_ok = can_capture & (rx.first | ~packet_lost)
        for i in range(ring):
            m.d.comb += [
                wr_ports[i].data.eq(Cat(rx.payload, rx.valid)),
                wr_ports[i].addr.eq(Mux(rx.first, 0, fill_count)),
                wr_ports[i].en  .eq((wr_idx == i) & word_ok
                                    & rx_word_valid),
            ]
        with m.If(rx_word_valid):
            with m.If(rx.first):
                m.d.ss += packet_lost.eq(~can_capture)
                with m.If(can_capture):
                    m.d.ss += fill_count.eq(1)
            with m.Elif(~can_capture):
                m.d.ss += packet_lost.eq(1)
            with m.Elif(~packet_lost):
                m.d.ss += fill_count.eq(fill_count + 1)

        m.d.comb += [
            self.debug_fill .eq(fill_count),
            self.debug_total.eq(total_words[rd_idx]),
        ]

        # ── packet adjudication (strobe-driven; no FSM needed) ───────
        with m.If(interface.rx_complete & is_our_packet):

            # The loss latch is consumed by every outcome below.
            m.d.ss += packet_lost.eq(0)

            with m.If(seq_matches & (occupied != ring) & ~packet_lost):

                with m.If(fill_count != 0):
                    # Commit the packet and acknowledge it immediately;
                    # the drain below catches up on its own.
                    m.d.comb += [
                        occ_inc.eq(1),
                        handshakes_out.retry_required    .eq(0),
                        handshakes_out.next_sequence     .eq(expected_seq + 1),
                        handshakes_out.number_of_packets .eq(free_after_commit),
                        handshakes_out.send_ack          .eq(1),
                    ]
                    m.d.ss += [
                        total_words[wr_idx].eq(fill_count),
                        wr_idx             .eq(wrap_inc(wr_idx)),
                        expected_seq       .eq(expected_seq + 1),
                        fill_count         .eq(0),
                    ]
                    # A zero advertisement parks the host.
                    with m.If(free_after_commit == 0):
                        m.d.ss += erdy_needed.eq(1)

                with m.Else():
                    # Zero-length packet: acknowledge without committing.
                    m.d.comb += [
                        handshakes_out.retry_required    .eq(0),
                        handshakes_out.next_sequence     .eq(expected_seq + 1),
                        handshakes_out.number_of_packets .eq(ring - occupied),
                        handshakes_out.send_ack          .eq(1),
                    ]
                    m.d.ss += expected_seq.eq(expected_seq + 1)

            with m.Elif(seq_matches):
                # In-sequence packet, but the ring was full -- either
                # still full now, or full when any of its words arrived
                # (``packet_lost``: the capture is a partial tail, or
                # nothing at all -- bug #35).  The host overran our
                # advertisement or raced our ACK: discard and park the
                # pipe; the ERDY after the next drain reopens it and the
                # host retransmits this packet.
                m.d.comb += handshakes_out.send_nrdy.eq(1)
                m.d.ss += [
                    fill_count .eq(0),
                    erdy_needed.eq(1),
                ]

            with m.Else():
                # Unexpected sequence: discard, ask for the expected one.
                m.d.comb += [
                    handshakes_out.retry_required    .eq(1),
                    handshakes_out.next_sequence     .eq(expected_seq),
                    handshakes_out.number_of_packets .eq(ring - occupied),
                    handshakes_out.send_ack          .eq(1),
                ]
                m.d.ss += fill_count.eq(0)

        # Bad CRC: discard and request retry.
        with m.If(interface.rx_invalid & is_our_packet):
            m.d.comb += [
                handshakes_out.retry_required    .eq(1),
                handshakes_out.next_sequence     .eq(expected_seq),
                handshakes_out.number_of_packets .eq(ring - occupied),
                handshakes_out.send_ack          .eq(1),
            ]
            m.d.ss += [
                fill_count .eq(0),
                packet_lost.eq(0),
            ]

        # ── drain engine ─────────────────────────────────────────────
        position = Signal(range(words + 1))
        fetched  = Signal()
        rd       = rd_ports[rd_idx]
        m.d.comb += [
            rd.addr.eq(position),
            self.debug_pos.eq(position),
        ]
        last_word_of_drain = (position + 1 == total_words[rd_idx])

        with m.FSM(domain="ss") as drain_fsm:

            with m.State("IDLE"):
                with m.If(occupied != 0):
                    m.d.ss += [position.eq(0), fetched.eq(0)]
                    m.next = "DRAIN"

            with m.State("DRAIN"):
                with m.If(~fetched):
                    m.d.ss += fetched.eq(1)
                with m.Else():
                    m.d.comb += [
                        stream.valid   .eq(rd.data[32:36]),
                        stream.payload .eq(rd.data[0:32]),
                        stream.first   .eq(position == 0),
                        stream.last    .eq(last_word_of_drain),
                    ]
                    with m.If(stream.ready):
                        with m.If(last_word_of_drain):
                            m.d.comb += occ_dec.eq(1)
                            m.d.ss += rd_idx.eq(wrap_inc(rd_idx))
                            # Reopen a parked pipe now that a buffer is
                            # free.  (Fire-and-forget: the handshake mux
                            # latches strobes losslessly.)
                            with m.If(erdy_needed):
                                m.d.comb += [
                                    handshakes_out.number_of_packets
                                        .eq(ring - occupied + 1),
                                    handshakes_out.send_erdy.eq(1),
                                ]
                                m.d.ss += erdy_needed.eq(0)
                            m.next = "IDLE"
                        with m.Else():
                            m.d.ss += [
                                position.eq(position + 1),
                                fetched .eq(0),
                            ]

        for _i, _name in enumerate(("IDLE", "DRAIN")):
            with m.If(drain_fsm.ongoing(_name)):
                m.d.comb += self.debug_fsm.eq(_i)

        # Endpoint reset (SET_CONFIGURATION): back to a clean ring.
        with m.If(interface.ep_reset):
            m.d.ss += [
                expected_seq.eq(0),
                fill_count  .eq(0),
                wr_idx      .eq(0),
                rd_idx      .eq(0),
                occupied    .eq(0),
                erdy_needed .eq(0),
                packet_lost .eq(0),
            ]

        return m
