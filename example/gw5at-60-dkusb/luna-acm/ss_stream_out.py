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

    def __init__(self, *, endpoint_number, max_packet_size=1024):
        self._endpoint_number = endpoint_number
        self._max_packet_size = max_packet_size

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

    def elaborate(self, platform):
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

        with m.FSM(domain="ss"):

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

        # Endpoint reset (SET_CONFIGURATION): back to sequence zero.
        # This is outside the FSM so it wins over any in-flight state.
        with m.If(interface.ep_reset):
            m.d.ss += [
                expected_seq.eq(0),
                fill_count.eq(0),
            ]

        return m
