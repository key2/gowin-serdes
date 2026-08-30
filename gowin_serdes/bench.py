"""Bench debug helpers shared by the SerDes/USB3 example tops.

Contents are the proven bench utilities from the vendor-stack
enumeration example (``example/gw5at-60-dkusb/usb31-enum``), promoted
into the package so out-of-tree consumers (the luna-ss fork's
hardware examples) can import them without sys.path reaches:

* ``AsyncSerialRX``/``AsyncSerialTX``/``AsyncSerial`` (+ stream
  variant) -- the debug-UART primitives (115200 8N1 on the bench).
* ``ClockFreqProbe`` -- frequency counter for SerDes-derived clocks,
  reported over UART (elaborates in a ``cfg`` domain; DomainRenamer
  onto a stable reference clock -- the tops run it on the 24 MHz
  oscillator).

The usb31-enum example keeps its local copies on purpose: it is the
untouched vendor-stack A/B baseline (HANDOVER 10c); do not refactor
it.  Fix bugs here and there in lockstep if any ever surface.
"""

from amaranth import *
from amaranth.lib import enum, data, wiring
from amaranth.lib.wiring import In, Out, connect, flipped
from amaranth.lib.cdc import FFSynchronizer
from amaranth.utils import bits_for


__all__ = [
    "Parity",
    "AsyncSerialRX",
    "AsyncSerialTX",
    "AsyncSerial",
    "StreamAsyncSerialTX",
    "ClockFreqProbe",
]


class Parity(enum.Enum):
    """Asynchronous serial parity mode."""

    NONE = "none"
    MARK = "mark"
    SPACE = "space"
    EVEN = "even"
    ODD = "odd"

    def _compute_bit(self, data):
        cast_data = Value.cast(data)
        if self == self.NONE:
            return Const(0, 0)
        if self == self.MARK:
            return Const(1, 1)
        if self == self.SPACE:
            return Const(0, 1)
        if self == self.EVEN:
            return cast_data.xor()
        if self == self.ODD:
            return ~cast_data.xor()
        assert False  # :nocov:


class _FrameLayout(data.StructLayout):
    def __init__(self, data_bits, parity):
        super().__init__(
            {
                "start": unsigned(1),
                "data": unsigned(data_bits),
                "parity": unsigned(0 if parity == Parity.NONE else 1),
                "stop": unsigned(1),
            }
        )


class AsyncSerialRX(wiring.Component):
    class Signature(wiring.Signature):
        """Asynchronous serial receiver signature.

        Parameters
        ----------
        divisor : int
            Clock divisor initial value. Should be set to ``int(clk_frequency // baudrate)``.
        divisor_bits : int
            Clock divisor width. Optional. If omitted, ``bits_for(divisor)`` is used instead.
        data_bits : int
            Data bits per frame.
        parity : :class:`Parity`
            Parity mode.

        Interface attributes
        --------------------
        divisor : Signal, in
            Clock divisor.
        data : Signal, out
            Read data. Valid only when ``rdy`` is asserted.
        err.overflow : Signal, out
            Error flag. A new frame has been received, but the previous one was not acknowledged.
        err.frame : Signal, out
            Error flag. The received bits do not fit in a frame.
        err.parity : Signal, out
            Error flag. The parity check has failed.
        rdy : Signal, out
            Read strobe.
        ack : Signal, in
            Read acknowledge. Must be held asserted while data can be read out of the receiver.
        i : Signal, in
            Serial input.

        Raises
        ------
        See :meth:`AsyncSerialRX.Signature.check_parameters`.
        """

        def __init__(self, *, divisor, divisor_bits=None, data_bits=8, parity="none"):
            self.check_parameters(
                divisor=divisor,
                divisor_bits=divisor_bits,
                data_bits=data_bits,
                parity=parity,
            )
            self._divisor = divisor
            self._divisor_bits = (
                divisor_bits if divisor_bits is not None else bits_for(divisor)
            )
            self._data_bits = data_bits
            self._parity = Parity(parity)

            super().__init__(
                {
                    "divisor": In(unsigned(self._divisor_bits), init=self._divisor),
                    "data": Out(unsigned(self._data_bits)),
                    "err": Out(
                        data.StructLayout({"overflow": 1, "frame": 1, "parity": 1})
                    ),
                    "rdy": Out(unsigned(1)),
                    "ack": In(unsigned(1)),
                    "i": In(unsigned(1), init=1),
                }
            )

        @classmethod
        def check_parameters(
            cls, *, divisor, divisor_bits=None, data_bits=8, parity="none"
        ):
            """Validate signature parameters.

            Raises
            ------
            :exc:`TypeError`
                If ``divisor`` is not an integer greater than or equal to 5.
            :exc:`TypeError`
                If ``divisor_bits`` is not `None` and not an integer greater than or equal to
                ``bits_for(divisor)``.
            :exc:`TypeError`
                If ``data_bits`` is not an integer greater than or equal to 0.
            :exc:`ValueError`
                If ``parity`` is not a :class:`Parity` member.
            """
            # The clock divisor must be >= 5 to keep the receiver FSM synchronized with its input
            # during a DONE->IDLE->BUSY transition.
            AsyncSerial.Signature._check_divisor(divisor, divisor_bits, min_divisor=5)
            if not isinstance(data_bits, int) or data_bits < 0:
                raise TypeError(
                    f"Data bits must be a non-negative integer, not {data_bits!r}"
                )
            # Raise a ValueError if parity is invalid.
            Parity(parity)

        @property
        def divisor(self):
            return self._divisor

        @property
        def divisor_bits(self):
            return self._divisor_bits

        @property
        def data_bits(self):
            return self._data_bits

        @property
        def parity(self):
            return self._parity

        def __eq__(self, other):
            """Compare signatures.

            Two signatures are equal if they have the same divisor value, divisor bits,
            data bits, and parity.
            """
            return (
                isinstance(other, AsyncSerialRX.Signature)
                and self.divisor == other.divisor
                and self.divisor_bits == other.divisor_bits
                and self.data_bits == other.data_bits
                and self.parity == other.parity
            )

        def __repr__(self):
            return f"AsyncSerialRX.Signature({self.members!r})"

    """Asynchronous serial receiver.

    Parameters
    ----------
    divisor : int
        Clock divisor initial value. Should be set to ``int(clk_frequency // baudrate)``.
    divisor_bits : int
        Clock divisor width. Optional. If omitted, ``bits_for(divisor)`` is used instead.
    data_bits : int
        Data bits per frame.
    parity : :class:`Parity`
        Parity mode.
    pins : :class:`amaranth.lib.io.Pin`
        UART pins. Optional. See :class:`amaranth_boards.resources.UARTResource` for layout.
        If provided, the ``i`` port of the receiver is internally connected to ``pins.rx.i``.

    Raises
    ------
    See :meth:`AsyncSerialRX.Signature.check_parameters`.
    """

    def __init__(
        self, *, divisor, divisor_bits=None, data_bits=8, parity="none", pins=None
    ):
        super().__init__(
            self.Signature(
                divisor=divisor,
                divisor_bits=divisor_bits,
                data_bits=data_bits,
                parity=parity,
            )
        )
        self._pins = pins

    def elaborate(self, platform):
        m = Module()

        timer = Signal.like(self.divisor)
        shreg = Signal(_FrameLayout(len(self.data), self.signature.parity))
        bitno = Signal(range(len(shreg.as_value())))

        if self._pins is not None:
            m.submodules += FFSynchronizer(self._pins.rx.i, self.i, init=1)

        with m.FSM() as fsm:
            with m.State("IDLE"):
                with m.If(~self.i):
                    m.d.sync += [
                        bitno.eq(len(shreg.as_value()) - 1),
                        timer.eq(self.divisor >> 1),
                    ]
                    m.next = "BUSY"

            with m.State("BUSY"):
                with m.If(timer != 0):
                    m.d.sync += timer.eq(timer - 1)
                with m.Else():
                    m.d.sync += [
                        shreg.eq(Cat(shreg.as_value()[1:], self.i)),
                        bitno.eq(bitno - 1),
                        timer.eq(self.divisor - 1),
                    ]
                    with m.If(bitno == 0):
                        m.next = "DONE"

            with m.State("DONE"):
                with m.If(self.ack):
                    m.d.sync += [
                        self.data.eq(shreg.data),
                        self.err.frame.eq(~((shreg.start == 0) & (shreg.stop == 1))),
                        self.err.parity.eq(
                            ~(
                                shreg.parity
                                == self.signature.parity._compute_bit(shreg.data)
                            )
                        ),
                    ]
                m.d.sync += self.err.overflow.eq(~self.ack)
                m.next = "IDLE"

        with m.If(self.ack):
            m.d.sync += self.rdy.eq(fsm.ongoing("DONE"))

        return m


class AsyncSerialTX(wiring.Component):
    class Signature(wiring.Signature):
        """Asynchronous serial transmitter signature.

        Parameters
        ----------
        divisor : int
            Clock divisor initial value. Should be set to ``int(clk_frequency // baudrate)``.
        divisor_bits : int
            Clock divisor width. Optional. If omitted, ``bits_for(divisor)`` is used instead.
        data_bits : int
            Data bits per frame.
        parity : :class:`Parity`
            Parity mode.

        Interface attributes
        --------------------
        divisor : Signal, in
            Clock divisor.
        data : Signal, in
            Write data. Valid only when ``ack`` is asserted.
        rdy : Signal, out
            Write ready. Asserted when the transmitter is ready to transmit data.
        ack : Signal, in
            Write strobe. Data gets transmitted when both ``rdy`` and ``ack`` are asserted.
        o : Signal, out
            Serial output.

        Raises
        ------
        See :meth:`AsyncSerialTX.Signature.check_parameters`.
        """

        def __init__(self, *, divisor, divisor_bits=None, data_bits=8, parity="none"):
            self.check_parameters(
                divisor=divisor,
                divisor_bits=divisor_bits,
                data_bits=data_bits,
                parity=parity,
            )
            self._divisor = divisor
            self._divisor_bits = (
                divisor_bits if divisor_bits is not None else bits_for(divisor)
            )
            self._data_bits = data_bits
            self._parity = Parity(parity)

            super().__init__(
                {
                    "divisor": In(unsigned(self._divisor_bits), init=self._divisor),
                    "data": In(unsigned(self._data_bits)),
                    "rdy": Out(unsigned(1)),
                    "ack": In(unsigned(1)),
                    "o": Out(unsigned(1), init=1),
                }
            )

        @classmethod
        def check_parameters(
            cls, *, divisor, divisor_bits=None, data_bits=8, parity="none"
        ):
            """Validate signature parameters.

            Raises
            ------
            :exc:`TypeError`
                If ``divisor`` is not an integer greater than or equal to 1.
            :exc:`TypeError`
                If ``divisor_bits`` is not `None` and not an integer greater than or equal to
                ``bits_for(divisor)``.
            :exc:`TypeError`
                If ``data_bits`` is not an integer greater than or equal to 0.
            :exc:`ValueError`
                If ``parity`` is not a :class:`Parity` member.
            """
            AsyncSerial.Signature._check_divisor(divisor, divisor_bits, min_divisor=1)
            if not isinstance(data_bits, int) or data_bits < 0:
                raise TypeError(
                    f"Data bits must be a non-negative integer, not {data_bits!r}"
                )
            # Raise a ValueError if parity is invalid.
            Parity(parity)

        @property
        def divisor(self):
            return self._divisor

        @property
        def divisor_bits(self):
            return self._divisor_bits

        @property
        def data_bits(self):
            return self._data_bits

        @property
        def parity(self):
            return self._parity

        def __eq__(self, other):
            """Compare signatures.

            Two signatures are equal if they have the same divisor value, divisor bits,
            data bits, and parity.
            """
            return (
                isinstance(other, AsyncSerialTX.Signature)
                and self.divisor == other.divisor
                and self.divisor_bits == other.divisor_bits
                and self.data_bits == other.data_bits
                and self.parity == other.parity
            )

        def __repr__(self):
            return f"AsyncSerialTX.Signature({self.members!r})"

    """Asynchronous serial transmitter.

    Parameters
    ----------
    divisor : int
        Clock divisor initial value. Should be set to ``int(clk_frequency // baudrate)``.
    divisor_bits : int
        Clock divisor width. Optional. If omitted, ``bits_for(divisor)`` is used instead.
    data_bits : int
        Data bits per frame.
    parity : :class:`Parity`
        Parity mode.
    pins : :class:`amaranth.lib.io.Pin`
        UART pins. Optional. See :class:`amaranth_boards.resources.UARTResource` for layout.
        If provided, the ``o`` port of the transmitter is internally connected to ``pins.tx.o``.

    Raises
    ------
    See :class:`AsyncSerialTX.Signature.check_parameters`.
    """

    def __init__(
        self, *, divisor, divisor_bits=None, data_bits=8, parity="none", pins=None
    ):
        super().__init__(
            signature=self.Signature(
                divisor=divisor,
                divisor_bits=divisor_bits,
                data_bits=data_bits,
                parity=parity,
            )
        )
        self._pins = pins

    def elaborate(self, platform):
        m = Module()

        timer = Signal.like(self.divisor)
        shreg = Signal(_FrameLayout(len(self.data), self.signature.parity))
        bitno = Signal(range(len(shreg.as_value())))

        if self._pins is not None:
            m.d.comb += self._pins.tx.o.eq(self.o)

        with m.FSM():
            with m.State("IDLE"):
                m.d.comb += self.rdy.eq(1)
                with m.If(self.ack):
                    m.d.sync += [
                        shreg.start.eq(0),
                        shreg.data.eq(self.data),
                        shreg.parity.eq(self.signature.parity._compute_bit(self.data)),
                        shreg.stop.eq(1),
                        bitno.eq(len(shreg.as_value()) - 1),
                        timer.eq(self.divisor - 1),
                    ]
                    m.next = "BUSY"

            with m.State("BUSY"):
                with m.If(timer != 0):
                    m.d.sync += timer.eq(timer - 1)
                with m.Else():
                    m.d.sync += [
                        Cat(self.o, shreg).eq(shreg),
                        bitno.eq(bitno - 1),
                        timer.eq(self.divisor - 1),
                    ]
                    with m.If(bitno == 0):
                        m.next = "IDLE"

        return m


class AsyncSerial(wiring.Component):
    class Signature(wiring.Signature):
        """Asynchronous serial transceiver signature.

        Parameters
        ----------
        divisor : int
            Clock divisor initial value. Should be set to ``int(clk_frequency // baudrate)``.
        divisor_bits : int
            Clock divisor width. Optional. If omitted, ``bits_for(divisor)`` is used instead.
        data_bits : int
            Data bits per frame.
        parity : :class:`Parity`
            Parity mode.

        Interface attributes
        --------------------
        divisor : Signal, in
            Clock divisor. It is internally connected to ``rx.divisor`` and ``tx.divisor``.
        rx : :class:`wiring.Interface`
            Receiver interface. See :class:`AsyncSerialRX.Signature`.
        tx : :class:`wiring.Interface`
            Transmitter interface. See :class:`AsyncSerialTX.Signature`.

        Raises
        ------
        See :meth:`AsyncSerial.Signature.check_parameters`.
        """

        def __init__(self, *, divisor, divisor_bits=None, data_bits=8, parity="none"):
            rx_sig = AsyncSerialRX.Signature(
                divisor=divisor,
                divisor_bits=divisor_bits,
                data_bits=data_bits,
                parity=parity,
            )
            tx_sig = AsyncSerialTX.Signature(
                divisor=divisor,
                divisor_bits=divisor_bits,
                data_bits=data_bits,
                parity=parity,
            )

            assert rx_sig.members["divisor"] == tx_sig.members["divisor"]
            divisor_shape = rx_sig.members["divisor"].shape
            divisor_init = rx_sig.members["divisor"].init

            super().__init__(
                {
                    "divisor": In(divisor_shape, init=divisor_init),
                    "rx": Out(rx_sig),
                    "tx": Out(tx_sig),
                }
            )

        @classmethod
        def _check_divisor(cls, divisor, divisor_bits, min_divisor=1):
            if not isinstance(divisor, int) or divisor < min_divisor:
                raise TypeError(
                    f"Divisor initial value must be an integer greater than or equal "
                    f"to {min_divisor}, not {divisor!r}"
                )
            if divisor_bits is not None:
                min_divisor_bits = bits_for(divisor)
                if not isinstance(divisor_bits, int) or divisor_bits < min_divisor_bits:
                    raise TypeError(
                        f"Divisor bits must be an integer greater than or equal to "
                        f"{min_divisor_bits}, not {divisor_bits!r}"
                    )

        @classmethod
        def check_parameters(
            cls, *, divisor, divisor_bits=None, data_bits=8, parity="none"
        ):
            """Validate signature parameters.

            Raises
            ------
            :exc:`TypeError`
                If ``divisor`` is not an integer greater than or equal to 5.
            :exc:`TypeError`
                If ``divisor_bits`` is not `None` and not an integer greater than or equal to
                ``bits_for(divisor)``.
            :exc:`TypeError`
                If ``data_bits`` is not an integer greater than or equal to 0.
            :exc:`ValueError`
                If ``parity`` is not a :class:`Parity` member.
            """
            AsyncSerialRX.Signature.check_parameters(
                divisor=divisor,
                divisor_bits=divisor_bits,
                data_bits=data_bits,
                parity=parity,
            )
            AsyncSerialTX.Signature.check_parameters(
                divisor=divisor,
                divisor_bits=divisor_bits,
                data_bits=data_bits,
                parity=parity,
            )

        @property
        def divisor(self):
            return self.members["rx"].signature.divisor

        @property
        def divisor_bits(self):
            return self.members["rx"].signature.divisor_bits

        @property
        def data_bits(self):
            return self.members["rx"].signature.data_bits

        @property
        def parity(self):
            return self.members["rx"].signature.parity

        def __eq__(self, other):
            """Compare signatures.

            Two signatures are equal if they have the same divisor value, divisor bits,
            data bits, and parity.
            """
            return (
                isinstance(other, AsyncSerial.Signature)
                and self.divisor == other.divisor
                and self.divisor_bits == other.divisor_bits
                and self.data_bits == other.data_bits
                and self.parity == other.parity
            )

        def __repr__(self):
            return f"AsyncSerial.Signature({self.members!r})"

    """Asynchronous serial transceiver.

    Parameters
    ----------
    divisor : int
        Clock divisor initial value. Should be set to ``int(clk_frequency // baudrate)``.
    divisor_bits : int
        Clock divisor width. Optional. If omitted, ``bits_for(divisor)`` is used instead.
    data_bits : int
        Data bits per frame.
    parity : :class:`Parity`
        Parity mode.
    pins : :class:`amaranth.lib.io.Pin`
        UART pins. Optional. See :class:`amaranth_boards.resources.UARTResource` for layout.
        If provided, the ``rx.i`` and ``tx.o`` ports of the transceiver are internally connected
        to ``pins.rx.i`` and ``pins.tx.o``, respectively.

    Raises
    ------
    See :meth:`AsyncSerial.Signature.check_parameters`.
    """

    def __init__(
        self, *, divisor, divisor_bits=None, data_bits=8, parity="none", pins=None
    ):
        super().__init__(
            self.Signature(
                divisor=divisor,
                divisor_bits=divisor_bits,
                data_bits=data_bits,
                parity=parity,
            )
        )
        self._pins = pins

    def elaborate(self, platform):
        m = Module()

        rx = AsyncSerialRX(
            divisor=self.signature.divisor,
            divisor_bits=self.signature.divisor_bits,
            data_bits=self.signature.data_bits,
            parity=self.signature.parity,
        )
        tx = AsyncSerialTX(
            divisor=self.signature.divisor,
            divisor_bits=self.signature.divisor_bits,
            data_bits=self.signature.data_bits,
            parity=self.signature.parity,
        )
        m.submodules.rx = rx
        m.submodules.tx = tx

        m.d.comb += [
            self.rx.divisor.eq(self.divisor),
            self.tx.divisor.eq(self.divisor),
        ]

        if self._pins is not None:
            m.submodules += FFSynchronizer(self._pins.rx.i, self.rx.i, init=1)
            m.d.comb += self._pins.tx.o.eq(self.tx.o)

        connect(m, flipped(self.rx), rx)
        connect(m, flipped(self.tx), tx)

        return m


class StreamAsyncSerialTX(Elaboratable):
    """Stream-interfaced asynchronous serial transmitter.

    Wraps :class:`AsyncSerialTX` with an ``amaranth_stream`` valid/ready
    input interface so it can be driven directly from stream pipelines.

    Parameters
    ----------
    divisor : int
        Clock divisor: ``round(clk_freq / baud_rate)``.
    data_bits : int
        Data width per frame (default 8).
    parity : str
        Parity mode (default ``"none"``).

    Attributes
    ----------
    sink : :class:`amaranth_stream.Interface`
        8-bit (or ``data_bits``-wide) stream input.
        A byte is accepted when ``sink.valid & sink.ready``.
    tx_o : :class:`Signal`
        Serial TX output (active-high idle, active-low start bit).
    """

    def __init__(self, *, divisor, data_bits=8, parity="none"):
        from amaranth_stream import Signature as StreamSignature

        self.sink = StreamSignature(data_bits).create()
        self.tx_o = Signal(init=1)
        self._divisor = divisor
        self._data_bits = data_bits
        self._parity = parity

    def elaborate(self, platform):
        m = Module()

        tx = AsyncSerialTX(
            divisor=self._divisor,
            data_bits=self._data_bits,
            parity=self._parity,
        )
        m.submodules.tx = tx

        # Stream valid/ready ↔ AsyncSerialTX rdy/ack mapping:
        #   stream.ready = tx.rdy  (TX idle → can accept)
        #   tx.ack       = stream.valid & stream.ready  (transfer)
        #   tx.data      = stream.payload
        m.d.comb += [
            self.sink.ready.eq(tx.rdy),
            tx.ack.eq(self.sink.valid & tx.rdy),
            tx.data.eq(self.sink.payload),
            self.tx_o.eq(tx.o),
        ]

        return m


class ClockFreqProbe(Elaboratable):
    """Frequency counter for SerDes-derived clocks, reported over UART.

    Elaborates in a domain called ``cfg``; rename with ``DomainRenamer``
    to place it on any stable reference clock (the enum top runs it on
    the 24 MHz oscillator).  Each measured clock drives a free-running
    counter snapshot via a request/ack handshake CDC; every
    2^gate_bits reference cycles the deltas are printed as
    ``C <pclk> <rxclk> <upar> <flags>\\r\\n`` (7 hex digits each).
    f[Hz] = delta * f_ref / 2^gate_bits.

    Rationale: the upar/DRP "life clock" of the GTR12 was measured to be
    a free-running ring oscillator (~56..118 MHz); this probe measured it
    (and shows the pclk 156.25 -> 125 MHz rate switch live).
    """

    WIDTH = 28

    def __init__(self, clk_freq, baud=115_200, gate_bits=23,
                 channels=(("pclk", None), ("rxprobe", None),
                           ("upar", None))):
        """`channels`: (domain, event) pairs.  event=None counts clock
        cycles (frequency); an event Signal counts its assertions in that
        domain (event rate)."""
        self._divisor = clk_freq // baud
        self._gate_bits = gate_bits
        self._channels = list(channels)
        self.flags = Signal(4)          # cfg domain; appended as hex digit
        self.tx_o = Signal(init=1)

    def elaborate(self, platform):
        m = Module()

        tx = AsyncSerialTX(divisor=self._divisor)
        m.submodules.tx = DomainRenamer("cfg")(tx)
        m.d.comb += self.tx_o.eq(tx.o)

        # -- per-clock counters, capture-and-hold handshake CDC ----------
        # A request toggle (cfg) makes each source domain latch its
        # free-running counter into a holding register and answer with an
        # ack toggle; cfg then reads the *stable* snapshot.  No decode
        # logic runs at speed and no gray-skew assumptions are needed.
        deltas = []
        gate = Signal(self._gate_bits)
        m.d.cfg += gate.eq(gate + 1)
        snapshot = gate == 0            # ~0.35 s at 24 MHz / 23 bits
        req = Signal()
        with m.If(snapshot):
            m.d.cfg += req.eq(~req)

        for ci, (dom, event) in enumerate(self._channels):
            tag = f"ch{ci}_{dom}"
            cnt = Signal(self.WIDTH, name=f"cnt_{tag}")
            if event is None:
                m.d[dom] += cnt.eq(cnt + 1)
            else:
                with m.If(event):
                    m.d[dom] += cnt.eq(cnt + 1)

            req_s = Signal(name=f"req_{tag}_s")
            m.submodules += FFSynchronizer(req, req_s, o_domain=dom)
            req_d = Signal(name=f"req_{tag}_d")
            snap = Signal(self.WIDTH, name=f"snap_{tag}")
            ack = Signal(name=f"ack_{tag}")
            m.d[dom] += req_d.eq(req_s)
            with m.If(req_s != req_d):
                m.d[dom] += [snap.eq(cnt), ack.eq(~ack)]

            ack_s = Signal(name=f"ack_{tag}_s")
            m.submodules += FFSynchronizer(ack, ack_s, o_domain="cfg")
            ack_d = Signal(name=f"ack_{tag}_d")
            m.d.cfg += ack_d.eq(ack_s)
            last = Signal(self.WIDTH, name=f"last_{tag}")
            delta = Signal(self.WIDTH, name=f"delta_{tag}")
            pending = Signal(name=f"pending_{tag}")
            with m.If(snapshot):
                m.d.cfg += pending.eq(1)
                with m.If(pending):     # no ack since last request:
                    m.d.cfg += delta.eq(0)   # the clock is dead -> read 0
            with m.If(ack_s != ack_d):
                # snap has been stable for >= 2 cfg cycles (synchronizer
                # latency); safe multi-bit read.
                m.d.cfg += [last.eq(snap), delta.eq(snap - last),
                            pending.eq(0)]
            deltas.append(delta)

        # -- line formatter: 'C' + one 7-digit hex group per clock,
        #    then ' ' + one flags digit -------------------------------
        flags_l = Signal(4)
        with m.If(snapshot):
            m.d.cfg += flags_l.eq(self.flags)

        DIGITS = self.WIDTH // 4
        n = len(deltas)
        per = 1 + DIGITS                      # ' ' + digits
        total = 1 + n * per + 2 + 2           # 'C' + groups + ' '+flag + CR LF
        idx = Signal(range(total))
        active = Signal()

        char = Signal(8)
        with m.Switch(idx):
            with m.Case(0):
                m.d.comb += char.eq(ord("C"))
            for gi, delta in enumerate(deltas):
                base = 1 + gi * per
                with m.Case(base):
                    m.d.comb += char.eq(ord(" "))
                for j in range(DIGITS):
                    nib = delta[(DIGITS - 1 - j) * 4:(DIGITS - j) * 4]
                    with m.Case(base + 1 + j):
                        m.d.comb += char.eq(
                            Mux(nib < 10, ord("0") + nib,
                                ord("a") - 10 + nib))
            with m.Case(1 + n * per):
                m.d.comb += char.eq(ord(" "))
            with m.Case(1 + n * per + 1):
                m.d.comb += char.eq(
                    Mux(flags_l < 10, ord("0") + flags_l,
                        ord("a") - 10 + flags_l))
            with m.Case(total - 2):
                m.d.comb += char.eq(ord("\r"))
            with m.Case(total - 1):
                m.d.comb += char.eq(ord("\n"))

        with m.If(active):
            m.d.comb += [tx.data.eq(char), tx.ack.eq(1)]
            with m.If(tx.rdy):
                with m.If(idx == total - 1):
                    m.d.cfg += active.eq(0)
                with m.Else():
                    m.d.cfg += idx.eq(idx + 1)
        with m.Elif(snapshot):
            m.d.cfg += [active.eq(1), idx.eq(0)]

        return m


