"""UART enumeration reporter.

Runs entirely in the ``upar`` clock domain (62.5 MHz SerDes life-clock, which
keeps ticking through USB rate changes -- unlike pclk, whose frequency moves
between 156.25 MHz/Gen2 and 125 MHz/Gen1).

Emits human-readable lines at 115200 baud:

* ``U <8 hex>``  -- status word, printed on change (debounced) and as a
  ~0.5 s heartbeat;
* ``R <bmRequestType> <bRequest> <wValue> <wIndex> <wLength>`` -- one line
  per SETUP request seen by the device controller (the enumeration itself,
  live).

Status word bits:
    [0]  cpll_ok           [8]  power_down[0]
    [1]  lane_ready        [9]  power_down[1]
    [2]  cdr_lock          [10] tx_detrx_lpbk (receiver detect / loopback)
    [3]  signal_detect     [11] ltssm_is_training
    [4]  rx_elecidle       [12] attached (LTSSM reached U0)
    [5]  rate (1 = Gen2)   [13] itp_received (host ITPs arriving in U0)
    [6]  tx_elecidle       [14] warm_or_hot_reset
    [7]  rx_termination    [15] phy_status (sticky since last line)
    [21:16] LTSSM state (binary-coded, from the vendor netlist)
    [23:22] SETUP request count (low 2 bits; R lines show each request)
    [31:24] heartbeat counter
"""

from amaranth.hdl import *
from amaranth.lib.cdc import FFSynchronizer
from amaranth.lib.wiring import Component, In, Out

from uart import AsyncSerialTX


def _hex_digit(m, nibble):
    """Comb expression: ASCII hex character for a 4-bit nibble."""
    return Mux(nibble < 10, ord("0") + nibble, ord("a") - 10 + nibble)


class UsbDebugReporter(Component):
    """Formats status/SETUP lines and drives one AsyncSerialTX ("upar" domain).

    All ``mon_*`` inputs live in the pclk domain and are synchronized or
    latched-and-handshaked internally.
    """

    # pclk-domain monitoring inputs
    mon_status_bits: In(16)     # level signals, packed as documented above
    mon_ltssm_state: In(6)      # binary-coded LTSSM state
    mon_request_stb: In(1)      # pclk pulse: SETUP request latched
    mon_bmRequestType: In(8)
    mon_bRequest: In(8)
    mon_wValue: In(16)
    mon_wIndex: In(16)
    mon_wLength: In(16)

    # UART output (upar domain)
    tx_o: Out(1)

    def __init__(self, divisor: int, heartbeat_bits: int = 25):
        self._divisor = divisor
        self._hb_bits = heartbeat_bits
        super().__init__()

    def elaborate(self, platform):
        m = Module()

        tx = AsyncSerialTX(divisor=self._divisor)
        m.submodules.tx = DomainRenamer("upar")(tx)
        m.d.comb += self.tx_o.eq(tx.o)

        # ------------------------------------------------------------
        # CDC: status levels into upar
        # ------------------------------------------------------------
        status_s = Signal(16)
        m.submodules += FFSynchronizer(self.mon_status_bits, status_s,
                                       o_domain="upar")
        ltssm_s = Signal(6)
        m.submodules += FFSynchronizer(self.mon_ltssm_state, ltssm_s,
                                       o_domain="upar")

        # sticky phy_status (bit 15): stretch pclk pulses via toggle capture
        # (already a level in mon_status_bits; treated like the others)

        # ------------------------------------------------------------
        # CDC: SETUP capture.  The pclk side latches fields and toggles a
        # flag; the upar side detects the toggle, waits a few cycles for the
        # quasi-static fields to settle, then prints.
        # ------------------------------------------------------------
        req_fields = Signal(64)   # pclk-domain latch
        req_tgl = Signal()
        with m.If(self.mon_request_stb):
            m.d.pclk += [
                req_fields.eq(Cat(self.mon_wLength, self.mon_wIndex,
                                  self.mon_wValue, self.mon_bRequest,
                                  self.mon_bmRequestType)),
                req_tgl.eq(~req_tgl),
            ]

        req_tgl_s = Signal()
        m.submodules += FFSynchronizer(req_tgl, req_tgl_s, o_domain="upar")
        req_tgl_d = Signal()
        req_pending = Signal()
        req_count = Signal(8)
        m.d.upar += req_tgl_d.eq(req_tgl_s)
        with m.If(req_tgl_s != req_tgl_d):
            m.d.upar += [req_pending.eq(1), req_count.eq(req_count + 1)]

        # ------------------------------------------------------------
        # heartbeat + change detection
        # ------------------------------------------------------------
        heartbeat = Signal(self._hb_bits)
        hb_count = Signal(8)
        m.d.upar += heartbeat.eq(heartbeat + 1)

        status_word = Signal(32)
        m.d.comb += status_word.eq(
            Cat(status_s, ltssm_s, req_count[0:2], hb_count))

        status_last = Signal(16)
        ltssm_last = Signal(6)
        want_status = Signal()
        with m.If((status_s != status_last) | (ltssm_s != ltssm_last)
                  | heartbeat.all()):
            m.d.upar += want_status.eq(1)

        # ------------------------------------------------------------
        # line formatter: prefix char + hex digits (MSB first) + CRLF
        # ------------------------------------------------------------
        SP = ord(" ")
        # message layouts: list of (kind, payload) steps
        #   status : 'U' + 8 digits of status_word
        #   request: 'R' + fields with spaces
        msg_hex = Signal(72)      # left-aligned hex payload
        msg_digits = Signal(5)    # digits remaining
        # request line: bm(2) bR(2) wV(4) wI(4) wL(4) => grouped w/ spaces
        req_shift = Signal(64)
        req_group = Signal(3)     # groups remaining
        GROUPS = ((8, 2), (8, 2), (16, 4), (16, 4), (16, 4))  # (bits, digits)

        def send_byte(value, next_state):
            m.d.comb += tx.data.eq(value)
            with m.If(tx.rdy):
                m.d.comb += tx.ack.eq(1)
                m.next = next_state

        with m.FSM(domain="upar"):
            with m.State("IDLE"):
                with m.If(req_pending):
                    m.d.upar += [
                        req_pending.eq(0),
                        req_shift.eq(req_fields),   # settled: >2 upar cycles old
                        req_group.eq(len(GROUPS)),
                    ]
                    m.next = "R_CHAR"
                with m.Elif(want_status):
                    m.d.upar += [
                        want_status.eq(0),
                        status_last.eq(status_s),
                        ltssm_last.eq(ltssm_s),
                        hb_count.eq(hb_count + 1),
                        msg_hex.eq(Cat(Const(0, 40), status_word)),
                        msg_digits.eq(8),
                    ]
                    m.next = "U_CHAR"

            # ---- status line: 'U' ' ' XXXXXXXX \r\n --------------------
            with m.State("U_CHAR"):
                send_byte(ord("U"), "U_SP")
            with m.State("U_SP"):
                send_byte(SP, "U_HEX")
            with m.State("U_HEX"):
                m.d.comb += tx.data.eq(_hex_digit(m, msg_hex[68:72]))
                with m.If(tx.rdy):
                    m.d.comb += tx.ack.eq(1)
                    m.d.upar += [
                        msg_hex.eq(msg_hex << 4),
                        msg_digits.eq(msg_digits - 1),
                    ]
                    with m.If(msg_digits == 1):
                        m.next = "CR"

            # ---- request line: 'R' then 5 space-separated hex groups ----
            with m.State("R_CHAR"):
                send_byte(ord("R"), "R_SP")
            with m.State("R_SP"):
                with m.If(req_group == 0):
                    m.next = "CR"
                with m.Else():
                    send_byte(SP, "R_LOAD")
            with m.State("R_LOAD"):
                # pop the next field (MSB-first layout in req_shift)
                for gi, (bits, digits) in enumerate(GROUPS):
                    with m.If(req_group == len(GROUPS) - gi):
                        m.d.upar += [
                            msg_hex.eq(Cat(Const(0, 72 - bits),
                                           req_shift[64 - bits:64])),
                            msg_digits.eq(digits),
                            req_shift.eq(req_shift << bits),
                        ]
                m.d.upar += req_group.eq(req_group - 1)
                m.next = "R_HEX"
            with m.State("R_HEX"):
                m.d.comb += tx.data.eq(_hex_digit(m, msg_hex[68:72]))
                with m.If(tx.rdy):
                    m.d.comb += tx.ack.eq(1)
                    m.d.upar += [
                        msg_hex.eq(msg_hex << 4),
                        msg_digits.eq(msg_digits - 1),
                    ]
                    with m.If(msg_digits == 1):
                        m.next = "R_SP"

            # ---- line ending -------------------------------------------
            with m.State("CR"):
                send_byte(ord("\r"), "LF")
            with m.State("LF"):
                send_byte(ord("\n"), "IDLE")

        return m
