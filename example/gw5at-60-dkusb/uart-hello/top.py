"""Dual debug-UART smoke test — Gowin DK_USB (GW5AT-LV60UG225).

Minimal sanity design for the board's bench debug setup: an FT4232H is
hand-wired to the J18 MIPI connector GPIOs, giving two 115200 8N1 serial
channels (platform resources ``uart`` 0 = ``/dev/ttyUSB4`` and ``uart`` 1 =
``/dev/ttyUSB5``).  This design proves, end to end:

* the Gowin toolchain flow for this platform (synthesis + PnR + bitstream),
* flashing through the on-board FT232H (``openFPGALoader -c ft232``,
  ``/dev/ttyUSB1`` on the bench),
* both debug UARTs, in both directions.

Behaviour, per UART:

* every ~1.4 s a banner ``DK60 uartN #xxxx\r\n`` (xxxx = banner counter in
  hex — proves the design is alive and running, not a stuck TX line);
* every received byte is echoed back.

The LED blinks at ~0.7 Hz.  ``USB_PWR_EN`` is driven low (never source
VBUS).  Runs entirely from the 24 MHz oscillator; no PLL, no SerDes.

Build & program:

    python top.py                # build only  (build/uart_hello.fs)
    python top.py program        # build + openFPGALoader
    python top.py flash          # flash the existing bitstream only

Verify (banner + echo):

    stty -F /dev/ttyUSB4 115200 raw -echo && cat /dev/ttyUSB4
"""

import subprocess
import sys
from pathlib import Path

from amaranth.hdl import *

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))                  # platform file
sys.path.insert(0, str(HERE.parent / "usb31-enum"))   # uart.py

from dk_usb_gw5at60 import DKUSBGW5AT60Platform
from uart import AsyncSerial

CLK_HZ = 24_000_000
BAUD = 115_200
DIVISOR = CLK_HZ // BAUD      # 208 -> 115384 baud (+0.16 %, well in spec)


class HelloUart(Elaboratable):
    """One 115200 8N1 endpoint: periodic banner + byte echo."""

    def __init__(self, pins, label):
        self._pins = pins
        self._label = label

    def elaborate(self, platform):
        m = Module()
        m.submodules.serial = serial = AsyncSerial(divisor=DIVISOR,
                                                   pins=self._pins)

        prefix = f"DK60 {self._label} #"
        suffix = "\r\n"
        total = len(prefix) + 4 + len(suffix)

        hexdig = Array(ord(c) for c in "0123456789abcdef")
        count = Signal(16)            # banner sequence number
        timer = Signal(25)            # ~1.4 s @ 24 MHz
        idx = Signal(range(total))
        active = Signal()

        echo_data = Signal(8)
        echo_pending = Signal()

        # Latch one received byte at a time; drop (with the RX overflow
        # flag set) anything that arrives while the echo is still queued.
        m.d.comb += serial.rx.ack.eq(~echo_pending)
        with m.If(serial.rx.rdy & ~echo_pending):
            m.d.sync += [echo_data.eq(serial.rx.data), echo_pending.eq(1)]

        m.d.sync += timer.eq(timer + 1)

        # Banner character mux: fixed text with four hex digits of `count`.
        char = Signal(8)
        with m.Switch(idx):
            for i, c in enumerate(prefix):
                with m.Case(i):
                    m.d.comb += char.eq(ord(c))
            for j in range(4):
                with m.Case(len(prefix) + j):
                    m.d.comb += char.eq(hexdig[count.word_select(3 - j, 4)])
            for k, c in enumerate(suffix):
                with m.Case(len(prefix) + 4 + k):
                    m.d.comb += char.eq(ord(c))

        with m.If(active):
            m.d.comb += [serial.tx.data.eq(char), serial.tx.ack.eq(1)]
            with m.If(serial.tx.rdy):
                with m.If(idx == total - 1):
                    m.d.sync += [active.eq(0), count.eq(count + 1)]
                with m.Else():
                    m.d.sync += idx.eq(idx + 1)
        with m.Elif(echo_pending):
            m.d.comb += [serial.tx.data.eq(echo_data), serial.tx.ack.eq(1)]
            with m.If(serial.tx.rdy):
                m.d.sync += echo_pending.eq(0)
        with m.Elif(timer.all()):
            m.d.sync += [active.eq(1), idx.eq(0)]

        return m


class UartHelloTop(Elaboratable):
    def elaborate(self, platform):
        m = Module()

        # 24 MHz oscillator drives the (reset-less) sync domain; the
        # platform intentionally creates no domains on its own.
        clk24 = platform.request("clk_24m", 0)
        m.domains += ClockDomain("sync", reset_less=True)
        m.d.comb += ClockSignal("sync").eq(clk24.i)

        # Device-mode safety: never source VBUS onto the Type-C port.
        usb_pwr_en = platform.request("usb_pwr_en", 0)
        m.d.comb += usb_pwr_en.o.eq(0)

        m.submodules.hello0 = HelloUart(platform.request("uart", 0), "uart0")
        m.submodules.hello1 = HelloUart(platform.request("uart", 1), "uart1")

        heartbeat = Signal(25)
        m.d.sync += heartbeat.eq(heartbeat + 1)
        m.d.comb += platform.request("led", 0).o.eq(heartbeat[-1])

        return m


# ======================================================================
# Build entry point
# ======================================================================

def _setup_gowin_env(platform):
    import os
    # Gowin-on-Linux quirks (same as the usb31-enum examples).
    os.environ.setdefault("LD_PRELOAD",
                          "/usr/lib/x86_64-linux-gnu/libfreetype.so.6")
    os.environ.setdefault("LD_LIBRARY_PATH",
                          str(Path(platform.gowin_path) / "IDE" / "lib"))
    gowin_bin = str(Path(platform.gowin_path) / "IDE" / "bin")
    if gowin_bin not in os.environ.get("PATH", ""):
        os.environ["PATH"] = gowin_bin + os.pathsep + os.environ["PATH"]


def build(do_program=False):
    platform = DKUSBGW5AT60Platform()
    _setup_gowin_env(platform)
    platform.build(UartHelloTop(), name="uart_hello", build_dir="build",
                   do_program=do_program)


def flash():
    bitstream = HERE / "build" / "uart_hello.fs"
    if not bitstream.exists():
        sys.exit(f"no bitstream at {bitstream}; run `python top.py` first")
    cmd = ["openFPGALoader", "-c", "ft232", str(bitstream)]
    try:
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError:
        subprocess.check_call(["sudo", "-n"] + cmd)


if __name__ == "__main__":
    if "flash" in sys.argv:
        flash()
    else:
        build(do_program="program" in sys.argv)
