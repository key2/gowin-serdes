"""Sipeed Slogic16U3 Platform — GW5AT-LV15MG132C1/I0.

Adapted for the Sipeed Slogic16U3 board which carries a Gowin
GW5AT-LV15MG132C1/I0 FPGA in a QFN132 package.

Pin assignments:
- Clock: 125 MHz oscillator on pin N1 (GCLKT_5)
- Clock Enable: D14 (active-high, enables the oscillator)
- UART TX: F14 (FPGA → host)
- UART RX: E14 (host → FPGA)

UART pins are taken from the LiteX board definition
(litex-hub/litex-boards, sipeed_slogic16u3.py).

Typical usage::

    from gw5at_dvk import GW5ATDVKPlatform

    platform = GW5ATDVKPlatform(toolchain="Gowin")
    platform.build(my_design, name="top", build_dir="build")
"""

import re
import subprocess

from amaranth.build import *
from amaranth.build.plat import TemplatedPlatform
from amaranth.vendor import GowinPlatform


__all__ = ["GW5ATDVKPlatform"]


class GW5ATDVKPlatform(GowinPlatform):
    """Sipeed Slogic16U3 Platform for the GW5AT-LV15MG132C1/I0.

    Targets the GW5AT-LV15MG132C1/I0 FPGA in the QFN132 package.
    The board provides a 125 MHz oscillator on pin N1.
    """

    # GowinPlatform requires 'part' and 'family' as class attributes.
    part = "GW5AT-LV15MG132C1/I0"
    family = "GW5AT-15A"

    def __init__(self, *, toolchain="Gowin"):
        super().__init__(toolchain=toolchain)

    def parse_part(self):
        """Override parse_part for GW5AT series not yet in Amaranth 0.5.x.

        Amaranth 0.5.8's GowinPlatform.parse_part() only recognises
        GW[12][AN]… series names.  The GW5AT is a newer Gowin family
        not covered by that regex, so we hard-code the parsed fields
        for the specific part used on this board.
        """
        # ---- part string: GW5AT-LV15MG132C1/I0 ----
        m = re.match(
            r"(GW5AT)-(LV)(15)()(MG132)(C1/I0)$",
            self.part,
        )
        if not m:
            raise ValueError(f"Unexpected part name: {self.part}")

        self.series = m.group(1)  # "GW5AT"
        self.voltage = m.group(2)  # "LV"
        self.size = m.group(3)  # "15"
        self.subseries = m.group(4)  # ""
        self.package = m.group(5)  # "MG132"
        self.speed = m.group(6)  # "C1/I0"

        # ---- family string: GW5AT-15A ----
        m2 = re.match(r"(GW5AT)-(15)(A?)$", self.family)
        if not m2:
            raise ValueError(f"Unexpected family name: {self.family}")

        self.series_f = m2.group(1)  # "GW5AT"
        self.size_f = m2.group(2)  # "15"
        self.subseries_f = m2.group(3)  # "A"

    # Path to the Gowin installation root (the directory that contains IDE/)
    gowin_path = "/home/key2/Downloads/gowin"

    # ------------------------------------------------------------------
    # Board resources
    # ------------------------------------------------------------------

    resources = [
        # 125 MHz board oscillator (N1, Bank 5 — VCCIO 2.8 V)
        # LVCMOS25 is the correct Gowin IO standard for 2.8 V banks.
        Resource(
            "clk125",
            0,
            Pins("N1", dir="i"),
            Clock(125e6),
            Attrs(IO_TYPE="LVCMOS25"),
        ),
        # Oscillator enable pin (D14, Bank 1 — VCCIO 3.3 V)
        # Must be driven high for the 125 MHz oscillator to run.
        Resource(
            "clk_en",
            0,
            Pins("D14", dir="o"),
            Attrs(IO_TYPE="LVCMOS33"),
        ),
        # UART (directly connected to host via USB bridge)
        # TX: F14 (FPGA → host), RX: E14 (host → FPGA)
        Resource(
            "uart",
            0,
            Subsignal("tx", Pins("F14", dir="o")),
            Subsignal("rx", Pins("E14", dir="i")),
            Attrs(IO_TYPE="LVCMOS33"),
        ),
    ]

    connectors = []

    # Default clock / reset resource names
    default_clk = "clk125"
    # No default_rst — design uses ResetSynchronizer with Const(0).

    # ------------------------------------------------------------------
    # Clock domain management
    # ------------------------------------------------------------------

    def create_missing_domain(self, name):
        """Return ``None`` for all domains.

        The top-level design manages its own clock domains explicitly.
        Returning ``None`` prevents Amaranth from auto-creating a
        ``sync`` domain or any other missing domain.
        """
        return None

    # ------------------------------------------------------------------
    # File templates — add set_csr for SerDes designs
    # ------------------------------------------------------------------

    @property
    def file_templates(self):
        """Override file templates to include .csr files in the TCL script.

        The base GowinPlatform TCL template only iterates over
        ``.v``, ``.sv``, ``.vhd``, ``.vhdl`` files.  SerDes designs
        need ``.csr`` (SerDes configuration) files to be added too.

        Additionally, the SerDes encrypted IP sub-modules need include
        paths set up so they can resolve ``define.vh`` and
        ``static_macro_define.vh``.
        """
        templates = dict(super().file_templates)

        # SDC: SerDes hard macro generates internal clocks.
        # CDC handled by AsyncFIFOs. No additional SDC constraints needed.

        templates["{{name}}.tcl"] = r"""
            # {{autogenerated}}
            {% for file in platform.iter_files(".v",".sv",".vhd",".vhdl") -%}
                add_file {{file}}
            {% endfor %}
            add_file -type verilog {{name}}.v
            add_file -type cst {{name}}.cst
            add_file -type sdc {{name}}.sdc
            {% for file in platform.iter_files(".csr") -%}
                set_csr {{file}}
            {% endfor %}
            {{get_override("add_serdes_ip")|default("# (no serdes IP files)")}}
            set_device -name {{platform.family}} {{platform.part}}
            {{get_override("add_options")|default("# (add_options placeholder)")}}
            run all
            file delete -force {{name}}.fs
            file copy -force impl/pnr/project.fs {{name}}.fs
        """
        return templates

    # ------------------------------------------------------------------
    # Toolchain overrides
    # ------------------------------------------------------------------

    def toolchain_prepare(self, fragment, name, **kwargs):
        """Prepare the build plan with Gowin-specific options."""
        add_options_lines = [
            "set_option -verilog_std v2001",
            "set_option -print_all_synthesis_warning 1",
            "set_option -show_all_warn 1",
            # Free up special-purpose pins for GPIO use
            "set_option -use_ready_as_gpio 1",
            "set_option -use_done_as_gpio 1",
            "set_option -use_mspi_as_gpio 1",
            "set_option -use_sspi_as_gpio 1",
            "set_option -use_cpu_as_gpio 1",
            # Bitstream options
            "set_option -bit_security 0",
            "set_option -bit_encrypt 0",
            "set_option -bit_compress 0",
        ]

        sdc_constraints = [
            "# SerDes cross-domain constraints",
            "# The UPAR and RX clocks originate from the GTR12 hard macro.",
            "# AsyncFIFOs handle CDC; false-path constraints are advisory.",
        ]

        # Build the add_serdes_ip TCL lines from stored IP file/dir lists
        serdes_ip_lines = []
        for d in getattr(self, "_serdes_ip_inc_dirs", []):
            serdes_ip_lines.append(f"set_option -include_path {d}")
        for f in getattr(self, "_serdes_ip_files", []):
            serdes_ip_lines.append(f"add_file {f}")

        overrides = {
            "add_options": "\n".join(add_options_lines),
            "add_constraints": "\n".join(sdc_constraints),
            "add_serdes_ip": "\n".join(serdes_ip_lines)
            if serdes_ip_lines
            else "# (no serdes IP files)",
        }
        merged = {**overrides, **kwargs}
        return super().toolchain_prepare(fragment, name, **merged)

    def toolchain_program(self, products, name, **kwargs):
        """Program the FPGA using openFPGALoader."""
        with products.extract("{}.fs".format(name)) as bitstream_filename:
            cmd = [
                "openFPGALoader",
                "--cable",
                "ft4232",
                "--bitstream",
                bitstream_filename,
            ]
            subprocess.check_call(cmd)
