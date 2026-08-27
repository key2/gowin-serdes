"""Gowin DK_USB_GW5AT-LV60UG225_V1.0 development board platform.

This is the board the Gowin USB3.1 UVC reference design ships for -- it has
a proper USB Type-C connector (both SuperSpeed pair orientations wired to
SerDes Quad 0), a FUSB302B CC controller, USB2.0 SoftPHY pins, DDR3, HDMI-RX,
MIPI, SDI and an on-board 12V-powered supply tree.

Pin provenance (two independent sources, cross-checked):

* ``doc/DBUG1280-1.0.3E_DK_USB_GW5AT-LV60UG225_V1.0 Development Board User
  Guide.pdf`` -- tables 3-1 (JTAG), 3-2 (clocks), 3-4 (DDR3), 3-5 (HDMI),
  3-6 (MIPI), 3-7 (I2C), 3-8/3-9 (key/LED), 3-10 (Type-C), 3-11 (SDI);
* ``Gowin_USB3.1_UVC_BULK_RefDesign/prj/src/top.cst`` -- the constraint
  file of the *working* vendor design for this exact board (authoritative
  for IO standards; e.g. clk_24m is LVCMOS33/PULL_UP there even though the
  guide table lists the bank at 1.8 V).

Clocking (guide figure 3-3 / reference top.sv):

* ``clk_24m``   -- 24 MHz single-ended oscillator on H5.
* ``sys_clk``   -- 200 MHz LVDS oscillator on K5/J5 (fabric).
* ``serdes_refclk_out`` -- differential *output* pair L9/K8 routed on the
  PCB to the SerDes Q0 REFPAD1 pins (D10/C10).  The reference design feeds
  the SerDes its 200 MHz reference by forwarding ``sys_clk`` through an
  ``ELVDS_OBUF`` onto these pins -- use :func:`add_serdes_refclk_forward`.
* Q0 REFPAD0 (B5/A5) additionally carries a 148.5 MHz oscillator
  (SDI use; usable as an alternative SerDes reference).

USB3 SuperSpeed pairs (SerDes pads -- NOT fabric IO, do not IO_LOC them;
see :data:`SERDES_PINS`): the straight plug orientation pair (TX1/RX1) is
the one the vendor USB3 design drives as **Quad 0 lane 1**; TX2/RX2 is the
flipped orientation (unsupported by the vendor IP, "single-sided plug").

There is no dedicated UART header.  Debug options:

* ``uart`` 0 / ``uart`` 1 -- two hand-wired FT4232H channels on the 80-pin
  J18 MIPI connector GPIOs (workspace setup, not stock board hardware),
  3.3 V, 115200 8N1:

  * ``uart`` 0 = FT4232H channel C = ``/dev/ttyUSB4``: FPGA RX on
    MIPI_GPIO1 (G5, J18.52), FPGA TX on MIPI_GPIO2 (H12, J18.54);
  * ``uart`` 1 = FT4232H channel D = ``/dev/ttyUSB5``: FPGA RX on
    MIPI_GPIO3 (H15, J18.56), FPGA TX on MIPI_GPIO4 (J15, J18.58).

  Directions are FPGA-side and were verified on hardware with an
  auto-polarity probe (the FTDI TX idles high; see
  ``TangMegaPro/luna_softphy_example/uart_probe_dk60.py``).
* ``uart_j21`` 0 -- TX only, on the J21 3-pin header (SCL G11 / SDA F11 /
  GND).  That header exposes the shared I2C bus of the FUSB302B and
  INA3221; transmitting UART on **SCL while leaving SDA untouched** never
  forms an I2C START condition (SDA must fall while SCL is high), so the
  on-board slaves stay idle.  Electrically safe at 115200 baud; do not use
  it if you also need CC/voltage-monitor I2C.
"""

import re
import subprocess

from amaranth.build import *
from amaranth.hdl import Instance, Signal
from amaranth.vendor import GowinPlatform


__all__ = ["DKUSBGW5AT60Platform", "SERDES_PINS", "add_serdes_refclk_forward"]


# SerDes hard-macro pads (documentation only -- bonded straight to the
# GTR12 quad, never constrained as fabric IO).
SERDES_PINS = {
    # USB Type-C SuperSpeed pairs, SerDes Quad 0
    "usb3_tx1": ("D6", "C6"),    # straight orientation, vendor Q0 lane 1
    "usb3_rx1": ("B7", "A7"),
    "usb3_tx2": ("D4", "C4"),    # flipped orientation
    "usb3_rx2": ("B3", "A3"),
    # SerDes reference clock pads
    "q0_refpad0_148m5": ("B5", "A5"),    # 148.5 MHz oscillator (SDI)
    "q0_refpad1_200m": ("D10", "C10"),   # driven by serdes_refclk_out L9/K8
    # SDI (LMH1218/LMH1219 redrivers), SerDes Quad 0
    "sdi_out1": ("D12", "C12"),
    "sdi_out2": ("D8", "C8"),
    "sdi_in1": ("B11", "A11"),
    "sdi_in2": ("B9", "A9"),
}


class DKUSBGW5AT60Platform(GowinPlatform):
    part = "GW5AT-LV60UG225C2/I1"
    family = "GW5AT-60B"

    def __init__(self, *, toolchain="Gowin"):
        super().__init__(toolchain=toolchain)

    def parse_part(self):
        # Amaranth 0.5.x's GowinPlatform.parse_part() does not know the
        # GW5AT series; parse the specific part used on this board.
        m = re.match(r"(GW5AT)-(LV)(60)()(UG225)(C2/I1)$", self.part)
        if not m:
            raise ValueError(f"Unexpected part name: {self.part}")
        self.series = m.group(1)
        self.voltage = m.group(2)
        self.size = m.group(3)
        self.subseries = m.group(4)
        self.package = m.group(5)
        self.speed = m.group(6)

        m2 = re.match(r"(GW5AT)-(60)(B?)$", self.family)
        if not m2:
            raise ValueError(f"Unexpected family name: {self.family}")
        self.series_f = m2.group(1)
        self.size_f = m2.group(2)
        self.subseries_f = m2.group(3)

    gowin_path = "/home/key2/Downloads/gowin"

    resources = [
        # ── Clocks (guide table 3-2, IO standards from the vendor cst) ──
        Resource("clk_24m", 0, Pins("H5", dir="i"), Clock(24e6),
                 Attrs(IO_TYPE="LVCMOS33", PULL_MODE="UP", BANK_VCCIO="3.3")),
        Resource("sys_clk", 0, DiffPairs("K5", "J5", dir="i"), Clock(200e6),
                 Attrs(IO_TYPE="LVDS25", PULL_MODE="NONE",
                       DIFF_RESISTOR="OFF", BANK_VCCIO="3.3")),
        # differential output pair looped on the PCB to Q0 REFPAD1 (D10/C10)
        Resource("serdes_refclk_out", 0, DiffPairs("L9", "K8", dir="o"),
                 Attrs(IO_TYPE="LVCMOS15D", PULL_MODE="NONE", DRIVE="8",
                       BANK_VCCIO="1.5")),

        # ── Key & LED (guide tables 3-8/3-9; key low when pressed) ──
        Resource("key", 0, PinsN("R10", dir="i"),
                 Attrs(IO_TYPE="LVCMOS15", PULL_MODE="NONE",
                       BANK_VCCIO="1.5")),
        Resource("led", 0, Pins("R11", dir="o"),   # on when high
                 Attrs(IO_TYPE="LVCMOS15", PULL_MODE="NONE", DRIVE="8",
                       BANK_VCCIO="1.5")),

        # ── Debug UARTs (see module docstring; hand-wired FT4232H) ──
        Resource("uart", 0,                               # ch C, /dev/ttyUSB4
                 Subsignal("tx", Pins("H12", dir="o")),   # MIPI_GPIO2, J18.54
                 Subsignal("rx", Pins("G5", dir="i")),    # MIPI_GPIO1, J18.52
                 Attrs(IO_TYPE="LVCMOS33", PULL_MODE="UP",
                       BANK_VCCIO="3.3")),
        Resource("uart", 1,                               # ch D, /dev/ttyUSB5
                 Subsignal("tx", Pins("J15", dir="o")),   # MIPI_GPIO4, J18.58
                 Subsignal("rx", Pins("H15", dir="i")),   # MIPI_GPIO3, J18.56
                 Attrs(IO_TYPE="LVCMOS33", PULL_MODE="UP",
                       BANK_VCCIO="3.3")),
        Resource("uart_j21", 0,
                 Subsignal("tx", Pins("G11", dir="o")),   # J21.1 (SCL line)
                 Attrs(IO_TYPE="LVCMOS33", PULL_MODE="UP",
                       BANK_VCCIO="3.3")),

        # ── Shared I2C bus: FUSB302B (CC) + INA3221 (monitor), J21 ──
        Resource("i2c", 0,
                 Subsignal("scl", Pins("G11", dir="io")),
                 Subsignal("sda", Pins("F11", dir="io")),
                 Attrs(IO_TYPE="LVCMOS33", PULL_MODE="NONE", DRIVE="8",
                       BANK_VCCIO="3.3")),
        Resource("fusb_int", 0, PinsN("G15", dir="i"),
                 Attrs(IO_TYPE="LVCMOS33", PULL_MODE="NONE",
                       BANK_VCCIO="3.3")),

        # ── USB Type-C, fabric side (guide table 3-10) ──
        Resource("usb_pwr_en", 0, Pins("J11", dir="o"),   # VBUS supply enable
                 Attrs(IO_TYPE="LVCMOS33", PULL_MODE="NONE", DRIVE="8",
                       BANK_VCCIO="3.3")),
        Resource("usb_cc", 0,
                 Subsignal("cc1", Pins("F8", dir="io")),
                 Subsignal("cc2", Pins("E9", dir="io")),
                 Attrs(IO_TYPE="LVCMOS25", PULL_MODE="NONE",
                       BANK_VCCIO="2.5")),
        Resource("usb_aux", 0,                              # SBU1/SBU2
                 Subsignal("dp", Pins("E7", dir="io")),
                 Subsignal("dn", Pins("E8", dir="io")),
                 Attrs(IO_TYPE="LVCMOS25", PULL_MODE="NONE",
                       BANK_VCCIO="2.5")),
        # USB 2.0 SoftPHY pin set (IPUG781 RC-network scheme)
        Resource("usb2_softphy", 0,
                 Subsignal("tx_dp", Pins("L15", dir="o")),
                 Subsignal("tx_dn", Pins("M15", dir="o")),
                 Subsignal("rx_dp", Pins("L13", dir="i")),
                 Subsignal("rx_dn", Pins("L14", dir="i")),
                 Subsignal("term_dp", Pins("K12", dir="o")),
                 Subsignal("term_dn", Pins("K13", dir="o")),
                 Subsignal("dp", Pins("J13", dir="io")),
                 Subsignal("dp_b", Pins("J14", dir="io")),
                 Subsignal("dn", Pins("N14", dir="io")),
                 Subsignal("dn_b", Pins("N15", dir="io")),
                 Subsignal("pullup_en", Pins("K11", dir="o")),
                 Attrs(IO_TYPE="LVCMOS33", PULL_MODE="NONE",
                       BANK_VCCIO="3.3")),

        # ── MIPI connector reserved GPIOs (J18; guide table 3-6) ──
        Resource("mipi_gpio", 0, Pins("G5", dir="io"),
                 Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3")),
        Resource("mipi_gpio", 1, Pins("H12", dir="io"),
                 Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3")),
        Resource("mipi_gpio", 2, Pins("H15", dir="io"),
                 Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3")),
        Resource("mipi_gpio", 3, Pins("J15", dir="io"),
                 Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3")),
        Resource("mipi_gpio", 4, Pins("F5", dir="io"),
                 Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3")),
        Resource("mipi_gpio", 5, Pins("K3", dir="io"),
                 Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3")),
        Resource("mipi_gpio", 6, Pins("K4", dir="io"),
                 Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3")),
        Resource("mipi_gpio", 7, Pins("G12", dir="io"),
                 Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3")),

        # ── SDI control (guide table 3-11; data pairs are SerDes pads) ──
        Resource("sdi_ctrl", 0,
                 Subsignal("out1_en", Pins("F10", dir="o"),
                           Attrs(IO_TYPE="LVCMOS25", BANK_VCCIO="2.5")),
                 Subsignal("out2_en", Pins("E10", dir="o"),
                           Attrs(IO_TYPE="LVCMOS25", BANK_VCCIO="2.5")),
                 Subsignal("scl", Pins("H10", dir="io"),
                           Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3")),
                 Subsignal("sda", Pins("H11", dir="io"),
                           Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3"))),

        # ── HDMI-RX (guide table 3-5; TMDS via LVCMOS25/bank 2) ──
        Resource("hdmi_rx", 0,
                 Subsignal("clk", DiffPairs("D13", "C14", dir="i")),
                 Subsignal("d0", DiffPairs("B14", "A14", dir="i")),
                 Subsignal("d1", DiffPairs("B13", "A13", dir="i")),
                 Subsignal("d2", DiffPairs("E5", "E6", dir="i")),
                 Attrs(IO_TYPE="LVCMOS25D", PULL_MODE="NONE",
                       BANK_VCCIO="2.5")),
        Resource("hdmi_ctrl", 0,
                 Subsignal("cec", Pins("H6", dir="io"),
                           Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3")),
                 Subsignal("scl", Pins("M13", dir="io"),
                           Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3")),
                 Subsignal("sda", Pins("L12", dir="io"),
                           Attrs(IO_TYPE="LVCMOS33", BANK_VCCIO="3.3"))),

        # ── DDR3, 16-bit (pins from the vendor cst; banks 8/9, SSTL15) ──
        Resource("ddr3", 0,
                 Subsignal("a", Pins("L7 M11 N11 N10 L10 M10 L8 N12 M9 R13 "
                                     "P13 K10 R14 P9", dir="o")),
                 Subsignal("ba", Pins("N2 P11 L6", dir="o")),
                 Subsignal("ras_n", Pins("N7", dir="o")),
                 Subsignal("cas_n", Pins("R2", dir="o")),
                 Subsignal("we_n", Pins("P1", dir="o")),
                 Subsignal("cs_n", Pins("P2", dir="o")),
                 Subsignal("cke", Pins("R12", dir="o")),
                 Subsignal("odt", Pins("N1", dir="o")),
                 Subsignal("reset_n", Pins("N9", dir="o")),
                 Subsignal("clk", DiffPairs("N8", "M8", dir="o"),
                           Attrs(IO_TYPE="SSTL15D_I")),
                 Subsignal("dm", Pins("R5 M3", dir="o")),
                 Subsignal("dq", Pins("M6 R4 M5 R6 L5 R7 N6 P7 "
                                      "M4 M1 N5 L1 N4 L3 P5 L2", dir="io"),
                           Attrs(VREF="INTERNAL")),
                 Subsignal("dqs", DiffPairs("R8 R3", "R9 P3", dir="io"),
                           Attrs(IO_TYPE="SSTL15D_I", DIFF_RESISTOR="OFF")),
                 Attrs(IO_TYPE="SSTL15_I", PULL_MODE="NONE", DRIVE="8",
                       BANK_VCCIO="1.5")),
    ]

    connectors = []
    default_clk = "clk_24m"

    def create_missing_domain(self, name):
        # SerDes designs manage their clock domains explicitly.
        return None

    # ------------------------------------------------------------------
    # Toolchain overrides (same proven structure as the GW5AST-138
    # platform: .csr/.vg handling + SystemVerilog + PnR process config)
    # ------------------------------------------------------------------

    @property
    def file_templates(self):
        templates = dict(super().file_templates)
        templates["{{name}}.tcl"] = r"""
            # {{autogenerated}}
            {% for file in platform.iter_files(".v",".sv",".vg",".vhd",".vhdl") -%}
                add_file {{file}}
            {% endfor %}
            add_file -type verilog {{name}}.v
            add_file -type cst {{name}}.cst
            add_file -type sdc {{name}}.sdc
            {% for file in platform.iter_files(".csr") -%}
                set_csr {{file}}
            {% endfor %}
            set_device -name {{platform.family}} {{platform.part}}
            {{get_override("add_options")|default("# (add_options placeholder)")}}
            run all
            file delete -force {{name}}.fs
            file copy -force impl/pnr/project.fs {{name}}.fs
        """
        templates["impl/project_process_config.json"] = r"""
            {
                "SerDes_retiming" : false,
                "Correct_Hold_Violation" : true,
                "Clock_Route_Order" : 1,
                "Place_Option" : "3",
                "Route_Option" : "1",
                "Route_Maxfan" : 23,
                "Run_Timing_Driven" : true,
                "Promote_Physical_Constraint_Warning_to_Error" : false,
                "Verilog_Standard" : "Vlg_Std_Sysv2017",
                "Process_Configuration_Verion" : "1.0"
            }
        """
        return templates

    def toolchain_prepare(self, fragment, name, **kwargs):
        add_options_lines = [
            "set_option -verilog_std sysv2017",
            "set_option -print_all_synthesis_warning 1",
            "set_option -show_all_warn 1",
            "set_option -use_ready_as_gpio 1",
            "set_option -use_done_as_gpio 1",
            "set_option -use_mspi_as_gpio 1",
            "set_option -use_sspi_as_gpio 1",
            "set_option -use_cpu_as_gpio 1",
            "set_option -use_i2c_as_gpio 1",
            "set_option -rw_check_on_ram 1",
            # DSRM = distributed SRAM (SSRAM).  Without this the IDE
            # reports RP0007 "no SSRAM resource in current device" for
            # RAM16SDP*/distributed-ram usage on GW5AT-60B -- discovered
            # in the vendor project's own OptionList (enable_dsrm=1).
            "set_option -enable_dsrm 1",
            "set_option -looplimit 2000",
            "set_option -bit_security 0",
            "set_option -bit_encrypt 0",
            "set_option -bit_compress 0",
        ]

        # SDC strategy (matches the working vendor project on this board):
        # constrain the SerDes fabric clocks by the nets feeding the PHY;
        # the vendor's own top.sdc uses 6.25 ns (160 MHz) for pclk on this
        # C2/I1 grade.  The board oscillator clocks are created by Amaranth.
        #
        # The LUNA tops (luna_enum / luna_acm) use the same serdes clock
        # nets but not the vendor controller, so they get the clock+group
        # constraints without the controller multicycle paths.  Without
        # these, GowinSynthesis/PnR fall back to a 100 MHz default goal --
        # while pclk really runs at 156.25 MHz (10G boot) / 125 MHz (5G):
        # luna_enum only met 125 MHz by luck, and the first design with a
        # bit more ss-domain logic (luna_acm) missed it and failed EP0
        # handshakes on hardware ("Device not responding to setup address").
        if name in ("luna_enum", "luna_acm", "luna_loopback"):
            sdc_constraints = [
                "create_clock -name pclk -period 6.4 "
                "[get_nets {serdes_pcs_tx_clk_i}]",
                "create_clock -name rxclk -period 6.4 "
                "[get_nets {serdes_pcs_rx_clk_i}]",
                "create_clock -name upar_clk -period 10.0 "
                "[get_nets {serdes_upar_clk_i}]",
                "",
                "set_false_path -from [get_clocks {pclk}] "
                "-to [get_clocks {rxclk upar_clk sys_clk_0__p "
                "clk_24m_0__io}]",
                "set_false_path -from [get_clocks {rxclk}] "
                "-to [get_clocks {pclk upar_clk sys_clk_0__p "
                "clk_24m_0__io}]",
                "set_false_path -from [get_clocks {upar_clk}] "
                "-to [get_clocks {pclk rxclk sys_clk_0__p "
                "clk_24m_0__io}]",
                "set_false_path -from [get_clocks {sys_clk_0__p}] "
                "-to [get_clocks {pclk rxclk upar_clk clk_24m_0__io}]",
                "set_false_path -from [get_clocks {clk_24m_0__io}] "
                "-to [get_clocks {pclk rxclk upar_clk sys_clk_0__p}]",
            ]
        elif name == "usb31_enum":
            sdc_constraints = [
                "create_clock -name pclk -period 6.4 "
                "[get_nets {serdes_pcs_tx_clk_i}]",
                "create_clock -name rxclk -period 6.4 "
                "[get_nets {serdes_pcs_rx_clk_i}]",
                # upar = quad life clock = refclk/2 = 100 MHz on this board
                # (200 MHz refclk; the vendor top.sdc also uses 10 ns).
                "create_clock -name upar_clk -period 10.0 "
                "[get_nets {serdes_upar_clk_i}]",
                "",
                # All clocks are mutually asynchronous, like the vendor
                # top.sdc (one -asynchronous group per clock there).  This
                # includes sys_clk (Amaranth auto-creates `sys_clk_0__p`,
                # 200 MHz; POR/reset deassertion into the USB domains is
                # asynchronous by design, as in the shipping reference
                # design) and the 24 MHz oscillator (debug domain,
                # `clk_24m_0__io`; everything crosses through
                # FFSynchronizers / handshakes).
                "set_false_path -from [get_clocks {pclk}] "
                "-to [get_clocks {rxclk upar_clk sys_clk_0__p "
                "clk_24m_0__io}]",
                "set_false_path -from [get_clocks {rxclk}] "
                "-to [get_clocks {pclk upar_clk sys_clk_0__p "
                "clk_24m_0__io}]",
                "set_false_path -from [get_clocks {upar_clk}] "
                "-to [get_clocks {pclk rxclk sys_clk_0__p "
                "clk_24m_0__io}]",
                "set_false_path -from [get_clocks {sys_clk_0__p}] "
                "-to [get_clocks {pclk rxclk upar_clk clk_24m_0__io}]",
                "set_false_path -from [get_clocks {clk_24m_0__io}] "
                "-to [get_clocks {pclk rxclk upar_clk sys_clk_0__p}]",
                "",
                # The EP0 SETUP-field latches are written during the SETUP
                # stage and consumed (descriptor-ROM read mux) at least
                # two pclk cycles later in the DATA-IN stage -- protocol-
                # stable by construction.  Registers-mapped ROMs (this die
                # has no SSRAM) make these the longest paths in the
                # design; declare the true 2-cycle allowance.
            ] + [
                line
                for latch in ("bmRequestType_latch", "bRequest_latch",
                              "wValue_latch", "wIndex_latch",
                              "wLength_latch")
                for line in (
                    f"set_multicycle_path -setup 2 -from [get_regs "
                    f"{{core/UserLayer_top_inst/ControlTransfer_inst/"
                    f"{latch}*}}]",
                    f"set_multicycle_path -hold 1 -from [get_regs "
                    f"{{core/UserLayer_top_inst/ControlTransfer_inst/"
                    f"{latch}*}}]",
                )
            ] + [
                # The controller's speed (Gen1/Gen2) select register fans
                # out through the whole frozen link/protocol netlist.  It
                # toggles only on an LTSSM rate change, always followed by
                # link retraining (thousands of idle cycles) before any
                # data flows -- a true multicycle signal.
                "set_multicycle_path -setup 2 -from [get_regs "
                "{core/usb3_2_device_controller_inst/speed*}]",
                "set_multicycle_path -hold 1 -from [get_regs "
                "{core/usb3_2_device_controller_inst/speed*}]",
            ]
        elif name == "luna_enum":
            # LUNA SuperSpeed stack on the gw_usb3 PHY: SerDes boots in
            # the 5G trim (Gen1-only), so pclk/rxclk are 125 MHz from
            # power-on.  upar = GTR12 life clock, a free-running ring
            # oscillator measured 56..118 MHz -- 10 ns bound.  All clocks
            # mutually asynchronous (CDC via synchronizers/handshakes).
            sdc_constraints = [
                "create_clock -name pclk -period 8.0 "
                "[get_nets {serdes_pcs_tx_clk_i}]",
                "create_clock -name rxclk -period 8.0 "
                "[get_nets {serdes_pcs_rx_clk_i}]",
                "create_clock -name upar_clk -period 10.0 "
                "[get_nets {serdes_upar_clk_i}]",
                "",
                "set_false_path -from [get_clocks {pclk}] "
                "-to [get_clocks {rxclk upar_clk sys_clk_0__p "
                "clk_24m_0__io}]",
                "set_false_path -from [get_clocks {rxclk}] "
                "-to [get_clocks {pclk upar_clk sys_clk_0__p "
                "clk_24m_0__io}]",
                "set_false_path -from [get_clocks {upar_clk}] "
                "-to [get_clocks {pclk rxclk sys_clk_0__p "
                "clk_24m_0__io}]",
                "set_false_path -from [get_clocks {sys_clk_0__p}] "
                "-to [get_clocks {pclk rxclk upar_clk clk_24m_0__io}]",
                "set_false_path -from [get_clocks {clk_24m_0__io}] "
                "-to [get_clocks {pclk rxclk upar_clk sys_clk_0__p}]",
            ]
        else:
            sdc_constraints = []

        overrides = {
            "add_options": "\n".join(add_options_lines),
            "add_constraints": "\n".join(sdc_constraints),
        }
        merged = {**overrides, **kwargs}
        plan = super().toolchain_prepare(fragment, name, **merged)
        self._merge_diff_pair_constraints(plan, name)
        self._apply_gowin_compat(plan, name)
        return plan

    @staticmethod
    def _apply_gowin_compat(plan, name):
        """Post-process the emitted Verilog for GowinSynthesis quirks.

        Forces ``syn_romstyle = "logic"`` on every module: GowinSynthesis
        V1.9.12.03 mis-infers a functionally WRONG BSRAM pROM from the
        Amaranth-emitted 8b/10b encoder tables (broken Gen1/5G TX on
        hardware; the vendor PHY uses zero BSRAM).  See
        gw_usb3/synthesis.py for the full story.
        """
        try:
            from gw_usb3.synthesis import gowin_compat
        except ImportError:
            return
        fname = f"{name}.v"
        if fname not in plan.files:
            return
        text = plan.files[fname]
        if isinstance(text, bytes):
            text = text.decode()
        plan.files[fname] = gowin_compat(text)

    @staticmethod
    def _merge_diff_pair_constraints(plan, name):
        """Rewrite the generated .cst to the Gowin differential convention.

        Amaranth emits DiffPairs as two independent single-ended ports with
        identical attributes; Gowin's PnR rejects that with ``ERROR CT1000``
        ("this constraint is defined again") on the N-side pair-level
        attributes.  The working vendor project instead constrains *only*
        the P port, listing both pads (``IO_LOC "sys_clk_p" K5,J5;``), and
        leaves the N port entirely unconstrained -- the tool binds the
        complement pad through the TLVDS/ELVDS buffer pairing.  Do the same
        for every ``X__p``/``X__n`` port pair found in the file.
        """
        fname = f"{name}.cst"
        if fname not in plan.files:
            return
        text = plan.files[fname]
        if isinstance(text, bytes):
            text = text.decode()
        locs = dict(re.findall(r'IO_LOC "([^"]+)" ([^;]+);', text))
        out = []
        for line in text.splitlines():
            m = re.match(r'IO_(LOC|PORT) "([^"]+)"', line)
            port = m.group(2) if m else None
            if port and port.endswith("__n") \
                    and port[:-3] + "__p" in locs:
                continue                      # N side: drop all constraints
            if m and m.group(1) == "LOC" and port.endswith("__p") \
                    and port[:-3] + "__n" in locs:
                n_pin = locs[port[:-3] + "__n"].strip()
                line = f'IO_LOC "{port}" {locs[port].strip()},{n_pin};'
            out.append(line)
        plan.files[fname] = "\n".join(out) + "\n"

    # The board's Mini USB-B download port is an FT232H (enumerates as
    # "FTDI Single RS232-HS", /dev/ttyUSB1 on the bench setup); verified
    # working with ``openFPGALoader -c ft232``.
    programmer_cable = "ft232"

    def toolchain_program(self, products, name, **kwargs):
        with products.extract("{}.fs".format(name)) as bitstream_filename:
            cmd = ["openFPGALoader", "--cable", self.programmer_cable,
                   bitstream_filename]
            try:
                subprocess.check_call(cmd)
            except subprocess.CalledProcessError:
                # USB permissions often require root for the FTDI cable.
                subprocess.check_call(["sudo", "-n"] + cmd)


def add_serdes_refclk_forward(m, platform):
    """Forward ``sys_clk`` (200 MHz LVDS) to the SerDes Q0 REFPAD1 pins.

    Reproduces the reference design's clocking: TLVDS_IBUF on the 200 MHz
    oscillator, ELVDS_OBUF onto the L9/K8 pair that the PCB routes to the
    Q0 REFPAD1 pads (D10/C10).  Returns the single-ended 200 MHz fabric
    clock Signal (the reference design's ``sys_clk``).

    The SerDes TOML must then select ``Q0_REFCLK1`` with
    ``ref_clk_freq="200M"`` -- exactly the reference design configuration
    whose CSR blob is reproduced byte-exact by the test suite.
    """
    sys_clk_pads = platform.request("sys_clk", 0, dir="-")
    refclk_out_pads = platform.request("serdes_refclk_out", 0, dir="-")

    sys_clk = Signal(name="sys_clk")
    m.submodules.sys_clk_ibuf = Instance(
        "TLVDS_IBUF",
        i_I=sys_clk_pads.p, i_IB=sys_clk_pads.n,
        o_O=sys_clk,
    )
    m.submodules.serdes_refclk_obuf = Instance(
        "ELVDS_OBUF",
        i_I=sys_clk,
        o_O=refclk_out_pads.p, o_OB=refclk_out_pads.n,
    )
    return sys_clk
