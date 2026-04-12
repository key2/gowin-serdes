"""Configuration dataclasses for Gowin SerDes IP.

These replace the Verilog `define.vh`, `parameter.vh`, and TOML configuration
files. All compile-time decisions (ifdef in Gowin's wrapper) become Python
conditionals driven by these dataclasses.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class GowinDevice(Enum):
    GW5AT_15 = "GW5AT-15"  # 1 quad, GTR12_QUADB, DRP_NUM=4
    GW5AT_60 = "GW5AT-60"  # 1 quad, GTR12_QUADA, DRP_NUM=4
    GW5AT_138 = "GW5AT-138"  # 2 quads, GTR12_QUADA, DRP_NUM=8


# Metadata per device for TOML / CSR generation:
#   (toml_device_name, csr_binary_suffix, num_quads, has_extra_pads)
DEVICE_META = {
    GowinDevice.GW5AT_15: ("GW5AT-15", "15k", 1, False),
    GowinDevice.GW5AT_60: ("GW5AT-60", "60k", 1, True),
    GowinDevice.GW5AT_138: ("GW5AT-138", "138k", 2, False),
}


class OperationMode(Enum):
    TX_RX = "TX_RX"
    TX_ONLY = "TX_ONLY"
    RX_ONLY = "RX_ONLY"


class EncodingMode(Enum):
    OFF = "OFF"
    B8B10B = "8B10B"
    B64B66B = "64B66B"
    B64B67B = "64B67B"


class GearRate(Enum):
    G1_1 = "1:1"
    G1_2 = "1:2"
    G1_4 = "1:4"


class B64B66BMode(Enum):
    BASER_WITH_FIFO = "10GBASER_WITH_FIFO"
    ASYNC_WITH_FIFO = "ASYNC_WITH_FIFO"


class PLLSelection(Enum):
    CPLL = "CPLL"
    QPLL0 = "QPLL0"
    QPLL1 = "QPLL1"


class RefClkSource(Enum):
    Q0_REFCLK0 = "Q0_REFCLK0"
    Q0_REFCLK1 = "Q0_REFCLK1"
    Q1_REFCLK0 = "Q1_REFCLK0"
    Q1_REFCLK1 = "Q1_REFCLK1"
    Q0_REFIN0 = "Q0_REFIN0"
    Q0_REFIN1 = "Q0_REFIN1"
    Q0_REFIN = "Q0_REFIN"
    MCLK = "MCLK"
    GPIO = "GPIO"


class ChannelBondingDepth(Enum):
    NONE = "NONE"
    ONE_WORD = "ONE_WORD"
    TWO_WORDS = "TWO_WORDS"
    FOUR_WORDS = "FOUR_WORDS"


@dataclass
class LaneConfig:
    """Per-lane configuration. Quad/lane index is set by position in the group."""

    operation_mode: OperationMode = OperationMode.TX_RX
    tx_data_rate: str = "1.25G"
    rx_data_rate: str = "1.25G"
    tx_low_rate_ratio: Optional[int] = None  # 5/10/20/40 when TX<1G
    tx_gear_rate: GearRate = GearRate.G1_1
    rx_gear_rate: GearRate = GearRate.G1_1
    pll: PLLSelection = PLLSelection.CPLL
    ref_clk_source: RefClkSource = RefClkSource.Q0_REFCLK0
    ref_clk_freq: str = "125M"
    clock_mode: str = "Auto"  # "Auto" or "Manual"
    width_mode: int = 10
    tx_encoding: EncodingMode = EncodingMode.OFF
    rx_encoding: EncodingMode = EncodingMode.OFF
    b64b66b_mode: Optional[B64B66BMode] = None
    ilkn_meta_frame_len: Optional[int] = None  # 5~16383 for 64B67B
    word_align: bool = False
    word_align_pattern: Optional[int] = None  # e.g. 0xBC for K28.5
    word_align_mask: Optional[int] = None  # 10-bit mask
    ctc_enable: bool = False
    cc_clk_source: Optional[str] = None

    @property
    def has_tx(self) -> bool:
        return self.operation_mode != OperationMode.RX_ONLY

    @property
    def has_rx(self) -> bool:
        return self.operation_mode != OperationMode.TX_ONLY

    @property
    def has_8b10b(self) -> bool:
        return (
            self.tx_encoding == EncodingMode.B8B10B
            or self.rx_encoding == EncodingMode.B8B10B
        )

    @property
    def has_64b66b(self) -> bool:
        return (
            self.tx_encoding == EncodingMode.B64B66B
            or self.rx_encoding == EncodingMode.B64B66B
        )

    @property
    def has_64b67b(self) -> bool:
        return (
            self.tx_encoding == EncodingMode.B64B67B
            or self.rx_encoding == EncodingMode.B64B67B
        )

    @property
    def has_64b(self) -> bool:
        return self.has_64b66b or self.has_64b67b

    @property
    def tx_data_width(self) -> int:
        return 80

    @property
    def rx_data_width(self) -> int:
        return 88


def device_num_quads(device: GowinDevice) -> int:
    return 2 if device == GowinDevice.GW5AT_138 else 1


def device_drp_num(device: GowinDevice) -> int:
    return 8 if device == GowinDevice.GW5AT_138 else 4


def device_quad_primitive(device: GowinDevice) -> str:
    """GW5AT-15 uses GTR12_QUADB. GW5AT-60 and GW5AT-138 use GTR12_QUADA."""
    if device == GowinDevice.GW5AT_15:
        return "GTR12_QUADB"
    return "GTR12_QUADA"


def device_upar_primitive(device: GowinDevice) -> str:
    if device == GowinDevice.GW5AT_138:
        return "GTR12_UPAR"
    return "GTR12_UPARA"


def device_is_quadb(device: GowinDevice) -> bool:
    """True only for GW5AT-15 which uses GTR12_QUADB."""
    return device == GowinDevice.GW5AT_15
