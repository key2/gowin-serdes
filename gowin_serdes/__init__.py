"""Gowin SerDes — Pure Amaranth reimplementation."""

from .config import (
    GowinDevice,
    OperationMode,
    EncodingMode,
    GearRate,
    B64B66BMode,
    PLLSelection,
    RefClkSource,
    ChannelBondingDepth,
    LaneConfig,
    device_num_quads,
    device_drp_num,
    device_quad_primitive,
    device_upar_primitive,
    device_is_quadb,
)
from .signature import (
    DRPSignature,
    UPARSignature,
    LaneTXSignature,
    LaneRXSignature,
    LaneStatusSignature,
    LaneResetSignature,
)
from .upar_arbiter import GowinUPARArbiter
from .lane import GowinSerDesLane
from .group import GowinSerDesGroup
from .serdes import GowinSerDes
from .toml_gen import generate_toml, generate_csr
from .usb3 import (
    USB3_LANE_OVERRIDES,
    USB3_QUAD_OVERRIDES,
    PHY_LANE_WIRING,
    attach_usb3_phy,
    make_usb3_serdes,
    usb3_boot_writes,
    usb3_lane_config,
)

__all__ = [
    "USB3_LANE_OVERRIDES",
    "USB3_QUAD_OVERRIDES",
    "PHY_LANE_WIRING",
    "attach_usb3_phy",
    "make_usb3_serdes",
    "usb3_boot_writes",
    "usb3_lane_config",
    # Config
    "GowinDevice",
    "OperationMode",
    "EncodingMode",
    "GearRate",
    "B64B66BMode",
    "PLLSelection",
    "RefClkSource",
    "ChannelBondingDepth",
    "LaneConfig",
    # Signatures
    "DRPSignature",
    "UPARSignature",
    "LaneTXSignature",
    "LaneRXSignature",
    "LaneStatusSignature",
    "LaneResetSignature",
    # Components
    "GowinUPARArbiter",
    "GowinSerDesLane",
    "GowinSerDesGroup",
    "GowinSerDes",
    # TOML / CSR generation
    "generate_toml",
    "generate_csr",
]
