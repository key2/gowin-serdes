"""GTR12 hard macro port tables.

Extracted from the generated serdes.v files for GW5AT-15 (GTR12_QUADB/UPARA)
and GW5AT-138 (GTR12_QUAD/UPAR). These tables drive the Instance() builder
in quad.py — no manual port wiring needed.

Port entries: (name_template, direction, width)
  - name_template uses {ln} for lane index (0-3) and {LN} for LANE index.
  - direction: "i" = input, "o" = output
"""

# ──────────────────────────────────────────────────────────────
# Per-lane port templates. {ln} = 0-3, {LN} = LANE0-LANE3
# ──────────────────────────────────────────────────────────────

# These are common to BOTH GTR12_QUADB and GTR12_QUAD
QUAD_LANE_PORTS_COMMON = [
    # TX analog
    ("LN{ln}_TXM_O", "o", 1),
    ("LN{ln}_TXP_O", "o", 1),
    # RX analog
    ("LN{ln}_RXM_I", "i", 1),
    ("LN{ln}_RXP_I", "i", 1),
    # TX data
    ("FABRIC_LN{ln}_TXDATA_I", "i", 80),
    ("FABRIC_LN{ln}_TX_VLD_IN", "i", 1),
    # RX data & status
    ("FABRIC_LN{ln}_RXDATA_O", "o", 88),
    ("FABRIC_LN{ln}_ASTAT_O", "o", 6),
    ("FABRIC_LN{ln}_STAT_O", "o", 13),
    ("FABRIC_LN{ln}_STAT_O_H", "o", 13),
    ("FABRIC_LN{ln}_PMA_RX_LOCK_O", "o", 1),
    ("FABRIC_LN{ln}_BURN_IN_TOGGLE_O", "o", 1),
    ("FABRIC_LN{ln}_RXDET_RESULT", "o", 1),
    ("FABRIC_LN{ln}_RX_VLD_OUT", "o", 1),
    ("FABRIC_LN{ln}_RXELECIDLE_O", "o", 1),
    ("FABRIC_LN{ln}_RXELECIDLE_O_H", "o", 1),
    # Per-lane CMU
    ("FABRIC_LANE{ln}_CMU_CK_REF_O", "o", 1),
    ("FABRIC_LANE{ln}_CMU_OK_O", "o", 1),
    # PCS 8b/10b
    ("LANE{ln}_ALIGN_LINK", "o", 1),
    ("LANE{ln}_K_LOCK", "o", 1),
    ("LANE{ln}_DISP_ERR_O", "o", 2),
    ("LANE{ln}_DEC_ERR_O", "o", 2),
    ("LANE{ln}_CUR_DISP_O", "o", 2),
    # PCS clocks
    ("LANE{ln}_PCS_RX_O_FABRIC_CLK", "o", 1),
    ("LANE{ln}_PCS_TX_O_FABRIC_CLK", "o", 1),
    # RX FIFO
    ("LANE{ln}_RX_IF_FIFO_RDUSEWD", "o", 5),
    ("LANE{ln}_RX_IF_FIFO_AEMPTY", "o", 1),
    ("LANE{ln}_RX_IF_FIFO_EMPTY", "o", 1),
    # TX FIFO
    ("LANE{ln}_TX_IF_FIFO_WRUSEWD", "o", 5),
    ("LANE{ln}_TX_IF_FIFO_AFULL", "o", 1),
    ("LANE{ln}_TX_IF_FIFO_FULL", "o", 1),
    # Lane control inputs
    ("FABRIC_LN{ln}_CTRL_I", "i", 43),
    ("FABRIC_LN{ln}_CTRL_I_H", "i", 43),
    ("FABRIC_LN{ln}_IDDQ_I", "i", 1),
    ("FABRIC_LN{ln}_PD_I", "i", 3),
    ("FABRIC_LN{ln}_PD_I_H", "i", 3),
    ("FABRIC_LN{ln}_RATE_I", "i", 2),
    ("FABRIC_LN{ln}_RATE_I_H", "i", 2),
    ("FABRIC_LN{ln}_RSTN_I", "i", 1),
    # PCS reset/control inputs
    ("LANE{ln}_PCS_RX_RST", "i", 1),
    ("LANE{ln}_ALIGN_TRIGGER", "i", 1),
    ("LANE{ln}_CHBOND_START", "i", 1),
    ("LANE{ln}_PCS_TX_RST", "i", 1),
    # Fabric clock inputs
    ("LANE{ln}_FABRIC_RX_CLK", "i", 1),
    ("LANE{ln}_FABRIC_C2I_CLK", "i", 1),
    ("LANE{ln}_FABRIC_TX_CLK", "i", 1),
    # RX FIFO control
    ("LANE{ln}_RX_IF_FIFO_RDEN", "i", 1),
    # CPLL control
    ("FABRIC_LN{ln}_CPLL_RESETN_I", "i", 1),
    ("FABRIC_LN{ln}_CPLL_PD_I", "i", 1),
    ("FABRIC_LN{ln}_CPLL_IDDQ_I", "i", 1),
]

# Ports only on GTR12_QUADB (GW5AT-15/60)
QUAD_LANE_PORTS_QUADB_ONLY = [
    ("FABRIC_LANE{ln}_64B66B_TX_INVLD_BLK", "o", 1),
    ("FABRIC_LANE{ln}_64B66B_TX_FETCH", "o", 1),
    ("FABRIC_LANE{ln}_64B66B_RX_VALID", "o", 1),
    ("FABRIC_LN{ln}_TX_DISPARITY_I", "i", 8),
]


# ──────────────────────────────────────────────────────────────
# Quad-level ports (not per-lane)
# ──────────────────────────────────────────────────────────────

QUAD_COMMON_PORTS = [
    # Reference clock inputs
    ("REFCLKM0_I", "i", 1),
    ("REFCLKM1_I", "i", 1),
    ("REFCLKP0_I", "i", 1),
    ("REFCLKP1_I", "i", 1),
    ("FABRIC_REFCLK1_INPUT_SEL_I", "i", 3),
    ("FABRIC_REFCLK_INPUT_SEL_I", "i", 3),
    ("FABRIC_PMA_PD_REFHCLK_I", "i", 1),
    ("FABRIC_REFCLK_GATE_I", "i", 1),
    ("FABRIC_CMU1_REFCLK_GATE_I", "i", 1),
    ("FABRIC_CMU_REFCLK_GATE_I", "i", 1),
    # CMU/PLL status outputs
    ("FABRIC_PMA_CM0_DR_REFCLK_DET_O", "o", 1),
    ("FABRIC_PMA_CM1_DR_REFCLK_DET_O", "o", 1),
    ("FABRIC_CM1_LIFE_CLK_O", "o", 1),
    ("FABRIC_CM_LIFE_CLK_O", "o", 1),
    ("FABRIC_CMU1_CK_REF_O", "o", 1),
    ("FABRIC_CMU1_OK_O", "o", 1),
    ("FABRIC_CMU1_REFCLK_GATE_ACK_O", "o", 1),
    ("FABRIC_CMU_CK_REF_O", "o", 1),
    ("FABRIC_CMU_OK_O", "o", 1),
    ("FABRIC_CMU_REFCLK_GATE_ACK_O", "o", 1),
    ("FABRIC_REFCLK_GATE_ACK_O", "o", 1),
    ("FABRIC_CMU0_CLK", "o", 1),
    ("FABRIC_CMU1_CLK", "o", 1),
    ("FABRIC_QUAD_CLK_RX", "o", 1),
    ("FABRIC_CLK_MON_O", "o", 1),
    ("FABRIC_GEARFIFO_ERR_RPT", "o", 1),
    # CMU/PLL control
    ("FABRIC_CMU0_RESETN_I", "i", 1),
    ("FABRIC_CMU0_PD_I", "i", 1),
    ("FABRIC_CMU0_IDDQ_I", "i", 1),
    ("FABRIC_CMU1_RESETN_I", "i", 1),
    ("FABRIC_CMU1_PD_I", "i", 1),
    ("FABRIC_CMU1_IDDQ_I", "i", 1),
    ("FABRIC_CM1_PD_REFCLK_DET_I", "i", 1),
    ("FABRIC_CM0_PD_REFCLK_DET_I", "i", 1),
    # Quad misc
    ("FABRIC_BURN_IN_I", "i", 1),
    ("FABRIC_CK_SOC_DIV_I", "i", 2),
    ("FABRIC_GLUE_MAC_INIT_INFO_I", "i", 1),
    ("FABRIC_POR_N_I", "i", 1),
    ("FABRIC_QUAD_MCU_REQ_I", "i", 1),
    ("CK_AHB_I", "i", 1),
    ("AHB_RSTN", "i", 1),
    ("TEST_DEC_EN", "i", 1),
    ("QUAD_PCIE_CLK", "i", 1),
    ("PCIE_DIV2_REG", "i", 1),
    ("PCIE_DIV4_REG", "i", 1),
    ("PMAC_LN_RSTN", "i", 1),
    ("FABRIC_PLL_CDN_I", "i", 1),
]

# Quad-level ports only on GTR12_QUADB
QUAD_PORTS_QUADB_ONLY = [
    # Only on GTR12_QUADB (GW5AT-15)
    ("FABRIC_PMA_CM2_DR_REFCLK_DET_O", "o", 1),
    ("FABRIC_PMA_CM3_DR_REFCLK_DET_O", "o", 1),
    ("FABRIC_CM2_PD_REFCLK_DET_I", "i", 1),
    ("FABRIC_CM3_PD_REFCLK_DET_I", "i", 1),
    ("CKP_MIPI_1", "o", 1),
    ("CKP_MIPI_0", "o", 1),
    ("CKN_MIPI_1", "o", 1),
    ("CKN_MIPI_0", "o", 1),
    ("QUAD_PCLK1", "o", 1),
    ("QUAD_PCLK0", "o", 1),
    ("CLK_VIQ_I", "i", 2),
    ("FABRIC_CLK_REF_CORE_I", "i", 4),
]

# Quad-level ports only on GTR12_QUADA (GW5AT-60)
QUAD_PORTS_QUADA_60_ONLY = [
    ("REFCLKM2_I", "i", 1),
    ("REFCLKM3_I", "i", 1),
    ("REFCLKP2_I", "i", 1),
    ("REFCLKP3_I", "i", 1),
    ("CLK_VIQ_I", "i", 4),
    ("FABRIC_CLK_REF_CORE_I", "i", 1),
]

# Quad-level ports only on GTR12_QUADA (GW5AT-138)
QUAD_PORTS_QUAD_ONLY = [
    ("INET_Q0_Q1", "o", 92),
    ("INET_Q_PMAC", "o", 532),
    ("INET_Q_TEST", "o", 228),
    ("INET_Q_UPAR", "o", 421),
    ("FABRIC_CLK_LIFE_DIV_I", "i", 2),
    ("FABRIC_CM0_RXCLK_OE_L_I", "i", 1),
    ("FABRIC_CM0_RXCLK_OE_R_I", "i", 1),
    ("FABRIC_REFCLK_OE_L_I", "i", 1),
    ("FABRIC_REFCLK_OE_R_I", "i", 1),
    ("FABRIC_REFCLK_OUTPUT_SEL_I", "i", 5),
    ("FABRIC_CLK_REF_CORE_I", "i", 1),
]


# ──────────────────────────────────────────────────────────────
# UPAR primitive ports
# ──────────────────────────────────────────────────────────────

UPAR_COMMON_PORTS = [
    # Outputs
    ("CSR_TDO", "o", 1),
    ("UPAR_RDDATA_S", "o", 32),
    ("UPAR_RDVLD_S", "o", 1),
    ("UPAR_READY_S", "o", 1),
    ("SPI_MISO", "o", 1),
    ("AHB_CLK_O", "o", 1),
    ("QUAD_CFG_TEST_DEC_EN", "o", 1),
    ("AHB_RSTN_O", "o", 1),
    # JTAG/CSR inputs
    ("CSR_TCK", "i", 1),
    ("CSR_TMS", "i", 1),
    ("CSR_TDI", "i", 1),
    ("CSR_MODE", "i", 5),
    # UPAR bus inputs
    ("UPAR_CLK", "i", 1),
    ("UPAR_RST", "i", 1),
    ("UPAR_WREN_S", "i", 1),
    ("UPAR_ADDR_S", "i", 24),
    ("UPAR_WRDATA_S", "i", 32),
    ("UPAR_RDEN_S", "i", 1),
    ("UPAR_STRB_S", "i", 8),
    ("UPAR_BUS_WIDTH_S", "i", 1),
    # SPI inputs
    ("SPI_CLK", "i", 1),
    ("SPI_MOSI", "i", 1),
    ("SPI_SS", "i", 1),
    # DFT inputs
    ("FABRIC_DFT_EDT_UPDATE", "i", 1),
    ("FABRIC_DFT_IJTAG_CE", "i", 1),
    ("FABRIC_DFT_IJTAG_RESET", "i", 1),
    ("FABRIC_DFT_IJTAG_SE", "i", 1),
    ("FABRIC_DFT_IJTAG_SEL", "i", 1),
    ("FABRIC_DFT_IJTAG_SI", "i", 1),
    ("FABRIC_DFT_IJTAG_TCK", "i", 1),
    ("FABRIC_DFT_IJTAG_UE", "i", 1),
    ("FABRIC_DFT_PLL_BYPASS_CLK", "i", 1),
    ("FABRIC_DFT_PLL_BYPASS_MODE", "i", 1),
    ("FABRIC_DFT_SCAN_CLK", "i", 1),
    ("FABRIC_DFT_SCAN_EN", "i", 1),
    ("FABRIC_DFT_SCAN_IN0", "i", 1),
    ("FABRIC_DFT_SCAN_IN1", "i", 1),
    ("FABRIC_DFT_SCAN_IN2", "i", 1),
    ("FABRIC_DFT_SCAN_IN3", "i", 1),
    ("FABRIC_DFT_SCAN_IN4", "i", 1),
    ("FABRIC_DFT_SCAN_IN5", "i", 1),
    ("FABRIC_DFT_SCAN_IN6", "i", 1),
    ("FABRIC_DFT_SCAN_RSTN", "i", 1),
    ("FABRIC_DFT_SHIFT_SCAN_EN", "i", 1),
]

# Ports only on GTR12_UPAR (GW5AT-138)
UPAR_PORTS_138_ONLY = [
    ("TL_CLKP_I", "o", 1),
    ("INET_UPAR_PMAC", "o", 5467),
    ("INET_UPAR_Q0", "o", 421),
    ("INET_UPAR_Q1", "o", 421),
    ("INET_UPAR_TEST", "o", 1329),
]


def get_quad_ports(device_name: str):
    """Return the full list of (port_name, direction, width) for the QUAD primitive."""
    from .config import GowinDevice, _is_138

    device = GowinDevice(device_name) if isinstance(device_name, str) else device_name

    ports = []
    # Per-lane ports
    lane_templates = list(QUAD_LANE_PORTS_COMMON)
    if device == GowinDevice.GW5AT_15:
        lane_templates += QUAD_LANE_PORTS_QUADB_ONLY
    for tmpl, direction, width in lane_templates:
        for ln in range(4):
            name = tmpl.format(ln=ln, LN=f"LANE{ln}")
            ports.append((name, direction, width))

    # Quad-level ports
    ports += list(QUAD_COMMON_PORTS)
    if device == GowinDevice.GW5AT_15:
        ports += list(QUAD_PORTS_QUADB_ONLY)
    elif device == GowinDevice.GW5AT_60:
        ports += list(QUAD_PORTS_QUADA_60_ONLY)
    elif _is_138(device):
        ports += list(QUAD_PORTS_QUAD_ONLY)

    return ports


def get_upar_ports(device_name: str):
    """Return the full list of (port_name, direction, width) for the UPAR primitive."""
    from .config import GowinDevice, _is_138

    device = GowinDevice(device_name) if isinstance(device_name, str) else device_name

    ports = list(UPAR_COMMON_PORTS)
    if _is_138(device):
        ports += list(UPAR_PORTS_138_ONLY)
    return ports
