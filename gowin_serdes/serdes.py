"""Top-level GowinSerDes Component.

The user creates GowinSerDesGroup objects and passes them here.
GowinSerDes is a wiring.Component with:
  - One DRP port per group (In(DRPSignature()) — user drives requests)
  - A por_n input
  - Internal: arbiter, QUAD Instances, UPAR Instance, lane wiring

All DRP signals live inside this Component's fragment, so there are
no cross-boundary signal ownership issues.
"""

from typing import List, Dict, Optional

from collections import defaultdict

from amaranth.hdl import Signal, Module, Const, ClockDomain, ClockSignal, ResetSignal
from amaranth.lib.wiring import Component, In, Out

from .config import (
    GowinDevice,
    _is_138,
    device_num_quads,
    device_drp_num,
    device_is_quadb,
)
from .signature import DRPSignature
from .group import GowinSerDesGroup
from .upar_arbiter import GowinUPARArbiter
from .quad import GowinSerDesQuadInstance, GowinUPARInstance
from .toml_gen import (
    build_toml_config as _build_toml_config,
    generate_toml as _generate_toml,
    generate_csr as _generate_csr,
)


class GowinSerDes(Component):
    """Top-level Gowin SerDes assembly.

    Parameters
    ----------
    device : GowinDevice
        Target device.
    groups : list of GowinSerDesGroup
        Lane groups to instantiate.

    Signature ports (dynamically built):
        por_n           : In(1)
        drp_q{Q}_ln{L}  : In(DRPSignature())   — one per group
    """

    def __init__(self, device: GowinDevice, groups: List[GowinSerDesGroup]):
        self.device = device
        self.groups = groups
        self.drp_num = device_drp_num(device)
        self.num_quads = device_num_quads(device)

        self._validate()

        self._groups_by_quad: Dict[int, List[GowinSerDesGroup]] = defaultdict(list)
        for g in groups:
            self._groups_by_quad[g.quad].append(g)

        # Build signature: por_n + one DRP port per group
        members = {
            "por_n": In(1),
            "dbg_arb_state": Out(2),
        }
        for g in groups:
            # In(DRPSignature()) means: the user (outside) is the DRP client.
            #   User drives Out members of DRP (addr, wren, wrdata, strb, rden)
            #   User reads In members of DRP (clk, ready, rdvld, rddata, resp)
            # From GowinSerDes's perspective (In), directions flip:
            #   GowinSerDes samples addr, wren, etc. and drives clk, ready, etc.
            members[g.drp_name] = In(DRPSignature())
        super().__init__(members)

    # ── TOML / CSR generation ─────────────────────────────────

    def toml_config(self) -> Dict:
        """Return the full TOML config dict for this SerDes instance.

        Useful for inspection or manual tweaking before writing.
        """
        return _build_toml_config(self.device, self.groups)

    def generate_toml(self, output_path: str) -> str:
        """Write a Gowin-compatible TOML file for this SerDes instance.

        Parameters
        ----------
        output_path : str
            Where to write the ``.toml`` file.

        Returns
        -------
        str
            The *output_path*.
        """
        return _generate_toml(self.device, self.groups, output_path)

    def generate_csr(
        self,
        output_path: str,
        toml_path: Optional[str] = None,
        gowin_bin_dir: Optional[str] = None,
    ) -> str:
        """Generate the ``.csr`` binary blob for this SerDes instance.

        Writes an intermediate TOML file, then invokes Gowin's
        ``serdes_toml_to_csr_*k.bin`` tool to produce the CSR blob.

        Parameters
        ----------
        output_path : str
            Where to write the ``.csr`` file.
        toml_path : str or None
            If given, keep the intermediate TOML file at this path.
            Otherwise a temporary file is used and cleaned up.
        gowin_bin_dir : str or None
            Explicit path to the Gowin IDE ``bin/`` directory.
            If None, the tool is searched on ``$PATH``,
            then ``$GOWIN_IDE/bin/``.

        Returns
        -------
        str
            The *output_path*.

        Raises
        ------
        FileNotFoundError
            If the Gowin CSR tool binary is not found.
        subprocess.CalledProcessError
            If the tool exits non-zero.
        """
        return _generate_csr(
            self.device,
            self.groups,
            output_path,
            toml_path=toml_path,
            gowin_bin_dir=gowin_bin_dir,
        )

    # ── Validation ─────────────────────────────────────────────

    def _validate(self):
        max_quads = self.num_quads
        occupied = set()
        for g in self.groups:
            assert 0 <= g.quad < max_quads, (
                f"Group quad={g.quad} exceeds device max ({max_quads})"
            )
            for i in range(g.num_lanes):
                lane_abs = (g.quad, g.first_lane + i)
                assert lane_abs not in occupied, (
                    f"Lane slot {lane_abs} used by multiple groups"
                )
                occupied.add(lane_abs)

    def elaborate(self, platform):
        m = Module()

        # ── UPAR clock domain ──────────────────────────────────
        life_clk = Signal(name="life_clk")
        ahb_clk = Signal(name="ahb_clk")
        ahb_rstn = Signal(name="ahb_rstn")
        test_dec_en = Signal(name="test_dec_en")

        m.domains += ClockDomain("upar")
        m.d.comb += [
            ClockSignal("upar").eq(life_clk),
            ResetSignal("upar").eq(Const(0)),
        ]

        # ── UPAR Arbiter (plain Elaboratable, runs on upar clock) ──
        arbiter = GowinUPARArbiter(drp_num=self.drp_num, domain="upar")
        m.submodules.arbiter = arbiter

        # Wire arbiter UPAR port to life_clk
        m.d.comb += arbiter.upar_clk.eq(life_clk)

        upar_sigs = {
            "rst": arbiter.upar_rst,
            "addr": arbiter.upar_addr,
            "wren": arbiter.upar_wren,
            "wrdata": arbiter.upar_wrdata,
            "strb": arbiter.upar_strb,
            "rden": arbiter.upar_rden,
            "bus_width": arbiter.upar_bus_width,
            "rddata": arbiter.upar_rddata,
            "rdvld": arbiter.upar_rdvld,
            "ready": arbiter.upar_ready,
        }
        m.d.comb += self.dbg_arb_state.eq(arbiter.dbg_state)

        # ── INET signals (GW5AT-138) ──────────────────────────
        inet_q = {}
        inet_upar = {}
        if _is_138(self.device):
            for qi in range(self.num_quads):
                inet_q[qi] = {
                    "INET_Q0_Q1": Signal(92, name=f"q{qi}_inet_q0_q1"),
                    "INET_Q_PMAC": Signal(532, name=f"q{qi}_inet_q_pmac"),
                    "INET_Q_TEST": Signal(228, name=f"q{qi}_inet_q_test"),
                    "INET_Q_UPAR": Signal(421, name=f"q{qi}_inet_q_upar"),
                }
            inet_upar = {
                "TL_CLKP_I": Signal(1, name="tl_clkp_i"),
                "INET_UPAR_PMAC": Signal(5467, name="inet_upar_pmac"),
                "INET_UPAR_Q0": Signal(421, name="inet_upar_q0"),
                "INET_UPAR_Q1": Signal(421, name="inet_upar_q1"),
                "INET_UPAR_TEST": Signal(1329, name="inet_upar_test"),
            }

        # ── Per-Quad: QUAD hard macro ──────────────────────────
        is_quadb = device_is_quadb(self.device)
        for qi in range(self.num_quads):
            groups_in_quad = self._groups_by_quad.get(qi, [])
            lane_signals = {}
            for group in groups_in_quad:
                for li, lane in enumerate(group.lanes):
                    abs_lane = group.first_lane + li
                    lane_signals[abs_lane] = self._build_lane_port_map(
                        lane, abs_lane, is_quadb
                    )

            # For multi-quad devices, only Q0 drives the global life_clk.
            # Other quads' FABRIC_CM_LIFE_CLK_O goes to a dummy signal
            # to avoid multiple-driver conflicts.
            if qi == 0:
                q_life_clk_out = life_clk
            else:
                q_life_clk_out = Signal(name=f"q{qi}_life_clk_out")

            quad_hw = GowinSerDesQuadInstance(
                device=self.device,
                quad_idx=qi,
                lane_signals=lane_signals,
                por_n=self.por_n,
                life_clk=life_clk,
                ahb_rstn=ahb_rstn,
                test_dec_en=test_dec_en,
                inet_signals=inet_q.get(qi, {}),
                life_clk_out=q_life_clk_out,
            )
            m.submodules[f"quad{qi}"] = quad_hw

        # ── UPAR hard macro (singleton) ────────────────────────
        upar_hw = GowinUPARInstance(
            device=self.device,
            upar_signals=upar_sigs,
            life_clk=life_clk,
            ahb_clk=ahb_clk,
            ahb_rstn=ahb_rstn,
            test_dec_en=test_dec_en,
            inet_signals=inet_upar,
        )
        m.submodules.upar = upar_hw

        # ── Wire Component DRP ports → arbiter slots ──────────
        for group in self.groups:
            m.submodules[f"group_q{group.quad}_ln{group.first_lane}"] = group

            slot = group.arbiter_slot
            my_port = getattr(self, group.drp_name)

            # Wire GowinSerDes Component port ↔ arbiter raw Signal arrays.
            # No direction conflicts — arbiter is a plain Elaboratable.
            # Request: user → my_port → arbiter.drp_*[slot]
            m.d.comb += [
                arbiter.drp_addr[slot].eq(my_port.addr),
                arbiter.drp_wren[slot].eq(my_port.wren),
                arbiter.drp_wrdata[slot].eq(my_port.wrdata),
                arbiter.drp_strb[slot].eq(my_port.strb),
                arbiter.drp_rden[slot].eq(my_port.rden),
            ]
            # Response: arbiter.drp_*[slot] → my_port → user
            m.d.comb += [
                my_port.clk.eq(arbiter.drp_clk[slot]),
                my_port.ready.eq(arbiter.drp_ready[slot]),
                my_port.rdvld.eq(arbiter.drp_rdvld[slot]),
                my_port.rddata.eq(arbiter.drp_rddata[slot]),
                my_port.resp.eq(arbiter.drp_resp[slot]),
            ]
            # Response: arbiter → serdes port → user
            m.d.comb += [
                my_port.clk.eq(arbiter.drp_clk[slot]),
                my_port.ready.eq(arbiter.drp_ready[slot]),
                my_port.rdvld.eq(arbiter.drp_rdvld[slot]),
                my_port.rddata.eq(arbiter.drp_rddata[slot]),
                my_port.resp.eq(arbiter.drp_resp[slot]),
            ]
        return m

    def _build_lane_port_map(self, lane, lane_idx, is_quadb):
        """Build dict mapping QUAD port names to lane internal signals."""
        li = lane_idx
        ports = {
            f"FABRIC_LN{li}_TXDATA_I": lane._quad_txdata,
            f"FABRIC_LN{li}_TX_VLD_IN": lane._quad_tx_vld,
            f"FABRIC_LN{li}_TX_DISPARITY_I": lane._quad_tx_disparity,
            f"FABRIC_LN{li}_RXDATA_O": lane._quad_rxdata,
            f"FABRIC_LN{li}_RX_VLD_OUT": lane._quad_rx_vld,
            f"FABRIC_LN{li}_ASTAT_O": lane._quad_astat,
            f"FABRIC_LN{li}_STAT_O": lane._quad_stat,
            f"FABRIC_LN{li}_PMA_RX_LOCK_O": lane._quad_pma_rx_lock,
            f"FABRIC_LANE{li}_CMU_CK_REF_O": lane._quad_cmu_ck_ref,
            f"FABRIC_LANE{li}_CMU_OK_O": lane._quad_cmu_ok,
            f"LANE{li}_ALIGN_LINK": lane._quad_align_link,
            f"LANE{li}_K_LOCK": lane._quad_k_lock,
            f"LANE{li}_PCS_RX_O_FABRIC_CLK": lane._quad_rx_pcs_clk,
            f"LANE{li}_PCS_TX_O_FABRIC_CLK": lane._quad_tx_pcs_clk,
            f"LANE{li}_RX_IF_FIFO_RDUSEWD": lane._quad_rx_fifo_rdusewd,
            f"LANE{li}_RX_IF_FIFO_AEMPTY": lane._quad_rx_fifo_aempty,
            f"LANE{li}_RX_IF_FIFO_EMPTY": lane._quad_rx_fifo_empty,
            f"LANE{li}_TX_IF_FIFO_WRUSEWD": lane._quad_tx_fifo_wrusewd,
            f"LANE{li}_TX_IF_FIFO_AFULL": lane._quad_tx_fifo_afull,
            f"LANE{li}_TX_IF_FIFO_FULL": lane._quad_tx_fifo_full,
            f"FABRIC_LN{li}_RSTN_I": lane._quad_rstn,
            f"LANE{li}_PCS_RX_RST": lane._quad_pcs_rx_rst,
            f"LANE{li}_PCS_TX_RST": lane._quad_pcs_tx_rst,
            f"LANE{li}_FABRIC_RX_CLK": lane._quad_rx_clk,
            f"LANE{li}_FABRIC_TX_CLK": lane._quad_tx_clk,
            f"LANE{li}_FABRIC_C2I_CLK": lane._quad_c2i_clk,
            f"LANE{li}_RX_IF_FIFO_RDEN": lane._quad_fifo_rden,
            f"LANE{li}_CHBOND_START": lane._quad_chbond_start,
        }
        if is_quadb:
            ports[f"FABRIC_LANE{li}_64B66B_TX_INVLD_BLK"] = (
                lane._quad_64b66b_tx_invld_blk
            )
            ports[f"FABRIC_LANE{li}_64B66B_TX_FETCH"] = lane._quad_64b66b_tx_fetch
            ports[f"FABRIC_LANE{li}_64B66B_RX_VALID"] = lane._quad_64b66b_rx_valid
        return ports
