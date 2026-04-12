"""UPAR arbiter — round-robin DRP bus arbiter.

Plain Elaboratable with raw Signal ports. No Component — avoids all
direction conflicts when wiring between Component boundaries.
"""

from math import ceil, log2

from amaranth.hdl import Signal, Module, Const, Array, Mux, Elaboratable


class GowinUPARArbiter(Elaboratable):
    """Round-robin arbiter between DRP_NUM clients and one UPAR bus.

    All ports are plain Signals owned by this module. The parent wires
    them directly — no Component port direction issues.

    Parameters
    ----------
    drp_num : int
        Number of DRP client ports (4 for single-quad, 8 for dual-quad).
    domain : str
        Clock domain for the FSM (default "sync").
    """

    def __init__(self, drp_num: int = 4, domain: str = "sync"):
        assert drp_num >= 1
        self._drp_num = drp_num
        self._idx_bits = max(1, int(ceil(log2(drp_num + 1))))
        self._domain = domain

        N = drp_num

        # DRP client ports (arrays of signals)
        self.drp_addr = [Signal(24, name=f"drp_addr_{i}") for i in range(N)]
        self.drp_wren = [Signal(name=f"drp_wren_{i}") for i in range(N)]
        self.drp_wrdata = [Signal(32, name=f"drp_wrdata_{i}") for i in range(N)]
        self.drp_strb = [Signal(8, name=f"drp_strb_{i}") for i in range(N)]
        self.drp_rden = [Signal(name=f"drp_rden_{i}") for i in range(N)]
        self.drp_clk = [Signal(name=f"drp_clk_{i}") for i in range(N)]
        self.drp_ready = [Signal(name=f"drp_ready_{i}") for i in range(N)]
        self.drp_rdvld = [Signal(name=f"drp_rdvld_{i}") for i in range(N)]
        self.drp_rddata = [Signal(32, name=f"drp_rddata_{i}") for i in range(N)]
        self.drp_resp = [Signal(name=f"drp_resp_{i}") for i in range(N)]

        # UPAR master port
        self.upar_clk = Signal(name="upar_clk")
        self.upar_rst = Signal(name="upar_rst")
        self.upar_addr = Signal(24, name="upar_addr")
        self.upar_wren = Signal(name="upar_wren")
        self.upar_wrdata = Signal(32, name="upar_wrdata")
        self.upar_strb = Signal(8, name="upar_strb")
        self.upar_rden = Signal(name="upar_rden")
        self.upar_bus_width = Signal(name="upar_bus_width")
        self.upar_ready = Signal(name="upar_ready")
        self.upar_rdvld = Signal(name="upar_rdvld")
        self.upar_rddata = Signal(32, name="upar_rddata")

        # Debug
        self.dbg_state = Signal(2, name="dbg_arb_state")

    def elaborate(self, platform):
        m = Module()

        N = self._drp_num
        IDX = self._idx_bits
        D = self._domain

        drp_addr_a = Array(self.drp_addr)
        drp_wren_a = Array(self.drp_wren)
        drp_wrdata_a = Array(self.drp_wrdata)
        drp_strb_a = Array(self.drp_strb)
        drp_rden_a = Array(self.drp_rden)

        # Distribute UPAR clock to all DRP clients
        for i in range(N):
            m.d.comb += self.drp_clk[i].eq(self.upar_clk)

        # ── Aggregate request vector ──────────────────────────
        drp_req = Signal(N)
        for i in range(N):
            m.d.comb += drp_req[i].eq(self.drp_wren[i] | self.drp_rden[i])

        wren_vec = Signal(N)
        rden_vec = Signal(N)
        for i in range(N):
            m.d.comb += [
                wren_vec[i].eq(self.drp_wren[i]),
                rden_vec[i].eq(self.drp_rden[i]),
            ]

        drp_wren_any = Signal()
        drp_rden_any = Signal()
        m.d.comb += [
            drp_wren_any.eq(wren_vec.any()),
            drp_rden_any.eq(rden_vec.any()),
        ]

        # ── Power-on reset (16 cycles) ────────────────────────
        rstn_cnt = Signal(4, init=0)
        rstn = Signal()
        with m.If(rstn_cnt == 0xF):
            m.d[D] += rstn_cnt.eq(rstn_cnt)
        with m.Else():
            m.d[D] += rstn_cnt.eq(rstn_cnt + 1)
        m.d.comb += rstn.eq(rstn_cnt == 0xF)

        # ── Masked round-robin arbitration ─────────────────────
        drp_req_masked = Signal(N)
        mask_higher_pri_reqs = Signal(N)
        grant_masked = Signal(N)
        unmask_higher_pri_reqs = Signal(N)
        grant_unmasked = Signal(N)
        no_req_masked = Signal()
        pre_grant = Signal(N)
        cur_grant = Signal(N, init=(1 << N) - 1)

        m.d.comb += [
            drp_req_masked.eq(drp_req & cur_grant),
            mask_higher_pri_reqs[0].eq(0),
            grant_masked.eq(drp_req_masked & ~mask_higher_pri_reqs),
            unmask_higher_pri_reqs[0].eq(0),
            grant_unmasked.eq(drp_req & ~unmask_higher_pri_reqs),
            no_req_masked.eq(~drp_req_masked.any()),
        ]
        for i in range(1, N):
            m.d.comb += [
                mask_higher_pri_reqs[i].eq(
                    mask_higher_pri_reqs[i - 1] | drp_req_masked[i - 1]
                ),
                unmask_higher_pri_reqs[i].eq(
                    unmask_higher_pri_reqs[i - 1] | drp_req[i - 1]
                ),
            ]
        for i in range(N):
            m.d.comb += pre_grant[i].eq(
                Mux(no_req_masked, grant_unmasked[i], grant_masked[i])
            )

        # Priority encoder
        drp_num = Signal(IDX)
        drp_num_pre = Signal(IDX)
        with m.Switch(pre_grant):
            for i in range(N):
                with m.Case(1 << i):
                    m.d.comb += drp_num_pre.eq(i)
            with m.Default():
                m.d.comb += drp_num_pre.eq(0)

        # ── FSM ────────────────────────────────────────────────
        IDLE = 0
        JUDG_ADDR = 1
        UPAR_EN = 2
        WAIT = 3

        cur_state = Signal(2, init=IDLE)
        judg_addr_state_cnt = Signal(2)
        judg_addr_state_end = Signal()

        drp_addr_serdes = Signal(24)
        drp_wren_serdes = Signal()
        drp_wrdata_serdes = Signal(32)
        drp_strb_serdes = Signal(8)
        drp_addr_resp = Signal()

        # UPAR-side timeout: if UPARA doesn't respond in 2^20 cycles (~16ms),
        # abandon the transaction and return to IDLE. Prevents permanent lockup.
        upar_timeout = Signal(20)

        # Debug high-water mark
        with m.If(cur_state > self.dbg_state):
            m.d[D] += self.dbg_state.eq(cur_state)

        # Update cur_grant
        with m.If(~rstn):
            m.d[D] += cur_grant.eq((1 << N) - 1)
        with m.Elif(cur_state == IDLE):
            with m.If(drp_req_masked.any()):
                m.d[D] += cur_grant.eq(mask_higher_pri_reqs)
            with m.Elif(drp_req.any()):
                m.d[D] += cur_grant.eq(unmask_higher_pri_reqs)

        # Latch drp_num
        with m.If(~rstn):
            m.d[D] += drp_num.eq(0)
        with m.Elif(cur_state == IDLE):
            m.d[D] += drp_num.eq(drp_num_pre)

        # FSM transitions
        next_state = Signal(2)
        m.d.comb += next_state.eq(IDLE)
        with m.Switch(cur_state):
            with m.Case(IDLE):
                with m.If(drp_wren_any | drp_rden_any):
                    m.d.comb += next_state.eq(JUDG_ADDR)
                with m.Else():
                    m.d.comb += next_state.eq(IDLE)
            with m.Case(JUDG_ADDR):
                m.d[D] += upar_timeout.eq(0)  # reset timeout before UPAR_EN
                with m.If(judg_addr_state_end):
                    with m.If(drp_addr_resp):
                        m.d.comb += next_state.eq(WAIT)
                    with m.Else():
                        m.d.comb += next_state.eq(UPAR_EN)
                with m.Else():
                    m.d.comb += next_state.eq(JUDG_ADDR)
            with m.Case(UPAR_EN):
                m.d[D] += upar_timeout.eq(upar_timeout + 1)
                with m.If(upar_timeout[-1]):
                    # UPARA didn't respond — abandon and return to IDLE
                    m.d.comb += next_state.eq(WAIT)
                with m.Elif(drp_wren_serdes):
                    m.d.comb += next_state.eq(Mux(self.upar_ready, WAIT, UPAR_EN))
                with m.Else():
                    m.d.comb += next_state.eq(Mux(self.upar_rdvld, WAIT, UPAR_EN))
            with m.Case(WAIT):
                m.d.comb += next_state.eq(IDLE)

        with m.If(~rstn):
            m.d[D] += cur_state.eq(IDLE)
        with m.Else():
            m.d[D] += cur_state.eq(next_state)

        # JUDG_ADDR counter
        with m.If(~rstn):
            m.d[D] += judg_addr_state_cnt.eq(0)
        with m.Elif(cur_state == JUDG_ADDR):
            m.d[D] += judg_addr_state_cnt.eq(judg_addr_state_cnt + 1)
        with m.Else():
            m.d[D] += judg_addr_state_cnt.eq(0)
        m.d.comb += judg_addr_state_end.eq(judg_addr_state_cnt == 2)

        # Latch client data
        with m.If(~rstn):
            m.d[D] += [
                drp_addr_serdes.eq(0),
                drp_wren_serdes.eq(0),
                drp_wrdata_serdes.eq(0),
                drp_strb_serdes.eq(0),
            ]
        with m.Else():
            m.d[D] += [
                drp_addr_serdes.eq(drp_addr_a[drp_num]),
                drp_wren_serdes.eq(drp_wren_a[drp_num]),
                drp_wrdata_serdes.eq(drp_wrdata_a[drp_num]),
                drp_strb_serdes.eq(drp_strb_a[drp_num]),
            ]

        # Address validation (always passes)
        with m.If(~rstn):
            m.d[D] += drp_addr_resp.eq(0)
        with m.Else():
            m.d[D] += drp_addr_resp.eq(0)

        # ── UPAR outputs ──────────────────────────────────────
        upar_timed_out = upar_timeout[-1]

        # Address
        with m.If(~rstn):
            m.d[D] += self.upar_addr.eq(0)
        with m.Elif(judg_addr_state_end & ~drp_addr_resp):
            m.d[D] += self.upar_addr.eq(drp_addr_serdes)
        with m.Elif(upar_timed_out):
            m.d[D] += self.upar_addr.eq(0)
        with m.Else():
            with m.If(drp_wren_serdes):
                with m.If(self.upar_ready):
                    m.d[D] += self.upar_addr.eq(0)
            with m.Else():
                with m.If(self.upar_rdvld):
                    m.d[D] += self.upar_addr.eq(0)

        # Write
        with m.If(~rstn):
            m.d[D] += [
                self.upar_wren.eq(0),
                self.upar_wrdata.eq(0),
                self.upar_strb.eq(0),
            ]
        with m.Elif(upar_timed_out):
            m.d[D] += [
                self.upar_wren.eq(0),
                self.upar_wrdata.eq(0),
                self.upar_strb.eq(0),
            ]
        with m.Elif(judg_addr_state_end & ~drp_addr_resp & drp_wren_serdes):
            m.d[D] += [
                self.upar_wren.eq(1),
                self.upar_wrdata.eq(drp_wrdata_serdes),
                self.upar_strb.eq(drp_strb_serdes),
            ]
        with m.Elif(drp_wren_serdes & self.upar_ready):
            m.d[D] += [
                self.upar_wren.eq(0),
                self.upar_wrdata.eq(0),
                self.upar_strb.eq(0),
            ]

        # Read
        with m.If(~rstn):
            m.d[D] += self.upar_rden.eq(0)
        with m.Elif(upar_timed_out):
            m.d[D] += self.upar_rden.eq(0)
        with m.Elif(judg_addr_state_end & ~drp_addr_resp & ~drp_wren_serdes):
            m.d[D] += self.upar_rden.eq(1)
        with m.Elif(~drp_wren_serdes & self.upar_rdvld):
            m.d[D] += self.upar_rden.eq(0)
        with m.Elif(judg_addr_state_end & ~drp_addr_resp & ~drp_wren_serdes):
            m.d[D] += self.upar_rden.eq(1)
        with m.Elif(~drp_wren_serdes & self.upar_rdvld):
            m.d[D] += self.upar_rden.eq(0)

        # Constants
        m.d.comb += [self.upar_rst.eq(0), self.upar_bus_width.eq(0)]

        # ── Response routing ───────────────────────────────────
        for j in range(N):
            # ready
            with m.If(~rstn):
                m.d[D] += self.drp_ready[j].eq(0)
            with m.Elif(drp_wren_serdes & (drp_num == j)):
                with m.If((judg_addr_state_end & drp_addr_resp) | self.upar_ready):
                    m.d[D] += self.drp_ready[j].eq(1)
                with m.Else():
                    m.d[D] += self.drp_ready[j].eq(0)
            with m.Else():
                m.d[D] += self.drp_ready[j].eq(0)

            # rdvld
            with m.If(~rstn):
                m.d[D] += self.drp_rdvld[j].eq(0)
            with m.Elif(~drp_wren_serdes & (drp_num == j)):
                with m.If((judg_addr_state_end & drp_addr_resp) | self.upar_rdvld):
                    m.d[D] += self.drp_rdvld[j].eq(1)
                with m.Else():
                    m.d[D] += self.drp_rdvld[j].eq(0)
            with m.Else():
                m.d[D] += self.drp_rdvld[j].eq(0)

            # rddata
            with m.If(~rstn):
                m.d[D] += self.drp_rddata[j].eq(0)
            with m.Elif(~drp_wren_serdes & (drp_num == j)):
                with m.If(self.upar_rdvld):
                    m.d[D] += self.drp_rddata[j].eq(self.upar_rddata)
                with m.Else():
                    m.d[D] += self.drp_rddata[j].eq(0)
            with m.Else():
                m.d[D] += self.drp_rddata[j].eq(0)

            # resp
            with m.If(~rstn):
                m.d[D] += self.drp_resp[j].eq(0)
            with m.Elif(drp_num == j):
                with m.If(judg_addr_state_end & drp_addr_resp):
                    m.d[D] += self.drp_resp[j].eq(1)
                with m.Else():
                    m.d[D] += self.drp_resp[j].eq(0)
            with m.Else():
                m.d[D] += self.drp_resp[j].eq(0)

        return m
