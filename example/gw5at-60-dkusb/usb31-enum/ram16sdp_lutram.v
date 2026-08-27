// Behavioral replacements for Gowin RAM16SDP* distributed-RAM primitives.
//
// The vendor usb3_link.vg netlist hard-instantiates RAM16SDP1/2/4 cells.
// Despite the vendor project targeting this very GW5AT-60B device, the
// current IDE (V1.9.12.03) reports "no SSRAM resource in current device"
// (RP0007) for it -- same as on the GW5AST-138.  The packaging step
// therefore renames those instances to usb3_ram16sdp* and these
// register-based equivalents are synthesized instead (16 deep -- a
// handful of FFs and LUT muxes each).
//
// Port/parameter contract copied from Gowin prim_sim.v:
//   sync write (posedge CLK, WRE), fully asynchronous read.

module usb3_ram16sdp1 (DO, DI, WAD, RAD, WRE, CLK);
    input  CLK, WRE;
    input  [3:0] WAD;
    input  [3:0] RAD;
    input  DI;
    output DO;
    parameter INIT_0 = 16'h0000;

    reg [15:0] mem /* synthesis syn_ramstyle="registers" */;
    initial mem = INIT_0;
    always @(posedge CLK)
        if (WRE) mem[WAD] <= DI;
    assign DO = mem[RAD];
endmodule

module usb3_ram16sdp2 (DO, DI, WAD, RAD, WRE, CLK);
    input  CLK, WRE;
    input  [3:0] WAD;
    input  [3:0] RAD;
    input  [1:0] DI;
    output [1:0] DO;
    parameter INIT_0 = 16'h0000;
    parameter INIT_1 = 16'h0000;

    reg [15:0] mem0 /* synthesis syn_ramstyle="registers" */;
    reg [15:0] mem1 /* synthesis syn_ramstyle="registers" */;
    initial begin
        mem0 = INIT_0;
        mem1 = INIT_1;
    end
    always @(posedge CLK)
        if (WRE) begin
            mem0[WAD] <= DI[0];
            mem1[WAD] <= DI[1];
        end
    assign DO = {mem1[RAD], mem0[RAD]};
endmodule

module usb3_ram16sdp4 (DO, DI, WAD, RAD, WRE, CLK);
    input  CLK, WRE;
    input  [3:0] WAD;
    input  [3:0] RAD;
    input  [3:0] DI;
    output [3:0] DO;
    parameter INIT_0 = 16'h0000;
    parameter INIT_1 = 16'h0000;
    parameter INIT_2 = 16'h0000;
    parameter INIT_3 = 16'h0000;

    reg [15:0] mem0 /* synthesis syn_ramstyle="registers" */;
    reg [15:0] mem1 /* synthesis syn_ramstyle="registers" */;
    reg [15:0] mem2 /* synthesis syn_ramstyle="registers" */;
    reg [15:0] mem3 /* synthesis syn_ramstyle="registers" */;
    initial begin
        mem0 = INIT_0;
        mem1 = INIT_1;
        mem2 = INIT_2;
        mem3 = INIT_3;
    end
    always @(posedge CLK)
        if (WRE) begin
            mem0[WAD] <= DI[0];
            mem1[WAD] <= DI[1];
            mem2[WAD] <= DI[2];
            mem3[WAD] <= DI[3];
        end
    assign DO = {mem3[RAD], mem2[RAD], mem1[RAD], mem0[RAD]};
endmodule
