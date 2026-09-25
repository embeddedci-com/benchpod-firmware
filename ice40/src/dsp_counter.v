// ============================================================================
// dsp_counter.v — a loadable 32-bit counter in one SB_MAC16 DSP block (v41).
//
// The up5k has 8 DSP blocks and the gateware used 2 (the control loop's multipliers), while
// logic cells were the scarce resource: a 24/32-bit fabric counter costs one LC per bit (LUT +
// carry + flip-flop) plus the LUTs of any terminal compare.  An SB_MAC16 in accumulator mode
// is exactly such a counter: its two 16-bit adders chain into one 32-bit add/subtract whose
// output register feeds back as the accumulator.  This wraps that configuration:
//
//   q <= load ? load_val : en ? q + 1 (UP=1) / q - 1 (UP=0) : q
//
// and exposes the adder's carry-out, which is a free 32-bit compare:
//   UP=0 (count down):  flag = (q != 0)      — the borrow out of q - 1
//   UP=1 (count up):    flag = (q == ~0)     — the carry out of q + 1
//
// Same clock, same cycle behaviour as the fabric `always` it replaces (tb_dsp_counter checks
// it against one, both directions, with load and enable in every combination).  Everything
// stays in the caller's clock domain; there is no reset (the caller loads before use, as it
// did before).  q is the registered accumulator output, so it is valid a clk-to-q after the
// edge like any flip-flop.
// ============================================================================
module dsp_counter #(
    parameter UP = 0                 // 1 = count up, 0 = count down
)(
    input  wire        clk,
    input  wire        load,         // load load_val this edge (wins over en)
    input  wire [31:0] load_val,
    input  wire        en,           // count this edge
    output wire [31:0] q,
    output wire        flag          // UP=0: q != 0   UP=1: q == 32'hFFFFFFFF
);
    wire co, accumco;
    SB_MAC16 #(
        .NEG_TRIGGER(1'b0),
        .C_REG(1'b0), .A_REG(1'b0), .B_REG(1'b0), .D_REG(1'b0),
        .TOP_8x8_MULT_REG(1'b0), .BOT_8x8_MULT_REG(1'b0),
        .PIPELINE_16x16_MULT_REG1(1'b0), .PIPELINE_16x16_MULT_REG2(1'b0),
        .TOPOUTPUT_SELECT(2'b01),        // O[31:16] = the top accumulator register
        .TOPADDSUB_LOWERINPUT(2'b00),    // top adds A (tied 0) ...
        .TOPADDSUB_UPPERINPUT(1'b0),     // ... to its own accumulator
        .TOPADDSUB_CARRYSELECT(2'b10),   // carry in = the bottom adder's carry out (32-bit chain)
        .BOTOUTPUT_SELECT(2'b01),        // O[15:0] = the bottom accumulator register
        .BOTADDSUB_LOWERINPUT(2'b00),    // bottom adds B (tied 0) ...
        .BOTADDSUB_UPPERINPUT(1'b0),     // ... to its own accumulator
        .BOTADDSUB_CARRYSELECT(2'b01),   // with a constant carry in of 1: the +/-1
        .MODE_8x8(1'b0), .A_SIGNED(1'b0), .B_SIGNED(1'b0)
    ) mac (
        .CLK(clk), .CE(1'b1),
        .C(load_val[31:16]), .A(16'd0), .B(16'd0), .D(load_val[15:0]),
        .AHOLD(1'b1), .BHOLD(1'b1), .CHOLD(1'b1), .DHOLD(1'b1),
        .IRSTTOP(1'b0), .IRSTBOT(1'b0), .ORSTTOP(1'b0), .ORSTBOT(1'b0),
        .OLOADTOP(load), .OLOADBOT(load),
        .ADDSUBTOP(UP ? 1'b0 : 1'b1), .ADDSUBBOT(UP ? 1'b0 : 1'b1),
        .OHOLDTOP(~(load | en)), .OHOLDBOT(~(load | en)),
        .CI(1'b0), .ACCUMCI(1'b0), .SIGNEXTIN(1'b0),
        .O(q), .CO(co), .ACCUMCO(accumco), .SIGNEXTOUT()
    );
    // Subtract: ACCUMCO = (q == 0), CO = ACCUMCO ^ 1 = (q != 0).  Add: ACCUMCO = (q == ~0).
    assign flag = UP ? accumco : co;
endmodule
