// ============================================================================
// dsp_counter2.v — two independent loadable 16-bit counters in one SB_MAC16 DSP block (v42).
//
// dsp_counter chains the DSP's two 16-bit adders into one 32-bit counter.  This one keeps them
// apart (each adder takes a constant carry-in of 1 instead of the other's carry), so one DSP holds
// two counters, each with its own load, enable and direction:
//
//   qt <= load_t ? val_t : en_t ? qt +/- 1 : qt      (top,    UP_T: 1 = up, 0 = down)
//   qb <= load_b ? val_b : en_b ? qb +/- 1 : qb      (bottom, UP_B)
//
// Only the top adder's carry-out leaves the block, so only the top counter has the free flag:
//   UP_T=0: flag = (qt != 0)      UP_T=1: flag = (qt == 16'hFFFF)
// Same cycle behaviour as the fabric counters it replaces (tb_dsp_counter checks both halves
// against them).  No reset: the caller loads before use.
// ============================================================================
module dsp_counter2 #(
    parameter UP_T = 0,
    parameter UP_B = 0
)(
    input  wire        clk,
    input  wire        load_t,       // top: load val_t this edge (wins over en_t)
    input  wire [15:0] val_t,
    input  wire        en_t,
    output wire [15:0] qt,
    output wire        flag,         // UP_T=0: qt != 0   UP_T=1: qt == 16'hFFFF
    input  wire        load_b,       // bottom: load val_b this edge (wins over en_b)
    input  wire [15:0] val_b,
    input  wire        en_b,
    output wire [15:0] qb
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
        .TOPADDSUB_CARRYSELECT(2'b01),   // with a constant carry in of 1 (NOT the bottom's carry)
        .BOTOUTPUT_SELECT(2'b01),        // O[15:0] = the bottom accumulator register
        .BOTADDSUB_LOWERINPUT(2'b00),
        .BOTADDSUB_UPPERINPUT(1'b0),
        .BOTADDSUB_CARRYSELECT(2'b01),
        .MODE_8x8(1'b0), .A_SIGNED(1'b0), .B_SIGNED(1'b0)
    ) mac (
        .CLK(clk), .CE(1'b1),
        .C(val_t), .A(16'd0), .B(16'd0), .D(val_b),
        .AHOLD(1'b1), .BHOLD(1'b1), .CHOLD(1'b1), .DHOLD(1'b1),
        .IRSTTOP(1'b0), .IRSTBOT(1'b0), .ORSTTOP(1'b0), .ORSTBOT(1'b0),
        .OLOADTOP(load_t), .OLOADBOT(load_b),
        .ADDSUBTOP(UP_T ? 1'b0 : 1'b1), .ADDSUBBOT(UP_B ? 1'b0 : 1'b1),
        .OHOLDTOP(~(load_t | en_t)), .OHOLDBOT(~(load_b | en_b)),
        .CI(1'b0), .ACCUMCI(1'b0), .SIGNEXTIN(1'b0),
        .O({qt, qb}), .CO(co), .ACCUMCO(accumco), .SIGNEXTOUT()
    );
    // As in dsp_counter: subtract gives CO = (qt != 0), add gives ACCUMCO = (qt == all ones).
    assign flag = UP_T ? accumco : co;
endmodule
