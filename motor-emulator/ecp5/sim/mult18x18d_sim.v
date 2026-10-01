// mult18x18d_sim.v — simulation model of the ECP5 MULT18X18D for the gate-level benches.
// yosys's ECP5 cell library has only a black box for it. Covers the way synth_ecp5 uses it:
// combinational (no input, pipeline or output registers), P = A x B, each operand signed when
// its SIGNEDx pin is high. The C port and the shift-chain options are not modelled.
`default_nettype none
module MULT18X18D (
    input  wire A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, B0, B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12, B13, B14, B15, B16, B17, C0, C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13, C14, C15, C16, C17, SIGNEDA, SIGNEDB, SOURCEA, SOURCEB,
    output wire P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15, P16, P17, P18, P19, P20, P21, P22, P23, P24, P25, P26, P27, P28, P29, P30, P31, P32, P33, P34, P35
);
    wire [17:0] a = {A17, A16, A15, A14, A13, A12, A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0};
    wire [17:0] b = {B17, B16, B15, B14, B13, B12, B11, B10, B9, B8, B7, B6, B5, B4, B3, B2, B1, B0};
    wire signed [18:0] ax = {SIGNEDA & a[17], a};
    wire signed [18:0] bx = {SIGNEDB & b[17], b};
    wire signed [37:0] p = ax * bx;
    assign {P35, P34, P33, P32, P31, P30, P29, P28, P27, P26, P25, P24, P23, P22, P21, P20, P19, P18, P17, P16, P15, P14, P13, P12, P11, P10, P9, P8, P7, P6, P5, P4, P3, P2, P1, P0} = p[35:0];
endmodule
`default_nettype wire
