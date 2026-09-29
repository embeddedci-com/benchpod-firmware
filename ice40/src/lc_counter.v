// ============================================================================
// lc_counter.v — a loadable up/down counter in ONE logic cell per bit (v45).
//
//   q <= rst ? 0 : load ? val : en ? q + 1 (UP=1) / q - 1 (UP=0) : q
//
// Written as RTL, yosys maps a loadable counter to two LUTs per bit: the carry
// cell's LUT (the sum) is a fixed box for abc9, so the load mux cannot fold into
// it.  Here each bit is one hand-placed SB_LUT4 + SB_CARRY, the pair nextpnr packs
// into one logic cell with the flip-flop:
//
//   LUT  I0 = val[i]  I1 = q[i]  I2 = b  I3 = carry in      O = load ? val[i] : q[i] +/- 1
//   CARRY I0 = q[i]   I1 = b     CI = carry in              (the LUT's I1/I2, as packing needs)
//
// The counter adds b to every bit: down counts add all-ones (b = ~load, carry in 0),
// up counts add 0 with a carry in of 1 (b = load).  While loading, b flips so the LUT
// selects val[i]; the carries are then meaningless, and nobody uses them.
//
// co is the chain's carry out, a free full-width compare (valid only while !load):
//   UP=0: co = (q != 0)          UP=1: co = (q == all-ones)
//
// hold0 (UP=0 only): while set and not loading, bit 0 takes val[0] and the rest counts
// down by 2 (the carry into bit 1 is 0 instead of q[0]).  uart_engine uses it to time a
// half bit from a full-bit load.  Tie it to 0 otherwise.
//
// Simulate with the yosys iCE40 cell models (cells_sim.v), as simtest and uarttest do.
// ============================================================================
module lc_counter #(
    parameter W  = 16,
    parameter UP = 0
)(
    input  wire         clk,
    input  wire         rst,      // synchronous, wins over everything
    input  wire         load,
    input  wire [W-1:0] val,
    input  wire         en,
    input  wire         hold0,
    output reg  [W-1:0] q,
    output wire         co
);
    wire [W:0]   c;
    wire [W-1:0] d;
    wire         b  = UP ? load : ~load;
    wire         b0 = UP ? load : ~(load | hold0);
    assign c[0] = UP ? 1'b1 : 1'b0;

    // Down (b = ~load): O = I2 ? ~(I1 ^ I3) : I0.   Up (b = load): O = I2 ? I0 : I1 ^ I3.
    localparam [15:0] INIT = UP ? 16'hA3AC : 16'hCA3A;

    genvar i;
    generate for (i = 0; i < W; i = i + 1) begin : bit_
        wire bi = (i == 0) ? b0 : b;
        SB_LUT4  #(.LUT_INIT(INIT)) lut (.I0(val[i]), .I1(q[i]), .I2(bi), .I3(c[i]), .O(d[i]));
        SB_CARRY                    cy  (.I0(q[i]), .I1(bi), .CI(c[i]), .CO(c[i+1]));
    end endgenerate
    assign co = c[W];

    always @(posedge clk) begin
        if (rst)             q <= {W{1'b0}};
        else if (load | en)  q <= d;
    end
endmodule
