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
// K / CI (v51) generalise the step: q <= q + K + CI (mod 2^W) when en.  The defaults are the
// counter above (UP=1: K = 0, CI = 1; UP=0: K = all-ones, CI = 0).  A bit whose K bit is 1 is a
// down-count cell (b = ~load), a bit whose K bit is 0 an up-count cell (b = load): either way the
// carry adds K's bit, and the LUT selects val[i] while loading.  co is then the carry out of
// q + K + CI (for a subtract of k, K = -k: co = (q >= k)).  la_change_capture subtracts 6.
//
// Simulate with the yosys iCE40 cell models (cells_sim.v), as simtest and uarttest do.
// ============================================================================
module lc_counter #(
    parameter W  = 16,
    parameter UP = 0,
    parameter [W-1:0] K = UP ? {W{1'b0}} : {W{1'b1}},
    parameter CI = UP
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
    assign c[0] = CI ? 1'b1 : 1'b0;

    // Down (b = ~load): O = I2 ? ~(I1 ^ I3) : I0.   Up (b = load): O = I2 ? I0 : I1 ^ I3.
    genvar i;
    generate for (i = 0; i < W; i = i + 1) begin : bit_
        wire bi = K[i] ? ((i == 0) ? ~(load | hold0) : ~load) : load;
        SB_LUT4  #(.LUT_INIT(K[i] ? 16'hCA3A : 16'hA3AC)) lut (.I0(val[i]), .I1(q[i]), .I2(bi), .I3(c[i]), .O(d[i]));
        SB_CARRY                    cy  (.I0(q[i]), .I1(bi), .CI(c[i]), .CO(c[i+1]));
    end endgenerate
    assign co = c[W];

    always @(posedge clk) begin
        if (rst)             q <= {W{1'b0}};
        else if (load | en)  q <= d;
    end
endmodule
