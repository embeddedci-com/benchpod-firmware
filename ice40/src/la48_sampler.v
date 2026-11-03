// ============================================================================
// la48_sampler.v — samples the LA channels at 48 MS/s and hands them to the 24 MHz
// `clk` domain as one PAIR per clk cycle (the change-capture producer, v50).
//
// The LA pads are combinational SB_IO inputs (la_bank), so this side is three clk48
// flops per channel: a 2-flop synchroniser (s1, s2) and one delay (s3) that holds the
// previous sample.  Every second clk48 edge — the one where `clkdiv` is 1 before the
// edge, i.e. the edge BEFORE clk rises — latches {s3, s2} = two consecutive samples
// into pa (earlier) and pb (later).
//
// clk is clkdiv through an SB_GB, so it rises one clk48 period after that latch edge
// (plus clkdiv's clock-to-out and the global buffer) and pa/pb then stay put until the
// clk48 edge after it: about one clk48 period of setup and one of hold on the clk
// capture, the same phase-locked 1:2 relationship cell_gearbox48 uses the other way.
// ============================================================================
module la48_sampler #(
    parameter N = 14
)(
    input  wire         clk48,
    input  wire         clkdiv,      // the clk48 flop that clk is buffered from (top_v2)
    input  wire [N-1:0] la_in,       // raw pad readback (asynchronous)
    output reg  [N-1:0] pa,          // sample 2k   (clk domain: stable around the clk edge)
    output reg  [N-1:0] pb           // sample 2k+1
);
    reg [N-1:0] s1, s2, s3;
    always @(posedge clk48) begin
        s1 <= la_in;
        s2 <= s1;
        s3 <= s2;
        if (clkdiv) begin
            pa <= s3;
            pb <= s2;
        end
    end
endmodule
