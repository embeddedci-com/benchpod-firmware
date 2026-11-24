// ============================================================================
// dac_pads.v — the DAC serial pads (v66): SCLK and DIN as DDR outputs, SYNC registered.
//
// dac8551_engine gives each clk48 cycle's pin values as half pairs: `*_a` for the clk48-high
// half, `*_b` for the low half, all written on one rising edge.  A DDR SB_IO shows D_OUT_0
// (captured at the rising edge) in the high half and D_OUT_1 (captured at the FALLING edge) in
// the low half.  So D_OUT_0 takes `*_a` straight from the engine's flop, and `*_b` goes through
// one more flop (written at the next rising edge, captured at the falling edge after it): both
// halves of a pair then show in the SAME cycle, one clk48 after the engine wrote them.  D_OUT_1's
// falling-edge capture only has half a period, so it is fed by a flop with no logic in between
// (the psram_pads rule).  SYNC goes through the pad's own output register (captured at the
// rising edge), which delays it by the same one clk48, so the three pins stay aligned.
//
// Power-up: SB_IO output registers configure as 0, like the fabric flop that drove SYNC up to
// v65, so SYNC reads low for the few clk48 before rst48 sets it (as before) and SCLK idles low.
// ============================================================================
module dac_pads (
    input  wire clk48,
    input  wire sync,
    input  wire sclk_a,
    input  wire sclk_b,
    input  wire din_a,
    input  wire din_b,
    output wire dac_sync,
    output wire dac_sclk,
    output wire dac_din
);
    reg sclk_bq = 1'b0, din_bq = 1'b0;
    always @(posedge clk48) begin
        sclk_bq <= sclk_b;
        din_bq  <= din_b;
    end

    // PIN_TYPE: output DDR (PIN_TYPE[5:2] = 0100), simple input (01).
    SB_IO #(.PIN_TYPE(6'b010001), .PULLUP(1'b0)) io_sclk_i (
        .PACKAGE_PIN(dac_sclk), .OUTPUT_CLK(clk48), .D_OUT_0(sclk_a), .D_OUT_1(sclk_bq));
    SB_IO #(.PIN_TYPE(6'b010001), .PULLUP(1'b0)) io_din_i (
        .PACKAGE_PIN(dac_din),  .OUTPUT_CLK(clk48), .D_OUT_0(din_a),  .D_OUT_1(din_bq));
    // PIN_TYPE: registered output (0101), simple input (01).
    SB_IO #(.PIN_TYPE(6'b010101), .PULLUP(1'b0)) io_sync_i (
        .PACKAGE_PIN(dac_sync), .OUTPUT_CLK(clk48), .D_OUT_0(sync));
endmodule
