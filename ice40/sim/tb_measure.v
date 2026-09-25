// tb_measure.v — self-checking testbench for the cmd_dispatch side of `measure`.
//
// v40 retired OP_START_MEASURE (0x30).  `measure` is now DAC_ARM_ON_CAPTURE (0x19) +
// START_DAC (0x11) + OP_CAPTURE (0x31): the co-trigger holds the DAC start until the capture's
// t0, so the DAC and the ADC still start on the same cycle (tb_top_capture checks that end to
// end, frames and samples).  This bench checks the dispatcher half:
//   * a stray 0x30 is inert: no DAC or capture start, no register change;
//   * the replacement sequence decodes: the co-trigger strobe, START_DAC's period + divider,
//     OP_CAPTURE's ADC count + divider, with the LA left out (count 0, its divider kept);
//   * DAC SOURCE SELECTION (gateware >= 31): dac_psram_mode / dac_loop_mode are LEVEL
//     registers, so a BRAM-waveform start (START_DAC) must clear both, or a `generate` after
//     a deep `replay` keeps streaming from PSRAM (and contends with a capture for the bus).
`timescale 1ns/1ps
module tb_measure;
    reg clk = 0;
    always #5 clk = ~clk;

    reg        rst = 1;
    reg  [7:0] rx_byte = 8'h00;
    reg        rx_valid = 0;
    reg        cs_active = 0;

    wire        dac_start, dac_stop, cap_start, la_cap_start, dac_cotrig_stb;
    wire [12:0] dac_period;
    wire [23:0] cap_count, la_cap_count;
    wire [15:0] dac_divider, cap_divider, la_cap_divider;
    wire        dac_psram_mode, dac_loop_mode;

    cmd_dispatch #(.GATEWARE_VERSION(8'd6)) dut (
        .clk(clk), .rst(rst),
        .rx_byte(rx_byte), .rx_valid(rx_valid), .cs_active(cs_active),
        .dac_start(dac_start), .dac_stop(dac_stop),
        .dac_period(dac_period), .dac_divider(dac_divider),
        .cap_start(cap_start), .cap_count(cap_count), .cap_divider(cap_divider),
        .la_cap_start(la_cap_start), .la_cap_count(la_cap_count), .la_cap_divider(la_cap_divider),
        .dac_cotrig_stb(dac_cotrig_stb),
        .dac_psram_mode(dac_psram_mode), .dac_loop_mode(dac_loop_mode)
        // remaining ports intentionally unconnected (not exercised here)
    );

    // Feed one byte to the dispatcher (sampled on the rising edge while valid).
    task feed(input [7:0] b);
        begin
            @(negedge clk); rx_byte = b; rx_valid = 1;
            @(negedge clk); rx_valid = 0;
        end
    endtask
    /* CS framing, same idiom as tb_dispatch_args: the FSM only returns to IDLE on
       the CS release edge, and that needs a couple of idle clocks to land. */
    task cs_hi; begin @(negedge clk); cs_active = 1; end endtask
    task cs_lo; begin @(negedge clk); cs_active = 0; @(negedge clk); @(negedge clk); end endtask

    integer errors = 0;
    // strobe counters (each opcode must fire exactly the strobes it owns)
    integer n_dac = 0, n_cap = 0, n_la = 0, n_cot = 0;
    always @(posedge clk) begin
        if (dac_start)      n_dac = n_dac + 1;
        if (cap_start)      n_cap = n_cap + 1;
        if (la_cap_start)   n_la  = n_la  + 1;
        if (dac_cotrig_stb) n_cot = n_cot + 1;
    end
    reg [15:0] cap_div0, la_div0;

    initial begin
        repeat (4) @(negedge clk); rst = 0;

        // ---- a retired OP_START_MEASURE (0x30) is inert ----
        cap_div0 = cap_divider; la_div0 = la_cap_divider;
        cs_hi;
        feed(8'h30); feed(8'h40); feed(8'h01); feed(8'h1D); feed(8'h00); feed(8'h50); feed(8'h00);
        @(posedge clk); #1;
        cs_lo;
        if (n_dac != 0 || n_cap != 0 || n_la != 0 || cap_divider !== cap_div0 || dac_divider !== 16'd2) begin
            $display("FAIL tb_measure: retired 0x30 acted (dac %0d cap %0d la %0d starts, cap_div %0d, dac_div %0d)",
                     n_dac, n_cap, n_la, cap_divider, dac_divider);
            errors = errors + 1; end

        // ---- the v40 `measure` sequence ----
        cs_hi; feed(8'h19); @(posedge clk); #1; cs_lo;                        // DAC_ARM_ON_CAPTURE
        cs_hi; feed(8'h11); feed(8'h40); feed(8'h01); feed(8'h1D); feed(8'h00); // START_DAC 320 / 29
        @(posedge clk); #1; cs_lo;
        cs_hi; feed(8'h31);                                                   // OP_CAPTURE
        feed(8'h40); feed(8'h01); feed(8'h00); feed(8'h50); feed(8'h00);       // ADC 320 @ 80
        feed(8'h00); feed(8'h00); feed(8'h00); feed(8'h09); feed(8'h00);       // LA 0 (div 9 ignored)
        @(posedge clk); #1;
        if (n_cot != 1 || n_dac != 1 || n_cap != 1) begin
            $display("FAIL tb_measure: sequence fired cotrig %0d / dac %0d / cap %0d (want 1 each)", n_cot, n_dac, n_cap);
            errors = errors + 1; end
        if (dac_period  !== 13'h140) begin
            $display("FAIL tb_measure: dac_period=%h want 140", dac_period);   errors = errors + 1; end
        if (cap_count   !== 24'h000140) begin
            $display("FAIL tb_measure: cap_count=%h want 140", cap_count);     errors = errors + 1; end
        if (dac_divider !== 16'd29)  begin
            $display("FAIL tb_measure: dac_divider=%0d want 29", dac_divider); errors = errors + 1; end
        if (cap_divider !== 16'd80)  begin
            $display("FAIL tb_measure: cap_divider=%0d want 80", cap_divider); errors = errors + 1; end
        if (la_cap_count !== 24'd0 || la_cap_divider !== la_div0) begin
            $display("FAIL tb_measure: LA count %0d / divider %0d (want 0 / unchanged %0d): an LA-less capture moved the LA divider",
                     la_cap_count, la_cap_divider, la_div0); errors = errors + 1; end
        cs_lo;

        // ---- an LA-only capture keeps the ADC divider (it paces the free-running ADC) ----
        cs_hi; feed(8'h31);
        feed(8'h00); feed(8'h00); feed(8'h00); feed(8'h07); feed(8'h00);       // ADC 0 (div 7 ignored)
        feed(8'h10); feed(8'h00); feed(8'h00); feed(8'h06); feed(8'h00);       // LA 16 @ 6
        @(posedge clk); #1;
        if (cap_divider !== 16'd80 || la_cap_divider !== 16'd6) begin
            $display("FAIL tb_measure: LA-only capture: cap_div %0d (want 80 kept), la_div %0d (want 6)",
                     cap_divider, la_cap_divider); errors = errors + 1; end
        cs_lo;

        // Same for a plain BRAM-waveform start (0x11) after a deep replay: this is
        // the `generate`-after-`replay` case, which was silently still replaying.
        cs_hi;
        feed(8'h13); feed(8'h00); feed(8'h00); feed(8'h40);
        feed(8'h00); feed(8'h10); feed(8'h00);
        feed(8'h04); feed(8'h00);
        @(posedge clk); #1;
        cs_lo;
        cs_hi;
        feed(8'h11); feed(8'h40); feed(8'h01); feed(8'h1D); feed(8'h00);  // START_DAC
        @(posedge clk); #1;
        if (dac_psram_mode !== 1'b0) begin
            $display("FAIL tb_measure: START_DAC left dac_psram_mode SET — `generate` after a deep `replay` keeps streaming from PSRAM instead of the loaded waveform");
            errors = errors + 1; end
        if (dac_loop_mode !== 1'b0) begin
            $display("FAIL tb_measure: START_DAC left dac_loop_mode SET"); errors = errors + 1; end
        cs_lo;

        if (errors == 0)
            $display("PASS tb_measure: retired 0x30 inert; the co-trigger measure sequence decodes; an absent producer keeps its divider; START_DAC reselects the BRAM source");
        $finish;
    end

    initial begin #100000 $display("FAIL tb_measure: timeout"); $finish; end
endmodule
