// tb_measure.v — self-checking testbench for the cmd_dispatch MEASURE decode.
// Feeds a START_MEASURE (0x30) 6-byte command with count=0x140 and SEPARATE
// dac_div=29 / cap_div=80, and checks the dispatcher co-asserts dac_start AND
// cap_start in the same cycle (phase-locked DAC waveform + ADC->PSRAM capture)
// with the matching period/count and the two independent dividers — the gateware
// half of the v2 `measure` path (gateware >= 12).
//
// Also (gateware >= 31) regression-locks DAC SOURCE SELECTION: dac_psram_mode and
// dac_loop_mode are LEVEL registers, and until v31 only OP_STOP_DAC cleared them.
// So OP_START_MEASURE (which cleared neither) and OP_START_DAC (which cleared only
// dac_loop_mode) kicked the DAC engine with a PREVIOUS source still selected.  In
// the deep image that leaves the dac_psram_reader requesting the shared quad bus
// while measure's own capture writer needs it — silent capture corruption; in the
// loop image the engine starves and the DAC holds a constant level.  A BRAM-
// waveform start must select the BRAM waveform, so both opcodes must clear both.
`timescale 1ns/1ps
module tb_measure;
    reg clk = 0;
    always #5 clk = ~clk;

    reg        rst = 1;
    reg  [7:0] rx_byte = 8'h00;
    reg        rx_valid = 0;
    reg        cs_active = 0;

    wire        dac_start, dac_stop, cap_start;
    wire [12:0] dac_period;
    wire [23:0] cap_count;   // 24-bit: matches cmd_dispatch's deep ADC capture count
    wire [15:0] dac_divider, cap_divider;
    wire        dac_psram_mode, dac_loop_mode;

    cmd_dispatch #(.GATEWARE_VERSION(8'd6)) dut (
        .clk(clk), .rst(rst),
        .rx_byte(rx_byte), .rx_valid(rx_valid), .cs_active(cs_active),
        .dac_start(dac_start), .dac_stop(dac_stop),
        .dac_period(dac_period), .dac_divider(dac_divider),
        .cap_start(cap_start), .cap_count(cap_count), .cap_divider(cap_divider),
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
    reg     saw_both = 0;
    // dac_start & cap_start must be asserted together (same FPGA cycle).
    always @(posedge clk) if (dac_start && cap_start) saw_both = 1;

    initial begin
        repeat (4) @(negedge clk); rst = 0;
        cs_hi;
        feed(8'h30);   // OP_START_MEASURE
        feed(8'h40);   // count_lo
        feed(8'h01);   // count_hi   -> count   = 0x0140 (320)
        feed(8'h1D);   // dac_div_lo
        feed(8'h00);   // dac_div_hi -> dac_div = 0x001D (29)
        feed(8'h50);   // cap_div_lo
        feed(8'h00);   // cap_div_hi -> cap_div = 0x0050 (80)
        @(posedge clk); #1;

        if (!saw_both) begin
            $display("FAIL tb_measure: dac_start & cap_start not co-asserted"); errors = errors + 1; end
        if (dac_period  !== 13'h140) begin
            $display("FAIL tb_measure: dac_period=%h want 140", dac_period);   errors = errors + 1; end
        if (cap_count   !== 24'h000140) begin
            $display("FAIL tb_measure: cap_count=%h want 140", cap_count);     errors = errors + 1; end
        if (dac_divider !== 16'd29)  begin
            $display("FAIL tb_measure: dac_divider=%0d want 29", dac_divider); errors = errors + 1; end
        if (cap_divider !== 16'd80)  begin
            $display("FAIL tb_measure: cap_divider=%0d want 80", cap_divider); errors = errors + 1; end
        cs_lo;

        // ---- source-select regression (v31) ----------------------------------
        // Arm DEEP PSRAM replay (0x13, 8-byte payload) so dac_psram_mode is set,
        // then MEASURE: the measure must take the DAC back to the BRAM waveform.
        cs_hi;
        feed(8'h13); feed(8'h00); feed(8'h00); feed(8'h40);   // base = 0x400000
        feed(8'h00); feed(8'h10); feed(8'h00);                // len  = 0x001000 samples
        feed(8'h04); feed(8'h00);                             // div  = 4
        @(posedge clk); #1;
        if (dac_psram_mode !== 1'b1) begin
            $display("FAIL tb_measure: START_DAC_PSRAM did not set dac_psram_mode"); errors = errors + 1; end
        cs_lo;

        cs_hi;
        feed(8'h30); feed(8'h40); feed(8'h01);
        feed(8'h1D); feed(8'h00); feed(8'h50); feed(8'h00);   // MEASURE again
        @(posedge clk); #1;
        if (dac_psram_mode !== 1'b0) begin
            $display("FAIL tb_measure: MEASURE left dac_psram_mode SET — the deep reader stays the DAC source and contends with measure's own capture writer for the shared PSRAM bus");
            errors = errors + 1; end
        if (dac_loop_mode !== 1'b0) begin
            $display("FAIL tb_measure: MEASURE left dac_loop_mode SET"); errors = errors + 1; end
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
            $display("PASS tb_measure: MEASURE co-starts DAC+ADC (period=%0d count=%0d dac_div=%0d cap_div=%0d) and both DAC starts reselect the BRAM source",
                     dac_period, cap_count, dac_divider, cap_divider);
        $finish;
    end

    initial begin #100000 $display("FAIL tb_measure: timeout"); $finish; end
endmodule
