// ============================================================================
// tb_top_boot.v — the PSRAM pads right after configuration (v52).
//
// iCE40 flip-flops configure as 0 and SB_IO output registers as 0.  Until v51 that left the
// capture writer out of reset with its CS flop at 0 for the 2-3 clk48 before rst48 reached it,
// and in the deep image the replay reader's CS at 0 made the pad mux select it, so PSRAM CS was
// driven low after configuration (no SCLK, so harmless to the chip, but not the idle state the
// STM32 expects of a bus it may own any moment).  This bench runs the whole top with the STM32
// NOT owning the bus from t = 0 and requires, for the first 64 clk48: CS high (pulled up or
// driven), SCLK never high, no IO pad driven, and no X on any of them.  Both images.
// ============================================================================
`timescale 1ns/1ps
module tb_top_boot;
    reg clk48 = 1'b0;
    always #10 clk48 = ~clk48;
    reg  sck = 1'b0, mosi = 1'b0, csn = 1'b1, bus_own = 1'b0, adc_sdo = 1'b1;
    wire miso, busy, adc_cnvst, adc_sclk, adc_sdi, dac_sync, dac_sclk, dac_din, led_g, led_b, led_r;
    tri  psram_sclk, psram_cs, psram_io0, psram_io1, psram_io2, psram_io3;
    pullup (psram_cs); pulldown (psram_sclk);           // the board's CS pull-up; SCLK idles low
    wire [13:0] la = 14'h0000;
    top dut (
        .clk48(clk48), .sck(sck), .mosi(mosi), .miso(miso), .csn(csn), .busy(busy), .bus_own(bus_own),
        .adc_cnvst(adc_cnvst), .adc_sclk(adc_sclk), .adc_sdi(adc_sdi), .adc_sdo(adc_sdo),
        .dac_sync(dac_sync), .dac_sclk(dac_sclk), .dac_din(dac_din),
        .psram_sclk(psram_sclk), .psram_cs(psram_cs), .psram_io0(psram_io0), .psram_io1(psram_io1),
        .psram_io2(psram_io2), .psram_io3(psram_io3), .la(la), .led_g(led_g), .led_b(led_b), .led_r(led_r));

    integer errors = 0, k;
    wire [3:0] io = {psram_io3, psram_io2, psram_io1, psram_io0};
    task check(input integer when);
        begin
            if (psram_cs !== 1'b1 || psram_sclk !== 1'b0 || io !== 4'bzzzz) begin
                if (errors < 6) $display("FAIL tb_top_boot: %0d clk48 after configuration: CS=%b SCLK=%b IO=%b (want 1, 0, zzzz)",
                                         when, psram_cs, psram_sclk, io);
                errors = errors + 1;
            end
        end
    endtask
    initial begin
        #1 check(0);
        for (k = 1; k <= 64; k = k + 1) begin
            @(posedge clk48); #1 check(k);
            @(negedge clk48); #1 check(k);
        end
        if (errors == 0) $display("PASS tb_top_boot: PSRAM CS high, SCLK low, IO released from configuration on");
        else             $display("FAIL tb_top_boot: %0d error(s)", errors);
        $finish;
    end
endmodule
