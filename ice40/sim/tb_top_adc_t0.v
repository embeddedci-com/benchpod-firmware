// ============================================================================
// tb_top_adc_t0.v — the ADC capture's timebase starts at the capture's t0 (v52).
//
// The ADC engine free-runs between captures.  Until v51 a capture took whatever conversion
// was in flight at t0 as its sample 0: converted up to a period BEFORE t0, or up to the
// previous capture's period after it, with the first interval at the previous divider.  The LA
// producer was exact, so "sample i of both streams at the same instant" held only for the LA.
//
// This bench drives the whole top over SPI and checks, for the ADC's first captured sample:
//   * its conversion started (CNVST rose) a FIXED number of clk after t0, whatever the arm's
//     phase against the free-running engine;
//   * that conversion is the LA's first sample instant: the CNVST rise and the clk edge at
//     which the LA producer's first sample was on the pad are the same cycle;
//   * every captured interval, the first included, is the NEW divider, after a slow capture
//     followed by a fast one and the other way round;
//   * the same holds for a triggered capture (t0 = the trigger cycle);
//   * an LA-only capture does not restart the engine (the loop image reads it live).
// Runs on both images (make topadct0test).
// ============================================================================
`timescale 1ns/1ps
module tb_top_adc_t0;
    reg clk48 = 1'b0;
    always #10 clk48 = ~clk48;

    reg  sck = 1'b0, mosi = 1'b0, csn = 1'b1, bus_own = 1'b0, adc_sdo = 1'b1;
    wire miso, busy, adc_cnvst, adc_sclk, adc_sdi, dac_sync, dac_sclk, dac_din, led_g, led_b, led_r;
    wire psram_sclk, psram_cs, psram_io0, psram_io1, psram_io2, psram_io3;
    reg  [13:0] la_drv = 14'h0000;
    wire [13:0] la = la_drv;

    top dut (
        .clk48(clk48), .sck(sck), .mosi(mosi), .miso(miso), .csn(csn), .busy(busy), .bus_own(bus_own),
        .adc_cnvst(adc_cnvst), .adc_sclk(adc_sclk), .adc_sdi(adc_sdi), .adc_sdo(adc_sdo),
        .dac_sync(dac_sync), .dac_sclk(dac_sclk), .dac_din(dac_din),
        .psram_sclk(psram_sclk), .psram_cs(psram_cs), .psram_io0(psram_io0), .psram_io1(psram_io1),
        .psram_io2(psram_io2), .psram_io3(psram_io3), .la(la),
        .led_g(led_g), .led_b(led_b), .led_r(led_r));

    localparam SPI_HALF = 200;
    integer errors = 0;

    task spi_byte(input [7:0] tx);
        integer b; begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b]; #(SPI_HALF); sck = 1'b1; #(SPI_HALF); sck = 1'b0;
            end
        end
    endtask
    // OP_CAPTURE: dividers are PERIODS here; LA goes out as the v40 wire value (period - 2).
    task cmd_capture(input [23:0] adc_cnt, input [15:0] adc_div, input [23:0] la_cnt, input [15:0] la_div_period);
        reg [15:0] la_div; begin
            la_div = la_div_period - 16'd2;
            csn = 1'b0; #(SPI_HALF); spi_byte(8'h31);
            spi_byte(adc_cnt[7:0]); spi_byte(adc_cnt[15:8]); spi_byte(adc_cnt[23:16]);
            spi_byte(adc_div[7:0]); spi_byte(adc_div[15:8]);
            spi_byte(la_cnt[7:0]);  spi_byte(la_cnt[15:8]);  spi_byte(la_cnt[23:16]);
            spi_byte(la_div[7:0]);  spi_byte(la_div[15:8]);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask
    task cmd_set_trigger(input [7:0] ch, input [7:0] mode);
        begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h33); spi_byte(ch); spi_byte(mode); spi_byte(8'h00);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask

    // ---- monitor (clk cycles) ----
    // cnv_cyc: the cycle whose edge raised CNVST last; cnv_stb: that value at the last sample
    // strobe, i.e. the start of the conversion the strobe carries.  la_pad: the edge at which the
    // pad value the LA producer latched first was sampled (la_psram_capture's q1).
    integer cyc = 0, t0_cyc = -1, cnv_cyc = -1, cnv_stb = -1, cnv_first = -1, la_pad = -1, restarts = 0;
    integer n_adc = 0, prev_adc = -1, bad_iv = 0, want_iv = 0;
    reg     cnv_q = 1'b0;
    always @(posedge dut.clk) begin
        cyc = cyc + 1;
        if (dut.adc_cnvst && !cnv_q) cnv_cyc = cyc - 1;     // risen at the previous edge
        cnv_q <= dut.adc_cnvst;
        if (dut.adc_restart) restarts = restarts + 1;
        // the strobe comes 58 clk after its conversion started, before the next one can start
        if (dut.adc_sample_stb) cnv_stb = cnv_cyc;
        if (dut.t0) begin t0_cyc = cyc; cnv_first = -1; la_pad = -1; n_adc = 0; prev_adc = -1; end
        // second byte of each captured ADC sample
        if (t0_cyc > 0 && dut.adc_wr_stb && dut.adc_pst == 2'd0) begin
            if (n_adc == 0) cnv_first = cnv_stb;
            else if (cyc - prev_adc != want_iv) bad_iv = bad_iv + 1;
            prev_adc = cyc; n_adc = n_adc + 1;
        end
        // the LA producer latches q2 (the pad two edges back) on the cycle it writes the low byte
        if (t0_cyc > 0 && la_pad < 0 && dut.la_ps_i.wr_stb && dut.la_ps_i.writing_hi) la_pad = cyc - 3;
    end

    integer i, off0, off, la_off0;
    task run_one(input [8*16-1:0] what, input [15:0] div, input integer gap, input integer with_la);
        begin
            want_iv = div; bad_iv = 0;
            cmd_capture(24'd5, div, with_la ? 24'd5 : 24'd0, div);
            #(gap);
            off = cnv_first - t0_cyc;
            if (n_adc != 5) begin
                $display("FAIL %0s: %0d ADC samples captured (want 5)", what, n_adc); errors = errors + 1;
            end
            if (off != div + off0) begin
                $display("FAIL %0s: first ADC conversion at t0+%0d clk (want divider %0d + %0d)", what, off, div, off0);
                errors = errors + 1;
            end
            if (bad_iv != 0) begin
                $display("FAIL %0s: %0d ADC interval(s) not %0d clk", what, bad_iv, div); errors = errors + 1;
            end
            if (with_la && cnv_first != la_pad) begin
                $display("FAIL %0s: ADC sample 0 converted at cycle %0d, LA sample 0 taken at %0d (want the same)",
                         what, cnv_first, la_pad);
                errors = errors + 1;
            end
        end
    endtask

    integer r0;
    initial begin
        #5000;
        // reference: the offset of the first conversion beyond the divider
        cmd_capture(24'd5, 16'd60, 24'd5, 16'd60); #60000;
        off0 = cnv_first - t0_cyc - 60;
        $display("     first ADC conversion at t0 + divider + (%0d) clk; LA sample 0 on the pad at cycle t0+%0d",
                 off0, la_pad - t0_cyc);
        // arm phase swept against the free-running engine (it runs at 60 between captures)
        for (i = 0; i < 16; i = i + 1) run_one("phase sweep", 16'd60, 40000 + i * 137, 1);
        // a slow capture, then a fast one, and back: the first interval follows the NEW divider
        for (i = 0; i < 3; i = i + 1) begin
            run_one("slow after fast", 16'd2000, 700000 + i * 531, 1);
            run_one("fast after slow", 16'd60,   60000  + i * 977, 1);
        end
        run_one("ADC only", 16'd240, 120000, 0);
        // a triggered capture: t0 is the trigger cycle, long after the arm
        cmd_set_trigger(8'd0, 8'd1);                 // LA1 rising
        cmd_capture(24'd5, 16'd60, 24'd5, 16'd60);
        #20013; la_drv[0] = 1'b1; #60000;
        off = cnv_first - t0_cyc;
        if (off != 60 + off0 || cnv_first != la_pad || n_adc != 5) begin
            $display("FAIL triggered: first ADC conversion at t0+%0d (want %0d), LA at %0d vs ADC %0d, %0d samples",
                     off, 60 + off0, la_pad, cnv_first, n_adc);
            errors = errors + 1;
        end
        cmd_set_trigger(8'd0, 8'd0); la_drv[0] = 1'b0;
        // an LA-only capture leaves the engine alone
        r0 = restarts;
        cmd_capture(24'd0, 16'd60, 24'd5, 16'd60); #40000;
        if (restarts != r0) begin
            $display("FAIL LA-only: the ADC engine was restarted"); errors = errors + 1;
        end
        if (errors == 0)
            $display("PASS tb_top_adc_t0: ADC sample 0 converted at t0 + divider + (%0d), the LA's sample-0 instant, every phase/divider/trigger",
                     off0);
        else
            $display("FAIL tb_top_adc_t0: %0d error(s)", errors);
        $finish;
    end
    initial begin #30000000 $display("FAIL tb_top_adc_t0: timeout"); $finish; end
endmodule
