// tb_adc_mcp33131.v — self-checking testbench for adc_mcp33131.
// Presents a known 16-bit word on SDO (MSB-first, shifted on the engine's SCLK
// falling edge) and checks the captured `sample` matches after one conversion,
// then checks the free-running sample period is EXACTLY `divider` clocks — the firmware
// reports the ADC rate as 24 MHz / divider.  v40: the 60-clock floor moved to the firmware
// (adc_min_divider); below it the engine runs conversions back to back (58 clocks, ~414 kS/s,
// inside the MCP33131-10's 1 MS/s), so a too-small divider is inexact but never unsafe.
// v49 recalibrate (datasheet 7.1, Figure 7-3): after the `recal` strobe the next read clocks
// exactly 1024 SCLK pulses with SDI high and CNVST low, `calibrating` stays high while the ADC
// holds SDO low (tCAL, shortened here), no conversion starts meanwhile, and sampling resumes
// with the right data once SDO is released.  An ADC that never pulls SDO low ends the wait after
// the hold-off instead of hanging the engine.
`timescale 1ns/1ps
module tb_adc_mcp33131;
    reg clk = 0;
    always #10 clk = ~clk;

    reg         rst = 1, en = 0, recal = 0;
    wire        calibrating;
    reg  [15:0] divider = 16'd100;
    wire        cnvst, sclk, sdi;
    reg         sdo_r;
    wire [15:0] sample;
    wire        sample_stb;

    localparam [15:0] PATTERN = 16'hA53C;
    reg [15:0] sr;
    // load the pattern when the read phase begins (CNVST falls), present MSB,
    // shift on each SCLK falling edge so the bit is stable on the rising edge.
    always @(negedge cnvst) sr = PATTERN;
    always @(negedge sclk)  sr = {sr[14:0], 1'b1};   // SDO Hi-Z after D0: the pull-up reads 1
    // The ADC holds SDO low while it calibrates; `cal_hold` stands in for that (the iCE40
    // pull-up makes SDO read high otherwise between reads).
    reg cal_hold = 0;
    always @(*) sdo_r = cal_hold ? 1'b0 : sr[15];

    // SCLK rising edges in the current CNVST-low window, and CNVST rising edges overall.
    integer sclk_n = 0, cnv_n = 0;
    reg     sdi_bad = 0;
    always @(posedge cnvst) begin sclk_n = 0; cnv_n = cnv_n + 1; end
    always @(posedge sclk)  sclk_n = sclk_n + 1;
    always @(*) if (sdi !== 1'b1) sdi_bad = 1;

    adc_mcp33131 dut (
        .clk(clk), .rst(rst), .en(en), .divider(divider),
        .recal(recal), .calibrating(calibrating),
        .adc_cnvst(cnvst), .adc_sclk(sclk), .adc_sdi(sdi), .adc_sdo(sdo_r),
        .sample(sample), .sample_stb(sample_stb)
    );

    integer errors = 0;
    integer k0;
    wire    st_is_cal = (dut.st == 2'd3);
    integer cyc = 0;
    always @(posedge clk) cyc = cyc + 1;

    // Measure the strobe-to-strobe period (in clocks) at `div`; expect `want`.
    // Gateware <= v31 took div+1 (the registered period_zero flag added a cycle).
    task check_period(input [15:0] div, input integer want);
        integer t0, k;
        begin
            divider = div;
            @(posedge sample_stb); @(posedge sample_stb);   // settle onto the new divider
            @(posedge sample_stb); t0 = cyc;
            for (k = 0; k < 3; k = k + 1) begin
                @(posedge sample_stb);
                if (cyc - t0 !== want) begin
                    $display("FAIL tb_adc_mcp33131: divider %0d period %0d clocks (want %0d)",
                             div, cyc - t0, want);
                    errors = errors + 1;
                end
                t0 = cyc;
            end
        end
    endtask

    initial begin
        repeat (4) @(posedge clk); rst = 0;
        @(posedge clk); en = 1;
        @(posedge sample_stb);
        #1;
        if (sample !== PATTERN) begin
            $display("FAIL tb_adc_mcp33131: sample = %04h (want %04h)", sample, PATTERN);
            errors = errors + 1;
        end

        check_period(16'd100, 100);
        check_period(16'd61,   61);
        check_period(16'd60,   60);   // the floor itself (~400 kS/s)
        check_period(16'd2,    58);   // below the firmware floor: back-to-back conversions
        check_period(16'd240, 240);   // 100 kS/s

        // ---- recalibrate: the ADC takes the command and holds SDO low for "tCAL" ----
        divider = 16'd100;
        @(posedge sample_stb);
        @(posedge clk) recal = 1; @(posedge clk) recal = 0;
        if (!calibrating) begin
            $display("FAIL tb_adc_mcp33131: calibrating not set by the strobe"); errors = errors + 1;
        end
        // Wait out the read that carries the command (CNVST low after its conversion).
        wait (st_is_cal);
        if (sclk_n !== 1024) begin
            $display("FAIL tb_adc_mcp33131: recalibrate sent %0d SCLK pulses (want 1024)", sclk_n);
            errors = errors + 1;
        end
        cal_hold = 1;                        // the ADC drives SDO low while calibrating
        k0 = cnv_n;
        repeat (20000) @(posedge clk);       // "tCAL": no conversion, still calibrating
        if (cnv_n !== k0 || !calibrating || cnvst || sclk) begin
            $display("FAIL tb_adc_mcp33131: during calibration cnvst=%0d sclk=%0d calibrating=%0d conversions=%0d",
                     cnvst, sclk, calibrating, cnv_n - k0);
            errors = errors + 1;
        end
        cal_hold = 0;                        // done: SDO released, pull-up high
        repeat (4) @(posedge clk);
        if (calibrating) begin
            $display("FAIL tb_adc_mcp33131: calibrating still set after SDO went high"); errors = errors + 1;
        end
        @(posedge sample_stb); #1;
        if (sample !== PATTERN || sclk_n !== 16) begin
            $display("FAIL tb_adc_mcp33131: after recalibrate sample = %04h, read clocks %0d", sample, sclk_n);
            errors = errors + 1;
        end
        check_period(16'd100, 100);

        // ---- recalibrate the ADC ignores (SDO never goes low): no hang ----
        @(posedge clk) recal = 1; @(posedge clk) recal = 0;
        wait (st_is_cal);
        k0 = cyc;
        wait (!calibrating);
        if (cyc - k0 > 1100) begin
            $display("FAIL tb_adc_mcp33131: ignored recalibrate took %0d clocks to clear", cyc - k0);
            errors = errors + 1;
        end
        @(posedge sample_stb); #1;
        if (sample !== PATTERN) begin
            $display("FAIL tb_adc_mcp33131: after ignored recalibrate sample = %04h", sample);
            errors = errors + 1;
        end
        if (sdi_bad) begin
            $display("FAIL tb_adc_mcp33131: SDI went low"); errors = errors + 1;
        end

        if (errors == 0)
            $display("PASS tb_adc_mcp33131: sample = %04h, period == divider (>= 60; back-to-back 58 below), recalibrate = 1024 SCLK + SDO-low wait", PATTERN);
        else
            $display("FAIL tb_adc_mcp33131: %0d error(s)", errors);
        $finish;
    end

    initial begin #2000000 $display("FAIL tb_adc_mcp33131: timeout"); $finish; end
endmodule
