// tb_adc_mcp33131.v — self-checking testbench for adc_mcp33131.
// Presents a known 16-bit word on SDO (MSB-first, shifted on the engine's SCLK
// falling edge) and checks the captured `sample` matches after one conversion,
// then checks the free-running sample period is EXACTLY `divider` clocks (floored
// at 60) — the firmware reports the ADC rate as 24 MHz / divider.
`timescale 1ns/1ps
module tb_adc_mcp33131;
    reg clk = 0;
    always #10 clk = ~clk;

    reg         rst = 1, en = 0;
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
    always @(negedge sclk)  sr = {sr[14:0], 1'b0};
    always @(*) sdo_r = sr[15];

    adc_mcp33131 dut (
        .clk(clk), .rst(rst), .en(en), .divider(divider),
        .adc_cnvst(cnvst), .adc_sclk(sclk), .adc_sdi(sdi), .adc_sdo(sdo_r),
        .sample(sample), .sample_stb(sample_stb)
    );

    integer errors = 0;
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
        check_period(16'd2,    60);   // below the floor -> floored
        check_period(16'd240, 240);   // 100 kS/s

        if (errors == 0)
            $display("PASS tb_adc_mcp33131: sample = %04h, period == divider (floor 60)", PATTERN);
        else
            $display("FAIL tb_adc_mcp33131: %0d error(s)", errors);
        $finish;
    end

    initial begin #2000000 $display("FAIL tb_adc_mcp33131: timeout"); $finish; end
endmodule
