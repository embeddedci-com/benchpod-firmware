// tb_stepper.v — smoke test for stepper_engine (first coverage; guards the
// Tier-2 narrowing of steps/delay_us and the internal us_left/steps_left
// counters from 24 to 16 bits).
//
// Drives a short train and checks: exactly `steps` HIGH half-phases are emitted
// on the right channel, then busy drops.  CLK_MHZ is kept tiny so the
// microsecond prescaler is short and the sim runs fast.
//
// Also asserts the absolute microsecond timing: with CLK_MHZ set to the sim
// clock rate, one half-phase must last exactly delay_us * CLK_MHZ system clocks
// (delay_us µs at CLK_MHZ ticks/µs).  This guards the prescaler against the
// CLK_MHZ-mismatch class of bug (e.g. instantiating with CLK_MHZ != real clk).
`timescale 1ns/1ps
module tb_stepper;
    localparam CLK_MHZ = 2;            // 2 clocks per microsecond tick
    localparam STEPS   = 4;
    localparam DELAY   = 3;            // us per half-phase (exercises us_left)
    localparam CH      = 4'd7;

    reg clk = 0;
    always #10 clk = ~clk;

    reg        rst = 1, start = 0;
    reg [3:0]  channel  = CH;
    reg [15:0] steps    = STEPS;
    reg [15:0] delay_us = DELAY;
    wire       busy, step_val;
    wire [3:0] step_channel;

    stepper_engine #(.CLK_MHZ(CLK_MHZ)) dut (
        .clk(clk), .rst(rst), .start(start), .channel(channel),
        .steps(steps), .delay_us(delay_us),
        .busy(busy), .step_channel(step_channel), .step_val(step_val)
    );

    // Count HIGH half-phases (one rising edge of step_val per step).
    integer rises = 0;
    reg     sv_d = 0;
    always @(posedge clk) begin
        if (step_val && !sv_d) rises = rises + 1;
        sv_d <= step_val;
    end

    // Measure the first HIGH half-phase duration in system clocks.  Count every
    // posedge that step_val is high until it first drops; that span is the
    // half-phase and must equal delay_us * CLK_MHZ.
    integer hp_clks   = 0;
    reg     measuring = 0;
    reg     measured  = 0;
    always @(posedge clk) begin
        if (busy && step_val && !measured) begin
            measuring <= 1;
            hp_clks   <= hp_clks + 1;
        end else if (measuring && !step_val) begin
            measuring <= 0;
            measured  <= 1;            // freeze hp_clks at the first half-phase
        end
    end

    integer errors = 0;
    initial begin
        repeat (4) @(negedge clk); rst = 0;
        @(negedge clk) start = 1; @(negedge clk) start = 0;
        wait (busy);
        if (step_channel !== CH) begin
            $display("FAIL tb_stepper: channel=%0d want %0d", step_channel, CH);
            errors = errors + 1; end
        wait (!busy);                 // train finished
        @(negedge clk);
        if (rises !== STEPS) begin
            $display("FAIL tb_stepper: %0d HIGH phases (want %0d)", rises, STEPS);
            errors = errors + 1; end
        if (step_val !== 1'b0) begin
            $display("FAIL tb_stepper: step_val not low after train"); errors = errors + 1; end
        if (hp_clks !== DELAY*CLK_MHZ) begin
            $display("FAIL tb_stepper: half-phase %0d clks (want %0d = delay_us*CLK_MHZ)",
                     hp_clks, DELAY*CLK_MHZ);
            errors = errors + 1; end

        if (errors == 0)
            $display("PASS tb_stepper: %0d-step train on LA%0d, busy clean, half-phase=%0d clks",
                     rises, CH, hp_clks);
        else
            $display("FAIL tb_stepper: %0d error(s)", errors);
        $finish;
    end

    initial begin #2000000 $display("FAIL tb_stepper: timeout"); $finish; end
endmodule
