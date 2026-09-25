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
        .steps(steps), .delay_us(delay_us - 16'd1),   // v40 wire: half-phase - 1
        .busy(busy), .step_channel(step_channel), .step_val(step_val)
    );

    // Phase 2: the same inputs into the v41 fabric engine; outputs must match every cycle.
    reg        r_start = 0, r_rst = 0, cmp = 0;
    reg [3:0]  r_ch = 0;
    reg [15:0] r_steps = 0, r_delay = 0;
    wire       rb, rv, db, dv;
    wire [3:0] rc, dc;
    stepper_engine_ref #(.CLK_MHZ(CLK_MHZ)) ref_i (
        .clk(clk), .rst(r_rst), .start(r_start), .channel(r_ch), .steps(r_steps),
        .delay_us(r_delay), .busy(rb), .step_channel(rc), .step_val(rv));
    stepper_engine #(.CLK_MHZ(CLK_MHZ)) dut2 (
        .clk(clk), .rst(r_rst), .start(r_start), .channel(r_ch), .steps(r_steps),
        .delay_us(r_delay), .busy(db), .step_channel(dc), .step_val(dv));
    integer mism = 0, cyc = 0, trains = 0;
    reg     rb_d = 0;
    always @(negedge clk) if (cmp) begin
        cyc = cyc + 1;
        if (rb && !rb_d) trains = trains + 1;
        rb_d = rb;
        if ({db, dc, dv} !== {rb, rc, rv}) begin
            if (mism < 5) $display("FAIL tb_stepper: cycle %0d busy/ch/val %b/%0d/%b, v41 engine %b/%0d/%b",
                                   cyc, db, dc, dv, rb, rc, rv);
            mism = mism + 1;
        end
    end

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

        // ---- phase 2: random trains vs the v41 engine ----
        r_rst = 1; repeat (2) @(negedge clk); r_rst = 0; cmp = 1;
        repeat (600) begin
            @(negedge clk);
            r_start = ($random % 7) == 0;
            if (($random % 5) == 0) begin
                r_steps = ($random % 9 == 0) ? 16'd0 : 1 + ($random & 3);
                r_delay = $random & 3;
                r_ch    = $random;
            end
            r_rst = ($random % 211) == 0;
            repeat ($random & 15) @(negedge clk);
            r_start = 0;
        end
        cmp = 0;
        if (mism != 0) errors = errors + 1;
        if (trains < 50) begin
            $display("FAIL tb_stepper: only %0d trains in phase 2", trains); errors = errors + 1; end

        if (errors == 0)
            $display("PASS tb_stepper: %0d-step train on LA%0d, busy clean, half-phase=%0d clks; %0d random trains over %0d cycles == v41 engine",
                     rises, CH, hp_clks, trains, cyc);
        else
            $display("FAIL tb_stepper: %0d error(s)", errors);
        $finish;
    end

    initial begin #20000000 $display("FAIL tb_stepper: timeout"); $finish; end
endmodule
