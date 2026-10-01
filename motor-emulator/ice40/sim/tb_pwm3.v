// tb_pwm3 — bridge PWM: interlock, dead time, duty average, 0/100 % flips, disable, re-arm.
//
// Checked on every clk: HS and LS of a leg are never high together, and a gate only turns on
// deadtime + 1 clk or more after its partner turned off. Per run: the period is the programmed
// one, and the HS on-time over many periods matches duty x period minus the dead-time loss.
// Run 2 re-programs period, dead time and duties and checks those are the ones used.
`timescale 1ns/1ps
`default_nettype none
module tb_pwm3;
    reg clk = 0;
    always #13.889 clk = ~clk;          // 36 MHz
    reg rst = 1, enable = 0;
    reg [9:0] period = 180;
    reg [5:0] deadtime = 2, min_on = 2;
    reg [15:0] duty_a = 16'h8000, duty_b = 16'h8000, duty_c = 16'h8000;
    wire [2:0] hs, ls;
    wire period_start, period_mid;

    pwm3 dut (.clk(clk), .rst(rst), .enable(enable), .period(period), .deadtime(deadtime),
              .min_on(min_on), .duty_a(duty_a), .duty_b(duty_b), .duty_c(duty_c),
              .hs(hs), .ls(ls), .period_start(period_start), .period_mid(period_mid));

    integer errors = 0;
    integer k, j;
    // per-leg time (in clk) since each gate last turned off
    integer since_hs_off [0:2];
    integer since_ls_off [0:2];
    reg [2:0] hs_q = 0, ls_q = 0;
    integer dt_now;
    always @(posedge clk) begin
        dt_now = dut.dt;
        for (j = 0; j < 3; j = j + 1) begin
            if (hs[j] && ls[j]) begin
                errors = errors + 1; $display("FAIL: leg %0d HS and LS both high at %0t", j, $time);
            end
            if (hs[j] && !hs_q[j] && since_ls_off[j] < dt_now + 1) begin
                errors = errors + 1;
                $display("FAIL: leg %0d HS on %0d clk after LS off (dead time %0d) at %0t", j, since_ls_off[j], dt_now, $time);
            end
            if (ls[j] && !ls_q[j] && since_hs_off[j] < dt_now + 1) begin
                errors = errors + 1;
                $display("FAIL: leg %0d LS on %0d clk after HS off (dead time %0d) at %0t", j, since_hs_off[j], dt_now, $time);
            end
            since_hs_off[j] = hs[j] ? 0 : since_hs_off[j] + 1;
            since_ls_off[j] = ls[j] ? 0 : since_ls_off[j] + 1;
        end
        hs_q <= hs; ls_q <= ls;
    end

    // measure over n periods: HS on clocks per leg and the period length
    integer on_a, on_b, on_c, clocks, starts;
    task measure(input integer n);
        begin
            @(posedge period_start); @(posedge clk);
            on_a = 0; on_b = 0; on_c = 0; clocks = 0; starts = 0;
            while (starts < n) begin
                @(posedge clk);
                clocks = clocks + 1;
                on_a = on_a + hs[0]; on_b = on_b + hs[1]; on_c = on_c + hs[2];
                if (period_start) starts = starts + 1;
            end
        end
    endtask

    // expected HS on-time: duty x period - (dead time + 1) per period, within 2 clk overall
    task check_on(input [8*4-1:0] name, input integer on, input [15:0] duty, input integer n,
                  input integer per, input integer dtc);
        real exp;
        begin
            exp = n * (duty * per / 65536.0 - (dtc + 1));
            if (on < exp - 2.0 || on > exp + 2.0) begin
                errors = errors + 1;
                $display("FAIL: %0s HS on %0d clk over %0d periods, expected %0.1f", name, on, n, exp);
            end else
                $display("  ok %0s HS on %0d clk (expected %0.1f)", name, on, exp);
        end
    endtask

    initial begin
        #20_000_000 $display("FAIL tb_pwm3: timeout"); $finish;
    end
    initial begin
        for (k = 0; k < 3; k = k + 1) begin since_hs_off[k] = 100; since_ls_off[k] = 100; end
        repeat (5) @(posedge clk); rst = 0;
        // ---- run 1: 200 kHz, dead time 2
        duty_a = 16'h4000; duty_b = 16'h8000; duty_c = 16'hC123;
        enable = 1;
        repeat (3) @(posedge period_start);
        measure(256);
        if (clocks != 256 * 180) begin errors = errors + 1; $display("FAIL: run 1 period %0d clk", clocks / 256); end
        check_on("A1", on_a, 16'h4000, 256, 180, 2);
        check_on("B1", on_b, 16'h8000, 256, 180, 2);
        check_on("C1", on_c, 16'hC123, 256, 180, 2);

        // ---- run 2: re-armed with a new period, dead time and duties
        enable = 0; repeat (3) @(posedge clk);
        if (hs != 0 || ls != 0) begin errors = errors + 1; $display("FAIL: gates on while disabled"); end
        period = 240; deadtime = 5; duty_a = 16'h1000; duty_b = 16'h7777; duty_c = 16'hE000;
        enable = 1;
        repeat (3) @(posedge period_start);
        measure(256);
        if (clocks != 256 * 240) begin errors = errors + 1; $display("FAIL: run 2 period %0d clk", clocks / 256); end
        check_on("A2", on_a, 16'h1000, 256, 240, 5);
        check_on("B2", on_b, 16'h7777, 256, 240, 5);
        check_on("C2", on_c, 16'hE000, 256, 240, 5);

        // ---- run 3: 0 % and 100 %, and a leg flipping between them every period
        duty_a = 16'h0000; duty_b = 16'hFFFF;
        repeat (3) @(posedge period_start);
        for (k = 0; k < 40; k = k + 1) begin
            @(posedge period_start);
            duty_c = (k % 2) ? 16'h0000 : 16'hFFFF;
        end
        measure(16);
        if (on_a != 0) begin errors = errors + 1; $display("FAIL: 0 %% duty still switches HS (%0d clk)", on_a); end
        if (on_b != clocks) begin errors = errors + 1; $display("FAIL: 100 %% duty HS on %0d of %0d clk", on_b, clocks); end

        // ---- disable in mid-period: every gate off on the next clk
        repeat (57) @(posedge clk);
        enable = 0; @(posedge clk); #1;
        if (hs != 0 || ls != 0) begin errors = errors + 1; $display("FAIL: gates still on 1 clk after disable"); end

        if (errors == 0) $display("PASS tb_pwm3");
        else $display("FAIL tb_pwm3: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
