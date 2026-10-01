// tb_sync_wd — stack SYNC: two boards on one open-drain line.
// Board 0 (master) pulses every PERIOD; board 1 only listens. Checks the pulse width and rate,
// that both see the ticks, that another board holding SYNC low trips both after LOW_TRIP (and a
// clear works once released), and that stack mode trips when the master stops. Small periods
// keep it quick; the logic is the same as the 36 MHz defaults.
`timescale 1ns/1ps
`default_nettype none
module tb_sync_wd;
    reg clk = 0;
    always #13.889 clk = ~clk;
    reg rst = 1;
    reg master0 = 1, hold1 = 0, required = 0, clear = 0;
    tri1 sync_line;
    wire oe0, oe1, tick0, tick1, seen0, seen1, trip0, trip1;
    assign sync_line = (oe0 | oe1) ? 1'b0 : 1'bz;

    // UNIT 4 clk: period 100 units = 400 clk, pulse one unit, trips at 10 and 300 units
    localparam U = 4, PU = 100, LTU = 10, MSU = 300;
    localparam P = PU * U, PW = U, LT = LTU * U, MS = MSU * U;
    wire us0, ms0, us1, ms1;
    sync_wd #(.UNIT(U), .PERIOD(PU), .LOW_TRIP(LTU), .MISSING(MSU)) b0 (
        .clk(clk), .rst(rst), .master(master0), .hold_low(1'b0), .required(required), .clear(clear),
        .sync_in(sync_line), .sync_oe(oe0), .tick(tick0), .seen(seen0), .trip(trip0), .us(us0), .ms(ms0));
    sync_wd #(.UNIT(U), .PERIOD(PU), .LOW_TRIP(LTU), .MISSING(MSU)) b1 (
        .clk(clk), .rst(rst), .master(1'b0), .hold_low(hold1), .required(required), .clear(clear),
        .sync_in(sync_line), .sync_oe(oe1), .tick(tick1), .seen(seen1), .trip(trip1), .us(us1), .ms(ms1));

    integer errors = 0, n, low, t0, t1;
    initial begin
        #10_000_000 $display("FAIL tb_sync_wd: timeout"); $finish;
    end
    initial begin
        repeat (4) @(posedge clk); rst = 0;
        // pulse width and period, ticks on both boards
        @(posedge oe0); low = 0;
        while (oe0 === 1'b1) begin @(negedge clk); if (oe0) low = low + 1; end
        if (low != PW) begin errors = errors + 1; $display("FAIL: SYNC low for %0d clk, expected %0d", low, PW); end
        @(posedge tick1); t0 = $time; @(posedge tick1); t1 = $time;
        if ((t1 - t0) < P * 27.7 || (t1 - t0) > P * 27.9) begin
            errors = errors + 1; $display("FAIL: tick spacing %0d ns", t1 - t0);
        end
        repeat (2 * P) @(posedge clk);
        if (!seen0 || !seen1 || trip0 || trip1) begin errors = errors + 1; $display("FAIL: normal running seen %b%b trip %b%b", seen0, seen1, trip0, trip1); end

        // board 1's gateware faults: holds SYNC low -> both trip after LOW_TRIP
        hold1 = 1; repeat (LT - 2 * U) @(posedge clk);
        if (trip0 || trip1) begin errors = errors + 1; $display("FAIL: tripped before LOW_TRIP"); end
        repeat (3 * U + 10) @(posedge clk);
        if (!trip0 || !trip1) begin errors = errors + 1; $display("FAIL: SYNC held low did not trip both (%b%b)", trip0, trip1); end
        hold1 = 0; repeat (5) @(posedge clk);
        @(negedge clk) clear = 1; @(negedge clk) clear = 0; repeat (2 * P) @(posedge clk);
        if (trip0 || trip1) begin errors = errors + 1; $display("FAIL: trip did not clear (%b%b)", trip0, trip1); end

        // stack mode: master stops -> both trip after MISSING; not before
        required = 1; master0 = 0;
        repeat (MS / 2) @(posedge clk);
        if (trip0 || trip1) begin errors = errors + 1; $display("FAIL: tripped early on missing pulses"); end
        repeat (MS) @(posedge clk);
        if (!trip0 || !trip1 || seen0) begin errors = errors + 1; $display("FAIL: missing pulses: trip %b%b seen %b", trip0, trip1, seen0); end
        // second run: master back, clear, stays healthy
        master0 = 1; repeat (2 * P) @(posedge clk);
        @(negedge clk) clear = 1; @(negedge clk) clear = 0; repeat (3 * P) @(posedge clk);
        if (trip0 || trip1 || !seen1) begin errors = errors + 1; $display("FAIL: after restart trip %b%b seen %b", trip0, trip1, seen1); end

        if (errors == 0) $display("PASS tb_sync_wd"); else $display("FAIL tb_sync_wd: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
