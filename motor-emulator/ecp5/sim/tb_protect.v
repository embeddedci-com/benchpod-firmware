// tb_protect — the gateware protection layer (src/protect.v).
//  1 overcurrent: one sample over the limit does not trip at OC_COUNT 2, two in a row do, on the
//    right phase only; negative currents count; clear works; a second run with a new limit
//  2 overvoltage brake: on above OV_ON, held between, off below OV_OFF; never off board 0
//  3 energy budget: the brake is inhibited when the bucket reaches the cap (time checked against
//    the arithmetic) and released at half; the brake request stays, so it cycles
//  4 brake test pulse: length and the two bus captures
//  5 hot-swap interlock: refused without a limit; four samples over the limit trip it
`timescale 1ns/1ps
`default_nettype none
module tb_protect;
    reg clk = 0;
    always #13.889 clk = ~clk;
    reg rst = 1, board0 = 1;
    reg us = 0;
    integer uc = 0;
    always @(posedge clk) begin uc = (uc == 35) ? 0 : uc + 1; us <= (uc == 0); end

    reg v_any = 0; reg [2:0] v_ch = 0;
    reg signed [15:0] s_a = 0, s_b = 0, s_c = 0, s_bus = 0;
    reg [15:0] oc_lim = 0, ov_on = 0, ov_off = 0, brk_g = 0, brk_pavg = 0, brk_cap = 0, hs_limit = 0, test_len = 0;
    reg [3:0] oc_count = 2, clear = 0;
    reg brake_man = 0, test_start = 0, hswap_on = 0;
    wire brake, inhibit, ov_brake, hswap_ok;
    wire [3:0] trips;
    wire [15:0] v_bus, tv0, tv1, energy;
    protect dut (.clk(clk), .rst(rst), .us(us), .board0(board0), .v_any(v_any), .v_ch(v_ch),
        .s_a(s_a), .s_b(s_b), .s_c(s_c), .s_bus(s_bus), .ofs_a(16'sd10), .ofs_b(-16'sd10), .ofs_c(16'sd0), .ofs_bus(16'sd5),
        .oc_lim(oc_lim), .oc_count(oc_count), .ov_on(ov_on), .ov_off(ov_off), .brk_g(brk_g), .brk_pavg(brk_pavg),
        .brk_cap(brk_cap), .hs_limit(hs_limit), .brake_man(brake_man), .test_start(test_start), .test_len(test_len),
        .hswap_on(hswap_on), .clear(clear), .brake(brake), .trips(trips), .inhibit(inhibit), .ov_brake(ov_brake),
        .hswap_ok(hswap_ok), .v_bus(v_bus), .test_v0(tv0), .test_v1(tv1), .energy(energy));

    integer errors = 0;
    task check(input [8*48-1:0] what, input ok);
        if (!ok) begin errors = errors + 1; $display("FAIL: %0s", what); end
    endtask
    task sample(input [2:0] ch, input signed [15:0] v);
        begin
            @(negedge clk);
            case (ch) 3'd0: s_a = v; 3'd1: s_b = v; 3'd2: s_c = v; default: s_bus = v; endcase
            v_ch = ch; v_any = 1; @(negedge clk) v_any = 0; repeat (4) @(negedge clk);
        end
    endtask
    task do_clear(input [3:0] m); begin @(negedge clk) clear = m; @(negedge clk) clear = 0; @(negedge clk); end endtask

    integer t0, t1, k;
    initial begin
        #200_000_000 $display("FAIL tb_protect: timeout"); $finish;
    end
    initial begin
        repeat (4) @(posedge clk); rst = 0;
        // ---- 1 overcurrent (limit 4000 codes, about 2 A; phase B offset -10)
        oc_lim = 4000; oc_count = 2;
        sample(3'd1, 3995);                                  // 4005 after the offset: over, 1st
        sample(3'd1, 100);                                   // back under: count restarts
        sample(3'd1, 3995);
        check("one sample over the limit must not trip", trips == 4'b0000);
        sample(3'd1, -4100);                                 // -4090: over again, 2nd in a row
        check("two samples over the limit trip phase B", trips == 4'b0010);
        sample(3'd0, 32000);
        sample(3'd0, 32000);
        check("phase A trips too", trips == 4'b0011);
        do_clear(4'b0011);
        check("clear", trips == 4'b0000);
        oc_lim = 30000; oc_count = 1;                         // second run, new limit
        sample(3'd2, 29000);
        check("under the new limit", trips == 4'b0000);
        sample(3'd2, -30500);
        check("new limit, count 1, phase C", trips == 4'b0100);
        oc_lim = 0; do_clear(4'b0100);
        sample(3'd0, 32000); sample(3'd0, 32000);
        check("limit 0 is off", trips == 4'b0000);
        $display("  ok overcurrent");

        // ---- 2 overvoltage brake (bus codes: on 20000, off 19000; offset 5)
        ov_on = 20000; ov_off = 19000;
        sample(3'd4, 19500); repeat (3) @(negedge clk); check("below on: no brake", !brake);
        sample(3'd4, 20100); repeat (3) @(negedge clk); check("above on: brake", brake && ov_brake);
        sample(3'd4, 19500); repeat (3) @(negedge clk); check("hysteresis band: held", brake);
        sample(3'd4, 18900); repeat (3) @(negedge clk); check("below off: released", !brake);
        board0 = 0; sample(3'd4, 21000); repeat (3) @(negedge clk); check("not board 0: no brake", !brake);
        board0 = 1; repeat (3) @(negedge clk); check("board 0 again: brake", brake);
        sample(3'd4, 10000); ov_on = 0; repeat (3) @(negedge clk);
        $display("  ok overvoltage brake");

        // ---- 3 budget: bus 20005 codes (51.2 V), R = 10 ohm: G = 2^24 / (391^2 x 10) = 10.97 -> 11
        //      P = 20000^2 x 11 >> 24 = 262 W; drain 12 W: net 250 uJ per us; cap 10 x 4096 uJ
        sample(3'd4, 20005);
        brk_g = 11; brk_pavg = 12; brk_cap = 10;
        @(negedge clk) brake_man = 1;
        @(posedge brake); t0 = $time;
        @(posedge inhibit); t1 = $time;
        // 40960 / 250 = 164 us
        check("inhibit after about 164 us", (t1 - t0) > 160_000 && (t1 - t0) < 168_000);
        repeat (3) @(negedge clk);
        check("inhibited: brake off with the request on", !brake);
        @(negedge inhibit); t0 = $time;
        // drains 12 uJ / us from 40960 to below 20480: about 1707 us
        check("released at half the cap (about 1707 us)", (t0 - t1) > 1_690_000 && (t0 - t1) < 1_725_000);
        repeat (3) @(negedge clk);
        check("released: the brake comes back", brake);
        @(negedge clk) brake_man = 0; brk_cap = 0;
        $display("  ok energy budget: released %0d us after the inhibit", (t0 - t1) / 1000);

        // ---- 4 brake test pulse: 200 us, bus moves during it
        sample(3'd4, 15005);
        @(negedge clk) begin test_len = 200; test_start = 1; end
        @(negedge clk) test_start = 0;
        @(posedge brake); t0 = $time;
        repeat (36 * 100) @(negedge clk);
        sample(3'd4, 14005);
        @(negedge brake); t1 = $time;
        check("test pulse about 200 us", (t1 - t0) > 198_000 && (t1 - t0) < 202_000);
        check("V0 / V1", tv0 == 16'd15000 && tv1 == 16'd14000);
        $display("  ok brake test pulse %0d ns, V0 %0d V1 %0d", t1 - t0, tv0, tv1);

        // ---- 5 hot-swap interlock (limit 18000)
        check("no limit: refused", !hswap_ok);
        hs_limit = 18000; #1;
        check("limit set: allowed", hswap_ok);
        hswap_on = 1;
        for (k = 0; k < 3; k = k + 1) sample(3'd4, 18100);
        check("three over: not yet", trips[3] == 1'b0);
        sample(3'd4, 17000);
        for (k = 0; k < 3; k = k + 1) sample(3'd4, 18100);
        check("a sample under restarts the count", trips[3] == 1'b0);
        sample(3'd4, 18100);
        check("four in a row: hot-swap trip", trips[3] == 1'b1 && !hswap_ok);
        do_clear(4'b1000); sample(3'd4, 15000);
        check("cleared", hswap_ok);
        oc_lim = 1000; sample(3'd0, 5000); sample(3'd0, 5000);
        check("an overcurrent trip also drops the hot-swap", !hswap_ok);
        $display("  ok hot-swap interlock");

        if (errors == 0) $display("PASS tb_protect"); else $display("FAIL tb_protect: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
