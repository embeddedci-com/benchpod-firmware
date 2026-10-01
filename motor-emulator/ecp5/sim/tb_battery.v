// tb_battery — battery model (src/battery.v) against an independent integer reference.
// The bench drives the sinc-set and ms strobes, so the reference sees exactly the same events.
// Checked on every set: the setpoint; on every ms: position, mean current, OCV.
// Profile: rest, 1.5 A discharge (the OCV walks down the table), a punch (R0 sag), rest (the RC
// branch recovers), charging (current reversed by the sign flip), the voltage clamps, a second run
// from a written position. PV_SET: high fraction = duty / 65536; parked on `park`.
`timescale 1ns/1ps
`default_nettype none
module tb_battery;
    reg clk = 0;
    always #13.889 clk = ~clk;
    reg rst = 1, set_tick = 0, ms = 0;
    reg signed [15:0] s_i = 0;
    reg run = 0, inv = 0, tbl_we = 0, pos_we = 0, pv_en = 0, park = 0;
    reg [5:0] qshift = 20, tbl_addr = 0;
    reg signed [15:0] r0 = 125, r1 = 200, vmin = 20000, vmax = 31000, pv_ofs = 0, pv_gain = 32767;
    reg [15:0] alpha = 20972, tbl_data = 0, pos_data = 0;
    wire signed [15:0] setpoint, i_ms, ocv;
    wire [15:0] pos;
    wire pv_pin;
    localparam signed [15:0] OFS = 37;
    battery dut (.clk(clk), .rst(rst), .set_tick(set_tick), .ms(ms), .s_i(s_i), .ofs_i(OFS), .run(run), .inv(inv),
        .qshift(qshift), .r0(r0), .r1(r1), .alpha(alpha), .vmin(vmin), .vmax(vmax), .tbl_we(tbl_we), .tbl_addr(tbl_addr),
        .tbl_data(tbl_data), .pos_we(pos_we), .pos_data(pos_data), .pv_en(pv_en), .park(park), .pv_ofs(pv_ofs),
        .pv_gain(pv_gain), .setpoint(setpoint), .i_ms(i_ms), .ocv(ocv), .pos(pos), .pv_pin(pv_pin));

    // ---- reference
    reg signed [15:0] tbl [0:63];
    reg signed [47:0] r_charge = 0, r_vrc = 0;
    reg signed [31:0] r_isum = 0;
    reg signed [15:0] r_ocv = 0, r_ims = 0, r_sp = 0;
    reg [15:0] r_pos = 0;
    function signed [15:0] s16(input signed [63:0] x);
        s16 = (x > 32767) ? 16'sd32767 : (x < -32768) ? -16'sd32768 : x[15:0];
    endfunction
    task ref_set;
        reg signed [31:0] i;
        reg signed [31:0] v;
        begin
            i = s_i - OFS; if (inv) i = -i; i = s16(i);
            if (run) r_isum = r_isum + i;
            v = r_ocv - s16((i * r0) >>> 15) - s16(r_vrc >>> 16);
            r_sp = (v > vmax) ? vmax : (v < vmin) ? vmin : v;
        end
    endtask
    task ref_ms;
        reg signed [63:0] sum, rcd, p;
        reg [5:0] q, idx, idx1;
        reg [7:0] fr;
        begin
            sum = r_isum; r_isum = 0;
            if (run) r_charge = r_charge + sum;
            r_ims = s16((sum * 233) >>> 16);
            rcd = (((r_ims * r1) >>> 15) <<< 16) - r_vrc;
            if (rcd > 64'sh7FFF_FFFF) rcd = 64'sh7FFF_FFFF; if (rcd < -64'sh8000_0000) rcd = -64'sh8000_0000;
            if (run) r_vrc = r_vrc + ((rcd * $signed({1'b0, alpha})) >>> 20);
            q = (qshift < 8) ? 8 : qshift;
            p = r_charge >>> (q - 8);
            r_pos = (p < 0) ? 0 : (p > 65535) ? 16'hFFFF : p[15:0];
            if (p < 0) p = 0; if (p > 16128) p = 16128;
            idx = p[13:8]; idx1 = (idx == 63) ? 63 : idx + 1; fr = p[7:0];
            r_ocv = s16(tbl[idx] + (((tbl[idx1] - tbl[idx]) * $signed({1'b0, fr})) >>> 8));
        end
    endtask

    integer errors = 0, nset = 0, nms = 0;
    task check(input [8*40-1:0] what, input okx); if (!okx) begin errors = errors + 1; if (errors < 12) $display("FAIL: %0s", what); end endtask
    // one ms: 281 sets every 128 clk, then the ms strobe half-way between two sets
    task run_ms(input signed [15:0] cur);
        integer n;
        begin
            s_i = cur;
            for (n = 0; n < 281; n = n + 1) begin
                @(negedge clk) set_tick = 1; @(negedge clk) set_tick = 0;
                ref_set; repeat (8) @(negedge clk);
                check("setpoint", setpoint === r_sp);
                if (setpoint !== r_sp && errors < 12) $display("   set %0d: %0d, expected %0d", nset, setpoint, r_sp);
                nset = nset + 1;
                repeat (118) @(negedge clk);
            end
            @(negedge clk) ms = 1; @(negedge clk) ms = 0;
            ref_ms; repeat (30) @(negedge clk);
            check("ms update", pos === r_pos && i_ms === r_ims && ocv === r_ocv);
            if ((pos !== r_pos || i_ms !== r_ims || ocv !== r_ocv) && errors < 12)
                $display("   ms %0d: pos %0d/%0d i_ms %0d/%0d ocv %0d/%0d", nms, pos, r_pos, i_ms, r_ims, ocv, r_ocv);
            nms = nms + 1;
        end
    endtask

    integer k, hi, t;
    reg signed [15:0] sp_rest, sp_punch, sp_after;
    initial begin
        #2_000_000_000 $display("FAIL tb_battery: timeout"); $finish;
    end
    initial begin
        for (k = 0; k < 64; k = k + 1) tbl[k] = 30000 - 150 * k;
        repeat (4) @(posedge clk); rst = 0;
        for (k = 0; k < 64; k = k + 1) begin
            @(negedge clk) begin tbl_we = 1; tbl_addr = k; tbl_data = tbl[k]; end
        end
        @(negedge clk) tbl_we = 0;
        run = 1;
        for (k = 0; k < 5; k = k + 1) run_ms(OFS);              // rest
        for (k = 0; k < 30; k = k + 1) run_ms(OFS + 3000);      // ~1.5 A
        check("the OCV walked down the table", pos > 16'd4000 && ocv < 16'sd29000);
        // a realistic capacity (about 1.2 A s per table entry) and pack resistance for the punch
        qshift = 30; r0 = 600;
        for (k = 0; k < 10; k = k + 1) run_ms(OFS + 3000);
        sp_rest = setpoint;
        for (k = 0; k < 3; k = k + 1) run_ms(OFS + 10000);      // punch
        sp_punch = setpoint;
        check("the punch sags the bus", sp_punch < sp_rest - 16'sd60);
        for (k = 0; k < 20; k = k + 1) run_ms(OFS);             // rest: recovers
        sp_after = setpoint;
        check("recovers after the punch", sp_after > sp_punch + 16'sd60);
        $display("  ok discharge: pos %0d, ocv %0d; setpoint %0d -> punch %0d -> rest %0d", pos, ocv, sp_rest, sp_punch, sp_after);
        qshift = 20; run_ms(OFS);
        hi = pos;
        inv = 1;
        for (k = 0; k < 10; k = k + 1) run_ms(OFS + 3000);      // flipped sign: charging
        check("charging moves the position back", pos < hi - 1500);
        inv = 0;
        vmax = 29500; vmin = 29400;
        for (k = 0; k < 3; k = k + 1) run_ms(OFS + 12000);
        check("clamped at VMIN", setpoint == 16'sd29400);
        vmin = 20000; vmax = 31000;
        $display("  ok charging and clamps");
        // second run: position written, new R0
        @(negedge clk) begin pos_we = 1; pos_data = 16'd8192; end
        @(negedge clk) pos_we = 0;
        r_charge = 48'sd8192 <<< 12;
        r0 = 300;
        for (k = 0; k < 5; k = k + 1) run_ms(OFS + 2000);
        check("second run from the written position", pos > 16'd8192 && pos < 16'd9000);
        $display("  ok second run from position 8192: now %0d", pos);
        // PV_SET
        pv_en = 1;
        hi = 0;
        for (t = 0; t < 65536; t = t + 1) begin @(posedge clk); #1; hi = hi + pv_pin; end
        // duty = setpoint x 32767 >>> 15
        check("PV_SET high fraction", hi > ((setpoint * 32767) >>> 15) - 40 && hi < ((setpoint * 32767) >>> 15) + 40);
        $display("  ok PV_SET: %0d of 65536 clk high (setpoint %0d)", hi, setpoint);
        park = 1; repeat (3) @(posedge clk); hi = 0;
        for (t = 0; t < 1000; t = t + 1) begin @(posedge clk); #1; hi = hi + pv_pin; end
        check("parked: always high", hi == 1000);
        if (errors == 0) $display("PASS tb_battery (%0d sets, %0d ms checked)", nset, nms);
        else $display("FAIL tb_battery: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
