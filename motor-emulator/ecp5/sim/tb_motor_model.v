// tb_motor_model — motor model against a bit-exact reference of the fixed-point spec in
// motor_model.v's header (written independently here), every tick.
//
//  run 1  fixed speed, no current: duties, angle, halls; physical check: the line-line voltage
//         amplitude is sqrt(3) x E / vbus
//  run 2  dynamic mode with currents, torque, load, damping, CM loop and dead-time comp
//  run 3  re-armed: new parameters + reset_state mid-run (second run uses the new values)
//  run 4  bus below VBUS_MIN: duties park at 0x8000
`timescale 1ns/1ps
`default_nettype none
module tb_motor_model;
    reg clk = 0;
    always #13.889 clk = ~clk;
    reg rst = 1, tick = 0, reset_state = 0;
    reg [1:0] mode = 0;
    reg shape_we = 0; reg [9:0] shape_addr = 0; reg [15:0] shape_data = 0;
    reg cm_en = 0, dt_en = 0;
    reg signed [15:0] sa = 0, sb = 0, sc = 0, sbus = 18760;
    reg signed [15:0] oa = 0, ob = 0, oc = 0, obus = 0;
    reg signed [15:0] w_set = 18350, kt = 0, load = 0, invj = 0, kp = 0, ki = 0;
    reg [4:0] wshift = 12, jshift = 10;
    reg [15:0] ke = 10468, damp = 0, dt_frac = 1009, dt_idb = 200, hall_ofs = 0, margin = 1311;
    reg [2:0]  adv = 3;
    reg signed [15:0] rsub = 0;

    wire [15:0] duty_a, duty_b, duty_c, count;
    wire [31:0] theta;
    wire signed [31:0] w32;
    wire signed [15:0] e_amp, i0, v_cm, iq;
    wire [2:0] hall;
    wire busy;
    reg pw_en = 0;
    reg [4:0] pw_addr = 0;
    reg [15:0] pw_data = 0;
    // parameters go in through the link's write port (register - 0x20), one per clk
    task pw(input [4:0] a, input [15:0] d);
        begin @(negedge clk) begin pw_en = 1; pw_addr = a; pw_data = d; end @(negedge clk) pw_en = 0; @(negedge clk); end
    endtask
    task set_params;
        begin
            pw(5'h01, w_set); pw(5'h02, {11'd0, wshift}); pw(5'h03, ke); pw(5'h04, kt); pw(5'h05, load);
            pw(5'h06, damp); pw(5'h07, invj); pw(5'h08, {11'd0, jshift}); pw(5'h09, kp); pw(5'h0A, ki);
            pw(5'h0B, margin); pw(5'h0C, dt_frac); pw(5'h0D, dt_idb); pw(5'h0E, hall_ofs); pw(5'h1E, {13'd0, adv});
            pw(5'h1A, oa); pw(5'h1B, ob); pw(5'h1C, oc); pw(5'h1D, obus); pw(5'h1F, rsub);
        end
    endtask

    motor_model dut (
        .clk(clk), .rst(rst), .tick(tick), .mode(mode), .cm_en(cm_en), .dt_en(dt_en), .reset_state(reset_state),
        .sinc_a(sa), .sinc_b(sb), .sinc_c(sc), .sinc_bus(sbus),
        .pw_en(pw_en), .pw_addr(pw_addr), .pw_data(pw_data),
        .shape_we(shape_we), .shape_addr(shape_addr), .shape_data(shape_data),
        .duty_a(duty_a), .duty_b(duty_b), .duty_c(duty_c), .theta(theta), .w32(w32), .e_amp(e_amp),
        .i0(i0), .v_cm(v_cm), .iq(iq), .hall(hall), .count(count), .busy(busy));

    // ------------------------------------------------------------------ reference
    reg [15:0] sine [0:1023];
    initial $readmemh("src/sine1024.hex", sine);
    reg [31:0] r_theta;
    reg [15:0] r_look;
    reg signed [63:0] r_ua, r_ub, r_uc;              // leg voltages before the offset (for run 5)
    reg signed [31:0] r_w32, r_cm_int;
    reg [15:0] r_da, r_db, r_dc;
    reg signed [15:0] r_e, r_i0, r_vcm, r_iq;
    reg [2:0] r_hall;

    function signed [15:0] sat;
        input signed [63:0] x;
        sat = (x > 32767) ? 16'sd32767 : (x < -32768) ? -16'sd32768 : x[15:0];
    endfunction
    function signed [15:0] neg;
        input signed [15:0] x;
        neg = (x == -16'sd32768) ? 16'sd32767 : -x;
    endfunction
    function signed [15:0] sn;               // signed sine entry
        input [9:0] idx;
        sn = sine[idx];
    endfunction
    // duty = 0x8000 +- (|v| x floor(2^30 / vbus)) >> 14, saturated
    function [15:0] to_duty;
        input signed [63:0] v;
        input signed [63:0] vbv;
        reg signed [63:0] q, r;
        begin
            r = (64'sd1 << 30) / vbv;
            q = ((v < 0 ? -v : v) * r) >>> 14;
            if (v < 0) to_duty = (q >= 32768) ? 16'h0000 : 16'h8000 - q[15:0];
            else to_duty = (q >= 32768) ? 16'hFFFF : 16'h8000 + q[15:0];
        end
    endfunction
    function signed [15:0] dtc_ref;
        input signed [15:0] i, v; input [15:0] idb;
        dtc_ref = (i > $signed({1'b0, idb[14:0]})) ? v : (i < -$signed({1'b0, idb[14:0]})) ? -v : 16'sd0;
    endfunction

    task ref_tick;
        reg signed [63:0] ia, ib, ic, vb, w16, kec, dmc, s_a, s_b, s_c, q_a, q_b, q_c, e_a, e_b, e_c, acc, trq, net, cmp, dtv;
        reg signed [63:0] u_a, u_b, u_c, mx, mn, v_a, v_b, v_c, hv, sum, i0sum4, mg, lim, off, spr, sh1, sh2, toff, vcm;
        reg [15:0] t16, tb_, tc_, lb_, lc_, advq, st16;
        reg [15:0] ha, hb, hc;
        reg signed [31:0] step;
        begin
            ia = sat(sa - oa); ib = sat(sb - ob); ic = sat(sc - oc); vb = sat(sbus - obus);
            if (mode == 1) r_w32 = {w_set, 16'h0000};
            w16 = r_w32 >>> 16;
            // 16-bit angles: 1/3 revolution = 0x5555, sine index = angle[15:6]
            t16 = r_theta[31:16];
            tb_ = t16 - 16'h5555;  tc_ = t16 + 16'h5555;
            lb_ = r_look - 16'h5555; lc_ = r_look + 16'h5555;
            s_a = sn(r_look[15:6]); s_b = sn(lb_[15:6]); s_c = sn(lc_[15:6]);         // back-EMF
            q_a = sn(t16[15:6]);    q_b = sn(tb_[15:6]); q_c = sn(tc_[15:6]);        // torque
            kec = ke[15] ? 32767 : ke; dmc = damp[15] ? 32767 : damp;
            hv = sat((vb < 0 ? 0 : vb) >>> 1);
            r_e = sat((w16 * kec) >>> 15);
            i0sum4 = (ia + ib + ic) >>> 2;
            r_i0 = sat((i0sum4 * 21845) >>> 14);
            dtv = sat((vb * (dt_frac >> 1)) >>> 15);
            mg = sat((vb * (margin >> 1)) >>> 15);
            lim = sat(hv - mg);
            e_a = sat((r_e * s_a) >>> 15); e_b = sat((r_e * s_b) >>> 15); e_c = sat((r_e * s_c) >>> 15);
            cmp = sat((kp * r_i0) >>> 8);
            if (cm_en) begin
                sum = r_cm_int + ki * r_i0;
                r_cm_int = (sum > 64'sh7FFFFFFF) ? 32'sh7FFFFFFF : (sum < -64'sh80000000) ? 32'sh80000000 : sum[31:0];
                vcm = -(cmp + (r_cm_int >>> 16));
                if (vcm > hv) vcm = hv; else if (vcm < -hv) vcm = -hv;
            end else begin r_cm_int = 0; vcm = 0; end
            r_vcm = sat(vcm);
            u_a = sat(e_a + ((ia * rsub) >>> 15) + (dt_en ? dtc_ref(ia, dtv, dt_idb) : 0));
            u_b = sat(e_b + ((ib * rsub) >>> 15) + (dt_en ? dtc_ref(ib, dtv, dt_idb) : 0));
            u_c = sat(e_c + ((ic * rsub) >>> 15) + (dt_en ? dtc_ref(ic, dtv, dt_idb) : 0));
            r_ua = u_a; r_ub = u_b; r_uc = u_c;
            mx = (u_a > u_b) ? ((u_a > u_c) ? u_a : u_c) : ((u_b > u_c) ? u_b : u_c);
            mn = (u_a < u_b) ? ((u_a < u_c) ? u_a : u_c) : ((u_b < u_c) ? u_b : u_c);
            off = sat((mx + mn) >>> 1);
            spr = sat(mx - off);
            sh2 = sat(r_vcm + spr - lim); if (sh2 < 0) sh2 = 0;
            sh1 = sat(spr - r_vcm - lim); if (sh1 < 0) sh1 = 0;
            toff = sat(r_vcm + sh1 - sh2 - off);
            v_a = u_a + toff; v_b = u_b + toff; v_c = u_c + toff;
            if (v_a > hv) v_a = hv; else if (v_a < -hv) v_a = -hv;
            if (v_b > hv) v_b = hv; else if (v_b < -hv) v_b = -hv;
            if (v_c > hv) v_c = hv; else if (v_c < -hv) v_c = -hv;
            if (vb < 512) begin r_da = 16'h8000; r_db = 16'h8000; r_dc = 16'h8000; end
            else begin r_da = to_duty(v_a, vb); r_db = to_duty(v_b, vb); r_dc = to_duty(v_c, vb); end
            // torque and mechanics (for the next tick)
            acc = neg(ia) * q_a + neg(ib) * q_b + neg(ic) * q_c;
            r_iq = sat(acc >>> 15);
            trq = sat((kt * r_iq) >>> 15);
            net = sat(trq - load - ((w16 * dmc) >>> 15));
            if (mode == 2) r_w32 = r_w32 + ((net * invj) >>> jshift);
            ha = t16 + hall_ofs; hb = tb_ + hall_ofs; hc = tc_ + hall_ofs;
            r_hall = {~hc[15], ~hb[15], ~ha[15]};
            step = r_w32 >>> (16 - wshift);          // signed on its own: an unsigned operand in
            r_theta = r_theta + step;                // the same expression would make >>> logical
            st16 = step[31:16];
            advq = (adv[2] ? {st16[14:0], 1'b0} : 16'd0) + (adv[1] ? st16 : 16'd0) +
                   (adv[0] ? {st16[15], st16[15:1]} : 16'd0);
            r_look = r_theta[31:16] + advq;
        end
    endtask

    integer errors = 0, ticks = 0;
    task do_tick;
        begin
            @(negedge clk) tick = 1; @(negedge clk) tick = 0;
            ref_tick;
            // busy sampled after clock edges (a level wait can catch a zero-width delta glitch
            // between two flops changing on the same edge)
            do begin @(posedge clk); #1; end while (busy !== 1'b0);
            repeat (2) @(posedge clk); #1;
            ticks = ticks + 1;
            if (duty_a !== r_da || duty_b !== r_db || duty_c !== r_dc || theta !== r_theta || w32 !== r_w32 ||
                e_amp !== r_e || i0 !== r_i0 || v_cm !== r_vcm || iq !== r_iq || hall !== r_hall) begin
                errors = errors + 1;
                if (errors < 8) $display("FAIL tick %0d: duty %h %h %h / %h %h %h th %h/%h w %0d/%0d E %0d/%0d i0 %0d/%0d vcm %0d/%0d iq %0d/%0d hall %b/%b",
                    ticks, duty_a, duty_b, duty_c, r_da, r_db, r_dc, theta, r_theta, w32, r_w32, e_amp, r_e,
                    i0, r_i0, v_cm, r_vcm, iq, r_iq, hall, r_hall);
            end
            repeat (180 - 110) @(posedge clk);
        end
    endtask

    integer k, n_hall_changes;
    real vll, vll_max;
    reg [2:0] hall_q;
    initial begin
        #200_000_000 $display("FAIL tb_motor_model: timeout"); $finish;
    end
    initial begin
        r_theta = 0; r_look = 0; r_w32 = 0; r_cm_int = 0;
        repeat (4) @(posedge clk); rst = 0;
        set_params;
        // ---- run 1: fixed speed: 18350 << 16 >>> 4 per tick = 57 ticks per electrical revolution
        mode = 1; w_set = 18350;
        vll_max = 0; n_hall_changes = 0; hall_q = 3'bxxx;
        for (k = 0; k < 400; k = k + 1) begin
            do_tick;
            vll = ($itor(duty_a) - $itor(duty_b)) / 65536.0 * 18760.0;
            if (vll > vll_max) vll_max = vll;
            if (hall !== hall_q && k > 0) n_hall_changes = n_hall_changes + 1;
            hall_q = hall;
        end
        // E = 18350 * 10468 >> 15 = 5862 codes; line-line peak sqrt(3) * E = 10153
        if (vll_max < 10153 * 0.99 || vll_max > 10153 * 1.01) begin
            errors = errors + 1; $display("FAIL: line-line peak %0.0f codes, expected 10153", vll_max);
        end else $display("  ok line-line peak %0.0f codes (sqrt(3) x E = 10153)", vll_max);
        if (n_hall_changes < 40 || n_hall_changes > 44) begin errors = errors + 1; $display("FAIL: only %0d hall changes", n_hall_changes); end
        else $display("  ok %0d hall edges over 400 ticks (7 revolutions x 6)", n_hall_changes);

        // ---- run 2: dynamic, currents in phase with the back-EMF, load, damping, CM loop, DT comp
        mode = 0;
        kt = 12000; load = 300; damp = 2000; invj = 9000; jshift = 6; kp = 7815; ki = 5000; rsub = 275;
        set_params;
        mode = 2; cm_en = 1; dt_en = 1;
        for (k = 0; k < 400; k = k + 1) begin
            // phase currents follow the model angle (motoring: motor current in phase = -i)
            sa = -$rtoi(3000.0 * $sin(2.0 * 3.14159265 * $itor(theta) / 4294967296.0)) + 150;
            sb = -$rtoi(3000.0 * $sin(2.0 * 3.14159265 * ($itor(theta) / 4294967296.0 - 1.0 / 3.0))) + 150;
            sc = -$rtoi(3000.0 * $sin(2.0 * 3.14159265 * ($itor(theta) / 4294967296.0 + 1.0 / 3.0))) + 150;
            do_tick;
        end
        $display("  ok dynamic: w16 %0d, iq %0d, i0 %0d, v_cm %0d after 400 ticks", w32 >>> 16, iq, i0, v_cm);

        // ---- run 3: re-arm with new parameters and reset the state
        ke = 20000; hall_ofs = 16'h2AAA; kt = -5000; load = -100; dt_frac = 2000; obus = 100; oa = 150; ob = 150; oc = 150;
        rsub = 1200;
        w_set = -9000;
        set_params;
        @(negedge clk) reset_state = 1; @(negedge clk) reset_state = 0;
        r_theta = 0; r_look = 0; r_w32 = {w_set, 16'h0000}; r_cm_int = 0;
        for (k = 0; k < 300; k = k + 1) do_tick;
        mode = 1;
        for (k = 0; k < 50; k = k + 1) do_tick;

        // ---- run 5: common-mode loop driven to the rail: shaping keeps every leg inside the margin
        // and the line-line voltages (u_a - u_b) intact while the spread fits
        mode = 1; w_set = 18350; ke = 10468; cm_en = 1; dt_en = 0; kp = 20000; ki = 20000; oa = 0; ob = 0; oc = 0;
        obus = 0; margin = 1311; adv = 3; rsub = 0; set_params;
        sa = 4000; sb = 4000; sc = 4000;                 // big i0: v_cm heads for the rail
        for (k = 0; k < 120; k = k + 1) begin
            do_tick;
            if (k > 20) begin
                if (duty_a < 1311 - 4 || duty_b < 1311 - 4 || duty_c < 1311 - 4 ||
                    duty_a > 65535 - 1311 + 4 || duty_b > 65535 - 1311 + 4 || duty_c > 65535 - 1311 + 4) begin
                    errors = errors + 1;
                    if (errors < 12) $display("FAIL: leg outside the margin: %h %h %h", duty_a, duty_b, duty_c);
                end
                vll = ($itor(duty_a) - $itor(duty_b)) - $itor(r_ua - r_ub) * 65536.0 / 18760.0;
                if (vll > 3.0 || vll < -3.0) begin
                    errors = errors + 1;
                    if (errors < 12) $display("FAIL: line-line changed by %0.1f duty LSB", vll);
                end
            end
        end
        $display("  ok shaping: legs inside the 2 %% margin, line-line kept (v_cm %0d)", v_cm);

        // ---- run 6: a trapezoidal back-EMF shape loaded through the shape port (and into the
        // reference): 120-degree flat tops, linear 60-degree ramps; bit-exact as before
        for (k = 0; k < 1024; k = k + 1) begin : trap
            integer v;
            v = (k < 85) ? (k * 30000) / 85 : (k < 427) ? 30000 : (k < 597) ? 30000 - ((k - 427) * 60000) / 170 :
                (k < 939) ? -30000 : -30000 + ((k - 939) * 30000) / 85;
            @(negedge clk) begin shape_we = 1; shape_addr = k; shape_data = v; end
            sine[k] = v;
        end
        @(negedge clk) shape_we = 0;
        mode = 2; cm_en = 1; dt_en = 1; sa = 1200; sb = -500; sc = -700; obus = 0; set_params;
        for (k = 0; k < 150; k = k + 1) do_tick;
        $display("  ok trapezoidal shape: %0d ticks bit-exact so far", ticks);
        for (k = 0; k < 1024; k = k + 1) begin : back
            @(negedge clk) begin shape_we = 1; shape_addr = k; shape_data = 0; end
        end
        @(negedge clk) shape_we = 0;
        $readmemh("src/sine1024.hex", sine);
        for (k = 0; k < 1024; k = k + 1) begin
            @(negedge clk) begin shape_we = 1; shape_addr = k; shape_data = sine[k]; end
        end
        @(negedge clk) shape_we = 0;

        // ---- run 4: bus collapsed
        sbus = 400; obus = 0; set_params;
        for (k = 0; k < 5; k = k + 1) do_tick;
        if (duty_a !== 16'h8000 || duty_b !== 16'h8000 || duty_c !== 16'h8000) begin
            errors = errors + 1; $display("FAIL: low bus did not park the duties");
        end

        if (errors == 0) $display("PASS tb_motor_model (%0d ticks bit-exact)", ticks);
        else $display("FAIL tb_motor_model: %0d of %0d ticks differ", errors, ticks);
        $finish;
    end
endmodule
`default_nettype wire
