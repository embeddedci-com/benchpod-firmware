// tb_encoder — encoder emulation (src/encoder.v) against independent references.
//
//  1 angle: theta driven like the model (one step per 180-clk tick, forwards and backwards, more
//    than one mechanical revolution); ENC_ANGLE must equal floor(E / POLES) (+ DIR, OFS) where E
//    is the unwrapped electrical angle (bench arithmetic, 64 bits)
//  2 ABZ: decoded by a quadrature counter here. Only one of A/B changes at a time; just before
//    each new target loads, the count is within 1 of floor(target x CPR / 2^16); Z only at count
//    0 (with A = B = 0); steps within a tick are evenly spaced
//  3 SPI at 8 MHz, MISO sampled 10 ns before each sampling edge (translator delay plus setup):
//    AS5047P (angle, DIAAGC, write ZPOSM, parity / invalid / framing errors and ERRFL), AS5048A
//    (angle, address map), MA730 in modes 0 and 3 (angle, partial read, register read and write)
//  4 second run: new POLES, CPR, OFS and DIR mid-run, and MODEL_RESET
`timescale 1ns/1ps
`default_nettype none
module tb_encoder;
    reg clk = 0;
    always #13.889 clk = ~clk;
    reg rst = 1, tick = 0, reset_state = 0;
    reg [31:0] theta = 0;
    reg pw_en = 0; reg [2:0] pw_addr = 0; reg [15:0] pw_data = 0;
    reg cs = 1, sck = 0, mosi = 0;
    wire a, b, z, miso;
    wire [15:0] rf_q, angle, count, status;
    encoder dut (.clk(clk), .rst(rst), .tick(tick), .reset_state(reset_state), .theta(theta), .period(10'd180),
        .pw_en(pw_en), .pw_addr(pw_addr), .pw_data(pw_data), .cs_in(cs), .sck_in(sck), .mosi_in(mosi),
        .enc_a(a), .enc_b(b), .enc_z(z), .enc_miso(miso),
        .ctrl(), .cpr(), .poles(), .ofs(), .rf_addr(), .rf_link_q(rf_q),
        .angle(angle), .count(count), .status(status));

    integer errors = 0;
    task fail(input [8*72-1:0] what); begin errors = errors + 1; $display("FAIL: %0s", what); end endtask

    task wr(input [2:0] ad, input [15:0] v);
        begin @(negedge clk) begin pw_en = 1; pw_addr = ad; pw_data = v; end @(negedge clk) pw_en = 0; end
    endtask

    // ---------------------------------------------------------------- reference angle
    reg signed [63:0] e_un = 0;          // unwrapped electrical angle, 2^32 per revolution
    integer pp = 1, cpr = 4000, dir = 0;
    reg [15:0] ofs = 0;
    function [15:0] ref_angle(input integer dummy);
        reg signed [63:0] hi, m;
        reg [15:0] mech;
        begin
            hi = e_un >>> 16;
            m = hi % (pp * 65536); if (m < 0) m = m + pp * 65536;
            mech = m / pp;
            ref_angle = (dir ? -mech : mech) + ofs;
        end
    endfunction

    // ---------------------------------------------------------------- ABZ decoder
    integer qpos = 0;                     // decoded position, unbounded
    reg [15:0] count_d1, count_d2;        // A/B are registered from the count: compare 2 clk back
    always @(posedge clk) begin count_d1 <= count; count_d2 <= count_d1; end
    reg [1:0] ab_q = 2'b00;
    integer last_step = -1, min_iv = 1 << 30, max_iv = 0, steps_tick = 0, track_iv = 0;
    reg [3:0] cw = 0;                     // a CPR write restarts the count (a jump): follow silently
    always @(posedge clk) cw <= {cw[2:0], pw_en && pw_addr == 3'd1};
    always @(posedge clk) if (!rst) begin
        if (|cw) begin ab_q = {a, b}; qpos = 0; end
        else if ({a, b} != ab_q) begin
            // states 0-3: AB 00, 10, 11, 01
            case ({ab_q, a, b})
                4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: qpos = qpos + 1;
                4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: qpos = qpos - 1;
                default: fail("A and B changed together");
            endcase
            if (track_iv && last_step >= 0) begin
                if ($time - last_step < min_iv) min_iv = $time - last_step;
                if ($time - last_step > max_iv) max_iv = $time - last_step;
            end
            last_step = $time; steps_tick = steps_tick + 1;
            ab_q = {a, b};
        end
        if (z && (a || b)) fail("Z high outside A = B = 0");
        if (z && (((qpos % cpr) + cpr) % cpr) != 0) fail("Z high away from count 0");
    end

    // check at every target load: the generator has reached the previous target
    reg [15:0] tgt_prev; reg tgt_valid = 0;
    reg strict = 1;                      // off while the speed is above one step per clk (clamped)
    integer abz_checks = 0;
    always @(posedge clk) if (dut.ld) begin
        if (tgt_valid && strict) begin : chk
            integer want, got, dlt;
            want = (tgt_prev * cpr) >> 16;
            got = ((qpos % cpr) + cpr) % cpr;
            dlt = got - want; if (dlt > cpr / 2) dlt = dlt - cpr; if (dlt < -cpr / 2) dlt = dlt + cpr;
            if (dlt > 1 || dlt < -1) begin
                errors = errors + 1;
                if (errors < 10) $display("FAIL: ABZ count %0d, target %0d (angle %h, cpr %0d)", got, want, tgt_prev, cpr);
            end
            if (got != count_d1) begin errors = errors + 1; if (errors < 10) $display("FAIL: decoded %0d, ENC_COUNT %0d", got, count_d1); end
            abz_checks = abz_checks + 1;
        end
        tgt_prev = dut.angle; tgt_valid = 1;
    end

    task resync;                         // after a CPR write: the decoder restarts with the outputs
        begin repeat (6) @(negedge clk); tgt_valid = 0; end
    endtask

    // ---------------------------------------------------------------- ticks
    integer angle_checks = 0;
    task run_ticks(input integer n, input signed [31:0] step);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                @(negedge clk) tick = 1; @(negedge clk) tick = 0;
                repeat (60) @(negedge clk);
                if (angle !== ref_angle(0)) begin
                    errors = errors + 1;
                    if (errors < 10) $display("FAIL: angle %h, expected %h (theta %h, pp %0d)", angle, ref_angle(0), theta, pp);
                end
                angle_checks = angle_checks + 1;
                // the model moves theta during the tick
                theta = theta + step; e_un = e_un + step;
                repeat (180 - 62) @(negedge clk);
            end
        end
    endtask

    // ---------------------------------------------------------------- SPI master (8 MHz)
    localparam real HALF = 62.5;
    function par16(input [14:0] v); par16 = ^v; endfunction
    function [15:0] as_cmd(input rd, input [13:0] ad);
        as_cmd = {par16({rd, ad}), rd, ad};
    endfunction
    // mode: 1 (AS504x), 0 or 3 (MA730); nbits up to 16
    task xfer(input integer mode, input integer nbits, input [15:0] tx, output [15:0] rx);
        integer i;
        begin
            rx = 0;
            sck = (mode == 3); #500;
            cs = 0;
            if (mode == 0) mosi = tx[15];
            #400;
            for (i = 0; i < nbits; i = i + 1) begin
                if (mode == 1) begin
                    sck = 1; mosi = tx[15 - i]; #(HALF - 10);
                    rx = {rx[14:0], miso}; #10; sck = 0; #HALF;
                end else if (mode == 0) begin
                    #(HALF - 10); rx = {rx[14:0], miso}; #10; sck = 1; #HALF;
                    sck = 0; if (i < 15) mosi = tx[14 - i];
                end else begin                               // mode 3
                    sck = 0; mosi = tx[15 - i]; #(HALF - 10);
                    rx = {rx[14:0], miso}; #10; sck = 1; #HALF;
                end
            end
            #100; cs = 1; sck = (mode == 3); #800;
        end
    endtask

    reg [15:0] r;
    task expect_as(input [8*40-1:0] what, input [15:0] got, input ef, input [13:0] dat);
        begin
            if (got !== {par16({ef, dat}), ef, dat}) begin
                errors = errors + 1; $display("FAIL: %0s = %h, expected %h", what, got, {par16({ef, dat}), ef, dat});
            end
        end
    endtask

    initial begin
        #400_000_000 $display("FAIL tb_encoder: timeout"); $finish;
    end
    initial begin
        repeat (4) @(posedge clk); rst = 0;
        wr(3'd0, 16'h0003);                                  // ABZ and SPI on, AS5047P
        pp = 7; wr(3'd2, 7);
        cpr = 4000; wr(3'd1, 4000); resync;

        // ---------------- 1 + 2: angle and ABZ, forwards, fast enough to clamp, backwards
        run_ticks(3, 0);
        track_iv = 1; min_iv = 1 << 30; max_iv = 0;
        run_ticks(40, 32'sd91_750_400);                      // ~200 angle LSB (100 steps) per tick
        track_iv = 0;
        // 100 generator steps (2 LSB) per tick, a count every 8.2 steps: count edges every ~408 ns,
        // with up to one generator step (~2 clk) of jitter either way
        if (max_iv - min_iv > 120) begin errors = errors + 1; $display("FAIL: count edge intervals %0d .. %0d ns", min_iv, max_iv); end
        else $display("  ok count edges every %0d .. %0d ns at constant speed", min_iv, max_iv);
        strict = 0;
        run_ticks(10, 32'sd400_000_000);                     // faster than one step per clk: clamped
        run_ticks(25, 0);                                    // catches up while stopped
        strict = 1;
        run_ticks(15, 0);
        run_ticks(120, -32'sd91_750_400);                    // backwards, more than a revolution of E
        strict = 0;
        run_ticks(5, -32'sd1_000_000_000);                   // 0.23 electrical rev per tick (limit 1/4)
        run_ticks(30, 0);
        strict = 1;
        run_ticks(10, 0);
        $display("  ok angle and ABZ: %0d angle checks, %0d ABZ checks", angle_checks, abz_checks);

        // ---------------- 3: SPI, theta still
        // AS5047P
        xfer(1, 16, as_cmd(1, 14'h3FFF), r);                 // read ANGLECOM (response next frame)
        xfer(1, 16, as_cmd(1, 14'h3FFC), r); expect_as("AS5047P angle", r, 1'b0, angle[15:2]);
        xfer(1, 16, as_cmd(1, 14'h0000), r); expect_as("AS5047P DIAAGC", r, 1'b0, 14'h0180);
        xfer(1, 16, as_cmd(0, 14'h0016), r);                 // write ZPOSM ...
        xfer(1, 16, {par16({1'b0, 14'h00AB}), 1'b0, 14'h00AB}, r); expect_as("ZPOSM old (data frame)", r, 1'b0, 14'h0000);
        xfer(1, 16, as_cmd(1, 14'h0000), r); expect_as("ZPOSM new", r, 1'b0, 14'h00AB);
        wr(3'd4, 6'h36); repeat (4) @(negedge clk);
        if (rf_q !== 16'h00AB) begin errors = errors + 1; $display("FAIL: register file 0x36 = %h", rf_q); end
        xfer(1, 16, as_cmd(1, 14'h3FFF) ^ 16'h8000, r);      // bad parity
        xfer(1, 16, as_cmd(1, 14'h0001), r); expect_as("after parity error", r, 1'b1, 14'h0000);
        xfer(1, 16, as_cmd(1, 14'h0000), r); expect_as("ERRFL", r, 1'b1, 14'h0004);
        xfer(1, 16, as_cmd(1, 14'h1234), r); expect_as("NOP after ERRFL read", r, 1'b0, 14'h0000);
        xfer(1, 16, as_cmd(1, 14'h0001), r); expect_as("invalid address", r, 1'b1, 14'h0000);
        xfer(1, 16, as_cmd(1, 14'h0000), r); expect_as("ERRFL (INVCOMM)", r, 1'b1, 14'h0002);
        xfer(1, 8, 16'hFFFF, r);                             // 8-bit frame
        xfer(1, 16, as_cmd(1, 14'h0001), r);
        xfer(1, 16, as_cmd(1, 14'h0000), r); expect_as("ERRFL (FRERR)", r, 1'b1, 14'h0001);
        xfer(1, 16, as_cmd(1, 14'h0000), r); expect_as("EF cleared", r, 1'b0, 14'h0000);
        if (status[15:8] < 8'd15) begin errors = errors + 1; $display("FAIL: frame count %0d", status[15:8]); end

        // AS5048A: diagnostics at 0x3FFD (BenchPod loads it), 0x3FFC does not exist
        wr(3'd0, 16'h0007);
        wr(3'd4, 6'h3D); wr(3'd5, 16'h0180);
        xfer(1, 16, as_cmd(1, 14'h3FFD), r);
        xfer(1, 16, as_cmd(1, 14'h3FFF), r); expect_as("AS5048A diagnostics", r, 1'b0, 14'h0180);
        xfer(1, 16, as_cmd(1, 14'h3FFC), r); expect_as("AS5048A angle", r, 1'b0, angle[15:2]);
        xfer(1, 16, as_cmd(1, 14'h0001), r); expect_as("AS5048A 0x3FFC invalid", r, 1'b1, 14'h0000);
        xfer(1, 16, as_cmd(1, 14'h0000), r); expect_as("AS5048A error register", r, 1'b1, 14'h0002);

        // MA730 in mode 0 and mode 3
        wr(3'd0, 16'h000B);
        begin : ma
            integer md;
            for (md = 0; md <= 3; md = md + 3) begin
                xfer(md, 16, 16'h0000, r);
                if (r !== {angle[15:2], 2'b00}) begin errors = errors + 1; $display("FAIL: MA730 mode %0d angle %h, expected %h", md, r, {angle[15:2], 2'b00}); end
                xfer(md, 8, 16'h0000, r);
                if (r[7:0] !== angle[15:8]) begin errors = errors + 1; $display("FAIL: MA730 mode %0d 8-bit read %h", md, r[7:0]); end
                xfer(md, 16, 16'h4500, r);                   // read PPT(9:2)
                if (r !== {angle[15:2], 2'b00}) begin errors = errors + 1; $display("FAIL: MA730 read cmd frame %h", r); end
                xfer(md, 16, 16'h0000, r);
                if (r !== 16'hFF00) begin errors = errors + 1; $display("FAIL: MA730 mode %0d reg 5 = %h", md, r); end
                xfer(md, 16, 16'h0000, r);
                if (r !== {angle[15:2], 2'b00}) begin errors = errors + 1; $display("FAIL: MA730 angle after the reply %h", r); end
            end
            xfer(0, 16, 16'h8980, r);                        // write reg 9 = 0x80
            xfer(0, 16, 16'h0000, r);
            if (r !== 16'h8000) begin errors = errors + 1; $display("FAIL: MA730 write reply %h", r); end
            wr(3'd4, 6'h09); repeat (4) @(negedge clk);
            if (rf_q !== 16'h0080) begin errors = errors + 1; $display("FAIL: MA730 register 9 = %h", rf_q); end
        end
        $display("  ok SPI (AS5047P, AS5048A, MA730 modes 0 and 3) at 8 MHz");

        // ---------------- 4: second run with new parameters, then MODEL_RESET
        pp = 11; wr(3'd2, 11);
        ofs = 16'h4000; wr(3'd3, 16'h4000);
        dir = 1; wr(3'd0, 16'h0013);
        cpr = 1024; wr(3'd1, 1024); resync;                 // restarts the count at 0: a jump
        // erev restarts mod the new POLES: the reference follows from here
        e_un = {32'd0, theta} ;
        begin : rearm
            // the encoder's revolution count is whatever it was, mod 11; take it from the gateware
            e_un = e_un + (64'sd1 << 32) * dut.erev;
        end
        strict = 0; run_ticks(100, 32'sd150_000_000);        // walks from 0 to the angle first (up to half a turn)
        strict = 1; run_ticks(40, 32'sd150_000_000);
        run_ticks(20, 0);
        @(negedge clk) reset_state = 1; @(negedge clk) reset_state = 0;
        theta = 0; e_un = 0;
        strict = 0; run_ticks(95, 0);                        // the angle jumped: up to half a turn at 360 LSB per tick
        strict = 1; run_ticks(70, -32'sd120_000_000);
        run_ticks(20, 0);
        $display("  ok second run (POLES 11, CPR 1024, OFS, DIR) and MODEL_RESET: %0d angle checks, %0d ABZ checks", angle_checks, abz_checks);

        if (errors == 0) $display("PASS tb_encoder"); else $display("FAIL tb_encoder: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
