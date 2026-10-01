// tb_top — whole-chip bench over the real SPI link (compiled with -DSIM: clk12 carries 36 MHz).
//
//  1  reset defaults: every gate low, hot-swap/brake off, halls released, PV_SET parked, MCLK 18 MHz
//  2  board id from the EEPROM (0x51 -> board 1); reads answered only when addressed; MISO floats
//     for another board's read
//  3  broadcast write, addressed write to another board ignored, auto-increment write + read
//  4  sinc readback against five delta-sigma modulator models
//  5  safety chain: PWM_EN alone does nothing; the latch arms it; FAULT stops every gate within a
//     few clk and clears PWM_EN; re-enable with new duties (second run) uses them
//  6  board 1 cannot enable hot-swap, brake or SYNC master
//  7  re-probe with the EEPROM at 0x50: board 0 now may (hot-swap, brake PWM, 1 ms SYNC)
//  8  ARM pulse 100 us; SYNC held low by another board trips the watchdog and WD_CLEAR clears it
//  9  motor model at fixed speed over the link: ticks, back-EMF amplitude, halls from the model
// 10  encoder over the link: ABZ moves with the model, and an AS5047P read on the encoder pins
//     returns ENC_ANGLE
`timescale 1ns/1ps
`default_nettype none
`include "emu_defs.vh"
module tb_top;
    reg clk = 0;
    always #13.889 clk = ~clk;              // 36 MHz on clk12 (SIM bypasses the PLL)

    reg sck = 0, ss = 1, mosi = 0;
    tri miso;
    tri1 sync_n, arm_req_n, led_n, sda, scl;
    wire arm;
    assign arm_req_n = arm ? 1'b0 : 1'bz;          // the board's N-FET
    reg  ecs = 1, esck = 0, emosi = 0;
    reg fault_n = 1, latch_q = 0, backstop = 0;
    reg ext_sync_low = 0;
    assign sync_n = ext_sync_low ? 1'b0 : 1'bz;
    wire ha, la, hb, lb, hc, lc, brake, hswap_en, pv_set, mclk, ee_wc_n;
    wire hall_a, hall_b, hall_c, enc_a, enc_b, enc_z, enc_miso;
    tri1 ee_wc_n_pu;
    reg  [4:0] dout = 0;

    top dut (
        .clk12(clk), .spi_sck(sck), .spi_ss(ss), .spi_mosi(mosi), .spi_miso(miso),
        .sync_n(sync_n), .fault_n(fault_n),
        .pwm_ha(ha), .pwm_la(la), .pwm_hb(hb), .pwm_lb(lb), .pwm_hc(hc), .pwm_lc(lc),
        .arm(arm), .latch_q(latch_q),
        .brake(brake), .hswap_en(hswap_en), .backstop(backstop), .pv_set(pv_set),
        .mclk(mclk), .dout_a(dout[0]), .dout_b(dout[1]), .dout_c(dout[2]), .dout_d(dout[3]), .dout_e(dout[4]),
        .i2c_sda(sda), .i2c_scl(scl), .ee_wc_n(ee_wc_n_pu),
        .hall_a(hall_a), .hall_b(hall_b), .hall_c(hall_c), .enc_a(enc_a), .enc_b(enc_b), .enc_z(enc_z),
        .enc_cs(ecs), .enc_sck(esck), .enc_mosi(emosi), .enc_miso(enc_miso), .led_n(led_n));

    reg [6:0] ee_addr = 7'h51;
    i2c_ack_model ee (.sda(sda), .scl(scl), .addr7(ee_addr), .present(1'b1));

    // ---- five second-order delta-sigma modulators on MCLK (DOUT changes 8 ns after the rise)
    real mx [0:4];
    real mi1 [0:4];
    real mi2 [0:4];
    integer m;
    initial for (m = 0; m < 5; m = m + 1) begin mi1[m] = 0; mi2[m] = 0; end
    initial begin mx[0] = 0.25; mx[1] = -0.5; mx[2] = 0.0; mx[3] = 0.1; mx[4] = 0.6; end
    always @(posedge mclk) begin : mod
        real fb;
        for (m = 0; m < 5; m = m + 1) begin
            fb = dout[m] ? 1.0 : -1.0;
            mi1[m] = mi1[m] + mx[m] - fb;
            mi2[m] = mi2[m] + mi1[m] - fb;
            dout[m] <= #8 (mi2[m] >= 0.0);
        end
    end

    // ---- SPI master, mode 0, 6 MHz
    localparam HALF = 83;
    reg [7:0] rx;
    integer miso_z_bits;
    task spi_byte(input [7:0] tx);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b]; #HALF; sck = 1;
                if (miso === 1'bz) miso_z_bits = miso_z_bits + 1;
                rx[b] = miso; #HALF; sck = 0;
            end
        end
    endtask
    // DUT-side SPI master on the encoder pins: mode 1, 4 MHz
    task enc_frame(input [15:0] tx, output [15:0] r);
        integer b;
        begin
            r = 0; #300; ecs = 0; #400;
            for (b = 15; b >= 0; b = b - 1) begin
                esck = 1; emosi = tx[b]; #115; r = {r[14:0], enc_miso}; #10; esck = 0; #125;
            end
            #100; ecs = 1; #500;
        end
    endtask
    task reg_write(input bcast, input [1:0] id, input [7:0] addr, input [15:0] data);
        begin
            ss = 0; #200;
            spi_byte({1'b0, bcast, id, 4'h0}); spi_byte(addr); spi_byte(data[15:8]); spi_byte(data[7:0]);
            #200; ss = 1; #400;
        end
    endtask
    task reg_read(input [1:0] id, input [7:0] addr, output [15:0] data);
        begin
            ss = 0; #200;
            spi_byte({1'b1, 1'b0, id, 4'h0}); spi_byte(addr); spi_byte(8'h00);
            spi_byte(8'h00); data[15:8] = rx; spi_byte(8'h00); data[7:0] = rx;
            #200; ss = 1; #400;
        end
    endtask

    integer errors = 0;
    reg [15:0] v, v2, v3;
    task expect16(input [8*32-1:0] what, input [15:0] got, input [15:0] exp);
        if (got !== exp) begin errors = errors + 1; $display("FAIL: %0s = %h, expected %h", what, got, exp); end
        else $display("  ok %0s = %h", what, got);
    endtask
    task expect1(input [8*40-1:0] what, input got, input exp);
        if (got !== exp) begin errors = errors + 1; $display("FAIL: %0s = %b, expected %b", what, got, exp); end
    endtask

    integer i, cnt_h, cnt_l, t_arm0, t_arm1, sync_pulses, n;
    initial begin
        #40_000_000 $display("FAIL tb_top: timeout"); $finish;
    end
    initial begin
        // ---------------- 1: reset defaults
        repeat (3000) @(posedge clk);
        expect1("gates", ha | la | hb | lb | hc | lc, 1'b0);
        expect1("hswap_en", hswap_en, 1'b0);
        expect1("brake", brake, 1'b0);
        expect1("hall pins (released)", hall_a | hall_b | hall_c, 1'b0);
        expect1("pv_set parked", pv_set, 1'b1);
        expect1("ee_wc_n", ee_wc_n_pu, 1'b1);
        expect1("arm_req_n", arm_req_n, 1'b1);
        n = 0; for (i = 0; i < 100; i = i + 1) begin @(posedge clk); #1 if (mclk) n = n + 1; end
        if (n != 50) begin errors = errors + 1; $display("FAIL: MCLK high %0d of 100 clk", n); end

        // ---------------- 2: board id and addressing
        wait (dut.probe_done === 1'b1);
        reg_read(2'd1, `R_BOARD, v);  expect16("R_BOARD (board 1)", v, 16'h0301);
        reg_read(2'd1, `R_ID, v);     expect16("R_ID", v, `EMU_ID);
        miso_z_bits = 0;
        reg_read(2'd0, `R_ID, v);
        if (miso_z_bits != 40) begin errors = errors + 1; $display("FAIL: MISO driven during another board's read (%0d of 40 bits Z)", miso_z_bits); end
        else $display("  ok MISO floats for another board's read");

        // ---------------- 3: broadcast, other-board write, auto-increment
        reg_write(1'b1, 2'd3, `R_SCRATCH, 16'hA55A);
        reg_read(2'd1, `R_SCRATCH, v); expect16("scratch after broadcast", v, 16'hA55A);
        reg_write(1'b0, 2'd2, `R_SCRATCH, 16'h1234);
        reg_read(2'd1, `R_SCRATCH, v); expect16("scratch after board-2 write", v, 16'hA55A);
        ss = 0; #200;                                  // DUTY_A, B, C in one transaction
        spi_byte(8'h10); spi_byte(`R_DUTY_A);
        spi_byte(8'h40); spi_byte(8'h00); spi_byte(8'h80); spi_byte(8'h00); spi_byte(8'hC0); spi_byte(8'h00);
        #200; ss = 1; #400;
        ss = 0; #200;
        spi_byte(8'h90); spi_byte(`R_DUTY_A); spi_byte(8'h00);
        spi_byte(0); v[15:8] = rx; spi_byte(0); v[7:0] = rx;
        spi_byte(0); v2[15:8] = rx; spi_byte(0); v2[7:0] = rx;
        spi_byte(0); v3[15:8] = rx; spi_byte(0); v3[7:0] = rx;
        #200; ss = 1; #400;
        expect16("DUTY_A", v, 16'h4000); expect16("DUTY_B", v2, 16'h8000); expect16("DUTY_C", v3, 16'hC000);

        // ---------------- 4: sinc readback
        for (i = 0; i < 5; i = i + 1) begin
            reg_read(2'd1, `R_SINC_A + i, v);
            if ($signed(v) < mx[i] * 32768 - 150 || $signed(v) > mx[i] * 32768 + 150) begin
                errors = errors + 1; $display("FAIL: sinc %0d = %0d, expected about %0.0f", i, $signed(v), mx[i] * 32768);
            end else $display("  ok sinc %0d = %0d", i, $signed(v));
        end

        // ---------------- 5: safety chain
        reg_write(1'b0, 2'd1, `R_DEADTIME, 16'd2);
        reg_write(1'b0, 2'd1, `R_CONTROL, 16'h0001);          // PWM_EN, latch not armed
        repeat (400) @(posedge clk);
        expect1("gates with latch open", ha | la | hb | lb | hc | lc, 1'b0);
        reg_read(2'd1, `R_STATUS, v); expect1("STATUS.pwm_active (latch open)", v[`S_PWM_ACTIVE], 1'b0);
        latch_q = 1;
        cnt_h = 0; cnt_l = 0;
        for (i = 0; i < 180 * 20; i = i + 1) begin @(posedge clk); cnt_h = cnt_h + ha; cnt_l = cnt_l + la; end
        if (cnt_h < 20 * (45 - 4) || cnt_h > 20 * (45 + 1) || cnt_l < 20 * 120) begin
            errors = errors + 1; $display("FAIL: leg A HS %0d LS %0d clk over 20 periods at 25 %%", cnt_h, cnt_l);
        end else $display("  ok leg A switching: HS %0d, LS %0d clk over 20 periods", cnt_h, cnt_l);
        reg_read(2'd1, `R_STATUS, v); expect1("STATUS.pwm_active", v[`S_PWM_ACTIVE], 1'b1);
        @(negedge clk) fault_n = 0;
        repeat (4) @(posedge clk); #1;
        expect1("gates 4 clk after FAULT", ha | la | hb | lb | hc | lc, 1'b0);
        repeat (20) @(posedge clk);
        fault_n = 1;
        repeat (400) @(posedge clk);
        expect1("gates after FAULT released", ha | la | hb | lb | hc | lc, 1'b0);
        reg_read(2'd1, `R_CONTROL, v); expect16("CONTROL after FAULT", v, 16'h0000);
        // second run with new duties
        reg_write(1'b0, 2'd1, `R_DUTY_A, 16'hC000);
        reg_write(1'b0, 2'd1, `R_CONTROL, 16'h0001);
        repeat (400) @(posedge clk);
        cnt_h = 0; for (i = 0; i < 180 * 20; i = i + 1) begin @(posedge clk); cnt_h = cnt_h + ha; end
        if (cnt_h < 20 * (135 - 4) || cnt_h > 20 * (135 + 1)) begin
            errors = errors + 1; $display("FAIL: re-enabled leg A HS %0d clk over 20 periods at 75 %%", cnt_h);
        end else $display("  ok re-enabled at 75 %%: HS %0d clk over 20 periods", cnt_h);

        // ---------------- 6: board 1 may not drive the power path or SYNC
        reg_write(1'b0, 2'd1, `R_BRAKE_DUTY, 16'd128);
        reg_write(1'b0, 2'd1, `R_CONTROL, 16'h000F);         // PWM, HSWAP, BRAKE, SYNC_MASTER
        sync_pulses = 0;
        for (i = 0; i < 40000; i = i + 1) begin
            @(posedge clk);
            if (hswap_en || brake) begin errors = errors + 1; $display("FAIL: board 1 drove hswap_en/brake"); i = 40000; end
            if (sync_n === 1'b0) sync_pulses = sync_pulses + 1;
        end
        if (sync_pulses != 0) begin errors = errors + 1; $display("FAIL: board 1 pulsed SYNC"); end

        // ---------------- 7: re-probe as board 0
        ee_addr = 7'h50;
        reg_write(1'b0, 2'd1, `R_BOARD, 16'h8000);
        wait (dut.probe_done === 1'b0); wait (dut.probe_done === 1'b1);
        reg_read(2'd0, `R_BOARD, v); expect16("R_BOARD (board 0)", v, 16'h0300);
        repeat (100) @(posedge clk);
        expect1("board 0 hswap_en", hswap_en, 1'b1);
        n = 0; sync_pulses = 0;
        for (i = 0; i < 80000; i = i + 1) begin
            @(posedge clk);
            n = n + brake;
            if (sync_n === 1'b0) sync_pulses = sync_pulses + 1;
        end
        if (n < 80000 * 120 / 256 || n > 80000 * 136 / 256) begin errors = errors + 1; $display("FAIL: brake on %0d of 80000 clk at 128/256", n); end
        else $display("  ok brake duty %0d / 80000", n);
        if (sync_pulses < 2 * 36 || sync_pulses > 3 * 36) begin errors = errors + 1; $display("FAIL: SYNC low %0d clk in 2.2 ms", sync_pulses); end
        else $display("  ok SYNC master: %0d clk low in 2.2 ms", sync_pulses);

        // ---------------- 8: ARM pulse and SYNC watchdog
        fork
            reg_write(1'b0, 2'd0, `R_ARM, 16'h0001);
            begin @(negedge arm_req_n); t_arm0 = $time; @(posedge arm_req_n); t_arm1 = $time; end
        join
        // 100-101 us: the gateware counts it in us ticks
        if (t_arm1 - t_arm0 < 99900 || t_arm1 - t_arm0 > 101100) begin errors = errors + 1; $display("FAIL: ARM pulse %0d ns", t_arm1 - t_arm0); end
        else $display("  ok ARM pulse %0d ns", t_arm1 - t_arm0);
        reg_write(1'b0, 2'd0, `R_CONTROL, 16'h0001);
        repeat (200) @(posedge clk);
        ext_sync_low = 1; #150_000; ext_sync_low = 0;
        reg_read(2'd0, `R_STATUS, v); expect1("STATUS.wd_trip", v[`S_WD_TRIP], 1'b1);
        expect1("gates after SYNC trip", ha | la | hb | lb | hc | lc, 1'b0);
        reg_write(1'b0, 2'd0, `R_WD_CLEAR, 16'h0001);
        reg_read(2'd0, `R_STATUS, v); expect1("STATUS.wd_trip after clear", v[`S_WD_TRIP], 1'b0);
        reg_read(2'd0, `R_CONTROL, v); expect1("PWM_EN cleared by the trip", v[`C_PWM_EN], 1'b0);

        // ---------------- 9: motor model over the link (fixed speed, halls from the model)
        reg_write(1'b0, 2'd0, `R_KE, 16'd10468);
        reg_write(1'b0, 2'd0, `R_W_SET, 16'd18350);
        reg_write(1'b0, 2'd0, `R_MODEL_RESET, 16'd1);
        reg_read(2'd0, `R_MODEL_COUNT, v2);
        reg_write(1'b0, 2'd0, `R_MODE, 16'h0011);                // fixed speed, halls from model
        n = 0;
        for (i = 0; i < 180 * 120; i = i + 1) begin
            @(posedge clk);
            if (i > 0 && hall_a != cnt_h) n = n + 1;
            cnt_h = hall_a;
        end
        reg_read(2'd0, `R_MODEL_COUNT, v3);
        if (v3 - v2 < 110) begin errors = errors + 1; $display("FAIL: model ran %0d ticks", v3 - v2); end
        else $display("  ok model ticks %0d over 120 periods", v3 - v2);
        if (n < 3) begin errors = errors + 1; $display("FAIL: hall A toggled %0d times from the model", n); end
        else $display("  ok hall A toggled %0d times (2 per electrical revolution)", n);
        reg_read(2'd0, `R_EMF, v); expect16("model E", v, 16'd5862);
        reg_read(2'd0, `R_MDUTY_A, v);
        if (v == 16'h8000) begin errors = errors + 1; $display("FAIL: model duty stuck at 0x8000"); end

        // ---------------- 10: encoder over the link (the model still turning)
        reg_write(1'b0, 2'd0, `R_ENC_POLES, 16'd7);
        reg_write(1'b0, 2'd0, `R_ENC_CPR, 16'd4000);
        reg_write(1'b0, 2'd0, `R_ENC_CTRL, 16'h0003);            // ABZ and SPI, AS5047P
        repeat (180 * 10) @(posedge clk);
        reg_read(2'd0, `R_ENC_ANGLE, v2);
        n = 0;
        for (i = 0; i < 180 * 20; i = i + 1) begin
            @(posedge clk);
            if (i > 0 && enc_a != cnt_h) n = n + 1;
            cnt_h = enc_a;
        end
        reg_read(2'd0, `R_ENC_ANGLE, v3);
        if (n < 20 || v3 == v2) begin errors = errors + 1; $display("FAIL: encoder: %0d A edges, angle %h -> %h", n, v2, v3); end
        else $display("  ok encoder: %0d A edges in 20 periods, angle %h -> %h", n, v2, v3);
        reg_write(1'b0, 2'd0, `R_W_SET, 16'd0);                 // stop: the angle holds
        repeat (180 * 4) @(posedge clk);
        reg_read(2'd0, `R_ENC_ANGLE, v2);
        enc_frame(16'hFFFF, v); enc_frame(16'hC000, v);       // read ANGLECOM, then NOP
        if (v[13:0] !== v2[15:2] || v[14] !== 1'b0) begin errors = errors + 1; $display("FAIL: AS5047P read %h, ENC_ANGLE %h", v, v2); end
        else $display("  ok AS5047P frame on the encoder pins: %h (ENC_ANGLE %h)", v, v2);
        reg_read(2'd0, `R_ENC_STATUS, v);
        if (v[15:8] != 8'd2 || v[2:0] != 3'd0) begin errors = errors + 1; $display("FAIL: ENC_STATUS %h", v); end

        reg_write(1'b0, 2'd0, `R_MODE, 16'h0000);
        if (errors == 0) $display("PASS tb_top"); else $display("FAIL tb_top: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
