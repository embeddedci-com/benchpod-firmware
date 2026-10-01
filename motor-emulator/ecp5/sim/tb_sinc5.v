// tb_sinc5 — five sinc3 channels with a shared comb, against five second-order delta-sigma models.
// Run 1 and run 2 (new values on every channel, no reset) check each channel's mean, so a comb
// state mixed up between channels, or a stale RAM word, shows. Also checks the per-channel rate
// (one output per OSR modulator bits) and the channel order. SINC_L picks the OSR (6: 64, 8: 256).
`timescale 1ns/1ps
`default_nettype none
`ifndef SINC_L
`define SINC_L 6
`endif
module tb_sinc5;
    localparam L = `SINC_L, PER = 2 << L;    // clk per output and channel (2 clk per modulator bit)
    reg clk = 0;
    always #13.889 clk = ~clk;
    reg rst = 1, bit_en = 0;
    reg [4:0] din = 0;
    wire signed [15:0] d0, d1, d2, d3, d4;
    wire valid, set_done;
    wire [2:0] valid_ch;
    sinc5 #(.L(L)) dut (.clk(clk), .rst(rst), .bit_en(bit_en), .din(din), .d0(d0), .d1(d1), .d2(d2), .d3(d3), .d4(d4),
               .valid(valid), .valid_ch(valid_ch), .set_done(set_done));

    real x [0:4];
    real i1 [0:4];
    real i2 [0:4];
    integer k;
    initial for (k = 0; k < 5; k = k + 1) begin i1[k] = 0; i2[k] = 0; end
    always @(posedge clk) begin : mod
        real fb; integer j;
        bit_en <= ~bit_en & ~rst;
        if (bit_en) for (j = 0; j < 5; j = j + 1) begin
            fb = din[j] ? 1.0 : -1.0;
            i1[j] = i1[j] + x[j] - fb;
            i2[j] = i2[j] + i1[j] - fb;
            din[j] <= (i2[j] >= 0.0);
        end
    end

    integer errors = 0, n, gap, last_ch;
    real sum [0:4];
    task run_and_check(input real v0, v1, v2, v3, v4);
        integer j;
        real got, want;
        begin
            x[0] = v0; x[1] = v1; x[2] = v2; x[3] = v3; x[4] = v4;
            n = 0; while (n < 5) begin @(posedge clk); if (set_done) n = n + 1; end
            for (j = 0; j < 5; j = j + 1) sum[j] = 0;
            n = 0;
            while (n < 8) begin
                @(posedge clk);
                if (set_done) begin
                    sum[0] = sum[0] + d0; sum[1] = sum[1] + d1; sum[2] = sum[2] + d2;
                    sum[3] = sum[3] + d3; sum[4] = sum[4] + d4; n = n + 1;
                end
            end
            for (j = 0; j < 5; j = j + 1) begin
                got = sum[j] / 8.0; want = x[j] * 32768.0;
                if (got < want - 40 || got > want + 40) begin
                    errors = errors + 1; $display("FAIL: ch %0d mean %0.1f, expected %0.1f", j, got, want);
                end else $display("  ok ch %0d mean %0.1f (target %0.1f)", j, got, want);
            end
        end
    endtask

    // rate and order: each channel every 128 clk, channels in order 0..4
    integer last_t [0:4];
    integer bad_rate = 0, bad_order = 0;
    initial for (k = 0; k < 5; k = k + 1) last_t[k] = -1;
    integer clkn = 0;
    always @(posedge clk) begin
        clkn = clkn + 1;
        if (valid) begin
            if (last_t[valid_ch] >= 0 && clkn - last_t[valid_ch] != PER) bad_rate = bad_rate + 1;
            if (last_ch >= 0 && valid_ch != (last_ch + 1) % 5) bad_order = bad_order + 1;
            last_t[valid_ch] = clkn; last_ch = valid_ch;
        end
    end

    initial begin
        #40_000_000 $display("FAIL tb_sinc5: timeout"); $finish;
    end
    initial begin
        last_ch = -1;
        x[0] = 0; x[1] = 0; x[2] = 0; x[3] = 0; x[4] = 0;
        repeat (4) @(posedge clk); rst = 0;
        run_and_check(0.25, -0.5, 0.0, 0.1, 0.6);
        run_and_check(-0.7, 0.3, 0.95, -0.05, 0.2);      // second run, every channel changed
        if (bad_rate) begin errors = errors + 1; $display("FAIL: %0d outputs off the %0d-clk rate", bad_rate, PER); end
        if (bad_order) begin errors = errors + 1; $display("FAIL: %0d outputs out of channel order", bad_order); end
        if (errors == 0) $display("PASS tb_sinc5 (OSR %0d)", 1 << L); else $display("FAIL tb_sinc5: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
