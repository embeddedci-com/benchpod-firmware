// ============================================================================
// pwm3.v — three-leg bridge PWM for the MP6539 (separate HS and LS inputs per leg).
//
// Duty is a Q0.16 fraction of the period: the time the phase node should sit at the bus. Each
// period the leg's high interval is centred in the period with one-clk resolution (180 steps at
// 200 kHz from 36 MHz); the fractional part is dithered with a first-order accumulator, so the
// average over a few periods has the full 16-bit resolution.
//
// Dead time is inserted as an on-delay: a gate turns on only once its leg's target level has been
// steady for `deadtime` clocks, so both gates are off for deadtime + 1 clk at every edge.
// That covers every transition, including 0 % <-> 100 % across a
// period boundary and enabling in the middle of a period. The MP6539 adds its own floor on top
// (both inputs low or both high = output off), so deadtime = 0 still never shoots through.
// Pulses shorter than `min_on` (after dead time) are dropped: duty snaps to 0 or 100 %.
//
// Safety: with `enable` low every gate is low (the MP6539 leaves the leg high-impedance) and the
// dead-time counters restart, so re-enabling waits a full dead time before any gate turns on.
// HS and LS of one leg are never high together (checked in tb_pwm3).
// New period, duties, dead time and min-on take effect at the start of the next period.
// ============================================================================
`default_nettype none

module pwm3 (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [9:0]  period,       // clocks per period (values below 32 are raised to 32)
    input  wire [5:0]  deadtime,     // clocks
    input  wire [5:0]  min_on,       // clocks
    input  wire [15:0] duty_a,
    input  wire [15:0] duty_b,
    input  wire [15:0] duty_c,
    output reg  [2:0]  hs,           // {C, B, A}
    output reg  [2:0]  ls,
    output reg         period_start, // strobe on the first clk of each period
    output reg         period_mid    // strobe at the centre of each period (sampling point)
);
    reg [9:0] per;          // period in use
    reg [5:0] dt;
    reg [9:0] cnt;

    wire [9:0] period_c = (period < 10'd32) ? 10'd32 : period;

    // per-leg high interval for the running period, the next period's, and the dither accumulators
    reg [9:0]  st_a, en_a, st_b, en_b, st_c, en_c;
    reg [9:0]  nst_a, nen_a, nst_b, nen_b, nst_c, nen_c;
    reg [15:0] acc_a, acc_b, acc_c, nacc_a, nacc_b, nacc_c;

    // Next period's intervals are prepared by one datapath, leg by leg (7 clk per leg), starting 24
    // clk before the wrap (the period is at least 32 clk). Compare values are registered once per
    // period, so the per-clk compares are cnt == const.
    reg [9:0] per_m1, per_m25, half_m1;
    reg       prep;
    always @(posedge clk) begin
        per_m1  <= per - 10'd1;
        per_m25 <= per - 10'd25;
        half_m1 <= (per >> 1) - 10'd1;
        prep    <= ~rst & (cnt == per_m25);          // registered: fires at cnt == per - 24
    end
    wire wrap = (cnt == per_m1);

    reg [9:0]  p_n;                                  // next period
    reg [5:0]  dt_n;
    reg [6:0]  lim_n;
    reg [1:0]  leg;
    reg [2:0]  stp;                                  // step within a leg, 7 = idle
    reg [15:0] duty_l, acc_l;
    reg [25:0] prod;
    reg [16:0] sum;
    reg [10:0] raw;
    reg [9:0]  cl, rem, d, nst;
    always @(posedge clk) begin
        if (rst) begin
            stp <= 3'd7; leg <= 2'd0;
            nst_a <= 10'd0; nen_a <= 10'd0; nst_b <= 10'd0; nen_b <= 10'd0; nst_c <= 10'd0; nen_c <= 10'd0;
            nacc_a <= 16'd0; nacc_b <= 16'd0; nacc_c <= 16'd0;
        end else begin
            if (prep) begin
                p_n   <= period_c;
                dt_n  <= deadtime;
                // the gate gap is deadtime + 1 clk (see below), so a kept pulse is at least min_on
                lim_n <= {1'b0, deadtime} + {1'b0, min_on} + 7'd1;
                leg <= 2'd0; stp <= 3'd0;
            end else if (stp != 3'd7) begin
                stp <= stp + 3'd1;
                case (stp)
                    3'd0: begin                          // select the leg's duty and dither state
                        duty_l <= (leg == 2'd0) ? duty_a : (leg == 2'd1) ? duty_b : duty_c;
                        acc_l  <= (leg == 2'd0) ? acc_a  : (leg == 2'd1) ? acc_b  : acc_c;
                    end
                    3'd1: prod <= duty_l * p_n;
                    3'd2: begin sum <= {1'b0, acc_l} + {1'b0, prod[15:0]}; raw <= {1'b0, prod[25:16]}; end
                    3'd3: begin                          // integer part + dither carry, clamp, remainder
                        raw <= raw + {10'd0, sum[16]};
                    end
                    3'd4: begin
                        cl  <= (raw > {1'b0, p_n}) ? p_n : raw[9:0];
                        rem <= (raw > {1'b0, p_n}) ? 10'd0 : p_n - raw[9:0];
                    end
                    3'd5: begin                          // snap slivers to 0 or 100 %, centred start
                        d   <= ({3'd0, cl} < lim_n) ? 10'd0 : ({3'd0, rem} < lim_n) ? p_n : cl;
                    end
                    default: begin                       // 6: start and end, store for the leg
                        case (leg)
                            2'd0: begin nst_a <= (p_n - d) >> 1; nen_a <= ((p_n - d) >> 1) + d; nacc_a <= sum[15:0]; end
                            2'd1: begin nst_b <= (p_n - d) >> 1; nen_b <= ((p_n - d) >> 1) + d; nacc_b <= sum[15:0]; end
                            default: begin nst_c <= (p_n - d) >> 1; nen_c <= ((p_n - d) >> 1) + d; nacc_c <= sum[15:0]; end
                        endcase
                        if (leg == 2'd2) stp <= 3'd7;
                        else begin leg <= leg + 2'd1; stp <= 3'd0; end
                    end
                endcase
            end
        end
    end

    always @(posedge clk) begin
        period_start <= 1'b0;
        period_mid   <= 1'b0;
        if (rst) begin
            cnt <= 10'd0; per <= 10'd180; dt <= 6'd0;
            acc_a <= 16'd0; acc_b <= 16'd0; acc_c <= 16'd0;
            st_a <= 10'd0; en_a <= 10'd0; st_b <= 10'd0; en_b <= 10'd0; st_c <= 10'd0; en_c <= 10'd0;
        end else if (wrap || cnt >= per) begin
            cnt <= 10'd0;
            period_start <= 1'b1;
            if (wrap) begin                          // the prep ran for this wrap
                per <= p_n; dt <= dt_n;
                acc_a <= nacc_a; acc_b <= nacc_b; acc_c <= nacc_c;
                st_a <= nst_a; en_a <= nen_a; st_b <= nst_b; en_b <= nen_b; st_c <= nst_c; en_c <= nen_c;
            end
        end else begin
            cnt <= cnt + 10'd1;
            if (cnt == half_m1) period_mid <= 1'b1;
        end
    end

    // target phase level per leg (1 = node at the bus)
    wire [2:0] want = {(cnt >= st_c) && (cnt < en_c),
                       (cnt >= st_b) && (cnt < en_b),
                       (cnt >= st_a) && (cnt < en_a)};

    // on-delay dead-time insertion
    reg [2:0] want_q;
    reg [5:0] hold_a, hold_b, hold_c;   // clocks the target has been steady (saturating at 63)
    always @(posedge clk) begin
        if (rst || !enable) begin
            want_q <= 3'b000;
            hold_a <= 6'd0; hold_b <= 6'd0; hold_c <= 6'd0;
            hs <= 3'b000; ls <= 3'b000;
        end else begin
            want_q <= want;
            hold_a <= (want[0] != want_q[0]) ? 6'd0 : (hold_a == 6'd63 ? hold_a : hold_a + 6'd1);
            hold_b <= (want[1] != want_q[1]) ? 6'd0 : (hold_b == 6'd63 ? hold_b : hold_b + 6'd1);
            hold_c <= (want[2] != want_q[2]) ? 6'd0 : (hold_c == 6'd63 ? hold_c : hold_c + 6'd1);
            // a gate turns on only once the target equals it on this clk and has been steady
            // for dt clocks; a change of target turns the conducting gate off on the same clk
            hs[0] <=  want[0] &  want_q[0] & (hold_a >= dt);
            ls[0] <= ~want[0] & ~want_q[0] & (hold_a >= dt);
            hs[1] <=  want[1] &  want_q[1] & (hold_b >= dt);
            ls[1] <= ~want[1] & ~want_q[1] & (hold_b >= dt);
            hs[2] <=  want[2] &  want_q[2] & (hold_c >= dt);
            ls[2] <= ~want[2] & ~want_q[2] & (hold_c >= dt);
        end
    end
endmodule
`default_nettype wire
