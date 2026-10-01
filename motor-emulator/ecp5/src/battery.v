// ============================================================================
// battery.v — battery model: the bus voltage a pack would show, from the measured DUT battery
// current, for the PV board's setpoint (design doc: Battery setpoint).
//
// All in sinc codes: current i (DUT battery channel, offset-corrected, positive = discharge after
// the optional sign flip), voltages in bus codes (about 391 / V). BenchPod converts.
//
//   every sinc set (281.25 kSPS):
//     setpoint = clamp(OCV - (i x R0 >>> 15) - v_rc, VMIN, VMAX)
//   every ms:
//     charge  += sum of i over the ms (code x samples; about 2048 x 281250 x 3600 per Ah)
//     i_ms     = that sum x 233 >>> 16 (the mean; 233 / 65536 = 1 / 281.3)
//     v_rc    += ((i_ms x R1 >>> 15) - v_rc) x ALPHA >>> 20       (ALPHA = 1 ms / tau x 2^20)
//     OCV      = table[p >> 8] + (table[p >> 8 + 1] - table[p >> 8]) x (p & 255) >>> 8,
//                p = charge >>> (QSHIFT - 8), clamped to 0 .. 63 x 256: 64 points of OCV over the
//                charge drawn, full at entry 0, BenchPod picks QSHIFT for the pack's capacity
//   POS (p) reads back the position in the table; a write sets the charge to POS << (QSHIFT - 8).
//
// PV_SET (the board's DNP option path): first-order sigma-delta of
//   duty = clamp(PV_OFS + (setpoint x PV_GAIN >>> 15), 0, 65535)  (fraction of time high)
// out of a flop; high (released: the divider's park level) unless enabled, and on `park`.
// ============================================================================
`default_nettype none
module battery (
    input  wire        clk,
    input  wire        rst,
    input  wire        set_tick,         // a new sinc sample set
    input  wire        ms,               // 1 ms strobe
    input  wire signed [15:0] s_i,       // DUT battery current channel
    input  wire signed [15:0] ofs_i,
    // parameters
    input  wire        run,
    input  wire        inv,
    input  wire [5:0]  qshift,
    input  wire signed [15:0] r0, r1,
    input  wire [15:0] alpha,
    input  wire signed [15:0] vmin, vmax,
    input  wire        tbl_we,
    input  wire [5:0]  tbl_addr,
    input  wire [15:0] tbl_data,
    input  wire        pos_we,
    input  wire [15:0] pos_data,
    input  wire        pv_en,
    input  wire        park,
    input  wire signed [15:0] pv_ofs, pv_gain,
    // outputs
    output reg  signed [15:0] setpoint,
    output reg  signed [15:0] i_ms,
    output reg  signed [15:0] ocv,
    output reg  [15:0] pos,
    output reg         pv_pin
);
    // ---- OCV table (block RAM)
    reg [15:0] tbl [0:63];
    integer k;
    initial for (k = 0; k < 64; k = k + 1) tbl[k] = 16'h0000;
    reg  [5:0]  t_ra;
    reg  [15:0] t_q;
    always @(posedge clk) begin
        if (tbl_we) tbl[tbl_addr] <= tbl_data;
        t_q <= tbl[t_ra];
    end

    wire [5:0] qs = (qshift < 6'd8) ? 6'd8 : qshift;
    function signed [15:0] sat16(input signed [47:0] x);
        sat16 = (x > 48'sd32767) ? 16'sh7FFF : (x < -48'sd32768) ? 16'sh8000 : x[15:0];
    endfunction

    // ---- per set: current, sag, setpoint
    reg signed [16:0] i_raw;
    reg signed [15:0] i_cur;
    reg signed [31:0] i_sum;
    reg signed [31:0] r0p;
    reg signed [47:0] vrc;               // v_rc, Q16
    reg [2:0]  ps;
    reg        ms_go;                    // the ms update took i_sum (set below)
    always @(posedge clk) begin
        if (rst) begin
            ps <= 3'd0; i_sum <= 32'sd0; setpoint <= 16'sd0;
        end else begin
            case (ps)
                3'd0: if (set_tick) begin i_raw <= s_i - ofs_i; ps <= 3'd1; end
                3'd1: begin i_cur <= sat16(inv ? -i_raw : i_raw); ps <= 3'd2; end
                3'd2: begin r0p <= i_cur * r0; i_sum <= i_sum + (run ? i_cur : 16'sd0); ps <= 3'd3; end
                default: begin : sp
                    reg signed [17:0] a, b, c;
                    a = ocv; b = sat16(r0p >>> 15); c = sat16(vrc >>> 16);
                    setpoint <= clamp(a - b - c);
                    ps <= 3'd0;
                end
            endcase
            if (ms_go) i_sum <= 32'sd0;         // the ms update took the sum (below)
        end
    end
    function signed [15:0] clamp(input signed [17:0] v);
        reg signed [17:0] hi, lo;
        begin
            hi = vmax; lo = vmin;
            clamp = (v > hi) ? vmax : (v < lo) ? vmin : v[15:0];
        end
    endfunction

    // ---- per ms: charge, RC branch, OCV (sequential; the set pipeline keeps running)
    reg signed [47:0] charge;
    reg signed [31:0] sum_q;
    reg signed [47:0] im_p, rc_t;
    reg signed [31:0] rc_d;
    function signed [31:0] sat32(input signed [47:0] x);
        sat32 = (x > 48'sh0000_7FFF_FFFF) ? 32'sh7FFF_FFFF : (x < -48'sh0000_8000_0000) ? 32'sh8000_0000 : x[31:0];
    endfunction
    reg signed [48:0] rc_p;
    reg signed [47:0] p_full;
    reg [3:0]  ms_st;
    reg [13:0] p14;
    reg signed [16:0] t0, t1;
    reg signed [25:0] dt;
    always @(posedge clk) begin
        ms_go <= 1'b0;
        if (rst) begin
            charge <= 48'sd0; vrc <= 48'sd0; ms_st <= 4'd0; ocv <= 16'sd0; i_ms <= 16'sd0; pos <= 16'd0;
        end else begin
            if (pos_we) charge <= $signed({32'd0, pos_data}) <<< (qs - 6'd8);
            case (ms_st)
                4'd0: if (ms && ps == 3'd0 && !set_tick) begin sum_q <= i_sum; ms_go <= 1'b1; ms_st <= 4'd1; end
                4'd1: begin
                    if (!pos_we && run) charge <= charge + sum_q;
                    // x 233 = 256 - 16 - 4 - 2 - 1, in adders (no multiplier)
                    im_p <= ($signed({{16{sum_q[31]}}, sum_q}) <<< 8) - ($signed({{16{sum_q[31]}}, sum_q}) <<< 4)
                          - ($signed({{16{sum_q[31]}}, sum_q}) <<< 2) - ($signed({{16{sum_q[31]}}, sum_q}) <<< 1)
                          - $signed({{16{sum_q[31]}}, sum_q});
                    ms_st <= 4'd2;
                end
                4'd2: begin i_ms <= sat16(im_p >>> 16); ms_st <= 4'd3; end
                4'd3: begin rc_t <= ((i_ms * r1) >>> 15) <<< 16; ms_st <= 4'd4; end
                4'd4: begin rc_d <= sat32(rc_t - vrc); ms_st <= 4'd5; end      // 32 bits: two multipliers
                4'd5: begin rc_p <= rc_d * $signed({1'b0, alpha}); ms_st <= 4'd6; end
                4'd6: begin if (run) vrc <= vrc + (rc_p >>> 20); p_full <= charge >>> (qs - 6'd8); ms_st <= 4'd7; end
                4'd7: begin
                    p14 <= (p_full < 0) ? 14'd0 : (p_full > 48'sd16128) ? 14'd16128 : p_full[13:0];
                    pos <= (p_full < 0) ? 16'd0 : (p_full > 48'sd65535) ? 16'hFFFF : p_full[15:0];
                    ms_st <= 4'd8;
                end
                4'd8: begin t_ra <= p14[13:8]; ms_st <= 4'd9; end
                4'd9: begin t_ra <= (p14[13:8] == 6'd63) ? 6'd63 : p14[13:8] + 6'd1; ms_st <= 4'd10; end
                4'd10: begin t0 <= $signed({1'b0, t_q}); ms_st <= 4'd11; end        // table[idx]
                4'd11: begin t1 <= $signed({1'b0, t_q}); ms_st <= 4'd12; end        // table[idx + 1]
                4'd12: begin dt <= (t1 - t0) * $signed({1'b0, p14[7:0]}); ms_st <= 4'd13; end
                default: begin ocv <= sat16(t0 + (dt >>> 8)); ms_st <= 4'd0; end
            endcase
        end
    end

    // ---- PV_SET sigma-delta
    reg signed [47:0] pv_p;
    reg [15:0] duty;
    reg [16:0] sd;
    always @(posedge clk) begin
        pv_p <= setpoint * pv_gain;
        begin : dmap
            reg signed [47:0] d;
            d = $signed({{32{pv_ofs[15]}}, pv_ofs}) + (pv_p >>> 15);
            duty <= (d < 0) ? 16'd0 : (d > 48'sd65535) ? 16'hFFFF : d[15:0];
        end
        sd <= rst ? 17'd0 : {1'b0, sd[15:0]} + {1'b0, duty};
        pv_pin <= (rst || !pv_en || park) ? 1'b1 : sd[16];
    end
endmodule
`default_nettype wire
