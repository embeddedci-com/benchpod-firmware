// ============================================================================
// protect.v — the gateware layer of the board's protection (design doc: "second layer in gateware").
//
// All values are sinc codes (offset-corrected here): currents about 2048 / A, bus about 391 / V.
//
//   Overcurrent: each phase sample with |i| > OC_LIM for OC_COUNT samples in a row sets that
//     phase's sticky trip (281 kSPS per phase, about 4.7 us filter delay: OC_COUNT 1 trips within
//     about 8 us of the current crossing the limit). OC_LIM = 0: off.
//   Bus overvoltage brake (board 0): the brake switches fully on above OV_ON and off below OV_OFF
//     (hysteresis), a per-DUT clamp below the LM393's fixed 57 V backstop. OV_ON = 0: off.
//   Brake energy budget: a leaky bucket in uJ. Every us with the brake on it fills by
//     P = v^2 x BRK_G >> 24 (W, so BRK_G = 2^24 / (391^2 x R) for a resistance R), and it drains by
//     BRK_PAVG (W) every us. Above BRK_CAP x 4096 uJ the brake is inhibited (manual, overvoltage
//     and test alike) until the bucket is below half of that; the LM393 backstop still brakes in
//     hardware. BRK_CAP = 0: no budget.
//   Brake test: a write of BRK_TEST (us) turns the brake on for that long and records the bus
//     before (V0) and at the end (V1): BenchPod gets the fitted resistance from C x dV/dt.
//   Hot-swap interlock: the hot-swap may only be on with a per-DUT bus limit set (HS_LIMIT != 0);
//     four bus samples in a row above it while it is on set a sticky trip that drops it.
//
// Trips are sticky until cleared (`clear`, per bit, the TRIPS register): [2:0] overcurrent on
// phase A, B, C, [3] hot-swap bus limit. `inhibit` and `ov_brake` are live states, not trips.
// ============================================================================
`default_nettype none
module protect (
    input  wire        clk,
    input  wire        rst,
    input  wire        us,               // 1 us strobe
    input  wire        board0,
    // samples
    input  wire        v_any,            // one channel's new sample (v_ch says which)
    input  wire [2:0]  v_ch,
    input  wire signed [15:0] s_a, s_b, s_c, s_bus,
    input  wire signed [15:0] ofs_a, ofs_b, ofs_c, ofs_bus,
    // parameters
    input  wire [15:0] oc_lim,
    input  wire [3:0]  oc_count,
    input  wire [15:0] ov_on, ov_off,
    input  wire [15:0] brk_g, brk_pavg, brk_cap,
    input  wire [15:0] hs_limit,
    // brake requests
    input  wire        brake_man,        // manual brake PWM (already board-0 and enable gated)
    input  wire        test_start,       // one-clk strobe
    input  wire [15:0] test_len,         // us
    input  wire        hswap_on,         // the hot-swap is being driven on
    input  wire [3:0]  clear,            // one-clk, per trip bit
    // outputs
    output reg         brake,            // to the pad flop
    output reg  [3:0]  trips,
    output reg         inhibit,
    output reg         ov_brake,
    output wire        hswap_ok,
    output reg  [15:0] v_bus,            // the latest offset-corrected bus sample
    output reg  [15:0] test_v0, test_v1,
    output wire [15:0] energy            // the bucket in 4096 uJ units
);
    // ---- samples
    reg signed [16:0] iv;                 // offset-corrected current of the sample in hand
    reg        [1:0]  iph;
    reg               ivalid, bvalid;
    always @(posedge clk) begin
        ivalid <= 1'b0; bvalid <= 1'b0;
        if (v_any) begin
            case (v_ch)
                3'd0: begin iv <= s_a - ofs_a; iph <= 2'd0; ivalid <= 1'b1; end
                3'd1: begin iv <= s_b - ofs_b; iph <= 2'd1; ivalid <= 1'b1; end
                3'd2: begin iv <= s_c - ofs_c; iph <= 2'd2; ivalid <= 1'b1; end
                3'd4: begin v_bus <= sat16(s_bus - ofs_bus); bvalid <= 1'b1; end
                default: ;
            endcase
        end
    end
    function [15:0] sat16(input signed [16:0] x);
        sat16 = (x[16] != x[15]) ? (x[16] ? 16'h8000 : 16'h7FFF) : x[15:0];
    endfunction
    wire [16:0] iabs = iv[16] ? -iv : iv;
    wire signed [16:0] vb = {v_bus[15], v_bus};

    // ---- overcurrent
    reg [3:0] occ0, occ1, occ2;
    reg [2:0] oc_hit;
    always @(posedge clk) begin
        oc_hit <= 3'b000;
        if (rst) begin occ0 <= 4'd0; occ1 <= 4'd0; occ2 <= 4'd0; end
        else if (ivalid) begin : oc
            reg over;
            over = (oc_lim != 16'd0) && (iabs > {1'b0, oc_lim});
            case (iph)
                2'd0: begin occ0 <= over ? ((occ0 == 4'hF) ? occ0 : occ0 + 4'd1) : 4'd0; if (over && occ0 + 4'd1 >= oc_count) oc_hit[0] <= 1'b1; end
                2'd1: begin occ1 <= over ? ((occ1 == 4'hF) ? occ1 : occ1 + 4'd1) : 4'd0; if (over && occ1 + 4'd1 >= oc_count) oc_hit[1] <= 1'b1; end
                default: begin occ2 <= over ? ((occ2 == 4'hF) ? occ2 : occ2 + 4'd1) : 4'd0; if (over && occ2 + 4'd1 >= oc_count) oc_hit[2] <= 1'b1; end
            endcase
        end
    end

    // ---- hot-swap bus limit
    reg [2:0] hsc;
    reg       hs_hit;
    always @(posedge clk) begin
        hs_hit <= 1'b0;
        if (rst || !hswap_on) hsc <= 3'd0;
        else if (bvalid) begin
            if (hs_limit != 16'd0 && vb > $signed({1'b0, hs_limit})) begin
                hsc <= (hsc == 3'd7) ? hsc : hsc + 3'd1;
                if (hsc >= 3'd3) hs_hit <= 1'b1;
            end else hsc <= 3'd0;
        end
    end

    always @(posedge clk) begin
        if (rst) trips <= 4'd0;
        else trips <= (trips & ~clear) | {hs_hit, oc_hit};
    end
    assign hswap_ok = (hs_limit != 16'd0) && !trips[3] && (trips[2:0] == 3'd0);

    // ---- overvoltage brake (hysteresis)
    always @(posedge clk) begin
        if (rst || ov_on == 16'd0) ov_brake <= 1'b0;
        else if (bvalid) begin
            if (vb > $signed({1'b0, ov_on})) ov_brake <= 1'b1;
            else if (vb < $signed({1'b0, ov_off})) ov_brake <= 1'b0;
        end
    end

    // ---- brake test pulse
    reg [15:0] tcnt;
    always @(posedge clk) begin
        if (rst) tcnt <= 16'd0;
        else if (test_start) begin tcnt <= test_len; test_v0 <= v_bus; end
        else if (us && tcnt != 16'd0) begin
            tcnt <= tcnt - 16'd1;
            if (tcnt == 16'd1) test_v1 <= v_bus;
        end
    end

    // ---- energy budget (per us, pipelined: v^2, then x G)
    reg [31:0] vsq;
    reg [47:0] pw;
    reg [31:0] bucket;
    reg [1:0]  ust;
    wire [15:0] vpos = v_bus[15] ? 16'd0 : v_bus;
    always @(posedge clk) begin
        ust <= {ust[0], us};
        vsq <= vpos * vpos;
        pw  <= vsq * brk_g;
        if (rst) begin bucket <= 32'd0; inhibit <= 1'b0; end
        else begin
            if (ust[1]) begin : fill
                reg [32:0] up;
                up = {1'b0, bucket} + (brake ? {9'd0, pw[47:24]} : 33'd0);
                if (up[32]) up = 33'h0_FFFF_FFFF;
                bucket <= (up[31:0] > {16'd0, brk_pavg}) ? up[31:0] - {16'd0, brk_pavg} : 32'd0;
            end
            if (brk_cap == 16'd0) inhibit <= 1'b0;
            else if (bucket[31:12] >= {4'd0, brk_cap}) inhibit <= 1'b1;
            else if (bucket[31:12] < {5'd0, brk_cap[15:1]}) inhibit <= 1'b0;
        end
    end
    assign energy = (bucket[31:28] != 4'd0) ? 16'hFFFF : bucket[27:12];

    always @(posedge clk)
        brake <= board0 && !inhibit && !rst && (brake_man || ov_brake || tcnt != 16'd0);
endmodule
`default_nettype wire
