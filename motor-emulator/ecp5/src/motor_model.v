// ============================================================================
// motor_model.v — PMSM model, common-mode loop and dead-time compensation, once per PWM period,
// as a small microcoded engine (program: model.masm, assembled by tools/masm.py).
//
// The emulator bridge is a voltage source: each leg outputs the modelled back-EMF of its phase
// (relative to a virtual star), plus dead-time compensation, plus a common-mode offset (min-max
// injection + the zero-sequence PI loop). Leg voltages are divided by the measured bus voltage to
// give the duties for pwm3. The DUT's current through the dummy inductors is measured, not
// modelled: it drives the torque, and so the mechanics.
//
// Units (the gateware is unit-agnostic; BenchPod's firmware converts physical values):
//   current  i : sinc code of the phase shunts (2048 / A nominal), minus I_OFS. Positive = out of
//                the emulator leg towards the DUT; the motor current is its negative.
//   voltage  v : sinc code of the bus channel (about 391 / V nominal), minus VBUS_OFS
//   angle    th: 32-bit electrical angle, 2^32 = one electrical revolution
//   speed    w32: signed Q16.16; th += w32 >>> (16 - WSHIFT) per tick; w16 = w32 >>> 16
//
// Per tick (16 x 16 signed products; sat16 = saturate to 16 bits):
//   E      = sat16(w16 * KE >>> 15)                back-EMF amplitude (KE, DAMP limited to 32767)
//   e_x    = sat16(E * sin(lk_x) >>> 15)           lk = advanced angle (16 bits), lk_b = lk - 0x5555,
//                                                   lk_c = lk + 0x5555; sine index = angle[15:6]
//   iq     = sat16(sum(-i_x * sin(th_x)) >>> 15)   current in phase with the back-EMF
//   T      = sat16(KT * iq >>> 15);  net = sat16(T - LOAD - (DAMP * w16 >>> 15))
//   w32   += net * INVJ >>> JSHIFT                 (MODE 2 only; MODE 1 holds w32 = W_SET << 16)
//   i0     = sat16((ia + ib + ic) / 4 * 21845 >>> 14)                         (= sum / 3)
//   v_cm   = -sat16((CM_KP * i0 >>> 8) + (cm_int >>> 16)),  cm_int += CM_KI * i0  (clamped)
//   dt_x   = sign(i_x) * sat16(vbus * (DT_FRAC >> 1) >>> 15), 0 inside +-DT_IDB  (DT comp)
//   u_x    = sat16(e_x + dt_x);  v_x = u_x - (max u + min u) / 2 + v_cm, clamped to +-vbus / 2
//   duty_x = 0x8000 + (v_x << 16) / vbus;  below 512 codes of bus: 0x8000 (parked)
//
// Engine: 4-stage pipeline, one instruction per clk (fetch, operand read from two block-RAM copies
// of the data memory, multiply, execute/store) with a 40-bit accumulator. No interlocks: the
// assembler spaces dependent instructions. Parameters are written into the data memory by the
// link (a pending write waits for a clk without an engine store). A tick takes about 146 clk, so
// the PWM period must be at least 180 clk in model modes (top.v enforces it: pwm3 reads the
// duties from 24 clk before the period end).
// ============================================================================
`default_nettype none

module motor_model (
    input  wire               clk,
    input  wire               rst,
    input  wire               tick,            // PWM period start
    input  wire [1:0]         mode,            // 0 off, 1 fixed speed, 2 dynamic (mechanics)
    input  wire               cm_en,
    input  wire               dt_en,
    input  wire               reset_state,     // one-clk strobe: th = 0, w32 = W_SET << 16, cm_int = 0
    input  wire signed [15:0] sinc_a, sinc_b, sinc_c, sinc_bus,
    // parameter writes: pw_addr = link register - 0x20 (see PROTOCOL.md)
    input  wire               pw_en,
    input  wire [4:0]         pw_addr,
    input  wire [15:0]        pw_data,
    // results
    output reg  [15:0]        duty_a, duty_b, duty_c,
    output reg  [31:0]        theta,
    output reg  signed [31:0] w32,
    output reg  signed [15:0] e_amp,
    output reg  signed [15:0] i0,
    output reg  signed [15:0] v_cm,
    output reg  signed [15:0] iq,
    output reg  [2:0]         hall,            // {C, B, A}: 1 while sin(th_x + HALL_OFS) >= 0
    output reg  [15:0]        count,
    output wire               busy
);
    // opcodes (tools/masm.py)
    localparam OP_NOP = 5'd0, OP_LD = 5'd1, OP_LDN = 5'd2, OP_ADDA = 5'd3, OP_SUBA = 5'd4, OP_MUL = 5'd5,
               OP_MAC = 5'd6, OP_SHR = 5'd7, OP_NEG = 5'd8, OP_MAX = 5'd9, OP_MIN = 5'd10, OP_MAX0 = 5'd11,
               OP_CLAMP = 5'd12, OP_STO = 5'd13, OP_DTC = 5'd14, OP_W32LD = 5'd15, OP_W32ADD = 5'd16,
               OP_CMADD = 5'd17, OP_CMZ = 5'd18, OP_DIV = 5'd19, OP_WAIT = 5'd20, OP_THADV = 5'd21,
               OP_END = 5'd22, OP_ACCW = 5'd24, OP_CMPS = 5'd25, OP_CMPN = 5'd26, OP_SELA = 5'd27,
               OP_SELNA = 5'd28, OP_SELNOT = 5'd29, OP_RCP = 5'd30, OP_DTS = 5'd23;
    // exported data addresses (model.masm DATA / TEMPS)
    localparam [7:0] A_E = 8'h49, A_I0 = 8'h4A, A_IQ = 8'h4F, A_VCM = 8'h53;
    localparam [15:0] T16 = 16'h5555;               // 1/3 revolution in 16-bit angle units

    function [15:0] duty_of;                          // 0x8000 +- q, saturated to 0 .. 0xFFFF
        input [22:0] q; input neg;
        duty_of = neg ? ((q >= 23'h8000) ? 16'h0000 : 16'h8000 - q[15:0])
                      : ((q >= 23'h8000) ? 16'hFFFF : 16'h8000 + q[15:0]);
    endfunction

    // ---------------------------------------------------------------- parameter write arbitration
    reg        pend;
    reg  [4:0] pend_addr;
    reg [15:0] pend_data;

    // ---------------------------------------------------------------- fabric parameters
    reg signed [15:0] w_set;
    reg [4:0]  wshift, jshift;
    reg [15:0] dt_idb, hall_ofs;
    reg [2:0]  adv;                                    // back-EMF angle advance, half ticks (0-3.5)

    // ---------------------------------------------------------------- program ROM
    reg  [31:0] prog [0:255];
    initial $readmemh("src/model_prog.hex", prog);
    reg  [7:0]  pc;
    reg  [31:0] prog_q;
    reg  [7:0]  fetch_addr;
    reg         fetch_en;
    always @(posedge clk) if (fetch_en) prog_q <= prog[fetch_addr];
    reg         running, r_valid;
    reg  [5:0]  hold;
    wire [31:0] ir_r = r_valid ? prog_q : 32'd0;
    wire [4:0]  r_op = ir_r[31:27];

    // ---------------------------------------------------------------- data RAM, two read ports
    reg  [15:0] dma [0:255];
    reg  [15:0] dmb [0:255];
    initial $readmemh("src/model_data.hex", dma);
    initial $readmemh("src/model_data.hex", dmb);
    // the store writes straight from the execute stage (a value stored by instruction i is read
    // back by instruction i + 3, the spacing tools/masm.py enforces); a parameter write from the
    // link takes a clk without a store
    wire        y_sto;
    wire        pw_go;
    wire [7:0]  wa;
    wire [15:0] wd;
    wire        we = y_sto | pw_go;
    reg  [15:0] qa, qb;
    always @(posedge clk) begin
        if (we) begin dma[wa] <= wd; dmb[wa] <= wd; end
        qa <= dma[ir_r[23:16]];
        qb <= dmb[ir_r[15:8]];
    end

    // ---------------------------------------------------------------- sine ROM (per tick: 3 reads)
    reg  [15:0] sine [0:1023];
    initial $readmemh("src/sine1024.hex", sine);
    reg  [9:0]  saddr;
    reg  signed [15:0] sq, s_a, s_b, s_c, r_a, r_b, r_c;
    always @(posedge clk) sq <= sine[saddr];
    // back-EMF angle (16 bits): advanced by ADV half ticks to cover the hold of the output for one
    // PWM period (the voltage computed at a tick is applied during the next period)
    reg  [15:0] th_look, adv_q;
    reg         adv_fix;
    // Per tick one 16-bit adder walks the angles: advanced a, b, c (back-EMF sines), a, b, c (torque
    // sines), then the halls (angle + HALL_OFS, - 1/3, + 1/3). sstep: 1..10, 0 = idle.
    reg  [3:0]  sstep;
    reg  [15:0] s_base, s_off, h_mt, h_pt;
    wire [15:0] s_sum = s_base + s_off;

    // ---------------------------------------------------------------- pipeline registers
    reg  [31:0] ir_x;
    reg         y_busy;                                 // an instruction in Y (for busy)
    reg  signed [15:0] a_y, b_y;
    reg  signed [31:0] p_y;
    // 34 bits: the largest value is a sum of three 16 x 16 products (< 2^32), so nothing overflows
    localparam AW = 34;
    reg  signed [AW-1:0] acc;
    // a second copy of acc for the shifters, STO and the CM add, so acc itself only feeds the adders
    // (keep: otherwise yosys merges the two)
    (* keep *) reg signed [AW-1:0] acc_c;
    reg  signed [31:0] cm_int;

    // A operand: data RAM, or a port for addresses 0xE0-0xE9. The port is picked one stage early
    // (R, registered), so only one select sits between the RAM output and the multiplier. Ports
    // change only between ticks or behind the assembler's 3-instruction spacing.
    reg  signed [15:0] port_q;
    reg                is_port;
    always @(posedge clk) begin
        is_port <= (prog_q[23:20] == 4'hE);            // ungated: a dropped slot becomes a NOP anyway
        case (prog_q[19:16])
            4'h0: port_q <= sinc_a;   4'h1: port_q <= sinc_b;   4'h2: port_q <= sinc_c;   4'h3: port_q <= sinc_bus;
            4'h4: port_q <= s_a;      4'h5: port_q <= s_b;      4'h6: port_q <= s_c;
            4'h7: port_q <= w32[31:16];
            4'h8: port_q <= cm_int[31:16];
            4'hA: port_q <= r_a;      4'hB: port_q <= r_b;      4'hC: port_q <= r_c;
            default: port_q <= 16'sd0;
        endcase
    end
    wire signed [15:0] a_x = is_port ? port_q : qa;
    wire signed [15:0] b_x = qb;
    reg  signed [15:0] nidb;                           // -DT_IDB, registered
    reg                dt_pos, dt_neg;                 // DTS: the current against the band, held for DTC
    wire signed [31:0] p_x = a_x * b_x;

    // ---------------------------------------------------------------- execute (Y stage)
    // decode in the X stage, registered: the Y stage starts from flops
    wire [4:0]  x_op   = ir_x[31:27];
    wire [2:0]  x_cond = ir_x[26:24];
    reg         x_go;
    always @(*) begin
        case (x_cond)
            3'd1: x_go = (mode == 2'd1);
            3'd2: x_go = (mode == 2'd2);
            3'd3: x_go = cm_en;
            3'd4: x_go = ~cm_en;
            3'd5: x_go = dt_en;
            default: x_go = 1'b1;
        endcase
    end
    reg         xd_acc, xd_inv;
    reg  [1:0]  xd_ysel;                                // 0 A, 1 P, 2 B, 3 acc
    always @(*) begin
        xd_acc = 1'b1; xd_inv = 1'b0; xd_ysel = 2'd0;
        case (x_op)
            OP_LD:   xd_acc = 1'b0;
            OP_LDN:  begin xd_acc = 1'b0; xd_inv = 1'b1; end
            OP_SUBA, OP_CMPS: xd_inv = 1'b1;                // acc - a
            OP_MUL:  begin xd_acc = 1'b0; xd_ysel = 2'd1; end
            OP_MAC:  xd_ysel = 2'd1;
            OP_DTC:  begin xd_ysel = 2'd2; xd_inv = dt_neg; end   // acc -/+ b by the sign DTS latched
            default: ;                                     // ADDA, CMPN: acc + a
        endcase
    end
    reg  [4:0]  y_op;
    reg         y_go, x_acc, inv;
    // one-hot accumulator updates, decoded (with the condition) in X
    reg         f_sum, f_shr, f_accw, f_max0, f_dtc, f_cmp, f_sela, f_selna, f_selnot;
    reg         cflag;                                  // compare result (sign of the adder)
    reg  [2:0]  f_div;                                  // DIV: duty for leg 0/1/2
    reg         f_rcp;                                  // RCP: start 2^30 / vbus
    reg  signed [AW-1:0] yval;                          // the adder's operand, selected in X
    reg  signed [16:0]   na_y;                          // -a, computed in X (17 bits: -(-32768))
    reg  signed [16:0]   cmp_y;                         // compare operand: ~a (CMPS, + 1 below) or a (CMPN)
    reg                  cmp_ci;
    reg  [1:0]  ysel;
    reg  [4:0]  shamt;                                  // W32ADD / THADV (runtime shifts)
    reg  [2:0]  scode;                                  // STO / SHR: 0,1,2,8,14,15
    reg  [7:0]  y_d;
    always @(posedge clk) begin
        y_op  <= x_op;
        y_go  <= x_go & ~rst;
        na_y  <= -{a_x[15], a_x};
        cmp_y  <= (x_op == OP_CMPS) ? ~{a_x[15], a_x} : {a_x[15], a_x};
        cmp_ci <= (x_op == OP_CMPS);
        f_div <= (x_op == OP_DIV) ? (3'b001 << ir_x[1:0]) : 3'b000;
        f_rcp <= (x_op == OP_RCP);
        // inverted here for subtracts (never a product), so only the carry-in waits on inv in Y
        yval  <= (xd_ysel == 2'd1) ? {{(AW-32){p_x[31]}}, p_x} :
                 {AW{xd_inv}} ^ ((xd_ysel == 2'd2) ? {{(AW-16){b_x[15]}}, b_x} : {{(AW-16){a_x[15]}}, a_x});
        f_sum  <= x_go & (x_op == OP_LD || x_op == OP_LDN || x_op == OP_ADDA || x_op == OP_SUBA ||
                          x_op == OP_MUL || x_op == OP_MAC);
        f_shr  <= x_go & (x_op == OP_SHR);
        f_accw <= x_go & (x_op == OP_ACCW);
        f_cmp    <= x_go & (x_op == OP_CMPS || x_op == OP_CMPN);
        f_sela   <= x_go & (x_op == OP_SELA);
        f_selna  <= x_go & (x_op == OP_SELNA);
        f_selnot <= x_go & (x_op == OP_SELNOT);
        f_max0 <= x_go & (x_op == OP_MAX0);
        f_dtc  <= x_go & (x_op == OP_DTC);
        x_acc <= xd_acc; inv <= xd_inv; ysel <= xd_ysel;
        y_d   <= ir_x[7:0];
        // STO / SHR: amount in the a field; W32ADD: JSHIFT; THADV: 16 - WSHIFT
        shamt <= (x_op == OP_W32ADD) ? jshift : (5'd16 - wshift);
        scode <= ir_x[18:16];
    end
    wire signed [AW-1:0] a_ext = {{(AW-16){a_y[15]}}, a_y};
    wire signed [AW-1:0] b_ext = {{(AW-16){b_y[15]}}, b_y};
    wire signed [AW-1:0] p_ext = {{(AW-32){p_y[31]}}, p_y};
    wire signed [AW-1:0] na_ext = {{(AW-17){na_y[16]}}, na_y};

    // W32ADD / THADV: acc >>> shamt (acc fits 32 bits there) on two DSPs instead of a barrel
    // shifter: y = acc, or acc >>> 16 when shamt >= 16, then y >>> s = (y x 2^(15 - s)) >> 15 for
    // s = shamt mod 16, as (hi16 x P) << 1 + (lo16 x P) >> 15. Pipelined: sh_q 3 clk after Y.
    // Both products unsigned (16 x 16 fits a DSP); a negative hi16 is corrected afterwards by
    // subtracting P << 17 (mod 2^32).
    reg  [31:0]        acc_dl;              // acc a clk later: the shifter's input stays away from the adder
    reg  [4:0]         shamt_d;
    reg  [31:0]        sh_y;
    reg  [15:0]        sh_p, sh_p2;
    reg  [31:0]        sh_ph, sh_pl;
    reg                sh_v0, sh_v1, sh_v2, sh_th, sh_neg;

    // STO / SHR: a fixed six-way shift, saturation flags straight from acc (no compare chains)
    reg  signed [AW-1:0] acc_s;
    always @(*) begin
        case (scode)
            3'd1: acc_s = acc_c >>> 1;   3'd2: acc_s = acc_c >>> 2;   3'd3: acc_s = acc_c >>> 8;
            3'd4: acc_s = acc_c >>> 14;  3'd5: acc_s = acc_c >>> 15;  default: acc_s = acc_c;
        endcase
    end
    function in16;                                      // bits [AW-1:15+s] all equal to the sign
        input [AW-1:0] v; input integer s;
        integer k;
        reg ones, zeros;
        begin
            ones = 1'b1; zeros = 1'b1;
            for (k = 15; k < AW; k = k + 1) if (k >= 15 + s) begin ones = ones & v[k]; zeros = zeros & ~v[k]; end
            in16 = ones | zeros;
        end
    endfunction
    // STO shifts by 0 or 15 only (tools/masm.py turns other shifts into SHR n + STO 0)
    wire               sto_ok = scode[0] ? in16(acc_c, 15) : in16(acc_c, 0);
    wire signed [15:0] sto_lo = scode[0] ? acc_c[30:15] : acc_c[15:0];
    wire signed [15:0] sto_v  = sto_ok ? sto_lo : (acc_c[AW-1] ? -16'sd32768 : 16'sd32767);

    wire signed [AW-1:0] y_raw = yval;                  // arrives inverted when inv is set
    wire               inv_e = inv;
    wire signed [AW-1:0] acc_other = f_shr ? acc_s : f_accw ? {{(AW-32){w32[31]}}, w32} :
                                     f_max0 ? {AW{1'b0}} : f_selna ? na_ext : a_ext;   // SELA / SELNOT
    wire signed [AW-1:0] sum   = (x_acc ? acc : {AW{1'b0}}) + y_raw + {{(AW-1){1'b0}}, inv_e};
    // compares get their own adder straight from flops (no operand muxes ahead of the carry chain)
    wire signed [AW-1:0] cmp_sum = acc + {{(AW-17){cmp_y[16]}}, cmp_y} + {{(AW-1){1'b0}}, cmp_ci};
    wire               acc_we = f_sum | f_shr | f_accw | (f_max0 & acc[AW-1]) | ((f_sela | f_selna) & cflag) |
                                (f_selnot & ~cflag) | (f_dtc & (dt_pos | dt_neg));
    wire signed [AW-1:0] acc_d = (f_sum | f_dtc) ? sum : acc_other;
    wire signed [32:0] cm_sum = $signed({cm_int[31], cm_int}) + $signed({acc_c[31], acc_c[31:0]});
    reg  signed [32:0] cm_raw;
    reg  [31:0]        sh_q;                           // shifted acc for W32ADD / THADV, added next clk
    wire signed [19:0] adv_p = $signed(sh_q[31:16]) * $signed({1'b0, adv});   // a DSP: step x ADV
    reg                w_fix, th_fix, w_hi, th_hi;
    reg                f_c;                            // carry between the halves (W32ADD, THADV never overlap)
    reg                cm_fix;                         // clamp cm_raw into cm_int on the next clk
    assign y_sto = y_go && (y_op == OP_STO) && !rst;
    assign pw_go = pend && !pw_en && !y_sto && !rst;
    assign wa = y_sto ? y_d : {3'b000, pend_addr};
    assign wd = y_sto ? sto_v : pend_data;


    // ---------------------------------------------------------------- dividers (3, in parallel)
    // q = (|v| << 16) / vb for |v| <= vb / 2: 16 restoring steps
    // duties: one reciprocal per tick, R = floor(2^30 / vbus) (RCP, 31 restoring steps), then per
    // leg q = (|v| x R) >> 14 on the DSPs and duty = 0x8000 +- q (DIV, 3 clk, pipelined)
    reg  [15:0] rdv;                                 // divisor (vbus)
    reg  [16:0] rrem;
    reg  [20:0] rq;                                  // R (up to 2^21 for vbus >= 512)
    reg  [30:0] rstep;                               // one-hot: bit 30 first step, 0 = done
    reg         rpark;                               // vbus < 512: park the duties
    reg  [20:0] recip;
    reg  [15:0] m_v;                                 // |v| of the leg in the multiplier
    reg  [36:0] m_p;                                 // |v| x R
    reg  [1:0]  m_sel, p_sel;
    reg         m_neg, p_neg, m_go, p_go;
    integer j;


    assign busy = running | r_valid | sh_v0 | sh_v1 | sh_v2 | w_fix | th_fix | w_hi | th_hi | adv_fix | cm_fix | (sstep != 4'd0) | (ir_x != 32'd0) | y_busy | (hold != 6'd0) |
                  (|rstep) | m_go | p_go;

    // fetch address: mirrors the fetch decisions in the main block
    always @(*) begin
        fetch_en = 1'b0; fetch_addr = pc;
        if (rst || mode == 2'd0) fetch_en = 1'b0;
        else if (tick && !running) begin fetch_en = 1'b1; fetch_addr = 8'd0; end
        else if (hold != 6'd0) fetch_en = (hold == 6'd1);
        else if (r_valid && (r_op == OP_WAIT || r_op == OP_END)) fetch_en = 1'b0;
        else fetch_en = running;
    end

    always @(posedge clk) begin
        if (rst) begin
            pc <= 8'd0; running <= 1'b0; r_valid <= 1'b0; hold <= 6'd0;
            ir_x <= 32'd0; y_busy <= 1'b0; acc <= {AW{1'b0}}; acc_c <= {AW{1'b0}};
            theta <= 32'd0; w32 <= 32'sd0; cm_int <= 32'sd0; count <= 16'd0;
            duty_a <= 16'h8000; duty_b <= 16'h8000; duty_c <= 16'h8000;
            e_amp <= 16'sd0; i0 <= 16'sd0; v_cm <= 16'sd0; iq <= 16'sd0; hall <= 3'b000;
            w_set <= 16'sd0; wshift <= 5'd12; jshift <= 5'd10; dt_idb <= 16'd200; hall_ofs <= 16'd0;
            adv <= 3'd3; th_look <= 16'd0; adv_fix <= 1'b0; sstep <= 4'd0;
            pend <= 1'b0; cm_fix <= 1'b0; w_fix <= 1'b0; th_fix <= 1'b0; sh_v0 <= 1'b0; sh_v1 <= 1'b0; sh_v2 <= 1'b0; w_hi <= 1'b0; th_hi <= 1'b0;
            rstep <= 31'd0; m_go <= 1'b0; p_go <= 1'b0; rpark <= 1'b1; recip <= 21'd0;
        end else begin
            // ---- parameter writes (fabric copies now, data RAM when the engine is not storing)
            if (pw_en) begin
                case (pw_addr)
                    5'h01: w_set <= pw_data;
                    5'h02: wshift <= (pw_data[4:0] > 5'd16) ? 5'd16 : pw_data[4:0];
                    5'h08: jshift <= pw_data[4:0];
                    5'h0D: dt_idb <= pw_data;
                    5'h0E: hall_ofs <= pw_data;
                    5'h1E: adv <= pw_data[2:0];
                    default: ;
                endcase
                pend <= 1'b1; pend_addr <= pw_addr;
                case (pw_addr)
                    5'h03, 5'h06: pend_data <= pw_data[15] ? 16'h7FFF : pw_data;   // KE, DAMP
                    5'h0B, 5'h0C: pend_data <= {1'b0, pw_data[15:1]};              // CM_MARGIN, DT_FRAC / 2
                    default:      pend_data <= pw_data;
                endcase
            end
            if (reset_state) begin
                theta <= 32'd0; th_look <= 16'd0; w32 <= {w_set, 16'h0000}; cm_int <= 32'sd0;
            end

            // ---- angles, sines and halls for the tick (sq is 2 clk behind s_sum)
            if (tick && mode != 2'd0 && !running) begin
                sstep <= 4'd1; s_base <= th_look; s_off <= 16'd0;
            end else if (sstep != 4'd0) begin
                sstep <= (sstep == 4'd13) ? 4'd0 : sstep + 4'd1;
                case (sstep)                                     // next sum's operands
                    4'd1: s_off <= -T16;
                    4'd2: s_off <= T16;
                    4'd3: begin s_base <= theta[31:16]; s_off <= 16'd0; end
                    4'd4: s_off <= -T16;
                    4'd5: s_off <= T16;
                    4'd6: begin s_base <= hall_ofs; s_off <= -T16; end   // HALL_OFS - 1/3
                    4'd7: s_off <= T16;                                  // HALL_OFS + 1/3
                    4'd8: begin s_base <= theta[31:16]; s_off <= hall_ofs; end
                    4'd9: s_off <= h_mt;
                    4'd10: s_off <= h_pt;
                    default: ;
                endcase
                if (sstep <= 4'd6) saddr <= s_sum[15:6];
                case (sstep)                                     // results
                    4'd3: s_a <= sq;   4'd4: s_b <= sq;   4'd5: s_c <= sq;
                    4'd6: r_a <= sq;   4'd7: r_b <= sq;   4'd8: r_c <= sq;
                    default: ;
                endcase
                if (sstep == 4'd7) h_mt <= s_sum;
                if (sstep == 4'd8) h_pt <= s_sum;
                if (sstep == 4'd9)  hall[0] <= ~s_sum[15];
                if (sstep == 4'd10) hall[1] <= ~s_sum[15];
                if (sstep == 4'd11) hall[2] <= ~s_sum[15];
            end

            // ---- fetch (the ROM read itself is the always block by the ROM, from fetch_addr/fetch_en)
            if (mode == 2'd0) begin
                running <= 1'b0; r_valid <= 1'b0; hold <= 6'd0;
                duty_a <= 16'h8000; duty_b <= 16'h8000; duty_c <= 16'h8000;
            end else if (tick && !running) begin
                running <= 1'b1; pc <= 8'd1; r_valid <= 1'b1;
            end else if (hold != 6'd0) begin
                hold <= hold - 6'd1;
                if (hold == 6'd1) begin pc <= pc + 8'd1; r_valid <= running; end
            end else if (r_valid && r_op == OP_WAIT) begin
                r_valid <= 1'b0; hold <= ir_r[5:0];      // the next instruction waits in pc
            end else if (r_valid && r_op == OP_END) begin
                running <= 1'b0; r_valid <= 1'b0;
            end else if (running) begin
                pc <= pc + 8'd1; r_valid <= 1'b1;
            end else r_valid <= 1'b0;

            // ---- R -> X -> Y
            ir_x <= (r_op == OP_WAIT || r_op == OP_END) ? 32'd0 : ir_r;
            y_busy <= (ir_x != 32'd0);
            nidb <= -$signed({1'b0, dt_idb[14:0]});
            // DTC's current operand is always in the data RAM: compare the RAM output directly
            if (x_op == OP_DTS) begin                   // the current is a data-RAM operand: compare qa
                dt_pos <= ($signed(qa) > $signed({1'b0, dt_idb[14:0]}));
                dt_neg <= ($signed(qa) < nidb);
            end
            a_y  <= a_x;
            b_y  <= b_x;
            p_y  <= p_x;

            // ---- execute: accumulator. Compares take two instructions (CMPx sets cflag from the adder's
            // sign, SELx then moves), so no adder result steers the accumulator in the same clk.
            // MIN = CMPS + SELNOT: takes a when acc >= a (same result when equal).
            if (f_cmp) cflag <= cmp_sum[AW-1];
            // the adder result is the last choice before the flop (one LUT after the carry chain)
            if (acc_we) begin acc <= acc_d; acc_c <= acc_d; end
            if (y_go) case (y_op)
                OP_STO: begin
                    case (y_d)
                        A_E:   e_amp <= sto_v;
                        A_I0:  i0    <= sto_v;
                        A_IQ:  iq    <= sto_v;
                        A_VCM: v_cm  <= sto_v;
                        default: ;
                    endcase
                end
                OP_W32LD: w32 <= {a_y, 16'h0000};
                OP_W32ADD: begin sh_v0 <= 1'b1; sh_th <= 1'b0; end         // acc = the 32-bit product
                OP_CMADD: begin cm_raw <= cm_sum; cm_fix <= 1'b1; end
                OP_CMZ:   cm_int <= 32'sd0;
                OP_THADV: begin                                   // acc = w32 (ACCW)
                    sh_v0 <= 1'b1; sh_th <= 1'b1;
                    count <= count + 16'd1;
                end
                default: ;
            endcase
            // the 32-bit adds go in two 16-bit halves (low half and carry, then high half): a 32-bit
            // carry chain placed across the die missed timing
            // the shift pipeline (sh_v1 is set by the Y stage above for W32ADD / THADV)
            acc_dl <= acc_c[31:0]; shamt_d <= shamt;
            sh_y  <= shamt_d[4] ? {{16{acc_dl[31]}}, acc_dl[31:16]} : acc_dl;
            sh_p  <= 16'd1 << (4'd15 - shamt_d[3:0]);
            sh_v1 <= sh_v0;
            sh_v2 <= sh_v1;
            if (sh_v0 && !(y_go && (y_op == OP_W32ADD || y_op == OP_THADV))) sh_v0 <= 1'b0;
            sh_ph <= sh_y[31:16] * sh_p;
            sh_pl <= sh_y[15:0] * sh_p;
            sh_neg <= sh_y[31]; sh_p2 <= sh_p;
            if (sh_v2) begin
                sh_q <= {sh_ph[30:0], 1'b0} + {15'd0, sh_pl[31:15]} - (sh_neg ? {sh_p2[14:0], 17'd0} : 32'd0);
                if (sh_th) th_fix <= 1'b1; else w_fix <= 1'b1;
            end
            if (w_fix)  begin w_fix <= 1'b0; w_hi <= 1'b1; {f_c, w32[15:0]} <= {1'b0, w32[15:0]} + {1'b0, sh_q[15:0]}; end
            if (w_hi)   begin w_hi <= 1'b0; w32[31:16] <= w32[31:16] + sh_q[31:16] + {15'd0, f_c}; end
            if (th_fix) begin
                th_fix  <= 1'b0; th_hi <= 1'b1;
                {f_c, theta[15:0]} <= {1'b0, theta[15:0]} + {1'b0, sh_q[15:0]};
                adv_q   <= adv_p[16:1];                     // ADV half ticks: (step x ADV) >>> 1
            end
            if (th_hi)  begin th_hi <= 1'b0; adv_fix <= 1'b1; theta[31:16] <= theta[31:16] + sh_q[31:16] + {15'd0, f_c}; end
            if (adv_fix) begin adv_fix <= 1'b0; th_look <= theta[31:16] + adv_q; end
            if (cm_fix) begin
                cm_fix <= 1'b0;
                cm_int <= (cm_raw[32] == cm_raw[31]) ? cm_raw[31:0] :           // saturating add
                          (cm_raw[32] ? 32'sh80000000 : 32'sh7FFFFFFF);
            end
            // reciprocal: RCP starts it with vbus in a_y
            if (f_rcp) begin
                rdv   <= a_y; rrem <= 17'd0; rq <= 21'd0; rstep <= 31'h40000000;
                rpark <= (a_y < 16'sd512);
            end
            // leg duty: DIV with |v| in a_y; the multiply and the offset are pipelined
            m_go <= |f_div;
            if (|f_div) begin
                m_v   <= a_y[15] ? -a_y : a_y;        // |v| <= vbus / 2
                m_neg <= a_y[15];
                m_sel <= f_div[1] ? 2'd1 : f_div[2] ? 2'd2 : 2'd0;
            end
            p_go <= m_go;
            if (m_go) begin m_p <= m_v * recip; p_neg <= m_neg; p_sel <= m_sel; end
            if (pw_go) pend <= 1'b0;

            // ---- reciprocal: 2^30 / vbus, one quotient bit per clk (dividend bit 30 is the only 1)
            if (|rstep) begin
                if ({rrem[15:0], rstep[30]} >= {1'b0, rdv}) begin
                    rrem <= {rrem[15:0], rstep[30]} - {1'b0, rdv};
                    rq   <= {rq[19:0], 1'b1};
                end else begin
                    rrem <= {rrem[15:0], rstep[30]};
                    rq   <= {rq[19:0], 1'b0};
                end
                rstep <= rstep >> 1;
                if (rstep[0]) recip <= {rq[19:0], ({rrem[15:0], 1'b0} >= {1'b0, rdv})};
            end
            // ---- duty: q = (|v| x R) >> 14
            if (p_go) begin
                case (p_sel)
                    2'd0: duty_a <= rpark ? 16'h8000 : duty_of(m_p[36:14], p_neg);
                    2'd1: duty_b <= rpark ? 16'h8000 : duty_of(m_p[36:14], p_neg);
                    default: duty_c <= rpark ? 16'h8000 : duty_of(m_p[36:14], p_neg);
                endcase
            end
        end
    end
endmodule
`default_nettype wire
