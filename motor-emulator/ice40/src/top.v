// ============================================================================
// top.v — BenchPod motor & battery emulator, iCE40UP5K-SG48I (schematic rev 0.4 pinout; REV03 for 0.3).
//
// Milestone 1: platform and safety layer. BenchPod drives the bridge duties directly over the
// link (open-loop test mode); the motor model, common-mode loop and dead-time compensation build
// on these blocks later.
//
//   PLL 12 -> 36 MHz `clk` (the only clock) -- MCLK 18 MHz to the five CA-IS1305 modulators
//   spi_link  BenchPod link / stack bus, 16-bit registers, board-addressed
//   pwm3      3 legs, HS/LS per leg, on-delay dead time, centred pulses, dithered duty
//   sinc5     5 x sinc3 OSR 64 (phase A/B/C, DUT battery, bus voltage), one shared comb
//   i2c_probe board id from the EEPROM strap address (0x50-0x53)
//   sync_wd   1 ms SYNC from board 0, stack watchdog
//
// Safety (the gateware side; the board's pull-downs cover a blank FPGA):
//   - bridge gates are low unless PWM_EN, the board fault latch is armed (LATCH_Q), FAULT is
//     high, the SYNC watchdog has not tripped and the PLL is locked; FAULT or a trip also clears
//     PWM_EN, so the bridge never restarts on its own;
//   - hot-swap enable, brake and the SYNC pulse are refused on any board but board 0 (the one
//     with the PV source and the DUT battery), and before the board id is known;
//   - ARM is a single 100 us current-sink pulse per register write (the latch input is AC-coupled).
// ============================================================================
`default_nettype none
`include "emu_defs.vh"

module top (
    input  wire clk12,          // 35  12 MHz oscillator (PLL pad)
    // BenchPod link / stack
    input  wire spi_sck,        // 15
    input  wire spi_ss,         // 16
    input  wire spi_mosi,       // 17
    inout  wire spi_miso,       // 14  driven only while this board answers a read
    inout  wire sync_n,         // 48  open drain, read back (40 = RGB1 on rev 0.3)
    input  wire fault_n,        // 46  shared FAULT line, 1 = no fault
    // bridge
    output wire pwm_ha, output wire pwm_la,   // 23, 25
    output wire pwm_hb, output wire pwm_lb,   // 26, 27
    output wire pwm_hc, output wire pwm_lc,   // 28, 31
    inout  wire arm_req_n,      // 39  RGB0, open drain, AC-coupled into the latch
    input  wire latch_q,        // 47  fault latch state
    // power path
    output wire brake,          // 32
    output wire hswap_en,       // 44
    input  wire backstop,       // 45
    output wire pv_set,         // 19  PV setpoint sigma-delta (DNP path); 1 = released = park
    // sensing
    output wire mclk,           // 34
    input  wire dout_a, input wire dout_b, input wire dout_c,   // 36, 37, 38
    input  wire dout_d, input wire dout_e,                      // 42, 43
    // board id EEPROM + trip recorder
    inout  wire i2c_sda,        // 20
    inout  wire i2c_scl,        // 21
    inout  wire ee_wc_n,        // 40  RGB1 current sink (48 on rev 0.3)
    // DUT signals
    output wire hall_a, output wire hall_b, output wire hall_c, // 2, 3, 4 (to 2N7002K gates)
    output wire enc_a, output wire enc_b, output wire enc_z,    // 6, 9, 10
    input  wire enc_cs, input wire enc_sck, input wire enc_mosi,// 11, 12, 13
    output wire enc_miso,       // 18
    inout  wire led_n           // 41  RGB2, open drain
);
    // ------------------------------------------------------------------ clock and reset
    wire clk, pll_lock;
`ifdef SIM
    assign clk = clk12;         // benches drive clk12 at 36 MHz
    assign pll_lock = 1'b1;
`else
    // 12 MHz x 48 / 16 = 36 MHz exactly (icepll -i 12 -o 36)
    SB_PLL40_PAD #(
        .FEEDBACK_PATH("SIMPLE"), .DIVR(4'b0000), .DIVF(7'b0101111), .DIVQ(3'b100),
        .FILTER_RANGE(3'b001)
    ) pll (
        .PACKAGEPIN(clk12), .PLLOUTGLOBAL(clk), .LOCK(pll_lock),
        .RESETB(1'b1), .BYPASS(1'b0)
    );
`endif

    // iCE40 flops configure to 0, so reset is built from a 0-initialised `ready`
    reg [1:0]  lock_s = 2'b00;
    reg [10:0] por = 11'd0;
    reg        ready = 1'b0;
    always @(posedge clk) begin
        lock_s <= {lock_s[0], pll_lock};
        if (!lock_s[1]) begin por <= 11'd0; ready <= 1'b0; end
        else if (por != 11'h7FF) por <= por + 11'd1;
        else ready <= 1'b1;
    end
    wire rst = ~ready;

    // ------------------------------------------------------------------ pads
    wire miso_d, miso_oe;
    SB_IO #(.PIN_TYPE(6'b101001)) io_miso (
        .PACKAGE_PIN(spi_miso), .OUTPUT_ENABLE(miso_oe), .D_OUT_0(miso_d));

    wire sda_oe, scl_oe, sda_in, scl_in;
    SB_IO #(.PIN_TYPE(6'b101001)) io_sda (
        .PACKAGE_PIN(i2c_sda), .OUTPUT_ENABLE(sda_oe), .D_OUT_0(1'b0), .D_IN_0(sda_in));
    SB_IO #(.PIN_TYPE(6'b101001)) io_scl (
        .PACKAGE_PIN(i2c_scl), .OUTPUT_ENABLE(scl_oe), .D_OUT_0(1'b0), .D_IN_0(scl_in));

    // RGB pins (39-41). With the open-source flow the only primitive nextpnr accepts on them is
    // SB_RGBA_DRV: an output-only constant-current sink (the pad cannot be read). So:
    //   ARM_REQ_N (RGB0) 24 mA: the latch input is AC-coupled through 100 nF, 24 mA pulls it down
    //                    in about 14 us, hence the 100 us ARM pulse
    //   LED_N     (RGB2)  4 mA
    //   EE_WC_N   (RGB1)  4 mA (it only ever pulls low against 10 k)
    //   SYNC is on pin 48, a normal I/O, open drain with readback (rev 0.4 pin swap). REV03 builds
    //   for the rev 0.3 pins, where SYNC sat on RGB1 and could not be read (no stack watchdog).
    wire arm_oe, sync_oe, led_on, sync_in;
    wire ee_we = 1'b0;                          // EEPROM stays write-protected for now
`ifdef SIM
    assign arm_req_n = arm_oe ? 1'b0 : 1'bz;
    assign led_n     = led_on ? 1'b0 : 1'bz;
`ifndef REV03
    assign sync_n    = sync_oe ? 1'b0 : 1'bz;
    assign sync_in   = sync_n;
    assign ee_wc_n   = ee_we ? 1'b0 : 1'bz;
`else
    assign sync_n    = sync_oe ? 1'b0 : 1'bz;
    assign sync_in   = 1'b1;                    // write-only on rev 0.3
    assign ee_wc_n   = ~ee_we;
`endif
`else
`ifndef REV03
    SB_RGBA_DRV #(.CURRENT_MODE("0b0"), .RGB0_CURRENT("0b111111"), .RGB1_CURRENT("0b000001"),
                  .RGB2_CURRENT("0b000001")) rgb (
        .CURREN(1'b1), .RGBLEDEN(1'b1), .RGB0PWM(arm_oe), .RGB1PWM(ee_we), .RGB2PWM(led_on),
        .RGB0(arm_req_n), .RGB1(ee_wc_n), .RGB2(led_n));
    SB_IO #(.PIN_TYPE(6'b101001)) io_sync (
        .PACKAGE_PIN(sync_n), .OUTPUT_ENABLE(sync_oe), .D_OUT_0(1'b0), .D_IN_0(sync_in));
`else
    SB_RGBA_DRV #(.CURRENT_MODE("0b0"), .RGB0_CURRENT("0b111111"), .RGB1_CURRENT("0b000011"),
                  .RGB2_CURRENT("0b000001")) rgb (
        .CURREN(1'b1), .RGBLEDEN(1'b1), .RGB0PWM(arm_oe), .RGB1PWM(sync_oe), .RGB2PWM(led_on),
        .RGB0(arm_req_n), .RGB1(sync_n), .RGB2(led_n));
    assign sync_in = 1'b1;
    assign ee_wc_n = ~ee_we;
`endif
`endif

    // encoder inputs float when the DUT supplies no VIO (translators disabled): pull them up
    wire enc_cs_i, enc_sck_i, enc_mosi_i;
    SB_IO #(.PIN_TYPE(6'b000001), .PULLUP(1'b1)) io_ecs  (.PACKAGE_PIN(enc_cs),   .D_IN_0(enc_cs_i));
    SB_IO #(.PIN_TYPE(6'b000001), .PULLUP(1'b1)) io_esck (.PACKAGE_PIN(enc_sck),  .D_IN_0(enc_sck_i));
    SB_IO #(.PIN_TYPE(6'b000001), .PULLUP(1'b1)) io_emo  (.PACKAGE_PIN(enc_mosi), .D_IN_0(enc_mosi_i));

    // slow status inputs, synchronised
    reg [1:0] fault_s, latch_s, back_s;
    always @(posedge clk) begin
        fault_s <= {fault_s[0], fault_n};
        latch_s <= {latch_s[0], latch_q};
        back_s  <= {back_s[0],  backstop};
    end
    wire fault_ok = fault_s[1];
    wire latched  = latch_s[1];

    // ------------------------------------------------------------------ board id
    wire [1:0] board_id;
    wire       probe_done, probe_ok;
    reg        probe_start;
    i2c_probe probe (
        .clk(clk), .rst(rst), .start(probe_start), .sda_in(sda_in),
        .scl_oe(scl_oe), .sda_oe(sda_oe), .board_id(board_id), .done(probe_done), .ok(probe_ok));
    wire is_board0 = probe_done && (board_id == 2'd0);

    // ------------------------------------------------------------------ registers
    wire [7:0]  r_addr;
    wire        r_wr;
    wire [15:0] r_wdata;
    reg  [15:0] r_rdata_q;               // read data, 2 clk after the address (two-stage mux)

    spi_link link (
        .clk(clk), .rst(rst), .board_id(board_id),
        .sck(spi_sck), .mosi(spi_mosi), .csn(spi_ss), .miso(miso_d), .miso_oe(miso_oe),
        .addr(r_addr), .rd_data(r_rdata_q), .wr_en(r_wr), .wr_data(r_wdata));

    reg [15:0] ctrl, scratch;
    reg [9:0]  period;
    reg [5:0]  deadtime, min_on;
    reg [15:0] duty_a, duty_b, duty_c;
    reg [7:0]  brake_duty;
    reg [2:0]  hall;
    reg [6:0]  arm_cnt;                  // us
    reg        wd_clear;
    // motor model: MODE here, parameters (0x21-0x3D) go into the model's data memory
    reg [4:0]  m_mode;
    reg        m_reset;
    // parameter writes are registered once (decode off the link's address path)
    reg        m_pw;
    reg [4:0]  m_pw_addr;
    reg [15:0] m_pw_data;
    always @(posedge clk) begin
        m_pw      <= r_wr && (r_addr > `R_MODE) && (r_addr <= `R_RSUB) && (r_addr != `R_MODEL_RESET);
        m_pw_addr <= r_addr[4:0];
        m_pw_data <= r_wdata;
    end

    // encoder writes (0x40-0x45), registered the same way
    reg        e_pw;
    reg  [2:0] e_pw_addr;
    reg [15:0] e_pw_data;
    always @(posedge clk) begin
        e_pw      <= r_wr && (r_addr[7:3] == 5'b01000) && (r_addr[2:0] <= 3'd5);
        e_pw_addr <= r_addr[2:0];
        e_pw_data <= r_wdata;
    end

    wire wd_trip, sync_seen, sync_tick, us_tick, ms_tick;
    wire pwm_run = ctrl[`C_PWM_EN] & latched & fault_ok & ~wd_trip & ~rst;

    always @(posedge clk) begin
        probe_start <= 1'b0;
        wd_clear    <= 1'b0;
        m_reset     <= 1'b0;
        if (rst) begin
            ctrl <= 16'h0000; scratch <= 16'h0000;
            period <= 10'd180; deadtime <= 6'd0; min_on <= 6'd2;
            duty_a <= 16'h8000; duty_b <= 16'h8000; duty_c <= 16'h8000;
            brake_duty <= 8'd0; hall <= 3'b111; arm_cnt <= 7'd0;
            m_mode <= 5'd0;
        end else begin
            if (us_tick && arm_cnt != 7'd0) arm_cnt <= arm_cnt - 7'd1;
            if (r_wr) case (r_addr)
                `R_BOARD:      if (r_wdata[15]) probe_start <= 1'b1;
                `R_CONTROL:    ctrl <= r_wdata;
                `R_ARM:        arm_cnt <= 7'd101;               // 100-101 us low (us ticks)
                `R_WD_CLEAR:   wd_clear <= 1'b1;
                `R_SCRATCH:    scratch <= r_wdata;
                `R_PWM_PERIOD: period <= r_wdata[9:0];
                `R_DEADTIME:   deadtime <= r_wdata[5:0];
                `R_DUTY_A:     duty_a <= r_wdata;
                `R_DUTY_B:     duty_b <= r_wdata;
                `R_DUTY_C:     duty_c <= r_wdata;
                `R_MIN_ON:     min_on <= r_wdata[5:0];
                `R_BRAKE_DUTY: brake_duty <= r_wdata[7:0];
                `R_HALL:       hall <= r_wdata[2:0];
                `R_MODE:       m_mode <= r_wdata[4:0];
                `R_MODEL_RESET: m_reset <= 1'b1;
                default: ;
            endcase
            // a fault or a stack trip stops the bridge for good: BenchPod must re-enable it
            if (!fault_ok || wd_trip) ctrl[`C_PWM_EN] <= 1'b0;
        end
    end

    // ------------------------------------------------------------------ bridge PWM
    wire [2:0] hs, ls;
    wire period_start, period_mid;
    wire model_on = (m_mode[1:0] != 2'd0);
    wire [15:0] md_a, md_b, md_c;
    // the model needs about 146 clk from the period start and pwm3 reads the duties from 24 clk
    // before the period end: periods under 180 clk (200 kHz) are raised in model modes
    wire [9:0] period_eff = (model_on && period < 10'd180) ? 10'd180 : period;
    pwm3 bridge (
        .clk(clk), .rst(rst), .enable(pwm_run), .period(period_eff), .deadtime(deadtime), .min_on(min_on),
        .duty_a(model_on ? md_a : duty_a), .duty_b(model_on ? md_b : duty_b), .duty_c(model_on ? md_c : duty_c),
        .hs(hs), .ls(ls), .period_start(period_start), .period_mid(period_mid));
    // gated again at the pins (registered): a fault drops every gate within 2 clk of FAULT
    // reaching the synchroniser, not at the next period
    reg [2:0] hs_pin, ls_pin;
    always @(posedge clk) begin
        hs_pin <= hs & {3{pwm_run}};
        ls_pin <= ls & {3{pwm_run}};
    end
    assign {pwm_hc, pwm_hb, pwm_ha} = hs_pin;
    assign {pwm_lc, pwm_lb, pwm_la} = ls_pin;

    // ------------------------------------------------------------------ modulators
    reg       mclk_q;
    reg [4:0] dq;
    reg       bit_en;
    always @(posedge clk) begin
        mclk_q <= rst ? 1'b0 : ~mclk_q;
        if (mclk_q) dq <= {dout_e, dout_d, dout_c, dout_b, dout_a};   // sample as MCLK falls
        bit_en <= mclk_q & ~rst;
    end
    assign mclk = mclk_q;

    wire signed [15:0] s_a, s_b, s_c, s_d, s_e;
    wire v_any, v_set;
    wire [2:0] v_ch;
    sinc5 filters (.clk(clk), .rst(rst), .bit_en(bit_en), .din(dq),
                   .d0(s_a), .d1(s_b), .d2(s_c), .d3(s_d), .d4(s_e),
                   .valid(v_any), .valid_ch(v_ch), .set_done(v_set));
    reg [15:0] sinc_count;
    always @(posedge clk) sinc_count <= rst ? 16'd0 : sinc_count + {15'd0, v_set};

    // ------------------------------------------------------------------ motor model
    wire [31:0] m_theta;
    wire signed [31:0] m_w32;
    wire signed [15:0] m_e, m_i0, m_vcm, m_iq;
    wire [2:0]  m_hall;
    wire [15:0] m_count;
    wire        m_busy;
    motor_model model (
        .clk(clk), .rst(rst), .tick(period_start), .mode(m_mode[1:0]), .cm_en(m_mode[2]), .dt_en(m_mode[3]),
        .reset_state(m_reset),
        .sinc_a(s_a), .sinc_b(s_b), .sinc_c(s_c), .sinc_bus(s_e),
        .pw_en(m_pw), .pw_addr(m_pw_addr), .pw_data(m_pw_data),
        .duty_a(md_a), .duty_b(md_b), .duty_c(md_c), .theta(m_theta), .w32(m_w32), .e_amp(m_e),
        .i0(m_i0), .v_cm(m_vcm), .iq(m_iq), .hall(m_hall), .count(m_count), .busy(m_busy));

    // ------------------------------------------------------------------ SYNC / stack watchdog
    sync_wd wd (
        .clk(clk), .rst(rst), .master(ctrl[`C_SYNC_MASTER] & is_board0), .hold_low(1'b0),
`ifndef REV03
        .required(ctrl[`C_SYNC_REQUIRED]),
`else
        .required(1'b0),                      // no SYNC readback on rev 0.3
`endif .clear(wd_clear), .sync_in(sync_in),
        .sync_oe(sync_oe), .tick(sync_tick), .seen(sync_seen), .trip(wd_trip), .us(us_tick), .ms(ms_tick));

    // ------------------------------------------------------------------ ARM, brake, hot-swap
    // pad outputs come straight from flops: a decoded counter can glitch, and ARM_REQ_N feeds an
    // AC-coupled latch input where a glitch is an edge (the gate-level bench caught it)
    reg arm_oe_q, hswap_q, led_q;
    always @(posedge clk) arm_oe_q <= (arm_cnt != 7'd0) & ~rst;
    assign arm_oe = arm_oe_q;

    // brake PWM about 20 kHz (36 MHz / 7 / 256); board 0 only
    reg [2:0] bpre;
    reg [7:0] bcnt;
    reg       brake_q;
    always @(posedge clk) begin
        if (rst) begin bpre <= 3'd0; bcnt <= 8'd0; brake_q <= 1'b0; end
        else begin
            bpre <= (bpre == 3'd6) ? 3'd0 : bpre + 3'd1;
            if (bpre == 3'd6) bcnt <= bcnt + 8'd1;
            brake_q <= ctrl[`C_BRAKE_EN] & is_board0 & (bcnt < brake_duty);
        end
    end
    assign brake    = brake_q;
    always @(posedge clk) hswap_q <= ctrl[`C_HSWAP_EN] & is_board0 & ~rst;
    assign hswap_en = hswap_q;

    // ------------------------------------------------------------------ DUT side and misc
    // FET on pulls the DUT's line low; MODE[4] takes the levels from the model's angle
    reg [2:0] hall_pin;
    always @(posedge clk) hall_pin <= ~((m_mode[4] && model_on) ? m_hall : hall);
    assign {hall_c, hall_b, hall_a} = hall_pin;

    // ------------------------------------------------------------------ encoder
    // Built with ENC (make ENC=1): it does not fit beside the model yet (see README, Next).
    wire [15:0] e_rf_q, e_angle, e_count, e_status;
`ifdef ENC
    encoder enc (
        .clk(clk), .rst(rst), .tick(period_start), .reset_state(m_reset), .theta(m_theta), .period(period_eff),
        .pw_en(e_pw), .pw_addr(e_pw_addr), .pw_data(e_pw_data),
        .cs_in(enc_cs_i), .sck_in(enc_sck_i), .mosi_in(enc_mosi_i),
        .enc_a(enc_a), .enc_b(enc_b), .enc_z(enc_z), .enc_miso(enc_miso),
        .ctrl(), .cpr(), .poles(), .ofs(), .rf_addr(), .rf_link_q(e_rf_q),
        .angle(e_angle), .count(e_count), .status(e_status));
`else
    assign enc_a = 1'b0; assign enc_b = 1'b0; assign enc_z = 1'b0; assign enc_miso = 1'b0;
    assign e_rf_q = 16'h0000; assign e_angle = 16'h0000; assign e_count = 16'h0000; assign e_status = 16'h0000;
`endif
    assign pv_set  = 1'b1;                     // buffer off: the divider holds the park level

    reg [8:0] blink;                          // ms
    always @(posedge clk) if (ms_tick) blink <= blink + 9'd1;
    always @(posedge clk) led_q <= wd_trip ? blink[5] : (pwm_run ? 1'b1 : blink[8]);
    assign led_on = led_q;

    // ------------------------------------------------------------------ read mux
    // Registers BenchPod only writes read back from a block-RAM mirror of every link write (the
    // value as written); only live values go through fabric muxes. Two registered stages (the
    // link sets each read address at least one byte time before it loads the data).
    reg [15:0] mirror [0:255];
    integer mi;
    initial begin                                           // the power-up defaults
        for (mi = 0; mi < 256; mi = mi + 1) mirror[mi] = 16'h0000;
        mirror[`R_PWM_PERIOD] = 16'd180; mirror[`R_MIN_ON] = 16'd2; mirror[`R_HALL] = 16'h0007;
        mirror[`R_DUTY_A] = 16'h8000; mirror[`R_DUTY_B] = 16'h8000; mirror[`R_DUTY_C] = 16'h8000;
        mirror[`R_WSHIFT] = 16'd12; mirror[`R_JSHIFT] = 16'd10; mirror[`R_DT_IDB] = 16'd200;
        mirror[`R_ADV] = 16'd3; mirror[`R_ENC_CPR] = 16'd4000; mirror[`R_ENC_POLES] = 16'd1;
    end
    reg [15:0] mq;
    always @(posedge clk) begin
        if (r_wr) mirror[r_addr] <= r_wdata;
        mq <= mirror[r_addr];
    end
    wire [15:0] status = {8'd0,
        pll_lock, sync_seen, pwm_run, wd_trip, sync_in, back_s[1], latched, fault_ok};
    // live: 0x00-0x04, 0x10-0x15, 0x30-0x39, 0x45-0x48
    reg  [15:0] lv0, lv2, lv6, lv8;
    reg  [1:0]  lsel;
    reg         live;
    always @(posedge clk) begin
        case (r_addr[2:0])                                  // 0x00-0x04
            3'd0: lv0 <= `EMU_ID;                3'd1: lv0 <= `EMU_VERSION;
            3'd2: lv0 <= {6'd0, probe_ok, probe_done, 6'd0, board_id};
            3'd3: lv0 <= status;                 default: lv0 <= ctrl;
        endcase
        case (r_addr[2:0])                                  // 0x10-0x15
            3'd0: lv2 <= s_a;   3'd1: lv2 <= s_b;   3'd2: lv2 <= s_c;
            3'd3: lv2 <= s_d;   3'd4: lv2 <= s_e;   default: lv2 <= sinc_count;
        endcase
        case (r_addr[3:0])                                  // 0x30-0x39
            4'h0: lv6 <= m_theta[31:16];   4'h1: lv6 <= m_w32[31:16];
            4'h2: lv6 <= m_e;              4'h3: lv6 <= m_i0;
            4'h4: lv6 <= m_vcm;            4'h5: lv6 <= m_iq;
            4'h6: lv6 <= md_a;             4'h7: lv6 <= md_b;
            4'h8: lv6 <= md_c;             default: lv6 <= m_count;
        endcase
        case (r_addr[3:0])                                  // 0x45-0x48
            4'h5: lv8 <= e_rf_q;           4'h6: lv8 <= e_angle;
            4'h7: lv8 <= e_count;          default: lv8 <= e_status;
        endcase
        lsel <= r_addr[6] ? 2'd3 : r_addr[5] ? 2'd2 : r_addr[4] ? 2'd1 : 2'd0;
        live <= (r_addr[7:3] == 5'b00000 && r_addr[2:0] <= 3'd4) ||
                (r_addr[7:3] == 5'b00010 && r_addr[2:0] <= 3'd5) ||
                (r_addr[7:4] == 4'h3 && r_addr[3:0] <= 4'h9) ||
                (r_addr[7:4] == 4'h4 && r_addr[3:0] >= 4'h5 && r_addr[3:0] <= 4'h8);
        r_rdata_q <= !live ? mq : (lsel == 2'd0) ? lv0 : (lsel == 2'd1) ? lv2 : (lsel == 2'd2) ? lv6 : lv8;
    end
endmodule
`default_nettype wire
