// ============================================================================
// top.v — BenchPod motor & battery emulator, LFE5U-25F-6BG256C (ECP5, schematic rev 0.5).
//
//   PLL 12 -> 36 MHz `clk` (the only clock) -- MCLK 18 MHz to the five CA-IS1305 modulators
//   spi_link  BenchPod link / stack bus, 16-bit registers, board-addressed
//   pwm3      3 legs, HS/LS per leg, on-delay dead time, centred pulses, dithered duty
//   sinc5     5 x sinc3 OSR 64 (phase A/B/C, DUT battery, bus voltage), one shared comb
//   motor_model  the PMSM model (microcoded engine, model.masm)
//   encoder   the rotor angle as ABZ and as an AS5047P / AS5048A / MA730 SPI slave
//   i2c_ctl   board id from the EEPROM strap address, trip recorder (PCAL6408A), EEPROM read / write
//   protect   overcurrent trip, per-DUT bus overvoltage brake, brake energy budget, hot-swap interlock
//   battery   battery model: the bus setpoint for the PV board (link, or the PV_SET output)
//   logbuf    logging stream (OSR 64 or 256 samples) through the link's 0xC0-0xFF window
//   sync_wd   1 ms SYNC from board 0, stack watchdog, the us / ms time base
//
// Safety (the gateware side; the board's pull-downs cover a blank FPGA):
//   - bridge gates are low unless PWM_EN, the board fault latch is armed (LATCH_Q), FAULT is
//     high, the SYNC watchdog has not tripped and the PLL is locked; FAULT or a trip also clears
//     PWM_EN, so the bridge never restarts on its own;
//   - hot-swap enable, brake and the SYNC pulse are refused on any board but board 0 (the one
//     with the PV source and the DUT battery), and before the board id is known;
//   - ARM is a single 100 us pulse per register write (the latch input is AC-coupled; the pin
//     drives the gate of the N-FET that pulls ARM_REQ_N low).
// Pads are plain Verilog (yosys maps them to the ECP5 I/O cells); pull-ups and drive strengths
// are in emu.lpf. Every pad output comes straight from a flop.
// ============================================================================
`default_nettype none
`include "emu_defs.vh"

module top (
    input  wire clk12,          // 12 MHz oscillator (PLL input ball)
    // BenchPod link / stack (the slave-SPI configuration pins, user I/O after configuration;
    // SCK also reaches a user ball, since CCLK is a dedicated configuration pin)
    input  wire spi_sck,
    input  wire spi_ss,
    input  wire spi_mosi,
    inout  wire spi_miso,       // driven only while this board answers a read
    inout  wire sync_n,         // open drain, read back
    input  wire fault_n,        // shared FAULT line, 1 = no fault
    // bridge
    output wire pwm_ha, output wire pwm_la,
    output wire pwm_hb, output wire pwm_lb,
    output wire pwm_hc, output wire pwm_lc,
    output wire arm,            // 1 = the N-FET pulls ARM_REQ_N low (AC-coupled into the latch)
    input  wire latch_q,        // fault latch state
    // power path
    output wire brake,
    output wire hswap_en,
    input  wire backstop,
    output wire pv_set,         // PV setpoint sigma-delta (DNP path); 1 = released = park
    // sensing
    output wire mclk,
    input  wire dout_a, input wire dout_b, input wire dout_c,
    input  wire dout_d, input wire dout_e,
    // board id EEPROM + trip recorder
    inout  wire i2c_sda,
    inout  wire i2c_scl,
    inout  wire ee_wc_n,        // open drain: low = EEPROM writable (10 k pull-up keeps it protected)
    // DUT signals
    output wire hall_a, output wire hall_b, output wire hall_c, // to 2N7002K gates
    output wire enc_a, output wire enc_b, output wire enc_z,
    input  wire enc_cs, input wire enc_sck, input wire enc_mosi,  // pull-ups in emu.lpf
    output wire enc_miso,
    inout  wire led_n           // open drain, sinks the LED
);
    // ------------------------------------------------------------------ clock and reset
    wire clk, pll_lock;
`ifdef SIM
    assign clk = clk12;         // benches drive clk12 at 36 MHz
    assign pll_lock = 1'b1;
`else
    // 12 MHz / 1 x 3 = 36 MHz exactly, VCO 612 MHz (ecppll -i 12 -o 36)
    (* FREQUENCY_PIN_CLKI="12" *) (* FREQUENCY_PIN_CLKOP="36" *)
    (* ICP_CURRENT="12" *) (* LPF_RESISTOR="8" *) (* MFG_ENABLE_FILTEROPAMP="1" *) (* MFG_GMCREF_SEL="2" *)
    EHXPLLL #(
        .PLLRST_ENA("DISABLED"), .INTFB_WAKE("DISABLED"), .STDBY_ENABLE("DISABLED"),
        .DPHASE_SOURCE("DISABLED"), .OUTDIVIDER_MUXA("DIVA"), .OUTDIVIDER_MUXB("DIVB"),
        .OUTDIVIDER_MUXC("DIVC"), .OUTDIVIDER_MUXD("DIVD"),
        .CLKI_DIV(1), .CLKOP_ENABLE("ENABLED"), .CLKOP_DIV(17), .CLKOP_CPHASE(8), .CLKOP_FPHASE(0),
        .FEEDBK_PATH("CLKOP"), .CLKFB_DIV(3)
    ) pll (
        .RST(1'b0), .STDBY(1'b0), .CLKI(clk12), .CLKOP(clk), .CLKFB(clk), .CLKINTFB(),
        .PHASESEL0(1'b0), .PHASESEL1(1'b0), .PHASEDIR(1'b1), .PHASESTEP(1'b1), .PHASELOADREG(1'b1),
        .PLLWAKESYNC(1'b0), .ENCLKOP(1'b0), .LOCK(pll_lock)
    );
`endif

    // flops start at 0 after configuration, so reset is built from a 0-initialised `ready`
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
    assign spi_miso = miso_oe ? miso_d : 1'bz;

    wire sda_oe, scl_oe;
    assign i2c_sda = sda_oe ? 1'b0 : 1'bz;
    assign i2c_scl = scl_oe ? 1'b0 : 1'bz;
    wire sda_in = i2c_sda;

    wire arm_oe, sync_oe, led_on;
    wire ee_we;                                 // low only while the I2C controller writes a byte
    wire sync_in = sync_n;
    assign sync_n  = sync_oe ? 1'b0 : 1'bz;
    assign ee_wc_n = ee_we   ? 1'b0 : 1'bz;
    assign led_n   = led_on  ? 1'b0 : 1'bz;
    assign arm     = arm_oe;

    wire enc_cs_i = enc_cs, enc_sck_i = enc_sck, enc_mosi_i = enc_mosi;

    // slow status inputs, synchronised
    reg [1:0] fault_s, latch_s, back_s;
    always @(posedge clk) begin
        fault_s <= {fault_s[0], fault_n};
        latch_s <= {latch_s[0], latch_q};
        back_s  <= {back_s[0],  backstop};
    end
    wire fault_ok = fault_s[1];
    wire latched  = latch_s[1];

    // ------------------------------------------------------------------ I2C: board id, trip recorder, EEPROM
    wire [1:0] board_id;
    wire       probe_done, probe_ok;
    reg        probe_start;
    reg        trip_rd, ee_wr, ee_rd;
    reg  [7:0] ee_addr, ee_wdata;
    wire [7:0] trip_src, ee_rdata;
    wire       trip_valid, pcal_ok, i2c_busy, i2c_err;
    // the expander is read when FAULT goes low (a trip fired) and on request
    reg fault_q;
    always @(posedge clk) fault_q <= fault_ok;
    i2c_ctl i2c (
        .clk(clk), .rst(rst), .sda_in(sda_in), .scl_oe(scl_oe), .sda_oe(sda_oe), .wc_low(ee_we),
        .probe_start(probe_start), .board_id(board_id), .done(probe_done), .ok(probe_ok),
        .trip_req(trip_rd | (fault_q & ~fault_ok)), .trip_src(trip_src), .trip_valid(trip_valid), .pcal_ok(pcal_ok),
        .ee_wr(ee_wr), .ee_rd(ee_rd), .ee_addr(ee_addr), .ee_wdata(ee_wdata), .ee_rdata(ee_rdata),
        .busy(i2c_busy), .err(i2c_err));
    wire is_board0 = probe_done && (board_id == 2'd0);

    // ------------------------------------------------------------------ registers
    wire [7:0]  r_addr;
    wire        r_wr;
    wire [15:0] r_wdata;
    reg  [15:0] r_rdata_q;               // read data, 2 clk after the address (two-stage mux)
    wire        rd_take, rd_done, cs_end;
    wire [7:0]  rd_addr;

    // port registers take a whole burst at one address: the table data registers and the log FIFO
    wire link_hold = (r_addr == `R_BAT_TBL_DATA) || (r_addr == `R_SHAPE_DATA) || (r_addr[7:6] == 2'b11);
    spi_link link (
        .clk(clk), .rst(rst), .board_id(board_id),
        .sck(spi_sck), .mosi(spi_mosi), .csn(spi_ss), .miso(miso_d), .miso_oe(miso_oe),
        .addr(r_addr), .rd_data(r_rdata_q), .addr_hold(link_hold), .wr_en(r_wr), .wr_data(r_wdata),
        .rd_take(rd_take), .rd_addr(rd_addr), .rd_done(rd_done), .cs_end(cs_end));

    reg [15:0] ctrl, scratch;
    reg [9:0]  period;
    reg [5:0]  deadtime, min_on;
    reg [15:0] duty_a, duty_b, duty_c;
    reg [7:0]  brake_duty;
    reg [2:0]  hall;
    reg [6:0]  arm_cnt;                  // us
    reg        wd_clear;
    // protection, battery, time base, faults, logging (see emu_defs.vh / PROTOCOL.md)
    reg signed [15:0] ofs_a, ofs_b, ofs_c, ofs_bus, bat_ofs;
    reg [15:0] oc_lim, ov_on, ov_off, brk_g, brk_pavg, brk_cap, brk_tlen, hs_limit;
    reg [3:0]  oc_count, trip_clr;
    reg        brk_test;
    reg [2:0]  bat_ctrl;
    reg [5:0]  bat_qshift, bat_ta;
    reg signed [15:0] bat_r0, bat_r1, bat_vmin, bat_vmax, pv_ofs, pv_gain;
    reg [15:0] bat_alpha, bat_wd;
    reg        bat_twe, bat_pwe;
    reg        sync_lock;
    reg [8:0]  hall_fault;
    reg [2:0]  enc_fault;
    reg [15:0] noise_amp;
    reg [9:0]  shape_a;
    reg [15:0] shape_d;
    reg        shape_we;
    reg [7:0]  log_ctrl;
    reg        log_clear;
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
    wire [3:0] trips;
    wire oc_trip = |trips[2:0];
    wire pwm_run = ctrl[`C_PWM_EN] & latched & fault_ok & ~wd_trip & ~oc_trip & ~rst;

    always @(posedge clk) begin
        probe_start <= 1'b0;
        wd_clear    <= 1'b0;
        m_reset     <= 1'b0;
        trip_clr <= 4'd0; brk_test <= 1'b0; trip_rd <= 1'b0; ee_wr <= 1'b0; ee_rd <= 1'b0;
        bat_twe <= 1'b0; bat_pwe <= 1'b0; shape_we <= 1'b0; log_clear <= 1'b0;
        if (bat_twe) bat_ta <= bat_ta + 6'd1;               // the next table word
        if (shape_we) shape_a <= shape_a + 10'd1;
        if (rst) begin
            ofs_a <= 16'sd0; ofs_b <= 16'sd0; ofs_c <= 16'sd0; ofs_bus <= 16'sd0; bat_ofs <= 16'sd0;
            oc_lim <= 16'd0; oc_count <= 4'd1; ov_on <= 16'd0; ov_off <= 16'd0;
            brk_g <= 16'd0; brk_pavg <= 16'd0; brk_cap <= 16'd0; brk_tlen <= 16'd0; hs_limit <= 16'd0;
            bat_ctrl <= 3'd0; bat_qshift <= 6'd30; bat_r0 <= 16'sd0; bat_r1 <= 16'sd0; bat_alpha <= 16'd0;
            bat_vmin <= 16'sd0; bat_vmax <= 16'sh7FFF; pv_ofs <= 16'sd0; pv_gain <= 16'sd0; bat_ta <= 6'd0;
            sync_lock <= 1'b0; hall_fault <= 9'd0; enc_fault <= 3'd0; noise_amp <= 16'd0; shape_a <= 10'd0;
            log_ctrl <= 8'd0; ee_addr <= 8'd0;
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
                `R_I_OFS_A:    ofs_a <= r_wdata;
                `R_I_OFS_B:    ofs_b <= r_wdata;
                `R_I_OFS_C:    ofs_c <= r_wdata;
                `R_VBUS_OFS:   ofs_bus <= r_wdata;
                `R_TRIPS:      trip_clr <= r_wdata[3:0];
                `R_OC_LIM:     oc_lim <= r_wdata;
                `R_OC_COUNT:   oc_count <= (r_wdata[3:0] == 4'd0) ? 4'd1 : r_wdata[3:0];
                `R_OV_ON:      ov_on <= r_wdata;
                `R_OV_OFF:     ov_off <= r_wdata;
                `R_BRK_G:      brk_g <= r_wdata;
                `R_BRK_PAVG:   brk_pavg <= r_wdata;
                `R_BRK_CAP:    brk_cap <= r_wdata;
                `R_BRK_TEST:   begin brk_tlen <= r_wdata; brk_test <= 1'b1; end
                `R_HS_LIMIT:   hs_limit <= r_wdata;
                `R_TRIP_SRC:   trip_rd <= 1'b1;
                `R_BAT_I_OFS:  bat_ofs <= r_wdata;
                `R_BAT_CTRL:   bat_ctrl <= r_wdata[2:0];
                `R_BAT_QSHIFT: bat_qshift <= r_wdata[5:0];
                `R_BAT_R0:     bat_r0 <= r_wdata;
                `R_BAT_R1:     bat_r1 <= r_wdata;
                `R_BAT_ALPHA:  bat_alpha <= r_wdata;
                `R_BAT_VMIN:   bat_vmin <= r_wdata;
                `R_BAT_VMAX:   bat_vmax <= r_wdata;
                `R_BAT_POS:    begin bat_wd <= r_wdata; bat_pwe <= 1'b1; end
                `R_BAT_TBL_ADDR: bat_ta <= r_wdata[5:0];
                `R_BAT_TBL_DATA: begin bat_wd <= r_wdata; bat_twe <= 1'b1; end
                `R_PV_OFS:     pv_ofs <= r_wdata;
                `R_PV_GAIN:    pv_gain <= r_wdata;
                `R_SYNC_CTRL:  sync_lock <= r_wdata[0];
                `R_HALL_FAULT: hall_fault <= r_wdata[8:0];
                `R_ENC_FAULT:  enc_fault <= r_wdata[2:0];
                `R_NOISE_AMP:  noise_amp <= r_wdata;
                `R_SHAPE_ADDR: shape_a <= r_wdata[9:0];
                `R_SHAPE_DATA: begin shape_d <= r_wdata; shape_we <= 1'b1; end
                `R_LOG_CTRL:   begin log_ctrl <= r_wdata[7:0]; log_clear <= r_wdata[15]; end
                `R_EE_ADDR:    ee_addr <= r_wdata[7:0];
                `R_EE_DATA:    begin ee_wdata <= r_wdata[7:0]; ee_wr <= 1'b1; end
                `R_EE_CMD:     ee_rd <= r_wdata[0];
                default: ;
            endcase
            // a fault, a stack trip or an overcurrent trip stops the bridge for good: BenchPod must
            // re-enable it; a hot-swap trip also drops HSWAP_EN
            if (!fault_ok || wd_trip || oc_trip) ctrl[`C_PWM_EN] <= 1'b0;
            if (trips[3] || oc_trip) ctrl[`C_HSWAP_EN] <= 1'b0;
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
    reg  [15:0] mdn_a, mdn_b, mdn_c;          // model duties (with injected noise)
    wire [9:0]  pwm_phase;
    reg         lk_stretch, lk_shrink;
    pwm3 bridge (
        .clk(clk), .rst(rst), .enable(pwm_run), .period(period_eff), .deadtime(deadtime), .min_on(min_on),
        .duty_a(model_on ? mdn_a : duty_a), .duty_b(model_on ? mdn_b : duty_b), .duty_c(model_on ? mdn_c : duty_c),
        .hs(hs), .ls(ls), .period_start(period_start), .period_mid(period_mid),
        .stretch(lk_stretch), .shrink(lk_shrink), .phase(pwm_phase));

    // ---- stack time base: TIME_MS counts SYNC pulses (local ms without SYNC); with SYNC_CTRL[0]
    // the PWM period (the model's tick) is pulled into phase with SYNC by one clk per period, so
    // every board's model runs on board 0's clock. The period must divide 36000 clk (1 ms).
    reg  signed [10:0] lk_pend;
    reg  signed [15:0] sync_err;
    reg  [15:0] time_ms, time_sub, time_sub_l;
    wire ms_evt = sync_seen ? sync_tick : ms_tick;
    always @(posedge clk) begin
        lk_stretch <= 1'b0; lk_shrink <= 1'b0;
        if (rst) begin
            lk_pend <= 11'sd0; sync_err <= 16'sd0; time_ms <= 16'd0; time_sub <= 16'd0;
        end else begin
            time_sub <= ms_evt ? 16'd0 : time_sub + 16'd1;
            if (ms_evt) time_ms <= time_ms + 16'd1;
            if (sync_tick) begin
                // phase 0 at the pulse is the target. A count past 0 means the period started that
                // early: stretch by it; a count in the second half means it starts soon: shrink
                sync_err <= (pwm_phase <= {1'b0, period_eff[9:1]}) ? $signed({6'd0, pwm_phase})
                                                                  : -$signed({6'd0, period_eff - pwm_phase});
                lk_pend  <= !sync_lock ? 11'sd0 :
                            (pwm_phase <= {1'b0, period_eff[9:1]}) ? $signed({1'b0, pwm_phase}) : -$signed({1'b0, period_eff - pwm_phase});
            end else if (period_start && lk_pend != 11'sd0) begin
                if (lk_pend > 0) begin lk_stretch <= 1'b1; lk_pend <= lk_pend - 11'sd1; end
                else begin lk_shrink <= 1'b1; lk_pend <= lk_pend + 11'sd1; end
            end
            if (rd_take && rd_addr == `R_TIME_MS) time_sub_l <= time_sub;
        end
    end
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

    // ---- logging: the same bits through an OSR-256 sinc3 (14.3 ENOB, 70 kSPS per channel), or the
    // OSR-64 samples, into the FIFO read through 0xC0-0xFF
    wire signed [15:0] l_a, l_b, l_c, l_d, l_e;
    wire l_set;
    sinc5 #(.L(8)) filters_log (.clk(clk), .rst(rst), .bit_en(bit_en), .din(dq),
                   .d0(l_a), .d1(l_b), .d2(l_c), .d3(l_d), .d4(l_e),
                   .valid(), .valid_ch(), .set_done(l_set));
    wire        lg256 = log_ctrl[5];
    wire [15:0] log_head, log_level, log_drops;
    reg         log_win;                       // the word in flight is a FIFO word
    always @(posedge clk) if (rd_take) log_win <= (rd_addr >= `R_LOG_FIFO);
    logbuf log (
        .clk(clk), .rst(rst), .clear(log_clear), .en(log_ctrl[6]), .mask(log_ctrl[4:0]), .seq_en(log_ctrl[7]),
        .set(lg256 ? l_set : v_set),
        .d0(lg256 ? l_a : s_a), .d1(lg256 ? l_b : s_b), .d2(lg256 ? l_c : s_c), .d3(lg256 ? l_d : s_d), .d4(lg256 ? l_e : s_e),
        .take(rd_take && rd_addr >= `R_LOG_FIFO), .commit(rd_done && log_win), .abort(cs_end),
        .head(log_head), .level(log_level), .drops(log_drops));

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
        .shape_we(shape_we), .shape_addr(shape_a), .shape_data(shape_d),
        .duty_a(md_a), .duty_b(md_b), .duty_c(md_c), .theta(m_theta), .w32(m_w32), .e_amp(m_e),
        .i0(m_i0), .v_cm(m_vcm), .iq(m_iq), .hall(m_hall), .count(m_count), .busy(m_busy));

    // ---- back-EMF noise (fault injection): each leg's model duty plus uniform noise of +-NOISE_AMP
    // (Q16 of the full duty, i.e. of the bus), from a 32-bit LFSR
    reg [31:0] lfsr = 32'hACE1_2468;
    always @(posedge clk) lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
    function [15:0] dnoise(input [15:0] d, input [15:0] r);
        reg signed [32:0] n;
        reg signed [17:0] v;
        begin
            n = $signed(r) * $signed({1'b0, noise_amp});
            v = $signed({2'b00, d}) + (n >>> 15);
            dnoise = (v < 0) ? 16'h0000 : (v > 18'sd65535) ? 16'hFFFF : v[15:0];
        end
    endfunction
    // one multiplier for the three legs in turn (a leg's noise changes every 3 clk)
    reg  [1:0]  nleg;
    wire [15:0] nsel = (nleg == 2'd0) ? md_a : (nleg == 2'd1) ? md_b : md_c;
    reg  [15:0] nout;
    reg  [1:0]  nleg_q;
    always @(posedge clk) begin
        nleg   <= (nleg == 2'd2 || rst) ? 2'd0 : nleg + 2'd1;
        nout   <= dnoise(nsel, lfsr[15:0]);
        nleg_q <= nleg;
        if (noise_amp == 16'd0) begin mdn_a <= md_a; mdn_b <= md_b; mdn_c <= md_c; end
        else case (nleg_q)
            2'd0: mdn_a <= nout;
            2'd1: mdn_b <= nout;
            default: mdn_c <= nout;
        endcase
    end

    // ------------------------------------------------------------------ SYNC / stack watchdog
    sync_wd wd (
        .clk(clk), .rst(rst), .master(ctrl[`C_SYNC_MASTER] & is_board0), .hold_low(oc_trip),   // an overcurrent trip stops the stack
        .required(ctrl[`C_SYNC_REQUIRED]), .clear(wd_clear), .sync_in(sync_in),
        .sync_oe(sync_oe), .tick(sync_tick), .seen(sync_seen), .trip(wd_trip), .us(us_tick), .ms(ms_tick));

    // ------------------------------------------------------------------ ARM, brake, hot-swap
    // pad outputs come straight from flops: a decoded counter can glitch, and ARM_REQ_N feeds an
    // AC-coupled latch input where a glitch is an edge (the gate-level bench caught it)
    reg arm_oe_q, hswap_q, led_q;
    always @(posedge clk) arm_oe_q <= (arm_cnt != 7'd0) & ~rst;
    assign arm_oe = arm_oe_q;

    // manual brake PWM about 20 kHz (36 MHz / 7 / 256); the protection block adds the
    // overvoltage clamp and the test pulse, applies the energy budget, and allows board 0 only
    reg [2:0] bpre;
    reg [7:0] bcnt;
    reg       brake_man;
    always @(posedge clk) begin
        if (rst) begin bpre <= 3'd0; bcnt <= 8'd0; brake_man <= 1'b0; end
        else begin
            bpre <= (bpre == 3'd6) ? 3'd0 : bpre + 3'd1;
            if (bpre == 3'd6) bcnt <= bcnt + 8'd1;
            brake_man <= ctrl[`C_BRAKE_EN] & (bcnt < brake_duty);
        end
    end
    wire        p_brake, brk_inhibit, ov_brake, hswap_ok;
    wire [15:0] v_bus, brk_v0, brk_v1, brk_energy;
    protect prot (
        .clk(clk), .rst(rst), .us(us_tick), .board0(is_board0), .v_any(v_any), .v_ch(v_ch),
        .s_a(s_a), .s_b(s_b), .s_c(s_c), .s_bus(s_e), .ofs_a(ofs_a), .ofs_b(ofs_b), .ofs_c(ofs_c), .ofs_bus(ofs_bus),
        .oc_lim(oc_lim), .oc_count(oc_count), .ov_on(ov_on), .ov_off(ov_off),
        .brk_g(brk_g), .brk_pavg(brk_pavg), .brk_cap(brk_cap), .hs_limit(hs_limit),
        .brake_man(brake_man), .test_start(brk_test), .test_len(brk_tlen), .hswap_on(hswap_q), .clear(trip_clr),
        .brake(p_brake), .trips(trips), .inhibit(brk_inhibit), .ov_brake(ov_brake), .hswap_ok(hswap_ok),
        .v_bus(v_bus), .test_v0(brk_v0), .test_v1(brk_v1), .energy(brk_energy));
    assign brake    = p_brake;                    // a flop inside protect
    // hot-swap: board 0, with a per-DUT bus limit set, no trip, no stack watchdog trip
    always @(posedge clk) hswap_q <= ctrl[`C_HSWAP_EN] & is_board0 & hswap_ok & ~wd_trip & ~rst;
    assign hswap_en = hswap_q;

    // ------------------------------------------------------------------ DUT side and misc
    // FET on pulls the DUT's line low; MODE[4] takes the levels from the model's angle
    reg [2:0] hall_pin;
    // fault injection: HALL_FAULT[8:6] inverts, [2:0] holds a line at [5:3]
    wire [2:0] hall_lv = ((m_mode[4] && model_on) ? m_hall : hall) ^ hall_fault[8:6];
    always @(posedge clk) hall_pin <= ~((hall_lv & ~hall_fault[2:0]) | (hall_fault[5:3] & hall_fault[2:0]));
    assign {hall_c, hall_b, hall_a} = hall_pin;

    // ------------------------------------------------------------------ encoder
    wire [15:0] e_rf_q, e_angle, e_count, e_status;
    encoder enc (
        .clk(clk), .rst(rst), .tick(period_start), .reset_state(m_reset), .theta(m_theta), .period(period_eff),
        .pw_en(e_pw), .pw_addr(e_pw_addr), .pw_data(e_pw_data),
        .cs_in(enc_cs_i), .sck_in(enc_sck_i), .mosi_in(enc_mosi_i), .fault(enc_fault),
        .enc_a(enc_a), .enc_b(enc_b), .enc_z(enc_z), .enc_miso(enc_miso),
        .ctrl(), .cpr(), .poles(), .ofs(), .rf_addr(), .rf_link_q(e_rf_q),
        .angle(e_angle), .count(e_count), .status(e_status));
    // ------------------------------------------------------------------ battery model, PV_SET
    wire signed [15:0] bat_set, bat_i, bat_ocv;
    wire [15:0] bat_pos;
    wire pv_pin;
    battery bat (
        .clk(clk), .rst(rst), .set_tick(v_set), .ms(ms_tick), .s_i(s_d), .ofs_i(bat_ofs),
        .run(bat_ctrl[0]), .inv(bat_ctrl[1]), .qshift(bat_qshift), .r0(bat_r0), .r1(bat_r1), .alpha(bat_alpha),
        .vmin(bat_vmin), .vmax(bat_vmax), .tbl_we(bat_twe), .tbl_addr(bat_ta), .tbl_data(bat_wd),
        .pos_we(bat_pwe), .pos_data(bat_wd), .pv_en(bat_ctrl[2] & is_board0),
        .park(|trips | wd_trip | ~fault_ok), .pv_ofs(pv_ofs), .pv_gain(pv_gain),
        .setpoint(bat_set), .i_ms(bat_i), .ocv(bat_ocv), .pos(bat_pos), .pv_pin(pv_pin));
    assign pv_set = pv_pin;                    // a flop inside battery; high = released = park

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
    // live registers; everything else reads the mirror
    reg  [15:0] lv;
    reg         live;
    wire [15:0] trips_r = {10'd0, ov_brake, brk_inhibit, trips};
    always @(posedge clk) begin
        live <= 1'b1;
        case (r_addr)
            8'h00: lv <= `EMU_ID;                8'h01: lv <= `EMU_VERSION;
            8'h02: lv <= {6'd0, probe_ok, probe_done, 6'd0, board_id};
            8'h03: lv <= status;                 8'h04: lv <= ctrl;
            8'h10: lv <= s_a;   8'h11: lv <= s_b;   8'h12: lv <= s_c;
            8'h13: lv <= s_d;   8'h14: lv <= s_e;   8'h15: lv <= sinc_count;
            8'h30: lv <= m_theta[31:16];   8'h31: lv <= m_w32[31:16];
            8'h32: lv <= m_e;              8'h33: lv <= m_i0;
            8'h34: lv <= m_vcm;            8'h35: lv <= m_iq;
            8'h36: lv <= md_a;             8'h37: lv <= md_b;
            8'h38: lv <= md_c;             8'h39: lv <= m_count;
            8'h45: lv <= e_rf_q;           8'h46: lv <= e_angle;
            8'h47: lv <= e_count;          8'h48: lv <= e_status;
            `R_TRIPS:      lv <= trips_r;
            `R_BRK_V0:     lv <= brk_v0;
            `R_BRK_V1:     lv <= brk_v1;
            `R_BRK_ENERGY: lv <= brk_energy;
            `R_TRIP_SRC:   lv <= {5'd0, i2c_err, pcal_ok, trip_valid, trip_src};
            `R_BAT_POS:    lv <= bat_pos;
            `R_BAT_SET:    lv <= bat_set;
            `R_BAT_I:      lv <= bat_i;
            `R_BAT_OCV:    lv <= bat_ocv;
            `R_TIME_MS:    lv <= time_ms;
            `R_TIME_SUB:   lv <= time_sub_l;
            `R_SYNC_ERR:   lv <= sync_err;
            `R_LOG_LEVEL:  lv <= log_level;
            `R_LOG_DROPS:  lv <= log_drops;
            `R_EE_DATA:    lv <= {8'd0, ee_rdata};
            `R_EE_CMD:     lv <= {14'd0, i2c_err, i2c_busy};
            default: begin
                lv <= log_head;
                live <= (r_addr >= `R_LOG_FIFO);
            end
        endcase
        r_rdata_q <= live ? lv : mq;
    end
endmodule
`default_nettype wire
