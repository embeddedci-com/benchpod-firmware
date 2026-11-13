// ============================================================================
// engine_block.v: the signal-engine control plane, instantiated by top_v2.v.
//
// Holds the SPI command path (spi_slave + cmd_dispatch), the LA GPIO bank with its
// stepper/SWD/SPI/I2C/UART drivers, the emulated I2C sensor (target + register file)
// and the waveform BRAM, together with the inter-engine wiring (la_bank arbitration,
// the la_in readback taps that feed I2C/UART/SWD, and the regfile<->target<->dispatch
// plumbing).  The analog datapath (DAC/ADC engines, the control loop, the PSRAM
// capture and replay pipeline) lives in top_v2 and connects through the boundary
// ports below.  It was split out when there were two tops (the retired v1 parallel-ADC
// board shared it); several port comments still say what v1 tied off.
//
// Parameters:
//   N                : LA bank width (14 on the v2 board).
//   GATEWARE_VERSION : VERSION-command reply byte, passed straight to cmd_dispatch;
//                      top_v2 sets it (the release value).
//   FEATURES         : FPGA_FEATURES reply byte (bit0 loop image, bit1 deep image).
// ============================================================================
`include "sys_config.vh"   // SYS_CLK_MHZ — single source (tools/gen_protocol.py)
`include "image_config.vh" // HAS_EMULATION: the sensor blocks are only in the emulation image (v54)

module engine_block #(
    parameter N                = 14,
    parameter GATEWARE_VERSION = 8'd6,
    parameter [7:0] FEATURES = 8'h01
)(
    input  wire         clk,
    input  wire         rst,

    // ---- SPI link to the host MCU ----
    input  wire         sck,
    input  wire         mosi,
    output wire         miso,
    input  wire         csn,

    // ---- LA GPIO bank — la_bank drives these pads (SB_IO inside) ----
    inout  wire [N-1:0] la,

    // ---- BUSY composition: each top OR's this into its own busy output, since
    //      the rest of the busy term (ADC capture / PSRAM idle) is version-specific.
    output wire         i2c_la_busy,

    // ---- activity readback for the v1 RGB "yellow" LED (v2 leaves unconnected) ----
    output wire         step_busy,
    output wire         swd_armed,

    // ---- host-driven status LEDs (SET_LED cmd -> cmd_dispatch.led_ctrl) ----
    output wire [7:0]   led_ctrl,      // bit0=green bit1=yellow bit2=red

    // ---- DAC waveform read port (cmd_dispatch writes wave_buf on `clk`) ----
    // wave_rclk is the read-port clock: v2 passes clk48 (DAC sequencer at 48 MHz),
    // v1 passes clk (same-clock).  See sample_buf.v (dual-clock BRAM).
    input  wire         wave_rclk,
    input  wire [11:0]  wave_raddr,
    output wire [7:0]   wave_rdata,

    // ---- DAC control -> the version-specific DAC engine in the top ----
    output wire         dac_start,
    output wire         dac_stop,
    // co-trigger arm (OP_DAC_ARM_ON_CAPTURE 0x19, v2 >= v27): 1-cycle strobe; the top defers the
    // next dac_start to the next capture arm so DAC sample 0 == capture t0.
    output wire         dac_cotrig,
    output wire [12:0]  dac_period,
    output wire [15:0]  dac_divider,
    input  wire         dac_running,

    // ---- DEEP DAC replay from PSRAM (v2 >= v17) -> the top's dac_psram_reader ----
    output wire         dac_psram_mode,   // level: 1 while streaming a PSRAM waveform
    output wire [23:0]  dac_psram_base,   // byte base in PSRAM
    output wire [23:0]  dac_psram_len,    // waveform length in SAMPLES

    // ---- capture-tied DAC auto-stop threshold (OP_SET_DAC_STOP_AFTER, v2 >= v21) -> top ----
    // 24 MHz clk cycles after a capture's t0 at which the top cuts a running DAC (0 = disarmed).
    output wire         stop_after_stb,   // v40: loads the top's countdown (one-shot)
    output wire [31:0]  stop_after_cfg,
    // ---- DAC frame control byte (OP_DAC_CTRL 0x38, v2 >= v49) -> top ----
    output wire [7:0]   dac_ctrl,         // stream control byte, taken at each DAC start
    output wire         dac_wr_stb,       // one static frame...
    output wire [23:0]  dac_wr_cfg,       // ...{ctrl, code_hi, code_lo}
    // closed-loop DAC control (OP_START_DAC_LOOP 0x15, >=v23)
    output wire         dac_loop_mode,
    // Loop parameters as strobe + payload (v38): top_v2 crosses each into clk48 registers.
    // See cmd_dispatch's port list for the field layouts.
    output wire         loop_arm_stb,     // OP_START_DAC_LOOP: {tick_div, vmax, vmin, k_q15}
    output wire [63:0]  loop_arm_cfg,
    output wire         loop_src_stb,     // OP_DAC_LOOP_SRC (v29): {step, fixed, src}
    output wire [33:0]  loop_src_cfg,
    output wire         loop_inmap_stb,   // OP_DAC_LOOP_INMAP (v30): {trip_en, map_en, trip, gain, zero}
    output wire [44:0]  loop_inmap_cfg,
    input  wire         loop_tripped,     // latched over-range trip -> STATUS bit 6

    // ---- capture control -> the version-specific ADC/capture datapath ----
    output wire         cap_start,
    output wire [23:0]  cap_count,     // 24-bit: deep ADC capture (up to 2,097,152 samples, multi-MB PSRAM region); decoupled from ADDR_W
    output wire [15:0]  cap_divider,
    output wire [23:0]  adc_cap_base,  // runtime ADC PSRAM base (SET_CAPTURE_BASES 0x32)
    output wire         cap_test_ramp,    // v2: 1=inject ramp instead of real ADC (cap-selftest)
    output wire         psram_cs_force,   // v2: 1 -> force PSRAM /CS low (boot /CE-net self-test)
    // capture trigger (OP_SET_TRIGGER 0x33 / TRIGGER_STATUS 0x34, v2 >= v35) <-> the top's gate
    output wire [3:0]   trig_ch,
    output wire         trig_en,
    output wire         trig_edge,
    output wire         trig_pol,
    output wire         trig_pre,
    input  wire         trig_wait,
    input  wire         trig_fired,
    input  wire         cap_busy,         // STATUS busy bit (top's capture engine)
    input  wire         cap_done,         // STATUS done bit (v2 OR's in corr_done)
    input  wire         cap_overflow,     // STATUS bit5: a PSRAM capture lost bytes
                                          // (v2 aggregates the writer/ring overflow
                                          // diagnostics; v1 ties 0)

    // ---- ADC_PROBE diagnostic: live 16-bit ADC sample straight off the engine
    //      (24 MHz), routed to cmd_dispatch for a PSRAM-bypassing direct SPI read.
    //      v1 has no serial ADC; tie to 0 there.
    input  wire [15:0]  adc_dbg_sample,
    input  wire [15:0]  dac_loop_v_dbg,   // DAC_PROBE: live loop DAC output
    input  wire [15:0]  dac_loop_in_dbg,  // DAC_LOOP_IN_PROBE: live loop INPUT (v29)

    // ---- deep LA capture into PSRAM (LA_CAPTURE) -> v2's la_psram_capture.
    //      la_sample is the live LA readback the sampler latches; the control
    //      trio is the arm pulse + 16-bit sample count + divider.  v1 leaves all
    //      of these outputs unconnected (no PSRAM datapath). ----
    output wire         la_cap_start,
    output wire [23:0]  la_cap_count,     // 24-bit: deep LA up to the full 8 MB PSRAM
    output wire [15:0]  la_cap_divider,
    output wire [13:0]  la_sample,
    output wire [13:0]  la_levels         // la_sample through a 2-flop synchroniser (GPIO_GET + trigger)
);

    // ---- SPI slave ----
    wire [7:0] rx_byte, tx_byte;
    wire       rx_valid, cs_active;
    spi_slave spi_i (
        .clk(clk), .rst(rst), .sck(sck), .mosi(mosi), .miso(miso), .csn(csn),
        .rx_byte(rx_byte), .rx_valid(rx_valid), .tx_byte(tx_byte), .cs_active(cs_active)
    );

    // ---- Waveform BRAM (cmd_dispatch writes, version DAC engine reads) ----
    wire        wave_we;
    wire [11:0] wave_waddr;
    wire [7:0]  wave_wdata;
    sample_buf wave_buf (
        .clk(clk), .a_we(wave_we), .a_addr(wave_waddr), .a_din(wave_wdata),
        .b_clk(wave_rclk), .b_addr(wave_raddr), .b_dout(wave_rdata)
    );

    // ---- engine wires (LA / stepper / SWD / I2C / UART) ----
    wire        gpio_set_stb;  wire [3:0] gpio_set_ch;  wire [1:0] gpio_set_mode;
    wire        step_start;    wire [3:0] step_channel;
    wire [15:0] step_steps, step_delay;   // step_busy is an output port
    wire [3:0]  step_la_ch;    wire step_la_val;
    wire        swd_arm_stb, swd_disarm_stb;
    wire [3:0]  swd_clk_ch_arg, swd_dio_ch_arg;
    wire        swd_feed_begin, swd_feed_stb;
    wire [7:0]  swd_feed_byte;  wire [8:0] swd_rd_addr;  wire [7:0] swd_rd_data;
    wire [15:0] swd_reply_count;          // swd_armed is an output port
    wire [3:0]  swd_clk_ch, swd_dio_ch;
    wire        swd_clk_val, swd_clk_oe, swd_dio_val, swd_dio_oe;
    wire        spi_arm_stb, spi_cs_stb, spi_cs_assert;
    wire [2:0]  spi_flags;
    wire [3:0]  spi_sck_ch, spi_mosi_ch, spi_miso_ch, spi_cs_ch_arg;
    wire [6:0]  spi_half;
    wire        spi_mode, spi_busy, swd_cs_on, swd_cs_val;
    wire [3:0]  swd_in_ch, swd_cs_ch;
    wire        swd_feed_q, q_cfg_stb;
    wire [5:0]  q_cfg_half;
    wire [4:0]  q_cfg_idle;
    wire [7:0]  q_done, q_flags;
    wire [N-1:0] la_in;
    // emulated sensors (v54, emulation image): cmd_dispatch <-> i2c_emu / spi target / FIFO / pins
    wire [2:0]  emu_t;
    wire        i2c_cfg_stb, i2c_cfg_en_only;  wire [40:0] i2c_cfg_data;
    wire        i2c_reg_we, i2c_reg_attr, i2c_reg_log;  wire [7:0] i2c_reg_addr, i2c_reg_wdata, i2c_reg_rdata;
    wire        i2c_tt_we;  wire [1:0] i2c_tt_idx;  wire [15:0] i2c_tt_val;
    wire        i2c_fire_stb, i2c_fire_set;
    wire [31:0] i2c_status;
    wire [7:0]  emu_dr_width;
    wire        emu_fifo_flush, emu_fifo_we, emu_fifo_stat_clr;
    wire [41:0] emu_pinmap;
    wire [11:0] emu_fifo_level;
    wire        emu_fifo_ovf, emu_fifo_unf;
    wire        spit_cfg_stb;  wire [55:0] spit_cfg;
    wire [N-1:0] emu_on, emu_out, emu_oe;
    reg  [13:0] la_s1, la_s2;   // the LA levels through a 2-flop synchroniser (see the end of the file)
    wire        uart_cfg_stb, uart_cfg_enable, uart_disable_stb;
    wire [3:0]  uart_cfg_rx_ch, uart_cfg_tx_ch;  wire [23:0] uart_cfg_div;
    wire        uart_tx_we, uart_rx_re, uart_rx_ovf_clr;
    wire [7:0]  uart_tx_wdata, uart_rx_rdata;  wire [8:0] uart_rx_avail;
    wire        uart_tx_full, uart_tx_empty, uart_rx_overflow, uart_rx_frame_err, uart_armed;
    wire [3:0]  uart_rx_ch, uart_tx_ch;  wire uart_tx_out;
    wire        uart_rx_in = la_in[uart_rx_ch];
    wire        uart2_cfg_stb, uart2_tx_we, uart2_tx_full, uart2_tx_empty, uart2_armed;
    wire        uart2_tx_out;
    wire        uart2_rx_re, uart2_rx_overflow, uart2_rx_frame_err, uart2_rx_ovf_clr;
    wire [7:0]  uart2_rx_rdata;  wire [8:0] uart2_rx_avail;

    // ---- command dispatcher ----
    // cap_done is the already-combined STATUS done bit (v2 folds in corr_done before
    // driving this port).  (The v1 BRAM capture read-back — cap_raddr/cap_rdata /
    // READ_CAPTURE — was removed in v17; v2 reads captures back from PSRAM over XSPI.)
    cmd_dispatch #(.GATEWARE_VERSION(GATEWARE_VERSION), .FEATURES(FEATURES)) dispatch_i (
        .clk(clk), .rst(rst),
        .rx_byte(rx_byte), .rx_valid(rx_valid), .tx_byte(tx_byte), .cs_active(cs_active),
        .wave_we(wave_we), .wave_waddr(wave_waddr), .wave_wdata(wave_wdata),
        .dac_start(dac_start), .dac_stop(dac_stop),
        .dac_cotrig_stb(dac_cotrig),
        .dac_period(dac_period), .dac_divider(dac_divider),
        .dac_psram_mode(dac_psram_mode), .dac_psram_base(dac_psram_base), .dac_psram_len(dac_psram_len),
        .stop_after_stb(stop_after_stb), .stop_after_cfg(stop_after_cfg),
        .dac_ctrl(dac_ctrl), .dac_wr_stb(dac_wr_stb), .dac_wr_cfg(dac_wr_cfg),
        .dac_loop_mode(dac_loop_mode),
        .loop_arm_stb(loop_arm_stb),     .loop_arm_cfg(loop_arm_cfg),
        .loop_src_stb(loop_src_stb),     .loop_src_cfg(loop_src_cfg),
        .loop_inmap_stb(loop_inmap_stb), .loop_inmap_cfg(loop_inmap_cfg),
        .loop_tripped(loop_tripped),
        .cap_start(cap_start), .cap_count(cap_count), .cap_divider(cap_divider),
        .adc_cap_base(adc_cap_base),
        .cap_test_ramp(cap_test_ramp),
        .psram_cs_force(psram_cs_force),
        .trig_ch(trig_ch), .trig_en(trig_en), .trig_edge(trig_edge), .trig_pol(trig_pol), .trig_pre(trig_pre),
        .trig_wait(trig_wait), .trig_fired(trig_fired), .la_levels(la_levels),
        .dac_running(dac_running), .cap_busy(cap_busy), .cap_done(cap_done),
        .cap_overflow(cap_overflow),
        .adc_dbg_sample(adc_dbg_sample), .dac_loop_v_dbg(dac_loop_v_dbg),
        .dac_loop_in_dbg(dac_loop_in_dbg),
        .gpio_set_stb(gpio_set_stb), .gpio_set_ch(gpio_set_ch), .gpio_set_mode(gpio_set_mode),
        .led_ctrl(led_ctrl),
        .step_start(step_start), .step_channel(step_channel),
        .step_steps(step_steps), .step_delay(step_delay), .step_busy(step_busy),
        .swd_arm_stb(swd_arm_stb), .swd_clk_ch(swd_clk_ch_arg),
        .swd_dio_ch(swd_dio_ch_arg), .swd_disarm_stb(swd_disarm_stb),
        .swd_feed_begin(swd_feed_begin), .swd_feed_stb(swd_feed_stb), .swd_feed_byte(swd_feed_byte),
        .swd_rd_addr(swd_rd_addr), .swd_rd_data(swd_rd_data),
        .swd_armed(swd_armed), .swd_reply_count(swd_reply_count),
        .spi_arm_stb(spi_arm_stb), .spi_sck_ch(spi_sck_ch), .spi_mosi_ch(spi_mosi_ch),
        .spi_miso_ch(spi_miso_ch), .spi_cs_ch(spi_cs_ch_arg), .spi_half(spi_half),
        .spi_flags(spi_flags), .spi_cs_stb(spi_cs_stb), .spi_cs_assert(spi_cs_assert),
        .spi_mode(spi_mode), .spi_busy(spi_busy),
        .swd_feed_q(swd_feed_q), .q_cfg_stb(q_cfg_stb), .q_cfg_half(q_cfg_half),
        .q_cfg_idle(q_cfg_idle), .q_done(q_done), .q_flags(q_flags),
        .emu_t(emu_t),
        .i2c_cfg_stb(i2c_cfg_stb), .i2c_cfg_en_only(i2c_cfg_en_only), .i2c_cfg_data(i2c_cfg_data),
        .i2c_reg_we(i2c_reg_we), .i2c_reg_attr(i2c_reg_attr), .i2c_reg_log(i2c_reg_log), .i2c_reg_addr(i2c_reg_addr),
        .i2c_reg_wdata(i2c_reg_wdata), .i2c_reg_rdata(i2c_reg_rdata),
        .i2c_tt_we(i2c_tt_we), .i2c_tt_idx(i2c_tt_idx), .i2c_tt_val(i2c_tt_val),
        .i2c_fire_stb(i2c_fire_stb), .i2c_fire_set(i2c_fire_set), .i2c_status(i2c_status),
        .emu_dr_width(emu_dr_width), .emu_fifo_flush(emu_fifo_flush), .emu_pinmap(emu_pinmap),
        .emu_fifo_we(emu_fifo_we), .emu_fifo_level(emu_fifo_level), .emu_fifo_ovf(emu_fifo_ovf),
        .emu_fifo_unf(emu_fifo_unf), .emu_fifo_stat_clr(emu_fifo_stat_clr),
        .spit_cfg_stb(spit_cfg_stb), .spit_cfg(spit_cfg),
        .la_cap_start(la_cap_start), .la_cap_count(la_cap_count), .la_cap_divider(la_cap_divider),
        .uart_cfg_stb(uart_cfg_stb), .uart_cfg_rx_ch(uart_cfg_rx_ch), .uart_cfg_tx_ch(uart_cfg_tx_ch),
        .uart_cfg_div(uart_cfg_div), .uart_cfg_enable(uart_cfg_enable), .uart_disable_stb(uart_disable_stb),
        .uart_tx_we(uart_tx_we), .uart_tx_wdata(uart_tx_wdata),
        .uart_rx_re(uart_rx_re), .uart_rx_rdata(uart_rx_rdata), .uart_rx_ovf_clr(uart_rx_ovf_clr),
        .uart_rx_avail(uart_rx_avail), .uart_tx_full(uart_tx_full),
        .uart_tx_empty(uart_tx_empty), .uart_rx_overflow(uart_rx_overflow), .uart_rx_frame_err(uart_rx_frame_err),
        .uart_armed(uart_armed),
        .uart2_cfg_stb(uart2_cfg_stb), .uart2_tx_we(uart2_tx_we), .uart2_tx_full(uart2_tx_full),
        .uart2_tx_empty(uart2_tx_empty), .uart2_armed(uart2_armed),
        .uart2_rx_re(uart2_rx_re), .uart2_rx_rdata(uart2_rx_rdata), .uart2_rx_avail(uart2_rx_avail),
        .uart2_rx_overflow(uart2_rx_overflow), .uart2_rx_frame_err(uart2_rx_frame_err),
        .uart2_rx_ovf_clr(uart2_rx_ovf_clr)
    );

`ifdef HAS_EMULATION
    // ---- UART2 (emulation image): v48 TX for streams the pod sends the DUT (an emulated GPS
    // receiver's NMEA) while the proxy keeps the console; v54 RX too (a UART sensor's requests).
    // Its TX reaches a pin through the pin map (EMU_PINMAP code 7), not a la_bank column. ----
    wire [3:0] uart2_rx_ch;
    uart_engine uart2_i (
        .clk(clk), .rst(rst),
        .cfg_stb(uart2_cfg_stb), .cfg_rx_ch(uart_cfg_rx_ch), .cfg_tx_ch(uart_cfg_tx_ch),
        .cfg_div(uart_cfg_div), .cfg_enable(uart_cfg_enable), .disable_stb(1'b0),
        .rx_ch(uart2_rx_ch), .tx_ch(), .armed(uart2_armed),
        .tx_we(uart2_tx_we), .tx_wdata(uart_tx_wdata), .tx_full(uart2_tx_full), .tx_empty(uart2_tx_empty),
        .rx_re(uart2_rx_re), .rx_rdata(uart2_rx_rdata), .rx_avail(uart2_rx_avail),
        .rx_overflow(uart2_rx_overflow), .rx_ovf_clr(uart2_rx_ovf_clr), .rx_frame_err(uart2_rx_frame_err),
        .rx_in(la_in[uart2_rx_ch]), .tx_out(uart2_tx_out)
    );
`else
    // loop and deep images (v54): no UART2
    assign uart2_armed = 1'b0; assign uart2_tx_out = 1'b1;
    assign uart2_tx_full = 1'b0; assign uart2_tx_empty = 1'b1;
    assign uart2_rx_rdata = 8'd0; assign uart2_rx_avail = 9'd0;
    assign uart2_rx_overflow = 1'b0; assign uart2_rx_frame_err = 1'b0;
`endif

    uart_engine uart_i (
        .clk(clk), .rst(rst),
        .cfg_stb(uart_cfg_stb), .cfg_rx_ch(uart_cfg_rx_ch), .cfg_tx_ch(uart_cfg_tx_ch),
        .cfg_div(uart_cfg_div), .cfg_enable(uart_cfg_enable), .disable_stb(uart_disable_stb),
        .rx_ch(uart_rx_ch), .tx_ch(uart_tx_ch), .armed(uart_armed),
        .tx_we(uart_tx_we), .tx_wdata(uart_tx_wdata), .tx_full(uart_tx_full), .tx_empty(uart_tx_empty),
        .rx_re(uart_rx_re), .rx_rdata(uart_rx_rdata), .rx_avail(uart_rx_avail),
        .rx_overflow(uart_rx_overflow), .rx_ovf_clr(uart_rx_ovf_clr), .rx_frame_err(uart_rx_frame_err),
        .rx_in(uart_rx_in), .tx_out(uart_tx_out)
    );

`ifdef HAS_EMULATION
    // ==== emulated sensors (v54, emulation image only) ====
    // Four I2C targets in one time-multiplexed engine (i2c_emu), the shared FIFO its FIFO registers
    // pop, and the pin map that puts their outputs (and the SPI target's MISO, UART2 TX) on LA
    // wires.  emu_t picks the target of the SPI-side commands: 0..3 I2C, 4 the SPI target.
    wire [N-1:0] i2c_pull;
    wire [3:0]   i2c_drdy;
    wire [15:0]  us_now;
    wire [7:0]   i2c_emu_rdata;
    wire [31:0]  i2c_emu_status;
    wire         fifo_pop, fifo_unf_p, fifo_empty, fifo_full;
    wire         i2c_fifo_pop, i2c_fifo_unf, spit_fifo_pop, spit_fifo_unf;
    wire [7:0]   log_rdata;
    wire         spit_ev;  wire [4:0] spit_ev_idx;  wire [15:0] spit_ev_data;
    // two FIFO readers: an I2C target's FIFO register and the SPI target's (use one at a time)
    assign fifo_pop   = i2c_fifo_pop | spit_fifo_pop;
    assign fifo_unf_p = i2c_fifo_unf | spit_fifo_unf;
    wire [7:0]   fifo_head;
    i2c_emu #(.CLK_MHZ(`SYS_CLK_MHZ), .N(N)) i2c_emu_i (
        .clk(clk), .rst(rst), .la_lvl(la_s2), .pull(i2c_pull), .drdy_pin(i2c_drdy), .us_now(us_now),
        .sel_t(emu_t[1:0]), .cfg_stb(i2c_cfg_stb & ~emu_t[2]), .cfg_en_only(i2c_cfg_en_only),
        .cfg_data(i2c_cfg_data),
        .reg_we(i2c_reg_we & ~emu_t[2]), .reg_attr(i2c_reg_attr), .reg_addr(i2c_reg_addr),
        .reg_wdata(i2c_reg_wdata), .reg_rdata(i2c_emu_rdata),
        .tt_we(i2c_tt_we & ~emu_t[2]), .tt_idx(i2c_tt_idx), .tt_val(i2c_tt_val),
        .fire_stb(i2c_fire_stb & ~emu_t[2]), .fire_set(i2c_fire_set), .dr_width(emu_dr_width),
        .status(i2c_emu_status),
        .log_t(emu_t), .log_idx(i2c_reg_addr[5:1]), .log_byte(i2c_reg_addr[0]), .log_rdata(log_rdata),
        .sp_ev(spit_ev), .sp_idx(spit_ev_idx), .sp_data(spit_ev_data),
        .fifo_pop(i2c_fifo_pop), .fifo_unf(i2c_fifo_unf), .fifo_head(fifo_head), .fifo_empty(fifo_empty)
    );

    // the shared sensor FIFO (2 KB in the free SPRAM): EMU_FIFO_WRITE pushes, FIFO registers pop
    wire fifo_ovf_p;
    wire [10:0] fifo_mlevel;
    emu_fifo #(.AW(11)) emu_fifo_i (
        .clk(clk), .rst(rst | emu_fifo_flush),
        .we(emu_fifo_we), .wdata(uart_tx_wdata), .pop(fifo_pop), .head(fifo_head),
        .empty(fifo_empty), .mlevel(fifo_mlevel), .full(fifo_full), .ovf(fifo_ovf_p)
    );
    // EMU_FIFO_STATUS: {bytes in the memory (11), the head is valid}; the firmware adds them
    assign emu_fifo_level = {fifo_mlevel, ~fifo_empty};
    reg fifo_ovf_r = 1'b0, fifo_unf_r = 1'b0;   // sticky; EMU_FIFO_STATUS clears them
    always @(posedge clk) begin
        if (rst | emu_fifo_stat_clr) begin fifo_ovf_r <= 1'b0; fifo_unf_r <= 1'b0; end
        else begin
            if (fifo_ovf_p)              fifo_ovf_r <= 1'b1;
            if (fifo_unf_p)              fifo_unf_r <= 1'b1;
        end
    end
    assign emu_fifo_ovf = fifo_ovf_r;
    assign emu_fifo_unf = fifo_unf_r;

    // the SPI target (target 4 of the I2C_* image commands)
    wire        spit_miso, spit_miso_oe;
    wire [7:0]  spit_rdata;
    wire [31:0] spit_status;
    spi_target #(.N(N)) spit_i (
        .clk(clk), .rst(rst), .la_lvl(la_s2), .miso(spit_miso), .miso_oe(spit_miso_oe),
        .cfg_stb(spit_cfg_stb), .cfg(spit_cfg),
        .reg_we(i2c_reg_we & emu_t[2]), .reg_attr(i2c_reg_attr), .reg_addr(i2c_reg_addr),
        .reg_wdata(i2c_reg_wdata), .reg_rdata(spit_rdata), .status(spit_status),
        .ev(spit_ev), .ev_idx(spit_ev_idx), .ev_data(spit_ev_data),
        .fifo_pop(spit_fifo_pop), .fifo_unf(spit_fifo_unf), .fifo_head(fifo_head), .fifo_empty(fifo_empty)
    );
    assign i2c_reg_rdata = i2c_reg_log ? log_rdata : emu_t[2] ? spit_rdata  : i2c_emu_rdata;
    assign i2c_status    = emu_t[2] ? spit_status : i2c_emu_status;

    // the pin map (EMU_PINMAP): per LA wire, which emulated output drives it
    genvar pc;
    generate for (pc = 0; pc < N; pc = pc + 1) begin : g_pin
        wire [2:0] code = emu_pinmap[3*pc +: 3];
        assign emu_on[pc]  = (code != 3'd0);
        // I2C: open drain; MISO: only while CS is low (and not muted); UART2 TX: only while armed
        assign emu_oe[pc]  = (code == 3'd1) ? i2c_pull[pc] : (code == 3'd6) ? spit_miso_oe :
                             (code == 3'd7) ? uart2_armed  : 1'b1;
        assign emu_out[pc] = (code == 3'd2) ? i2c_drdy[0] : (code == 3'd3) ? i2c_drdy[1] :
                             (code == 3'd4) ? i2c_drdy[2] : (code == 3'd5) ? i2c_drdy[3] :
                             (code == 3'd6) ? spit_miso   : (code == 3'd7) ? uart2_tx_out : 1'b0;
    end endgenerate
`else
    // loop and deep images (v54): no emulated sensors
    assign i2c_reg_rdata = 8'd0; assign i2c_status = 32'd0;
    assign emu_fifo_level = 12'd0; assign emu_fifo_ovf = 1'b0; assign emu_fifo_unf = 1'b0;
    assign emu_on = {N{1'b0}}; assign emu_out = {N{1'b0}}; assign emu_oe = {N{1'b0}};
`endif

    // (The on-FPGA raw I2C-bus logic-analyzer capture — i2c_la_capture + its la_capture_buf
    //  SPRAM trace buffer, armed by OP_I2C_LA_START 0x65 / read back via 0x66 — was removed
    //  in v24 to reclaim ~150 LC + an SB_SPRAM256KA block.  The `sensor_la` feature now
    //  samples the two I2C pins through the general deep-LA path (la_psram_capture, OP_LA_
    //  CAPTURE 0x69) and the firmware re-packs them into the same 4-samples/byte layout, so
    //  the server/python I2C decoder is unchanged.)  BUSY for an LA capture is now carried by
    //  the deep-LA path's own `la_busy` (top_v2), so this legacy term is tied off.
    assign i2c_la_busy = 1'b0;

    // ---- Stepper engine ----
    stepper_engine #(.CLK_MHZ(`SYS_CLK_MHZ)) stepper_i (
        .clk(clk), .rst(rst), .start(step_start), .channel(step_channel),
        .steps(step_steps), .delay_us(step_delay),
        .busy(step_busy), .step_channel(step_la_ch), .step_val(step_la_val)
    );

    // ---- SWD bit-bang engine + SPI master (SWDIO / MISO sampled back via la_in[swd_in_ch]) ----
    swd_engine #(.REPLY_AW(9)) swd_i (
        .clk(clk), .rst(rst),
        .arm_stb(swd_arm_stb), .arm_clk_ch(swd_clk_ch_arg), .arm_dio_ch(swd_dio_ch_arg),
        .spi_arm_stb(spi_arm_stb), .spi_sck_ch(spi_sck_ch), .spi_mosi_ch(spi_mosi_ch),
        .spi_miso_ch(spi_miso_ch), .spi_cs_ch(spi_cs_ch_arg), .spi_half(spi_half),
        .spi_flags(spi_flags), .spi_cs_stb(spi_cs_stb), .spi_cs_assert(spi_cs_assert),
        .disarm_stb(swd_disarm_stb),
        .q_cfg_stb(q_cfg_stb), .q_cfg_half(q_cfg_half), .q_cfg_idle(q_cfg_idle), .feed_q(swd_feed_q),
        .feed_begin(swd_feed_begin), .feed_stb(swd_feed_stb), .feed_byte(swd_feed_byte),
        .dio_in(la_in[swd_in_ch]), .scl_in(la_in[swd_cs_ch]), .rd_addr(swd_rd_addr), .rd_data(swd_rd_data),
        .reply_count(swd_reply_count), .armed(swd_armed),
        .spi_mode(spi_mode), .spi_busy(spi_busy), .q_done(q_done), .q_flags(q_flags),
        .clk_ch(swd_clk_ch), .clk_val(swd_clk_val), .clk_oe(swd_clk_oe),
        .dio_ch(swd_dio_ch), .dio_val(swd_dio_val), .dio_oe(swd_dio_oe),
        .in_ch(swd_in_ch), .cs_ch(swd_cs_ch), .cs_val(swd_cs_val), .cs_on(swd_cs_on)
    );

    // ---- LA GPIO bank: arbitration of all the engine drivers onto the pads,
    //      plus the la_in readback that I2C/UART/SWD sample above ----
    la_bank #(.N(N)) la_bank_i (
        .clk(clk), .rst(rst),
        .set_stb(gpio_set_stb), .set_ch(gpio_set_ch), .set_mode(gpio_set_mode),
        .emu_on(emu_on), .emu_out(emu_out), .emu_oe(emu_oe),
        .uart_active(uart_armed), .uart_tx_ch(uart_tx_ch), .uart_tx_val(uart_tx_out),
        .step_active(step_busy), .step_ch(step_la_ch), .step_val(step_la_val),
        .swd_active(swd_armed),
        .swd_clk_ch(swd_clk_ch), .swd_clk_val(swd_clk_val), .swd_clk_oe(swd_clk_oe),
        .swd_dio_ch(swd_dio_ch), .swd_dio_val(swd_dio_val), .swd_dio_oe(swd_dio_oe),
        .swd_cs_active(swd_cs_on), .swd_cs_ch(swd_cs_ch), .swd_cs_val(swd_cs_val),
        .la(la), .la_in(la_in)
    );

    // Live 14-channel LA readback for the v2 deep-PSRAM sampler (la_psram_capture
    // in top_v2 re-synchronizes it into the 48 MHz domain).  Same low-14 readback
    // the on-FPGA la_capture samples; unused on v1.
    assign la_sample = la_in[13:0];

    // The same 14 levels through a 2-flop synchroniser (v35): GPIO_GET reads them and the capture
    // trigger in top_v2 tests them.  Deliberately no power-up init, so these flops are identical to
    // la_psram_capture's own q1/q2 and synthesis can merge the two chains.
    always @(posedge clk) begin
        la_s1 <= la_in[13:0];
        la_s2 <= la_s1;
    end
    assign la_levels = la_s2;

endmodule
