// ============================================================================
// engine_block.v — the shared signal-engine control plane for both bench-pod
// gateware tops (top.v / top_v2.v).
//
// Holds every engine that is identical between the v1 (parallel-ADC) and v2
// (serial-ADC + PSRAM) boards: the SPI command path (spi_slave + cmd_dispatch),
// the LA GPIO bank with its stepper/SWD/I2C/UART drivers, the emulated I2C
// sensor (target + register file) and its raw-bus LA capture, and the shared
// waveform BRAM — together with all the inter-engine wiring (la_bank
// arbitration, the la_in readback taps that feed I2C/UART/SWD, and the
// regfile<->target<->dispatch plumbing).
//
// Each top keeps only its version-specific *analog datapath* (the DAC/ADC front
// end and, for v2, the PSRAM streaming pipeline) and wires it to this block via
// the boundary ports below.  This is a PURE STRUCTURAL extraction: instantiating
// engine_block from each top must yield a netlist byte-for-byte-equivalent to
// the pre-refactor inline instantiations (verified by matching synth cell
// counts — see the P4 refactor notes).
//
// Parameters:
//   N                — LA bank width (12 for v1, 14 for v2).
//   GATEWARE_VERSION — VERSION-command reply byte, passed straight to
//                      cmd_dispatch (v1 historically used cmd_dispatch's own
//                      default of 5; v2 passes 6).  Each top sets it explicitly.
// ============================================================================
`include "sys_config.vh"   // SYS_CLK_MHZ — single source (tools/gen_protocol.py)

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
    wire        swd_clk_val, swd_dio_val, swd_dio_oe;
    wire [N-1:0] la_in;
    wire        i2c_cfg_stb, i2c_cfg_enable, i2c_disable_stb;
    wire [6:0]  i2c_cfg_addr7;  wire [3:0] i2c_cfg_sda_ch, i2c_cfg_scl_ch;
    wire [7:0]  i2c_cfg_trig_reg, i2c_cfg_busy_reg, i2c_cfg_busy_mask;
    wire [15:0] i2c_cfg_conv_us;
    wire        i2c_t_reg_we;  wire [7:0] i2c_t_reg_waddr, i2c_t_reg_wdata, i2c_t_reg_raddr, i2c_t_reg_rdata;
    wire        i2c_reg_we;    wire [7:0] i2c_reg_waddr, i2c_reg_wdata, i2c_reg_raddr, i2c_reg_rdata;
    wire        i2c_armed;     wire [15:0] i2c_xfer_count, i2c_wr_count;
    wire [7:0]  i2c_last_wr_addr, i2c_last_wr_val;
    wire        i2c_sda_drive_low;  wire [3:0] i2c_sda_ch, i2c_scl_ch;
    wire        i2c_scl_in = la_in[i2c_scl_ch];
    wire        i2c_sda_in = la_in[i2c_sda_ch];
    wire        uart_cfg_stb, uart_cfg_enable, uart_disable_stb;
    wire [3:0]  uart_cfg_rx_ch, uart_cfg_tx_ch;  wire [23:0] uart_cfg_div;
    wire        uart_tx_we, uart_rx_re, uart_rx_ovf_clr;
    wire [7:0]  uart_tx_wdata, uart_rx_rdata;  wire [8:0] uart_rx_avail;
    wire        uart_tx_full, uart_tx_empty, uart_rx_overflow, uart_armed;
    wire [3:0]  uart_rx_ch, uart_tx_ch;  wire uart_tx_out;
    wire        uart_rx_in = la_in[uart_rx_ch];

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
        .dac_loop_mode(dac_loop_mode),
        .loop_arm_stb(loop_arm_stb),     .loop_arm_cfg(loop_arm_cfg),
        .loop_src_stb(loop_src_stb),     .loop_src_cfg(loop_src_cfg),
        .loop_inmap_stb(loop_inmap_stb), .loop_inmap_cfg(loop_inmap_cfg),
        .loop_tripped(loop_tripped),
        .cap_start(cap_start), .cap_count(cap_count), .cap_divider(cap_divider),
        .adc_cap_base(adc_cap_base),
        .cap_test_ramp(cap_test_ramp),
        .psram_cs_force(psram_cs_force),
        .trig_ch(trig_ch), .trig_en(trig_en), .trig_edge(trig_edge), .trig_pol(trig_pol),
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
        .i2c_cfg_stb(i2c_cfg_stb), .i2c_cfg_addr7(i2c_cfg_addr7),
        .i2c_cfg_sda_ch(i2c_cfg_sda_ch), .i2c_cfg_scl_ch(i2c_cfg_scl_ch), .i2c_cfg_enable(i2c_cfg_enable),
        .i2c_cfg_trig_reg(i2c_cfg_trig_reg), .i2c_cfg_busy_reg(i2c_cfg_busy_reg),
        .i2c_cfg_busy_mask(i2c_cfg_busy_mask), .i2c_cfg_conv_us(i2c_cfg_conv_us),
        .i2c_disable_stb(i2c_disable_stb),
        .i2c_reg_we(i2c_reg_we), .i2c_reg_waddr(i2c_reg_waddr), .i2c_reg_wdata(i2c_reg_wdata),
        .i2c_reg_raddr(i2c_reg_raddr), .i2c_reg_rdata(i2c_reg_rdata),
        .i2c_armed(i2c_armed), .i2c_xfer_count(i2c_xfer_count), .i2c_wr_count(i2c_wr_count),
        .i2c_last_wr_addr(i2c_last_wr_addr), .i2c_last_wr_val(i2c_last_wr_val),
        .la_cap_start(la_cap_start), .la_cap_count(la_cap_count), .la_cap_divider(la_cap_divider),
        .uart_cfg_stb(uart_cfg_stb), .uart_cfg_rx_ch(uart_cfg_rx_ch), .uart_cfg_tx_ch(uart_cfg_tx_ch),
        .uart_cfg_div(uart_cfg_div), .uart_cfg_enable(uart_cfg_enable), .uart_disable_stb(uart_disable_stb),
        .uart_tx_we(uart_tx_we), .uart_tx_wdata(uart_tx_wdata),
        .uart_rx_re(uart_rx_re), .uart_rx_rdata(uart_rx_rdata), .uart_rx_ovf_clr(uart_rx_ovf_clr),
        .uart_rx_avail(uart_rx_avail), .uart_tx_full(uart_tx_full),
        .uart_tx_empty(uart_tx_empty), .uart_rx_overflow(uart_rx_overflow), .uart_armed(uart_armed)
    );

    // ---- UART proxy engine ----
    uart_engine uart_i (
        .clk(clk), .rst(rst),
        .cfg_stb(uart_cfg_stb), .cfg_rx_ch(uart_cfg_rx_ch), .cfg_tx_ch(uart_cfg_tx_ch),
        .cfg_div(uart_cfg_div), .cfg_enable(uart_cfg_enable), .disable_stb(uart_disable_stb),
        .rx_ch(uart_rx_ch), .tx_ch(uart_tx_ch), .armed(uart_armed),
        .tx_we(uart_tx_we), .tx_wdata(uart_tx_wdata), .tx_full(uart_tx_full), .tx_empty(uart_tx_empty),
        .rx_re(uart_rx_re), .rx_rdata(uart_rx_rdata), .rx_avail(uart_rx_avail),
        .rx_overflow(uart_rx_overflow), .rx_ovf_clr(uart_rx_ovf_clr),
        .rx_in(uart_rx_in), .tx_out(uart_tx_out)
    );

    // ---- Emulated I2C sensor register file (256 bytes) ----
    i2c_regfile i2c_regfile_i (
        .clk(clk),
        .i2c_we(i2c_t_reg_we), .i2c_waddr(i2c_t_reg_waddr), .i2c_wdata(i2c_t_reg_wdata),
        .i2c_raddr(i2c_t_reg_raddr), .i2c_rdata(i2c_t_reg_rdata),
        .spi_we(i2c_reg_we), .spi_waddr(i2c_reg_waddr), .spi_wdata(i2c_reg_wdata),
        .spi_raddr(i2c_reg_raddr), .spi_rdata(i2c_reg_rdata)
    );

    // ---- Generic I2C target (slave) ----
    i2c_target #(.CLK_MHZ(`SYS_CLK_MHZ)) i2c_target_i (
        .clk(clk), .rst(rst),
        .cfg_stb(i2c_cfg_stb), .cfg_addr7(i2c_cfg_addr7),
        .cfg_sda_ch(i2c_cfg_sda_ch), .cfg_scl_ch(i2c_cfg_scl_ch), .cfg_enable(i2c_cfg_enable),
        .cfg_trig_reg(i2c_cfg_trig_reg), .cfg_busy_reg(i2c_cfg_busy_reg),
        .cfg_busy_mask(i2c_cfg_busy_mask), .cfg_conv_us(i2c_cfg_conv_us), .disable_stb(i2c_disable_stb),
        .sda_ch(i2c_sda_ch), .scl_ch(i2c_scl_ch), .scl_in(i2c_scl_in), .sda_in(i2c_sda_in),
        .sda_drive_low(i2c_sda_drive_low),
        .reg_we(i2c_t_reg_we), .reg_waddr(i2c_t_reg_waddr), .reg_wdata(i2c_t_reg_wdata),
        .reg_raddr(i2c_t_reg_raddr), .reg_rdata(i2c_t_reg_rdata),
        .armed(i2c_armed), .xfer_count(i2c_xfer_count), .wr_count(i2c_wr_count),
        .last_wr_addr(i2c_last_wr_addr), .last_wr_val(i2c_last_wr_val)
    );

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

    // ---- SWD bit-bang engine (SWDIO sampled back via la_in[swd_dio_ch]) ----
    swd_engine #(.REPLY_AW(9)) swd_i (
        .clk(clk), .rst(rst),
        .arm_stb(swd_arm_stb), .arm_clk_ch(swd_clk_ch_arg), .arm_dio_ch(swd_dio_ch_arg),
        .disarm_stb(swd_disarm_stb),
        .feed_begin(swd_feed_begin), .feed_stb(swd_feed_stb), .feed_byte(swd_feed_byte),
        .dio_in(la_in[swd_dio_ch]), .rd_addr(swd_rd_addr), .rd_data(swd_rd_data),
        .reply_count(swd_reply_count), .armed(swd_armed),
        .clk_ch(swd_clk_ch), .clk_val(swd_clk_val),
        .dio_ch(swd_dio_ch), .dio_val(swd_dio_val), .dio_oe(swd_dio_oe)
    );

    // ---- LA GPIO bank: arbitration of all the engine drivers onto the pads,
    //      plus the la_in readback that I2C/UART/SWD sample above ----
    la_bank #(.N(N)) la_bank_i (
        .clk(clk), .rst(rst),
        .set_stb(gpio_set_stb), .set_ch(gpio_set_ch), .set_mode(gpio_set_mode),
        .i2c_active(i2c_armed), .i2c_sda_ch(i2c_sda_ch), .i2c_sda_drive_low(i2c_sda_drive_low),
        .uart_active(uart_armed), .uart_tx_ch(uart_tx_ch), .uart_tx_val(uart_tx_out),
        .step_active(step_busy), .step_ch(step_la_ch), .step_val(step_la_val),
        .swd_active(swd_armed),
        .swd_clk_ch(swd_clk_ch), .swd_clk_val(swd_clk_val),
        .swd_dio_ch(swd_dio_ch), .swd_dio_val(swd_dio_val), .swd_dio_oe(swd_dio_oe),
        .la(la), .la_in(la_in)
    );

    // Live 14-channel LA readback for the v2 deep-PSRAM sampler (la_psram_capture
    // in top_v2 re-synchronizes it into the 48 MHz domain).  Same low-14 readback
    // the on-FPGA la_capture samples; unused on v1.
    assign la_sample = la_in[13:0];

    // The same 14 levels through a 2-flop synchroniser (v35): GPIO_GET reads them and the capture
    // trigger in top_v2 tests them.  Deliberately no power-up init, so these flops are identical to
    // la_psram_capture's own q1/q2 and synthesis can merge the two chains.
    reg [13:0] la_s1, la_s2;
    always @(posedge clk) begin
        la_s1 <= la_in[13:0];
        la_s2 <= la_s1;
    end
    assign la_levels = la_s2;

endmodule
