// ============================================================================
// cmd_dispatch.v — command FSM sitting between spi_slave and the engines.
//                  Decodes one SPI transaction at a time.
//
// Protocol:
//   First byte received after CSn↓ is CMD.  Subsequent bytes are arguments
//   or payload depending on CMD.  Responses are presented on tx_byte so the
//   spi_slave shifts them out on MISO during the next read cycles.
//   CSn↑ resets back to IDLE.
//
// Commands (must match rp2350 signal_engine.c):
//   -- signal engine (DAC/ADC) --
//   0x01 PING            → returns 0xA5
//   0x02 VERSION         → returns 1 byte version
//   0x03 STATUS          → returns 1 byte flags
//   0x10 LOAD_WAVE       [len_lo][len_hi][N data bytes]
//   0x11 START_DAC       [period_lo][period_hi][div_lo][div_hi]
//   0x12 STOP_DAC
//   0x20 START_CAPTURE   [count_lo][count_hi][div_lo][div_hi]
//   0x22 READ_CAPTURE    [len_lo][len_hi] → returns N bytes
//   0x30 START_MEASURE   [count_lo][count_hi][div_lo][div_hi]
//   0x31 START_CORRELATED [adc_cnt(2)][adc_div(2)][la_cnt(2)][la_div(2)]
//                         ADC + I2C-LA merged into one tagged/timestamped PSRAM
//                         stream (capture_merge -> spram_ring -> psram_writer;
//                         record format in sim/capture_format.h)
//   0x33 SET_TRIGGER     [channel][mode][flags]   (v35 capture trigger, see top_v2.v)
//   0x34 TRIGGER_STATUS  → [status: bit0 waiting, bit1 fired since the arm]
//   -- logic-analyzer GPIO bank (stepper + static, see la_bank/stepper_engine) --
//   0x40 GPIO_SET        [channel][mode]            (mode 0=low,1=high,2=high-Z)
//   0x41 GPIO_STEP       [channel][steps(2)][delay_us(2)]   (16-bit each, LE)
//   0x43 GPIO_GET        → 2 bytes LE: synchronised LA1..LA12 levels (v35)
//   -- SWD bit-bang (see swd_engine) --
//   0x50 SWD_ARM         [swclk_ch][swdio_ch][nreset_ch]   (nreset 0xFF = none)
//   0x51 SWD_FEED        [len_lo][len_hi][N remote_bitbang bytes]
//   0x52 SWD_READ        [len_lo][len_hi] → returns N sample bytes ('0'/'1')
//   0x53 SWD_DISARM
//   0x54 SWD_STATUS      → returns reply byte count of the last feed (low 8 bits)
//   -- emulated I2C sensor (generic target + register file, see i2c_target) --
//   0x60 I2C_SENSOR_CONFIG  [addr7][sda_ch][scl_ch][flags][trig_reg]
//                           [busy_reg][busy_mask][conv_lo][conv_hi]
//   0x61 I2C_SENSOR_DISABLE
//   0x62 I2C_LOAD_REGS      [start_addr][len_lo][len_hi][N data bytes]
//   0x63 I2C_READ_REGS      [start_addr][len_lo][len_hi] → returns N bytes
//   0x64 I2C_SENSOR_STATUS  → 7 bytes: armed, xfer_lo, xfer_hi, wr_lo, wr_hi,
//                              last_wr_addr, last_wr_val
//   (0x65 I2C_LA_START / 0x66 I2C_LA_READ removed v24 — on-FPGA I2C-LA retired; sensor_la
//    now uses the deep LA_CAPTURE 0x69 path + a firmware re-pack)
//   -- UART proxy (soft UART on two LA channels, see uart_engine) --
//   0x70 UART_CONFIG        [rx_ch][tx_ch][div_lo][div_mid][div_hi][flags]
//   0x71 UART_DISABLE
//   0x72 UART_WRITE         [len_lo][len_hi][N data bytes]   → TX FIFO
//   0x73 UART_READ          [len_lo][len_hi] → returns N bytes from RX FIFO
//   0x74 UART_STATUS        → 3 bytes: rx_avail_lo, rx_avail_hi, flags
// ============================================================================

module cmd_dispatch #(
    parameter ADDR_W           = 12,
    parameter REPLY_AW         = 9,
    parameter GATEWARE_VERSION = 8'd5,  // fallback only — each top overrides this
    parameter [7:0] FEATURES = 8'h01    // FPGA_FEATURES: bit0=closed-loop, bit1=deep-replay
                                        // via engine_block (v1 -> 5, v2 -> 9)
)(
    input  wire              clk,
    input  wire              rst,

    // SPI slave interface
    input  wire [7:0]        rx_byte,
    input  wire              rx_valid,
    output reg  [7:0]        tx_byte,
    input  wire              cs_active,

    // Waveform BRAM write port (LOAD_WAVE drives this)
    output reg               wave_we,
    output reg  [ADDR_W-1:0] wave_waddr,
    output reg  [7:0]        wave_wdata,

    // DAC engine control
    output reg               dac_start,
    output reg               dac_stop,
    // Co-trigger arm (OP_DAC_ARM_ON_CAPTURE 0x19, v2 >= v27): 1-cycle strobe that tells the top
    // to defer the NEXT dac_start until the next capture arm (dac_pend), so the DAC and the
    // capture start on the same cycle.  See top_v2.v.
    output reg               dac_cotrig_stb,
    output reg  [ADDR_W:0]   dac_period,    // 0..1<<ADDR_W (needs ADDR_W+1 bits)
    output reg  [15:0]       dac_divider,

    // DEEP DAC replay from PSRAM (OP_START_DAC_PSRAM 0x13, v2 >= v17).  dac_psram_mode
    // is a LEVEL: 1 from START_DAC_PSRAM until STOP_DAC.  The top-level latches base/
    // len at the arm and streams the waveform out of PSRAM (see dac_psram_reader).
    output reg               dac_psram_mode,
    output reg  [23:0]       dac_psram_base,   // byte base in PSRAM
    output reg  [23:0]       dac_psram_len,    // waveform length in SAMPLES (top => *2 bytes)

    // Capture-tied DAC auto-stop threshold (OP_SET_DAC_STOP_AFTER 0x14, v2 >= v21): the number
    // of 24 MHz clk cycles after a capture's t0 at which the top cuts a concurrently-running
    // DAC.  0 = disarmed.  Persistent (holds until the next SET) so the top can snapshot it at
    // the capture arm.
    output reg  [31:0]       dac_stop_after,

    // In-fabric DAC control loop (OP_START_DAC_LOOP 0x15, v2 >= v23).  dac_loop_mode is a
    // LEVEL (1 while the loop runs); cleared by STOP_DAC or any normal DAC start.  The
    // params are latched at arm.  The curve is the LOAD_WAVE BRAM, indexed by the loop input.
    output reg               dac_loop_mode,
    output reg  [15:0]       dac_loop_k,      // damping coefficient, Q15
    output reg  [15:0]       dac_loop_vmin,   // output clamp low
    output reg  [15:0]       dac_loop_vmax,   // output clamp high
    output reg  [15:0]       dac_loop_tick,   // control period in clk48 cycles

    // Loop INPUT SOURCE (OP_DAC_LOOP_SRC 0x1A, v2 >= v29).  Separate from the arm opcode so
    // the host can step the input of a RUNNING loop (the open-loop bring-up test: hold a
    // point, meter the DAC, move on) without re-arming or re-uploading the curve.  Persistent
    // across arms; 0 = live ADC (closed loop) is the power-on default, so a client that never
    // sends this behaves exactly as it did before v29.
    output reg  [1:0]        dac_loop_src,    // 0 = ADC, 1 = fixed, 2 = time sweep
    output reg  [15:0]       dac_loop_in,     // fixed input value / sweep start
    output reg  [15:0]       dac_loop_step,   // sweep increment per tick

    // Loop INPUT MAP + safety bounds (OP_DAC_LOOP_INMAP 0x1C, v2 >= v30).  Also persistent
    // across arms and separate from the arm opcode, for the same reason: the map describes
    // the BENCH (what the ADC is wired to), which does not change when a curve does.  All
    // zero = the v29 behaviour (legacy `in >> SHIFT` index, no trip, no slew bound), so a
    // client that never sends this is bit-for-bit unchanged.
    output reg  [15:0]       dac_loop_in_zero,
    output reg  [15:0]       dac_loop_in_gain,   // signed Q15
    output reg  [10:0]       dac_loop_in_trip,   // trip threshold as a curve index
    output reg               dac_loop_map_en,
    output reg               dac_loop_trip_en,

    // ADC engine control
    output reg               cap_start,
    output wire [23:0]       cap_count,     // 24-bit: deep ADC capture spanning the multi-MB PSRAM ADC region (up to 2,097,152 samples). Decoupled from ADDR_W (the DAC/BRAM width). Mirrors the 24-bit deep-LA count.
    output reg  [15:0]       cap_divider,
    // Runtime ADC PSRAM region base for dynamic tri-capture zone allocation
    // (SET_CAPTURE_BASES 0x32, v2 >= v22).  Latched before a CAPTURE; defaults to the
    // legacy fixed 4 MB base so behaviour is unchanged if never set.  (LA is ALWAYS
    // packed at 0 by the firmware allocator, so its base stays a top-level constant —
    // the 0x32 payload still carries la_base but the gateware ignores it.)
    output reg  [23:0]       adc_cap_base,
    // CAPTURE_TEST (0x23): when 1, the capture datapath sources a known +0x0101
    // ramp instead of the real ADC, so the whole capture->writer->PSRAM->readback
    // path is verifiable end-to-end (cap-selftest).  Persistent (set until cleared).
    output reg               cap_test_ramp,
    // PSRAM_CS (0x25) boot self-test: force the iCE40's PSRAM /CS output LOW so the
    // STM32 can check (PE4 as input) whether the iCE40 actually reaches the shared
    // /CE net — pins the "iCE40 can't write PSRAM" fault on the pad45->/CE joint.
    output reg               psram_cs_force,
    // Capture trigger (OP_SET_TRIGGER 0x33, v2 >= v35).  Persistent; the mode byte is decoded
    // here at SET time so the top's per-cycle test is one mux and two compares.  trig_en = 0 is
    // an untriggered capture (the <= v34 behaviour).  trig_pol = the level that fires (1 for
    // rising/high); trig_edge = the previous synced level must also be the opposite one.
    output reg  [3:0]        trig_ch,
    output reg               trig_en,
    output reg               trig_edge,
    output reg               trig_pol,
    input  wire              trig_wait,     // TRIGGER_STATUS (0x34) bit0: armed, condition not seen yet
    input  wire              trig_fired,    // TRIGGER_STATUS (0x34) bit1: fired since the last arm
    // Status inputs (for STATUS cmd)
    input  wire              dac_running,
    input  wire              cap_busy,
    input  wire              cap_done,
    // Sticky "a PSRAM-path capture lost bytes" flag (v2: OR of the ADC/correlated/
    // deep-LA writer + ring overflow diagnostics; cleared when the next capture is
    // armed).  v1 ties this 0.  Surfaced as STATUS bit 5 so firmware can tell a
    // truncated/corrupt capture from a clean one.
    input  wire              cap_overflow,
    // Latched control-loop over-range trip (v30).  Surfaced as STATUS bit 6 rather than a
    // new opcode: it is one bit, it is a SAFETY state, and STATUS is the thing a host
    // already polls.  Cleared when the loop is disarmed.
    input  wire              loop_tripped,

    // ADC_PROBE (0x21) diagnostic: the LIVE 16-bit ADC sample straight off the
    // free-running adc_mcp33131 engine (24 MHz domain, same as this FSM).  Read
    // directly over SPI, bypassing the capture orchestrator, the 24->48 MHz CDC,
    // and the PSRAM datapath — isolates whether a bad ADC read comes from the
    // engine/silicon or from the streaming path.
    input  wire [15:0]       adc_dbg_sample,
    input  wire [15:0]       dac_loop_v_dbg,   // DAC_PROBE (0x16): live loop output
    input  wire [15:0]       dac_loop_in_dbg,  // DAC_LOOP_IN_PROBE (0x1B): live loop INPUT
    input  wire [11:0]       la_levels,        // GPIO_GET (0x43, v35): 2-flop-synced LA1..LA12

    // ---- LA static GPIO control (GPIO_SET) ----
    output reg               gpio_set_stb,
    output wire [3:0]        gpio_set_ch,
    output wire [1:0]        gpio_set_mode,

    // SET_LED (0x42): persistent status-LED mask driven by the host — bit0=green,
    // bit1=yellow, bit2=red.  Not a 1-cycle strobe; holds until the next SET_LED.
    output reg  [7:0]        led_ctrl,

    // ---- stepper engine (GPIO_STEP) ----
    output reg               step_start,
    output wire [3:0]        step_channel,
    output wire [15:0]       step_steps,
    output wire [15:0]       step_delay,
    input  wire              step_busy,

    // ---- SWD engine ----
    output reg               swd_arm_stb,
    output wire [3:0]        swd_clk_ch,
    output wire [3:0]        swd_dio_ch,
    output wire [3:0]        swd_nrst_ch,
    output wire              swd_nrst_present,
    output reg               swd_disarm_stb,
    output reg               swd_feed_begin,
    output reg               swd_feed_stb,
    output reg  [7:0]        swd_feed_byte,
    output reg  [REPLY_AW-1:0] swd_rd_addr,
    input  wire [7:0]        swd_rd_data,
    input  wire              swd_armed,
    input  wire [15:0]       swd_reply_count,

    // ---- emulated I2C sensor: configuration ----
    output reg               i2c_cfg_stb,
    output wire [6:0]        i2c_cfg_addr7,
    output wire [3:0]        i2c_cfg_sda_ch,
    output wire [3:0]        i2c_cfg_scl_ch,
    output wire              i2c_cfg_enable,
    output wire [7:0]        i2c_cfg_trig_reg,
    output wire [7:0]        i2c_cfg_busy_reg,
    output wire [7:0]        i2c_cfg_busy_mask,
    output wire [15:0]       i2c_cfg_conv_us,
    output reg               i2c_disable_stb,

    // ---- emulated I2C sensor: register file (SPI/load+readback port) ----
    output reg               i2c_reg_we,
    output reg  [7:0]        i2c_reg_waddr,
    output reg  [7:0]        i2c_reg_wdata,
    output reg  [7:0]        i2c_reg_raddr,
    input  wire [7:0]        i2c_reg_rdata,

    // ---- emulated I2C sensor: status ----
    input  wire              i2c_armed,
    input  wire [15:0]       i2c_xfer_count,
    input  wire [15:0]       i2c_wr_count,
    input  wire [7:0]        i2c_last_wr_addr,
    input  wire [7:0]        i2c_last_wr_val,

    // (The on-FPGA I2C-bus logic-analyzer capture — OP_I2C_LA_START 0x65 / OP_I2C_LA_READ
    //  0x66 → i2c_la_capture + la_capture_buf — was removed in v24 to reclaim ~150 LC + an
    //  SPRAM block.  The `sensor_la` feature now samples the two I2C pins via the general
    //  deep-LA path (OP_LA_CAPTURE 0x69) and the firmware re-packs them into the same
    //  4-samples/byte layout, so the server/python decoder is unchanged.)

    // ---- deep multi-channel LA capture into PSRAM (LA_CAPTURE, v2 only).  Its
    //      own 16-bit sample count (depth bounded by PSRAM, not the 4 KB trace
    //      buffer) + divider; top_v2 streams it through psram_writer.  v1 leaves
    //      these outputs unconnected. ----
    output reg               la_cap_start,
    output wire [23:0]       la_cap_count,     // 24-bit: deep LA up to the full 8 MB PSRAM
    output reg  [15:0]       la_cap_divider,

    // ---- UART proxy: configuration ----
    output reg               uart_cfg_stb,
    output wire [3:0]        uart_cfg_rx_ch,
    output wire [3:0]        uart_cfg_tx_ch,
    output wire [23:0]       uart_cfg_div,
    output wire              uart_cfg_enable,
    output reg               uart_disable_stb,

    // ---- UART proxy: TX FIFO write / RX FIFO read ----
    output reg               uart_tx_we,
    output reg  [7:0]        uart_tx_wdata,
    output reg               uart_rx_re,
    input  wire [7:0]        uart_rx_rdata,
    output reg               uart_rx_ovf_clr,

    // ---- UART proxy: status ----
    input  wire [8:0]        uart_rx_avail,
    input  wire              uart_tx_full,
    input  wire              uart_tx_empty,
    input  wire              uart_rx_overflow,
    input  wire              uart_armed
);

    // Opcodes — generated from tools/gen_protocol.py (single source of truth).
    `include "cmd_opcodes.vh"

    // ------------------------------------------------------------------------
    // FSM state-flow.  One SPI transaction (CSn low..high) walks one path:
    //
    //   S_IDLE ── opcode byte ──┬─ instant (PING/VERSION/STATUS/STOP_DAC/
    //                           │   *_DISABLE/SWD_DISARM) ─► S_DONE
    //                           │
    //                           ├─ fixed-len collect (START_DAC/CAPTURE/MEASURE,
    //                           │   I2C_LA_START, GPIO_SET/STEP, SWD_ARM,
    //                           │   I2C_CONFIG, UART_CONFIG, CAPTURE)
    //                           │   ─► S_COLLECT (decode on last byte) ─► S_DONE
    //                           │
    //                           ├─ length-prefixed stream ─► S_READ_LEN0→1, then
    //                           │   IN  : S_LOAD_DATA / S_SWD_FEED / S_REG_LOAD /
    //                           │         S_UART_WRITE   (write each rx byte)
    //                           │   OUT : S_READ_CAP / S_SWD_READ / S_REG_READ /
    //                           │         S_LA_READ / S_UART_READ  (present each
    //                           │         tx byte; read paths preload byte 0 in
    //                           │         S_READ_LEN1 to hide BRAM/FIFO latency)
    //                           │
    //                           └─ status stream (I2C_STATUS/UART_STATUS) ─►
    //                               S_I2C_STATUS / S_UART_STATUS (emit the block)
    //
    //   any state ── CSn↑ (cs_release) ─► S_IDLE.   S_DONE parks until CSn↑.
    //
    // The IN/OUT/status streaming states share the arg_count (addressing) +
    // arg_rem (down-counter) machinery and the `last_byte` terminal; they differ
    // only in source/sink.  See tb_dispatch_args / tb_measure for coverage.
    // ------------------------------------------------------------------------
    localparam S_IDLE       = 5'd0;
    localparam S_READ_LEN0  = 5'd1;
    localparam S_READ_LEN1  = 5'd2;
    localparam S_LOAD_DATA  = 5'd3;
    localparam S_DONE       = 5'd8;
    localparam S_COLLECT    = 5'd10;  // fixed-length arg collector (GPIO_SET/STEP, SWD_ARM, I2C_CONFIG)
    localparam S_SWD_FEED   = 5'd11;  // stream remote_bitbang bytes to swd_engine
    localparam S_SWD_READ   = 5'd12;  // stream sample bytes back from swd_engine
    localparam S_REG_ADDR   = 5'd13;  // capture start_addr for I2C_LOAD_REGS/READ_REGS
    localparam S_REG_LOAD   = 5'd14;  // stream bytes into the I2C register file
    localparam S_REG_READ   = 5'd15;  // stream register file back out
    localparam S_I2C_STATUS = 5'd16;  // stream the 7-byte I2C status block
    // (S_LA_READ 5'd17 retired with OP_I2C_LA_READ in v24)
    localparam S_UART_WRITE = 5'd18;  // stream bytes into the UART TX FIFO
    localparam S_UART_READ  = 5'd19;  // stream bytes back from the UART RX FIFO
    localparam S_UART_STATUS= 5'd20;  // stream the 3-byte UART status block
    localparam S_ADC_PROBE_HI=5'd21;  // ADC_PROBE: emit the high byte of the live sample

    localparam I2C_STATUS_LEN  = 16'd7;
    localparam UART_STATUS_LEN = 16'd3;

    reg [4:0]  state;
    reg [7:0]  current_cmd;
    reg [15:0] arg_len;     // payload byte count or response byte count
    reg [11:0] arg_count;   // bytes consumed in current payload phase (addressing).
                            // 12-bit: the WIDEST address use is the 4 KB wave/LA buffers
                            // ([ADDR_W-1:0]=12); longer streams (UART up to 64 KB) advance
                            // it too but never read it as an address, and arg_rem (16-bit)
                            // is the sole terminal, so a 12-bit wrap is harmless.
    reg [15:0] arg_rem;     // bytes still to process (= arg_len - arg_count).
                            // Down-counter loaded with arg_len at each payload
                            // entry and decremented in lockstep with arg_count;
                            // replaces the per-state `arg_count + 1 >= arg_len`
                            // 16-bit add+magnitude-compare carry chain that used
                            // to cap fmax (see last_byte below).
    reg [7:0]  arg_buf [0:8];   // 9 slots: the deepest S_COLLECT payload is the 10-byte
                                // OP_CAPTURE, which stores bytes 0..8 here (byte 9 = rx_byte).
    // Capture sample counts: top_v2 reads cap_count only with cap_start (producer load, run_adc)
    // and la_cap_count only with la_cap_start (la_psram_capture load, run_la), so they are also
    // decoded straight off the payload of the command that pulsed the strobe.
    assign cap_count         = (current_cmd == OP_CAPTURE) ? {arg_buf[2], arg_buf[1], arg_buf[0]} : {8'h00, arg_buf[1], arg_buf[0]};
    assign la_cap_count      = (current_cmd == OP_CAPTURE) ? {arg_buf[7], arg_buf[6], arg_buf[5]} : {arg_buf[2], arg_buf[1], arg_buf[0]};
    // Strobe-qualified config fields (GPIO_SET / GPIO_STEP / SWD_ARM / I2C_CONFIG / UART_CONFIG)
    // are WIRES straight off the collected payload, not registered copies.  Every consumer
    // (la_bank, stepper_engine, swd_engine, i2c_target, uart_engine) samples them ONLY in the
    // 1-cycle strobe after the last payload byte, when arg_buf[] and rx_byte (held by spi_slave
    // until the next byte completes, >= 16 clk later) still carry this command's bytes.  A
    // registered copy cost 1 LC per bit for ~145 bits.
    assign gpio_set_ch       = arg_buf[0][3:0];
    assign gpio_set_mode     = rx_byte[1:0];
    assign step_channel      = arg_buf[0][3:0];
    assign step_steps        = {arg_buf[2], arg_buf[1]};
    assign step_delay        = {rx_byte, arg_buf[3]};
    assign swd_clk_ch        = arg_buf[0][3:0];
    assign swd_dio_ch        = arg_buf[1][3:0];
    assign swd_nrst_ch       = rx_byte[3:0];
    assign swd_nrst_present  = (rx_byte != 8'hFF);
    assign i2c_cfg_addr7     = arg_buf[0][6:0];
    assign i2c_cfg_sda_ch    = arg_buf[1][3:0];
    assign i2c_cfg_scl_ch    = arg_buf[2][3:0];
    assign i2c_cfg_enable    = arg_buf[3][0];
    assign i2c_cfg_trig_reg  = arg_buf[4];
    assign i2c_cfg_busy_reg  = arg_buf[5];
    assign i2c_cfg_busy_mask = arg_buf[6];
    assign i2c_cfg_conv_us   = {rx_byte, arg_buf[7]};
    assign uart_cfg_rx_ch    = arg_buf[0][3:0];
    assign uart_cfg_tx_ch    = arg_buf[1][3:0];
    assign uart_cfg_div      = {arg_buf[4], arg_buf[3], arg_buf[2]};
    assign uart_cfg_enable   = rx_byte[0];
    reg [7:0]  reg_base;    // I2C_LOAD_REGS/READ_REGS start address
    reg [15:0] adc_dbg_latch;  // ADC_PROBE: sample latched at opcode so both bytes
                               // come from ONE conversion (engine free-runs)

    // Terminal predicate: the byte being processed this beat is the last of the
    // payload.  A shallow zero-compare on the down-counter (one shared net for
    // all 12 streaming/collect states) instead of an open-coded carry chain.
    //   arg_count + 1 >= arg_len   <=>   arg_rem <= 1   (=  last_byte)
    //   arg_count + 1 <  arg_len   <=>   arg_rem >  1   (= !last_byte)
    wire last_byte = (arg_rem <= 16'd1);

    // I2C status block: byte at a given index (combinational, no BRAM latency).
    function [7:0] i2c_status_byte;
        input [2:0] idx;
        begin
            case (idx)
                3'd0:    i2c_status_byte = {7'b0, i2c_armed};
                3'd1:    i2c_status_byte = i2c_xfer_count[7:0];
                3'd2:    i2c_status_byte = i2c_xfer_count[15:8];
                3'd3:    i2c_status_byte = i2c_wr_count[7:0];
                3'd4:    i2c_status_byte = i2c_wr_count[15:8];
                3'd5:    i2c_status_byte = i2c_last_wr_addr;
                default: i2c_status_byte = i2c_last_wr_val;
            endcase
        end
    endfunction

    // UART status block: rx_avail(2) + flags.  flags bit0=tx_full, bit1=tx_empty,
    // bit2=rx_overflow, bit3=armed.
    function [7:0] uart_status_byte;
        input [1:0] idx;
        begin
            case (idx)
                2'd0:    uart_status_byte = uart_rx_avail[7:0];
                2'd1:    uart_status_byte = {7'b0, uart_rx_avail[8]};
                default: uart_status_byte = {4'b0, uart_armed, uart_rx_overflow,
                                             uart_tx_empty, uart_tx_full};
            endcase
        end
    endfunction

    // Detect CS deassert as state reset
    reg cs_active_d;
    wire cs_release = (cs_active_d && !cs_active);

    integer i;

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            current_cmd <= 8'h00;
            arg_len     <= 16'd0;
            arg_count   <= 16'd0;
            arg_rem     <= 16'd0;
            for (i = 0; i < 9; i = i + 1) arg_buf[i] <= 8'h00;

            tx_byte     <= 8'h00;
            wave_we     <= 1'b0;
            wave_waddr  <= {ADDR_W{1'b0}};
            wave_wdata  <= 8'h00;

            dac_start   <= 1'b0;
            dac_stop    <= 1'b0;
            dac_cotrig_stb <= 1'b0;
            dac_period  <= {(ADDR_W+1){1'b0}};
            dac_divider <= 16'd2;
            dac_psram_mode <= 1'b0;
            dac_psram_base <= 24'd0;
            dac_psram_len  <= 24'd0;
            dac_stop_after <= 32'd0;
            dac_loop_mode <= 1'b0;
            dac_loop_k    <= 16'd0;     dac_loop_vmin <= 16'd0;
            dac_loop_vmax <= 16'hFFFF;  dac_loop_tick <= 16'd64;
            dac_loop_src  <= 2'd0;      /* live ADC: the pre-v29 behaviour */
            dac_loop_in   <= 16'd0;     dac_loop_step <= 16'd0;
            dac_loop_in_zero   <= 16'd0; dac_loop_in_gain   <= 16'd0;
            dac_loop_in_trip   <= 11'd0;
            dac_loop_map_en    <= 1'b0;  dac_loop_trip_en   <= 1'b0;
            cap_start     <= 1'b0;
            cap_divider   <= 16'd2;
            adc_cap_base  <= 24'h400000;   /* legacy fixed map until SET_CAPTURE_BASES */
            cap_test_ramp <= 1'b0;   /* persistent — reset only, not per-cycle */
            psram_cs_force <= 1'b0;
            trig_ch   <= 4'd0;  trig_en  <= 1'b0;   /* untriggered until SET_TRIGGER */
            trig_edge <= 1'b0;  trig_pol <= 1'b0;
            cs_active_d <= 1'b0;

            gpio_set_stb     <= 1'b0;
            led_ctrl         <= 8'd0;   /* persistent — reset only, not per-cycle */
            step_start       <= 1'b0;
            swd_arm_stb      <= 1'b0;
            swd_disarm_stb   <= 1'b0;
            swd_feed_begin   <= 1'b0;
            swd_feed_stb     <= 1'b0;
            swd_feed_byte    <= 8'h00;
            swd_rd_addr      <= {REPLY_AW{1'b0}};

            reg_base          <= 8'h00;
            i2c_cfg_stb       <= 1'b0;
            i2c_disable_stb   <= 1'b0;
            i2c_reg_we        <= 1'b0;
            i2c_reg_waddr     <= 8'h00;
            i2c_reg_wdata     <= 8'h00;
            i2c_reg_raddr     <= 8'h00;
            la_cap_start      <= 1'b0;
            la_cap_divider    <= 16'd2;

            uart_cfg_stb      <= 1'b0;
            uart_disable_stb  <= 1'b0;
            uart_tx_we        <= 1'b0;
            uart_tx_wdata     <= 8'h00;
            uart_rx_re        <= 1'b0;
            uart_rx_ovf_clr   <= 1'b0;
        end else begin
            // 1-cycle defaults
            wave_we        <= 1'b0;
            dac_start      <= 1'b0;
            dac_stop       <= 1'b0;
            dac_cotrig_stb <= 1'b0;
            cap_start      <= 1'b0;
            gpio_set_stb   <= 1'b0;
            step_start     <= 1'b0;
            swd_arm_stb    <= 1'b0;
            swd_disarm_stb <= 1'b0;
            swd_feed_begin <= 1'b0;
            swd_feed_stb   <= 1'b0;
            i2c_cfg_stb     <= 1'b0;
            i2c_disable_stb <= 1'b0;
            i2c_reg_we      <= 1'b0;
            la_cap_start    <= 1'b0;
            uart_cfg_stb     <= 1'b0;
            uart_disable_stb <= 1'b0;
            uart_tx_we       <= 1'b0;
            uart_rx_re       <= 1'b0;
            uart_rx_ovf_clr  <= 1'b0;

            cs_active_d <= cs_active;

            // CSn↑ resets the FSM
            if (cs_release) begin
                state     <= S_IDLE;
                arg_count <= 16'd0;
                arg_rem   <= 16'd0;
                tx_byte   <= 8'h00;
            end else if (rx_valid) begin
                case (state)
                    S_IDLE: begin
                        current_cmd <= rx_byte;
                        arg_count   <= 16'd0;
                        case (rx_byte)
                            OP_PING:    begin tx_byte <= 8'hA5;            state <= S_DONE; end
                            OP_VERSION: begin tx_byte <= GATEWARE_VERSION; state <= S_DONE; end
                            OP_FPGA_FEATURES: begin tx_byte <= FEATURES; state <= S_DONE; end
                            OP_STATUS:  begin
                                tx_byte <= { 1'b0, loop_tripped, cap_overflow, swd_armed,
                                             step_busy, cap_done, cap_busy, dac_running };
                                state   <= S_DONE;
                            end
                            OP_STOP_DAC: begin
                                dac_stop       <= 1'b1;
                                dac_psram_mode <= 1'b0;   // leave PSRAM-streaming replay
                                dac_loop_mode  <= 1'b0;   // leave closed-loop control
                                state          <= S_DONE;
                            end
                            // Co-trigger: defer the NEXT DAC start until the next capture
                            // arm (top latches dac_pend).  Instant, no args.  The DAC start
                            // opcode that follows still latches period/divider/base/len and
                            // primes the deep reader — only the engine-start pulse is held.
                            OP_DAC_ARM_ON_CAPTURE: begin
                                dac_cotrig_stb <= 1'b1;
                                state          <= S_DONE;
                            end
                            OP_LOAD_WAVE:    state <= S_READ_LEN0;
                            // 4-byte [count(2)][div(2)] payloads share the uniform
                            // S_COLLECT collector (was a dedicated S_ARG0..3 chain).
                            OP_START_DAC,
                            OP_START_CAPTURE: begin arg_len <= 16'd4; arg_rem <= 16'd4; state <= S_COLLECT; end
                            // MEASURE now carries SEPARATE dac + adc dividers (6-byte
                            // payload) so firmware can offset the DAC sequencer's per-
                            // sample overhead and run the DAC + ADC at the same real
                            // rate: [count(2)][dac_div(2)][cap_div(2)].
                            OP_START_MEASURE: begin arg_len <= 16'd6; arg_rem <= 16'd6; state <= S_COLLECT; end
                            // Control loop arm: 8-byte [k(2)][vmin(2)][vmax(2)][tick_div(2)].
                            OP_START_DAC_LOOP: begin arg_len <= 16'd8; arg_rem <= 16'd8; state <= S_COLLECT; end
                            // Loop INPUT SOURCE (v29): 5-byte [src][fixed(2)][step(2)].  Legal
                            // whether or not the loop is armed — a running loop picks the new
                            // source/value up on its next tick.
                            OP_DAC_LOOP_SRC: begin arg_len <= 16'd5; arg_rem <= 16'd5; state <= S_COLLECT; end
`ifndef USE_DEEP_REPLAY
                            // Only the control-loop image has a loop to map an input for. The
                            // registers themselves already DCE away in the deep build, but the
                            // DECODE did not, and perturbing this netlist cost the deep image
                            // ~5 MHz of clk48 margin on its pinned seed. Gate it out.
                            OP_DAC_LOOP_INMAP: begin arg_len <= 16'd7; arg_rem <= 16'd7; state <= S_COLLECT; end
`endif
                            // DEEP DAC replay: 8-byte payload [base(3)][count(3)][div(2)];
                            // the waveform streams straight from PSRAM (up to 8 MB).
                            OP_START_DAC_PSRAM: begin arg_len <= 16'd8; arg_rem <= 16'd8; state <= S_COLLECT; end
                            // Capture-tied DAC auto-stop threshold: 4-byte payload
                            // [cyc_lo][cyc_1][cyc_2][cyc_hi] (24 MHz clk cycles; 0 = disarm).
                            OP_SET_DAC_STOP_AFTER: begin arg_len <= 16'd4; arg_rem <= 16'd4; state <= S_COLLECT; end
                            // ADC_PROBE: latch the live sample and shift out its 2
                            // bytes (LE) with NO PSRAM/CDC/orchestrator in the path.
                            OP_ADC_PROBE: begin
                                adc_dbg_latch <= adc_dbg_sample;
                                tx_byte       <= adc_dbg_sample[7:0];   // low byte first
                                state         <= S_ADC_PROBE_HI;
                            end
                            // DAC_PROBE: live closed-loop DAC output (reuses the latch +
                            // high-byte shift state).  Pair with ADC_PROBE for (I,V) polling.
                            OP_DAC_PROBE: begin
                                adc_dbg_latch <= dac_loop_v_dbg;
                                tx_byte       <= dac_loop_v_dbg[7:0];
                                state         <= S_ADC_PROBE_HI;
                            end
                            // DAC_LOOP_IN_PROBE (v29): the value the loop's last tick actually
                            // indexed the curve with.  In a fixed/sweep run there is no ADC in
                            // the path at all, so this — not ADC_PROBE — is the loop's input.
                            OP_DAC_LOOP_IN_PROBE: begin
                                adc_dbg_latch <= dac_loop_in_dbg;
                                tx_byte       <= dac_loop_in_dbg[7:0];
                                state         <= S_ADC_PROBE_HI;
                            end
                            // GPIO_GET (v35): the live LA1..LA12 levels through the 2-flop
                            // synchroniser, LE, latched here so both bytes are one snapshot.
                            OP_GPIO_GET: begin
                                adc_dbg_latch <= {4'b0000, la_levels};
                                tx_byte       <= la_levels[7:0];
                                state         <= S_ADC_PROBE_HI;
                            end
                            // TRIGGER_STATUS (v35): bit0 waiting for the trigger, bit1 fired since
                            // the arm.  One reply byte, like STATUS.
                            OP_TRIGGER_STATUS: begin
                                tx_byte <= {6'b0, trig_fired, trig_wait};
                                state   <= S_DONE;
                            end
                            // Capture trigger (v35): [channel][mode][flags(reserved)].
                            OP_SET_TRIGGER: begin arg_len <= 16'd3; arg_rem <= 16'd3; state <= S_COLLECT; end
                            // 10-byte payload: [adc_cnt(3)][adc_div(2)][la_cnt(3)][la_div(2)]
                            // — both counts 24-bit so a single unified capture can span
                            // the multi-MB ADC region AND the multi-MB LA region.
                            OP_CAPTURE: begin arg_len <= 16'd10; arg_rem <= 16'd10; state <= S_COLLECT; end
                            // Runtime PSRAM bases for tri-capture (0x32): 6-byte payload.
                            OP_SET_CAPTURE_BASES: begin arg_len <= 16'd6; arg_rem <= 16'd6; state <= S_COLLECT; end

                            // ---- LA GPIO bank ----
                            OP_GPIO_SET:  begin arg_len <= 16'd2; arg_rem <= 16'd2; state <= S_COLLECT; end
                            OP_SET_LED:   begin arg_len <= 16'd1; arg_rem <= 16'd1; state <= S_COLLECT; end
                            // OP_WARMBOOT (0x17) removed in v24 — SB_WARMBOOT is HW-dead on this
                            // board (config-SPI = shared PSRAM bus).  An incoming 0x17 now falls
                            // through to `default: state <= S_DONE` and is safely ignored.
                            OP_CAPTURE_TEST: begin arg_len <= 16'd1; arg_rem <= 16'd1; state <= S_COLLECT; end
                            OP_PSRAM_CS:     begin arg_len <= 16'd1; arg_rem <= 16'd1; state <= S_COLLECT; end
                            OP_GPIO_STEP: begin arg_len <= 16'd5; arg_rem <= 16'd5; state <= S_COLLECT; end

                            // ---- SWD ----
                            OP_SWD_ARM:    begin arg_len <= 16'd3; arg_rem <= 16'd3; state <= S_COLLECT; end
                            OP_SWD_FEED:   begin swd_feed_begin <= 1'b1; state <= S_READ_LEN0; end
                            OP_SWD_READ:   begin
                                // Preload reply BRAM addr 0 (same 1-cycle
                                // latency trick as READ_CAPTURE).
                                swd_rd_addr <= {REPLY_AW{1'b0}};
                                state       <= S_READ_LEN0;
                            end
                            OP_SWD_DISARM: begin swd_disarm_stb <= 1'b1; state <= S_DONE; end

                            // ---- emulated I2C sensor ----
                            OP_I2C_CONFIG:  begin arg_len <= 16'd9; arg_rem <= 16'd9; state <= S_COLLECT; end
                            OP_I2C_DISABLE: begin i2c_disable_stb <= 1'b1; state <= S_DONE; end
                            OP_I2C_LOAD_REGS,
                            OP_I2C_READ_REGS: state <= S_REG_ADDR;
                            OP_I2C_STATUS: begin
                                arg_len   <= I2C_STATUS_LEN;
                                arg_rem   <= I2C_STATUS_LEN;
                                tx_byte   <= i2c_status_byte(3'd0);
                                state     <= S_I2C_STATUS;
                            end
                            // (OP_I2C_LA_START 0x65 / OP_I2C_LA_READ 0x66 removed in v24 —
                            //  the on-FPGA I2C-LA sampler is gone; sensor_la now uses the deep
                            //  LA_CAPTURE 0x69 path + a firmware re-pack.)
                            // (shallow BRAM LA_START 0x67 / LA_READ 0x68 removed in v17 —
                            //  v2 uses the deep LA_CAPTURE 0x69 below; HAS_WIDE_LA=0.)
                            // deep LA capture into PSRAM (v2): 5-byte payload
                            // [cnt_lo][cnt_mid][cnt_hi][div_lo][div_hi] — 24-bit sample
                            // count so a single capture can span the full 8 MB.
                            OP_LA_CAPTURE: begin arg_len <= 16'd5; arg_rem <= 16'd5; state <= S_COLLECT; end

                            // ---- UART proxy ----
                            OP_UART_CONFIG:  begin arg_len <= 16'd6; arg_rem <= 16'd6; state <= S_COLLECT; end
                            OP_UART_DISABLE: begin uart_disable_stb <= 1'b1; state <= S_DONE; end
                            OP_UART_WRITE,
                            OP_UART_READ:    state <= S_READ_LEN0;
                            OP_UART_STATUS: begin
                                arg_len         <= UART_STATUS_LEN;
                                arg_rem         <= UART_STATUS_LEN;
                                tx_byte         <= uart_status_byte(2'd0);
                                uart_rx_ovf_clr <= 1'b1;   // status read clears sticky overflow
                                state           <= S_UART_STATUS;
                            end

                            default: state <= S_DONE; // unknown cmd -> wait for CSn↑
                        endcase
                    end

                    S_READ_LEN0: begin
                        arg_len[7:0] <= rx_byte;
                        state        <= S_READ_LEN1;
                    end

                    S_READ_LEN1: begin
                        arg_len[15:8] <= rx_byte;
                        arg_count     <= 16'd0;
                        // Full 16-bit length is the byte just received (hi) with
                        // the low byte latched in S_READ_LEN0.  Loading arg_rem
                        // here covers every length-prefixed streaming state.
                        arg_rem       <= {rx_byte, arg_len[7:0]};
                        // Dispatch on the latched command.  Empty-payload reads
                        // (len==0) skip straight to S_DONE.  The read paths preload
                        // their first BRAM/FIFO byte here to hide the 1-cycle read
                        // latency before the streaming state shifts it out.
                        case (current_cmd)
                            OP_LOAD_WAVE: begin
                                wave_waddr <= {ADDR_W{1'b0}};
                                state      <= S_LOAD_DATA;
                            end
                            OP_SWD_FEED:
                                state <= ({rx_byte, arg_len[7:0]} == 16'd0)
                                           ? S_DONE : S_SWD_FEED;
                            OP_SWD_READ:
                                if ({rx_byte, arg_len[7:0]} == 16'd0) state <= S_DONE;
                                else begin
                                    swd_rd_addr <= {REPLY_AW{1'b0}};
                                    tx_byte     <= swd_rd_data;
                                    state       <= S_SWD_READ;
                                end
                            // reg_base preset in S_REG_ADDR; write/read port addr is
                            // reg_base + arg_count in S_REG_LOAD/S_REG_READ.
                            OP_I2C_LOAD_REGS:
                                state <= ({rx_byte, arg_len[7:0]} == 16'd0)
                                           ? S_DONE : S_REG_LOAD;
                            OP_I2C_READ_REGS:
                                if ({rx_byte, arg_len[7:0]} == 16'd0) state <= S_DONE;
                                else begin
                                    tx_byte <= i2c_reg_rdata;   // i2c_reg_raddr preset
                                    state   <= S_REG_READ;
                                end
                            OP_UART_WRITE:
                                state <= ({rx_byte, arg_len[7:0]} == 16'd0)
                                           ? S_DONE : S_UART_WRITE;
                            OP_UART_READ:
                                if ({rx_byte, arg_len[7:0]} == 16'd0) state <= S_DONE;
                                else begin
                                    // preload head byte + pop so the FIFO advances.
                                    tx_byte    <= uart_rx_rdata;
                                    uart_rx_re <= 1'b1;
                                    state      <= S_UART_READ;
                                end
                            default: state <= S_DONE;
                        endcase
                    end

                    S_LOAD_DATA: begin
                        wave_we    <= 1'b1;
                        wave_waddr <= arg_count[ADDR_W-1:0];
                        wave_wdata <= rx_byte;
                        if (last_byte) state <= S_DONE;
                        arg_count <= arg_count + 12'd1;
                        arg_rem   <= arg_rem   - 16'd1;
                    end

                    // ---- fixed-length argument collector ----
                    // Stores bytes into arg_buf; on the final byte it dispatches
                    // using rx_byte for the last slot (which isn't latched yet).
                    S_COLLECT: begin
                        arg_buf[arg_count[3:0]] <= rx_byte;
                        if (last_byte) begin
                            case (current_cmd)
                                // 4-byte [count(2)][div(2)] arming opcodes: the
                                // last byte (div_hi) is rx_byte, the rest are in
                                // arg_buf[0..2].  (Was the dedicated S_ARG0..3 chain.)
                                OP_START_DAC: begin
                                    dac_period  <= {arg_buf[1][ADDR_W-8:0], arg_buf[0]};
                                    dac_divider <= {rx_byte, arg_buf[2]};
                                    dac_start   <= 1'b1;
                                    // A BRAM-waveform start must SELECT the BRAM waveform.  The
                                    // mode registers are LEVELS, so leaving either set here armed
                                    // the engine with a DIFFERENT source than the caller loaded:
                                    // dac_psram_mode kept the deep reader as the source, which in
                                    // the deep image also keeps it CONTENDING FOR THE SHARED QUAD
                                    // BUS against the ADC/LA capture writer (corrupt capture), and
                                    // in the loop image starves the engine (rd_strm_valid tied 0 ->
                                    // the DAC holds a constant level).  Only OP_STOP_DAC used to
                                    // clear it, so a `generate` after a deep `replay` was silently
                                    // still replaying.  Both cleared unconditionally now (v31).
                                    dac_loop_mode  <= 1'b0;   // normal replay exits the loop
                                    dac_psram_mode <= 1'b0;   // ...and exits deep PSRAM streaming
                                end
                                // Loop input source (v29): [src][fixed(2)][step(2)].  Sets the
                                // registers only — arming and stopping stay with 0x15/0x12.
                                OP_DAC_LOOP_SRC: begin
                                    dac_loop_src  <= arg_buf[0][1:0];
                                    dac_loop_in   <= {arg_buf[2], arg_buf[1]};
                                    dac_loop_step <= {rx_byte,    arg_buf[3]};
                                end
`ifndef USE_DEEP_REPLAY
                                // Loop input map (v30): [in_zero(2)][in_gain(2)][in_trip(2)]
                                // [flags(1)], flags bit0=map_en, bit1=trip_en.
                                OP_DAC_LOOP_INMAP: begin
                                    dac_loop_in_zero   <= {arg_buf[1], arg_buf[0]};
                                    dac_loop_in_gain   <= {arg_buf[3], arg_buf[2]};
                                    dac_loop_in_trip   <= {arg_buf[5][2:0], arg_buf[4]};
                                    dac_loop_map_en    <= rx_byte[0];
                                    dac_loop_trip_en   <= rx_byte[1];
                                end
`endif
                                // Control loop arm: [k(2)][vmin(2)][vmax(2)][tick_div(2)].
                                OP_START_DAC_LOOP: begin
                                    dac_loop_k    <= {arg_buf[1], arg_buf[0]};
                                    dac_loop_vmin <= {arg_buf[3], arg_buf[2]};
                                    dac_loop_vmax <= {arg_buf[5], arg_buf[4]};
                                    dac_loop_tick <= {rx_byte,    arg_buf[6]};
                                    dac_loop_mode <= 1'b1;
                                    dac_start     <= 1'b1;   // kick the DAC8551 into streaming
                                end
                                OP_START_CAPTURE: begin
                                    cap_divider <= {rx_byte, arg_buf[2]};
                                    cap_start   <= 1'b1;
                                end
                                OP_GPIO_SET: begin
                                    gpio_set_stb  <= 1'b1;
                                end
                                OP_SET_LED: begin
                                    led_ctrl <= rx_byte;   // 1 arg: the LED mask
                                end
                                OP_CAPTURE_TEST: begin
                                    cap_test_ramp <= rx_byte[0];    // 0=real ADC, 1=ramp self-test
                                end
                                OP_PSRAM_CS: begin
                                    psram_cs_force <= rx_byte[0];   // 1=force PSRAM /CS low
                                end
                                OP_SWD_ARM: begin
                                    swd_arm_stb      <= 1'b1;
                                end
                                OP_GPIO_STEP: begin
                                    // [channel][steps_lo][steps_hi][delay_lo][delay_hi(=rx_byte)]
                                    step_start   <= 1'b1;
                                end
                                // Unified capture: arm the ADC and the raw 12-ch LA
                                // producers on the SAME cycle (shared t0), each with
                                // its own count+divider -> two independent PSRAM
                                // regions.  adc_cnt=0 => ADC not captured; la_cnt=0 =>
                                // LA not captured.  See top_v2.v / psram_dual_writer.
                                OP_CAPTURE: begin
                                    // [adc_cnt_lo][adc_cnt_mid][adc_cnt_hi][adc_div_lo][adc_div_hi]
                                    // [la_cnt_lo][la_cnt_mid][la_cnt_hi][la_div_lo][la_div_hi(=rx_byte)]
                                    // BOTH counts are now full 24-bit so a single unified
                                    // capture spans the multi-MB ADC region AND the multi-MB
                                    // LA region (each read back chunked from PSRAM — no longer
                                    // bounded by the old shared 32768-sample RAM buffer).
                                    cap_divider    <= {arg_buf[4], arg_buf[3]};
                                    la_cap_divider <= {rx_byte,    arg_buf[8]};
                                    cap_start      <= 1'b1;   // arm ADC producer
                                    la_cap_start   <= 1'b1;   // arm LA producer (same cycle)
                                end
                                // Latch runtime PSRAM region bases (6-byte payload):
                                // [la_base_lo,mid,hi][adc_base_lo,mid,hi(=rx_byte)].
                                OP_SET_CAPTURE_BASES: begin
                                    /* payload [la_base(3)][adc_base(3)]; LA base ignored
                                       (always 0), only the ADC base is programmable. */
                                    adc_cap_base <= {rx_byte, arg_buf[4], arg_buf[3]};
                                end
                                // deep standalone LA capture (v2): 5-byte payload with a
                                // 24-bit sample count -> a single capture can span 8 MB.
                                OP_LA_CAPTURE: begin
                                    // [cnt_lo][cnt_mid][cnt_hi][div_lo][div_hi(=rx_byte)]
                                    la_cap_divider <= {rx_byte,    arg_buf[3]};
                                    la_cap_start   <= 1'b1;
                                end
                                // MEASURE: start the DAC (from the loaded waveform) and
                                // the ADC capture on the SAME cycle, but with SEPARATE
                                // dividers.  The DAC sequencer costs extra clocks/sample
                                // (BRAM reads + SPI shift), so firmware sends
                                // dac_div = cap_div - overhead to make the DAC and ADC
                                // step at the same real rate (aligned capture).
                                //   [count_lo][count_hi][dac_div_lo][dac_div_hi]
                                //   [cap_div_lo][cap_div_hi(=rx_byte)]
                                OP_START_MEASURE: begin
                                    dac_period  <= {arg_buf[1][ADDR_W-8:0], arg_buf[0]};   // 13-bit: DAC waveform BRAM bounded
                                    dac_divider <= {arg_buf[3], arg_buf[2]};
                                    cap_divider <= {rx_byte,    arg_buf[4]};
                                    dac_start   <= 1'b1;
                                    cap_start   <= 1'b1;
                                    // Same source-selection rule as OP_START_DAC above, and it
                                    // matters MORE here: measure arms a capture in the same cycle,
                                    // so a stale dac_psram_mode points the deep reader at the PSRAM
                                    // bus exactly while the capture writer needs it.  (v31)
                                    dac_loop_mode  <= 1'b0;
                                    dac_psram_mode <= 1'b0;
                                end
                                // DEEP DAC replay from PSRAM: [base_lo][base_mid][base_hi]
                                // [cnt_lo][cnt_mid][cnt_hi][div_lo][div_hi(=rx_byte)].  Set
                                // the streaming mode LEVEL and pulse the DAC engine start;
                                // the top latches base/len into the DAC clock domain and
                                // the reader streams the waveform out of PSRAM.
                                OP_START_DAC_PSRAM: begin
                                    dac_psram_base <= {arg_buf[2], arg_buf[1], arg_buf[0]};  // 24-bit byte base
                                    dac_psram_len  <= {arg_buf[5], arg_buf[4], arg_buf[3]};  // 24-bit SAMPLE count
                                    dac_divider    <= {rx_byte,    arg_buf[6]};
                                    dac_psram_mode <= 1'b1;
                                    dac_start      <= 1'b1;
                                end
                                // Capture-tied DAC auto-stop threshold (persistent; the top
                                // snapshots it at the next capture arm).  0 disarms.
                                //   [cyc_lo][cyc_1][cyc_2][cyc_hi(=rx_byte)]
                                OP_SET_DAC_STOP_AFTER: begin
                                    dac_stop_after <= {rx_byte, arg_buf[2], arg_buf[1], arg_buf[0]};
                                end
                                // Capture trigger (persistent; the top applies it at every later
                                // arm): [channel][mode][flags(reserved, =rx_byte)].  The mode is
                                // decoded here so the top's per-cycle test is a mux + two compares.
                                OP_SET_TRIGGER: begin
                                    trig_ch   <= arg_buf[0][3:0];
                                    trig_en   <= (arg_buf[1] != 8'd0) && (arg_buf[1] <= 8'd4);
                                    trig_edge <= (arg_buf[1] == 8'd1) || (arg_buf[1] == 8'd2);
                                    trig_pol  <= (arg_buf[1] == 8'd1) || (arg_buf[1] == 8'd3);
                                end
                                OP_I2C_CONFIG: begin
                                    // [addr7][sda_ch][scl_ch][flags][trig_reg]
                                    // [busy_reg][busy_mask][conv_lo][conv_hi(=rx_byte)]
                                    i2c_cfg_stb       <= 1'b1;
                                end
                                OP_UART_CONFIG: begin
                                    // [rx_ch][tx_ch][div_lo][div_mid][div_hi(=rx_byte)][flags]
                                    // NOTE: flags is the final byte (rx_byte); div_hi is arg_buf[4].
                                    uart_cfg_stb     <= 1'b1;
                                end
                                default: ;
                            endcase
                            state <= S_DONE;
                        end
                        arg_count <= arg_count + 12'd1;
                        arg_rem   <= arg_rem   - 16'd1;
                    end

                    S_SWD_FEED: begin
                        swd_feed_stb  <= 1'b1;
                        swd_feed_byte <= rx_byte;
                        if (last_byte) state <= S_DONE;
                        arg_count <= arg_count + 12'd1;
                        arg_rem   <= arg_rem   - 16'd1;
                    end

                    S_SWD_READ: begin
                        // Mirror of the length-prefixed read-back loop for the SWD reply buffer.
                        if (!last_byte) begin
                            swd_rd_addr <= arg_count[REPLY_AW-1:0] + 1'b1;
                            tx_byte     <= swd_rd_data;
                            arg_count   <= arg_count + 12'd1;
                            arg_rem     <= arg_rem   - 16'd1;
                        end else begin
                            state <= S_DONE;
                        end
                    end

                    // ---- emulated I2C sensor: register file load / read ----
                    S_REG_ADDR: begin
                        reg_base      <= rx_byte;
                        i2c_reg_raddr <= rx_byte;   // preload for READ_REGS
                        state         <= S_READ_LEN0;
                    end

                    S_REG_LOAD: begin
                        i2c_reg_we    <= 1'b1;
                        i2c_reg_waddr <= reg_base + arg_count[7:0];
                        i2c_reg_wdata <= rx_byte;
                        if (last_byte) state <= S_DONE;
                        arg_count <= arg_count + 12'd1;
                        arg_rem   <= arg_rem   - 16'd1;
                    end

                    S_REG_READ: begin
                        if (!last_byte) begin
                            i2c_reg_raddr <= reg_base + arg_count[7:0] + 8'd1;
                            tx_byte       <= i2c_reg_rdata;
                            arg_count     <= arg_count + 12'd1;
                            arg_rem       <= arg_rem   - 16'd1;
                        end else begin
                            state <= S_DONE;
                        end
                    end

                    S_I2C_STATUS: begin
                        if (!last_byte) begin
                            tx_byte   <= i2c_status_byte(arg_count[2:0] + 3'd1);
                            arg_count <= arg_count + 12'd1;
                            arg_rem   <= arg_rem   - 16'd1;
                        end else begin
                            state <= S_DONE;
                        end
                    end

                    // ADC_PROBE second byte: low byte already shifted out (loaded in
                    // S_IDLE); this rx_valid clocks the high byte, then park.
                    S_ADC_PROBE_HI: begin
                        tx_byte <= adc_dbg_latch[15:8];
                        state   <= S_DONE;
                    end

                    // (S_LA_READ — the I2C-LA read-back stream — removed in v24.)

                    // ---- UART proxy: stream bytes into the TX FIFO ----
                    S_UART_WRITE: begin
                        uart_tx_we    <= 1'b1;
                        uart_tx_wdata <= rx_byte;
                        if (last_byte) state <= S_DONE;
                        arg_count <= arg_count + 12'd1;
                        arg_rem   <= arg_rem   - 16'd1;
                    end

                    // ---- UART proxy: stream bytes out of the RX FIFO ----
                    // head byte was preloaded + popped in S_READ_LEN1; each
                    // subsequent padding byte sends the new head and pops again.
                    S_UART_READ: begin
                        if (!last_byte) begin
                            tx_byte    <= uart_rx_rdata;
                            uart_rx_re <= 1'b1;
                            arg_count  <= arg_count + 12'd1;
                            arg_rem    <= arg_rem   - 16'd1;
                        end else begin
                            state <= S_DONE;
                        end
                    end

                    S_UART_STATUS: begin
                        if (!last_byte) begin
                            tx_byte   <= uart_status_byte(arg_count[1:0] + 2'd1);
                            arg_count <= arg_count + 12'd1;
                            arg_rem   <= arg_rem   - 16'd1;
                        end else begin
                            state <= S_DONE;
                        end
                    end

                    S_DONE: begin
                        // Stay here until CSn↑; ignore further bytes.
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
