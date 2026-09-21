// ============================================================================
// top_v2.v — top-level for the STM32 "vbench-pod" board (VERSION=v2).
//
// UNIFIED CAPTURE (ground-up redesign — see docs/unified-adc-la-capture-redesign.md):
// the 16-bit serial ADC (MCP33131) and the raw 12-channel logic analyzer are
// captured SIMULTANEOUSLY off one trigger, each into its OWN contiguous PSRAM
// region, by two independent producers -> two SPRAM rings -> one
// psram_dual_writer.  There are no tags and no per-sample timestamps: sample i of
// each stream is at t0 + i*divider, and both counters start at the same arm cycle,
// so the two captures are time-aligned by construction.  Protocol interpretation
// (I2C/SPI/UART decode) is done OFF the FPGA (server / STM32) from the raw LA
// words — the FPGA does only deterministic capture.
//
// This replaces the old mode-exclusive zoo (ADC-only / correlated ADC+I2C-LA /
// deep-LA) and the tagged capture_merge stream; capture_merge/spram_ring/cdc/
// la_drain_ctrl/i2c_la (as a capture path) are no longer instantiated here.
//
// SINGLE-CLOCK CAPTURE + 48 MHz DDR DRAIN.  The whole capture datapath — ADC/LA
// producers, both rings, the writer FSM and FIFOs, and the control plane — runs on
// one 24 MHz `clk` (clk48÷2), preserving the single-clock collapse that fixed the
// "ADC reads 0x5555" mis-latch (docs/adc-capture-cdc-review.md).  ONLY the PSRAM
// write OUTPUT is sped up: psram_dual_writer emits one "cell" (=1 byte) per clk
// cycle and a thin clk48 serializer shifts it out at one nibble per clk48 cycle
// (~24 MB/s), with SCLK driven by a DDR SB_IO on clk48 (rises mid-eye).  The
// clk->clk48 hop is a synchronous 1:2 gearbox, NOT the async class that caused
// 0x5555 (docs/task8-48mhz-ddr-drain.md).  The shared quad bus is tristated when
// the STM32 owns it (bus_own / PG0).  v1's parallel-board gateware (top.v)
// instantiates the same engine_block.
// ============================================================================
`include "sys_config.vh"   // SYS_CLK_MHZ — single source (tools/gen_protocol.py)
module top (
    input  wire        clk48,

    // SPI link from the STM32
    input  wire        sck,
    input  wire        mosi,
    output wire        miso,
    input  wire        csn,
    output wire        busy,
    input  wire        bus_own,

    // serial ADC (MCP33131-10)
    output wire        adc_cnvst,
    output wire        adc_sclk,
    output wire        adc_sdi,
    input  wire        adc_sdo,

    // serial DAC (DAC8551)
    output wire        dac_sync,
    output wire        dac_sclk,
    output wire        dac_din,

    // shared quad-SPI bus to the APS6404L PSRAM
    inout  wire        psram_sclk,
    inout  wire        psram_cs,
    inout  wire        psram_io0,
    inout  wire        psram_io1,
    inout  wire        psram_io2,
    inout  wire        psram_io3,

    // logic-analyzer bank (14 channels)
    inout  wire [13:0] la,

    // status LEDs — iCE40 dedicated RGB-driver pads (SB_RGBA_DRV), host-driven
    // via the SET_LED command (green = "a target eFuse is ON").
    output wire        led_g,   // pad 39 (green)
    output wire        led_b,   // pad 40 (yellow)
    output wire        led_r    // pad 41 (red)
);
    // ---- clocking (single 24 MHz logic domain) ------------------------------
    // clk48 (the external oscillator, on the global clock pad) is the primary
    // clock, but NO functional logic runs on it: it only derives `clk` (÷2) and
    // clocks the power-on-reset counter.  The whole design runs on the derived
    // 24 MHz `clk` (the single-clock collapse — docs/adc-capture-cdc-review.md).
    reg clkdiv = 1'b0;
    always @(posedge clk48) clkdiv <= ~clkdiv;
    wire clk;
    SB_GB clk_gb (.USER_SIGNAL_TO_GLOBAL_BUFFER(clkdiv), .GLOBAL_BUFFER_OUTPUT(clk));

    // ---- reset (async-assert at power-up, sync-deassert per domain) ----------
    reg [5:0] por_cnt = 6'd0;
    reg       por_n   = 1'b0;      // 0 = in power-on reset, 1 = released
    always @(posedge clk48) begin
        if (por_cnt != 6'h3F) por_cnt <= por_cnt + 6'd1;
        else                  por_n   <= 1'b1;
    end
    reg [1:0] rst_q = 2'b11;
    always @(posedge clk) rst_q <= {rst_q[0], ~por_n};
    wire rst = rst_q[1];           // 24 MHz reset (whole design)
    reg [1:0] rst48_q = 2'b11;
    always @(posedge clk48) rst48_q <= {rst48_q[0], ~por_n};
    wire rst48 = rst48_q[1];       // 48 MHz reset (the clk48 DAC sequencer)

    // ---- waveform BRAM read port (dac8551_engine reads the shared wave_buf
    //      that lives in engine_block; cmd_dispatch writes it) ----
    wire [11:0] wave_raddr;
    wire [7:0]  wave_rdata;

    // ---- DAC engine (serial DAC8551) — on clk48 (48 MHz) for 2x the update rate
    //      => smoother output.  Control comes from engine_block on `clk`; the arm/
    //      stop pulses and the (arm-time-stable) period/divider cross a small
    //      control-plane CDC into clk48, and `running` crosses back with a 2-FF.
    //      The waveform BRAM read port is on clk48 too (wave_rclk=clk48), so
    //      wave_raddr/wave_rdata stay in-domain — no data CDC.  (See
    //      docs/ice40-48mhz-migration.md; the DAC never touches the capture path,
    //      so this does NOT re-open the collapsed 24/48 capture CDC.) ----
    wire        dac_start, dac_stop_pulse;   // clk (24 MHz) pulses from engine_block
    wire [12:0] dac_period;
    wire [15:0] dac_divider;
    wire        dac_running_48;              // engine's running, native clk48
    wire        dac_running;                 // synchronized to clk for STATUS

    // ---- capture-tied DAC auto-stop (OP_SET_DAC_STOP_AFTER 0x14, v2 >= v21) ----
    // Firmware writes a threshold in 24 MHz clk cycles before arming a capture; the counter in
    // the capture FSM (below) trips `dac_autostop` when it reaches the threshold, cutting a
    // concurrently-running DAC at a sample-precise point WITHIN the capture window.  The trip is
    // OR'd into the clk48 STOP (below) to mute the DAC8551, and latched to gate the deep-replay
    // reader off (dac_psram_run) until the next DAC arm.  dac_stop_after==0 disarms.
    wire [31:0] dac_stop_after;               // threshold in clk cycles (0 = disarmed)
    wire        dac_autostop;                 // 1-cycle trip pulse (driven in the capture FSM)
    reg         dac_autostop_lat = 1'b0;      // holds the reader gated off after a trip

    // ---- co-trigger: fire the deferred DAC start on the capture arm (OP_DAC_ARM_ON_CAPTURE
    // 0x19, v2 >= v27) ----  `dac_cotrig` (a 1-cycle strobe from cmd_dispatch) sets `dac_pend`;
    // while pending, a DAC start opcode's ENGINE-start pulse is HELD and instead fires on the next
    // capture `arm`, so the DAC's sample 0 and the capture's t0 land on the SAME clk cycle.  The
    // RAW dac_start still runs the reader-prefill/lat-clear at stage time (dac_autostop_lat below),
    // so the deep FIFO is primed and the first PSRAM sample pops at t0 with no fill latency.
    // dac_pend / dac_start_eff are driven after `arm` is declared (capture FSM section).
    wire        dac_cotrig;                    // 1-cycle co-trigger arm (from engine_block)
    reg         dac_pend = 1'b0;               // a DAC start is staged, waiting for the capture arm
    wire        dac_start_eff;                 // effective engine start: immediate, or at arm if pending

    // clk -> clk48 control CDC through cdc_pulse_payload (v34): START carries period/divider with
    // it — the clk48 copies are written one clk48 cycle BEFORE the engine's start pulse, so the
    // engine loads THIS start's values.  (Until v33 the copy and the start were the same edge and
    // every start's first waveform pass ran at the PREVIOUS period.)  STOP rides an identical
    // instance with no payload, so start and stop keep the same latency and relative order.
    wire [28:0] dac_ctl_48;
    wire [12:0] dac_period_48  = dac_ctl_48[28:16];
    wire [15:0] dac_divider_48 = dac_ctl_48[15:0];
    wire        dac_start_48, dac_stop_48;
    cdc_pulse_payload #(.W(29), .INIT({13'd0, 16'd2})) dac_start_cdc (
        .src_clk(clk), .src_pulse(dac_start_eff), .src_data({dac_period, dac_divider}),
        .dst_clk(clk48), .dst_pulse(dac_start_48), .dst_data(dac_ctl_48));
    cdc_pulse_payload #(.W(1)) dac_stop_cdc (
        .src_clk(clk), .src_pulse(dac_stop_pulse | dac_autostop), .src_data(1'b0),
        .dst_clk(clk48), .dst_pulse(dac_stop_48), .dst_data());

    // ---- DEEP replay control plane (OP_START_DAC_PSRAM 0x13) ----
    // dac_psram_mode/base/len are clk-domain (straight from cmd_dispatch), and the
    // dac_psram_reader's QPI FSM runs on clk (24 MHz) — deliberately NOT clk48: the
    // 24-bit-address logic will not close 48 MHz on the congested up5k (the ~39 MHz
    // clk48 ceiling), but clk has margin.  So the reader takes its control directly
    // in the clk domain; only the DAC engine's psram_mode crosses into clk48 (a 2-FF
    // sync of the mode LEVEL — goes low on STOP_DAC too).  The reader's prefetch FIFO
    // is itself dual-clock (write clk / read clk48), so the engine pops in-domain.
    wire        dac_psram_mode;
    wire [23:0] dac_psram_base;
    wire [23:0] dac_psram_len;                       // in samples
    wire [23:0] psram_len_bytes = {dac_psram_len[22:0], 1'b0};   // samples*2 (<= 8 MB)
    // Gate the deep-replay reader/engine off once the capture-tied auto-stop has tripped, until
    // the next DAC arm re-enables it (dac_autostop_lat) — mirrors OP_STOP_DAC clearing the mode.
    wire        dac_psram_run = dac_psram_mode & ~dac_autostop_lat;
    reg  [1:0]  psram_run_s = 2'b0;
    always @(posedge clk48) psram_run_s <= {psram_run_s[0], dac_psram_run};
    wire        psram_run_48 = psram_run_s[1];       // engine psram_mode (clk48)

    // ---- closed-loop DAC control engine (OP_START_DAC_LOOP 0x15, >=v23) ----
    // Params come from engine_block (clk domain); the loop runs on clk48 (DAC + BRAM-read
    // domain).  It reuses the LOAD_WAVE BRAM as a curve LUT (read via wave_raddr, muxed at
    // dac_i) and feeds the computed `v` to the DAC through the strm port.  No PSRAM — so an
    // LA capture runs alongside it untouched.  Inert (mux hands the DAC back) when arm=0.
    // Closed-loop and deep-DAC-PSRAM-replay are MUTUALLY-EXCLUSIVE DAC advanced modes of
    // near-identical size; keeping both overflows the up5k (~93% LC).  The DEFAULT build
    // ships the closed loop; a `USE_DEEP_REPLAY` build swaps it for the PSRAM read master.
    // Both stay in the codebase — a given bitstream just carries one (they never run at the
    // same instant).  ADC+LA capture, shallow DAC generate, etc. are in EVERY build.
    //
    // Forward declarations: the closed-loop block below references the live ADC sample
    // (adc_sample/adc_sample_stb, from the ADC engine) and the DAC strm-pop handshake
    // (dac_strm_pop, from the dac8551 engine), both instantiated further down.  iverilog
    // cannot bind a net used before its declaration, so declare them here.
    wire [15:0] adc_sample;
    wire        adc_sample_stb;
    wire        dac_strm_pop;                // dac8551's strm pop, routed to loop or reader
    wire        dac_loop_mode;
    wire [15:0] dac_loop_k, dac_loop_vmin, dac_loop_vmax, dac_loop_tick;
    wire [1:0]  dac_loop_src;                             // 0 = ADC, 1 = fixed, 2 = sweep (v29)
    wire [15:0] dac_loop_in, dac_loop_step;
    // Input conditioner + safety bounds (v30): affine map onto the curve index, over-range
    // trip, per-tick slew bound.  All zero => the v29 behaviour.
    wire [15:0] dac_loop_in_zero, dac_loop_in_gain;
    wire [10:0] dac_loop_in_trip;
    wire        dac_loop_map_en, dac_loop_trip_en;
    wire        dac_loop_mode48;
    wire [11:0] loop_raddr;
    wire [7:0]  loop_strm_data;
    wire        loop_strm_valid;
    wire [15:0] loop_v;                                  // current DAC output, for telemetry
    wire [15:0] loop_in;                                 // input the last tick used (v29)
    wire [15:0] loop_idx;                                // curve index it resolved to (v30)
    wire        loop_tripped;                            // latched over-range trip (v30)
    reg  [15:0] loop_v_clk = 16'd0;                       // clk-domain snapshot for DAC_PROBE telemetry
    reg  [15:0] loop_in_clk = 16'd0;                      // ditto for DAC_LOOP_IN_PROBE
    always @(posedge clk) loop_v_clk  <= loop_v;
    always @(posedge clk) loop_in_clk <= loop_in;
`ifndef USE_DEEP_REPLAY
    reg  [1:0]  loop_mode_s = 2'b0;
    always @(posedge clk48) loop_mode_s <= {loop_mode_s[0], dac_loop_mode};
    assign      dac_loop_mode48 = loop_mode_s[1];
    // live ADC sample synced into clk48 with its strobe (never a torn 16-bit read).  The loop
    // reads adc48 on its own tick, so the crossed pulse itself is unused (and pruned).
    wire [15:0] adc48;
    cdc_pulse_payload #(.W(16)) adc_cdc (
        .src_clk(clk), .src_pulse(adc_sample_stb), .src_data(adc_sample),
        .dst_clk(clk48), .dst_pulse(), .dst_data(adc48));
    dac_loop #(.ADDR_W(12), .SHIFT(5)) loop_i (
        .clk48(clk48), .rst48(rst48), .arm(dac_loop_mode48),
        .adc_sample(adc48),
        .k_q15(dac_loop_k), .vmin(dac_loop_vmin), .vmax(dac_loop_vmax), .tick_div(dac_loop_tick),
        // Loop input source (v29).  src/fixed/step are plain held registers written from the
        // clk domain and only SAMPLED here on a tick, exactly like k/vmin/vmax/tick_div — no
        // CDC beyond that is warranted (a torn value would have to be written mid-tick and
        // would be corrected on the next one, and the host never writes while metering).
        .src_sel(dac_loop_src), .in_fixed(dac_loop_in), .sweep_step(dac_loop_step),
        .in_zero(dac_loop_in_zero), .in_gain(dac_loop_in_gain),
        .in_trip(dac_loop_in_trip),
        .map_en(dac_loop_map_en), .trip_en(dac_loop_trip_en),
        .lut_raddr(loop_raddr), .lut_rdata(wave_rdata),
        .strm_data(loop_strm_data), .strm_valid(loop_strm_valid),
        .strm_pop(dac_loop_mode48 & dac_strm_pop),
        .v_out(loop_v), .in_used(loop_in),
        .idx_used(loop_idx), .tripped(loop_tripped)
    );
`else
    assign dac_loop_mode48 = 1'b0;      // deep-replay build: no control loop
    assign loop_idx = 16'd0; assign loop_tripped = 1'b0;
    assign loop_raddr = 12'd0; assign loop_strm_data = 8'd0;
    assign loop_strm_valid = 1'b0; assign loop_v = 16'd0; assign loop_in = 16'd0;
`endif

    // reader (FSM @clk, FIFO read @clk48) <-> DAC engine + PSRAM pads (muxed below).
    wire [3:0]  rd_io_o, rd_io_i;
    wire        rd_io_oe, rd_cs, rd_sclk, rd_active;
    wire [7:0]  rd_strm_data;
    wire        rd_strm_valid, rd_strm_pop;

    // ---- shared-PSRAM bus arbiter handshakes (clk domain) ----
    // The DAC read master and the capture write master now TIME-MULTIPLEX the single
    // quad bus at burst granularity so deep DAC replay and ADC/LA capture can run at
    // the same time in disjoint PSRAM regions (docs/psram-concurrent-arbiter).  When
    // only one is active the arbiter grants it every burst, so single-use is unchanged.
    wire        rd_req, rd_busy, rd_gnt;
    wire        wr_req, wr_busy, wr_gnt;

    // The DAC engine's BRAM read + strm source are MUXED between deep-replay and the
    // closed-loop engine below (dac_loop): in closed-loop mode the loop drives the
    // sample_buf read (as a curve LUT) and feeds `v` in via the strm port.
    wire [11:0] dac_wave_addr;               // dac8551's own BRAM read addr (replay path)
    dac8551_engine dac_i (
        .clk(clk48), .rst(rst48), .start(dac_start_48), .stop(dac_stop_48),
        .period_samples(dac_period_48), .divider(dac_divider_48),
        .wave_addr(dac_wave_addr), .wave_data(wave_rdata),
        .psram_mode(psram_run_48 | dac_loop_mode48),
        .strm_data(dac_loop_mode48 ? loop_strm_data : rd_strm_data),
        .strm_valid(dac_loop_mode48 ? loop_strm_valid : rd_strm_valid),
        .strm_pop(dac_strm_pop),
        .dac_sync(dac_sync), .dac_sclk(dac_sclk), .dac_din(dac_din),
        .running(dac_running_48)
    );
    // curve/BRAM read addr: loop when armed, else the DAC replay sequencer.
    assign wave_raddr   = dac_loop_mode48 ? loop_raddr : dac_wave_addr;
    // the deep-replay reader is popped only when the loop is NOT driving the DAC.
    assign rd_strm_pop  = ~dac_loop_mode48 & dac_strm_pop;

    // Deep-replay PSRAM read master: FSM/pads on clk, FIFO read on clk48 for the DAC.
    // FIFO kept small (32 B): the gray read-pointer's width sits on the clk48 critical
    // path (rgray -> strm_valid -> DAC strm_pop), and 32 B still dwarfs any reader gap
    // vs the DAC's <=1.81 MB/s drain (recordings replay at <=0.4 MS/s in practice).
`ifdef USE_TRI_MASTER
    // (the tri-master below provides the DAC read master)
`elsif USE_DEEP_REPLAY
    dac_psram_reader #(.CHUNK_BYTES(9'd8), .WAIT_CYCLES(4'd6), .FIFO_AW(5)) rdr_i (
        .clk(clk), .rst(deep_rst), .clk48(clk48), .rst48(deep_rst48),
        .run(dac_psram_run), .base_addr(dac_psram_base), .len_bytes(psram_len_bytes),
        .data(rd_strm_data), .data_valid(rd_strm_valid), .data_pop(rd_strm_pop),
        .bus_gnt(rd_gnt), .bus_req(rd_req), .bus_busy(rd_busy),
        .io_o(rd_io_o), .io_oe(rd_io_oe), .io_i(rd_io_i),
        .cs(rd_cs), .sclk(rd_sclk), .active(rd_active)
    );
`else
    // loop build: no deep-replay reader — the closed loop drives the DAC. Tie reader nets.
    assign rd_strm_data = 8'd0; assign rd_strm_valid = 1'b0;
    assign rd_req = 1'b0; assign rd_busy = 1'b0; assign rd_active = 1'b0;
    assign rd_io_o = 4'h0; assign rd_io_oe = 1'b0; assign rd_cs = 1'b1; assign rd_sclk = 1'b0;
`endif
    // running: clk48 -> clk 2-FF sync (STATUS/dac_running reads on `clk`).
    reg [1:0] dac_run_sync = 2'b0;
    always @(posedge clk) dac_run_sync <= {dac_run_sync[0], dac_running_48};
    assign dac_running = dac_run_sync[1];

    // ========================================================================
    // Unified capture datapath — one 24 MHz `clk` domain.
    // ========================================================================

    // ---- control/config from engine_block (24 MHz) ----
    wire        cap_start;        // ADC arm pulse   (OP_START_CAPTURE 0x20, OR the unified OP_CAPTURE 0x31)
    wire        la_cap_start;     // LA  arm pulse   (OP_LA_CAPTURE   0x69, OR the unified OP_CAPTURE 0x31)
    wire        cap_test_ramp;    // 1 = inject a known ramp instead of the real ADC (cap-selftest)
    wire        psram_cs_force;   // 1 -> force PSRAM /CS low (boot /CE-net self-test)
    wire [23:0] cap_count;        // ADC sample count (0 => ADC not part of this capture); 24-bit for deep capture (up to 2,097,152, multi-MB PSRAM region)
    wire [15:0] cap_divider;      // ADC sample period in clk cycles
    wire [23:0] adc_cap_base;     // runtime ADC region base (SET_CAPTURE_BASES 0x32; default 4 MB). LA base is const 0.
    wire [23:0] la_cap_count;     // LA sample count  (24-bit: deep LA up to full 8 MB; 0 => LA not in this capture)
    wire [15:0] la_cap_divider;   // LA sample period in clk cycles
    wire [11:0] la_sample;        // live 12-channel LA readback
    wire [11:0] la_levels;        // la_sample through a 2-flop synchroniser (GPIO_GET + trigger)
    wire [3:0]  trig_ch;          // capture trigger config (OP_SET_TRIGGER 0x33, v35) — see below
    wire        trig_en, trig_edge, trig_pol;

    // The unified OP_CAPTURE (0x31) pulses cap_start AND la_cap_start on the SAME
    // cycle, so the two captures share t0.  OP_START_CAPTURE (0x20) pulses only
    // cap_start (ADC-only); OP_LA_CAPTURE (0x69) pulses only la_cap_start (LA-only).
    wire        arm = cap_start | la_cap_start;
    // A producer is "in this run" only if it was armed AND its count is non-zero (see the drain
    // controller below).  Named here because the trigger needs the same terms.
    wire        arm_adc = cap_start    & (cap_count    != 24'd0);
    wire        arm_la  = la_cap_start & (la_cap_count != 24'd0);

    // ---- capture trigger: gated start (OP_SET_TRIGGER 0x33 / OP_TRIGGER_STATUS 0x34, v2 >= v35) ----
    // With a trigger set, an arm still LOADS the producers (their counts are only valid on the arm
    // strobe) and starts the writer, but `trig_wait` holds both producers until the condition holds
    // on the synchronised LA levels.  That cycle is t0: the DAC co-trigger fires on it and
    // SET_DAC_STOP_AFTER counts from it.  trig_wait drops on the clk after t0, so the producers run
    // one clk after t0 exactly as they run one clk after an untriggered arm — every sample,
    // period, co-trigger and stop-after relationship is the same, measured from t0.
    //   * An arm with no producer in it (count 0) never waits, so re-arming with count 0 still
    //     aborts a waiting capture.  With a trigger set, t0 is ONLY the trigger cycle — never an
    //     arm — so such an abort does not fire a staged co-trigger (STOP_DAC cancels it), and the
    //     arm's count decode stays off the co-trigger path (it cost the loop image ~3 MHz of clk).
    //   * Edges are seen only after the arm: the previous level is a free-running flop, and the
    //     arm cycle itself is never a waiting cycle.  A level mode fires on the first waiting
    //     cycle if the pin already sits at that level.
    //   * trig_en = 0 keeps trig_wait at 0 and t0 = arm: the <= v34 behaviour.
    //   * There is NO LA pre-trigger: a triggered capture starts at the trigger, so the window is
    //     entirely post-trigger.  A drop-oldest LA ring was built and measured (see the v35 note at
    //     the version log below) and cost the deep image its clk48 margin, so it was dropped.
    wire        arm_trig  = arm & trig_en & (arm_adc | arm_la);
    reg         trig_wait = 1'b0;             // armed, condition not seen yet (holds the producers)
    reg         trig_fired = 1'b0;            // a triggered capture fired since the last arm
    reg         trig_prev = 1'b0;             // the selected level one clk ago (edge modes)
    wire [15:0] trig_lv16 = {4'b0000, la_levels};   // channels 12..15 read as 0
    wire        trig_lvl  = trig_lv16[trig_ch];
    always @(posedge clk) trig_prev <= trig_lvl;
    wire        trig_fire = trig_wait & (trig_lvl == trig_pol) & (~trig_edge | (trig_prev != trig_pol));
    always @(posedge clk) begin
        if (rst)            begin trig_wait <= 1'b0;     trig_fired <= 1'b0; end
        else if (arm)       begin trig_wait <= arm_trig; trig_fired <= 1'b0; end
        else if (trig_fire) begin trig_wait <= 1'b0;     trig_fired <= 1'b1; end
    end
    wire        t0 = (arm & ~trig_en) | trig_fire;

    // ---- co-trigger latch (see the dac_cotrig/dac_pend declarations above) ----
    // While a co-trigger is pending, the DAC start opcode's engine pulse is masked and re-issued on
    // the capture's t0 (the arm, or the trigger cycle); a plain (non-co-trigger) start passes
    // straight through (dac_pend stays 0).
    assign dac_start_eff = (dac_start & ~dac_pend) | (t0 & dac_pend);
    always @(posedge clk) begin
        if (rst)                 dac_pend <= 1'b0;
        else if (dac_cotrig)     dac_pend <= 1'b1;   // OP_DAC_ARM_ON_CAPTURE staged a co-trigger
        else if (dac_stop_pulse) dac_pend <= 1'b0;   // STOP_DAC cancels a staged-but-uncaptured start
        else if (t0)             dac_pend <= 1'b0;   // consumed by this capture's t0
    end

    // 2-FF sync of the STM32's bus_own (the only true async input).
    reg  [1:0]  bus_own_sync = 2'b00;
    always @(posedge clk) bus_own_sync <= {bus_own_sync[0], bus_own};
    wire        bus_own_synced = bus_own_sync[1];
    // ---- deep-image PSRAM-master reset gating (runtime image-swap fix) ----
    // The deep-replay reader + burst arbiter (present only in USE_DEEP_REPLAY) keep their FSMs
    // RUNNING whenever the STM32 owns the bus (bus_own) — the top only tristates their PADS, not
    // their state.  During a RUNTIME image swap the STM's post-reconfig psram_init YANKS the bus
    // and RESETS the PSRAM (0x66/0x99->QPI) while these just-booted masters are live; that
    // desyncs the arbiter (it stops granting the writer -> captures write nothing: "sentinel
    // survived") and the wedge only clears on a fresh reconfig/power-cycle.  Holding the deep
    // masters in reset for as long as the STM owns the bus keeps them quiescent across the yank,
    // so they come up clean once bus_own drops.  The STM only takes the bus while these masters
    // are idle (waveform staging / capture read-back happen between jobs, never mid-burst), so
    // this reset is always at a safe boundary.  Guarded so the LOOP netlist is byte-unchanged.
`ifdef USE_DEEP_REPLAY
    reg  [1:0]  bus_own_sync48 = 2'b00;      // clk48 copy of bus_own for the reader's rst48
    always @(posedge clk48) bus_own_sync48 <= {bus_own_sync48[0], bus_own};
    wire        deep_rst   = rst   | bus_own_synced;
    wire        deep_rst48 = rst48 | bus_own_sync48[1];
`endif

    // ---- ADC engine (serial MCP33131, 24 MHz clk, free-running) ----
    // (adc_sample / adc_sample_stb forward-declared above for the closed-loop block)
    wire        adc_sdo_pu;
    wire [15:0] adc_sample_sel;   // real ADC, or the self-test ramp
    SB_IO #(.PIN_TYPE(6'b000001), .PULLUP(1'b1)) io_adc_sdo (
        .PACKAGE_PIN(adc_sdo), .D_IN_0(adc_sdo_pu));
    wire adc_sample_stb_raw;
    adc_mcp33131 adc_i (
        .clk(clk), .rst(rst), .en(1'b1), .divider(cap_divider),
        .adc_cnvst(adc_cnvst), .adc_sclk(adc_sclk),
        .adc_sdi(adc_sdi), .adc_sdo(adc_sdo_pu),
        .sample(adc_sample), .sample_stb(adc_sample_stb_raw)
    );
    // Self-test ramp (+0x0101/sample) so the whole capture->writer->PSRAM->readback
    // path is verifiable end-to-end (CAPTURE_TEST / cap-selftest).
    reg  [15:0] adc_test_cnt;
    always @(posedge clk) begin
        if (rst)                     adc_test_cnt <= 16'd0;
        else if (adc_sample_stb_raw) adc_test_cnt <= adc_test_cnt + 16'h0101;
    end
    assign adc_sample_stb = adc_sample_stb_raw;
    assign adc_sample_sel = cap_test_ramp ? adc_test_cnt : adc_sample;

    // ---- ADC producer: pack each ADC sample into 2 bytes (LE) -> ADC ring ----
    // Armed by cap_start; captures cap_count samples at the ADC engine's rate.  It
    // pushes into the ADC SPRAM ring (not straight to the writer), so the writer's
    // per-region arbitration drains it independently of the LA stream.
    reg        adc_run, adc_done_r, adc_ovf_r;
    reg [23:0] adc_left;          // 24-bit countdown: deep ADC capture up to 2,097,152 samples
    reg [15:0] adc_hold;
    reg [1:0]  adc_pst;
    reg        adc_wr_stb;
    reg  [7:0] adc_wr_data;
    wire       adc_ring_in_full;
    always @(posedge clk) begin
        adc_wr_stb <= 1'b0;
        if (rst) begin
            adc_run<=1'b0; adc_done_r<=1'b0; adc_ovf_r<=1'b0; adc_left<=24'd0; adc_pst<=2'd0;
        end else if (cap_start) begin
            adc_run<=(cap_count!=24'd0); adc_done_r<=(cap_count==24'd0); adc_ovf_r<=1'b0;
            adc_left<=cap_count; adc_pst<=2'd0;
        end else if (adc_run) begin
            if (adc_sample_stb && adc_pst==2'd0 && !trig_wait) begin adc_hold<=adc_sample_sel; adc_pst<=2'd1; end
            else if (adc_pst==2'd1) begin
                adc_wr_data<=adc_hold[7:0];  adc_wr_stb<=1'b1; adc_pst<=2'd2;
                if (adc_ring_in_full) adc_ovf_r<=1'b1;
            end else if (adc_pst==2'd2) begin
                adc_wr_data<=adc_hold[15:8]; adc_wr_stb<=1'b1; adc_pst<=2'd0;
                if (adc_ring_in_full) adc_ovf_r<=1'b1;
                adc_left<=adc_left-24'd1;
                if (adc_left==24'd1) begin adc_run<=1'b0; adc_done_r<=1'b1; end
            end
        end
    end

    // ---- LA producer: la_psram_capture samples the 12-ch LA word into 2 bytes/
    //      sample -> LA ring.  Armed by la_cap_start.  (Its own ps_start/ps_stop
    //      writer-control outputs are unused — the writer is driven by the unified
    //      trigger + drain controller below.) ----
    wire       la_wr_stb, la_busy, la_done, la_overflow;
    wire [7:0] la_wr_data;
    wire       la_ring_in_full;
    la_psram_capture #(.N(12), .CNT_W(24)) la_ps_i (
        .clk(clk), .rst(rst), .la_in(la_sample),
        .start(la_cap_start), .sample_count(la_cap_count), .divider(la_cap_divider),
        .hold(trig_wait),
        .ps_start(), .ps_stop(),
        .wr_data(la_wr_data), .wr_stb(la_wr_stb), .full(la_ring_in_full),
        .busy(la_busy), .done(la_done), .overflow(la_overflow)
    );

    // ---- capture burst buffering ----
    // The LA stream (12 MS/s burst) keeps its deep 16-bit-packed SPRAM ring
    // (spram_ring16, AW=15): a byte-granular ring's read FSM would steal the single
    // SPRAM port so the 2nd of each sample's two writes collides + drops; packing each
    // sample into ONE 16-bit word makes it 1 write + 1 read per sample, which the
    // single port sustains.
    //
    // The ADC stream, by contrast, is SLOW (~0.8 MB/s) and now writes STRAIGHT into
    // psram_dual_writer's 32-byte ADC staging FIFO — no deep SPRAM ring.  The arbiter
    // is starvation-free, so the staging FIFO covers any bus-busy window (an LA/DAC
    // burst is < ~1.5 us CS-low; the ADC dribbles < 2 bytes into the 32-byte FIFO in
    // that time).  This drops a full spram_ring16 (~120 LC + an SB_SPRAM256KA block)
    // that only the LA burst actually needed — headroom for the 87%-full up5k.  If the
    // FIFO ever fills mid-capture the producer latches adc_ovf_r -> STATUS CAP_OVF, so
    // firmware still sees a truncated ADC capture as an overflow (not silent loss).
    wire [7:0] la_ring_out;
    wire       la_ring_out_stb;
    wire       la_ring_empty;
    wire       la_ring_ovf;
    wire       la_wr_full;             // LA staging-FIFO full (backpressure to LA ring)
    // adc_ring_in_full (declared up in the ADC producer section) is now driven by
    // psram_dual_writer.adc_full — the ADC producer's backpressure/overflow input.

    spram_ring16 #(.AW(15)) la_ring_i (
        .clk(clk), .rst(rst),
        .in_data(la_wr_data), .in_stb(la_wr_stb), .in_full(la_ring_in_full),
        .out_data(la_ring_out), .out_stb(la_ring_out_stb), .out_full(la_wr_full),
        .empty(la_ring_empty), .overflow(la_ring_ovf)
    );

    // ---- dual-region PSRAM writer (48 MHz DDR drain) ----
    // FSM/FIFOs on clk (24 MHz); output serialized on clk48 (one nibble per clk48
    // cycle).  psram_io_o/cs are clk48 regs (driven to COMB pads below); the SCLK pad
    // is a DDR SB_IO whose D_OUT_1 = ps_sclk_d1 (see the pad section).
    wire       ps_idle, ps_sclk_d1, ps_cs;
    wire [3:0] ps_io_o;
    wire       ps_io_oe;
    wire       ps_sclk_d0_tri;         // tri-master's read-mode SCLK d0 (Path B only; 0 in Path A)
    reg        dual_stop;
`ifndef USE_TRI_MASTER
    // ===== Path A: two masters (DAC read + ADC/LA write) + burst arbiter =====
    psram_dual_writer #(
        .CHUNK_BYTES(9'd16)            // 48 Mnib/s: (2+6+32) nib = 0.83 us CS-low < tCEM; fits the 32 B staging FIFO
    ) ps_i (
        .clk(clk), .clk48(clk48), .rst(rst), .bus_own(bus_own_synced),
        // LA is ALWAYS packed at base 0 (flexible allocator), so keep it a compile-time
        // constant — only ADC's base is runtime.  Reclaims the LC the folded LA_BASE=0
        // arithmetic used to save (the runtime-both-bases variant cost +130 LC / 88% fill).
        .adc_base(adc_cap_base), .la_base(24'h000000),
        .start(arm), .stop(dual_stop),
        .bus_gnt(wr_gnt), .bus_req(wr_req), .bus_busy(wr_busy),
        .adc_data(adc_wr_data), .adc_stb(adc_wr_stb), .adc_full(adc_ring_in_full),
        .la_data (la_ring_out),  .la_stb (la_ring_out_stb),  .la_full (la_wr_full),
        .idle(ps_idle),
        .psram_io_o(ps_io_o), .psram_io_oe(ps_io_oe), .psram_cs(ps_cs),
        .psram_sclk_d1(ps_sclk_d1)
    );
    assign ps_sclk_d0_tri = 1'b0;      // Path A drives read SCLK via rd_sclk (mux below)

    // ---- shared-bus arbiter (burst granularity) ----
    // Only needed when the deep-replay reader contends for the bus; in the DEFAULT (closed-
    // loop) build there is no reader, so the writer owns the bus every burst (wr_gnt=1).
`ifdef USE_DEEP_REPLAY
    psram_bus_arbiter arb_i (
        .clk(clk), .rst(deep_rst),
        .rd_req(rd_req), .rd_busy(rd_busy), .rd_gnt(rd_gnt),
        .wr_req(wr_req), .wr_busy(wr_busy), .wr_gnt(wr_gnt),
        .owner()   // pad mux uses rd_busy instead; owner unused at top
    );
`else
    assign wr_gnt = 1'b1;   // no reader to arbitrate against
    assign rd_gnt = 1'b0;
`endif
`else
    // ===== Path B: ONE unified tri-master (LA+ADC write + DAC read) =====
    // Drives the same ps_* pad wires the writer used; ties off the reader/arbiter nets
    // so the existing pad mux (replay=rd_busy=0 -> picks ps_*) and drain controller
    // (wr_req/wr_busy=0, ps_idle from the tri-master) work unchanged, except the read
    // SCLK d0 which the tri-master supplies on ps_sclk_d0_tri (see the sclk mux below).
`ifdef USE_DEEP_REPLAY
    localparam TRI_HAS_READ = 1;   // deep image: DAC PSRAM read job present
`else
    localparam TRI_HAS_READ = 0;   // loop image: capture-only (no deep replay) — compile the read path out
`endif
    psram_tri_master #(.WCHUNK(9'd16), .RCHUNK(9'd8), .RWAIT(4'd6), .RD_FIFO_AW(5), .HAS_DAC_READ(TRI_HAS_READ)) tri_i (
        .clk(clk), .rst(rst), .clk48(clk48), .rst48(rst48),
        .start(arm),
        .la_base(24'h000000),    .la_len({la_cap_count[22:0], 1'b0}),   // LA always at base 0
        .adc_base(adc_cap_base), .adc_len({cap_count[22:0], 1'b0}),
        // loop image: statically no DAC read job — tie dac_run 0 so the DAC region regs
        // + priming prune (HAS_DAC_READ=0 already drops the FIFO/read FSM).
        .dac_base(dac_psram_base), .dac_len(psram_len_bytes),
        .dac_run(TRI_HAS_READ ? dac_psram_run : 1'b0),
        .adc_data(adc_wr_data), .adc_stb(adc_wr_stb), .adc_full(adc_ring_in_full),
        .la_data (la_ring_out),  .la_stb (la_ring_out_stb),  .la_full (la_wr_full),
        .dac_data(rd_strm_data), .dac_valid(rd_strm_valid), .dac_pop(rd_strm_pop),
        .idle(ps_idle),
        .psram_io_o(ps_io_o), .psram_io_oe(ps_io_oe), .psram_io_i(rd_io_i),
        .psram_cs(ps_cs), .psram_sclk_d0(ps_sclk_d0_tri), .psram_sclk_d1(ps_sclk_d1),
        .active()
    );
    // Tie off the Path-A-only reader/arbiter nets so the shared mux/controller degrade
    // cleanly: replay=rd_busy=0 selects ps_*, and wr_req/wr_busy=0 leaves pipe_active
    // keyed on ~ps_idle (the tri-master's idle is a clean drained signal).
    assign rd_busy = 1'b0; assign rd_active = 1'b0;
    assign rd_io_o = 4'h0; assign rd_io_oe = 1'b0; assign rd_cs = 1'b1; assign rd_sclk = 1'b0;
    assign rd_req = 1'b0;  assign wr_req = 1'b0;  assign wr_busy = 1'b0;
    assign rd_gnt = 1'b0;  assign wr_gnt = 1'b0;
`endif

    // ---- unified capture / drain controller ----
    // On arm, latch which producers are in this run (run_adc/run_la), start the
    // writer, then hold `active` until BOTH armed producers have finished AND both
    // rings have drained to PSRAM (pipe quiet for a margin) — then stop the writer
    // and raise cap_done (STATUS CAP_DONE) so firmware can read both regions back.
    reg        cap_active, cap_done_r, run_adc, run_la;
    reg [6:0]  quiet;
    // Writer "still has work" indicator.  Under the bus arbiter `ps_idle` (=in S_IDLE)
    // blips HIGH whenever the writer is merely WAITING FOR A GRANT with data pending,
    // so it is no longer a reliable "drained" signal — OR in the arbiter handshake
    // (wr_req = staging FIFO non-empty, wr_busy = mid-burst) which stays high across a
    // grant wait.  (`~ps_idle` kept too; as an OR term it can only add activity.)
    // (The ADC ring's ~empty term is gone with the ring; ADC bytes now sit in the
    //  writer's staging FIFO, covered by wr_req | wr_busy | ~ps_idle below.)
    wire       pipe_active = adc_wr_stb | la_wr_stb | ~la_ring_empty
                             | ~ps_idle | wr_req | wr_busy;
    wire       producers_done = (~run_adc | adc_done_r) & (~run_la | la_done);
    always @(posedge clk) begin
        dual_stop <= 1'b0;
        if (rst) begin
            cap_active<=1'b0; cap_done_r<=1'b0; run_adc<=1'b0; run_la<=1'b0; quiet<=7'd0;
        end else if (arm) begin
            cap_active<=1'b1; cap_done_r<=1'b0; quiet<=7'd0;
            // A producer is "in this run" only if it was armed AND its count is
            // non-zero.  The unified OP_CAPTURE pulses BOTH arms even when one
            // count is 0 (ADC-only or LA-only via one command); without the count
            // gate, run_la would wait on an LA producer that captures 0 samples and
            // never asserts done -> CAP_DONE would hang.
            run_adc<=arm_adc;
            run_la <=arm_la;
        end else if (cap_active) begin
            if (!producers_done)   quiet<=7'd0;
            else if (pipe_active)  quiet<=7'd0;
            else if (quiet==7'd100) begin
                dual_stop<=1'b1; cap_active<=1'b0; cap_done_r<=1'b1;
            end else quiet<=quiet+7'd1;
        end
    end

    // ---- capture-tied DAC auto-stop DOWN-counter (24 MHz clk) ----
    // Loaded at the capture t0 (arm) with the firmware-set threshold (clk cycles), counts DOWN
    // while the capture is active and trips ONCE when it reaches 1.  A down-counter + compare-to
    // -1 avoids a separate threshold register and a wide magnitude comparator (LC-lean — the
    // up5k is ~86% full).  cnt==0 means disarmed (firmware writes 0 to disarm), so it never
    // trips; `cap_active` gates it, so a capture that ends before the count expires never trips
    // — the cut is bounded to the capture window.  (v35) While a triggered capture waits, the
    // counter holds, so it counts from the trigger cycle instead of the arm.
    reg  [31:0] cap_stop_cnt = 32'd0;
    wire        cap_counting = cap_active & ~trig_wait;
    always @(posedge clk) begin
        if (rst) begin
            cap_stop_cnt <= 32'd0;
        end else if (arm) begin
            cap_stop_cnt <= dac_stop_after;                     // 0 => disarmed
        end else if (cap_counting && cap_stop_cnt != 32'd0) begin
            cap_stop_cnt <= cap_stop_cnt - 32'd1;               // reaches 1 -> trip below -> 0
        end
    end
    // 1-cycle trip the single cycle the active capture's counter is at 1 (then it decs to 0).
    assign dac_autostop = cap_counting & (cap_stop_cnt == 32'd1);

    // Latch that holds the deep-replay reader gated off from the trip until the next DAC arm.
    // NB: cleared by the RAW dac_start (not dac_start_eff) on purpose — under a co-trigger the
    // engine start is deferred to `arm`, but we want the reader re-enabled at STAGE time (the
    // START_DAC_PSRAM opcode) so its FIFO prefills before t0; the first sample then pops at t0.
    always @(posedge clk) begin
        if (rst)               dac_autostop_lat <= 1'b0;
        else if (dac_start)    dac_autostop_lat <= 1'b0;   // a new DAC arm re-enables replay
        else if (dac_autostop) dac_autostop_lat <= 1'b1;
    end

    // ---- STATUS aggregation ----
    wire cap_busy_w  = cap_active | ~ps_idle;
    // Sticky overflow: a clean arm clears it; any drop during a run sets it.  Sources:
    // adc_ovf_r (ADC staging-FIFO full at write), la_overflow (LA producer), la_ring_ovf
    // (LA SPRAM ring).  The ADC no longer has its own ring, so adc_ring_ovf is gone.
    reg  cap_overflow_r;
    always @(posedge clk) begin
        if (rst)                                       cap_overflow_r <= 1'b0;
        else if (arm)                                  cap_overflow_r <= 1'b0;
        else if (adc_ovf_r | la_overflow | la_ring_ovf) cap_overflow_r <= 1'b1;
    end

    // ---- iCE40 multi-image swap (reflash-based; NO SB_WARMBOOT) ----
    // Image swap on this board is done by the STM32 REFLASHING the config flash with the
    // selected image ({"cmd":"fpga_image"} -> ice40_reflash_image + CRESET), NOT by an
    // in-fabric SB_WARMBOOT.  A runtime SB_WARMBOOT can never work here: the config flash and
    // the PSRAM SHARE SCK/SO/SI (they ARE the iCE40 config-SPI pins — see the .pcf), so the
    // reconfig controller can't read the flash and CDONE never rises (HW-confirmed 2026-07-21,
    // even a warmboot to the running image hangs while a cold boot is clean).  The SB_WARMBOOT
    // hard block + its pad-tristate sequencer + the OP_WARMBOOT (0x17) decode were removed
    // (gateware v24) because a stray 0x17 would fire a reconfig that wedges the fabric until a
    // reflash/power-cycle.  0x17 stays RESERVED in the opcode table (firmware never sends it —
    // fpga_warmboot() reflashes) so a future board with an off-bus config flash can reuse it.

    // Shared quad bus: tristate on RAW bus_own (fail-safe-to-release).  Explicit SB_IO with
    // OUTPUT_ENABLE (the inferred `?:1'bz` did NOT release the SPI_SO config pin — see the
    // psram_io0 note).  psram_cs_force statically drives a known pattern for the boot /CE-net
    // self-test.
    wire        drive  = ~bus_own;
    // SHARED-BUS pad mux: route the PSRAM pins to the reader whenever it is mid-burst,
    // else to the capture writer (also the safe default between bursts: the writer sits
    // in S_IDLE with CS high / OE low).  Select on `rd_active` = the reader's clk48 CS-low
    // flop (~cs): it tracks the ACTUAL pad activity in the clk48 domain the pads live in, so
    // the mux and the reader's clk48 pad registers switch together.  (`rd_busy` is the
    // clk-domain arbiter-facing flop (st != S_IDLE) — the arbiter runs on clk and reads it
    // directly; the reader FSM is on clk, so no CDC.)  `drive` still gates both on bus_own;
    // each data SB_IO wires D_IN_0 back to rd_io_i.
    wire        replay = rd_active; // 1 = READER driving the bus (clk48 CS-low)
    wire        io_oe  = drive & (replay ? rd_io_oe : ps_io_oe);
    wire [3:0]  io_dat = replay ? rd_io_o : ps_io_o;
    wire        cs_lvl = replay ? rd_cs   : ps_cs;
    wire        ps_cs_out   = psram_cs_force ? 1'b0    : cs_lvl;
    wire        ps_io_oe_f  = psram_cs_force ? 1'b1    : io_oe;
    wire [3:0]  ps_io_f     = psram_cs_force ? 4'b0011 : io_dat;
    // DDR SCLK pad (PIN_TYPE[3:2]=00): for CAPTURE writes the pad shows D_OUT_0 during
    // the clk48-high half and D_OUT_1 during the low half, so with D_OUT_0=0/D_OUT_1=
    // gate the SCLK RISES at the clk48 negedge = dead-centre of each data nibble's eye
    // (robust mode-0 setup, independent of the SB_GB skew).  For deep-replay READS the
    // reader supplies a plain clk48-domain SCLK level (rd_sclk); driving BOTH DDR halves
    // with it just re-registers it on clk48 into a clean ~24 MHz read clock.
    // psram_cs_force -> static high for the boot /CE-net self-test.
    wire        ps_sclk_d0   = psram_cs_force ? 1'b1 : (replay ? rd_sclk : ps_sclk_d0_tri);
    wire        ps_sclk_d1_f = psram_cs_force ? 1'b1 : (replay ? rd_sclk : ps_sclk_d1);

    SB_IO #(.PIN_TYPE(6'b100000), .PULLUP(1'b0)) io_sclk_i (
        .PACKAGE_PIN(psram_sclk), .OUTPUT_ENABLE(drive), .OUTPUT_CLK(clk48),
        .D_OUT_0(ps_sclk_d0), .D_OUT_1(ps_sclk_d1_f));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_cs_i (
        .PACKAGE_PIN(psram_cs),   .OUTPUT_ENABLE(drive),      .D_OUT_0(ps_cs_out));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_d0_i (
        .PACKAGE_PIN(psram_io0),  .OUTPUT_ENABLE(ps_io_oe_f), .D_OUT_0(ps_io_f[0]), .D_IN_0(rd_io_i[0]));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_d1_i (
        .PACKAGE_PIN(psram_io1),  .OUTPUT_ENABLE(ps_io_oe_f), .D_OUT_0(ps_io_f[1]), .D_IN_0(rd_io_i[1]));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_d2_i (
        .PACKAGE_PIN(psram_io2),  .OUTPUT_ENABLE(ps_io_oe_f), .D_OUT_0(ps_io_f[2]), .D_IN_0(rd_io_i[2]));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_d3_i (
        .PACKAGE_PIN(psram_io3),  .OUTPUT_ENABLE(ps_io_oe_f), .D_OUT_0(ps_io_f[3]), .D_IN_0(rd_io_i[3]));

    // ---- shared signal-engine control plane (v2: 14 LA channels, version 35) ----
    // GATEWARE_VERSION 35 = v34 + PIN READ-BACK + CAPTURE TRIGGER (both images):
    //   * OP_GPIO_GET (0x43): the 12 LA levels through a 2-flop synchroniser (engine_block), 2 bytes
    //     LE — firmware reads a gpio input pin's level without arming a capture.
    //   * OP_SET_TRIGGER (0x33) [channel][mode][flags] + OP_TRIGGER_STATUS (0x34): gated start.  An
    //     arm loads the producers but holds them (la_psram_capture `hold`, the ADC producer's first
    //     sample) until rising/falling/high/low is seen on the synced level; that cycle becomes t0
    //     for the DAC co-trigger and SET_DAC_STOP_AFTER.  Untriggered captures (mode 0, the reset
    //     default) are cycle-identical to v34.
    //   * NO LA pre-trigger.  A drop-oldest window in the LA SPRAM ring (spram_ring16 pre_len/
    //     hold_out + an `uncounted` la_psram_capture mode, plus SET_TRIGGER's optional [pre(2)]
    //     and TRIGGER_STATUS bytes 1-2) was built and it WORKED in sim — but it took the deep
    //     image from 4155 to 4280 LC and, at that fill, its clk48 closed on only 7 of 16 seeds
    //     with a best of 49.75 MHz (v34: 13/16, best 51.13).  clk48 margin under 50 has silently
    //     HW-regressed deep DAC PSRAM replay before (see the v28/v31/v32 seed notes in the
    //     Makefile), so the window was dropped rather than shipped on a thin placement.  Bring it
    //     back only after an LC-reduction pass on the deep image.
    //   tb_top_capture covers the four modes, the co-trigger + stop-after relative to the trigger,
    //   abort while waiting, and re-arm with different parameters.
    // GATEWARE_VERSION 34 = v33 + ROBUSTNESS PASS after the v33 DAC restart bug:
    //   * cdc_pulse_payload: ONE module for every clk->clk48 pulse(+value) crossing — DAC start
    //     with period/divider, DAC stop, the loop's ADC sample.  It writes the value one clk48
    //     cycle before it emits the pulse, so the v33 bug class can't return through another
    //     hand-rolled sync chain.  Same latency as v33's DAC crossing and v33's adc48 path.
    //   * dac8551_engine SERIAL TIMING vs the DAC8551 datasheet (SLAS429E 6.6): SYNC rose on the
    //     same clk48 edge as the 24th SCLK fall (t7 >= 0 ns — which pin won was routing skew), and
    //     the next frame's SYNC fell 5 clk = 104 ns after it at divider 2 against t9 >= 100 ns
    //     (less at divider 0/1; the loop image powers up at divider 2).  SYNC now rises one clk
    //     later (no rate cost) and the inter-sample divider is floored at 3 (t9 >= 125 ns).  A
    //     sample is max(divider,3)+51 clk48, so the peak DAC rate drops 905.7 -> 888.9 kS/s;
    //     firmware floors its DAC divider at 3 to match (harmless on older gateware).
    //   * LC reduction (the loop image was failing placement on half the seeds at 87%): cmd_dispatch's
    //     strobe-qualified config fields (GPIO_SET / GPIO_STEP / SWD_ARM / I2C_CONFIG / UART_CONFIG) and
    //     cap_count / la_cap_count are now WIRES decoded off the collected payload instead of ~145
    //     registered copies.  Every consumer copies them in the 1-cycle strobe, while arg_buf /
    //     rx_byte / current_cmd still hold that command — read in the pulse cycle, never through an
    //     intermediate copy.  Plus fsm_encoding="none" on the i2c_target / uart_engine FSMs.  ~-200 LC,
    //     trace-identical against the old netlist in a random-command top-level bench.
    //   * clk48 margin: the seed sweeps of this pass found the DAC engine's frame-end clock-enable
    //     cone binding clk48 in both images (~21 ns: pulse/running -> st/ph -> bitc==23 ->
    //     psram_mode -> sidx/bitc CEN).  cdc_pulse_payload's pulse is now a flop, the engine tests
    //     a registered `last` flag instead of the 5-bit compare, and sidx/sleft no longer gate on
    //     psram_mode.  All cycle-exact (tb_dac8551 checks the pins and clk/sample).
    //   * dac_psram_reader (deep image): START_DAC_PSRAM raises `run` on the same clk edge it writes
    //     base/len, and the reader reloaded its address only while !run — so the first burst of
    //     every re-arm read the PREVIOUS replay's region (found by the new tb_dac_psram_replay
    //     re-arm pass).  Its FSM now starts one clk after `run`.
    //   tb_cdc_pulse_payload, tb_dac8551 (pin timing t5-t9 + clk/sample) and the second-run passes
    //   in tb_top_capture / tb_dac_psram_replay lock these in.
    // GATEWARE_VERSION 33 = v32 + DAC RESTART PERIOD FIX (the clk->clk48 DAC control CDC above): the
    // clk48 copies of START_DAC's period/divider were latched by the synchronized start pulse, and
    // dac8551_engine loaded sleft/div_cnt from them on THAT SAME pulse — so every start used the
    // PREVIOUS start's period for its first pass and the previous divider for its first sample.
    // A start after a longer waveform therefore played past the end of the new one into stale
    // BRAM: on the bench a 5 kHz square (181 samples) after a 200 Hz one (2034) gave ~2.25 ms of the
    // old square's flat levels before looping correctly, which broke
    // test_adc_sample_rate_matches_the_host_clock after test_stop_dac_after_cuts_the_output_mid_capture
    // (the capture-tied auto-stop was only the previous test, not the cause).  START_MEASURE and the
    // co-triggered start had it too.  The engine's start/stop now come one clk48 stage after the
    // latch (+2 FF; DAC sample 0 lands ~21 ns later, well inside a co-trigger's ~1.1 us sample).
    // tb_top_capture checks the frames played after a restart onto a shorter waveform.
    // GATEWARE_VERSION 32 = v31 + EXACT CAPTURE DIVIDERS (la_psram_capture, adc_mcp33131): the LA
    // and ADC sample period is now EXACTLY `divider` clocks, as the protocol and the firmware's
    // reported rate (24 MHz / divider) always assumed.  Both engines took divider + 1: the LA's
    // hi-byte cycle did not count toward the period, and the ADC's registered period_zero flag
    // added a cycle.  So the real LA/ADC rate was d/(d+1) of the reported one — measured on the
    // bench against a DUT UART as 0.922x at 2 MS/s, 0.959x at 1 MS/s — which skewed every LA
    // timing and made MEASURE's ADC drift against the (correctly compensated) DAC.  DAC→ADC
    // loopback could never see it: both sides share the FPGA clock.  Constants-only change in
    // both engines (no LC cost).  Firmware sends divider - 1 to gateware < 32
    // (EXACT_CAP_DIVIDER_MIN_GW) so the rate is right on either.  NOTE the LA's divider 2 now
    // really is 12 MS/s (it was 8), so firmware re-expressed the ring drain budget in real units.
    // GATEWARE_VERSION 31 = v30 + DAC SOURCE-SELECT FIX (cmd_dispatch): a BRAM-waveform DAC start
    // now SELECTS the BRAM waveform.  dac_psram_mode / dac_loop_mode are LEVEL registers that only
    // OP_STOP_DAC (0x12) ever cleared, so OP_START_DAC (0x11) — which cleared dac_loop_mode but not
    // dac_psram_mode — and OP_START_MEASURE (0x30) — which cleared NEITHER — kicked the engine while
    // a PREVIOUS source was still selected.  Consequences, both silent: in the deep image the
    // dac_psram_reader stays the source and keeps REQUESTING THE SHARED QUAD BUS, so a `measure`
    // (which arms a capture in the same cycle) has the reader contending with the capture writer for
    // the PSRAM pads — the 0x5555-family corruption class; in the loop image rd_strm_valid is tied 0,
    // so the engine starves and the DAC holds a constant level instead of playing the waveform.  A
    // `generate`/`measure` issued after a deep `replay` without an intervening `dac_stop` hit this.
    // Both opcodes now clear both mode levels.  Cost: two extra decode terms on registers the build
    // already has (the deep image DCEs dac_loop_mode away entirely — it is unused there).
    // GATEWARE_VERSION 29 = v28 + SELECTABLE CONTROL-LOOP INPUT SOURCE (dac_loop): the loop's
    // input no longer has to be the live ADC.  A new OP_DAC_LOOP_SRC (0x1A) picks between the
    // ADC (closed loop, the power-on default = exactly the pre-v29 behaviour), a host-held
    // FIXED value, or an internal per-tick SWEEP accumulator — the last two run the same curve
    // + damping + clamp datapath OPEN-loop, with the ADC and the whole analog input path out of
    // the picture.  That is what makes the DAC/output side testable on its own: hold a curve
    // point, meter the SMA, move to the next one — if that is wrong, no amount of loop debugging
    // was ever going to help.  OP_DAC_LOOP_IN_PROBE (0x1B) reads back the input the last tick
    // actually used, so telemetry never has to infer it (in fixed/sweep mode ADC_PROBE is
    // reporting something the loop never looked at).  The source registers are separate from the
    // arm opcode on purpose: a RUNNING loop picks up a new source/value on its next tick, so the
    // host steps through curve points without re-arming or re-uploading the curve.  Cost is a
    // 3:1 input mux + one 16-bit accumulator in the clk48 loop.
    // GATEWARE_VERSION 28 = v27 + DEEP-REPLAY READER STRUCTURAL FIX (dac_psram_reader): the
    // recurring "deep replay silently dead after a seed/yosys/netlist change while static timing
    // stayed green" curse is CURED at the source.  ROOT CAUSE: the old reader generated the PSRAM
    // read SCLK from a `clk`-domain signal into a clk48 DDR pad and sampled the return data in the
    // `clk` domain; since `clk` is clk48/2 via a FABRIC divider + SB_GB, nextpnr treats clk/clk48 as
    // unrelated and never timed those pad paths, so the SCLK-to-sample skew re-rolled on every
    // placement and starved the reader (the DAC held a constant level — the 0x5555/seed-roulette
    // PSRAM class).  FIX: split the reader like the PROVEN psram_dual_writer — the 24-bit address
    // arithmetic + burst sequencing FSM stay on `clk` (they close there), while a clk48 serializer
    // drives io_o/oe/cs + the SCLK gate from clk48 REGISTERS and samples the return data on clk48 at
    // a FIXED phase.  Now every PSRAM-facing edge is one tool-timed, placement-INVARIANT clk48
    // relationship.  The prefetch FIFO is also single-clock (captured+read on clk48), deleting the
    // dual-clock gray-pointer + odd-pop byte-swap wedge class.  No protocol change; deep replay is
    // byte-identical, just no longer placement-fragile.  A skew-modelled regression bench
    // (tb_dac_psram_skew) locks it in.  Reader PSRAM paths are now OFF the clk48 critical path.
    // GATEWARE_VERSION 27 = v26 + HARDWARE CO-TRIGGERED DAC START: a new 0-arg OP_DAC_ARM_ON_CAPTURE
    // (0x19) sets `dac_pend`; while pending, the next START_DAC / START_DAC_PSRAM latches its
    // period/divider/base/len and (deep) PRIMES the reader FIFO, but its engine-start pulse is HELD
    // and re-issued on the next capture `arm` — so DAC sample 0 and the capture t0 fire on the SAME
    // 24 MHz clk cycle (dac_start_eff = (dac_start & ~dac_pend) | (arm & dac_pend)).  The reader's
    // lat-clear stays on the RAW dac_start so the deep FIFO prefills at stage time and the first
    // PSRAM sample pops at t0 with no fill latency.  Nothing on the clk48-critical path changed
    // (the added gate is on the relaxed 24 MHz domain).  STOP_DAC cancels a staged-but-uncaptured
    // start.  Sub-v27 gateware ignores 0x19 (unknown opcode) so firmware falls back to the old
    // sequential start.  Both images (loop + deep) carry it.
    // GATEWARE_VERSION 26 = v25 + RUNTIME-IMAGE-SWAP PSRAM-WEDGE FIX (deep image only): the
    // deep-replay reader + burst arbiter are now held in RESET while the STM32 owns the bus
    // (deep_rst/deep_rst48 = rst | bus_own).  Previously they kept their FSMs running through the
    // STM's post-reconfig psram_init bus-yank + PSRAM reset during a swap TO the deep image, which
    // intermittently desynced the arbiter so it stopped granting the writer — captures then wrote
    // nothing ("sentinel survived") until a fresh reconfig / power-cycle.  The LOOP image touches
    // PSRAM only when a capture is armed, so it was immune and its netlist is byte-unchanged here.
    // GATEWARE_VERSION 25 = v24 + RETIRE the on-FPGA I2C-bus logic-analyzer (P5): the
    // i2c_la_capture sampler + its la_capture_buf SPRAM trace buffer + the OP_I2C_LA_START
    // (0x65) / OP_I2C_LA_READ (0x66) decode + the S_LA_READ read-back stream were all removed,
    // reclaiming ~150 LC + an SB_SPRAM256KA block.  The `sensor_la` feature is UNCHANGED at the
    // wire level: the firmware now samples the two emulated-I2C pins through the general
    // deep-LA-into-PSRAM path (OP_LA_CAPTURE 0x69) and re-packs them into the same
    // 4-samples/byte layout, so the server/python I2C decoder needs no change.  NOT
    // protocol-transparent (0x65/0x66 gone, reserved).  HW-verified: sensor_la returns
    // byte-identical packed data (idle bus 0xFF, SCL-low 0x55).
    // GATEWARE_VERSION 24 = v23 + an LC/stability pass (docs/ice40-lut-optimization.md):
    //   * REMOVED the dead SB_WARMBOOT hard block + its pad-tristate sequencer + the OP_WARMBOOT
    //     (0x17) decode.  Runtime SB_WARMBOOT can never work on this board (config-SPI = the
    //     shared PSRAM bus), so image swap is reflash-based; a stray 0x17 fired a reconfig that
    //     WEDGED the fabric until reflash.  0x17 stays reserved in the opcode table.
    //   * cmd_dispatch: dropped the write-only 16-bit arg_len (now an 8-bit len_lo), narrowed
    //     arg_count 16->12 bits, and dac_stop_after 32->28 bits.
    //   * uart_engine: narrowed the bit-period counters 24->18 bits (>=92 baud), div clamped.
    //   * dac_loop: tick timer is now a registered down-counter (no 16-bit clk48 magnitude
    //     compare — the last wide clk48 compare, the class that makes seeds fragile).
    // Behaviour change: OP_WARMBOOT (0x17) is now a no-op instead of a fabric-wedging reconfig;
    // all captures/replays/loops are byte-for-byte unchanged.  Re-swept the placement seed.
    // GATEWARE_VERSION 21 = v20 + CAPTURE-TIED DAC AUTO-STOP: a new OP_SET_DAC_STOP_AFTER (0x14)
    // latches a threshold (24 MHz clk cycles) that firmware writes before arming a capture; a
    // counter seeded at the capture's hardware t0 (arm) trips at the threshold and cuts a
    // concurrently-running DAC (both the clk48 DAC8551 STOP and the deep-replay reader `run`),
    // so the captured window shows the DAC switch off at a sample-precise point instead of via
    // an imprecise host timer.  NOT protocol-transparent (new opcode); all existing captures/
    // replays are byte-for-byte unchanged when the threshold is 0 (disarmed).
    // GATEWARE_VERSION 20 = v19 + a deep-DAC-replay byte-alignment fix (no protocol change).
    // dac8551_engine pops the reader's prefetch FIFO in (lo,hi) PAIRS; a replay STOPPED
    // mid-pair (every capture/replay cycle does, via STOP_DAC) left the FIFO popped an ODD
    // number of bytes, so its head sat on a HIGH byte and the NEXT replay popped hi-as-lo,
    // byte-SWAPPING every sample (0xC000 -> 0x00C0 ~= 0 V).  Under sustained stop/start this
    // LATCHED until an FPGA reconfig (HW-found: sustained hwe2e concurrent+deep-replay wedged
    // replay to ~0 V; only flash-ice40 cleared it — not psram_init/bus-cycle/re-arm).  Fix =
    // two coupled changes in dac_psram_reader: (1) FLUSH the prefetch FIFO on every arm
    // (hold both ends in reset while run=0) so it always starts empty and byte-aligned; and
    // (2) PIPELINE the reader<->DAC pop handshake (`fetch` no longer depends combinationally
    // on the DAC's data_pop this cycle — one clk48 refill bubble, harmless vs the DAC's
    // <=1.8 MB/s drain).  The pipeline break moves the FIFO OFF the clk48-critical path,
    // which is what makes the flush's reset gating affordable (naive flush alone dropped
    // clk48 to ~44 MHz).  clk48 closes at 48.2 MHz (seed 12).  PROTOCOL-TRANSPARENT.
    // GATEWARE_VERSION 19 = v18 + an LC-reduction pass (no protocol/behaviour change):
    // the shallow ADC capture stream no longer has its own deep SPRAM ring — the ADC
    // producer writes straight into psram_dual_writer's 32-byte ADC staging FIFO (the
    // ADC is ~0.8 MB/s and the arbiter is starvation-free, so the staging FIFO covers
    // any bus-busy window).  That frees a whole spram_ring16 (~120 LC + an SB_SPRAM256KA
    // block) the LA burst still needs, giving placement headroom on the ~87%-full up5k.
    // Also removed dead plumbing: the never-driven corr_start net and the retired
    // correlated-capture i2c_la aux taps (i2c_la_cap_*/i2c_la_start_aux) + three unused
    // source files (la_capture.v / spram_ring.v / psram_writer.v).  PROTOCOL-TRANSPARENT
    // — the bump just lets `ping`/the cloud confirm the reduced netlist is deployed.
    // GATEWARE_VERSION 18 = v17 + CONCURRENT PSRAM: a burst-granular psram_bus_arbiter
    // time-multiplexes the one quad bus between the DAC read master and the ADC/LA write
    // master, so deep DAC replay and an ADC/LA capture can run AT THE SAME TIME in disjoint
    // regions (was mutually exclusive).  Region map re-laid (sys_config.vh): LA at the FRONT
    // (base 0), ADC fixed at 4 MB, DAC replay floating DOWN from the top (base = TOTAL -
    // dac_bytes, sent per-command in 0x13 — the reader base was already a runtime knob).
    // The pad mux now selects on arbiter ownership, not rd_active.  Firmware caps LA/ADC to
    // keep the capture below dac_base (clean budget: max_LA = (TOTAL - dac_bytes)/2).
    // GATEWARE_VERSION 17 = v16 + DEEP DAC REPLAY: OP_START_DAC_PSRAM (0x13) streams a
    // waveform straight out of PSRAM to the DAC8551 via a new clk48 QPI read master
    // (dac_psram_reader) + prefetch FIFO, so replay depth jumps from the 4 KB LOAD_WAVE
    // BRAM (2048 samples) to the full 8 MB PSRAM (up to 4,194,304 samples) — enough to
    // replay a whole stored ADC recording.  The reader shares the PSRAM quad pads with
    // the capture writer (muxed on rd_active; replay and capture are mutually exclusive)
    // and the data pads gain a D_IN_0 read path.  The STM32 stages the waveform into
    // PSRAM over XSPI, releases the bus, then arms 0x13.  NOT protocol-transparent (new
    // opcode); DAC BRAM path (LOAD_WAVE/START_DAC 0x11) is unchanged for shallow replay.
    // GATEWARE_VERSION 16 = v15 + DEEP ADC: the ADC sample count is widened 16->24-bit
    // end-to-end (cmd_dispatch cap_count, top_v2 adc_left) so a single capture can span
    // the multi-MB PSRAM ADC region (up to 2,097,152 samples, ~5.2 s @ 0.4 MS/s).  The
    // 8 MB PSRAM is re-laid to a 4 MB/4 MB ADC/LA split (PSRAM_ADC_BASE/LA_BASE in
    // sys_config.vh), and the unified OP_CAPTURE (0x31) payload grows 8->10 bytes so
    // BOTH its ADC and LA counts are full 24-bit (each read back chunked from PSRAM).
    // NOT protocol-transparent: OP_CAPTURE arg layout changed (firmware must match).
    // Placement re-swept for clk48@48 + clk@24 closure with the wider counters.
    // GATEWARE_VERSION 15 = v14 + the unified-capture ADC-count truncation fix: the
    // OP_CAPTURE (0x31) and OP_START_MEASURE (0x30) collect paths in cmd_dispatch now
    // latch the FULL 16-bit cap_count (they masked it to 13 bits, capping deep unified
    // captures at 8191 samples).  Placement re-tuned to --seed 3 (clk48 closes with the
    // now-live top 3 count bits).  PROTOCOL-TRANSPARENT — no opcode change; the bump
    // just lets `ping`/status confirm the fixed netlist is the one deployed.
    // GATEWARE_VERSION 14 = v13 + a LUT-reduction pass (i2c_target bus-recovery
    // watchdog -> power-of-two threshold; the shallow OP_LA_START `la_capture` gated
    // out of v2 via engine_block HAS_WIDE_LA=0).  PROTOCOL-TRANSPARENT — no opcode or
    // behaviour change; the bump is only so `ping`/the cloud can tell the reduced
    // netlist (79% LC, all seeds place) is the one deployed (docs/ice40-lut-optimization.md).
    // A single-48-MHz-fabric experiment on top of this is preserved on the
    // `single-48-experiment` branch (sim-verified but timing-blocked ~39 MHz — see
    // docs/ice40-48mhz-migration.md).  v13 = v12 (faster ADC ~400 kS/s + 6-byte
    // START_MEASURE) plus:
    // the DAC8551 sequencer now runs on clk48 (48 MHz) instead of clk (24 MHz), so
    // the DAC update rate doubles (~905 kS/s) for smoother output — see
    // docs/ice40-48mhz-migration.md.  Firmware must scale the DAC divider maths to a
    // 48 MHz DAC clock (DAC_CLK_HZ) and, for MEASURE, dac_div = 2*cap_div - K (the
    // ADC is still on the 24 MHz clk).  v12 = v11 (unified ADC+LA capture + 48 MHz
    // DDR PSRAM drain) + the ADC CONV_CYCLES/floor speedup + 6-byte measure dividers.
    // The correlated/I2C-LA capture path is retired: i2c_la_cap_we/wdata are left
    // open and i2c_la_start_aux tied 0.  cap_raddr open, cap_rdata 8'h00 (v2 reads
    // capture data back from PSRAM).
    wire [7:0] led_ctrl;
    wire       i2c_la_busy;

    // (SB_WARMBOOT + the pad-tristate-before-fire logic are declared above, near the pad mux.)

    // FPGA_FEATURES byte: which optional block THIS warmboot image carries (they are
    // mutually exclusive across images).  The firmware reads it to advertise capabilities
    // per running image (closed-loop vs deep-replay), not by version.
`ifdef USE_DEEP_REPLAY
    localparam [7:0] IMG_FEATURES = 8'h02;   // deep-DAC-PSRAM-replay
`else
    localparam [7:0] IMG_FEATURES = 8'h01;   // closed-loop DAC control
`endif
    engine_block #(.N(14), .GATEWARE_VERSION(8'd35), .FEATURES(IMG_FEATURES)) engines_i (
        .clk(clk), .rst(rst),
        .sck(sck), .mosi(mosi), .miso(miso), .csn(csn),
        .la(la),
        .led_ctrl(led_ctrl),
        .i2c_la_busy(i2c_la_busy),
        .wave_rclk(clk48), .wave_raddr(wave_raddr), .wave_rdata(wave_rdata),
        .dac_start(dac_start), .dac_stop(dac_stop_pulse),
        .dac_cotrig(dac_cotrig),
        .dac_period(dac_period), .dac_divider(dac_divider), .dac_running(dac_running),
        .dac_psram_mode(dac_psram_mode), .dac_psram_base(dac_psram_base), .dac_psram_len(dac_psram_len),
        .dac_stop_after(dac_stop_after),
        .dac_loop_mode(dac_loop_mode), .dac_loop_k(dac_loop_k),
        .dac_loop_vmin(dac_loop_vmin), .dac_loop_vmax(dac_loop_vmax),
        .dac_loop_tick(dac_loop_tick),
        .dac_loop_src(dac_loop_src), .dac_loop_in(dac_loop_in), .dac_loop_step(dac_loop_step),
        .dac_loop_in_zero(dac_loop_in_zero), .dac_loop_in_gain(dac_loop_in_gain),
        .dac_loop_in_trip(dac_loop_in_trip),
        .dac_loop_map_en(dac_loop_map_en),
        .dac_loop_trip_en(dac_loop_trip_en), .loop_tripped(loop_tripped),
        .cap_start(cap_start), .cap_count(cap_count), .cap_divider(cap_divider),
        .adc_cap_base(adc_cap_base),
        .cap_test_ramp(cap_test_ramp), .psram_cs_force(psram_cs_force),
        .trig_ch(trig_ch), .trig_en(trig_en), .trig_edge(trig_edge), .trig_pol(trig_pol),
        .trig_wait(trig_wait), .trig_fired(trig_fired),
        .cap_busy(cap_busy_w),
        .cap_done(cap_done_r),
        .cap_overflow(cap_overflow_r),
        .adc_dbg_sample(adc_sample),   // live 24 MHz ADC sample -> ADC_PROBE direct read
        .dac_loop_v_dbg(loop_v_clk),   // clk snapshot of the loop's DAC output -> DAC_PROBE
        .dac_loop_in_dbg(loop_in_clk), // clk snapshot of the loop's INPUT -> DAC_LOOP_IN_PROBE
        .la_cap_start(la_cap_start), .la_cap_count(la_cap_count),
        .la_cap_divider(la_cap_divider), .la_sample(la_sample), .la_levels(la_levels)
    );

    // BUSY (active low): any capture in flight or LA busy.
    assign busy = ~(cap_busy_w | i2c_la_busy | la_busy);

    // ---- Status LEDs on the iCE40 dedicated RGB-driver pads (SB_RGBA_DRV) ----
    localparam [7:0] LED_DUTY = 8'd8;         // ~3% duty — dim
    reg  [7:0] pwm_cnt = 8'd0;
    always @(posedge clk) pwm_cnt <= pwm_cnt + 8'd1;
    wire dim = (pwm_cnt < LED_DUTY);
    wire green_on  = led_ctrl[0] & dim;
    wire yellow_on = led_ctrl[1] & dim;
    wire red_on    = led_ctrl[2] & dim;

    SB_RGBA_DRV #(
        .CURRENT_MODE("0b1"),
        .RGB0_CURRENT("0b000001"),
        .RGB1_CURRENT("0b000001"),
        .RGB2_CURRENT("0b000001")
    ) rgb_drv (
        .CURREN  (1'b1),
        .RGBLEDEN(1'b1),
        .RGB0PWM (green_on),
        .RGB1PWM (yellow_on),
        .RGB2PWM (red_on),
        .RGB0    (led_g),
        .RGB1    (led_b),
        .RGB2    (led_r)
    );
endmodule
