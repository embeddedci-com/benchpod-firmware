// ============================================================================
// dac8551_engine.v — looping 16-bit waveform sequencer for the TI DAC8551
// serial DAC (v2 board).
//
// Reuses the existing 8-bit waveform BRAM (sample_buf): each 16-bit sample is
// two consecutive bytes (little-endian) at wave_addr = 2*i and 2*i+1, so the
// host uploads waveforms with the normal LOAD_WAVE (no protocol change).  Each
// sample is shifted out as a 24-bit DAC8551 frame {8'h00, data[15:0]} (PD=00,
// normal) on DIN, framed by SYNC, MSB first, latched on SCLK falling edges.
//
// `period_samples` is in 16-bit samples.  Clocked on `clk`: gateware >= 13 (top_v2)
// runs it on clk48 (48 MHz) for a ~2x update rate (SCLK 24 MHz until v65, 19.2 MHz in v66, 16 MHz from v67,
// see SERIAL TIMING); the control-plane inputs cross a CDC in top_v2 and the
// waveform BRAM read port shares this clock.
//
// SERIAL TIMING (v67) vs the DAC8554 datasheet (SLAS431B, the IOVDD 2.7-3.6 V column: analog v3r2
// and later run IOVDD at 3.3 V), which is stricter than the DAC8551 (SLAS429E, VDD 3.6-5.5 V) on
// every line, so the one timing serves both boards.  SCLK and DIN leave through DDR pads
// (dac_pads), so their edges sit on a half-clk48 grid (10.4 ns).  One bit takes 3 clk:
//     halves   q0    q1    q2
//     SCLK     H H   H L   L L        16 MHz: high 3 halves, low 3 halves
//     DIN      d d   d d   d d        (d = this bit; it changes with the SCLK rise)
//   t1 SCLK period 3 clk = 62.5 ns (>= 40)
//   t2 / t3 SCLK high / low 1.5 clk = 31.3 ns each (>= 20).  Both phases run between a rising and
//      a falling clk48 edge, so the oscillator's duty cycle moves them (+-1 ns at 45/55 %), with
//      about 10 ns to spare on each
//   t4 SYNC fall -> SCLK rise 1 clk (>= 0)        t5/t6 DIN changes on SCLK RISES only: setup
//                                                 31.3 ns, hold 31.3 ns (>= 5 / 4.5)
//   t7 24th SCLK fall -> SYNC rise 1.5 clk (>= 0) t8 SYNC high >= DIV_MIN+2 clk = 125 ns (>= 40)
//   t9 24th SCLK fall -> next SYNC fall >= DIV_MIN+3.5 clk = 156 ns (>= 130)   (divider floor 4)
// Why (2026-11-24 bench, benchpod-emc si-sim/dac-sclk-alternatives.md and dac-adc-placeholders.md):
// v65's 24 MHz SCLK (1 clk high, 1 clk low) measured 21.8 / 19.8 ns at the DAC connector, under
// t3.  v66 (19.2 MHz, one phase 1 clk48 and the other 1.5) measured the one-clk48 phase at about
// 20.0-20.3 ns whichever way round, so no phase is one clk48 long any more: both are 1.5.
// Every frame gap, static frames on an idle engine included, goes through the divider floor
// below, so t8/t9 hold whatever divider the firmware last sent.
// One sample takes max(divider, DIV_MIN) + 75 clk (v66: + 63; v65 and older: + 51) — the
// firmware's DAC_SEQ_OVERHEAD_CLK model: 3 to fetch, 72 to shift.
// tb_dac8551 checks all of these on the pins (through dac_pads), half-cycle-exactly.
//
// DEEP replay (psram_mode=1, gateware >= v17): instead of the 4 KB LOAD_WAVE BRAM,
// the engine pulls each 16-bit sample (little-endian, low byte first) out of the
// dac_psram_reader prefetch FIFO, which streams the waveform straight from PSRAM and
// loops it at len_bytes.  The engine then has no depth limit of its own — it just
// pops bytes forever until `stop` — so replay spans the full 8 MB PSRAM.  The
// sidx/sleft/period loop machinery (and wave_addr) are used ONLY by the BRAM path;
// in psram_mode looping is the reader's job.  psram_mode=0 is byte-for-byte the
// original BRAM engine (guarded so tb_dac8551 / tb_dac8551_loop are unaffected).
//
// QUAD DAC8554 (analog v3r2 board, gateware >= v49).  The DAC8554 takes the same 24-bit frame
// with a control byte on top: DB23:22 A1:A0 (00), DB21:20 LD1:LD0 (01 = load and update the
// channel), DB19 x, DB18:17 the channel, DB16 PD0 (0), then the 16 data bits.  `ctrl` is that
// byte for every streamed frame; it is 0 on a DAC8551 board (the PD=00 frame it always sent).
// It crosses in with each start, so it is stable while the engine runs.  `wr_req` asks for ONE
// extra frame, `wr_frame` = {ctrl, code}, so another channel can be set while one streams: it
// takes the next sample slot (the `dz` point) and the stream continues one slot later.  The
// static frame runs through the BRAM fetch states (S_RDLO..S_LAT) whatever the source, so its
// SYNC and frame-gap timing are the stream's, and does not step sidx/sleft.  The engine also
// runs while one is pending or in flight when it is not streaming; STOP sets `dz` so an idle
// engine sends it at once.  A START or STOP during the static frame itself (about 1 us) aborts
// it, like a stream frame: the firmware sends DAC_CTRL frames after its start/stop, not before.
// ============================================================================
module dac8551_engine #(
    parameter ADDR_W = 12
)(
    input  wire              clk,
    input  wire              rst,
    input  wire              start,
    input  wire              stop,
    input  wire [ADDR_W:0]   period_samples,  // 16-bit samples before looping
    input  wire [15:0]       divider,         // v40: the RELOAD, i.e. inter-sample clocks - 1
    input  wire [7:0]        ctrl,            // v49: control byte of every streamed frame
    input  wire              wr_req,          // v49: send wr_frame once (1-clk pulse)
    input  wire [23:0]       wr_frame,        // v49: {ctrl, code}; held until the next wr_req

    // 8-bit waveform BRAM read port (shared sample_buf) — used when psram_mode=0
    output reg  [ADDR_W-1:0] wave_addr,
    input  wire [7:0]        wave_data,

    // ---- streaming source (psram_mode=1): pop 16-bit LE samples from a FIFO ----
    // Fed by dac_psram_reader.  strm_pop is a clean 1-cycle pulse per byte; the FIFO
    // has one cycle of read latency, so a byte read this cycle is popped, and the new
    // head appears the cycle after the pop.
    input  wire              psram_mode,
    input  wire [7:0]        strm_data,
    input  wire              strm_valid,
    output reg               strm_pop,

    // DAC8551 serial bus (v66): SYNC, and SCLK / DIN as DDR half pairs for dac_pads.  `*_a` is
    // the first (clk-high) half of a cycle, `*_b` the second; dac_pads puts all three on the pins
    // one clk later, so they stay aligned with each other.
    output reg               dac_sync,
    output reg               sclk_a,
    output reg               sclk_b,
    output reg               din_a,
    output reg               din_b,
    output reg               running
);
    localparam S_IDLE=3'd0, S_RDLO=3'd1, S_RDHI=3'd2, S_LAT=3'd3, S_SHIFT=3'd4,
               S_PLO=3'd5, S_PLW=3'd6, S_PHI=3'd7;   // psram_mode fetch: pop-lo, wait, pop-hi
    reg [2:0]        st;
    reg [15:0]       div_cnt;
    // `dz` = registered "divider expired" flag (== div_cnt==0), maintained
    // incrementally so the S_IDLE state transition reads ONE bit instead of a 16-bit
    // equality — that wide compare was the clk48 critical path (div_cnt -> next state).
    reg              dz;
    // Divider floor: the inter-sample gap is max(divider, DIV_MIN) clk, so the next SYNC fall
    // lands >= DIV_MIN + 3 clk after the 24th SCLK fall (t9).  v34 set DIV_MIN = 3 (125 ns) for
    // the DAC8551's 100 ns; v59 raises it to 4 (7 x 20.8 = 146 ns) for the DAC8554's 130 ns.  The
    // firmware has sent a divider of at least 4 since 3.10.10, but static frames (wr_req) on an
    // engine that was never started still used the reset divider 0, i.e. this floor.
    localparam [15:0] DIV_MIN        = 16'd4;
    // v40: the firmware sends divider - 1 (the reload), so the 16-bit compare and subtract that
    // sat in this clk48 cone are gone.  The DIV_MIN floor stays in gateware because it is a
    // DATASHEET timing guarantee (t9): a reload below DIV_MIN-1 is raised to it with a 14-bit
    // OR instead of a magnitude compare.  v59: a reload below DIV_MIN-1 = 3 is raised to 3.
    wire             div_nz          = |divider[15:2];      // reload >= 4, or ...
    wire [15:0]      div_reload      = {divider[15:2],
                                        divider[1] | ~div_nz,                 // 0..3 -> 3
                                        divider[0] | ~div_nz};
    wire             div_reload_zero = 1'b0;                  // reload >= DIV_MIN-1 >= 3
    reg [ADDR_W:0]   sidx;        // 16-bit-sample index (also the BRAM address)
    reg [ADDR_W:0]   sleft;       // samples left in this loop (= period_samples - sidx);
                                  // down-counter terminal, replaces the per-sample
                                  // `sidx + 1 >= period_samples` add+compare.
    // `slast` = registered (sleft <= 1), recomputed every clk (v42).  sleft only changes at a
    // start or a frame end, and the next frame end is >= 51 clk later, so the flag is always
    // current when the frame end reads it.  Keeps the compare out of the sleft/sidx update cone,
    // which bound the loop image's clk48 on several seeds.
    reg              slast;
    always @(posedge clk) slast <= (sleft <= {{ADDR_W{1'b0}}, 1'b1});
    reg [7:0]        lo;
    reg [23:0]       sh;
    reg [4:0]        bitc;        // v67: bits done in this frame (24 per frame)
    // `last` = registered "this is the 24th bit" (bitc == 23; v66: the 12th bit pair).  The frame-end
    // branch updates st/sidx/sleft, so its clock-enable cone decoded the 5-bit compare plus st/ph/
    // running/psram_mode in ONE clk48 cycle — the v34 sweeps found that cone binding clk48 (~21 ns)
    // in both images.  Same trick as `dz` below: test one flop instead of the wide compare.
    reg              last;
    // v67: one-hot phase of the 3-clk bit (q[0] .. q[2]); see the SCLK plan in the header.
    reg [2:0]        q;
    reg              wpend;       // a static frame is waiting for a slot
    reg              wfr;         // the frame being fetched/shifted is the static one
    // `run_i` = the FSM's enable: `running` (streaming), or a static frame pending or in flight.
    // One flop like `running` was, so the enable cone did not grow (an OR of the three in the
    // enable cost the loop image ~10 MHz of clk48 on its pinned seed).
    reg              run_i;

    always @(posedge clk) begin
        strm_pop <= 1'b0;                      // default: single-cycle pop pulse
        if (rst) begin
            st<=S_IDLE; running<=1'b0; dac_sync<=1'b1; sclk_a<=1'b0; sclk_b<=1'b0; din_a<=1'b0; din_b<=1'b0;
            div_cnt<=0; dz<=1'b1; wfr<=1'b0; run_i<=1'b0;      // (sidx/sleft: below)
        end else if (stop) begin
            st<=S_IDLE; running<=1'b0; dac_sync<=1'b1; sclk_a<=1'b0; sclk_b<=1'b0;
            dz<=1'b1;                          // v49: an idle engine takes a static frame at once
            run_i<=wpend;                      // ...if one is pending
        end else if (start) begin
            running<=1'b1; run_i<=1'b1; dac_sync<=1'b1; sclk_a<=1'b0; sclk_b<=1'b0;   // (sidx/sleft: below)
            div_cnt<=div_reload; dz<=div_reload_zero; st<=S_IDLE;
        end else if (run_i) begin
            case (st)
            S_IDLE: begin
                dac_sync<=1'b1; sclk_a<=1'b0; sclk_b<=1'b0; wfr<=1'b0;
                if (!running && !wpend) run_i<=1'b0;    // idle again (SYNC rises this edge)
                if (dz) begin
                    // (wave_addr <= 2*sidx: the datapath block below)
                    // An idle engine (only here for a static frame) keeps dz for the next one.
                    if (wpend | running) begin div_cnt<=div_reload; dz<=div_reload_zero; end
                    if (wpend) begin
                        wpend<=1'b0; wfr<=1'b1;    // this slot sends the static frame
                        st<=S_RDLO;                // BRAM-path timing; S_LAT loads wr_frame
                    end else if (running) begin
                        st <= psram_mode ? S_PLO : S_RDLO;   // stream FIFO, or the BRAM
                    end
                end else begin
                    div_cnt <= div_cnt - 16'd1;
                    dz      <= (div_cnt == 16'd1);   // becomes 0 next cycle
                end
            end
            // ---- psram_mode fetch: pop low byte, wait for FIFO read latency, pop
            //      high byte, then join the shared shift-out at S_SHIFT.  Stalls in
            //      S_PLO whenever the reader hasn't refilled (never happens after the
            //      first fill given the >2x read-bandwidth headroom, but correct if it
            //      does — the DAC just holds SYNC high until the next byte arrives). ----
            S_PLO: begin
                dac_sync<=1'b1; sclk_a<=1'b0; sclk_b<=1'b0;
                if (strm_valid) begin strm_pop<=1'b1; st<=S_PLW; end
            end
            S_PLW: st<=S_PHI;                      // pop effective; head advances to the high byte
            S_PHI: begin
                if (strm_valid) begin
                    strm_pop<=1'b1;
                    dac_sync<=1'b0; st<=S_SHIFT;
                end
            end
            S_RDLO: st<=S_RDHI;                    // BRAM latency: addr now valid (wave_addr: below)
            S_RDHI: st<=S_LAT;                     // (lo captures the low byte)
            S_LAT: begin
                dac_sync<=1'b0; st<=S_SHIFT;       // (sh loads {ctrl, high, low})
            end
            S_SHIFT: begin
                // v67: one bit per 3 clk, halves H H H L L L (see the header).
                // (sh/bitc/last/q step in the datapath block below)
                sclk_a <= q[0] | q[1];
                sclk_b <= q[0];
                if (q[0]) begin din_a <= sh[23]; din_b <= sh[23]; end
                if (q[2] & last)
                    // The 24th SCLK fall is in the middle of the q1 cycle; SYNC rises in
                    // S_IDLE one clk after this one (t7 = 1.5 clk), never on the fall itself.
                    // (the stream steps in its own block below, off `fe`)
                    st<=S_IDLE;
            end
            default: st<=S_IDLE;
            endcase
        end
        // A request wins over the take above (set after it), and survives start/stop.
        if (rst) wpend <= 1'b0;
        else if (wr_req) begin wpend <= 1'b1; run_i <= 1'b1; end
    end

    // The stream index (v51): sidx/sleft step at the frame end, i.e. the edge where the FSM sees
    // S_SHIFT with q[2] and last up (v67; v66 q[4]; before: ph).  That edge always follows a cycle in
    // S_SHIFT with q[1] and last up (q rotates, and last only changes on a q[2] edge), so `fe` registers the
    // decision one cycle early and the enable of these 2 x (ADDR_W+1) flops tests rst/stop/start
    // and one flop, instead of the st/ph/last/wfr/run_i cone that bound the v50 loop image's clk48
    // (ph -> last -> 3 LUTs -> sidx/sleft CEN, 22-23 ns on the failing seeds).  `fe` also leaves
    // out a cycle with rst/stop/start, which takes the FSM out of S_SHIFT before that edge.
    //   Loop the waveform.  `sleft <= 1` (shallow compare on the down-counter) is the last
    //   sample, equivalent to the old `sidx + 1 >= period_samples` without the add+compare.
    //   Stepped in EVERY mode (v34): psram_mode never reads sidx/sleft (the reader loops the
    //   stream) and every start reloads them.  A static frame (wfr) took a slot: no step.
    reg fe;
    always @(posedge clk) begin
        fe <= ~rst & ~stop & ~start & run_i & (st == S_SHIFT) & q[1] & last & ~wfr;
        if (rst) begin
            sidx <= 0; sleft <= 0;
        end else if (stop) begin
            // hold
        end else if (start) begin
            sidx <= 0; sleft <= period_samples;
        end else if (fe) begin
            if (slast) begin                     // == (sleft <= 1)
                sidx  <= 0;
                sleft <= period_samples;         // reload for the next loop
            end else begin
                sidx  <= sidx  + 1'b1;
                sleft <= sleft - 1'b1;
            end
        end
    end

    // Frame datapath (v42): lo, sh, bitc, last and q are loaded at S_PLO/S_RDHI and S_PHI/S_LAT
    // before every frame and only read in S_SHIFT, so they need no reset, stop or start term.  In
    // the FSM above they sat behind its rst/stop/start/running priority chain, which put all of
    // that into the clock enable of ~35 flops; the v42 loop image's clk48 then closed below
    // 48 MHz on most seeds, bound by that fan-out (rst48 -> sh/bitc CEN).  Here each register's
    // enable is its own state decode.  A stop or restart mid-frame still aborts it (the FSM
    // leaves S_SHIFT and raises SYNC); whatever these hold then is reloaded before the next one.
    //
    // wave_addr (v49) moved here too: its enable sat behind the same chain (start -> wave_addr
    // CEN bound the v49 loop image).  It is only a BRAM read address, so loading 2*sidx at every
    // expired S_IDLE (idle, static-frame and psram_mode slots included) reads harmlessly; a
    // BRAM slot then reads 2*sidx in S_RDLO and 2*sidx+1 in S_RDHI as before.
    always @(posedge clk) begin
        case (st)
        S_IDLE: if (dz) wave_addr <= {sidx[ADDR_W-1:0], 1'b0};                       // 2*sidx
        S_RDLO: wave_addr <= {sidx[ADDR_W-1:0], 1'b0} | {{(ADDR_W-1){1'b0}},1'b1};  // 2*sidx+1
        S_PLO:  if (strm_valid) lo <= strm_data;
        S_RDHI: lo <= wave_data;                                   // capture low byte
        S_PHI:  if (strm_valid) begin
                    sh <= {ctrl, strm_data, lo};                   // {ctrl, high, low}
                    bitc <= 5'd0; last <= 1'b0; q <= 3'b001;
                end
        S_LAT:  begin
                    sh <= wfr ? wr_frame : {ctrl, wave_data, lo};  // static, or {ctrl, high, low}
                    bitc <= 5'd0; last <= 1'b0; q <= 3'b001;
                end
        S_SHIFT: begin
                    q <= {q[1:0], q[2]};
                    if (q[0]) sh <= {sh[22:0], 1'b0};              // the bit just put out
                    if (q[2]) begin
                        bitc <= bitc + 5'd1;
                        last <= (bitc == 5'd22);                   // the next bit is the 24th
                    end
                end
        default: ;
        endcase
    end
endmodule
