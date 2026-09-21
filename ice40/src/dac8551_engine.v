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
// runs it on clk48 (48 MHz) for a ~2x update rate (SCLK = clk/2 = 24 MHz, still under
// the DAC8551's 30 MHz max); the control-plane inputs cross a CDC in top_v2 and the
// waveform BRAM read port shares this clock.  (v1 top.v uses the separate dac_engine
// on the 24 MHz clk.)
//
// SERIAL TIMING vs the DAC8551 datasheet (SLAS429E 6.6, the VDD 3.6-5.5 V column — the 24 MHz
// SCLK already requires it: SCLK max is 20 MHz below 3.6 V), at clk = 48 MHz (20.8 ns):
//   t1 SCLK period 2 clk = 41.7 ns (>= 33)       t2/t3 SCLK high/low 1 clk = 20.8 ns (>= 13)
//   t4 SYNC fall -> SCLK rise 1 clk (>= 0)        t5/t6 DIN changes on SCLK RISES only, so setup
//                                                 and hold at each fall are 1 clk (>= 5 / 4.5)
//   t7 24th SCLK fall -> SYNC rise 1 clk (>= 0)   (v34; it used to be the SAME edge)
//   t8 SYNC high >= DIV_MIN+2 clk = 104 ns (>= 33)
//   t9 24th SCLK fall -> next SYNC fall >= DIV_MIN+3 clk = 125 ns (>= 100)   (v34 divider floor)
// One sample takes max(divider, DIV_MIN) + 51 clk — the firmware's DAC_SEQ_OVERHEAD_CLK model.
// tb_dac8551 checks all of these on the pins, cycle-exactly.
//
// DEEP replay (psram_mode=1, gateware >= v17): instead of the 4 KB LOAD_WAVE BRAM,
// the engine pulls each 16-bit sample (little-endian, low byte first) out of the
// dac_psram_reader prefetch FIFO, which streams the waveform straight from PSRAM and
// loops it at len_bytes.  The engine then has no depth limit of its own — it just
// pops bytes forever until `stop` — so replay spans the full 8 MB PSRAM.  The
// sidx/sleft/period loop machinery (and wave_addr) are used ONLY by the BRAM path;
// in psram_mode looping is the reader's job.  psram_mode=0 is byte-for-byte the
// original BRAM engine (guarded so tb_dac8551 / tb_dac8551_loop are unaffected).
// ============================================================================
module dac8551_engine #(
    parameter ADDR_W = 12
)(
    input  wire              clk,
    input  wire              rst,
    input  wire              start,
    input  wire              stop,
    input  wire [ADDR_W:0]   period_samples,  // 16-bit samples before looping
    input  wire [15:0]       divider,

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

    // DAC8551 serial bus
    output reg               dac_sync,
    output reg               dac_sclk,
    output reg               dac_din,
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
    // Divider floor (v34): the inter-sample gap is max(divider, DIV_MIN) clk, so the next SYNC
    // fall lands >= DIV_MIN + 3 clk after the 24th SCLK fall.  DAC8551 t9 needs >= 100 ns;
    // divider 2 gave exactly 5 x 20.8 = 104 ns and 0/1 gave less.  DIV_MIN = 3 -> 125 ns.
    localparam [15:0] DIV_MIN        = 16'd3;
    wire             div_small       = (divider < DIV_MIN);
    wire [15:0]      div_reload      = div_small ? (DIV_MIN - 16'd1) : (divider - 16'd1);
    wire             div_reload_zero = 1'b0;                  // reload >= DIV_MIN-1 >= 2
    reg [ADDR_W:0]   sidx;        // 16-bit-sample index (also the BRAM address)
    reg [ADDR_W:0]   sleft;       // samples left in this loop (= period_samples - sidx);
                                  // down-counter terminal, replaces the per-sample
                                  // `sidx + 1 >= period_samples` add+compare.
    reg [7:0]        lo;
    reg [23:0]       sh;
    reg [4:0]        bitc;
    // `last` = registered (bitc == 23), set on the falling half of bit 22 (v34).  The frame-end
    // branch updates st/sidx/sleft, so its clock-enable cone decoded the 5-bit compare plus st/ph/
    // running/psram_mode in ONE clk48 cycle — the v34 sweeps found that cone binding clk48 (~21 ns)
    // in both images.  Same trick as `dz` below: test one flop instead of the wide compare.
    reg              last;
    reg              ph;

    always @(posedge clk) begin
        strm_pop <= 1'b0;                      // default: single-cycle pop pulse
        if (rst) begin
            st<=S_IDLE; running<=1'b0; dac_sync<=1'b1; dac_sclk<=1'b0; dac_din<=1'b0;
            sidx<=0; sleft<=0; wave_addr<=0; div_cnt<=0; dz<=1'b1;
        end else if (stop) begin
            st<=S_IDLE; running<=1'b0; dac_sync<=1'b1; dac_sclk<=1'b0;
        end else if (start) begin
            running<=1'b1; sidx<=0; sleft<=period_samples; wave_addr<=0; dac_sync<=1'b1; dac_sclk<=1'b0;
            div_cnt<=div_reload; dz<=div_reload_zero; st<=S_IDLE;
        end else if (running) begin
            case (st)
            S_IDLE: begin
                dac_sync<=1'b1; dac_sclk<=1'b0;
                if (dz) begin
                    div_cnt<=div_reload; dz<=div_reload_zero;
                    if (psram_mode) begin
                        st<=S_PLO;                 // fetch this sample from the stream FIFO
                    end else begin
                        wave_addr <= {sidx[ADDR_W-1:0], 1'b0};   // 2*sidx (low byte)
                        st<=S_RDLO;
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
                dac_sync<=1'b1; dac_sclk<=1'b0;
                if (strm_valid) begin lo<=strm_data; strm_pop<=1'b1; st<=S_PLW; end
            end
            S_PLW: st<=S_PHI;                      // pop effective; head advances to the high byte
            S_PHI: begin
                if (strm_valid) begin
                    sh <= {8'h00, strm_data, lo};  // {ctrl, high, low}
                    strm_pop<=1'b1;
                    bitc<=5'd0; last<=1'b0; ph<=1'b0; dac_sync<=1'b0; st<=S_SHIFT;
                end
            end
            S_RDLO: begin                          // BRAM latency: addr now valid
                wave_addr <= {sidx[ADDR_W-1:0], 1'b0} | {{(ADDR_W-1){1'b0}},1'b1}; // 2*sidx+1
                st<=S_RDHI;
            end
            S_RDHI: begin lo <= wave_data; st<=S_LAT; end   // capture low byte
            S_LAT: begin
                sh <= {8'h00, wave_data, lo};      // {ctrl, high, low}
                bitc<=5'd0; last<=1'b0; ph<=1'b0; dac_sync<=1'b0; st<=S_SHIFT;
            end
            S_SHIFT: begin
                ph<=~ph;
                if (!ph) begin dac_din<=sh[23]; dac_sclk<=1'b1; end
                else begin
                    dac_sclk<=1'b0; sh<={sh[22:0],1'b0}; bitc<=bitc+5'd1;
                    last<=(bitc==5'd22);                  // next bit is the 24th
                    if (last) begin                       // == (bitc == 23)
                        // SYNC is NOT raised here: this edge IS the 24th SCLK fall, which the
                        // DAC8551 must see with SYNC still low (t7 >= 0 ns).  Raising both on
                        // one edge left their order to routing skew; S_IDLE raises SYNC on the
                        // next clk instead (t7 = 1 clk), which costs no sample time.
                        // Loop the waveform.  `sleft <= 1` (shallow compare on the
                        // down-counter) is the last sample, equivalent to the old
                        // `sidx + 1 >= period_samples` but without the add+compare.
                        // Stepped in EVERY mode (v34): psram_mode never reads
                        // sidx/sleft (the reader loops the stream) and every start
                        // reloads them, so gating on psram_mode only put that OR of
                        // two synced flops into this branch's clock-enable cone.
                        if (sleft <= {{ADDR_W{1'b0}}, 1'b1}) begin
                            sidx  <= 0;
                            sleft <= period_samples;         // reload for the next loop
                        end else begin
                            sidx  <= sidx  + 1'b1;
                            sleft <= sleft - 1'b1;
                        end
                        st<=S_IDLE;
                    end
                end
            end
            default: st<=S_IDLE;
            endcase
        end
    end
endmodule
