// ============================================================================
// la_psram_capture.v — deep multi-channel logic-analyzer sampler → PSRAM (v2).
//
// Companion to the retired la_capture.v, which packed the same LA word into
// the small on-FPGA la_capture_buf SPRAM (≤4096 bytes ⇒ ≤2048 samples, read back
// over SPI with LA_READ).  THIS module instead streams the samples straight into
// the APS6404L PSRAM via psram_writer — reusing the v2 ADC→PSRAM datapath — so the
// capture depth is bounded by PSRAM, not by a 4 KB BRAM.  The STM32 reads the
// region back over its own XSPI (psram_read), exactly as it does for an ADC PSRAM
// capture.  Driven by the LA_CAPTURE (0x69) opcode.
//
// Each sample is recorded as TWO little-endian bytes (N=14 on v2):
//   byte 2k    = la_in[7:0]          (LA1..LA8)
//   byte 2k+1  = {2'b0, la_in[13:8]} (LA9..LA14 in the low 6 bits)
//
// `sample_count` is the number of SAMPLES (2 bytes each); its width is the CNT_W
// parameter (top_v2 uses 24-bit, so a single capture can span the full 8 MB PSRAM =
// up to ~4.16M samples).  Sample period = `divider` clocks; the
// divider / 2-byte-write cadence mirrors la_capture.v byte-for-byte so the two LA
// paths produce identically-timed traces.
//
// Runs on the 48 MHz capture clock (clk48) alongside the ADC-only orchestrator in
// top_v2; its writer feed (ps_start/ps_stop/wr_data/wr_stb) is muxed into the same
// psram_writer, and the three capture modes (ADC-only / correlated / LA) are
// mutually exclusive (the firmware never arms one while another runs).
//
// THROUGHPUT CAVEAT: psram_writer DROPS bytes when its 64-byte FIFO is full
// (wr_en = wr_stb && !full), and unlike the correlated path there is no spram_ring
// in front of it here.  So the sustained sample rate must stay within the writer's
// drain rate (~12 MB/s ⇒ a few MS/s at 2 B/sample); the firmware clamps the
// divider accordingly.  `overflow` latches if a byte was ever pushed while full —
// a diagnostic that the requested rate exceeded the writer.  NEEDS HARDWARE
// VALIDATION before relying on long/fast captures (see psram_writer.v's note).
// ============================================================================

module la_psram_capture #(
    parameter N     = 12,          // LA channels sampled (low N bits of la_in; 9..16)
    parameter CNT_W = 16           // sample-count width
)(
    input  wire              clk,            // 48 MHz capture clock
    input  wire              rst,

    // raw LA channel readback (synchronized internally)
    input  wire [N-1:0]      la_in,

    // control (start is already in this clock domain — top crosses it via cdc)
    input  wire              start,          // 1-cycle pulse
    // v35 capture trigger: 1 = the run `start` loaded waits here (nothing counts, nothing is
    // written) until hold drops.  The run then proceeds exactly as it would have from `start`,
    // so the divider/period logic below is untouched.  Tie 0 for an untriggered producer.
    input  wire              hold,
    input  wire [CNT_W-1:0]  sample_count,   // number of samples (2 bytes each)
    input  wire [15:0]       divider,        // v40: sample period in clocks MINUS 2 (the reload;
                                             // the firmware encodes it, 0 => the 2-clock floor)

    // psram_writer feed
    output reg               ps_start,       // 1-cycle: open a PSRAM write at addr 0
    output reg               ps_stop,        // 1-cycle: close it (flush + idle)
    output reg  [7:0]        wr_data,
    output reg               wr_stb,
    input  wire              full,           // writer FIFO full (backpressure/diag)

    // status
    output reg               busy,
    output reg               done,           // sticky until next start
    output reg               overflow        // sticky: a byte was pushed while full
);
    // 2-flop sync of the raw LA lines (metastability guard), as in la_capture.v.
    reg [N-1:0] q1, q2;
    always @(posedge clk) begin
        q1 <= la_in;
        q2 <= q1;
    end

    reg [15:0]      divcnt;        // sample-period DOWN-counter (ticks at 0)
    // samples still to record: an SB_MAC16 down-counter since v41 (dsp_counter), loaded at start
    // and stepped with each sample's high byte, exactly where the fabric counter was.
    wire [31:0]      samps_q;
    wire [CNT_W-1:0] samps_left = samps_q[CNT_W-1:0];
    reg             writing_hi;    // 0 = write low byte on tick; 1 = write high byte
    reg [N-1:0]     latched;       // sample held across the 2-byte write
    dsp_counter #(.UP(0)) samps_i (
        .clk(clk), .load(~rst & start), .load_val({{(32-CNT_W){1'b0}}, sample_count}),
        .en(~rst & ~start & busy & ~hold & writing_hi),
        .q(samps_q), .flag());

    // Sample period = EXACTLY `divider` clocks (floor 2).  One period is: the tick cycle
    // (low byte), the hi-byte cycle — which does NOT decrement divcnt — then div_reload
    // down-count cycles, so div_reload = divider - 2.  (Gateware <= v31 reloaded
    // divider - 1, so every sample took divider + 1 clocks and the real LA rate was
    // d/(d+1) of the 24 MHz / d the firmware reports: -8% at 2 MS/s.)  Same LC cost as
    // the old reload — only the constants changed.
    // v40: the firmware sends the reload itself (period - 2, floored at 0), so no compare and
    // subtract here.
    wire [15:0] div_reload = divider;

    // high byte: LA channels above bit 7, zero-extended to a byte (9 <= N <= 16).
    wire [7:0] hi_byte = { {(16-N){1'b0}}, latched[N-1:8] };

    always @(posedge clk) begin
        ps_start <= 1'b0;
        ps_stop  <= 1'b0;
        wr_stb   <= 1'b0;
        if (rst) begin
            divcnt     <= 16'd0;
            writing_hi <= 1'b0;
            busy       <= 1'b0;
            done       <= 1'b0;
            overflow   <= 1'b0;
        end else if (start) begin
            divcnt     <= div_reload;
            writing_hi <= 1'b0;
            busy       <= (sample_count != {CNT_W{1'b0}});
            done       <= 1'b0;
            overflow   <= 1'b0;
            ps_start   <= (sample_count != {CNT_W{1'b0}});
        end else if (busy && !hold) begin
            if (writing_hi) begin
                // second byte of the sample — emitted the clock after the low byte.
                wr_data    <= hi_byte;
                wr_stb     <= 1'b1;
                if (full) overflow <= 1'b1;
                writing_hi <= 1'b0;
                if (samps_left == {{(CNT_W-1){1'b0}}, 1'b1}) begin
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    ps_stop <= 1'b1;   // last byte + flush in the same cycle (as ADC)
                end
            end else if (divcnt == 16'd0) begin
                // sample tick: latch the word and write the low byte.
                divcnt     <= div_reload;
                latched    <= q2;
                wr_data    <= q2[7:0];
                wr_stb     <= 1'b1;
                if (full) overflow <= 1'b1;
                writing_hi <= 1'b1;
            end else begin
                divcnt <= divcnt - 16'd1;
            end
        end
    end

endmodule
