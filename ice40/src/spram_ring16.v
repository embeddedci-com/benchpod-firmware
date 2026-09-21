// ============================================================================
// spram_ring16.v — deep byte FIFO in iCE40 SINGLE-PORT SPRAM, 16-bit-packed.
//
// Same byte-in / byte-out interface as bram_ring, but backed by SB_SPRAM256KA —
// the pod has 8x more SPRAM than BRAM (128 KB vs 15 KB), so the deep-LA burst
// window is ~16x deeper.  The trick that lets SINGLE-PORT SPRAM keep up with the
// 6 MS/s LA stream: store a whole 16-bit SAMPLE per word — pack the low+high byte
// on the way in, unpack on the way out — so the shared port does 1 write + 1 read
// per SAMPLE instead of per byte.  At 6 MS/s in + ~3 MS/s drain that is roughly
// 12M of the port's 24M ops/s (half the port free); a byte-granular SPRAM ring
// (spram_ring, ~6 port-cycles/sample) saturates and drops the burst.
//
// Structure mirrors spram_ring: a dedicated write-XOR-read SPRAM block (the FSM
// guarantees reads never coincide with a write, so it maps to true single-port
// SPRAM) and registered cnt_nz/cnt_nfull flags kept off the arbitration path.
//
// Depth = (1<<AW) words = 2*(1<<AW) bytes.  AW=15 -> 32K words -> 64 KB -> 32K
// samples, 2 SB_SPRAM256KA blocks (the pod has 2 free after the correlated ring).
// ============================================================================
module spram_ring16 #(
    parameter AW = 15
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] in_data,
    input  wire       in_stb,
    output wire       in_full,
    output wire [7:0] out_data,
    output wire       out_stb,
    input  wire       out_full,
    output wire       empty,
    output reg        overflow
);
    localparam DEPTH = (1 << AW);

    reg [15:0]   sram [0:DEPTH-1];           // -> SB_SPRAM256KA (16-bit word)
    reg [AW-1:0] sram_addr;
    reg          sram_we;
    reg [15:0]   sram_din;
    reg [15:0]   sram_dout;
    // True single port: write XOR read per cycle (the FSM never reads during a
    // write), so this maps to SB_SPRAM256KA.
    always @(posedge clk) begin
        if (sram_we) sram[sram_addr] <= sram_din;
        else         sram_dout      <= sram[sram_addr];
    end

    reg [AW-1:0] wr_ptr, rd_ptr;
    reg [AW:0]   count;                       // WORDS resident in SPRAM

    // input: pack low then high byte into one 16-bit word (word[7:0]=low), held in
    // the 1-deep in_p register until the port writes it.
    reg [7:0]  lo;   reg lo_v;
    reg [15:0] in_p_d; reg in_p_v;
    // Backpressure only when the NEXT byte would COMPLETE a word (lo already held)
    // AND the previous completed word is still waiting for the port (in_p_v).  A low
    // byte never needs in_p, so it is always accepted — that gives the port an extra
    // cycle to write the pending word before the pair completes.
    assign in_full = lo_v && in_p_v;

    // output: a fetched word being unpacked into two bytes (low first, matching the
    // little-endian layout la_psram_capture / la_capture produce).  out_stb/out_data
    // are COMBINATIONAL off out_full so a byte is only committed the cycle the
    // consumer can take it — a registered strobe would commit against a stale
    // out_full and the writer would drop the byte.
    reg [15:0] out_word; reg out_word_v; reg out_hi;
    assign out_stb  = out_word_v && !out_full;
    assign out_data = out_hi ? out_word[15:8] : out_word[7:0];

    // registered empty/full flags, maintained incrementally (count only ±1/cycle),
    // to keep the wide count compares off the per-cycle arbitration path.
    reg cnt_nz;                               // count != 0
    reg cnt_nfull;                            // count != DEPTH

    reg [1:0] st;
    localparam S_IDLE = 2'd0, S_RD1 = 2'd1, S_RD2 = 2'd2;

    // Write priority: a completed word writes in S_IDLE.  A read is only STARTED when
    // no input is pending (in_p_v/lo_v both clear) — otherwise the 2-cycle read
    // blackout (S_RD1/S_RD2) can strand a just-packed word in the 1-deep in_p while
    // the sampler emits its next byte (overflow).  Deferring the read to the sampler's
    // idle gap keeps the single port free for the steady 1-word-per-4-cycles input;
    // the writer's own FIFO covers the brief read pause.
    wire do_write = in_p_v && cnt_nfull;
    // Read a word when the port is free (no write this cycle), the output slot is empty,
    // no half-formed low byte, and SRAM has data.  Normally DEFER the read while an input
    // word is pending (in_p_v) so the 2-cycle read blackout can't strand it — EXCEPT when
    // SRAM is FULL (!cnt_nfull): then do_write can't fire either, so refusing the read too
    // deadlocks (in_p can't write into a full SRAM, SRAM can't be read to make room).  At
    // full, the read MUST win.  (Deadlock reproduced in scratchpad/tb_deepdrain.v.)
    wire do_read  = !do_write && !lo_v && !out_word_v && cnt_nz && (!in_p_v || !cnt_nfull);

    // empty = nothing resident, nothing packed/in-flight/prefetched (ignores a lone
    // half-formed lo byte — la always emits complete 2-byte samples).
    assign empty = !cnt_nz && !in_p_v && !out_word_v && !lo_v && (st == S_IDLE);

    always @(posedge clk) begin
        sram_we <= 1'b0;
        if (rst) begin
            wr_ptr <= 0; rd_ptr <= 0; count <= 0;
            cnt_nz <= 1'b0; cnt_nfull <= 1'b1;
            lo_v <= 1'b0; in_p_v <= 1'b0;
            out_word_v <= 1'b0; out_hi <= 1'b0;
            st <= S_IDLE; overflow <= 1'b0;
        end else begin
            // ---- input: byte -> 16-bit word (low byte first) ----
            if (in_stb) begin
                if (!lo_v) begin lo <= in_data; lo_v <= 1'b1; end       // low byte: always OK
                else if (!in_p_v) begin in_p_d <= {in_data, lo}; in_p_v <= 1'b1; lo_v <= 1'b0; end
                else begin overflow <= 1'b1; lo_v <= 1'b0; end  // high byte but in_p full -> DROP the
                    // WHOLE sample and CLEAR lo_v.  Leaving lo_v stuck at 1 permanently blocked do_read
                    // (which requires !lo_v) once the sampler stopped feeding, so the ring could never
                    // drain its remaining SRAM words -> the pipe never emptied -> CAP_DONE never set
                    // (deep-capture hang, reproduced in scratchpad/tb_deepdrain.v).  Dropping a whole
                    // 2-byte sample preserves alignment.
            end

            // ---- output: advance the unpack when a byte is taken (out_stb) ----
            if (out_word_v && !out_full) begin
                if (!out_hi) out_hi <= 1'b1;
                else begin out_hi <= 1'b0; out_word_v <= 1'b0; end
            end

            // ---- single SPRAM port: one op per cycle ----
            case (st)
            S_IDLE: begin
                if (do_write) begin
                    sram_addr <= wr_ptr; sram_din <= in_p_d; sram_we <= 1'b1;
                    wr_ptr <= wr_ptr + 1'b1; count <= count + 1'b1; in_p_v <= 1'b0;
                    cnt_nz    <= 1'b1;
                    cnt_nfull <= (count != (DEPTH[AW:0] - 1'b1));
                end else if (do_read) begin
                    sram_addr <= rd_ptr;                   // present read address
                    st <= S_RD1;
                end
            end
            S_RD1: st <= S_RD2;                            // wait: registered dout settles
            S_RD2: begin                                   // sram_dout = sram[rd_ptr]
                out_word   <= sram_dout;
                out_word_v <= 1'b1;
                out_hi     <= 1'b0;
                rd_ptr     <= rd_ptr + 1'b1;
                count      <= count - 1'b1;
                cnt_nfull  <= 1'b1;
                cnt_nz     <= (count != {{AW{1'b0}}, 1'b1});
                st         <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
