// ============================================================================
// spram_ring48.v — the LA burst FIFO (v50): 48-bit words in SPRAM, bytes out.
//
// Replaces spram_ring16 for the LA stream.  A word is 6 bytes and both LA formats fill
// it whole: three raw samples (2 bytes each, la_byte_pack) or two change events (3 bytes
// each, la_change_capture).  Three SB_SPRAM256KA side by side give 16K words = 96 KB.
// Bytes leave little-endian (byte k = word[8k+7:8k]), so the PSRAM image is the same
// byte stream the producer meant.
//
// One SPRAM op per clk: write a word, or read one.  A read's data reaches `nxt` one edge
// after the read, and the port is free again on the very next edge, so a word written
// every clk (the change encoder's peak) only blocks reads, never itself.
// Writes have priority; a read starts only when no word is waiting, except when the ring
// is FULL, where the read must win (a full ring that refused both would deadlock; see
// spram_ring16).  `nxt` prefetches the next word while `out` is being sent, so the
// output side sustains one byte per clk.
//
// Input handshake: in_rdy = the word offered this cycle is taken at this edge.  A word
// strobed while !in_rdy is dropped and sets `overflow` (the raw path; the change encoder
// never strobes then).
//
// PRE-TRIGGER (v50): while `hold_out` is up nothing leaves the ring, and once it holds PRE_WORDS
// words each new word drops the oldest, so it keeps the newest pre-trigger window (half the ring:
// the other half takes the post-trigger burst while the drain catches up).
//
// A word strobed with in_len = 1..5 is a short TAIL (la_byte_pack's flush): the producer
// writes nothing after it, so it is the word on the output when nothing is left behind it,
// and only its first in_len bytes are sent.
// ============================================================================
module spram_ring48 #(
    parameter AW = 14,
    parameter PRE_AW = AW - 1          // PRE_WORDS = 1 << PRE_AW
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        clr_ovf,
    input  wire        hold_out,            // pre-trigger: send nothing, drop the oldest past PRE_WORDS
    input  wire [47:0] in_word,
    input  wire        in_stb,
    input  wire [2:0]  in_len,              // 0 = whole word, 1..5 = a tail of that many bytes
    output wire        in_rdy,
    output wire [7:0]  out_data,
    output wire        out_stb,
    input  wire        out_full,
    output wire        empty,
    output reg         overflow
);
    localparam DEPTH = (1 << AW);

    reg  [AW:0]  wr_ptr, rd_ptr;             // one wrap bit above the address
    wire         s_full  = (wr_ptr == {~rd_ptr[AW], rd_ptr[AW-1:0]});
    wire         s_empty = (wr_ptr == rd_ptr);

    reg  [47:0]  in_p;   reg in_p_v;         // the word waiting for the port
    reg          rd1;                        // a read happened at the last edge (sram_dout)
    reg  [47:0]  nxt;    reg nxt_v;          // prefetched word
    reg  [47:0]  out_w;  reg out_v;          // word being sent
    reg  [2:0]   out_i;                      // byte index 0..5
    reg  [2:0]   tail_len;                   // 0, or the bytes of the tail word written last

    wire [AW:0] fill = wr_ptr - rd_ptr;
    wire pre_drop = hold_out && (fill[AW:PRE_AW] != 0);   // >= PRE_WORDS resident
    wire do_write = in_p_v && !s_full;
    wire do_read  = !hold_out && !do_write && !s_empty && !nxt_v && !rd1;

    // The port, write XOR read per edge (true single port, as spram_ring16), driven straight
    // from the decision: the SPRAM registers its own inputs, so the fabric registers spram_ring16
    // put in front of it only cost 63 LC and a cycle.  A read's word is in sram_dout after the
    // edge and moves to nxt at the next one (rd1).
    reg [47:0]   sram [0:DEPTH-1];          // -> 3 x SB_SPRAM256KA
    reg [47:0]   sram_dout;
    wire [AW-1:0] sram_addr = do_write ? wr_ptr[AW-1:0] : rd_ptr[AW-1:0];
    always @(posedge clk) begin
        if (do_write) sram[sram_addr] <= in_p;
        else          sram_dout       <= sram[sram_addr];
    end
    assign in_rdy = !in_p_v || do_write;

    assign out_stb  = out_v && !out_full;
    assign out_data = out_w[7:0];                 // out_w shifts down a byte per byte sent
    // the word being sent is the tail: one was written and nothing is behind it
    wire   is_tail  = (tail_len != 3'd0) && s_empty && !in_p_v && !rd1 && !nxt_v;
    wire   out_last = out_stb && (out_i == (is_tail ? tail_len - 3'd1 : 3'd5));

    assign empty = s_empty && !in_p_v && !rd1 && !nxt_v && !out_v;

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr <= 0; rd_ptr <= 0;
            in_p_v <= 1'b0; rd1 <= 1'b0; nxt_v <= 1'b0;
            out_v <= 1'b0; out_i <= 3'd0; overflow <= 1'b0; tail_len <= 3'd0;
        end else begin
            if (clr_ovf) overflow <= 1'b0;

            // ---- the port's pointers ----
            if (do_write) wr_ptr <= wr_ptr + 1'b1;
            if (do_read || (do_write && pre_drop)) rd_ptr <= rd_ptr + 1'b1;   // read, or drop the oldest
            rd1 <= do_read;

            // ---- input ----
            if (do_write) in_p_v <= 1'b0;
            if (in_stb) begin
                if (in_rdy) begin in_p <= in_word; in_p_v <= 1'b1; tail_len <= in_len; end
                else overflow <= 1'b1;
            end

            // ---- output: nxt -> out, bytes out ----
            if (out_stb) begin
                out_i <= out_last ? 3'd0 : out_i + 3'd1;
                out_w <= {8'd0, out_w[47:8]};
            end
            if ((!out_v || out_last) && nxt_v) begin
                out_w <= nxt; out_v <= 1'b1; nxt_v <= 1'b0;
            end else if (out_last) begin
                out_v <= 1'b0;
            end
            if (rd1) begin nxt <= sram_dout; nxt_v <= 1'b1; end
        end
    end
endmodule
