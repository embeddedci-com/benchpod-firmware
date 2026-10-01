// ============================================================================
// logbuf.v — sample FIFO for the logging stream, read through the link's 0xC0-0xFF window.
//
// Push: on each sample set (`set`) the channels in `mask` (d0..d4: phase A, B, C, DUT battery,
// bus) go in, in that order, behind an optional sequence word (`seq_en`: a counter of sets, so
// BenchPod can see a gap). If the free space is short the whole set is dropped (and counted), so
// the stream never loses alignment.
//
// Pop: the link captures a read word a byte before it sends it, and a burst can end at any word.
// So reads move a speculative pointer (`take`), a word counts as read only when its last byte is
// out (`commit`), and when the transaction ends (`abort`) the speculative pointer falls back to
// the committed one: no word is lost and none is read twice. An empty FIFO reads 0x0000 and does
// not move.
// ============================================================================
`default_nettype none
module logbuf #(
    parameter AW = 11                      // 2048 words
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        clear,
    input  wire        en,
    input  wire [4:0]  mask,
    input  wire        seq_en,
    input  wire        set,
    input  wire [15:0] d0, d1, d2, d3, d4,
    input  wire        take,
    input  wire        commit,
    input  wire        abort,
    output wire [15:0] head,
    output wire [15:0] level,
    output reg  [15:0] drops
);
    reg [15:0] mem [0:(1 << AW) - 1];
    reg [AW:0] wptr, rptr_c, rptr_s;
    reg [15:0] q, seq;
    reg        q_ok;
    always @(posedge clk) begin
        q    <= mem[rptr_s[AW-1:0]];
        q_ok <= (rptr_s != wptr);
    end
    assign head  = q_ok ? q : 16'h0000;
    assign level = wptr - rptr_c;

    // push sequencer: latch the set, then one word per clk
    reg [15:0] l0, l1, l2, l3, l4;
    reg [5:0]  todo;                       // {seq, d4..d0} still to push
    reg        we;
    reg [15:0] wd;
    wire [2:0] n_words = mask[0] + mask[1] + mask[2] + mask[3] + mask[4] + seq_en;
    wire [AW:0] free = (1 << AW) - (wptr - rptr_c);
    always @(posedge clk) begin
        we <= 1'b0;
        if (rst || clear) begin
            wptr <= 0; rptr_c <= 0; rptr_s <= 0; todo <= 6'd0; drops <= 16'd0; seq <= 16'd0;
        end else begin
            if (set && en) begin
                seq <= seq + 16'd1;
                if (free >= n_words && todo == 6'd0) begin
                    l0 <= d0; l1 <= d1; l2 <= d2; l3 <= d3; l4 <= d4;
                    todo <= {seq_en, mask[4], mask[3], mask[2], mask[1], mask[0]};
                end else if (drops != 16'hFFFF) drops <= drops + 16'd1;
            end else if (todo != 6'd0) begin
                we <= 1'b1;
                if (todo[5])      begin wd <= seq - 16'd1; todo[5] <= 1'b0; end
                else if (todo[0]) begin wd <= l0; todo[0] <= 1'b0; end
                else if (todo[1]) begin wd <= l1; todo[1] <= 1'b0; end
                else if (todo[2]) begin wd <= l2; todo[2] <= 1'b0; end
                else if (todo[3]) begin wd <= l3; todo[3] <= 1'b0; end
                else              begin wd <= l4; todo[4] <= 1'b0; end
            end
            if (we) wptr <= wptr + 1'b1;
            // read side
            if (abort) rptr_s <= rptr_c;
            else if (take && rptr_s != wptr) rptr_s <= rptr_s + 1'b1;
            if (commit && rptr_c != rptr_s) rptr_c <= rptr_c + 1'b1;
        end
    end
    always @(posedge clk) if (we) mem[wptr[AW-1:0]] <= wd;
endmodule
`default_nettype wire
