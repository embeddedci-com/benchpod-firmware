// ============================================================================
// la_byte_pack.v — packs the raw LA byte stream (la_psram_capture, 2 bytes/sample)
// into the 48-bit words spram_ring48 stores (v50): byte k of a word = the k-th byte.
//
// The sixth byte completes a word, offered on the same cycle.  `flush` offers a
// partial word with `wlen` = its valid bytes (1..5; 0 = a whole word), and spram_ring48
// sends only those: a padded tail would land in PSRAM past the capture's last byte, where
// the ADC region may start.
// ============================================================================
module la_byte_pack (
    input  wire        clk,
    input  wire        rst,
    input  wire [7:0]  in_data,
    input  wire        in_stb,
    input  wire        flush,      // offer a partial word (ignored when empty)
    output wire [47:0] wd,
    output wire        wstb,
    output wire [2:0]  wlen,       // valid bytes of a flushed partial word; 0 = whole word
    output wire        partial     // bytes are held
);
    reg [39:0] sh;                 // bytes 0..4 of the word being filled
    reg [2:0]  n;                  // bytes held, 0..5

    wire complete = in_stb && (n == 3'd5);
    assign partial = (n != 3'd0);
    // Byte 5 is in_data: the sixth byte on a complete word, and never sent from a partial one
    // (wlen <= 5), so it needs no flop.
    assign wd      = {in_data, sh};
    assign wstb    = complete || (flush && partial && !in_stb);
    assign wlen    = complete ? 3'd0 : n;

    // Byte lane k takes in_data when it is the k-th byte: one enable per lane, so each flop's
    // D is in_data itself (a dynamic part-select mapped to a 40-bit mux, 76 LUTs in v50).
    genvar k;
    generate for (k = 0; k < 5; k = k + 1) begin : lane
        always @(posedge clk) if (!rst && in_stb && n == k) sh[8*k +: 8] <= in_data;
    end endgenerate
    always @(posedge clk) begin
        if (rst) n <= 3'd0;
        else if (in_stb) n <= complete ? 3'd0 : n + 3'd1;
        else if (flush && partial) n <= 3'd0;
    end
endmodule
