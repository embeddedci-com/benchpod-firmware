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
    reg [47:0] sh;
    reg [2:0]  n;                  // bytes held, 0..5

    wire complete = in_stb && (n == 3'd5);
    assign partial = (n != 3'd0);
    assign wd      = complete ? {in_data, sh[39:0]} : sh;
    assign wstb    = complete || (flush && partial && !in_stb);
    assign wlen    = complete ? 3'd0 : n;

    always @(posedge clk) begin
        if (rst) n <= 3'd0;
        else if (in_stb) begin
            sh[8*n +: 8] <= in_data;
            n <= complete ? 3'd0 : n + 3'd1;
        end else if (flush && partial) n <= 3'd0;
    end
endmodule
