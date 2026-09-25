// ============================================================================
// uart_engine.v — soft UART (8N1) with TX/RX FIFOs, for the UART proxy feature.
//
// The bench pod can mock/bridge a DUT UART through two LA channels: this engine
// serialises TX bytes onto `tx_out` (idle high, push-pull via la_bank) and
// deserialises `rx_in` into bytes.  The RP2350 (SPI orchestrator) pushes TX
// bytes and drains RX bytes over SPI (cmd_dispatch opcodes 0x70-0x74) and
// bridges them to the console / a TCP client.
//
// Frame: 8 data bits, no parity, 1 stop bit (8N1).  The bit period in system
// clocks is supplied as `cfg_div` (the RP computes round(24 MHz / baud) so the
// FPGA needs no divider).  RX uses mid-bit sampling: on the start edge it waits
// half a bit to the middle of the start bit (re-checking it is still low to
// reject glitches), then one full bit period to the middle of each data bit.
// Accurate for our clk/baud ratios (208 clks/bit @115200); very high baud
// (>~230400) gets coarse and is best-effort.
//
// Two independent 256-byte ring FIFOs (one writer + one reader each → one
// SB_RAM40_4K block each) decouple the baud-paced wire from the bursty SPI
// link.  RX overflow drops the byte and latches a sticky flag (cleared on a
// status read).
// ============================================================================

// ---- small 1-writer / 1-reader ring FIFO (depth 256) ----------------------
module uart_fifo (
    input  wire        clk,
    input  wire        rst,
    input  wire        we,
    input  wire [7:0]  wdata,
    input  wire        re,
    output wire [7:0]  rdata,   // current head (1-cycle settle after a pop)
    output wire [8:0]  count,   // occupancy 0..256
    output wire        full,
    output wire        empty
);
    reg [7:0] mem [0:255];
    reg [7:0] wptr, rptr;
    reg [8:0] cnt;
    reg [7:0] rdata_r;

    assign rdata = rdata_r;
    assign count = cnt;
    assign full  = (cnt == 9'd256);
    assign empty = (cnt == 9'd0);

    wire we_e = we & ~full;
    wire re_e = re & ~empty;

    always @(posedge clk) begin
        if (rst) begin
            wptr <= 8'd0;
            rptr <= 8'd0;
            cnt  <= 9'd0;
        end else begin
            if (we_e) begin mem[wptr] <= wdata; wptr <= wptr + 8'd1; end
            if (re_e) rptr <= rptr + 8'd1;
            cnt <= cnt + (we_e ? 9'd1 : 9'd0) - (re_e ? 9'd1 : 9'd0);
        end
        // Present the next head: after a pop, show mem[rptr+1] so the head is
        // ready by the next consumer access (SPI/baud are far slower).
        rdata_r <= mem[re_e ? (rptr + 8'd1) : rptr];
    end
endmodule


module uart_engine (
    input  wire        clk,
    input  wire        rst,

    // ---- configuration (latched on cfg_stb; cleared on disable_stb) ----
    input  wire        cfg_stb,
    input  wire [3:0]  cfg_rx_ch,
    input  wire [3:0]  cfg_tx_ch,
    input  wire [23:0] cfg_div,      // bit period in system clocks
    input  wire        cfg_enable,
    input  wire        disable_stb,

    // latched channel assignment (top routes the bus + la_bank from these)
    output reg  [3:0]  rx_ch,
    output reg  [3:0]  tx_ch,
    output reg         armed,

    // ---- TX FIFO write port (cmd_dispatch UART_WRITE) ----
    input  wire        tx_we,
    input  wire [7:0]  tx_wdata,
    output wire        tx_full,
    output wire        tx_empty,

    // ---- RX FIFO read port (cmd_dispatch UART_READ / UART_STATUS) ----
    input  wire        rx_re,
    output wire [7:0]  rx_rdata,
    output wire [8:0]  rx_avail,
    output reg         rx_overflow,   // sticky; cleared by status read strobe
    input  wire        rx_ovf_clr,

    // ---- UART bus (to/from la_bank) ----
    input  wire        rx_in,
    output reg         tx_out         // idle high; la_bank drives push-pull when armed
);

    // ---- latched config ----
    // The bit period `div` is stored in 18 bits, not the 24-bit protocol width: 2^18-1 =
    // 262143 clks/bit @24 MHz is ~92 baud — below any real UART — so the top 6 bits of
    // cfg_div only ever carry an out-of-range value, which we clamp at the config latch.
    // This narrows the three bit-period counters (div/tx_cnt/rx_cnt) 24->18 bits.  The
    // SPI/protocol cfg_div field stays 24-bit (firmware unchanged).
    localparam DIV_W = 18;
    reg [DIV_W-1:0] div;
    wire [DIV_W-1:0] half_div = {1'b0, div[DIV_W-1:1]};
    // The bit counters reload with `div` (or half_div) and expire at 1, not reload `div - 1`
    // and expire at 0: the same number of clocks per bit (div..1 == div-1..0), without the
    // two 18-bit subtractors.  Safe because div >= 2 (clamped below), so half_div >= 1.

    // ---- RX 2-flop synchroniser ----
    reg rx_s0, rx_s1;
    always @(posedge clk) begin
        if (rst) begin rx_s0 <= 1'b1; rx_s1 <= 1'b1; end
        else     begin rx_s0 <= rx_in; rx_s1 <= rx_s0; end
    end

    // ---- config / arm ----
    always @(posedge clk) begin
        if (rst) begin
            armed <= 1'b0; rx_ch <= 4'd0; tx_ch <= 4'd0; div <= 18'd208;
        end else begin
            if (cfg_stb) begin
                rx_ch <= cfg_rx_ch;
                tx_ch <= cfg_tx_ch;
                // Clamp cfg_div into DIV_W: floor at 2, saturate anything that overflows
                // 18 bits (>=2^18) to the max period (slowest supported baud).
                div   <= (cfg_div < 24'd2)      ? {{(DIV_W-2){1'b0}}, 2'd2} :
                         (|cfg_div[23:DIV_W])   ? {DIV_W{1'b1}} :
                                                  cfg_div[DIV_W-1:0];
                armed <= cfg_enable;
            end
            if (disable_stb) armed <= 1'b0;
        end
    end

    // ============================ TX path ===================================
    wire [7:0] tx_head;
    wire       tx_fifo_empty;
    reg        tx_pop;

    uart_fifo tx_fifo (
        .clk(clk), .rst(rst),
        .we(tx_we), .wdata(tx_wdata),
        .re(tx_pop), .rdata(tx_head),
        .count(/*unused*/), .full(tx_full), .empty(tx_fifo_empty)
    );
    assign tx_empty = tx_fifo_empty;

    localparam TX_IDLE = 3'd0, TX_LOAD = 3'd1, TX_START = 3'd2, TX_DATA = 3'd3, TX_STOP = 3'd4;
    (* fsm_encoding = "none" *) reg [2:0]  tx_state;
    reg [DIV_W-1:0] tx_cnt;
    reg [2:0]  tx_bit;
    reg [7:0]  tx_sr;

    always @(posedge clk) begin
        tx_pop <= 1'b0;
        if (rst || !armed) begin
            tx_state <= TX_IDLE;
            tx_out   <= 1'b1;       // idle high
            tx_cnt   <= 18'd0;
            tx_bit   <= 3'd0;
            tx_sr    <= 8'h00;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    tx_out <= 1'b1;
                    // The transition itself gives the FIFO one cycle to present
                    // a valid head (registered read settles after push-into-empty).
                    if (!tx_fifo_empty) tx_state <= TX_LOAD;
                end
                TX_LOAD: begin
                    tx_sr    <= tx_head;    // now-valid head
                    tx_pop   <= 1'b1;       // advance FIFO
                    tx_cnt   <= div;
                    tx_out   <= 1'b0;        // start bit
                    tx_state <= TX_START;
                end
                TX_START: begin
                    if (tx_cnt == 18'd1) begin
                        tx_cnt   <= div;
                        tx_out   <= tx_sr[0];   // LSB first
                        tx_bit   <= 3'd0;
                        tx_state <= TX_DATA;
                    end else tx_cnt <= tx_cnt - 18'd1;
                end
                TX_DATA: begin
                    if (tx_cnt == 18'd1) begin
                        tx_cnt <= div;
                        if (tx_bit == 3'd7) begin
                            tx_out   <= 1'b1;   // stop bit
                            tx_state <= TX_STOP;
                        end else begin
                            tx_sr  <= {1'b0, tx_sr[7:1]};
                            tx_out <= tx_sr[1];
                            tx_bit <= tx_bit + 3'd1;
                        end
                    end else tx_cnt <= tx_cnt - 18'd1;
                end
                TX_STOP: begin
                    if (tx_cnt == 18'd1) tx_state <= TX_IDLE;
                    else                 tx_cnt   <= tx_cnt - 18'd1;
                end
            endcase
        end
    end

    // ============================ RX path ===================================
    reg        rx_push;
    reg  [7:0] rx_pushdata;
    wire       rx_fifo_full;

    uart_fifo rx_fifo (
        .clk(clk), .rst(rst),
        .we(rx_push), .wdata(rx_pushdata),
        .re(rx_re), .rdata(rx_rdata),
        .count(rx_avail), .full(rx_fifo_full), .empty(/*unused*/)
    );

    localparam RX_IDLE = 2'd0, RX_START = 2'd1, RX_DATA = 2'd2, RX_STOP = 2'd3;
    (* fsm_encoding = "none" *) reg [1:0]  rx_state;
    reg [DIV_W-1:0] rx_cnt;
    reg [2:0]  rx_bit;
    reg [7:0]  rx_sr;

    always @(posedge clk) begin
        rx_push <= 1'b0;
        if (rx_ovf_clr) rx_overflow <= 1'b0;

        if (rst || !armed) begin
            rx_state    <= RX_IDLE;
            rx_cnt      <= 18'd0;
            rx_bit      <= 3'd0;
            rx_sr       <= 8'h00;
            if (rst) rx_overflow <= 1'b0;
        end else begin
            case (rx_state)
                RX_IDLE: begin
                    if (rx_s1 == 1'b0) begin   // start edge (line went low)
                        rx_cnt   <= half_div;  // → middle of start bit
                        rx_state <= RX_START;
                    end
                end
                RX_START: begin
                    if (rx_cnt == 18'd1) begin
                        if (rx_s1 == 1'b0) begin       // genuine start
                            rx_cnt   <= div;   // → middle of bit0
                            rx_bit   <= 3'd0;
                            rx_state <= RX_DATA;
                        end else rx_state <= RX_IDLE;   // glitch
                    end else rx_cnt <= rx_cnt - 18'd1;
                end
                RX_DATA: begin
                    if (rx_cnt == 18'd1) begin
                        rx_sr  <= {rx_s1, rx_sr[7:1]};  // LSB first
                        rx_cnt <= div;
                        if (rx_bit == 3'd7) rx_state <= RX_STOP;
                        else                rx_bit   <= rx_bit + 3'd1;
                    end else rx_cnt <= rx_cnt - 18'd1;
                end
                RX_STOP: begin
                    if (rx_cnt == 18'd1) begin
                        // sample point at middle of stop bit; push regardless of
                        // framing (best-effort), flag overflow if FIFO is full.
                        if (rx_fifo_full) rx_overflow <= 1'b1;
                        else begin rx_push <= 1'b1; rx_pushdata <= rx_sr; end
                        rx_state <= RX_IDLE;
                    end else rx_cnt <= rx_cnt - 18'd1;
                end
            endcase
        end
    end

endmodule
