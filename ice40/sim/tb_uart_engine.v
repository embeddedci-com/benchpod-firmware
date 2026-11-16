// ============================================================================
// tb_uart_engine.v — self-checking loopback testbench for the soft UART.
//
// Ties tx_out → rx_in (external loopback), pushes bytes into the TX FIFO,
// lets the engine serialise + deserialise them, then pops the RX FIFO and
// checks the bytes round-trip.  Also checks rx_avail tracking and the
// overflow flag, and the EXACT bit period on the wire (a 0x55 frame toggles every bit,
// so each of its 9 in-frame edges must be exactly div clocks apart), re-armed with an
// odd divider to cover the half-bit RX start wait.  v56: 8E1 / 8O1 / 8N2 / 8E2 frames on the
// wire bit by bit, the gap two stop bits leave between back-to-back frames, and a parity error
// seen by a second receiver set to the other parity (the byte is kept, the sticky flag set).
//
// Run:  make -C ice40 uarttest      (needs iverilog/vvp)
// ============================================================================

`timescale 1ns/1ps

module tb_uart_engine;

    reg clk = 1'b0;
    always #5 clk = ~clk;     // 100 MHz sim clock (timing irrelevant in sim)
    reg rst = 1'b1;

    // config
    reg        cfg_stb = 1'b0;
    reg [23:0] cfg_div = 24'd16;   // small bit period for a fast sim
    reg        cfg_enable = 1'b0;
    reg [2:0]  cfg_frame = 3'd0;     // {stop2, odd, parity}

    // TX FIFO write
    reg        tx_we = 1'b0;
    reg [7:0]  tx_wdata = 8'h00;
    wire       tx_full, tx_empty;

    // RX FIFO read
    reg        rx_re = 1'b0;
    wire [7:0] rx_rdata;
    wire [8:0] rx_avail;
    wire       rx_overflow, rx_frame_err, rx_parity_err;
    reg        rx_ovf_clr = 1'b0;

    // loopback
    wire       tx_out;
    reg        glitch = 1'b0;      // 1 pulls the RX line low (start-bit / glitch tests)
    wire       uart_armed;

    uart_engine dut (
        .clk(clk), .rst(rst),
        .cfg_stb(cfg_stb), .cfg_rx_ch(4'd0), .cfg_tx_ch(4'd1),
        .cfg_div(cfg_div), .cfg_enable(cfg_enable), .cfg_frame(cfg_frame), .disable_stb(1'b0),
        .rx_ch(), .tx_ch(), .armed(uart_armed), .rx_parity_err(rx_parity_err),
        .tx_we(tx_we), .tx_wdata(tx_wdata), .tx_full(tx_full), .tx_empty(tx_empty),
        .rx_re(rx_re), .rx_rdata(rx_rdata), .rx_avail(rx_avail),
        .rx_overflow(rx_overflow), .rx_ovf_clr(rx_ovf_clr), .rx_frame_err(rx_frame_err),
        .rx_in(tx_out & ~glitch),  // <-- loopback (+ injected low pulses)
        .tx_out(tx_out)
    );

    // ---- second receiver on the same wire (v56): its own frame format, for parity errors ----
    reg        cfg2_stb = 1'b0;
    reg [2:0]  cfg2_frame = 3'd0;
    reg        rx2_re = 1'b0, rx2_ovf_clr = 1'b0;
    wire [7:0] rx2_rdata;
    wire [8:0] rx2_avail;
    wire       rx2_parity_err, rx2_frame_err;
    uart_engine rx2 (
        .clk(clk), .rst(rst),
        .cfg_stb(cfg2_stb), .cfg_rx_ch(4'd0), .cfg_tx_ch(4'd1),
        .cfg_div(24'd16), .cfg_enable(1'b1), .cfg_frame(cfg2_frame), .disable_stb(1'b0),
        .rx_ch(), .tx_ch(), .armed(),
        .tx_we(1'b0), .tx_wdata(8'h00), .tx_full(), .tx_empty(),
        .rx_re(rx2_re), .rx_rdata(rx2_rdata), .rx_avail(rx2_avail),
        .rx_overflow(), .rx_ovf_clr(rx2_ovf_clr), .rx_frame_err(rx2_frame_err),
        .rx_parity_err(rx2_parity_err),
        .rx_in(tx_out), .tx_out()
    );

    integer errors = 0;

    // ---- wire sampler (v56): the next frame's bits at mid-bit, start bit first ----
    // Waits for a falling edge on tx_out, then samples n bits (start, data LSB first, parity,
    // stops) div clocks apart, the first half a bit in.
    task sample_frame(input integer div, input integer n, output [11:0] bits);
        integer k;
        begin
            bits = 12'h000;
            @(negedge tx_out);
            tick(div / 2);
            for (k = 0; k < n; k = k + 1) begin
                bits[k] = tx_out;
                tick(div);
            end
        end
    endtask

    task set_frame(input [2:0] f);
        begin
            @(posedge clk); cfg_frame = f; cfg_stb = 1'b1;
            @(posedge clk); cfg_stb = 1'b0;
            tick(4);
        end
    endtask

    // push d, check the n bits on the wire against want (bit 0 = start), then the loopback byte
    task frame_check(input [7:0] d, input integer n, input [11:0] want, input [255:0] label);
        reg [11:0] got;
        reg [7:0]  r;
        begin
            fork
                push(d);
                sample_frame(16, n, got);
            join
            tick(4 * 16);
            if (got !== want) begin
                $display("FAIL: %0s wire bits %b want %b", label, got, want); errors = errors + 1;
            end else $display("ok:   %0s wire bits %b", label, got);
            pop(r); check_eq(r, d, label);
            if (rx_parity_err !== 1'b0 || rx_frame_err !== 1'b0) begin
                $display("FAIL: %0s set an error flag", label); errors = errors + 1;
            end
        end
    endtask

    // ---- bit-period monitor: armed per 0x55 frame by the test (chk_div = expected clks) ----
    integer cyc = 0, last_edge = 0, nb = 0, chk_div = 0, edges_ok = 0;
    reg     tx_prev = 1'b1;
    always @(posedge clk) begin
        cyc <= cyc + 1;
        tx_prev <= tx_out;
        if (chk_div != 0 && tx_out !== tx_prev) begin
            if (nb == 0) begin
                if (tx_prev && !tx_out) begin nb <= 1; last_edge <= cyc; end   // start-bit fall
            end else begin
                if (cyc - last_edge != chk_div) begin
                    $display("FAIL: bit %0d lasted %0d clks, want %0d", nb - 1, cyc - last_edge, chk_div);
                    errors = errors + 1;
                end else edges_ok <= edges_ok + 1;
                last_edge <= cyc;
                nb <= (nb == 9) ? 0 : nb + 1;                                  // 9th edge = stop rise
            end
        end
    end

    task tick(input integer n); begin repeat (n) @(posedge clk); end endtask

    task push(input [7:0] d);
        begin
            @(posedge clk); tx_wdata = d; tx_we = 1'b1;
            @(posedge clk); tx_we = 1'b0;
        end
    endtask

    task pop(output [7:0] d);
        begin
            d = rx_rdata;            // current head (settled)
            @(posedge clk); rx_re = 1'b1;
            @(posedge clk); rx_re = 1'b0;
            tick(2);                 // let the FIFO present the next head
        end
    endtask

    task check_eq(input [7:0] got, input [7:0] exp, input [255:0] label);
        begin
            if (got !== exp) begin
                $display("FAIL: %0s got 0x%02x expected 0x%02x", label, got, exp);
                errors = errors + 1;
            end else $display("ok:   %0s = 0x%02x", label, got);
        end
    endtask

    // RX start-bit check: the line is re-sampled floor(div/2) clocks after the start edge,
    // so a low pulse of floor(div/2) clocks is a glitch and one clock more is a start bit
    // (whose data bits then read all ones: 0xFF).  Checks the exact boundary.
    task start_boundary(input integer div);
        reg [8:0] av0;
        begin
            av0 = rx_avail;
            @(posedge clk); glitch = 1'b1; tick(div / 2); glitch = 1'b0;
            tick(12 * div);
            if (rx_avail !== av0) begin
                $display("FAIL: div %0d: a %0d-clk low pulse was taken as a start bit", div, div / 2);
                errors = errors + 1;
            end else $display("ok:   div %0d: %0d-clk low pulse rejected", div, div / 2);
            @(posedge clk); glitch = 1'b1; tick(div / 2 + 1); glitch = 1'b0;
            tick(12 * div);
            if (rx_avail !== av0 + 9'd1) begin
                $display("FAIL: div %0d: a %0d-clk low pulse was not a start bit", div, div / 2 + 1);
                errors = errors + 1;
            end else $display("ok:   div %0d: %0d-clk low pulse starts a frame", div, div / 2 + 1);
        end
    endtask

    // One UART frame = 10 bit times; div clks per bit → ~10*div clks + sync.
    localparam integer FRAME_CLKS = 10 * 16 + 40;

    reg [7:0] b0, b1, b2;
    integer i;

    initial begin
        tick(4); rst = 1'b0; tick(4);

        // configure + enable
        @(posedge clk); cfg_enable = 1'b1; cfg_stb = 1'b1;
        @(posedge clk); cfg_stb = 1'b0;
        tick(4);

        // ---- Test 1: round-trip three bytes ----
        push(8'h55);
        push(8'hA3);
        push(8'h00);
        // wait for all three frames to serialise + loop back + deserialise
        tick(3 * FRAME_CLKS + 200);

        check_eq(rx_avail[7:0], 8'd3, "rx_avail after 3 bytes");

        pop(b0); check_eq(b0, 8'h55, "rx[0]");
        pop(b1); check_eq(b1, 8'hA3, "rx[1]");
        pop(b2); check_eq(b2, 8'h00, "rx[2]");
        check_eq(rx_avail[7:0], 8'd0, "rx_avail drained");

        // ---- Test 1b: exact bit period, then re-arm with an ODD divider ----
        chk_div = 16; edges_ok = 0; push(8'h55); tick(FRAME_CLKS + 100); chk_div = 0;
        if (edges_ok != 9) begin $display("FAIL: div 16: %0d/9 exact bit edges", edges_ok); errors = errors + 1; end
        else $display("ok:   div 16: 9/9 bit edges exactly 16 clks");
        pop(b0); check_eq(b0, 8'h55, "rx @div16");
        @(posedge clk); cfg_div = 24'd13; cfg_stb = 1'b1;
        @(posedge clk); cfg_stb = 1'b0;
        tick(4);
        chk_div = 13; edges_ok = 0; push(8'h55); tick(FRAME_CLKS + 100); chk_div = 0;
        if (edges_ok != 9) begin $display("FAIL: div 13: %0d/9 exact bit edges", edges_ok); errors = errors + 1; end
        else $display("ok:   div 13: 9/9 bit edges exactly 13 clks");
        push(8'hC6); tick(FRAME_CLKS + 100);
        pop(b0); check_eq(b0, 8'h55, "rx @div13");
        pop(b1); check_eq(b1, 8'hC6, "rx @div13 (2nd)");
        start_boundary(13);
        pop(b0); check_eq(b0, 8'hFF, "rx @div13 (bare start bit)");
        @(posedge clk); cfg_div = 24'd16; cfg_stb = 1'b1;
        @(posedge clk); cfg_stb = 1'b0;
        tick(4);
        start_boundary(16);
        pop(b0); check_eq(b0, 8'hFF, "rx @div16 (bare start bit)");

        // ---- Test 1c (v52): a frame whose stop bit is low is a framing error, not a byte ----
        // A 30-bit-time break (the line held low) used to read as a 0x00, then the still-low line
        // started the next frame at once: a stream of 0x00 until the line rose.  Now no byte is
        // stored, the sticky rx_frame_err is set (cleared with rx_overflow by the status read),
        // and the receiver waits for the line to go high before it looks for a start bit.
        tick(20);
        b2 = rx_avail[7:0];
        @(posedge clk); glitch = 1'b1; tick(30 * 16); glitch = 1'b0;
        tick(3 * 16);
        if (rx_avail[7:0] !== b2) begin
            $display("FAIL: a break stored %0d byte(s)", rx_avail[7:0] - b2); errors = errors + 1;
        end else $display("ok:   a 30-bit break stores no byte");
        if (rx_frame_err !== 1'b1) begin $display("FAIL: rx_frame_err not set by a break"); errors = errors + 1; end
        else $display("ok:   rx_frame_err set by a break");
        @(posedge clk); rx_ovf_clr = 1'b1; @(posedge clk); rx_ovf_clr = 1'b0;
        if (rx_frame_err !== 1'b0) begin $display("FAIL: rx_frame_err not cleared by the status read"); errors = errors + 1; end
        push(8'h3C); tick(FRAME_CLKS + 100);
        if (rx_avail[7:0] !== b2 + 8'd1) begin $display("FAIL: the byte after a break was not received"); errors = errors + 1; end
        else begin pop(b0); check_eq(b0, 8'h3C, "rx after a break"); end
        if (rx_frame_err !== 1'b0) begin $display("FAIL: a good frame set rx_frame_err"); errors = errors + 1; end

        // ---- Test 1d (v56): parity and stop bits on the wire, and in loopback ----
        // bits[0] start (0), [1..8] data LSB first, then parity, then stop(s) (1)
        // 0xA3 = 1010_0011: four ones -> even parity bit 0, odd parity bit 1
        // 0x07: three ones -> even 1, odd 0
        set_frame(3'b001);   // 8E1
        frame_check(8'hA3, 11, {1'b1, 1'b0, 8'hA3, 1'b0}, "8E1 0xA3");
        frame_check(8'h07, 11, {1'b1, 1'b1, 8'h07, 1'b0}, "8E1 0x07");
        set_frame(3'b011);   // 8O1
        frame_check(8'hA3, 11, {1'b1, 1'b1, 8'hA3, 1'b0}, "8O1 0xA3");
        frame_check(8'h07, 11, {1'b1, 1'b0, 8'h07, 1'b0}, "8O1 0x07");
        set_frame(3'b100);   // 8N2: two stop bits, then the line idles
        frame_check(8'h5A, 11, {2'b11, 8'h5A, 1'b0}, "8N2 0x5A");
        set_frame(3'b101);   // 8E2
        frame_check(8'h07, 12, {2'b11, 1'b1, 8'h07, 1'b0}, "8E2 0x07");

        // back-to-back frames: the next start bit falls 10 / 11 / 12 bit times after the last, plus
        // the TX_IDLE -> TX_LOAD handover's 2 clocks (8N1 has had them since v1)
        begin : gaps
            integer t0, t1, f, want_bits;
            for (f = 0; f < 3; f = f + 1) begin
                set_frame(f == 0 ? 3'b000 : (f == 1 ? 3'b100 : 3'b101));
                want_bits = 10 + f;
                push(8'h01); push(8'h01);
                @(negedge tx_out); t0 = cyc;
                @(posedge tx_out);                    // end of the start bit (data bit 0 = 1)
                @(negedge tx_out); t1 = cyc;          // data bit 1 = 0 ...
                @(negedge tx_out); t1 = cyc;          // next frame's start bit
                if (t1 - t0 !== want_bits * 16 + 2) begin
                    $display("FAIL: frame %0d: start-to-start %0d clks, want %0d", f, t1 - t0, want_bits * 16 + 2);
                    errors = errors + 1;
                end else $display("ok:   frame format %0d: start-to-start %0d bit times", f, want_bits);
                tick(2 * 12 * 16);
                pop(b0); pop(b1);
                check_eq(b0, 8'h01, "back-to-back[0]"); check_eq(b1, 8'h01, "back-to-back[1]");
            end
        end

        // parity error: dut sends 8E1, the second receiver expects 8O1 -> byte kept, flag set
        set_frame(3'b001);
        @(posedge clk); cfg2_frame = 3'b011; cfg2_stb = 1'b1; @(posedge clk); cfg2_stb = 1'b0;
        @(posedge clk); rx2_ovf_clr = 1'b1; @(posedge clk); rx2_ovf_clr = 1'b0;
        b2 = rx2_avail[7:0];
        push(8'hA3); tick(FRAME_CLKS + 200);
        if (rx2_avail[7:0] !== b2 + 8'd1) begin $display("FAIL: a parity-error byte was dropped"); errors = errors + 1; end
        else $display("ok:   a parity-error byte is kept");
        if (rx2_parity_err !== 1'b1) begin $display("FAIL: rx_parity_err not set"); errors = errors + 1; end
        else $display("ok:   rx_parity_err set on a wrong parity bit");
        if (rx2_frame_err !== 1'b0) begin $display("FAIL: a parity error also set rx_frame_err"); errors = errors + 1; end
        @(posedge clk); rx2_ovf_clr = 1'b1; @(posedge clk); rx2_ovf_clr = 1'b0;
        if (rx2_parity_err !== 1'b0) begin $display("FAIL: rx_parity_err not cleared by the status read"); errors = errors + 1; end
        // the matching receiver (dut, 8E1) saw no error
        pop(b0); check_eq(b0, 8'hA3, "8E1 loopback (parity test)");
        if (rx_parity_err !== 1'b0) begin $display("FAIL: matching receiver flagged parity"); errors = errors + 1; end
        set_frame(3'b000);
        tick(20);

        // ---- Test 2: overflow flag when RX FIFO fills (257 bytes) ----
        // push 257 bytes through; FIFO depth is 256, so at least one is dropped.
        for (i = 0; i < 257; i = i + 1) push(8'hAA);
        tick(258 * FRAME_CLKS + 500);
        if (!rx_overflow) begin
            $display("FAIL: rx_overflow not set after 257-byte burst");
            errors = errors + 1;
        end else $display("ok:   rx_overflow set on FIFO full");

        if (errors == 0) $display("\nALL TESTS PASSED");
        else             $display("\n%0d TEST(S) FAILED", errors);
        $finish;
    end

    initial begin
        #50000000;
        $display("FAIL: testbench timeout");
        $finish;
    end

endmodule
