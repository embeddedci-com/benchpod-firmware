// ============================================================================
// tb_uart_engine.v — self-checking loopback testbench for the soft UART.
//
// Ties tx_out → rx_in (external loopback), pushes bytes into the TX FIFO,
// lets the engine serialise + deserialise them, then pops the RX FIFO and
// checks the bytes round-trip.  Also checks rx_avail tracking and the
// overflow flag.
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

    // TX FIFO write
    reg        tx_we = 1'b0;
    reg [7:0]  tx_wdata = 8'h00;
    wire       tx_full, tx_empty;

    // RX FIFO read
    reg        rx_re = 1'b0;
    wire [7:0] rx_rdata;
    wire [8:0] rx_avail;
    wire       rx_overflow;
    reg        rx_ovf_clr = 1'b0;

    // loopback
    wire       tx_out;
    wire       uart_armed;

    uart_engine dut (
        .clk(clk), .rst(rst),
        .cfg_stb(cfg_stb), .cfg_rx_ch(4'd0), .cfg_tx_ch(4'd1),
        .cfg_div(cfg_div), .cfg_enable(cfg_enable), .disable_stb(1'b0),
        .rx_ch(), .tx_ch(), .armed(uart_armed),
        .tx_we(tx_we), .tx_wdata(tx_wdata), .tx_full(tx_full), .tx_empty(tx_empty),
        .rx_re(rx_re), .rx_rdata(rx_rdata), .rx_avail(rx_avail),
        .rx_overflow(rx_overflow), .rx_ovf_clr(rx_ovf_clr),
        .rx_in(tx_out),            // <-- loopback
        .tx_out(tx_out)
    );

    integer errors = 0;

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
