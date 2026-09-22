// ============================================================================
// tb_top_v2.v — top-level integration smoke test for the v2 gateware.
//
// The per-module benches cover the engines in isolation, but nothing exercises
// top_v2 as a whole — SPI pad -> spi_slave -> cmd_dispatch -> tx_byte -> MISO pad,
// through the real SB_IO / SB_GB / SB_RGBA_DRV primitives and the clk48÷2 clock
// derivation and power-on reset.  That top-level seam is exactly where the last
// two field bugs lived (the 0x5555 mis-latch and the config-pin tristate), so this
// drives a behavioural SPI master (mode 0) and checks the instant-response
// commands: PING -> 0xA5, VERSION -> 36, STATUS -> known flag byte, TRIGGER_STATUS -> idle, and
// GPIO_GET (v35) returning a pattern driven onto the LA pads.
//
// It needs the iCE40 primitive sim models (yosys' cells_sim.v), passed on the
// iverilog command line by the `topsmoketest` Makefile target.
//
// Run:  make topsmoketest
// ============================================================================
`timescale 1ns/1ps
module tb_top_v2;
    // 48 MHz oscillator on the global clock pad (clk = clk48/2 = 24 MHz inside).
    reg clk48 = 1'b0;
    always #10 clk48 = ~clk48;          // ~50 MHz sim (exact rate immaterial)

    // SPI link (mode 0) + misc top ports
    reg  sck = 1'b0, mosi = 1'b0, csn = 1'b1;
    reg  bus_own = 1'b1;                 // STM32 owns the shared bus -> iCE40 releases
                                         // the PSRAM pads (command-only test)
    reg  adc_sdo = 1'b1;
    wire miso, busy;
    wire adc_cnvst, adc_sclk, adc_sdi, dac_sync, dac_sclk, dac_din;
    wire led_g, led_b, led_r;
    // shared quad bus + LA bank: left to float (pulled down so la_in is defined;
    // no LA driver is armed in this test).
    wire psram_sclk, psram_cs, psram_io0, psram_io1, psram_io2, psram_io3;
    wire [13:0] la;
    genvar gi;
    generate
        for (gi = 0; gi < 14; gi = gi + 1) begin : g_pd
            pulldown (la[gi]);
        end
    endgenerate
    reg [13:0] la_pat = 14'h0000;         // LA1..LA14 driven for GPIO_GET (la_bank never drives here)
    assign la = la_pat;

    top dut (
        .clk48(clk48),
        .sck(sck), .mosi(mosi), .miso(miso), .csn(csn), .busy(busy),
        .bus_own(bus_own),
        .adc_cnvst(adc_cnvst), .adc_sclk(adc_sclk), .adc_sdi(adc_sdi), .adc_sdo(adc_sdo),
        .dac_sync(dac_sync), .dac_sclk(dac_sclk), .dac_din(dac_din),
        .psram_sclk(psram_sclk), .psram_cs(psram_cs),
        .psram_io0(psram_io0), .psram_io1(psram_io1),
        .psram_io2(psram_io2), .psram_io3(psram_io3),
        .la(la),
        .led_g(led_g), .led_b(led_b), .led_r(led_r)
    );

    localparam SPI_HALF = 500;           // 1 us bit period (1 MHz) — far below clk/4
    integer errors = 0;

    // One SPI byte, mode 0: master drives MOSI while SCK low, slave samples on the
    // rising edge; master samples MISO on the rising edge (slave drove it on the
    // previous falling edge / preload).  MSB first.
    task spi_byte(input [7:0] tx, output [7:0] rx);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b];
                #(SPI_HALF);
                sck = 1'b1;
                rx[b] = miso;
                #(SPI_HALF);
                sck = 1'b0;
            end
        end
    endtask

    // A command transaction: CSn low, send opcode, send one dummy byte to clock the
    // 1-byte reply out, CSn high.  Returns the reply byte.
    task cmd1(input [7:0] op, output [7:0] reply);
        reg [7:0] junk;
        begin
            csn = 1'b0;   #(SPI_HALF);
            spi_byte(op,     junk);
            spi_byte(8'h00,  reply);
            #(SPI_HALF);   csn = 1'b1;
            #(4*SPI_HALF);            // idle gap (FSM returns to IDLE on CSn high)
        end
    endtask

    task check(input [7:0] got, input [7:0] want, input [127:0] name);
        begin
            if (got !== want) begin
                $display("FAIL tb_top_v2: %0s = 0x%02x, want 0x%02x", name, got, want);
                errors = errors + 1;
            end else
                $display("  ok: %0s = 0x%02x", name, got);
        end
    endtask

    // opcode + two dummy bytes clocking out a 2-byte LE reply (ADC_PROBE / GPIO_GET)
    task cmd2(input [7:0] op, output [15:0] reply);
        reg [7:0] junk, lo, hi;
        begin
            csn = 1'b0;   #(SPI_HALF);
            spi_byte(op,    junk);
            spi_byte(8'h00, lo);
            spi_byte(8'h00, hi);
            #(SPI_HALF);   csn = 1'b1;
            #(4*SPI_HALF);
            reply = {hi, lo};
        end
    endtask

    reg [7:0] r;
    reg [15:0] v;
    initial begin
        // Wait out the internal power-on reset (64 clk48 + sync) with margin.
        #5000;
        cmd1(8'h01, r); check(r, 8'hA5, "PING");
        cmd1(8'h02, r); check(r, 8'd36, "VERSION");
        // Fresh boot: dac/cap/step/swd all idle, no capture overflow -> STATUS 0x00.
        cmd1(8'h03, r); check(r, 8'h00, "STATUS");
        cmd1(8'h34, r); check(r, 8'h00, "TRIGGER_STATUS");
        // GPIO_GET: pad levels through SB_IO -> la_bank -> the 2-flop synchroniser -> MISO.
        // Bits 12/13 are LA13/LA14; bits 15:14 of the reply must stay 0.
        la_pat = 14'h2A5C; #2000;
        cmd2(8'h43, v); check(v[7:0], 8'h5C, "GPIO_GET lo 2A5C"); check(v[15:8], 8'h2A, "GPIO_GET hi 2A5C");
        la_pat = 14'h13C3; #2000;
        cmd2(8'h43, v); check(v[7:0], 8'hC3, "GPIO_GET lo 13C3"); check(v[15:8], 8'h13, "GPIO_GET hi 13C3");
        la_pat = 14'h3FFF; #2000;
        cmd2(8'h43, v); check(v[7:0], 8'hFF, "GPIO_GET lo 3FFF"); check(v[15:8], 8'h3F, "GPIO_GET hi 3FFF");

        if (errors == 0)
            $display("PASS tb_top_v2: PING/VERSION/STATUS/TRIGGER_STATUS/GPIO_GET answer correctly through the SPI+pad top");
        else
            $display("FAIL tb_top_v2: %0d error(s)", errors);
        $finish;
    end

    initial begin #200000 $display("FAIL tb_top_v2: timeout"); $finish; end
endmodule
