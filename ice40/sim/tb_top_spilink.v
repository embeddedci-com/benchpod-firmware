// ============================================================================
// tb_top_spilink.v — the STM32 <-> iCE40 command link at full speed, through the whole top.
//
// spi_slave runs its shift logic on SCK (v46 prototype) and needs an idle gap between bytes
// (STM32H5 MIDI) so cmd_dispatch can answer.  This drives the link like the H5 master does:
// mode 1 (launch on rise, sample on fall), frames of 8 SCK, rise-to-rise (1 + MIDI) * T between frames, CSn from a GPIO.
//
// Per link setting (25 MHz / MIDI 6 and 15.625 MHz / MIDI 4, then 25 MHz again after the
// slower run, i.e. re-armed with different parameters):
//   * PING / VERSION / STATUS,
//   * I2C_LOAD_REGS of 256 bytes, then I2C_READ_REGS of all 256 back (a long read stream that
//     hits the one-byte-ahead preload on every byte), with a different pattern each run.
//     READ_REGS sends register `base` twice, like SWD_READ does with byte 0 (cmd_dispatch
//     preloads it in S_READ_LEN1 and again on the first S_REG_READ byte; the old slave does
//     the same), so reply byte i is register base + max(i - 1, 0),
//   * CSn abort mid-byte: an opcode cut after 3..7 bits must be dropped (the next command
//     works), and a LOAD_REGS cut mid-data-byte must not write that byte,
//   * CSn rising right after the last SCK edge must not lose the last byte (the CS_SYNC depth),
//   * MISO must never change while SCK is low with CSn low (the pad register launches it on
//     rising edges only, so it is still during the sampling fall and the gap).
// Then, as an expected failure (indented rows), MIDI 0 at 25 MHz: the reply's first bit is the
// previous tx_byte's, which is why the firmware must program MIDI.
//
// clk48 is the real 48 MHz here (clk = 24 MHz), since the gap is counted in clk cycles.
// Run:  make spilinktest
// ============================================================================
`timescale 1ns/1ps
module tb_top_spilink;
    reg clk48 = 1'b0;
    always #10.4167 clk48 = ~clk48;

    reg  sck = 1'b0, mosi = 1'b0, csn = 1'b1;
    reg  bus_own = 1'b1;
    reg  adc_sdo = 1'b1;
    wire miso, busy;
    wire adc_cnvst, adc_sclk, adc_sdi, dac_sync, dac_sclk, dac_din;
    wire led_g, led_b, led_r;
    wire psram_sclk, psram_cs, psram_io0, psram_io1, psram_io2, psram_io3;
    wire [13:0] la;
    genvar gi;
    generate for (gi = 0; gi < 14; gi = gi + 1) begin : g_pd pulldown (la[gi]); end endgenerate

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

    // ---- link settings (changed between runs) ----
    real    half = 20.0;       // SCK half period, ns
    integer midi = 6;          // idle SCK periods between frames
    integer errors = 0;
    integer quiet = 0;         // 1: expected-failure run, report indented and don't count

    // MISO must not move while SCK is low inside a transfer.
    always @(miso)
        if (!csn && !sck && $time > 0) begin
            if (!quiet) begin
                $display("FAIL tb_top_spilink: MISO changed while SCK low at %0t", $time);
                errors = errors + 1;
            end
        end

    task spi_byte(input [7:0] tx, output [7:0] rx);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin   // mode 1: launch on rise, sample on fall
                sck = 1'b1; mosi = tx[b]; #(half);
                sck = 1'b0; rx[b] = miso;  #(half);
            end
            #(2.0 * half * midi);            // MIDI: rise-to-rise = (1 + midi) * T
        end
    endtask

    // Cut a byte after nbits rising edges (CSn abort mid-byte).
    task spi_partial(input [7:0] tx, input integer nbits);
        integer b;
        begin
            for (b = 7; b > 7 - nbits; b = b - 1) begin
                sck = 1'b1; mosi = tx[b]; #(half); sck = 1'b0; #(half);
            end
        end
    endtask

    reg [7:0] junk, r;
    task cs_lo; begin csn = 1'b0; #(2.0 * half); end endtask
    // CSn up after the last frame's gap (the H5 raises it from a GPIO after EOT).
    task cs_hi; begin csn = 1'b1; #(400); end endtask

    task check(input [7:0] got, input [7:0] want, input [255:0] name);
        begin
            if (got !== want) begin
                if (quiet) $display("    expected-fail row: %0s = 0x%02x, want 0x%02x", name, got, want);
                else begin
                    $display("FAIL tb_top_spilink: %0s = 0x%02x, want 0x%02x", name, got, want);
                    errors = errors + 1;
                end
            end
        end
    endtask

    task cmd1(input [7:0] op, output [7:0] reply);
        begin cs_lo; spi_byte(op, junk); spi_byte(8'h00, reply); cs_hi; end
    endtask

    function [7:0] pat(input integer i, input integer seed);
        pat = (i * 37 + seed * 11 + (i >> 3)) ^ seed;
    endfunction

    task load_regs(input [7:0] base, input integer n, input integer seed);
        integer i;
        begin
            cs_lo;
            spi_byte(8'h62, junk); spi_byte(base, junk);
            spi_byte(n[7:0], junk); spi_byte(n[15:8], junk);
            for (i = 0; i < n; i = i + 1) spi_byte(pat(i, seed), junk);
            cs_hi;
        end
    endtask

    integer nbad;
    integer data_midi = -1;    // >= 0: MIDI for the data phase only (header keeps `midi`)
    task read_regs(input [7:0] base, input integer n, input integer seed, input [255:0] name);
        integer i;
        begin
            nbad = 0;
            cs_lo;
            spi_byte(8'h63, junk); spi_byte(base, junk);
            spi_byte(n[7:0], junk); spi_byte(n[15:8], junk);
            if (data_midi >= 0) midi = data_midi;
            for (i = 0; i < n; i = i + 1) begin
                spi_byte(8'h00, r);
                if (r !== pat(i > 0 ? i - 1 : 0, seed)) begin
                    if (nbad < 3 && !quiet)
                        $display("FAIL tb_top_spilink: %0s byte %0d = 0x%02x, want 0x%02x",
                                 name, i, r, pat(i > 0 ? i - 1 : 0, seed));
                    nbad = nbad + 1;
                end
            end
            cs_hi;
            if (quiet)
                $display("    expected-fail row: %0s: %0d/%0d bytes wrong", name, nbad, n);
            else if (nbad) errors = errors + 1;
            else $display("  ok: %0s: %0d bytes read back", name, n);
        end
    endtask

    task run(input real h, input integer m, input integer seed, input [255:0] tag);
        integer k;
        begin
            half = h; midi = m;
            $display("  -- %0s: SCK %0.3f MHz, MIDI %0d (rise-to-rise %0.0f ns)",
                     tag, 1000.0 / (2.0 * h), m, 2.0 * h * (1 + m));
            cmd1(8'h01, r); check(r, 8'hA5, "PING");
            cmd1(8'h02, r); check(r, 8'd46, "VERSION");
            cmd1(8'h03, r); check(r, 8'h00, "STATUS");
            load_regs(8'h00, 256, seed);
            read_regs(8'h00, 256, seed, "READ_REGS 256");
            // a short read right after, with a different length
            read_regs(8'h00, 7, seed, "READ_REGS 7");

            // CSn abort mid-opcode after 3..7 bits: dropped, next command answers.
            for (k = 3; k <= 7; k = k + 1) begin
                cs_lo; spi_partial(8'h02, k); cs_hi;
                cmd1(8'h01, r); check(r, 8'hA5, "PING after a cut opcode");
            end
            // CSn abort in the middle of a LOAD_REGS data byte: byte 2 must stay unwritten.
            cs_lo;
            spi_byte(8'h62, junk); spi_byte(8'h00, junk); spi_byte(8'd4, junk); spi_byte(8'd0, junk);
            spi_byte(8'h5A, junk); spi_byte(8'hC3, junk); spi_partial(8'hFF, 5);
            cs_hi;
            cs_lo;
            spi_byte(8'h63, junk); spi_byte(8'h00, junk); spi_byte(8'd4, junk); spi_byte(8'd0, junk);
            spi_byte(8'h00, r); check(r, 8'h5A, "reg0 (sent twice) after cut LOAD");
            spi_byte(8'h00, r); check(r, 8'h5A, "reg0 after cut LOAD");
            spi_byte(8'h00, r); check(r, 8'hC3, "reg1 after cut LOAD");
            spi_byte(8'h00, r); check(r, pat(2, seed), "reg2 untouched by the cut byte");
            cs_hi;

            // CSn right after the last SCK fall (no gap): the last byte must still land.
            cs_lo;
            spi_byte(8'h62, junk); spi_byte(8'h10, junk); spi_byte(8'd1, junk); spi_byte(8'd0, junk);
            for (k = 7; k >= 0; k = k - 1) begin
                sck = 1'b1; mosi = 8'h96 >> k; #(half); sck = 1'b0; #(half);
            end
            csn = 1'b1; #(400);
            cs_lo;
            spi_byte(8'h63, junk); spi_byte(8'h10, junk); spi_byte(8'd2, junk); spi_byte(8'd0, junk);
            spi_byte(8'h00, r); check(r, 8'h96, "last byte with CSn right after it");
            spi_byte(8'h00, r); check(r, 8'h96, "last byte with CSn right after it (2nd)");
            cs_hi;
        end
    endtask

    initial begin
        #5000;   // power-on reset
        run(20.0, 6, 1, "25 MHz");
        run(32.0, 4, 2, "15.625 MHz");
        run(20.0, 6, 3, "25 MHz again");
        // 31.25 MHz (/8) with MIDI 8: the logic is fine at any rate; the pad timing is what
        // limits it (see the nextpnr sck reports), which sim can't see.
        run(16.0, 8, 4, "31.25 MHz");
        // The firmware's boot setting (/128, no MIDI): its half period alone covers the gap, so
        // the firmware can talk to this gateware and to the old clk-sampled slave (which also
        // works in mode 1 at /128) before it reads VERSION and raises the clock.
        run(256.0, 0, 5, "1.95 MHz boot setting");

        // Expected failure: no inter-byte gap at 25 MHz.
        quiet = 1;
        $display("  -- expected failure: 25 MHz, MIDI 0");
        half = 20.0; midi = 6;
        load_regs(8'h00, 32, 5);
        data_midi = 0;
        read_regs(8'h00, 32, 5, "READ_REGS data phase at MIDI 0");
        data_midi = -1; midi = 6;
        quiet = 0;
        cmd1(8'h01, r); check(r, 8'hA5, "PING after the MIDI-0 run");

        if (errors == 0)
            $display("PASS tb_top_spilink: PING/VERSION/STATUS, 256-byte load + read-back streams and CSn aborts at 25/15.6/25/31.25 MHz with MIDI and 1.95 MHz without; MIDI 0 at 25 MHz fails as expected");
        else
            $display("FAIL tb_top_spilink: %0d error(s)", errors);
        $finish;
    end
endmodule
