// ============================================================================
// tb_top_spi.v — the SPI master (v44, swd_engine's second job) end to end through top_v2.
//
// The host side is the real link: SPI_ARM / SPI_CS / SPI_STATUS / SWD_FEED / SWD_READ go
// through the SPI pads, spi_slave and cmd_dispatch.  The target side is a behavioural
// W25Q-style flash on four LA pads (JEDEC ID, RDSR with a busy WIP bit, WREN, page
// program, 4 KB sector erase, read), whose MISO answers late (MISO_DELAY after SCK falls,
// the flash's tCLQV plus the 330 R and cable) to check the engine samples after the
// synchroniser, not before.
//
// Checked on the pins, not only the data:
//   * MOSI is stable for >= one clk before every SCK rise, and does not change within
//     10 ns after it (target setup / hold);
//   * every SCK high and low phase inside CS lasts >= half clk (the rate we asked for);
//   * CS is driven high (released) after SPI_ARM, and high-Z again after SWD_ARM / DISARM;
//   * SCK idles at CPOL.
// Second run with different parameters (AGENTS.md): re-armed on other channels, mode 3 and
// half = 40, where the engine is slower than the link so the queue actually fills.  Then
// SWD_ARM on the same engine must drive plain SWD again, and DISARM clears SPI_STATUS.
//
// Run:  make spitest
// ============================================================================
`timescale 1ns/1ps
module tb_top_spi;
    reg clk48 = 1'b0;
    always #10 clk48 = ~clk48;          // 50 MHz sim: clk = 25 MHz, 40 ns

    localparam CLK_NS     = 40;
    localparam MISO_DELAY = 30;          // flash tCLQV + RC, after SCK falls

    reg  sck = 1'b0, mosi = 1'b0, csn = 1'b1;
    reg  bus_own = 1'b1;
    reg  adc_sdo = 1'b1;
    wire miso, busy;
    wire adc_cnvst, adc_sclk, adc_sdi, dac_sync, dac_sclk, dac_din;
    wire led_g, led_b, led_r;
    wire psram_sclk, psram_cs, psram_io0, psram_io1, psram_io2, psram_io3;
    wire [13:0] la;
    genvar gi;
    generate
        for (gi = 0; gi < 14; gi = gi + 1) begin : g_pd
            pulldown (la[gi]);           // an undriven pad reads 0: "released" is visible
        end
    endgenerate

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

    integer errors = 0;

    // ------------------------------------------------------------------------
    // Behavioural SPI flash on runtime-selected LA channels (0-based).
    // ------------------------------------------------------------------------
    integer f_sck = 2, f_mosi = 3, f_miso = 4, f_cs = 5;
    integer half_want = 2;               // clk per SCK phase the pod was armed with
    wire    F_SCK  = la[f_sck];
    wire    F_MOSI = la[f_mosi];
    wire    F_CS   = la[f_cs];
    reg     f_out = 1'b0, f_oe = 1'b0;
    generate
        for (gi = 0; gi < 14; gi = gi + 1) begin : g_miso
            assign la[gi] = (gi == f_miso && f_oe) ? f_out : 1'bz;
        end
    endgenerate

    reg [7:0]  mem [0:65535];
    reg [7:0]  in_sr, out_sr, opcode;
    integer    nbits, nbytes;
    reg [23:0] addr;
    reg        wel = 1'b0;
    realtime   busy_until = 0;
    function wip; input dummy; wip = ($realtime < busy_until); endfunction
    integer    k;
    initial for (k = 0; k < 65536; k = k + 1) mem[k] = 8'hFF;

    realtime t_rise = 0, t_fall = 0, t_mosi = 0;
    realtime min_hi = 1e9, min_lo = 1e9;

    always @(negedge F_CS) begin
        nbits = 0; nbytes = 0; out_sr = 8'h00; f_oe = 1'b1;
    end
    always @(posedge F_CS) begin
        f_oe = 1'b0;
        // A write finishes on CS rising after a whole number of bytes.
        if (opcode == 8'h02 || opcode == 8'h20) begin
            wel = 1'b0;
            busy_until = $realtime + 300000;  // 300 us of WIP: several RDSR polls over the link
        end
        opcode = 8'h00;
    end

    always @(F_MOSI) if (!F_CS) begin
        if ($realtime - t_rise < 10) begin
            $display("FAIL: MOSI changed %0t ns after SCK rose (hold)", $realtime - t_rise);
            errors = errors + 1;
        end
        t_mosi = $realtime;
    end

    always @(posedge F_SCK) if (!F_CS) begin
        if ($realtime - t_mosi < CLK_NS - 1) begin
            $display("FAIL: MOSI set up only %0t ns before SCK rose", $realtime - t_mosi);
            errors = errors + 1;
        end
        if (nbits > 0 && $realtime - t_fall < min_lo) min_lo = $realtime - t_fall;
        t_rise = $realtime;
        in_sr = {in_sr[6:0], F_MOSI};
        nbits = nbits + 1;
        if (nbits % 8 == 0) begin
            if (nbytes == 0) begin
                opcode = in_sr;
                case (in_sr)
                    8'h9F: out_sr = 8'hEF;
                    8'h05: out_sr = {7'b0, wip(1'b0)};
                    8'h06: wel = 1'b1;
                    default: ;
                endcase
            end else begin
                case (opcode)
                    8'h9F: out_sr = (nbytes == 1) ? 8'h40 : (nbytes == 2) ? 8'h17 : 8'h00;
                    8'h05: out_sr = {7'b0, wip(1'b0)};
                    8'h02, 8'h03, 8'h20: begin
                        if (nbytes <= 3) addr = {addr[15:0], in_sr};
                        if (opcode == 8'h02 && nbytes >= 4 && wel && !wip(1'b0))
                            mem[{addr[15:8], addr[7:0] + 8'(nbytes - 4)}] =
                                mem[{addr[15:8], addr[7:0] + 8'(nbytes - 4)}] & in_sr;
                        if (opcode == 8'h20 && nbytes == 3 && wel && !wip(1'b0))
                            for (k = 0; k < 4096; k = k + 1) mem[{addr[15:12], 12'h000} + k] = 8'hFF;
                        if (opcode == 8'h03 && nbytes >= 3)
                            out_sr = mem[addr[15:0] + (nbytes - 3)];
                    end
                    default: ;
                endcase
            end
            nbytes = nbytes + 1;
        end
    end

    always @(negedge F_SCK) if (!F_CS) begin
        if (nbits > 0 && $realtime - t_rise < min_hi) min_hi = $realtime - t_rise;
        t_fall = $realtime;
        f_out <= #(MISO_DELAY) out_sr[7];
        out_sr = {out_sr[6:0], 1'b0};
    end

    // ------------------------------------------------------------------------
    // Host side: the STM32's link framing (signal_engine.c spi_cmd_write/read).
    // ------------------------------------------------------------------------
    localparam SPI_HALF = 250;           // 2 MHz link, like the pod's 1.95 MHz

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

    reg [7:0] junk, r;
    reg [7:0] txb [0:511];
    reg [7:0] rxb [0:511];

    task cs_lo; begin csn = 1'b0; #(SPI_HALF); end endtask
    task cs_hi; begin #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF); end endtask

    task cmd0(input [7:0] op);
        begin cs_lo; spi_byte(op, junk); cs_hi; end
    endtask
    task cmd_arg1(input [7:0] op, input [7:0] a);
        begin cs_lo; spi_byte(op, junk); spi_byte(a, junk); cs_hi; end
    endtask
    task cmd1(input [7:0] op, output [7:0] reply);
        begin cs_lo; spi_byte(op, junk); spi_byte(8'h00, reply); cs_hi; end
    endtask

    task spi_arm(input [3:0] s, input [3:0] mo, input [3:0] mi, input [3:0] c,
                 input [7:0] half, input [7:0] flags);
        begin
            cs_lo;
            spi_byte(8'h55, junk);
            spi_byte({4'h0, s}, junk);  spi_byte({4'h0, mo}, junk);
            spi_byte({4'h0, mi}, junk); spi_byte({4'h0, c}, junk);
            spi_byte(half, junk);       spi_byte(flags, junk);
            cs_hi;
            f_sck = s; f_mosi = mo; f_miso = mi; f_cs = c; half_want = half;
            min_hi = 1e9; min_lo = 1e9;
        end
    endtask

    // Queue txb[0..n-1] (SWD_FEED), wait for SPI_STATUS idle, read the n replies (SWD_READ).
    task xfer(input integer n);
        integer j, polls;
        reg [15:0] n1;
        begin
            n1 = n + 1;
            cs_lo;
            spi_byte(8'h51, junk); spi_byte(n[7:0], junk); spi_byte(n[15:8], junk);
            // v45: the engine shifts bit 0 first; an MSB-first device gets bit-reversed bytes
            for (j = 0; j < n; j = j + 1) spi_byte(rev8(txb[j]), junk);
            cs_hi;
            polls = 0;
            r = 8'h02;
            while (r[1] && polls < 2000) begin cmd1(8'h57, r); polls = polls + 1; end
            if (r !== 8'h01) begin
                $display("FAIL: SPI_STATUS after a feed = 0x%02x (want 0x01, armed + idle)", r);
                errors = errors + 1;
            end
            // SWD_READ sends reply byte 0 twice (it always has; the SWD firmware's ack_lead
            // absorbs it), so ask for one more byte and drop the first.
            cs_lo;
            spi_byte(8'h52, junk); spi_byte(n1[7:0], junk); spi_byte(n1[15:8], junk);
            spi_byte(8'h00, junk);
            for (j = 0; j < n; j = j + 1) begin spi_byte(8'h00, rxb[j]); rxb[j] = rev8(rxb[j]); end
            cs_hi;
        end
    endtask

    function [7:0] rev8(input [7:0] b);
        integer q; begin for (q = 0; q < 8; q = q + 1) rev8[q] = b[7 - q]; end
    endfunction

    task check(input [7:0] got, input [7:0] want, input [255:0] name);
        begin
            if (got !== want) begin
                $display("FAIL tb_top_spi: %0s = 0x%02x, want 0x%02x", name, got, want);
                errors = errors + 1;
            end
        end
    endtask

    task pin(input got, input want, input [255:0] name);
        begin
            if (got !== want) begin
                $display("FAIL tb_top_spi: %0s = %b, want %b", name, got, want);
                errors = errors + 1;
            end else $display("  ok: %0s = %b", name, got);
        end
    endtask

    task check_rates;
        begin
            if (min_hi < half_want * CLK_NS - 1 || min_lo < half_want * CLK_NS - 1) begin
                $display("FAIL: SCK phases hi %0t / lo %0t ns, want >= %0d", min_hi, min_lo, half_want * CLK_NS);
                errors = errors + 1;
            end else
                $display("  ok: SCK phases hi %0t / lo %0t ns (half %0d clk)", min_hi, min_lo, half_want);
        end
    endtask

    task jedec_id(input [255:0] tag);
        begin
            cmd_arg1(8'h56, 8'h01);
            txb[0] = 8'h9F; txb[1] = 0; txb[2] = 0; txb[3] = 0;
            xfer(4);
            cmd_arg1(8'h56, 8'h00);
            check(rxb[1], 8'hEF, "JEDEC manufacturer"); check(rxb[2], 8'h40, "JEDEC type");
            check(rxb[3], 8'h17, "JEDEC capacity");
            $display("  ok?: %0s JEDEC ID %02x %02x %02x", tag, rxb[1], rxb[2], rxb[3]);
        end
    endtask

    task wait_wip;
        integer polls;
        begin
            polls = 0;
            rxb[1] = 8'h01;
            while (rxb[1][0] && polls < 200) begin
                cmd_arg1(8'h56, 8'h01);
                txb[0] = 8'h05; txb[1] = 8'h00;
                xfer(2);
                cmd_arg1(8'h56, 8'h00);
                polls = polls + 1;
            end
            if (polls < 2) begin
                $display("FAIL: WIP never read busy (the RDSR path returned %02x at once)", rxb[1]);
                errors = errors + 1;
            end
            if (rxb[1][0]) begin $display("FAIL: WIP stuck"); errors = errors + 1; end
        end
    endtask

    // WREN, page program n bytes of (seed + i*7) at a, wait, read them back.
    task program_and_verify(input [15:0] a, input integer n, input [7:0] seed);
        integer j;
        begin
            cmd_arg1(8'h56, 8'h01); txb[0] = 8'h06; xfer(1); cmd_arg1(8'h56, 8'h00);
            cmd_arg1(8'h56, 8'h01);
            txb[0] = 8'h02; txb[1] = 8'h00; txb[2] = a[15:8]; txb[3] = a[7:0];
            for (j = 0; j < n; j = j + 1) txb[4 + j] = seed + j * 7;
            xfer(4 + n);
            cmd_arg1(8'h56, 8'h00);
            wait_wip;
            cmd_arg1(8'h56, 8'h01);
            txb[0] = 8'h03; txb[1] = 8'h00; txb[2] = a[15:8]; txb[3] = a[7:0];
            for (j = 0; j < n; j = j + 1) txb[4 + j] = 8'h00;
            xfer(4 + n);
            cmd_arg1(8'h56, 8'h00);
            for (j = 0; j < n; j = j + 1) check(rxb[4 + j], seed + j * 7, "read back");
            $display("  ok?: programmed + read back %0d bytes at 0x%04x", n, a);
        end
    endtask

    initial begin
        #5000;
        cmd1(8'h02, r); check(r, 8'd46, "VERSION");
        cmd1(8'h57, r); check(r, 8'h00, "SPI_STATUS at boot");
        pin(la[5], 1'b0, "CS pad before arm (high-Z)");

        // ---- run 1: LA3..LA6, mode 0, half 2 (6 MHz at the real 24 MHz clk) ----
        spi_arm(4'd2, 4'd3, 4'd4, 4'd5, 8'd2, 8'h00);
        #2000;
        cmd1(8'h57, r); check(r, 8'h01, "SPI_STATUS armed");
        pin(la[5], 1'b1, "CS driven high after SPI_ARM");
        pin(la[2], 1'b0, "SCK idles low in mode 0");
        jedec_id("mode 0 half 2");
        program_and_verify(16'h0100, 16, 8'h11);
        program_and_verify(16'h0200, 256, 8'h3C);     // a full page: 260-byte feed
        // erase the 4 KB sector holding 0x0100/0x0200 and check it reads FF
        cmd_arg1(8'h56, 8'h01); txb[0] = 8'h06; xfer(1); cmd_arg1(8'h56, 8'h00);
        cmd_arg1(8'h56, 8'h01); txb[0] = 8'h20; txb[1] = 0; txb[2] = 8'h00; txb[3] = 8'h00;
        xfer(4); cmd_arg1(8'h56, 8'h00);
        wait_wip;
        cmd_arg1(8'h56, 8'h01); txb[0] = 8'h03; txb[1] = 0; txb[2] = 8'h02; txb[3] = 8'h00;
        for (k = 4; k < 12; k = k + 1) txb[k] = 0;
        xfer(12); cmd_arg1(8'h56, 8'h00);
        for (k = 4; k < 12; k = k + 1) check(rxb[k], 8'hFF, "erased byte");
        check_rates;

        // ---- run 2: re-arm on LA9..LA12, mode 3, half 40 (engine slower than the link) ----
        spi_arm(4'd8, 4'd9, 4'd10, 4'd11, 8'd40, 8'h01);
        #2000;
        pin(la[8], 1'b1, "SCK idles high in mode 3");
        pin(la[11], 1'b1, "new CS driven high");
        pin(la[5], 1'b0, "old CS released");
        jedec_id("mode 3 half 40");
        program_and_verify(16'h0300, 32, 8'h5A);
        pin(la[8], 1'b1, "SCK back at idle high");
        check_rates;

        // ---- the same engine back on SWD ----
        cs_lo; spi_byte(8'h50, junk); spi_byte(8'd0, junk); spi_byte(8'd1, junk); spi_byte(8'hFF, junk); cs_hi;
        cmd1(8'h57, r); check(r, 8'h00, "SPI_STATUS after SWD_ARM");
        pin(la[11], 1'b0, "CS released after SWD_ARM");
        cs_lo; spi_byte(8'h51, junk); spi_byte(8'd1, junk); spi_byte(8'd0, junk); spi_byte("g", junk); cs_hi;
        #1000;
        pin(la[0], 1'b1, "SWCLK after 'g'"); pin(la[1], 1'b1, "SWDIO after 'g'");
        cmd1(8'h03, r); check(r & 8'h10, 8'h10, "STATUS swd_armed");
        cmd0(8'h53);
        cmd1(8'h03, r); check(r & 8'h10, 8'h00, "STATUS after DISARM");

        if (errors == 0)
            $display("PASS tb_top_spi: SPI master reads ID, programs, erases and reads a flash in modes 0 and 3, re-arms, hands back to SWD");
        else
            $display("FAIL tb_top_spi: %0d error(s)", errors);
        $finish;
    end

    initial begin #80000000 $display("FAIL tb_top_spi: timeout"); $finish; end
endmodule
