// ============================================================================
// tb_top_uart2.v — UART2 (v48 TX, v54 RX; the emulation image) end to end through top_v2.
//
// The host side is the real link: UART_CONFIG / UART2_CONFIG / UART_WRITE / UART2_WRITE /
// UART_STATUS go through the SPI pads, spi_slave and cmd_dispatch.  The DUT side is two
// 8N1 receivers on the LA pads that decode each byte at mid-bit and time every bit edge.
//
// Checked:
//   * UART2 and the UART proxy run at the same time on their own channels and rates, each
//     byte lands on its own pin only (0x72 writes the proxy FIFO, 0x76 the UART2 FIFO);
//   * every bit of a 0x55 frame lasts exactly `div` clk (start, 8 data, stop);
//   * UART_STATUS flags: bit6 UART2 armed, bit5 UART2 TX empty once drained, proxy bits as
//     before (bit3 armed, bit1 TX empty);
//   * second run with different parameters (AGENTS.md): UART2 re-armed on another channel at
//     another rate drives the new pin at the new rate and releases the old one;
//   * UART2_CONFIG with enable 0 disarms it and releases the pin; the proxy keeps running.
//   * v54: UART2's TX pin comes from the pin map (EMU_PINMAP code 7, driven while armed);
//     UART2 RX on its rx_ch: bytes the DUT sends are read back with UART2_READ, UART2_STATUS
//     counts them and flags a break as a framing error (sticky, cleared by the read).
//
// Run:  make uart2test
// ============================================================================
`timescale 1ns/1ps
module tb_top_uart2;
    reg clk48 = 1'b0;
    always #10 clk48 = ~clk48;          // 50 MHz sim: clk = 25 MHz, 40 ns

    localparam CLK_NS = 40;

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
    // the DUT's TX into UART2 RX (LA10)
    reg dut_tx_en = 1'b0, dut_tx = 1'b1;
    assign la[9] = dut_tx_en ? dut_tx : 1'bz;

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
    reg [7:0] junk;
    task cs_lo; begin csn = 1'b0; #(SPI_HALF); end endtask
    task cs_hi; begin #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF); end endtask

    // UART_CONFIG (0x70) / UART2_CONFIG (0x75): [rx_ch][tx_ch][div lo/mid/hi][flags]
    task uart_cfg(input [7:0] op, input [3:0] rx, input [3:0] tx, input [23:0] div, input en);
        begin
            cs_lo;
            spi_byte(op, junk);
            spi_byte({4'h0, rx}, junk); spi_byte({4'h0, tx}, junk);
            spi_byte(div[7:0], junk); spi_byte(div[15:8], junk); spi_byte(div[23:16], junk);
            spi_byte({7'b0, en}, junk);
            cs_hi;
        end
    endtask
    reg [7:0] txb [0:63];
    // UART_WRITE (0x72) / UART2_WRITE (0x76): [len lo][len hi][bytes]
    task uart_write(input [7:0] op, input integer n);
        integer j;
        begin
            cs_lo;
            spi_byte(op, junk); spi_byte(n[7:0], junk); spi_byte(n[15:8], junk);
            for (j = 0; j < n; j = j + 1) spi_byte(txb[j], junk);
            cs_hi;
        end
    endtask
    // EMU_PINMAP (0x6E): code on pad ch, every other pad 0
    task emu_pin(input [3:0] ch, input [2:0] code);
        integer j; reg [7:0] b;
        begin
            cs_lo; spi_byte(8'h6E, junk);
            for (j = 0; j < 7; j = j + 1) begin
                b = 8'h00;
                if (ch == 2*j)     b[2:0] = code;
                if (ch == 2*j + 1) b[6:4] = code;
                spi_byte(b, junk);
            end
            cs_hi;
        end
    endtask
    reg [7:0] st0, st1, st2;
    task uart_status;
        begin
            cs_lo;
            spi_byte(8'h74, junk);
            spi_byte(8'h00, st0); spi_byte(8'h00, st1); spi_byte(8'h00, st2);
            cs_hi;
        end
    endtask

    task check(input [7:0] got, input [7:0] want, input [255:0] name);
        if (got !== want) begin
            $display("FAIL tb_top_uart2: %0s = 0x%02x, want 0x%02x", name, got, want);
            errors = errors + 1;
        end
    endtask

    // ------------------------------------------------------------------------
    // DUT side: two 8N1 receivers.  rx_n counts decoded bytes into rx_buf; a 0x55 frame
    // also has every bit edge timed against the expected period.
    // ------------------------------------------------------------------------
    reg [7:0] buf_a [0:63];   integer n_a = 0;   // UART2
    reg [7:0] buf_b [0:63];   integer n_b = 0;   // UART proxy TX
    integer ch_a = 5, div_a = 30;
    integer ch_b = 1, div_b = 50;

    task automatic rx_byte(input integer ch, input integer div, output [7:0] d, output ok);
        integer b;
        realtime t0, per;
        begin
            per = div * CLK_NS;
            t0 = $realtime;
            ok = 1'b1;
            #(per / 2);
            if (la[ch] !== 1'b0) ok = 1'b0;               // start bit still low mid-bit
            for (b = 0; b < 8; b = b + 1) begin #(per); d[b] = la[ch]; end
            #(per);
            if (la[ch] !== 1'b1) ok = 1'b0;               // stop bit
        end
    endtask

    // Bit-edge timing of a 0x55 frame: start(0) 1 0 1 0 1 0 1 0 stop(1) toggles every bit.
    task automatic time_55(input integer ch, input integer div, input realtime t_start);
        integer e;
        realtime t_prev, per;
        begin
            per = div * CLK_NS;
            t_prev = t_start;
            for (e = 0; e < 9; e = e + 1) begin
                @(la[ch]);
                if ($realtime - t_prev < per - 1 || $realtime - t_prev > per + 1) begin
                    $display("FAIL tb_top_uart2: LA%0d bit %0d lasted %0t ns, want %0t",
                             ch + 1, e, $realtime - t_prev, per);
                    errors = errors + 1;
                end
                t_prev = $realtime;
            end
        end
    endtask

    reg [7:0] d_a, d_b;
    reg ok_a, ok_b;
    reg rx_on = 1'b0;
    always @(negedge la[ch_a]) if (rx_on) begin : rxa
        realtime ts;
        ts = $realtime;
        fork
            rx_byte(ch_a, div_a, d_a, ok_a);
            if (n_a == 0) time_55(ch_a, div_a, ts);       // the first byte of each run is 0x55
        join
        if (!ok_a) begin $display("FAIL tb_top_uart2: UART2 framing error on LA%0d", ch_a + 1); errors = errors + 1; end
        buf_a[n_a] = d_a; n_a = n_a + 1;
    end
    always @(negedge la[ch_b]) if (rx_on) begin : rxb
        rx_byte(ch_b, div_b, d_b, ok_b);
        if (!ok_b) begin $display("FAIL tb_top_uart2: proxy framing error on LA%0d", ch_b + 1); errors = errors + 1; end
        buf_b[n_b] = d_b; n_b = n_b + 1;
    end

    // Every pad other than the expected drivers must stay released (pulldown: 0).
    task pads_released(input integer keep1, input integer keep2, input [255:0] when);
        integer c;
        for (c = 0; c < 14; c = c + 1)
            if (c != keep1 && c != keep2 && la[c] !== 1'b0) begin
                $display("FAIL tb_top_uart2: LA%0d driven (%b) %0s", c + 1, la[c], when);
                errors = errors + 1;
            end
    endtask

    // the DUT sends one 8N1 byte at div_r clk per bit
    integer div_r = 30;
    task dut_send(input [7:0] b);
        integer k;
        begin
            dut_tx = 1'b0; #(div_r * CLK_NS);
            for (k = 0; k < 8; k = k + 1) begin dut_tx = b[k]; #(div_r * CLK_NS); end
            dut_tx = 1'b1; #(div_r * CLK_NS * 2);
        end
    endtask
    // UART2_STATUS (0x78): [avail_lo][flags]
    task uart2_status;
        begin cs_lo; spi_byte(8'h78, junk); spi_byte(8'h00, st0); spi_byte(8'h00, st1); cs_hi; end
    endtask
    reg [7:0] r2b [0:15];
    task uart2_read(input integer n);   // UART2_READ (0x77): [len lo][len hi] then n bytes
        integer k;
        begin
            cs_lo; spi_byte(8'h77, junk); spi_byte(n[7:0], junk); spi_byte(n[15:8], junk);
            for (k = 0; k < n; k = k + 1) spi_byte(8'h00, r2b[k]);
            cs_hi;
        end
    endtask

    integer j;
    localparam [8*6-1:0] NMEA = "$GPRMC";
    initial begin
        #(2000);

        // ---- run 1: proxy TX on LA2 @ div 50, UART2 on LA6 @ div 30 ----
        uart_cfg(8'h70, 4'd0, 4'd1, 24'd50, 1'b1);
        uart_cfg(8'h75, 4'd0, 4'd5, 24'd30, 1'b1);
        emu_pin(4'd5, 3'd7);
        #(1000);
        if (la[5] !== 1'b1) begin $display("FAIL tb_top_uart2: UART2 TX not idle high after arm"); errors = errors + 1; end
        if (la[1] !== 1'b1) begin $display("FAIL tb_top_uart2: proxy TX not idle high after arm"); errors = errors + 1; end
        pads_released(1, 5, "after both arms");
        uart_status;
        check(st2 & 8'h7A, 8'h6A, "run1 armed status flags (uart2 armed+empty, proxy armed+empty)");

        rx_on = 1'b1;
        txb[0] = 8'h55;
        for (j = 0; j < 6; j = j + 1) txb[1 + j] = NMEA[8*(5 - j) +: 8];
        uart_write(8'h76, 7);
        txb[0] = 8'hA3; txb[1] = 8'h0F;
        uart_write(8'h72, 2);
        #(30 * CLK_NS * 10 * 8 + 50 * CLK_NS * 10 * 3);
        check(n_a, 7, "run1 UART2 byte count");
        check(n_b, 2, "run1 proxy byte count");
        check(buf_a[0], 8'h55, "run1 UART2 byte 0");
        for (j = 0; j < 6; j = j + 1) check(buf_a[1 + j], NMEA[8*(5 - j) +: 8], "run1 UART2 NMEA byte");
        check(buf_b[0], 8'hA3, "run1 proxy byte 0");
        check(buf_b[1], 8'h0F, "run1 proxy byte 1");
        uart_status;
        check(st2 & 8'h70, 8'h60, "run1 drained: UART2 armed + TX empty, not full");

        // ---- run 2: UART2 re-armed on LA8 @ div 20; LA6 must be released ----
        rx_on = 1'b0;
        uart_cfg(8'h75, 4'd0, 4'd7, 24'd20, 1'b1);
        emu_pin(4'd7, 3'd7);
        ch_a = 7; div_a = 20; n_a = 0;
        #(1000);
        if (la[7] !== 1'b1) begin $display("FAIL tb_top_uart2: re-armed UART2 not idle high on LA8"); errors = errors + 1; end
        pads_released(1, 7, "after the UART2 re-arm");
        rx_on = 1'b1;
        txb[0] = 8'h55; txb[1] = 8'h2A; txb[2] = 8'h0D; txb[3] = 8'h0A;
        uart_write(8'h76, 4);
        #(20 * CLK_NS * 10 * 6);
        check(n_a, 4, "run2 UART2 byte count");
        check(buf_a[0], 8'h55, "run2 byte 0");
        check(buf_a[1], 8'h2A, "run2 byte 1");
        check(buf_a[2], 8'h0D, "run2 byte 2");
        check(buf_a[3], 8'h0A, "run2 byte 3");
        check(n_b, 2, "run2 proxy got nothing extra");

        // ---- disarm UART2 (enable 0); the proxy keeps running ----
        rx_on = 1'b0;
        uart_cfg(8'h75, 4'd0, 4'd7, 24'd20, 1'b0);
        #(1000);
        pads_released(1, -1, "after the UART2 disarm");
        uart_status;
        check(st2 & 8'h48, 8'h08, "disarmed: UART2 armed bit clear, proxy still armed");
        rx_on = 1'b1;
        txb[0] = 8'h3C;
        uart_write(8'h72, 1);
        txb[0] = 8'h99;
        uart_write(8'h76, 1);            // disarmed: must not reach any pin
        #(50 * CLK_NS * 10 * 2);
        check(n_b, 3, "proxy after the UART2 disarm");
        check(buf_b[2], 8'h3C, "proxy byte after the UART2 disarm");
        check(n_a, 4, "disarmed UART2 sent nothing");

        // ---- v54: UART2 RX on LA10 @ div 30 (TX left on LA8) ----
        rx_on = 1'b0;
        dut_tx_en = 1'b1; dut_tx = 1'b1; #(2000);     // the DUT's TX idles high before RX is armed
        uart_cfg(8'h75, 4'd9, 4'd7, 24'd30, 1'b1);
        #(2000);
        uart2_status;                                   // clears the framing errors of the runs above (RX on a low pad)
        dut_send(8'h41); dut_send(8'h54); dut_send(8'h0D);
        uart2_status; check(st0, 8'd3, "UART2 RX: 3 bytes available"); check(st1 & 8'hC1, 8'h00, "UART2 RX: no errors");
        uart2_read(3);
        check(r2b[0], 8'h41, "UART2 RX byte 0"); check(r2b[1], 8'h54, "UART2 RX byte 1"); check(r2b[2], 8'h0D, "UART2 RX byte 2");
        uart2_status; check(st0, 8'd0, "UART2 RX: drained");
        dut_tx = 1'b0; #(30 * CLK_NS * 25); dut_tx = 1'b1; #(30 * CLK_NS * 4);    // a break
        dut_send(8'h7E);
        uart2_status; check(st1 & 8'h80, 8'h80, "UART2 RX: break = framing error"); check(st0, 8'd1, "UART2 RX: the byte after the break");
        uart2_status; check(st1 & 8'h80, 8'h00, "UART2 RX: framing error cleared by the status read");
        uart2_read(1); check(r2b[0], 8'h7E, "UART2 RX byte after the break");
        dut_tx_en = 1'b0;

        if (errors == 0) $display("PASS tb_top_uart2: UART2 and the proxy run side by side, re-arm moves pin and rate, disarm releases the pin");
        else             $display("FAIL tb_top_uart2: %0d error(s)", errors);
        $finish;
    end

    initial begin #(60_000_000); $display("FAIL tb_top_uart2: timeout"); $finish; end
endmodule
