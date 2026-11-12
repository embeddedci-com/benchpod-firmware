// ============================================================================
// tb_top_emu.v: the emulated sensors (v54) through the whole emulation image (top_v2,
// -DUSE_EMULATION): every command goes over the real SPI link (spi_slave, cmd_dispatch) and the
// DUT side is bus models on the LA pads (SB_IO, la_bank, the pin map).
//
// Checked:
//   * I2C_CONFIG / I2C_LOAD_REGS / I2C_LOAD_ATTRS / I2C_TIMES / EMU_PINMAP: two I2C targets on one
//     bus (LA2 SDA, LA3 SCL, pulled up) answer an I2C master at 400 kHz; I2C_READ_REGS and
//     I2C_STATUS see the DUT's writes (count, last address and value); a TRIG write makes the BUSY
//     register read busy until the I2C_TIMES time has passed
//   * EMU_FIFO_WRITE / EMU_FIFO_STATUS: a FIFO register pops what the STM32 pushed (the SPRAM
//     FIFO), the level counts down, an empty read sets underflow, the status read clears it
//   * I2C_DRDY + EMU_PINMAP code 2: the data-ready pin on LA5 follows the firmware command
//   * the SPI target (SPIT_CONFIG, target 4 of I2C_LOAD_REGS/READ_REGS/STATUS) in modes 0 and 3
//     against an SPI master model on LA9..LA12, reads and writes, the address mask, read and
//     increment bits, a dummy byte, mute
//
// Run:  make topemutest
// ============================================================================
`timescale 1ns/1ps
module tb_top_emu;
    reg clk48 = 1'b0;
    always #10.4167 clk48 = ~clk48;     // 48 MHz: clk = 24 MHz

    reg  sck = 1'b0, mosi = 1'b0, csn = 1'b1;
    reg  bus_own = 1'b1;
    reg  adc_sdo = 1'b1;
    wire miso, busy;
    wire adc_cnvst, adc_sclk, adc_sdi, dac_sync, dac_sclk, dac_din;
    wire led_g, led_b, led_r;
    wire psram_sclk, psram_cs, psram_io0, psram_io1, psram_io2, psram_io3;
    wire [13:0] la;

    // I2C bus on LA2 (SDA, la[1]) / LA3 (SCL, la[2]); pull-ups; the master pulls low
    localparam SDA = 1, SCL = 2, DRDY = 4;
    reg m_sda = 1'b1, m_scl = 1'b1;
    pullup (la[SDA]);  pullup (la[SCL]);
    assign la[SDA] = m_sda ? 1'bz : 1'b0;
    assign la[SCL] = m_scl ? 1'bz : 1'b0;
    // SPI master on LA9 CS (la[8]), LA10 SCK, LA11 MOSI, LA12 MISO (from the target)
    localparam SCS = 8, SSCK = 9, SMOSI = 10, SMISO = 11;
    reg s_cs = 1'b1, s_sck = 1'b0, s_mosi = 1'b0;
    assign la[SCS] = s_cs;  assign la[SSCK] = s_sck;  assign la[SMOSI] = s_mosi;
    pulldown (la[SMISO]);
    pulldown (la[DRDY]);

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
    task check(input [8*64-1:0] what, input [31:0] got, input [31:0] want);
        if (got !== want) begin
            $display("FAIL tb_top_emu: %0s: got %0h want %0h", what, got, want); errors = errors + 1;
        end
    endtask

    // ---- the STM32 link ----
    localparam SPI_HALF = 128;           // ~3.9 MHz, the firmware's rate on gateware >= 52
    reg [7:0] junk;
    task spi_byte(input [7:0] tx, output [7:0] rx);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b]; #(SPI_HALF); sck = 1'b1; rx[b] = miso; #(SPI_HALF); sck = 1'b0;
            end
        end
    endtask
    task cs_lo; begin csn = 1'b0; #(SPI_HALF); end endtask
    task cs_hi; begin #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF); end endtask
    task tx(input [7:0] b); spi_byte(b, junk); endtask

    task i2c_config(input [2:0] t, input [6:0] a, input [7:0] flags, input [7:0] mask, input [7:0] drdy);
        begin cs_lo; tx(8'h60); tx(t); tx(a); tx(SDA); tx(SCL); tx(flags); tx(8'h00); tx(mask); tx(drdy); cs_hi; end
    endtask
    task load(input [7:0] op, input [2:0] t, input [7:0] a, input integer n, input [63:0] d);
        integer i;
        begin
            cs_lo; tx(op); tx(t); tx(a); tx(n[7:0]); tx(n[15:8]);
            for (i = 0; i < n; i = i + 1) tx(d[8*i +: 8]);
            cs_hi;
        end
    endtask
    // READ_REGS/ATTRS send r[start] twice (like SWD_READ): ask for n + 1, drop the first
    task readback(input [7:0] op, input [2:0] t, input [7:0] a, input integer n, output [63:0] d);
        integer i; reg [7:0] b;
        begin
            d = 0; cs_lo; tx(op); tx(t); tx(a); tx(n[7:0] + 8'd1); tx(8'd0);
            spi_byte(8'h00, b);
            for (i = 0; i < n; i = i + 1) begin spi_byte(8'h00, b); d[8*i +: 8] = b; end
            cs_hi;
        end
    endtask
    task status(input [2:0] t, output [15:0] st);   // {flags, wr_count}
        reg [7:0] b0, b1, b2;
        begin
            cs_lo; tx(8'h64); tx(t); spi_byte(0, b0); spi_byte(0, b1); spi_byte(0, b2); cs_hi;
            st = {b1, b2};
        end
    endtask
    // I2C_READ_LOG: n entries {addr, value} from entry i (2n + 1 bytes, the first dropped)
    task read_log(input [2:0] t, input [4:0] i, input integer n, output [63:0] d);
        integer k; reg [7:0] b;
        begin
            d = 0; cs_lo; tx(8'h79); tx(t); tx({2'b00, i, 1'b0}); tx(2*n + 1); tx(8'd0);
            spi_byte(0, b);
            for (k = 0; k < 2*n; k = k + 1) begin spi_byte(0, b); d[8*k +: 8] = b; end
            cs_hi;
        end
    endtask
    // ---- the SPI master (the DUT) on LA9..LA12 ----
    integer SH = 300;                     // half period: ~1.7 MHz
    reg cpol = 0, cpha = 0;
    reg [7:0] sbuf [0:7];
    task s_xfer(input integer n);         // sbuf out, the MISO bytes back in sbuf
        integer k, b; reg [7:0] o, r;
        begin
            s_sck = cpol; #(SH); s_cs = 0; #(SH);
            for (k = 0; k < n; k = k + 1) begin
                o = sbuf[k];
                for (b = 7; b >= 0; b = b - 1) begin
                    if (!cpha) begin s_mosi = o[b]; #(SH); s_sck = ~cpol; r[b] = la[SMISO]; #(SH); s_sck = cpol; end
                    else begin s_sck = ~cpol; s_mosi = o[b]; #(SH); s_sck = cpol; r[b] = la[SMISO]; #(SH); end
                end
                sbuf[k] = r;
            end
            #(SH); s_cs = 1; #(4*SH);
        end
    endtask
    task spit_config(input [7:0] flags, input [7:0] rb, input [7:0] ib, input [7:0] mask);
        begin cs_lo; tx(8'h7C); tx(SCS); tx(SSCK); tx(SMOSI); tx(flags); tx(rb); tx(ib); tx(mask); cs_hi; end
    endtask
    // EMU_PINMAP: the 14 codes in pm (3 bits each, wire 0 lowest)
    reg [41:0] pm = 42'd0;
    task pinmap;
        integer j;
        begin
            cs_lo; tx(8'h6E);
            for (j = 0; j < 7; j = j + 1) tx({1'b0, pm[3*(2*j+1) +: 3], 1'b0, pm[3*(2*j) +: 3]});
            cs_hi;
        end
    endtask
    task two(input [7:0] op, output [15:0] v);   // a 2-byte status opcode
        reg [7:0] lo, hi;
        begin cs_lo; tx(op); spi_byte(0, lo); spi_byte(0, hi); cs_hi; v = {hi, lo}; end
    endtask

    // ---- the I2C master (400 kHz) ----
    localparam TLOW = 1300, THIGH = 1200;
    task scl_up; begin m_scl = 1'b1; while (la[SCL] !== 1'b1) #10; end endtask
    task i2c_start; begin m_sda = 1; #(TLOW); scl_up; #(THIGH); m_sda = 0; #(THIGH); m_scl = 0; #(TLOW/2); end endtask
    task i2c_rstart; begin m_sda = 1; #(TLOW/2); scl_up; #(THIGH); m_sda = 0; #(THIGH); m_scl = 0; #(TLOW/2); end endtask
    task i2c_stop; begin m_sda = 0; #(TLOW/2); scl_up; #(THIGH); m_sda = 1; #(TLOW); end endtask
    task i2c_wbyte(input [7:0] b, output ack);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin m_sda = b[i]; #(TLOW/2); scl_up; #(THIGH); m_scl = 0; #(TLOW/2); end
            m_sda = 1; #(TLOW/2); scl_up; ack = ~la[SDA]; #(THIGH); m_scl = 0; #(TLOW/2);
        end
    endtask
    task i2c_rbyte(input ack, output [7:0] b);
        integer i;
        begin
            m_sda = 1;
            for (i = 7; i >= 0; i = i - 1) begin #(TLOW/2); scl_up; b[i] = la[SDA]; #(THIGH); m_scl = 0; #(TLOW/2); end
            m_sda = ~ack; #(TLOW/2); scl_up; #(THIGH); m_scl = 0; #(TLOW/2); m_sda = 1;
        end
    endtask
    reg ack;
    task i2c_wr(input [6:0] a, input [7:0] ptr, input integer n, input [63:0] d, output ok);
        integer i;
        begin
            ok = 1; i2c_start; i2c_wbyte({a, 1'b0}, ack); ok = ok & ack; i2c_wbyte(ptr, ack); ok = ok & ack;
            for (i = 0; i < n; i = i + 1) begin i2c_wbyte(d[8*i +: 8], ack); ok = ok & ack; end
            i2c_stop;
        end
    endtask
    task i2c_rd(input [6:0] a, input [7:0] ptr, input integer n, output [63:0] d, output ok);
        integer i; reg [7:0] b;
        begin
            ok = 1; d = 0;
            i2c_start; i2c_wbyte({a, 1'b0}, ack); ok = ok & ack; i2c_wbyte(ptr, ack); ok = ok & ack;
            i2c_rstart; i2c_wbyte({a, 1'b1}, ack); ok = ok & ack;
            if (ok) for (i = 0; i < n; i = i + 1) begin i2c_rbyte(i != n - 1, b); d[8*i +: 8] = b; end
            i2c_stop;
        end
    endtask

    reg ok;  reg [63:0] d;  reg [15:0] st;  reg [15:0] v;  integer m;
    realtime t0;
    initial begin
        #(5000);
        // ---- I2C: two targets on one bus ----
        // t0 0x76: flag busy, mask 0x08; t1 0x68: data ready level, active high
        i2c_config(0, 7'h76, 8'h01, 8'h08, 8'h00);
        i2c_config(1, 7'h68, 8'h01, 8'h00, 8'h06);
        pm[3*SDA +: 3] = 3'd1;  pm[3*SCL +: 3] = 3'd1;  pm[3*DRDY +: 3] = 3'd3;   // DRDY = target 1
        pinmap;
        load(8'h62, 0, 8'hD0, 1, 8'h58);
        load(8'h62, 0, 8'hF3, 1, 8'h00);
        load(8'h6A, 0, 8'hF3, 1, 8'h04);              // F3: BUSY
        load(8'h6A, 0, 8'hF4, 1, 8'h01);              // F4: TRIG, time 0
        cs_lo; tx(8'h6C); tx(0); tx(0); tx(8'd200); tx(8'd0); cs_hi;   // I2C_TIMES t0 #0 = 200 us
        load(8'h62, 1, 8'h75, 1, 8'h68);
        load(8'h6A, 1, 8'h74, 1, 8'h08);              // t1 0x74: FIFO
        readback(8'h63, 0, 8'hD0, 1, d); check("READ_REGS t0 D0", d[7:0], 8'h58);
        readback(8'h6B, 0, 8'hF3, 2, d); check("READ_ATTRS t0 F3..F4", d[15:0], 16'h0104);

        i2c_rd(7'h76, 8'hD0, 1, d, ok); check("t0 ack", ok, 1); check("t0 D0", d[7:0], 8'h58);
        i2c_rd(7'h68, 8'h75, 1, d, ok); check("t1 ack", ok, 1); check("t1 75", d[7:0], 8'h68);
        i2c_wr(7'h76, 8'hF4, 1, 8'h27, ok); check("t0 write ack", ok, 1);
        t0 = $realtime;
        i2c_rd(7'h76, 8'hF3, 1, d, ok); check("t0 F3 busy", d[7:0], 8'h08);
        readback(8'h63, 0, 8'hF4, 1, d); check("READ_REGS sees the DUT write", d[7:0], 8'h27);
        status(0, st);
        check("STATUS t0 flags enabled+busy", st[15:14], 2'b11);
        check("STATUS t0 write count", st[7:0], 8'd1);
        read_log(0, 0, 1, d); check("write log t0 entry 0", d[15:0], 16'h27F4);
        i2c_wr(7'h76, 8'hE0, 3, 24'h0302B6, ok);           // three writes between two firmware reads
        read_log(0, 1, 3, d); check("write log t0 entries 1..3", d[47:0], 48'h03E2_02E1_B6E0);
        while ($realtime - t0 < 220000) #1000;
        i2c_rd(7'h76, 8'hF3, 1, d, ok); check("t0 F3 done", d[7:0], 8'h00);

        // ---- FIFO ----
        cs_lo; tx(8'h7A); tx(8'd3); tx(8'd0); tx(8'hC0); tx(8'hC1); tx(8'hC2); cs_hi;
        two(8'h7B, v); check("FIFO level 3 (head + 2)", v, 16'h0005);
        i2c_rd(7'h68, 8'h74, 4, d, ok); check("FIFO register pops", d[31:0], 32'h00C2C1C0);
        two(8'h7B, v); check("FIFO empty, underflow", v, 16'h8000);
        two(8'h7B, v); check("FIFO underflow cleared by the read", v, 16'h0000);

        // ---- data ready ----
        #2000; check("DRDY idle low", la[DRDY], 0);
        cs_lo; tx(8'h6D); tx(1); tx(1); cs_hi; #2000;
        check("DRDY asserted", la[DRDY], 1);
        cs_lo; tx(8'h6D); tx(1); tx(0); cs_hi; #2000;
        check("DRDY deasserted", la[DRDY], 0);

        // ---- disable ----
        cs_lo; tx(8'h61); tx(0); cs_hi;
        i2c_rd(7'h76, 8'hD0, 1, d, ok); check("t0 disabled", ok, 0);
        i2c_rd(7'h68, 8'h75, 1, d, ok); check("t1 still answers", d[7:0], 8'h68);

        // ---- the SPI target (BMP280 layout: bit7 = 1 read, mask 0x7F) ----
        pm[3*SMISO +: 3] = 3'd6; pinmap;
        load(8'h62, 4, 8'h50, 4, 32'h44332211);
        load(8'h62, 4, 8'h20, 1, 8'h99);
        for (m = 0; m < 4; m = m + 1) begin
            cpol = m[1]; cpha = m[0];
            spit_config({5'b0, m[1:0], 1'b1}, 8'h87, 8'h00, 8'h7F);
            sbuf[0] = 8'hD0; sbuf[1] = 0; sbuf[2] = 0; sbuf[3] = 0; sbuf[4] = 0;
            s_xfer(5); check("SPI read 0x50..0x53", {sbuf[4], sbuf[3], sbuf[2], sbuf[1]}, 32'h44332211);
            check("SPI MISO 0 during the address byte", sbuf[0], 8'h00);
            sbuf[0] = 8'h20; sbuf[1] = 8'h5A + m[7:0]; s_xfer(2);
            readback(8'h63, 4, 8'h20, 1, d); check("SPI write stored", d[7:0], 8'h5A + m[7:0]);
        end
        status(4, st); check("SPI status enabled, CS high, 4 writes", st, 16'h8004);
        read_log(4, 0, 4, d); check("SPI write log", d[63:0], 64'h5D20_5C20_5B20_5A20);
        // LIS3DH layout: bit7 read, bit6 increment, mask 0x3F; one dummy byte
        cpol = 0; cpha = 0;
        load(8'h62, 4, 8'h10, 2, 16'h2211);
        spit_config(8'h11, 8'h87, 8'hC6, 8'h3F);
        sbuf[0] = 8'h80 | 8'h10; sbuf[1] = 0; sbuf[2] = 0; sbuf[3] = 0; s_xfer(4);
        check("SPI no increment + dummy", {sbuf[3], sbuf[2]}, 16'h1111);
        sbuf[0] = 8'hC0 | 8'h10; s_xfer(4);
        check("SPI increment + dummy", {sbuf[3], sbuf[2]}, 16'h2211);
        spit_config(8'h19, 8'h87, 8'hC6, 8'h3F);              // mute
        sbuf[0] = 8'hC0 | 8'h10; s_xfer(4);
        check("SPI muted: MISO released", {sbuf[3], sbuf[2]}, 16'h0000);

        if (errors == 0) $display("PASS tb_top_emu: I2C targets, write log, FIFO, data ready, SPI target modes 0-3 through the SPI link and the pads");
        else             $display("FAIL tb_top_emu: %0d error(s)", errors);
        $finish;
    end
    initial begin #(20_000_000); $display("FAIL tb_top_emu: timeout"); $finish; end
endmodule
