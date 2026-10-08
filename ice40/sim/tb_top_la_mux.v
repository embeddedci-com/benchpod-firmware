// ============================================================================
// tb_top_la_mux.v — la_bank's driver priority (swd/spi > stepper > i2c > uart > static) end to
// end through top_v2: every driver is armed over the real SPI link (GPIO_SET, UART_CONFIG,
// I2C_CONFIG, GPIO_STEP, SPI_ARM, SWD_DISARM, I2C_DISABLE, UART_DISABLE) onto the SAME pad, and
// the pad level, the bank's output enable and GPIO_GET are checked after each step, as each
// driver takes the pad over and as each lets go again (falling back to the next one down).
//
// The per-module form of this was proven formally when the mux became one-hot masks; this bench
// covers what that could not: the channel and arm/disarm plumbing from cmd_dispatch through
// engine_block into la_bank, and the `ch < N` guard for a channel that does not exist.
// Second run with other parameters: the same climb on another pad, with UART pointed at a
// channel past the bank (15), which must drive nothing.
//
// Run:  make topmuxtest
// ============================================================================
`timescale 1ns/1ps
module tb_top_la_mux;
    reg clk48 = 1'b0;
    always #10 clk48 = ~clk48;          // 50 MHz sim: clk = 25 MHz

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

    wire [13:0] oe = dut.engines_i.la_bank_i.eff_oe;

    localparam SPI_HALF = 250;           // 2 MHz link, like the pod
    integer errors = 0;
    reg [7:0] junk;

    task spi_byte(input [7:0] tx, output [7:0] rx);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b]; #(SPI_HALF);
                sck = 1'b1; rx[b] = miso; #(SPI_HALF);
                sck = 1'b0;
            end
        end
    endtask
    task cs_lo; begin csn = 1'b0; #(SPI_HALF); end endtask
    task cs_hi; begin #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF); end endtask
    task cmd0(input [7:0] op); begin cs_lo; spi_byte(op, junk); cs_hi; end endtask

    task gpio_set(input [3:0] ch, input [1:0] mode);   // 0 low, 1 high, 2 high-Z
        begin cs_lo; spi_byte(8'h40, junk); spi_byte({4'h0, ch}, junk); spi_byte({6'h0, mode}, junk); cs_hi; end
    endtask
    task gpio_step(input [3:0] ch, input [15:0] steps, input [15:0] half_us_m1);
        begin
            cs_lo; spi_byte(8'h41, junk); spi_byte({4'h0, ch}, junk);
            spi_byte(steps[7:0], junk); spi_byte(steps[15:8], junk);
            spi_byte(half_us_m1[7:0], junk); spi_byte(half_us_m1[15:8], junk); cs_hi;
        end
    endtask
    task uart_config(input [3:0] rx_ch, input [3:0] tx_ch);   // 24 clk per bit, flags: enable
        begin
            cs_lo; spi_byte(8'h70, junk); spi_byte({4'h0, rx_ch}, junk); spi_byte({4'h0, tx_ch}, junk);
            spi_byte(8'd24, junk); spi_byte(8'd0, junk); spi_byte(8'd0, junk); spi_byte(8'h01, junk); cs_hi;
        end
    endtask
    task i2c_config(input [3:0] sda_ch, input [3:0] scl_ch);   // addr 0x76, flags: enable
        begin
            cs_lo; spi_byte(8'h60, junk); spi_byte(8'h76, junk);
            spi_byte({4'h0, sda_ch}, junk); spi_byte({4'h0, scl_ch}, junk);
            spi_byte(8'h01, junk); spi_byte(8'h00, junk); spi_byte(8'h00, junk);
            spi_byte(8'h00, junk); spi_byte(8'h00, junk); spi_byte(8'h00, junk); cs_hi;
        end
    endtask
    task spi_arm(input [3:0] s, input [3:0] mo, input [3:0] mi, input [3:0] c, input cpol);
        begin
            cs_lo; spi_byte(8'h55, junk);
            spi_byte({4'h0, s}, junk);  spi_byte({4'h0, mo}, junk);
            spi_byte({4'h0, mi}, junk); spi_byte({4'h0, c}, junk);
            spi_byte(8'd4, junk);       spi_byte({7'h0, cpol}, junk); cs_hi;
        end
    endtask
    task gpio_get(output [15:0] lv);
        begin cs_lo; spi_byte(8'h43, junk); spi_byte(8'h00, lv[7:0]); spi_byte(8'h00, lv[15:8]); cs_hi; end
    endtask

    // pad c: level and output enable; every other pad not in `others_ok` released
    task expect_pad(input [8*28-1:0] what, input integer c, input lvl, input en, input [13:0] others_ok);
        reg [13:0] stray;
        begin
            #400;
            if (la[c] !== lvl) begin $display("FAIL %0s: LA%0d = %b, want %b", what, c + 1, la[c], lvl); errors = errors + 1; end
            if (oe[c] !== en)  begin $display("FAIL %0s: LA%0d oe = %b, want %b", what, c + 1, oe[c], en); errors = errors + 1; end
            stray = oe & ~others_ok & ~(14'd1 << c);
            if (stray !== 14'd0) begin $display("FAIL %0s: other pads driven (oe %014b)", what, stray); errors = errors + 1; end
        end
    endtask

    // count rising edges on pad c over `ns`
    integer edges;
    task count_edges(input integer c, input integer ns);
        reg prev; integer t;
        begin
            edges = 0; prev = la[c];
            for (t = 0; t < ns; t = t + 20) begin
                #20; if (la[c] === 1'b1 && prev !== 1'b1) edges = edges + 1;
                prev = la[c];
            end
        end
    endtask

    task wait_step_idle;
        integer k; begin
            k = 0;
            while (dut.engines_i.step_busy && k < 20000) begin #100; k = k + 1; end
            if (dut.engines_i.step_busy) begin $display("FAIL: step train never ended"); errors = errors + 1; end
        end
    endtask

    // The whole climb on pad c: static -> uart -> i2c -> step -> spi (with a step under it) -> back
    // down. mo/mi/cs are the SPI engine's other pads (cs has a static low under it).
    task climb(input [8*8-1:0] run, input integer c, input integer urx, input integer scl,
               input integer mo, input integer mi, input integer cs);
        reg [15:0] lv;
        reg [13:0] spi_pads;
        begin
            spi_pads = (14'd1 << mo) | (14'd1 << cs);
            expect_pad("idle", c, 1'b0, 1'b0, 14'd0);

            gpio_set(c, 2'd1);       expect_pad("static high", c, 1'b1, 1'b1, 14'd0);
            gpio_set(c, 2'd0);       expect_pad("static low", c, 1'b0, 1'b1, 14'd0);

            uart_config(urx, c);     expect_pad("uart over static", c, 1'b1, 1'b1, 14'd0);
            if (!dut.engines_i.uart_armed) begin $display("FAIL %0s: uart not armed", run); errors = errors + 1; end
            gpio_get(lv);
            if (lv !== (16'd1 << c)) begin $display("FAIL %0s: GPIO_GET %04h, want %04h", run, lv, 16'd1 << c); errors = errors + 1; end

            i2c_config(c, scl);      expect_pad("i2c over uart (released)", c, 1'b0, 1'b0, 14'd0);
            if (!dut.engines_i.i2c_armed) begin $display("FAIL %0s: i2c not armed", run); errors = errors + 1; end

            gpio_step(c, 16'd4, 16'd0);
            count_edges(c, 6000);
            if (edges < 2) begin $display("FAIL %0s: step over i2c: %0d rising edges while stepping", run, edges); errors = errors + 1; end
            wait_step_idle;
            expect_pad("i2c again after the step", c, 1'b0, 1'b0, 14'd0);

            gpio_set(cs, 2'd0);      expect_pad("static low on cs pad", cs, 1'b0, 1'b1, 14'd0);
            spi_arm(c, mo, mi, cs, 1'b1);   // CPOL 1: SCK idles high, CS released (driven high)
            expect_pad("spi sck over i2c", c, 1'b1, 1'b1, spi_pads);
            expect_pad("spi cs over static", cs, 1'b1, 1'b1, (14'd1 << c) | (14'd1 << mo));
            if (oe[mi] !== 1'b0) begin $display("FAIL %0s: MISO pad driven", run); errors = errors + 1; end

            gpio_step(c, 16'd4, 16'd0);     // under the SPI engine: must not reach the pad
            count_edges(c, 6000);
            if (edges != 0) begin $display("FAIL %0s: step under spi: %0d edges on SCK", run, edges); errors = errors + 1; end
            if (la[c] !== 1'b1) begin $display("FAIL %0s: SCK left idle during the step", run); errors = errors + 1; end
            wait_step_idle;

            cmd0(8'h53);                    // SWD_DISARM: back to i2c on c, static low on cs
            expect_pad("disarm: i2c again", c, 1'b0, 1'b0, (14'd1 << cs));
            expect_pad("disarm: static on cs", cs, 1'b0, 1'b1, 14'd0);

            cmd0(8'h61);                    // I2C_DISABLE: uart again
            expect_pad("i2c off: uart again", c, 1'b1, 1'b1, (14'd1 << cs));

            cmd0(8'h71);                    // UART_DISABLE: the static low underneath
            expect_pad("uart off: static low", c, 1'b0, 1'b1, (14'd1 << cs));

            gpio_set(c, 2'd2); gpio_set(cs, 2'd2);
            expect_pad("all released", c, 1'b0, 1'b0, 14'd0);
            $display("  [%0s] LA%0d: static, uart, i2c, step, spi (step under it) and back down", run, c + 1);
        end
    endtask

    initial begin
        #5000;
        climb("run 1", 3, 9, 10, 6, 7, 8);
        climb("run 2", 12, 0, 1, 2, 4, 13);

        // a channel past the bank drives nothing (la_bank's ch < N guard)
        gpio_set(5, 2'd1);
        uart_config(4'd11, 4'd15);
        #400;
        if (!dut.engines_i.uart_armed) begin $display("FAIL: uart to LA16 not armed"); errors = errors + 1; end
        if (oe !== (14'd1 << 5) || la[5] !== 1'b1) begin
            $display("FAIL: uart on channel 15 drove oe %014b (only LA6's static high expected)", oe); errors = errors + 1; end
        cmd0(8'h71); gpio_set(5, 2'd2);
        $display("  [guard] UART TX on channel 15: no pad driven");

        if (errors == 0) $display("PASS tb_top_la_mux");
        else             $display("FAIL tb_top_la_mux: %0d error(s)", errors);
        $finish;
    end

    initial begin #20000000; $display("FAIL tb_top_la_mux: timeout"); $finish; end
endmodule
