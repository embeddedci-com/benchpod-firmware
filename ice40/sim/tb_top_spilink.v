// ============================================================================
// tb_top_spilink.v — the STM32 -> iCE40 SPI command link at speed (v52).
//
// The link is mode 0 into spi_slave, which samples SCK/CSn/MOSI on the 24 MHz clk.  Until v51
// it took MOSI 2-3 clk after it saw SCK rise and moved MISO 3-4 clk after it saw SCK fall, so the
// STM32 (which samples MISO on the rising edge) needed a half period of ~170 ns: 1.95 MHz (SPI1
// kernel / 128) was clean on hardware and 3.9 MHz (/64) failed ~95% of PINGs.
//
// This bench models the STM32 master: SCK at a given half period, MOSI changed on the falling
// edge, MISO sampled on the rising edge with a setup time, an optional idle gap between bytes
// (the H5 SPI's MIDI, master inter-data idleness), and board/pad delays on SCK, MOSI and MISO.
// The master's clock has an arbitrary, drifting phase against clk48.  Each rate runs PING,
// VERSION, STATUS and, in the emulation image (v54: the only one with I2C targets), a 256-byte
// I2C register-image write + read-back (every byte value, a streaming reply), so every reply
// path the firmware uses goes through it.
//
//   * /128 (1.95 MHz), no gap: what every firmware so far sends: must pass.
//   * /64  (3.9 MHz),  no gap: must pass from v52 (v51 fails it here as on hardware).
//   * a 100 ns half period (5 MHz), no gap: the margin guard.  v52 fails below ~95 ns (the reply
//     turnaround); a change that eats /64's ~30% margin fails here first.
// (/32, 7.8 MHz, is out of reach: the two-flop SCK synchroniser puts ~140 ns between a rise and
// the next MISO bit; see spi_slave.v.)
// Runs on all three images (make topspilinktest).
// ============================================================================
`timescale 1ns/1ps
module tb_top_spilink;
    reg clk48 = 1'b0;
    always #10.4166 clk48 = ~clk48;

    // master-side levels and the delayed copies the FPGA / the master see
    reg  m_sck = 1'b0, m_mosi = 1'b0, m_csn = 1'b1;
    wire miso_pad;
    localparam real T_SCK = 6.0, T_MOSI = 7.5, T_MISO = 9.0;   // ns: board + pads
    wire sck, mosi, csn, miso_m;
    assign #(T_SCK)  sck    = m_sck;
    assign #(T_MOSI) mosi   = m_mosi;
    assign #(T_SCK)  csn    = m_csn;
    assign #(T_MISO) miso_m = miso_pad;
    localparam real T_SU = 5.0;                                   // STM32 MISO setup

    reg  bus_own = 1'b1, adc_sdo = 1'b1;
    wire busy, adc_cnvst, adc_sclk, adc_sdi, dac_sync, dac_sclk, dac_din, led_g, led_b, led_r;
    tri  psram_sclk, psram_cs, psram_io0, psram_io1, psram_io2, psram_io3;
    wire [13:0] la = 14'h0000;
    top dut (
        .clk48(clk48), .sck(sck), .mosi(mosi), .miso(miso_pad), .csn(csn), .busy(busy), .bus_own(bus_own),
        .adc_cnvst(adc_cnvst), .adc_sclk(adc_sclk), .adc_sdi(adc_sdi), .adc_sdo(adc_sdo),
        .dac_sync(dac_sync), .dac_sclk(dac_sclk), .dac_din(dac_din),
        .psram_sclk(psram_sclk), .psram_cs(psram_cs), .psram_io0(psram_io0), .psram_io1(psram_io1),
        .psram_io2(psram_io2), .psram_io3(psram_io3), .la(la), .led_g(led_g), .led_b(led_b), .led_r(led_r));

    real    half = 256.0;          // SCK half period, ns
    integer gap  = 0;              // idle SCK periods between bytes (MIDI)
    real    lead = 300.0;          // CS fall to the first SCK edge, ns

    // one byte, MSB first, mode 0: MOSI set half a period before the rise, MISO sampled T_SU
    // before the rise (the H5 samples on the edge; the setup is its requirement)
    task xfer(input [7:0] tx, output [7:0] rx);
        integer b; reg s; begin
            for (b = 7; b >= 0; b = b - 1) begin
                m_mosi = tx[b];
                #(half - T_SU); s = miso_m; #(T_SU);
                m_sck = 1'b1; rx[b] = s;
                #(half); m_sck = 1'b0;
            end
            if (gap) #(2.0 * half * gap);
        end
    endtask
    task cs_low;  begin m_csn = 1'b0; #(lead); end endtask
    task cs_high; begin #(half); m_csn = 1'b1; #(8.0 * half + 400.0); end endtask

    integer errors = 0, fails_rate = 0;
    reg [7:0] r, j;
    integer i, k;
    task check(input [8*24-1:0] what, input [7:0] got, input [7:0] want);
        begin
            if (got !== want) begin
                if (fails_rate < 8) $display("FAIL: %0s = %02h, want %02h (half %0.1f ns, gap %0d)", what, got, want, half, gap);
                fails_rate = fails_rate + 1;
            end
        end
    endtask

    task run_rate(input real h, input integer g, input integer rounds);
        integer n; begin
            half = h; gap = g; fails_rate = 0;
            for (n = 0; n < rounds; n = n + 1) begin
                // a drifting phase against clk48: the master's edges sweep the 24 MHz clk
                #(3.7 * n + 1.3);
                cs_low; xfer(8'h01, j); xfer(8'h00, r); cs_high; check("PING", r, 8'hA5);
                cs_low; xfer(8'h02, j); xfer(8'h00, r); cs_high; check("VERSION", r, dut.engines_i.GATEWARE_VERSION);
                cs_low; xfer(8'h03, j); xfer(8'h00, r); cs_high; check("STATUS bit 7", r & 8'h80, 8'h00);
            end
`ifdef USE_EMULATION
            // 256-byte register image of I2C target 2 (v54: the emulation image's I2C_* commands
            // take a target byte): write every value, read it back streamed
            cs_low; xfer(8'h62, j); xfer(8'h02, j); xfer(8'h00, j); xfer(8'h00, j); xfer(8'h01, j);
            for (k = 0; k < 256; k = k + 1) xfer(k[7:0] ^ 8'h5A, j);
            cs_high;
            // read as the firmware does (fpga_i2c_read_regs): ask for len + 1, drop the lead byte
            cs_low; xfer(8'h63, j); xfer(8'h02, j); xfer(8'h00, j); xfer(8'h01, j); xfer(8'h01, j);
            xfer(8'h00, j);
            for (k = 0; k < 256; k = k + 1) begin xfer(8'h00, r); check("REG read-back", r, k[7:0] ^ 8'h5A); end
            cs_high;
`endif
            if (fails_rate == 0)
                $display("     %0.2f MHz, %0d-SCK gap: %0d PING/VERSION/STATUS rounds + 256-byte write/read-back clean",
                         500.0 / half, g, rounds);
            errors = errors + fails_rate;
        end
    endtask

    initial begin
        #3000;
        run_rate(256.0, 0, 40);     // /128: 1.95 MHz (every firmware so far)
        run_rate(128.0, 0, 40);     // /64:  3.9 MHz
        run_rate(100.0, 0, 40);     // 5 MHz: the margin guard
        if (errors == 0) $display("PASS tb_top_spilink: SPI link clean at 1.95, 3.9 and 5 MHz with no inter-byte gap");
        else             $display("FAIL tb_top_spilink: %0d error(s)", errors);
        $finish;
    end
    initial begin #60000000 $display("FAIL tb_top_spilink: timeout"); $finish; end
endmodule
