// ============================================================================
// spi_slave.v — SPI slave (mode 0) for the iCE40 signal-engine FPGA.
//
// Receives bytes from the RP2350 on MOSI, presents them as rx_byte + rx_valid
// strobe to the cmd_dispatch FSM.  Shifts tx_byte out on MISO during the
// 8 SCK cycles after each received byte.  CSn high resets the bit counter
// and forces MISO high-Z (handled at top level).
//
// All FF-driven on `clk` (the 24 MHz logic clock — SYS_CLK_MHZ).  SCK and CSn are
// double-flopped for clock-domain crossing, so the maximum reliable SPI clock is
// ~clk/4 = 6 MHz.  The STM32 currently clocks the link well below that (a large
// baud prescaler); do NOT raise the host SPI rate past ~6 MHz without re-checking
// this synchroniser depth.  (Historical note: this header used to say 48 MHz /
// 8 MHz, from before the single-clock collapse dropped the design to 24 MHz.)
// ============================================================================

module spi_slave (
    input  wire        clk,        // 24 MHz system clock (SYS_CLK_MHZ)
    input  wire        rst,        // synchronous reset, active high

    // SPI bus (mode 0)
    input  wire        sck,
    input  wire        mosi,
    output reg         miso,
    input  wire        csn,

    // Parallel byte interface to cmd_dispatch
    output reg  [7:0]  rx_byte,    // last received byte
    output reg         rx_valid,   // 1-cycle strobe when rx_byte is fresh
    input  wire [7:0]  tx_byte,    // byte to shift out on next 8 SCK cycles
    output wire        cs_active   // mirrors !csn (synchronised)
);

    // ---- SCK and CSn synchronisation (CDC) ----
    reg sck_s0, sck_s1, sck_s2;
    reg csn_s0, csn_s1;

    always @(posedge clk) begin
        if (rst) begin
            sck_s0 <= 1'b0; sck_s1 <= 1'b0; sck_s2 <= 1'b0;
            csn_s0 <= 1'b1; csn_s1 <= 1'b1;
        end else begin
            sck_s0 <= sck;  sck_s1 <= sck_s0;  sck_s2 <= sck_s1;
            csn_s0 <= csn;  csn_s1 <= csn_s0;
        end
    end

    wire sck_rise = (sck_s1 & ~sck_s2);
    wire sck_fall = (~sck_s1 & sck_s2);
    assign cs_active = ~csn_s1;

    // ---- Shift registers ----
    reg [7:0] rx_sr;
    reg [7:0] tx_sr;
    reg [3:0] bit_cnt;  // counts 0..7 for each byte

    always @(posedge clk) begin
        if (rst) begin
            rx_sr    <= 8'h00;
            tx_sr    <= 8'h00;
            bit_cnt  <= 4'd0;
            rx_byte  <= 8'h00;
            rx_valid <= 1'b0;
            miso     <= 1'b0;
        end else begin
            rx_valid <= 1'b0;

            if (~cs_active) begin
                // CSn high: reset bit counter, preload tx_sr for byte 0.
                bit_cnt <= 4'd0;
                tx_sr   <= tx_byte;
                miso    <= tx_byte[7];
            end else begin
                // CSn low: shift in on SCK rising, shift out on SCK falling.
                if (sck_rise) begin
                    rx_sr <= {rx_sr[6:0], mosi};
                    if (bit_cnt == 4'd7) begin
                        rx_byte  <= {rx_sr[6:0], mosi};
                        rx_valid <= 1'b1;
                        bit_cnt  <= 4'd0;
                        // NOTE: do NOT preload tx_sr here.  rx_valid lets
                        // cmd_dispatch update tx_byte, but that takes ≥1
                        // FPGA clock.  Preloading on the same edge captures
                        // the STALE tx_byte → response byte is always 0.
                        // Defer the preload to the next SCK falling edge
                        // (sees bit_cnt==0), which gives cmd_dispatch the
                        // ~3 FPGA-clock slack between SCK rise and fall to
                        // produce the new tx_byte.
                    end else begin
                        bit_cnt <= bit_cnt + 4'd1;
                    end
                end
                if (sck_fall) begin
                    if (bit_cnt == 4'd0) begin
                        // Inter-byte falling edge: preload fresh tx_byte.
                        tx_sr <= tx_byte;
                        miso  <= tx_byte[7];
                    end else begin
                        // Mid-byte falling edge: shift the in-flight byte.
                        tx_sr <= {tx_sr[6:0], 1'b0};
                        miso  <= tx_sr[6];
                    end
                end
            end
        end
    end

endmodule
