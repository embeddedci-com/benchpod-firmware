// ============================================================================
// spi_slave.v — SPI slave (mode 0) for the iCE40 signal-engine FPGA.
//
// Receives bytes from the STM32H563 on MOSI, presents them as rx_byte + rx_valid
// strobe to the cmd_dispatch FSM, and shifts tx_byte out on MISO during the byte
// after the one it answers.  CSn high resets the bit counter.
//
// Everything runs on `clk` (24 MHz, SYS_CLK_MHZ): SCK is oversampled, not used as a clock.
// (A slave clocked by SCK itself would run far faster, but SCK is not on a global-clock pin and
// all eight global buffers are in use; see the v52 notes in top_v2.v.)
//
// v52 timing (until v51 the STM32 had to give a half period of ~170 ns: 1.95 MHz clean, 3.9 MHz
// failed on hardware):
//   * SCK and CSn pass two flops.  MOSI passes the SAME two flops, so the bit taken at a detected
//     rise is MOSI as it was within one clk of that rise, not 2-3 clk later: MOSI only has to be
//     held ~45 ns after the rise (the master changes it half a period later).
//   * MISO moves on the detected RISE (the master has just sampled it), so the next bit is on the
//     pin ~3 clk after a rise and stays a whole period minus that before the next one.  Until v51
//     it moved on the detected fall and had only half a period minus ~4 clk.
//   * Between bytes (bit counter 0) MISO is tx_byte[7] straight from cmd_dispatch's register, so
//     the reply to a byte only has to be ready by the NEXT byte's first rise (one period after the
//     last rise, plus any inter-byte gap), not by the falling edge after it.  The rest of the byte
//     is loaded into the shift register on that first rise.
// tb_top_spilink (an STM32 model with pad delays and a drifting phase): clean down to a 95 ns half
// period with no inter-byte gap (the reply turnaround, ~5 clk from a byte's last rise to its new
// tx_byte, must fit in one period) and to 75 ns with a 1-2 SCK gap (then the mid-byte MISO path,
// ~140 ns from a rise to the next bit, is the limit).  The STM32's SPI1 prescalers are powers of
// two, so that is /64 = 3.9 MHz with ~30% margin, twice v51's /128; /32 (7.8 MHz, 64 ns) would need
// a one-flop SCK synchroniser or an SCK-clocked slave, i.e. a global buffer.
// ============================================================================

module spi_slave (
    input  wire        clk,        // 24 MHz system clock (SYS_CLK_MHZ)
    input  wire        rst,        // synchronous reset, active high

    // SPI bus (mode 0)
    input  wire        sck,
    input  wire        mosi,
    output wire        miso,
    input  wire        csn,

    // Parallel byte interface to cmd_dispatch
    output reg  [7:0]  rx_byte,    // last received byte
    output reg         rx_valid,   // 1-cycle strobe when rx_byte is fresh
    input  wire [7:0]  tx_byte,    // byte to shift out during the next byte
    output wire        cs_active   // mirrors !csn (synchronised)
);

    // ---- SCK, CSn and MOSI synchronisation (CDC), MOSI kept aligned with SCK ----
    reg sck_s0, sck_s1, sck_s2;
    reg csn_s0, csn_s1;
    reg mosi_s0, mosi_s1;

    always @(posedge clk) begin
        if (rst) begin
            sck_s0 <= 1'b0; sck_s1 <= 1'b0; sck_s2 <= 1'b0;
            csn_s0 <= 1'b1; csn_s1 <= 1'b1;
        end else begin
            sck_s0 <= sck;  sck_s1 <= sck_s0;  sck_s2 <= sck_s1;
            csn_s0 <= csn;  csn_s1 <= csn_s0;
        end
        mosi_s0 <= mosi; mosi_s1 <= mosi_s0;
    end

    // sck_s1 & ~sck_s2: s1 holds the first sample that saw SCK high, and mosi_s1 the MOSI of that
    // same sample.
    wire sck_rise = (sck_s1 & ~sck_s2);
    assign cs_active = ~csn_s1;

    // ---- Shift registers ----
    reg [6:0] rx_sr;
    reg [6:0] tx_sr;                 // bits 6..0 of the byte being sent, MSB first
    reg [2:0] bit_cnt;               // bits of this byte received, 0..7

    // MISO: the byte's MSB comes live from tx_byte until the byte's first rise; then the shift
    // register, which moves one bit right after each rise.
    reg miso_r;
    assign miso = (bit_cnt == 3'd0) ? tx_byte[7] : miso_r;

    always @(posedge clk) begin
        if (rst) begin
            rx_sr    <= 7'h00;
            tx_sr    <= 7'h00;
            bit_cnt  <= 3'd0;
            rx_byte  <= 8'h00;
            rx_valid <= 1'b0;
            miso_r   <= 1'b0;
        end else begin
            rx_valid <= 1'b0;

            if (~cs_active) begin
                bit_cnt <= 3'd0;      // CSn high: the next byte starts from tx_byte's MSB
            end else if (sck_rise) begin
                rx_sr   <= {rx_sr[5:0], mosi_s1};
                bit_cnt <= bit_cnt + 3'd1;
                if (bit_cnt == 3'd7) begin
                    rx_byte  <= {rx_sr, mosi_s1};
                    rx_valid <= 1'b1;            // cmd_dispatch answers in tx_byte (MISO follows it)
                end
                if (bit_cnt == 3'd0) begin
                    // the master took the MSB (tx_byte[7]): keep the rest of THIS reply
                    miso_r <= tx_byte[6];
                    tx_sr  <= {tx_byte[5:0], 1'b0};
                end else begin
                    miso_r <= tx_sr[6];
                    tx_sr  <= {tx_sr[5:0], 1'b0};
                end
            end
        end
    end

endmodule
