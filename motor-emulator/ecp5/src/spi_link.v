// ============================================================================
// spi_link.v — BenchPod link: SPI slave (mode 0, MSB first) onto a 16-bit register bus.
//
// Several boards share SCK/MOSI/MISO/SS (the stack harness), so every transaction starts with a
// command byte that names the board:
//
//   byte 0  command   [7] 1 = read, 0 = write   [6] broadcast (writes only)   [5:4] board id
//   byte 1  address   first register; it auto-increments after each 16-bit word
//   write:  data MSB, data LSB, data MSB, data LSB, ...   (one register write per LSB)
//   read:   one turnaround byte (MISO 0x00), then MSB, LSB of each register
//
// A board ignores a transaction that names another board. MISO leaves the pad only while this
// board is addressed by a read (miso_oe); otherwise it floats, so the four boards' 470 R MISO
// resistors never fight. A read captures the whole 16-bit register when its MSB goes out, so a
// live value (a sinc sample) cannot tear between the two bytes.
//
// SCK, SS and MOSI are synchronised to `clk` (36 MHz). MOSI is sampled on the detected rising
// edge. MISO moves to the next bit just after the detected rising edge (the master has already
// sampled the current bit), which leaves nearly a whole SCK period of setup for the next rising
// edge. The first bit of a byte comes about 5 clk after the rising edge, so SCK must stay at or
// below 6 MHz (what tb_top runs). The read address always settles a whole byte before its data is
// loaded (it advances right after a word's MSB is captured), so the register read may be pipelined.
// ============================================================================
`default_nettype none

module spi_link (
    input  wire        clk,
    input  wire        rst,
    input  wire [1:0]  board_id,

    input  wire        sck,
    input  wire        mosi,
    input  wire        csn,
    output wire        miso,
    output reg         miso_oe,

    // register bus
    output reg  [7:0]  addr,        // current register (read address; write address with wr_en)
    input  wire [15:0] rd_data,     // read of `addr`, pipelined in the regfile (settled within a byte)
    output reg         wr_en,       // one-clk write strobe
    output reg  [15:0] wr_data
);

    // ---- synchronisers: sck and mosi go through the same depth so they stay aligned ----
    reg [2:0] sck_s;
    reg [1:0] csn_s;
    reg [1:0] mosi_s;
    always @(posedge clk) begin
        if (rst) begin
            sck_s <= 3'b000; csn_s <= 2'b11; mosi_s <= 2'b00;
        end else begin
            sck_s <= {sck_s[1:0], sck};
            csn_s <= {csn_s[0], csn};
            mosi_s <= {mosi_s[0], mosi};
        end
    end
    wire sck_rise  = sck_s[1] & ~sck_s[2];
    wire cs_active = ~csn_s[1];

    localparam PH_CMD = 3'd0, PH_ADDR = 3'd1, PH_WHI = 3'd2, PH_WLO = 3'd3,
               PH_RDUMMY = 3'd4, PH_RHI = 3'd5, PH_RLO = 3'd6, PH_IGNORE = 3'd7;

    reg [2:0]  phase;
    reg [2:0]  bit_cnt;
    reg [6:0]  rx_sr;
    reg [7:0]  tx_sr;
    reg [7:0]  wr_hi;
    reg [15:0] rd_latch;
    reg        is_read;
    reg        load_pend;           // byte ended: the read address is set
    reg        load_now;            // one clk later: rd_data (registered in the regfile) is valid

    wire [7:0] rx_byte = {rx_sr, mosi_s[1]};
    assign miso = tx_sr[7];

    always @(posedge clk) begin
        wr_en <= 1'b0;
        if (rst || !cs_active) begin
            phase     <= PH_CMD;
            bit_cnt   <= 3'd0;
            tx_sr     <= 8'h00;
            miso_oe   <= 1'b0;
            load_pend <= 1'b0; load_now <= 1'b0;
        end else begin
            load_now <= load_pend;
            if (load_pend) load_pend <= 1'b0;
            if (load_now) begin
                case (phase)
                    // capture the whole word, then move on: the next word's address is then
                    // set a whole byte before its data is needed (the regfile read is pipelined)
                    PH_RHI: begin tx_sr <= rd_data[15:8]; rd_latch <= rd_data; addr <= addr + 8'd1; end
                    PH_RLO:       tx_sr <= rd_latch[7:0];
                    default:      tx_sr <= 8'h00;
                endcase
            end
            if (sck_rise) begin
                bit_cnt <= bit_cnt + 3'd1;
                rx_sr   <= rx_byte[6:0];
                if (bit_cnt != 3'd7) begin
                    tx_sr <= {tx_sr[6:0], 1'b0};          // next bit of the current byte
                end else begin
                    load_pend <= 1'b1;                     // byte complete: rx_byte is whole
                    case (phase)
                        PH_CMD: begin
                            // broadcast reads are not allowed: every board would drive MISO
                            if (rx_byte[6] ? !rx_byte[7] : (rx_byte[5:4] == board_id)) begin
                                phase   <= PH_ADDR;
                                miso_oe <= rx_byte[7];
                                is_read <= rx_byte[7];
                            end else begin
                                phase   <= PH_IGNORE;
                            end
                        end
                        PH_ADDR: begin
                            addr  <= rx_byte;
                            phase <= is_read ? PH_RDUMMY : PH_WHI;
                        end
                        PH_WHI: begin
                            wr_hi <= rx_byte;
                            phase <= PH_WLO;
                        end
                        PH_WLO: begin
                            wr_data <= {wr_hi, rx_byte};
                            wr_en   <= 1'b1;
                            phase   <= PH_WHI;
                        end
                        PH_RDUMMY: phase <= PH_RHI;
                        PH_RHI:    phase <= PH_RLO;
                        PH_RLO:    phase <= PH_RHI;
                        default:   phase <= PH_IGNORE;
                    endcase
                end
            end
            // the write address advances on the clk after the strobe, so the regfile sees the
            // strobe with the address it was written for
            if (wr_en) addr <= addr + 8'd1;
        end
    end
endmodule
`default_nettype wire
