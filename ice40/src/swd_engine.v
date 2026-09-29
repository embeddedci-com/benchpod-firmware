// ============================================================================
// swd_engine.v — full OpenOCD remote_bitbang (SWD) decoder in the FPGA, and
//                (v44) a byte-wide SPI master: the engine's second job.
//
// Replaces the RP2350's old swd.pio + swd_rbb.c.  The RP now just forwards the
// raw remote_bitbang byte stream to the FPGA (SWD_FEED) and reads the sampled
// bits back (SWD_READ); ALL wire driving happens here.
//
// Command bytes (mapping identical to the old swd_rbb.c so the same OpenOCD
// remote_bitbang config keeps working):
//   'd'/'e'/'f'/'g' : swd_write(swclk, swdio) = (0,0)(0,1)(1,0)(1,1)
//   'O' / 'o'       : SWDIO drive (output) / release (input, turnaround)
//   'c'             : sample SWDIO → push one ASCII '0'/'1' into the reply buf
//   's'/'u'/'r'/'t' : nRESET — ignored since v37.  nRESET is the pod's own /NRST_CONTROL
//                     pin on v3 (stm32h563 nrst_ctrl.c), never an LA channel; the firmware
//                     has always sent SWD_ARM's nreset_ch as 0xFF, so this driver was dead.
//   'B'/'b'         : blink LED — ignored
// 'Q' (quit) and '{' (exit-to-JSON) are transport control: the RP handles
// those and never forwards them here.
//
// Reply buffer: a small inferred BRAM.  wptr resets at the START of every feed
// (feed_begin), so each feed's '0'/'1' samples land at indices 0..k-1 and the
// matching SWD_READ drains them from 0.  The RP bounds each feed so the 'c'
// count never exceeds REPLY_DEPTH.
//
// SPI mode (v44, SPI_ARM).  The same two slots carry SCK (clk slot) and MOSI (dio slot,
// always driven), plus a CS output (driven high = released until SPI_CS asserts it) and
// a MISO input channel.  Each SWD_FEED byte is queued in a second BRAM (tmem) and shifted
// out MSB first; the byte clocked in lands in the reply buffer at the same index, so
// SWD_READ returns the response of the same feed.  The shifter runs on its own clock
// (`half` clk per SCK half period), so the link rate and the SPI rate are independent:
// the host queues up to 512 bytes and polls `spi_busy` (SPI_STATUS).
//
// Bit timing (modes 0 and 3 both change data on the falling edge and sample on the
// rising one; they differ only in the idle level, `cpol`): SCK low + next MOSI bit for
// `half` clk, SCK high for `half` clk, then MISO is taken.  MISO goes through the same
// 2-flop synchroniser as SWDIO, so the value used at the end of the high phase is the
// pin as it was 2 clk earlier.  With half >= 2 that is at or after the rising edge the
// target saw, a full half period after its data changed.  half 0 or 1 would sample
// before the edge, so the firmware clamps half to 2..63 (6 MHz .. 190 kHz at 24 MHz).
// ============================================================================

module swd_engine #(
    parameter REPLY_AW = 9    // 512-entry reply buffer (1 BRAM); also the SPI queue depth
)(
    input  wire        clk,
    input  wire        rst,

    // ---- arm / disarm ----
    input  wire        arm_stb,           // 1-cycle: SWD_ARM
    input  wire [3:0]  arm_clk_ch,
    input  wire [3:0]  arm_dio_ch,
    input  wire        spi_arm_stb,       // 1-cycle: SPI_ARM
    input  wire [3:0]  spi_sck_ch,
    input  wire [3:0]  spi_mosi_ch,
    input  wire [3:0]  spi_miso_ch,
    input  wire [3:0]  spi_cs_ch,
    input  wire [5:0]  spi_half,          // clk per SCK half period, 2..63 (the firmware clamps)
    input  wire        spi_cpol,          // SCK idle level: 0 = mode 0, 1 = mode 3
    input  wire        spi_cs_stb,        // 1-cycle: SPI_CS
    input  wire        spi_cs_assert,     // 1 = drive CS low
    input  wire        disarm_stb,        // 1-cycle

    // ---- feed (SWD_FEED) ----
    input  wire        feed_begin,        // 1-cycle: new feed → reset reply ptr
    input  wire        feed_stb,          // 1-cycle per command byte
    input  wire [7:0]  feed_byte,
    input  wire        dio_in,            // live level of in_ch (SWDIO, or MISO in SPI mode)

    // ---- reply read (SWD_READ) ----
    input  wire [REPLY_AW-1:0] rd_addr,
    output reg  [7:0]          rd_data,   // registered (BRAM) read
    output wire [15:0]         reply_count,

    // ---- outputs to la_bank ----
    output wire        armed,
    output wire        spi_mode,
    output wire        spi_busy,
    output wire [3:0]  clk_ch,
    output wire        clk_val,
    output wire [3:0]  dio_ch,
    output wire        dio_val,
    output wire        dio_oe,
    output wire [3:0]  in_ch,             // the channel dio_in must come from
    output wire [3:0]  cs_ch,
    output wire        cs_val,
    output wire        cs_on              // CS slot active (SPI armed)
);

    // ---- session state ----
    reg        armed_r;
    reg        spi_r;
    reg [3:0]  clk_ch_r, dio_ch_r, in_ch_r, cs_ch_r;

    reg        swclk_lvl;                 // SWCLK, or SCK in SPI mode
    reg        swdio_out;                 // SWDIO, or MOSI in SPI mode
    reg        swdio_oe;
    reg        cs_n;
    reg [5:0]  half_r;
    reg        cpol_r;

    // ---- reply buffer (inferred BRAM) ----
    reg [7:0]  rmem [0:(1<<REPLY_AW)-1];
    reg [REPLY_AW-1:0] wptr;              // next reply index (SWD 'c' sample / SPI rx byte)

    // ---- SPI transmit queue (inferred BRAM) ----
    reg [7:0]  tmem [0:(1<<REPLY_AW)-1];
    reg [REPLY_AW-1:0] fptr;              // next queue index; bytes wptr..fptr-1 are pending
    reg [7:0]  t_rd;

    assign reply_count = {{(16-REPLY_AW){1'b0}}, wptr};

    // SWDIO / MISO is a pad-driven async input; 2-flop synchronise it before sampling, same
    // metastability idiom as spi_slave / i2c_target.  SWD holds the line static for many
    // clocks around a 'c'; SPI timing accounts for the 2-cycle latency (see the header).
    reg dio_s0, dio_s1;
    always @(posedge clk) begin
        if (rst) begin dio_s0 <= 1'b0; dio_s1 <= 1'b0; end
        else     begin dio_s0 <= dio_in; dio_s1 <= dio_s0; end
    end

    // ---- SPI shifter ----
    localparam SP_IDLE = 2'd0, SP_LOAD = 2'd1, SP_LOW = 2'd2, SP_HIGH = 2'd3;
    reg [1:0]  sst;
    reg [5:0]  hcnt;
    reg [2:0]  bcnt;
    reg [7:0]  sh;
    wire       pending = (fptr != wptr);
    wire [7:0] sh_in   = {sh[6:0], dio_s1};

    always @(posedge clk) begin
        if (rst) begin
            armed_r        <= 1'b0;
            spi_r          <= 1'b0;
            clk_ch_r       <= 4'd0;
            dio_ch_r       <= 4'd1;
            in_ch_r        <= 4'd1;
            cs_ch_r        <= 4'd2;
            swclk_lvl      <= 1'b0;
            swdio_out      <= 1'b0;
            swdio_oe       <= 1'b1;
            cs_n           <= 1'b1;
            half_r         <= 6'd2;
            cpol_r         <= 1'b0;
            wptr           <= {REPLY_AW{1'b0}};
            fptr           <= {REPLY_AW{1'b0}};
            sst            <= SP_IDLE;
        end else begin
            if (disarm_stb) begin
                armed_r <= 1'b0;
                spi_r   <= 1'b0;
                sst     <= SP_IDLE;
            end else if (arm_stb) begin
                // Initial line state: SWCLK low, SWDIO driven low.  Matches the
                // old swd_probe_arm().
                armed_r        <= 1'b1;
                spi_r          <= 1'b0;
                clk_ch_r       <= arm_clk_ch;
                dio_ch_r       <= arm_dio_ch;
                in_ch_r        <= arm_dio_ch;
                swclk_lvl      <= 1'b0;
                swdio_out      <= 1'b0;
                swdio_oe       <= 1'b1;
                wptr           <= {REPLY_AW{1'b0}};
                sst            <= SP_IDLE;
            end else if (spi_arm_stb) begin
                // SCK at its idle level, MOSI driven low, CS driven high (released).
                armed_r        <= 1'b1;
                spi_r          <= 1'b1;
                clk_ch_r       <= spi_sck_ch;
                dio_ch_r       <= spi_mosi_ch;
                in_ch_r        <= spi_miso_ch;
                cs_ch_r        <= spi_cs_ch;
                half_r         <= spi_half;
                cpol_r         <= spi_cpol;
                swclk_lvl      <= spi_cpol;
                swdio_out      <= 1'b0;
                swdio_oe       <= 1'b1;
                cs_n           <= 1'b1;
                wptr           <= {REPLY_AW{1'b0}};
                fptr           <= {REPLY_AW{1'b0}};
                sst            <= SP_IDLE;
            end

            if (spi_cs_stb) cs_n <= ~spi_cs_assert;

            if (feed_begin) begin
                wptr <= {REPLY_AW{1'b0}};
                fptr <= {REPLY_AW{1'b0}};
            end

            if (feed_stb && armed_r && !spi_r) begin
                case (feed_byte)
                    "d": begin swclk_lvl <= 1'b0; swdio_out <= 1'b0; end
                    "e": begin swclk_lvl <= 1'b0; swdio_out <= 1'b1; end
                    "f": begin swclk_lvl <= 1'b1; swdio_out <= 1'b0; end
                    "g": begin swclk_lvl <= 1'b1; swdio_out <= 1'b1; end
                    "O": swdio_oe <= 1'b1;
                    "o": swdio_oe <= 1'b0;
                    "c": begin
                        rmem[wptr] <= dio_s1 ? "1" : "0";   // synchronised SWDIO
                        wptr       <= wptr + 1'b1;
                    end
                    default: ;   // 'B'/'b', nRESET 's'/'u'/'r'/'t' and others: ignore
                endcase
            end

            // ---- SPI: queue a feed byte ----
            if (feed_stb && armed_r && spi_r) begin
                tmem[fptr] <= feed_byte;
                fptr       <= fptr + 1'b1;
            end

            // ---- SPI: shift the queue out, one byte at a time ----
            // t_rd is tmem[wptr] read one edge earlier: SP_LOAD waits that edge out.
            if (armed_r && spi_r) begin
                case (sst)
                    SP_IDLE: if (pending) sst <= SP_LOAD;
                    SP_LOAD: begin
                        sh        <= t_rd;
                        swdio_out <= t_rd[7];
                        swclk_lvl <= 1'b0;
                        bcnt      <= 3'd7;
                        hcnt      <= half_r - 6'd1;
                        sst       <= SP_LOW;
                    end
                    SP_LOW:
                        if (hcnt == 6'd0) begin
                            swclk_lvl <= 1'b1;            // rising edge: target samples MOSI
                            hcnt      <= half_r - 6'd1;
                            sst       <= SP_HIGH;
                        end else hcnt <= hcnt - 6'd1;
                    SP_HIGH:
                        if (hcnt != 6'd0) hcnt <= hcnt - 6'd1;
                        else if (bcnt == 3'd0) begin     // last bit: store the byte
                            rmem[wptr] <= sh_in;
                            wptr       <= wptr + 1'b1;
                            swclk_lvl  <= cpol_r;         // back to idle
                            sst        <= SP_IDLE;
                        end else begin                    // falling edge + next MOSI bit
                            sh        <= sh_in;
                            swdio_out <= sh[6];
                            swclk_lvl <= 1'b0;
                            bcnt      <= bcnt - 3'd1;
                            hcnt      <= half_r - 6'd1;
                            sst       <= SP_LOW;
                        end
                endcase
            end

            // registered BRAM reads: SWD_READ, and the SPI queue head
            rd_data <= rmem[rd_addr];
            t_rd    <= tmem[wptr];
        end
    end

    assign armed        = armed_r;
    assign spi_mode     = armed_r & spi_r;
    assign spi_busy     = spi_mode & (pending | (sst != SP_IDLE));
    assign clk_ch       = clk_ch_r;
    assign clk_val      = swclk_lvl;
    assign dio_ch       = dio_ch_r;
    assign dio_val      = swdio_out;
    assign dio_oe       = swdio_oe;
    assign in_ch        = in_ch_r;
    assign cs_ch        = cs_ch_r;
    assign cs_val       = cs_n;
    assign cs_on        = armed_r & spi_r;

endmodule
