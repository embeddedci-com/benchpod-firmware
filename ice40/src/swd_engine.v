// ============================================================================
// swd_engine.v — full OpenOCD remote_bitbang (SWD) decoder in the FPGA,
//                (v44) a byte-wide SPI master: the engine's second job, and
//                (v45) a queue of whole SWD transfers run in the fabric.
//
// Replaced the RP2350's old swd.pio + swd_rbb.c.  The STM32 forwards the raw
// remote_bitbang byte stream to the FPGA (SWD_FEED) and reads the sampled bits
// back (SWD_READ); ALL wire driving happens here.  (The pod's CMSIS-DAP bridge
// uses the transfer queue below.)
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
// out BIT 0 FIRST (v45: the SPI byte runs on the SWD transfer sequencer, which shifts right;
// the firmware bit-reverses bytes for an MSB-first device, both ways); the byte clocked in
// lands in the reply buffer at the same index, so SWD_READ returns the response of the
// same feed.  The shifter runs on its own clock
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
//
// SWD transfer queue (v45, SWD_QFEED / SWD_QCONFIG / SWD_QSTATUS).  With SWD armed, a
// SWD_QFEED queues whole transfers in tmem: a request byte (the 8-bit SWD packet, start bit in
// bit 0) and, for a write (bit 2 = RnW clear), 4 data bytes LE.  The engine runs them back to
// back, CMSIS-DAP's sequence, one SWCLK cycle = `half` clk low then `half` clk high:
//   request 8 (driven) | turnaround 1 | ACK 3 (sampled) |
//     read:  data 32 + parity (sampled) | turnaround 1 |
//     write: turnaround 1 | data 32 + parity (driven) |
//   idle cycles driving 0
// The host drives SWDIO on the falling edge; the target changes it on the rising edge, and the
// engine samples it at the END of the low phase, just before the next rising edge (CMSIS-DAP's
// SW_READ_BIT).  Through the 2-flop synchroniser that is the pin 2 clk earlier, still a whole
// high phase after the target's edge, so half = 2 (6 MHz) keeps full margin.  A read stores its
// 4 data bytes (LE) in the reply buffer.  An ACK other than OK, or a read parity error, stops the
// queue after a turnaround (+ idle); SWD_QSTATUS reports the transfers completed, the ACK and
// the parity flag, and the firmware retries a WAIT by queueing the rest again.  SWD_QSTATUS
// reports how far the queue got as the read pointer: a stopped transfer has consumed its request
// byte only, so the firmware maps the pointer back to the op that stopped.  When the next
// queued byte has not arrived yet (the link is slower than the wire), the engine holds SWCLK high
// until it has: SWD lets the host stop the clock anywhere.  In queue mode SWDIO comes straight
// from the sequencer state, which changes only on the falling edge.
// ============================================================================

module swd_engine #(
    parameter REPLY_AW = 9    // 512-entry reply buffer (1 BRAM); also the queue depth
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

    // ---- SWD transfer queue (v45) ----
    input  wire        q_cfg_stb,         // 1-cycle: SWD_QCONFIG
    input  wire [5:0]  q_cfg_half,        // clk per SWCLK half period, 2..63
    input  wire [4:0]  q_cfg_idle,        // idle cycles after each transfer
    input  wire        feed_q,            // level: this feed (SWD_QFEED) queues transfers

    // ---- feed (SWD_FEED / SWD_QFEED) ----
    input  wire        feed_begin,        // 1-cycle: new feed → reset reply ptr
    input  wire        feed_stb,          // 1-cycle per command byte
    input  wire [7:0]  feed_byte,
    input  wire        dio_in,            // live level of in_ch (SWDIO, or MISO in SPI mode)

    // ---- reply read (SWD_READ) ----
    input  wire [REPLY_AW-1:0] rd_addr,
    output reg  [7:0]          rd_data,   // registered (BRAM) read
    output wire [15:0]         reply_count,

    // ---- status ----
    output wire        armed,
    output wire        spi_mode,
    output wire        spi_busy,
    output wire [7:0]  q_done,            // queue bytes consumed, low 8 bits (qptr)
    output wire [7:0]  q_flags,           // {1'b0, qptr[8], perr, ack[2:0], stopped, busy}

    // ---- outputs to la_bank ----
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
    reg [5:0]  half_r;                    // SCK / SWCLK half period (SPI_ARM or SWD_QCONFIG)
    reg        cpol_r;

    // ---- reply buffer (inferred BRAM) ----
    reg [7:0]  rmem [0:(1<<REPLY_AW)-1];
    reg [REPLY_AW-1:0] wptr;              // next reply index

    // ---- transmit queue (inferred BRAM): SPI bytes / SWD transfers ----
    reg [7:0]  tmem [0:(1<<REPLY_AW)-1];
    reg [REPLY_AW-1:0] fptr;              // next free queue index
    reg [REPLY_AW-1:0] qptr;              // next queue byte to run; qptr..fptr-1 are pending
    reg [7:0]  t_rd;                      // tmem[qptr], one edge late

    assign reply_count = {{(16-REPLY_AW){1'b0}}, wptr};

    // SWDIO / MISO is a pad-driven async input; 2-flop synchronise it before sampling, same
    // metastability idiom as spi_slave / i2c_target.  SWD bit-bang holds the line static for
    // many clocks around a 'c'; the shifters account for the 2-cycle latency (see the header).
    reg dio_s0, dio_s1;
    always @(posedge clk) begin
        if (rst) begin dio_s0 <= 1'b0; dio_s1 <= 1'b0; end
        else     begin dio_s0 <= dio_in; dio_s1 <= dio_s0; end
    end

    // ---- the sequencer: SPI bytes or SWD transfers from the queue (one job at a time) ----
    // One cycle = `half` clk low then `half` clk high.  Its index `idx` (and SWDIO) moves on at
    // the end of the high half, i.e. on the falling edge.
    //   SPI:  a unit is one byte, idx 0..7; MOSI = sh[0]; MISO is taken at the END of the high
    //         half (the device changed it on the falling edge; the 2-flop synchroniser then sees
    //         the pin at the rising edge) and shifted in at the top as the byte shifts out.
    //   SWD:  a unit is one transfer (see the header), idx 0..end_idx; the target's bit is taken
    //         at the end of the LOW half.
    reg [5:0]  hcnt;                      // clk left in this half period
    reg [7:0]  sh;
    reg        qmode;                     // the current feed queues SWD transfers
    localparam Q_IDLE = 1'b0, Q_RUN = 1'b1;
    reg        qst;
    reg [6:0]  idx;
    reg [6:0]  end_idx;                   // SPI 7; SWD 45 + idle cycles (SWD_QCONFIG)
    reg        ph;                        // 0 = low half, 1 = high half
    reg        qready;                    // qptr != fptr, one edge late (t_rd is valid with it)
    reg        isrd, par, fail, perr, qstop;
    reg [2:0]  ack;
    wire       run    = armed_r & (spi_r | qmode);
    wire       q_active = armed_r & ~spi_r & qmode;
    wire       ack_ok = (ack == 3'b001);
    wire       spi_pending = (fptr != qptr);

    // SWDIO in queue mode: a function of the sequencer state (updated on the falling edge)
    wire       q_idle_cyc = (idx >= 7'd46) | (qst == Q_IDLE);
    wire       q_oe  = (idx < 7'd8) | q_idle_cyc | (~isrd & (idx >= 7'd13));
    wire       q_out = ~q_idle_cyc & ((idx == 7'd45) ? par : sh[0]);
    wire       q_byte_in = ~spi_r & ~isrd & ((idx == 7'd12) | (idx == 7'd20) | (idx == 7'd28) | (idx == 7'd36));

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
            half_r         <= 6'd4;
            cpol_r         <= 1'b1;
            end_idx        <= 7'd53;
            wptr           <= {REPLY_AW{1'b0}};
            fptr           <= {REPLY_AW{1'b0}};
            qptr           <= {REPLY_AW{1'b0}};
            qst            <= Q_IDLE;
            qmode          <= 1'b0;
            qready         <= 1'b0;
            qstop          <= 1'b0;
            ack            <= 3'd0;
            perr           <= 1'b0;
        end else begin
            if (disarm_stb) begin
                armed_r <= 1'b0;
                spi_r   <= 1'b0;
                qst     <= Q_IDLE;
            end else if (arm_stb) begin
                // Initial line state: SWCLK low, SWDIO driven low.  Matches the
                // old swd_probe_arm().  Between queued transfers SWCLK rests high.
                armed_r        <= 1'b1;
                spi_r          <= 1'b0;
                clk_ch_r       <= arm_clk_ch;
                dio_ch_r       <= arm_dio_ch;
                in_ch_r        <= arm_dio_ch;
                swclk_lvl      <= 1'b0;
                swdio_out      <= 1'b0;
                swdio_oe       <= 1'b1;
                cpol_r         <= 1'b1;
                end_idx        <= 7'd53;               // 8 idle cycles until SWD_QCONFIG
                wptr           <= {REPLY_AW{1'b0}};
                qst            <= Q_IDLE;
                qmode          <= 1'b0;
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
                end_idx        <= 7'd7;                // a unit is one byte
                swclk_lvl      <= spi_cpol;
                swdio_out      <= 1'b0;
                swdio_oe       <= 1'b1;
                cs_n           <= 1'b1;
                wptr           <= {REPLY_AW{1'b0}};
                fptr           <= {REPLY_AW{1'b0}};
                qptr           <= {REPLY_AW{1'b0}};
                qst            <= Q_IDLE;
                qmode          <= 1'b0;
                qstop          <= 1'b0;
            end

            if (spi_cs_stb) cs_n <= ~spi_cs_assert;
            if (q_cfg_stb) begin half_r <= q_cfg_half; end_idx <= 7'd45 + {2'b00, q_cfg_idle}; end

            if (feed_begin) begin
                wptr  <= {REPLY_AW{1'b0}};
                fptr  <= {REPLY_AW{1'b0}};
                qptr  <= {REPLY_AW{1'b0}};
                qmode <= feed_q;
                qst   <= Q_IDLE;
                qstop <= 1'b0;
                ack   <= 3'd0;
                perr  <= 1'b0;
            end

            // ---- SWD bit-bang (remote_bitbang bytes) ----
            if (feed_stb && armed_r && !spi_r && !qmode) begin
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

            // ---- SPI data bytes / SWD transfers: into the queue ----
            if (feed_stb && run) begin
                tmem[fptr] <= feed_byte;
                fptr       <= fptr + 1'b1;
            end

            // ---- the sequencer ----
            // A step that takes a queued data byte (q_byte_in) waits, SWCLK high, until qready
            // says it has arrived.
            qready <= (qptr != fptr) & ~feed_begin;   // a new feed resets the pointers this edge
            if (run) begin
                if (qst == Q_IDLE) begin
                    if (!qstop && qready) begin             // t_rd = the SPI byte / request byte
                        sh        <= t_rd;
                        isrd      <= t_rd[2];
                        qptr      <= qptr + 1'b1;
                        idx       <= 7'd0;
                        fail      <= 1'b0;
                        par       <= 1'b0;
                        ph        <= 1'b0;
                        hcnt      <= half_r - 6'd1;
                        swclk_lvl <= 1'b0;
                        qst       <= Q_RUN;
                    end
                end else if (hcnt != 6'd0) hcnt <= hcnt - 6'd1;
                else if (!ph) begin
                    // end of the low half: SWD takes the bit the target drove on the last rise
                    if (!spi_r && idx >= 7'd9 && idx <= 7'd11) ack <= {dio_s1, ack[2:1]};
                    if (!spi_r && isrd && idx >= 7'd12 && idx <= 7'd43) begin
                        par <= par ^ dio_s1;
                        sh  <= {dio_s1, sh[7:1]};
                        if (idx[2:0] == 3'd3) begin          // 19, 27, 35, 43: a byte is in
                            rmem[wptr] <= {dio_s1, sh[7:1]};
                            wptr       <= wptr + 1'b1;
                        end
                    end
                    if (!spi_r && isrd && idx == 7'd44) perr <= par ^ dio_s1;
                    swclk_lvl <= 1'b1;
                    ph        <= 1'b1;
                    hcnt      <= half_r - 6'd1;
                end else if (!(q_byte_in && !qready)) begin
                    // end of the high half: the falling edge and the next cycle
                    ph        <= 1'b0;
                    hcnt      <= half_r - 6'd1;
                    swclk_lvl <= 1'b0;
                    idx       <= idx + 7'd1;
                    if (spi_r) begin
                        sh <= {dio_s1, sh[7:1]};                            // MISO in, next MOSI bit out
                        if (idx == 7'd7) begin
                            rmem[wptr] <= {dio_s1, sh[7:1]};
                            wptr       <= wptr + 1'b1;
                        end
                    end else begin
                        if (idx < 7'd8) sh <= {1'b0, sh[7:1]};             // next request bit
                        if (idx == 7'd11 && !ack_ok) begin                  // turnaround, then idle
                            fail <= 1'b1; isrd <= 1'b1; idx <= 7'd45;
                        end
                        if (!isrd && idx >= 7'd13 && idx <= 7'd44) par <= par ^ sh[0];
                        if (q_byte_in) begin sh <= t_rd; qptr <= qptr + 1'b1; end
                        else if (!isrd && idx >= 7'd13) sh <= {1'b0, sh[7:1]};
                    end
                    if (idx == end_idx) begin                               // the unit is over
                        swclk_lvl <= cpol_r;                                // SPI idle level / SWD high
                        qst       <= Q_IDLE;
                        if (!spi_r && (fail || (isrd && perr))) qstop <= 1'b1;
                    end
                end
            end

            // registered BRAM reads: SWD_READ, and the queue head
            rd_data <= rmem[rd_addr];
            t_rd    <= tmem[qptr];
        end
    end

    assign armed        = armed_r;
    assign spi_mode     = armed_r & spi_r;
    assign spi_busy     = spi_mode & (spi_pending | (qst != Q_IDLE));
    wire   q_busy       = q_active & ~qstop & ((qptr != fptr) | (qst != Q_IDLE));
    assign q_done       = qptr[7:0];
    assign q_flags      = {1'b0, qptr[8], perr, ack, qstop, q_busy};
    assign clk_ch       = clk_ch_r;
    assign clk_val      = swclk_lvl;
    assign dio_ch       = dio_ch_r;
    assign dio_val      = q_active ? q_out : (spi_r ? sh[0] : swdio_out);
    assign dio_oe       = q_active ? q_oe  : (spi_r | swdio_oe);
    assign in_ch        = in_ch_r;
    assign cs_ch        = cs_ch_r;
    assign cs_val       = cs_n;
    assign cs_on        = armed_r & spi_r;

endmodule
