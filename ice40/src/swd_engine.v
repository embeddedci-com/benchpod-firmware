// ============================================================================
// swd_engine.v — full OpenOCD remote_bitbang (SWD) decoder in the FPGA.
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
// ============================================================================

module swd_engine #(
    parameter REPLY_AW = 9    // 512-entry reply buffer (1 BRAM)
)(
    input  wire        clk,
    input  wire        rst,

    // ---- arm / disarm ----
    input  wire        arm_stb,           // 1-cycle
    input  wire [3:0]  arm_clk_ch,
    input  wire [3:0]  arm_dio_ch,
    input  wire        disarm_stb,        // 1-cycle

    // ---- feed (SWD_FEED) ----
    input  wire        feed_begin,        // 1-cycle: new feed → reset reply ptr
    input  wire        feed_stb,          // 1-cycle per command byte
    input  wire [7:0]  feed_byte,
    input  wire        dio_in,            // live SWDIO pin level (for 'c')

    // ---- reply read (SWD_READ) ----
    input  wire [REPLY_AW-1:0] rd_addr,
    output reg  [7:0]          rd_data,   // registered (BRAM) read
    output wire [15:0]         reply_count,

    // ---- outputs to la_bank ----
    output wire        armed,
    output wire [3:0]  clk_ch,
    output wire        clk_val,
    output wire [3:0]  dio_ch,
    output wire        dio_val,
    output wire        dio_oe
);

    // ---- session state ----
    reg        armed_r;
    reg [3:0]  clk_ch_r, dio_ch_r;

    reg        swclk_lvl;
    reg        swdio_out;
    reg        swdio_oe;

    // ---- reply buffer (inferred BRAM) ----
    reg [7:0]  rmem [0:(1<<REPLY_AW)-1];
    reg [REPLY_AW-1:0] wptr;

    assign reply_count = {{(16-REPLY_AW){1'b0}}, wptr};

    // SWDIO is a pad-driven async input (the target drives it during read
    // turnaround); 2-flop synchronise it before the 'c' sample, same metastability
    // idiom as spi_slave / i2c_target.  The line is held static by the target for
    // many clocks around a 'c', so the 2-cycle latency is immaterial.
    reg dio_s0, dio_s1;
    always @(posedge clk) begin
        if (rst) begin dio_s0 <= 1'b0; dio_s1 <= 1'b0; end
        else     begin dio_s0 <= dio_in; dio_s1 <= dio_s0; end
    end

    always @(posedge clk) begin
        if (rst) begin
            armed_r        <= 1'b0;
            clk_ch_r       <= 4'd0;
            dio_ch_r       <= 4'd1;
            swclk_lvl      <= 1'b0;
            swdio_out      <= 1'b0;
            swdio_oe       <= 1'b1;
            wptr           <= {REPLY_AW{1'b0}};
        end else begin
            if (disarm_stb) begin
                armed_r <= 1'b0;
            end else if (arm_stb) begin
                // Initial line state: SWCLK low, SWDIO driven low.  Matches the
                // old swd_probe_arm().
                armed_r        <= 1'b1;
                clk_ch_r       <= arm_clk_ch;
                dio_ch_r       <= arm_dio_ch;
                swclk_lvl      <= 1'b0;
                swdio_out      <= 1'b0;
                swdio_oe       <= 1'b1;
                wptr           <= {REPLY_AW{1'b0}};
            end

            if (feed_begin) wptr <= {REPLY_AW{1'b0}};

            if (feed_stb && armed_r) begin
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

            // registered BRAM read for SWD_READ
            rd_data <= rmem[rd_addr];
        end
    end

    assign armed        = armed_r;
    assign clk_ch       = clk_ch_r;
    assign clk_val      = swclk_lvl;
    assign dio_ch       = dio_ch_r;
    assign dio_val      = swdio_out;
    assign dio_oe       = swdio_oe;

endmodule
