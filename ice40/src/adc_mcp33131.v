// ============================================================================
// adc_mcp33131.v — serial capture engine for the MCP33131D-10 (16-bit, 1 MSPS
// SAR ADC) on the STM32 "vbench-pod" board (VERSION=v2).
//
// 3-wire read: pulse CNVST high to start a conversion, wait tCONV, then drop
// CNVST and clock 16 bits out on SDO (MSB first).  `en` runs conversions
// back-to-back, rate-limited by `divider` (sample period in clk cycles).  Each
// completed sample is presented on `sample` with a 1-cycle `sample_stb`.
//
// CLOCK = 24 MHz control clock (clk), NOT the 48 MHz capture clock.  SCLK = clk/2 =
// 12 MHz.  At 24 MHz SCLK (the old 48 MHz engine) the round trip (SCLK out through the
// pad and module cable, tDO, SDO back) did not fit the half-period, so SDO was sampled
// mid-transition and read back as coupling (0x5555).  SDO also needs the iCE40 internal
// pull-up (defines the idle level so a real driven-low read isn't masked); the pad's
// SB_IO is in this module since v63.  `sample`/`sample_stb` stay in this single 24 MHz
// domain: the capture datapath runs on the same clk, so there is no crossing.
//
// v63: SDO is sampled late, for margin with more series resistance on the module cable.
// Datasheet DS20005947B: tDO (SDO valid after SCLK low) is 9.5 ns max at DVIO 3.3 V (16 ns
// is the 1.7 V figure), tEN (MSB valid after CNVST low) 10 ns max.
//   * Each bit is sampled on the FALLING clk edge in the middle of SCLK high, 1.5 clk
//     (62.5 ns) after the clk edge that drove the SCLK fall that shifted it out.  Until v62
//     it was the rising edge 1 clk (41.7 ns) after: the round trip (SCLK pad and cable out,
//     tDO, SDO cable and pad back) gets 20.8 ns more.  Hold: the next bit only changes tDO
//     after the next SCLK fall, which leaves the FPGA half a clk after the sample.
//   * One idle clk after CNVST falls: the MSB is sampled 2.5 clk (104 ns) after the edge
//     that dropped CNVST (41.7 ns until v62).
//   The falling-edge sample is the SB_IO's DDR input register (D_IN_1), not a fabric negedge
//   flop: the pad-to-register delay is fixed in the IO tile, the same on every PnR seed, so
//   the margin does not depend on placement on this congested up5k.  It costs no logic cell
//   (a negedge flop would take an LC and force its whole PLB to the falling edge), and the
//   register to the fabric shift register is a timed half-period (20.8 ns) path.
//   Cost: start-to-strobe is 59 clk (58 until v62), still under the firmware's 60-clk floor,
//   so every sample period and the v52 t0 alignment (the CNVST rises) are unchanged.
// Verified: a DAC swept into the ADC input tracks linearly with ~2 LSB noise.
// tb_adc_timing checks the read against mcp33131_model with pad and cable delays.
// ============================================================================
module adc_mcp33131 (
    input  wire        clk,           // 24 MHz control clock
    input  wire        rst,
    input  wire        en,            // capture active (level; synchronised in top_v2)
    input  wire [15:0] divider,       // sample period in clk cycles
    input  wire        restart,       // v52: 1-cycle: drop any conversion, first new one `divider` clk later
    // ADC pins
    output reg         adc_cnvst,
    output reg         adc_sclk,
    output wire        adc_sdi,
    input  wire        adc_sdo,       // the PAD: its SB_IO (pull-up, DDR input register) is below
    // result
    output reg  [15:0] sample,
    output reg         sample_stb
);
    // SDI must be held HIGH (tied to DVIO) for normal 3-wire operation
    // (datasheet DS20005947B Fig 7-2 Note 1).
    assign adc_sdi = 1'b1;

    // v63: the SDO pad, a registered input with the pull-up.  D_IN_1 is the pad as the IO
    // register saw it on the last FALLING edge of clk (the mid-SCLK-high sample, see the
    // header); the fabric shifts it in on the next rising edge.  PIN_TYPE 6'b000000: no
    // output, registered (DDR) input.  Until v62 top_v2 had a combinational SB_IO
    // (6'b000001) here and the fabric sampled the pad on the rising edge.
    wire sdo_fall;
    SB_IO #(.PIN_TYPE(6'b000000), .PULLUP(1'b1)) io_adc_sdo (
        .PACKAGE_PIN(adc_sdo), .INPUT_CLK(clk), .D_IN_1(sdo_fall));

    // Conversion wait: the MCP33131D-10 (1 MSPS) has tCNV = 710 ns; 24 cycles at
    // 24 MHz = 1.0 us gives ~40% margin.  With the idle clk after CNVST falls (v63) and the
    // 16-bit read (32 cycles @ 12 MHz SCLK) one conversion is 59 clk start to strobe, so the
    // firmware's sample-period floor of 60 -> ~400 kS/s still holds.
    localparam [7:0] CONV_CYCLES = 8'd24;   // ~1.0 us at 24 MHz (> tCNV 710 ns)
    localparam S_IDLE = 2'd0, S_CONV = 2'd1, S_READ = 2'd2, S_LEAD = 2'd3;

    reg [1:0]  st;
    reg [15:0] period_cnt;
    reg        first_p;           // v52: the period after a restart ends 3 clk early (see restart)
    reg        period_zero;       // registered "period expired": keeps the 16-bit
                                  // compare off the conversion-start transition path
    // One 5-bit counter serves the conversion wait (S_CONV, to CONV_CYCLES) and the bit count
    // (S_READ, to 16): the two states never overlap (v45; was an 8-bit and a 5-bit counter).
    reg [4:0]  cnt;
    reg        conv_done;         // registered (cnt == CONV_CYCLES)
    reg        bits_done;         // registered (cnt == 16 in S_READ)
    reg [15:0] shreg;
    reg        sclk_ph;

    always @(posedge clk) begin
        sample_stb <= 1'b0;
        if (rst) begin
            st <= S_IDLE; adc_cnvst <= 1'b0; adc_sclk <= 1'b0;
            period_cnt <= 16'd0; period_zero <= 1'b1; cnt <= 5'd0; sclk_ph <= 1'b0; first_p <= 1'b0;
        end else begin
            // Sample period = EXACTLY max(divider, 60) clocks.  period_cnt loads the
            // period at a conversion start and counts down; the registered flag is set
            // as the count goes 2 -> 1, so S_IDLE sees it `period` clocks after that
            // start.  (Gateware <= v31 set it at 1 -> 0 and the flag's register delay
            // made every period divider + 1 clocks.)  Safe because the load is >= 60.
            if (!period_zero) begin
                period_cnt  <= period_cnt - 16'd1;
                // Set as the count leaves 2 (normally) or 5 (the first period after a restart).
                // v52: tested as <= 2 / <= 5, not == 2 / == 5, so a divider below that (0 or 1 used
                // to wrap the count into a 65536-clock period) expires at once: back to back.
                // Same LUT tree as the old == compare; the load below stays a constant.
                period_zero <= (period_cnt[15:3] == 13'd0) &
                               (first_p ? ~(period_cnt[2] & period_cnt[1])
                                        : ~(period_cnt[2] | (period_cnt[1] & period_cnt[0])));
            end
            if (restart) begin
                // v52: the capture's t0.  The engine free-runs (the loop image reads it between
                // captures), so before v52 the first captured sample was whatever conversion was
                // in flight at t0: up to a period EARLY, or up to the PREVIOUS capture's period
                // late, with its first interval at the old divider.  Abort the conversion or read
                // in flight (its sample is never strobed) and start the period counter as a
                // conversion start would, so the first conversion of the capture starts exactly
                // `divider` clocks after t0, like the LA producer's first sample, and every later
                // one `divider` after that.  CNVST drops here; the next rise is >= divider clk
                // later, which meets the ADC's 1 us tCYC for any divider >= 27 (firmware floors 60).
                // The first period is 3 clk short: the LA producer's sample is the pad as its
                // 2-flop synchroniser saw it, 3 clk before the edge that latches it, so with
                // divider - 3 the ADC's first conversion starts on the very edge the LA's first
                // sample was taken (tb_top_adc_t0).  A divider <= 5 starts at once.
                st          <= S_IDLE;
                adc_cnvst   <= 1'b0;
                adc_sclk    <= 1'b0;
                period_cnt  <= divider;
                first_p     <= 1'b1;
                period_zero <= 1'b0;
            end else case (st)
            S_IDLE: begin
                adc_sclk <= 1'b0;
                if (en && period_zero) begin
                    adc_cnvst   <= 1'b1;              // start conversion
                    cnt         <= 5'd0;
                    conv_done   <= 1'b0;
                    // Floor the sample period at the engine's own min duration
                    // (59 clk: 1 idle + 24 conv + 1 lead + 32 read + transitions) so a small
                    // requested divider can't ask for a rate the FSM can't sustain.
                    // v40: no 60-clock floor here; the firmware floors the divider
                    // (adc_min_divider).  A shorter period cannot break the engine: the next
                    // conversion just starts as soon as this one has been read out.
                    period_cnt  <= divider;
                    first_p     <= 1'b0;
                    period_zero <= 1'b0;
                    st          <= S_CONV;
                end
            end
            // conv_done/bits_done are registered one tick ahead of the count==N
            // compares so those compares stay off the FSM's transition paths.
            S_CONV: begin
                if (conv_done) begin
                    adc_cnvst <= 1'b0;              // end convert, begin read
                    cnt       <= 5'd0;
                    bits_done <= 1'b0;
                    sclk_ph   <= 1'b0;
                    st        <= S_LEAD;
                end else begin
                    cnt       <= cnt + 5'd1;
                    conv_done <= (cnt == CONV_CYCLES[4:0] - 5'd1);
                end
            end
            // v63: one idle clk after CNVST falls, so the MSB (valid tEN after the fall) gets
            // 2.5 clk to the first sample instead of 1.
            S_LEAD: st <= S_READ;
            // Read 16 bits, MSB first.  SDO changes tDO after each SCLK falling edge.  SCLK rises
            // on the sclk_ph=0 edge; the SB_IO samples SDO half a clk later, in the middle of SCLK
            // high (sdo_fall), and the sclk_ph=1 edge shifts it in as it drops SCLK.  The last
            // bit goes straight into `sample` on that same edge.
            S_READ: begin
                sclk_ph <= ~sclk_ph;
                if (!sclk_ph) begin
                    adc_sclk  <= 1'b1;               // SCLK high next cycle
                    cnt       <= cnt + 5'd1;
                    bits_done <= (cnt == 5'd15);
                end else begin
                    adc_sclk <= 1'b0;               // falling edge
                    shreg    <= {shreg[14:0], sdo_fall};
                    if (bits_done) begin
                        sample     <= {shreg[14:0], sdo_fall};
                        sample_stb <= 1'b1;
                        st         <= S_IDLE;
                    end
                end
            end
            endcase
        end
    end
endmodule
