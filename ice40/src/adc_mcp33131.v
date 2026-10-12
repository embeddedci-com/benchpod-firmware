// ============================================================================
// adc_mcp33131.v — serial capture engine for the MCP33131D-10 (16-bit, 1 MSPS
// SAR ADC) on the STM32 "vbench-pod" board (VERSION=v2).
//
// 3-wire read: pulse CNVST high to start a conversion, wait tCONV, then drop
// CNVST and clock 16 bits out on SDO (MSB first).  `en` runs conversions
// back-to-back, rate-limited by `divider` (sample period in clk cycles).  Each
// completed sample is presented on `sample` with a 1-cycle `sample_stb`.
//
// CLOCK = 24 MHz control clock (clk), NOT the 48 MHz capture clock.  Running the
// engine here (SCLK = clk/2 = 12 MHz) is what makes the read work: at 24 MHz
// SCLK the datasheet tDO (SDO valid after the SCLK edge, up to ~16 ns) did NOT
// settle within the ~21 ns half-period, so SDO was sampled mid-transition and
// read back as coupling (0x5555).  At 12 MHz SCLK it settles with margin.  SDO
// also needs the iCE40 internal pull-up enabled at the top level (defines the
// idle level so a real driven-low read isn't masked).  `sample`/`sample_stb` stay
// in this single 24 MHz domain — the capture datapath runs on the same clk, so
// there is no crossing (post single-clock collapse).
// Verified: a DAC swept into the ADC input tracks linearly with ~2 LSB noise.
//
// RECALIBRATE (v49, datasheet DS20005947B section 7.1, Figure 7-3): `recal` (a 1-cycle
// strobe) makes the NEXT conversion's read clock 1024 SCLK pulses instead of 16 (the 1024
// include the 16 data clocks; the first 16 bits are still delivered as a normal sample).  The
// ADC then self-calibrates for tCAL (~500 ms, max 650 ms) with SDO driven low, and releases SDO
// (pulled high by the iCE40 pull-up) when it is done.  `calibrating` is high from the strobe
// until SDO is seen high again; no conversion starts meanwhile.  The power-up self-calibration
// runs when AVDD comes up, possibly before the reference has settled, so the firmware sends one
// after boot (datasheet 7.1, first use case).  If SDO never goes low (the ADC
// did not take the command) the engine gives up after S_CAL's hold-off, ~43 us, so the host
// sees a calibration that took no time instead of a hung ADC.
// ============================================================================
module adc_mcp33131 (
    input  wire        clk,           // 24 MHz control clock
    input  wire        rst,
    input  wire        en,            // capture active (level; synchronised in top_v2)
    input  wire [15:0] divider,       // sample period in clk cycles
    input  wire        recal,         // 1-cycle strobe: send the recalibrate command (v49)
    output wire        calibrating,   // recalibration requested or running (STATUS bit 7)
    // ADC pins
    output reg         adc_cnvst,
    output reg         adc_sclk,
    output wire        adc_sdi,
    input  wire        adc_sdo,       // has the iCE40 pull-up (enabled in top_v2)
    // result
    output reg  [15:0] sample,
    output reg         sample_stb
);
    // SDI must be held HIGH (tied to DVIO) for normal 3-wire operation
    // (datasheet DS20005947B Fig 7-2 Note 1).
    assign adc_sdi = 1'b1;

    // Conversion wait: the MCP33131D-10 (1 MSPS) has tCNV = 710 ns; 24 cycles at
    // 24 MHz = 1.0 us gives ~40% margin.  Combined with the 16-bit read (32 cycles
    // @ 12 MHz SCLK) this makes one conversion ~59 clk, so the sample-period floor
    // below is 60 -> ~400 kS/s (was CONV_CYCLES=48 / floor 80 -> ~300 kS/s).
    localparam [7:0] CONV_CYCLES = 8'd24;   // ~1.0 us at 24 MHz (> tCNV 710 ns)
    localparam S_IDLE = 2'd0, S_CONV = 2'd1, S_READ = 2'd2, S_CAL = 2'd3;

    reg [1:0]  st;
    reg [15:0] period_cnt;
    reg        period_zero;       // registered "period expired": keeps the 16-bit
                                  // compare off the conversion-start transition path
    // One counter serves the conversion wait (S_CONV, to CONV_CYCLES), the bit count (S_READ,
    // to 16, or 1024 for a recalibrate) and S_CAL's hold-off: the states never overlap (v45; was
    // an 8-bit and a 5-bit counter).  10 bits since v49 for the 1024-clock recalibrate command.
    reg [9:0]  cnt;
    reg        conv_done;         // registered (cnt == CONV_CYCLES)
    reg        bits_done;         // registered (cnt == 16 in S_READ)
    reg        clks_done;         // registered (cnt == 1024 in S_READ): the recalibrate command is out
    reg [15:0] shreg;
    reg        sclk_ph;
    // Recalibrate (v49): pending from the strobe until SDO comes back high after tCAL.
    reg        cal_pend;
    reg  [1:0] sdo_s;             // SDO synchronizer for S_CAL (the read path samples it raw)
    assign calibrating = cal_pend;

    always @(posedge clk) begin
        sample_stb <= 1'b0;
        sdo_s <= {sdo_s[0], adc_sdo};
        if (rst) begin
            st <= S_IDLE; adc_cnvst <= 1'b0; adc_sclk <= 1'b0;
            period_cnt <= 16'd0; period_zero <= 1'b1; cnt <= 10'd0; sclk_ph <= 1'b0;
            cal_pend <= 1'b0;
        end else begin
            if (recal) cal_pend <= 1'b1;
            // Sample period = EXACTLY max(divider, 60) clocks.  period_cnt loads the
            // period at a conversion start and counts down; the registered flag is set
            // as the count goes 2 -> 1, so S_IDLE sees it `period` clocks after that
            // start.  (Gateware <= v31 set it at 1 -> 0 and the flag's register delay
            // made every period divider + 1 clocks.)  Safe because the load is >= 60.
            if (!period_zero) begin
                period_cnt  <= period_cnt - 16'd1;
                period_zero <= (period_cnt == 16'd2);
            end
            case (st)
            S_IDLE: begin
                adc_sclk <= 1'b0;
                // A pending recalibrate runs on the next conversion whether or not `en` is set.
                if ((en || cal_pend) && period_zero) begin
                    adc_cnvst   <= 1'b1;              // start conversion
                    cnt         <= 10'd0;
                    conv_done   <= 1'b0;
                    // Floor the sample period at the engine's own min duration
                    // (~59 clk: 1 idle + 24 conv + 32 read + transitions) so a small
                    // requested divider can't ask for a rate the FSM can't sustain.
                    // v40: no 60-clock floor here; the firmware floors the divider
                    // (adc_min_divider).  A shorter period cannot break the engine: the next
                    // conversion just starts as soon as this one has been read out.
                    period_cnt  <= divider;
                    period_zero <= 1'b0;
                    st          <= S_CONV;
                end
            end
            // conv_done/bits_done are registered one tick ahead of the count==N
            // compares so those compares stay off the FSM's transition paths.
            S_CONV: begin
                if (conv_done) begin
                    adc_cnvst <= 1'b0;              // end convert, begin read
                    cnt       <= 10'd0;
                    bits_done <= 1'b0;
                    clks_done <= 1'b0;
                    sclk_ph   <= 1'b0;
                    st        <= S_READ;
                end else begin
                    cnt       <= cnt + 10'd1;
                    conv_done <= (cnt[4:0] == CONV_CYCLES[4:0] - 5'd1);
                end
            end
            // Read 16 bits, MSB first. SDO changes on the SCLK falling edge; we
            // sample one clk cycle (~42 ns > tDO at 12 MHz SCLK) after the previous
            // falling edge, while SDO is stable.  A recalibrate keeps clocking to 1024.
            S_READ: begin
                sclk_ph <= ~sclk_ph;
                if (!sclk_ph) begin
                    adc_sclk  <= 1'b1;               // SCLK high next cycle
                    if (!bits_done) shreg <= {shreg[14:0], adc_sdo};
                    cnt       <= cnt + 10'd1;
                    if (cnt == 10'd15)   bits_done <= 1'b1;
                    if (cnt == 10'd1023) clks_done <= 1'b1;
                end else begin
                    adc_sclk <= 1'b0;               // falling edge
                    if (bits_done && cnt == 10'd16) begin
                        sample     <= shreg;
                        sample_stb <= 1'b1;
                    end
                    if (bits_done && !cal_pend) st <= S_IDLE;
                    if (clks_done) begin
                        cnt     <= 10'd0;
                        st      <= S_CAL;
                    end
                end
            end
            // Recalibrating: CNVST and SCLK low, wait for the ADC to release SDO.  The 1024-clock
            // hold-off covers the gap between the last SCLK and SDO going low (Hi-Z, pulled high,
            // in Figure 7-3); after it, a high SDO ends the wait whether or not it was ever low.
            S_CAL: begin
                if (cnt != 10'd1023) cnt <= cnt + 10'd1;
                else if (sdo_s[1]) begin
                    cal_pend <= 1'b0;
                    st       <= S_IDLE;
                end
            end
            endcase
        end
    end
endmodule
