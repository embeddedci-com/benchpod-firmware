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
// there is no crossing (post single-clock collapse; see docs/adc-capture-cdc-review.md).
// Verified: a DAC swept into the ADC input tracks linearly with ~2 LSB noise.
// ============================================================================
module adc_mcp33131 (
    input  wire        clk,           // 24 MHz control clock
    input  wire        rst,
    input  wire        en,            // capture active (level; synchronised in top_v2)
    input  wire [15:0] divider,       // sample period in clk cycles
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
    localparam S_IDLE = 2'd0, S_CONV = 2'd1, S_READ = 2'd2;

    reg [1:0]  st;
    reg [15:0] period_cnt;
    reg        period_zero;       // registered "period expired": keeps the 16-bit
                                  // compare off the conversion-start transition path
    reg [7:0]  conv_cnt;
    reg        conv_done;         // registered (conv_cnt == CONV_CYCLES)
    reg [4:0]  bit_cnt;
    reg        bits_done;         // registered (bit_cnt == 16)
    reg [15:0] shreg;
    reg        sclk_ph;

    always @(posedge clk) begin
        sample_stb <= 1'b0;
        if (rst) begin
            st <= S_IDLE; adc_cnvst <= 1'b0; adc_sclk <= 1'b0;
            period_cnt <= 16'd0; period_zero <= 1'b1; bit_cnt <= 5'd0; sclk_ph <= 1'b0;
        end else begin
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
                if (en && period_zero) begin
                    adc_cnvst   <= 1'b1;              // start conversion
                    conv_cnt    <= 8'd0;
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
                    bit_cnt   <= 5'd0;
                    bits_done <= 1'b0;
                    sclk_ph   <= 1'b0;
                    st        <= S_READ;
                end else begin
                    conv_cnt  <= conv_cnt + 8'd1;
                    conv_done <= (conv_cnt == CONV_CYCLES - 8'd1);
                end
            end
            // Read 16 bits, MSB first. SDO changes on the SCLK falling edge; we
            // sample one clk cycle (~42 ns > tDO at 12 MHz SCLK) after the previous
            // falling edge, while SDO is stable.
            S_READ: begin
                sclk_ph <= ~sclk_ph;
                if (!sclk_ph) begin
                    adc_sclk  <= 1'b1;               // SCLK high next cycle
                    shreg     <= {shreg[14:0], adc_sdo};
                    bit_cnt   <= bit_cnt + 5'd1;
                    bits_done <= (bit_cnt == 5'd15);
                end else begin
                    adc_sclk <= 1'b0;               // falling edge
                    if (bits_done) begin
                        sample     <= shreg;
                        sample_stb <= 1'b1;
                        st         <= S_IDLE;
                    end
                end
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
