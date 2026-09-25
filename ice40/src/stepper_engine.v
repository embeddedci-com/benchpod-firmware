// ============================================================================
// stepper_engine.v — non-blocking step-pulse generator (FPGA replacement for
// the RP2350's old stepper.pio).
//
// On a `start` pulse it drives the assigned LA channel HIGH for `delay_us`
// microseconds, then LOW for `delay_us` microseconds, repeated `steps` times,
// then drops `busy`.  Identical semantics to the old gpio_control_step():
// one full step = one HIGH half-phase + one LOW half-phase.
//
// Microsecond timing is exact (no PIO instruction-overhead fudge): a ÷CLK_MHZ
// prescaler turns the 24 MHz system clock into 1 MHz microsecond ticks.
//
// The engine only OWNS its channel while `busy` is high; la_bank mux gates
// step_active=busy so the pin returns to its static state when the train ends.
// ============================================================================

module stepper_engine #(
    parameter CLK_MHZ = 24    // system clock in MHz → microsecond prescaler
)(
    input  wire        clk,
    input  wire        rst,

    // control
    input  wire        start,        // 1-cycle pulse
    input  wire [3:0]  channel,      // LA channel to pulse (0-based)
    input  wire [15:0] steps,        // number of pulses (1..65535)
    input  wire [15:0] delay_us,     // v40: microseconds per half-phase MINUS ONE (0..65534);
                                     // the firmware sends delay - 1, so no subtract here

    // outputs to la_bank
    output reg         busy,
    output reg  [3:0]  step_channel,
    output reg         step_val
);

    localparam [4:0] PRESC_MAX = CLK_MHZ[4:0] - 5'd1;

    reg [4:0]  presc;
    reg [15:0] delay_lat;
    reg        phase;        // 0 = HIGH half-phase, 1 = LOW half-phase

    // v42: us_left (top, with the free != 0 flag) and steps_left (bottom) are the two halves of
    // one SB_MAC16 (dsp_counter2) instead of 32 fabric LCs plus a 16-bit zero test and the reload
    // muxes.  Same loads and steps, on the same edges, as the fabric counters they replace
    // (tb_stepper checks this engine against the v41 one, sim/stepper_engine_ref.v).  The DSP has
    // no reset; both counters are loaded when a train starts, before anything reads them.
    wire [15:0] us_left, steps_left;
    wire        us_nz;                                   // us_left != 0
    wire        fresh     = start && !busy && steps != 16'd0;
    wire        us_tick   = busy && presc == PRESC_MAX;  // one microsecond elapsed
    wire        half_done = us_tick && !us_nz;           // a half-phase ends this edge
    wire        last      = steps_left <= 16'd1;
    dsp_counter2 #(.UP_T(0), .UP_B(0)) cnt_i (
        .clk(clk),
        // us_left: the new train's delay, or delay_lat for the next half-phase (not after the last)
        .load_t(~rst & (fresh | (half_done & ~(phase & last)))),
        .val_t(fresh ? delay_us : delay_lat),
        .en_t(~rst & us_tick & us_nz), .qt(us_left), .flag(us_nz),
        // steps_left: one step done at the end of each LOW half-phase but the last
        .load_b(~rst & fresh), .val_b(steps),
        .en_b(~rst & half_done & phase & ~last), .qb(steps_left));

    always @(posedge clk) begin
        if (rst) begin
            busy         <= 1'b0;
            step_channel <= 4'd0;
            step_val     <= 1'b0;
            presc        <= 5'd0;
            delay_lat    <= 16'd0;
            phase        <= 1'b0;
        end else if (fresh) begin
            // Latch a fresh train (us_left and steps_left load in cnt_i).  delay_lat>=1
            // guaranteed by firmware clamp.
            busy         <= 1'b1;
            step_channel <= channel;
            step_val     <= 1'b1;             // first HIGH half-phase
            phase        <= 1'b0;
            delay_lat    <= delay_us;     // half-phase - 1 (v40 wire encoding)
            presc        <= 5'd0;
        end else if (busy) begin
            // microsecond prescaler
            if (us_tick) begin
                presc <= 5'd0;
                // one microsecond elapsed (us_left counts down in cnt_i while it is != 0)
                if (!half_done) begin
                end else if (phase == 1'b0) begin
                    // HIGH half-phase done → go LOW (us_left reloads delay_lat)
                    step_val <= 1'b0;
                    phase    <= 1'b1;
                end else begin
                    // LOW half-phase done → one full step complete
                    if (last) begin
                        busy     <= 1'b0;     // train finished
                        step_val <= 1'b0;
                    end else begin
                        // (steps_left steps down and us_left reloads in cnt_i)
                        step_val   <= 1'b1;   // next HIGH half-phase
                        phase      <= 1'b0;
                    end
                end
            end else begin
                presc <= presc + 5'd1;
            end
        end
    end

endmodule
