// ============================================================================
// stepper_engine_ref.v — the v41 all-fabric stepper_engine, kept as the reference model for
// tb_stepper: v42 moved its two counters into an SB_MAC16 (dsp_counter2), and the bench checks
// the new engine against this one cycle for cycle.  Not synthesized.
// ============================================================================

module stepper_engine_ref #(
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
    reg [15:0] us_left;
    reg [15:0] delay_lat;
    reg [15:0] steps_left;
    reg        phase;        // 0 = HIGH half-phase, 1 = LOW half-phase

    always @(posedge clk) begin
        if (rst) begin
            busy         <= 1'b0;
            step_channel <= 4'd0;
            step_val     <= 1'b0;
            presc        <= 5'd0;
            us_left      <= 16'd0;
            delay_lat    <= 16'd0;
            steps_left   <= 16'd0;
            phase        <= 1'b0;
        end else if (start && !busy && steps != 16'd0) begin
            // Latch a fresh train.  delay_lat>=1 guaranteed by firmware clamp.
            busy         <= 1'b1;
            step_channel <= channel;
            step_val     <= 1'b1;             // first HIGH half-phase
            phase        <= 1'b0;
            delay_lat    <= delay_us;     // half-phase - 1 (v40 wire encoding)
            us_left      <= delay_us;
            steps_left   <= steps;
            presc        <= 5'd0;
        end else if (busy) begin
            // microsecond prescaler
            if (presc == PRESC_MAX) begin
                presc <= 5'd0;
                // one microsecond elapsed
                if (us_left != 16'd0) begin
                    us_left <= us_left - 16'd1;
                end else if (phase == 1'b0) begin
                    // HIGH half-phase done → go LOW
                    step_val <= 1'b0;
                    phase    <= 1'b1;
                    us_left  <= delay_lat;
                end else begin
                    // LOW half-phase done → one full step complete
                    if (steps_left <= 16'd1) begin
                        busy     <= 1'b0;     // train finished
                        step_val <= 1'b0;
                    end else begin
                        steps_left <= steps_left - 16'd1;
                        step_val   <= 1'b1;   // next HIGH half-phase
                        phase      <= 1'b0;
                        us_left    <= delay_lat;
                    end
                end
            end else begin
                presc <= presc + 5'd1;
            end
        end
    end

endmodule
