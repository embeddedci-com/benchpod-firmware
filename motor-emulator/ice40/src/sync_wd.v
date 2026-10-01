// ============================================================================
// sync_wd.v — stack SYNC line: 1 ms time base and the stack watchdog.
//
// SYNC is one open-drain line shared by the stack (4.7 k pull-up per board). The design doc:
//  - board 0 sends a 1 us low pulse every 1 ms, so every motor model shares one time base;
//  - a board whose gateware faults holds SYNC low; any board seeing it low for more than 100 us
//    trips and stops its bridge (they share one DUT bus).
//
// master   : emit the pulse (the top level allows it on board 0 only)
// hold_low : this board's own gateware fault -> hold SYNC low (the whole stack stops)
// required : stack mode; missing pulses (none for 3 ms) also trip
// tick     : one-clk strobe on each received pulse's rising edge (the shared time base)
// trip     : sticky until `clear`
// ============================================================================
`default_nettype none

module sync_wd #(
    parameter PERIOD   = 36000,    // 1 ms at 36 MHz
    parameter PULSE    = 36,       // 1 us
    parameter LOW_TRIP = 3600,     // 100 us held low
    parameter MISSING  = 108000    // 3 ms without a pulse
) (
    input  wire clk,
    input  wire rst,
    input  wire master,
    input  wire hold_low,
    input  wire required,
    input  wire clear,
    input  wire sync_in,
    output reg  sync_oe,           // 1 pulls SYNC low
    output reg  tick,
    output reg  seen,              // a pulse arrived within the last 3 ms
    output reg  trip
);
    reg [1:0]  s;
    reg [15:0] pcnt;
    reg [11:0] low_cnt;
    reg [16:0] miss_cnt;
    always @(posedge clk) s <= {s[0], sync_in};
    wire rise = s[0] & ~s[1];

    always @(posedge clk) begin
        tick <= 1'b0;
        if (rst) begin
            pcnt <= 16'd0; low_cnt <= 12'd0; miss_cnt <= 17'd0;
            sync_oe <= 1'b0; seen <= 1'b0; trip <= 1'b0;
        end else begin
            // generator (master only)
            pcnt <= (pcnt == PERIOD - 1) ? 16'd0 : pcnt + 16'd1;
            sync_oe <= hold_low | (master & (pcnt < PULSE));

            // watchdog: SYNC held low too long (not counting our own pulse)
            if (s[1]) low_cnt <= 12'd0;
            else if (low_cnt != LOW_TRIP) low_cnt <= low_cnt + 12'd1;

            if (rise) begin
                tick <= 1'b1;
                miss_cnt <= 17'd0;
                seen <= 1'b1;
            end else if (miss_cnt != MISSING) begin
                miss_cnt <= miss_cnt + 17'd1;
            end else begin
                seen <= 1'b0;
            end

            if (clear) trip <= 1'b0;
            else if (low_cnt == LOW_TRIP || (required && miss_cnt == MISSING)) trip <= 1'b1;
        end
    end
endmodule
`default_nettype wire
