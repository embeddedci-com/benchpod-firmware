// i2c_ack_model — bench model of an I2C target that only ACKs its address byte (enough for the
// board-id probe). `addr7` can be changed between runs; `present` = 0 removes it from the bus.
// Open drain: it only ever pulls SDA low. The bench supplies the pull-ups.
`timescale 1ns/1ps
module i2c_ack_model (
    inout  wire       sda,
    input  wire       scl,
    input  wire [6:0] addr7,
    input  wire       present
);
    reg       pull = 1'b0;
    reg [3:0] nbit = 0;
    reg [7:0] sr = 0;
    reg       active = 0;
    reg       ack_phase = 0;
    assign sda = pull ? 1'b0 : 1'bz;

    // START: SDA falls while SCL high; STOP: SDA rises while SCL high
    always @(negedge sda) if (scl === 1'b1) begin active = 1; nbit = 0; ack_phase = 0; pull = 0; end
    always @(posedge sda) if (scl === 1'b1) begin active = 0; pull = 0; end

    always @(posedge scl) if (active && !ack_phase) begin
        sr = {sr[6:0], (sda === 1'b0) ? 1'b0 : 1'b1};
        nbit = nbit + 1;
    end
    always @(negedge scl) if (active) begin
        if (ack_phase) begin
            pull = 0; ack_phase = 0; active = 0;       // only the address byte matters here
        end else if (nbit == 8) begin
            if (present && sr[7:1] == addr7 && sr[0] == 1'b0) pull = 1;
            ack_phase = 1;
        end
    end
endmodule
