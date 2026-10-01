// i2c_dev_model — bench models of the board's I2C targets (behavioural, open drain).
//   KIND 0: M24C02 EEPROM at addr7. Byte write (address, data) when WC is low; a write cycle of
//           WRITE_NS during which it NACKs its address (ACK polling); random read.
//   KIND 1: PCAL6408A at addr7. Register writes (pointer, data); reads from the pointer. The input
//           port (0x00) returns, for latched pins (0x42), the state a pin changed to, held until
//           that register is read.
`timescale 1ns/1ps
module i2c_dev_model #(parameter KIND = 0, parameter WRITE_NS = 3000000) (
    inout  wire       sda,
    input  wire       scl,
    input  wire [6:0] addr7,
    input  wire       present,
    input  wire       wc_n,
    input  wire [7:0] pins
);
    reg pull = 0;
    assign sda = pull ? 1'b0 : 1'bz;
    reg [7:0] mem [0:255];
    reg [7:0] regs [0:255];
    integer i;
    initial for (i = 0; i < 256; i = i + 1) begin mem[i] = i[7:0] ^ 8'h5A; regs[i] = 8'h00; end
    reg [7:0] ptr = 0, sr = 0, tx = 0, wdat = 0;
    integer nbit = 0, nbyte = 0;
    reg active = 0, mine = 0, reading = 0, ackbit = 0, drive_tx = 0;
    realtime busy_until = 0;
    // PCAL input latch
    reg [7:0] held = 8'h00, last = 8'h00, hvalid = 8'h00;
    always @(pins) for (i = 0; i < 8; i = i + 1)
        if (regs[8'h42][i] && !hvalid[i] && pins[i] !== last[i]) begin held[i] = pins[i]; hvalid[i] = 1; end
    function [7:0] in_port(input dummy);
        integer k; begin for (k = 0; k < 8; k = k + 1) in_port[k] = hvalid[k] ? held[k] : pins[k]; end
    endfunction

    always @(negedge sda) if (scl === 1'b1) begin active = 1; nbit = 0; nbyte = 0; mine = 0; reading = 0; ackbit = 0; drive_tx = 0; pull = 0; end
    always @(posedge sda) if (scl === 1'b1) begin
        if (active && mine && !reading && KIND == 0 && nbyte == 3 && !wc_n) begin
            mem[ptr] = wdat; busy_until = $realtime + WRITE_NS; ptr = ptr + 1;
        end
        active = 0; pull = 0;
    end
    always @(posedge scl) if (active && !ackbit) begin
        if (!drive_tx) sr = {sr[6:0], (sda === 1'b0) ? 1'b0 : 1'b1};
        nbit = nbit + 1;
    end else if (active && ackbit && reading && drive_tx) begin
        if (sda !== 1'b0) begin drive_tx = 0; end            // master NACK: stop sending
    end
    always @(negedge scl) if (active) begin
        if (ackbit) begin
            ackbit = 0; pull = 0;
            if (reading && drive_tx) begin
                tx = (KIND == 0) ? mem[ptr] : ((ptr == 0) ? in_port(0) : regs[ptr]);
                if (KIND == 1 && ptr == 0) begin hvalid = 0; last = pins; end
                ptr = ptr + 1; nbit = 0; pull = ~tx[7];
            end
        end else if (drive_tx) begin
            if (nbit == 8) begin pull = 0; ackbit = 1; end
            else pull = ~tx[7 - nbit];
        end else if (nbit == 8) begin
            nbit = 0; nbyte = nbyte + 1;
            if (nbyte == 1) begin
                mine = present && sr[7:1] == addr7 && !(KIND == 0 && $realtime < busy_until);
                reading = sr[0];
                if (mine) pull = 1;
                if (mine && reading) drive_tx = 1;
            end else if (mine && !reading) begin
                pull = 1;
                if (nbyte == 2) ptr = sr;
                else if (KIND == 0) wdat = sr;          // committed at STOP
                else if (KIND == 1) begin regs[ptr] = sr; if (ptr == 8'h42) begin last = pins; hvalid = 0; end ptr = ptr + 1; end
            end
            ackbit = mine;
            if (!mine) active = 0;
        end
    end
endmodule
