// ============================================================================
// i2c_probe.v — find this board's stack position from its ID EEPROM (M24C02).
//
// The EEPROM's E0/E1 pins are solder jumpers (E2 grounded), so it answers on one of 0x50-0x53.
// After reset (and on `start`) this sends START, address + W, and checks the ACK for each of the
// four addresses in turn, ~100 kHz SCL. The first one that ACKs gives board_id = address[1:0].
// If none answers, board_id stays 0 and ok = 0 (a board without an EEPROM behaves as board 0).
//
// Open drain: scl_oe / sda_oe = 1 pulls the line low; the 4.7 k pull-ups release it. SDA is
// sampled in the middle of the SCL-high quarter of the ACK bit. The PCAL6408A (0x20) shares the
// bus and never answers these addresses.
// ============================================================================
`default_nettype none

module i2c_probe #(
    parameter QUARTER = 90                  // clk per quarter bit: 36 MHz / (4 x 90) = 100 kHz
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       start,                // re-probe (one clk strobe); also runs after reset
    input  wire       sda_in,
    output reg        scl_oe,
    output reg        sda_oe,
    output reg  [1:0] board_id,
    output reg        done,
    output reg        ok
);
    reg [1:0] sda_s;
    always @(posedge clk) sda_s <= {sda_s[0], sda_in};

    localparam S_IDLE = 3'd0, S_START = 3'd1, S_BITS = 3'd2, S_STOP = 3'd3, S_GAP = 3'd4, S_NEXT = 3'd5;
    reg [2:0]  st;
    reg [1:0]  q;                          // quarter within the current step
    reg [7:0]  qcnt;
    reg [3:0]  bitn;                       // 0..8 (8 = ACK)
    reg [1:0]  idx;                        // address being probed: 0x50 + idx
    reg        acked;
    reg        pending;

    wire [7:0] addr_w = {5'b10100, idx, 1'b0};  // 7-bit 0x50+idx, R/W = 0
    wire       tick   = (qcnt == QUARTER - 1);

    always @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE; q <= 2'd0; qcnt <= 8'd0; bitn <= 4'd0; idx <= 2'd0;
            scl_oe <= 1'b0; sda_oe <= 1'b0;
            board_id <= 2'd0; done <= 1'b0; ok <= 1'b0; acked <= 1'b0;
            pending <= 1'b1;                    // probe once after reset
        end else begin
            if (start) pending <= 1'b1;
            qcnt <= tick ? 8'd0 : qcnt + 8'd1;
            case (st)
                S_IDLE: begin
                    scl_oe <= 1'b0; sda_oe <= 1'b0;
                    if (pending && tick) begin
                        pending <= 1'b0; done <= 1'b0; ok <= 1'b0; board_id <= 2'd0;
                        idx <= 2'd0; st <= S_START; q <= 2'd0;
                    end
                end
                // bus idle -> SDA low (START) -> SCL low
                S_START: if (tick) begin
                    q <= q + 2'd1;
                    case (q)
                        2'd0: begin scl_oe <= 1'b0; sda_oe <= 1'b0; acked <= 1'b0; end
                        2'd1: sda_oe <= 1'b1;
                        2'd2: begin scl_oe <= 1'b1; q <= 2'd0; bitn <= 4'd0; st <= S_BITS; end
                        default: ;
                    endcase
                end
                // each bit: set SDA with SCL low, raise SCL, sample, lower SCL
                S_BITS: if (tick) begin
                    q <= q + 2'd1;
                    case (q)
                        2'd0: sda_oe <= (bitn == 4'd8) ? 1'b0 : ~addr_w[7 - bitn[2:0]];
                        2'd1: scl_oe <= 1'b0;
                        2'd2: if (bitn == 4'd8) acked <= ~sda_s[1];
                        2'd3: begin
                            scl_oe <= 1'b1;
                            if (bitn == 4'd8) st <= S_STOP; else bitn <= bitn + 4'd1;
                        end
                    endcase
                end
                // SDA low with SCL low, SCL up, SDA up (STOP)
                S_STOP: if (tick) begin
                    q <= q + 2'd1;
                    case (q)
                        2'd0: sda_oe <= 1'b1;
                        2'd1: scl_oe <= 1'b0;
                        2'd2: begin sda_oe <= 1'b0; q <= 2'd0; st <= S_GAP; end
                        default: ;
                    endcase
                end
                S_GAP: if (tick) begin
                    q <= q + 2'd1;
                    if (q == 2'd3) st <= S_NEXT;
                end
                S_NEXT: begin
                    q <= 2'd0;
                    if (acked) begin
                        board_id <= idx; ok <= 1'b1; done <= 1'b1; st <= S_IDLE;
                    end else if (idx == 2'd3) begin
                        board_id <= 2'd0; ok <= 1'b0; done <= 1'b1; st <= S_IDLE;
                    end else begin
                        idx <= idx + 2'd1; st <= S_START;
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
