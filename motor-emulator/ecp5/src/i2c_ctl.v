// ============================================================================
// i2c_ctl.v — the board's I2C bus: ID EEPROM (M24C02 at 0x50-0x53) and the trip-recorder
// expander (PCAL6408A at 0x20), about 100 kHz, open drain.
//
// Jobs, one at a time, highest priority first:
//   probe     after reset and on `probe_start`: the first of 0x50-0x53 that ACKs is this
//             board's stack position (board_id); none: id 0, ok = 0. Then pcal_init.
//   pcal_init input latch on P0-P6 (register 0x42 = 0x7F), interrupts unmasked (0x45 = 0x00),
//             then one read of the input port to clear the latches.
//   pcal_read on `trip_req` (the top level asks on FAULT and on a link request): read the input
//             port (0x00). With the latches on it returns the state each input changed to, held
//             until this read: which trip fired. trip_src, trip_valid.
//   ee_write  `ee_wr` with ee_addr / ee_wdata: WC low, write the byte, ACK-poll the write cycle
//             (up to 255 polls, about 23 ms), WC high.
//   ee_read   `ee_rd` with ee_addr: random read into ee_rdata.
// `busy` while a job runs; `err` (sticky until the next EEPROM or expander job starts) when a
// device NACKed where an ACK was needed. A request that arrives while busy waits its turn.
// ============================================================================
`default_nettype none
module i2c_ctl #(
    parameter QUARTER = 90                  // clk per quarter bit: 36 MHz / (4 x 90) = 100 kHz
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       sda_in,
    output reg        scl_oe,
    output reg        sda_oe,
    output reg        wc_low,               // EEPROM write enable (WC pin low)
    // probe
    input  wire       probe_start,
    output reg  [1:0] board_id,
    output reg        done,
    output reg        ok,
    // trip recorder
    input  wire       trip_req,
    output reg  [7:0] trip_src,
    output reg        trip_valid,
    output reg        pcal_ok,
    // EEPROM
    input  wire       ee_wr,
    input  wire       ee_rd,
    input  wire [7:0] ee_addr,
    input  wire [7:0] ee_wdata,
    output reg  [7:0] ee_rdata,
    output wire       busy,
    output reg        err
);
    reg [1:0] sda_s;
    always @(posedge clk) sda_s <= {sda_s[0], sda_in};
    reg [7:0] qcnt;
    wire tick = (qcnt == QUARTER - 1);
    always @(posedge clk) qcnt <= (rst || tick) ? 8'd0 : qcnt + 8'd1;

    // ---- step table: {op, src, const}
    localparam OP_END = 3'd0, OP_START = 3'd1, OP_WR = 3'd2, OP_RDA = 3'd3, OP_RDN = 3'd4,
               OP_STOP = 3'd5, OP_POLL = 3'd6;
    localparam S_K = 3'd0, S_EEW = 3'd1, S_EER = 3'd2, S_EEA = 3'd3, S_EED = 3'd4, S_PRB = 3'd5;
    localparam J_NONE = 3'd0, J_PROBE = 3'd1, J_PINIT = 3'd2, J_PREAD = 3'd3, J_EEW = 3'd4, J_EER = 3'd5;
    reg [2:0] job;
    reg [4:0] sidx;
    reg [7:0] ee_addr_q, ee_wdata_q;
    reg [13:0] step;
    always @(*) begin
        step = {OP_END, S_K, 8'h00};
        case (job)
            J_PROBE: case (sidx)
                5'd0: step = {OP_START, S_K, 8'h00};
                5'd1: step = {OP_WR, S_PRB, 8'h00};
                5'd2: step = {OP_STOP, S_K, 8'h00};
                default: ;
            endcase
            J_PINIT: case (sidx)
                5'd0: step = {OP_START, S_K, 8'h00};  5'd1: step = {OP_WR, S_K, 8'h40};
                5'd2: step = {OP_WR, S_K, 8'h42};     5'd3: step = {OP_WR, S_K, 8'h7F};
                5'd4: step = {OP_STOP, S_K, 8'h00};
                5'd5: step = {OP_START, S_K, 8'h00};  5'd6: step = {OP_WR, S_K, 8'h40};
                5'd7: step = {OP_WR, S_K, 8'h45};     5'd8: step = {OP_WR, S_K, 8'h00};
                5'd9: step = {OP_STOP, S_K, 8'h00};
                5'd10: step = {OP_START, S_K, 8'h00}; 5'd11: step = {OP_WR, S_K, 8'h40};
                5'd12: step = {OP_WR, S_K, 8'h00};    5'd13: step = {OP_START, S_K, 8'h00};
                5'd14: step = {OP_WR, S_K, 8'h41};    5'd15: step = {OP_RDN, S_K, 8'h00};
                5'd16: step = {OP_STOP, S_K, 8'h00};
                default: ;
            endcase
            J_PREAD: case (sidx)
                5'd0: step = {OP_START, S_K, 8'h00};  5'd1: step = {OP_WR, S_K, 8'h40};
                5'd2: step = {OP_WR, S_K, 8'h00};     5'd3: step = {OP_START, S_K, 8'h00};
                5'd4: step = {OP_WR, S_K, 8'h41};     5'd5: step = {OP_RDN, S_K, 8'h00};
                5'd6: step = {OP_STOP, S_K, 8'h00};
                default: ;
            endcase
            J_EEW: case (sidx)
                5'd0: step = {OP_START, S_K, 8'h00};  5'd1: step = {OP_WR, S_EEW, 8'h00};
                5'd2: step = {OP_WR, S_EEA, 8'h00};   5'd3: step = {OP_WR, S_EED, 8'h00};
                5'd4: step = {OP_STOP, S_K, 8'h00};
                5'd5: step = {OP_START, S_K, 8'h00};  5'd6: step = {OP_WR, S_EEW, 8'h00};   // ACK poll
                5'd7: step = {OP_POLL, S_K, 8'h05};   // no ACK: STOP, back to step 5
                5'd8: step = {OP_STOP, S_K, 8'h00};
                default: ;
            endcase
            J_EER: case (sidx)
                5'd0: step = {OP_START, S_K, 8'h00};  5'd1: step = {OP_WR, S_EEW, 8'h00};
                5'd2: step = {OP_WR, S_EEA, 8'h00};   5'd3: step = {OP_START, S_K, 8'h00};
                5'd4: step = {OP_WR, S_EER, 8'h00};   5'd5: step = {OP_RDN, S_K, 8'h00};
                5'd6: step = {OP_STOP, S_K, 8'h00};
                default: ;
            endcase
            default: ;
        endcase
    end
    wire [2:0] s_op  = step[13:11];
    wire [2:0] s_src = step[10:8];
    wire [7:0] s_k   = step[7:0];
    reg  [1:0] pidx;                        // probe address 0x50 + pidx
    wire [7:0] tx_of = (s_src == S_EEW) ? {5'b10100, board_id, 1'b0} :
                       (s_src == S_EER) ? {5'b10100, board_id, 1'b1} :
                       (s_src == S_EEA) ? ee_addr_q :
                       (s_src == S_EED) ? ee_wdata_q :
                       (s_src == S_PRB) ? {5'b10100, pidx, 1'b0} : s_k;

    // ---- requests
    reg p_probe, p_pinit, p_pread, p_eew, p_eer;
    assign busy = (job != J_NONE) | p_probe | p_pinit | p_pread | p_eew | p_eer;

    // ---- engine
    localparam E_IDLE = 3'd0, E_START = 3'd1, E_BIT = 3'd2, E_STOP = 3'd3, E_NEXT = 3'd4;
    reg [2:0] es;
    reg [1:0] q;
    reg [3:0] bitn;
    reg [7:0] sr;
    reg       rd_mode, rd_nack, acked, failed;
    reg [7:0] polls;

    always @(posedge clk) begin
        if (rst) begin
            es <= E_IDLE; job <= J_NONE; sidx <= 5'd0; q <= 2'd0;
            scl_oe <= 1'b0; sda_oe <= 1'b0; wc_low <= 1'b0;
            board_id <= 2'd0; done <= 1'b0; ok <= 1'b0; pcal_ok <= 1'b0;
            trip_valid <= 1'b0; err <= 1'b0;
            p_probe <= 1'b1; p_pinit <= 1'b0; p_pread <= 1'b0; p_eew <= 1'b0; p_eer <= 1'b0;
        end else begin
            if (probe_start) p_probe <= 1'b1;
            if (trip_req) p_pread <= 1'b1;
            if (ee_wr) begin p_eew <= 1'b1; ee_addr_q <= ee_addr; ee_wdata_q <= ee_wdata; end
            if (ee_rd) begin p_eer <= 1'b1; ee_addr_q <= ee_addr; end
            case (es)
                E_IDLE: begin
                    if (job == J_NONE) begin
                        scl_oe <= 1'b0; sda_oe <= 1'b0;             // between jobs the bus is free
                        sidx <= 5'd0; failed <= 1'b0; polls <= 8'd0;
                        if (p_probe) begin
                            p_probe <= 1'b0; job <= J_PROBE; pidx <= 2'd0;
                            done <= 1'b0; ok <= 1'b0; board_id <= 2'd0;
                        end else if (p_pinit) begin p_pinit <= 1'b0; job <= J_PINIT; err <= 1'b0; end
                        else if (p_pread) begin p_pread <= 1'b0; job <= J_PREAD; err <= 1'b0; end
                        else if (p_eew)   begin p_eew <= 1'b0; job <= J_EEW; err <= 1'b0; wc_low <= 1'b1; end
                        else if (p_eer)   begin p_eer <= 1'b0; job <= J_EER; err <= 1'b0; end
                    end else if (tick) begin
                        q <= 2'd0;
                        case (s_op)
                            OP_START: es <= E_START;
                            OP_WR:    begin es <= E_BIT; sr <= tx_of; rd_mode <= 1'b0; bitn <= 4'd0; end
                            OP_RDA, OP_RDN: begin es <= E_BIT; rd_mode <= 1'b1; rd_nack <= (s_op == OP_RDN); bitn <= 4'd0; end
                            OP_STOP:  es <= E_STOP;
                            OP_POLL:  begin
                                if (acked || polls == 8'd255) begin
                                    sidx <= sidx + 5'd1;
                                    if (!acked) failed <= 1'b1;
                                end else begin polls <= polls + 8'd1; es <= E_STOP; sidx <= s_k[4:0] - 5'd1; end
                            end
                            default: es <= E_NEXT;          // END
                        endcase
                    end
                end
                // (SCL may be low after a byte) SDA up, SCL up, SDA down, SCL down
                E_START: if (tick) begin
                    q <= q + 2'd1;
                    case (q)
                        2'd0: sda_oe <= 1'b0;
                        2'd1: scl_oe <= 1'b0;
                        2'd2: sda_oe <= 1'b1;
                        2'd3: begin scl_oe <= 1'b1; es <= E_IDLE; sidx <= sidx + 5'd1; end
                    endcase
                end
                E_BIT: if (tick) begin
                    q <= q + 2'd1;
                    case (q)
                        2'd0: sda_oe <= (bitn == 4'd8) ? (rd_mode ? ~rd_nack : 1'b0)
                                                      : (rd_mode ? 1'b0 : ~sr[7]);
                        2'd1: scl_oe <= 1'b0;
                        2'd2: if (bitn == 4'd8) begin
                                  if (!rd_mode) acked <= ~sda_s[1];
                              end else begin
                                  sr <= {sr[6:0], rd_mode ? sda_s[1] : 1'b0};
                              end
                        2'd3: begin
                            scl_oe <= 1'b1;
                            if (bitn != 4'd8) bitn <= bitn + 4'd1;
                            else begin
                                sda_oe <= 1'b0;
                                if (rd_mode) begin
                                    if (job == J_PREAD) begin trip_src <= sr; trip_valid <= 1'b1; end
                                    if (job == J_EER) ee_rdata <= sr;
                                    es <= E_IDLE; sidx <= sidx + 5'd1;
                                end else if (!acked && job != J_PROBE && !(job == J_EEW && sidx == 5'd6)) begin
                                    failed <= 1'b1; es <= E_STOP; sidx <= 5'd30;    // abort: STOP, then END
                                end else begin es <= E_IDLE; sidx <= sidx + 5'd1; end
                            end
                        end
                    endcase
                end
                // SDA low (SCL low), SCL up, SDA up, then a quarter of bus-free time
                E_STOP: if (tick) begin
                    q <= q + 2'd1;
                    case (q)
                        2'd0: sda_oe <= 1'b1;
                        2'd1: scl_oe <= 1'b0;
                        2'd2: sda_oe <= 1'b0;
                        2'd3: begin es <= E_IDLE; sidx <= sidx + 5'd1; end
                    endcase
                end
                default: begin                          // E_NEXT: the job is over
                    es <= E_IDLE;
                    case (job)
                        J_PROBE: begin
                            if (acked) begin board_id <= pidx; ok <= 1'b1; done <= 1'b1; job <= J_NONE; p_pinit <= 1'b1; end
                            else if (pidx == 2'd3) begin board_id <= 2'd0; ok <= 1'b0; done <= 1'b1; job <= J_NONE; p_pinit <= 1'b1; end
                            else begin pidx <= pidx + 2'd1; sidx <= 5'd0; end
                        end
                        J_PINIT: begin pcal_ok <= !failed; err <= failed; job <= J_NONE; end
                        J_EEW:   begin wc_low <= 1'b0; err <= failed; job <= J_NONE; end
                        default: begin err <= failed; job <= J_NONE; end
                    endcase
                end
            endcase
        end
    end
endmodule
`default_nettype wire
