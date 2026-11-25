// ============================================================================
// jtag_seq.v: the JTAG master (v61), swd_engine's fourth job.
//
// Like i2c_master.v, it has its own small counters and shares only swd_engine's queue (tmem:
// `t_rd` / `qready`, `take` advances qptr) and reply buffer (`push` writes `pdata` at wptr).
//
// The queue holds CMSIS-DAP DAP_JTAG_Sequence sequences, exactly as the host sends them:
//   info byte: bits 5:0 = TCK cycles (0 means 64), bit 6 = TMS level for the whole sequence,
//              bit 7 = capture TDO
//   then ceil(n/8) TDI bytes, bit 0 first.
// A capturing sequence appends ceil(n/8) TDO bytes to the reply buffer, bit 0 first.  One shift
// register carries TDI out (bit 0) and TDO in (at the top), so in a last byte of k < 8 bits the
// TDO bits are its TOP k bits (the low bits are leftover TDI): the firmware shifts that byte
// right by 8 - k.  Otherwise this is DAP_JTAG_Sequence's request and response data as is.
//
// Bit timing, one TCK cycle = `half` clk low then `half` clk high: TDI and TMS change while TCK
// is low (at the falling edge, or when a sequence starts), TCK rises, and TDO is taken at the END
// of the high half.  The target changes TDO on the falling edge, so it has been stable for a whole
// cycle; through swd_engine's input flop the value used is the pin 1 clk before the falling edge
// (since v64 the pins lag the engine by 1 clk, la_bank; up to v63 2 flops and combinational pads).
// TCK rests LOW between sequences and while a TDI byte has not arrived yet (JTAG allows stopping
// the clock low); TMS and TDI keep their last level.
// ============================================================================

module jtag_seq (
    input  wire       clk,
    input  wire       rst,
    input  wire       on,          // armed in JTAG mode
    input  wire       arm,         // 1-cycle: (re)armed: idle, TCK low, TMS high, TDI high
    input  wire [6:0] half,        // clk per TCK half period
    input  wire       tdo_s,       // synchronised TDO
    input  wire [7:0] t_rd,        // the queue head (one edge after qptr moves)
    input  wire       qready,      // the queue holds a byte (one edge after qptr moves)
    output reg        take,        // 1-cycle: consume the queue head
    output reg        push,        // 1-cycle: write pdata to the reply buffer
    output wire [7:0] pdata,       // valid while push is high
    output wire       busy,        // a sequence is running
    output reg        tck,
    output reg        tms,
    output reg        tdi
);
    localparam IDLE = 2'd0, DATA = 2'd1, LOW = 2'd2, HIGH = 2'd3;
    reg [1:0] st;
    reg [6:0] hcnt;
    reg [6:0] n;                   // TCK cycles left in this sequence, 1..64
    reg [2:0] bi;                  // bit index in the current TDI/TDO byte
    reg [7:0] sh;                  // TDI out at bit 0, TDO in at bit 7
    reg       cap;                 // this sequence captures TDO
    reg       tms_n;               // this sequence's TMS
    reg [1:0] w;                   // t_rd / qready follow `take` two edges later (take is registered)

    wire hz     = (hcnt == 7'd0);
    wire last   = (n == 7'd1);

    assign busy  = (st != IDLE);
    assign pdata = sh;             // push follows the shift that completed the byte

    always @(posedge clk) begin
        take <= 1'b0;
        push <= 1'b0;
        if (rst || !on || arm) begin
            st <= IDLE; tck <= 1'b0; w <= 2'b00;
            if (rst || arm) begin tms <= 1'b1; tdi <= 1'b1; end
        end else begin
            w <= {1'b0, w[1]};
            if (!hz) hcnt <= hcnt - 7'd1;
            case (st)
            IDLE: if (qready && w == 2'b00) begin              // the info byte
                    n     <= (t_rd[5:0] == 6'd0) ? 7'd64 : {1'b0, t_rd[5:0]};
                    tms_n <= t_rd[6];
                    cap   <= t_rd[7];
                    take  <= 1'b1; w <= 2'b11;
                    st    <= DATA;
                  end
            DATA: if (qready && w == 2'b00) begin              // the next TDI byte; TCK is low
                    sh   <= t_rd;
                    tdi  <= t_rd[0];
                    tms  <= tms_n;
                    bi   <= 3'd0;
                    take <= 1'b1; w <= 2'b11;
                    hcnt <= half - 7'd1;
                    st   <= LOW;
                  end
            LOW:  if (hz) begin
                    tck  <= 1'b1;
                    hcnt <= half - 7'd1;
                    st   <= HIGH;
                  end
            HIGH: if (hz) begin                                 // TDO in, then the falling edge
                    tck  <= 1'b0;
                    n    <= n - 7'd1;
                    bi   <= bi + 3'd1;
                    sh   <= {tdo_s, sh[7:1]};
                    if ((bi == 3'd7 || last) && cap) push <= 1'b1;
                    if (last)              st <= IDLE;
                    else if (bi == 3'd7)   st <= DATA;
                    else begin
                        tdi  <= sh[1];
                        hcnt <= half - 7'd1;
                        st   <= LOW;
                    end
                  end
            endcase
        end
    end
endmodule
