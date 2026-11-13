// ============================================================================
// i2c_master.v: the I2C master (v55), swd_engine's third job.
//
// Kept out of swd_engine's sequencer on purpose: sharing that sequencer's registers (hcnt, idx,
// sh, qptr) put a wide priority mux on every one of them and cost ~330 LUTs; its own small
// counters cost far less.  It shares only swd_engine's queue (tmem: `t_rd` / `qready`, `take`
// advances qptr) and reply buffer (`push` writes `pdata` at wptr).
//
// Tokens (one queue byte each, low 3 bits):
//   1 START (also repeated START)   2 STOP
//   3 WRITE: the NEXT queue byte is the data; the reply buffer gets {7'b0, nack}
//   4 READ, then ACK                5 READ, then NACK; the reply buffer gets the byte
// A bit is `half` clk with SCL low (SDA changes 1 clk after SCL falls), then SCL released: the
// high half counts only once SCL reads high, so a target may stretch it (the firmware times out a
// transfer that never ends).  SDA is sampled at the end of the high half.  Between tokens SCL rests
// low, except after STOP (bus idle).  A WRITE bit sent as 1 that reads 0 is a lost arbitration:
// both lines are released, the queue stops and `arb` is set until the next arm.
// ============================================================================

module i2c_master (
    input  wire       clk,
    input  wire       rst,
    input  wire       on,          // armed in I2C mode
    input  wire       arm,         // 1-cycle: (re)armed, release the bus and clear the flags
    input  wire [6:0] half,        // clk per half bit
    input  wire       sda_s,       // synchronised SDA
    input  wire       scl_s,       // synchronised SCL
    input  wire [7:0] t_rd,        // the queue head (one edge after qptr moves)
    input  wire       qready,      // the queue holds a byte (one edge after qptr moves)
    output reg        take,        // 1-cycle: consume the queue head
    output reg        push,        // 1-cycle: write pdata to the reply buffer
    output reg  [7:0] pdata,
    output wire       busy,        // a token is running
    output reg        arb,         // arbitration lost (stops the queue)
    output reg        scl_rel,     // 1 = SCL released
    output reg        sda_rel      // 1 = SDA released
);
    localparam IDLE = 2'd0, DATA = 2'd1, LOW = 2'd2, HIGH = 2'd3;
    reg [1:0] st;
    reg [6:0] hcnt;
    reg [3:0] idx;                 // bit 0..8 of a byte token, step 0..1 of START / STOP
    reg [7:0] sh;
    reg [2:0] tok;
    reg       ent;                 // first clk of a LOW half: SDA changes now
    reg [1:0] w;                   // t_rd / qready follow `take` two edges later (take is registered)

    wire t_start = (tok == 3'd1), t_stop = (tok == 3'd2), t_wr = (tok == 3'd3);
    wire t_cond  = ~tok[2] & ~tok[1] | t_stop;          // tok 1 or 2
    wire last    = idx[3] | (t_cond & idx[0]);          // the token's last step
    // SDA for the LOW half that starts now (see the header): START releases then pulls it with SCL
    // high, STOP pulls then releases it, WRITE sends sh[7] then releases for the ACK, READ releases
    // then ACKs unless the token is READ+NACK (tok[0]).
    wire sda_next = t_cond ? (idx[0] ^ t_start) :
                    t_wr   ? (idx[3] | sh[7]) :
                             (~idx[3] | tok[0]);
    wire hz = (hcnt == 7'd0);

    assign busy = (st != IDLE);

    always @(posedge clk) begin
        take <= 1'b0;
        push <= 1'b0;
        if (rst || !on || arm) begin
            st <= IDLE; scl_rel <= 1'b1; sda_rel <= 1'b1; ent <= 1'b0; w <= 2'b00;
            if (rst || arm) arb <= 1'b0;
        end else begin
            w <= {1'b0, w[1]};
            if (ent) begin ent <= 1'b0; sda_rel <= sda_next; end
            if (!hz && !(st == HIGH && !scl_s)) hcnt <= hcnt - 7'd1;
            if (st == HIGH && !scl_s) hcnt <= half - 7'd1;      // stretched: wait for SCL high
            case (st)
            IDLE: if (qready && w == 2'b00 && !arb) begin
                    tok <= t_rd[2:0]; take <= 1'b1; w <= 2'b11;
                    idx <= 4'd0; hcnt <= half - 7'd1;
                    if (t_rd[2:0] == 3'd3) st <= DATA;
                    else begin st <= LOW; ent <= 1'b1; end
                  end
            DATA: if (qready && w == 2'b00) begin
                    sh <= t_rd; take <= 1'b1; w <= 2'b11;
                    hcnt <= half - 7'd1; st <= LOW; ent <= 1'b1;
                  end
            LOW:  if (hz && !ent) begin
                    if (t_cond && idx[0]) begin                 // START / STOP step 1 ends here
                        if (t_start) scl_rel <= 1'b0;
                        st <= IDLE;
                    end else begin
                        scl_rel <= 1'b1; hcnt <= half - 7'd1; st <= HIGH;
                    end
                  end
            HIGH: if (hz && scl_s) begin
                    idx  <= idx + 4'd1;
                    hcnt <= half - 7'd1;
                    st   <= LOW;
                    ent  <= 1'b1;
                    if (!t_cond) begin
                        scl_rel <= 1'b0;
                        sh      <= {sh[6:0], sda_s};
                        pdata   <= t_wr ? {7'd0, sda_s} : {sh[6:0], sda_s};
                        push    <= t_wr ? idx[3] : (idx == 4'd7);
                        if (idx[3]) st <= IDLE;                 // the ACK bit is done
                        if (t_wr && !idx[3] && sda_rel && !sda_s) begin   // lost arbitration
                            arb <= 1'b1; st <= IDLE; scl_rel <= 1'b1; sda_rel <= 1'b1; ent <= 1'b0;
                        end
                    end
                  end
            endcase
        end
    end
    // (`last` documents the step structure; the case arms test it directly.)
    wire unused_last = last;
endmodule
