// ============================================================================
// psram_bus_arbiter.v — burst-granular arbiter for the shared APS6404L quad bus.
//
// PROTOTYPE (docs: concurrent DAC-replay + capture PSRAM partition).  Today the
// dac_psram_reader (deep DAC replay, QPI 0xEB) and the psram_dual_writer (ADC+LA
// capture, QPI 0x38) are MUTUALLY EXCLUSIVE — the top-level pad mux picks one on a
// static level (rd_active) and firmware guarantees they never run together.  To let
// a DAC waveform stream OUT of the top of PSRAM while a capture streams INTO the
// bottom (disjoint regions), the two masters must time-multiplex the single quad
// bus.  This arbiter grants the bus to exactly one master at a time.
//
// ---- Why burst granularity is safe ----
// Each master structures every PSRAM access as a CS-LOW burst (cmd+addr+data)
// followed by a CS-HIGH refresh gap (tCEM), then returns to its S_IDLE state with
// CS high and its IO output-enable deasserted.  This arbiter only ever CHANGES the
// owner while BOTH masters are in that quiescent CS-high state:
//   * NONE -> X   happens only when the bus is free (previous owner already released
//                 on its burst-done, i.e. it is back in S_IDLE, CS high).
//   * X -> NONE   happens only when the owner's `busy` falls (it returned to S_IDLE).
// So the top-level pad mux switch is always at a CS-high boundary — never mid-burst,
// never a bus collision, never a turnaround glitch.  A master may only LEAVE its idle
// state (start a burst) when it holds the grant; once started it runs the burst to
// completion (grant is held until `busy` falls), then the bus is re-arbitrated.
//
// ---- Fairness ----
// Round-robin at burst granularity: a continuously-busy master cannot hog the bus —
// it is forced to release after every burst and the token alternates to the other
// master if it also wants the bus.  (An earlier revision added per-master `urgent`
// preempt inputs to bias whoever was nearest its FIFO failure boundary; they were
// dropped to reclaim LC on the ~88%-full up5k.  Round-robin alone is starvation-free
// and the sim shows both FIFOs stay within bounds at representative rates — urgency
// only trimmed underrun/overflow TAIL latency, not correctness.)
// ============================================================================
module psram_bus_arbiter (
    input  wire clk,
    input  wire rst,

    // reader (dac_psram_reader) handshake
    input  wire rd_req,      // wants a burst (has room + data to fetch)
    input  wire rd_busy,     // currently in a CS-low burst
    output wire rd_gnt,      // grant: reader may run a burst

    // writer (psram_dual_writer) handshake
    input  wire wr_req,      // wants a burst (a staging FIFO has data)
    input  wire wr_busy,     // currently in a CS-low burst
    output wire wr_gnt,      // grant: writer may run a burst

    output wire [1:0] owner  // 0=NONE, 1=READER, 2=WRITER  (for the top pad mux)
);
    localparam [1:0] NONE = 2'd0, RD = 2'd1, WR = 2'd2;

    reg [1:0] own_r;
    reg       started;   // owner has actually begun its burst (busy seen high)
    reg       last_rd;   // round-robin memory: 1 => reader went last, prefer writer

    assign owner  = own_r;

    // The cycle the owner returns to idle after a burst (busy fell, having been high)
    // is the cycle the arbiter releases it.  In that SAME cycle the owner still nominally
    // holds the grant and is sitting in its S_IDLE decision — so it would re-commit to a
    // fresh, ungranted burst (which the next owner's pads then clobber).  Mask the grant
    // with the release condition so it drops in that cycle: the owner cannot start a new
    // burst as it is being released; it must wait to be re-granted.
    wire rel_rd = (own_r == RD) && started && !rd_busy;
    wire rel_wr = (own_r == WR) && started && !wr_busy;

    // Grant also masked by the OTHER master's busy so a burst can only ever start while
    // the other is quiescent -> "both mid-burst" is impossible by construction.
    assign rd_gnt = (own_r == RD) && !wr_busy && !rel_rd;
    assign wr_gnt = (own_r == WR) && !rd_busy && !rel_wr;

    // Pick logic when the bus is free: round-robin between the two, else whoever asks.
    wire both_req = rd_req && wr_req;
    wire [1:0] pick =
        both_req ? (last_rd ? WR : RD) :
        rd_req   ? RD :
        wr_req   ? WR : NONE;

    always @(posedge clk) begin
        if (rst) begin
            own_r <= NONE; started <= 1'b0; last_rd <= 1'b0;
        end else begin
            case (own_r)
                NONE: begin
                    started <= 1'b0;
                    own_r   <= pick;   // may stay NONE if nobody is asking
                end
                RD: begin
                    if (rd_busy) started <= 1'b1;
                    // release after the burst completes, OR if the grantee never
                    // started and no longer wants the bus (defensive).
                    if ((started && !rd_busy) || (!started && !rd_busy && !rd_req)) begin
                        own_r   <= NONE;
                        started <= 1'b0;
                        last_rd <= 1'b1;
                    end
                end
                WR: begin
                    if (wr_busy) started <= 1'b1;
                    if ((started && !wr_busy) || (!started && !wr_busy && !wr_req)) begin
                        own_r   <= NONE;
                        started <= 1'b0;
                        last_rd <= 1'b0;
                    end
                end
                default: own_r <= NONE;
            endcase
        end
    end
endmodule
