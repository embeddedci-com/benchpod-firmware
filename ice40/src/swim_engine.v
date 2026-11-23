// ============================================================================
// swim_engine.v: the SWIM master (v62), swd_engine's fifth job.  SWIM is the STM8's one-wire
// debug interface (ST UM0470).  Like jtag_seq.v it has its own small counters and shares only
// swd_engine's queue (tmem: `t_rd` / `qready`, `take` advances qptr) and reply buffer (`push`
// writes `pdata` at wptr).  SPI_ARM flags bit 4 arms it; the SWIM pin is the dio slot (mosi_ch =
// miso_ch), OPEN DRAIN: the pod pulls it low or releases it, never drives it high.  The line
// rises through a pull-up (the pod's own, or an external ~1 kohm); the chip's NRST is the
// firmware's business (the pod's NRST pin), not this engine's.
//
// Time base.  SWIM bits are measured in the chip's SWIM clock, which the chip tells us: after the
// entry sequence it sends a sync pulse of 128 SWIM clocks low.  The engine counts it in clk
// (24 MHz) as L, then a phase accumulator ticks once per SWIM clock (128 ticks per L clk).  L is
// 384 after arming (a nominal 8 MHz SWIM clock, 16 us) so a chip that is already in SWIM mode
// works without a new entry.  Every bit below is a whole number of those ticks.
//   low speed  (the default after entry): a 0 is 20 ticks low + 2 high, a 1 is 2 low + 20 high
//   high speed (CFG bit 7):              a 0 is  8 ticks low + 2 high, a 1 is 2 low +  8 high
// A device-to-host bit is read by sampling the line mid-bit: from its falling edge, at 11 ticks
// (low speed) or 5 (high speed): still low is a 0, back high is a 1.
//
// Frames (UM0470): header bit (0 host to device, 1 device to host; not checked on receive),
//   command  = header, 3 command bits MSB first, parity                (5 bits)
//   data     = header, 8 data bits MSB first, parity                   (10 bits)
//   parity   = even over the command or data bits
// and every frame is answered with one bit by the receiver: 1 = ACK, 0 = NACK.
// Commands: SRST 0, ROTF 1 (read on the fly), WOTF 2 (write on the fly).  ROTF and WOTF each
// send the command, then the byte count N, then the 24-bit address as three data frames (every
// frame ACKed), then N data frames: from the chip (ROTF, each ACKed by the pod) or to it (WOTF,
// each ACKed by the chip).  The ROTF/WOTF token runs all of that by itself, so the pod is
// listening when the chip starts sending (the chip does not wait for a slow host link).
//
// Queue tokens (each SWD_FEED byte; bits 2:0 = op, the rest is the argument):
//   0 NOP
//   1 ENTRY  The entry sequence, then the sync pulse.  The pod pulls SWIM low for 16 us, then
//            8 pulses (4 at 1 kHz, 4 at 2 kHz, 50% duty), releases it and waits for the chip's
//            sync pulse (up to 4 ms), counting it.  Reply: 2 bytes, L (clk per 128 SWIM
//            clocks) low byte first, high byte (low nibble); both 0 when no sync pulse came.
//            The SWIM clock in MHz is 3072 / L.  A sync outside 192..4095 clk or no sync stops
//            the queue (flags below).  Resets high speed.
//   2 CMD    bits 5:3 = the 3-bit command (0 = SRST): header, command, parity, then the ACK.
//   3 TXB    the NEXT feed byte is data: header, 8 bits, parity, then the ACK.
//   4 RXB    wait for a data frame from the chip (up to the CFG timeout), check its parity, send
//            ACK (good) or NACK (bad: the chip sends the frame again).  Reply: the byte.
//   5 CFG    the next 3 feed bytes: [7] high speed, [3:0] NACK retries (default 3);
//            timeout in units of 16 ticks (0 = 4096, default 32 = 512 ticks);
//            line-high wait limit in units of 4 ticks (0 = do not wait, default 64 = 256 ticks).
//   6 REPORT reply: 3 bytes, ACK count, NACK count (both saturate at 255), flags.  The
//            counts start at 0 when the engine is armed.
//   7 ROTF / WOTF  bit 3 = 0 ROTF, 1 WOTF.  The next 4 feed bytes: N (0 = 256), address bits
//            23:16, 15:8, 7:0; the engine starts when all 4 are in.  WOTF then takes N data
//            bytes from the queue, one per frame (as each frame is due).  ROTF's reply is the
//            N bytes read.  SRST is CMD(0); CMD, TXB and RXB are the raw frames.
// A NACKed frame is sent again (CMD, TXB) or asked for again (RXB) up to the retry count, then
// the queue stops; a chip that never answers (no ACK, no frame start within the timeout) stops
// it too, and a failure appends the REPORT record (3 bytes) to the reply buffer.  Stopped: the
// remaining tokens stay in the queue, q_done says how far it got; re-arm to run again.
//
// Flags (REPORT's third byte; q_flags in SWD_QSTATUS is {timeout, qptr[8], nack-failed,
// NACK count (3 bit), stopped, busy}, timeout = any of bits 3:0):
//   bit0 no ACK or no frame within the timeout     bit1 no sync pulse after the entry sequence
//   bit2 sync pulse out of range                   bit3 the line did not rise within the limit
//   bit4 NACK retries used up                      bit5 a received frame had bad parity
//   bit6 high speed on                             bit7 stopped
//
// Open drain: after each pulled-low part the engine releases the pin and waits (up to the CFG
// limit, 4 ticks per unit) until it READS high before it counts the high part, so a weak
// pull-up only stretches the bit instead of corrupting it.  A line that stays low past the limit
// stops the queue (bit 3).  Device-to-host bits get the same wait at the end of each bit.
// ============================================================================

module swim_engine (
    input  wire       clk,         // 24 MHz
    input  wire       rst,
    input  wire       on,          // armed in SWIM mode
    input  wire       arm,         // 1-cycle: (re)armed: everything back to defaults
    input  wire       dio_s,       // the synchronised SWIM level
    input  wire [7:0] t_rd,        // the queue head (one edge after qptr moves)
    input  wire       qready,      // the queue holds a byte (one edge after qptr moves)
    output reg        take,        // 1-cycle: consume the queue head
    output reg        push,        // 1-cycle: write pdata to the reply buffer
    output reg  [7:0] pdata,       // valid while push is high
    output wire       busy,
    output reg        low,         // 1 = pull SWIM low (open drain)
    output wire       stopped,
    output wire [7:0] flags,
    output wire [2:0] nack3        // NACK count, saturating at 7
);
    // entry sequence, in microseconds (clk is 24 MHz)
    localparam HOLD_US = 16, P1_US = 500, P2_US = 250, SYNC_TMO_US = 4000, RISE_TMO_US = 200;
    localparam [11:0] L_DEF = 12'd384, L_MIN = 12'd192;

    localparam [4:0] IDLE = 0,  ARGTX = 1,  ARGCFG = 2,  ARGM = 18, M_STEP = 19, M_DFETCH = 20,
                     E_HOLD = 3, E_HI = 4,  E_LO = 5,  E_RISE = 6, E_WAIT = 7, E_MEAS = 8,
                     F_START = 9, T_NEXT = 10, T_LOW = 11, T_REL = 12, T_HI = 13,
                     R_WAIT = 14, R_SMP = 15, R_END = 16, R_HI = 17;
    reg [4:0] st;
    reg [1:0] w;                   // t_rd / qready follow `take` two edges later
    reg       stop;

    // ---- the tick: one per SWIM clock ----
    reg  [11:0] L;
    reg  [12:0] acc;
    wire [12:0] acc_n = acc + 13'd128;
    wire        tick  = (acc_n >= {1'b0, L});
    reg  [11:0] tk;
    wire        tz    = (tk == 12'd0);

    // ---- the microsecond timer (entry sequence) ----
    reg  [4:0]  pre;
    reg  [12:0] us;
    wire        uz = (us == 13'd0);

    reg         hs;
    reg  [3:0]  retries, rl;
    reg  [7:0]  tmo, rise;
    reg  [9:0]  sr, fr;            // frame being sent, and its copy for a retry
    reg  [3:0]  nb, fn;
    reg  [9:0]  rx;
    reg         job_rx;            // 0: sending a frame and waiting for its ACK, 1: receiving a frame
    reg         sub_ack;           // the ACK bit phase of the job
    reg         rxok;
    reg  [1:0]  ai;
    reg  [3:0]  ei;
    reg  [2:0]  blank;
    reg         needhi;        // the ACK wait starts only once the line has read high
    reg  [12:0] m;
    reg  [7:0]  ackc, nackc, fl;
    reg  [23:0] pq;
    reg         mact, mw;          // a ROTF / WOTF macro runs; it is a write
    reg  [2:0]  mstep;             // frames of the macro acked so far (saturates at 4)
    reg  [8:0]  mcnt;              // data frames left
    reg  [31:0] mbuf;              // N, A2, A1, A0, the next one to send in the top byte
    reg  [1:0]  pn;

    assign busy    = (st != IDLE) | (pn != 2'd0);
    assign stopped = stop;
    assign flags   = {stop, hs, fl[5:0]};
    assign nack3   = (nackc > 8'd7) ? 3'd7 : nackc[2:0];

    function [7:0] lowticks(input b, input h);   // low part of a bit, in ticks
        lowticks = b ? 8'd2 : (h ? 8'd8 : 8'd20);
    endfunction
    function [7:0] hiticks(input b, input h);    // high part
        hiticks = b ? (h ? 8'd8 : 8'd20) : 8'd2;
    endfunction

    // timer loads: ticks restart the accumulator so N ticks last ceil(N * L / 128) clk
    task ldt(input [11:0] n); begin tk <= n; acc <= 13'd0; end endtask
    task ldus(input [12:0] n); begin us <= n; pre <= 5'd23; end endtask

    // a frame that stopped the queue: flags, and its report record in the reply buffer
    task failit(input [7:0] f, input [7:0] nk); begin
        fl   <= fl | f;
        stop <= 1'b1;
        low  <= 1'b0;
        pq   <= {1'b1, hs, fl[5:0] | f[5:0], nk, ackc};
        pn   <= 2'd3;
        mact <= 1'b0;
        st   <= IDLE;
    end endtask

    wire [11:0] rise_t = {rise, 2'b00};
    wire [11:0] tmo_t  = (tmo == 8'd0) ? 12'hFFF : {tmo, 4'b0000};
    wire [3:0]  en     = ei + 4'd1;
    wire [7:0]  nackc_n = (nackc == 8'd255) ? nackc : nackc + 8'd1;   // nackc as it will be
    wire [11:0] rise_r = (rise == 8'd0) ? 12'd64 : rise_t;   // receiving always waits for the line to read high
    wire        take_ok = qready && (w == 2'b00) && (pn == 2'd0);

    always @(posedge clk) begin
        take  <= 1'b0;
        push  <= 1'b0;
        // the push sequencer: one byte per clk from pq
        if (pn != 2'd0) begin
            push <= 1'b1; pdata <= pq[7:0]; pq <= {8'd0, pq[23:8]}; pn <= pn - 2'd1;
        end
        // the tick
        acc <= tick ? (acc_n - {1'b0, L}) : acc_n;
        if (tick && !tz) tk <= tk - 12'd1;
        // the microsecond timer
        pre <= (pre == 5'd0) ? 5'd23 : (pre - 5'd1);
        if (pre == 5'd0 && !uz) us <= us - 13'd1;

        if (rst || !on || arm) begin
            st <= IDLE; w <= 2'b11; low <= 1'b0; stop <= 1'b0;   // w: a stale qready after a re-arm is not a token
            L <= L_DEF; hs <= 1'b0; retries <= 4'd3; tmo <= 8'd32; rise <= 8'd64;
            ackc <= 8'd0; nackc <= 8'd0; fl <= 8'd0; pn <= 2'd0; mact <= 1'b0;
            acc <= 13'd0; tk <= 12'd0; us <= 13'd0; pre <= 5'd0; blank <= 3'd0; needhi <= 1'b0;
        end else begin
            w <= {1'b0, w[1]};
            if (blank != 3'd0) blank <= blank - 3'd1;
            case (st)
            IDLE: if (!stop && take_ok) begin
                take <= 1'b1; w <= 2'b11;
                case (t_rd[2:0])
                3'd1: begin low <= 1'b1; ldus(HOLD_US); st <= E_HOLD; end
                3'd2: begin
                    fr <= {1'b0, t_rd[5:3], ^t_rd[5:3], 5'd0}; fn <= 4'd5;
                    job_rx <= 1'b0; sub_ack <= 1'b0; rl <= retries; st <= F_START;
                end
                3'd3: st <= ARGTX;
                3'd4: begin job_rx <= 1'b1; sub_ack <= 1'b0; rl <= retries; nb <= 4'd10; ldt(tmo_t); st <= R_WAIT; end
                3'd5: begin ai <= 2'd0; st <= ARGCFG; end
                3'd6: begin pq <= {stop, hs, fl[5:0], nackc, ackc}; pn <= 2'd3; end
                3'd7: begin mw <= t_rd[3]; ai <= 2'd0; st <= ARGM; end
                default: ;
                endcase
            end
            ARGTX: if (qready && w == 2'b00) begin
                take <= 1'b1; w <= 2'b11;
                fr <= {1'b0, t_rd, ^t_rd}; fn <= 4'd10;
                job_rx <= 1'b0; sub_ack <= 1'b0; rl <= retries; st <= F_START;
            end
            ARGM: if (qready && w == 2'b00) begin
                take <= 1'b1; w <= 2'b11;
                mbuf <= {mbuf[23:0], t_rd};
                ai   <= ai + 2'd1;
                if (ai == 2'd3) begin                  // N, A2, A1, A0 are in: send the command
                    mcnt <= (mbuf[23:16] == 8'd0) ? 9'd256 : {1'b0, mbuf[23:16]};
                    mact <= 1'b1; mstep <= 3'd0;
                    fr <= {1'b0, mw ? 3'd2 : 3'd1, 1'b1, 5'd0}; fn <= 4'd5;
                    job_rx <= 1'b0; sub_ack <= 1'b0; rl <= retries; st <= F_START;
                end
            end
            M_STEP: begin                              // a frame of the macro was ACKed
                if (mstep < 3'd4) begin                // N, then the address, one data frame each
                    fr <= {1'b0, mbuf[31:24], ^mbuf[31:24]}; fn <= 4'd10; mbuf <= {mbuf[23:0], 8'd0};
                    mstep <= mstep + 3'd1;
                    job_rx <= 1'b0; sub_ack <= 1'b0; rl <= retries; st <= F_START;
                end else if (mcnt == 9'd0) begin mact <= 1'b0; st <= IDLE; end
                else if (!mw) begin                    // ROTF: the chip sends the next byte
                    mcnt <= mcnt - 9'd1;
                    job_rx <= 1'b1; sub_ack <= 1'b0; rl <= retries; nb <= 4'd10; ldt(tmo_t); st <= R_WAIT;
                end else st <= M_DFETCH;
            end
            M_DFETCH: if (qready && w == 2'b00) begin  // WOTF: the next data byte from the queue
                take <= 1'b1; w <= 2'b11;
                fr <= {1'b0, t_rd, ^t_rd}; fn <= 4'd10; mcnt <= mcnt - 9'd1;
                job_rx <= 1'b0; sub_ack <= 1'b0; rl <= retries; st <= F_START;
            end
            ARGCFG: if (qready && w == 2'b00) begin
                take <= 1'b1; w <= 2'b11;
                ai <= ai + 2'd1;
                case (ai)
                2'd0: begin hs <= t_rd[7]; retries <= t_rd[3:0]; end
                2'd1: tmo <= t_rd;
                default: begin rise <= t_rd; st <= IDLE; end
                endcase
            end

            // ---- entry: hold low, 8 pulses, release, the chip's sync pulse ----
            E_HOLD: if (uz) begin ei <= 4'd0; low <= 1'b0; ldus(P1_US); st <= E_HI; end
            E_HI: if (uz) begin low <= 1'b1; ldus(ei[3:2] == 2'd0 ? P1_US : P2_US); st <= E_LO; end
            E_LO: if (uz) begin
                ei <= en;
                if (ei == 4'd7) begin
                    low <= 1'b0; blank <= 3'd5; ldus(RISE_TMO_US); st <= E_RISE;
                end else begin
                    low <= 1'b0; ldus(en[3:2] == 2'd0 ? P1_US : P2_US); st <= E_HI;
                end
            end
            E_RISE: if (blank == 3'd0) begin     // the line must read high before the sync is awaited
                if (dio_s) begin ldus(SYNC_TMO_US); st <= E_WAIT; end
                else if (uz) begin
                    fl <= fl | 8'h08; stop <= 1'b1; pq <= 24'd0; pn <= 2'd2; st <= IDLE;
                end
            end
            E_WAIT: if (!dio_s) begin m <= 13'd1; st <= E_MEAS; end
                    else if (uz) begin
                        fl <= fl | 8'h02; stop <= 1'b1; pq <= 24'd0; pn <= 2'd2; st <= IDLE;
                    end
            E_MEAS: begin
                if (!dio_s) begin
                    if (m[12]) begin                       // 4096 clk and still low
                        fl <= fl | 8'h04; stop <= 1'b1; pq <= 24'h000FFF; pn <= 2'd2; st <= IDLE;
                    end else m <= m + 13'd1;
                end else begin
                    pq <= {12'd0, m[11:0]}; pn <= 2'd2;
                    if (m[11:0] < L_MIN) begin fl <= fl | 8'h04; stop <= 1'b1; end
                    else begin L <= m[11:0]; hs <= 1'b0; end
                    st <= IDLE;
                end
            end

            // ---- frames sent: header..parity, bit by bit, then the receiver's ACK ----
            F_START: begin sr <= fr; nb <= fn; st <= T_NEXT; end
            T_NEXT: begin
                if (nb != 4'd0) begin
                    low <= 1'b1; ldt({4'd0, lowticks(sr[9], hs)}); st <= T_LOW;
                end else if (!job_rx) begin          // frame out: wait for its ACK bit
                    sub_ack <= 1'b1; nb <= 4'd1; needhi <= 1'b1; ldt(tmo_t); st <= R_WAIT;
                end else begin                       // the ACK for a received frame went out
                    if (rxok) begin
                        ackc <= (ackc == 8'd255) ? ackc : ackc + 8'd1;
                        pq <= {16'd0, rx[8:1]}; pn <= 2'd1; st <= mact ? M_STEP : IDLE;
                    end else begin
                        nackc <= nackc_n;
                        fl <= fl | 8'h20;
                        if (rl != 4'd0) begin
                            rl <= rl - 4'd1; sub_ack <= 1'b0; nb <= 4'd10; ldt(tmo_t); st <= R_WAIT;
                        end else failit(8'h10, nackc_n);
                    end
                end
            end
            T_LOW: if (tz) begin
                low <= 1'b0; blank <= 3'd5; ldt(rise_t); st <= T_REL;
            end
            T_REL: if (blank == 3'd0) begin
                if (dio_s || rise == 8'd0) begin
                    if (!job_rx && !sub_ack && nb == 4'd1) begin   // last bit of a frame: the ACK may come in
                        sr <= {sr[8:0], 1'b0}; nb <= 4'd0; st <= T_NEXT;   // what would have been its high part
                    end else begin ldt({4'd0, hiticks(sr[9], hs)}); st <= T_HI; end
                end
                else if (tz) failit(8'h08, nackc);
            end
            T_HI: if (tz) begin sr <= {sr[8:0], 1'b0}; nb <= nb - 4'd1; st <= T_NEXT; end

            // ---- bits received: wait for the falling edge, sample mid-bit, wait for the rise ----
            R_WAIT: begin
                if (dio_s) needhi <= 1'b0;
                if (!dio_s && !needhi) begin ldt(hs ? 12'd5 : 12'd11); st <= R_SMP; end
                else if (tz) failit(8'h01, nackc);
            end
            R_SMP: if (tz) begin
                rx <= {rx[8:0], dio_s}; nb <= nb - 4'd1;
                if (nb == 4'd1) begin ldt(hs ? 12'd5 : 12'd11); st <= R_END; end   // the last bit: out to the end of its slot
                else begin ldt(rise_r); st <= R_HI; end                              // the chip's next bit may start at once
            end
            R_END: if (tz) begin ldt(rise_r); st <= R_HI; end
            R_HI: if (dio_s) begin
                if (nb != 4'd0) begin ldt(tmo_t); st <= R_WAIT; end
                else if (!job_rx) begin              // the ACK bit of a frame we sent
                    if (rx[0]) begin
                        ackc <= (ackc == 8'd255) ? ackc : ackc + 8'd1; st <= mact ? M_STEP : IDLE;
                    end else begin
                        nackc <= nackc_n;
                        if (rl != 4'd0) begin rl <= rl - 4'd1; sub_ack <= 1'b0; st <= F_START; end
                        else failit(8'h10, nackc_n);
                    end
                end else begin                       // a frame from the chip: check it, ACK or NACK
                    rxok <= ~(^rx[8:0]);
                    sr   <= {~(^rx[8:0]), 9'd0}; nb <= 4'd1; sub_ack <= 1'b1; st <= T_NEXT;
                end
            end else if (tz) failit(8'h08, nackc);
            default: st <= IDLE;
            endcase
        end
    end
endmodule
