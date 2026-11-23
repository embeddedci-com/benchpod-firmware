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
// Commands: SRST 0, ROTF 1 (read on the fly), WOTF 2 (write on the fly).  The firmware composes
// them from the raw tokens below (a macro in the gateware did not fit the loop image):
//   SRST   CMD(0)
//   ROTF   CMD(1) TXB(N) TXB(addr23:16) TXB(addr15:8) TXB(addr7:0) RXB x N     (N = byte count)
//   WOTF   CMD(2) TXB(N) TXB(addr23:16) TXB(addr15:8) TXB(addr7:0) TXB(data) x N
// Queue the whole command in one SWD_FEED: the chip starts sending a ROTF's first byte soon after
// the address is ACKed, and the engine runs a token as soon as it has it.
//
// Queue tokens (each SWD_FEED byte; bits 2:0 = op, the rest is the argument):
//   0 NOP
//   1 ENTRY  The entry sequence, then the sync pulse.  The pod pulls SWIM low for 16 us, then
//            8 pulses (4 at 1 kHz, 4 at 2 kHz, 50% duty), releases it and waits for the chip's
//            sync pulse (up to 0.5 ms), counting it.  Reply: 2 bytes, L (clk per 128 SWIM
//            clocks) low byte first, high byte (low nibble); both 0 when no sync pulse came.
//            The SWIM clock in MHz is 3072 / L.  A sync outside 192..1023 clk or no sync stops
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
//   7 NOP
// A NACKed frame is sent again (CMD, TXB) or asked for again (RXB) up to the retry count, then
// the queue stops; a chip that never answers (no ACK, no frame start within the timeout) stops
// it too, and a failure appends the REPORT record (3 bytes) to the reply buffer (ENTRY
// failures reply with the 2 sync bytes only).  Stopped: the
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
    localparam [9:0] L_DEF = 10'd384, L_MIN = 10'd192;
    // timer kinds for the long (watchdog) timer, in ticks; the entry sequence runs at L_DEF
    // (1 tick = 3 clk = 125 ns): hold 16 us, pulses 500 / 250 us, 200 us for the line to rise,
    // 4095 ticks (0.5 ms) for the chip's sync pulse
    localparam [2:0] K_TMO = 0, K_RISE = 1, K_HOLD = 2, K_P1 = 3, K_P2 = 4, K_RE = 5, K_SYNC = 6;

    localparam [4:0] IDLE = 0, ARGTX = 1, ARGCFG = 2,
                     E_HOLD = 3, E_HI = 4, E_LO = 5, E_RISE = 6, E_WAIT = 7, E_MEAS = 8,
                     T_NEXT = 10, T_LOW = 11, T_REL = 12, T_HI = 13,
                     R_WAIT = 14, R_SMP = 15, R_END = 16, R_HI = 17;
    reg [4:0] st;
    reg [1:0] w;                   // t_rd / qready follow `take` two edges later
    reg       stop, failgo, isent;

    // ---- the tick: one per SWIM clock ----
    reg  [9:0]  L;
    reg  [10:0] acc;
    wire [10:0] acc_n = acc + 11'd128;
    wire        tick  = (acc_n >= {1'b0, L});
    reg  [4:0]  tk;                // bit timing, ticks
    wire        tz    = (tk == 5'd0);
    reg  [11:0] wd;                // timeouts and the entry sequence, ticks
    reg         wdl;
    reg  [2:0]  wdk;
    wire        wz    = (wd == 12'd0) && !wdl;

    reg         hs;
    reg  [3:0]  retries, rl;
    reg  [7:0]  tmo, rise;
    reg  [9:0]  fr;                // the frame being sent (right-justified, MSB first), kept for a retry
    reg         isdat;             // fr is a 10-bit data frame (else a 5-bit command frame)
    reg  [3:0]  nb;                // bits left to send / receive
    reg  [9:0]  rx;
    reg         job_rx, sub_ack, rxok, needhi;
    reg  [1:0]  ai;
    reg  [3:0]  ei;
    reg  [2:0]  blank;
    reg  [7:0]  ackc, nackc, fl;
    // reply pushes: kind 0 REPORT (3 bytes), 1 sync length (2), 2 data byte (1)
    reg         pact;
    reg  [1:0]  pk, pi;

    wire [7:0] ackc_n = ackc + 8'd1, nackc_n = nackc + 8'd1;
    assign busy    = (st != IDLE) | pact | failgo;
    assign stopped = stop;
    assign flags   = {stop, hs, fl[5:0]};
    assign nack3   = (|nackc[7:3]) ? 3'd7 : nackc[2:0];

    wire [11:0] rise_t = (rise == 8'd0) ? 12'd64 : {2'b00, rise, 2'b00};   // never waits less than 64 ticks to receive
    wire [11:0] tmo_t  = (tmo == 8'd0) ? 12'hFFF : {tmo, 4'b0000};
    wire        take_ok = qready && (w == 2'b00) && !pact && !failgo;
    wire        curbit  = fr[nb - 4'd1];
    wire [3:0]  en      = ei + 4'd1;

    function [4:0] lowt(input b, input h); lowt = b ? 5'd2 : (h ? 5'd8 : 5'd20); endfunction
    function [4:0] hit (input b, input h); hit  = b ? (h ? 5'd8 : 5'd20) : 5'd2; endfunction

    // bit timer: restart the accumulator so N ticks last ceil(N * L / 128) clk
    task ldt(input [4:0] n); begin tk <= n; acc <= 11'd0; end endtask
    task ldw(input [2:0] k); begin wdk <= k; wdl <= 1'b1; end endtask
    task fail(input [7:0] f); begin fl <= fl | f; failgo <= 1'b1; end endtask
    task push_go(input [1:0] k); begin pk <= k; pi <= 2'd0; pact <= 1'b1; end endtask

    always @(posedge clk) begin
        take <= 1'b0;
        push <= 1'b0;
        // the push sequencer
        if (pact) begin
            push <= 1'b1; pi <= pi + 2'd1;
            case (pk)
            2'd0: begin pdata <= (pi == 2'd0) ? ackc : (pi == 2'd1) ? nackc : {stop, hs, fl[5:0]}; if (pi == 2'd2) pact <= 1'b0; end
            2'd1: begin pdata <= (pi == 2'd0) ? L[7:0] : {6'd0, L[9:8]}; if (pi == 2'd1) pact <= 1'b0; end
            default: begin pdata <= rx[8:1]; pact <= 1'b0; end
            endcase
        end
        // the tick and the long timer
        acc <= tick ? (acc_n - {1'b0, L}) : acc_n;
        if (tick && !tz) tk <= tk - 5'd1;
        if (wdl) begin
            wdl <= 1'b0;
            case (wdk)
            K_TMO:  wd <= tmo_t;
            K_RISE: wd <= rise_t;
            K_HOLD: wd <= 12'd128;
            K_P1:   wd <= 12'd4000;
            K_P2:   wd <= 12'd2000;
            K_RE:   wd <= 12'd1600;
            default: wd <= 12'd4095;
            endcase
        end else if (tick && wd != 12'd0) wd <= wd - 12'd1;

        if (rst || !on || arm) begin
            st <= IDLE; w <= 2'b11; low <= 1'b0; stop <= 1'b0; failgo <= 1'b0; isent <= 1'b0;   // w: a stale qready after a re-arm is not a token
            L <= L_DEF; hs <= 1'b0; retries <= 4'd3; tmo <= 8'd32; rise <= 8'd64;
            ackc <= 8'd0; nackc <= 8'd0; fl <= 8'd0; pact <= 1'b0; wdl <= 1'b0;
            acc <= 11'd0; tk <= 5'd0; wd <= 12'd0; blank <= 3'd0; needhi <= 1'b0;
        end else begin
            w <= {1'b0, w[1]};
            if (blank != 3'd0) blank <= blank - 3'd1;
            if (failgo) begin                      // a failure: stop, and report
                failgo <= 1'b0; stop <= 1'b1; low <= 1'b0; st <= IDLE;
                push_go(isent ? 2'd1 : 2'd0);
            end else case (st)
            IDLE: if (!stop && take_ok) begin
                take <= 1'b1; w <= 2'b11; isent <= 1'b0;
                case (t_rd[2:0])
                3'd1: begin L <= L_DEF; isent <= 1'b1; low <= 1'b1; ldw(K_HOLD); st <= E_HOLD; end
                3'd2: begin
                    fr <= {5'd0, 1'b0, t_rd[5:3], ^t_rd[5:3]}; isdat <= 1'b0;
                    job_rx <= 1'b0; sub_ack <= 1'b0; rl <= retries; nb <= 4'd5; st <= T_NEXT;
                end
                3'd3: st <= ARGTX;
                3'd4: begin job_rx <= 1'b1; sub_ack <= 1'b0; rl <= retries; nb <= 4'd10; ldw(K_TMO); st <= R_WAIT; end
                3'd5: begin ai <= 2'd0; st <= ARGCFG; end
                3'd6: push_go(2'd0);
                default: ;
                endcase
            end
            ARGTX: if (qready && w == 2'b00) begin
                take <= 1'b1; w <= 2'b11;
                fr <= {1'b0, t_rd, ^t_rd}; isdat <= 1'b1;
                job_rx <= 1'b0; sub_ack <= 1'b0; rl <= retries; nb <= 4'd10; st <= T_NEXT;
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
            E_HOLD: if (wz) begin ei <= 4'd0; low <= 1'b0; ldw(K_P1); st <= E_HI; end
            E_HI: if (wz) begin low <= 1'b1; ldw(ei[3:2] == 2'd0 ? K_P1 : K_P2); st <= E_LO; end
            E_LO: if (wz) begin
                ei <= en; low <= 1'b0;
                if (ei == 4'd7) begin blank <= 3'd5; ldw(K_RE); st <= E_RISE; end
                else begin ldw(en[3:2] == 2'd0 ? K_P1 : K_P2); st <= E_HI; end
            end
            E_RISE: if (blank == 3'd0) begin       // the line must read high before the sync is awaited
                if (dio_s) begin L <= 10'd0; ldw(K_SYNC); st <= E_WAIT; end
                else if (wz) fail(8'h08);
            end
            E_WAIT: if (!dio_s) begin L <= 10'd1; st <= E_MEAS; end        // L counts the sync pulse in clk
                    else if (wz) fail(8'h02);
            E_MEAS: begin
                if (!dio_s) begin
                    if (&L) fail(8'h04); else L <= L + 10'd1;               // 1023 clk and still low
                end else begin
                    if (L < L_MIN) fail(8'h04);
                    else begin hs <= 1'b0; push_go(2'd1); st <= IDLE; end
                end
            end

            // ---- frames sent: header..parity, bit by bit, then the receiver's ACK ----
            T_NEXT: begin
                if (nb != 4'd0) begin
                    low <= 1'b1; ldt(lowt(curbit, hs)); st <= T_LOW;
                end else if (!job_rx) begin          // frame out: wait for its ACK bit
                    sub_ack <= 1'b1; nb <= 4'd1; needhi <= 1'b1; ldw(K_TMO); st <= R_WAIT;
                end else begin                       // the ACK for a received frame went out
                    if (rxok) begin
                        if (~&ackc) ackc <= ackc_n;
                        push_go(2'd2); st <= IDLE;
                    end else begin
                        if (~&nackc) nackc <= nackc_n;
                        fl[5] <= 1'b1;
                        if (rl != 4'd0) begin rl <= rl - 4'd1; sub_ack <= 1'b0; nb <= 4'd10; ldw(K_TMO); st <= R_WAIT; end
                        else fail(8'h10);
                    end
                end
            end
            T_LOW: if (tz) begin low <= 1'b0; blank <= 3'd5; ldw(K_RISE); st <= T_REL; end
            T_REL: if (blank == 3'd0) begin
                if (dio_s || rise == 8'd0) begin
                    if (!job_rx && !sub_ack && nb == 4'd1) begin   // last bit of a frame: the ACK may come in
                        nb <= 4'd0; st <= T_NEXT;                   // at once, so skip its high part
                    end else begin ldt(hit(curbit, hs)); st <= T_HI; end
                end else if (wz) fail(8'h08);
            end
            T_HI: if (tz) begin nb <= nb - 4'd1; st <= T_NEXT; end

            // ---- bits received: wait for the falling edge, sample mid-bit, wait for the rise ----
            R_WAIT: begin
                if (dio_s) needhi <= 1'b0;
                if (!dio_s && !needhi) begin ldt(hs ? 5'd5 : 5'd11); st <= R_SMP; end
                else if (wz) fail(8'h01);
            end
            R_SMP: if (tz) begin
                rx <= {rx[8:0], dio_s}; nb <= nb - 4'd1;
                if (nb == 4'd1) begin ldt(hs ? 5'd5 : 5'd11); st <= R_END; end   // the last bit: out to the end of its slot
                else begin ldw(K_RISE); st <= R_HI; end                           // the chip's next bit may start at once
            end
            R_END: if (tz) begin ldw(K_RISE); st <= R_HI; end
            R_HI: if (dio_s) begin
                if (nb != 4'd0) begin ldw(K_TMO); st <= R_WAIT; end
                else if (!job_rx) begin              // the ACK bit of a frame we sent
                    if (rx[0]) begin
                        if (~&ackc) ackc <= ackc_n;
                        st <= IDLE;
                    end else begin
                        if (~&nackc) nackc <= nackc_n;
                        if (rl != 4'd0) begin rl <= rl - 4'd1; sub_ack <= 1'b0; nb <= isdat ? 4'd10 : 4'd5; st <= T_NEXT; end
                        else fail(8'h10);
                    end
                end else begin                       // a frame from the chip: check it, ACK or NACK
                    rxok <= ~(^rx[8:0]);
                    fr[0] <= ~(^rx[8:0]); nb <= 4'd1; sub_ack <= 1'b1; st <= T_NEXT;
                end
            end else if (wz) fail(8'h08);
            default: st <= IDLE;
            endcase
        end
    end
endmodule
