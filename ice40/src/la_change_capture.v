// ============================================================================
// la_change_capture.v — LA "change" capture (v50): records only the samples where a
// channel changed, at the full 48 MS/s, so a long capture costs bytes per EDGE instead
// of bytes per sample.
//
// Input: one PAIR of 48 MS/s samples per 24 MHz clk (la48_sampler): pa = tick 2k,
// pb = tick 2k+1.  A tick is one 48 MHz sample period (20.83 ns).  The pair is clk48
// logic, so it is registered here (ra, rb) before anything reads it: the clk48 -> clk
// path is untimed and has only about one clk48 period, enough for a flop-to-flop hop
// but not for the compares below (32 ns when they read pa/pb directly).
//
// ---- stream format (what lands in PSRAM, 3 bytes per event, little-endian) ----
//   event = {delta[9:0], state[13:0]}: byte0 = state[7:0],
//           byte1 = {delta[1:0], state[13:8]}, byte2 = delta[9:2]
//   The first event has delta 0 and gives the state at t0 (the arm, or the trigger).
//   Every later event has delta >= 1: advance the time by `delta` ticks, then the
//   channels read `state`.  When nothing changes for ~1020 ticks the encoder writes
//   the unchanged state as a keep-alive event, so delta never overflows.  The last
//   tick of the capture always gets an event, so the stream ends exactly at
//   count * 256 - 1.  The stream then ends with an END event: delta 0 again (any
//   later delta-0 event is END; the bytes after it are padding or stale).  END's state field
//   says why the stream ended: 0 = the full duration, 1 = the ring was full, 2 = the region
//   end (see below).
//
// ---- duration ----
//   `count` is in units of 256 ticks (128 clk = 5.33 us); 24 bits reach ~89 s.
//
// ---- packing and overflow ----
//   A cycle yields 0, 1 or 2 events; they are paired into 48-bit words (2 events) for
//   spram_ring48, holding an odd one over in `l`.  That is at most one word per clk,
//   which is what the ring's port can write.  If the ring cannot take a word, the
//   capture STOPS there instead of dropping it: `overflow` is set, the held event and
//   END are written as soon as the ring has room, and everything before the stop is
//   exact.  The firmware reports such a capture as cut short (CAP_OVF).
//
// ---- pre-trigger (v50) ----
//   With `pretrig`, the encoder also records while `hold` (the trigger wait) is up, from the arm;
//   only the ticks after the trigger count toward the duration.  spram_ring48 keeps the newest
//   pre-trigger words and drops the oldest (top_v2), so the stream may start mid-way: every event
//   carries the full state, and the last event sits on the last tick, so the decoder counts the
//   times back from the end: the trigger is count * 256 ticks before it.
//
// ---- region end (v50) ----
//   `limit` is the LA region's size in bytes, latched at the arm (top_v2 passes the ADC
//   region base: the firmware sets it to the end of the free PSRAM before a change capture,
//   which never runs with the ADC).  A word that would leave no room for the END word stops
//   the capture the same way a full ring does, so the stream never runs past the region.
//   Pre-trigger words don't count here: the ring keeps at most its PRE_WORDS of them, and the
//   firmware takes those 6 * PRE_WORDS bytes off the limit it sends.
// ============================================================================
module la_change_capture #(
    parameter N  = 14,
    parameter DW = 10,               // delta width (24 - N)
    // EXT_CNT = 1 (top_v2): the 256-tick unit counter is la_psram_capture's SB_MAC16 down-counter
    // (a change capture never runs alongside the raw producer): the caller loads it with `count`
    // on the `start` edge, `cnt_en` steps it, and `cnt_one` says it reads 1.  EXT_CNT = 0 keeps a
    // fabric counter here (tb_la_change).
    parameter EXT_CNT = 0
)(
    input  wire          clk,
    input  wire          rst,
    input  wire [N-1:0]  pa,         // sample at tick 2k
    input  wire [N-1:0]  pb,         // sample at tick 2k+1
    input  wire          start,      // 1-cycle arm
    input  wire          hold,       // trigger wait: nothing is counted (nor recorded without pretrig)
    input  wire          pretrig,    // record during hold too (a pre-trigger window)
    input  wire [23:0]   count,      // duration in units of 256 ticks (0 = off)
    input  wire [23:0]   limit,      // region size in bytes (latched at start)
    output wire [47:0]   wd,         // word for the ring: {later event, earlier event}
    output wire          wstb,
    input  wire          wrdy,       // ring takes a word strobed this cycle
    output reg           busy,       // encoding or waiting to write END
    output reg           done,       // sticky until the next start
    output reg           overflow,   // sticky until the next start: stopped early
    output wire          cnt_en,     // EXT_CNT: step the unit counter this edge
    input  wire          cnt_one     // EXT_CNT: the unit counter reads 1
);
    localparam [DW-1:0] KEEPALIVE = 10'd1020;

    reg [N-1:0]  ra, rb;             // the pair, in the clk domain
    always @(posedge clk) begin ra <= pa; rb <= pb; end

    reg          run;                // encoding (busy and not yet ending)
    reg          ending;             // write the held event + END, then done
    reg          first;              // next recorded sample is t0
    reg [N-1:0]  p;                  // state of the last recorded tick
    reg [DW-1:0] since;              // ticks from the last event to ra
    reg [6:0]    pre;                // clk cycles within the current 256-tick unit
    reg [23:0]   left;               // 256-tick units left, including this one (EXT_CNT = 0)
    wire [23:0]  room;               // region bytes left (one logic cell per bit, lc_counter)
    reg  [1:0]   cause;              // why the stream ends (END's state): 0 done, 1 ring full, 2 region end
    reg          lv;                 // an odd event is held in l
    reg [23:0]   l;

    // ---- this cycle's events ----
    wire          ev_a    = first | (ra != p);
    wire          left_one = EXT_CNT ? cnt_one : (left == 24'd1);
    wire          last    = !hold && left_one && (pre == 7'd127);
    // since >= KEEPALIVE (1020 = 0b1111111100) is since[9:2] all ones: an AND, not a compare
    wire          keep    = &since[DW-1:2];
    wire          force_b = last | (!ev_a && keep);
    wire          ev_b    = (rb != ra) | force_b;
    // t0's delta is 0: `first` and since = 0 are both set at the arm and both change only on a
    // step, so since is still 0 while first is up and needs no mux here
    wire [DW-1:0] dl_a    = since;
    wire [DW-1:0] dl_b    = ev_a ? {{(DW-1){1'b0}}, 1'b1} : since + 1'b1;
    wire [23:0]   e0      = {dl_a, ra};
    wire [23:0]   e1      = {dl_b, rb};
    wire [23:0]   e_one   = ev_a ? e0 : e1;            // the event when only one fired
    wire          two     = ev_a & ev_b;
    wire          any     = ev_a | ev_b;
    wire [23:0]   e_end   = {{(24-2){1'b0}}, cause};   // END: delta 0, state = the cause

    // A word goes out when two events meet: two this cycle, or one plus the held one.
    wire          step      = run & (~hold | pretrig);
    wire          need_word = two | (lv & any);
    wire [47:0]   run_word  = lv ? {e_one, l} : {e1, e0};
    assign wd   = ending ? (lv ? {e_end, l} : {e_end, e_end}) : run_word;
    wire   room_ok = (|room[23:4]) | (&room[3:2]);       // room >= 12: this word and END fit
    assign wstb = (ending | (step & need_word & room_ok)) & wrdy;
    // the unit counter steps where `left` does below
    wire   take   = ~rst & ~start & ~ending & step & ~(need_word & (~wrdy | ~room_ok));
    assign cnt_en = take & ~hold & (pre == 7'd127);
    // room: loads `limit` at the arm and loses 6 per post-trigger word, in one logic cell per bit
    // (lc_counter adding -6; an RTL loadable subtract maps to two LUTs per bit)
    lc_counter #(.W(24), .K(24'hFFFFFA), .CI(0)) room_i (
        .clk(clk), .rst(1'b0), .load(~rst & start), .val(limit),
        .en(take & need_word & ~hold), .hold0(1'b0), .q(room), .co());

    always @(posedge clk) begin
        if (rst) begin
            run <= 1'b0; ending <= 1'b0; busy <= 1'b0; done <= 1'b0; overflow <= 1'b0;
            lv <= 1'b0;
        end else if (start) begin
            run      <= (count != 24'd0);
            busy     <= (count != 24'd0);
            ending   <= 1'b0;
            done     <= (count == 24'd0);
            overflow <= 1'b0;
            cause    <= 2'd0;
            first    <= 1'b1;
            since    <= {DW{1'b0}};
            pre      <= 7'd0;
            if (!EXT_CNT) left <= count;
            lv       <= 1'b0;
        end else if (ending) begin
            if (wrdy) begin
                ending <= 1'b0; busy <= 1'b0; done <= 1'b1; lv <= 1'b0;
            end
        end else if (step) begin
            if (need_word && (!wrdy || !room_ok)) begin
                // ring full, or no room for this word and END: stop before this cycle's events,
                // keep everything before exact
                run <= 1'b0; ending <= 1'b1; overflow <= 1'b1;
                cause <= wrdy ? 2'd2 : 2'd1;
            end else begin
                // room loses 6 for a word here (room_i); pre-trigger words don't: the firmware reserves them
                first <= 1'b0;
                p     <= rb;
                since <= ev_b ? {{(DW-1){1'b0}}, 1'b1}
                       : ev_a ? {{(DW-2){1'b0}}, 2'd2}
                       :        since + 2'd2;
                // pair bookkeeping: what is held after this cycle
                if (lv) begin
                    if (two) l <= e1;
                    lv <= lv ^ any ^ two;   // held+1 -> word, none held; held+2 -> word, e1 held
                end else if (any && !two) begin
                    l <= e_one; lv <= 1'b1;
                end
                if (!hold) begin                        // only the post-trigger ticks count
                    pre <= pre + 7'd1;
                    if (!EXT_CNT && pre == 7'd127) left <= left - 24'd1;
                end
                if (last) begin run <= 1'b0; ending <= 1'b1; end
            end
        end
    end
endmodule
