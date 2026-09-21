// ============================================================================
// dac_loop.v — deterministic in-fabric DAC control engine (gateware >= v23;
// selectable loop INPUT SOURCE since v29).
//
// Turns the DAC into a scriptable transfer function: each control TICK it takes an
// INPUT value, looks the target output up on a reloadable CURVE (out = curve[in]),
// damps toward it and clamps, then drives that value out the DAC8551.  The loop runs
// entirely in-fabric (input + BRAM curve + DSP + DAC) so it is DETERMINISTIC — no host
// in the control path — and touches NO PSRAM, so an LA capture can run alongside it.
// A solar-panel / MPPT emulator is one curve you can load (input = the DUT's demanded
// current, output = the panel voltage); nothing here is panel-specific.
//
// ---- Where the INPUT comes from (SRC_SEL, v29) ------------------------------
//   SRC_ADC   (0) live ADC reading   -> CLOSED loop: the output reacts to the DUT.
//   SRC_FIXED (1) `in_fixed` register -> OPEN loop: the host holds one point of the
//                 curve, so the DAC output can be metered against curve[in] with the
//                 ADC/front-end taken entirely out of the picture.  This is the first
//                 bring-up test of any new curve or output path.
//   SRC_SWEEP (2) internal accumulator: in += sweep_step every tick (mod 65536)
//                 -> OPEN loop function of TIME, walking the whole curve at a
//                 deterministic rate with no ADC involved.
// The source can change while armed; the next tick uses the new one.  `in_used`
// reports the value the last tick actually indexed with (telemetry read-back), so a
// host never has to infer the loop's input.
//
// ---- INPUT CONDITIONER (v30): why the raw count was never the right index ----
// Until v30 the curve index was the RAW input count (`in >> SHIFT`).  That is only
// sane when the input axis happens to BE the count axis.  On this board's analog front
// end it is not: the fit is volts = 65.7889 - 0.001003762*count (cal_data.h ADC_CAL_EXT),
// i.e. INVERTING, offset ~65.79 V, and WRAPPING past the 16-bit top near 0 V.  A real
// bench window — say 0..1 A through a 0.04 ohm shunt and an INA282 — lands on counts
// 63550..65542(->6): ~1992 counts, 3% of the index range, running BACKWARDS, straddling
// the wrap.  So the curve got indexed inside 3% of itself, in reverse, and only ~60 of
// the 2048 LUT entries were ever reachable.
//
// The conditioner puts an affine map in front of the index:
//
//     d   = in - in_zero                (16-bit MODULAR subtract, always)
//     idx = sat_0_2047( (d * in_gain) >>> 15 )
//
// Three properties earn their keep:
//   * in_gain is SIGNED, so an inverting front end is just a negative gain — the curve
//     is authored in the natural direction and the hardware runs it the right way round.
//   * the modular subtract IS the unwrap.  For any window narrower than 32768 counts,
//     `in - in_zero` read as signed is exactly the true distance even when the count
//     wrapped between them, so the seam at 0 V costs nothing and needs no special case.
//   * saturation is the physically correct clamp: below the window the source is
//     unloaded (index 0), above it, past short-circuit (index 2047).
//
// |in_gain| <= 1.0 is all that is ever useful: one LUT entry per input count is the
// hardware's own limit, and the map only ever needs to SHRINK a wide window onto 2048
// entries. With the reference bench (1992 counts) the window maps ~1:1 and the loop
// resolves 0.50 mA — one ADC LSB, the floor.
//
// map_en=0 restores `in >> SHIFT` bit-for-bit (the multiplier result is simply not
// selected), so a host that knows nothing about the conditioner behaves exactly as on
// v29.  The two extra pipeline states run either way, so tick timing is one constant.
//
// ---- TRIP + SLEW (v30): this loop now drives real power ----
// With an op-amp/MOSFET stage on the output the loop can command real current into a
// real DUT, so two bounds moved into fabric where they cannot be missed:
//   * in_trip  — index at or past which the loop LATCHES tripped and forces v = vmin.
//                A shorted DUT otherwise has the loop commanding full current.
// A programmable per-tick SLEW BOUND was designed alongside the trip and then CUT: two
// 17-bit variable compares plus a negate cost ~50 carry cells on an image that could not
// place at all with them in.  k_q15 already bounds the step as a fraction of the error and
// tick_div bounds the rate, which is most of what the slew bound would have added; the trip
// is the part that actually protects the pass device, so the trip is what stayed.  Revisit
// if bench testing finds a transient the loop gain cannot tame.
//
// ---- Curve = the existing waveform BRAM (sample_buf), no new BRAM ----
// In loop mode the DAC8551 is fed from this engine (strm source), so its
// sample_buf READ port is free — we reuse it as the curve LUT.  The host loads the
// curve with the ordinary LOAD_WAVE (16-bit LE, 2 bytes/entry).  The LUT is indexed
// by the top (ADDR_W-1) bits of the input value: idx = in >> SHIFT, byte addr =
// {idx, byte_sel}.  SHIFT is compile-time (no runtime barrel shifter) — 2048-entry
// curve over the full 16-bit input range at SHIFT=5.
//
// ---- The loop, per tick (pipelined over clk48 so no long combinational path) ----
//   target = curve[in >> SHIFT]                         (2 BRAM reads: lo, hi)
//   delta  = clamp16(target - v)                        (slew-bounded error)
//   v     += (k * delta) >>> 15                         (Q15 damping, ONE DSP mult)
//   v      = clamp(v, vmin, vmax)                        (output window)
// The DAC8551 continuously streams the current `v` (this engine is a 2-byte strm
// producer, LE, byte-aligned from arm — same contract as dac_psram_reader).
//
// Inert when not armed (arm=0): the FSM idles, strm_valid=0, lut_raddr=0 — so the
// top-level muxes hand the DAC/BRAM back to the normal replay paths untouched.
// ============================================================================
module dac_loop #(
    parameter ADDR_W = 12,          // sample_buf byte-address width (4 KB => 12)
    parameter SHIFT  = 5            // in >> SHIFT = curve index (2048 pts @ SHIFT=5)
)(
    input  wire        clk48,
    input  wire        rst48,
    input  wire        arm,              // level: 1 = loop active

    // live ADC reading, already synchronised into clk48 (see top_v2)
    input  wire [15:0] adc_sample,

    // control parameters (clk48; held stable while armed)
    input  wire [15:0] k_q15,            // damping coefficient, Q15 (32768 = 1.0)
    input  wire [15:0] vmin,             // output clamp low
    input  wire [15:0] vmax,             // output clamp high
    input  wire [15:0] tick_div,         // control period in clk48 cycles (>= 8)

    // loop INPUT source (v29).  0 = live ADC, 1 = in_fixed, 2/3 = internal sweep.
    input  wire [1:0]  src_sel,
    input  wire [15:0] in_fixed,         // SRC_FIXED value; also the SRC_SWEEP start
    input  wire [15:0] sweep_step,       // SRC_SWEEP increment per tick (mod 65536)

    // ---- INPUT CONDITIONER (v30) — see the header block ------------------------
    input  wire [15:0] in_zero,          // input value that maps to curve index 0
    input  wire signed [15:0] in_gain,   // Q15 index gain; NEGATIVE for an inverting front end
    input  wire [10:0] in_trip,          // trip threshold, as a curve index (0..2047)
    input  wire        map_en,           // 0 = legacy idx = in >> SHIFT, conditioner bypassed
    input  wire        trip_en,          // 1 = arm the over-range trip

    // curve LUT read port (drives sample_buf's read port at the top when armed)
    output reg  [ADDR_W-1:0] lut_raddr,
    input  wire [7:0]        lut_rdata,

    // 2-byte LE strm source into dac8551_engine (mux'd with the PSRAM reader at top)
    output wire [7:0]  strm_data,
    output wire        strm_valid,
    input  wire        strm_pop,

    output wire [15:0] v_out,            // current output, for telemetry read-back
    output wire [15:0] in_used,          // input the last tick indexed with (telemetry)
    output wire [15:0] idx_used,         // curve index that input resolved to (telemetry)
    output wire        tripped           // latched: the trip fired, output forced to vmin
);
    localparam [1:0] SRC_ADC = 2'd0, SRC_FIXED = 2'd1;
    // ---- output register + 2-byte strm producer -------------------------------
    reg  [15:0] v;
    reg         byte_sel;                // 0 => low byte next, 1 => high byte
    assign strm_data  = byte_sel ? v[15:8] : v[7:0];
    assign strm_valid = arm;             // always ready while armed (DAC holds `v`)
    assign v_out      = v;

    always @(posedge clk48) begin
        if (rst48 || !arm) byte_sel <= 1'b0;         // arm => aligned, low byte first
        else if (strm_pop) byte_sel <= ~byte_sel;
    end

    // ---- damping multiply (inferred SB_MAC16 DSP: k_q15 * delta, both signed) ---
    // Pipelined so no clk48 cycle carries MAC16 + adds + clamp at once (that path only
    // closed ~44 MHz).  prod_r is a registered MAC16 output (fast); the add and the clamp
    // then live in their own cycles.  Extra latency is a few clk48 per tick — negligible.
    reg  signed [15:0] delta_q;          // clamped error, signed 16-bit
    reg  signed [15:0] k_s;              // k_q15 as signed (always >= 0)
    reg  signed [31:0] prod_r;           // registered DSP product (absorbed into SB_MAC16)
    reg  signed [17:0] vsum;             // v + damped step, pre-clamp

    // ---- control FSM (clk48), one pipelined pass per tick ----------------------
    // S_IMUL/S_IDX (v30) run the input conditioner's multiply and saturate BEFORE the
    // LUT address exists. They run in every mode — including map_en=0, where the result
    // is simply not selected — so one tick is one constant number of cycles either way
    // and the host's sweep arithmetic has nothing to branch on.
    localparam S_IDLE=4'd0, S_IMUL=4'd1, S_IDX=4'd2, S_RDLO=4'd3, S_RDHI=4'd4,
               S_ASM=4'd5, S_MUL=4'd6, S_MAC=4'd7, S_UPD=4'd8, S_CLAMP=4'd9;
    reg [3:0]  st;
    // Tick timer: a registered DOWN-counter (reload = tick_div, fire at 0) with a
    // registered `tick_z` flag, so the S_IDLE fire test reads ONE bit instead of a
    // 16-bit `tick_cnt >= tick_div` magnitude compare.  That compare was the last wide
    // magnitude comparator in the clk48 domain — the exact class of path that makes
    // clk48 seed-fragile (mirrors dac8551_engine `dz` / adc_mcp33131 `period_zero`).
    // Cycle-exact with the old up-counter (dwell = tick_div+1 S_IDLE cycles) for all
    // tick_div incl. 0/1, so tb_dac_loop timing is unchanged.
    reg [15:0] tick_cnt;
    reg        tick_z;                    // registered (tick_cnt == 0): fire next S_IDLE
    reg [7:0]  lo;
    reg [15:0] target;
    reg signed [16:0] delta17;

    // ---- loop input selection (v29) -------------------------------------------
    // The sweep accumulator holds `in_fixed` while disarmed, so a sweep always STARTS
    // from the host-set point (and a fixed->sweep switch continues from where the fixed
    // point was) instead of from whatever the previous run left behind.
    reg  [15:0] sweep_acc;
    wire [15:0] in_now = (src_sel == SRC_ADC)   ? adc_sample :
                         (src_sel == SRC_FIXED) ? in_fixed   : sweep_acc;
    reg  [15:0] in_lat;                   // the input the last tick actually used
    assign in_used = in_lat;

    // ---- input conditioner datapath (v30) --------------------------------------
    // The legacy index, kept as its own wire so map_en=0 is provably unchanged.
    // The legacy index, read off the already-latched input rather than a register of its
    // own, so map_en=0 costs nothing but the mux.
    wire [ADDR_W-2:0] idx_leg = in_lat[15:SHIFT];

    // d = in - in_zero, ALWAYS as a 16-bit modular subtract read as signed. For any window
    // narrower than 32768 counts that is exactly the unwrapped distance, so the ADC's wrap
    // costs nothing. A window wider than that is refused by the host (it cannot be indexed
    // unambiguously anyway), and an axis that genuinely wants the whole count range is what
    // map_en=0 is for -- so there is no second subtract mode to pay for.
    wire signed [15:0] d_now = $signed(in_now - in_zero);
    reg  signed [15:0] d_r;                  // registered at the tick
    wire signed [31:0] prod_f = d_r * in_gain;   // 2nd SB_MAC16 (hard block)
    reg  signed [16:0] prod_i;               // only (d*gain)>>>15 is ever read

    // (d*gain) >>> 15, saturated into the LUT's 0..2047. Negative (below the window) is
    // the unloaded end; over-range is the short-circuit end.
    wire signed [16:0] idx_s   = prod_i;
    wire               idx_neg = idx_s[16];
    wire               idx_ovf = ~idx_neg & (|idx_s[15:ADDR_W-1]);
    wire [ADDR_W-2:0]  idx_sat = idx_neg ? {(ADDR_W-1){1'b0}} :
                                 idx_ovf ? {(ADDR_W-1){1'b1}} : idx_s[ADDR_W-2:0];
    wire [ADDR_W-2:0]  idx_next = map_en ? idx_sat : idx_leg;

    reg trip_l;                              // latched until disarm
    assign tripped  = trip_l;
    assign idx_used = {{(17-ADDR_W){1'b0}}, lut_raddr[ADDR_W-1:1]};

    always @(posedge clk48) begin
        if (rst48 || !arm) begin
            st <= S_IDLE; tick_cnt <= tick_div; tick_z <= (tick_div == 16'd0); v <= vmin;
            lut_raddr <= {ADDR_W{1'b0}}; lo <= 8'd0; target <= 16'd0;
            delta_q <= 16'sd0; k_s <= 16'sd0; delta17 <= 17'sd0;
            prod_r <= 32'sd0; vsum <= 18'sd0;
            sweep_acc <= in_fixed;        // a sweep starts from the host-set point
            in_lat    <= 16'd0;
            d_r <= 16'sd0; prod_i <= 17'sd0;
            trip_l <= 1'b0;               // the trip clears only on disarm/reset
        end else begin
            case (st)
            S_IDLE: begin
                // lut_raddr is NOT cleared while idling: it is the index register now (see
                // S_RDLO), and holding it keeps `idx_used` telemetry readable between ticks.
                // Harmless — the loop owns the curve BRAM's read port while armed, and the
                // held address just re-reads a byte nothing samples until the next tick.
                if (tick_z) begin
                    tick_cnt <= tick_div;                // reload for the next tick
                    tick_z   <= (tick_div == 16'd0);
                    in_lat   <= in_now;                  // telemetry: what this tick used
                    d_r      <= d_now;                   // conditioner: distance from zero
                    // Advance the time sweep once per tick.  Free-running (mod 65536) in
                    // every mode so a source switch never resumes a stale ramp: while a
                    // fixed/ADC source is selected the accumulator is held at in_fixed.
                    sweep_acc <= (src_sel[1] ? sweep_acc + sweep_step : in_fixed);
                    st <= S_IMUL;
                end else begin
                    tick_cnt <= tick_cnt - 16'd1;
                    tick_z   <= (tick_cnt == 16'd1);     // reaches 0 next cycle
                end
            end
            S_IMUL: begin
                prod_i <= prod_f[31:15];                 // (d*gain)>>>15
                st <= S_IDX;
            end
            S_IDX: begin
                lut_raddr <= {idx_next, 1'b0};           // curve[idx] low byte
                st <= S_RDLO;
            end
            S_RDLO: begin
                // lut_raddr already carries the index from S_IDX, so there is no separate
                // `idx` register to keep in step with it.
                lut_raddr[0] <= 1'b1;                    // high byte (lo arrives next cyc)
                // Trip test runs HERE, off the REGISTERED index, not in S_IDX off the
                // combinational one: the saturate already costs ~4 LUT levels out of the
                // multiplier, and hanging an 11-bit comparator's carry chain on the end of
                // it was the clk48 critical path (42 MHz, FAIL at 48).  The output is still
                // parked before the curve value can reach it — S_CLAMP is five states away.
                if (map_en && trip_en && (lut_raddr[ADDR_W-1:1] >= in_trip)) trip_l <= 1'b1;
                st <= S_RDHI;
            end
            S_RDHI: begin
                lo <= lut_rdata;                         // low byte captured
                st <= S_ASM;
            end
            S_ASM: begin
                target  <= {lut_rdata, lo};              // {hi, lo}
                // delta = clamp16(target - v)
                delta17 <= $signed({1'b0, {lut_rdata, lo}}) - $signed({1'b0, v});
                st <= S_MUL;
            end
            S_MUL: begin
                // slew-bound the error to signed 16-bit for the DSP
                delta_q <= (delta17 > 17'sd32767)  ? 16'sd32767 :
                           (delta17 < -17'sd32768) ? -16'sd32768 : delta17[15:0];
                k_s     <= $signed({1'b0, k_q15[14:0]}); // Q15 magnitude (>=0)
                st <= S_MAC;
            end
            S_MAC: begin
                prod_r <= k_s * delta_q;                 // registered DSP product
                st <= S_UPD;
            end
            S_UPD: begin
                // v + (k*delta)>>>15 (round-to-nearest) — one registered add
                vsum <= $signed({2'b0, v}) + $signed(prod_r[30:15] + prod_r[14]);
                st <= S_CLAMP;
            end
            S_CLAMP: begin
                // A latched trip outranks the curve: the loop keeps running (so the host
                // can see the input that tripped it) but the output is parked at vmin
                // until the loop is disarmed.
                if (trip_l)                            v <= vmin;
                else if (vsum < $signed({2'b0, vmin})) v <= vmin;
                else if (vsum > $signed({2'b0, vmax})) v <= vmax;
                else                                   v <= vsum[15:0];
                st <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end
endmodule
