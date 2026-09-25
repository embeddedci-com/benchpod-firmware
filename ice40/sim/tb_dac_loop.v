// ============================================================================
// tb_dac_loop.v — self-checking test of the closed-loop DAC control engine.
//
// Models the curve LUT (like sample_buf: registered 1-cycle read) and a simple
// "plant" (adc = f(v)) closing the loop, then checks:
//   * CONVERGENCE: with a flat curve = TARGET and a direct-loopback plant, v damps
//     to TARGET and holds (steady state).
//   * CLAMP: with vmax below TARGET, v settles at vmax (never overshoots the clamp).
//   * STRM: the 2-byte LE producer hands out {lo,hi} of the current v, byte-aligned.
//   * DETERMINISM: the same arm run reproduces the same trajectory.
//   * CURVE INDEX: with the plant detached, a held ADC value drives the output to
//     curve[adc >> SHIFT] out of an all-distinct ramp curve — the one check a loop that
//     ignored the ADC (or indexed it wrongly) cannot pass, since every other case above
//     uses a FLAT curve and a wide-open clamp would explain the result equally well.
//   * FIXED SOURCE (v29): with src=1 the loop indexes the curve with the host-set value and
//     IGNORES the ADC — checked with the ADC parked on a DIFFERENT curve entry, so a loop
//     that quietly kept reading the ADC fails.  This is the open-loop bring-up mode.
//   * SWEEP SOURCE (v29): with src=2 the input advances by `step` every tick with no ADC in
//     the path, so the output walks the curve as a function of TIME (checked at two points
//     of the ramp, plus that it actually moved).
//   * in_used TELEMETRY: reports the input the last tick indexed with, in every source mode.
// Run: make looptest
// ============================================================================
`timescale 1ns/1ps
module tb_dac_loop;
    localparam ADDR_W = 12, SHIFT = 5;

    reg clk48 = 0; always #10.4166 clk48 = ~clk48;
    reg rst48 = 1, arm = 0;
    reg  [15:0] adc_sample = 0;
    reg  [15:0] k_q15 = 16'd0, vmin = 16'd0, vmax = 16'hFFFF, tick_div = 16'd8;
    reg  [1:0]  src_sel = 2'd0;                  // 0 = ADC (pre-v29 behaviour)
    reg  [15:0] in_fixed = 16'd0, sweep_step = 16'd0;
    // v30 input conditioner: all-zero + map_en 0 = the legacy `in >> SHIFT` index, so every
    // pre-v30 case below runs against exactly the old datapath.
    reg  [15:0] in_zero = 16'd0, in_trip = 16'd0;
    reg  signed [15:0] in_gain = 16'sd0;
    reg  map_en = 1'b0, trip_en = 1'b0;
    wire [ADDR_W-1:0] lut_raddr;
    reg  [7:0]  lut_rdata;
    wire [7:0]  strm_data; wire strm_valid; reg strm_pop = 0;
    wire [15:0] v_out;
    wire [15:0] in_used;
    // the curve index the last tick resolved to, read off the loop's registered LUT address
    wire [15:0] idx_used = {{(17-ADDR_W){1'b0}}, dut.lut_raddr[ADDR_W-1:1]};
    wire        tripped;

    dac_loop #(.ADDR_W(ADDR_W), .SHIFT(SHIFT)) dut (
        .clk48(clk48), .rst48(rst48), .arm(arm),
        .adc_sample(adc_sample),
        .k_q15(k_q15), .vmin(vmin), .vmax(vmax), .tick_div(tick_div),
        .src_sel(src_sel), .in_fixed(in_fixed), .sweep_step(sweep_step),
        .in_zero(in_zero), .in_gain(in_gain), .in_trip(in_trip[10:0]),
        .map_en(map_en), .trip_en(trip_en),
        .lut_raddr(lut_raddr), .lut_rdata(lut_rdata),
        .strm_data(strm_data), .strm_valid(strm_valid), .strm_pop(strm_pop),
        .v_out(v_out), .in_used(in_used), .tripped(tripped)
    );

    // ---- curve LUT model: 4 KB byte BRAM, registered read (matches sample_buf) ----
    reg [7:0] lut [0:(1<<ADDR_W)-1];
    always @(posedge clk48) lut_rdata <= lut[lut_raddr];

    task load_flat(input [15:0] val); integer i; begin
        for (i = 0; i < (1<<(ADDR_W-1)); i = i + 1) begin
            lut[{i[ADDR_W-2:0],1'b0}] = val[7:0];
            lut[{i[ADDR_W-2:0],1'b1}] = val[15:8];
        end
    end endtask

    // A curve whose every entry DIFFERS, so a wrong LUT index is visible as a wrong output
    // (a flat curve cannot tell "indexed by the ADC" from "ignored the ADC").
    function [15:0] ramp_val(input integer idx); begin
        ramp_val = 16'd1000 + idx[15:0] * 16'd29;   // idx 0..2047 -> 1000..60363, all distinct
    end endfunction
    task load_ramp; integer i; reg [15:0] val; begin
        for (i = 0; i < (1<<(ADDR_W-1)); i = i + 1) begin
            val = ramp_val(i);
            lut[{i[ADDR_W-2:0],1'b0}] = val[7:0];
            lut[{i[ADDR_W-2:0],1'b1}] = val[15:8];
        end
    end endtask

    // ---- plant: adc = v (direct DAC->ADC loopback), registered (plant latency) ----
    // plant_en=0 detaches it so a test can HOLD the ADC at a chosen "current" and check the
    // output the curve prescribes for it (open-loop index check).
    reg plant_en = 1'b1;
    always @(posedge clk48) if (arm && plant_en) adc_sample <= v_out;

    // ---- DAC popping the strm at a steady rate (like dac8551 in strm mode) ----
    integer popc = 0;
    always @(posedge clk48) begin
        strm_pop <= 1'b0;
        if (arm) begin popc <= popc + 1; if (popc[2:0] == 3'd0) strm_pop <= 1'b1; end
    end

    integer errors = 0, i;
    reg [15:0] traj1;

    task run_arm; begin
        arm = 0; @(posedge clk48); @(posedge clk48);
        arm = 1;
        // let the loop iterate to steady state
        repeat (4000) @(posedge clk48);
    end endtask

    initial begin
        errors = 0;
        repeat (5) @(posedge clk48); rst48 = 0;

        // ---- 1. convergence to a flat curve ----
        k_q15 = 16'd8192;   // ~0.25 damping
        vmin = 16'd0; vmax = 16'hFFFF; tick_div = 16'd8;
        load_flat(16'd30000);
        run_arm;
        if (v_out < 16'd29900 || v_out > 16'd30100) begin
            $display("FAIL convergence: v=%0d want ~30000", v_out); errors = errors + 1;
        end else $display("  converge: v=%0d -> TARGET 30000 OK", v_out);
        traj1 = v_out;

        // ---- 2. clamp: vmax below target ----
        vmax = 16'd20000;
        load_flat(16'd30000);
        run_arm;
        if (v_out !== 16'd20000) begin
            $display("FAIL clamp: v=%0d want 20000 (vmax)", v_out); errors = errors + 1;
        end else $display("  clamp:    v=%0d pinned at vmax OK", v_out);

        // ---- 3. strm producer emits {lo,hi} of v ----
        // v is 20000 = 0x4E20 -> low 0x20, high 0x4E
        // sample the two bytes across two pops
        begin : strmcheck
            reg [7:0] b0, b1;
            vmax = 16'hFFFF; load_flat(16'd20000); run_arm;
            // capture the byte the DAC latches on two consecutive pops (lo then hi)
            @(posedge clk48); while (!strm_pop) @(posedge clk48); b0 = strm_data;
            @(posedge clk48); while (!strm_pop) @(posedge clk48); b1 = strm_data;
            if (!((b0 == v_out[7:0] && b1 == v_out[15:8]) ||
                  (b0 == v_out[15:8] && b1 == v_out[7:0]))) begin
                $display("FAIL strm: bytes %02h,%02h not the halves of v=%04h", b0, b1, v_out);
                errors = errors + 1;
            end else $display("  strm:     bytes match v=%04h OK", v_out);
        end

        // ---- 4. determinism: same run reproduces the trajectory ----
        k_q15 = 16'd8192; vmin = 16'd0; vmax = 16'hFFFF; load_flat(16'd30000);
        run_arm;
        if (v_out !== traj1) begin
            $display("FAIL determinism: v=%0d vs first run %0d", v_out, traj1); errors = errors + 1;
        end else $display("  determ:   reproduced v=%0d OK", v_out);

        // ---- 5. THE CURVE IS INDEXED BY THE ADC (not just "some constant came out") ----
        // Every earlier case uses a FLAT curve, which a loop that ignored the ADC entirely
        // would also pass.  Detach the plant, hold the ADC at a chosen current, and require
        // the output to land on curve[adc >> SHIFT] — for two currents far apart, with the
        // clamp wide open so it cannot be what produced the value.
        begin : curveidx
            reg [15:0] want;
            plant_en = 1'b0;
            @(posedge clk48); @(posedge clk48);   // let the last plant update retire first,
                                                  // else its NBA overwrites the forced value
            k_q15 = 16'd8192; vmin = 16'd0; vmax = 16'hFFFF; tick_div = 16'd8;
            load_ramp;

            adc_sample = 16'h1000;                       // idx = 0x1000>>5 = 128
            run_arm;
            want = ramp_val(16'h1000 >> SHIFT);
            if (v_out < want - 16'd16 || v_out > want + 16'd16) begin
                $display("FAIL curve index: adc=%04h -> v=%0d, want curve[%0d]=%0d",
                         16'h1000, v_out, 16'h1000 >> SHIFT, want);
                errors = errors + 1;
            end else $display("  curve[i]: adc=1000h -> v=%0d (curve[%0d]=%0d) OK",
                              v_out, 16'h1000 >> SHIFT, want);

            adc_sample = 16'hC000;                       // idx = 0xC000>>5 = 1536
            run_arm;
            want = ramp_val(16'hC000 >> SHIFT);
            if (v_out < want - 16'd16 || v_out > want + 16'd16) begin
                $display("FAIL curve index: adc=%04h -> v=%0d, want curve[%0d]=%0d",
                         16'hC000, v_out, 16'hC000 >> SHIFT, want);
                errors = errors + 1;
            end else $display("  curve[i]: adc=C000h -> v=%0d (curve[%0d]=%0d) OK",
                              v_out, 16'hC000 >> SHIFT, want);
            plant_en = 1'b1;
        end

        // ---- 6. FIXED input source (v29): the ADC is out of the path entirely ----
        // The whole point of this mode is bring-up WITHOUT the ADC: hold one curve point,
        // meter the DAC.  So park the ADC on a curve entry far from the requested one — a
        // loop that still read the ADC would land on the ADC's entry and fail.
        begin : fixedsrc
            reg [15:0] want;
            plant_en = 1'b0;
            @(posedge clk48); @(posedge clk48);
            k_q15 = 16'd8192; vmin = 16'd0; vmax = 16'hFFFF; tick_div = 16'd8;
            load_ramp;
            adc_sample = 16'hC000;                 // decoy: curve[1536], nowhere near below
            src_sel = 2'd1; in_fixed = 16'h2000;   // idx = 0x2000>>5 = 256
            run_arm;
            want = ramp_val(16'h2000 >> SHIFT);
            if (v_out < want - 16'd16 || v_out > want + 16'd16) begin
                $display("FAIL fixed src: in=2000h -> v=%0d, want curve[%0d]=%0d (adc decoy C000h)",
                         v_out, 16'h2000 >> SHIFT, want);
                errors = errors + 1;
            end else $display("  fixed:    in=2000h -> v=%0d (curve[%0d]=%0d), ADC ignored OK",
                              v_out, 16'h2000 >> SHIFT, want);
            if (in_used !== 16'h2000) begin
                $display("FAIL fixed src telemetry: in_used=%04h want 2000h", in_used);
                errors = errors + 1;
            end
            // A live input change must move the output without re-arming (the "step through
            // the curve while metering" flow the UI drives).
            in_fixed = 16'h6000;                   // idx = 768
            repeat (4000) @(posedge clk48);
            want = ramp_val(16'h6000 >> SHIFT);
            if (v_out < want - 16'd16 || v_out > want + 16'd16) begin
                $display("FAIL fixed src live update: in=6000h -> v=%0d, want %0d", v_out, want);
                errors = errors + 1;
            end else $display("  fixed:    live step to 6000h -> v=%0d OK", v_out);
        end

        // ---- 7. SWEEP input source (v29): input = f(time), no ADC ----
        begin : sweepsrc
            reg [15:0] v_early, want;
            arm = 0; @(posedge clk48); @(posedge clk48);
            k_q15 = 16'd32767;                     // full step: follow the sweep closely
            vmin = 16'd0; vmax = 16'hFFFF; tick_div = 16'd8;
            load_ramp;
            adc_sample = 16'hC000;                 // decoy again
            src_sel = 2'd2; in_fixed = 16'd0; sweep_step = 16'd64;
            arm = 1;
            repeat (2000) @(posedge clk48);
            v_early = v_out;
            if (in_used === 16'd0) begin
                $display("FAIL sweep: input never advanced (in_used=0)"); errors = errors + 1;
            end
            // The output must be the curve entry for the input the loop reports using.
            want = ramp_val(in_used >> SHIFT);
            if (v_out < want - 16'd64 || v_out > want + 16'd64) begin
                $display("FAIL sweep: in_used=%04h -> v=%0d, want curve[%0d]=%0d",
                         in_used, v_out, in_used >> SHIFT, want);
                errors = errors + 1;
            end else $display("  sweep:    in_used=%04h -> v=%0d (curve[%0d]=%0d) OK",
                              in_used, v_out, in_used >> SHIFT, want);
            repeat (4000) @(posedge clk48);
            if (v_out === v_early) begin
                $display("FAIL sweep: output stalled at %0d (input not advancing)", v_out);
                errors = errors + 1;
            end else $display("  sweep:    output advanced %0d -> %0d OK", v_early, v_out);
            // Back to the ADC source: the closed loop must still work afterwards.
            src_sel = 2'd0; plant_en = 1'b1; k_q15 = 16'd8192;
            load_flat(16'd30000);
            run_arm;
            if (v_out < 16'd29900 || v_out > 16'd30100) begin
                $display("FAIL src back to ADC: v=%0d want ~30000", v_out); errors = errors + 1;
            end else $display("  src=adc:  closed loop still converges (v=%0d) OK", v_out);
        end

        // ---- 8. INPUT CONDITIONER (v30) --------------------------------------------
        // The reference bench: 0.04 ohm shunt + INA282 (2 mV/mA) on the inverting front
        // end, 0..1000 mA. Counts run 65542(->6) down to 63550, i.e. BACKWARDS and across
        // the 16-bit wrap. in_zero = the count at 0 mA, in_gain negative so the index still
        // rises with current. 1992 counts onto 2047 entries clips |gain| to 1.0.
        begin : inmap
            reg [15:0] want;
            localparam [15:0] ZERO_CNT = 16'd6;      // 0 mA
            localparam [15:0] FULL_CNT = 16'd63550;  // 1000 mA
            arm = 0; @(posedge clk48); @(posedge clk48);
            k_q15 = 16'd32767; vmin = 16'd0; vmax = 16'hFFFF; tick_div = 16'd8;
            load_ramp; plant_en = 1'b0; src_sel = 2'd1;
            map_en = 1'b1; in_zero = ZERO_CNT; in_gain = -16'sd32767;

            // 0 mA sits on the WRAP SEAM (count 6). Without the modular subtract this reads
            // as a huge positive distance and lands at the wrong end of the curve entirely.
            in_fixed = ZERO_CNT; run_arm;
            want = ramp_val(0);
            if (v_out < want - 16'd16 || v_out > want + 16'd16) begin
                $display("FAIL inmap zero: count %0d -> v=%0d want curve[0]=%0d",
                         ZERO_CNT, v_out, want); errors = errors + 1;
            end else $display("  inmap:    0 mA (count %0d, on the wrap seam) -> curve[0] OK", ZERO_CNT);

            // Full scale must reach the far end of the curve, not 3% into it.
            in_fixed = FULL_CNT;
            repeat (4000) @(posedge clk48);
            if (idx_used < 16'd1900) begin
                $display("FAIL inmap full: count %0d -> idx %0d, want ~1992 (window must span the LUT)", FULL_CNT, idx_used); errors = errors + 1;
            end else $display("  inmap:    1000 mA (count %0d) -> idx %0d (~1:1 on 1992 counts) OK",
                              FULL_CNT, idx_used);

            // DIRECTION: this is the bug the conditioner exists to fix. On the raw axis a
            // HIGHER current is a LOWER count, so the output rose with load. It must fall.
            begin : direction
                reg [15:0] v_lo, v_hi;
                in_fixed = 16'd64546;              // 500 mA
                repeat (4000) @(posedge clk48); v_lo = v_out;
                in_fixed = 16'd63948;              // 800 mA
                repeat (4000) @(posedge clk48); v_hi = v_out;
                if (!(v_hi > v_lo)) begin
                    $display("FAIL inmap direction: 500 mA -> %0d, 800 mA -> %0d (index must RISE with current)", v_lo, v_hi); errors = errors + 1;
                end else $display("  inmap:    index rises with current (500 mA -> %0d, 800 mA -> %0d) OK",
                                  v_lo, v_hi);
            end

            // SATURATION: outside the window, pin to the ends rather than wrap into nonsense.
            in_fixed = 16'd30000;                  // unwraps far NEGATIVE -> below the window
            repeat (4000) @(posedge clk48);
            if (idx_used !== 16'd0) begin
                $display("FAIL inmap sat-low: count 30000 -> idx %0d want 0", idx_used);
                errors = errors + 1;
            end else $display("  inmap:    below-window count pins to idx 0 OK");
            in_fixed = 16'd40000;                  // +25.6 V -> way past full scale
            repeat (4000) @(posedge clk48);
            if (idx_used !== 16'd2047) begin
                $display("FAIL inmap sat-high: count 40000 -> idx %0d want 2047", idx_used);
                errors = errors + 1;
            end else $display("  inmap:    above-window count pins to idx 2047 OK");
        end

        // ---- 9. TRIP + SLEW BOUND (v30) --------------------------------------------
        begin : safety
            reg [15:0] v_before;
            // Trip at index 1500; hold an input past it and the output must park at vmin
            // and LATCH there even after the input comes back into range.
            arm = 0; @(posedge clk48); @(posedge clk48);
            k_q15 = 16'd32767; vmin = 16'd1234; vmax = 16'hFFFF; tick_div = 16'd8;
            load_ramp; plant_en = 1'b0; src_sel = 2'd1;
            map_en = 1'b1; in_zero = 16'd6; in_gain = -16'sd32767;
            in_trip = 16'd1500; trip_en = 1'b1;
            in_fixed = 16'd64546;                  // 500 mA: idx ~996, below the trip
            run_arm;
            if (tripped !== 1'b0) begin
                $display("FAIL trip: fired below the threshold (idx %0d)", idx_used);
                errors = errors + 1;
            end else $display("  trip:     idle below the threshold (idx %0d) OK", idx_used);
            in_fixed = 16'd63550;                  // 1000 mA: idx ~1992, past the trip
            repeat (4000) @(posedge clk48);
            if (tripped !== 1'b1 || v_out !== 16'd1234) begin
                $display("FAIL trip: idx %0d tripped=%0b v=%0d, want tripped + v=vmin(1234)",
                         idx_used, tripped, v_out); errors = errors + 1;
            end else $display("  trip:     fired past the threshold, output parked at vmin OK");
            in_fixed = 16'd64546;                  // back in range: must STAY tripped
            repeat (4000) @(posedge clk48);
            if (tripped !== 1'b1 || v_out !== 16'd1234) begin
                $display("FAIL trip: cleared itself when the input returned (v=%0d)", v_out);
                errors = errors + 1;
            end else $display("  trip:     latched until disarm OK");
            // Disarm clears it.
            arm = 0; repeat (4) @(posedge clk48);
            if (tripped !== 1'b0) begin
                $display("FAIL trip: not cleared by disarm"); errors = errors + 1;
            end else $display("  trip:     cleared by disarm OK");

            // The slew duty now falls to k_q15: a small damping coefficient must still make
            // the approach gradual rather than a single jump to the target.
            arm = 0; @(posedge clk48); @(posedge clk48);
            trip_en = 1'b0; map_en = 1'b0; k_q15 = 16'd328;   // ~1% of full step
            vmin = 16'd0; vmax = 16'hFFFF;
            load_flat(16'd60000); plant_en = 1'b0; src_sel = 2'd0; adc_sample = 16'd0;
            arm = 1;
            repeat (200) @(posedge clk48);         // ~12 ticks at tick_div 8
            v_before = v_out;
            if (v_before > 16'd12000) begin
                $display("FAIL damping: v=%0d after ~12 ticks at k=1%%, should still be climbing",
                         v_before); errors = errors + 1;
            end else $display("  damping:  bounded climb (v=%0d after ~12 ticks at k=1%%) OK", v_before);
            repeat (60000) @(posedge clk48);       // given time it still reaches the target
            if (v_out < 16'd59000) begin
                $display("FAIL damping: never reached the target (v=%0d)", v_out);
                errors = errors + 1;
            end else $display("  damping:  still converges to the target (v=%0d) OK", v_out);
            map_en = 1'b0;
        end

        if (errors == 0) $display("PASS tb_dac_loop: converge + clamp + strm + determinism + curve-index + fixed + sweep + inmap + trip + damping");
        else             $display("FAIL tb_dac_loop: %0d error(s)", errors);
        $finish;
    end

    initial begin #5000000 $display("FAIL tb_dac_loop: timeout"); $finish; end
endmodule
