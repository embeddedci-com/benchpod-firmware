// ============================================================================
// tb_dac_loop_frames.v — every DAC frame the closed loop sends is a value the loop's
// output `v` actually held: never a TORN frame mixing two of them.
//
// dac8551_engine takes a stream sample as two pops, the low byte (S_PLO) and then the
// high byte (S_PHI) two clk48 cycles later.  Before v38, dac_loop served both bytes
// straight from `v`, so a tick that updated `v` between the two pops sent
// {new high byte, old low byte}: e.g. 0x00FF -> 0x0100 went out as 0x01FF.  The loop
// drives real power, so one such frame is a real output glitch.
//
// Real dac_loop + real dac8551_engine (stream mode) on one clk48, frames decoded off the
// SPI pins exactly like tb_dac8551.  The loop is made to move `v` several times per DAC
// frame (sweep source over an all-distinct ramp curve, k = 1.0, the shortest tick and DAC
// divider), so ticks land on every phase of the frame.  Every decoded frame must be a value
// `v` held at some point; the run must also produce many frames and many distinct values,
// so a stuck loop cannot pass.
//
// Also the telemetry contract (v39) top_v2's cdc_pulse_payload relies on: v_out/in_used/tripped
// change ONLY on the edge that raises tlm_stb, and then hold >= TLM_HOLD clk48 (the clk-side
// latch lands within 3 clk = 6 clk48).  This bench runs the shortest tick, the worst case.
// Run: make loopframetest
// ============================================================================
`timescale 1ns/1ps
module tb_dac_loop_frames;
    localparam ADDR_W = 12, SHIFT = 5;

    reg clk48 = 0; always #10.4166 clk48 = ~clk48;
    reg rst48 = 1, arm = 0, start = 0, stop = 0;

    wire [ADDR_W-1:0] lut_raddr;
    reg  [7:0]  lut_rdata;
    wire [7:0]  strm_data; wire strm_valid, strm_pop;
    wire [15:0] v_out, in_used;
    wire        tripped, tlm_stb;
    wire [11:0] wave_addr;
    wire        sync, sclk, din, running;

    dac_loop #(.ADDR_W(ADDR_W), .SHIFT(SHIFT)) loop (
        .clk48(clk48), .rst48(rst48), .arm(arm),
        .adc_sample(16'd0),
        .k_q15(16'd32767), .vmin(16'd0), .vmax(16'hFFFF), .tick_div(16'd0),
        .src_sel(2'd2), .in_fixed(16'd0), .sweep_step(16'd7 << SHIFT),   // index +7 per tick
        .in_zero(16'd0), .in_gain(16'sd0), .in_trip(11'd0),
        .map_en(1'b0), .trip_en(1'b0),
        .lut_raddr(lut_raddr), .lut_rdata(lut_rdata),
        .strm_data(strm_data), .strm_valid(strm_valid), .strm_pop(strm_pop),
        .v_out(v_out), .in_used(in_used), .tripped(tripped),
        .tlm_stb(tlm_stb)
    );
    dac8551_engine #(.ADDR_W(12)) dac (
        .clk(clk48), .rst(rst48), .start(start), .stop(stop),
        .period_samples(13'd0), .divider(16'd0),
        .wave_addr(wave_addr), .wave_data(8'd0),
        .psram_mode(1'b1), .strm_data(strm_data), .strm_valid(strm_valid), .strm_pop(strm_pop),
        .dac_sync(sync), .dac_sclk(sclk), .dac_din(din), .running(running)
    );

    // ---- curve: every entry distinct, and both bytes move from one entry to the next ----
    reg [7:0] lut [0:(1<<ADDR_W)-1];
    always @(posedge clk48) lut_rdata <= lut[lut_raddr];
    integer i;
    reg [15:0] val;
    initial for (i = 0; i < (1<<(ADDR_W-1)); i = i + 1) begin
        val = 16'd1000 + i[15:0] * 16'd29;
        lut[{i[ADDR_W-2:0],1'b0}] = val[7:0];
        lut[{i[ADDR_W-2:0],1'b1}] = val[15:8];
    end

    // ---- every value v has held ----
    reg seen [0:65535];
    integer nvals = 0;
    initial for (i = 0; i < 65536; i = i + 1) seen[i] = 1'b0;
    always @(posedge clk48) if (!seen[loop.v]) begin seen[loop.v] = 1'b1; nvals = nvals + 1; end

    // ---- telemetry contract: change only with tlm_stb, then hold >= TLM_HOLD clk48 ----
    localparam TLM_HOLD = 8;
    reg  [32:0] tlm_prev = 33'd0;
    integer     since_stb = 1000, tlm_bad = 0, nstb = 0;
    always @(posedge clk48) if (!rst48) begin
        if ({tripped, in_used, v_out} !== tlm_prev) begin
            if (!tlm_stb) begin                              // stb is registered with the change
                tlm_bad = tlm_bad + 1;
                if (tlm_bad <= 3) $display("FAIL telemetry changed without tlm_stb (%09h -> %09h)",
                                           tlm_prev, {tripped, in_used, v_out});
            end else if (since_stb < TLM_HOLD) begin
                tlm_bad = tlm_bad + 1;
                if (tlm_bad <= 3) $display("FAIL telemetry held only %0d clk48 (want >= %0d)", since_stb, TLM_HOLD);
            end
        end
        if (tlm_stb && since_stb < TLM_HOLD) begin               // cdc_pulse_payload: strobes
            tlm_bad = tlm_bad + 1;                                // >= 2 clk (4 clk48) apart, and
            if (tlm_bad <= 3) $display("FAIL telemetry strobes only %0d clk48 apart", since_stb);
        end                                                       // the value held 4 clk
        if (tlm_stb) begin since_stb = 1; nstb = nstb + 1; end else since_stb = since_stb + 1;
        tlm_prev = {tripped, in_used, v_out};
    end

    // ---- frame decoder (as tb_dac8551): DIN taken on SCLK falling with SYNC low ----
    integer errors = 0, bits = 0, nfr = 0, torn = 0, k, stale = 0;
    reg     ps = 1'b1, pc = 1'b0, pd = 1'b0;
    reg [23:0] sh = 24'd0;
    always @(posedge clk48) if (!rst48) begin
        if (!sclk && pc && !ps) begin
            sh = {sh[22:0], pd};
            bits = bits + 1;
            if (bits == 24) begin
                nfr = nfr + 1;
                if (!seen[sh[15:0]]) begin
                    torn = torn + 1;
                    if (torn <= 5)
                        $display("FAIL frame %0d = %04h: never a value of v (torn lo/hi)", nfr, sh[15:0]);
                end
            end
        end
        if (!sync && ps) bits = 0;
        ps = sync; pc = sclk; pd = din;
    end

    initial begin
        repeat (5) @(posedge clk48); rst48 = 0;
        @(negedge clk48) arm = 1;
        @(negedge clk48) start = 1; @(negedge clk48) start = 0;
        repeat (60000) @(posedge clk48);
        // Disarm 0..3 clk48 after a tick's commit (S_CLAMP), then re-arm: the disarm's own update
        // must not land on the commit's strobe, and once quiet the published snapshot must be the
        // loop's real (disarmed) state, not the last tick's.
        for (k = 0; k < 200; k = k + 1) begin
            @(posedge clk48); while (loop.st != 4'd9) @(posedge clk48);
            repeat (k % 4) @(posedge clk48);
            @(negedge clk48) arm = 0;           // STOP_DAC: the loop disarms, the engine stops
            repeat (2) @(negedge clk48); stop = 1; @(negedge clk48) stop = 0;
            repeat (40) @(posedge clk48);
            if ({tripped, in_used, v_out} !== {loop.trip_pub, loop.in_pub, loop.v}) begin
                stale = stale + 1;
                if (stale <= 3) $display("FAIL telemetry stale after a disarm: %09h, loop is %09h",
                                         {tripped, in_used, v_out}, {loop.trip_pub, loop.in_pub, loop.v});
            end
            // re-arm as top_v2 does (START_DAC_LOOP: arm + engine start)
            @(negedge clk48) arm = 1;
            @(negedge clk48) start = 1; @(negedge clk48) start = 0;
            repeat (30 + (k % 7) * 13) @(posedge clk48);
        end
        if (stale != 0) begin
            $display("FAIL: %0d of 200 disarms left stale telemetry", stale); errors = errors + 1;
        end
        if (torn != 0) begin
            $display("FAIL: %0d of %0d DAC frames were torn", torn, nfr); errors = errors + 1;
        end
        if (tlm_bad != 0) begin
            $display("FAIL: %0d telemetry contract violations", tlm_bad); errors = errors + 1;
        end
        if (nstb < 1000) begin
            $display("FAIL: only %0d telemetry strobes", nstb); errors = errors + 1;
        end
        if (nfr < 500 || nvals < 1000) begin
            $display("FAIL: only %0d frames / %0d distinct v values: the loop did not exercise the DAC", nfr, nvals);
            errors = errors + 1;
        end
        if (errors == 0)
            $display("PASS tb_dac_loop_frames: %0d DAC frames over %0d distinct v values, none torn; %0d telemetry snapshots, each held >= %0d clk48, 200 disarms right after a commit",
                     nfr, nvals, nstb, TLM_HOLD);
        $finish;
    end
endmodule
