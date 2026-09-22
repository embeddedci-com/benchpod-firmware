// tb_la_psram_capture.v — self-checking testbench for the deep LA→PSRAM sampler.
//
// Drives a known LA word, arms a short capture, and checks the byte stream the
// module presents to psram_writer: 2 little-endian bytes per sample
// (byte0 = la[7:0], byte1 = {2'b0, la[13:8]}), exactly sample_count samples, with
// ps_start at arm and ps_stop coincident with the final byte, and consecutive
// samples exactly `divider` clocks apart.  A second pass checks the `overflow`
// diagnostic latches when `full` is held.
`timescale 1ns/1ps
module tb_la_psram_capture;
    localparam N     = 14;
    localparam CNT_W = 16;

    reg              clk = 1'b0;
    reg              rst = 1'b1;
    reg  [N-1:0]     la_in = {N{1'b0}};
    reg              start = 1'b0;
    reg  [CNT_W-1:0] sample_count = 0;
    reg  [15:0]      divider = 16'd4;
    reg              full = 1'b0;
    reg              hold = 1'b0;   // v35 capture-trigger hold

    wire             ps_start, ps_stop, wr_stb, busy, done, overflow;
    wire [7:0]       wr_data;

    la_psram_capture #(.N(N), .CNT_W(CNT_W)) dut (
        .clk(clk), .rst(rst), .la_in(la_in),
        .start(start), .hold(hold), .sample_count(sample_count), .divider(divider),
        .ps_start(ps_start), .ps_stop(ps_stop),
        .wr_data(wr_data), .wr_stb(wr_stb), .full(full),
        .busy(busy), .done(done), .overflow(overflow)
    );

    always #5 clk = ~clk;   // 100 MHz sim clock (period irrelevant to logic)

    integer errors = 0;
    integer nbytes = 0;
    reg [7:0] cap [0:255];      // captured byte stream
    integer   ps_start_seen, ps_stop_seen;
    integer   ps_stop_at_byte;  // nbytes when ps_stop fired (expect == 2*count)
    integer   cyc = 0;          // clock-cycle counter (sample-period measurement)
    integer   lo_at [0:127];    // cycle of each sample's LOW-byte strobe
    integer   start_cyc = 0;    // cycle whose edge sampled `start`
    integer   release_cyc, hold_ref, k6;
    reg       hold_d = 1'b0;

    // Record the byte stream + control pulses every cycle.
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (start) start_cyc = cyc;
        if (hold_d && !hold) release_cyc = cyc;   // the first edge that sees hold low
        hold_d = hold;
        if (ps_start) ps_start_seen = ps_start_seen + 1;
        if (wr_stb) begin
            if (nbytes < 256) cap[nbytes] = wr_data;
            if (nbytes < 256 && (nbytes % 2) == 0) lo_at[nbytes/2] = cyc;
            nbytes = nbytes + 1;
        end
        if (ps_stop) begin
            ps_stop_seen   = ps_stop_seen + 1;
            ps_stop_at_byte = nbytes;   // counts the byte strobed this same cycle
        end
    end

    task reset_capture;
        begin
            nbytes = 0; ps_start_seen = 0; ps_stop_seen = 0; ps_stop_at_byte = -1;
        end
    endtask

    // Run one capture of `cnt` samples of constant word `word`; check the stream.
    task run_capture;
        input [CNT_W-1:0] cnt;
        input [N-1:0]     word;
        input [15:0]      div;
        integer k;
        reg [7:0] exp_lo, exp_hi;
        integer   timeout;
        begin
            reset_capture;
            la_in        = word;
            sample_count = cnt;
            divider      = div;
            // Drive `start` 1ns after the edge so the DUT samples it on exactly
            // one posedge (avoids the same-edge stimulus race that double-counts).
            @(posedge clk); #1 start = 1'b1;
            @(posedge clk); #1 start = 1'b0;
            // wait for done (bounded)
            timeout = 0;
            while (!done && timeout < 100000) begin @(posedge clk); timeout = timeout + 1; end
            if (!done) begin $display("FAIL: capture never asserted done"); errors = errors + 1; end
            @(posedge clk);  // let the final ps_stop/byte settle into the recorder

            // exactly 2 bytes per sample
            if (nbytes !== cnt*2) begin
                $display("FAIL: byte count %0d, expected %0d", nbytes, cnt*2);
                errors = errors + 1;
            end
            // ps_start once at arm, ps_stop once coincident with the final byte
            if (ps_start_seen !== 1) begin
                $display("FAIL: ps_start fired %0d times (expected 1)", ps_start_seen);
                errors = errors + 1;
            end
            if (ps_stop_seen !== 1) begin
                $display("FAIL: ps_stop fired %0d times (expected 1)", ps_stop_seen);
                errors = errors + 1;
            end
            if (ps_stop_at_byte !== cnt*2) begin
                $display("FAIL: ps_stop at byte %0d, expected %0d (flush after last byte)",
                         ps_stop_at_byte, cnt*2);
                errors = errors + 1;
            end
            // byte packing: every sample = {word[7:0], {2'b0,word[13:8]}}
            exp_lo = word[7:0];
            exp_hi = {{(16-N){1'b0}}, word[N-1:8]};
            for (k = 0; k < cnt && k*2+1 < 256; k = k + 1) begin
                if (cap[k*2] !== exp_lo) begin
                    $display("FAIL: sample %0d lo byte %02x, expected %02x", k, cap[k*2], exp_lo);
                    errors = errors + 1;
                end
                if (cap[k*2+1] !== exp_hi) begin
                    $display("FAIL: sample %0d hi byte %02x, expected %02x", k, cap[k*2+1], exp_hi);
                    errors = errors + 1;
                end
            end
            // sample period: consecutive samples are EXACTLY `div` clocks apart (the
            // firmware reports rate = 24 MHz / div).  Gateware <= v31 took div+1 — the
            // hi-byte cycle didn't count toward the period — so the real LA rate was
            // d/(d+1) of the reported one.
            for (k = 1; k < cnt && k < 128; k = k + 1) begin
                if (lo_at[k] - lo_at[k-1] !== div) begin
                    $display("FAIL: div %0d: sample %0d came %0d clocks after sample %0d (want %0d)",
                             div, k, lo_at[k] - lo_at[k-1], k-1, div);
                    errors = errors + 1;
                end
            end
        end
    endtask

    initial begin
        // power-on reset
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // Pass 1: 4 samples of 0xABC at divider 4.
        run_capture(16'd4, 14'h2ABC, 16'd4);
        // Pass 2: different word + count + divider to be sure nothing is hardcoded.
        run_capture(16'd8, 14'h1135, 16'd2);
        // Pass 3: single sample (terminal-on-first-sample edge case).
        run_capture(16'd1, 14'h30FF, 16'd6);
        // Pass 3b: a large divider (24 = 1 MS/s), where the period is what matters.
        run_capture(16'd5, 14'h13C3, 16'd24);

        // Pass 4: overflow diagnostic — hold `full`, expect overflow to latch but
        // the capture to still complete (writer would drop, module flags it).
        full = 1'b1;
        run_capture(16'd3, 14'h22A5, 16'd3);
        if (!overflow) begin
            $display("FAIL: overflow did not latch while full was held");
            errors = errors + 1;
        end
        full = 1'b0;

        // Pass 5: after a clean run, overflow must be clear again (cleared at start).
        run_capture(16'd2, 14'h3111, 16'd4);
        if (overflow) begin
            $display("FAIL: overflow still set after a no-full capture");
            errors = errors + 1;
        end

        // Pass 6 (v35 trigger hold): a run started under `hold` writes nothing until hold drops,
        // then its first sample comes as many clocks after the release as an unheld run's comes
        // after the clock following its start — and the rest of the run is unchanged.
        run_capture(16'd6, 14'h15A5, 16'd5);
        hold_ref = lo_at[0] - start_cyc - 1;
        hold = 1'b1;
        reset_capture;
        la_in = 14'h269C; sample_count = 16'd6; divider = 16'd5;
        @(posedge clk); #1 start = 1'b1;
        @(posedge clk); #1 start = 1'b0;
        repeat (40) @(posedge clk);
        if (nbytes !== 0 || !busy || done) begin
            $display("FAIL: held run wrote %0d bytes / busy=%b done=%b while hold was set", nbytes, busy, done);
            errors = errors + 1;
        end
        #1 hold = 1'b0;
        while (!done) @(posedge clk);
        @(posedge clk);
        if (nbytes !== 12) begin $display("FAIL: held run wrote %0d bytes (want 12)", nbytes); errors = errors + 1; end
        if (lo_at[0] - release_cyc !== hold_ref) begin
            $display("FAIL: held run's first sample came %0d clocks after the release (unheld: %0d)",
                     lo_at[0] - release_cyc, hold_ref);
            errors = errors + 1;
        end
        for (k6 = 1; k6 < 6; k6 = k6 + 1)
            if (lo_at[k6] - lo_at[k6-1] !== 5) begin
                $display("FAIL: held run sample %0d came %0d clocks after the last (want 5)", k6, lo_at[k6] - lo_at[k6-1]);
                errors = errors + 1;
            end
        if (cap[0] !== 8'h9C || cap[1] !== 8'h26) begin
            $display("FAIL: held run sample 0 = %02x %02x (want 9c 26)", cap[0], cap[1]); errors = errors + 1;
        end

        if (errors == 0) $display("PASS tb_la_psram_capture: byte stream/framing/overflow OK");
        else             $display("FAIL tb_la_psram_capture: %0d error(s)", errors);
        $finish;
    end

    initial begin #500000 $display("FAIL tb_la_psram_capture: timeout"); $finish; end
endmodule
