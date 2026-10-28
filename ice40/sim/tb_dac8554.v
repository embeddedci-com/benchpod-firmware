// tb_dac8554.v — self-checking testbench for dac8551_engine's DAC8554 support (v49): the stream
// control byte (`ctrl`) and the one-shot static frame (`wr_req` / `wr_frame`).
//
// Same pin monitor and DAC8551/DAC8554 timing checks as tb_dac8551 (t5/t6, t7, t8 >= 3 clk,
// t9 >= 6 clk), applied to EVERY frame, static ones included.  Cases:
//   1. BRAM stream with ctrl 0x12 (DAC8554 channel B): frames {12, sample} in loop order.
//   2. A static frame while a BRAM stream runs: it takes one sample slot, the stream skips no
//      sample (the frames around it stay in loop order), and every gap is still max(div,3)+51.
//   3. The same in psram_mode (stream source 0xA5A5, ctrl 0x14 = channel C).
//   4. A static frame while the engine is idle (after STOP): exactly one frame, `running` stays
//      0, SYNC ends high.  Then a second one: one more frame.
//   5. A static frame requested on the same cycle as a start: it goes out in the first slot,
//      then the stream from sample 0.
//   6. Back-to-back with the minimum divider: a static frame mid-stream at div 0.
//
// Run:  make dac8554test
`timescale 1ns/1ps
module tb_dac8554;
    reg clk = 0;
    always #10 clk = ~clk;

    reg         rst = 1, start = 0, stop = 0, psram_mode = 0;
    reg  [12:0] period  = 13'd2;
    reg  [15:0] divider = 16'd4;
    reg  [7:0]  ctrl    = 8'h00;
    reg         wr_req  = 1'b0;
    reg  [23:0] wr_frame = 24'd0;
    wire [11:0] waddr;
    reg  [7:0]  wdata;
    wire        sync, sclk, din, running, strm_pop;

    reg [7:0] wave [0:7];
    initial begin wave[0] = 8'h34; wave[1] = 8'h12; wave[2] = 8'h78; wave[3] = 8'h56; end
    always @(posedge clk) wdata <= wave[waddr];

    dac8551_engine #(.ADDR_W(12)) dut (
        .clk(clk), .rst(rst), .start(start), .stop(stop),
        .period_samples(period), .divider(divider),
        .ctrl(ctrl), .wr_req(wr_req), .wr_frame(wr_frame),
        .wave_addr(waddr), .wave_data(wdata),
        .psram_mode(psram_mode), .strm_data(8'hA5), .strm_valid(1'b1), .strm_pop(strm_pop),
        .dac_sync(sync), .dac_sclk(sclk), .dac_din(din), .running(running)
    );

    // ---- pin monitor (as tb_dac8551) ----
    integer    errors = 0, cyc = 0, bits = 0, nfr = 0;
    integer    c_fall24 = -1, c_sync_rise = -1, c_sync_fall = -1;
    reg        ps = 1'b1, pc = 1'b0, pd = 1'b0;
    reg [23:0] sh = 24'd0;
    reg [23:0] frames [0:31];
    integer    gaps   [0:31];
    always @(posedge clk) if (!rst) begin
        cyc = cyc + 1;
        if (!sclk && pc && !ps) begin
            if (din !== pd) begin
                $display("FAIL t5/t6: DIN changed on the SCLK falling edge (cycle %0d, bit %0d)", cyc, bits);
                errors = errors + 1;
            end
            sh   = {sh[22:0], pd};
            bits = bits + 1;
            if (bits == 24) begin
                c_fall24 = cyc;
                if (nfr < 32) frames[nfr] = sh;
                nfr = nfr + 1;
            end
        end
        if (sync && !ps) begin
            if (c_fall24 == cyc) begin
                $display("FAIL t7: SYNC rose on the 24th SCLK falling edge (cycle %0d)", cyc);
                errors = errors + 1;
            end
            if (bits != 24) begin
                $display("FAIL: SYNC rose after %0d SCLK falls (a cut frame, cycle %0d)", bits, cyc);
                errors = errors + 1;
            end
            c_sync_rise = cyc;
        end
        if (!sync && ps) begin
            if (c_fall24 >= 0 && cyc - c_fall24 < 6) begin
                $display("FAIL t9: SYNC fell %0d clk after the 24th SCLK fall (want >= 6)", cyc - c_fall24);
                errors = errors + 1;
            end
            if (c_sync_rise >= 0 && cyc - c_sync_rise < 3) begin
                $display("FAIL t8: SYNC high for only %0d clk (want >= 3)", cyc - c_sync_rise);
                errors = errors + 1;
            end
            if (nfr < 32) gaps[nfr] = (c_sync_fall >= 0) ? cyc - c_sync_fall : -1;
            c_sync_fall = cyc;
            bits = 0;
        end
        ps = sync; pc = sclk; pd = din;
    end

    task send_static(input [23:0] f);
        begin
            @(negedge clk) begin wr_frame = f; wr_req = 1; end
            @(negedge clk) wr_req = 0;
        end
    endtask

    task stop_between_frames;
        begin
            wait (dut.st == 3'd0 && sync);
            @(negedge clk) stop = 1; @(negedge clk) stop = 0;
            repeat (80) @(posedge clk);
        end
    endtask

    function [23:0] stream_word(input psm, input [7:0] c, input integer k);
        stream_word = psm ? {c, 16'hA5A5} : ((k % 2) ? {c, 16'h5678} : {c, 16'h1234});
    endfunction

    // A stream with one static frame after `before` stream frames.  Checks the frame sequence
    // (the static one in its slot, the stream in loop order around it) and every gap.
    task run_static_midstream(input integer div, input psm, input [7:0] c, input [23:0] sf);
        integer k, s, want_gap, n;
        begin
            divider = (div > 0) ? div - 16'd1 : 16'd0; psram_mode = psm; ctrl = c; nfr = 0;
            c_sync_fall = -1;
            @(negedge clk) start = 1; @(negedge clk) start = 0;
            wait (nfr >= 3);
            send_static(sf);
            wait (nfr >= 8);
            stop_between_frames;
            n = (nfr < 32) ? nfr : 32;
            s = 0;                                       // stream frames seen
            for (k = 0; k < n; k = k + 1) begin
                if (frames[k] === sf) begin
                    if (k < 3 || k > 5) begin
                        $display("FAIL div=%0d psram=%0d: static frame at %0d, want slot 3..5", div, psm, k);
                        errors = errors + 1;
                    end
                end else begin
                    if (frames[k] !== stream_word(psm, c, s)) begin
                        $display("FAIL div=%0d psram=%0d: frame %0d = %06h, want stream sample %0d = %06h",
                                 div, psm, k, frames[k], s, stream_word(psm, c, s));
                        errors = errors + 1;
                    end
                    s = s + 1;
                end
            end
            if (s != n - 1) begin
                $display("FAIL div=%0d psram=%0d: %0d static frames, want 1", div, psm, n - s);
                errors = errors + 1;
            end
            want_gap = ((div < 3) ? 3 : div) + 51;
            for (k = 2; k < n; k = k + 1)
                if (gaps[k] != want_gap) begin
                    $display("FAIL div=%0d psram=%0d: frame %0d came %0d clk after the previous, want %0d",
                             div, psm, k, gaps[k], want_gap);
                    errors = errors + 1;
                end
        end
    endtask

    integer k;
    initial begin
        repeat (4) @(posedge clk); rst = 0;

        // 1. stream ctrl byte
        divider = 16'd3; psram_mode = 0; ctrl = 8'h12; nfr = 0;
        @(negedge clk) start = 1; @(negedge clk) start = 0;
        wait (nfr >= 4);
        stop_between_frames;
        for (k = 0; k < 4; k = k + 1)
            if (frames[k] !== stream_word(0, 8'h12, k)) begin
                $display("FAIL ctrl: frame %0d = %06h, want %06h", k, frames[k], stream_word(0, 8'h12, k));
                errors = errors + 1;
            end

        // 2/3/6. static frame mid-stream
        run_static_midstream(8, 0, 8'h10, 24'h16BEEF);
        run_static_midstream(4, 1, 8'h14, 24'h12C0DE);
        run_static_midstream(0, 0, 8'h12, 24'h160001);
        run_static_midstream(0, 1, 8'h10, 24'h16FFFF);

        // 4. static frames while idle
        nfr = 0;
        send_static(24'h161234);
        repeat (120) @(posedge clk);
        if (nfr != 1 || frames[0] !== 24'h161234 || running || !sync) begin
            $display("FAIL idle: %0d frame(s), first %06h, running %b, sync %b (want 1, 161234, 0, 1)",
                     nfr, frames[0], running, sync);
            errors = errors + 1;
        end
        send_static(24'h14ABCD);
        repeat (120) @(posedge clk);
        if (nfr != 2 || frames[1] !== 24'h14ABCD || running || !sync) begin
            $display("FAIL idle 2: %0d frame(s), second %06h, running %b, sync %b", nfr, frames[1], running, sync);
            errors = errors + 1;
        end

        // 5. requested with the start: sent first, then the stream
        nfr = 0; divider = 16'd7; psram_mode = 0; ctrl = 8'h10;
        @(negedge clk) begin wr_frame = 24'h165555; wr_req = 1; start = 1; end
        @(negedge clk) begin wr_req = 0; start = 0; end
        wait (nfr >= 4);
        stop_between_frames;
        if (frames[0] !== 24'h165555 || frames[1] !== 24'h101234 || frames[2] !== 24'h105678) begin
            $display("FAIL pre-start: frames %06h %06h %06h, want 165555 101234 105678",
                     frames[0], frames[1], frames[2]);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASS tb_dac8554: stream ctrl byte; static frame mid-stream (BRAM/stream, div 0/4/8) takes one slot with no skipped sample and t5-t9 on every frame; idle and pre-start static frames");
        else
            $display("FAIL tb_dac8554: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #3000000 $display("FAIL tb_dac8554: timeout"); $finish;
    end
endmodule
