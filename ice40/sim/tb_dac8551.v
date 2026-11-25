// tb_dac8551.v — self-checking testbench for dac8551_engine: frame CONTENT and the serial TIMING
// on the pins (DAC8554 limits from v59, which also cover the DAC8551), through dac_pads (v66).
//
// Content: a 2-sample BRAM waveform (0x1234, 0x5678) must come out as 24-bit frames
// {0x00, sample}, MSB first, in loop order; in psram_mode the streamed bytes (0xA5) must.
// Timing (DAC8554 SLAS431B, IOVDD 2.7-3.6 V column — see the dac8551_engine header), checked in
// HALF clk48 cycles (10.4 ns, the DDR pads' grid) on every frame, for dividers 0,1,2,3,4,8 (BRAM)
// and 2,8 (stream):
//   t2/t3  SCLK high and low >= 2 halves inside a frame (the plan, v67: 3 halves = 31.3 ns each)
//   t5/t6  DIN stable for the 2 halves before each SCLK fall and the half after it
//   t7     SYNC rises >= 1 half after the 24th SCLK fall
//   t8     SYNC high >= 6 halves between frames
//   t9     24th SCLK fall -> next SYNC fall >= 13 halves (135 ns; DAC8554 >= 130 ns)
//   rate   one sample = max(divider, 4) + 63 clk (firmware DAC_SEQ_OVERHEAD_CLK = 75 from v67)
// The monitor samples the pins 1 ns after every clk edge, so each sample is one half-cycle's
// value and two pins changing in the same half land in the same sample.  The DAC is modelled as
// taking DIN's value from BEFORE the falling edge, with SYNC as it was before that edge.
//
// Run:  make dactest
`timescale 1ns/1ps
module tb_dac8551;
    reg clk = 0;
    always #10 clk = ~clk;

    reg         rst = 1, start = 0, stop = 0, psram_mode = 0;
    reg  [12:0] period  = 13'd2;
    reg  [15:0] divider = 16'd4;
    wire [11:0] waddr;
    reg  [7:0]  wdata;
    wire        sync, sclk, din, running, strm_pop;
    wire        sync_e, sclk_a, sclk_b, din_a, din_b;

    // 8-bit waveform memory model (1-cycle read latency, like sample_buf).
    reg [7:0] wave [0:7];
    initial begin wave[0] = 8'h34; wave[1] = 8'h12; wave[2] = 8'h78; wave[3] = 8'h56; end
    always @(posedge clk) wdata <= wave[waddr];

    dac8551_engine #(.ADDR_W(12)) dut (
        .clk(clk), .rst(rst), .start(start), .stop(stop),
        .period_samples(period), .divider(divider),
        .ctrl(8'h00), .wr_req(1'b0), .wr_frame(24'd0),
        .wave_addr(waddr), .wave_data(wdata),
        .psram_mode(psram_mode), .strm_data(8'hA5), .strm_valid(1'b1), .strm_pop(strm_pop),
        .dac_sync(sync_e), .sclk_a(sclk_a), .sclk_b(sclk_b), .din_a(din_a), .din_b(din_b), .running(running)
    );
    dac_pads pads (.clk48(clk), .sync(sync_e), .sclk_a(sclk_a), .sclk_b(sclk_b), .din_a(din_a), .din_b(din_b),
                   .dac_sync(sync), .dac_sclk(sclk), .dac_din(din));

    // ---- pin monitor (v66: per HALF clk48, 1 ns after each edge, through dac_pads) ----
    // `h` counts halves (10.4 ns at 48 MHz).  DIN is taken at an SCLK fall with SYNC low, as the
    // value BEFORE the fall.  Checks: t2 SCLK high >= 2 halves and t3 low >= 2 halves inside
    // a frame (the plan, v67: 3 and 3); t5 DIN stable for the 2 halves before each fall; t6 DIN unchanged in the half after
    // it; t7 SYNC rises >= 1 half after the 24th fall; t8 SYNC high >= 6 halves; t9 24th fall ->
    // next SYNC fall >= 13 halves (135 ns >= 130).
    integer    errors = 0, h = 0, bits = 0, nfr = 0;
    integer    h_fall24 = -1, h_sync_rise = -1, h_sync_fall = -1, h_sclk_edge = -1;
    integer    min_t9 = 1000000, min_lo = 1000000, max_hi = 0, min_hi = 1000000;
    reg        ps = 1'b1, pc = 1'b0, pd = 1'b0, ppd = 1'b0, chk_hold = 1'b0;
    reg [23:0] sh = 24'd0;
    reg [23:0] frames [0:15];
    integer    gaps   [0:15];
    always @(clk) if (!rst) begin
        #1;
        h = h + 1;
        if (chk_hold) begin
            chk_hold = 1'b0;
            if (din !== pd) begin
                $display("FAIL t6: DIN changed in the half after an SCLK fall (half %0d, bit %0d)", h, bits);
                errors = errors + 1;
            end
        end
        if (sclk !== pc && !ps && !sync) begin        // an SCLK edge inside a frame
            if (pc && h_sclk_edge >= 0) begin           // SCLK fell: the high phase ended
                if (h - h_sclk_edge < min_hi) min_hi = h - h_sclk_edge;
                if (h - h_sclk_edge > max_hi) max_hi = h - h_sclk_edge;
                if (h - h_sclk_edge < 2) begin
                    $display("FAIL t2: SCLK high for %0d halves (want >= 2), half %0d", h - h_sclk_edge, h);
                    errors = errors + 1;
                end
            end
            if (!pc && bits > 0 && h_sclk_edge >= 0) begin   // SCLK rose after a bit: low phase
                if (h - h_sclk_edge < min_lo) min_lo = h - h_sclk_edge;
                if (h - h_sclk_edge < 2) begin
                    $display("FAIL t3: SCLK low for %0d halves (want >= 2), half %0d", h - h_sclk_edge, h);
                    errors = errors + 1;
                end
            end
            h_sclk_edge = h;
        end
        if (!sclk && pc && !ps) begin          // SCLK fell with SYNC low: the DAC takes DIN
            if (pd !== ppd) begin
                $display("FAIL t5: DIN changed one half before the SCLK fall (half %0d, bit %0d)", h, bits);
                errors = errors + 1;
            end
            if (din !== pd) begin
                $display("FAIL t5/t6: DIN changed on the SCLK falling edge (half %0d, bit %0d)", h, bits);
                errors = errors + 1;
            end
            chk_hold = 1'b1;
            sh   = {sh[22:0], pd};
            bits = bits + 1;
            if (bits == 24) begin
                h_fall24 = h;
                if (nfr < 16) frames[nfr] = sh;
                nfr = nfr + 1;
            end
        end
        if (sync && !ps) begin                 // SYNC rose
            if (h_fall24 >= 0 && h - h_fall24 < 1) begin
                $display("FAIL t7: SYNC rose on the 24th SCLK falling edge (half %0d)", h);
                errors = errors + 1;
            end
            h_sync_rise = h;
        end
        if (!sync && ps) begin                 // SYNC fell: a frame starts
            if (h_fall24 >= 0 && h - h_fall24 < min_t9) min_t9 = h - h_fall24;
            if (h_fall24 >= 0 && h - h_fall24 < 13) begin
                $display("FAIL t9: SYNC fell %0d halves after the 24th SCLK fall (want >= 13)", h - h_fall24);
                errors = errors + 1;
            end
            if (h_sync_rise >= 0 && h - h_sync_rise < 6) begin
                $display("FAIL t8: SYNC high for only %0d halves (want >= 6)", h - h_sync_rise);
                errors = errors + 1;
            end
            if (nfr < 16) gaps[nfr] = (h_sync_fall >= 0) ? h - h_sync_fall : -1;
            h_sync_fall = h;
            h_sclk_edge = -1;
            bits = 0;
        end
        ppd = pd; ps = sync; pc = sclk; pd = din;
    end

    // One run: start, collect 6 frames, stop between frames (so no frame is cut), check.
    task run_case(input integer div, input psm);
        integer k, want_gap;
        reg [23:0] want;
        begin
            // v40 wire: the reload, div - 1 (div 0 sends 0).  Reloads 0..2 (div 0..3) exercise
            // the gateware's own DIV_MIN floor, which must give max(div,4) (v59; 3 before).
            divider = (div > 0) ? div - 16'd1 : 16'd0; psram_mode = psm; nfr = 0;
            @(negedge clk) start = 1; @(negedge clk) start = 0;
            wait (nfr >= 6);
            wait (dut.st == 3'd0);
            @(negedge clk) stop = 1; @(negedge clk) stop = 0;
            for (k = 0; k < 6; k = k + 1) begin
                want = psm ? 24'h00A5A5 : ((k % 2) ? 24'h005678 : 24'h001234);
                if (frames[k] !== want) begin
                    $display("FAIL div=%0d psram=%0d: frame %0d = %06h, want %06h", div, psm, k, frames[k], want);
                    errors = errors + 1;
                end
            end
            want_gap = 2 * (((div < 4) ? 4 : div) + 75);   // in halves
            for (k = 2; k < 6; k = k + 1)
                if (gaps[k] != want_gap) begin
                    $display("FAIL div=%0d psram=%0d: sample %0d took %0d halves, want 2*(max(div,4)+75) = %0d",
                             div, psm, k, gaps[k], want_gap);
                    errors = errors + 1;
                end
            repeat (40) @(posedge clk);
        end
    endtask

    initial begin
        repeat (4) @(posedge clk); rst = 0;
        run_case(4, 0); run_case(0, 0); run_case(1, 0); run_case(2, 0); run_case(3, 0); run_case(8, 0);
        run_case(2, 1); run_case(8, 1);
        if (errors == 0)
            $display("PASS tb_dac8551: frames 001234/005678 (BRAM) + 00a5a5 (stream); t2-t9 on the pins (high %0d-%0d, low >= %0d, t9 >= %0d halves) and max(div,4)+75 clk/sample for div 0,1,2,3,4,8", min_hi, max_hi, min_lo, min_t9);
        else
            $display("FAIL tb_dac8551: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #2000000 $display("FAIL tb_dac8551: timeout"); $finish;
    end
endmodule
