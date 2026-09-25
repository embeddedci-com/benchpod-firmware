// tb_dac8551.v — self-checking testbench for dac8551_engine: frame CONTENT and the DAC8551
// serial TIMING on the pins.
//
// Content: a 2-sample BRAM waveform (0x1234, 0x5678) must come out as 24-bit frames
// {0x00, sample}, MSB first, in loop order; in psram_mode the streamed bytes (0xA5) must.
// Timing (datasheet SLAS429E 6.6 — see the dac8551_engine header), checked in clk cycles on
// every frame, for dividers 0,1,2,3,4,8 (BRAM) and 2,8 (stream):
//   t5/t6  DIN must not change on the clk edge that makes SCLK fall inside a frame
//   t7     SYNC must not rise on the edge of the 24th SCLK fall
//   t8     SYNC high >= 3 clk between frames
//   t9     24th SCLK fall -> next SYNC fall >= 6 clk (125 ns at 48 MHz; datasheet >= 100 ns)
//   rate   one sample = max(divider, 3) + 51 clk (firmware DAC_SEQ_OVERHEAD_CLK = 51)
// The pins are registered on posedge clk, so the monitor samples them on posedge clk and
// compares with the previous sample: an edge "at cycle k" is a change between samples k-1 and
// k.  That is race-free, and two pins changing on the SAME clk edge land in the same sample —
// exactly the placement-fragile case t7 is about.  The DAC is modelled as taking DIN's value
// from BEFORE the falling edge, with SYNC as it was before that edge.
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

    // 8-bit waveform memory model (1-cycle read latency, like sample_buf).
    reg [7:0] wave [0:7];
    initial begin wave[0] = 8'h34; wave[1] = 8'h12; wave[2] = 8'h78; wave[3] = 8'h56; end
    always @(posedge clk) wdata <= wave[waddr];

    dac8551_engine #(.ADDR_W(12)) dut (
        .clk(clk), .rst(rst), .start(start), .stop(stop),
        .period_samples(period), .divider(divider),
        .wave_addr(waddr), .wave_data(wdata),
        .psram_mode(psram_mode), .strm_data(8'hA5), .strm_valid(1'b1), .strm_pop(strm_pop),
        .dac_sync(sync), .dac_sclk(sclk), .dac_din(din), .running(running)
    );

    // ---- pin monitor ----
    integer    errors = 0, cyc = 0, bits = 0, nfr = 0;
    integer    c_fall24 = -1, c_sync_rise = -1, c_sync_fall = -1;
    reg        ps = 1'b1, pc = 1'b0, pd = 1'b0;
    reg [23:0] sh = 24'd0;
    reg [23:0] frames [0:15];
    integer    gaps   [0:15];
    always @(posedge clk) if (!rst) begin
        cyc = cyc + 1;
        if (!sclk && pc && !ps) begin          // SCLK fell with SYNC low: the DAC takes DIN
            if (din !== pd) begin
                $display("FAIL t5/t6: DIN changed on the SCLK falling edge (cycle %0d, bit %0d)", cyc, bits);
                errors = errors + 1;
            end
            sh   = {sh[22:0], pd};
            bits = bits + 1;
            if (bits == 24) begin
                c_fall24 = cyc;
                if (nfr < 16) frames[nfr] = sh;
                nfr = nfr + 1;
            end
        end
        if (sync && !ps) begin                 // SYNC rose
            if (c_fall24 == cyc) begin
                $display("FAIL t7: SYNC rose on the 24th SCLK falling edge (cycle %0d)", cyc);
                errors = errors + 1;
            end
            c_sync_rise = cyc;
        end
        if (!sync && ps) begin                 // SYNC fell: a frame starts
            if (c_fall24 >= 0 && cyc - c_fall24 < 6) begin
                $display("FAIL t9: SYNC fell %0d clk after the 24th SCLK fall (want >= 6)", cyc - c_fall24);
                errors = errors + 1;
            end
            if (c_sync_rise >= 0 && cyc - c_sync_rise < 3) begin
                $display("FAIL t8: SYNC high for only %0d clk (want >= 3)", cyc - c_sync_rise);
                errors = errors + 1;
            end
            if (nfr < 16) gaps[nfr] = (c_sync_fall >= 0) ? cyc - c_sync_fall : -1;
            c_sync_fall = cyc;
            bits = 0;
        end
        ps = sync; pc = sclk; pd = din;
    end

    // One run: start, collect 6 frames, stop between frames (so no frame is cut), check.
    task run_case(input integer div, input psm);
        integer k, want_gap;
        reg [23:0] want;
        begin
            // v40 wire: the reload, div - 1 (div 0 sends 0).  Reloads 0 and 1 (div 0..2) exercise
            // the gateware's own DIV_MIN floor, which must still give max(div,3).
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
            want_gap = ((div < 3) ? 3 : div) + 51;
            for (k = 2; k < 6; k = k + 1)
                if (gaps[k] != want_gap) begin
                    $display("FAIL div=%0d psram=%0d: sample %0d took %0d clk, want max(div,3)+51 = %0d",
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
            $display("PASS tb_dac8551: frames 001234/005678 (BRAM) + 00a5a5 (stream); t5-t9 on the pins and max(div,3)+51 clk/sample for div 0,1,2,3,4,8");
        else
            $display("FAIL tb_dac8551: %0d error(s)", errors);
        $finish;
    end

    initial begin
        #2000000 $display("FAIL tb_dac8551: timeout"); $finish;
    end
endmodule
