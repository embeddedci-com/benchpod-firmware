// ============================================================================
// tb_dac_psram_replay.v — end-to-end deep-replay datapath test:
//     modelled APS6404L  ->  dac_psram_reader (FIFO)  ->  dac8551_engine (psram_mode)
//
// Checks that the DAC8551 serial frames the engine emits are exactly the 16-bit
// little-endian samples stored in PSRAM, in order, looping at len_bytes — i.e. a
// waveform staged in PSRAM replays faithfully and repeats.  This is the gateware
// half of "deep DAC replay" (the counterpart to the ADC deep-capture path).
//
// Run:  make dacpsramreplaytest
// ============================================================================
`timescale 1ns/1ps
module tb_dac_psram_replay;
    localparam [8:0]  CHUNK = 9'd8;
    localparam [3:0]  WAIT  = 4'd6;
    localparam [23:0] BASE  = 24'h000040;
    localparam [23:0] LEN   = 24'd8;        // 4 samples; small so the wrap is exercised fast

    // clk48 primary; clk = clk48/2, phase-locked (exactly like top_v2).
    reg clk48 = 0; always #10 clk48 = ~clk48;
    reg clk = 0;   always @(posedge clk48) clk <= ~clk;
    reg rst = 1, rst48 = 1, run = 0, start = 0, stop = 0;
    reg [23:0] base = BASE, len = LEN;      // runtime region: re-armed onto a different one below

    // reader <-> tri bus
    // v41: the reader runs as shipped, PAD_PIPE=1 behind the real psram_pads; the model watches the pins
    wire [3:0] rd_io_o, rd_io_i; wire rd_io_oe, rd_cs, rd_sclk, rd_active;
    tri        psram_sclk, psram_cs;
    tri  [3:0] psram_io;
    wire       ps_cs = psram_cs, ps_sclk = psram_sclk;
    wire [7:0] strm_data; wire strm_valid, strm_pop;

    // reader FSM on clk (24 MHz), FIFO read on clk48 (the DAC engine's domain).
    dac_psram_reader #(.CHUNK_BYTES(CHUNK), .WAIT_CYCLES(WAIT), .FIFO_AW(5), .PAD_PIPE(1)) rd (
        .clk(clk), .rst(rst), .clk48(clk48), .rst48(rst48), .run(run),
        .base_addr(base), .len_bytes(len),
        .bus_gnt(1'b1), .bus_req(), .bus_busy(),   // standalone: reader always holds the bus
        .data(strm_data), .data_valid(strm_valid), .data_pop(strm_pop),
        .io_o(rd_io_o), .io_oe(rd_io_oe), .io_i(rd_io_i),
        .cs(rd_cs), .sclk(rd_sclk), .active(rd_active)
    );
    psram_pads pads (
        .clk48(clk48), .bus_own(1'b0), .selftest(1'b0), .replay(rd_active),
        .rd_io_o(rd_io_o), .rd_io_oe(rd_io_oe), .rd_cs(rd_cs), .rd_sclk(rd_sclk), .rd_io_i(rd_io_i),
        .ps_io_o(4'h0), .ps_io_oe(1'b0), .ps_cs(1'b1), .ps_sclk_d1(1'b0),   // writer idle
        .psram_sclk(psram_sclk), .psram_cs(psram_cs),
        .psram_io0(psram_io[0]), .psram_io1(psram_io[1]), .psram_io2(psram_io[2]), .psram_io3(psram_io[3]));

    // streaming DAC engine (psram_mode=1) on clk48; BRAM port unused.
    wire sync, dsclk, din, running;
    dac8551_engine #(.ADDR_W(12)) dac (
        .clk(clk48), .rst(rst48), .start(start), .stop(stop),
        .period_samples(13'd0), .divider(16'd8),
        .wave_addr(), .wave_data(8'h00),
        .psram_mode(1'b1), .strm_data(strm_data), .strm_valid(strm_valid), .strm_pop(strm_pop),
        .dac_sync(sync), .dac_sclk(dsclk), .dac_din(din), .running(running)
    );

    // ---- APS6404L read model (same contract as tb_dac_psram_reader) ----
    reg  [7:0] mem [0:1023];
    integer    mi;
    initial for (mi=0;mi<1024;mi=mi+1) mem[mi] = (mi[7:0]*8'd7) ^ 8'h3C;   // arbitrary pattern
    integer    mnib;
    reg [23:0] maddr;
    reg [3:0]  mdrive; reg mdriving; reg [7:0] mbyte;
    localparam DATA0 = 2 + 6 + 6;
    assign psram_io = mdriving ? mdrive : 4'bzzzz;
    always @(negedge ps_cs) begin mnib = 0; mdriving = 1'b0; end
    always @(posedge ps_sclk) if (!ps_cs) begin
        if (mnib < 2) mdriving <= 1'b0;
        else if (mnib < 8) begin
            case (mnib)
                2: maddr[23:20]=psram_io; 3: maddr[19:16]=psram_io;
                4: maddr[15:12]=psram_io; 5: maddr[11:8]=psram_io;
                6: maddr[7:4]=psram_io;   7: maddr[3:0]=psram_io;
            endcase
            mdriving <= 1'b0;
        end else if (mnib < DATA0) mdriving <= 1'b0;
        else begin
            if (((mnib - DATA0) & 1) == 0) begin mbyte = mem[maddr[9:0]]; mdrive <= mbyte[7:4]; end
            else begin mdrive <= mbyte[3:0]; maddr = maddr + 24'd1; end
            mdriving <= 1'b1;
        end
        mnib = mnib + 1;
    end
    always @(posedge ps_cs) mdriving <= 1'b0;

    // ---- capture each replayed sample word ----
    // tb_dac8551 already proves the sh-register -> DAC8551 serial framing; THIS test
    // targets the reader->engine sample assembly, so it records the 16-bit word the
    // engine loads to shift out (sh[15:0]) at each entry into the shift state.  That
    // is exactly the sample the DAC will emit, and the capture is race-free (a single
    // clocked process on the engine's own state).
    localparam ST_SHIFT = 3'd4;
    integer fidx = 0, errors = 0, k;
    reg [15:0] frames [0:63];
    reg [2:0]  st_d = 3'd0;
    always @(posedge clk48) begin        // DAC engine runs on clk48
        st_d <= dac.st;
        if (dac.st == ST_SHIFT && st_d != ST_SHIFT) begin
            frames[fidx] = dac.sh[15:0];
            fidx = fidx + 1;
        end
    end

    localparam NFRAMES = 11;   // > 2 loops of 4 samples -> exercises the wrap
    function [15:0] want_sample(input integer i);
        reg [9:0] lo_a, hi_a;
        begin
            lo_a = (base[9:0] + ((2*i)   % len)) & 10'h3FF;
            hi_a = (base[9:0] + ((2*i+1) % len)) & 10'h3FF;
            want_sample = {mem[hi_a], mem[lo_a]};
        end
    endfunction

    task check_frames(input [8*8-1:0] what);
        begin
            for (k = 0; k < NFRAMES; k = k + 1)
                if (frames[k] !== want_sample(k)) begin
                    $display("FAIL %0s: frame[%0d]=%04h want %04h", what, k, frames[k], want_sample(k));
                    errors = errors + 1;
                end
        end
    endtask

    initial begin
        repeat (4) @(posedge clk48); rst = 0; rst48 = 0;
        @(negedge clk); run = 1'b1;   // negedge: avoid the posedge sampling race (run is registered in real HW)
        // let the FIFO prefill so the engine never stalls on the first sample
        repeat (200) @(posedge clk48);
        @(negedge clk48); start = 1'b1;    // drive on negedge (clk48 = engine domain)
        @(negedge clk48); start = 1'b0;
        wait (fidx >= NFRAMES);
        check_frames("arm");

        // ---- RE-ARM onto a DIFFERENT region and length (v34) — what every new replay does.
        //      STOP_DAC drops `run`; START_DAC_PSRAM then writes base/len and raises `run` on the
        //      SAME clk edge (all three are cmd_dispatch registers), so the reader must stream the
        //      new region from its first burst and the engine must play it from sample 0. ----
        @(negedge clk48); stop = 1'b1;
        @(negedge clk48); stop = 1'b0;
        @(negedge clk); run = 1'b0;
        repeat (100) @(posedge clk48);
        @(negedge clk); base = 24'h000100; len = 24'd6; run = 1'b1;
        repeat (200) @(posedge clk48);
        fidx = 0;
        @(negedge clk48); start = 1'b1;
        @(negedge clk48); start = 1'b0;
        wait (fidx >= NFRAMES);
        check_frames("re-arm");
        if (errors == 0)
            $display("PASS tb_dac_psram_replay: %0d DAC frames replayed from PSRAM in order across the wrap (len=%0d B), and again after a re-arm onto base 0x100 / len 6 B", NFRAMES, LEN);
        else
            $display("FAIL tb_dac_psram_replay: %0d error(s)", errors);
        $finish;
    end
    initial begin #20000000
        $display("FAIL tb_dac_psram_replay: timeout — fidx=%0d run_dac=%b dac.st=%0d rd.st=%0d wbin=%0d rbin=%0d valid=%b pop=%b run=%b active=%b",
                 fidx, running, dac.st, rd.st, rd.wptr, rd.rptr, strm_valid, strm_pop, run, rd.active);
        $finish; end
endmodule
