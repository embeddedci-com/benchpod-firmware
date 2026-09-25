// ============================================================================
// tb_psram_arbiter.v — self-checking test of CONCURRENT DAC replay + capture over
// the shared APS6404L quad bus, arbitrated by psram_bus_arbiter at burst granularity.
//
// This is the de-risking prototype for the PSRAM partition scheme (DAC replay at the
// TOP of memory, LA/ADC capture at the BOTTOM, disjoint regions).  It runs the
// dac_psram_reader (0xEB, streaming a preloaded waveform OUT of the top region) and
// the psram_dual_writer (0x38, draining ADC+LA streams INTO the bottom regions) AT
// THE SAME TIME through the arbiter and ONE shared set of pads (mirroring top_v2's
// real pad mux: DDR SCLK on clk48, combinational IO with D_IN_0 fed back to the
// reader).  A single behavioural APS6404L model serves both 0x38 writes and 0xEB
// reads against ONE mem[] array.
//
// Checks:
//   * INVARIANT: the two masters are never both mid-burst (rd_busy && wr_busy).
//   * INVARIANT: a master output-enable and the model's data drive never collide.
//   * Reader streams the preloaded top-region waveform out in order across the loop
//     wrap, with ZERO underruns at a DAC-representative pop cadence.
//   * Writer's ADC + LA bytes land contiguously in their bottom regions, in order.
//   * Reports worst-case reader-FIFO min occupancy and writer-FIFO max occupancy +
//     effective per-master throughput, so the bus-time margin is measurable.
//
// Run:  make arbtest   (needs yosys' cells_sim.v for SB_IO)
// ============================================================================
`timescale 1ns/1ps
module tb_psram_arbiter;
    // ---- tunables (representative worst case) ----
    localparam [8:0]  RD_CHUNK = 9'd16;      // reader burst bytes (amortises 0xEB cmd+addr+dummy)
    localparam [3:0]  WAIT     = 4'd6;
    localparam        RD_FIFO_AW = 8;        // 256 B prefetch FIFO (1 BRAM either way)
    localparam [8:0]  WR_CHUNK = 9'd32;      // writer burst bytes (as top_v2)

    localparam [23:0] DAC_BASE = 24'h000300; // waveform at the TOP of the tiny test map
    localparam [23:0] DAC_LEN  = 24'd64;     // 32 samples, loops
    localparam [23:0] LA_BASE  = 24'h000000; // capture at the BOTTOM
    localparam [23:0] ADC_BASE = 24'h000180;

    localparam integer POP_EVERY = 26;       // clk48 per DAC byte ~= 0.906 MS/s (worst case)
    localparam integer LA_EVERY   = 12;      // clk   per LA byte  ~= 1.0 MS/s deep LA
    localparam integer ADC_EVERY  = 30;      // clk   per ADC byte ~= 0.4 MS/s ADC
    localparam integer NPOP = 200;           // > 3 loop wraps of DAC_LEN
    localparam integer NL   = 96;            // LA bytes to capture
    localparam integer NA   = 32;            // ADC bytes to capture

    // clk48 primary; clk = clk48/2, phase-locked (exactly like top_v2).
    reg clk48 = 0; always #10.4166 clk48 = ~clk48;
    reg clk = 0;   always @(posedge clk48) clk <= ~clk;
    reg rst = 1, rst48 = 1;

    // ---- reader (deep DAC replay) ----
    reg         rd_run = 0;
    wire [3:0]  rd_io_o, rd_io_i;
    wire        rd_io_oe, rd_cs, rd_sclk, rd_active;
    wire [7:0]  rd_data; wire rd_valid; reg rd_pop = 0;
    wire        rd_req, rd_busy, rd_gnt;

    dac_psram_reader #(.CHUNK_BYTES(RD_CHUNK), .WAIT_CYCLES(WAIT), .FIFO_AW(RD_FIFO_AW), .PAD_PIPE(1)) rdr (
        .clk(clk), .rst(rst), .clk48(clk48), .rst48(rst48), .run(rd_run),
        .base_addr(DAC_BASE), .len_bytes(DAC_LEN),
        .bus_gnt(rd_gnt), .bus_req(rd_req), .bus_busy(rd_busy),
        .data(rd_data), .data_valid(rd_valid), .data_pop(rd_pop),
        .io_o(rd_io_o), .io_oe(rd_io_oe), .io_i(rd_io_i),
        .cs(rd_cs), .sclk(rd_sclk), .active(rd_active)
    );

    // ---- writer (ADC + LA capture) ----
    reg        wr_start = 0, wr_stop = 0;
    reg  [7:0] adc_data = 0, la_data = 0;
    reg        adc_stb = 0, la_stb = 0;
    wire       adc_full, la_full, wr_idle;
    wire [3:0] w_io_o; wire w_io_oe, w_cs, w_sclk_d1;
    wire       wr_req, wr_busy, wr_gnt;

    psram_dual_writer #(.CHUNK_BYTES(WR_CHUNK)) wrt (
        .clk(clk), .clk48(clk48), .rst(rst), .bus_own(1'b0),
        .adc_base(ADC_BASE), .la_base(LA_BASE),
        .start(wr_start), .stop(wr_stop),
        .bus_gnt(wr_gnt), .bus_req(wr_req), .bus_busy(wr_busy),
        .adc_data(adc_data), .adc_stb(adc_stb), .adc_full(adc_full),
        .la_data(la_data),   .la_stb(la_stb),   .la_full(la_full),
        .idle(wr_idle),
        .psram_io_o(w_io_o), .psram_io_oe(w_io_oe), .psram_cs(w_cs),
        .psram_sclk_d1(w_sclk_d1)
    );

    // ---- arbiter ----
    wire [1:0] owner;
    psram_bus_arbiter arb (
        .clk(clk), .rst(rst),
        .rd_req(rd_req), .rd_busy(rd_busy), .rd_gnt(rd_gnt),
        .wr_req(wr_req), .wr_busy(wr_busy), .wr_gnt(wr_gnt),
        .owner(owner)
    );
    wire rd_own = (owner == 2'd1);

    // ---- the real shared pads (v41: mux + one-clk48 output retime, SB_IO models) ----
    tri  psram_sclk, psram_cs; tri [3:0] psram_io;
    pulldown (psram_sclk); pulldown (psram_cs);
    pullup   (psram_io[0]); pullup (psram_io[1]); pullup (psram_io[2]); pullup (psram_io[3]);
    psram_pads pads (
        .clk48(clk48), .bus_own(1'b0), .selftest(1'b0), .replay(rd_active),
        .rd_io_o(rd_io_o), .rd_io_oe(rd_io_oe), .rd_cs(rd_cs), .rd_sclk(rd_sclk), .rd_io_i(rd_io_i),
        .ps_io_o(w_io_o), .ps_io_oe(w_io_oe), .ps_cs(w_cs), .ps_sclk_d1(w_sclk_d1),
        .psram_sclk(psram_sclk), .psram_cs(psram_cs),
        .psram_io0(psram_io[0]), .psram_io1(psram_io[1]), .psram_io2(psram_io[2]), .psram_io3(psram_io[3]));

    wire       ps_cs   = psram_cs;
    wire       ps_sclk = psram_sclk;
    wire [3:0] ps_io   = psram_io;

    // ==========================================================================
    // Combined APS6404L behavioural model: one mem[], serves 0x38 writes AND 0xEB
    // reads.  Decodes the command from the first two nibbles of each CS-low burst.
    // ==========================================================================
    reg  [7:0]  mem [0:1023];
    integer     mi;
    reg  [7:0]  d_cmd;
    reg  [23:0] m_addr;
    integer     mnib;
    reg  [3:0]  mdrive; reg mdriving;
    reg  [7:0]  mbyte;
    localparam  RDATA0 = 2 + 6 + WAIT;   // first read-data nibble index

    assign psram_io = mdriving ? mdrive : 4'bzzzz;   // model drives ONLY during read-data

    // ONE decode block (no cross-block race on mnib/m_addr): capture cmd+addr, then
    // either STORE (0x38 write) or DRIVE (0xEB read) the data nibbles.  Read-drive
    // mirrors the proven standalone tb_dac_psram_reader model.
    always @(negedge ps_cs) begin mnib = 0; mdriving = 1'b0; d_cmd = 8'h00; end
    always @(posedge ps_cs) mdriving <= 1'b0;
    always @(posedge ps_sclk) if (!ps_cs) begin
        case (mnib)
            0: d_cmd[7:4]    = ps_io;
            1: d_cmd[3:0]    = ps_io;
            2: m_addr[23:20] = ps_io;
            3: m_addr[19:16] = ps_io;
            4: m_addr[15:12] = ps_io;
            5: m_addr[11:8]  = ps_io;
            6: m_addr[7:4]   = ps_io;
            7: m_addr[3:0]   = ps_io;
            default: ;
        endcase
        if (d_cmd == 8'h38 && mnib >= 8) begin
            // WRITE: store data bytes (high then low nibble) at the running address.
            if (((mnib-8) & 1) == 0) mbyte[7:4] = ps_io;
            else begin mbyte[3:0] = ps_io; mem[m_addr[9:0]] = mbyte; m_addr = m_addr + 24'd1; end
        end else if (d_cmd == 8'hEB && mnib >= RDATA0) begin
            // READ: drive data nibbles (high then low) from mem for the reader to sample.
            if (((mnib-RDATA0) & 1) == 0) begin mbyte = mem[m_addr[9:0]]; mdrive <= mbyte[7:4]; end
            else begin mdrive <= mbyte[3:0]; m_addr = m_addr + 24'd1; end
            mdriving <= 1'b1;
        end else begin
            mdriving <= 1'b0;   // cmd / addr / dummy phases: model floats the bus
        end
        mnib = mnib + 1;
    end

    // ==========================================================================
    // Invariants + watermarks
    // ==========================================================================
    integer both_busy_err, collide_err;
    integer rd_min_occ, wr_max_occ;
    initial begin both_busy_err=0; collide_err=0; rd_min_occ=1<<20; wr_max_occ=0; end

    always @(posedge clk) begin
        if (rd_busy && wr_busy) both_busy_err = both_busy_err + 1;
        // a master driving the bus AND the model driving = collision
        if (pads.io_oe && mdriving) collide_err = collide_err + 1;   // the real pad OE (v41)
        // watermarks (hierarchical peeks)
        if (rd_run && rdr.occ < rd_min_occ) rd_min_occ = rdr.occ;
        if (wrt.a_cnt > wr_max_occ) wr_max_occ = wrt.a_cnt;
        if (wrt.l_cnt > wr_max_occ) wr_max_occ = wrt.l_cnt;
    end

    // ==========================================================================
    // Stimulus
    // ==========================================================================
    integer i, errors, underruns, got_cnt;
    reg [7:0] got;
    reg done_rd, done_wr;
    real t_first_pop, t_last_pop, t_first_wr, t_last_wr;

    // Reader consumer: pop at a fixed DAC cadence; a pop attempt with !rd_valid is an
    // UNDERRUN (the DAC engine would have stalled = audible glitch).
    initial begin
        errors = 0; underruns = 0; got_cnt = 0; done_rd = 0;
        t_first_pop = 0.0; t_last_pop = 0.0;
        @(negedge rst48);
        @(negedge clk); rd_run = 1'b1;
        // Prime: real firmware starts the DAC only once the prefetch FIFO has data,
        // so wait for the first fill before applying the steady-state pop cadence.
        @(posedge clk48); while (!rd_valid) @(posedge clk48);
        for (i = 0; i < NPOP; i = i + 1) begin
            repeat (POP_EVERY) @(posedge clk48);
            if (!rd_valid) begin
                underruns = underruns + 1;
                while (!rd_valid) @(posedge clk48);   // wait, then take it (keep checking order)
            end
            got = rd_data; rd_pop <= 1'b1; @(posedge clk48); rd_pop <= 1'b0;
            if (i == 0) t_first_pop = $realtime;
            t_last_pop = $realtime;
            if (got !== mem[(DAC_BASE[9:0] + (i % DAC_LEN)) & 10'h3FF]) begin
                $display("FAIL: DAC byte[%0d] got %02h want %02h (pos %0d)",
                         i, got, mem[(DAC_BASE[9:0] + (i % DAC_LEN)) & 10'h3FF], i % DAC_LEN);
                errors = errors + 1;
            end
            got_cnt = got_cnt + 1;
        end
        done_rd = 1;
    end

    // Writer producer: feed LA (fast) + ADC (slow) at capture cadence with backpressure.
    integer la_i, adc_i, tick;
    initial begin
        done_wr = 0; la_i = 0; adc_i = 0; tick = 0;
        t_first_wr = 0.0; t_last_wr = 0.0;
        @(negedge rst48);
        @(posedge clk); wr_start <= 1'b1; @(posedge clk); wr_start <= 1'b0;
        while (la_i < NL || adc_i < NA) begin
            @(posedge clk); tick = tick + 1;
            if ((tick % LA_EVERY) == 0 && la_i < NL && !la_full) begin
                la_data <= 8'h50 + la_i[7:0]; la_stb <= 1'b1;
                if (t_first_wr == 0.0) t_first_wr = $realtime;
                t_last_wr = $realtime;
                la_i = la_i + 1;
            end else la_stb <= 1'b0;
            if ((tick % ADC_EVERY) == 0 && adc_i < NA && !adc_full) begin
                adc_data <= 8'hA0 + adc_i[7:0]; adc_stb <= 1'b1;
                adc_i = adc_i + 1;
            end else adc_stb <= 1'b0;
        end
        @(posedge clk); la_stb <= 1'b0; adc_stb <= 1'b0;
        repeat (20) @(posedge clk);
        @(posedge clk); wr_stop <= 1'b1; @(posedge clk); wr_stop <= 1'b0;
        // NOTE: under arbitration `wr_idle` (in-S_IDLE) blips high whenever the writer
        // is merely waiting for a grant, so it is NOT a reliable "drained" signal — wait
        // for the staging FIFOs to actually empty.  (top_v2's cap-done logic keys off the
        // same `ps_idle`; it needs the same treatment at integration.)
        while (wrt.a_cnt != 0 || wrt.l_cnt != 0 || wr_busy) @(posedge clk);
        repeat (40) @(posedge clk);   // let the last clk48-pipelined burst reach mem
        done_wr = 1;
    end

    // ---- preload + supervise ----
    initial begin
        for (mi = 0; mi < 1024; mi = mi + 1) mem[mi] = 8'hXX;
        // preload the DAC waveform region with a deterministic pattern
        for (mi = 0; mi < DAC_LEN; mi = mi + 1)
            mem[(DAC_BASE[9:0] + mi) & 10'h3FF] = (mi[7:0] * 8'd7) ^ 8'hC3;
        repeat (6) @(posedge clk48); rst = 0; rst48 = 0;

        wait (done_rd && done_wr);
        repeat (10) @(posedge clk);

        // verify writer regions
        for (i = 0; i < NL; i = i + 1)
            if (mem[(LA_BASE[9:0]+i) & 10'h3FF] !== (8'h50 + i[7:0])) begin
                $display("FAIL: LA[%0d]=%02h want %02h", i, mem[(LA_BASE[9:0]+i)&10'h3FF], 8'h50+i[7:0]);
                errors = errors + 1;
            end
        for (i = 0; i < NA; i = i + 1)
            if (mem[(ADC_BASE[9:0]+i) & 10'h3FF] !== (8'hA0 + i[7:0])) begin
                $display("FAIL: ADC[%0d]=%02h want %02h", i, mem[(ADC_BASE[9:0]+i)&10'h3FF], 8'hA0+i[7:0]);
                errors = errors + 1;
            end

        if (both_busy_err != 0) begin
            $display("FAIL: reader+writer both mid-burst for %0d clk cycles", both_busy_err);
            errors = errors + 1;
        end
        if (collide_err != 0) begin
            $display("FAIL: master-drive vs model-drive bus collision for %0d cycles", collide_err);
            errors = errors + 1;
        end
        if (underruns != 0) begin
            $display("FAIL: %0d DAC underrun(s) at pop cadence 1/%0d clk48", underruns, POP_EVERY);
            errors = errors + 1;
        end

        $display("  --- concurrent replay+capture over one bus ---");
        $display("  DAC: %0d bytes streamed (loop len %0d), pop 1/%0d clk48; reader FIFO min occ = %0d / %0d B",
                 got_cnt, DAC_LEN, POP_EVERY, rd_min_occ, (1<<RD_FIFO_AW));
        if (t_last_pop > t_first_pop && got_cnt > 1)
            $display("  DAC effective read rate = %0.2f MB/s over %0.1f us",
                     (got_cnt-1)*1000.0/(t_last_pop-t_first_pop), (t_last_pop-t_first_pop)/1000.0);
        $display("  capture: LA %0d B @%06h + ADC %0d B @%06h; writer FIFO max occ = %0d / 63 B",
                 NL, LA_BASE, NA, ADC_BASE, wr_max_occ);
        if (t_last_wr > t_first_wr)
            $display("  capture effective write rate = %0.2f MB/s over %0.1f us",
                     (NL+NA-1)*1000.0/(t_last_wr-t_first_wr), (t_last_wr-t_first_wr)/1000.0);

        if (errors == 0)
            $display("PASS tb_psram_arbiter: concurrent DAC replay + ADC/LA capture, disjoint regions, no collisions, no underruns");
        else
            $display("FAIL tb_psram_arbiter: %0d error(s)", errors);
        $finish;
    end

    initial begin #8000000 $display("FAIL tb_psram_arbiter: timeout"); $finish; end
endmodule
