// ============================================================================
// tb_dac_psram_skew.v — REGRESSION guard for the deep-replay/PSRAM curse.
//
// This is the test that would have caught — in simulation, before ever touching a
// bench — the class of failure that silently killed deep DAC replay across seeds,
// yosys versions and netlist edits while static timing stayed green (the DAC held a
// constant level; "reader delivered zero bytes").
//
// ROOT CAUSE it models: in top_v2, `clk` is clk48/2 made by a FABRIC divider + SB_GB,
// so clk and clk48 share a frequency but have a PLACEMENT-DEPENDENT phase skew that
// nextpnr never times.  The OLD reader generated the PSRAM read SCLK from a clk-domain
// signal (into a clk48 DDR pad) and SAMPLED the return data in the clk domain — so the
// SCLK-edge-to-sample-edge relationship moved with that skew and, on some placements,
// sampled outside the PSRAM's data-valid window -> garbage/starvation.
//
// This bench recreates BOTH physical effects the design must survive:
//   * a swept clk<->clk48 SKEW (the SB_GB delay), and
//   * a realistic PSRAM read round-trip TACC (data valid TACC ns after the SCLK edge),
// then streams a waveform out of a modelled APS6404L and checks EVERY byte (incl. the
// loop wrap and the burst-boundary low nibble) is delivered in order — for every
// (skew, tacc) in the sweep.
//
// The v28 single-clk48-domain reader PASSES for all skews (its SCLK and its data
// sampler are both clk48, a fixed placement-invariant phase).  A reader that samples
// the PSRAM in the clk domain (the pre-v28 structure, or any future regression to it)
// FAILS here at the larger skews — exactly the bug, now caught in `make test`.
//
// v41: the reader runs as shipped, PAD_PIPE=1 behind the real psram_pads (the one-clk48 output
// retime, SB_IO DDR SCLK and registered CS/data models), so the model sees the actual pin
// waveforms and the sweep covers the retimed read path.
//
// Run:  make dacpsramskewtest
// ============================================================================
`timescale 1ns/1ps
module tb_dac_psram_skew;
    localparam [8:0]  CHUNK = 9'd16;
    localparam [3:0]  WAIT  = 4'd6;
    localparam [23:0] BASE  = 24'h000100;
    localparam [23:0] LEN   = 24'd64;        // 64 bytes (32 samples); > CHUNK -> multi-burst + wrap

    // ---- clocks: clk48 = ~48 MHz; clk = clk48/2 with a RUNTIME-VARIABLE skew ----
    localparam real P48 = 20.833;            // clk48 period (ns) ~ 48 MHz
    reg  clk48 = 1'b0;  always #(P48/2.0) clk48 = ~clk48;
    reg  clk_ideal = 1'b0;
    always @(posedge clk48) clk_ideal = ~clk_ideal;     // ideal divide-by-2
    real skew;                                          // ns: clk global-net delay vs clk48 (SB_GB)
    reg  clk = 1'b0;
    always @(clk_ideal) clk <= #(skew) clk_ideal;       // clk lags the ideal edge by `skew`

    reg  rst = 1, rst48 = 1, run = 0;

    // reader -> psram_pads (retimed pins) <-> modelled PSRAM bus
    wire [3:0] rd_io_o, rd_io_i;  wire rd_io_oe, rd_cs, rd_sclk, rd_active;
    tri        psram_sclk, psram_cs;
    tri  [3:0] psram_io;
    wire       ps_cs = psram_cs, ps_sclk = psram_sclk;  // the model watches the PINS
    wire [7:0] rd_data;  wire rd_valid;  reg rd_pop = 0;

    dac_psram_reader #(.CHUNK_BYTES(CHUNK), .WAIT_CYCLES(WAIT), .FIFO_AW(5), .PAD_PIPE(1)) dut (
        .clk(clk), .rst(rst), .clk48(clk48), .rst48(rst48), .run(run),
        .base_addr(BASE), .len_bytes(LEN),
        .bus_gnt(1'b1), .bus_req(), .bus_busy(),
        .data(rd_data), .data_valid(rd_valid), .data_pop(rd_pop),
        .io_o(rd_io_o), .io_oe(rd_io_oe), .io_i(rd_io_i),
        .cs(rd_cs), .sclk(rd_sclk), .active(rd_active)
    );
    psram_pads pads (
        .clk48(clk48), .bus_own(1'b0), .selftest(1'b0), .replay(rd_active),
        .rd_io_o(rd_io_o), .rd_io_oe(rd_io_oe), .rd_cs(rd_cs), .rd_sclk(rd_sclk), .rd_io_i(rd_io_i),
        .ps_io_o(4'h0), .ps_io_oe(1'b0), .ps_cs(1'b1), .ps_sclk_d1(1'b0),   // writer idle
        .psram_sclk(psram_sclk), .psram_cs(psram_cs),
        .psram_io0(psram_io[0]), .psram_io1(psram_io[1]), .psram_io2(psram_io[2]), .psram_io3(psram_io[3]));

    // ---- APS6404L read model with a REALISTIC round-trip TACC ----
    reg  [7:0] mem [0:1023];
    integer    mi;
    initial for (mi=0; mi<1024; mi=mi+1) mem[mi] = mi[7:0] ^ 8'h5A;

    integer    mnib;
    reg [23:0] maddr;
    reg [3:0]  mdrive;  reg mdriving;
    reg [7:0]  mbyte;
    real       tacc;                                    // ns: SCLK-edge -> data-valid delay
    localparam DATA0 = 2 + 6 + 6;                       // cmd + addr + WAIT dummy nibbles

    assign psram_io = mdriving ? mdrive : 4'bzzzz;

    always @(negedge ps_cs) begin mnib = 0; mdriving = 1'b0; end
    always @(posedge ps_sclk) if (!ps_cs) begin
        if (mnib < 2) mdriving <= #(tacc) 1'b0;
        else if (mnib < 8) begin
            case (mnib)
                2: maddr[23:20] = psram_io;  3: maddr[19:16] = psram_io;
                4: maddr[15:12] = psram_io;  5: maddr[11:8]  = psram_io;
                6: maddr[7:4]   = psram_io;  7: maddr[3:0]   = psram_io;
            endcase
            mdriving <= #(tacc) 1'b0;
        end else if (mnib < DATA0) mdriving <= #(tacc) 1'b0;
        else begin
            if (((mnib - DATA0) & 1) == 0) begin mbyte = mem[maddr[9:0]]; mdrive <= #(tacc) mbyte[7:4]; end
            else begin mdrive <= #(tacc) mbyte[3:0]; maddr = maddr + 24'd1; end
            mdriving <= #(tacc) 1'b1;                    // data valid TACC ns AFTER the SCLK edge
        end
        mnib = mnib + 1;
    end
    always @(posedge ps_cs) mdriving <= 1'b0;

    // ---- pop bytes at a DAC-like slow rate ----
    task pop_one(output [7:0] b); begin
        @(posedge clk48); while (!rd_valid) @(posedge clk48);
        b = rd_data; rd_pop <= 1'b1; @(posedge clk48); rd_pop <= 1'b0;
        repeat (20) @(posedge clk48);
    end endtask

    // ---- run ONE stream (from a clean arm) and check every byte ----
    localparam NPOP = 160;                               // > 2 loops of LEN=64 -> exercises the wrap
    integer i, errs_total;
    reg [7:0] got;
    task run_stream(input real sk, input real ta, output integer errs); begin
        errs = 0;
        skew = sk; tacc = ta;
        // clean arm
        run = 1'b0; rst = 1'b1; rst48 = 1'b1; repeat (6) @(posedge clk48);
        rst = 1'b0; rst48 = 1'b0; repeat (4) @(posedge clk48);
        @(negedge clk); run = 1'b1;
        for (i = 0; i < NPOP; i = i + 1) begin
            pop_one(got);
            if (got !== mem[(BASE[9:0] + (i % LEN)) & 10'h3FF]) errs = errs + 1;
        end
        run = 1'b0; repeat (8) @(posedge clk48);
    end endtask

    // ---- the sweep: every (skew, tacc) the design must survive ----
    integer si, ti, e;
    real    skews [0:5];
    real    taccs [0:2];
    initial begin
        skews[0]=0.0; skews[1]=4.0; skews[2]=8.0; skews[3]=12.0; skews[4]=16.0; skews[5]=19.0;
        taccs[0]=4.0; taccs[1]=12.0; taccs[2]=22.0;   // APS6404L QPI tACC + board round-trip
        errs_total = 0;
        for (si = 0; si <= 5; si = si + 1)
          for (ti = 0; ti <= 2; ti = ti + 1) begin
            run_stream(skews[si], taccs[ti], e);
            $display("  skew=%4.1f ns  tacc=%4.1f ns -> %0d byte error(s)%s",
                     skews[si], taccs[ti], e, (e==0) ? "" : "   <-- REGRESSION");
            errs_total = errs_total + e;
          end
        if (errs_total == 0)
            $display("PASS tb_dac_psram_skew: reader delivers every byte across all clk/clk48 skews x PSRAM round-trips");
        else
            $display("FAIL tb_dac_psram_skew: %0d byte error(s) — the reader samples PSRAM outside its data-valid window on some clk/clk48 skew (deep-replay-curse class)", errs_total);
        $finish;
    end

    initial begin #200000000 $display("FAIL tb_dac_psram_skew: timeout"); $finish; end
endmodule
