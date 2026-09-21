// ============================================================================
// psram_tri_master.v — ONE APS6404L QPI master serving up to THREE region jobs
// on the single shared quad bus: LA capture (write), ADC capture (write), and DAC
// deep replay (read).  Replaces the psram_dual_writer + dac_psram_reader +
// psram_bus_arbiter trio with one module, one FSM, one pad set (docs/
// tri-capture-unified-psram.md).
//
// ---- One path, zero-length disables a job ----
// Each of the three jobs has a runtime {base, len} (bytes).  A job with len==0 is
// simply never "needy", so it is skipped — no separate datapath, no extra pads.
// This is what lets ANY subset (1, 2 or 3 streams) run through the same logic.
//
// ---- Two timing regimes, one bus (deliberately preserved) ----
// WRITES need the fast 48 MHz DDR drain (LA bursts up to ~24 MB/s), so write bursts
// use the proven clk48 1:2 gearbox + DDR SCLK (from psram_dual_writer, verbatim).
// READS must sample PSRAM-driven data with margin, so the DAC read burst uses the
// proven clk-domain sub-cycle sampler (~12 MHz SCLK) from dac_psram_reader.  Only ONE
// burst is ever in flight (they share the bus), so the FSM is in write-states OR
// read-states, never both; the pad outputs are muxed on `rd_path`.  This mirrors the
// pad mux that lived in top_v2 (replay = rd_busy), moved inside the one module.
//
// ---- Arbitration folded in (no separate arbiter) ----
// The old psram_bus_arbiter is gone: S_IDLE round-robins among the needy jobs and a
// burst always runs CS-low..CS-high to completion before the next job is picked, so
// the pad-mux switch is always at a quiescent CS-high boundary (same safety argument
// the arbiter made).  A continuously-busy stream cannot hog the bus — one burst per
// grant, token rotates.
//
// ---- CDC ----
// FSM + staging FIFOs + read sampler on `clk` (24 MHz).  The DAC prefetch FIFO is
// dual-clock (written on clk by the read burst, popped on clk48 by dac8551_engine).
// The write gearbox is the phase-locked 1:2 clk->clk48 hop (NOT the async class that
// caused the 0x5555 bug).  Reset `rst` is a clk-domain level; a 2-FF sync gives rst48.
// ============================================================================
module psram_tri_master #(
    parameter [8:0] WCHUNK     = 9'd16,   // write burst bytes  (tCEM budget @48 Mnib/s)
    parameter [8:0] RCHUNK     = 9'd16,   // read  burst bytes
    parameter [3:0] RWAIT      = 4'd6,    // 0xEB dummy cycles (APS6404L default)
    parameter       RD_FIFO_AW = 8,       // DAC prefetch FIFO = 2^8 = 256 B
    // Compile out the DAC READ job (prefetch FIFO + read-burst FSM/pads) entirely.
    // The loop image has no deep-DAC-PSRAM replay, so it instantiates the tri-master
    // with HAS_DAC_READ=0 and pays only the write (ADC+LA capture) path — no reason to
    // carry the read datapath dead.  The deep image uses HAS_DAC_READ=1.
    parameter       HAS_DAC_READ = 1
)(
    input  wire        clk,               // 24 MHz logic clock
    input  wire        rst,               // sync reset in the clk domain
    input  wire        clk48,             // 48 MHz output/serializer clock
    input  wire        rst48,             // reset in the clk48 domain (DAC FIFO read side)

    // ---- write-capture arm (clk) ----
    // `start` primes the LA+ADC region pointers.  There is no `stop`: draining is
    // governed by FIFO-occupancy + the per-region backstop, and top keys capture-done
    // off `idle` (S_IDLE + both write FIFOs empty), matching the old dual_writer flow.
    input  wire        start,

    // ---- runtime region map (bytes; held stable while the job is active) ----
    input  wire [23:0] la_base,  input wire [23:0] la_len,
    input  wire [23:0] adc_base, input wire [23:0] adc_len,
    input  wire [23:0] dac_base, input wire [23:0] dac_len,
    input  wire        dac_run,           // level: DAC replay active (read job enabled)

    // ---- write producer streams (clk) ----
    input  wire [7:0]  adc_data, input wire adc_stb, output wire adc_full,
    input  wire [7:0]  la_data,  input wire la_stb,  output wire la_full,

    // ---- DAC sample-fetch port (clk48; FIFO head) ----
    output wire [7:0]  dac_data, output wire dac_valid, input wire dac_pop,

    // ---- status (clk) ----
    output wire        idle,              // no burst in flight AND both write FIFOs empty

    // ---- shared PSRAM pads (module does the internal read/write mux) ----
    output wire [3:0]  psram_io_o,
    output wire        psram_io_oe,
    input  wire [3:0]  psram_io_i,
    output wire        psram_cs,
    output wire        psram_sclk_d0,     // DDR D_OUT_0 (0 for writes; rd_sclk for reads)
    output wire        psram_sclk_d1,     // DDR D_OUT_1 (gearbox gate for writes; rd_sclk for reads)
    output wire        active             // 1 = a burst is in flight (st != S_IDLE)
);
    localparam WAW = 5;   // write staging FIFO addr width (32 B, as psram_dual_writer)

    // =====================================================================
    // Runtime region registers (clk): running address + remaining bytes.
    //   write jobs: rem is a BACKSTOP — it counts down len and stops the job at the
    //     region end even if the producer over-feeds (hard no-overrun guarantee).
    //   dac read job: rem counts down len and WRAPS to base (replay loops).
    // =====================================================================
    reg  [23:0] la_addr, adc_addr, dac_addr;
    reg  [23:0] la_rem,  adc_rem,  dac_rem;
    reg         dac_run_d;

    // =====================================================================
    // Write staging FIFOs (clk) — one per write job.  Async (combinational) read so
    // they map to FF+mux, not BRAM (no LUT-RAM on iCE40).  32 B covers one WCHUNK
    // burst; deep buffering is the upstream spram_ring16 (LA) in top_v2.
    // =====================================================================
    reg  [7:0]  afifo [0:(1<<WAW)-1];
    reg  [WAW:0] a_wr, a_rd, a_cnt;
    wire        a_full  = (a_cnt >= ({1'b0,{WAW{1'b1}}}));
    wire        a_empty = (a_cnt == 0);
    wire        a_wr_en = adc_stb && !a_full;
    assign      adc_full = a_full;

    reg  [7:0]  lfifo [0:(1<<WAW)-1];
    reg  [WAW:0] l_wr, l_rd, l_cnt;
    wire        l_full  = (l_cnt >= ({1'b0,{WAW{1'b1}}}));
    wire        l_empty = (l_cnt == 0);
    wire        l_wr_en = la_stb && !l_full;
    assign      la_full = l_full;

    // =====================================================================
    // DAC prefetch FIFO (dual-clock): written on clk by the read burst, read on clk48
    // by the DAC engine.  Gray-coded pointers, dual-clock BRAM, FWFT output register.
    // Ported verbatim from dac_psram_reader (incl. the flush-on-!run byte-alignment fix
    // and the pipelined FWFT refill that keeps it off the clk48 critical path).
    // =====================================================================
    localparam RAW = RD_FIFO_AW;
    // FSM-visible read-job signals, hoisted out of the generate so the shared FSM can
    // reference them in both configs.  When HAS_DAC_READ=0 the read FSM states are
    // unreachable (needy_dac is a constant 0), so push/push_byte are pruned and `room`
    // is tied 0 below.
    reg          push;
    reg  [7:0]   push_byte;
    wire [RAW:0] room;

    generate if (HAS_DAC_READ) begin : g_rdfifo
        // ---- DAC prefetch FIFO (dual-clock) — only in the deep-replay image ----
        reg  [7:0]  fmem [0:(1<<RAW)-1];
        reg  [1:0]  run48_s = 2'b0;
        wire        run48   = run48_s[1];
        always @(posedge clk48) run48_s <= {run48_s[0], dac_run};

        reg  [RAW:0] wbin  = 0, wgray = 0;
        reg  [RAW:0] rbin  = 0, rgray = 0;
        reg  [RAW:0] rgray_c1 = 0, rgray_c2 = 0;
        reg  [RAW:0] wgray_r1 = 0, wgray_r2 = 0;

        wire [RAW:0] wbin_nxt  = wbin + 1'b1;
        wire [RAW:0] wgray_nxt = (wbin_nxt >> 1) ^ wbin_nxt;
        wire         f_full  = (wgray_nxt == {~rgray_c2[RAW:RAW-1], rgray_c2[RAW-2:0]});

        function [RAW:0] gray2bin(input [RAW:0] g);
            integer b;
            begin
                gray2bin[RAW] = g[RAW];
                for (b = RAW-1; b >= 0; b = b - 1)
                    gray2bin[b] = gray2bin[b+1] ^ g[b];
            end
        endfunction
        wire [RAW:0] rbin_c    = gray2bin(rgray_c2);
        wire [RAW:0] occupancy = wbin - rbin_c;
        assign       room      = ({1'b1,{RAW{1'b0}}}) - occupancy;

        reg  [7:0]  dout;
        reg         dout_vld;
        wire        bram_empty = (rgray == wgray_r2);
        wire        fetch      = !dout_vld && !bram_empty;
        assign      dac_data   = dout;
        assign      dac_valid  = dout_vld;

        always @(posedge clk) begin
            if (rst || !dac_run) begin
                wbin <= 0; wgray <= 0; rgray_c1 <= 0; rgray_c2 <= 0;
            end else begin
                rgray_c1 <= rgray; rgray_c2 <= rgray_c1;
                if (push && !f_full) begin
                    fmem[wbin[RAW-1:0]] <= push_byte;
                    wbin  <= wbin_nxt;
                    wgray <= wgray_nxt;
                end
            end
        end
        always @(posedge clk48) begin
            if (rst48 || !run48) begin
                rbin <= 0; rgray <= 0; wgray_r1 <= 0; wgray_r2 <= 0;
                dout <= 8'd0; dout_vld <= 1'b0;
            end else begin
                wgray_r1 <= wgray; wgray_r2 <= wgray_r1;
                if (fetch) begin
                    dout     <= fmem[rbin[RAW-1:0]];
                    rbin     <= rbin + 1'b1;
                    rgray    <= ((rbin + 1'b1) >> 1) ^ (rbin + 1'b1);
                    dout_vld <= 1'b1;
                end else if (dac_pop) begin
                    dout_vld <= 1'b0;
                end
            end
        end
    end else begin : g_nordfifo
        // ---- write-only image: no DAC read job ----
        assign room      = {(RAW+1){1'b0}};   // 0 => needy_dac is false (read never picked)
        assign dac_data  = 8'd0;
        assign dac_valid = 1'b0;
    end endgenerate

    // =====================================================================
    // FSM (clk).  Shared S_IDLE round-robin; write path feeds the clk48 gearbox,
    // read path drives the pads on clk with sub-cycle sampling.
    // =====================================================================
    localparam [3:0] S_IDLE=4'd0,
                     S_WCMD=4'd1, S_WADDR=4'd2, S_WDATA=4'd3, S_WCSH=4'd4,
                     S_RLEAD=4'd5, S_RCMD=4'd6, S_RADDR=4'd7, S_RDUMMY=4'd8,
                     S_RDATA=4'd9, S_RCSH=4'd10;
    reg  [3:0]  st;

    // ---- write-path working regs ----
    reg         sel;                  // 0 = LA job, 1 = ADC job (the write job being drained)
    reg  [8:0]  chunk_left;
    reg  [1:0]  addr_idx;
    reg  [1:0]  rr;                   // round-robin pointer: last job served (0=LA,1=ADC,2=DAC)

    wire [7:0]  wsel_head = sel ? afifo[a_rd[WAW-1:0]] : lfifo[l_rd[WAW-1:0]];
    wire        wsel_empty= sel ? a_empty : l_empty;
    wire [23:0] wsel_addr = sel ? adc_addr : la_addr;
    wire        wsel_rem0 = sel ? (adc_rem == 24'd0) : (la_rem == 24'd0);

    // job neediness (S_IDLE arbitration)
    wire        needy_la  = !l_empty && (la_rem  != 24'd0);
    wire        needy_adc = !a_empty && (adc_rem != 24'd0);
    wire        needy_dac = dac_run  && (dac_rem != 24'd0) && (room > {1'b0, RCHUNK});
    wire        any_needy = needy_la | needy_adc | needy_dac;

    // pop one write byte this cycle iff draining with data left AND region not full
    wire        w_pop  = (st == S_WDATA) && !wsel_empty && (chunk_left != 9'd0) && !wsel_rem0;
    wire        a_pop  = w_pop &&  sel;
    wire        l_pop  = w_pop && !sel;

    // ---- write gearbox cell (fed to the clk48 serializer) ----
    reg         cell_tgl;
    reg  [3:0]  cell_n0, cell_n1;
    reg         cell_drv, cell_cs, cell_clk;

    // ---- read-path working regs (clk sub-cycle; ported from dac_psram_reader) ----
    reg         sub;                  // 0 = SCLK low half, 1 = SCLK high half of a nibble
    reg  [3:0]  nib;                  // nibble index within RCMD/RADDR/RDUMMY
    reg  [9:0]  rdcnt;                // read data nibbles remaining (bytes*2)
    reg  [8:0]  rburst;               // bytes this read burst
    reg  [3:0]  rx_hi;                // high nibble of the byte being assembled
    reg  [2:0]  rcsh_cnt;
    reg         rd_path;              // 1 while in a read burst (pad mux select)

    wire [8:0]  dac_next = (dac_rem >= {15'd0, RCHUNK}) ? RCHUNK : dac_rem[8:0];

    // read-path pad drive (clk domain)
    wire        r_sclk_en = (st==S_RCMD)||(st==S_RADDR)||(st==S_RDUMMY)||(st==S_RDATA);
    reg  [3:0]  r_addr_nib;
    always @(*) begin
        case (nib)
            4'd0:    r_addr_nib = dac_addr[23:20];
            4'd1:    r_addr_nib = dac_addr[19:16];
            4'd2:    r_addr_nib = dac_addr[15:12];
            4'd3:    r_addr_nib = dac_addr[11:8];
            4'd4:    r_addr_nib = dac_addr[7:4];
            default: r_addr_nib = dac_addr[3:0];
        endcase
    end
    wire        r_io_oe = (st==S_RCMD) || (st==S_RADDR);
    wire [3:0]  r_io_o  = (st==S_RCMD) ? ((nib==4'd0) ? 4'hE : 4'hB)
                        : (st==S_RADDR) ? r_addr_nib : 4'h0;
    wire        r_cs    = ((st==S_RLEAD) || r_sclk_en) ? 1'b0 : 1'b1;
    wire        r_sclk  = r_sclk_en & sub;

    // =====================================================================
    // clk48 write serializer + DDR SCLK gate (verbatim from psram_dual_writer).
    // Emits one nibble per clk48; detects the FSM's per-cell toggle, latches both
    // nibbles + controls on the edge, emits nib0 then nib1.
    // =====================================================================
    reg        tgl_m, w_sub;
    reg [3:0]  gb_io_o;  reg gb_io_oe, gb_cs, gb_sclk_d1;
    reg [3:0]  n1_l;     reg drv_l, cs_l, clk_l;

    always @(posedge clk48) begin
        tgl_m <= cell_tgl;
        if (rst48) begin
            w_sub <= 1'b0;
            gb_io_o <= 4'h0; gb_io_oe <= 1'b0; gb_cs <= 1'b1; gb_sclk_d1 <= 1'b0;
        end else if (cell_tgl != tgl_m) begin
            n1_l <= cell_n1; drv_l <= cell_drv; cs_l <= cell_cs; clk_l <= cell_clk;
            gb_io_o <= cell_n0; gb_io_oe <= cell_drv; gb_cs <= cell_cs; gb_sclk_d1 <= cell_clk;
            w_sub <= 1'b1;
        end else if (w_sub) begin
            gb_io_o <= n1_l; gb_io_oe <= drv_l; gb_cs <= cs_l; gb_sclk_d1 <= clk_l;
            w_sub <= 1'b0;
        end else begin
            gb_io_oe <= 1'b0; gb_sclk_d1 <= 1'b0;
        end
    end

    // =====================================================================
    // Shared pad mux: read burst -> clk-domain read drive; else -> clk48 gearbox regs.
    // (idle: gearbox sits at cs=1/oe=0, so the pads are safe between bursts.)
    // =====================================================================
    assign psram_io_o    = rd_path ? r_io_o  : gb_io_o;
    assign psram_io_oe   = rd_path ? r_io_oe : gb_io_oe;
    assign psram_cs      = rd_path ? r_cs    : gb_cs;
    assign psram_sclk_d0 = rd_path ? r_sclk  : 1'b0;
    assign psram_sclk_d1 = rd_path ? r_sclk  : gb_sclk_d1;
    assign active        = (st != S_IDLE);
    assign idle          = (st == S_IDLE) && a_empty && l_empty;

    // =====================================================================
    // Main FSM (clk)
    // =====================================================================
    always @(posedge clk) begin
        // FIFO writes + occupancy
        push <= 1'b0;
        if (rst) begin
            a_wr<=0; a_rd<=0; a_cnt<=0; l_wr<=0; l_rd<=0; l_cnt<=0;
        end else begin
            if (a_wr_en) begin afifo[a_wr[WAW-1:0]] <= adc_data; a_wr <= a_wr + 1'b1; end
            if (l_wr_en) begin lfifo[l_wr[WAW-1:0]] <= la_data;  l_wr <= l_wr + 1'b1; end
            if (a_pop) a_rd <= a_rd + 1'b1;
            if (l_pop) l_rd <= l_rd + 1'b1;
            a_cnt <= a_cnt + (a_wr_en ? 1'b1 : 1'b0) - (a_pop ? 1'b1 : 1'b0);
            l_cnt <= l_cnt + (l_wr_en ? 1'b1 : 1'b0) - (l_pop ? 1'b1 : 1'b0);
        end

        // write gearbox cell default = idle (no SCLK, not driving, CS high)
        cell_tgl <= ~cell_tgl;
        cell_n0  <= 4'h0; cell_n1 <= 4'h0;
        cell_drv <= 1'b0; cell_clk <= 1'b0; cell_cs <= 1'b1;

        dac_run_d <= dac_run;

        if (rst) begin
            st <= S_IDLE; sel <= 1'b0; chunk_left <= 9'd0; addr_idx <= 2'd0; rr <= 2'd2;
            la_addr<=24'd0; adc_addr<=24'd0; dac_addr<=24'd0;
            la_rem<=24'd0;  adc_rem<=24'd0;  dac_rem<=24'd0;
            sub<=1'b0; nib<=4'd0; rdcnt<=10'd0; rburst<=9'd0; rx_hi<=4'd0;
            rcsh_cnt<=3'd0; rd_path<=1'b0; cell_tgl<=1'b0; dac_run_d<=1'b0;
        end else begin
            // ---- region priming ----
            if (start) begin
                la_addr<=la_base; la_rem<=la_len; adc_addr<=adc_base; adc_rem<=adc_len;
            end
            if (dac_run && !dac_run_d) begin           // DAC replay armed: prime + wrap-from base
                dac_addr<=dac_base; dac_rem<=dac_len;
            end

            case (st)
            // ---------------- shared idle / round-robin ----------------
            S_IDLE: begin
                rd_path <= 1'b0;
                sub <= 1'b0;
                if (any_needy) begin
                    // rotate priority starting after the last-served job (rr)
                    // order of preference this cycle: (rr+1), (rr+2), (rr+3)==rr
                    if (rr == 2'd0) begin
                        if      (needy_adc) begin sel<=1'b1; rr<=2'd1; chunk_left<=WCHUNK; addr_idx<=2'd0; cell_cs<=1'b0; st<=S_WCMD; end
                        else if (needy_dac) begin rr<=2'd2; rburst<=dac_next; nib<=4'd0; rd_path<=1'b1; st<=S_RLEAD; end
                        else                 begin sel<=1'b0; rr<=2'd0; chunk_left<=WCHUNK; addr_idx<=2'd0; cell_cs<=1'b0; st<=S_WCMD; end
                    end else if (rr == 2'd1) begin
                        if      (needy_dac) begin rr<=2'd2; rburst<=dac_next; nib<=4'd0; rd_path<=1'b1; st<=S_RLEAD; end
                        else if (needy_la)  begin sel<=1'b0; rr<=2'd0; chunk_left<=WCHUNK; addr_idx<=2'd0; cell_cs<=1'b0; st<=S_WCMD; end
                        else                 begin sel<=1'b1; rr<=2'd1; chunk_left<=WCHUNK; addr_idx<=2'd0; cell_cs<=1'b0; st<=S_WCMD; end
                    end else begin // rr == 2 (DAC last) -> prefer LA, ADC, DAC
                        if      (needy_la)  begin sel<=1'b0; rr<=2'd0; chunk_left<=WCHUNK; addr_idx<=2'd0; cell_cs<=1'b0; st<=S_WCMD; end
                        else if (needy_adc) begin sel<=1'b1; rr<=2'd1; chunk_left<=WCHUNK; addr_idx<=2'd0; cell_cs<=1'b0; st<=S_WCMD; end
                        else                 begin rr<=2'd2; rburst<=dac_next; nib<=4'd0; rd_path<=1'b1; st<=S_RLEAD; end
                    end
                end
            end

            // ---------------- write path (clk48 gearbox) ----------------
            S_WCMD: begin
                cell_n0<=4'h3; cell_n1<=4'h8; cell_drv<=1'b1; cell_clk<=1'b1; cell_cs<=1'b0;
                addr_idx<=2'd0; st<=S_WADDR;
            end
            S_WADDR: begin
                cell_drv<=1'b1; cell_clk<=1'b1; cell_cs<=1'b0;
                case (addr_idx)
                    2'd0: begin cell_n0<=wsel_addr[23:20]; cell_n1<=wsel_addr[19:16]; end
                    2'd1: begin cell_n0<=wsel_addr[15:12]; cell_n1<=wsel_addr[11:8];  end
                    default: begin cell_n0<=wsel_addr[7:4]; cell_n1<=wsel_addr[3:0];  end
                endcase
                if (addr_idx==2'd2) st<=S_WDATA;
                else addr_idx<=addr_idx+2'd1;
            end
            S_WDATA: begin
                if (w_pop) begin
                    cell_n0<=wsel_head[7:4]; cell_n1<=wsel_head[3:0];
                    cell_drv<=1'b1; cell_clk<=1'b1; cell_cs<=1'b0;
                    if (sel) begin adc_addr<=adc_addr+24'd1; adc_rem<=adc_rem-24'd1; end
                    else     begin la_addr <=la_addr +24'd1; la_rem <=la_rem -24'd1; end
                    chunk_left<=chunk_left-9'd1;
                    if (chunk_left==9'd1 || (sel ? (adc_rem==24'd1) : (la_rem==24'd1)))
                        begin cell_cs<=1'b0; st<=S_WCSH; end
                end else begin
                    cell_cs<=1'b0; st<=S_WCSH;   // stream drained mid-chunk -> close burst
                end
            end
            S_WCSH: begin
                cell_cs<=1'b1; cell_drv<=1'b0; cell_clk<=1'b0;
                st<=S_IDLE;
            end

            // ---------------- read path (clk sub-cycle -> DAC prefetch FIFO) ----------------
            S_RLEAD: begin
                sub<=~sub;
                if (sub) begin nib<=4'd0; sub<=1'b0; st<=S_RCMD; end
            end
            S_RCMD: begin
                sub<=~sub;
                if (sub) begin
                    if (nib==4'd1) begin nib<=4'd0; st<=S_RADDR; end
                    else nib<=nib+4'd1;
                end
            end
            S_RADDR: begin
                sub<=~sub;
                if (sub) begin
                    if (nib==4'd5) begin nib<=4'd0; st<=S_RDUMMY; end
                    else nib<=nib+4'd1;
                end
            end
            S_RDUMMY: begin
                sub<=~sub;
                if (sub) begin
                    if (nib==(RWAIT-4'd1)) begin nib<=4'd0; rdcnt<={rburst,1'b0}; st<=S_RDATA; end
                    else nib<=nib+4'd1;
                end
            end
            S_RDATA: begin
                sub<=~sub;
                if (sub) begin
                    if (rdcnt[0]) begin push_byte<={rx_hi, psram_io_i}; push<=1'b1; end
                    else rx_hi<=psram_io_i;
                    rdcnt<=rdcnt-10'd1;
                    if (rdcnt==10'd1) begin
                        // burst done: advance/wrap the DAC region pointer
                        if (dac_rem == {15'd0, rburst}) begin
                            dac_addr<=dac_base; dac_rem<=dac_len;      // loop
                        end else begin
                            dac_addr<=dac_addr+{15'd0, rburst};
                            dac_rem <=dac_rem -{15'd0, rburst};
                        end
                        rcsh_cnt<=3'd0; sub<=1'b0; st<=S_RCSH;
                    end
                end
            end
            S_RCSH: begin
                sub<=1'b0;
                if (rcsh_cnt==3'd3) begin rd_path<=1'b0; st<=S_IDLE; end
                else rcsh_cnt<=rcsh_cnt+3'd1;
            end
            default: st<=S_IDLE;
            endcase
        end
    end
endmodule
