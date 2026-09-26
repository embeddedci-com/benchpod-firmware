// ============================================================================
// psram_dual_writer.v — APS6404L QPI streaming-write master for TWO independent
// streams into TWO contiguous PSRAM regions (VERSION=v2 unified capture).
//
// Drains two byte streams — the ADC capture and the raw 12-channel LA capture —
// into two separate PSRAM regions (ADC_BASE / LA_BASE) off ONE shared trigger, so
// the two captures are time-aligned by construction (sample i of each stream is at
// t0 + i*divider, and both counters start at `start`).  See
// docs/unified-adc-la-capture-redesign.md.
//
// ---- 48 MHz DDR drain (docs/task8-48mhz-ddr-drain.md) ----
// The control FSM, both staging FIFOs and the region arbitration all run on the
// 24 MHz logic clock `clk` (single-clock capture datapath — the collapse that fixed
// the 0x5555 mis-latch, docs/adc-capture-cdc-review.md).  Only the OUTPUT is sped
// up: the FSM emits one "cell" (= 2 nibbles = 1 byte + bus controls) per `clk`
// cycle, and a thin serializer on `clk48` shifts each cell out as ONE NIBBLE PER
// clk48 CYCLE (48 Mnib/s -> ~24 MB/s raw).  The clk->clk48 hop is a SYNCHRONOUS 1:2
// gearbox (integer ratio, phase-locked — NOT the async class that caused 0x5555):
// the FSM flips `cell_tgl` every clk cycle; the serializer detects that toggle,
// latches BOTH nibbles at detection (so nib1 is never re-read from the clk domain
// at the clk boundary — the one race that would depend on clock skew), then emits
// nib0 then nib1.
//
// SCLK is generated OUTSIDE this module by a DDR SB_IO on `clk48` (D_OUT_0=0,
// D_OUT_1 = `psram_sclk_d1`), so the pad shows 0 during the clk48-high half and the
// gate during the low half -> SCLK rises at the clk48 negedge = dead-centre of each
// nibble's data eye.  The data pads are COMBINATIONAL SB_IO fed by the serializer's
// clk48 register `psram_io_o` (stable for the full clk48 period).  Data-to-SCLK
// setup/hold are each ~10.4 ns, guaranteed by the posedge-vs-negedge hardware
// relationship — independent of the SB_GB skew (the robustness win over driving
// data on clk and SCLK on clk48).  QPI mechanics are otherwise identical to the old
// writer (PSRAM already in QPI mode; writes = 0x38 + 24-bit addr; chunked so CS-low
// stays < APS6404L tCEM ~8 us).  Region arbitration: each chunk services whichever
// staging FIFO is fuller (proportional, starvation-free), re-issuing 0x38 + that
// region's running address.
//
//   • CHUNK 32 @ 48 Mnib/s: (2+6+64) nib = 72 nib -> 1.5 us CS-low (safe vs tCEM).
// The DEEP burst buffering is the two SPRAM rings upstream (top_v2); these staging
// FIFOs are just the chunk-assembly buffers.  The shared quad bus is tristated at
// the pads (top_v2) when bus_own (STM32 PG0) is asserted for read-back.
// ============================================================================
module psram_dual_writer #(
    parameter        CHUNK_BYTES = 9'd32         // bytes per CS-low burst (tCEM)
)(
    input  wire        clk,               // 24 MHz logic clock (FSM + FIFOs)
    input  wire        clk48,             // 48 MHz output clock (serializer + DDR SCLK)
    input  wire        rst,               // sync reset in the clk domain
    input  wire        bus_own,           // 1 = STM32 owns the bus (top tristates pads; synced)
    // RUNTIME region bases (byte addr) — latched at `start` for dynamic tri-capture
    // zone allocation (was compile-time ADC_BASE/LA_BASE params).
    input  wire [23:0] adc_base,          // ADC region base
    input  wire [23:0] la_base,           // LA region base
    // control (clk domain)
    input  wire        start,             // 1-cycle: reset both region ptrs, go active
    input  wire        stop,              // 1-cycle: end (flush both FIFOs + idle)
    // bus arbitration handshake (clk domain; tie bus_gnt=1 for exclusive use)
    input  wire        bus_gnt,           // 1 = may start a burst (held through the burst)
    output wire        bus_req,           // 1 = wants a burst (a staging FIFO has data)
    output wire        bus_busy,          // 1 = currently in a CS-low burst
    // ADC input stream (clk domain)
    input  wire [7:0]  adc_data,
    input  wire        adc_stb,
    output wire        adc_full,          // ADC staging-FIFO full (backpressure)
    // LA input stream (clk domain)
    input  wire [7:0]  la_data,
    input  wire        la_stb,
    output wire        la_full,           // LA staging-FIFO full (backpressure)
    // status (clk domain)
    output reg         idle,              // both FIFOs drained + not in a burst
    output reg         bus_lost,          // 1 clk: a burst was cut short by bus_own (v43)
    // pad-facing outputs (clk48-timed — see header)
    output reg  [3:0]  psram_io_o,        // nibble on the bus this clk48 cycle
    output reg         psram_io_oe,       // 1 = drive io pads this clk48 cycle
    output reg         psram_cs,          // active low (clk48 reg)
    output reg         psram_sclk_d1      // DDR D_OUT_1 for the SCLK pad (top ties D_OUT_0=0)
);
    // 32-byte staging FIFO per stream.  These FIFOs have an ASYNC (combinational) read
    // (sel_head below), so on the iCE40 — which has no LUT-RAM — they map to flip-flops
    // plus a read mux, NOT BRAM.  Depth only needs to cover one CHUNK-byte burst plus a
    // little write-side headroom (the deep buffering is the upstream spram_ring16), so
    // 32 is plenty for CHUNK_BYTES<=16 and halves the FF+mux LC vs the old 64 — LC the
    // concurrent-PSRAM arbiter needs back to place on the ~87%-full up5k.
    localparam AW = 5;

    // =====================================================================
    // clk domain: staging FIFOs + region arbitration + cell-producer FSM
    // =====================================================================

    // ---- ADC staging FIFO ----
    reg  [7:0]  afifo [0:(1<<AW)-1];
    reg  [AW:0] a_wr, a_rd, a_cnt;
    wire        a_full  = (a_cnt >= ({1'b0,{AW{1'b1}}}));
    wire        a_empty = (a_cnt == 0);
    wire        a_wr_en = adc_stb && !a_full;
    assign      adc_full = a_full;

    // ---- LA staging FIFO ----
    reg  [7:0]  lfifo [0:(1<<AW)-1];
    reg  [AW:0] l_wr, l_rd, l_cnt;
    wire        l_full  = (l_cnt >= ({1'b0,{AW{1'b1}}}));
    wire        l_empty = (l_cnt == 0);
    wire        l_wr_en = la_stb && !l_full;
    assign      la_full = l_full;

    // ---- cell-producer FSM state ----
    localparam S_IDLE=0, S_CMD=1, S_ADDR=2, S_DATA=3, S_CSH=4;
    reg  [2:0]  st;
    wire [23:0] addr_adc, addr_la;        // running byte address per region (DSP counters, below)
    reg         active;
    reg         sel;                      // 0 = ADC region, 1 = LA region (latched per burst)
    reg  [8:0]  chunk_left;
    reg  [1:0]  addr_idx;                 // 0..2: the three 8-bit address cells

    // The cell handed to the clk48 serializer.  cell_tgl flips EVERY clk cycle so the
    // serializer always gets exactly one cell per clk period.
    reg         cell_tgl;
    reg  [3:0]  cell_n0, cell_n1;         // nibble 0 (first), nibble 1 (second)
    reg         cell_drv;                 // 1 = drive io during this cell
    reg         cell_cs;                  // CS level for this cell (0 = asserted low)
    reg         cell_clk;                 // 1 = pulse SCLK for these 2 nibbles

    // Region pick for the NEXT burst: prefer whichever FIFO is fuller (ties -> LA,
    // the higher-rate stream), fall back to the non-empty one.
    wire        any_data = !a_empty || !l_empty;
    wire        pick_la  = !l_empty && ((l_cnt >= a_cnt) || a_empty);

    // ---- bus arbitration handshake ----
    // req: a staging FIFO has data to drain.  busy: we hold the bus for the whole
    // burst (S_CMD..S_CSH); only S_IDLE is quiescent (CS high).
    assign bus_req    = any_data;
    assign bus_busy   = (st != S_IDLE);

    // Selected-FIFO head + emptiness for the DATA state.
    wire [7:0]  sel_head  = sel ? lfifo[l_rd[AW-1:0]] : afifo[a_rd[AW-1:0]];
    wire        sel_empty = sel ? l_empty : a_empty;

    // Running address of the selected region (sliced in S_ADDR; a bare wire so the
    // part-selects below are legal Verilog, not a slice of a parenthesised ternary).
    wire [23:0] cur_addr  = sel ? addr_la : addr_adc;

    // A data byte is popped this cycle iff we're in DATA with data left in the chunk.
    // v43: nothing is popped while the STM32 owns the bus (the pads are tristated, so the byte
    // would be lost silently).
    wire        pop_this  = (st == S_DATA) && !bus_own && !sel_empty && (chunk_left != 9'd0);
    wire        a_pop      = pop_this && !sel;
    wire        l_pop      = pop_this &&  sel;

    // Running region addresses: SB_MAC16 up-counters since v41 (dsp_counter), not 2 x 24 fabric
    // LCs.  Same behaviour as the old always-block: reset and `start` load the base, a pop steps
    // the selected region, and a pop on the same edge as `start` wins (the old code's later
    // assignment did).
    wire [31:0] addr_adc_q, addr_la_q;
    assign addr_adc = addr_adc_q[23:0];
    assign addr_la  = addr_la_q[23:0];
    wire        adc_step = ~rst & a_pop;
    wire        la_step  = ~rst & l_pop;
    dsp_counter #(.UP(1)) addr_adc_i (
        .clk(clk), .load(rst | (start & ~adc_step)), .load_val({8'd0, adc_base}),
        .en(adc_step), .q(addr_adc_q), .flag());
    dsp_counter #(.UP(1)) addr_la_i (
        .clk(clk), .load(rst | (start & ~la_step)), .load_val({8'd0, la_base}),
        .en(la_step), .q(addr_la_q), .flag());

    always @(posedge clk) begin
        // ---- FIFO writes + occupancy (both FIFOs; +1 on accepted write, -1 on pop) ----
        if (rst) begin
            a_wr <= 0; a_rd <= 0; a_cnt <= 0;
            l_wr <= 0; l_rd <= 0; l_cnt <= 0;
        end else begin
            if (a_wr_en) begin afifo[a_wr[AW-1:0]] <= adc_data; a_wr <= a_wr + 1'b1; end
            if (l_wr_en) begin lfifo[l_wr[AW-1:0]] <= la_data;  l_wr <= l_wr + 1'b1; end
            if (a_pop) a_rd <= a_rd + 1'b1;
            if (l_pop) l_rd <= l_rd + 1'b1;
            a_cnt <= a_cnt + (a_wr_en ? 1'b1 : 1'b0) - (a_pop ? 1'b1 : 1'b0);
            l_cnt <= l_cnt + (l_wr_en ? 1'b1 : 1'b0) - (l_pop ? 1'b1 : 1'b0);
        end

        // ---- cell-producer FSM ----
        // Default: this cell is idle (no SCLK, not driving, CS held high).  Each state
        // overrides as needed.  cell_tgl always flips -> one cell per clk period.
        cell_tgl <= ~cell_tgl;
        cell_n0  <= 4'h0; cell_n1 <= 4'h0;
        cell_drv <= 1'b0; cell_clk <= 1'b0; cell_cs <= 1'b1;
        // v43: bus_own mid-burst.  The pads were already tristated (raw bus_own), so this burst's
        // bytes never reached the PSRAM; close it (CS high) and report it, so the firmware fails
        // the capture instead of reading back a hole.  Never cut here in normal use: the firmware
        // only takes the bus while the writer is idle.
        bus_lost <= ~rst & bus_own & (st == S_CMD || st == S_ADDR || st == S_DATA);

        if (rst) begin
            st <= S_IDLE; active <= 1'b0;   // (addr_adc/addr_la load in their dsp_counters)
            idle <= 1'b1; sel <= 1'b0; chunk_left <= 9'd0; addr_idx <= 2'd0;
            cell_tgl <= 1'b0;   // defined start so ~cell_tgl toggles (never stays X)
        end else begin
            if (start) active <= 1'b1;     // (addr_adc/addr_la load in their dsp_counters)
            if (stop)  active <= 1'b0;

            case (st)
            // Idle until either FIFO has data (capturing OR draining the tail after
            // `stop`).  On data, assert CS one cell early (setup) and go issue the cmd.
            S_IDLE: begin
                idle <= 1'b1;
                // Only commit to a burst when the arbiter has granted the bus.
                // bus_gnt=1 (exclusive use) reduces this to the original `any_data`.
                if (any_data && bus_gnt && !bus_own) begin   // v43: never while the STM32 owns it
                    idle <= 1'b0;
                    sel        <= pick_la;
                    chunk_left <= CHUNK_BYTES;
                    addr_idx   <= 2'd0;
                    cell_cs    <= 1'b0;          // CS-low lead cell (no SCLK yet)
                    st <= S_CMD;
                end
            end
            // 0x38 write command: nib0=0x3, nib1=0x8.
            S_CMD: if (bus_own) st <= S_CSH; else begin
                cell_n0 <= 4'h3; cell_n1 <= 4'h8;
                cell_drv <= 1'b1; cell_clk <= 1'b1; cell_cs <= 1'b0;
                addr_idx <= 2'd0;
                st <= S_ADDR;
            end
            // 24-bit address of the SELECTED region: three 8-bit cells, MSB first.
            S_ADDR: if (bus_own) st <= S_CSH; else begin
                cell_drv <= 1'b1; cell_clk <= 1'b1; cell_cs <= 1'b0;
                case (addr_idx)
                    2'd0: begin cell_n0 <= cur_addr[23:20]; cell_n1 <= cur_addr[19:16]; end
                    2'd1: begin cell_n0 <= cur_addr[15:12]; cell_n1 <= cur_addr[11:8];  end
                    default: begin cell_n0 <= cur_addr[7:4]; cell_n1 <= cur_addr[3:0];  end
                endcase
                if (addr_idx == 2'd2) st <= S_DATA;
                else addr_idx <= addr_idx + 2'd1;
            end
            // Data: one byte (2 nibbles) per cell, popped from the selected FIFO.
            S_DATA: begin
                if (pop_this) begin
                    cell_n0 <= sel_head[7:4]; cell_n1 <= sel_head[3:0];
                    cell_drv <= 1'b1; cell_clk <= 1'b1; cell_cs <= 1'b0;
                    // (addr_la / addr_adc step in their dsp_counters on this edge)
                    chunk_left <= chunk_left - 9'd1;
                    // last byte of the chunk -> close the burst next cell
                    if (chunk_left == 9'd1) begin cell_cs <= 1'b0; st <= S_CSH; end
                end else begin
                    // selected stream drained mid-chunk (or bus_own: nothing popped) -> close
                    cell_cs <= bus_own; st <= S_CSH;
                end
            end
            // Chip-select-high gap (lets the PSRAM refresh between bursts): one cell
            // with CS released and no SCLK, then back to idle.
            S_CSH: begin
                cell_cs <= 1'b1; cell_drv <= 1'b0; cell_clk <= 1'b0;
                st <= S_IDLE;
            end
            default: st <= S_IDLE;
            endcase
        end
    end

    // =====================================================================
    // clk48 domain: 1:2 serializer (one nibble per clk48 cycle) + DDR SCLK gate
    // =====================================================================
    // Detect the FSM's per-cell toggle with a single clk48 capture; on the edge,
    // LATCH both nibbles + controls (cell is stable for the whole clk period, so this
    // capture has ~one clk48 of setup) and emit nib0; the following cycle emit the
    // latched nib1.  All clk-domain reads happen only on the detection cycle, so the
    // nib1 emission never races the next clk edge.
    reg        tgl_m;                     // last-cycle capture of cell_tgl
    reg        sub;                       // 0 = waiting for a new cell, 1 = emit nib1 next
    reg [3:0]  n1_l;                      // latched second nibble
    reg        drv_l, cs_l, clk_l;        // latched controls
    reg        rst48_q, rst48;            // reset synced into clk48 (release only)

    always @(posedge clk48) begin
        rst48_q <= rst; rst48 <= rst48_q; // 2-FF: rst is a clk-domain level
        tgl_m   <= cell_tgl;
        if (rst48) begin
            sub <= 1'b0;
            psram_io_o <= 4'h0; psram_io_oe <= 1'b0; psram_cs <= 1'b1; psram_sclk_d1 <= 1'b0;
        end else if (cell_tgl != tgl_m) begin
            // new cell this clk48 cycle -> latch it and emit nibble 0
            n1_l  <= cell_n1; drv_l <= cell_drv; cs_l <= cell_cs; clk_l <= cell_clk;
            psram_io_o    <= cell_n0;
            psram_io_oe   <= cell_drv;
            psram_cs      <= cell_cs;
            psram_sclk_d1 <= cell_clk;    // SCLK pulses (rises at clk48 negedge) this cycle
            sub <= 1'b1;
        end else if (sub) begin
            // second clk48 cycle of the cell -> emit the latched nibble 1
            psram_io_o    <= n1_l;
            psram_io_oe   <= drv_l;
            psram_cs      <= cs_l;
            psram_sclk_d1 <= clk_l;
            sub <= 1'b0;
        end else begin
            // no new cell arrived when expected (only at start-up / stalls): idle safe
            psram_io_oe   <= 1'b0;
            psram_sclk_d1 <= 1'b0;
        end
    end
endmodule
