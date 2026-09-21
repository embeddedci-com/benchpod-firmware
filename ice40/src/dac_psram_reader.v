// ============================================================================
// dac_psram_reader.v — APS6404L QPI streaming-READ master feeding the DAC.
//
// The counterpart to psram_dual_writer: that DRAINS captures INTO PSRAM, this one
// STREAMS a stored waveform BACK OUT so the DAC8551 can replay it (OP_START_DAC_PSRAM
// 0x13, gateware >= v17).  Reads [base_addr, base_addr+len_bytes), loops at the end,
// and pushes bytes into a prefetch FIFO the DAC engine pops two-at-a-time.
//
// ★ STRUCTURAL SPLIT (the cure for the recurring deep-replay/PSRAM curse) ★
// Mirrors the PROVEN psram_dual_writer exactly:
//   * clk (24 MHz) domain — the "cell producer" FSM: 24-bit address + remaining
//     arithmetic, burst sequencing, loop wrap.  Wide adders live here where they
//     close timing (the reason the old reader put the FSM on clk).  It emits ONE
//     "cell" (2 nibbles + drive/cs/sclk/capture controls) per cell period, flipping
//     `cell_tgl` so the clk48 side gets exactly one cell each time.
//   * clk48 (48 MHz) domain — the serializer/sampler: latches the cell (stable for a
//     full cell period, so a plain clk48 capture has margin), then drives io_o/oe/cs
//     and the SCLK GATE from clk48 REGISTERS, and samples the PSRAM's return data on
//     clk48.  SINGLE-CLOCK prefetch FIFO (captured on clk48, read on clk48 by the DAC).
//
// WHY THIS ENDS THE CURSE: every PSRAM-facing signal — SCLK, CS, IO drive/OE, AND the
// register that samples the return data — is now clk48, so their mutual timing is a
// FIXED, nextpnr-TIMED, placement-INVARIANT relationship.  The old reader generated
// SCLK from a `clk`-domain signal into a clk48 DDR pad and sampled the input in `clk`;
// since `clk` is clk48/2 via a *fabric* divider + SB_GB, nextpnr treats clk and clk48
// as unrelated and NEVER timed those pad paths, so their real skew re-rolled on every
// placement/synth and silently starved the reader on HW (the 0x5555 / seed- and
// yosys-version-roulette PSRAM class).  Only `run`, the static `base/len`, the arbiter
// handshake, and a `room_ok` backpressure level cross domains now — all 2FF-synced
// LEVELS, never combinational fabric paths.  The single-clock FIFO also deletes the
// dual-clock gray-pointer + odd-pop byte-swap wedge class.
//
// Read SCLK = clk48 / 4 = 12 MHz (8 clk48 per cell, 2 nibbles), matching the proven v26
// read margin; the return data is sampled ~41.6 ns after each SCLK rising edge, inside the
// cell.  Read bandwidth (>=3 MB/s) >> the DAC8551's <=1.8 MB/s drain, so the FIFO never
// underruns (verified in tb_psram_arbiter / tb_psram_tri_master at representative rates).
// ============================================================================
module dac_psram_reader #(
    parameter [8:0] CHUNK_BYTES = 9'd16,   // bytes per CS-low burst (tCEM budget)
    parameter [3:0] WAIT_CYCLES = 4'd6,    // 0xEB dummy cycles (APS6404L default; matches fw QREAD_DUMMY)
    parameter       FIFO_AW     = 8        // 2^FIFO_AW-byte prefetch FIFO
)(
    input  wire        clk,                // 24 MHz — cell-producer FSM + arithmetic
    input  wire        rst,                // reset in the clk domain
    input  wire        clk48,              // 48 MHz — serializer/sampler + FIFO
    input  wire        rst48,              // reset in the clk48 domain
    // ---- control (clk domain; base/len held while run=1, and may change on the edge run rises) ----
    input  wire        run,
    input  wire [23:0] base_addr,
    input  wire [23:0] len_bytes,
    // ---- bus arbitration handshake (clk domain; tie bus_gnt=1 for exclusive use) ----
    input  wire        bus_gnt,
    output wire        bus_req,
    output wire        bus_busy,
    // ---- DAC sample-fetch port (clk48 domain; FIFO head) ----
    output wire [7:0]  data,
    output wire        data_valid,
    input  wire        data_pop,
    // ---- PSRAM pad interface (ALL clk48-registered; muxed onto the pins in top_v2) ----
    output reg  [3:0]  io_o,
    output reg         io_oe,
    input  wire [3:0]  io_i,
    output reg         cs,
    output reg         sclk,               // clk48 SCLK level (drive both DDR halves in top)
    output wire        active              // clk48: reader mid-burst (immediate pad-mux select)
);
    // Nibble timing on clk48: each nibble spans 2 clk48 SCLK-low then 2 clk48 SCLK-high
    // (SCLK = clk48/4 = 12 MHz), matching the proven v26 read margin.  A cell = 2 nibbles
    // = 8 clk48; the FSM emits one cell per 4 clk (= 8 clk48).  Crucially BOTH nibbles are
    // sampled WITHIN the cell (CS still low), so the last byte of a burst never samples the
    // released/floating bus.
    localparam integer CELL_CLK48  = 8;                 // clk48 per cell

    // ================================================================
    // clk domain: cell-producer FSM (mirror of psram_dual_writer).  Emits one cell per
    // cell period; `cell_ph` counts clk within a cell period so the FSM steps once per
    // CELL (keeps the clk48 side one-cell-per-toggle, exactly like the writer).
    // ================================================================
    localparam CELL_CLK = CELL_CLK48/2;                 // clk cycles per cell (clk=clk48/2)
    reg  [2:0]  cell_ph = 0;                             // 0..CELL_CLK-1
    wire        cell_go = (cell_ph == (CELL_CLK-1));     // last clk of the cell period

    localparam S_IDLE=3'd0, S_CMD=3'd1, S_ADDR=3'd2, S_DUMMY=3'd3, S_DATA=3'd4, S_CSH=3'd5;
    reg  [2:0]  st = S_IDLE;
    reg  [23:0] rd_addr;
    reg  [23:0] remaining;
    reg  [8:0]  burst_bytes;
    reg  [8:0]  data_left;                               // data cells (bytes) left this burst
    reg  [2:0]  addr_idx;                                // 0..2: three 8-bit address cells
    reg  [2:0]  dum_idx;                                 // dummy cells left

    // the cell handed to the clk48 side
    reg         cell_tgl = 0;
    reg  [3:0]  cell_n0, cell_n1;
    reg         cell_drv, cell_cs, cell_clk, cell_cap;

    // room backpressure: `room_ok` (a stored waveform can fill CHUNK) comes from clk48.
    reg  [1:0]  roomok_s = 0;
    wire        room_ok = roomok_s[1];

    // `run` rises on the SAME clk edge that START_DAC_PSRAM writes base_addr/len_bytes (all three
    // are cmd_dispatch registers).  rd_addr/remaining reload from the ports while !run, so bursting
    // on the first run cycle used the PREVIOUS values: every re-arm's first burst came from the last
    // replay's region (v34 fix — the v33 DAC bug class, see cdc_pulse_payload.v).  The FSM
    // therefore runs one clk after `run`: the first run cycle is still a reload, of the new values.
    reg         run_d = 1'b0;
    always @(posedge clk) run_d <= rst ? 1'b0 : run;
    wire        run_i = run & run_d;

    wire [8:0]  next_burst = (remaining >= {15'd0, CHUNK_BYTES}) ? CHUNK_BYTES : remaining[8:0];
    wire        want_burst = run_i && (remaining != 24'd0) && room_ok;

    assign bus_req  = want_burst;      // clk-domain -> arbiter directly (no CDC; FSM is on clk)
    assign bus_busy = (st != S_IDLE);

    localparam integer DUM_CELLS = (WAIT_CYCLES + 1) / 2;   // dummy NIBBLES -> cells (2 nib/cell)

    always @(posedge clk) begin
        if (rst) begin
            st<=S_IDLE; rd_addr<=base_addr; remaining<=len_bytes; burst_bytes<=0;
            data_left<=0; addr_idx<=0; dum_idx<=0; cell_ph<=0; cell_tgl<=1'b0;
            cell_n0<=0; cell_n1<=0; cell_drv<=0; cell_cs<=1'b1; cell_clk<=0; cell_cap<=0;
        end else if (!run_i) begin
            st<=S_IDLE; rd_addr<=base_addr; remaining<=len_bytes;
            cell_cs<=1'b1; cell_drv<=0; cell_clk<=0; cell_cap<=0;
        end else begin
            cell_ph <= cell_go ? 3'd0 : (cell_ph + 3'd1);
            if (cell_go) begin
                // advance one CELL: flip the toggle + default the cell to idle, then
                // each state overrides.
                cell_tgl <= ~cell_tgl;
                cell_n0<=4'h0; cell_n1<=4'h0; cell_drv<=1'b0; cell_cs<=1'b1; cell_clk<=1'b0; cell_cap<=1'b0;
                case (st)
                S_IDLE: begin
                    if (want_burst && bus_gnt) begin
                        burst_bytes <= next_burst;
                        data_left   <= next_burst;
                        addr_idx    <= 3'd0;
                        cell_cs     <= 1'b0;             // CS-low lead cell (no SCLK yet)
                        st <= S_CMD;
                    end
                end
                S_CMD: begin                            // 0xEB -> nib0=0xE, nib1=0xB
                    cell_n0<=4'hE; cell_n1<=4'hB;
                    cell_drv<=1'b1; cell_clk<=1'b1; cell_cs<=1'b0;
                    addr_idx<=3'd0;
                    st<=S_ADDR;
                end
                S_ADDR: begin                           // 24-bit address, MSB nibble first
                    cell_drv<=1'b1; cell_clk<=1'b1; cell_cs<=1'b0;
                    case (addr_idx)
                        3'd0: begin cell_n0<=rd_addr[23:20]; cell_n1<=rd_addr[19:16]; end
                        3'd1: begin cell_n0<=rd_addr[15:12]; cell_n1<=rd_addr[11:8];  end
                        default: begin cell_n0<=rd_addr[7:4]; cell_n1<=rd_addr[3:0];  end
                    endcase
                    if (addr_idx==3'd2) begin dum_idx<=DUM_CELLS[2:0]; st<=S_DUMMY; end
                    else addr_idx<=addr_idx+3'd1;
                end
                S_DUMMY: begin                          // dummy clocks, bus released
                    cell_drv<=1'b0; cell_clk<=1'b1; cell_cs<=1'b0;
                    if (dum_idx==3'd1) st<=S_DATA;
                    dum_idx<=dum_idx-3'd1;
                end
                S_DATA: begin                           // capture one byte (2 nibbles) per cell
                    cell_drv<=1'b0; cell_clk<=1'b1; cell_cs<=1'b0; cell_cap<=1'b1;
                    data_left<=data_left-9'd1;
                    if (data_left==9'd1) begin          // last byte: advance / wrap next cell
                        if (remaining == {15'd0, burst_bytes}) begin
                            rd_addr<=base_addr; remaining<=len_bytes;
                        end else begin
                            rd_addr<=rd_addr+{15'd0, burst_bytes};
                            remaining<=remaining-{15'd0, burst_bytes};
                        end
                        cell_cs<=1'b0; st<=S_CSH;
                    end
                end
                S_CSH: begin                            // CS-high refresh gap
                    cell_cs<=1'b1; cell_drv<=1'b0; cell_clk<=1'b0; cell_cap<=1'b0;
                    st<=S_IDLE;
                end
                default: st<=S_IDLE;
                endcase
            end
        end
    end

    // The pad-mux select: the clk48 CS being asserted (low) means the reader is driving the
    // shared bus this clk48 — tracks the actual pad activity, not the clk-domain FSM state.
    assign active = ~cs;

    // ================================================================
    // clk48 domain: cell serializer + data sampler.  Detect the cell toggle, latch the
    // cell (stable a full cell period), then walk the 2 nibbles.  `ph` counts clk48
    // within the cell (0..CELL_CLK48-1).  SCLK = high during the 2nd half of each
    // nibble.  Driven nibbles are registered a half-nibble early (setup before SCLK
    // rises).  Data nibbles are sampled at SAMPLE_PH (late) for round-trip margin.
    // ================================================================
    reg         tgl_m = 0;
    reg  [3:0]  n0_l, n1_l;
    reg         drv_l, cs_l, clk_l, cap_l;
    reg  [2:0]  ph = 0;                                  // clk48 phase within the 8-clk48 cell
    reg         busy48 = 0;                              // 1 while walking a cell

    // 8-clk48 cell, nibble = ph[2] (0=nib0, 1=nib1).  SCLK is 25%-low / 75%-high per nibble
    // (low only on ph0/ph4), so the SCLK RISES EARLY — at ph0->1 (nib0) and ph4->5 (nib1) —
    // and we sample the PSRAM's return data at the END of the long high phase (ph3 for nib0,
    // ph7 for nib1) = a FIXED 2 clk48 (~41.6 ns) after the rising edge, matching the proven
    // v26 read margin, placement-invariant, and BOTH inside the cell so CS is still low when
    // the last byte's low nibble is sampled.  Driven cmd/addr nibbles still get 1 clk48 of
    // SCLK-low setup before the rising.
    reg  [3:0]  cap_hi;                                 // captured high nibble (nib0)

    // ---- single-clock (clk48) prefetch FIFO ----
    localparam AW = FIFO_AW;
    reg  [7:0]  fmem [0:(1<<AW)-1];
    reg  [AW:0] wptr = 0, rptr = 0;
    wire [AW:0] occ   = wptr - rptr;
    wire        f_full = occ[AW];
    wire [AW:0] room  = (1 << AW) - occ;
    reg  [7:0]  push_byte;
    reg         push;

    reg  [7:0]  dout = 0;
    reg         dout_vld = 0;
    // FWFT: refill the head only when it is empty (a 1-clk48 bubble per pop; still >=3 MB/s vs the
    // DAC's <=1.8 MB/s drain).  Empty = the DIRECT compare of the two binary pointers: for a
    // SINGLE-clock FIFO that is the shortest path (a registered "next-cycle empty" pipeline HURT
    // timing here — it added the next-pointer mux — the opposite of the dual-clock FIFO it once
    // helped).  The FIFO read side is not the clk48 critical path binding the deep image anyway.
    wire        fifo_empty = (wptr == rptr);
    wire        fetch = !dout_vld && !fifo_empty;
    assign      data       = dout;
    assign      data_valid = dout_vld;

    // room_ok (clk48) -> clk, 2FF level
    reg         roomok48 = 0;
    always @(posedge clk48) roomok48 <= (room > {1'b0, CHUNK_BYTES});
    always @(posedge clk) roomok_s <= {roomok_s[0], roomok48};

    // run 2FF into clk48 (FIFO reset uses it so each arm starts clean/aligned)
    reg  [1:0]  run48_s = 0;
    wire        run48 = run48_s[1];
    always @(posedge clk48) run48_s <= {run48_s[0], run_i};

    always @(posedge clk48) begin
        push   <= 1'b0;
        tgl_m  <= cell_tgl;

        if (rst48 || !run48) begin
            ph<=0; busy48<=0; io_o<=4'h0; io_oe<=1'b0; cs<=1'b1; sclk<=1'b0;
            cap_hi<=0;
            wptr<=0; rptr<=0; dout<=8'd0; dout_vld<=1'b0;
            n0_l<=0; n1_l<=0; drv_l<=0; cs_l<=1'b1; clk_l<=0; cap_l<=0;
        end else begin
            // ---- FIFO write (capture) + FWFT read side ----
            if (push && !f_full) begin fmem[wptr[AW-1:0]] <= push_byte; wptr <= wptr + 1'b1; end
            if (fetch) begin dout <= fmem[rptr[AW-1:0]]; rptr <= rptr + 1'b1; dout_vld <= 1'b1; end
            else if (data_pop) dout_vld <= 1'b0;

            // ---- cell serializer (8 clk48 per cell) ----
            if (cell_tgl != tgl_m) begin
                // new cell -> latch it, drive nib0 (ph0, SCLK low), walk ph 1..7
                n0_l<=cell_n0; n1_l<=cell_n1; drv_l<=cell_drv; cs_l<=cell_cs; clk_l<=cell_clk; cap_l<=cell_cap;
                io_o  <= cell_n0; io_oe <= cell_drv; cs <= cell_cs; sclk <= 1'b0;
                ph<=3'd1; busy48<=1'b1;
            end else if (busy48) begin
                ph <= (ph == 3'd7) ? 3'd0 : (ph + 3'd1);
                // nibble select + SCLK level for THIS ph
                io_o  <= ph[2] ? n1_l : n0_l;
                io_oe <= drv_l;
                cs    <= cs_l;
                sclk  <= clk_l & (ph[0] | ph[1]); // low only on ph0/ph4 -> rises early (ph0->1, ph4->5)
                case (ph)
                    3'd3: if (cap_l) cap_hi <= io_i;                 // sample nib0 (high nibble)
                    3'd7: begin                                     // sample nib1 (low nibble) + emit byte
                        if (cap_l) begin push_byte <= {cap_hi, io_i}; push <= 1'b1; end
                        busy48 <= 1'b0;                             // cell done; await next toggle
                    end
                    default: ;
                endcase
            end else begin
                sclk <= 1'b0;                     // between cells: SCLK idle low, hold cs/io
            end
        end
    end

    // ================================================================
    // CDC OUT: reader busy/req (clk domain here) already in clk; no resync needed.
    // ================================================================
endmodule
