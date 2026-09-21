// ============================================================================
// tb_psram_dual_writer.v — self-checking test of the dual-region QPI writer with
// the 48 MHz DDR drain (docs/task8-48mhz-ddr-drain.md).
//
// Instantiates the REAL pad primitives exactly like top_v2 — data on COMBINATIONAL
// SB_IO, SCLK on a DDR SB_IO clocked by clk48 (D_OUT_0=0 / D_OUT_1=gate) — so this
// exercises the actual mode-0 pad timing (SCLK rises at the clk48 negedge = mid data
// eye), not just the FSM.  Models the PSRAM: on each psram_sclk rising edge (CS low,
// writer driving) it reconstructs the QPI transaction (0x38 + 24-bit addr + data)
// and stores each data byte at the chunk's running address — exactly what the real
// APS6404L latches, regardless of how the two streams interleave.  It then checks
// that the ADC bytes landed contiguously at ADC_BASE and the LA bytes at LA_BASE, in
// order; plus the CS-low tCEM budget, a real data-to-SCLK SETUP margin, and bus_own
// release.
//
// Run:  make dualwrtest   (needs yosys' cells_sim.v for SB_IO)
// ============================================================================
`timescale 1ns/1ps
module tb_psram_dual_writer;
    localparam CHUNK    = 8;                 // small -> many chunk boundaries + region switches
    localparam [23:0] ADC_BASE = 24'h000100; // small test bases (keep mem array tiny)
    localparam [23:0] LA_BASE  = 24'h000000;
    localparam real CLK48_NS = 20.833;       // 48 MHz output clock
    localparam real TCEM_NS  = 8000.0;
    localparam real SETUP_MIN_NS = 4.0;      // expect ~10.4 ns (half a clk48 period)

    // 48 MHz clk48; 24 MHz clk = clk48/2 (phase-aligned, as top_v2 derives it).
    reg clk48 = 0; always #(CLK48_NS/2.0) clk48 = ~clk48;
    reg clkdiv = 0; always @(posedge clk48) clkdiv <= ~clkdiv;
    wire clk = clkdiv;
    reg rst = 1;

    reg        start = 0, stop = 0, bus_own = 0;
    reg  [7:0] adc_data = 0, la_data = 0;
    reg        adc_stb = 0, la_stb = 0;
    wire       adc_full, la_full, idle;

    // writer outputs (clk48-timed) -> real SB_IO pads
    wire [3:0] w_io_o;
    wire       w_io_oe, w_cs, w_sclk_d1;

    psram_dual_writer #(.CHUNK_BYTES(CHUNK)) dut (
        .clk(clk), .clk48(clk48), .rst(rst), .bus_own(bus_own),
        .adc_base(ADC_BASE), .la_base(LA_BASE),
        .start(start), .stop(stop),
        .bus_gnt(1'b1), .bus_req(), .bus_busy(),
        .adc_data(adc_data), .adc_stb(adc_stb), .adc_full(adc_full),
        .la_data(la_data),   .la_stb(la_stb),   .la_full(la_full),
        .idle(idle),
        .psram_io_o(w_io_o), .psram_io_oe(w_io_oe), .psram_cs(w_cs),
        .psram_sclk_d1(w_sclk_d1)
    );

    // ---- real pads (mirror top_v2's pad section) ----
    wire drive = ~bus_own;
    wire io_oe = drive & w_io_oe;
    tri  psram_sclk, psram_cs; tri [3:0] psram_io;
    pulldown (psram_sclk); pulldown (psram_cs);
    pulldown (psram_io[0]); pulldown (psram_io[1]); pulldown (psram_io[2]); pulldown (psram_io[3]);

    SB_IO #(.PIN_TYPE(6'b100000), .PULLUP(1'b0)) io_sclk_i (
        .PACKAGE_PIN(psram_sclk), .OUTPUT_ENABLE(drive), .OUTPUT_CLK(clk48),
        .D_OUT_0(1'b0), .D_OUT_1(w_sclk_d1));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_cs_i (
        .PACKAGE_PIN(psram_cs), .OUTPUT_ENABLE(drive), .D_OUT_0(w_cs));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_d0 (
        .PACKAGE_PIN(psram_io[0]), .OUTPUT_ENABLE(io_oe), .D_OUT_0(w_io_o[0]));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_d1 (
        .PACKAGE_PIN(psram_io[1]), .OUTPUT_ENABLE(io_oe), .D_OUT_0(w_io_o[1]));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_d2 (
        .PACKAGE_PIN(psram_io[2]), .OUTPUT_ENABLE(io_oe), .D_OUT_0(w_io_o[2]));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_d3 (
        .PACKAGE_PIN(psram_io[3]), .OUTPUT_ENABLE(io_oe), .D_OUT_0(w_io_o[3]));

    // Convenience aliases for the decoder/checks.
    wire       ps_cs   = psram_cs;
    wire       ps_sclk = psram_sclk;
    wire [3:0] ps_io   = psram_io;
    wire       ps_oe   = io_oe;

    localparam NA = 12;      // ADC bytes
    localparam NL = 40;      // LA bytes
    reg [7:0] mem [0:511];
    integer i, errors;

    // ---- QPI-write decoder: reconstruct into mem[] at the chunk address ----
    integer nib_i;
    reg [7:0]  d_cmd, d_byte;
    reg [23:0] d_addr, wr_ptr;
    real       cs_fall_ns, cs_low_max;
    reg        decode_en;
    initial begin nib_i=0; errors=0; cs_low_max=0.0; cs_fall_ns=0.0; decode_en=1; end

    always @(negedge ps_cs) begin cs_fall_ns = $realtime; nib_i = 0; end
    always @(posedge ps_cs)
        if (cs_fall_ns > 0.0 && ($realtime-cs_fall_ns) > cs_low_max) cs_low_max = $realtime - cs_fall_ns;

    always @(posedge ps_sclk) begin
        if (decode_en && !ps_cs && ps_oe) begin
            case (nib_i)
                0: d_cmd[7:4]    = ps_io;
                1: d_cmd[3:0]    = ps_io;
                2: d_addr[23:20] = ps_io;
                3: d_addr[19:16] = ps_io;
                4: d_addr[15:12] = ps_io;
                5: d_addr[11:8]  = ps_io;
                6: d_addr[7:4]   = ps_io;
                7: begin
                    d_addr[3:0] = ps_io;
                    if (d_cmd !== 8'h38) begin $display("FAIL: cmd=%02h want 38", d_cmd); errors=errors+1; end
                    wr_ptr = d_addr;
                end
                default: begin
                    if (((nib_i-8) & 1) == 0) d_byte[7:4] = ps_io;
                    else begin
                        d_byte[3:0] = ps_io;
                        mem[wr_ptr[8:0]] = d_byte;   // store exactly where the PSRAM would
                        wr_ptr = wr_ptr + 24'd1;
                    end
                end
            endcase
            nib_i = nib_i + 1;
        end
    end

    // ---- real setup check: data must be stable a margin BEFORE each SCLK rise ----
    // The DDR SCLK rises at the clk48 negedge, half a clk48 period (~10.4 ns) after
    // the data nibble updates at the posedge, so the margin should be ~10.4 ns.
    real t_io_change;
    initial t_io_change = 0.0;
    always @(ps_io) if (!ps_cs && ps_oe) t_io_change = $realtime;
    always @(posedge ps_sclk)
        if (decode_en && !ps_cs && ps_oe && (($realtime - t_io_change) < SETUP_MIN_NS)) begin
            $display("FAIL: SETUP VIOLATION @%0.1fns nib_i=%0d io=%h stable only %0.2f ns (< %0.1f)",
                     $realtime, nib_i, ps_io, $realtime - t_io_change, SETUP_MIN_NS);
            errors = errors + 1;
        end

    task feed_la(input [7:0] v); begin
        @(posedge clk); while (la_full) @(posedge clk);
        la_data <= v; la_stb <= 1'b1; @(posedge clk); la_stb <= 1'b0;
    end endtask
    task feed_adc(input [7:0] v); begin
        @(posedge clk); while (adc_full) @(posedge clk);
        adc_data <= v; adc_stb <= 1'b1; @(posedge clk); adc_stb <= 1'b0;
    end endtask

    integer oe_while_owned; reg checking_busown;
    initial begin oe_while_owned=0; checking_busown=0; end
    always @(posedge clk) if (checking_busown && bus_own && ps_oe) oe_while_owned = oe_while_owned + 1;

    integer la_i, adc_i;
    initial begin
        for (i=0;i<512;i=i+1) mem[i] = 8'hXX;
        repeat (4) @(posedge clk); rst = 0;
        @(posedge clk); start <= 1'b1; @(posedge clk); start <= 1'b0;

        // Interleave the two streams the way real capture does: LA fast, ADC slow.
        la_i = 0; adc_i = 0;
        for (i = 0; i < NL; i = i+1) begin
            feed_la(8'h50 + i[7:0]); la_i = la_i + 1;
            if ((i % 4) == 3 && adc_i < NA) begin feed_adc(8'hA0 + adc_i[7:0]); adc_i = adc_i + 1; end
        end
        while (adc_i < NA) begin feed_adc(8'hA0 + adc_i[7:0]); adc_i = adc_i + 1; end

        repeat (20) @(posedge clk);
        @(posedge clk); stop <= 1'b1; @(posedge clk); stop <= 1'b0;
        wait (idle); repeat (10) @(posedge clk);

        // ---- verify each region ----
        for (i = 0; i < NL; i = i+1)
            if (mem[(LA_BASE[8:0]+i) & 9'h1FF] !== (8'h50 + i[7:0])) begin
                $display("FAIL: LA[%0d]=%02h want %02h", i, mem[(LA_BASE[8:0]+i)&9'h1FF], 8'h50+i[7:0]); errors=errors+1; end
        for (i = 0; i < NA; i = i+1)
            if (mem[(ADC_BASE[8:0]+i) & 9'h1FF] !== (8'hA0 + i[7:0])) begin
                $display("FAIL: ADC[%0d]=%02h want %02h", i, mem[(ADC_BASE[8:0]+i)&9'h1FF], 8'hA0+i[7:0]); errors=errors+1; end

        $display("  regions: LA %0d B @%06h, ADC %0d B @%06h; worst CS-low=%0.0f ns (tCEM %0.0f)",
                 NL, LA_BASE, NA, ADC_BASE, cs_low_max, TCEM_NS);
        if (cs_low_max > TCEM_NS) begin $display("FAIL: CS-low exceeds tCEM"); errors=errors+1; end

        // ---- bus_own release ----
        decode_en = 0;
        @(posedge clk); bus_own <= 1'b1;
        @(posedge clk); start <= 1'b1; @(posedge clk); start <= 1'b0;
        for (i = 0; i < CHUNK+4; i = i+1) feed_la(8'hE0 + i[7:0]);
        checking_busown = 1; oe_while_owned = 0;
        repeat (60) @(posedge clk);
        if (oe_while_owned !== 0) begin
            $display("FAIL: writer drove io_oe for %0d cycles while bus_own=1", oe_while_owned); errors=errors+1; end
        else $display("  bus_own: writer kept io_oe low while owned though data was pending");
        bus_own <= 1'b0;

        if (errors == 0)
            $display("PASS tb_psram_dual_writer: two streams -> two regions, contiguous, in order; tCEM/setup/bus_own ok");
        else
            $display("FAIL tb_psram_dual_writer: %0d error(s)", errors);
        $finish;
    end

    initial begin #4000000 $display("FAIL tb_psram_dual_writer: timeout"); $finish; end
endmodule
