// ============================================================================
// tb_psram_tri_master.v — self-checking test of the UNIFIED tri-capture master:
// DAC deep replay (read) + ADC capture (write) + LA capture (write) all at once
// through ONE psram_tri_master and ONE shared set of pads (docs/tri-capture-unified-
// psram.md).  Reuses the combined behavioural APS6404L model from tb_psram_arbiter
// (one mem[], serves both 0x38 writes and 0xEB reads).
//
// Checks (3-way concurrent):
//   * INVARIANT: master output-enable and the model's read-drive never collide.
//   * DAC reads the preloaded waveform out in order across the loop wrap, ZERO
//     underruns at a DAC-representative pop cadence.
//   * ADC + LA bytes land in their runtime-based regions, in order, none dropped.
//
// Run:  make tritest   (needs yosys' cells_sim.v for SB_IO)
// ============================================================================
`timescale 1ns/1ps
module tb_psram_tri_master;
    localparam [8:0]  WCHUNK = 9'd16;
    localparam [8:0]  RCHUNK = 9'd16;
    localparam [3:0]  WAIT   = 4'd6;
    localparam        RD_FIFO_AW = 8;
    localparam        RDATA0 = 2 + 6 + WAIT;   // first read-data nibble index

    // runtime region map (tiny, disjoint, within the 1024-byte model)
    localparam [23:0] LA_BASE  = 24'h000000, LA_LEN  = 24'd96;   // 96 LA bytes
    localparam [23:0] ADC_BASE = 24'h000100, ADC_LEN = 24'd32;   // 32 ADC bytes
    localparam [23:0] DAC_BASE = 24'h000300, DAC_LEN = 24'd64;   // 32 samples, loops

    localparam integer POP_EVERY = 26;   // clk48 per DAC byte ~= 0.9 MS/s
    localparam integer LA_EVERY  = 12;   // clk per LA byte  ~= 1.0 MS/s
    localparam integer ADC_EVERY = 30;   // clk per ADC byte ~= 0.4 MS/s
    localparam integer NPOP = 200;       // > 3 loop wraps
    localparam integer NL   = LA_LEN;
    localparam integer NA   = ADC_LEN;

    reg clk48 = 0; always #10.4166 clk48 = ~clk48;
    reg clk = 0;   always @(posedge clk48) clk <= ~clk;
    reg rst = 1, rst48 = 1;

    // ---- DUT I/O ----
    reg         start = 0, dac_run = 0;
    reg  [7:0]  adc_data = 0, la_data = 0;
    reg         adc_stb = 0, la_stb = 0;
    wire        adc_full, la_full, dut_idle, dut_active;
    wire [7:0]  dac_data; wire dac_valid; reg dac_pop = 0;
    wire [3:0]  io_o; wire io_oe, cs, sclk_d0, sclk_d1;
    wire [3:0]  io_i;

    psram_tri_master #(.WCHUNK(WCHUNK), .RCHUNK(RCHUNK), .RWAIT(WAIT), .RD_FIFO_AW(RD_FIFO_AW)) dut (
        .clk(clk), .rst(rst), .clk48(clk48), .rst48(rst48),
        .start(start),
        .la_base(LA_BASE),  .la_len(LA_LEN),
        .adc_base(ADC_BASE),.adc_len(ADC_LEN),
        .dac_base(DAC_BASE),.dac_len(DAC_LEN), .dac_run(dac_run),
        .adc_data(adc_data), .adc_stb(adc_stb), .adc_full(adc_full),
        .la_data(la_data),   .la_stb(la_stb),   .la_full(la_full),
        .dac_data(dac_data), .dac_valid(dac_valid), .dac_pop(dac_pop),
        .idle(dut_idle),
        .psram_io_o(io_o), .psram_io_oe(io_oe), .psram_io_i(io_i),
        .psram_cs(cs), .psram_sclk_d0(sclk_d0), .psram_sclk_d1(sclk_d1),
        .active(dut_active)
    );

    // ---- pads (mirror top_v2): DDR SCLK on clk48, combinational IO with D_IN_0 back ----
    tri  psram_sclk, psram_cs_w; tri [3:0] psram_io;
    pulldown (psram_sclk); pulldown (psram_cs_w);
    pullup (psram_io[0]); pullup (psram_io[1]); pullup (psram_io[2]); pullup (psram_io[3]);

    SB_IO #(.PIN_TYPE(6'b100000), .PULLUP(1'b0)) io_sclk_i (
        .PACKAGE_PIN(psram_sclk), .OUTPUT_ENABLE(1'b1), .OUTPUT_CLK(clk48),
        .D_OUT_0(sclk_d0), .D_OUT_1(sclk_d1));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_cs_i (
        .PACKAGE_PIN(psram_cs_w), .OUTPUT_ENABLE(1'b1), .D_OUT_0(cs));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_d0 (
        .PACKAGE_PIN(psram_io[0]), .OUTPUT_ENABLE(io_oe), .D_OUT_0(io_o[0]), .D_IN_0(io_i[0]));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_d1 (
        .PACKAGE_PIN(psram_io[1]), .OUTPUT_ENABLE(io_oe), .D_OUT_0(io_o[1]), .D_IN_0(io_i[1]));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_d2 (
        .PACKAGE_PIN(psram_io[2]), .OUTPUT_ENABLE(io_oe), .D_OUT_0(io_o[2]), .D_IN_0(io_i[2]));
    SB_IO #(.PIN_TYPE(6'b101001), .PULLUP(1'b0)) io_d3 (
        .PACKAGE_PIN(psram_io[3]), .OUTPUT_ENABLE(io_oe), .D_OUT_0(io_o[3]), .D_IN_0(io_i[3]));

    wire       ps_cs   = psram_cs_w;
    wire       ps_sclk = psram_sclk;
    wire [3:0] ps_io   = psram_io;

    // ---- combined APS6404L model (from tb_psram_arbiter) ----
    reg  [7:0]  mem [0:1023];
    integer     mi;
    reg  [7:0]  d_cmd;
    reg  [23:0] m_addr;
    integer     mnib;
    reg  [3:0]  mdrive; reg mdriving;
    reg  [7:0]  mbyte;

    assign psram_io = mdriving ? mdrive : 4'bzzzz;

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
            if (((mnib-8) & 1) == 0) mbyte[7:4] = ps_io;
            else begin mbyte[3:0] = ps_io; mem[m_addr[9:0]] = mbyte; m_addr = m_addr + 24'd1; end
        end else if (d_cmd == 8'hEB && mnib >= RDATA0) begin
            if (((mnib-RDATA0) & 1) == 0) begin mbyte = mem[m_addr[9:0]]; mdrive <= mbyte[7:4]; end
            else begin mdrive <= mbyte[3:0]; m_addr = m_addr + 24'd1; end
            mdriving <= 1'b1;
        end else begin
            mdriving <= 1'b0;
        end
        mnib = mnib + 1;
    end

    // ---- invariants ----
    integer collide_err;
    initial collide_err = 0;
    always @(posedge clk) if (io_oe && mdriving) collide_err = collide_err + 1;

    // ==========================================================================
    // Stimulus
    // ==========================================================================
    integer i, errors, underruns, got_cnt;
    reg [7:0] got;
    reg done_rd, done_wr;

    // DAC consumer: pop at a fixed cadence; a pop with !dac_valid is an UNDERRUN.
    initial begin
        errors=0; underruns=0; got_cnt=0; done_rd=0;
        @(negedge rst48);
        @(negedge clk); dac_run = 1'b1;
        @(posedge clk48); while (!dac_valid) @(posedge clk48);
        for (i=0; i<NPOP; i=i+1) begin
            repeat (POP_EVERY) @(posedge clk48);
            if (!dac_valid) begin
                underruns = underruns + 1;
                while (!dac_valid) @(posedge clk48);
            end
            got = dac_data; dac_pop <= 1'b1; @(posedge clk48); dac_pop <= 1'b0;
            if (got !== mem[(DAC_BASE[9:0] + (i % DAC_LEN)) & 10'h3FF]) begin
                $display("FAIL: DAC byte[%0d] got %02h want %02h (pos %0d)",
                         i, got, mem[(DAC_BASE[9:0] + (i % DAC_LEN)) & 10'h3FF], i % DAC_LEN);
                errors = errors + 1;
            end
            got_cnt = got_cnt + 1;
        end
        done_rd = 1;
    end

    // capture producer: LA (fast) + ADC (slow) with backpressure.
    integer la_i, adc_i, tick;
    initial begin
        done_wr=0; la_i=0; adc_i=0; tick=0;
        @(negedge rst48);
        @(posedge clk); start <= 1'b1; @(posedge clk); start <= 1'b0;
        while (la_i < NL || adc_i < NA) begin
            @(posedge clk); tick = tick + 1;
            if ((tick % LA_EVERY)==0 && la_i<NL && !la_full) begin
                la_data <= 8'h50 + la_i[7:0]; la_stb <= 1'b1; la_i = la_i + 1;
            end else la_stb <= 1'b0;
            if ((tick % ADC_EVERY)==0 && adc_i<NA && !adc_full) begin
                adc_data <= 8'hA0 + adc_i[7:0]; adc_stb <= 1'b1; adc_i = adc_i + 1;
            end else adc_stb <= 1'b0;
        end
        @(posedge clk); la_stb <= 1'b0; adc_stb <= 1'b0;
        // wait for both staging FIFOs to actually drain to mem
        while (dut.a_cnt != 0 || dut.l_cnt != 0 || dut_active) @(posedge clk);
        repeat (40) @(posedge clk);
        done_wr = 1;
    end

    // ---- preload + supervise ----
    initial begin
        for (mi=0; mi<1024; mi=mi+1) mem[mi] = 8'hXX;
        for (mi=0; mi<DAC_LEN; mi=mi+1)
            mem[(DAC_BASE[9:0] + mi) & 10'h3FF] = (mi[7:0] * 8'd7) ^ 8'hC3;
        repeat (6) @(posedge clk48); rst = 0; rst48 = 0;

        wait (done_rd && done_wr);
        repeat (10) @(posedge clk);

        for (i=0; i<NL; i=i+1)
            if (mem[(LA_BASE[9:0]+i) & 10'h3FF] !== (8'h50 + i[7:0])) begin
                $display("FAIL: LA[%0d]=%02h want %02h", i, mem[(LA_BASE[9:0]+i)&10'h3FF], 8'h50+i[7:0]);
                errors = errors + 1;
            end
        for (i=0; i<NA; i=i+1)
            if (mem[(ADC_BASE[9:0]+i) & 10'h3FF] !== (8'hA0 + i[7:0])) begin
                $display("FAIL: ADC[%0d]=%02h want %02h", i, mem[(ADC_BASE[9:0]+i)&10'h3FF], 8'hA0+i[7:0]);
                errors = errors + 1;
            end

        if (collide_err != 0) begin
            $display("FAIL: master-drive vs model-drive collision for %0d cycles", collide_err);
            errors = errors + 1;
        end
        if (underruns != 0) begin
            $display("FAIL: %0d DAC underrun(s) at pop 1/%0d clk48", underruns, POP_EVERY);
            errors = errors + 1;
        end

        $display("  --- tri-capture: DAC replay + ADC + LA over ONE master ---");
        $display("  DAC %0d bytes (loop %0d), ADC %0d @%06h, LA %0d @%06h",
                 got_cnt, DAC_LEN, NA, ADC_BASE, NL, LA_BASE);
        if (errors == 0)
            $display("PASS tb_psram_tri_master: 3-way concurrent, runtime regions, no collisions, no underruns");
        else
            $display("FAIL tb_psram_tri_master: %0d error(s)", errors);
        $finish;
    end

    initial begin #12000000 $display("FAIL tb_psram_tri_master: timeout"); $finish; end
endmodule
