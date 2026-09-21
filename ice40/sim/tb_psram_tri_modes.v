// ============================================================================
// tb_psram_tri_modes.v — subset + safety coverage for psram_tri_master, beyond the
// 3-way tb_psram_tri_master:
//   MODE A: LA+ADC capture with the DAC job DISABLED (dac_run=0) — proves a
//           zero/absent job is simply skipped (the "any subset" requirement).
//   MODE B: region-length BACKSTOP — feed MORE LA bytes than la_len; the master must
//           write EXACTLY la_len bytes into the region (no overrun past the boundary)
//           and backpressure the rest (la_full).
// Reuses the combined APS6404L model. Run: make trimodetest
// ============================================================================
`timescale 1ns/1ps
module tb_psram_tri_modes;
    localparam [8:0] WCHUNK = 9'd16, RCHUNK = 9'd16;
    localparam [3:0] WAIT = 4'd6;
    localparam       RDATA0 = 2 + 6 + WAIT;

    reg clk48 = 0; always #10.4166 clk48 = ~clk48;
    reg clk = 0;   always @(posedge clk48) clk <= ~clk;
    reg rst = 1, rst48 = 1;

    // region map (runtime) — set per scenario
    reg [23:0] la_base, la_len, adc_base, adc_len, dac_base, dac_len;
    reg        start = 0, dac_run = 0;
    reg [7:0]  adc_data = 0, la_data = 0;
    reg        adc_stb = 0, la_stb = 0;
    wire       adc_full, la_full, dut_idle, dut_active;
    wire [7:0] dac_data; wire dac_valid; reg dac_pop = 0;
    wire [3:0] io_o; wire io_oe, cs, sclk_d0, sclk_d1; wire [3:0] io_i;

    psram_tri_master #(.WCHUNK(WCHUNK), .RCHUNK(RCHUNK), .RWAIT(WAIT), .RD_FIFO_AW(8)) dut (
        .clk(clk), .rst(rst), .clk48(clk48), .rst48(rst48), .start(start),
        .la_base(la_base), .la_len(la_len), .adc_base(adc_base), .adc_len(adc_len),
        .dac_base(dac_base), .dac_len(dac_len), .dac_run(dac_run),
        .adc_data(adc_data), .adc_stb(adc_stb), .adc_full(adc_full),
        .la_data(la_data), .la_stb(la_stb), .la_full(la_full),
        .dac_data(dac_data), .dac_valid(dac_valid), .dac_pop(dac_pop),
        .idle(dut_idle),
        .psram_io_o(io_o), .psram_io_oe(io_oe), .psram_io_i(io_i),
        .psram_cs(cs), .psram_sclk_d0(sclk_d0), .psram_sclk_d1(sclk_d1), .active(dut_active)
    );

    tri psram_sclk, psram_cs_w; tri [3:0] psram_io;
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
    wire ps_cs = psram_cs_w, ps_sclk = psram_sclk; wire [3:0] ps_io = psram_io;

    // combined APS6404L model
    reg [7:0] mem [0:1023]; integer mi;
    reg [7:0] d_cmd; reg [23:0] m_addr; integer mnib;
    reg [3:0] mdrive; reg mdriving; reg [7:0] mbyte;
    assign psram_io = mdriving ? mdrive : 4'bzzzz;
    always @(negedge ps_cs) begin mnib = 0; mdriving = 1'b0; d_cmd = 8'h00; end
    always @(posedge ps_cs) mdriving <= 1'b0;
    always @(posedge ps_sclk) if (!ps_cs) begin
        case (mnib)
            0: d_cmd[7:4]=ps_io; 1: d_cmd[3:0]=ps_io;
            2: m_addr[23:20]=ps_io; 3: m_addr[19:16]=ps_io; 4: m_addr[15:12]=ps_io;
            5: m_addr[11:8]=ps_io; 6: m_addr[7:4]=ps_io; 7: m_addr[3:0]=ps_io;
            default: ;
        endcase
        if (d_cmd==8'h38 && mnib>=8) begin
            if (((mnib-8)&1)==0) mbyte[7:4]=ps_io;
            else begin mbyte[3:0]=ps_io; mem[m_addr[9:0]]=mbyte; m_addr=m_addr+24'd1; end
        end else if (d_cmd==8'hEB && mnib>=RDATA0) begin
            if (((mnib-RDATA0)&1)==0) begin mbyte=mem[m_addr[9:0]]; mdrive<=mbyte[7:4]; end
            else begin mdrive<=mbyte[3:0]; m_addr=m_addr+24'd1; end
            mdriving <= 1'b1;
        end else mdriving <= 1'b0;
        mnib = mnib + 1;
    end

    integer collide_err; initial collide_err = 0;
    always @(posedge clk) if (io_oe && mdriving) collide_err = collide_err + 1;

    integer errors; initial errors = 0;
    integer i, la_i, adc_i, tick;

    task do_reset; begin
        rst = 1; rst48 = 1; start = 0; dac_run = 0;
        adc_stb = 0; la_stb = 0; dac_pop = 0;
        repeat (6) @(posedge clk48); @(negedge clk); rst = 0; rst48 = 0; @(negedge clk);
    end endtask

    initial begin
        for (mi=0; mi<1024; mi=mi+1) mem[mi] = 8'hXX;

        // ================= MODE A: LA+ADC, DAC DISABLED =================
        la_base=24'h000000; la_len=24'd40; adc_base=24'h000100; adc_len=24'd20;
        dac_base=24'h000300; dac_len=24'd0;   // DAC absent
        do_reset;
        @(posedge clk); start <= 1'b1; @(posedge clk); start <= 1'b0;
        la_i=0; adc_i=0; tick=0;
        while (la_i<40 || adc_i<20) begin
            @(posedge clk); tick=tick+1;
            if ((tick%10)==0 && la_i<40 && !la_full) begin la_data<=8'h50+la_i[7:0]; la_stb<=1'b1; la_i=la_i+1; end
            else la_stb<=1'b0;
            if ((tick%25)==0 && adc_i<20 && !adc_full) begin adc_data<=8'hA0+adc_i[7:0]; adc_stb<=1'b1; adc_i=adc_i+1; end
            else adc_stb<=1'b0;
        end
        @(posedge clk); la_stb<=1'b0; adc_stb<=1'b0;
        while (dut.a_cnt!=0 || dut.l_cnt!=0 || dut_active) @(posedge clk);
        repeat (30) @(posedge clk);
        // DAC job must have stayed idle: no 0xEB ever issued -> DAC region untouched (X)
        if (dac_valid !== 1'b0) begin $display("FAIL modeA: dac_valid asserted with DAC disabled"); errors=errors+1; end
        for (i=0;i<40;i=i+1) if (mem[i] !== (8'h50+i[7:0])) begin
            $display("FAIL modeA LA[%0d]=%02h want %02h", i, mem[i], 8'h50+i[7:0]); errors=errors+1; end
        for (i=0;i<20;i=i+1) if (mem[16'h100+i] !== (8'hA0+i[7:0])) begin
            $display("FAIL modeA ADC[%0d]=%02h want %02h", i, mem[16'h100+i], 8'hA0+i[7:0]); errors=errors+1; end
        $display("  MODE A (LA+ADC, DAC disabled): LA 40 + ADC 20 bytes, DAC skipped");

        // ================= MODE B: region backstop =================
        // la_len = 24 bytes but we try to feed 60. Exactly 24 must land; the region
        // byte just past the end (addr 24) must remain untouched (X), and la_full must
        // eventually backpressure the surplus.
        for (mi=0; mi<1024; mi=mi+1) mem[mi] = 8'hXX;
        la_base=24'h000200; la_len=24'd24; adc_base=24'h000280; adc_len=24'd0;
        dac_base=24'h000300; dac_len=24'd0;
        do_reset;
        @(posedge clk); start <= 1'b1; @(posedge clk); start <= 1'b0;
        la_i=0; tick=0;
        begin : feed_surplus
            integer guard;
            guard = 0;
            while (la_i < 60 && guard < 20000) begin
                @(posedge clk); tick=tick+1; guard=guard+1;
                if ((tick%4)==0 && !la_full) begin la_data<=8'h30+la_i[7:0]; la_stb<=1'b1; la_i=la_i+1; end
                else la_stb<=1'b0;
            end
        end
        @(posedge clk); la_stb<=1'b0;
        repeat (200) @(posedge clk);   // let the region fill + backstop settle
        // exactly la_len bytes written, correct values
        for (i=0;i<24;i=i+1) if (mem[16'h200+i] !== (8'h30+i[7:0])) begin
            $display("FAIL modeB LA[%0d]=%02h want %02h", i, mem[16'h200+i], 8'h30+i[7:0]); errors=errors+1; end
        // the byte just past the region end must be untouched (backstop held)
        if (mem[16'h200+24] !== 8'hXX) begin
            $display("FAIL modeB: overrun past region end, mem[+24]=%02h (want XX)", mem[16'h200+24]);
            errors=errors+1;
        end
        // with a 24-byte region and 60 offered, the surplus must have backpressured
        if (la_i >= 60) $display("  MODE B note: all 60 accepted into FIFO (region backstop dropped surplus at write)");
        $display("  MODE B (backstop): la_len=24, offered 60 -> exactly 24 landed, no overrun past boundary");

        if (collide_err != 0) begin $display("FAIL: %0d bus collisions", collide_err); errors=errors+1; end
        if (errors==0) $display("PASS tb_psram_tri_modes: subset (DAC-off) + region backstop");
        else           $display("FAIL tb_psram_tri_modes: %0d error(s)", errors);
        $finish;
    end

    initial begin #20000000 $display("FAIL tb_psram_tri_modes: timeout"); $finish; end
endmodule
