// ============================================================================
// tb_top_lachange.v — the v50 LA change capture through the whole top: SPI OP_CAPTURE with
// la_div = 0xFFFF -> cmd_dispatch -> la48_sampler + la_change_capture -> spram_ring48 ->
// psram_dual_writer -> the LA PSRAM region, decoded back from the QPI writes.
//
// Checks: CAP_DONE without CAP_OVF; the decoded stream covers exactly la_cnt * 256 ticks and
// ends with END; every decoded tick equals the pins at one fixed latency (so 48 MS/s really
// samples every clk48 edge, in order).  Then a raw LA capture (la_div = period - 2) right after,
// to show the mode goes back and the raw bytes still arrive 2 per sample through the new ring.
//
// Run:  make toplachangetest   (both images)
// ============================================================================
`timescale 1ns/1ps
module tb_top_lachange;
    reg clk48 = 1'b0;
    always #10 clk48 = ~clk48;

    reg  sck = 1'b0, mosi = 1'b0, csn = 1'b1;
    reg  bus_own = 1'b1;
    reg  adc_sdo = 1'b1;
    wire miso, busy;
    wire adc_cnvst, adc_sclk, adc_sdi, dac_sync, dac_sclk, dac_din;
    wire led_g, led_b, led_r;
    wire psram_sclk, psram_cs, psram_io0, psram_io1, psram_io2, psram_io3;
    reg  [13:0] la_drv = 14'h0155;
    wire [13:0] la;
    assign la = la_drv;

    top dut (
        .clk48(clk48),
        .sck(sck), .mosi(mosi), .miso(miso), .csn(csn), .busy(busy),
        .bus_own(bus_own),
        .adc_cnvst(adc_cnvst), .adc_sclk(adc_sclk), .adc_sdi(adc_sdi), .adc_sdo(adc_sdo),
        .dac_sync(dac_sync), .dac_sclk(dac_sclk), .dac_din(dac_din),
        .psram_sclk(psram_sclk), .psram_cs(psram_cs),
        .psram_io0(psram_io0), .psram_io1(psram_io1),
        .psram_io2(psram_io2), .psram_io3(psram_io3),
        .la(la),
        .led_g(led_g), .led_b(led_b), .led_r(led_r)
    );

    localparam SPI_HALF = 200;
    integer errors = 0;
    task spi_byte(input [7:0] tx, output [7:0] rx);
        integer b; begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b]; #(SPI_HALF);
                sck = 1'b1; rx[b] = miso; #(SPI_HALF);
                sck = 1'b0;
            end
        end
    endtask
    task cmd1(input [7:0] op, output [7:0] reply);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(op, junk); spi_byte(8'h00, reply);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask
    // OP_CAPTURE, LA only, la_div sent as is (0xFFFF = change capture, else period - 2)
    task cmd_la(input [23:0] la_cnt, input [15:0] la_div);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h31, junk);
            spi_byte(8'h00, junk); spi_byte(8'h00, junk); spi_byte(8'h00, junk);
            spi_byte(8'h28, junk); spi_byte(8'h00, junk);
            spi_byte(la_cnt[7:0], junk); spi_byte(la_cnt[15:8], junk); spi_byte(la_cnt[23:16], junk);
            spi_byte(la_div[7:0], junk); spi_byte(la_div[15:8], junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask
    task wait_done(input integer max_polls, input want_ovf);
        reg [7:0] st; integer k; begin
            k = 0; st = 8'h00;
            while (!(st[2]) && k < max_polls) begin cmd1(8'h03, st); k = k + 1; end
            if (!st[2]) begin $display("FAIL: CAP_DONE never set (STATUS=0x%02x)", st); errors = errors + 1; end
            else if (st[5] !== want_ovf) begin $display("FAIL: CAP_OVF %b, want %b (STATUS=0x%02x)", st[5], want_ovf, st); errors = errors + 1; end
        end
    endtask

    // ---- PSRAM QPI write-slave: LA region bytes by offset ----
    wire [3:0] ps_io = {psram_io3, psram_io2, psram_io1, psram_io0};
    localparam MEM = 1 << 16;
    reg [7:0]  la_mem [0:MEM-1];
    integer    la_n = 0, la_hi = 0, nib_i = 0;
    realtime   t_first = 0, t_last = 0;           // first / last LA byte written (drain rate)
    reg [7:0]  d_cmd, d_byte;
    reg [23:0] d_addr, wr_ptr;
    always @(negedge psram_cs) nib_i = 0;
    always @(posedge psram_sclk) if (psram_cs === 1'b0) begin
        case (nib_i)
            0: d_cmd[7:4] = ps_io;      1: d_cmd[3:0] = ps_io;
            2: d_addr[23:20] = ps_io;   3: d_addr[19:16] = ps_io;
            4: d_addr[15:12] = ps_io;   5: d_addr[11:8]  = ps_io;
            6: d_addr[7:4]   = ps_io;
            7: begin d_addr[3:0] = ps_io; wr_ptr = d_addr; end
            default: if (((nib_i - 8) & 1) == 0) d_byte[7:4] = ps_io;
                     else begin
                         d_byte[3:0] = ps_io;
                         if (wr_ptr < MEM) begin
                             la_mem[wr_ptr] = d_byte; la_n = la_n + 1;
                             if (la_n == 1) t_first = $realtime;
                             t_last = $realtime;
                             if (wr_ptr + 1 > la_hi) la_hi = wr_ptr + 1;
                         end
                         wr_ptr = wr_ptr + 24'd1;
                     end
        endcase
        nib_i = nib_i + 1;
    end

    // ---- pins at every clk48 edge, and the patterns ----
    localparam MAXT = 1 << 17;
    reg [13:0] pin [0:MAXT-1];
    integer    npin = 0, pin0 = 0;
    always @(posedge clk48) begin if (npin < MAXT) pin[npin] = la; npin = npin + 1; end
    always @(posedge dut.clk) if (dut.la_cap_start) pin0 = npin;

    integer pat = 0, k = 0;
    always @(negedge clk48) begin
        k = k + 1;
        case (pat)
            1: begin
                la_drv[0] <= ~la_drv[0];                       // every tick: 24 MHz square wave
                if (k % 5 == 0) la_drv[13:8] <= la_drv[13:8] + 6'd1;
                if (k == 600) la_drv[3] <= ~la_drv[3];
            end
            default: ;
        endcase
    end

    // ---- decode (format in la_change_capture.v) ----
    reg [13:0] dec [0:MAXT-1];
    integer    ndec, end_seen, nev;
    task decode;
        integer i, t, d; reg [23:0] e; reg [13:0] st; begin
            ndec = 0; end_seen = 0; t = -1; st = 0; nev = 0;
            for (i = 0; i + 2 < la_hi && !end_seen; i = i + 3) begin
                e = {la_mem[i+2], la_mem[i+1], la_mem[i]};
                d = e[23:14]; nev = nev + 1;
                if (i == 0) begin
                    if (d != 0) begin $display("FAIL first event delta %0d, want 0", d); errors = errors + 1; end
                    t = 0; st = e[13:0];
                end else if (d == 0) end_seen = 1;
                else begin
                    while (ndec < t + d) begin dec[ndec] = st; ndec = ndec + 1; end
                    t = t + d; st = e[13:0];
                end
            end
            if (t >= 0) begin dec[ndec] = st; ndec = ndec + 1; end
        end
    endtask

    reg [7:0] r;
    integer i, o, bad, best, cnt;
    initial begin
        #5000;
        cmd1(8'h01, r); if (r !== 8'hA5) begin $display("FAIL: PING=%02h", r); errors = errors + 1; end

        // ---- change capture: 6 x 256 ticks with a 24 MHz square wave on LA1 ----
        cnt = 6;
        bus_own = 1'b0; #1000;
        pat = 1;
        cmd_la(cnt, 16'hFFFF);
        wait_done(4000, 1'b0);
        pat = 0;
        #4000; bus_own = 1'b1;
        decode;
        if (!end_seen) begin $display("FAIL change: no END event"); errors = errors + 1; end
        if (ndec != cnt * 256) begin $display("FAIL change: decoded %0d ticks, want %0d", ndec, cnt * 256); errors = errors + 1; end
        best = -99;
        for (o = -8; o < 16 && best == -99; o = o + 1) begin
            bad = 0;
            for (i = 0; i < ndec && bad == 0; i = i + 1)
                if (pin0 + o + i < 0 || dec[i] !== pin[pin0 + o + i]) bad = 1;
            if (bad == 0) best = o;
        end
        if (best == -99) begin $display("FAIL change: decoded ticks match the pins at no fixed latency"); errors = errors + 1; end
        $display("  [change] %0d bytes in PSRAM, %0d events, %0d ticks, pin latency %0d", la_hi, nev, ndec, best);
        // The burst outruns the writer for the whole run, so this is the PSRAM drain rate: the
        // sustained change-capture limit is this / 3 bytes per edge.
        $display("  [change] PSRAM drain %0.2f MB/s = %0.2f M events/s sustained",
                 la_n / (t_last - t_first) * 1000.0, la_n / (t_last - t_first) * 1000.0 / 3.0);
        if (la_n / (t_last - t_first) * 1000.0 < 15.0) begin
            $display("FAIL change: PSRAM drain below 15 MB/s"); errors = errors + 1; end

        // ---- raw capture right after: steady pins, 10 samples at period 6 ----
        la_drv = 14'h2A5A; la_hi = 0; la_n = 0;
        bus_own = 1'b0; #1000;
        cmd_la(24'd10, 16'd4);
        wait_done(4000, 1'b0);
        #4000; bus_own = 1'b1;
        if (la_n != 20) begin $display("FAIL raw: %0d LA bytes, want 20 (no tail padding)", la_n); errors = errors + 1; end
        for (i = 0; i < 10; i = i + 1)
            if ({la_mem[2*i+1], la_mem[2*i]} !== 16'h2A5A) begin
                $display("FAIL raw: sample %0d = %02h%02h, want 2a5a", i, la_mem[2*i+1], la_mem[2*i]); errors = errors + 1;
            end
        $display("  [raw]    %0d bytes written (10 samples)", la_n);

        if (errors == 0) $display("PASS tb_top_lachange");
        else             $display("FAIL tb_top_lachange: %0d errors", errors);
        $finish;
    end
    initial begin #20000000 $display("FAIL tb_top_lachange: timeout"); $finish; end
endmodule
