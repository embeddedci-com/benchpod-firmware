// ============================================================================
// tb_top_swdq.v — the SWD transfer queue (v45) end to end through top_v2.
//
// Host side: the real SPI link (SWD_ARM, SWD_QCONFIG, SWD_QFEED, SWD_QSTATUS, SWD_READ).
// Target side: a behavioural SW-DP on LA11 (SWCLK) / LA12 (SWDIO) with a pull-up, timed like
// the STM32 the bit-bang path flashes on hardware: it samples SWDIO on SWCLK rising edges and
// changes its own output TGT_DELAY after a rising edge.  Rising edges of a transfer, counted from
// the request's start bit:
//   1..8 request | 9 ACK0 out | 10, 11 ACK1, ACK2 out |
//   read:  12..43 data out, 44 parity out, 45 released
//   write: 12 released, 14..45 data sampled, 46 parity sampled
// Checked:
//   * writes land in the target's registers, reads come back LE in the reply buffer (both kinds
//     in one queue), with the SWDIO setup before every rising edge the target samples;
//   * nobody drives SWDIO while the target does (contention);
//   * a queue bigger than the link delivers at once (the engine waits for whole transfers);
//   * WAIT stops the queue with done = the transfers before it; the rest re-queued completes;
//   * a read parity error stops it with the parity flag; no target (ACK 111) stops at once;
//   * re-config (half 4, no idle cycles) and the bit-bang path afterwards.
//
// Run:  make swdqtest
// ============================================================================
`timescale 1ns/1ps
module tb_top_swdq;
    reg clk48 = 1'b0;
    always #10 clk48 = ~clk48;          // clk = 25 MHz in sim (40 ns)
    localparam CLK_NS    = 40;
    localparam TGT_DELAY = 25;          // target output delay after SWCLK rises

    reg  sck = 1'b0, mosi = 1'b0, csn = 1'b1;
    reg  bus_own = 1'b1, adc_sdo = 1'b1;
    wire miso, busy;
    wire adc_cnvst, adc_sclk, adc_sdi, dac_sync, dac_sclk, dac_din, led_g, led_b, led_r;
    wire psram_sclk, psram_cs, psram_io0, psram_io1, psram_io2, psram_io3;
    wire [13:0] la;
    genvar gi;
    generate for (gi = 0; gi < 14; gi = gi + 1) begin : g_pd
        if (gi == 11) begin : g_up pullup (la[gi]); end
        else begin : g_dn pulldown (la[gi]); end
    end endgenerate

    top dut (
        .clk48(clk48), .sck(sck), .mosi(mosi), .miso(miso), .csn(csn), .busy(busy),
        .bus_own(bus_own),
        .adc_cnvst(adc_cnvst), .adc_sclk(adc_sclk), .adc_sdi(adc_sdi), .adc_sdo(adc_sdo),
        .dac_sync(dac_sync), .dac_sclk(dac_sclk), .dac_din(dac_din),
        .psram_sclk(psram_sclk), .psram_cs(psram_cs),
        .psram_io0(psram_io0), .psram_io1(psram_io1), .psram_io2(psram_io2), .psram_io3(psram_io3),
        .la(la), .led_g(led_g), .led_b(led_b), .led_r(led_r)
    );

    integer errors = 0;

    // ------------------------------------------------------------------------
    // Behavioural SW-DP
    // ------------------------------------------------------------------------
    wire SWCLK = la[10];
    wire SWDIO = la[11];
    reg  t_oe = 1'b0, t_out = 1'b0;
    assign la[11] = t_oe ? t_out : 1'bz;
    wire host_oe = dut.engines_i.swd_i.dio_oe & dut.engines_i.swd_i.armed_r;

    reg        present = 1'b1;
    integer    wait_on = -1;            // answer WAIT to the transfer with this index
    integer    corrupt_on = -1;         // flip the parity of the read with this index
    integer    xfer_idx = 0;            // transfers answered since the last reset of the model
    reg [31:0] dp [0:3];
    reg [31:0] ap [0:3];
    integer    k;
    initial begin
        for (k = 0; k < 4; k = k + 1) begin dp[k] = 32'h0; ap[k] = 32'h0; end
        dp[0] = 32'h2BA01477;           // DPIDR
    end

    integer    edge_n = 0;              // rising edges into the current transfer (0 = idle)
    reg [7:0]  req;
    reg        rnw, apndp;
    reg [1:0]  a;
    reg [2:0]  ackv;
    reg [31:0] rdata, wdata;
    reg        wpar;
    realtime   t_host_dio = 0;

    always @(SWDIO) if (host_oe) t_host_dio = $realtime;

    task tdrive(input v); begin t_oe <= #(TGT_DELAY) 1'b1; t_out <= #(TGT_DELAY) v; end endtask
    task trelease;        begin t_oe <= #(TGT_DELAY) 1'b0; end endtask

    always @(posedge SWCLK) if (present) begin
        if (edge_n == 100) begin trelease; edge_n = 0; end       // after a non-OK ACK
        else if (edge_n == 0) begin
            if (host_oe && SWDIO === 1'b1) begin req = 8'h01; edge_n = 1; end   // start bit
        end else begin
            edge_n = edge_n + 1;
            if (edge_n <= 8) begin
                if ($realtime - t_host_dio < CLK_NS - 1) begin
                    $display("FAIL: request bit %0d set up only %0t ns before SWCLK rose", edge_n, $realtime - t_host_dio);
                    errors = errors + 1;
                end
                req[edge_n - 1] = SWDIO;
                if (edge_n == 8) begin
                    apndp = req[1]; rnw = req[2]; a = req[4:3];
                    if (req[5] !== (req[1] ^ req[2] ^ req[3] ^ req[4]) || req[6] !== 1'b0 || req[7] !== 1'b1) begin
                        $display("FAIL: malformed request %02x", req); errors = errors + 1; edge_n = 0;
                    end
                    ackv = (xfer_idx == wait_on) ? 3'b010 : 3'b001;
                end
            end else if (edge_n <= 11) begin
                tdrive(ackv[edge_n - 9]);                       // ACK0..2 out on rises 9..11
                if (edge_n == 11 && ackv != 3'b001) begin xfer_idx = xfer_idx + 1; edge_n = 100; end
            end else if (rnw) begin
                if (edge_n == 12) rdata = apndp ? ap[a] : dp[a];
                if (edge_n <= 43) tdrive(rdata[edge_n - 12]);
                else if (edge_n == 44) tdrive((^rdata) ^ (xfer_idx == corrupt_on));
                else begin trelease; xfer_idx = xfer_idx + 1; edge_n = 0; end   // 45
            end else begin
                if (edge_n == 12) trelease;
                else if (edge_n >= 14 && edge_n <= 45) begin
                    if ($realtime - t_host_dio < CLK_NS - 1) begin
                        $display("FAIL: write bit set up only %0t ns before SWCLK rose", $realtime - t_host_dio);
                        errors = errors + 1;
                    end
                    wdata[edge_n - 14] = SWDIO;
                end else if (edge_n == 46) begin
                    if (SWDIO !== ^wdata) begin $display("FAIL: write parity"); errors = errors + 1; end
                    if (apndp) ap[a] = wdata; else dp[a] = wdata;
                    xfer_idx = xfer_idx + 1; edge_n = 0;
                end
            end
        end
    end

    always @(posedge clk48) if (t_oe && host_oe) begin
        $display("FAIL: SWDIO contention at %0t", $realtime); errors = errors + 1;
    end

    // ------------------------------------------------------------------------
    // Host side (the STM32's link framing)
    // ------------------------------------------------------------------------
    localparam SPI_HALF = 250;
    task spi_byte(input [7:0] tx, output [7:0] rx);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b]; #(SPI_HALF); sck = 1'b1; rx[b] = miso; #(SPI_HALF); sck = 1'b0;
            end
        end
    endtask
    reg [7:0] junk, r, st_done, st_flags;
    task cs_lo; begin csn = 1'b0; #(SPI_HALF); end endtask
    task cs_hi; begin #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF); end endtask

    reg [7:0] q [0:511];
    integer   qn;
    reg [7:0] rb [0:511];

    function [7:0] reqb(input apd, input rd, input [1:0] ad);
        reqb = {1'b1, 1'b0, apd ^ rd ^ ad[0] ^ ad[1], ad[1], ad[0], rd, apd, 1'b1};
    endfunction
    task q_write(input apd, input [1:0] ad, input [31:0] v);
        begin q[qn] = reqb(apd, 1'b0, ad); q[qn+1] = v[7:0]; q[qn+2] = v[15:8];
              q[qn+3] = v[23:16]; q[qn+4] = v[31:24]; qn = qn + 5; end
    endtask
    task q_read(input apd, input [1:0] ad); begin q[qn] = reqb(apd, 1'b1, ad); qn = qn + 1; end endtask

    task qstatus;
        begin cs_lo; spi_byte(8'h5A, junk); spi_byte(8'h00, st_done); spi_byte(8'h00, st_flags); cs_hi; end
    endtask
    // queue q[0..qn), wait until the engine is idle, read `nread` words back
    task run_queue(input integer nread);
        integer j, polls; reg [15:0] n1;
        begin
            cs_lo; spi_byte(8'h58, junk); spi_byte(qn[7:0], junk); spi_byte(qn[15:8], junk);
            for (j = 0; j < qn; j = j + 1) spi_byte(q[j], junk);
            cs_hi;
            polls = 0; st_flags = 8'h01;
            while (st_flags[0] && polls < 4000) begin qstatus; polls = polls + 1; end
            if (st_flags[0]) begin $display("FAIL: queue never went idle"); errors = errors + 1; end
            n1 = 4 * nread + 1;                            // SWD_READ repeats byte 0: read one more
            if (nread > 0) begin
                cs_lo; spi_byte(8'h52, junk); spi_byte(n1[7:0], junk); spi_byte(n1[15:8], junk);
                spi_byte(8'h00, junk);
                for (j = 0; j < 4 * nread; j = j + 1) spi_byte(8'h00, rb[j]);
                cs_hi;
            end
            qn = 0;
        end
    endtask
    function [31:0] word(input integer i); word = {rb[4*i+3], rb[4*i+2], rb[4*i+1], rb[4*i]}; endfunction

    task check32(input [31:0] got, input [31:0] want, input [255:0] name);
        if (got !== want) begin $display("FAIL: %0s = %08x, want %08x", name, got, want); errors = errors + 1; end
    endtask
    task check8(input [7:0] got, input [7:0] want, input [255:0] name);
        if (got !== want) begin $display("FAIL: %0s = %02x, want %02x", name, got, want); errors = errors + 1; end
    endtask

    integer i;
    realtime t0;
    initial begin
        qn = 0;
        #5000;
        cs_lo; spi_byte(8'h02, junk); spi_byte(8'h00, r); cs_hi; check8(r, 8'd45, "VERSION");
        // SWD_ARM SWCLK=LA11 SWDIO=LA12, then half 2 (6 MHz at the real clk), 2 idle cycles
        cs_lo; spi_byte(8'h50, junk); spi_byte(8'd10, junk); spi_byte(8'd11, junk); spi_byte(8'hFF, junk); cs_hi;
        cs_lo; spi_byte(8'h59, junk); spi_byte(8'd2, junk); spi_byte(8'd2, junk); cs_hi;

        // ---- mixed reads and writes ----
        q_read(1'b0, 2'd0);                              // DPIDR
        q_write(1'b1, 2'd1, 32'h20000000);               // AP TAR
        q_write(1'b1, 2'd3, 32'hDEADBEEF);               // AP DRW
        q_read(1'b1, 2'd3);
        q_read(1'b1, 2'd1);
        xfer_idx = 0;
        run_queue(3);
        check8(st_done, 8'd13, "qptr (mixed: 3 reads + 2 writes)"); check8(st_flags, 8'h04, "flags (mixed: ACK OK, idle)");
        check32(word(0), 32'h2BA01477, "DPIDR");
        check32(word(1), 32'hDEADBEEF, "AP DRW");
        check32(word(2), 32'h20000000, "AP TAR");
        check32(ap[1], 32'h20000000, "target TAR"); check32(ap[3], 32'hDEADBEEF, "target DRW");
        $display("  ok?: mixed queue done=%0d flags=%02x", st_done, st_flags);

        // ---- 100 writes (500 bytes): the link is far slower than the engine ----
        for (i = 0; i < 100; i = i + 1) q_write(1'b1, 2'd3, 32'h1000 + i * 32'h01010101);
        t0 = $realtime;
        run_queue(0);
        check8(st_done, 8'd244, "qptr (100 writes = 500 bytes, low 8)"); check8(st_flags & 8'h40, 8'h40, "qptr bit 8"); check32(ap[3], 32'h1000 + 99 * 32'h01010101, "last write");
        $display("  ok?: 100 writes in %0t us", ($realtime - t0) / 1000000);

        // ---- WAIT on the third transfer: stops with done = 2, ACK 010; the rest completes ----
        xfer_idx = 0; wait_on = 2;
        q_write(1'b1, 2'd1, 32'h11111111); q_write(1'b1, 2'd2, 32'h22222222);
        q_write(1'b1, 2'd3, 32'h33333333); q_read(1'b1, 2'd3);
        run_queue(0);
        check8(st_done, 8'd11, "qptr (WAIT: 2 writes + the WAITed request byte)"); check8(st_flags, 8'h0A, "flags (WAIT: stopped, ACK 010)");
        check32(ap[3], 32'h1000 + 99 * 32'h01010101, "the WAITed write did not land");
        wait_on = -1;
        q_write(1'b1, 2'd3, 32'h33333333); q_read(1'b1, 2'd3);
        run_queue(1);
        check8(st_done, 8'd6, "qptr (re-queued)"); check32(word(0), 32'h33333333, "re-queued read");
        $display("  ok?: WAIT stop + re-queue");

        // ---- read parity error ----
        xfer_idx = 0; corrupt_on = 1;
        q_read(1'b0, 2'd0); q_read(1'b0, 2'd0); q_read(1'b0, 2'd0);
        run_queue(1);
        check8(st_done, 8'd2, "qptr (parity: read 1 + the failing read)"); check8(st_flags, 8'h26, "flags (parity: perr, ACK OK, stopped)");
        corrupt_on = -1;

        // ---- re-config: half 4, no idle cycles; then no target at all ----
        cs_lo; spi_byte(8'h59, junk); spi_byte(8'd4, junk); spi_byte(8'd0, junk); cs_hi;
        q_write(1'b0, 2'd2, 32'h000000F0); q_read(1'b0, 2'd2); q_read(1'b0, 2'd0);
        run_queue(2);
        check8(st_done, 8'd7, "qptr (half 4)"); check32(word(0), 32'h000000F0, "DP SELECT read back");
        check32(word(1), 32'h2BA01477, "DPIDR (half 4)");
        present = 1'b0;
        q_read(1'b0, 2'd0); q_read(1'b0, 2'd0);
        run_queue(0);
        check8(st_done, 8'd1, "qptr (no target: the request byte)"); check8(st_flags, 8'h1E, "flags (no target: ACK 111, stopped)");
        present = 1'b1;

        // ---- the bit-bang path still drives the pins ----
        cs_lo; spi_byte(8'h51, junk); spi_byte(8'd2, junk); spi_byte(8'd0, junk); spi_byte("O", junk); spi_byte("g", junk); cs_hi;
        #1000;
        if (la[10] !== 1'b1 || la[11] !== 1'b1) begin $display("FAIL: bit-bang 'g' after the queue"); errors = errors + 1; end

        if (errors == 0)
            $display("PASS tb_top_swdq: SWD queue writes/reads a SW-DP model at 6 and 3 MHz, stops on WAIT/parity/no target, re-queues, bit-bang still works");
        else
            $display("FAIL tb_top_swdq: %0d error(s)", errors);
        $finish;
    end
    initial begin #200000000 $display("FAIL tb_top_swdq: timeout"); $finish; end
endmodule
