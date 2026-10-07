// ============================================================================
// tb_top_deep.v — whole-top bench for the DEEP image (-DUSE_DEEP_REPLAY), v46.
//
// The deep replay path was only ever simulated as modules (tb_dac_psram_replay: reader +
// engine; tb_psram_arbiter: reader + writer + arbiter).  This bench drives the real top through
// its pads: SPI commands, an APS6404L model that serves the reader's 0xEB reads and the writer's
// 0x38 writes from one memory, an STM32 model that takes the bus (bus_own) and runs its own QPI
// reads, and a DAC8551 pin decoder.
//
//   1. CORR-1: a deep replay keeps its sample order while the STM32 takes the bus.  bus_own is
//      raised at every phase of a reader burst (a 2-clk48 sweep over the whole burst, including
//      the commit boundary), right after the DAC engine's low-byte pop (so a flush would land
//      between its two pops), and at random times, for short and long holds; long holds run
//      STM32 reads that make the PSRAM drive other data.  Every DAC frame must be the next
//      sample of the waveform: no byte swap, no restart at the base, no lost or repeated sample.
//      (v45 reset the reader on bus_own: the stream restarted at the base after every hold.)
//   2. Replay alongside a capture: an LA capture runs while the replay streams, then the STM32
//      reads the LA region back over the bus while the replay is still running.
//   3. CORR-2: an LA capture that overflows the SPRAM ring (the STM32 holds the bus) reports
//      CAP_OVF, and the next clean capture does not (v45: the ring's flag was sticky).
//
// Run:  make topdeeptest
// ============================================================================
`timescale 1ns/1ps
module tb_top_deep;
    reg clk48 = 1'b0;
    always #10.4166 clk48 = ~clk48;

    reg  sck = 1'b0, mosi = 1'b0, csn = 1'b1;
    reg  bus_own = 1'b1;            // the STM32 owns the bus at boot
    reg  adc_sdo = 1'b1;
    wire miso, busy;
    wire adc_cnvst, adc_sclk, adc_sdi, dac_sync, dac_sclk, dac_din;
    wire led_g, led_b, led_r;
    tri  psram_sclk, psram_cs, psram_io0, psram_io1, psram_io2, psram_io3;
    pulldown (psram_sclk);
    pullup (psram_cs); pullup (psram_io0); pullup (psram_io1); pullup (psram_io2); pullup (psram_io3);

    localparam [13:0] LA_WORD = 14'h2A5A;
    wire [13:0] la = LA_WORD;

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

    integer errors = 0;
    localparam [23:0] WBASE = 24'h030000;   // waveform
    localparam [23:0] DECOY = 24'h020000;   // what the STM32 reads (0xEE)
    localparam integer NS = 50;             // samples (100 bytes: 12 whole 8-byte bursts + 4)

    // ---------------------------------------------------------------- SPI ----
    localparam SPI_HALF = 200;
    task spi_byte(input [7:0] tx, output [7:0] rx);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b]; #(SPI_HALF);
                sck = 1'b1; rx[b] = miso; #(SPI_HALF);
                sck = 1'b0;
            end
        end
    endtask
    task cmd1(input [7:0] op, output [7:0] reply);
        reg [7:0] junk;
        begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(op, junk); spi_byte(8'h00, reply);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask
    task cmd_op0(input [7:0] op);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF); spi_byte(op, junk); #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask
    // OP_CAPTURE (0x31): adc_cnt(3) + adc_div(2) + la_cnt(3) + la_div(2) (LA: period - 2 on the wire)
    task cmd_capture(input [23:0] adc_cnt, input [15:0] adc_div, input [23:0] la_cnt, input [15:0] la_div_period);
        reg [7:0] junk; reg [15:0] la_div; begin
            la_div = (la_div_period > 2) ? la_div_period - 16'd2 : 16'd0;
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h31, junk);
            spi_byte(adc_cnt[7:0], junk); spi_byte(adc_cnt[15:8], junk); spi_byte(adc_cnt[23:16], junk);
            spi_byte(adc_div[7:0], junk); spi_byte(adc_div[15:8], junk);
            spi_byte(la_cnt[7:0],  junk); spi_byte(la_cnt[15:8],  junk); spi_byte(la_cnt[23:16], junk);
            spi_byte(la_div[7:0],  junk); spi_byte(la_div[15:8],  junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask
    // OP_START_DAC_PSRAM (0x13): base(3) + count(3, samples) + divider(2, period - 1)
    task cmd_start_dac_psram(input [23:0] base, input [23:0] count, input [15:0] div_period);
        reg [7:0] junk; reg [15:0] divider; begin
            divider = (div_period > 0) ? div_period - 16'd1 : 16'd0;
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h13, junk);
            spi_byte(base[7:0],   junk); spi_byte(base[15:8],  junk); spi_byte(base[23:16],  junk);
            spi_byte(count[7:0],  junk); spi_byte(count[15:8], junk); spi_byte(count[23:16], junk);
            spi_byte(divider[7:0],junk); spi_byte(divider[15:8], junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask
    // poll STATUS until CAP_DONE (bit 2); returns the final STATUS (bit 5 = CAP_OVF)
    task wait_done(input integer max_polls, output [7:0] st);
        integer k; begin
            st = 0; k = 0;
            while (!(st[2]) && k < max_polls) begin cmd1(8'h03, st); k = k + 1; #(2000); end
            if (!st[2]) begin $display("FAIL: CAP_DONE never set (STATUS=0x%02x)", st); errors = errors + 1; end
        end
    endtask

    // --------------------------------------------- APS6404L model (QPI) ----
    // One memory for 0x38 writes and 0xEB reads (6 wait cycles), whoever is the master.
    localparam integer MEM_AW = 18;
    reg  [7:0]  mem [0:(1<<MEM_AW)-1];
    wire [3:0]  ps_io = {psram_io3, psram_io2, psram_io1, psram_io0};
    reg  [7:0]  d_cmd, mbyte;
    reg  [23:0] m_addr;
    integer     mnib = 0;
    reg  [3:0]  mdrive = 4'h0;
    reg         mdriving = 1'b0;
    localparam  RDATA0 = 2 + 6 + 6;
    assign {psram_io3, psram_io2, psram_io1, psram_io0} = mdriving ? mdrive : 4'bzzzz;
    always @(negedge psram_cs) begin mnib = 0; mdriving = 1'b0; d_cmd = 8'h00; end
    always @(posedge psram_cs) mdriving <= 1'b0;
    always @(posedge psram_sclk) if (psram_cs === 1'b0) begin
        case (mnib)
            0: d_cmd[7:4]    = ps_io;   1: d_cmd[3:0]    = ps_io;
            2: m_addr[23:20] = ps_io;   3: m_addr[19:16] = ps_io;
            4: m_addr[15:12] = ps_io;   5: m_addr[11:8]  = ps_io;
            6: m_addr[7:4]   = ps_io;   7: m_addr[3:0]   = ps_io;
            default: ;
        endcase
        if (mnib == 1 && d_cmd !== 8'h38 && d_cmd !== 8'hEB) begin
            $display("FAIL: PSRAM saw command %02h at %0t", d_cmd, $time); errors = errors + 1;
        end
        if (d_cmd == 8'h38 && mnib >= 8) begin
            if (((mnib-8) & 1) == 0) mbyte[7:4] = ps_io;
            else begin mbyte[3:0] = ps_io; mem[m_addr[MEM_AW-1:0]] = mbyte; m_addr = m_addr + 24'd1; end
        end else if (d_cmd == 8'hEB && mnib >= RDATA0) begin
            if (((mnib-RDATA0) & 1) == 0) begin mbyte = mem[m_addr[MEM_AW-1:0]]; mdrive <= mbyte[7:4]; end
            else begin mdrive <= mbyte[3:0]; m_addr = m_addr + 24'd1; end
            mdriving <= 1'b1;
        end else mdriving <= 1'b0;
        mnib = mnib + 1;
    end
    integer collide = 0;
    always @(posedge clk48) if (dut.pads_i.io_oe && mdriving) collide = collide + 1;

    // ------------------------------------------------------ STM32 model ----
    // Drives the bus only inside a bus_own window (its own XSPI reads), released before bus_own falls.
    reg        stm_en = 1'b0, stm_cs = 1'b1, stm_sclk = 1'b0, stm_oe = 1'b0;
    reg  [3:0] stm_io = 4'h0;
    assign psram_cs   = stm_en ? stm_cs   : 1'bz;
    assign psram_sclk = stm_en ? stm_sclk : 1'bz;
    assign {psram_io3, psram_io2, psram_io1, psram_io0} = (stm_en && stm_oe) ? stm_io : 4'bzzzz;
    reg  [7:0] stm_buf [0:255];
    task stm_nib_out(input [3:0] n);
        begin stm_io = n; stm_oe = 1'b1; #20; stm_sclk = 1'b1; #40; stm_sclk = 1'b0; #20; end
    endtask
    task stm_read(input [23:0] addr, input integer n);   // 0xEB, 6 dummies, n bytes -> stm_buf
        integer k; reg [7:0] b; begin
            stm_cs = 1'b0; #30;
            stm_nib_out(4'hE); stm_nib_out(4'hB);
            for (k = 5; k >= 0; k = k - 1) stm_nib_out(addr[4*k +: 4]);
            stm_oe = 1'b0;
            for (k = 0; k < 6; k = k + 1) begin #20; stm_sclk = 1'b1; #40; stm_sclk = 1'b0; #20; end
            for (k = 0; k < 2*n; k = k + 1) begin
                #20; stm_sclk = 1'b1; #40; stm_sclk = 1'b0; #10;
                if (k[0] == 1'b0) b[7:4] = ps_io; else begin b[3:0] = ps_io; stm_buf[k/2] = b; end
                #10;
            end
            stm_cs = 1'b1; #50;
        end
    endtask
    // Take the bus for hold_ns; with `reads`, run STM32 reads of the decoy region meanwhile.
    task stm_take(input integer hold_ns, input integer reads);
        realtime t_end; begin
            t_end = $realtime + hold_ns;
            bus_own = 1'b1;
            #60; stm_en = 1'b1;
            while (reads && $realtime + 3000 < t_end) stm_read(DECOY, 16);
            while ($realtime + 60 < t_end) #10;
            stm_en = 1'b0; #60;
            bus_own = 1'b0;
        end
    endtask

    // ------------------------------------------------- DAC8551 decoder ----
    // Every frame must be the next waveform sample: sample k = {2k+1, 2k} (LE bytes 2k, 2k+1).
    integer dac_bits = 0, dac_in_frame = 0, nfr = 0, fr_bad = 0, expi = 0;
    reg [23:0] dac_sh = 24'd0;
    reg [15:0] want_fr;
    always @(negedge dac_sync) dac_bits = 0;
    always @(posedge dac_sclk) dac_in_frame = (dac_sync === 1'b0);
    always @(negedge dac_sclk) if (dac_in_frame) begin
        dac_sh = {dac_sh[22:0], dac_din};
        dac_bits = dac_bits + 1;
        if (dac_bits == 24) begin
            want_fr = {8'd2*expi[7:0] + 8'd1, 8'd2*expi[7:0]};
            if (dac_sh[15:0] !== want_fr) begin
                if (fr_bad < 6) $display("FAIL: DAC frame %0d = %04h, want sample %0d = %04h (t=%0t)",
                                          nfr, dac_sh[15:0], expi, want_fr, $time);
                fr_bad = fr_bad + 1;
                // resynchronise on the value if it is a valid sample, so one slip is one error
                if (dac_sh[0] == 1'b0 && dac_sh[8] == 1'b1 && dac_sh[15:8] == dac_sh[7:0] + 8'd1
                    && dac_sh[7:1] < NS) expi = dac_sh[7:1];
            end
            expi = (expi + 1) % NS;
            nfr = nfr + 1;
        end
    end
    // underrun stalls: the engine waits in S_PLO/S_PHI with an empty FIFO (allowed, counted)
    integer stall = 0;
    always @(posedge clk48) if ((dut.dac_i.st == 3'd5 || dut.dac_i.st == 3'd7) && !dut.rd_strm_valid) stall = stall + 1;

    // reader coverage: which burst states bus_own first landed in, and the bursts it cut
    reg  [6:0] hit = 7'd0;
    integer    aborts = 0, commits = 0;
    always @(posedge dut.clk) begin
        if (dut.rdr_i.run_i && dut.bus_own_synced && !dut.rdr_i.lost) hit[dut.rdr_i.st] = 1'b1;
        if (dut.rdr_i.cell_go && dut.rdr_i.st == 3'd6) begin
            if (dut.rdr_i.lost_now) aborts = aborts + 1; else commits = commits + 1;
        end
    end

    // ------------------------------------------------------- stimulus ----
    integer i, d, seed, la_bad, nev;
    reg [7:0] st;
    reg [15:0] w;
    initial begin
        seed = 46;
        for (i = 0; i < (1<<MEM_AW); i = i + 1) mem[i] = 8'h00;
        for (i = 0; i < 2*NS; i = i + 1) mem[WBASE[MEM_AW-1:0] + i] = i[7:0];
        for (i = 0; i < 64; i = i + 1)   mem[DECOY[MEM_AW-1:0] + i] = 8'hEE;

        #5000;
        cmd1(8'h01, st); if (st !== 8'hA5) begin $display("FAIL: PING=%02h", st); errors = errors + 1; end
        bus_own = 1'b0; #1000;

        // ==== 1. replay across bus_own ====
        expi = 0;
        cmd_start_dac_psram(WBASE, NS, 16'd20);
        wait (nfr >= 20);
        nev = 0;
        // (a) the whole reader burst, every 2 clk48 from its first command cell
        for (d = 0; d < 160; d = d + 2) begin
            @(posedge dut.clk); while (dut.rdr_i.st != 3'd1) @(posedge dut.clk);
            repeat (d) @(posedge clk48); #1;
            stm_take(600 + (d % 5) * 300, 0); nev = nev + 1;
            #(1500 + (d % 7) * 400);
        end
        // (b) right after the engine's low-byte pop (a reset here used to byte-swap the stream)
        for (d = 0; d < 24; d = d + 1) begin
            @(posedge clk48); while (!(dut.dac_i.st == 3'd6)) @(posedge clk48);
            repeat (d % 8) @(posedge clk48); #1;
            stm_take((d % 3 == 0) ? 9000 : 700, d % 3 == 0); nev = nev + 1;
            #(2000 + (d % 5) * 700);
        end
        // (c) random times and holds, some with STM32 reads
        for (d = 0; d < 30; d = d + 1) begin
            #($urandom(seed) % 5000);
            stm_take(500 + $urandom(seed) % 12000, $urandom(seed) % 2); nev = nev + 1;
        end
        #20000;
        $display("  [replay] %0d bus_own holds, %0d DAC frames, %0d clk48 of underrun stall", nev, nfr, stall);
        $display("  [replay] reader: %0d bursts committed, %0d cut by bus_own; bus_own seen in states %b (6..0)",
                 commits, aborts, hit);
        if (hit !== 7'h7F || aborts < 20) begin
            $display("FAIL: bus_own did not reach every reader state (want 1111111) or cut too few bursts"); errors = errors + 1;
        end
        if (nfr < 400) begin $display("FAIL: only %0d DAC frames during the bus_own run", nfr); errors = errors + 1; end

        // ==== 2. a capture alongside the replay, then the STM32 reads it back mid-replay ====
        for (i = 0; i < 256; i = i + 1) mem[i] = 8'h00;
        cmd_capture(24'd0, 16'd0, 24'd64, 16'd6);
        wait_done(400, st);
        if (st[5]) begin $display("FAIL: capture alongside the replay overflowed (STATUS=%02x)", st); errors = errors + 1; end
        #3000;
        bus_own = 1'b1; #60; stm_en = 1'b1;
        stm_read(24'h000000, 128);
        stm_en = 1'b0; #60; bus_own = 1'b0;
        la_bad = 0;
        for (i = 0; i < 64; i = i + 1) begin
            w = {stm_buf[2*i+1], stm_buf[2*i]};
            if (w !== {2'b00, LA_WORD}) begin
                if (la_bad < 4) $display("FAIL: LA sample %0d read back %04h, want %04h", i, w, {2'b00, LA_WORD});
                la_bad = la_bad + 1;
            end
        end
        if (la_bad) errors = errors + 1;
        else $display("  [capture] 64 LA samples captured alongside the replay, read back by the STM32 mid-replay");
        i = nfr; #60000;
        if (nfr < i + 30) begin $display("FAIL: the replay stalled after the read-back (%0d frames)", nfr - i); errors = errors + 1; end
        cmd_op0(8'h12);                          // STOP_DAC
        #5000;
        if (fr_bad) begin $display("FAIL: %0d of %0d DAC frames out of order", fr_bad, nfr); errors = errors + 1; end
        else $display("  [replay] all %0d DAC frames in order (no swap, restart, loss or repeat)", nfr);

        // ==== 3. CORR-2: an overflowed capture, then a clean one ====
        bus_own = 1'b1;                          // the writer cannot drain: the LA ring fills
        cmd_capture(24'd0, 16'd0, 24'd40000, 16'd2);
        while (dut.la_busy) #1000;
        if (!dut.la_ring_ovf) begin $display("FAIL: the LA ring did not overflow"); errors = errors + 1; end
        bus_own = 1'b0;
        wait_done(4000, st);
        if (!st[5]) begin $display("FAIL: an overflowed capture did not report CAP_OVF (STATUS=%02x)", st); errors = errors + 1; end
        cmd_capture(24'd0, 16'd0, 24'd64, 16'd6);
        wait_done(400, st);
        if (st[5]) begin $display("FAIL: a clean capture after an overflow reports CAP_OVF (STATUS=%02x)", st); errors = errors + 1; end
        else $display("  [ovf] overflowed capture: CAP_OVF set; next clean capture: CAP_OVF clear");

        if (collide) begin $display("FAIL: FPGA and PSRAM drove the data bus together on %0d clk48", collide); errors = errors + 1; end
        if (errors == 0) $display("PASS tb_top_deep: deep replay keeps its order across bus_own, replay + capture + read-back, overflow clears on arm");
        else $display("FAIL tb_top_deep: %0d error(s)", errors);
        $finish;
    end
    initial begin #40000000 $display("FAIL tb_top_deep: timeout"); $finish; end
endmodule
