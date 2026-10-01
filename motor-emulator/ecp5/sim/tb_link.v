// tb_link — the BenchPod link (src/spi_link.v) with the log FIFO (src/logbuf.v) behind it, wired
// as in top.v: two boards (ids 2 and 1) on one SCK / MOSI / MISO / SS, a register file with the
// two-clk read pipeline, the port registers (0x69, 0x78, the 0xC0-0xFF FIFO window).
//  1  single write and read, board addressing: the other board neither writes nor drives MISO
//  2  burst write and burst read: the address increments
//  3  broadcast write reaches both boards; a broadcast read is ignored (nobody drives MISO)
//  4  a port register takes a whole write burst: the registers after it are untouched
//  5  log FIFO: a burst longer than the 64-address window, in order; a burst cut in the middle
//     of a word loses nothing and repeats nothing; an empty FIFO reads 0 and stays put
//  6  a full FIFO drops whole sets and counts them; clear empties it
//  7  a transaction for another board, then one for this board straight after
// Also run on the gate-level netlists (tb_link_gl).
`timescale 1ns/1ps
`default_nettype none
module tb_link;
    reg clk = 0;
    always #13.889 clk = ~clk;              // 36 MHz
    reg rst = 1;
    reg sck = 0, mosi = 0, ss = 1;

    // ---- board 2: register file + log FIFO, as top.v wires them
    wire [7:0]  addr, rd_addr;
    wire [15:0] wr_data, log_head, log_level, log_drops;
    wire        wr_en, rd_take, rd_done, cs_end, miso2, oe2;
    reg  [15:0] rdata_q, rdata_p;
    reg  [15:0] regs [0:255];
    reg  [15:0] tbl [0:63];
    integer     tbl_n = 0, shape_n = 0;
    wire hold = (addr == 8'h69) || (addr == 8'h78) || (addr[7:6] == 2'b11);
    spi_link link2 (.clk(clk), .rst(rst), .board_id(2'd2), .sck(sck), .mosi(mosi), .csn(ss),
        .miso(miso2), .miso_oe(oe2), .addr(addr), .rd_data(rdata_q), .addr_hold(hold), .wr_en(wr_en),
        .wr_data(wr_data), .rd_take(rd_take), .rd_addr(rd_addr), .rd_done(rd_done), .cs_end(cs_end));
    integer k;
    initial for (k = 0; k < 256; k = k + 1) regs[k] = 16'h0000;
    always @(posedge clk) begin
        if (wr_en) begin
            if (addr == 8'h69) begin tbl[tbl_n & 63] <= wr_data; tbl_n = tbl_n + 1; end
            else if (addr == 8'h78) shape_n = shape_n + 1;
            else regs[addr] <= wr_data;
        end
        // two-stage read mux, like top.v: live values (the FIFO head, level, drops), else the mirror
        rdata_p <= (addr >= 8'hC0) ? log_head : (addr == 8'h7A) ? log_level : (addr == 8'h7B) ? log_drops : regs[addr];
        rdata_q <= rdata_p;
    end

    reg        log_clear = 0, log_set = 0, log_win = 0;
    reg [15:0] log_d0 = 0;
    always @(posedge clk) if (rd_take) log_win <= (rd_addr >= 8'hC0);
    logbuf log (.clk(clk), .rst(rst), .clear(log_clear), .en(1'b1), .mask(5'b00001), .seq_en(1'b1),
        .set(log_set), .d0(log_d0), .d1(16'd0), .d2(16'd0), .d3(16'd0), .d4(16'd0),
        .take(rd_take && rd_addr >= 8'hC0), .commit(rd_done && log_win), .abort(cs_end),
        .head(log_head), .level(log_level), .drops(log_drops));

    // ---- board 1: only its strobes matter
    wire [7:0]  addr1, rd_addr1;
    wire [15:0] wr_data1;
    wire        wr_en1, rd_take1, rd_done1, cs_end1, miso1, oe1;
    spi_link link1 (.clk(clk), .rst(rst), .board_id(2'd1), .sck(sck), .mosi(mosi), .csn(ss),
        .miso(miso1), .miso_oe(oe1), .addr(addr1), .rd_data(16'hB1B1), .addr_hold(1'b0), .wr_en(wr_en1),
        .wr_data(wr_data1), .rd_take(rd_take1), .rd_addr(rd_addr1), .rd_done(rd_done1), .cs_end(cs_end1));
    integer wr1 = 0, oe1_clk = 0, oe2_clk = 0, both_clk = 0;
    reg [15:0] last1 = 0;
    reg [7:0]  last1_addr = 0;
    always @(posedge clk) begin
        if (wr_en1) begin wr1 = wr1 + 1; last1 <= wr_data1; last1_addr <= addr1; end
        if (oe1 === 1'b1) oe1_clk = oe1_clk + 1;
        if (oe2 === 1'b1) oe2_clk = oe2_clk + 1;
        if (oe1 === 1'b1 && oe2 === 1'b1) both_clk = both_clk + 1;
    end
    wire miso = oe2 ? miso2 : oe1 ? miso1 : 1'bz;

    // ---- SPI master, mode 0, 6 MHz
    localparam HALF = 83;
    reg [7:0] rx;
    integer z_bits = 0;
    task spi_byte(input [7:0] tx);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b]; #HALF; sck = 1;
                if (miso === 1'bz) z_bits = z_bits + 1;
                rx[b] = miso; #HALF; sck = 0;
            end
        end
    endtask
    task reg_write(input bcast, input [1:0] id, input [7:0] a, input [15:0] d);
        begin
            ss = 0; #200;
            spi_byte({1'b0, bcast, id, 4'h0}); spi_byte(a); spi_byte(d[15:8]); spi_byte(d[7:0]);
            #200; ss = 1; #400;
        end
    endtask
    // burst write of n words d0, d0 + 1, ...
    task reg_wburst(input [1:0] id, input [7:0] a, input integer n, input [15:0] d0);
        integer w;
        reg [15:0] d;
        begin
            ss = 0; #200;
            spi_byte({1'b0, 1'b0, id, 4'h0}); spi_byte(a);
            for (w = 0; w < n; w = w + 1) begin d = d0 + w; spi_byte(d[15:8]); spi_byte(d[7:0]); end
            #200; ss = 1; #400;
        end
    endtask
    reg [15:0] burst [0:255];
    // burst read of n words; half = 1: one more byte (the MSB of word n), then SS goes high
    task reg_burst(input bcast, input [1:0] id, input [7:0] a, input integer n, input half);
        integer w;
        begin
            ss = 0; #200;
            spi_byte({1'b1, bcast, id, 4'h0}); spi_byte(a); spi_byte(8'h00);
            for (w = 0; w < n; w = w + 1) begin
                spi_byte(8'h00); burst[w][15:8] = rx; spi_byte(8'h00); burst[w][7:0] = rx;
            end
            if (half) spi_byte(8'h00);
            #200; ss = 1; #400;
        end
    endtask
    // n sample sets into the FIFO (each: a sequence word, then 0x1000 + its number)
    integer set_no = 0;
    task push_sets(input integer n);
        integer s;
        begin
            for (s = 0; s < n; s = s + 1) begin
                @(negedge clk) begin log_set = 1; log_d0 = 16'h1000 + set_no[15:0]; end
                @(negedge clk) log_set = 0;
                set_no = set_no + 1;
                repeat (4) @(negedge clk);
            end
        end
    endtask

    integer errors = 0, i, expect_seq, bad, prev_n;
    task check(input [8*56-1:0] what, input okx);
        begin
            if (!okx) begin errors = errors + 1; $display("FAIL: %0s", what); end
        end
    endtask
    // words[0 .. n-1] are whole sets (seq, 0x1000 + seq) continuing at expect_seq
    task check_sets(input integer n);
        integer w;
        begin
            for (w = 0; w < n; w = w + 2) begin
                if (burst[w] !== expect_seq[15:0] || burst[w + 1] !== 16'h1000 + expect_seq[15:0]) bad = bad + 1;
                expect_seq = expect_seq + 1;
            end
        end
    endtask

    initial begin
        #400_000_000 $display("FAIL tb_link: timeout"); $finish;
    end
    initial begin
        repeat (6) @(posedge clk); rst = 0; #500;

        // ---------------- 1: single write / read, board addressing
        reg_write(1'b0, 2'd2, 8'h0A, 16'h4000);
        check("write to board 2", regs[8'h0A] === 16'h4000 && wr1 == 0);
        z_bits = 0; prev_n = oe1_clk;
        reg_burst(1'b0, 2'd2, 8'h0A, 1, 1'b0);
        check("read back", burst[0] === 16'h4000);
        check("board 1 stays off MISO", oe1_clk == prev_n);
        // the command byte is not driven (the board is not yet known), the rest is
        check("MISO floats only during the command byte", z_bits == 8);
        reg_write(1'b0, 2'd1, 8'h0A, 16'h1111);
        check("write to board 1 only", regs[8'h0A] === 16'h4000 && wr1 == 1 && last1 === 16'h1111 && last1_addr === 8'h0A);
        reg_burst(1'b0, 2'd1, 8'h0A, 1, 1'b0);
        check("board 1 answers its own read", burst[0] === 16'hB1B1);
        z_bits = 0;
        reg_burst(1'b0, 2'd3, 8'h0A, 1, 1'b0);
        check("an absent board: MISO floats throughout", z_bits == 40);
        $display("  ok addressing");

        // ---------------- 2: bursts
        reg_wburst(2'd2, 8'h20, 4, 16'hA000);
        check("burst write", regs[8'h20] === 16'hA000 && regs[8'h21] === 16'hA001 && regs[8'h22] === 16'hA002 &&
              regs[8'h23] === 16'hA003 && regs[8'h24] === 16'h0000);
        reg_burst(1'b0, 2'd2, 8'h20, 5, 1'b0);
        check("burst read", burst[0] === 16'hA000 && burst[1] === 16'hA001 && burst[2] === 16'hA002 &&
              burst[3] === 16'hA003 && burst[4] === 16'h0000);
        $display("  ok bursts");

        // ---------------- 3: broadcast
        prev_n = wr1;
        reg_write(1'b1, 2'd0, 8'h07, 16'hBEEF);
        check("broadcast write reaches both", regs[8'h07] === 16'hBEEF && wr1 == prev_n + 1 && last1 === 16'hBEEF);
        z_bits = 0;
        reg_burst(1'b1, 2'd2, 8'h07, 1, 1'b0);
        check("broadcast read: nobody drives MISO", z_bits == 40);
        $display("  ok broadcast");

        // ---------------- 4: a port register takes the whole burst
        regs[8'h6A] = 16'h1234; regs[8'h6B] = 16'h5678;
        reg_wburst(2'd2, 8'h69, 64, 16'h2000);
        check("64 table words", tbl_n == 64 && tbl[0] === 16'h2000 && tbl[63] === 16'h203F);
        check("the registers after the port are untouched", regs[8'h6A] === 16'h1234 && regs[8'h6B] === 16'h5678);
        reg_wburst(2'd2, 8'h78, 300, 16'h0000);
        check("300 shape words", shape_n == 300 && regs[8'h79] === 16'h0000);
        reg_wburst(2'd2, 8'h68, 2, 16'h0005);          // 0x68 increments onto the port: one word each
        check("a burst running into a port stays there", regs[8'h68] === 16'h0005 && tbl_n == 65);
        $display("  ok port writes");

        // ---------------- 5: log FIFO
        push_sets(110);
        reg_burst(1'b0, 2'd2, 8'h7A, 2, 1'b0);
        check("LOG_LEVEL 220, no drops", burst[0] === 16'd220 && burst[1] === 16'd0);
        expect_seq = 0; bad = 0;
        reg_burst(1'b0, 2'd2, 8'hC0, 150, 1'b0);       // longer than the window
        check_sets(150);
        reg_burst(1'b0, 2'd2, 8'h7A, 1, 1'b0);
        check("70 words left", burst[0] === 16'd70);
        reg_burst(1'b0, 2'd2, 8'hE7, 3, 1'b1);         // anywhere in the window; cut inside word 4
        check("the cut burst", burst[0] === expect_seq[15:0] && burst[2] === expect_seq[15:0] + 16'd1);
        reg_burst(1'b0, 2'd2, 8'hC0, 1, 1'b0);         // the cut word again: the second set's sample
        check("the cut word comes again", burst[0] === 16'h1000 + expect_seq[15:0] + 16'd1);
        expect_seq = expect_seq + 2;
        reg_burst(1'b0, 2'd2, 8'hC0, 70, 1'b0);        // 66 left, then empty
        check_sets(66);
        check("an empty FIFO reads 0", burst[66] === 16'h0000 && burst[69] === 16'h0000);
        reg_burst(1'b0, 2'd2, 8'h7A, 1, 1'b0);
        check("empty", burst[0] === 16'd0);
        check("every set in order", bad == 0 && expect_seq == 110);
        $display("  ok log FIFO: %0d sets in order through a 150-word burst and a cut one", expect_seq);

        // ---------------- 6: full FIFO, drops, clear
        push_sets(1030);                               // 2048 words fit: 1024 sets
        reg_burst(1'b0, 2'd2, 8'h7A, 2, 1'b0);
        check("full: 2048 words, 6 sets dropped", burst[0] === 16'd2048 && burst[1] === 16'd6);
        reg_burst(1'b0, 2'd2, 8'hC0, 2, 1'b0);
        check("the oldest set is still first", burst[0] === 16'd110 && burst[1] === 16'h1000 + 16'd110);
        @(negedge clk) log_clear = 1; @(negedge clk) log_clear = 0;
        reg_burst(1'b0, 2'd2, 8'h7A, 2, 1'b0);
        check("clear empties it", burst[0] === 16'd0 && burst[1] === 16'd0);
        $display("  ok full FIFO");

        // ---------------- 7: back to back, another board first
        prev_n = wr1;
        ss = 0; #200; spi_byte(8'h10); spi_byte(8'h30); spi_byte(8'hAA); spi_byte(8'h55); #200; ss = 1; #170;
        ss = 0; #200; spi_byte(8'h20); spi_byte(8'h30); spi_byte(8'hC3); spi_byte(8'h3C); #200; ss = 1; #400;
        check("back-to-back transactions", wr1 == prev_n + 1 && last1 === 16'hAA55 && regs[8'h30] === 16'hC33C);
        // SS released in the middle of a write word: nothing is written
        ss = 0; #200; spi_byte(8'h20); spi_byte(8'h31); spi_byte(8'hFF); #200; ss = 1; #400;
        check("half a word writes nothing", regs[8'h31] === 16'h0000);
        check("the two boards never drive MISO together", both_clk == 0 && oe1_clk > 0 && oe2_clk > 0);
        $display("  ok back to back");

        if (errors == 0) $display("PASS tb_link");
        else $display("FAIL tb_link: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
