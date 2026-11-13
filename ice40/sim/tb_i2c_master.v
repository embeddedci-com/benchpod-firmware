// tb_i2c_master.v: swd_engine's I2C master (v55) and SPI modes 0-3, unit level.
//
// I2C: a behavioural target at 0x48 (open drain, pull-ups on the bus) with a register pointer,
// auto-increment, read back, and optional clock stretching after every ACK.  A bus monitor counts
// any SDA change while SCL is high that is not an expected START/STOP.  Runs at the three
// half periods the firmware will use (13 = ~1 MHz, 33 = ~400 kHz, 120 = 100 kHz at 24 MHz).
// SPI: a slave per mode that samples MOSI and drives MISO on that mode's edges.
`timescale 1ns/1ps
module tb_i2c_master;
    reg clk = 0; always #20.833 clk = ~clk;   // 24 MHz
    reg rst = 1;

    reg        spi_arm_stb = 0, disarm_stb = 0, feed_begin = 0, feed_stb = 0;
    reg  [7:0] feed_byte = 0;
    reg  [6:0] half = 7'd13;
    reg  [2:0] flags = 3'b100;
    reg  [8:0] rd_addr = 0;
    wire [7:0] rd_data;
    wire [15:0] reply_count;
    wire [7:0] q_done, q_flags;
    wire armed, spi_mode, spi_busy;
    wire [3:0] clk_ch, dio_ch, in_ch, cs_ch;
    wire clk_val, clk_oe, dio_val, dio_oe, cs_val, cs_on;

    // ---- the bus ----
    reg  t_sda_low = 0, t_scl_low = 0;    // target pulls
    reg  x_sda_low = 0;                   // another master (arbitration test)
    wire m_scl_low = clk_oe & ~clk_val;
    wire m_sda_low = dio_oe & ~dio_val;
    wire scl = ~(m_scl_low | t_scl_low);
    wire sda = ~(m_sda_low | t_sda_low | x_sda_low);

    // SPI wires (same engine pins in SPI mode)
    reg  miso = 0;
    wire sck  = clk_val;
    wire mosi = dio_val;

    swd_engine #(.REPLY_AW(9)) dut (
        .clk(clk), .rst(rst),
        .arm_stb(1'b0), .arm_clk_ch(4'd0), .arm_dio_ch(4'd0),
        .spi_arm_stb(spi_arm_stb), .spi_sck_ch(4'd0), .spi_mosi_ch(4'd1), .spi_miso_ch(4'd1),
        .spi_cs_ch(4'd0), .spi_half(half), .spi_flags(flags), .spi_cs_stb(1'b0), .spi_cs_assert(1'b0),
        .disarm_stb(disarm_stb),
        .q_cfg_stb(1'b0), .q_cfg_half(6'd0), .q_cfg_idle(5'd0), .feed_q(1'b0),
        .feed_begin(feed_begin), .feed_stb(feed_stb), .feed_byte(feed_byte),
        .dio_in(flags[2] ? sda : miso), .scl_in(scl),
        .rd_addr(rd_addr), .rd_data(rd_data), .reply_count(reply_count),
        .armed(armed), .spi_mode(spi_mode), .spi_busy(spi_busy), .q_done(q_done), .q_flags(q_flags),
        .clk_ch(clk_ch), .clk_val(clk_val), .clk_oe(clk_oe),
        .dio_ch(dio_ch), .dio_val(dio_val), .dio_oe(dio_oe),
        .in_ch(in_ch), .cs_ch(cs_ch), .cs_val(cs_val), .cs_on(cs_on)
    );

    integer errors = 0;
    task fail(input [8*80-1:0] msg); begin $display("FAIL: %0s", msg); errors = errors + 1; end endtask

    // ---- behavioural I2C target ----
    reg [7:0] regs [0:15];
    reg [3:0] ptr;
    reg [7:0] shr, txb;
    integer   bitn;
    reg       active = 0, selected = 0, rw = 0, first = 0, expect_cond = 0, master_nack = 0;
    integer   stretch = 0;                // clk of SCL held low after each ACK
    integer   starts = 0, stops = 0, glitches = 0;
    reg       sda_q = 1, scl_q = 1;

    always @(posedge clk) begin
        sda_q <= sda; scl_q <= scl;
        // START / STOP: SDA moves while SCL is high
        if (scl && scl_q && sda_q && !sda) begin
            starts <= starts + 1; active <= 1; selected <= 0; bitn = 0; first <= 1; t_sda_low <= 0; master_nack = 0;
        end
        if (scl && scl_q && !sda_q && sda) begin
            stops <= stops + 1; active <= 0; selected <= 0; t_sda_low <= 0;
        end
    end

    // bits on SCL edges: a bit is taken on the rising edge; the target changes SDA after a fall
    reg was_addr = 0;
    always @(posedge scl) if (active) begin
        if (bitn < 8) begin
            if (!(selected && rw)) shr = {shr[6:0], sda};
        end else if (selected && rw) master_nack = sda;
        bitn = bitn + 1;
    end

    always @(negedge scl) if (active) begin
        if (bitn == 8) begin                                   // byte in: ACK or release
            was_addr = first;
            if (first) begin
                first = 0;
                if (shr[7:1] == 7'h48) begin selected = 1; rw = shr[0]; t_sda_low = 1; end
                else begin selected = 0; t_sda_low = 0; end
            end else if (selected && !rw) begin
                if (expect_cond) begin ptr = shr[3:0]; expect_cond = 0; end
                else begin regs[ptr] = shr; ptr = ptr + 1; end
                t_sda_low = 1;
            end else t_sda_low = 0;                            // read: the master ACKs
        end else if (bitn == 9) begin                          // end of the ACK bit
            bitn = 0;
            if (selected && rw) begin
                if (!was_addr && master_nack) begin t_sda_low = 0; selected = 0; end
                else begin
                    if (!was_addr) ptr = ptr + 1;
                    txb = regs[ptr]; t_sda_low = ~txb[7];
                end
            end else t_sda_low = 0;
            if (stretch > 0) begin t_scl_low = 1; repeat (stretch) @(posedge clk); t_scl_low = 0; end
        end else if (bitn >= 1 && bitn <= 7 && selected && rw && !first)
            t_sda_low = ~txb[7 - bitn];
    end

    // monitor: SDA must not move while SCL is high except at START / STOP (counted above)
    always @(posedge clk) if (scl && scl_q && (sda != sda_q)) glitches <= glitches + 1;

    // ---- host side ----
    task feed(input [7:0] b); begin
        @(posedge clk); feed_byte <= b; feed_stb <= 1; @(posedge clk); feed_stb <= 0;
    end endtask
    task begin_feed; begin @(posedge clk); feed_begin <= 1; @(posedge clk); feed_begin <= 0; end endtask
    task arm(input [2:0] f, input [6:0] h); begin
        @(posedge clk); flags <= f; half <= h; spi_arm_stb <= 1; @(posedge clk); spi_arm_stb <= 0;
    end endtask
    task wait_idle; integer n; begin
        n = 0; repeat (4) @(posedge clk);
        while (spi_busy && n < 2000000) begin @(posedge clk); n = n + 1; end
        if (spi_busy) fail("engine stuck busy");
    end endtask
    task reply(input integer i, output [7:0] v); begin
        @(posedge clk); rd_addr <= i; @(posedge clk); @(posedge clk); v = rd_data;
    end endtask
    task expect_reply(input integer i, input [7:0] want); reg [7:0] v; begin
        reply(i, v);
        if (v !== want) begin $display("  reply[%0d] = %02h, want %02h", i, v, want); fail("reply"); end
    end endtask
    localparam S = 8'd1, P = 8'd2, W = 8'd3, RA = 8'd4, RN = 8'd5;

    task i2c_suite(input [6:0] h, input integer st); integer s0, p0, g0; begin
        $display("I2C half=%0d stretch=%0d", h, st);
        stretch = st; s0 = starts; p0 = stops; g0 = glitches;
        arm(3'b100, h);
        // write 0x01 <- AB CD
        begin_feed; expect_cond = 1;
        feed(S); feed(W); feed(8'h90); feed(W); feed(8'h01); feed(W); feed(8'hAB); feed(W); feed(8'hCD); feed(P);
        wait_idle;
        if (reply_count != 4) begin $display("  count %0d", reply_count); fail("write reply count"); end
        expect_reply(0, 8'h00); expect_reply(1, 8'h00); expect_reply(2, 8'h00); expect_reply(3, 8'h00);
        if (regs[1] !== 8'hAB || regs[2] !== 8'hCD) fail("target registers after write");
        // write-then-read 0x01: AB CD
        begin_feed; expect_cond = 1;
        feed(S); feed(W); feed(8'h90); feed(W); feed(8'h01);
        feed(S); feed(W); feed(8'h91); feed(RA); feed(RN); feed(P);
        wait_idle;
        if (reply_count != 5) begin $display("  count %0d", reply_count); fail("write-read reply count"); end
        expect_reply(0, 8'h00); expect_reply(1, 8'h00); expect_reply(2, 8'h00);
        expect_reply(3, 8'hAB); expect_reply(4, 8'hCD);
        // nobody at 0x50: NACK
        begin_feed;
        feed(S); feed(W); feed(8'hA0); feed(P);
        wait_idle;
        expect_reply(0, 8'h01);
        if (starts - s0 != 4 || stops - p0 != 3) begin
            $display("  starts %0d stops %0d", starts - s0, stops - p0); fail("START/STOP count");
        end
        if (glitches - g0 != 7) begin $display("  SDA moves with SCL high: %0d (7 are START/STOP)", glitches - g0); fail("SDA moved while SCL high"); end
        if (!scl || !sda) fail("bus not idle after STOP");
        if (q_flags[7]) fail("arbitration flag set");
    end endtask

    // ---- SPI slave, one mode at a time ----
    reg [1:0] mode = 0;
    reg [7:0] s_rx, s_tx;
    integer   s_bits;
    wire cpol = mode[1], cpha = mode[0];
    // leading edge = the first edge away from idle
    wire lead_rise = ~cpol;
    always @(sck) if (spi_mode) begin
        if ((sck == lead_rise) ^ cpha) begin            // sample edge
            s_rx = {s_rx[6:0], mosi}; s_bits = s_bits + 1;
            if (cpha == 0 && s_bits < 8) ;              // CPHA 0 shifts out on the other edge
        end else begin                                   // shift edge
            if (cpha == 1) begin miso = s_tx[7]; s_tx = {s_tx[6:0], 1'b0}; end
            else if (s_bits > 0) begin miso = s_tx[7]; s_tx = {s_tx[6:0], 1'b0}; end
        end
    end

    function [7:0] rev8(input [7:0] b); integer k; begin for (k = 0; k < 8; k = k + 1) rev8[k] = b[7-k]; end endfunction

    task spi_mode_test(input [1:0] m); reg [7:0] v; begin
        mode = m;
        // the engine idles at CPOL whatever the mode: flags = {0, inv, cpol}, inv = CPOL ^ CPHA
        arm({1'b0, m[1] ^ m[0], m[1]}, 7'd4);
        repeat (3) @(posedge clk);
        if (sck !== m[1]) fail("SCK idle level");
        s_bits = 0; s_rx = 0; s_tx = 8'hC5;
        if (m[0] == 0) begin miso = s_tx[7]; s_tx = {s_tx[6:0], 1'b0}; end   // CPHA 0: first bit at CS
        begin_feed;
        feed(rev8(8'h3A));                               // the engine shifts bit 0 first
        wait_idle;
        if (sck !== m[1]) fail("SCK idle level after the byte");
        if (s_rx !== 8'h3A) begin $display("  mode %0d slave got %02h", m, s_rx); fail("SPI MOSI"); end
        reply(0, v);
        if (rev8(v) !== 8'hC5) begin $display("  mode %0d master got %02h", m, rev8(v)); fail("SPI MISO"); end
    end endtask

    initial begin
        regs[0] = 0; regs[1] = 0; regs[2] = 0; regs[3] = 8'h5A; ptr = 0;
        repeat (5) @(posedge clk); rst = 0;
        i2c_suite(7'd13, 0);
        i2c_suite(7'd33, 0);
        i2c_suite(7'd120, 0);
        i2c_suite(7'd13, 200);      // the target stretches SCL after every ACK

        // arbitration: another master holds SDA low during our first address bit (a 1)
        $display("I2C arbitration");
        arm(3'b100, 7'd13);
        begin_feed;
        feed(S); feed(W); feed(8'h90);
        wait (scl == 0); wait (scl == 1); x_sda_low = 1;   // after START, during the first bit
        repeat (2000) @(posedge clk); x_sda_low = 0;
        wait_idle;
        if (!q_flags[7]) fail("lost arbitration not flagged");
        if (!clk_oe && 0) ;
        if (m_scl_low || m_sda_low) fail("lines not released after lost arbitration");

        $display("SPI modes");
        spi_mode_test(2'd0);
        spi_mode_test(2'd1);
        spi_mode_test(2'd2);
        spi_mode_test(2'd3);

        if (errors == 0) $display("PASS tb_i2c_master");
        else $display("FAILED tb_i2c_master: %0d errors", errors);
        $finish;
    end
    initial begin #400_000_000; $display("FAIL: timeout"); $finish; end
endmodule
