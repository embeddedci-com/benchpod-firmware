// ============================================================================
// tb_i2c_emu.v: self-checking bench of the time-multiplexed I2C target engine (i2c_emu, v54).
//
// Four targets on two open-drain buses (two per bus), driven by an I2C master model at 400 kHz
// (and 100 kHz for one test) that honours clock stretching.  The SPI side (cmd_dispatch's
// strobes) is driven directly.  Checks:
//   1  8-bit register reads (single and burst) on two targets of one bus, an absent address NACKs
//   2  DUT writes: stored, counted, last address/value; a read-only register is not stored
//   3  busy flag: a TRIG write starts a conversion, the BUSY register ORs in the mask until it ends
//   4  16-bit register addressing: the window at `page`, 0x00 outside it, writes outside ignored
//   5  per-command busy (CMD attribute) with while_busy = nack: the read address NACKs until done
//   6  while_busy = stretch: SCL is held low until the conversion ends, then the byte is right
//   7  FIFO register: reads pop the FIFO, the pointer stays
//   8  faults: mute (no ACK), bus_stuck (SDA held low), then cleared
//   9  data ready: pulse at conversion end, firmware assert / deassert, a level cleared by a read
//  10  the SCL-silence watchdog releases a target that was left driving SDA
//  11  a reloaded register is served on the next read (the cache refresh)
// ============================================================================
`timescale 1ns/1ps
module tb_i2c_emu;
    reg clk = 0;
    always #20.833 clk = ~clk;            // 24 MHz
    reg rst = 1;

    // ---- LA pads: 14 channels, bus 0 on LA2/LA3 (SDA/SCL), bus 1 on LA8/LA9 ----
    localparam SDA0 = 2, SCL0 = 3, SDA1 = 8, SCL1 = 9, DRDY_CH = 5;
    wire [13:0] pull;
    reg  [13:0] m_pull = 14'd0;           // the master model's pull-downs
    wire [13:0] line = ~(pull | m_pull);  // pulled up, open drain
    reg  [13:0] s1, s2;                   // the 2-flop synchroniser in front of the engine
    always @(posedge clk) begin s1 <= line; s2 <= s1; end

    // ---- SPI-side stimulus ----
    reg  [1:0]  sel_t = 0;
    reg         cfg_stb = 0, cfg_en_only = 0;
    reg  [40:0] cfg_data = 0;
    reg         reg_we = 0, reg_attr = 0;
    reg  [7:0]  reg_addr = 0, reg_wdata = 0;
    wire [7:0]  reg_rdata;
    reg         tt_we = 0;  reg [1:0] tt_idx = 0;  reg [15:0] tt_val = 0;
    reg         fire_stb = 0, fire_set = 0;
    reg  [7:0]  dr_width = 8'd20;
    wire [31:0] status;
    wire [3:0]  drdy_pin;
    wire [15:0] us_now;
    wire        fifo_pop;
    // FIFO model: 0xA0, 0xA1, ...
    reg  [7:0]  fifo_head = 8'hA0;
    reg  [7:0]  fifo_left = 8'd0;
    wire        fifo_empty = (fifo_left == 0);
    always @(posedge clk) if (fifo_pop) begin fifo_head <= fifo_head + 1; fifo_left <= fifo_left - 1; end

    i2c_emu #(.CLK_MHZ(24), .N(14)) dut (
        .clk(clk), .rst(rst), .la_lvl(s2), .pull(pull), .drdy_pin(drdy_pin), .us_now(us_now),
        .sel_t(sel_t), .cfg_stb(cfg_stb), .cfg_en_only(cfg_en_only), .cfg_data(cfg_data),
        .reg_we(reg_we), .reg_attr(reg_attr), .reg_addr(reg_addr), .reg_wdata(reg_wdata),
        .reg_rdata(reg_rdata), .tt_we(tt_we), .tt_idx(tt_idx), .tt_val(tt_val),
        .fire_stb(fire_stb), .fire_set(fire_set), .dr_width(dr_width), .status(status),
        .fifo_pop(fifo_pop), .fifo_head(fifo_head), .fifo_empty(fifo_empty));

    integer errors = 0;
    task check(input [8*64-1:0] what, input [31:0] got, input [31:0] want);
        if (got !== want) begin
            $display("FAIL %0s: got %0h want %0h", what, got, want); errors = errors + 1;
        end
    endtask

    // ---- SPI-side tasks (one strobe, then >= 16 clk like the real link) ----
    task spi_gap; repeat (20) @(posedge clk); endtask
    // cfg_data = {en, addr7, a16, wb(2), mute, stuck, page(8), mask(8), sda(4), scl(4), dmode(2), dpol, dconv}
    task config_t(input [1:0] t, input en, input [6:0] a, input a16, input [1:0] wb, input mute,
                  input stuck, input [7:0] page, input [7:0] mask, input [3:0] sda, input [3:0] scl,
                  input [1:0] dmode, input dpol, input dconv);
        begin
            @(posedge clk); sel_t <= t; cfg_data <= {en, a, a16, wb, mute, stuck, page, mask, sda, scl, dmode, dpol, dconv};
            cfg_stb <= 1; @(posedge clk); cfg_stb <= 0; spi_gap;
        end
    endtask
    task disable_t(input [1:0] t);
        begin
            @(posedge clk); sel_t <= t; cfg_data <= 41'd0; cfg_en_only <= 1; cfg_stb <= 1;
            @(posedge clk); cfg_stb <= 0; spi_gap; cfg_en_only <= 0;
        end
    endtask
    task load(input [1:0] t, input attr, input [7:0] a, input [7:0] d);
        begin
            @(posedge clk); sel_t <= t; reg_attr <= attr; reg_addr <= a; reg_wdata <= d; reg_we <= 1;
            @(posedge clk); reg_we <= 0; reg_addr <= a + 1; spi_gap;   // cmd_dispatch steps the address
        end
    endtask
    task readback(input [1:0] t, input attr, input [7:0] a, output [7:0] d);
        begin
            @(posedge clk); sel_t <= t; reg_attr <= attr; reg_addr <= a; spi_gap; d = reg_rdata;
        end
    endtask
    task set_time(input [1:0] t, input [1:0] i, input [15:0] us);
        begin
            @(posedge clk); sel_t <= t; tt_idx <= i; tt_val <= us; tt_we <= 1;
            @(posedge clk); tt_we <= 0; spi_gap;
        end
    endtask
    task fire(input [1:0] t, input set);
        begin
            @(posedge clk); sel_t <= t; fire_set <= set; fire_stb <= 1;
            @(posedge clk); fire_stb <= 0; spi_gap;
        end
    endtask
    task get_status(input [1:0] t, output [31:0] st);
        begin @(posedge clk); sel_t <= t; spi_gap; st = status; end
    endtask

    // ---- I2C master model ----
    integer TLOW = 1300, THIGH = 1200;   // ns: 400 kHz
    integer bus = 0;                     // which bus the master drives
    integer stretch_ns = 0;              // the longest SCL stretch of the last transfer
    function integer sda_c; input integer b; sda_c = b ? SDA1 : SDA0; endfunction
    function integer scl_c; input integer b; scl_c = b ? SCL1 : SCL0; endfunction
    task sda(input v); m_pull[sda_c(bus)] = ~v; endtask
    task scl(input v); m_pull[scl_c(bus)] = ~v; endtask
    // release SCL and wait until it is really high (a target may stretch it)
    task scl_up;
        realtime t0;
        begin
            scl(1); t0 = $realtime;
            while (line[scl_c(bus)] !== 1'b1) #10;
            if ($realtime - t0 > stretch_ns) stretch_ns = $realtime - t0;
        end
    endtask
    task i2c_start;
        begin sda(1); #(TLOW); scl_up; #(THIGH); sda(0); #(THIGH); scl(0); #(TLOW/2); end
    endtask
    task i2c_rstart;
        begin sda(1); #(TLOW/2); scl_up; #(THIGH); sda(0); #(THIGH); scl(0); #(TLOW/2); end
    endtask
    task i2c_stop;
        begin sda(0); #(TLOW/2); scl_up; #(THIGH); sda(1); #(TLOW); end
    endtask
    // SCL is low on entry and exit; SDA changes TLOW/2 into the low phase
    task i2c_wbyte(input [7:0] b, output ack);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                sda(b[i]); #(TLOW/2); scl_up; #(THIGH); scl(0); #(TLOW/2);
            end
            sda(1); #(TLOW/2); scl_up; ack = ~line[sda_c(bus)]; #(THIGH); scl(0); #(TLOW/2);
        end
    endtask
    task i2c_rbyte(input ack, output [7:0] b);
        integer i;
        begin
            sda(1);
            for (i = 7; i >= 0; i = i - 1) begin
                #(TLOW/2); scl_up; b[i] = line[sda_c(bus)]; #(THIGH); scl(0); #(TLOW/2);
            end
            sda(~ack); #(TLOW/2); scl_up; #(THIGH); scl(0); #(TLOW/2); sda(1);
        end
    endtask

    // register write / read with 8- or 16-bit pointers
    reg ack;
    task wr_regs(input [6:0] a, input a16, input [15:0] ptr, input integer n, input [63:0] data, output ok);
        integer i;
        begin
            ok = 1;
            i2c_start; i2c_wbyte({a, 1'b0}, ack); ok = ok & ack;
            if (a16) begin i2c_wbyte(ptr[15:8], ack); ok = ok & ack; end
            i2c_wbyte(ptr[7:0], ack); ok = ok & ack;
            for (i = 0; i < n; i = i + 1) begin i2c_wbyte(data[8*i +: 8], ack); ok = ok & ack; end
            i2c_stop;
        end
    endtask
    task rd_regs(input [6:0] a, input a16, input [15:0] ptr, input integer n, output [63:0] data, output ok);
        integer i;  reg [7:0] b;
        begin
            ok = 1; data = 0;
            i2c_start; i2c_wbyte({a, 1'b0}, ack); ok = ok & ack;
            if (a16) begin i2c_wbyte(ptr[15:8], ack); ok = ok & ack; end
            i2c_wbyte(ptr[7:0], ack); ok = ok & ack;
            i2c_rstart; i2c_wbyte({a, 1'b1}, ack); ok = ok & ack;
            if (ok) for (i = 0; i < n; i = i + 1) begin i2c_rbyte(i != n - 1, b); data[8*i +: 8] = b; end
            i2c_stop;
        end
    endtask
    // a read that does not set the pointer first
    task rd_cur(input [6:0] a, input integer n, output [63:0] data, output ok);
        integer i;  reg [7:0] b;
        begin
            ok = 1; data = 0;
            i2c_start; i2c_wbyte({a, 1'b1}, ack); ok = ack;
            if (ok) for (i = 0; i < n; i = i + 1) begin i2c_rbyte(i != n - 1, b); data[8*i +: 8] = b; end
            i2c_stop;
        end
    endtask

    reg ok;  reg [63:0] d;  reg [7:0] b8;  reg [31:0] st;
    realtime t0, t1;
    integer i;
    initial begin
        repeat (10) @(posedge clk); rst = 0; repeat (10) @(posedge clk);
        // bus 0: t0 = 0x76 (BMP280-like), t1 = 0x44 (level data ready on DRDY_CH... pin only)
        config_t(0, 1, 7'h76, 0, 2'd0, 0, 0, 8'h00, 8'h08, SDA0, SCL0, 2'd1, 1'b1, 1'b1);
        config_t(1, 1, 7'h44, 0, 2'd0, 0, 0, 8'h00, 8'h00, SDA0, SCL0, 2'd2, 1'b0, 1'b0);
        // bus 1: t2 = 0x50 (16-bit addressing, page 0x24, stretch), t3 = 0x45 (nack while busy)
        config_t(2, 1, 7'h50, 1, 2'd2, 0, 0, 8'h24, 8'h01, SDA1, SCL1, 2'd0, 1'b1, 1'b0);
        config_t(3, 1, 7'h45, 0, 2'd1, 0, 0, 8'h00, 8'h00, SDA1, SCL1, 2'd0, 1'b1, 1'b0);
        load(0, 0, 8'hD0, 8'h58);
        load(0, 0, 8'hF7, 8'h11); load(0, 0, 8'hF8, 8'h22); load(0, 0, 8'hF9, 8'h33);
        load(0, 0, 8'hF3, 8'h01);
        load(0, 1, 8'hF4, 8'h41);        // F4: TRIG, time 1
        load(0, 1, 8'hF3, 8'h04);        // F3: BUSY
        load(0, 1, 8'hD0, 8'h20);        // D0: RO
        set_time(0, 1, 16'd300);
        load(1, 0, 8'h0F, 8'hBC);
        load(1, 0, 8'h3A, 8'h5A);  load(1, 1, 8'h3A, 8'h10);   // 3A: CLR
        load(2, 0, 8'h00, 8'hE0);  load(2, 0, 8'h01, 8'hE1);  load(2, 0, 8'hFF, 8'hEF);
        load(2, 1, 8'h10, 8'h01);  load(2, 0, 8'h10, 8'h70);   // 0x2410: TRIG, time 0
        set_time(2, 0, 16'd400);
        load(3, 1, 8'hFD, 8'h42);        // FD: CMD, time 1
        load(3, 0, 8'hFD, 8'hC1); load(3, 0, 8'hFE, 8'hC2);
        load(3, 1, 8'h74, 8'h08);        // 74: FIFO
        load(3, 0, 8'h75, 8'h68);
        set_time(3, 1, 16'd500);
        readback(0, 0, 8'hF8, b8); check("readback F8", b8, 8'h22);
        readback(0, 1, 8'hF4, b8); check("readback attr F4", b8, 8'h41);

        // ---- 1: reads ----
        bus = 0;
        rd_regs(7'h76, 0, 16'hD0, 1, d, ok); check("1 t0 ack", ok, 1); check("1 t0 D0", d[7:0], 8'h58);
        rd_regs(7'h76, 0, 16'hF7, 3, d, ok); check("1 t0 F7..F9", d[23:0], 24'h332211);
        rd_regs(7'h44, 0, 16'h0F, 1, d, ok); check("1 t1 ack", ok, 1); check("1 t1 0F", d[7:0], 8'hBC);
        rd_regs(7'h77, 0, 16'h00, 1, d, ok); check("1 absent address NACKs", ok, 0);
        // the pointer only advances on an ACK, so it is still on F9 (the last, NACKed byte)
        rd_cur(7'h76, 2, d, ok);             check("1 t0 read from the pointer (F9, FA)", d[15:0], 16'h0033);

        // ---- 2: writes ----
        get_status(0, st);  i = st[23:16];
        wr_regs(7'h76, 0, 16'hF5, 2, 16'hA5A0, ok); check("2 write ack", ok, 1);
        readback(0, 0, 8'hF5, b8); check("2 F5 stored", b8, 8'hA0);
        readback(0, 0, 8'hF6, b8); check("2 F6 stored", b8, 8'hA5);
        get_status(0, st); check("2 write count +2", st[23:16], i + 2); check("2 last write", st[15:0], 16'hF6A5);
        wr_regs(7'h76, 0, 16'hD0, 1, 8'h99, ok);
        readback(0, 0, 8'hD0, b8); check("2 read-only D0 kept", b8, 8'h58);

        // ---- 3: busy flag ----
        wr_regs(7'h76, 0, 16'hF4, 1, 8'h27, ok);           // starts a 300 us conversion
        t0 = $realtime;
        rd_regs(7'h76, 0, 16'hF3, 1, d, ok); check("3 F3 busy", d[7:0], 8'h09);
        get_status(0, st); check("3 status busy", st[30], 1);
        while ($realtime - t0 < 320000) #1000;
        rd_regs(7'h76, 0, 16'hF3, 1, d, ok); check("3 F3 done", d[7:0], 8'h01);
        // 9 (part): t0 has pulse data ready at conversion end, 20 us wide
        // (checked by the monitor below)

        // ---- 4: 16-bit addressing ----
        bus = 1;
        rd_regs(7'h50, 1, 16'h2401, 1, d, ok); check("4 ack", ok, 1); check("4 0x2401", d[7:0], 8'hE1);
        rd_regs(7'h50, 1, 16'h24FF, 2, d, ok); check("4 0x24FF wraps to 0x2400", d[15:0], 16'hE0EF);
        rd_regs(7'h50, 1, 16'h2501, 1, d, ok); check("4 outside the window", d[7:0], 8'h00);
        wr_regs(7'h50, 1, 16'h2501, 1, 8'h55, ok);
        readback(2, 0, 8'h01, b8); check("4 write outside ignored", b8, 8'hE1);
        wr_regs(7'h50, 1, 16'h2402, 1, 8'h66, ok);
        readback(2, 0, 8'h02, b8); check("4 write inside", b8, 8'h66);

        // ---- 5: command busy + nack ----
        wr_regs(7'h45, 0, 16'hFD, 0, 0, ok); check("5 command ack", ok, 1);   // pointer = FD (CMD)
        t0 = $realtime;
        rd_cur(7'h45, 2, d, ok); check("5 read NACKed while busy", ok, 0);
        while ($realtime - t0 < 520000) #1000;
        rd_cur(7'h45, 2, d, ok); check("5 read after", ok, 1); check("5 answer", d[15:0], 16'hC2C1);

        // ---- 6: stretch ----
        wr_regs(7'h50, 1, 16'h2410, 1, 8'h01, ok);         // TRIG: 400 us
        stretch_ns = 0;  t0 = $realtime;
        rd_regs(7'h50, 1, 16'h2410, 1, d, ok);
        check("6 stretched read ack", ok, 1); check("6 value", d[7:0], 8'h01);
        if (stretch_ns < 250000) begin $display("FAIL 6: SCL stretched only %0d ns", stretch_ns); errors = errors + 1; end
        else $display("  6: SCL stretched %0d ns for a 400 us conversion", stretch_ns);
        stretch_ns = 0;
        rd_regs(7'h50, 1, 16'h2410, 1, d, ok);
        if (stretch_ns > 2000) begin $display("FAIL 6: stretched %0d ns with no conversion", stretch_ns); errors = errors + 1; end

        // ---- 7: FIFO ----
        fifo_left = 3;
        rd_regs(7'h45, 0, 16'h74, 4, d, ok);
        check("7 FIFO pops 3 then reads 0", d[31:0], 32'h00A2A1A0);
        rd_cur(7'h45, 1, d, ok); check("7 pointer stayed on the FIFO", d[7:0], 8'h00);
        rd_regs(7'h45, 0, 16'h75, 1, d, ok); check("7 next register", d[7:0], 8'h68);

        // ---- 8: faults ----
        bus = 0;
        config_t(0, 1, 7'h76, 0, 2'd0, 1, 0, 8'h00, 8'h08, SDA0, SCL0, 2'd1, 1'b1, 1'b1);   // mute
        rd_regs(7'h76, 0, 16'hD0, 1, d, ok); check("8 muted NACK", ok, 0);
        rd_regs(7'h44, 0, 16'h0F, 1, d, ok); check("8 the other target still answers", d[7:0], 8'hBC);
        config_t(0, 1, 7'h76, 0, 2'd0, 0, 1, 8'h00, 8'h08, SDA0, SCL0, 2'd1, 1'b1, 1'b1);   // stuck
        #2000; check("8 bus stuck: SDA low", line[SDA0], 0);
        config_t(0, 1, 7'h76, 0, 2'd0, 0, 0, 8'h00, 8'h08, SDA0, SCL0, 2'd1, 1'b1, 1'b1);
        #2000; check("8 released", line[SDA0], 1);
        rd_regs(7'h76, 0, 16'hD0, 1, d, ok); check("8 answers again", d[7:0], 8'h58);

        // ---- 9: data ready ----
        check("9 t1 level idle (active low)", drdy_pin[1], 1);
        fire(1, 1); check("9 t1 asserted", drdy_pin[1], 0);
        rd_regs(7'h44, 0, 16'h0F, 1, d, ok); check("9 t1 other read keeps it", drdy_pin[1], 0);
        rd_regs(7'h44, 0, 16'h3A, 1, d, ok); check("9 3A value", d[7:0], 8'h5A);
        check("9 t1 cleared by reading 3A", drdy_pin[1], 1);
        fire(1, 1); fire(1, 0); check("9 t1 firmware deassert", drdy_pin[1], 1);
        fire(0, 1); t0 = $realtime;
        while (drdy_pin[0] === 1'b1 && $realtime - t0 < 100000) #100;
        t1 = $realtime - t0;
        if (t1 < 18000 || t1 > 23000) begin $display("FAIL 9: t0 firmware pulse %0d ns", t1); errors = errors + 1; end

        // ---- 10: watchdog ----
        i2c_start; i2c_wbyte({7'h76, 1'b0}, ack);
        // the master stops with SCL low after the address ACK: drive SCL low, the target does not hold SDA
        // here; make it hold SDA: send the pointer's first 7 bits, then stop mid-ACK
        sda(1);
        for (i = 7; i >= 0; i = i - 1) begin sda(i[0]); #(TLOW/2); scl_up; #(THIGH); scl(0); #(TLOW/2); end
        sda(1); #3000; check("10 target drives the ACK", line[SDA0], 0);
        #9000000; check("10 watchdog released SDA", line[SDA0], 1);
        scl(1); sda(1); #5000;
        rd_regs(7'h76, 0, 16'hD0, 1, d, ok); check("10 target answers after", d[7:0], 8'h58);

        // ---- 11: reload served on the next read ----
        rd_regs(7'h44, 0, 16'h3B, 1, d, ok); check("11 0x3B before", d[7:0], 8'h00);   // pointer stays on 0x3B
        load(1, 0, 8'h3B, 8'h77);
        rd_cur(7'h44, 1, d, ok); check("11 reload served without a pointer write", d[7:0], 8'h77);
        load(1, 0, 8'h3B, 8'h78);
        rd_cur(7'h44, 1, d, ok); check("11 reload again", d[7:0], 8'h78);

        // ---- 100 kHz, and a disable ----
        TLOW = 4700; THIGH = 4000;
        rd_regs(7'h76, 0, 16'hF7, 3, d, ok); check("100k F7..F9", d[23:0], 24'h332211);
        disable_t(0);
        rd_regs(7'h76, 0, 16'hD0, 1, d, ok); check("disabled NACKs", ok, 0);
        rd_regs(7'h44, 0, 16'h0F, 1, d, ok); check("t1 unaffected", d[7:0], 8'hBC);
        readback(0, 0, 8'hD0, b8); check("disable keeps the image", b8, 8'h58);

        if (errors == 0) $display("PASS tb_i2c_emu: 4 targets on 2 buses: reads, writes, busy flag/nack/stretch, 16-bit, FIFO, faults, data ready, watchdog");
        else             $display("FAIL tb_i2c_emu: %0d error(s)", errors);
        $finish;
    end

    // 9: t0's conversion-end pulse (the conversion of test 3) must be ~20 us
    realtime pr; reg pseen = 0;
    always @(drdy_pin[0]) begin
        // (from 100 us on: before its configuration the pin idles high, the power-up active-low default)
        if (drdy_pin[0] === 1'b1 && !pseen && $realtime > 100000) pr = $realtime;
        else if (drdy_pin[0] === 1'b0 && !pseen && $realtime > 100000) begin
            pseen = 1;
            if ($realtime - pr < 18000 || $realtime - pr > 23000) begin
                $display("FAIL 9: t0 conversion-end pulse %0d ns", $realtime - pr); errors = errors + 1;
            end else $display("  9: t0 conversion-end pulse %0d ns", $realtime - pr);
        end
    end
    initial begin #80_000_000; $display("FAIL tb_i2c_emu: timeout"); $finish; end
endmodule
