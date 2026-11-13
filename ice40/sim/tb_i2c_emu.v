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
//  12  word registers: pointer p = bytes 2p/2p+1, reads and writes per word, auto-increment
//      to the next register, the CMD attribute on a word, p[7] ignored
//  13  command map: 16-bit commands in several high-byte windows (SHT3x/SCD4x-like), CMD
//      busy + nack, writes with data, the log; 16-bit single-window and byte mode unchanged
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
    reg  [2:0]  log_t = 0;  reg [4:0] log_idx = 0;  reg log_byte = 0;  wire [7:0] log_rdata;
    reg         sp_ev = 0;  reg [4:0] sp_idx = 0;  reg [15:0] sp_data = 0;
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
        .log_t(log_t), .log_idx(log_idx), .log_byte(log_byte), .log_rdata(log_rdata),
        .sp_ev(sp_ev), .sp_idx(sp_idx), .sp_data(sp_data),
        .fifo_pop(fifo_pop), .fifo_unf(), .fifo_head(fifo_head), .fifo_empty(fifo_empty));

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
    // one write-log entry {address, value}
    task get_log(input [2:0] t, input [4:0] i, output [15:0] e);
        begin
            @(posedge clk); log_t <= t; log_idx <= i; log_byte <= 0; repeat (3) @(posedge clk); e[15:8] = log_rdata;
            log_byte <= 1; repeat (2) @(posedge clk); e[7:0] = log_rdata;
        end
    endtask
    reg [15:0] le;
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
        integer i, k;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                sda(b[i]); #(TLOW/2); scl_up; #(THIGH); scl(0); #(TLOW/2);
            end
            sda(1); #(TLOW/2); scl_up; ack = ~line[sda_c(bus)]; #(THIGH); scl(0); #(TLOW/2);
        end
    endtask
    task i2c_rbyte(input ack, output [7:0] b);
        integer i, k;
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
        integer i, k;
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
        integer i, k;  reg [7:0] b;
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
        integer i, k;  reg [7:0] b;
        begin
            ok = 1; data = 0;
            i2c_start; i2c_wbyte({a, 1'b1}, ack); ok = ack;
            if (ok) for (i = 0; i < n; i = i + 1) begin i2c_rbyte(i != n - 1, b); data[8*i +: 8] = b; end
            i2c_stop;
        end
    endtask

    reg ok;  reg [63:0] d;  reg [7:0] b8;  reg [31:0] st;
    realtime t0, t1;
    integer i, k;
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
        get_status(0, st); check("2 write count +2", st[23:16], i + 2); get_log(0, i, le);     check("2 log entry 1", le, 16'hF5A0);
        get_log(0, i + 1, le); check("2 log entry 2", le, 16'hF6A5);
        wr_regs(7'h76, 0, 16'hD0, 1, 8'h99, ok);
        readback(0, 0, 8'hD0, b8); check("2 read-only D0 kept", b8, 8'h58);
        get_log(0, i + 2, le); check("2 read-only write still logged", le, 16'hD099);
        // a burst of 5 writes between two reads: all 5 are in the log
        wr_regs(7'h76, 0, 16'h40, 5, 40'h0504030201, ok);
        for (k = 0; k < 5; k = k + 1) begin get_log(0, i + 3 + k, le); check("2 burst logged", le, {8'h40 + k[7:0], k[7:0] + 8'd1}); end
        get_status(0, st); check("2 count after the burst", st[23:16], i + 8);
        // the SPI target's events go to ring 4 in slot 4
        @(posedge clk); sp_idx <= 5'd7; sp_data <= 16'h1234; sp_ev <= 1; @(posedge clk); sp_ev <= 0; spi_gap;
        get_log(4, 7, le); check("SPI target event in ring 4", le, 16'h1234);

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

        // ---- 12: word registers (target 3, 0x45 on bus 1, nack while busy) ----
        bus = 1;
        // page field bit0 = word registers (cmd_dispatch: I2C_CONFIG flags bit6)
        config_t(3, 1, 7'h45, 0, 2'd1, 0, 0, 8'h01, 8'h00, SDA1, SCL1, 2'd0, 1'b1, 1'b0);
        load(3, 0, 8'h00, 8'h0C); load(3, 0, 8'h01, 8'h80);      // reg 0x00 = 0x0C80
        load(3, 0, 8'h02, 8'h02); load(3, 0, 8'h03, 8'h20);      // reg 0x01 = 0x0220
        load(3, 0, 8'h04, 8'h00); load(3, 0, 8'h05, 8'h00);      // reg 0x02 = 0x0000
        load(3, 0, 8'h1E, 8'h01); load(3, 0, 8'h1F, 8'h17);      // reg 0x0F = 0x0117
        load(3, 0, 8'hFC, 8'h54); load(3, 0, 8'hFD, 8'h49);      // reg 0x7E (= 0xFE) = 0x5449
        load(3, 0, 8'hFE, 8'h22); load(3, 0, 8'hFF, 8'h60);      // reg 0x7F (= 0xFF) = 0x2260
        load(3, 1, 8'h0A, 8'h42);                                // reg 0x05: CMD, time 1 (500 us)
        load(3, 0, 8'h0A, 8'hBE); load(3, 0, 8'h0B, 8'hEF);
        rd_regs(7'h45, 0, 16'h0F, 2, d, ok); check("12 ack", ok, 1); check("12 reg 0x0F", d[15:0], 16'h1701);
        rd_regs(7'h45, 0, 16'h01, 2, d, ok); check("12 reg 0x01 (no overlap with 0x00)", d[15:0], 16'h2002);
        rd_regs(7'h45, 0, 16'h00, 6, d, ok); check("12 regs 0x00..0x02 in one read", d[47:0], 48'h0000_2002_800C);
        rd_cur(7'h45, 2, d, ok);             check("12 read on from the pointer (reg 0x02 LSB, reg 0x03)", d[15:0], 16'h0000);
        rd_regs(7'h45, 0, 16'hFE, 4, d, ok); check("12 0xFE/0xFF alias 0x7E/0x7F", d[31:0], 32'h6022_4954);
        rd_regs(7'h45, 0, 16'h7F, 3, d, ok); check("12 auto-increment wraps 0x7F -> 0x00", d[23:0], 24'h0C_6022);
        get_status(3, st);  i = st[23:16];
        wr_regs(7'h45, 0, 16'h07, 2, 16'hCDAB, ok); check("12 write ack", ok, 1);
        readback(3, 0, 8'h0E, b8); check("12 reg 0x07 MSB at byte 0x0E", b8, 8'hAB);
        readback(3, 0, 8'h0F, b8); check("12 reg 0x07 LSB at byte 0x0F", b8, 8'hCD);
        get_log(3, i, le);     check("12 log MSB", le, 16'h0EAB);
        get_log(3, i + 1, le); check("12 log LSB", le, 16'h0FCD);
        rd_regs(7'h45, 0, 16'h07, 2, d, ok); check("12 reg 0x07 read back", d[15:0], 16'hCDAB);
        wr_regs(7'h45, 0, 16'h02, 4, 32'h5678_1234, ok);         // regs 0x02 and 0x03 in one write
        readback(3, 0, 8'h06, b8); check("12 burst write: reg 0x03 MSB", b8, 8'h78);
        rd_regs(7'h45, 0, 16'h02, 2, d, ok); check("12 reg 0x02 after the burst", d[15:0], 16'h1234);
        // the CMD attribute on a word register: setting the pointer starts a conversion
        wr_regs(7'h45, 0, 16'h05, 0, 0, ok); t0 = $realtime;
        rd_cur(7'h45, 2, d, ok); check("12 CMD word: read NACKed while busy", ok, 0);
        while ($realtime - t0 < 520000) #1000;
        rd_cur(7'h45, 2, d, ok); check("12 CMD word: answer", d[15:0], 16'hEFBE);

        // ---- 13: command map (target 2, 0x50 on bus 1, nack while busy) ----
        // page field bit1 = command map (I2C_CONFIG flags bit7): image[h] = base, offset = base ^ l
        config_t(2, 1, 7'h50, 0, 2'd1, 0, 0, 8'h02, 8'h00, SDA1, SCL1, 2'd0, 1'b1, 1'b0);
        load(2, 0, 8'h24, 8'h40);       // 0x24xx: 0x2400 -> 0x40, 0x2416 -> 0x56
        load(2, 0, 8'hE0, 8'h50);       // 0xE000 -> 0x50
        load(2, 0, 8'hF3, 8'h4D);       // 0xF32D -> 0x60
        load(2, 0, 8'h37, 8'hF0);       // 0x3780 -> 0x70
        for (k = 0; k < 6; k = k + 1) begin
            load(2, 0, 8'h40 + k[7:0], 8'hA0 + k[7:0]);   // 0x2400 answer
            load(2, 0, 8'h50 + k[7:0], 8'hB0 + k[7:0]);   // 0xE000 answer
            load(2, 0, 8'h56 + k[7:0], 8'hC0 + k[7:0]);   // 0x2416 answer
        end
        load(2, 0, 8'h60, 8'h80); load(2, 0, 8'h61, 8'h10); load(2, 0, 8'h62, 8'hE1);
        load(2, 1, 8'h40, 8'h02);       // 0x2400: CMD, time 0
        load(2, 1, 8'h56, 8'h42);       // 0x2416: CMD, time 1
        set_time(2, 0, 16'd600);  set_time(2, 1, 16'd200);
        wr_regs(7'h50, 1, 16'h2400, 0, 0, ok); check("13 0x2400 ack", ok, 1); t0 = $realtime;
        rd_cur(7'h50, 6, d, ok); check("13 0x2400 NACKed while measuring", ok, 0);
        while ($realtime - t0 < 620000) #1000;
        rd_cur(7'h50, 6, d, ok); check("13 0x2400 answer", d[47:0], 48'hA5A4A3A2A1A0);
        wr_regs(7'h50, 1, 16'hE000, 0, 0, ok);
        rd_cur(7'h50, 6, d, ok); check("13 0xE000 (no conversion) answers at once", ok, 1);
        check("13 0xE000 answer", d[47:0], 48'hB5B4B3B2B1B0);
        rd_regs(7'h50, 1, 16'hF32D, 3, d, ok); check("13 0xF32D in one transfer", d[23:0], 24'hE11080);
        wr_regs(7'h50, 1, 16'h2416, 0, 0, ok); t0 = $realtime;
        rd_cur(7'h50, 6, d, ok); check("13 0x2416 NACKed (time 1)", ok, 0);
        while ($realtime - t0 < 220000) #1000;
        rd_cur(7'h50, 6, d, ok); check("13 0x2416 answer", d[47:0], 48'hC5C4C3C2C1C0);
        get_status(2, st);  i = st[23:16];
        wr_regs(7'h50, 1, 16'h3780, 3, 24'h33_2211, ok); check("13 write with data ack", ok, 1);
        readback(2, 0, 8'h70, b8); check("13 0x3780 data 0", b8, 8'h11);
        readback(2, 0, 8'h72, b8); check("13 0x3780 data 2", b8, 8'h33);
        get_log(2, i, le);     check("13 log entry at the translated offset", le, 16'h7011);
        get_log(2, i + 2, le); check("13 log entry 3", le, 16'h7233);
        readback(2, 0, 8'h37, b8); check("13 the map entry is kept", b8, 8'hF0);
        rd_regs(7'h50, 1, 16'h3780, 3, d, ok); check("13 0x3780 read back", d[23:0], 24'h332211);
        // back to the 16-bit single window: test 4's layout still serves
        config_t(2, 1, 7'h50, 1, 2'd2, 0, 0, 8'h24, 8'h01, SDA1, SCL1, 2'd0, 1'b1, 1'b0);
        load(2, 0, 8'h00, 8'hE0); load(2, 0, 8'h01, 8'hE1);
        rd_regs(7'h50, 1, 16'h2401, 1, d, ok); check("13 single window 0x2401", d[7:0], 8'hE1);
        rd_regs(7'h50, 1, 16'hE001, 1, d, ok); check("13 single window: 0xE001 outside", d[7:0], 8'h00);
        // and byte mode: target 3 back to 8-bit, byte 0x01 is the LSB of word 0x00
        config_t(3, 1, 7'h45, 0, 2'd1, 0, 0, 8'h00, 8'h00, SDA1, SCL1, 2'd0, 1'b1, 1'b0);
        rd_regs(7'h45, 0, 16'h01, 2, d, ok); check("13 byte mode again", d[15:0], 16'h0280);
        bus = 0;

        // ---- 100 kHz, and a disable ----
        TLOW = 4700; THIGH = 4000;
        rd_regs(7'h76, 0, 16'hF7, 3, d, ok); check("100k F7..F9", d[23:0], 24'h332211);
        disable_t(0);
        rd_regs(7'h76, 0, 16'hD0, 1, d, ok); check("disabled NACKs", ok, 0);
        rd_regs(7'h44, 0, 16'h0F, 1, d, ok); check("t1 unaffected", d[7:0], 8'hBC);
        readback(0, 0, 8'hD0, b8); check("disable keeps the image", b8, 8'h58);

        if (errors == 0) $display("PASS tb_i2c_emu: 4 targets on 2 buses: reads, writes, busy flag/nack/stretch, 16-bit, FIFO, faults, data ready, watchdog, word registers, command map");
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
