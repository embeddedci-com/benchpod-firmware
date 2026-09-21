// ============================================================================
// tb_i2c_target.v — self-checking testbench for the generic I2C target +
// register file.  Drives a synthetic I2C master against i2c_target/i2c_regfile
// and verifies:
//   1. address match + ACK
//   2. register-pointer write then repeated-START read (chip-ID, calibration,
//      measurement bytes) with auto-increment
//   3. write capture into the register file (ctrl_meas)
//   4. the generic conversion handshake: writing the trigger register sets the
//      configured busy bit in the status register for the configured time
//
// Run:  make -C ice40 simtest      (needs iverilog/vvp)
// ============================================================================

`timescale 1ns/1ps

module tb_i2c_target;

    // 24 MHz-ish system clock (≈41.7 ns period).  CLK_MHZ is the rate the DUT's
    // µs prescaler is told to assume; the timing assertion below checks the
    // conversion window in clocks against it (a pure clock-count invariant, so
    // the ~23.8 vs 24 MHz period rounding is irrelevant).
    localparam integer CLK_MHZ = 24;
    reg clk = 1'b0;
    always #21 clk = ~clk;

    reg rst = 1'b1;

    // ---- I2C bus model (open-drain, wired-AND) ----
    reg  master_scl = 1'b1;
    reg  master_sda_low = 1'b0;   // 1 → master pulls SDA low; 0 → master releases
    wire tgt_sda_drive_low;
    wire scl_bus = master_scl;
    wire sda_bus = (master_sda_low | tgt_sda_drive_low) ? 1'b0 : 1'b1;

    // ---- config wires ----
    reg        cfg_stb = 1'b0;
    reg [6:0]  cfg_addr7 = 7'h76;
    reg [7:0]  cfg_trig_reg = 8'hF4;
    reg [7:0]  cfg_busy_reg = 8'hF3;
    reg [7:0]  cfg_busy_mask = 8'h08;
    reg [15:0] cfg_conv_us = 16'd2000;   // 2 ms conversion (realistic BMP280)

    // ---- register file SPI-side preload ----
    reg        pre_we = 1'b0;
    reg [7:0]  pre_waddr = 8'h00;
    reg [7:0]  pre_wdata = 8'h00;

    // regfile <-> target
    wire       t_reg_we;
    wire [7:0] t_reg_waddr, t_reg_wdata, t_reg_raddr, t_reg_rdata;

    wire        i2c_armed;
    wire [15:0] xfer_count, wr_count;
    wire [7:0]  last_wr_addr, last_wr_val;

    i2c_regfile regfile_i (
        .clk(clk),
        .i2c_we(t_reg_we), .i2c_waddr(t_reg_waddr), .i2c_wdata(t_reg_wdata),
        .i2c_raddr(t_reg_raddr), .i2c_rdata(t_reg_rdata),
        .spi_we(pre_we), .spi_waddr(pre_waddr), .spi_wdata(pre_wdata),
        .spi_raddr(8'h00), .spi_rdata()
    );

    i2c_target #(.CLK_MHZ(CLK_MHZ)) dut (
        .clk(clk), .rst(rst),
        .cfg_stb(cfg_stb), .cfg_addr7(cfg_addr7),
        .cfg_sda_ch(4'd0), .cfg_scl_ch(4'd1), .cfg_enable(1'b1),
        .cfg_trig_reg(cfg_trig_reg), .cfg_busy_reg(cfg_busy_reg),
        .cfg_busy_mask(cfg_busy_mask), .cfg_conv_us(cfg_conv_us),
        .disable_stb(1'b0),
        .sda_ch(), .scl_ch(),
        .scl_in(scl_bus), .sda_in(sda_bus),
        .sda_drive_low(tgt_sda_drive_low),
        .reg_we(t_reg_we), .reg_waddr(t_reg_waddr), .reg_wdata(t_reg_wdata),
        .reg_raddr(t_reg_raddr), .reg_rdata(t_reg_rdata),
        .armed(i2c_armed), .xfer_count(xfer_count), .wr_count(wr_count),
        .last_wr_addr(last_wr_addr), .last_wr_val(last_wr_val)
    );

    // ---- I2C bit timing (in clk cycles per half-phase) ----
    localparam integer SCL_LO = 24;
    localparam integer SCL_HI = 24;

    integer errors = 0;

    // ---- conversion-window timing monitor ----
    // The DUT's µs prescaler decrements conv_left once per CLK_MHZ clocks, so the
    // conversion window (conv_busy high) must last exactly conv_us * CLK_MHZ
    // system clocks.  Count clocks across one window via a hierarchical peek at
    // the internal conv_busy wire; assert the latched length below.  Guards the
    // prescaler against the CLK_MHZ-mismatch class of bug (CLK_MHZ != real clk,
    // or CLK_MHZ>31 silently truncated by the [4:0] prescaler width).
    integer conv_clks = 0;
    integer conv_meas = 0;            // latched window length (clocks); 0 = never ran
    reg     conv_busy_d = 1'b0;
    always @(posedge clk) begin
        conv_busy_d <= dut.conv_busy;
        if (dut.conv_busy && !conv_busy_d)      conv_clks <= 1;             // rising: first busy clk
        else if (dut.conv_busy)                 conv_clks <= conv_clks + 1; // running
        else if (!dut.conv_busy && conv_busy_d) conv_meas <= conv_clks;     // falling: latch length
    end

    task tick(input integer n);
        begin repeat (n) @(posedge clk); end
    endtask

    task i2c_start;
        begin
            master_sda_low = 1'b0; master_scl = 1'b1; tick(SCL_HI);
            master_sda_low = 1'b1;                     tick(SCL_HI);   // SDA↓ while SCL high
            master_scl     = 1'b0;                     tick(SCL_LO);
        end
    endtask

    task i2c_stop;
        begin
            master_sda_low = 1'b1; master_scl = 1'b0; tick(SCL_LO);
            master_scl     = 1'b1;                    tick(SCL_HI);
            master_sda_low = 1'b0;                    tick(SCL_HI);   // SDA↑ while SCL high
        end
    endtask

    // write one bit (master drives SDA)
    task wr_bit(input b);
        begin
            master_sda_low = ~b;             // drive low for 0, release for 1
            master_scl = 1'b0; tick(SCL_LO);
            master_scl = 1'b1; tick(SCL_HI);
            master_scl = 1'b0;
        end
    endtask

    // write a byte; returns slave ACK (1 = ACKed)
    task wr_byte(input [7:0] d, output ack);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) wr_bit(d[i]);
            // 9th clock: master releases, sample slave ACK mid-high
            master_sda_low = 1'b0;
            master_scl = 1'b0; tick(SCL_LO);
            master_scl = 1'b1; tick(SCL_HI/2);
            ack = ~sda_bus;                  // ACK = SDA pulled low by slave
            tick(SCL_HI/2);
            master_scl = 1'b0; tick(SCL_LO); // hold SCL low so slave sees the
                                             // 9th fall (releases ACK / advances)
        end
    endtask

    // read a byte; master drives ACK (do_ack=1) or NACK afterwards
    task rd_byte(input do_ack, output [7:0] d);
        integer i;
        begin
            d = 8'h00;
            for (i = 7; i >= 0; i = i - 1) begin
                master_sda_low = 1'b0;        // release so slave drives
                master_scl = 1'b0; tick(SCL_LO);
                master_scl = 1'b1; tick(SCL_HI/2);
                d[i] = sda_bus;
                tick(SCL_HI/2);
                master_scl = 1'b0;
            end
            // 9th clock: master drives ACK/NACK
            master_sda_low = do_ack;          // 1 → pull low = ACK
            master_scl = 1'b0; tick(SCL_LO);
            master_scl = 1'b1; tick(SCL_HI);
            master_scl = 1'b0; tick(SCL_LO);  // hold SCL low after the ACK clock
            master_sda_low = 1'b0;
        end
    endtask

    task check_eq(input [7:0] got, input [7:0] exp, input [319:0] label);
        begin
            if (got !== exp) begin
                $display("FAIL: %0s got 0x%02x expected 0x%02x", label, got, exp);
                errors = errors + 1;
            end else begin
                $display("ok:   %0s = 0x%02x", label, got);
            end
        end
    endtask

    reg ack;
    reg [7:0] b0, b1, b2;

    initial begin
        // ---- reset ----
        tick(8);
        rst = 1'b0;
        tick(8);

        // ---- preload register image (SPI side) ----
        // chip ID @ 0xD0, a couple of measurement bytes @ 0xF7..0xF8, status 0xF3=0
        @(posedge clk); pre_waddr=8'hD0; pre_wdata=8'h58; pre_we=1'b1;
        @(posedge clk); pre_waddr=8'hF7; pre_wdata=8'hAB;
        @(posedge clk); pre_waddr=8'hF8; pre_wdata=8'hCD;
        @(posedge clk); pre_waddr=8'hF3; pre_wdata=8'h00;
        @(posedge clk); pre_we=1'b0;
        tick(4);

        // ---- configure & enable the target ----
        @(posedge clk); cfg_stb=1'b1;
        @(posedge clk); cfg_stb=1'b0;
        tick(4);

        // ============================================================
        // Test 1: write reg pointer 0xD0, repeated-START read chip ID
        // ============================================================
        i2c_start;
        wr_byte({cfg_addr7, 1'b0}, ack);   // address + write
        if (!ack) begin $display("FAIL: no ACK on address(W)"); errors=errors+1; end
        wr_byte(8'hD0, ack);               // register pointer
        if (!ack) begin $display("FAIL: no ACK on reg pointer"); errors=errors+1; end
        i2c_start;                          // repeated START
        wr_byte({cfg_addr7, 1'b1}, ack);   // address + read
        if (!ack) begin $display("FAIL: no ACK on address(R)"); errors=errors+1; end
        rd_byte(1'b0, b0);                  // single byte, NACK
        i2c_stop;
        check_eq(b0, 8'h58, "chip ID @0xD0");

        // ============================================================
        // Test 2: pointer 0xF7, read two bytes with auto-increment
        // ============================================================
        i2c_start;
        wr_byte({cfg_addr7, 1'b0}, ack);
        wr_byte(8'hF7, ack);
        i2c_start;
        wr_byte({cfg_addr7, 1'b1}, ack);
        rd_byte(1'b1, b0);                  // ACK → continue
        rd_byte(1'b0, b1);                  // NACK → last
        i2c_stop;
        check_eq(b0, 8'hAB, "meas[0] @0xF7");
        check_eq(b1, 8'hCD, "meas[1] @0xF8 (auto-inc)");

        // ============================================================
        // Test 3: write capture — write 0x55 to ctrl_meas 0xF4
        //         (also the trigger register → starts conversion)
        // ============================================================
        i2c_start;
        wr_byte({cfg_addr7, 1'b0}, ack);
        wr_byte(8'hF4, ack);               // pointer
        wr_byte(8'h55, ack);               // data → stored at 0xF4, triggers conv
        i2c_stop;
        check_eq(last_wr_addr, 8'hF4, "last_wr_addr");
        check_eq(last_wr_val,  8'h55, "last_wr_val");

        // ============================================================
        // Test 4: busy overlay — read status 0xF3 right after trigger.
        // conversion is 2 ms; a status-read transaction takes ~70 µs, so the
        // busy bit (0x08) must still be set.
        // ============================================================
        i2c_start;
        wr_byte({cfg_addr7, 1'b0}, ack);
        wr_byte(8'hF3, ack);
        i2c_start;
        wr_byte({cfg_addr7, 1'b1}, ack);
        rd_byte(1'b0, b0);
        i2c_stop;
        check_eq(b0, 8'h08, "status 0xF3 busy bit during conversion");

        // wait out the conversion, then status must clear
        #3000000;                           // 3 ms > 2 ms conv
        i2c_start;
        wr_byte({cfg_addr7, 1'b0}, ack);
        wr_byte(8'hF3, ack);
        i2c_start;
        wr_byte({cfg_addr7, 1'b1}, ack);
        rd_byte(1'b0, b0);
        i2c_stop;
        check_eq(b0, 8'h00, "status 0xF3 cleared after conversion");

        // ============================================================
        // Test 5: addressing a different device must NOT be ACKed
        // ============================================================
        i2c_start;
        wr_byte({7'h20, 1'b0}, ack);
        i2c_stop;
        if (ack) begin $display("FAIL: ACKed a foreign address"); errors=errors+1; end
        else        $display("ok:   foreign address not ACKed");

        // ============================================================
        // Test 6: conversion-timer absolute timing.  The window triggered in
        // Test 3 (write to trig_reg 0xF4) completed during Test 4's 3 ms wait;
        // its length must be exactly conv_us * CLK_MHZ system clocks.
        // ============================================================
        if (conv_meas !== cfg_conv_us * CLK_MHZ) begin
            $display("FAIL: conv window %0d clks (want %0d = conv_us*CLK_MHZ)",
                     conv_meas, cfg_conv_us * CLK_MHZ);
            errors = errors + 1;
        end else
            $display("ok:   conv window = %0d clks (conv_us*CLK_MHZ)", conv_meas);

        if (errors == 0) $display("\nALL TESTS PASSED");
        else             $display("\n%0d TEST(S) FAILED", errors);
        $finish;
    end

    // safety timeout
    initial begin
        #5000000;
        $display("FAIL: testbench timeout");
        $finish;
    end


endmodule
