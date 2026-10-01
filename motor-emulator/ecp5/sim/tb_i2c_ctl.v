// tb_i2c_ctl — the I2C controller (src/i2c_ctl.v) against an M24C02 and a PCAL6408A model.
//  1 after reset: probe finds the EEPROM at 0x52 (id 2), then the expander is set up
//  2 trip recorder: OC_TRIP_N (P0) pulses low and recovers; the read still shows it low (latched);
//    a second read shows the live state again
//  3 EEPROM: random read; byte write with WC low during it and ACK polling through the write
//    cycle; the byte reads back; WC high again
//  4 errors: a write to an absent EEPROM sets err; a read of the expander with it gone sets err
//  5 second run: re-probe with the EEPROM at 0x51 -> id 1; no EEPROM -> id 0, ok 0
//  SCL never faster than 100 kHz.
`timescale 1ns/1ps
`default_nettype none
module tb_i2c_ctl;
    reg clk = 0;
    always #13.889 clk = ~clk;
    reg rst = 1, probe_start = 0, trip_req = 0, ee_wr = 0, ee_rd = 0;
    reg [7:0] ee_addr = 0, ee_wdata = 0;
    wire scl_oe, sda_oe, wc_low, done, ok, trip_valid, pcal_ok, busy, err;
    wire [1:0] board_id;
    wire [7:0] trip_src, ee_rdata;
    tri1 sda, scl;
    assign sda = sda_oe ? 1'b0 : 1'bz;
    assign scl = scl_oe ? 1'b0 : 1'bz;
    reg [6:0] ee_a7 = 7'h52;
    reg ee_present = 1, pc_present = 1;
    reg [7:0] pins = 8'h7F;
    i2c_dev_model #(.KIND(0), .WRITE_NS(3_000_000)) ee (.sda(sda), .scl(scl), .addr7(ee_a7), .present(ee_present), .wc_n(~wc_low), .pins(8'h00));
    i2c_dev_model #(.KIND(1)) pc (.sda(sda), .scl(scl), .addr7(7'h20), .present(pc_present), .wc_n(1'b1), .pins(pins));
    i2c_ctl #(.QUARTER(90)) dut (.clk(clk), .rst(rst), .sda_in(sda), .scl_oe(scl_oe), .sda_oe(sda_oe), .wc_low(wc_low),
        .probe_start(probe_start), .board_id(board_id), .done(done), .ok(ok),
        .trip_req(trip_req), .trip_src(trip_src), .trip_valid(trip_valid), .pcal_ok(pcal_ok),
        .ee_wr(ee_wr), .ee_rd(ee_rd), .ee_addr(ee_addr), .ee_wdata(ee_wdata), .ee_rdata(ee_rdata), .busy(busy), .err(err));

    realtime t_last = 0; integer bad_rate = 0;
    always @(posedge scl) begin
        if (t_last > 0 && ($realtime - t_last) < 9900.0) bad_rate = bad_rate + 1;
        t_last = $realtime;
    end
    integer errors = 0;
    task check(input [8*48-1:0] what, input okx); if (!okx) begin errors = errors + 1; $display("FAIL: %0s", what); end endtask
    // strobes (a task output argument is only copied back when the task ends, so one task each)
    task p_trip;  begin @(negedge clk) trip_req = 1;    @(negedge clk) trip_req = 0;    end endtask
    task p_rd;    begin @(negedge clk) ee_rd = 1;       @(negedge clk) ee_rd = 0;       end endtask
    task p_wr;    begin @(negedge clk) ee_wr = 1;       @(negedge clk) ee_wr = 0;       end endtask
    task p_probe; begin @(negedge clk) probe_start = 1; @(negedge clk) probe_start = 0; end endtask
    task wait_idle; begin repeat (4) @(negedge clk); while (busy) @(negedge clk); end endtask
    reg wc_seen;
    always @(posedge wc_low) wc_seen = 1;

    initial begin
        #400_000_000 $display("FAIL tb_i2c_ctl: timeout"); $finish;
    end
    initial begin
        repeat (4) @(posedge clk); rst = 0;
        // ---- 1
        wait_idle;
        check("probe: id 2, ok", done && ok && board_id == 2'd2);
        check("expander set up", pcal_ok && !err);
        check("expander latch register", pc.regs[8'h42] == 8'h7F && pc.regs[8'h45] == 8'h00);
        $display("  ok probe id %0d, expander set up", board_id);
        // ---- 2
        #1000 pins = 8'h7E; #20000 pins = 8'h7F;             // OC_TRIP_N pulses low for 20 us
        #5000 p_trip; wait_idle;
        check("trip read shows the latched P0", trip_valid && trip_src == 8'h7E && !err);
        p_trip; wait_idle;
        check("second read: live again", trip_src == 8'h7F);
        pins = 8'h5F;                                       // LATCH_Q (P5) low and stays
        p_trip; wait_idle;
        check("a level change", trip_src == 8'h5F);
        pins = 8'h7F;
        $display("  ok trip recorder");
        // ---- 3
        @(negedge clk) ee_addr = 8'h10; p_rd; wait_idle;
        check("EEPROM read", ee_rdata == (8'h10 ^ 8'h5A) && !err);
        wc_seen = 0;
        @(negedge clk) begin ee_addr = 8'h10; ee_wdata = 8'hC3; end p_wr;
        repeat (4) @(negedge clk);
        while (busy) @(negedge clk);
        check("WC went low for the write and is back high", wc_seen && !wc_low);
        check("write: no error", !err);
        check("the model stored it", ee.mem[8'h10] == 8'hC3);
        check("the controller waited out the write cycle", $realtime > ee.busy_until);
        @(negedge clk) ee_addr = 8'h10; p_rd; wait_idle;
        check("reads back", ee_rdata == 8'hC3);
        @(negedge clk) begin ee_addr = 8'h11; ee_wdata = 8'h3C; end p_wr; wait_idle;
        @(negedge clk) ee_addr = 8'h11; p_rd; wait_idle;
        check("second write and read (new values)", ee_rdata == 8'h3C);
        $display("  ok EEPROM read, write with ACK polling, read back");
        // ---- 4
        ee_present = 0;
        @(negedge clk) begin ee_addr = 8'h12; ee_wdata = 8'h11; end p_wr;
        repeat (4) @(negedge clk); while (busy) @(negedge clk);
        check("write to an absent EEPROM: err", err);
        check("WC released after the failed write", !wc_low);
        ee_present = 1; pc_present = 0;
        p_trip; wait_idle;
        check("expander gone: err", err);
        pc_present = 1;
        $display("  ok errors");
        // ---- 5
        ee_a7 = 7'h51; p_probe; wait_idle;
        check("re-probe: id 1", done && ok && board_id == 2'd1);
        ee_present = 0; p_probe; wait_idle;
        check("no EEPROM: id 0, ok 0", done && !ok && board_id == 2'd0);
        check("SCL at most 100 kHz", bad_rate == 0);
        $display("  ok re-probe");
        if (errors == 0) $display("PASS tb_i2c_ctl"); else $display("FAIL tb_i2c_ctl: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
