// tb_i2c_probe — board id from the EEPROM strap address.
// Run 1: EEPROM at 0x52 (probed after reset) -> id 2. Run 2: re-probe with it at 0x51 -> id 1.
// Run 3: no EEPROM -> done, ok = 0, id 0. Also checks SCL/SDA are never driven high (open drain
// is structural here: the module only has pull-low enables) and the SCL rate.
`timescale 1ns/1ps
`default_nettype none
module tb_i2c_probe;
    reg clk = 0;
    always #13.889 clk = ~clk;
    reg rst = 1, start = 0;
    wire scl_oe, sda_oe, done, ok;
    wire [1:0] board_id;
    tri1 sda, scl;                         // pull-ups
    assign sda = sda_oe ? 1'b0 : 1'bz;
    assign scl = scl_oe ? 1'b0 : 1'bz;
    reg [6:0] ee_addr = 7'h52;
    reg ee_present = 1;
    i2c_ack_model ee (.sda(sda), .scl(scl), .addr7(ee_addr), .present(ee_present));
    i2c_probe #(.QUARTER(90)) dut (.clk(clk), .rst(rst), .start(start), .sda_in(sda),
        .scl_oe(scl_oe), .sda_oe(sda_oe), .board_id(board_id), .done(done), .ok(ok));

    // SCL period check: rising edges at 360 clk (10 us) inside a byte
    realtime t_last = 0; integer bad_rate = 0;
    always @(posedge scl) begin
        if (t_last > 0 && ($realtime - t_last) < 9900.0) bad_rate = bad_rate + 1;
        t_last = $realtime;
    end

    integer errors = 0;
    task expect_result(input [1:0] id, input okx);
        begin
            wait (done === 1'b0); wait (done === 1'b1);
            if (board_id !== id || ok !== okx) begin
                errors = errors + 1; $display("FAIL: id %0d ok %0b, expected id %0d ok %0b", board_id, ok, id, okx);
            end else $display("  ok id %0d ok %0b", board_id, ok);
        end
    endtask

    initial begin
        #20_000_000 $display("FAIL tb_i2c_probe: timeout"); $finish;
    end
    initial begin
        repeat (4) @(posedge clk); rst = 0;
        expect_result(2'd2, 1'b1);
        ee_addr = 7'h51;
        @(negedge clk) start = 1; @(negedge clk) start = 0;
        expect_result(2'd1, 1'b1);
        ee_present = 0;
        @(negedge clk) start = 1; @(negedge clk) start = 0;
        expect_result(2'd0, 1'b0);
        if (bad_rate != 0) begin errors = errors + 1; $display("FAIL: %0d SCL periods under 10 us", bad_rate); end
        if (errors == 0) $display("PASS tb_i2c_probe"); else $display("FAIL tb_i2c_probe: %0d errors", errors);
        $finish;
    end
endmodule
`default_nettype wire
