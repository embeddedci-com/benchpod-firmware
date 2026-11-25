// image_boot.v — runtime gateware image switch by SB_WARMBOOT (experiment, branch ice40-warmboot).
//
// The iCE40's config flash starts with a multi-boot header (icemulti layout, written by the
// firmware): entry 0 = the power-on image, entries 1..4 = the warm-boot targets for S1:S0 = 0..3,
// each pointing at a gateware slot in the same flash.  SB_WARMBOOT makes the iCE40 reconfigure
// itself from the selected entry, reading the flash in SPI master mode over its config pins.
//
// Trigger: one SPI transaction of exactly  1E 57 42 sel  (opcode IMAGE_BOOT + the key "WB" + the
// image 0..3).  cmd_dispatch ignores 0x1E (unknown opcode: S_DONE until CS rises), so it needs no
// decode there.  BOOT fires once CS has risen (the transaction is over), held high until the
// reconfiguration resets the fabric.  Anything else (a wrong key, a 5th byte) disarms it.
// The firmware holds bus_own (PG0) high around the switch, so the PSRAM pads are tristated
// (psram_pads), its own OCTOSPI pins are Hi-Z and the PSRAM /CS is driven high.
module image_boot (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] rx_byte,
    input  wire       rx_valid,
    input  wire       cs_active
);
    reg [2:0] n = 3'd0;        // bytes of this transaction that matched so far (4 = armed)
    reg [1:0] sel = 2'd0;
    reg       bad = 1'b0;
    reg       cs_q = 1'b0;
    reg       boot = 1'b0;
    always @(posedge clk) begin
        cs_q <= cs_active;
        if (rst) begin
            n <= 3'd0; bad <= 1'b0; boot <= 1'b0;
        end else if (cs_active & ~cs_q) begin
            n <= 3'd0; bad <= 1'b0;                        // a new transaction
        end else if (rx_valid & cs_active & ~bad) begin
            case (n)
                3'd0: if (rx_byte == 8'h1E) n <= 3'd1; else bad <= 1'b1;
                3'd1: if (rx_byte == 8'h57) n <= 3'd2; else bad <= 1'b1;
                3'd2: if (rx_byte == 8'h42) n <= 3'd3; else bad <= 1'b1;
                3'd3: if (rx_byte[7:2] == 6'd0) begin sel <= rx_byte[1:0]; n <= 3'd4; end
                      else bad <= 1'b1;
                default: bad <= 1'b1;                      // more bytes than the command has
            endcase
        end else if (~cs_active & cs_q & (n == 3'd4) & ~bad) begin
            boot <= 1'b1;                                  // CS rose after exactly the command
        end
    end
    SB_WARMBOOT warmboot_i (.BOOT(boot), .S1(sel[1]), .S0(sel[0]));
endmodule
