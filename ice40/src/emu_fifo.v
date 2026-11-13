// ============================================================================
// emu_fifo.v: the emulation image's shared sensor FIFO, in the one free SPRAM block (v54).
//
// The STM32 pushes bytes (EMU_FIFO_WRITE); reads of a FIFO register (attribute bit3, on an I2C
// target or the SPI target) pop them.  It lives in a SB_SPRAM256KA so that it costs no block RAM
// (the emulation image uses all 30) and is deep: 2^AW - 1 bytes (2047), one per 16-bit word.
//
// The SPRAM has one port, so the head byte is kept in a register (`head`, valid while !empty) and
// refilled from the memory after each pop.  Per clk the port does at most one access: a pending
// push first (the pushed byte, rx_byte in cmd_dispatch, holds >= 16 clk), else a head refill.  A
// pop is followed by a refill within 2 clk; the next pop is at least one bus byte later.
//
// The level is mlevel + the head (cmd_dispatch reports both).  A push into a full FIFO is
// dropped (ovf pulse).
// ============================================================================
module emu_fifo #(
    parameter AW = 11
)(
    input  wire         clk,
    input  wire         rst,          // also the flush
    input  wire         we,           // push wdata (strobe)
    input  wire [7:0]   wdata,
    input  wire         pop,          // the head was consumed (only while !empty)
    output reg  [7:0]   head,
    output wire         empty,
    output wire [AW-1:0] mlevel,      // bytes in the memory (the level is mlevel + !empty)
    output wire         full,
    output wire         ovf           // a push was dropped (pulse)
);
    reg  [AW-1:0] wp, rp;
    reg           hv;                 // head valid
    reg           wpend;              // a push waits for the port
    reg           fetch;              // a head refill read was issued last clk
    wire [15:0]   dout;

    // The memory holds wp - rp bytes (one slot stays free, so full is all ones): one subtract
    // instead of a separate up/down counter.
    assign mlevel = wp - rp;
    assign empty  = ~hv;
    assign full   = &mlevel;
    assign ovf    = we & full;

    wire do_wr = wpend;
    wire do_rd = ~wpend & ~fetch & (~hv | pop) & (mlevel != {AW{1'b0}});

    SB_SPRAM256KA ram (
        .ADDRESS({{(14-AW){1'b0}}, do_wr ? wp : rp}), .DATAIN({8'd0, wdata}), .MASKWREN(4'b0011),
        .WREN(do_wr), .CHIPSELECT(do_wr | do_rd), .CLOCK(clk),
        .STANDBY(1'b0), .SLEEP(1'b0), .POWEROFF(1'b1), .DATAOUT(dout));

    always @(posedge clk) begin
        if (rst) begin
            wp <= {AW{1'b0}};  rp <= {AW{1'b0}};
            hv <= 1'b0;  wpend <= 1'b0;  fetch <= 1'b0;
        end else begin
            fetch <= do_rd;
            if (we && !full) wpend <= 1'b1;
            else if (do_wr)  wpend <= 1'b0;
            if (do_wr) wp <= wp + {{(AW-1){1'b0}}, 1'b1};
            if (do_rd) rp <= rp + {{(AW-1){1'b0}}, 1'b1};
            if (fetch)    begin head <= dout[7:0]; hv <= 1'b1; end
            else if (pop) hv <= 1'b0;
        end
    end
endmodule
