// ============================================================================
// spi_slave.v — SPI slave (MODE 1: CPOL 0, CPHA 1) for the iCE40 signal-engine FPGA,
// clocked by SCK.
//
// Receives bytes from the STM32 on MOSI and presents them to cmd_dispatch as rx_byte + a
// 1-cycle rx_valid in the 24 MHz `clk` domain.  Shifts tx_byte out on MISO during the next
// byte.  CSn high resets the bit position (a byte cut short by CSn is dropped) and, through
// cs_active, resets cmd_dispatch's FSM.  Same ports as the old clk-sampled slave.
//
// Mode 1, because in mode 1 every bit has a launching edge: the master launches MOSI and the
// slave launches MISO on the RISING edge, both sample on the FALLING edge.  Bit 7 of a reply is
// launched by the byte's first rise, which comes after the inter-byte gap, so MISO can come
// straight from an SB_IO output register (no LUT, no fabric route between flop and pad).  In
// mode 0 bit 7 must be on MISO before the byte's first edge, i.e. driven combinationally.
//
//   posedge sck : nbit <= nbit + 1 (CSn high: 0, async); MISO pad register <= tx_byte[7 - nbit]
//   negedge sck : rx_sr <= {rx_sr, mosi}; the 8th fall (nbit wrapped to 0) completes the byte
//                 and toggles a cdc_pulse_payload, which copies rx_sr into rx_byte (clk) and
//                 then raises rx_valid.
//
// There is no tx shift register: tx_byte (a cmd_dispatch register in clk) is only written on
// rx_valid, so it is stable while a byte shifts out, and the pad register samples it directly.
//
// INTER-BYTE GAP (the host's contract).  From the 8th SCK FALL of byte N to the first SCK rise
// of byte N+1 the master must wait at least
//     5 clk (toggle catch <= 2 edges, latch, rx_valid, cmd_dispatch writes tx_byte)
//   + tx_byte -> mux -> MISO pad register setup (~12 ns)
//   ~= 225 ns at clk = 24 MHz.
// That also keeps rx_sr stable until the crossing latches it (<= 4 clk).  On the STM32H5 this is
// MIDI (idle SCK periods between frames): fall-to-rise = MIDI * T + T/2, so
//   15.6 MHz (/16): MIDI >= 4;  25 MHz: MIDI >= 6;  31.25 MHz (/8): MIDI >= 7.
//
// CSn ordering.  cs_active reaches cmd_dispatch through CS_SYNC flops.  A byte's rx_valid is
// consumed <= 5 clk edges after its 8th SCK fall; cs_release (cs_active falling, seen one edge
// later) lands >= CS_SYNC + 1 edges after CSn rises.  CS_SYNC = 5 keeps the last byte of a
// transfer ahead of the FSM reset however soon after the last SCK edge CSn rises.
// ============================================================================

module spi_slave #(
    parameter CS_SYNC = 5
)(
    input  wire        clk,        // 24 MHz system clock (SYS_CLK_MHZ)
    input  wire        rst,        // unused (kept for the port list): the SCK side resets on CSn

    // SPI bus (mode 1)
    input  wire        sck,
    input  wire        mosi,
    output wire        miso,       // the PAD: an SB_IO output register lives in here
    input  wire        csn,

    // Parallel byte interface to cmd_dispatch
    output wire [7:0]  rx_byte,    // last received byte (held until the next rx_valid)
    output wire        rx_valid,   // 1-cycle strobe when rx_byte is fresh
    input  wire [7:0]  tx_byte,    // byte to shift out during the next byte
    output wire        cs_active   // !csn, synchronised (CS_SYNC flops)
);

    // ---- SCK domain ----
    reg [2:0] nbit = 3'd0;         // SCK rises seen in this byte
    reg [7:0] rx_sr = 8'h00;

    always @(posedge sck or posedge csn)
        if (csn) nbit <= 3'd0;
        else     nbit <= nbit + 3'd1;

    always @(negedge sck) rx_sr <= {rx_sr[6:0], mosi};

    // MISO: registered in the pad on the rising edge (PIN_TYPE 0101 = registered output,
    // 01 = plain input, unused).  Rise k of a byte (nbit = k-1) launches bit 8-k.
    SB_IO #(.PIN_TYPE(6'b010101)) io_miso (
        .PACKAGE_PIN(miso),
        .OUTPUT_CLK(sck),
        .CLOCK_ENABLE(1'b1),
        .D_OUT_0(tx_byte[~nbit])
    );

    // ---- byte handoff into clk: toggle on the 8th fall + 3-flop sync, then the pulse ----
    wire sck_n = ~sck;
    cdc_pulse_payload #(.W(8)) rx_cdc (
        .src_clk(sck_n), .src_pulse(nbit == 3'd0 && !csn), .src_data(rx_sr),
        .dst_clk(clk), .dst_pulse(rx_valid), .dst_data(rx_byte)
    );

    // ---- CSn into clk ----
    reg [CS_SYNC-1:0] cs_s = {CS_SYNC{1'b1}};
    always @(posedge clk) cs_s <= {cs_s[CS_SYNC-2:0], csn};   // no rst: CSn is high at boot
    assign cs_active = ~cs_s[CS_SYNC-1];

endmodule
