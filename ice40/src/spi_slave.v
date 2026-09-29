// ============================================================================
// spi_slave.v — SPI slave (mode 0) for the iCE40 signal-engine FPGA, clocked by SCK.
//
// Receives bytes from the STM32 on MOSI and presents them to cmd_dispatch as rx_byte + a
// 1-cycle rx_valid in the 24 MHz `clk` domain.  Shifts tx_byte out on MISO during the next
// byte.  CSn high resets the bit position (a byte cut short by CSn is dropped) and, through
// cs_active, resets cmd_dispatch's FSM.  Same ports as the old clk-sampled slave.
//
// The shift logic runs on SCK itself, so the link rate is no longer bounded by oversampling
// SCK with the 24 MHz clk (that capped it at ~2 MHz, see signal_engine.c):
//
//   posedge sck : rx_sr <= {rx_sr, mosi}; nbit <= nbit + 1       (CSn high: nbit = 0, async)
//                 8th rise: the byte is complete in rx_sr and toggles a cdc_pulse_payload,
//                 which copies rx_sr into rx_byte (clk) and then raises rx_valid.
//   negedge sck : fall k of a byte (k = 1..7) loads sh <= tx_byte[7 - k]; fall 8 sets `first`.
//                 MISO = first ? tx_byte[7] : sh, so bits 6..0 change on falling edges, half a
//                 period ahead of the master's rising-edge sample, launched from a flop.
//
// There is no tx shift register: tx_byte (a cmd_dispatch register in clk) is only written on
// rx_valid, so it is stable while a byte shifts out and `sh` samples it directly.  Bit 7 has no
// SCK edge to launch it (the SCK idles low between bytes), so MISO shows tx_byte[7] through
// the `first` mux: the master samples it at the byte's first rise, and cmd_dispatch writes
// tx_byte a few clk after the previous byte's 8th rise.
//
// INTER-BYTE GAP (the host's contract).  From the 8th SCK rise of byte N to the first SCK rise
// of byte N+1 the master must wait at least
//     5 clk (toggle catch <= 2 edges, latch, rx_valid, cmd_dispatch writes tx_byte)
//   + MISO path (tx_byte flop -> mux -> pad -> board -> STM32 setup, ~20 ns)
//   ~= 230 ns at clk = 24 MHz.
// That is also what keeps rx_sr stable until the crossing latches it (<= 4 clk).  On the STM32H5
// this is MIDI (idle SCK periods between frames): rise-to-rise = (1 + MIDI) * T_sck, so
//   15.6 MHz (/16): MIDI >= 3;  25 MHz: MIDI >= 5;  31.25 MHz (/8): MIDI >= 7  (+1 for margin).
// Without the gap the first bit of each reply byte is the previous tx_byte's MSB.
//
// CSn ordering.  cs_active reaches cmd_dispatch through CS_SYNC flops.  A byte's rx_valid is
// consumed <= 5 clk edges after its 8th SCK rise; cs_release (cs_active falling, seen one edge
// later) lands >= CS_SYNC + 1 edges after CSn rises.  CS_SYNC = 5 keeps the last byte of a
// transfer ahead of the FSM reset however soon after the last SCK edge CSn rises (its first
// catching edge can be one earlier than the toggle's when the toggle goes metastable).
//
// SCK is a clock net now (posedge and negedge flops).  The MISO pad is fed by one LUT (the
// `first` mux), not an SB_IO output register, because bit 7 has no launching edge.
// ============================================================================

module spi_slave #(
    parameter CS_SYNC = 5
)(
    input  wire        clk,        // 24 MHz system clock (SYS_CLK_MHZ)
    input  wire        rst,        // unused (kept for the port list): the SCK side resets on CSn

    // SPI bus (mode 0)
    input  wire        sck,
    input  wire        mosi,
    output wire        miso,
    input  wire        csn,

    // Parallel byte interface to cmd_dispatch
    output wire [7:0]  rx_byte,    // last received byte (held until the next rx_valid)
    output wire        rx_valid,   // 1-cycle strobe when rx_byte is fresh
    input  wire [7:0]  tx_byte,    // byte to shift out during the next byte
    output wire        cs_active   // !csn, synchronised (CS_SYNC flops)
);

    // ---- SCK domain ----
    reg [2:0] nbit = 3'd0;         // bits received in this byte (posedge)
    reg [7:0] rx_sr = 8'h00;
    reg       first = 1'b1;        // negedge: MISO shows tx_byte[7] (no SCK edge launches it)
    reg       sh = 1'b0;           // negedge: MISO bits 6..0, straight off a flop

    always @(posedge sck or posedge csn)
        if (csn) nbit <= 3'd0;
        else     nbit <= nbit + 3'd1;

    always @(posedge sck) rx_sr <= {rx_sr[6:0], mosi};

    // Fall k of a byte (nbit = k, k = 1..7) launches bit 7-k; fall 8 (nbit = 0) hands MISO
    // back to tx_byte[7] for the next byte.
    always @(negedge sck or posedge csn)
        if (csn) first <= 1'b1;
        else     first <= (nbit == 3'd0);

    always @(negedge sck) sh <= tx_byte[~nbit];

    assign miso = first ? tx_byte[7] : sh;

    // ---- byte handoff into clk: toggle + 3-flop sync, rx_sr copied, then the pulse ----
    cdc_pulse_payload #(.W(8)) rx_cdc (
        .src_clk(sck), .src_pulse(nbit == 3'd7), .src_data(rx_sr),
        .dst_clk(clk), .dst_pulse(rx_valid), .dst_data(rx_byte)
    );

    // ---- CSn into clk ----
    reg [CS_SYNC-1:0] cs_s = {CS_SYNC{1'b1}};
    always @(posedge clk) cs_s <= {cs_s[CS_SYNC-2:0], csn};   // no rst: CSn is high at boot
    assign cs_active = ~cs_s[CS_SYNC-1];

endmodule
