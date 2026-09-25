// ============================================================================
// psram_pads.v — the shared quad-SPI PSRAM pads: master mux + a one-clk48 output retime (v41).
//
// Two masters share the APS6404L bus: the capture writer (psram_dual_writer) and, in the deep
// image, the replay reader (dac_psram_reader).  `replay` (the reader's clk48 CS-low flop) picks
// the reader while it is mid-burst; otherwise the writer (idle: CS high, OE low).  `bus_own`
// (the STM32 owns the bus) tristates everything immediately, and `selftest` drives the boot
// /CE-net self-test pattern.
//
// Why the retime.  SCLK is a DDR pad: during clk48 cycle k it shows D_OUT_0 from cycle k-1 in
// the clk48-high half and D_OUT_1 from cycle k in the low half, so a write clock rises mid-eye.
// But the pad captures D_OUT_1 on the clk48 FALLING edge, so the path from the masters' rising-
// edge registers through the mux LUT into D_OUT_1 had only HALF a period (10.4 ns).  That path
// capped the deep image near 51 MHz and moved with every placement: the reason each gateware
// change needed a fresh seed hunt.
//
// Every PSRAM output now leaves exactly one clk48 cycle later, so the waveforms are bit-identical
// to v40's, just 20.8 ns later, and nothing in fabric has a half-cycle path:
//   * CS and IO data: the pads' own output registers (PIN_TYPE registered output; no logic).
//   * IO output-enable: its per-cycle part (which master drives) through one flop.  The bus_own
//     release and the self-test pattern stay immediate, so the STM32 gets the bus back as before.
//   * SCLK: F0/F1 register the mux; D_OUT_0/D_OUT_1 are fed straight from them (no LUT), so
//     D_OUT_1's falling-edge capture sees a flop output with only a short route.
// The reader samples its returning nibbles one clk48 later to match (dac_psram_reader PAD_PIPE=1);
// the writer needs nothing (the PSRAM samples on the retimed SCLK).  tb_dac_psram_skew and
// tb_psram_arbiter drive both masters through THIS module and the SB_IO models.
// ============================================================================
module psram_pads (
    input  wire       clk48,
    input  wire       bus_own,        // 1 = the STM32 owns the bus: tristate (raw, immediate)
    input  wire       selftest,       // boot /CE-net self-test: CS low, IO 0011, SCLK high
    input  wire       replay,         // 1 = the reader is mid-burst (its clk48 CS-low flop)
    // reader (deep replay)
    input  wire [3:0] rd_io_o,
    input  wire       rd_io_oe,
    input  wire       rd_cs,
    input  wire       rd_sclk,        // level: drives both DDR halves
    output wire [3:0] rd_io_i,
    // capture writer
    input  wire [3:0] ps_io_o,
    input  wire       ps_io_oe,
    input  wire       ps_cs,
    input  wire       ps_sclk_d1,     // DDR low half; the high half is 0 (SCLK rises mid-eye)
    // pads
    inout  wire       psram_sclk,
    inout  wire       psram_cs,
    inout  wire       psram_io0,
    inout  wire       psram_io1,
    inout  wire       psram_io2,
    inout  wire       psram_io3
);
    wire       drive = ~bus_own;

    // cycle-k values, exactly v40's mux
    wire       oe_k  = replay ? rd_io_oe : ps_io_oe;
    wire [3:0] dat_k = selftest ? 4'b0011 : (replay ? rd_io_o : ps_io_o);
    wire       cs_k  = selftest ? 1'b0    : (replay ? rd_cs   : ps_cs);
    wire       d0_k  = selftest ? 1'b1    : (replay ? rd_sclk : 1'b0);
    wire       d1_k  = selftest ? 1'b1    : (replay ? rd_sclk : ps_sclk_d1);

    // one-clk48 retime (CS/data retime in their pads' own registers below)
    reg        oe_q = 1'b0, f0 = 1'b0, f1 = 1'b0;
    always @(posedge clk48) begin
        oe_q <= oe_k;
        f0   <= d0_k;
        f1   <= d1_k;
    end
    wire       io_oe = selftest | (drive & oe_q);

    // SCLK: DDR (PIN_TYPE 1000_00), OE = drive.  High half = D_OUT_0 captured at the rising edge,
    // low half = D_OUT_1 captured at the falling edge; both now come straight from flops.
    SB_IO #(.PIN_TYPE(6'b100000), .PULLUP(1'b0)) io_sclk_i (
        .PACKAGE_PIN(psram_sclk), .OUTPUT_ENABLE(drive), .OUTPUT_CLK(clk48),
        .D_OUT_0(f0), .D_OUT_1(f1));
    // CS and IO: registered output (PIN_TYPE[3:2] = 01), combinational OE (PIN_TYPE[5:4] = 10),
    // unregistered input (PIN_TYPE[1:0] = 01): 6'b1001_01.
    SB_IO #(.PIN_TYPE(6'b100101), .PULLUP(1'b0)) io_cs_i (
        .PACKAGE_PIN(psram_cs),  .OUTPUT_ENABLE(drive), .OUTPUT_CLK(clk48), .D_OUT_0(cs_k));
    SB_IO #(.PIN_TYPE(6'b100101), .PULLUP(1'b0)) io_d0_i (
        .PACKAGE_PIN(psram_io0), .OUTPUT_ENABLE(io_oe), .OUTPUT_CLK(clk48), .D_OUT_0(dat_k[0]), .D_IN_0(rd_io_i[0]));
    SB_IO #(.PIN_TYPE(6'b100101), .PULLUP(1'b0)) io_d1_i (
        .PACKAGE_PIN(psram_io1), .OUTPUT_ENABLE(io_oe), .OUTPUT_CLK(clk48), .D_OUT_0(dat_k[1]), .D_IN_0(rd_io_i[1]));
    SB_IO #(.PIN_TYPE(6'b100101), .PULLUP(1'b0)) io_d2_i (
        .PACKAGE_PIN(psram_io2), .OUTPUT_ENABLE(io_oe), .OUTPUT_CLK(clk48), .D_OUT_0(dat_k[2]), .D_IN_0(rd_io_i[2]));
    SB_IO #(.PIN_TYPE(6'b100101), .PULLUP(1'b0)) io_d3_i (
        .PACKAGE_PIN(psram_io3), .OUTPUT_ENABLE(io_oe), .OUTPUT_CLK(clk48), .D_OUT_0(dat_k[3]), .D_IN_0(rd_io_i[3]));
endmodule
