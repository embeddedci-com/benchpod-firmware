// ============================================================================
// sample_buf.v — dual-port BRAM wrapper for the waveform and capture buffers.
//
// Each buffer is 4096 x 8 bits.  Implementation uses inferred dual-port BRAM
// (yosys + nextpnr-ice40 will map to SB_RAM40_4K primitives).
//
// Waveform buffer:
//   - Write port (A): driven by cmd_dispatch on LOAD_WAVE
//   - Read  port (B): driven by dac_engine
//
// Capture buffer:
//   - Write port (A): driven by adc_engine
//   - Read  port (B): driven by cmd_dispatch on READ_CAPTURE
// ============================================================================

// The two ports have INDEPENDENT clocks (SB_RAM40_4K is a true dual-clock BRAM):
// the write port on `clk`, the read port on `b_clk`.  For the v2 DAC waveform this
// lets the DAC sequencer read on clk48 (48 MHz) while cmd_dispatch loads on clk
// (24 MHz); pass `b_clk = clk` for a same-clock buffer (v1 capture buffer, etc.).
module sample_buf #(
    parameter ADDR_W = 12,        // 4096 entries
    parameter DATA_W = 8
)(
    // Port A (write)
    input  wire                  clk,
    input  wire                  a_we,
    input  wire [ADDR_W-1:0]     a_addr,
    input  wire [DATA_W-1:0]     a_din,

    // Port B (read)
    input  wire                  b_clk,
    input  wire [ADDR_W-1:0]     b_addr,
    output reg  [DATA_W-1:0]     b_dout
);

    reg [DATA_W-1:0] mem [0:(1<<ADDR_W)-1];

    always @(posedge clk)   if (a_we) mem[a_addr] <= a_din;   // write port
    always @(posedge b_clk)           b_dout <= mem[b_addr];   // read port (own clock)

endmodule
