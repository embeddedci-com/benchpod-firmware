// ============================================================================
// i2c_regfile.v — 256-byte register memory for the emulated I2C sensor.
//
// This is the "register image" the RP2350B fills and the DUT reads/writes over
// the emulated I2C bus.  It is deliberately sensor-agnostic: it holds whatever
// bytes the orchestrator loads (chip ID, calibration, measurement bytes, …).
// All sensor-specific knowledge lives on the RP2350B.
//
// Two access sides share one inferred memory (1-write / 2-read):
//   I2C side  (i2c_target)   — reads served to the DUT, writes captured from it
//   SPI side  (cmd_dispatch) — load the image (I2C_LOAD_REGS) and read it back
//                              for debugging (I2C_READ_REGS)
//
// Write port is shared with I2C priority.  In practice the two masters never
// write at the same instant: the DUT bus runs at ≤400 kHz and the RP only
// loads/reads between transactions, so a single muxed write port is plenty and
// keeps this to one small BRAM region.  yosys replicates the read port into a
// second block — both are a fraction of an SB_RAM40_4K, well within budget.
// ============================================================================

module i2c_regfile (
    input  wire        clk,

    // ---- I2C target side ----
    input  wire        i2c_we,       // 1-cycle write strobe (DUT wrote a byte)
    input  wire [7:0]  i2c_waddr,
    input  wire [7:0]  i2c_wdata,
    input  wire [7:0]  i2c_raddr,    // = register pointer; held while serving reads
    output reg  [7:0]  i2c_rdata,

    // ---- SPI / cmd_dispatch side ----
    input  wire        spi_we,       // I2C_LOAD_REGS write strobe
    input  wire [7:0]  spi_waddr,
    input  wire [7:0]  spi_wdata,
    input  wire [7:0]  spi_raddr,    // I2C_READ_REGS read pointer
    output reg  [7:0]  spi_rdata
);

    reg [7:0] mem [0:255];

    // Single write port, I2C wins if both strobe in the same cycle.
    wire        we    = i2c_we | spi_we;
    wire [7:0]  waddr = i2c_we ? i2c_waddr : spi_waddr;
    wire [7:0]  wdata = i2c_we ? i2c_wdata : spi_wdata;

    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        i2c_rdata <= mem[i2c_raddr];
        spi_rdata <= mem[spi_raddr];
    end

endmodule
