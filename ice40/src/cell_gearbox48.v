// ============================================================================
// cell_gearbox48.v — the clk48 receive half of the clk -> clk48 "cell" gearbox used by
// both PSRAM masters: psram_dual_writer (capture drain) and dac_psram_reader (deep replay).
//
// The clk-domain FSM presents a cell (the nibbles + bus controls for its next PSRAM bus
// slot) and flips `tgl` once per cell.  clk is clk48/2 from the same source (an integer,
// phase-locked ratio, NOT the async class that caused 0x5555), and a cell is held for at
// least one whole clk period, so a plain clk48 capture of it has about one clk48 of setup.
// This side captures `tgl` on every clk48 edge, flags the edge where it differs from that
// capture (`new_cell`), and on that edge latches the whole cell into `cell_l`.  The
// serializer drives its first output straight from `cell_in` on the new_cell cycle and the
// rest from `cell_l`, so it never reads the clk domain again before the next toggle (a
// later nibble can never race the next clk edge, the one race that would depend on skew).
//
//   clr  sync clear of cell_l to CLR_VAL (priority over the latch); tie 0 if unused.
//   en   latch allowed on a new cell; tie 1 if unconditional.
// ============================================================================
module cell_gearbox48 #(
    parameter integer        W       = 8,
    parameter        [W-1:0] CLR_VAL = {W{1'b0}}
)(
    input  wire          clk48,
    input  wire          tgl,        // clk domain: flips once per cell
    input  wire [W-1:0]  cell_in,    // clk domain: stable while tgl is unchanged
    input  wire          clr,
    input  wire          en,
    output wire          new_cell,   // a new cell is on `cell_in` this clk48 cycle
    output reg  [W-1:0]  cell_l      // the last cell latched
);
    reg tgl_m = 1'b0;                // last-cycle capture of tgl
    always @(posedge clk48) tgl_m <= tgl;
    assign new_cell = (tgl != tgl_m);

    always @(posedge clk48)
        if (clr)                 cell_l <= CLR_VAL;
        else if (en && new_cell) cell_l <= cell_in;
endmodule
