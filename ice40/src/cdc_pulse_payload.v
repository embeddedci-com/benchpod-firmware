// ============================================================================
// cdc_pulse_payload.v — carry a 1-cycle PULSE, and the VALUE that belongs to it, from a
// source clock into a destination clock (gateware >= v34).
//
// THE RULE (AGENTS.md, "Clock-domain crossings"): the pulse that LATCHES a crossed value must
// not be the pulse that CONSUMES it.  Registers update with nonblocking assignment, so a
// consumer that reads the copy on the same edge the copy is written reads the PREVIOUS value.
// top_v2 did exactly that with START_DAC's period/divider until v33: every DAC start ran its
// first waveform pass at the previous start's length, through stale BRAM.
//
// Here the value is copied when the synchronizer edge is detected and the pulse leaves one
// dst_clk cycle later, so `dst_data` already holds this pulse's value when `dst_pulse` fires:
//
//   src_clk:  tgl ^= src_pulse
//   dst_clk:  s <= {s[1:0], tgl}      latch = s[2]^s[1] -> dst_data  <= src_data
//                                                       -> dst_pulse <= latch   (registered)
//
// BOTH outputs are flops.  An XOR-of-two-flops pulse output puts a LUT in front of every
// consumer's control chain: the v34 deep-image sweep found `dac_stop_48 = s[3]^s[2]` heading the
// clk48 critical path (stop -> DAC engine running/st/ph/bitc enables, 21 ns), costing ~3 MHz.
//
// Source-side contract:
//   * src_data holds its value for >= 4 dst_clk cycles after the src_clk edge that samples
//     src_pulse (the latch lands 3-4 dst edges after the toggle, depending on which dst edge
//     first catches it).  cmd_dispatch writes argument registers on the same edge that raises
//     the pulse and keeps them until the next command, which is milliseconds away.
//   * each toggle level lasts >= 2 dst_clk cycles, i.e. pulses are >= 1 src_clk cycle apart
//     for clk -> clk48 (clk = clk48/2); closer pulses merge.
// Outputs nobody reads cost nothing: an unconnected dst_pulse or a constant src_data prunes.
// tb_cdc_pulse_payload checks it at a synchronous 1:2 ratio and an asynchronous one.
// ============================================================================
module cdc_pulse_payload #(
    parameter         W    = 1,
    parameter [W-1:0] INIT = {W{1'b0}}
)(
    input  wire         src_clk,
    input  wire         src_pulse,
    input  wire [W-1:0] src_data,
    input  wire         dst_clk,
    output wire         dst_pulse,
    output reg  [W-1:0] dst_data = INIT
);
    reg       tgl = 1'b0;
    always @(posedge src_clk) if (src_pulse) tgl <= ~tgl;

    reg [2:0] s = 3'b000;
    always @(posedge dst_clk) s <= {s[1:0], tgl};

    wire latch = s[2] ^ s[1];
    always @(posedge dst_clk) if (latch) dst_data <= src_data;

    reg pulse_r = 1'b0;                  // same cycle as the old s[3]^s[2], but straight off a flop
    always @(posedge dst_clk) pulse_r <= latch;
    assign dst_pulse = pulse_r;
endmodule
