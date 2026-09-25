// synth/sb_mac16_keep.v — a blackbox with SB_MAC16's ports, used only during synthesis.
//
// yosys 0.65's ice40_dsp pass (run by `synth_ice40 -dsp`) treats EVERY SB_MAC16 as a
// multiplier to pack registers around, and rewrites an explicitly instantiated one: it tied
// dsp_counter's CLK to 0, zeroed its C/D load inputs and switched its output to the multiplier
// path, so the counter was dead in the netlist while RTL simulation still passed (v41 bring-up).
// The Makefile's synthesis therefore renames the explicit SB_MAC16 cells to this type for the
// `coarse` step (where ice40_dsp runs; the control loop's multipliers are still $mul there, so
// they are inferred as before) and renames them back afterwards.  check_dsp.py then asserts on
// the JSON that every dsp_counter cell kept its clock and configuration.
(* blackbox *)
module SB_MAC16_KEEP (
    input CLK, CE,
    input [15:0] C, A, B, D,
    input AHOLD, BHOLD, CHOLD, DHOLD,
    input IRSTTOP, IRSTBOT,
    input ORSTTOP, ORSTBOT,
    input OLOADTOP, OLOADBOT,
    input ADDSUBTOP, ADDSUBBOT,
    input OHOLDTOP, OHOLDBOT,
    input CI, ACCUMCI, SIGNEXTIN,
    output [31:0] O,
    output CO, ACCUMCO, SIGNEXTOUT
);
endmodule
