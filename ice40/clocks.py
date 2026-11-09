# nextpnr-ice40 pre-pack clock constraints (P7: two clock domains).
#
# Without these, nextpnr place & route targets a low default frequency and the
# reported fmax says nothing about whether the real clocks close.  Constrain each
# domain to its true rate so placement is timing-driven and the fmax report is
# meaningful:
#   clk48 — 48 MHz capture datapath (external oscillator on the global clock pad)
#   clk   — 24 MHz control plane (clk48/2 via SB_GB; cmd_dispatch caps it ~36 MHz)
#
# Applied only to the v2 build (see PREPACK in the Makefile).  A missing net FAILS place & route:
# nextpnr would otherwise time that domain at its 12 MHz default and still report PASS.  yosys
# 0.70 renamed the 24 MHz net after one of its loads (adc_i.clk in deep, adc_cdc.src_clk in loop),
# which is why top_v2 keeps `wire clk`.
for _name, _freq in (("clk48", 48.0), ("clk", 24.0)):
    if _name not in ctx.nets:
        raise RuntimeError("clocks.py: no net named %r: the %.0f MHz clock would go unconstrained "
                           "(did synthesis rename it? keep the wire in top_v2.v)" % (_name, _freq))
    ctx.addClock(_name, _freq)
