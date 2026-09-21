# nextpnr-ice40 pre-pack clock constraints (P7: two clock domains).
#
# Without these, nextpnr place & route targets a low default frequency and the
# reported fmax says nothing about whether the real clocks close.  Constrain each
# domain to its true rate so placement is timing-driven and the fmax report is
# meaningful:
#   clk48 — 48 MHz capture datapath (external oscillator on the global clock pad)
#   clk   — 24 MHz control plane (clk48/2 via SB_GB; cmd_dispatch caps it ~36 MHz)
#
# Applied only to the v2 build (see PREPACK in the Makefile); the v1 tops have no
# clk48.  Guarded so a missing net is a no-op rather than an error.
for _name, _freq in (("clk48", 48.0), ("clk", 24.0)):
    try:
        ctx.addClock(_name, _freq)
    except Exception as _e:
        print("clocks.py: skipping %s (%s)" % (_name, _e))
