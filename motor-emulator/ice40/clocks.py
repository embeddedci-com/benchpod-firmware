# nextpnr-ice40 pre-pack clock constraint: the whole design runs on one 36 MHz clock from the PLL.
try:
    ctx.addClock("clk", 36.0)
except Exception as _e:
    print("clocks.py: skipping clk (%s)" % _e)
