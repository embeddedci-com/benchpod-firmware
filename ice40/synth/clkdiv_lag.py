# nextpnr-ice40 post-route hook: routed delay of the clkdiv net, divider flop -> SB_GB input.
#
# clk is SB_GB(clkdiv).  nextpnr models the global network as zero delay and the flop's
# clk-to-Q as a fixed cell delay, so the part of clk's lag behind clk48 that depends on
# placement is this routed net.  It is not in --report: the SB_GB input is no timing endpoint.
# Not part of any build; run it by hand on a placed seed (from ice40/):
#
#   LAG_OUT=lag.txt nextpnr-ice40 --up5k --package sg48 --pcf vbench_pod.pcf --pcf-allow-unconstrained \
#     --pre-pack clocks.py [--pre-pack pin_clkdiv.py] --threads 1 --seed 12 \
#     --json bench_pod_fpga_vbench_pod_loop.json --post-route synth/clkdiv_lag.py
#
# LAG_OUT defaults to stdout.  Appends "route_ns <ns> hops <n>" plus the pips of the route.
# With pin_clkdiv.py the route is 2 pips, ~1.10 ns; without it 1.1 to 9.2 ns depending on seed.
import os

out = os.environ.get("LAG_OUT", "/dev/stdout")
lines = []


def ns(dq):
    try:
        return ctx.getDelayNS(dq.maxDelay())
    except Exception:
        try:
            return ctx.getDelayNS(dq)
        except Exception:
            return float("nan")


net = ctx.nets["clkdiv"]
drv = net.driver
gb = [u for u in net.users if u.cell.type == "SB_GB"][0]
lines.append("driver_bel %s gb_bel %s" % (drv.cell.bel, gb.cell.bel))
wire = ctx.getBelPinWire(gb.cell.bel, gb.port)
total = 0.0
hops = []
for _ in range(200):
    pip = net.wires[wire].pip
    if pip is None or str(pip) in ("", "None"):
        break
    total += ns(ctx.getPipDelay(pip))
    hops.append(str(ctx.getPipName(pip)) if hasattr(ctx, "getPipName") else str(pip))
    wire = ctx.getPipSrcWire(pip)
lines.append("route_ns %.3f hops %d" % (total, len(hops)))
lines.append("path " + " | ".join(reversed(hops)))
with open(out, "a") as f:
    f.write("\n".join(lines) + "\n")
