# nextpnr-ice40 pre-pack (opt-in, make PIN_CLKDIV=1): lock the `clk` divider flop and its SB_GB.
#
# `clk` (24 MHz) is the clk48/2 divider flop (net `clkdiv`) through an SB_GB.  Unpinned, the
# placer puts that flop anywhere in the fabric (the net also feeds the la48 sampler clock
# enables), so clk's lag behind clk48 moves ~4 ns from seed to seed.  Pinning the flop to
# X13/Y30/lc0 and the SB_GB to X13/Y31 makes the routed clkdiv -> SB_GB delay a constant ~1.1 ns.
# See docs/design/ice40-seed-history.md ("Pinning the clk divider") and synth/clkdiv_lag.py.
#
# The BELs can be overridden with PIN_DFF_BEL / PIN_GB_BEL.  Any surprise raises, so a netlist
# change that renames the net or the cells fails the build instead of silently pinning nothing.
import os

DFF_BEL = os.environ.get("PIN_DFF_BEL", "X13/Y30/lc0")
GB_BEL = os.environ.get("PIN_GB_BEL", "X13/Y31/gb")


def _fail(msg):
    raise RuntimeError("pin_clkdiv.py: " + msg)


def _setbel(cell, bel):
    try:
        cell.setAttr("BEL", bel)
    except Exception:
        cell.attrs["BEL"] = bel


if "clkdiv" not in ctx.nets:
    _fail("net 'clkdiv' not found (renamed in the RTL?)")
_net = ctx.nets["clkdiv"]
if _net.driver.cell is None:
    _fail("net 'clkdiv' has no driver")
_dff = _net.driver.cell
if not str(_dff.type).startswith("SB_DFF"):
    _fail("clkdiv is driven by %s, expected an SB_DFF*" % _dff.type)
_gbs = [u.cell for u in _net.users if u.cell.type == "SB_GB"]
if len(_gbs) != 1:
    _fail("clkdiv drives %d SB_GB cells, expected exactly 1" % len(_gbs))

_setbel(_dff, DFF_BEL)
_setbel(_gbs[0], GB_BEL)
print("pin_clkdiv.py: %s (%s) -> %s, %s -> %s" % (_dff.name, _dff.type, DFF_BEL, _gbs[0].name, GB_BEL))
