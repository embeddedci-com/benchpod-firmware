# iCE40 global buffers

The iCE40 UP5K has 8 global buffers (SB_GB). nextpnr reports all 8 used on every gateware image,
which reads as a hard limit. It is not one: only two of the eight carry a clock, and nextpnr fills
the other six with the resets and clock enables that have the highest fanout. This file lists the
nets per image and says which could be freed. Regenerate them from the `Promoting globals` lines of each
image's nextpnr log (`ice40/bench_pod_fpga_vbench_pod_<image>.asc.pnr.log`) after a gateware
change.

## How the eight are filled

1. `clk`, the 24 MHz logic clock: an explicit `SB_GB` in `top_v2.v` (`clk_gb`, clk48 divided by 2
   in a fabric flop). It does not appear in the "promoting" list because the design instantiates it.
2. `clk48`, the 48 MHz oscillator from its pad (`clk48$SB_IO_IN`): promoted by nextpnr.
3. Six more, promoted by nextpnr in order of fanout: the reset and clock-enable nets of the flops.
   nextpnr promotes a reset or enable once its fanout is high enough and a buffer is free, and stops
   when the eight are used.

So a new high-fanout reset or enable does not fail to place: it takes a buffer from the
lowest-fanout promoted net, or it stays on general routing if its fanout is lower than all of them.
A net that loses its buffer goes back to general routing, which costs routing resources and a few
ns on the paths it drives.

## The nets per image (v60)

From the v60 builds on their pinned seeds (loop 2, deep 24, emu 57). "Logic" is the driving
LUT's inputs (one level, in the synthesized netlist), which names the net that yosys's generated
net name hides. `rst` is the clk domain's power-on reset (`top_v2.v`); yosys names it after a flop
it merged with (`adc_cnt_nz...`, `adc_i.cnt...`).

| net | logic | loop | deep | emu |
|---|---|---|---|---|
| `clk` (24 MHz) | `SB_GB clk_gb` on `clkdiv` | yes | yes | yes |
| `clk48` | its pad | 748 | 396 | 296 |
| reset: `rst` | `rst` | 485 | 569 | 686 |
| reset: ADC producer restart | `rst`, capture `arm` | 92 | 92 | 92 |
| reset: UART proxy engine (`rst \|\| !armed`) | `rst`, `uart_armed` | - | 72 | 72 |
| reset: UART2 engine (`rst \|\| !armed`) | `rst`, `uart2_armed` | - | - | 68 |
| reset: DAC engine stream (clk48) | `dac_i.psram_mode`, `dac_i.run_i` | 173, 80 | - | - |
| reset: deep reader / DAC phase (clk48) | `rdr_i.run48`, `dac_i.ph` | - | 56 | - |
| enable: cmd_dispatch payload registers | dispatch `state` / `rx_valid`, `cs_active_d` | 72 | 72 | 72 |
| enable: the control loop's parameter crossing | `loop_arm_cdc.s` | 63 | - | - |
| enable: the LA capture packer | `la_pk_i.rst`, LA ring and capture state | - | 48 | 48 |

The numbers are fanouts; "-" means that image's eight buffers went to other nets (the net still
exists in the loop image's UART, for example, on general routing).

## Which could be freed

None of the six promoted resets and enables is a clock: nextpnr gives them a buffer because one
is free, and the same nets sit on general routing in the images where other nets won the buffers
(the UART proxy's reset in the loop image, for example). The two clocks need theirs.
If a later change needs a buffer for a new net with a higher fanout, nextpnr hands it one by
itself. If it needs one for a net with a lower fanout (a second clock, say), the candidates are, in
the order that costs least:

1. The UART engine resets (`rst || !armed`, fanout about 70): fold `!armed` into each engine's
   enables and keep only `rst` as the reset, or let them go to general routing (their logic runs at
   24 MHz, where every image has 2 to 4 MHz of margin).
2. The ADC producer's restart (`rst | arm`, fanout 92): the same, as an enable.
3. The clock enables (dispatch payload, LA packer, the loop's parameter crossing): plain enables,
   fine on general routing.
4. The clk48 DAC resets (loop, deep): they are on the 48 MHz domain, so moving them costs clk48
   margin, the scarcer one. Last.

`rst` itself (fanout about 500-700) and the two clocks should keep theirs.
