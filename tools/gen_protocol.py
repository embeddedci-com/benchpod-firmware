#!/usr/bin/env python3
"""Single source of truth for the bench-pod SPI command protocol.

OPCODES below is the ONE authoritative table.  This script generates:
  - ice40/src/cmd_opcodes.vh        Verilog  `localparam OP_<NAME> = 8'h..;`
  - rp2350/src/cmd_opcodes.h        C  `#define CMD_<NAME> 0x..`
  - stm32h563/src/cmd_opcodes.h     (same C header)
  - ice40/PROTOCOL.md               human-readable command table

Edit OPCODES, then run `python3 tools/gen_protocol.py` (or `make check-protocol`
in ice40/, which fails if the committed outputs are stale).  Never hand-edit the
generated files — the banner says so.

The gateware uses the OP_ prefix and the firmware the CMD_ prefix, but the bare
NAME and the value are shared, so the two can never disagree again.
"""
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# (NAME, value, payload, reply)  — payload/reply are doc-only.
OPCODES = [
    ("PING",            0x01, "—",                                              "0xA5"),
    ("VERSION",         0x02, "—",                                              "version byte"),
    ("STATUS",          0x03, "—",                                              "flag byte: bit0 DAC running, bit1 capture busy, bit2 capture done, bit3 step train busy, bit4 SWD armed, bit5 sticky capture overflow (dropped bytes, an overlapping arm, or from v43 a writer burst cut because the STM32 took the bus; cleared by the next arm), bit6 control loop tripped (v30), bit7 always 0 (a 1 means the iCE40 is not driving MISO)"),
    ("LOAD_WAVE",       0x10, "len(2) + N data bytes",                          "—"),
    ("START_DAC",       0x11, "period(2) + divider(2)",                         "— (shallow: loops the 4 KB LOAD_WAVE BRAM, <=2048 samples. A DAC sample takes max(divider, 3) + 51 clk48: v2 >=v34 floors the divider at 3 for the DAC8551's frame-gap timing (t9), earlier gateware at 0. v40+: the wire carries divider - 1 (the reload), and the gateware floors it at 2)"),
    ("STOP_DAC",        0x12, "—",                                              "—"),
    ("START_DAC_PSRAM", 0x13, "base(3) + count(3) + divider(2)",                "— (v2 >=v17; DEEP replay: iCE40 streams the waveform straight out of PSRAM, up to the full 8 MB / 4,194,304 16-bit samples; MCU stages it into PSRAM over XSPI first; divider as START_DAC)"),
    ("SET_DAC_STOP_AFTER", 0x14, "cycles(4)",                                    "— (v2 >=v21; arms the iCE40 to cut a concurrently-running DAC `cycles` of the 24 MHz clk after the NEXT capture's t0, so the captured window shows the DAC switch off; cycles=0 disarms; send BEFORE the capture-start command. v40+: one-shot, the capture's end clears it; until v39 it re-applied to every later capture)"),
    ("DAC_ARM_ON_CAPTURE", 0x19, "—",                                            "— (v2 >=v27; CO-TRIGGER: defer the NEXT DAC start (START_DAC / START_DAC_PSRAM) so it fires on the SAME hardware cycle as the next capture's t0 instead of immediately — DAC sample 0 == capture t0, sample-exact and jitter-free. The deep reader still prefills its FIFO at the (suppressed) start, so the first PSRAM sample pops at t0 with zero latency. Consumed by that one arm; STOP_DAC cancels a staged-but-uncaptured start. Sequence: [stage waveform] → DAC_ARM_ON_CAPTURE → START_DAC(_PSRAM) → SET_DAC_STOP_AFTER → CAPTURE/LA_CAPTURE.)"),
    ("DAC_PROBE",       0x16, "—",                                              "v(2) LE — the control loop's current DAC output (telemetry; pair with DAC_LOOP_IN_PROBE for the input it came from). ONLY meaningful on the loop image: the deep-replay image has no loop engine and returns a tied-off 0, indistinguishable from 'the loop is holding 0 V' — check FPGA_FEATURES bit0 first (the firmware refuses the probe otherwise)."),
    ("DAC_LOOP_SRC",    0x1A, "src(1: 0=ADC, 1=fixed, 2=sweep) + fixed(2) + step(2)", "— (v2 >=v29; selects WHERE THE CONTROL LOOP'S INPUT COMES FROM. 0 = the live ADC (closed loop, the pre-v29 behaviour and the power-on default). 1 = the `fixed` register (OPEN loop: the host holds one point of the curve so the DAC output can be metered against curve[fixed] with the ADC and analog front end out of the picture — the first bring-up test of a curve or output path). 2 = an internal accumulator that adds `step` to the input every control tick, mod 65536 (OPEN loop function of TIME: walks the whole curve at a deterministic rate, again with no ADC). `fixed` is also the sweep's START value. Legal whether or not the loop is armed — a RUNNING loop picks the new source/value up on its next tick, so the host can step through curve points without re-arming or re-uploading the curve. Registers persist across arms.)"),
    ("DAC_LOOP_IN_PROBE", 0x1B, "—",                                            "in(2) LE — the input value the control loop's LAST TICK actually indexed the curve with, whatever the source. In a fixed/sweep run there is no ADC in the path at all, so this — not ADC_PROBE — is the loop's input. (v2 >=v29; same image gate as DAC_PROBE.)"),
    ("FPGA_FEATURES",   0x18, "—",                                              "features(1) — which optional blocks THIS flash image carries: bit0=closed-loop DAC, bit1=deep-DAC-PSRAM-replay (mutually exclusive across warmboot images). Lets firmware advertise capabilities per running image instead of by version."),
    ("WARMBOOT",        0x17, "image(1: 0..3)",                                 "— (v2 >=v23; iCE40 SB_WARMBOOT: reconfigure to another gateware image stored in the config flash — e.g. slot0=closed-loop, slot1=deep-DAC-replay — WITHOUT a reflash. The FPGA reboots into `image` immediately; no SPI reply. Firmware waits for CDONE + re-reads the version. Every image carries this so you can switch back.)"),
    ("DAC_LOOP_INMAP",  0x1C, "in_zero(2) + in_gain(2, signed Q15) + in_trip(2) + flags(1: b0=map_en, b1=trip_en)", "— (v2 >=v30; AFFINE MAP FROM THE RAW INPUT COUNT ONTO THE CURVE INDEX, plus an over-range trip. The loop indexes the curve with the raw input value, which on an inverting/offset/wrapping analog front end means a real measurement window lands backwards inside a few percent of the curve. This makes the index idx = sat_0_2047(((in - in_zero) * in_gain) >> 15), where the subtract is 16-bit MODULAR — for a window under 32768 counts that is exactly the unwrapped distance, so an ADC that wraps mid-range costs nothing — and in_gain is SIGNED so an inverting front end is just a negative gain. Saturation is the physical clamp: below the window = index 0 (source unloaded), above = 2047 (past short-circuit). |in_gain| <= 1.0 (32767) is all that is useful: one curve entry per input count is the hardware limit. flags b0=0 restores the legacy idx = in>>5 exactly. b1 arms the trip: at an index >= in_trip the loop LATCHES tripped, forces the output to vmin until disarmed, and raises STATUS bit 6 — the guard for a loop driving a real pass device into a shorted DUT. Registers persist across arms; the map describes the BENCH, not the curve. Loop image only.)"),
    ("START_DAC_LOOP",  0x15, "k(2) + vmin(2) + vmax(2) + tick_div(2)",          "— (v2 >=v23; IN-FABRIC CONTROL LOOP: each tick it takes an INPUT (the live ADC by default; see DAC_LOOP_SRC 0x1A for the fixed/sweep open-loop sources, v29), indexes the LOAD_WAVE curve as a LUT (target=curve[in>>5]), damps v+=k*(target-v) [k Q15], clamps [vmin,vmax] and drives the DAC — a deterministic transfer function with no host in the control path (a solar-panel/MPPT emulator is one curve you can load). Load the curve with LOAD_WAVE first; STOP with STOP_DAC. Uses NO PSRAM so an LA capture can run alongside. PRESENT ONLY IN THE LOOP IMAGE (FPGA_FEATURES bit0): the deep-replay image ties the engine off and both images report the SAME version, so gate on FPGA_FEATURES, never on the version — arming on the wrong image otherwise looks like it worked and drives nothing. vmin must be <= vmax: the clamp tests vmin first, so an inverted window pins the output at vmin. k is used as k[14:0] (>32767 wraps to a small gain); tick_div < 8 outruns the pipelined tick.)"),
    ("START_CAPTURE",   0x20, "count(2) + divider(2)",                          "— RETIRED in v40 (ignored): send CAPTURE 0x31 with la_cnt=0. (16-bit shallow ADC. CAPTURE DIVIDERS (here, START_MEASURE cap_div, CAPTURE adc_div/la_div, LA_CAPTURE) are the sample PERIOD in 24 MHz clk: rate = 24 MHz / divider EXACTLY on >=v32; ADC floors it at 60, LA at 2. Gateware <=v31 sampled every divider+1 clk — firmware sends divider-1 there. v40+: the LA divider is period - 2 on the wire, and the ADC one is not floored in gateware — firmware floors it at 60.)"),
    ("ADC_PROBE",       0x21, "—",                                              "sample(2) LE — live ADC, no PSRAM/CDC"),
    # 0x22 READ_CAPTURE removed (v17): v2 reads captures back from PSRAM over XSPI,
    # never via a BRAM read-back opcode — the gateware S_READ_CAP/cap_raddr path was dead.
    ("CAPTURE_TEST",    0x23, "mode(1: 1=ramp self-test, 0=real ADC)",          "— (ramp self-test then CAPTURE/XSPI read-back)"),
    ("PSRAM_CS",        0x25, "drive(1: 1=force PSRAM /CS low, 0=release)",      "— (boot self-test: iCE40 reaches the /CE net?)"),
    ("START_MEASURE",   0x30, "count(2) + dac_div(2) + cap_div(2)",             "— RETIRED in v40 (ignored): DAC_ARM_ON_CAPTURE + START_DAC + CAPTURE (ADC only). (DAC preloaded; 16-bit count; cap_div is a capture divider — see START_CAPTURE; dac_div is not: a DAC sample takes max(dac_div, 3) + 51 clk48, see START_DAC)"),
    ("CAPTURE",         0x31, "adc_cnt(3)+adc_div(2)+la_cnt(3)+la_div(2)",      "— (v2; unified ADC+LA, one trigger, two PSRAM regions; both 24-bit deep; divider = sample period, see START_CAPTURE. v40+: the only capture arm; a producer with count 0 keeps its divider)"),
    ("SET_CAPTURE_BASES", 0x32, "la_base(3) + adc_base(3)",                     "— (v2 >=v22; latch the RUNTIME PSRAM byte-bases for the next CAPTURE — dynamic tri-capture zone allocation, firmware packs LA/ADC/DAC to fit; send BEFORE CAPTURE)"),
    ("SET_TRIGGER",     0x33, "channel(1: LA 0..11) + mode(1: 0=off, 1=rising, 2=falling, 3=high, 4=low) + flags(1: reserved, send 0)", "— (v2 >=v35; CAPTURE TRIGGER, gated start.  Persistent until changed; applies to every later capture arm (CAPTURE 0x31, LA_CAPTURE 0x69, START_CAPTURE 0x20).  With a mode set, the arm loads the producers but they start sampling only on the first clk cycle the condition holds on the synchronised LA levels; that cycle is t0, and the DAC co-trigger (DAC_ARM_ON_CAPTURE) fires and SET_DAC_STOP_AFTER counts from it.  Re-arming with count 0 aborts a waiting capture.  Mode 0 (and 5..7) = untriggered, identical to <=v34.  Firmware should send mode 0 after each triggered capture.  There is NO pre-trigger: a triggered capture's window is entirely POST-trigger, so the first sample is at t0.)"),
    ("TRIGGER_STATUS",  0x34, "—",                                              "status(1): bit0 = waiting for the trigger (armed, condition not seen yet), bit1 = a triggered capture fired since the last arm (v2 >=v35)"),
    ("GPIO_SET",        0x40, "channel + mode(0=low,1=high,2=hi-Z)",            "—"),
    ("GPIO_STEP",       0x41, "channel + steps(2) + delay_us(2)",               "— (v40+: the wire carries the half-phase in us minus 1)"),
    ("SET_LED",         0x42, "mask (bit0=green, bit1=yellow, bit2=red)",       "—"),
    ("GPIO_GET",        0x43, "—",                                              "levels(2) LE — live LA1..LA14 pin levels (bit n = LA n+1) through a 2-flop synchroniser, bits 15:14 = 0 (v2 >=v35)"),
    ("SWD_ARM",         0x50, "swclk_ch + swdio_ch + nreset_ch(ignored, send 0xFF)", "—"),
    ("SWD_FEED",        0x51, "len(2) + N remote_bitbang bytes",                "—"),
    ("SWD_READ",        0x52, "len(2)",                                         "N sample bytes"),
    ("SWD_DISARM",      0x53, "—",                                              "—"),
    # 0x54 SWD_STATUS removed (v17): never sent by v2 firmware (dead gateware decode).
    ("I2C_CONFIG",      0x60, "addr7+sda_ch+scl_ch+flags+trig_reg+busy_reg+busy_mask+conv(2)", "—"),
    ("I2C_DISABLE",     0x61, "—",                                              "—"),
    ("I2C_LOAD_REGS",   0x62, "start_addr + len(2) + N data bytes",             "—"),
    ("I2C_READ_REGS",   0x63, "start_addr + len(2)",                            "N bytes"),
    ("I2C_STATUS",      0x64, "—",                                              "7 status bytes"),
    # 0x65 I2C_LA_START / 0x66 I2C_LA_READ removed (v24): the on-FPGA I2C-bus LA sampler
    # (i2c_la_capture + la_capture_buf SPRAM) was retired to reclaim ~150 LC + an SPRAM
    # block.  The `sensor_la` feature now captures the two I2C pins via the general deep-LA
    # path (0x69) and the FIRMWARE re-packs them into the same 4-samples/byte layout, so the
    # server/python I2C decoder is byte-for-byte unchanged.  0x65/0x66 stay reserved.
    # 0x67 LA_START / 0x68 LA_READ removed (v17): the shallow-BRAM LA sampler is not
    # built in v2 (engine_block HAS_WIDE_LA=0); v2 uses the deep LA_CAPTURE 0x69 instead.
    ("LA_CAPTURE",      0x69, "count(3) + divider(2)  (24-bit samples; 2 B/sample)", "— RETIRED in v40 (ignored): send CAPTURE 0x31 with adc_cnt=0. (v2; deep PSRAM stream up to 8 MB, MCU reads back over XSPI; divider = sample period, see START_CAPTURE)"),
    ("UART_CONFIG",     0x70, "rx_ch+tx_ch+div(3)+flags",                       "— (div = 24 MHz clk per bit, 2 .. 2^18-1; v40+ takes it as sent, the firmware clamps)"),
    ("UART_DISABLE",    0x71, "—",                                              "—"),
    ("UART_WRITE",      0x72, "len(2) + N data bytes",                          "— (TX FIFO)"),
    ("UART_READ",       0x73, "len(2)",                                         "N bytes (RX FIFO)"),
    ("UART_STATUS",     0x74, "—",                                              "3 status bytes"),
]

# System logic-clock frequency — the SINGLE source for both the gateware
# (engine CLK_MHZ params, divider math) and the firmware (FPGA_HFOSC_HZ used to
# turn requested rates/baud into dividers).  They must match the silicon; keeping
# them in one place stops the gateware and firmware clocks from drifting apart.
SYS_CLK_MHZ = 24

# Dedicated ADC capture-domain clock (P7).  In the v2 (>=v7) gateware the capture
# datapath (ADC + PSRAM streaming) runs on its own 48 MHz clock; the firmware must
# turn requested ADC sample rates into dividers against THIS clock, not the 24 MHz
# control clock.  Single-sourced here so firmware and gateware can't drift.
CAPTURE_CLK_MHZ = 48

# DAC8551 waveform-sequencer clock.  In the v2 gateware (>= v13) the DAC engine runs
# on clk48 (48 MHz) for a ~2x update rate; the STM32 firmware turns requested DAC
# rates into dividers against THIS clock (see docs/ice40-48mhz-migration.md).  The
# v1/RP2350 gateware keeps its DAC on the 24 MHz HFOSC, so that header omits it.
DAC_CLK_MHZ = 48

# PSRAM regions (v2 gateware).  The 8 MB APS6404L is shared by three streams that
# now TIME-MULTIPLEX the one quad bus via psram_bus_arbiter (>=v18), so a deep DAC
# replay can run at the same time as an ADC/LA capture as long as their regions do
# not overlap.  Layout (docs/psram-concurrent-arbiter):
#
#     0x000000  +-----------------+  LA_BASE   — LA capture grows UP from the front
#               |  LA capture     |
#     0x400000  +-----------------+  ADC_BASE  — ADC capture (fixed base)
#               |  ADC capture    |
#   dac_base -> + - - - - - - - - +  <- DYNAMIC: TOTAL - dac_bytes (firmware-computed,
#               |  DAC replay     |            sent per-command in CMD_START_DAC_PSRAM;
#     0x800000  +-----------------+  TOTAL      the reader base is the one dynamic knob)
#
# Why this arrangement: the DAC replay base is ALREADY a per-command knob (the reader
# takes base_addr from the 0x13 payload), while the capture writer bases are compile-
# time gateware params — so we anchor the fixed thing (LA) at 0 and float the dynamic
# thing (DAC) down from the top.  Both LA and DAC are 2 B/sample, giving the clean
# budget max_LA_samples = (dac_base - LA_BASE)/2 = (TOTAL - dac_bytes)/2 when ADC is
# idle.  ADC+LA dual capture still uses the static 4 MB / 4 MB split below.  Nothing in
# the gateware bounds a stream to its region (addresses just increment / loop), so the
# firmware sample-count caps (static below + the dynamic guard in signal_engine) are
# the authoritative guard against a stream overrunning its neighbour.
PSRAM_TOTAL    = 0x800000   # APS6404L: 8 MB total
PSRAM_LA_BASE  = 0x000000   # LA region: FRONT of memory (grows up; ceiling = dac_base)
PSRAM_ADC_BASE = 0x400000   # ADC region: 4 MB (fixed base; ADC+LA dual uses [4MB,8MB))

# DEEP DAC replay (v2 >= v17 gateware, OP_START_DAC_PSRAM 0x13).  No fixed base: the
# waveform is placed at the TOP of PSRAM, base = TOTAL - dac_bytes, computed by the
# STM32 (which knows the total length from load_bin `total`) and sent in the 0x13
# payload.  DAC-only use can still fill all 4,194,304 sample slots (base -> 0).
PSRAM_DAC_TOP  = 0x800000   # DAC replay grows DOWN from here (= TOTAL)

BANNER_V = "// GENERATED by tools/gen_protocol.py — DO NOT EDIT.  Edit the table there.\n"
BANNER_C = "/* GENERATED by tools/gen_protocol.py — DO NOT EDIT.  Edit the table there. */\n"

def gen_vh():
    w = max(len(n) for n, *_ in OPCODES)
    lines = [BANNER_V, "// Bench-pod SPI command opcodes (gateware side).\n\n"]
    for name, val, *_ in OPCODES:
        lines.append(f"    localparam OP_{name.ljust(w)} = 8'h{val:02X};\n")
    return "".join(lines)

def gen_h():
    w = max(len(n) for n, *_ in OPCODES)
    lines = [BANNER_C,
             "/* Bench-pod SPI command opcodes (firmware side). */\n",
             "#ifndef CMD_OPCODES_H\n#define CMD_OPCODES_H\n\n"]
    for name, val, *_ in OPCODES:
        lines.append(f"#define CMD_{name.ljust(w)} 0x{val:02X}u\n")
    lines.append("\n#endif /* CMD_OPCODES_H */\n")
    return "".join(lines)

def gen_md():
    lines = ["<!-- GENERATED by tools/gen_protocol.py — DO NOT EDIT. -->\n",
             "# Bench-pod SPI command protocol\n\n",
             "| Opcode | Command | Payload (master→FPGA) | Reply (MISO) |\n",
             "|---|---|---|---|\n"]
    for name, val, payload, reply in OPCODES:
        lines.append(f"| 0x{val:02X} | {name} | {payload} | {reply} |\n")
    return "".join(lines)

def gen_sys_vh():
    # NB: no PSRAM_DAC_BASE — the DAC reader takes its base from the 0x13 command
    # (dynamic, top-anchored by firmware), so the gateware needs no DAC constant.
    return (BANNER_V + "// System logic-clock frequency + PSRAM capture regions (gateware side).\n\n"
            f"`define SYS_CLK_MHZ {SYS_CLK_MHZ}\n"
            f"`define PSRAM_LA_BASE  24'h{PSRAM_LA_BASE:06X}\n"
            f"`define PSRAM_ADC_BASE 24'h{PSRAM_ADC_BASE:06X}\n")

def gen_sys_h(dac48=False):
    # v2 (stm32h563) runs the DAC on clk48 (48 MHz) and computes DAC dividers against
    # DAC_CLK_HZ; v1 (rp2350) keeps the DAC on the 24 MHz HFOSC and omits DAC_CLK_HZ.
    hfosc_uses = "UART / I2C-LA / LA / ADC" if dac48 else "DAC / UART / I2C-LA"
    dac_block = (
        "/* DAC8551 waveform-sequencer clock.  Gateware >= 13 runs the DAC engine on clk48\n"
        "   (48 MHz) for a ~2x update rate (smoother output), so the DAC divider maths use\n"
        "   this, NOT FPGA_HFOSC_HZ.  Older gateware ran the DAC at FPGA_HFOSC_HZ (24 MHz).\n"
        "   (See docs/ice40-48mhz-migration.md.) */\n"
        f"#define DAC_CLK_HZ {DAC_CLK_MHZ}000000u\n\n"
    ) if dac48 else ""
    return (BANNER_C + "#ifndef FPGA_CONFIG_H\n#define FPGA_CONFIG_H\n\n"
            "/* iCE40 control-plane logic-clock frequency — match the gateware\n"
            f"   (see top*.v).  {hfosc_uses} dividers are computed against this. */\n"
            f"#define FPGA_HFOSC_HZ {SYS_CLK_MHZ}000000u\n\n"
            + dac_block +
            "/* Dedicated ADC capture-domain clock (>=v7 gateware, P7).  ADC sample\n"
            "   dividers are computed against this when the connected gateware reports\n"
            "   version >= 7; older gateware captures at FPGA_HFOSC_HZ. */\n"
            f"#define FPGA_CAPTURE_HZ {CAPTURE_CLK_MHZ}000000u\n\n"
            "/* PSRAM regions (>=v18: LA at the FRONT, ADC fixed at 4 MB, DAC replay\n"
            "   floating down from the TOP).  psram_bus_arbiter time-multiplexes the one\n"
            "   quad bus so DAC replay + an ADC/LA capture can run concurrently in disjoint\n"
            "   regions.  Read each capture region back over XSPI at these bases. */\n"
            f"#define FPGA_PSRAM_LA_BASE  0x{PSRAM_LA_BASE:06X}u   /* LA: front of memory   */\n"
            f"#define FPGA_PSRAM_ADC_BASE 0x{PSRAM_ADC_BASE:06X}u   /* ADC: fixed 4 MB base   */\n"
            f"#define FPGA_PSRAM_TOTAL    0x{PSRAM_TOTAL:06X}u   /* 8 MB APS6404L          */\n"
            f"#define FPGA_PSRAM_DAC_TOP  0x{PSRAM_DAC_TOP:06X}u   /* DAC grows DOWN from here */\n\n"
            "/* STATIC per-region caps (2 B/sample) for ADC+LA DUAL capture (no DAC resident):\n"
            "   LA lives in [LA_BASE, ADC_BASE), ADC in [ADC_BASE, TOTAL) — each 4 MB / 2 M\n"
            "   samples.  These are the authoritative guard for the dual-capture case; the\n"
            "   DYNAMIC single-capture + DAC-resident caps are computed in signal_engine.c. */\n"
            f"#define FPGA_LA_PSRAM_MAX_SAMPLES ((FPGA_PSRAM_ADC_BASE - FPGA_PSRAM_LA_BASE) / 2u)\n"
            f"#define FPGA_ADC_CAP_MAX_SAMPLES  ((FPGA_PSRAM_TOTAL    - FPGA_PSRAM_ADC_BASE) / 2u)\n\n"
            "/* DEEP DAC replay (>=v17 gateware, CMD_START_DAC_PSRAM 0x13).  base is TOP-\n"
            "   anchored: FPGA_DAC_BASE_FOR(bytes) = TOTAL - bytes.  DAC-only use can still\n"
            "   span the whole chip (base -> 0).  When a waveform is resident the LA/ADC caps\n"
            "   shrink to keep the capture below dac_base (see signal_engine.c). */\n"
            f"#define FPGA_DAC_REPLAY_MAX_SAMPLES (FPGA_PSRAM_TOTAL / 2u)\n"
            f"#define FPGA_DAC_BASE_FOR(bytes)    ((uint32_t)FPGA_PSRAM_DAC_TOP - (uint32_t)(bytes))\n"
            "\n#endif /* FPGA_CONFIG_H */\n")

TARGETS = {
    "ice40/src/cmd_opcodes.vh":      gen_vh,
    "ice40/src/sys_config.vh":       gen_sys_vh,
    "stm32h563/src/cmd_opcodes.h":   gen_h,
    "stm32h563/src/fpga_config.h":   lambda: gen_sys_h(dac48=True),
    "ice40/PROTOCOL.md":             gen_md,
}

def main():
    check = "--check" in sys.argv
    stale = []
    for rel, fn in TARGETS.items():
        path = os.path.join(REPO, rel)
        want = fn()
        have = open(path).read() if os.path.exists(path) else None
        if check:
            if have != want:
                stale.append(rel)
        else:
            with open(path, "w") as f:
                f.write(want)
            print(f"wrote {rel}")
    if check:
        if stale:
            sys.stderr.write("STALE (run tools/gen_protocol.py): " + ", ".join(stale) + "\n")
            return 1
        print("protocol files up to date")
    return 0

if __name__ == "__main__":
    sys.exit(main())
