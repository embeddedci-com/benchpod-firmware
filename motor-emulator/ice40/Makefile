# Motor & battery emulator gateware (iCE40UP5K-SG48I), open-source IceStorm toolchain.
#
#   make            synthesise, place and route, pack: build/motor_emulator.bin
#   make test       run every bench (each must print PASS; sim/run_vvp.sh enforces it)
#   make tb_<name>  run one bench (tb_pwm3, tb_sinc5, tb_i2c_probe, tb_sync_wd, tb_motor_model, tb_top)
#   make clean
#
# One clock (36 MHz, PLL from the 12 MHz oscillator). The bitstream is loaded by BenchPod over
# slave SPI (no configuration flash on the board); see README.md.

PROJECT ?= motor_emulator
BUILD    = build
SRC      = src/top.v src/spi_link.v src/pwm3.v src/sinc5.v src/i2c_probe.v src/sync_wd.v src/motor_model.v
INC      = src/emu_defs.vh src/sine1024.hex src/model_prog.hex src/model_data.hex
# Default: schematic rev 0.4 pinout (SYNC on pin 48 with readback, EE_WC_N on RGB1).
# REV03=1: the rev 0.3 pins (SYNC on RGB1, write-only: no stack watchdog input).
ifeq ($(REV03),1)
PCF      = emu_rev03.pcf
YDEFS    = -DREV03
PROJECT  = motor_emulator_rev03
else
PCF      = emu.pcf
YDEFS    =
endif
SEED    ?= 1
CELLS_SIM = $(shell yosys-config --datdir)/ice40/cells_sim.v
IVERILOG = iverilog -g2012 -Wall -I src
VVP      = sh sim/run_vvp.sh

all: $(BUILD)/$(PROJECT).bin

$(BUILD):
	mkdir -p $(BUILD)

# generated tables (*.hex is ignored repo-wide): sine, Q15, one revolution in 1024 entries
src/sine1024.hex:
	python3 -c "import math; print(''.join('%04x\\n' % (round(32767 * math.sin(2 * math.pi * k / 1024)) & 0xffff) for k in range(1024)), end='')" > $@

# the model program (tools/masm.py checks the pipeline spacing and pads with NOPs)
src/model_prog.hex src/model_data.hex: src/model.masm tools/masm.py
	python3 tools/masm.py src/model.masm src/model_prog.hex src/model_data.hex

$(BUILD)/$(PROJECT).json: $(SRC) $(INC) | $(BUILD)
	yosys -q -l $(BUILD)/yosys.log -p "read_verilog $(YDEFS) -I src $(SRC); synth_ice40 -dsp -top top -json $@"
	@# a signal driven from two always blocks simulates one way and synthesises another (yosys
	@# resolves the conflict to a constant): fail the build instead of shipping that
	@if grep -q "conflicting drivers\|Driver-driver conflict" $(BUILD)/yosys.log; then \
		grep "conflicting drivers\|Driver-driver conflict" $(BUILD)/yosys.log; rm -f $@; \
		echo "*** multiply-driven signals (see above)"; exit 1; fi

$(BUILD)/$(PROJECT).asc: $(BUILD)/$(PROJECT).json $(PCF) clocks.py
	nextpnr-ice40 --up5k --package sg48 --json $< --pcf $(PCF) --asc $@ \
		--pre-pack clocks.py --seed $(SEED) --threads 1 --log $(BUILD)/nextpnr.log
	@grep -E "Max frequency for clock|Device utilisation|ICESTORM_LC|ICESTORM_DSP|ICESTORM_RAM" $(BUILD)/nextpnr.log | tail -8

$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).asc
	icepack $< $@

# ---- benches
BENCHES = tb_pwm3 tb_sinc5 tb_i2c_probe tb_sync_wd tb_motor_model tb_top tb_top_rev03 tb_motor_model_gl tb_top_gl

tb_pwm3: | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v src/pwm3.v
	$(VVP) $(BUILD)/$@.vvp

tb_sinc5: | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v src/sinc5.v
	$(VVP) $(BUILD)/$@.vvp

tb_i2c_probe: | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v sim/i2c_ack_model.v src/i2c_probe.v
	$(VVP) $(BUILD)/$@.vvp

tb_motor_model: src/model_prog.hex src/sine1024.hex | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v src/motor_model.v
	$(VVP) $(BUILD)/$@.vvp

tb_sync_wd: | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v src/sync_wd.v
	$(VVP) $(BUILD)/$@.vvp

# whole chip: -DSIM bypasses the PLL (the bench drives 36 MHz on clk12); cells_sim.v models SB_IO
tb_top: src/model_prog.hex src/sine1024.hex | $(BUILD)
	iverilog -g2012 -I src -DSIM -DNO_ICE40_DEFAULT_ASSIGNMENTS -o $(BUILD)/$@.vvp \
		sim/$@.v sim/i2c_ack_model.v $(SRC) $(CELLS_SIM)
	$(VVP) $(BUILD)/$@.vvp

tb_top_rev03: src/model_prog.hex src/sine1024.hex | $(BUILD)
	iverilog -g2012 -I src -DSIM -DREV03 -DNO_ICE40_DEFAULT_ASSIGNMENTS -o $(BUILD)/$@.vvp \
		sim/tb_top.v sim/i2c_ack_model.v $(SRC) $(CELLS_SIM)
	$(VVP) $(BUILD)/$@.vvp

# gate level: the same benches on the yosys netlist (catches what RTL simulation hides: signals
# driven from two always blocks, glitching decoded outputs, inference surprises)
$(BUILD)/gl_top.v: $(SRC) $(INC) | $(BUILD)
	yosys -q -l $(BUILD)/gl_top.log -p "read_verilog -DSIM -I src $(SRC); synth_ice40 -dsp -top top; write_verilog -noattr $@"
$(BUILD)/gl_model.v: src/motor_model.v $(INC) | $(BUILD)
	yosys -q -l $(BUILD)/gl_model.log -p "read_verilog -I src src/motor_model.v; synth_ice40 -dsp -top motor_model; write_verilog -noattr $@"

tb_top_gl: $(BUILD)/gl_top.v
	iverilog -g2012 -I src -DSIM -DNO_ICE40_DEFAULT_ASSIGNMENTS -o $(BUILD)/$@.vvp \
		sim/tb_top.v sim/i2c_ack_model.v sim/tbuf_cell.v $(BUILD)/gl_top.v $(CELLS_SIM)
	$(VVP) $(BUILD)/$@.vvp

tb_motor_model_gl: $(BUILD)/gl_model.v
	iverilog -g2012 -I src -DNO_ICE40_DEFAULT_ASSIGNMENTS -o $(BUILD)/$@.vvp \
		sim/tb_motor_model.v $(BUILD)/gl_model.v $(CELLS_SIM)
	$(VVP) $(BUILD)/$@.vvp

test: $(BENCHES)

clean:
	rm -rf $(BUILD)

.PHONY: all test clean $(BENCHES)
