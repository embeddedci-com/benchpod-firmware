# Motor & battery emulator gateware (ECP5 LFE5U-25F-6BG256C), open-source flow:
# yosys, nextpnr-ecp5, prjtrellis (ecppack). The tools come from the OSS CAD Suite.
#
#   make            synthesise, place and route, pack: build/motor_emulator.bit
#   make test       run every bench (each must print PASS; sim/run_vvp.sh enforces it)
#   make tb_<name>  run one bench (tb_pwm3, tb_sinc5, tb_i2c_ctl, tb_sync_wd, tb_motor_model,
#                   tb_encoder, tb_top, and the gate-level tb_motor_model_gl, tb_top_gl)
#   make SEED=n     another placement seed
#   make clean
#
# One clock (36 MHz, PLL from the 12 MHz oscillator). The bitstream is loaded by BenchPod over
# slave SPI (no configuration flash on the board); see README.md.

TOOLS   ?= /opt/oss-cad-suite/bin
YOSYS   ?= $(TOOLS)/yosys
NEXTPNR ?= $(TOOLS)/nextpnr-ecp5
ECPPACK ?= $(TOOLS)/ecppack
PROJECT  = motor_emulator
BUILD    = build
SRC      = src/top.v src/spi_link.v src/pwm3.v src/sinc5.v src/i2c_ctl.v src/protect.v src/battery.v src/logbuf.v src/sync_wd.v src/motor_model.v src/encoder.v
INC      = src/emu_defs.vh src/sine1024.hex src/model_prog.hex src/model_data.hex
LPF      = emu.lpf
SEED    ?= 1
CELLS_DIR = $(shell $(YOSYS)-config --datdir)/ecp5
CELLS_SIM = $(CELLS_DIR)/cells_sim.v
IVERILOG = iverilog -g2012 -Wall -I src
VVP      = sh sim/run_vvp.sh

all: $(BUILD)/$(PROJECT).bit

$(BUILD):
	mkdir -p $(BUILD)

# generated tables (*.hex is ignored repo-wide): sine, Q15, one revolution in 1024 entries
src/sine1024.hex:
	python3 -c "import math; print(''.join('%04x\\n' % (round(32767 * math.sin(2 * math.pi * k / 1024)) & 0xffff) for k in range(1024)), end='')" > $@

# the model program (tools/masm.py checks the pipeline spacing and pads with NOPs)
src/model_prog.hex src/model_data.hex: src/model.masm tools/masm.py
	python3 tools/masm.py src/model.masm src/model_prog.hex src/model_data.hex

$(BUILD)/$(PROJECT).json: $(SRC) $(INC) | $(BUILD)
	$(YOSYS) -q -l $(BUILD)/yosys.log -p "read_verilog -I src $(SRC); synth_ecp5 -top top -json $@"
	@# a signal driven from two always blocks simulates one way and synthesises another (yosys
	@# resolves the conflict to a constant): fail the build instead of shipping that
	@if grep -q "conflicting drivers\|Driver-driver conflict" $(BUILD)/yosys.log; then \
		grep "conflicting drivers\|Driver-driver conflict" $(BUILD)/yosys.log; rm -f $@; \
		echo "*** multiply-driven signals (see above)"; exit 1; fi

$(BUILD)/$(PROJECT).config: $(BUILD)/$(PROJECT).json $(LPF)
	$(NEXTPNR) --25k --package CABGA256 --speed 6 --json $< --lpf $(LPF) --textcfg $@ \
		--seed $(SEED) --threads 1 --log $(BUILD)/nextpnr.log
	@grep -E "Max frequency for clock|TRELLIS_COMB|TRELLIS_FF|MULT18X18D|DP16KD" $(BUILD)/nextpnr.log | tail -6

$(BUILD)/$(PROJECT).bit: $(BUILD)/$(PROJECT).config
	$(ECPPACK) --compress --input $< --bit $@

# ---- benches
BENCHES = tb_pwm3 tb_sinc5 tb_sinc5_256 tb_i2c_ctl tb_sync_wd tb_motor_model tb_encoder tb_protect tb_battery tb_top tb_motor_model_gl tb_protect_gl tb_battery_gl tb_i2c_ctl_gl tb_encoder_gl tb_top_gl

tb_pwm3: | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v src/pwm3.v
	$(VVP) $(BUILD)/$@.vvp

tb_sinc5: | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v src/sinc5.v
	$(VVP) $(BUILD)/$@.vvp

tb_sinc5_256: | $(BUILD)
	$(IVERILOG) -DSINC_L=8 -o $(BUILD)/$@.vvp sim/tb_sinc5.v src/sinc5.v
	$(VVP) $(BUILD)/$@.vvp

tb_i2c_ctl: | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v sim/i2c_dev_model.v src/i2c_ctl.v
	$(VVP) $(BUILD)/$@.vvp

tb_motor_model: src/model_prog.hex src/sine1024.hex | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v src/motor_model.v
	$(VVP) $(BUILD)/$@.vvp

tb_encoder: | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v src/encoder.v
	$(VVP) $(BUILD)/$@.vvp

tb_battery: | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v src/battery.v
	$(VVP) $(BUILD)/$@.vvp

tb_protect: | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v src/protect.v
	$(VVP) $(BUILD)/$@.vvp

tb_sync_wd: | $(BUILD)
	$(IVERILOG) -o $(BUILD)/$@.vvp sim/$@.v src/sync_wd.v
	$(VVP) $(BUILD)/$@.vvp

# whole chip: -DSIM bypasses the PLL (the bench drives 36 MHz on clk12)
tb_top: src/model_prog.hex src/sine1024.hex | $(BUILD)
	$(IVERILOG) -DSIM -o $(BUILD)/$@.vvp sim/$@.v sim/i2c_dev_model.v $(SRC)
	$(VVP) $(BUILD)/$@.vvp

# gate level: the same benches on the yosys netlist (catches what RTL simulation hides: signals
# driven from two always blocks, glitching decoded outputs, inference surprises). -nobram: yosys's
# ECP5 library has no behaviour for DP16KD (parameters only, its outputs read Z), so memories go
# to LUT RAM here; block-RAM mapping itself is not covered. sim/mult18x18d_sim.v stands in for
# the multiplier, which the library has only as a black box.
$(BUILD)/gl_top.v: $(SRC) $(INC) | $(BUILD)
	$(YOSYS) -q -l $(BUILD)/gl_top.log -p "read_verilog -DSIM -I src $(SRC); synth_ecp5 -nobram -top top; write_verilog -noattr $@"
$(BUILD)/gl_model.v: src/motor_model.v $(INC) | $(BUILD)
	$(YOSYS) -q -l $(BUILD)/gl_model.log -p "read_verilog -I src src/motor_model.v; synth_ecp5 -nobram -top motor_model; write_verilog -noattr $@"

tb_top_gl: $(BUILD)/gl_top.v
	iverilog -g2012 -I src -I $(CELLS_DIR) -DNO_INCLUDES -DSIM -DGL -o $(BUILD)/$@.vvp \
		sim/tb_top.v sim/i2c_dev_model.v sim/mult18x18d_sim.v sim/tbuf_cell.v $(BUILD)/gl_top.v $(CELLS_SIM)
	$(VVP) $(BUILD)/$@.vvp

tb_motor_model_gl: $(BUILD)/gl_model.v
	iverilog -g2012 -I src -I $(CELLS_DIR) -DNO_INCLUDES -o $(BUILD)/$@.vvp \
		sim/tb_motor_model.v sim/mult18x18d_sim.v $(BUILD)/gl_model.v $(CELLS_SIM)
	$(VVP) $(BUILD)/$@.vvp

# gate-level unit benches for the newer blocks (tb_top_gl skips their long steps)
$(BUILD)/gl_%.v: src/%.v | $(BUILD)
	$(YOSYS) -q -l $(BUILD)/gl_$*.log -p "read_verilog -I src src/$*.v; synth_ecp5 -nobram -top $*; write_verilog -noattr $@"
tb_protect_gl: $(BUILD)/gl_protect.v
	iverilog -g2012 -I src -I $(CELLS_DIR) -DNO_INCLUDES -o $(BUILD)/$@.vvp sim/tb_protect.v sim/mult18x18d_sim.v $< $(CELLS_SIM)
	$(VVP) $(BUILD)/$@.vvp
tb_battery_gl: $(BUILD)/gl_battery.v
	iverilog -g2012 -I src -I $(CELLS_DIR) -DNO_INCLUDES -o $(BUILD)/$@.vvp sim/tb_battery.v sim/mult18x18d_sim.v $< $(CELLS_SIM)
	$(VVP) $(BUILD)/$@.vvp
tb_i2c_ctl_gl: $(BUILD)/gl_i2c_ctl.v
	iverilog -g2012 -I src -I $(CELLS_DIR) -DNO_INCLUDES -o $(BUILD)/$@.vvp sim/tb_i2c_ctl.v sim/i2c_dev_model.v sim/mult18x18d_sim.v $< $(CELLS_SIM)
	$(VVP) $(BUILD)/$@.vvp
tb_encoder_gl: $(BUILD)/gl_encoder.v
	iverilog -g2012 -I src -I $(CELLS_DIR) -DNO_INCLUDES -o $(BUILD)/$@.vvp sim/tb_encoder.v sim/mult18x18d_sim.v $< $(CELLS_SIM)
	$(VVP) $(BUILD)/$@.vvp

test: $(BENCHES)

clean:
	rm -rf $(BUILD)

.PHONY: all test clean $(BENCHES)
