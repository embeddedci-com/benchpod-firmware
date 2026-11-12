# The sensor definition engine (../src/sensor_def*.c, ../src/sensor_stim.c) as WebAssembly, for
# the server's validator and the web/desktop editors.  README.md documents the ABI.
#
#   make            build build/sensor_def.wasm
#   make test       shim math vs libm, host golden outputs, then the same calls in Node
#   make clean
#
# Needs Homebrew LLVM + LLD (brew install llvm lld); the host parts use the system cc.

LLVM    ?= /opt/homebrew/opt/llvm/bin
WASMLD  ?= /opt/homebrew/opt/lld/bin/wasm-ld
CLANG   ?= $(LLVM)/clang
NODE    ?= node
CC      ?= cc

ENGINE := ../src/sensor_def_json.c ../src/sensor_def_math.c ../src/sensor_def_vm.c ../src/sensor_def_expr.c \
          ../src/sensor_def_compile.c ../src/sensor_def_inst.c ../src/sensor_def_regs.c \
          ../src/sensor_def_uart.c ../src/sensor_def_analog.c ../src/sensor_stim.c
HEADERS := $(wildcard ../src/sensor_def*.h) sd_wasm.h $(wildcard libc/include/*.h)
FIXTURES := ../test/fixtures/sensor_defs

# -ffp-contract=off: no fused multiply-add, so every platform rounds the same way.
WASM_CFLAGS := --target=wasm32 -nostdlib -ffreestanding -O2 -ffp-contract=off -std=c11 \
               -Wall -Wextra -Wno-unused-parameter -isystem libc/include -I../src -fvisibility=hidden
# The exports come from __attribute__((export_name(...))) in sd_wasm.c, so they keep their ABI
# names (validate, preview, ...) while the C symbols stay sdw_*.
WASM_LDFLAGS := --no-entry --gc-sections --strip-all -z stack-size=1048576
HOST_CFLAGS := -std=gnu11 -O2 -ffp-contract=off -Wall -Wextra -Wno-unused-parameter -I../src

all: build/sensor_def.wasm

build:
	mkdir -p build

build/obj: | build
	mkdir -p build/obj

WASM_OBJS := $(patsubst ../src/%.c,build/obj/%.o,$(ENGINE)) build/obj/sd_wasm.o build/obj/shim.o

build/obj/%.o: ../src/%.c $(HEADERS) | build/obj
	$(CLANG) $(WASM_CFLAGS) -c $< -o $@

build/obj/sd_wasm.o: sd_wasm.c $(HEADERS) | build/obj
	$(CLANG) $(WASM_CFLAGS) -c $< -o $@

build/obj/shim.o: libc/shim.c $(HEADERS) | build/obj
	$(CLANG) $(WASM_CFLAGS) -fno-builtin -c $< -o $@

build/sensor_def.wasm: $(WASM_OBJS)
	$(WASMLD) $(WASM_LDFLAGS) $(WASM_OBJS) -o $@
	@ls -l $@ | awk '{print "sensor_def.wasm: " $$5 " bytes"}'

build/golden_host: golden_host.c sd_wasm.c $(ENGINE) $(HEADERS) | build
	$(CC) $(HOST_CFLAGS) -o $@ golden_host.c sd_wasm.c $(ENGINE) -lm

build/shim_math_test: shim_math_test.c libc/shim.c | build
	$(CC) -std=gnu11 -O2 -ffp-contract=off -DSHIM_MATH_ONLY -o $@ shim_math_test.c libc/shim.c -lm

build/golden.jsonl: build/golden_host $(wildcard $(FIXTURES)/*.json)
	./build/golden_host $(FIXTURES) > $@

test: build/sensor_def.wasm build/shim_math_test build/golden.jsonl
	./build/shim_math_test
	@if command -v $(NODE) >/dev/null 2>&1; then \
		$(NODE) test_wasm.mjs build/sensor_def.wasm build/golden.jsonl $(FIXTURES); \
	else echo "SKIP: node not found, the WASM golden comparison did not run"; fi

clean:
	rm -rf build

.PHONY: all test clean
