# Host unit tests for components/bridge_core (no ESP-IDF needed).
#   make -C test        build and run
CC ?= cc
CFLAGS ?= -std=c11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined
CORE := ../components/bridge_core
MONO := ../components/monocypher
# The pod firmware's signing test vectors: the bridge must agree with the pod on every case.
POD_VECTORS := ../../stm32h563/test/vectors
SRCS := test_main.c $(CORE)/bwp.c $(CORE)/l2fwd.c $(CORE)/bridge_util.c $(CORE)/hmac_sha256.c $(CORE)/pairing.c \
        $(CORE)/bwp_crypto.c $(CORE)/fwsig.c $(MONO)/monocypher.c $(MONO)/monocypher-ed25519.c

all: run

build/test_bridge: $(SRCS) $(wildcard $(CORE)/include/*.h) $(POD_VECTORS)/fwsign_vectors.h
	@mkdir -p build
	$(CC) $(CFLAGS) -I$(CORE)/include -I$(MONO) -I$(POD_VECTORS) -o $@ $(SRCS)

run: build/test_bridge
	./build/test_bridge

clean:
	rm -rf build

.PHONY: all run clean
