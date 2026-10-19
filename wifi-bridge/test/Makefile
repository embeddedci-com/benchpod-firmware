# Host unit tests for components/bridge_core (no ESP-IDF needed).
#   make -C test        build and run
CC ?= cc
CFLAGS ?= -std=c11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined
CORE := ../components/bridge_core
MONO := ../components/monocypher
SRCS := test_main.c $(CORE)/bwp.c $(CORE)/l2fwd.c $(CORE)/bridge_util.c $(CORE)/hmac_sha256.c $(CORE)/pairing.c \
        $(CORE)/bwp_crypto.c $(MONO)/monocypher.c

all: run

build/test_bridge: $(SRCS) $(wildcard $(CORE)/include/*.h)
	@mkdir -p build
	$(CC) $(CFLAGS) -I$(CORE)/include -I$(MONO) -o $@ $(SRCS)

run: build/test_bridge
	./build/test_bridge

clean:
	rm -rf build

.PHONY: all run clean
