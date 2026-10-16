# Host unit tests for components/bridge_core (no ESP-IDF needed).
#   make -C test        build and run
CC ?= cc
CFLAGS ?= -std=c11 -O1 -g -Wall -Wextra -Werror -fsanitize=address,undefined
CORE := ../components/bridge_core
SRCS := test_main.c $(CORE)/bwp.c $(CORE)/l2fwd.c $(CORE)/bridge_util.c

all: run

build/test_bridge: $(SRCS) $(wildcard $(CORE)/include/*.h)
	@mkdir -p build
	$(CC) $(CFLAGS) -I$(CORE)/include -o $@ $(SRCS)

run: build/test_bridge
	./build/test_bridge

clean:
	rm -rf build

.PHONY: all run clean
