#!/bin/sh
# The OTA commit runs from RAM while internal flash is erased (ota_commit.c, .RamFunc). Any call
# from that code into flash (a libcall such as memset, a HAL function) faults mid-install and
# bricks the pod. Fail if a ram_* function branches to an address outside RAM.
#   usage: check_ramfunc_calls.sh build/bench_pod_stm32.elf
set -eu
ELF=${1:-build/bench_pod_stm32.elf}
OBJDUMP=${OBJDUMP:-arm-none-eabi-objdump}
dis=$("$OBJDUMP" -d "$ELF" | awk '/^2[0-9a-f]+ <ram_[a-zA-Z0-9_]+>:/{f=1} /^$/{f=0} f')
[ -n "$dis" ] || { echo "check_ramfunc_calls: no ram_* functions found in $ELF"; exit 1; }
bad=$(printf '%s\n' "$dis" | grep -E '[[:space:]](bl|blx|b|b\.w)[[:space:]]+[0-9a-f]+ <' \
      | grep -vE '[[:space:]](bl|blx|b|b\.w)[[:space:]]+2[0-9a-f]{7} <' || true)
if [ -n "$bad" ]; then
    echo "check_ramfunc_calls: RAM-resident OTA code branches out of RAM:"
    printf '%s\n' "$bad"
    exit 1
fi
echo "check_ramfunc_calls: $(printf '%s\n' "$dis" | grep -c '>:$') ram_* functions, no calls out of RAM"
