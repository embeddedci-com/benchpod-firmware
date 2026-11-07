#!/bin/sh
# The OTA commit runs from RAM while internal flash is erased (ota_commit.c, .RamFunc). Any call
# from that code into flash (a libcall such as memset, a HAL function) or any read of a flash
# constant faults mid-install and bricks the pod. Fail if a ram_* function
#   - branches to an address outside RAM (b, bl, blx, b.w and every conditional b<cc>[.w]),
#   - calls or jumps through a register (blx rN, bx rN other than lr): the target is not checked,
#   - holds a flash address in its literal pool (.word 0x08......), e.g. a const table -O2 put
#     in .rodata.
#   usage: check_ramfunc_calls.sh build/bench_pod_stm32.elf
set -eu
ELF=${1:-build/bench_pod_stm32.elf}
OBJDUMP=${OBJDUMP:-arm-none-eabi-objdump}
dis=$("$OBJDUMP" -d "$ELF" | awk '/^2[0-9a-f]+ <ram_[a-zA-Z0-9_]+>:/{f=1} /^$/{f=0} f')
[ -n "$dis" ] || { echo "check_ramfunc_calls: no ram_* functions found in $ELF"; exit 1; }
CC='(eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le|al)'
out=$(printf '%s\n' "$dis" | grep -E "[[:space:]](bl|blx|b|b${CC})(\.w|\.n)?[[:space:]]+[0-9a-f]+ <" \
      | grep -vE "[[:space:]](bl|blx|b|b${CC})(\.w|\.n)?[[:space:]]+2[0-9a-f]{7} <" || true)
ind=$(printf '%s\n' "$dis" | grep -E '[[:space:]](blx|bx)[[:space:]]+(r[0-9]+|ip|sb|sl|fp)([[:space:]]|$)' || true)
lit=$(printf '%s\n' "$dis" | grep -E '\.word[[:space:]]+0x0[89][0-9a-f]{6}' || true)
if [ -n "$out$ind$lit" ]; then
    echo "check_ramfunc_calls: RAM-resident OTA code reaches flash:"
    [ -z "$out" ] || { echo "branches out of RAM:"; printf '%s\n' "$out"; }
    [ -z "$ind" ] || { echo "calls through a register:"; printf '%s\n' "$ind"; }
    [ -z "$lit" ] || { echo "flash addresses in the literal pool:"; printf '%s\n' "$lit"; }
    exit 1
fi
echo "check_ramfunc_calls: $(printf '%s\n' "$dis" | grep -c '>:$') ram_* functions, no branch, indirect call or literal into flash"
