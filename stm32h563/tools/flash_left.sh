#!/bin/sh
# Print how much of the code region (the linker script's FLASH region) is left after the link,
# and fail when it is below a floor, so a release cannot creep up to the region's end unnoticed.
#   usage: flash_left.sh build/bench_pod_stm32.elf config/STM32H563ZITX_FLASH.ld [floor_kb]
# Used = the end of the last loaded byte in FLASH (.text, .rodata, the .data load image, ...),
# the same figure as the .bin size and the linker's FLASH line. With floor_kb set, less than
# floor_kb KB left fails the build; without it, less than 24 KB left only warns.
set -eu
ELF=$1
LD=$2
FLOOR_KB=${3:-}
READELF=${READELF:-arm-none-eabi-readelf}
line=$(grep -E '^[[:space:]]*FLASH[[:space:]]*\(' "$LD")
origin=$(printf '%s\n' "$line" | sed -E 's/.*ORIGIN[[:space:]]*=[[:space:]]*(0x[0-9A-Fa-f]+).*/\1/')
len=$(printf '%s\n' "$line" | sed -E 's/.*LENGTH[[:space:]]*=[[:space:]]*([0-9]+)K.*/\1/')
origin=$(printf '%d' "$origin")
size=$((len * 1024))
# PT_LOAD rows: Type Offset VirtAddr PhysAddr FileSiz MemSiz ...; only bytes stored in flash count.
end=$("$READELF" -lW "$ELF" | awk '$1 == "LOAD" { print $4, $5 }' | while read -r pa fs; do
    pa=$(printf '%d' "$pa"); fs=$(printf '%d' "$fs")
    if [ "$fs" -gt 0 ] && [ "$pa" -ge "$origin" ] && [ "$pa" -lt $((origin + size)) ]; then
        echo $((pa + fs))
    fi
done | sort -n | tail -n 1)
used=$((end - origin))
left=$((size - used))
echo "flash left: $left bytes ($used of $size used)"
if [ -n "$FLOOR_KB" ]; then
    if [ "$left" -lt $((FLOOR_KB * 1024)) ]; then
        echo "flash left: below the floor of $FLOOR_KB KB (FLASH_FLOOR_KB)" >&2
        exit 1
    fi
elif [ "$left" -lt $((24 * 1024)) ]; then
    # Not "warning:": the CI warning check would fail a build that should only warn.
    echo "flash left: below 24 KB (set FLASH_FLOOR_KB to make this fail)" >&2
fi
