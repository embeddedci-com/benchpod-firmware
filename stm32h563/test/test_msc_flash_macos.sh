#!/bin/bash
# test_msc_flash_macos.sh: cross-check the FAT32 and exFAT writers (xip_mod_msc_flash.c) against
# macOS's own file systems. NOT part of `make test` (it needs macOS, hdiutil, fsck_msdos and
# fsck_exfat).
#
# For each FAT32 case it makes a FAT32 image with hdiutil (64 MB, plain or with an MBR), optionally puts
# files on it through the macOS mount, writes a file with the pod's writer into a raw copy, then
#  - runs fsck_msdos -n on the raw image (must report no errors),
#  - attaches the image read-only and compares the file's size and content with the original,
#  - detaches.
# The exFAT cases (256 MB, plain or with an MBR, files through the mount, a full root) do the same
# with fsck_exfat -n.
# Images live in a scratch directory (default: a mktemp directory, removed at the end; pass
# SCRATCH=/some/dir to keep them).
#
#   stm32h563/test/test_msc_flash_macos.sh
set -u
here=$(cd "$(dirname "$0")" && pwd)
scratch=${SCRATCH:-$(mktemp -d /tmp/msc-fat32.XXXXXX)}
mkdir -p "$scratch"
bin="$scratch/msc_flash_image"
cc -std=gnu11 -Wall -Wextra -O1 -I"$here/../src" "$here/msc_flash_image.c" "$here/../src/xip_mod_msc_flash.c" \
   "$here/../src/msc_flash_names.c" -o "$bin" || exit 1
fail=0

bytes() { head -c "$1" /dev/urandom > "$2"; }

attach_ro() {   # image -> prints the mount point
    hdiutil attach -readonly -nobrowse -imagekey diskimage-class=CRawDiskImage "$1" 2>/dev/null \
        | awk -F'\t' '/\/Volumes\// { print $NF }' | head -1
}

check() {   # name layout fill(0|1|2) filesize
    local name=$1 layout=$2 fill=$3 size=$4 d="$scratch/$1"
    mkdir -p "$d"
    hdiutil create -size 64m -fs "MS-DOS FAT32" -volname T -layout "$layout" "$d/a.dmg" >/dev/null 2>&1 || { echo "FAIL $name: hdiutil create"; fail=1; return; }
    if [ "$fill" != 0 ]; then   # files through the macOS mount: some, or enough to need a second root cluster
        local mp
        mp=$(hdiutil attach -nobrowse "$d/a.dmg" 2>/dev/null | awk -F'\t' '/\/Volumes\// { print $NF }' | head -1)
        if [ -z "$mp" ]; then echo "FAIL $name: attach for fill"; fail=1; return; fi
        bytes 20000 "$d/old.bin"
        cp "$d/old.bin" "$mp/FIRMWARE.BIN"; cp "$d/old.bin" "$mp/KEEP.TXT"
        if [ "$fill" = 2 ]; then   # enough long names to fill the first root cluster (512 B: 16 slots; macOS uses LFN)
            for i in $(seq 1 40); do : > "$mp/file-with-a-long-name-$i.txt"; done
        fi
        sync; hdiutil detach "$mp" >/dev/null 2>&1 || hdiutil detach -force "$mp" >/dev/null
    fi
    hdiutil convert "$d/a.dmg" -format UDTO -o "$d/raw" >/dev/null 2>&1
    mv "$d/raw.cdr" "$d/raw.img"
    bytes "$size" "$d/new.bin"
    out=$("$bin" "$d/raw.img" "$d/new.bin" FIRMWARE.BIN) || { echo "FAIL $name: writer: $out"; fail=1; return; }
    # fsck wants the file system alone: with an MBR it runs on the partition's device node.
    local dev
    dev=$(hdiutil attach -nomount -imagekey diskimage-class=CRawDiskImage "$d/raw.img" 2>/dev/null | awk 'NR==1 { print $1 }')
    if [ "$layout" = NONE ]; then fsck=$(fsck_msdos -n "$d/raw.img" 2>&1); frc=$?
    else fsck=$(fsck_msdos -n "${dev}s1" 2>&1); frc=$?; fi
    hdiutil detach "$dev" >/dev/null 2>&1
    mp=$(attach_ro "$d/raw.img")
    if [ -z "$mp" ]; then echo "FAIL $name: attach: $fsck"; fail=1; return; fi
    if [ $frc -ne 0 ]; then echo "FAIL $name: fsck_msdos rc $frc"; echo "$fsck" | sed 's/^/    /'; fail=1; hdiutil detach "$mp" >/dev/null 2>&1; return; fi
    if cmp -s "$d/new.bin" "$mp/FIRMWARE.BIN" && [ "$(stat -f %z "$mp/FIRMWARE.BIN")" = "$size" ]; then
        ok=1
        if [ "$fill" != 0 ]; then cmp -s "$d/old.bin" "$mp/KEEP.TXT" || ok=0; fi
        [ $ok = 1 ] && echo "ok   $name: $out; fsck_msdos clean; mounted copy matches ($size bytes)" || { echo "FAIL $name: KEEP.TXT changed"; fail=1; }
    else
        echo "FAIL $name: content or size differs"; fail=1
    fi
    hdiutil detach "$mp" >/dev/null 2>&1 || hdiutil detach -force "$mp" >/dev/null 2>&1
}

check plain-small      NONE     0 1000
check plain-big        NONE     0 3000000
check plain-replace    NONE     1 700000
check plain-fullroot   NONE     2 12345
check mbr-small        MBRSPUD  0 70000
check mbr-replace      MBRSPUD  1 5000000

check_exfat() {   # name layout fill(0|1|2) filesize
    local name=$1 layout=$2 fill=$3 size=$4 d="$scratch/$1" mp dev
    mkdir -p "$d"
    hdiutil create -size 256m -fs ExFAT -volname T -layout "$layout" "$d/a.dmg" >/dev/null 2>&1 || { echo "FAIL $name: hdiutil create"; fail=1; return; }
    if [ "$fill" != 0 ]; then
        mp=$(hdiutil attach -nobrowse "$d/a.dmg" 2>/dev/null | awk -F'\t' '/\/Volumes\// { print $NF }' | head -1)
        if [ -z "$mp" ]; then echo "FAIL $name: attach for fill"; fail=1; return; fi
        bytes 200000 "$d/old.bin"
        cp "$d/old.bin" "$mp/FIRMWARE.BIN"; cp "$d/old.bin" "$mp/KEEP.TXT"
        if [ "$fill" = 2 ]; then   # long names until the root directory needs a second cluster
            for i in $(seq 1 700); do : > "$mp/file-with-a-rather-long-name-number-$i.txt"; done
        fi
        sync; hdiutil detach "$mp" >/dev/null 2>&1 || hdiutil detach -force "$mp" >/dev/null
    fi
    hdiutil convert "$d/a.dmg" -format UDTO -o "$d/raw" >/dev/null 2>&1
    mv "$d/raw.cdr" "$d/raw.img"
    bytes "$size" "$d/new.bin"
    if [ "$fill" = 2 ]; then   # more small files with the writer until the root directory has to grow
        bytes 3000 "$d/small.bin"
        for i in $(seq 1 700); do
            o=$("$bin" "$d/raw.img" "$d/small.bin" "F$i.BIN") || { echo "FAIL $name: writer F$i: $o"; fail=1; return; }
            case "$o" in *"root-extended 1"*) break ;; esac
        done
        case "$o" in *"root-extended 1"*) ;; *) echo "FAIL $name: the root directory never grew"; fail=1; return ;; esac
    fi
    out=$("$bin" "$d/raw.img" "$d/new.bin" FIRMWARE.BIN) || { echo "FAIL $name: writer: $out"; fail=1; return; }
    dev=$(hdiutil attach -nomount -imagekey diskimage-class=CRawDiskImage "$d/raw.img" 2>/dev/null | awk 'NR==1 { print $1 }')
    if [ "$layout" = NONE ]; then fsck=$(fsck_exfat -n "$dev" 2>&1); frc=$?
    else fsck=$(fsck_exfat -n "${dev}s1" 2>&1); frc=$?; fi
    hdiutil detach "$dev" >/dev/null 2>&1
    if [ $frc -ne 0 ]; then echo "FAIL $name: fsck_exfat rc $frc"; echo "$fsck" | sed 's/^/    /'; fail=1; return; fi
    mp=$(attach_ro "$d/raw.img")
    if [ -z "$mp" ]; then echo "FAIL $name: attach: $fsck"; fail=1; return; fi
    if cmp -s "$d/new.bin" "$mp/FIRMWARE.BIN" && [ "$(stat -f %z "$mp/FIRMWARE.BIN")" = "$size" ]; then
        ok=1
        if [ "$fill" != 0 ]; then cmp -s "$d/old.bin" "$mp/KEEP.TXT" || ok=0; fi
        [ $ok = 1 ] && echo "ok   $name: $out; fsck_exfat clean; mounted copy matches ($size bytes)" || { echo "FAIL $name: KEEP.TXT changed"; fail=1; }
    else
        echo "FAIL $name: content or size differs"; fail=1
    fi
    hdiutil detach "$mp" >/dev/null 2>&1 || hdiutil detach -force "$mp" >/dev/null 2>&1
}

check_exfat exfat-plain-small   NONE     0 1000
check_exfat exfat-plain-big     NONE     0 5000000
check_exfat exfat-plain-replace NONE     1 700000
check_exfat exfat-fullroot      NONE     2 12345
check_exfat exfat-mbr-small     MBRSPUD  0 70000
check_exfat exfat-mbr-replace   MBRSPUD  1 3000000

[ -z "${SCRATCH:-}" ] && rm -rf "$scratch"
[ $fail = 0 ] && echo "macOS cross-check: all passed" || echo "macOS cross-check: FAILED"
exit $fail
