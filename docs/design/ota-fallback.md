# OTA fallback for the STM32H563 BenchPod

Status: design note, 2026-10-04. No code changed. Covers the firmware in `stm32h563/` as of the
current `build/bench_pod_stm32.bin` (523,984 bytes).

## Summary

**Status (3.5.1):** the first step is in. The in-place install checks every sector (option 1:
per-sector CRC of the hashed image, a pre-pass over the raw read path before the first erase,
flash error flags, read-back, retries, and a CI check that the RAM code never calls into flash),
and every install writes the verified image into the W25Q `fw` slot at 0x600000 first (option 4,
first half; written only when an update is actually installed, never by a dry run). The slot is
internal: `blob_status` and installers do not list it; the console `blobs` command prints a
`fw-copy` line. A pod coming from older firmware runs that firmware's install code once, so its
first update into 3.5.1 has the old risk; every update made from 3.5.1 on has the checks and leaves
a copy. A power cut during the rewrite still needs USB DFU until the bootloader (next release)
uses the copy.


Today a firmware OTA rewrites internal flash in place from PSRAM, starting at sector 0. A power
cut during that window leaves no runnable app, and the only way back is BOOT0 + USB DFU
(`benchpod flash-self`). The fix that works on both the 2 MB (ZIT6) and the 1 MB (ZGT6) part is
a small bootloader in the first 32 KB that copies a staged image from the W25Q, so a cut copy
simply restarts on the next boot. Dual-bank swap does not fit the 1 MB part. Recommended path:

1. **This release:** harden the in-place commit (option 1) and stage the firmware into a new W25Q
   slot as well as PSRAM (option 4, first half). Both are small, testable and keep the layout.
2. **Next release:** bootloader + W25Q copy (option 2 + 4), installed once through today's OTA,
   then the pending/confirm/rollback scheme (option 5) on top.

## Today's code, as found

| Item | Where | Fact |
|---|---|---|
| Code area | `config/STM32H563ZITX_FLASH.ld` | `FLASH` = 0x08000000, 928 KB, fits both parts |
| Image size | `build/` | 523,984 B (`.text` 0x63450, `.rodata` 0x1c0b0, `.data` 0x718) |
| RAM | `arm-none-eabi-size` | `.bss` 629,552 B of 640 KB: about 25 KB headroom |
| Persistence | `flash_layout.h`, `*_OFFSET` | top 96 KB: 0x1E8000..0x1FFFFF (2 MB) or 0x0E8000..0x0FFFFF (1 MB) |
| fw_info | `fw_info.h`, `.fw_info` at +0x400 | magic, layout 2, `min_flash_kb` 1024 |
| Staging | `ota.c` | PSRAM offset 0 (shared with LA capture), SHA-256 via HAL `psram_read` |
| Commit | `ota_commit.c` | `.RamFunc`, IRQs off, sector 0 upward: raw XSPI read, erase, program, kick IWDG, reset |
| Error checks | `ram_flash_erase/program` | wait on `BSY` only; `NSSR` error flags never read; no read-back |
| Opt level | `Makefile` | `OPT := -Og -g3`; disassembly of `.RamFunc` today shows only RAM-to-RAM `bl` calls |
| Boot guard | `boot_guard.c`, `main.c` | `.noinit` failed-boot counter, safe mode after 2; healthy = hw ready + 15 s |
| DFU entry | `dfu_boot.c` | `.noinit` magic, jump to ROM at 0x0BF97000 before `HAL_Init` |
| Blob slots | `blob_store.c` | gw0 0x100000, gw1 0x140000, esp 0x200000 (4 MB); 0x180000..0x1FFFFF and 0x600000..0x7FFFFF free on a W25Q64 |
| Server | `embeddedci-server/api/benchpod_ota.go` | pushes firmware first, then blobs; waits for the pod to come back and records `version_after` |

Note: the header comment of `ota_commit.c` still says the image is "~1.6 MB, larger than one
1 MB bank". That predates moving the blobs to the W25Q and is no longer true.

## STM32H563 facts relied on

| Fact | Source | Confidence |
|---|---|---|
| 8 KB sectors, 128 per bank on the 2 MB part | CMSIS `FLASH_SECTOR_SIZE 0x2000`, `FLASH_SECTOR_NB 128`; `flash_compat.c` | verified |
| 2 banks of half the flash each: 2x1 MB (ZIT6), 2x512 KB (ZGT6) | RM0481, `flash_layout_bank_size()` | verified |
| Program unit 128-bit quad-word with ECC | RM0481, `flash_compat.c` | verified |
| Error flags `WRPERR`, `PGSERR`, `STRBERR`, `INCERR`, `OPTCHANGEERR` in `NSSR`, cleared via `NSCCR` | CMSIS `FLASH_SR_*` | verified names; clear register to confirm in RM0481 |
| `SWAP_BANK` exists in both `FLASH_OPTCR` (bit 31) and `FLASH_OPTSR_*` (bit 31) | CMSIS | verified bits; exact sequence (OPTSTRT, reset) and power-loss behavior of option programming: **check RM0481** |
| WRP: 32-bit `WRPSG` per bank, 1 bit = 4 sectors (32 KB) | HAL `OB_WRP_SECTOR_0TO3` (for 128 sectors/bank) | verified for 2 MB; **grouping on the 64-sector 1 MB part uncertain** |
| HDP areas per bank, `HDPL` levels in SBS | CMSIS `FLASH_HDP*`, `SBS_HDPL*` | exists; not needed here |
| `NSBOOTADD` / `NSBOOT_LOCK` select and lock the boot address (TZEN=0) | HAL `OB_BOOT_LOCK` | exists; leave alone |
| HASH and CRC peripherals present on H563 | CMSIS `HASH_BASE`, `CRC_BASE` | verified |
| H5 falls back to the ROM bootloader on an empty user flash (like G0/G4 "empty check") | none found | **uncertain, probably not**; one bench test: erase sector 0 over SWD, power-cycle |
| HAL `FLASH_SIZE` / `FLASH_BANK_SIZE` macros read `FLASHSIZE_BASE` as 16 bits | CMSIS | verified; that narrow read is a precise bus fault on this chip (see `flash_layout.c`), so new code must not use them |

## Option 1: harden the in-place commit

Changes, all inside `ota_commit.c`:

- **Check flash errors.** After every erase and every quad-word, read `NSSR` for the error flags
  above, clear them through `NSCCR`, and treat any as a failed sector.
- **Close the "hashed path vs flashed path" gap.** Before erasing anything (flash code still
  intact, so mbedTLS may be called), run one pass through the *same* `ram_psram_read_burst` the
  commit uses, hashing with SHA-256 and also storing a CRC32 per 8 KB sector in a RAM table
  (116 sectors x 4 B, under 0.5 KB). Refuse the commit if that SHA differs from the expected one.
  During the real pass, CRC each sector buffer with the CRC peripheral (register-only, safe from
  RAM) and re-read from PSRAM up to 3 times on mismatch *before* erasing that sector.
- **Read back.** After programming a sector, CRC the flash contents and compare with the table.
  On mismatch, erase and reprogram up to 3 times.
- **Sector 0 last.** Erase sector 0 first, write sectors 1..N, then write sector 0. A cut then
  leaves an erased vector table instead of new vectors pointing into half-old code: a
  deterministic failure, and an automatic ROM DFU if the H5 turns out to have an empty-flash
  check (bench test above).
- **Unrecoverable sector.** Jump straight to ROM DFU from RAM (`VTOR = 0x0BF97000`, load MSP, jump)
  instead of resetting into a broken image. This is the "mid-application" jump `dfu_boot.c`
  avoids, so validate it on the bench; the fallback is a reset.
- **-Og-proof.** Mark the RAM functions `optimize("no-tree-loop-distribute-patterns")` (or build
  the file with `-fno-tree-loop-distribute-patterns -fno-builtin`), and add a build check that
  disassembles `.RamFunc` and fails if any `bl`/`b.w` leaves RAM. Today's build passes that
  check; the point is that `-O2` or a refactor would not silently break it.
- **Measure the window.** Record the DWT cycle count of `ram_commit_all` into `.noinit` and print
  it next boot, so the brick window is a number, not "multi-second".

Remaining failure modes: a power cut between erasing sector 0 and writing it back still leaves no
app (BOOT0 + DFU). Brown-out mid quad-word can leave an ECC-torn word, which read-back catches
only while power stays up. The hardening removes the silent-corruption cases, not the power
window.

Cost: about 1 to 2 days plus bench time; under 1 KB RAM, about 1 KB flash. Low risk, no layout or
tooling change. **Ship this release.**

## Option 2: small immutable bootloader

**What it does,** running from reset on HSI with no HAL:

1. Kick the IWDG if it is running (a hardware-mode IWDG option would leave it on).
2. Honor the DFU `.noinit` magic (move `dfu_boot_check` here).
3. Read the W25Q "fw" slot header (option 4). If it holds a *pending* image whose SHA differs
   from the app's, hold the iCE40 in `CRESET` and take the shared bus (PG0), verify the slot's
   SHA, copy slot to internal flash sector by sector with the option 1 checks, verify the app's
   SHA, mark the slot "installed", release the bus.
4. Validate the app: fw_info magic and layout at app+0x400, a length and SHA-256 (or CRC32) in an
   extended fw_info, initial SP in RAM, reset vector inside the app.
5. Valid: set `VTOR` to the app base, load MSP, jump. Invalid with a good slot: copy again.
   Invalid without one: jump to ROM DFU, so `benchpod flash-self` works without touching BOOT0.

A power cut during step 3 is harmless: the slot is still there and the next boot redoes the copy.

**Size.** Register-level W25Q single-SPI over XSPI, flash erase/program, software SHA-256 (or the
HASH peripheral) and fw_info parsing fit in 16 KB at `-Os`. Reserve **32 KB** (sectors 0..3)
because that is one WRP group on the 2 MB part. The app moves to 0x08008000; `FLASH` becomes
896 KB. With a 512 KB image that leaves plenty on the 1 MB part. App RAM is unchanged, and the
app's `.RamFunc` commit code (and its 8 KB sector buffer) can go, freeing RAM.

**Relinking the app.** `ORIGIN(FLASH) = 0x08008000`, `LENGTH = 896K`; `SystemInit` sets
`SCB->VTOR` to the linked vector table (or the bootloader sets it before the jump). fw_info stays
at image+0x400 and moves to `FW_INFO_LAYOUT 3`. Every installer must refuse a layout mismatch: OTA
(`staged_fits_this_flash`), the server's `fwImageNeedsKB`, `make flash-dfu` (`DFU_FLASH_ADDR`),
`benchpod flash-self`, and factory programming. A layout-2 image flashed at 0x08000000 on a
bootloader pod would overwrite the bootloader; WRP turns that into a DFU write error.

**Protecting it.** Set WRP on sectors 0..3 (`OB_WRP_SECTOR_0TO3`, bank 1) from the app on the
first boot that sees a valid bootloader. WRP only guards against our own bugs and stray DFU
writes; the app can clear it again, which is what we want for a future bootloader update. Skip
HDP and `NSBOOT_LOCK`: they add lock-out risk and protect against nothing in our threat model.
Confirm the WRP group size on the 1 MB part before relying on "4 sectors".

**Getting it onto existing pods.** No special installer is needed: build a combined image
(bootloader at 0, app at 0x8000, under 928 KB) and push it with *today's* OTA, which does not care
what the bytes are. That one update carries today's power-cut risk, once; with option 1 in place
it is the same risk as any update now. Combined images must carry a fw_info at +0x400 so the
server and `staged_fits_this_flash` accept them (the bootloader's own fw_info, layout 3). After
that, OTA sends app-only images. DFU-only rollout is not required, but the factory and the
`flash-self` recovery path must learn the combined image.

Updating the bootloader itself later is the same in-place risk, so keep it minimal and rarely
changed.

Cost: 1 to 2 weeks including tooling (Makefile, CLI, server layout check, CI artifact for the
combined image, hwe2e). Medium risk, almost all of it in the transition and the tooling.
**Next release.**

## Option 3: dual-bank with SWAP_BANK

On the **2 MB part** this is attractive: the app runs from bank 1 while the new image is written
into bank 2 at 0x08100000..0x081E7FFF (928 KB, read-while-write across banks, no RAM-resident
code, IRQs stay on), verified, then `SWAP_BANK` is toggled and the pod resets. Rollback is
toggling it back. Both banks run the same 0x08000000-linked binary.

Persistence complicates it: the records sit physically in bank 2. With `SWAP_BANK=1` they appear
at logical 0x0E8000, so `flash_layout_store_off` and `addr_to_bank_sector` would have to follow
the swap state (and whether `BKSEL` addresses the logical or physical bank under swap is
**uncertain, check RM0481**). The 928 KB link limit happens to keep bank 1's top 96 KB free, so
the geometry works.

On the **1 MB part** it does not fit. Each bank is 512 KB (524,288 B); the image is 523,984 B,
304 bytes under a whole bank, and bank 2 loses its top 96 KB to persistence, leaving 416 KB.
Moving persistence elsewhere (the 96 KB EDATA area, or the W25Q, which would expose the identity
key on an external chip) is a larger project. Since future pods are 1 MB, dual-bank would be a
second update path for the old pods only, doubling test burden. **Not recommended.**

## Option 4: staging in the W25Q

The W25Q survives a power cut, PSRAM does not, so any resumable scheme needs the image there.

Proposed map (W25Q64, 8 MB; a W25Q128 adds 8 MB free):

| Range | Use |
|---|---|
| 0x000000 | iCE40 boot gateware (unchanged) |
| 0x100000 / 0x140000 | gw0 / gw1 (unchanged) |
| 0x180000..0x1FFFFF | free (512 KB) |
| 0x200000..0x5FFFFF | esp (unchanged) |
| 0x600000..0x6FFFFF | **fw**: staged firmware, 4 KB header + up to 1020 KB |
| 0x700000..0x7FFFFF | **fw-prev**: last known-good firmware, for rollback |

Implementation reuses `blob_store_write`: add `BLOB_FW` (and later `BLOB_FW_PREV`) and keep PSRAM
staging plus `ota_end` as they are. After verify, copy PSRAM to the slot; that path already
erases the header first, programs, re-hashes the written data and commits last, so a cut leaves an
empty slot, never a valid-looking partial one. `blob_store_load` checks capacity >= 0x600000;
raise that to 0x800000 for the new slots (every W25Q64 qualifies). Header `reserved` words carry
the pending/confirm state (option 5).

Bus sharing: every access already goes through `w25q_open` after
`signal_engine_quiesce_psram_masters`, with the iCE40 running and tristated. The bootloader has no
gateware to negotiate with: it holds `CRESET` low (the iCE40 drops off the bus), drives PG0 and
the flash CS (PE3) itself, and releases `CRESET` before jumping, after which the iCE40 configures
from offset 0 as on any power-up. Programming 1 MB of W25Q adds a few seconds to an OTA.

Even before a bootloader exists, writing the slot this release is useful: the image survives a cut,
so a future DFU-free recovery (or the bootloader's first boot) can use it, and `ota_commit` could
read from the W25Q instead of the LA-shared PSRAM region.

Cost: 1 to 2 days for the slot; 4 KB RAM worst case reused from existing buffers. Low risk.
**Slot in this release; the bootloader consumer next release.**

## Option 5: pending, confirm, roll back

Fits on top of options 2 and 4:

1. App stages to `fw`, marks it *pending* with `tries = 0`, resets.
2. Bootloader: if pending and the app does not match, first make sure `fw-prev` holds the running
   app (copy internal flash out if its SHA differs), then install `fw`, mark it *trial*.
3. Each trial boot clears one bit of a "tries" bitfield in the slot header. Clearing bits needs no
   erase, so the counter survives power cycles, unlike `boot_guard`'s `.noinit` counter, which a
   power-on resets.
4. The app confirms by writing a *confirmed* word. Criterion: today's `boot_guard_healthy` (hw ready
   and 15 s up), plus, if the pod was cloud-connected before the update (stored in the pending
   header), a successful cloud connection. A firmware that runs but cannot reach the cloud can
   no longer be fixed remotely, so it should not confirm.
5. Bootloader sees a trial with all tries used (say 3): installs `fw-prev`, marks `fw` *rejected*.
   The server already waits for the pod and records `version_after`; it reports "rolled back"
   when the old version returns.

Relationship to `boot_guard`/`boot_policy`: safe mode stays the answer for a *confirmed* image
that crash-loops in net or iCE40 bring-up. For a trial image, a failed boot counts toward
rollback instead; safe mode must not confirm (it already refuses to clear its counter in safe
mode). Crashes before USB comes up are caught only by the bootloader counter, which is why the
counter lives there.

Without a bootloader, an app-level rollback only works when the bad firmware still runs well
enough to reinstall the old one through the risky in-place path. Limited value; not worth building
alone.

Cost: 3 to 5 days on top of 2 + 4, mostly host tests of the state machine (the pattern of
`boot_policy.c`) plus hwe2e cases that cut power at each step. **Next release, after option 2.**

## Option 6: iCE40 gateware and ESP32-C3

| Part | Fallback today | Gap |
|---|---|---|
| iCE40 boot image (W25Q 0x000000) | `psram_boot_selftest_with_recovery`: if `CDONE` low or the iCE40-to-PSRAM write fails, reflash image 0 from gw0 and retest | A cut while rewriting offset 0 heals on the next boot; needs gw0 intact |
| gw0 / gw1 slots | Header-first erase, commit word last: a cut leaves an empty slot, and offset 0 still holds the old gateware | Both offset 0 torn *and* gw0 empty needs two separate cuts; recovery then is `install-blobs` over LAN/USB |
| ESP32-C3 | No boot event in 20 s: flash once per boot from the esp slot (`esp_wifi_ctrl.c`); the C3's ROM loader cannot be erased | A cut mid-flash retries next boot. Wi-Fi only; Ethernet and USB unaffected |

These are in good shape. The STM32 firmware is the only component without a fallback.

## Comparison

| | Effort | Risk | Flash | RAM | Survives power cut | Both parts | When |
|---|---|---|---|---|---|---|---|
| 1 Harden in place | 1-2 d | low | ~1 KB | <1 KB | no (window remains) | yes | this release |
| 2 Bootloader | 1-2 wk | medium (transition, tooling) | 32 KB reserved | app: frees `.RamFunc` + 8 KB | yes, with 4 | yes | next |
| 3 Dual-bank | ~1 wk | medium | none | none | yes | **2 MB only** | not recommended |
| 4 W25Q staging | 1-2 d | low | ~1 KB | reuse | the image survives | yes | slot now, consumer next |
| 5 Confirm / rollback | 3-5 d | low-medium | in bootloader | none | yes | yes | next, after 2 |

## Open items to check on the bench or in RM0481

- Does the H5 enter the ROM bootloader on erased user flash? (decides how much option 1's sector-0-last ordering buys)
- WRP group size on the 1 MB ZGT6.
- `SWAP_BANK` sequence and `BKSEL` semantics under swap (only if option 3 is revisited).
- A RAM-resident jump to ROM DFU after the flash has been rewritten.
- Real duration of `ram_commit_all` (DWT count) on both parts.
