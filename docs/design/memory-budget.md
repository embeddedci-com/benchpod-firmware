# Memory budget (STM32H563)

Where the RAM and flash go, what each pool really uses, and where to find room when a feature
needs more. Update the tables when a change moves a number.

- **Link-time sizes** (flash, static RAM, the FreeRTOS heap size) are from the build of firmware
  3.10.13 (main `a594c0e`, 2026-11-19).
- **Run-time peaks** (heap and pool high-water marks, task stack low-water marks) are from
  2026-11-20 on benchpod-c2f662 (v3 board with an analog module), firmware 4.0.0 and then 4.0.0
  with this page's heap split: the SDK LAN e2e suite over TLS with two more LAN TLS sessions held
  open, then the SDK cloud tier (tunnel captures, UART, waveform replay) with three LAN TLS
  sessions open. Not covered yet: Wi-Fi traffic through the ESP32-C3.

## Read the numbers on a pod

- JSON `diag` (firmware 3.10.9+; `status` up to 3.10.8): `heap_free`, `heap_min` (lowest since boot), `stack_min`, `stacks` (free bytes per
  task, lowest since boot), `lwip_mem_max` / `lwip_mem_size`, `pbuf_pool_max` / `pbuf_pool_size`.
- Console `status`: the same as `stack`, `lwip` and `heap` lines.
- Build: `arm-none-eabi-size -A build/bench_pod_stm32.elf`, and the largest symbols with
  `arm-none-eabi-nm -S --size-sort -t d build/bench_pod_stm32.elf | tail -30`.

The `_min` / `_max` values are since boot. Read them after the workload you care about.

## Flash (2 MB part; a 1 MB part has the same code budget)

| Region | Size | Use |
|---|---|---|
| Code (`.text` + `.rodata` + vectors) | 865,572 B of 896 KB (94.3%, 51,932 B left) in a release build (2026-11-25); 871,800 B (95.0%) in a dev build | Firmware only. `0x08000000`-`0x080DFFFF`, cached by the ICACHE. |
| Persistence records | top 128 KB of the chip | Config, cloud config, identity, ADC calibration, DAC limits, pod policy, LAN TLS certificate, OTA self-test scratch. Not cached (MPU); `flash_compat` refuses writes anywhere else. |

Measured 2026-11-19 on main `a594c0e` (firmware 3.10.13): the release build is
`make -C ice40 release-images && make -C stm32h563 RELEASE=1`, the dev build `make -C stm32h563`.
Measured 2026-11-25 on main `4b3d6cd` (firmware 4.0.6), "flash left" as the build prints it:
895,820 of 917,504 B used (21,684 left, below the 24 KB floor) before, 865,572 B used (51,932 left)
after three no-behavior-change steps:

| Step | Flash left | Change |
|---|---|---|
| main `4b3d6cd` | 21,684 B | |
| `wave16.c` uses `sd_sin` (the sensor engine's own sine) instead of libm `sin`: no double-precision libm in the image; 0 of 6e8 samples differ in the 16-bit codes | 26,644 B | +4,960 |
| `-Wl,--wrap=HAL_RCCEx_GetPeriphCLKFreq` (`src/uart_clk.c`): the two HAL-initialized UARTs (USART2 console, USART1 C3 flasher) are on their PCLK kernel clock; drops the HAL clock-tree walker and `HAL_RCCEx_GetPLLxClockFreq` | 33,740 B | +7,096 |
| Cold files at `-Os` (`COLD_OS_OBJS` in the Makefile: console, calibration/limits, OTA, Wi-Fi bridge, usb_check, LAN TLS setup, sensor compile/expr/json, lwIP dhcp and mDNS); `scpi_server.c`, `cloud_client.c` and everything on a data path stay `-O2` | 51,932 B | +18,192 |

The 3.10.10 release build was 839,216 B: 20 KB more in three patch releases.
The gateware images and the ESP32-C3 image are not in this region: they are blobs in the W25Q
slots, which the firmware build stages in `build/blobs/` and an installer writes to the pod
(`stm32h563/Makefile`).

The largest flash users in the release build:

| What | Size |
|---|---|
| Merged string literals (`.rodata.str1.1`) | about 133.8 KB |
| `signal_engine` | 28 KB |
| `sensor_def_vm` | 20 KB |
| `sensor_def_compile` | 19.6 KB |
| `sensor_mgr` | 18.4 KB |
| `monocypher` | 17 KB |

The 896 KB code limit is fixed by the linker script and by the start of the non-cacheable region
(`main.c icache_init`, a linker `ASSERT` keeps the two together). Large data (DAC waveforms,
captures, the EMI run log, staged OTA images, gateware blobs) lives in the 8 MB PSRAM or the W25Q.

## Code that runs from the W25Q (XIP): two images, one RAM region (the adc_pool)

Rarely used code is linked into images that live in W25Q blob slots instead of internal flash
(`src/xip_image.h`, `src/xip_session.h`). There are two, each with its own table, expected hash
(patched into internal flash by `tools/xip_finish.py`) and blob slot:

| Image | Section | Slot (W25Q) | Code | Size (release = dev build) | Of the 64 KB region |
|---|---|---|---|---|---|
| usb | `.xip` | `xip` (0x200000) | the USB flash protocols (`xip_mod_*.c`) | 17,700 B | 27.0% |
| prog | `.xip_prog` | `xip_prog` (0x240000) | the six `prog_flash` pin programmers (`prog_updi.c`, `prog_pic.c`, `prog_c2.c`, `prog_tpi.c`, `prog_pdi.c`, `prog_sbw.c`) and the dispatch (`xip_prog_mod.c`) | 16,340 B | 24.9% |

- **XIP_RAM=1 (the release default, every board shipped so far):** both images are linked over
  `adc_pool` (the 64 KB shared sample buffer, below), an overlay of all three. From 2026-11-25 the
  pool has a fixed region of its own at the end of RAM (linker region `POOL`, 64 KB at
  `0x20090000`, a NOLOAD `.adc_pool` section; RAM is 576 KB, `_estack` is the pool's start), and
  `XIP` and `XIP_PROG` are linked at that same address with the pool's length. Until then the
  images had a separate 20 KB region (`0x2009B000`) taken from the FreeRTOS heap: that is back in
  the heap, and an image may now be up to 64 KB (`XIP_RAM_KB` in the Makefile; the link asserts it
  equals the pool's size, that both images start at the pool and that each fits it). A session names
  its image and `xip_session_begin(img)` copies that image's slot over the pool after checking
  its length and SHA-256. Only one session runs at a time (a `usb_flash` or a `prog_flash` job
  holds its image until it ends), so the second image costs no RAM.
  The pool is idle then: `heavy_in_flight()` includes `xip_session_active()`, so every pool user
  that runs under the heavy gate is refused (BP0048); `adc_pool_overlay_begin()` makes begin refuse
  (`XIP_ERR_POOL_BUSY`) while a capture, a paced send or a `load_bin` upload is mid-way, and takes a
  new pool generation so a kept trace (a SCPI `READ?`, a JSON `capture`/`load`) is gone: its replay
  says "nothing to replay" or refuses. The pool does not feed the DAC or any DMA after the call
  that arms it (`fpga_load_wave` copies it into the FPGA BRAM with a blocking SPI write). Code that
  must survive a session may not keep data in the pool.
- **XIP_RAM=0 (memory-mapped, digital v3r4):** each image at its own W25Q address (usb
  `0x90201000`, prog `0x90241000`), no copy. The linker script keeps three regions for this: `XIP`,
  `XIP_PROG`, and the RAM build's sed swaps both for the pool's RAM region (and adds `POOL`).
- Moving the protocols out of internal flash saved about 10 KB of code in both builds, and no static RAM:

| Build | Internal flash before | after | Static RAM before | after |
|---|---|---|---|---|
| release (`make RELEASE=1`) | 896,884 B (97.75%) | 886,708 B (96.64%) | 630,152 B | 630,152 B |
| dev (`make`) | 910,412 B (99.23%) | 900,228 B (98.12%) | 634,472 B | 634,472 B |

  (Measured 2026-11-23 on the branch that adds the second image, on top of the `reset` pin
  change of PR 239. The expected-hash records of the two images are in internal flash and are
  also the blobs' manifest entries, so the image bookkeeping takes no RAM.) The EMC test build
  (`EMI_TEST=1`) overflows its RAM by 24 B before and after this change.
- Images over the adc_pool (2026-11-25, main `4b3d6cd` plus this change; the table is the 20 KB
  region against the pool overlay):

| Build | FreeRTOS heap before | after | Static RAM before (of) | after (of) | Internal flash left before | after |
|---|---|---|---|---|---|---|
| release (`make RELEASE=1`) | 233 KB | 253 KB | 629,672 B (620 KB) | 584,616 B (576 KB) | 21,684 B | 21,508 B |
| dev (`make`) | 233 KB | 253 KB | 634,024 B (620 KB) | 588,968 B (576 KB) | about 8,020 B | 7,844 B |

  Static RAM now excludes the pool (it is its own 64 KB region), so the free RAM at link time is the
  same as before (5,208 B release): the 20 KB moved from the region to the heap, the pool moved
  from `.bss` to `POOL`. Flash grew by 176 B (the pool takeover and one error code).
- What stays in internal flash: the protocol metadata (`prog_flash.c`: name, uart, uses_nrst,
  mems), `prog_job.c`, `prog_wire.c` (its functions are the callbacks the job stores in
  `prog_wire_ops_t`, and internal code may not hold pointers to XIP functions) and the command.
- Rules for XIP code are in `xip_image.h` and checked after every link by
  `tools/check_xip_refs.py`: internal code reaches an image only through its table, an image
  hands out no pointer to its own functions, XIP objects have no `.data`/`.bss`.

## RAM: 640 KB, 99.1% assigned at link time (576 KB + the 64 KB adc_pool region in XIP_RAM builds)

Static RAM is 649,296 B (99.07%) in the release build and 653,424 B (99.70%) in the dev build
(main `a594c0e`). Almost all of it is pools sized at build time. What matters is how full each
pool gets. The "Peak used" column and the stack figures below are from the 2026-11-20 run (see the
top of this page).

| Pool | Size | Peak used | Notes |
|---|---|---|---|
| FreeRTOS heap (`ucHeap`, `configTOTAL_HEAP_SIZE`) | 259 KB (243 KB up to 4.0.0); 2026-11-25 builds with USB host mode: 253 KB (233 KB before the XIP images moved onto the adc_pool) | 209 KB of 259 KB (49.9 KB free at worst; 33.5 KB of 243 KB on 4.0.0) | mbedTLS (cloud link, LAN TLS: 22,000 B per open LAN session), task stacks, cloud buffers. 164.8 KB free when idle and connected (148.4 KB on 4.0.0). |
| lwIP heap (`ram_heap`, `MEM_SIZE`) | 48 KB (64 KB up to 4.0.0) | 19.4 KB | TCP send data (`PBUF_RAM`). |
| Ethernet RX pool (`RX_POOL`) | 25 KB | | 16 x 1.5 KB zero-copy receive buffers (`ETH_RX_BUFFER_CNT`). |
| lwIP PBUF pool (`PBUF_POOL`) | 25 KB | 2 of 16 | 16 x 1.5 KB (lwIP default). |
| `adc_pool` | 64 KB | full by design | The one sample/trace buffer shared by JSON, SCPI and the console. In XIP_RAM=1 builds also where the W25Q images run (above): its own `POOL` region at 0x20090000, 64 KB. |
| Other fixed buffers | about 110 KB | | Largest: per-connection TX rings (`conn_tx.c s_rings`, 16 KB), cloud receive buffer (`cloud_client.c s_rx`, 16 KB), JSON line assembly per connection (`command_handler_transport.c line_asm`, 12.6 KB), ESP32-C3 SPI TX (`esp_hosted_spi.c`, 9.4 KB), USB NCM endpoints (9.4 KB), USB CDC RX + TX (8.9 KB), signal engine sample buffers (16 KB), OTA commit sector copy (`ota_commit.c s_sector`, 8 KB). |

Task stacks (free bytes at worst, 2026-11-20 run): `IDLE` 400, `usb` 2,672, `console` 2,208,
`hw` 4,924, `net` 11,936.

## Floors that must hold

- **Cloud reconnect room: 48 KB of FreeRTOS heap** (`LAN_TLS_HEAP_FLOOR`, `net_server.c`). A new LAN
  TLS session is admitted only while the free heap is above the floor plus 2 KB for the handshake
  (the session's own 22,000 B are already taken when the check runs). Lowest seen: 49.9 KB with the
  259 KB heap, just above the floor. With the old 243 KB heap the same load went down to 33.5 KB,
  under the floor, and new LAN TLS clients were refused until it recovered.
- **LAN TLS sessions: at most 3** (`LAN_TLS_MAX_SESSIONS`). A 4th costs 22 KB the heap does not have
  at worst, so it stays at 3. The SDK opens one connection per command and keeps one per UART
  session or capture, so a test that holds two sessions needs both free slots.
- **Stacks:** keep every task at 256 B free or more at worst. `IDLE` (400 B) is the closest.

## Where to find room later (not done; measure first)

In order of how cheap they are. Each needs the worst case measured before shrinking: a cloud
bulk capture (the second cloud TLS session), Wi-Fi traffic through the ESP32-C3, three LAN TLS
sessions and a deep capture at once.

1. **lwIP heap: done (16 KB to the FreeRTOS heap).** Peak 19.4 KB of 48 KB with LAN TLS, SDK and
   cloud tunnel captures. Another 16 KB is possible after `lwip_mem_max` is measured with Wi-Fi
   traffic; keep at least 50% headroom above the peak.
2. **lwIP PBUF pool, up to about 15 KB.** Peak 2 of 16 buffers here, but the Wi-Fi (ESP32-C3) receive
   path and the USB network adapter may use it; measure `pbuf_pool_max` on a Wi-Fi pod.
3. **LAN TLS input buffer.** 16 KB per session (`MBEDTLS_SSL_IN_CONTENT_LEN`), because Python's
   `ssl` and Go's `crypto/tls` do not negotiate a smaller maximum fragment length. Only an option
   if the clients change.
4. **OTA commit sector copy (8 KB).** Only used during the in-place OTA commit. It could share a
   buffer that is idle then (check that the commit's RAM-resident code can reach it).
5. **Other fixed buffers.** Sized for the worst concurrent case; review each before shrinking.

The FreeRTOS heap is what features normally grow into. Room taken from a static pool can be
given to it through `configTOTAL_HEAP_SIZE`.
