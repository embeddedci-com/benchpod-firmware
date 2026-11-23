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
| Code (`.text` + `.rodata` + vectors) | 859,680 B of 896 KB (93.7%, 57,824 B left) in a release build; 871,800 B (95.0%) in a dev build | Firmware only. `0x08000000`-`0x080DFFFF`, cached by the ICACHE. |
| Persistence records | top 128 KB of the chip | Config, cloud config, identity, ADC calibration, DAC limits, pod policy, LAN TLS certificate, OTA self-test scratch. Not cached (MPU); `flash_compat` refuses writes anywhere else. |

Measured 2026-11-19 on main `a594c0e` (firmware 3.10.13): the release build is
`make -C ice40 release-images && make -C stm32h563 RELEASE=1`, the dev build `make -C stm32h563`.
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

## Code that runs from the W25Q (XIP): two images, one RAM region

Rarely used code is linked into images that live in W25Q blob slots instead of internal flash
(`src/xip_image.h`, `src/xip_session.h`). There are two, each with its own table, expected hash
(patched into internal flash by `tools/xip_finish.py`) and blob slot:

| Image | Section | Slot (W25Q) | Code | Size (release = dev build) | Of the 20 KB region |
|---|---|---|---|---|---|
| usb | `.xip` | `xip` (0x200000) | the USB flash protocols (`xip_mod_*.c`) | 17,684 B | 86.4% |
| prog | `.xip_prog` | `xip_prog` (0x240000) | the five `prog_flash` pin programmers (`prog_updi.c`, `prog_pic.c`, `prog_c2.c`, `prog_tpi.c`, `prog_pdi.c`) and the dispatch (`xip_prog_mod.c`) | 11,480 B | 56.0% |

- **XIP_RAM=1 (the release default, every board shipped so far):** both images are linked at the
  same 20 KB RAM region (`0x2009B000`, taken from the FreeRTOS heap), an overlay. A session names
  its image and `xip_session_begin(img)` copies that image's slot into the region after checking
  its length and SHA-256. Only one session runs at a time (a `usb_flash` or a `prog_flash` job
  holds its image until it ends), so the second image costs no RAM; each image only has to fit
  the 20 KB. The prog image has 9 KB of room, the usb image 2.8 KB.
- **XIP_RAM=0 (memory-mapped, digital v3r4):** each image at its own W25Q address (usb
  `0x90201000`, prog `0x90241000`), no copy. The linker script keeps three regions for this: `XIP`,
  `XIP_PROG`, and the RAM build's sed swaps both for the same RAM region.
- Moving the protocols out of internal flash saved 10,136 B of code in both builds:

| Build | Internal flash before | after | Static RAM before | after |
|---|---|---|---|---|
| release (`make RELEASE=1`) | 896,884 B (97.75%) | 886,748 B (96.65%) | 630,152 B | 630,216 B (+64) |
| dev (`make`) | 910,412 B (99.23%) | 900,276 B (98.12%) | 634,472 B | 634,568 B (+96) |

  (`prog_flash` branch `claude/prog-xip-ec5da4`, on top of PR 239, 2026-11-23. The static RAM
  difference is the second image's table descriptor and the job's session bookkeeping.)
- What stays in internal flash: the protocol metadata (`prog_flash.c`: name, uart, uses_nrst,
  mems), `prog_job.c`, `prog_wire.c` (its functions are the callbacks the job stores in
  `prog_wire_ops_t`, and internal code may not hold pointers to XIP functions) and the command.
- Rules for XIP code are in `xip_image.h` and checked after every link by
  `tools/check_xip_refs.py`: internal code reaches an image only through its table, an image
  hands out no pointer to its own functions, XIP objects have no `.data`/`.bss`.

## RAM: 640 KB, 99.1% assigned at link time

Static RAM is 649,296 B (99.07%) in the release build and 653,424 B (99.70%) in the dev build
(main `a594c0e`). Almost all of it is pools sized at build time. What matters is how full each
pool gets. The "Peak used" column and the stack figures below are from the 2026-11-20 run (see the
top of this page).

| Pool | Size | Peak used | Notes |
|---|---|---|---|
| FreeRTOS heap (`ucHeap`, `configTOTAL_HEAP_SIZE`) | 259 KB (243 KB up to 4.0.0) | 209 KB of 259 KB (49.9 KB free at worst; 33.5 KB of 243 KB on 4.0.0) | mbedTLS (cloud link, LAN TLS: 22,000 B per open LAN session), task stacks, cloud buffers. 164.8 KB free when idle and connected (148.4 KB on 4.0.0). |
| lwIP heap (`ram_heap`, `MEM_SIZE`) | 48 KB (64 KB up to 4.0.0) | 19.4 KB | TCP send data (`PBUF_RAM`). |
| Ethernet RX pool (`RX_POOL`) | 25 KB | | 16 x 1.5 KB zero-copy receive buffers (`ETH_RX_BUFFER_CNT`). |
| lwIP PBUF pool (`PBUF_POOL`) | 25 KB | 2 of 16 | 16 x 1.5 KB (lwIP default). |
| `adc_pool` | 64 KB | full by design | The one sample/trace buffer shared by JSON, SCPI and the console. |
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
