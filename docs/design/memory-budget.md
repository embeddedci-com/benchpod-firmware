# Memory budget (STM32H563)

Where the RAM and flash go, what each pool really uses, and where to find room when a feature
needs more. Measured on benchpod-30a9f1 (analog v3r2) with firmware 3.9.7, after the LAN hardware
tests, the SDK, MCP and OpenHTF suites, the process-node suite over LAN TLS and a cloud reconnect
with three LAN TLS sessions open. Update the tables when a change moves a number.

## Read the numbers on a pod

- JSON `status`: `heap_free`, `heap_min` (lowest since boot), `stack_min`, `stacks` (free bytes per
  task, lowest since boot), `lwip_mem_max` / `lwip_mem_size`, `pbuf_pool_max` / `pbuf_pool_size`.
- Console `status`: the same as `stack`, `lwip` and `heap` lines.
- Build: `arm-none-eabi-size -A build/bench_pod_stm32.elf`, and the largest symbols with
  `arm-none-eabi-nm -S --size-sort -t d build/bench_pod_stm32.elf | tail -30`.

The `_min` / `_max` values are since boot. Read them after the workload you care about.

## Flash (2 MB part; a 1 MB part has the same code budget)

| Region | Size | Use |
|---|---|---|
| Code (`.text` + `.rodata` + vectors) | 676 KB of 896 KB (75%) | Firmware, embedded gateware images, ESP32-C3 image. `0x08000000`-`0x080DFFFF`, cached by the ICACHE. |
| Persistence records | top 128 KB of the chip | Config, cloud config, identity, ADC calibration, DAC limits, pod policy, LAN TLS certificate, OTA self-test scratch. Not cached (MPU); `flash_compat` refuses writes anywhere else. |

The 896 KB code limit is fixed by the linker script and by the start of the non-cacheable region
(`main.c icache_init`, a linker `ASSERT` keeps the two together). Large data (DAC waveforms,
captures, the EMI run log, staged OTA images, gateware blobs) lives in the 8 MB PSRAM or the W25Q.

## RAM: 640 KB, about 99.8% assigned at link time

Almost all of it is pools sized at build time. What matters is how full each pool gets.

| Pool | Size | Peak used | Notes |
|---|---|---|---|
| FreeRTOS heap (`ucHeap`, `configTOTAL_HEAP_SIZE`) | 248 KB | 181 KB (66.7 KB free at worst) | mbedTLS (cloud link, LAN TLS: 22,000 B per open LAN session), task stacks, cloud buffers. 155 KB free when idle and connected. |
| lwIP heap (`ram_heap`, `MEM_SIZE`) | 64 KB | 20 KB | TCP send data (`PBUF_RAM`). |
| Ethernet RX pool (`RX_POOL`) | 25 KB | | 16 x 1.5 KB zero-copy receive buffers (`ETH_RX_BUFFER_CNT`). |
| lwIP PBUF pool (`PBUF_POOL`) | 25 KB | 2 of 16 | 16 x 1.5 KB (lwIP default). |
| `adc_pool` | 64 KB | full by design | The one sample/trace buffer shared by JSON, SCPI and the console. |
| Other fixed buffers | about 110 KB | | Largest: per-connection TX rings (`conn_tx.c s_rings`, 16 KB), cloud receive buffer (`cloud_client.c s_rx`, 16 KB), JSON line assembly per connection (`command_handler_transport.c line_asm`, 12.6 KB), ESP32-C3 SPI TX (`esp_hosted_spi.c`, 9.4 KB), USB NCM endpoints (9.4 KB), USB CDC RX + TX (8.9 KB), signal engine sample buffers (16 KB), OTA commit sector copy (`ota_commit.c s_sector`, 8 KB). |

Task stacks (free bytes at worst): `IDLE` 400, `usb` 2,792, `console` 3,680, `hw` 7,008,
`net` 12,064.

## Floors that must hold

- **Cloud reconnect room: 48 KB of FreeRTOS heap** (`LAN_TLS_HEAP_FLOOR`, `net_server.c`). A new LAN
  TLS session is admitted only if the heap stays above the floor plus 24 KB for the session. Lowest
  seen: 66.7 KB, so about 19 KB above the floor.
- **Stacks:** keep every task at 256 B free or more at worst. `IDLE` (400 B) is the closest.

## Where to find room later (not done; measure first)

In order of how cheap they are. Each needs the worst case measured before shrinking: a cloud
bulk capture (the second cloud TLS session), Wi-Fi traffic through the ESP32-C3, three LAN TLS
sessions and a deep capture at once.

1. **lwIP heap, up to 32 KB.** Peak 20 KB of 64 KB in LAN and SDK use. Measure `lwip_mem_max`
   during a cloud bulk capture and Wi-Fi traffic first; keep at least 50% headroom above that peak.
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
