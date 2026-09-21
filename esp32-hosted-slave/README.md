# ESP32-C3 esp-hosted slave firmware (Wi-Fi co-processor)

The vbench-pod board's **ESP32-C3-MINI-1 (U4)** runs Espressif's stock
[esp-hosted-mcu](https://github.com/espressif/esp-hosted-mcu) **slave** firmware
and acts purely as a Wi-Fi NIC for the STM32H563 host over SPI. The STM32 keeps
LwIP/TCP-IP and the whole cloud stack; the ESP32 is just the radio.

The slave source is unmodified upstream — this directory holds the **build
recipe**, the board `sdkconfig` overrides, and the **prebuilt merged image** that
the STM32 embeds and flashes itself (see "On-device flash" below).

> Host side: `../stm32h563/src/esp_hosted_spi.c` (SPI transport), `esp_netif.c`
> (LwIP netif), `esp_wifi_ctrl.c` (RPC). Flasher: `../stm32h563/src/esp_rom_flash.c`.
> Wire-format headers vendored under `../stm32h563/lib/esp_hosted/`.

## Wiring (netlist `../stm32h563/vbench-pod.net`, U4=C3, U5=STM32)

| Signal | STM32H563 (U5) | ESP32-C3 (U4) | Used for |
|--------|----------------|---------------|----------|
| SCLK   | PE12 (SPI4)    | IO6  | SPI transport |
| MISO   | PE13 (SPI4)    | IO2  | SPI transport |
| MOSI   | PE14 (SPI4)    | IO7  | SPI transport |
| CS     | PB2  (GPIO)    | IO10 | SPI transport |
| HANDSHAKE  | PB0 (in)   | IO0  | SPI transport |
| DATA_READY | PB1 (in)   | IO1  | SPI transport |
| EN (reset) | PF11 (out) | EN   | reset / power-gate |
| BOOT       | PF12 (out) | IO9  | download strap |
| UART TX    | PA9 (USART1)  | U0RXD (pin30) | ROM flash |
| UART RX    | PA10 (USART1) | U0TXD (pin31) | ROM flash |

There is **no programming header** — the C3's UART and straps reach only the
STM32, so the pod flashes the C3 itself over its ROM bootloader.

## Build (ESP-IDF v5.3+; we build with the v5.5 Docker image)

The merged image is a ~1.07 MB vendored binary and is **not committed** (git
ignores `*.bin`). The STM32 build `.incbin`s it, so a fresh checkout must produce
it first — otherwise `make` in `../stm32h563` fails loudly with the command to
run. From the firmware dir:

```sh
cd ../stm32h563 && make fetch          # reproducible ESP-IDF Docker build
# or, to download a prebuilt asset instead of running Docker:
cd ../stm32h563 && make fetch ESP_SLAVE_URL=https://…/esp32c3-hosted-slave-merged.bin
```

`make fetch` verifies the result against `prebuilt/esp32c3-hosted-slave-merged.bin.sha256`.

The slave GPIO map is baked at compile time, so a generic prebuilt won't match —
it's built against `sdkconfig.defaults.board`. `make fetch` wraps this container
build (equivalent to running it by hand):

```sh
docker run --rm -v "$PWD:/board" -v "$PWD/work:/work" -v "$PWD/prebuilt:/out" \
  espressif/idf:release-v5.5 bash /board/build.sh
```

`build.sh` runs `idf.py create-project-from-example "espressif/esp_hosted==2.12.9:slave"`
(component v2.12.9 == upstream commit `8f0770d`, 2026-06-19 — matches the vendored
host headers), applies `sdkconfig.defaults.board`, sets target `esp32c3`, builds,
and emits `prebuilt/esp32c3-hosted-slave-merged.bin` (a single image for offset 0).

### Board overrides (`sdkconfig.defaults.board`) — symbol names verified

The slave's `main/Kconfig.projbuild` names these `CONFIG_ESP_SPI_*` (NOT the
`CONFIG_ESP_HOSTED_SPI_*` from the component's top-level Kconfig — an easy trap).
Only three settings differ from the stock esp32c3 slave defaults:

```
CONFIG_ESP_SPI_HOST_INTERFACE=y          # SPI full-duplex (already the default)
CONFIG_ESP_SPI_GPIO_HANDSHAKE=0          # IO0 (stock default IO3)
CONFIG_ESP_SPI_GPIO_DATA_READY=1         # IO1 (stock default IO4)
```

Already correct by default (do NOT override): SPI **mode 3**, MOSI=7, MISO=2,
CLK=6, CS=10. Verify after `set-target` with
`grep CONFIG_ESP_SPI_GPIO sdkconfig`.

## On-device flash (how the C3 actually gets programmed)

The merged image is embedded in the STM32 firmware as a `.incbin` blob
(`../stm32h563/src/esp_slave_fw.s`, ~1.07 MB) and flashed by the STM32 driving
the C3's ROM serial bootloader — mirroring `flash-ice40`. No host esptool, no
programming header. From the pod's USB-CDC console:

```
flash-esp32-sync    # strap BOOT=0 + EN pulse, SYNC — proves EN/BOOT/UART wiring
flash-esp32         # erase + write the embedded image at 0x0, on-chip MD5 verify,
                    # then reset the C3 into the new app (~2 min at 115200 baud)
```

`esp_rom_flash.c` implements the esptool SLIP command set (ROM-loader variant, no
stub): SYNC / SPI_ATTACH / FLASH_BEGIN / FLASH_DATA / FLASH_END / SPI_FLASH_MD5.

## Must-match constants (host ↔ slave)
- **SPI mode 3**, **1600-byte** transfer buffer — the host's values are in
  `../stm32h563/src/esp_hosted_spi.h` (`ESP_HOSTED_SPI_MODE`, `ESP_HOSTED_SPI_BUF_SIZE`).
- Pin esp-hosted-mcu to component **2.12.9** (commit `8f0770d`). Re-vendor
  `../stm32h563/lib/esp_hosted/common/*.h` and re-check the frame codec if bumped.

## ⚠ Bench bring-up notes
None of this is hardware-verified yet. Prove the flash path first with
`flash-esp32-sync` (SYNC alone). Then `flash-esp32`. Only after the C3 runs the
slave does the SPI link come into play — likely first-bring-up tuning there: SPI
**clock** (host `SPI_BAUDRATEPRESCALER_32` is conservative), the
HANDSHAKE/DATA_READY **polarity**, and C3 SPI-slave GPIO routability. See the
`⚠`-marked constants in `../stm32h563/src/esp_hosted_spi.{c,h}`.

To slim the image (drop ~300 KB): the stock build enables BT — set
`CONFIG_BT_ENABLED=n` in `sdkconfig.defaults.board` for a Wi-Fi-only slave.
