# Vendored esp-hosted-mcu wire-format headers

These are the authoritative ESP-Hosted transport definitions, copied **verbatim**
from Espressif's [esp-hosted-mcu](https://github.com/espressif/esp-hosted-mcu)
(Apache-2.0). The STM32 host talks to the on-board ESP32-C3 (running the stock
esp-hosted-mcu *slave* firmware) over SPI using this framing.

| File | Upstream path |
|------|---------------|
| `common/esp_hosted_header.h`    | `common/esp_hosted_header.h`    |
| `common/esp_hosted_interface.h` | `common/esp_hosted_interface.h` |

- Upstream commit: `8f0770d39065c2a9ff6828268709c3502e0d5349` (2026-06-19)
- We deliberately vendor **only** the wire-format headers — not the ESP-IDF-coupled
  host driver. Our lean SPI host (`src/esp_hosted_frame.*`, `src/esp_hosted_spi.*`)
  is built on top of these structs. Re-sync these two files (and re-check the frame
  codec) when bumping the slave firmware version.
