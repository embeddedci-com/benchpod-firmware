# Dependencies

Everything the STM32H563 firmware and the iCE40 gateware are built from, where each one is
pinned, and whether Dependabot sees it. Update this file in the same change as a bump.

Dependabot (`.github/dependabot.yml`) checks the git submodules monthly, the workflow actions
weekly and the calibration scripts' Python requirements monthly. It opens security updates on its
own. A submodule bump needs a firmware build, the host tests and a bench run before it merges.

## Firmware libraries (git submodules, seen by Dependabot)

| Library | Path | Pinned at | Notes |
|---|---|---|---|
| STM32H5 HAL | `stm32h563/lib/stm32cube/stm32h5xx_hal_driver` | v1.6.0 (`2e49bfe`) | The commit STM32CubeH5 v1.6.0 pins. |
| CMSIS device (STM32H5) | `stm32h563/lib/stm32cube/cmsis_device_h5` | v1.6.0 (`4a9d636`) | The commit STM32CubeH5 v1.6.0 pins. |
| CMSIS core | `stm32h563/lib/stm32cube/cmsis_core` | v5.9.0_20250520 | `Include/` is byte-identical to STM32CubeH5 v1.6.0's `Drivers/CMSIS/Include` (Core 5.6). Dependabot ignores it: bump it by hand with the HAL. |
| LAN8742 PHY driver | `stm32h563/lib/stm32cube/stm32-lan8742` | v1.0.4 (`9f365d0`) | The commit STM32CubeH5 v1.6.0 pins. |
| mbedTLS (ST package) | `stm32h563/lib/stm32cube/stm32-mw-mbedtls` | v3.6.6_20260511 | TLS for the cloud link and the LAN TLS server, SHA-256 for OTA and blobs. Ahead of STM32CubeH5 v1.6.0 (3.6.4) for the 3.6.5 and 3.6.6 security fixes. |
| FreeRTOS kernel | `stm32h563/lib/FreeRTOS-Kernel` | `ca75944` (main) | Port `GCC/ARM_CM33_NTZ`. |
| lwIP (ST package) | `stm32h563/lib/stm32_mw_lwip` | v2.2.1_20250804 | Patched altcp TLS glue lives in `src/altcp_tls_bp/`. |
| TinyUSB | `stm32h563/lib/tinyusb` | `e477e9f` (master) | CDC + NCM device. |
| scpi-parser | `stm32h563/lib/scpi-parser` | `8866159` | |
| Monocypher | `stm32h563/lib/monocypher` | 4.0.3+ (`ab2b16d`) | Ed25519 firmware and blob signatures. |

Until firmware 3.9.6 the HAL, CMSIS, LAN8742 and mbedTLS came from a separate STM32CubeH5 v1.6.0
checkout (`CUBE=`), which Dependabot could not see. `STM32CUBE=<dir>` still points the build at
another copy laid out like `lib/stm32cube/`.

## Not seen by Dependabot (check by hand)

| What | Pinned in | Pinned at | How to check |
|---|---|---|---|
| ESP32-C3 esp-hosted slave: ESP-IDF image | `stm32h563/Makefile` (`fetch`) | `espressif/idf:release-v5.5@sha256:cb51f786...` | ESP-IDF release notes and security advisories. The image is pinned by digest, so a rebuild is reproducible. |
| ESP32-C3 esp-hosted slave: components | `esp32-hosted-slave/dependencies.lock` | see the lock file | `idf.py update-dependencies` in the pinned image. |
| ESP32-C3 slave image | `esp32-hosted-slave/prebuilt/*.sha256` | the hash of the last build | Rebuild with `make -C stm32h563 fetch` after either row above changes. |
| ARM GCC (CI, release) | `.github/workflows/stm32-build.yml`, `release-stm32.yml` | 13.3.Rel1 | Arm GNU Toolchain releases. |
| OSS CAD Suite (yosys, nextpnr) | `.github/workflows/test.yml`, `ice40-seed-sweep.yml` | 2026-05-13 (yosys 0.65) | Stay on 0.65: yosys 0.66's abc9 change places the up5k about 7 MHz worse. A bump needs a seed sweep in both toolchains. |

## Open

- **STM32CubeH5 v1.7.0** (HAL, CMSIS device, mbedTLS 3.6.6). Taking mbedTLS on its own was
  the security fix. The v1.7.0 HAL changes 66 files, including `stm32h5xx_hal_eth.c`, which the
  Ethernet restart fix in firmware 3.9.7 depends on, so it gets its own release and a full bench
  run. Dependabot opened the HAL and CMSIS device bumps (#112, #113); take CMSIS core with them.

## RAM and flash

How full each memory pool gets and where to find room: [design/memory-budget.md](design/memory-budget.md).
