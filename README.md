# bench-pod-firmware

Firmware and gateware for the **EmbeddedCI BenchPod** — a network-attached
bench instrument that exposes a calibrated DAC/ADC, a logic analyzer, digital
IO, an I²C sensor bench, CAN, and an SWD debug probe over a JSON/SCPI API, on
the LAN and through the EmbeddedCI cloud.

The board ("vbench-pod", v2) is a **two-chip** system:

- An **STM32H563ZIT6** runs the C firmware (FreeRTOS + LwIP): Ethernet + optional
  ESP32-C3 Wi-Fi, a cloud WebSocket control channel + byte-tunnel (TLS), a
  JSON-over-TCP / SCPI command server, a USB-CDC console, and an on-pod
  CMSIS-DAP SWD probe.
- A **Lattice iCE40UltraPlus (iCE40UP5K-SG48)** FPGA is an SPI slave to the
  STM32 and owns the timing-critical analog + digital datapath: it drives the
  16-bit serial DAC (DAC8551) and samples the 16-bit serial ADC (MCP33131D-10)
  and the 12-channel logic-analyzer bank, streaming captures into an APS6404L
  PSRAM that the STM32 reads back over its memory-mapped XSPI.

The two chips are joined by an SPI link (SPI1) plus a shared quad bus to the
PSRAM. Analog front-end, power (INA238 + eFuse), I²C mux (TCA9554), and CAN
(FDCAN1 + TCAN1044) are on the STM32 side.

```
bench-pod-firmware/
├── stm32h563/   ← STM32H563 firmware (C, FreeRTOS + LwIP + ST USB) — THE firmware
├── ice40/       ← iCE40 gateware (Verilog, IceStorm toolchain) — v2
├── docs/        ← protocol + design docs (API.md, datapath reviews, …)
└── calibration/ ← per-board DAC/ADC cal (measured against a 34401A DMM)
```

> **History:** the original BenchPod ran on an **RP2350** (Raspberry Pi Pico 2)
> with a parallel-ADC "v1" iCE40 gateware. That firmware and the v1 gateware were
> **retired (2026-07-09)**; the STM32H563 port + v2 gateware are the only
> supported build. See the git history and `docs/` for the migration record.

## Firmware — `stm32h563/`

Bare-metal-ish C on **FreeRTOS**, with **LwIP** for networking and ST's USB
device stack for the console. Highlights:

- **Connectivity.** Ethernet (LAN8742 PHY, DHCP-only) is primary; an on-board
  **ESP32-C3** provides Wi-Fi over `esp-hosted` (SPI4), unified with the eth
  netif (eth-priority). The pod also flashes its own blank C3 over the ROM
  bootloader.
- **Cloud.** A direct **WSS** control channel to the EmbeddedCI server (LwIP
  `altcp_tls` + mbedTLS, cert-pinned) carries commands, plus a **byte-tunnel**
  that streams the full capture API (scope / analyzer / flash) through the cloud
  exactly as a local TCP client would.
- **Command API.** JSON-over-TCP (port 8080) and SCPI, dispatched by a
  worker-task transport (`hw_worker` owns command execution + hardware; the net
  task owns LwIP). See [`docs/API.md`](docs/API.md).
- **Instruments.** Calibrated DAC out + ADC read (v2 `dac_out` / `adc_read` / `analog_path`, see [`docs/API.md`](docs/API.md)),
  deep ADC/LA capture to PSRAM, arbitrary DAC replay, I²C sensor emulation +
  logic-analyzer decode, stepper/GPIO, CAN, and a network **CMSIS-DAP** SWD probe
  (`dap.c` + the iCE40 `swd_engine`).
- **OTA.** PSRAM-staged, SHA-256-verified firmware images over WS/LAN
  (`docs/` / `ota.c`); the RAM-resident PSRAM→flash commit is a guarded stub.

Build + flash (needs `arm-none-eabi-gcc`, `dfu-util`):

```
cd stm32h563
make all                       # build .elf/.hex/.bin (embeds the iCE40 v2 bitstream)
make fetch                     # one-time: fetch the ESP32-C3 esp-hosted slave blob
printf 'dfu\r\n' > <usb-cdc>   # console cmd → ROM DFU  (or hold BOOT0 on a blank board)
make flash-dfu                 # dfu-util writes flash and reboots into the app
```

The STM32 can also reprogram the iCE40's config flash from its embedded copy of
the bitstream (`flash-ice40` console command).

## Gateware — `ice40/`

The v2 iCE40 design (`src/top_v2.v`) is the SPI slave that owns the analog +
digital datapath. Two producers (ADC and the 12-ch LA) capture off a shared
trigger into per-stream SPRAM rings, a single 48 MHz DDR writer drains both into
two PSRAM regions, and the STM32 reads the result back over XSPI. A separate
concurrent path shares the PSRAM bus between DAC replay, ADC capture and LA
capture at once. See [`ice40/README.md`](ice40/README.md) and
[`docs/tri-capture-unified-psram.md`](docs/tri-capture-unified-psram.md).

## Security

The **LAN command API (JSON/TCP 8080 and SCPI) is unauthenticated and unencrypted** —
any host that can reach the port gets the full command set, including target power,
SWD access to an attached target, and pod OTA. Treat it as a trusted-network-only
interface (lab VLAN or firewall rule), the same way you would a SCPI-over-LAN scope.

The **cloud channel is authenticated and encrypted**: outbound-only TLS with the
server chain verified against Let's Encrypt roots embedded in the firmware, and the
pod authenticating itself by Ed25519 challenge–response with a key generated
on-device that never leaves it. LAN and cloud signatures are domain-separated, so a
proof obtained over the LAN cannot be replayed as a cloud login.

Full detail — including the certificate-expiry caveat — is in
[`docs/API.md` § Security model](docs/API.md#security-model).

## Docs

- [`docs/API.md`](docs/API.md) — the JSON/SCPI command protocol.
- [`docs/tri-capture-unified-psram.md`](docs/tri-capture-unified-psram.md) — the concurrent DAC/ADC/LA PSRAM datapath.
- [`docs/dac-control-loop.md`](docs/dac-control-loop.md) — in-fabric DAC closed-loop control.
- [`docs/dap-over-tunnel.md`](docs/dap-over-tunnel.md) — CMSIS-DAP over the cloud tunnel.
- [`docs/usb-serial-interface.md`](docs/usb-serial-interface.md) — the USB CDC-ACM console.
- [`docs/solarpanel-simulation.md`](docs/solarpanel-simulation.md) — analog front-end calibration.
