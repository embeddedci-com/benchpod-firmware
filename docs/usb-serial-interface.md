# bench-pod-firmware USB Serial Console

The firmware exposes its interactive console over the STM32H563's **USB CDC-ACM**
port (ST's stock CDC-ACM class device), mirrored on the USART2 debug pins. A host
tool (the `benchpod` CLI, a provisioning script) can open the serial port and
drive text commands such as `status`, `wifi-set`, `la-voltage` and `dfu`.

This document is the contract for talking to that interface. The console is the
pod's **physical-presence** channel: it can change any setting, including the ones
the LAN and the cloud cannot (loosening the signature policy, wiping the device
identity), and it is the recovery path when the pod is in safe mode.

The console does **not** run the JSON commands of [API.md](API.md). It has its own
text vocabulary, listed below; many commands are diagnostics with no JSON
counterpart.

---

## Transport

| Property | Value |
|---|---|
| Class | USB CDC-ACM (virtual serial port) |
| USB VID/PID | `0x0483:0x5740` (STMicroelectronics CDC; the host's stock CDC-ACM driver binds) |
| Line settings | `115200 8N1` |
| Framing | Line-oriented; commands terminated with `\n` (or `\r`, or `\r\n`) |
| Line length | At most 255 characters |
| Encoding | ASCII |

Because it is a standard CDC-ACM device, the host OS class driver creates a
serial port automatically: **no libusb, no custom driver, and no cgo** are
needed. The serial port appears as:

| OS | Device path |
|---|---|
| macOS | `/dev/tty.usbmodem*` (or `/dev/cu.usbmodem*`) |
| Linux | `/dev/ttyACM*` |
| Windows | `COMx` |

> CDC-ACM ignores the actual baud rate on the wire, but set `115200 8N1` anyway
> for compatibility with terminal tools.

The same console is **also** available on the USART2 debug pins; both run
simultaneously and mirror each other's output.

---

## Command protocol

Send a command line terminated by `\n`; the device executes it on its hardware
worker task, prints the output, then prints the prompt `"> "`.

### Behavior a parser must account for

- **Character echo:** typed characters are echoed back as you send them.
- **Prompt delimiter:** after each command the device prints `"> "`. Read until
  the next prompt to capture a command's full output.
- **Interleaved logs:** boot messages and asynchronous events (`[wifi] ...`,
  `[cloud] ...`, `[cmd] ...` traces of LAN and cloud commands) are printed on the
  same channel and may appear between or during command output. Parse by
  scanning for the documented reply lines rather than assuming the response is
  contiguous.
- **Busy worker:** if the worker's queue is full the console prints
  `busy — try again` and drops the line.
- **Quoting:** only `wifi-set` understands double quotes (an SSID or password
  with spaces). Every other command splits on whitespace.
- **Machine-readable replies:** the commands an installer drives
  (`identity*`, `sig-policy`, `lan-policy`, `ca*`, `proxy*`, `upload-*`,
  `blobs`) start their reply with the command name and then `ok`, `error <why>`
  or the value, with no leading spaces. The other commands print indented,
  human-readable lines.
- **Shared hardware:** the commands that capture into the PSRAM or drive the
  shared bus (`adc`, `measure`, `capture-psram`, `dualcap`, `lastress`,
  `psram*`, `cap-selftest`, `flash-ice40`) take the same lock as the JSON
  captures. While a capture, upload or update runs they print
  `<cmd> refused: busy: a capture, upload or update is using the PSRAM bus; try again when it ends`.

### Command reference

`help` prints a short version of this list on the device. On a usage error a
command prints `usage: ...` and changes nothing.

#### System

| Command | Description |
|---|---|
| `help` | List the commands. |
| `status` | One `key : value` line each: `device : benchpod`, `mac`, `ip`, `dma`, `psram` (boot self-test), `psbus`, `board` (board, firmware version, revision, `nrst_pin`, `usb_cc`), `fpga` (gateware version, reachable or not), `gw` (when the running gateware differs from the embedded one), `hint`, `safe` (safe-mode report), `clock`, `adc`, `dac`, `replay`, `cloud` (`registered state=... device_id=...` or `not registered`), `reset`, `crash`, `boot`, one `stack` line per task, `flash`, `lwip`, `heap`. Tools grep for `benchpod` to recognize a pod. |
| `ping` | SPI-ping the iCE40: `PING ok  version=0x..` or `PING failed`. |
| `uid` | The MCU's 96-bit unique ID. |
| `selftest` | STM32H563 silicon health check (HSE/PLL clocks, timer, SRAM, RNG). Does not touch the FPGA, PSRAM or flash. |
| `reboot` | System reset. |
| `dfu` | Reboot into the STM32 ROM USB DFU bootloader (see [Firmware update via `dfu`](#firmware-update-via-dfu)). |
| `test-bootloop [net\|hw] yes` | Crash the next two boots on purpose (in the network task, or in the iCE40/PSRAM bring-up) to prove safe mode; the third boot comes up in safe mode with that part off. Without `yes` it prints what it would do. |
| `test-hang <net\|hw> yes` | Hang a task on purpose; the watchdog resets the pod (about 30 s for `net`, 90 s for `hw`) and the next boot reports the stall as the last crash. |
| `test-crash yes` | Fault on purpose; the next boot reports it as the last crash. |

#### Device identity and policies

| Command | Description |
|---|---|
| `identity` | `identity <short id> <public key base64url>`, or `identity unknown <why>`. |
| `identity-wipe <short id\|unknown>` | Erase the device key and make a new one, the way back for a pod whose identity record the firmware will not replace. The argument must be the current short id (6 hex digits), or `unknown` when there is none; anything else prints `identity-wipe error confirm with the current id: identity-wipe <id>`. Replies `identity-wipe ok <new short id>`, then the pod logs in to the cloud again (the server needs a new registration). Only here: the JSON `identity_wipe` is refused on the LAN and the cloud. `benchpod identity wipe --connection usb` drives it. |
| `sig-policy [audit\|permissive\|required]` | Show or set which updates the pod accepts ([API.md, Pod policies](API.md#pod-policies)). The console may set any value, so it is the way back from `required`. Replies `sig-policy <policy> <keys>` or `sig-policy error <why>`. |
| `lan-policy [open\|locked\|off]` | Show or set what the LAN API may do. Replies `lan-policy <policy>` or `lan-policy error <why>`. `off` stops the TCP listener and mDNS until set back. |
| `cloud-ca`, `cloud-ca-clear` (also `ca`, `ca-clear`) | The company CA for the cloud link: one `ca <subject> <sha256>` line per certificate, or `ca none`, plus `ca error <why>` when the installed CA does not parse. Install one with `upload-begin ca <size> <sha256>` (no signature). `ca-clear` replies `ca-clear ok` or `ca-clear error <why>`. |
| `cloud-proxy`, `cloud-proxy-set <host:port> [user password]`, `cloud-proxy-clear` (also `proxy`, `proxy-set`, `proxy-clear`) | The HTTP proxy for the cloud link. Replies `proxy <host:port> auth\|noauth`, `proxy none` or `proxy error <why>`; never shows the password. |
| `ca-damage` | Development builds only: damage the CA slot to try the built-in-roots fallback. |

`ca-clear`, `proxy-set` and `proxy-clear` write the W25Q, so they are refused with
`busy: ...` while a capture, upload or update is using the shared bus.

#### Network

| Command | Description |
|---|---|
| `wifi-set "<ssid>" "<password>"` | Save Wi-Fi credentials to the pod's flash and (re)connect the ESP32-C3 in the background (the C3 is given them on each bring-up and keeps them in RAM only). Prints `[cfg] credentials written to flash`, then `saved SSID "<ssid>" — connecting in the background; run wifi-show for status`. Errors: `wifi-set: ssid too long`, `wifi-set: password too long`, `wifi-set: config save failed`. |
| `wifi-show` | `ssid`, `state` (as `status.wifi` in the JSON API), `ip`, `rssi` and the C3 restart counters. The password is never printed. |
| `wifi-clear` | Erase the stored credentials and drop Wi-Fi, then erase the ESP32-C3's NVS partition, where firmware up to 3.7.0 left a second copy (about 3 s). See [`wifi-clear` result](#wifi-clear-result). |
| `wifi-static <ip> <netmask> <gateway>` | Give the Wi-Fi interface a static address (diagnostic; the pod uses DHCP by default). |
| `eth <stop\|start\|restart>` | Bring the wired link down or up (PHY reset + DHCP). |
| `eth stats` | Wired link: negotiated mode, MAC mode, error and drop counters. |
| `eth speed <auto\|10\|100> [full]` | Force the link mode (half duplex unless `full`), or go back to autonegotiation. |
| `eth refclk` | Measure the PHY's 50 MHz RMII clock against the MCU crystal (drops the link for about 200 ms). |
| `eth loopback <10\|100> [n]` | PHY loopback test with `n` frames (default 200, at most 1000). |
| `esp-reset-pulse` | Diagnostic: reset the C3 unannounced; Wi-Fi must recover on its own. |
| `esp-mon [ms]` | Print the C3's UART output for `ms` milliseconds (default 5000, at most 20000). |
| `flash-esp32-sync` | Put the C3 in its ROM download mode and sync (a wiring test). |
| `flash-esp32` | Flash the ESP32-C3 Wi-Fi image from the W25Q `esp` slot (about 140 s; refused while the shared bus is busy). |

#### Firmware, gateware and blob uploads

| Command | Description |
|---|---|
| `blobs` | One `blob <name> <state> <size> <version> <sha256\|->` line per W25Q release slot (`gw0`, `gw1`, `esp`); `state` is `ok`, `outdated`, `missing` or `unknown` (see JSON `blob_status`). Then `fw-copy <size> bytes, sha256 <prefix>...` or `fw-copy none`: the firmware image the last install stored in the W25Q. |
| `upload-begin <firmware\|gw0\|gw1\|esp\|ca> <size> <sha256> [version]` | Start an upload over this console: the same PSRAM staging, SHA-256 and signature checks as an OTA. Replies `upload-begin ok` or `upload-begin error <why>` (for example `busy: <target> update from <holder> is in progress`, `busy: a capture or upload is running`, `signature: <result>`, or the safe-mode refusal). |
| `upload-data <offset> <len> <crc32 hex>` | Followed by exactly `len` raw bytes (at most 512, no echo). Replies `upload-data ok <offset>`, `upload-data retry crc`, `upload-data retry timeout`, `upload-data busy` or `upload-data error usage: ...`. |
| `upload-end` | Verify the staged image: `upload-end ok` or `upload-end error <why>`. |
| `upload-commit` | Firmware: `upload-commit resetting`, then the pod writes the image and resets. A blob: `upload-commit ok`. Errors: `upload-commit error no verified image staged` and the others. |
| `upload-status` | `upload-status <state> <target> <received>/<size> <error\|->`. |
| `upload-abort` | `upload-abort ok`, or `upload-abort error <why>` when another transport holds the session. |
| `upload-sig <0\|1> <base64url>`, `upload-sig clear`, `upload-sig` | The image's signed manifest (171 base64url characters, sent in two halves before `upload-begin`, which uses it once; see `docs/design/firmware-signing.md`). Without arguments: `upload-sig result <none\|ok\|format\|unknown-key\|signature\|target\|image> <key_id\|->` for the last begin. Whether a missing or bad signature is refused depends on `sig-policy`. Older firmware answers "unknown command", which is how installers tell. |
| `flash-ice40` | Reflash the iCE40 configuration flash with gateware image 0 from the W25Q `gw0` slot and reconfigure. |
| `flash-id` | Read the iCE40 configuration flash's JEDEC ID (W25Q64 = `ef 40 17`). |

`benchpod install-blobs` and `benchpod firmware install --connection usb` drive the
upload commands.

#### LA bank, target power and reset

| Command | Description |
|---|---|
| `la-voltage [1800\|3300]` | Set or show the LA I/O-bank voltage (required before any LA operation over the LAN or cloud). Refused, naming the pins, while an LA pin has a function. 1.8 V needs a v3 pod. |
| `power <1\|2> <on\|off>` | Switch a target eFuse (1 = internal 5 V, 2 = external): `eFuse<N> ON` / `OFF`, or `bad eFuse index`. |
| `pstat` | Both eFuses: `eFuse<N>: en=.. valid=.. fault=..`. |
| `nrst [assert\|release\|<ms>]` | Drive the target reset pin (J1 pin 22, v3), or pulse it for `ms` milliseconds; with no argument, show it. |
| `usb-cc` | USB-C CC lines: voltages, orientation and the source's current advertisement (v3). |

#### Analog (DAC8551 / MCP33131)

| Command | Description |
|---|---|
| `dac <off\|3v3\|5v\|12v> [volts]` | Route the DAC output path and, with `volts`, hold a calibrated DC voltage. |
| `adc [ext\|cal1\|cal2\|current_in]` | Route an ADC source and print a calibrated reading in mV (default `ext`; `current_in` also in µA). |
| `measure` | The ADC input SMA in mV (= `adc ext`). |
| `path <name>` | Apply a named analog path (routing only): `off`, `3v3`, `5v`, `12v`, `ext`, `cal1`, `cal2`, `current_in`. |
| `current-out [mA]` | Hold a current on the 4-20 mA output (J9, needs an external floating loop supply); with no value, show the range. |
| `calibrate [current_in\|clear]` | Show this pod's ADC calibration, calibrate the `current_in` input (J8 disconnected), or remove it. |
| `dac-limits [clear]` | Show the DAC output limits, or clear them. Setting them is JSON `dac_limits`. |
| `dacraw <0-255> [divider]` | Raw DAC code, no routing or calibration (debug); `divider` sets the DAC engine rate (48 MHz / divider, default 240). |
| `dacmux <en> <sel> [en2 sel2]` | Low-level U55 output mux (debug; prefer `dac`/`path`). |
| `calsw <cal1> <cal2> <current_in> <cal_path>` | Low-level U58 calibration relays (debug; prefer `path`/`adc`). |
| `adcraw` | One raw ADC probe byte (debug). |
| `adc-spi [n]` | `n` live ADC reads directly over SPI, skipping the PSRAM (debug; prints on the device log). |

With DAC limits set, `dac`, `path`, `adc`, `current-out`, `dacraw`, `dacmux` and
`calsw` are refused when they would break them, with the same text as the JSON API.

#### I²C peripherals

| Command | Description |
|---|---|
| `i2c-scan` | Scan the power/IO I²C bus (prints on the device log). |
| `ina` | Read the INA238 power monitors: `int(0x40)` internal 5 V, `ext(0x44)` external supply, and `pod(0x41)` when the pod's own monitor is fitted. |
| `expdump` | Dump the four TCA9554 expanders (LA pull-ups, eFuse control, DAC mux, analog switch). |

#### CAN

| Command | Description |
|---|---|
| `can config <bitrate> [normal\|internal\|external\|listen] [term]` | Bring CAN up (classic CAN on FDCAN1). `internal` and `external` are loopback modes for a single pod. |
| `can write <id> [b0 .. b7]` | Send one standard frame. |
| `can read` | Print up to 8 received frames. |
| `can status` | State, error counters, queue and responder counters. |
| `can respond <match_id> <reply_id> [bytes...]`, `can respond clear` | Add or clear an automatic reply rule. |
| `can term <0\|1>` | Switch the 120 Ω termination. |
| `can off` | Turn CAN off. |

#### PSRAM, capture and iCE40 diagnostics

| Command | Description |
|---|---|
| `psram-selftest` | The layered boot test (STM32 to PSRAM, iCE40 write, iCE40 /CE reach); prints on the device log. Part of the safe-mode recovery path. |
| `cap-selftest [n]` | Ramp through the ADC to PSRAM path (default 64): PASS = the path is fine, FAIL = an iCE40 capture timing error. |
| `psram` | Diagnose the PSRAM bus, CS and arbitration (an `0xFF` ID). |
| `psram-test` | Bring up and pattern-test the PSRAM. |
| `psram-addrtest` | Address-tagged write and read-back across the whole 8 MB from the STM32 alone. |
| `psram-bench [kb]` | PSRAM throughput (default 256 KB). |
| `psram-clk <prescaler>` | Set the OCTOSPI clock prescaler (debug). |
| `psram-chunk <bytes>` | Set the PSRAM chip-select chunk size (debug). |
| `capture-psram [n]` | Capture `n` ADC samples (at most 64) and print them. |
| `dualcap [adc_n] [la_n]` | One-trigger ADC (100 kS/s) + LA (1 MS/s) capture, at most 256 each; prints the first samples. |
| `lastress [n]` | Deep LA capture at the maximum rate as a drain stress test (default 65535). |
| `spi-clk <128\|256>` | Set the STM32 to iCE40 SPI prescaler (debug). |
| `spi-diag [n]` | SPI link diagnostic (prints on the device log). |

The console does **not** get the safe-mode refusals of the JSON API: in safe mode
it is how you test the iCE40 and PSRAM (`psram-selftest`, `psram-test`,
`flash-ice40`) before you power-cycle.

### `wifi-clear` result

`wifi-clear` wipes both places Wi-Fi credentials can be: the pod's own config store, and the
ESP32-C3's NVS partition (erased through the C3's ROM loader and verified blank by MD5). What it
guarantees is described under [`wifi_set` / `wifi_clear`](API.md#wifi_set--wifi_clear) in API.md.

| Outcome | Output |
|---|---|
| Both wiped | `Wi-Fi credentials cleared (pod flash and the ESP32-C3's NVS)` |
| Pod wiped, C3 not | `Wi-Fi credentials cleared on the pod, but the ESP32-C3's NVS was not erased (<why>): an old copy may remain there; run wifi-clear again or flash-esp32` |

`<why>` is `safe mode: ...`, `busy: ...` (a capture, upload or update holds the shared bus), or
`the C3 did not answer its ROM loader or the erase did not verify (see log)`; the `[espflash]`
log lines above the result say which step failed. `flash-esp32` rewrites the whole C3 image, NVS
region included (about 2 to 3 minutes).

### `wifi-set` result markers

After `wifi-set "<ssid>" "<password>"`, scan the output for:

| Outcome | Marker line (substring) |
|---|---|
| Credentials persisted | `[cfg] credentials written to flash` |
| Join progress and result | the asynchronous `[wifi] ...` log lines; or poll `wifi-show` until `state: connected` |

A typical transcript:

```
> wifi-set "MyNet" "s3cr3t"
[cfg] credentials written to flash
  saved SSID "MyNet" — connecting in the background; run wifi-show for status
> wifi-show
  ssid: MyNet
  state: connected
  ip: 192.168.1.42
  rssi: -57 dBm
  c3 restarts: 0 no response, 0 rebooted while up
>
```

The credentials are stored in flash and survive reboots and firmware updates
(the config sector is not part of the firmware image).

---

## Firmware update via `dfu`

Sending `dfu` reboots the device into the STM32H563's ROM USB **DFU** bootloader.
Flash a new image from the host with `dfu-util` (`benchpod flash-self`, or
`make flash-dfu`, which runs `dfu-util ... :leave` and reboots into the app). The
application does **not** flash itself here; it only hands off to the built-in ROM
bootloader. For an update without DFU, use `upload-begin` (above).

```
> dfu
  entering DFU (USB bootloader) — flash with dfu-util / `benchpod flash-self`...
```

The CDC serial port disappears at this point and the STM32 ROM DFU device
(`0483:df11`) enumerates instead.

---

## Talking to it from Go

Any pure-Go serial library works since the device is a normal serial port, with no
libusb/cgo dependency. For example, with `go.bug.st/serial`:

```go
port, err := serial.Open("/dev/cu.usbmodem1101", &serial.Mode{BaudRate: 115200})
// write: port.Write([]byte("status\n"))
// read lines until the "> " prompt, then look for "device : benchpod".
```

Discover the port by enumerating serial ports and matching USB VID `0x0483` and
PID `0x5740` (`enumerator.GetDetailedPortsList()` exposes the VID/PID), or by
matching the platform path patterns in the table above. The UART bridge and the
CMSIS-DAP probe are not on the console; use the LAN or the cloud for those
(`uart_proxy_start`, `dap_start` in [API.md](API.md)).
