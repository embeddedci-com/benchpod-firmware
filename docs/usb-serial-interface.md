# bench-pod-firmware USB Serial Console

The firmware exposes its interactive console over the STM32H563's **USB CDC-ACM**
port, mirrored on the USART2 debug pins. The same USB device is also a network
adapter (CDC-NCM) that carries the [JSON API](API.md): see
[USB network adapter](#usb-network-adapter). A host
tool (the `benchpod` CLI, a provisioning script) can open the serial port and
drive text commands such as `status`, `wifi-set`, `identity` and `dfu`.

This document is the contract for talking to that interface. The console is the
pod's **physical-presence** channel: it can change any setting, including the ones
the LAN and the cloud cannot (loosening the signature policy, wiping the device
identity), and it is the recovery path when the pod is in safe mode.

The console does **not** run the JSON commands of [API.md](API.md). It has its own
text vocabulary, listed below. From firmware 3.9.0 it keeps only what needs physical
presence or has to work without the network: identity and policies, recovery, Wi-Fi
and the wired link. Instrument control (DAC, ADC, target power, LA voltage, CAN)
is the JSON API, which a computer on the USB cable reaches through the [USB network
adapter](#usb-network-adapter); see [Moved to the JSON API](#moved-to-the-json-api).
Development builds add bench diagnostics, marked below; release firmware leaves them out.

---

## Transport

| Property | Value |
|---|---|
| Class | USB CDC-ACM (virtual serial port), interfaces 0-1 of a composite device |
| USB VID/PID | `0x0483:0x5740` (ST's stock CDC ID), before and from firmware 3.9.0; 3.9.0 reports `bcdDevice` `0x0390`. Pre-release 3.9.0 builds used `0x1209:0x0001`. The host's stock CDC-ACM driver binds |
| USB strings | manufacturer `EmbeddedCI`, product `BenchPod`, serial = 12 hex digits from the MCU's unique ID (unchanged from older firmware) |
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

## USB network adapter

From firmware 3.9.0 the pod's USB device also has a CDC-NCM network adapter
(interfaces 2-3), so a computer on the USB cable reaches the full JSON API, SCPI
and the byte tunnels on port `8080` with no Ethernet or Wi-Fi. macOS, Linux and
Windows 11 bind it with their in-box drivers (Windows through a Microsoft OS 2.0
descriptor, compatible ID `WINNCM`); those three are tested. Windows 10 (1903 or
newer) is supported but not tested: if the adapter shows up without a driver there,
update its driver in Device Manager and pick "UsbNcm Host Device".

| Property | Value |
|---|---|
| Pod address | `10.237.x.y`, the first host of a `/30` derived from the MCU's unique ID (stable per pod) |
| Computer's address | The other host of that `/30`, handed out by the pod's DHCP server (lease 1 hour) |
| Router, DNS | None: the computer keeps its own default route and DNS, and only the `/30` goes to the pod |
| mDNS | `_benchpod._tcp` and `_scpi-raw._tcp` are advertised on this link too (under every LAN policy) |
| Security | LAN tier: what the LAN may never do (`identity_wipe`, loosening `sig_policy`, changing `lan_policy`) is refused on it too. The one exception: it may install or clear the company CA and set or clear the cloud proxy (physical presence, like the console). The [LAN policy](API.md#lan_policy--what-the-external-lan-may-do) is for the external LAN only: the USB adapter stays on and open under `locked` and `off` (physical presence) |
| SCPI and VISA | The same `:8080` socket: `TCPIP::10.237.x.y::8080::SOCKET` |
| Several pods | Each pod has its own `/30`, so two pods on one computer get two adapters and two addresses. Not tested yet with two physical pods |
| WSL2, Docker on Windows | Not tested. Windows routes the pod's `10.237.x.y` address itself, so a WSL2 distribution or a Docker container that uses the Windows host's network should reach it; mDNS from inside them may not |
| Turning it off | `usb-network off` on the console (or `benchpod usb-network off` over USB): the pod re-enumerates as its console only and the computer sees no network adapter. `usb-network on` on the console brings it back. Stored, separate from the LAN policy; Ethernet, Wi-Fi and the cloud may only read it ([`usb_network`](API.md#usb_network--the-usb-network-adapter-on-or-off)) |
| Throughput | USB full speed (12 Mbit/s): about 0.9 MB/s from the pod (`speedtest`), against 1.3 MB/s on 100 Mbit Ethernet |

### Windows with ST's VCP driver

The pod keeps ST's stock USB ID `0483:5740`. On a Windows computer where ST's Virtual COM Port
driver (`STMicroelectronics Virtual COM Port`, from ST's STSW-STM32102 package) was installed, that
driver can claim the whole device: the pod then shows up as one COM port and the USB network
adapter does not load (`benchpod status` over `usb` uses the console, the network commands say the
USB network is not up). Windows 11 and 10 without that driver bind their in-box drivers: a "USB
Serial Device (COMn)" and the "EmbeddedCI BenchPod network" adapter.

To let Windows use its own drivers for the pod (this test has not been run on hardware yet):

1. Plug in the pod and open Device Manager.
2. Under **Ports (COM & LPT)**, right-click **STMicroelectronics Virtual COM Port (COMn)** and
   choose **Uninstall device**.
3. Tick **Attempt to remove the driver for this device** and click **Uninstall**.
4. Unplug the pod and plug it in again. Windows now lists a "USB Serial Device (COMn)" and, under
   **Network adapters**, "EmbeddedCI BenchPod network".

Removing the driver also affects other ST boards that use it; Windows 10 and 11 drive those with
their in-box serial driver too. Without the USB network, the pod still works over Ethernet, Wi-Fi
or the cloud.

The JSON `status` reports it as `usb` (`net`, `setting`, `off_reason`, `link`, `ip`, `host_ip`) and the
console `status` as the `usb` line. `benchpod`, the Python SDK and the MCP server
connect to it with the connection `usb`.

**USB networking or Wi-Fi.** The two are one or the other: while Wi-Fi credentials
are stored (`wifi-set`, JSON `wifi_set`) USB networking is off, and the pod reports
the adapter's link down to the computer; `wifi-clear` turns it back on. The
console works in both cases. Ethernet is independent of both.

**macOS and a locked screen.** macOS ignores a network interface that appears while
the screen is locked. If the pod was plugged in or rebooted then, run
`usb-reattach` on the console (or unplug and plug the cable) after unlocking.

The cloud link does not ride this adapter on its own (the pod gets no router from
the computer). `benchpod usb-bridge` runs an HTTP CONNECT proxy on the computer's
end of the link and sets it as the pod's cloud proxy (`proxy-set`), so the pod
reaches embeddedci.com through the computer; it clears the proxy again when it
stops. With a proxy set, the pod's cloud client starts on the USB link alone; it
no longer waits for an Ethernet or Wi-Fi address.

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
  (`identity*`, `sig-policy`, `lan-policy`, `ca*`, `proxy*`, `blobs`) start their reply with the command name and then `ok`, `error <why>`
  or the value, with no leading spaces. The other commands print indented,
  human-readable lines.
- **Shared hardware:** the commands that capture into the PSRAM or drive the
  shared bus (`psram-test`, `psram-selftest`, `flash-ice40`, and in development
  builds `capture-psram`, `dualcap`, `lastress`, `psram*`, `cap-selftest`) take the same lock as the JSON
  captures. While a capture, upload or update runs they print
  `<cmd> refused: busy: a capture, upload or update is using the PSRAM bus; try again when it ends`.

### Stable replies

These console replies are read by host tools (the `benchpod` CLI, the `embeddedci` SDK and MCP
server, the web app's setup wizard over WebSerial), including older releases still in use, so
their formats do not change: `status` (`device : benchpod`, the `ip`, `usb` and `board` lines),
`ping` (`PING ok`), `identity`, `identity-wipe ok`, `sig-policy`, `lan-policy`, `ca`/`ca none`,
`proxy`/`proxy none`, `wifi-show` (`ssid:`, `state:`, `ip:`), `blobs`, `usb-reattach ok` and `dfu`. A new field is added as a new line or at the end of a line; an
existing one keeps its place. `stm32h563/test/check_console_contract.py` (run with the host tests)
fails when one of these format strings changes.

### Command reference

`help` prints a short version of this list on the device. On a usage error a
command prints `usage: ...` and changes nothing.

#### System

| Command | Description |
|---|---|
| `help` | List the commands. |
| `status` | One `key : value` line each: `device : benchpod`, `mac`, `ip`, `usb` (`net on` or `net off (Wi-Fi configured)`, the pod's and the computer's address, `link up\|down`, frame counters), `dma`, `psram` (boot self-test), `psbus`, `board` (board, firmware version, revision, `nrst_pin`, `usb_cc`), `fpga` (gateware version, reachable or not), `gw` (when the running gateware differs from the embedded one), `hint`, `safe` (safe-mode report), `clock`, `adc`, `dac`, `replay` (on the digital board, firmware 3.9.2+: one `analog : none` line instead), `cloud` (`registered state=... device_id=...` or `not registered`), `reset`, `crash`, `boot`, one `stack` line per task, `flash`, `lwip`, `heap`. Tools grep for `benchpod` to recognize a pod. |
| `ping` | SPI-ping the iCE40: `PING ok  version=0x..` or `PING failed`. |
| `uid` | The MCU's 96-bit unique ID. |
| `selftest` | STM32H563 silicon health check (HSE/PLL clocks, timer, SRAM, RNG). Does not touch the FPGA, PSRAM or flash. |
| `reboot` | System reset. |
| `usb-reattach` | Replies `usb-reattach ok`, then drops off the USB bus for half a second and connects again, so the computer enumerates the console and the network adapter anew (the serial port closes). |
| `dfu` | Reboot into the STM32 ROM USB DFU bootloader (see [Firmware update via `dfu`](#firmware-update-via-dfu)). |
| `test-bootloop [net\|hw] yes` | Development builds only. Crash the next two boots on purpose (in the network task, or in the iCE40/PSRAM bring-up) to prove safe mode; the third boot comes up in safe mode with that part off. Without `yes` it prints what it would do. |
| `test-hang <net\|hw> yes` | Development builds only. Hang a task on purpose; the watchdog resets the pod (about 30 s for `net`, 90 s for `hw`) and the next boot reports the stall as the last crash. |
| `test-crash yes` | Development builds only. Fault on purpose; the next boot reports it as the last crash. |

#### Device identity and policies

| Command | Description |
|---|---|
| `identity` | `identity <short id> <public key base64url>`, or `identity unknown <why>`. |
| `identity-wipe <short id\|unknown>` | Erase the device key and make a new one, the way back for a pod whose identity record the firmware will not replace. The argument must be the current short id (6 hex digits), or `unknown` when there is none; anything else prints `identity-wipe error confirm with the current id: identity-wipe <id>`. Replies `identity-wipe ok <new short id>`, then the pod logs in to the cloud again (the server needs a new registration). Only here: the JSON `identity_wipe` is refused on the LAN and the cloud. `benchpod identity wipe --connection usb` drives it. |
| `sig-policy [audit\|permissive\|required]` | Show or set which updates the pod accepts ([API.md, Pod policies](API.md#pod-policies)). The console may set any value, so it is the way back from `required`. Replies `sig-policy <policy> <keys>` or `sig-policy error <why>`. |
| `usb-network [on\|off]` | Show or set the [USB network adapter](#usb-network-adapter) setting (default `on`). Replies `usb-network <on\|off>` or `usb-network error <why>`. `off` re-enumerates the pod as its console only (the computer sees no network adapter); `on` brings the adapter back. Stored, separate from `lan-policy`; only the console and the USB link itself may change it. |
| `lan-policy [open\|locked\|off]` | Show or set what the external LAN (Ethernet and Wi-Fi) may do. Replies `lan-policy <policy>` or `lan-policy error <why>`. `off` refuses connections over Ethernet and Wi-Fi and stops mDNS there until set back. The USB network adapter stays on and open under every policy. |
| `cloud-ca`, `cloud-ca-clear` (also `ca`, `ca-clear`) | The company CA for the cloud link: one `ca <subject> <sha256>` line per certificate, or `ca none`, plus `ca error <why>` when the installed CA does not parse. Installing one needs the network: JSON `ota_begin` with target `ca` over the USB network adapter, the cloud or (firmware before 3.9.0) the console's `upload-begin ca`; `benchpod cloud ca set` does it. `ca-clear` replies `ca-clear ok` or `ca-clear error <why>`. |
| `cloud-proxy`, `cloud-proxy-set <host:port> [user password]`, `cloud-proxy-clear` (also `proxy`, `proxy-set`, `proxy-clear`) | The HTTP proxy for the cloud link. Replies `proxy <host:port> auth\|noauth`, `proxy none` or `proxy error <why>`; never shows the password. |
| `ca-damage` | Development builds only: damage the CA slot to try the built-in-roots fallback. |

`ca-clear`, `proxy-set` and `proxy-clear` write the W25Q, so they are refused with
`busy: ...` while a capture, upload or update is using the shared bus.

#### Network

| Command | Description |
|---|---|
| `wifi-set "<ssid>" "<password>"` | Save Wi-Fi credentials to the pod's flash and (re)connect the ESP32-C3 in the background (the C3 is given them on each bring-up and keeps them in RAM only). Turns [USB networking](#usb-network-adapter) off. Prints `[cfg] credentials written to flash`, then `saved SSID "<ssid>" — connecting in the background; run wifi-show for status`. Errors: `wifi-set: ssid too long`, `wifi-set: password too long`, `wifi-set: config save failed`. |
| `wifi-show` | `ssid`, `state` (as `status.wifi` in the JSON API), `ip`, `rssi` and the C3 restart counters. The password is never printed. |
| `wifi-clear` | Erase the stored credentials and drop Wi-Fi (USB networking comes back on), then erase the ESP32-C3's NVS partition, where firmware up to 3.7.0 left a second copy (about 3 s). See [`wifi-clear` result](#wifi-clear-result). |
| `wifi-static <ip> <netmask> <gateway>` | Give the Wi-Fi interface a static address (diagnostic; the pod uses DHCP by default). |
| `eth <stop\|start\|restart>` | Bring the wired link down or up (PHY reset + DHCP). |
| `eth stats` | Wired link: negotiated mode, MAC mode, error and drop counters. |
| `eth speed <auto\|10\|100> [full]` | Development builds only (JSON `eth` has it in every build). Force the link mode (half duplex unless `full`), or go back to autonegotiation. |
| `eth refclk` | Development builds only (JSON `eth` has it in every build). Measure the PHY's 50 MHz RMII clock against the MCU crystal (drops the link for about 200 ms). |
| `eth loopback <10\|100> [n]` | Development builds only (JSON `eth` has it in every build). PHY loopback test with `n` frames (default 200, at most 1000). |
| `esp-reset-pulse` | Diagnostic: reset the C3 unannounced; Wi-Fi must recover on its own. |
| `esp-mon [ms]` | Print the C3's UART output for `ms` milliseconds (default 5000, at most 20000). |
| `flash-esp32-sync` | Put the C3 in its ROM download mode and sync (a wiring test). |
| `flash-esp32` | Flash the ESP32-C3 Wi-Fi image from the W25Q `esp` slot (about 140 s; refused while the shared bus is busy). |

#### Firmware, gateware and blobs

| Command | Description |
|---|---|
| `blobs` | One `blob <name> <state> <size> <version> <sha256\|->` line per W25Q release slot (`gw0`, `gw1`, `esp`); `state` is `ok`, `outdated`, `missing` or `unknown` (see JSON `blob_status`). Then `fw-copy <size> bytes, sha256 <prefix>...` or `fw-copy none`: the firmware image the last install stored in the W25Q. |
| `flash-ice40` | Reflash the iCE40 configuration flash with gateware image 0 from the W25Q `gw0` slot and reconfigure. |
| `flash-id` | Development builds only. Read the iCE40 configuration flash's JEDEC ID (W25Q64 = `ef 40 17`). |
| `ice40-erase [gw0] yes` | Development builds only. Erase the start of the iCE40 configuration flash (and, with `gw0`, that blob slot) and hold the iCE40 in reset, so the next boot sees a new board: firmware 3.9.2+ loads image 0 from `gw0` at boot, or as soon as `gw0` is installed. |

Firmware 3.9.0 removed the console uploads (`upload-begin`, `-data`, `-end`, `-commit`,
`-status`, `-abort`, `upload-sig`): firmware, blob and CA uploads go over the JSON API (`ota_*`),
which a computer on the USB cable reaches through the USB network adapter (`benchpod
install-blobs`, `benchpod cloud ca set`, the web app). The way back without any network is
`dfu` and `benchpod flash-self`.

#### I²C peripherals (development builds only)

The pod's own current monitor (`pod(0x41)` in `ina`) is JSON `power_status` `pod` in every build.

| Command | Description |
|---|---|
| `i2c-scan` | Scan the power/IO I²C bus (prints on the device log). |
| `ina` | Read the INA238 power monitors: `int(0x40)` internal 5 V, `ext(0x44)` external supply, and `pod(0x41)` when the pod's own monitor is fitted. |
| `expdump` | Dump the four TCA9554 expanders (LA pull-ups, eFuse control, DAC mux, analog switch). |

#### PSRAM, capture and iCE40 diagnostics

`psram-selftest` and `psram-test` are in every build; the others are development builds only.

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
| `adcraw` | One raw ADC probe byte. |
| `adc-spi [n]` | `n` live ADC reads directly over SPI, skipping the PSRAM (prints on the device log). |

The console does **not** get the safe-mode refusals of the JSON API: in safe mode
it is how you test the iCE40 and PSRAM (`psram-selftest`, `psram-test`,
`flash-ice40`) before you power-cycle.

### Moved to the JSON API

Firmware 3.9.0 removed these console commands; their JSON equivalents work over the
USB network adapter, the LAN and the cloud (`benchpod` 0.6.0+, the `embeddedci` SDK and
MCP server 2.12.0+ use them over `--connection usb`). The console still answers a removed
command, with one line instead of `unknown command`, so an older CLI or SDK tells its user
what to update:

```
la-voltage: moved to the network API (JSON la_voltage) in firmware 3.9.0; update the benchpod CLI to 0.6.0 or later (or embeddedci to 2.12.0), which uses it over USB
```

The upload commands keep the reply shape older CLIs wait for, so they print the line instead of
timing out: `upload-begin error moved to the network API (JSON ota_begin) ...` (the same for
`upload-data`, `-end`, `-commit`, `-status`, `-abort`), and `unknown command 'upload-sig': moved
to the network API (JSON ota_begin sig) ...`.

The commands that stay keep their names and reply formats, so older CLIs and SDKs keep
working with them: `status`, `dfu`, `identity`, `identity-wipe`, `sig-policy`,
`lan-policy`, the Wi-Fi, CA and proxy commands, `blobs`.

| Removed console command | JSON command |
|---|---|
| `la-voltage` | `la_voltage` |
| `power`, `pstat` | `target_power`, `target_status` |
| `nrst` | `nrst` |
| `usb-cc` | `usb_cc` |
| `dac`, `path` | `dac_out`, `analog_path` |
| `adc`, `measure` | `adc_read` (`measure` was `adc_read` with source `ext`) |
| `current-out` | `current_out` |
| `calibrate` | `calibrate` |
| `dac-limits` | `dac_limits` |
| `dacraw`, `dacmux`, `calsw` | `dac_set`, `dac_mux`, `cal_switch` |
| `upload-begin`, `-data`, `-end`, `-commit`, `-status`, `-abort`, `upload-sig` | `ota_begin` (with `sig`), `ota_data`, `ota_end`, `ota_commit`, `ota_status`, `ota_abort` |
| `can ...` | `can_config`, `can_write`, `can_read`, `can_status`, `can_respond`, `can_term`, `can_disable` |

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
bootloader. For an update without DFU, use the JSON `ota_*` commands over the USB network
adapter (`benchpod flash-self` uses DFU; the web app's one-click install uses the cloud).

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
PID `0x5740` (every STM32 firmware; pre-release 3.9.0 builds used `0x1209:0x0001`); `enumerator.GetDetailedPortsList()` exposes the VID/PID. Or match
the platform path patterns in the table above. The UART bridge and the
CMSIS-DAP probe are not on the console; use the USB network adapter, the LAN or the cloud for those
(`uart_proxy_start`, `dap_start` in [API.md](API.md)).
