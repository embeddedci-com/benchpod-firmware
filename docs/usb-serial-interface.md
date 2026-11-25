# bench-pod-firmware USB Serial Console

The firmware exposes its interactive console over the STM32H563's **USB CDC-ACM**
port, mirrored on the USART2 debug pins. The same USB device is also a network
adapter (CDC-NCM) that carries the [JSON API](API.md): see
[USB network adapter](#usb-network-adapter). A host
tool (the `benchpod` CLI, a provisioning script) can open the serial port and
drive text commands such as `status`, `identity`, `wifi-set`, `net-reset` and `dfu`.

This document is the contract for talking to that interface. The console is the
pod's **physical-presence** channel and, from firmware 4.0.0, a **recovery tool**: it
keeps what needs someone at the pod or has to work when no network reaches it (the
USB network adapter switch, loosening the LAN policy, the LAN TLS and SCPI switches,
resetting the network settings, DFU, the self-test), sets up Wi-Fi (`wifi-set`, `wifi-show`:
the web app's setup wizard talks to the pod over WebSerial, and a browser cannot reach the
JSON API on the USB network adapter), and it is the recovery path when
the pod is in safe mode.

The console does **not** run the JSON commands of [API.md](API.md). Everything else,
the Wi-Fi scan and clear, the device identity wipe, the cloud CA and proxy, blobs and the
Wi-Fi bridge included, is the JSON API, which a computer on the USB cable reaches through
the [USB network adapter](#usb-network-adapter); see [Moved to the JSON
API](#moved-to-the-json-api). Development builds add bench diagnostics, marked below;
release firmware leaves them out.

---

## Transport

| Property | Value |
|---|---|
| Class | USB CDC-ACM (virtual serial port), interfaces 0-1 of a composite device |
| USB VID/PID | `0x1209:0xEC10`, the pod's own ID from [pid.codes](https://pid.codes/1209/EC10/), from the next firmware release after 4.0.4 (`bcdDevice` `0x0405`). Firmware up to and including 4.0.x uses `0x0483:0x5740` (ST's stock CDC ID, `bcdDevice` `0x0390` from 3.9.0); pre-release 3.9.0 builds used `0x1209:0x0001`. The CLI, Python SDK and web app find the pod under both IDs from their next releases. The host's stock CDC-ACM driver binds |
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
(interfaces 2-3), so a computer on the USB cable reaches the full JSON API, SCPI (while it is on)
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
| mDNS | `_benchpod._tcp` (and `_scpi-raw._tcp` while SCPI is on) are advertised on this link too (under every LAN policy) |
| TLS | Port `8443` (TLS) answers here too. Plaintext `8080` answers on this link under [HTTPS-only](API.md#lan_tls--the-lan-api-over-tls), the default from firmware 4.0.0: the link is a point-to-point cable that never crosses a network. `benchpod` and the SDK use `8080` here and TLS on the LAN |
| Security | LAN tier: what the LAN may never do (changing `lan_policy`) is refused on it too. The exceptions need someone at the pod, like the console: it may install or clear the company CA, set or clear the cloud proxy, wipe the device identity (`identity_wipe`), clear the rollback floor and install a Wi-Fi bridge certificate. The [LAN policy](API.md#lan_policy--what-the-external-lan-may-do) is for the external LAN only: the USB adapter stays on and open under `locked` and `off` (physical presence) |
| SCPI and VISA | The same `:8080` socket: `TCPIP::10.237.x.y::8080::SOCKET`, while SCPI is on (off by default from firmware 4.0.0: `lan-tls scpi on`) |
| Several pods | Each pod has its own `/30`, so two pods on one computer get two adapters and two addresses. Not tested yet with two physical pods |
| WSL2, Docker on Windows | Not tested. Windows routes the pod's `10.237.x.y` address itself, so a WSL2 distribution or a Docker container that uses the Windows host's network should reach it; mDNS from inside them may not |
| Turning it off | `usb-network off` on the console (or `benchpod usb-network off` over USB): the pod re-enumerates as its console only and the computer sees no network adapter. `usb-network on` on the console brings it back. Stored, separate from the LAN policy; Ethernet, Wi-Fi and the cloud may only read it ([`usb_network`](API.md#usb_network--the-usb-network-adapter-on-or-off)) |
| Throughput | USB full speed (12 Mbit/s): about 0.9 MB/s from the pod (`speedtest`), against 1.3 MB/s on 100 Mbit Ethernet |

### Windows with ST's VCP driver

This section applies to firmware up to and including 4.0.x, which keeps ST's stock USB ID
`0483:5740`. From the next firmware release after 4.0.4 the pod enumerates as `1209:EC10`, which
ST's driver does not claim, so Windows binds its in-box drivers without the steps below. On a
Windows computer where ST's Virtual COM Port
driver (`STMicroelectronics Virtual COM Port`, from ST's STSW-STM32102 package) was installed, that
driver can claim the whole device of the older firmware: the pod then shows up as one COM port and the USB network
adapter does not load (`benchpod status` over `usb` uses the console, the network commands say the
USB network is not up). Windows 11 and 10 without that driver bind their in-box drivers: a "USB
Serial Device (COMn)" and the "EmbeddedCI BenchPod network" adapter.

To let Windows use its own drivers for a pod on firmware up to 4.0.x (this test has not been run on hardware yet):

1. Plug in the pod and open Device Manager.
2. Under **Ports (COM & LPT)**, right-click **STMicroelectronics Virtual COM Port (COMn)** and
   choose **Uninstall device**.
3. Tick **Attempt to remove the driver for this device** and click **Uninstall**.
4. Unplug the pod and plug it in again. Windows now lists a "USB Serial Device (COMn)" and, under
   **Network adapters**, "EmbeddedCI BenchPod network".

Removing the driver also affects other ST boards that use it; Windows 10 and 11 drive those with
their in-box serial driver too. Without the USB network, the pod still works over Ethernet, Wi-Fi
or the cloud.

The JSON `status` reports it as `usb` (`net`, `enabled`, `off_reason`, `link`, `ip`, `host_ip`) and the
console `status` as the `usb` line. `benchpod`, the Python SDK and the MCP server
connect to it with the connection `usb`.

**USB networking or Wi-Fi.** The two are one or the other: while Wi-Fi credentials
are stored (JSON `wifi_set`, or `wifi-set` on the console) USB networking is off, and the pod reports the adapter's
link down to the computer; JSON `wifi_clear`, or `net-reset` on the console, turns it
back on. The console works in both cases. Ethernet is independent of both.

**macOS and a locked screen.** macOS ignores a network interface that appears while
the screen is locked. If the pod was plugged in or rebooted then, unplug and plug the
cable after unlocking, or send JSON `usb_reattach` over Ethernet or Wi-Fi (the
console's `usb-reattach` up to firmware 3.10.13).

The cloud link does not ride this adapter on its own (the pod gets no router from
the computer). `benchpod usb-bridge` runs an HTTP CONNECT proxy on the computer's
end of the link and sets it as the pod's cloud proxy (JSON `cloud_proxy`), so the pod
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
  `[cloud] ...`) are printed on the same channel and may appear during command
  output. Parse by scanning for the documented reply lines rather than assuming
  the response is contiguous. How much the pod prints is its [log level](#log-levels).
- **Output after the prompt** (firmware 3.10.13+): a log line or a late
  reply that arrives while the console
  waits at the prompt starts on a line of its own and is followed by the prompt
  again, like a shell. A bare prompt is overwritten (`"\r"` then the line); a
  prompt with typed text is left as it is (`"\r\n"`) and the prompt and the typed
  text are printed again after the line. So the console always ends with `"> "`
  (plus what was typed) when it is idle. Older firmware printed such lines after
  the prompt without one.
- **First prompt** (firmware 3.10.13+): the pod prints
  `console ready — type 'help'` and the first prompt once the iCE40/PSRAM
  bring-up has finished (a few seconds after power-on), so the boot lines come
  before it. A line typed earlier waits and runs then.
- **Busy worker:** if the worker's queue is full the console prints
  `busy — try again` and drops the line.
- **Quoting:** every command splits on whitespace, except `wifi-set`: its SSID and
  password go in double quotes and may hold spaces (not a double quote or a line break).
- **Machine-readable replies:** the commands an installer drives
  (`identity`, `usb-network`, `lan-policy`, `lan-pairing`, `lan-clients`, `lan-tls`, `log`, `net-reset`) start their reply with the command name and then `ok`, `error <why>`
  or the value, with no leading spaces. The other commands print indented,
  human-readable lines.
- **Shared hardware:** the development-build commands that capture into the PSRAM or drive the
  shared bus (`psram-test`, `psram-selftest`, `flash-ice40`, `capture-psram`, `dualcap`,
  `lastress`, `psram*`, `cap-selftest`) take the same lock as the JSON captures. While a capture, upload or update runs they print
  `<cmd> refused: busy: a capture, upload or update is using the PSRAM bus; try again when it ends`.

### Stable replies

These console replies are read by host tools (the `benchpod` CLI, the `embeddedci` SDK and MCP
server, the web app's setup wizard over WebSerial), including older releases still in use, so
their formats do not change: `status` (`device : benchpod`, the `ip`, `usb` and `board` lines),
`ping` (`PING ok`), `identity`, `lan-policy`, `log`, `net-reset`, `dfu`, `wifi-set` (`[cfg]
credentials written to flash`, `wifi-set: <why>`) and `wifi-show` (`ssid:`, `state:`, `ip:`,
`via:`, and `rssi:` as the last line). Firmware 4.0.0 removed the other commands these tools
read (`identity-wipe`, `sig-policy`, `ca`, `proxy`, `blobs`, `usb-reattach`): CLI 0.10 does
the same over JSON. A new field is added as a new line or at the end of a line; an
existing one keeps its place. `stm32h563/test/check_console_contract.py` (run with the host tests)
fails when one of these format strings changes.

### Command reference

`help` prints a short version of this list on the device. On a usage error a
command prints `usage: ...` and changes nothing.

#### Recovery (every build)

| Command | Description |
|---|---|
| `help` | List the commands. |
| `status` | One `key : value` line each: `device : benchpod`, `mac` (the Ethernet MAC), `wifimac` (pods with Wi-Fi, firmware 3.10.8+: the MAC the pod uses on Wi-Fi, or `not known yet` until a `wifi-scan` or a join; see `wifi_mac` in the JSON [`status`](API.md#status)), `ip`, `usb` (`net on` or `net off (Wi-Fi configured)`, the pod's and the computer's address, `link up\|down`, frame counters), `dma`, `psram` (boot self-test), `psram rd` (capture read-backs from the PSRAM that needed a retry or failed, and the last error code), `ice40boot` (what the boot did about the iCE40: after any reset but a power-on it reloads the iCE40 from its flash first, so it starts from the same power-on state as the firmware (`software reset: reload ok, self-test passed`); then the self-test passed, reloaded it again, or rewrote a damaged image), `psbus`, `board` (board, firmware version, revision, `nrst_pin`, `usb_cc`), `fpga` (gateware version, reachable or not), `gw` (when the running gateware differs from the embedded one), `hint`, `safe` (safe-mode report), `clock`, `adc`, `dac`, `replay` (on the digital board, firmware 3.9.2+: one `analog : none` line instead), `cloud` (`registered state=... device_id=...` or `not registered`), `reset`, `crash`, `boot`, `c3` (boards with the built-in Wi-Fi: ESP32-C3 restarts since power-on and the last reason), one `stack` line per task, `flash` (the size; it adds `RECORD WRITES REFUSED` if the firmware ever tried to write a settings record outside the record area, a bug to report), `lwip`, `heap`. Tools grep for `benchpod` to recognize a pod. |
| `ping` | SPI-ping the iCE40: `PING ok  version=0x..` or `PING failed`. |
| `selftest` | STM32H563 silicon health check (HSE/PLL clocks, timer, SRAM, RNG). Does not touch the FPGA, PSRAM or flash. |
| `reboot` | System reset. |
| `dfu` | Reboot into the MCU's ROM USB DFU bootloader (see [Firmware update via `dfu`](#firmware-update-via-dfu)). Always allowed from firmware 4.0.0: it is the way to install firmware you built yourself (3.10.7 to 3.10.13 refused it under `sig-policy required`). |
| `identity` | `identity <short id> <public key base64url>`, or `identity unknown <why>`. |
| `usb-network [on\|off]` | Show or set the [USB network adapter](#usb-network-adapter) setting (default `on`). Replies `usb-network <on\|off>` or `usb-network error <why>`. `off` re-enumerates the pod as its console only (the computer sees no network adapter); `on` brings the adapter back. Stored, separate from `lan-policy`; only the console and the USB link itself may change it. |
| `lan-policy [open\|locked\|off]` | Show or set what the external LAN (Ethernet and Wi-Fi) may do. Replies `lan-policy <policy>` or `lan-policy error <why>`. `off` refuses connections over Ethernet and Wi-Fi and stops mDNS there until set back. The USB network adapter stays on and open under every policy. |
| `lan-pairing [on\|off]` | Firmware 4.0.1+. Show or set whether computers on Ethernet and Wi-Fi must be [paired](API.md#lan_clients--paired-computers) (default `on`). Replies `lan-pairing <on\|off>` or `lan-pairing error <why>`. `off` gives every LAN computer what the LAN policy allows, as before 4.0.1. |
| `lan-clients [add <name> <key> \| remove <name\|key> \| clear]` | Firmware 4.0.1+. The [paired computers](API.md#lan_clients--paired-computers): `lan-clients <n> of <max>, pairing <on\|off>`, then one line `  <name> <spki sha256>` each; or `lan-clients error <why>`. `add` pairs a computer (a name without spaces here; its key: 64 hex digits, the SHA-256 of its TLS client key's SubjectPublicKeyInfo) and first prints `lan-clients added`, `renamed` or `same`; `benchpod setup` uses it, so pairing works also while the USB network adapter is off. `remove` takes a name or 8+ hex digits of a key. |
| `lan-tls [https-only on\|off \| scpi on\|off \| regenerate]` | The [LAN API over TLS](API.md#lan_tls--the-lan-api-over-tls) on port 8443 (firmware 3.9.3+). Replies `lan-tls port 8443 https-only <on\|off> sha256 <hex> <generated\|user> "<key>" "<subject>" scpi <on\|off>`, `lan-tls port 8443 https-only <on\|off> none <why> scpi <on\|off>` without a certificate, or `lan-tls error <why>` (`scpi` at the end from firmware 4.0.0). `https-only on` (the default from 4.0.0) refuses plaintext 8080 on Ethernet and Wi-Fi (USB keeps it), `off` lets plaintext JSON in there again; `scpi on` (off by default from 4.0.0) makes SCPI answer on the TCP connections; `regenerate` makes a new key and self-signed certificate. The console may always change them. |
| `net-reset` | The way back to a pod that no network reaches (firmware 4.0.0). Forgets the Wi-Fi credentials (the pod's flash and the ESP32-C3's NVS, or the Wi-Fi bridge's), which turns [USB networking](#usb-network-adapter) back on, and puts Ethernet back to autonegotiation with a PHY reset and a new DHCP lease. The policies, the cloud settings and the device key stay. Replies `net-reset eth autonegotiation, link and DHCP again`, then a `net-reset wifi ...` line (see [`net-reset` result](#net-reset-result)). Up to firmware 3.10.13 `wifi-clear` did the Wi-Fi part. |
| `log [error\|warn\|info\|debug]` | Show or set the console [log level](#log-levels) (firmware 3.10.13+; default `info`). Replies `log <level>` or `log error <why>`. Stored: it survives a reboot and a firmware update. The JSON [`log_level`](API.md#log_level--the-console-log-level) command reads and sets the same setting. |
| `wifi-set "<ssid>" "<password>"` | Store the Wi-Fi credentials and join in the background (the web app's setup wizard and `benchpod set-wifi` send it; JSON [`wifi_set`](API.md#wifi_set--wifi_clear) does the same). Both values in double quotes; the password may be empty (`""`) for an open network. Replies `[cfg] credentials written to flash`, then `saved SSID "<ssid>": connecting in the background; run wifi-show for status` (`... on the Wi-Fi bridge: it joins in the background; ...` when a [Wi-Fi bridge](#wi-fi-bridge) is in use), or `wifi-set: <why>` (`ssid too long`, `password too long`, no Wi-Fi module, `config save failed`, a refusal from the bridge). Turns [USB networking](#usb-network-adapter) off, like any stored Wi-Fi. |
| `wifi-show` | The Wi-Fi state, one `label: value` line each: `ssid`, `state` (`flashing-c3` while the pod installs the Wi-Fi module's firmware on a first setup, with a `c3 flash: block <n>/<total>` line), `ip`, and `rssi` (`<n> dBm`, empty when not joined) last. Through a Wi-Fi bridge: the bridge's `ssid` and `state` (`unconfigured`, `wait-pod`, `joining`, `connected`, `auth-failed`, `no-ap`), the pod's `ip`, `via: wifi-bridge <mac> fw <version>, paired\|not paired`, `problem: <why>` when the bridge does not take the pod's requests, then `rssi`. JSON [`wifi_status`](API.md#wifi_status) has the details. |

#### Development builds only

| Command | Description |
|---|---|
| `uid` | The MCU's 96-bit unique ID. |
| `test-bootloop [net\|hw] yes` | Development builds only. Crash the next two boots on purpose (in the network task, or in the iCE40/PSRAM bring-up) to prove safe mode; the third boot comes up in safe mode with that part off. Without `yes` it prints what it would do. |
| `test-hang <net\|hw> yes` | Development builds only. Hang a task on purpose; the watchdog resets the pod (about 30 s for `net`, 90 s for `hw`) and the next boot reports the stall as the last crash. |
| `test-ice40-hold yes` | Development builds only. Hold the iCE40 in reset, as an ESD hit that unconfigured it would leave it. When the pod is idle it pings the iCE40 every 30 s and reloads one that stopped answering from its config flash (no write) after 10 s, at most 6 times an hour, then parks the outputs; the device log says so. |
| `test-crash yes` | Development builds only. Fault on purpose; the next boot reports it as the last crash. |
| `ca-damage` | Damage the CA slot to try the built-in-roots fallback. |
| `wifi-static <ip> <netmask> <gateway>` | Give the Wi-Fi interface a static address until the next reboot, DHCP off (a diagnostic; not stored). |
| `esp-reset-pulse` | Reset the C3 unannounced; Wi-Fi must recover on its own. |
| `esp-mon [ms]` | Print the C3's UART output for `ms` milliseconds (default 5000, at most 20000). |
| `flash-esp32-sync` | Put the C3 in its ROM download mode and sync (a wiring test). |
| `flash-ice40` | Reflash the iCE40 configuration flash with gateware image 0 from the W25Q `gw0` slot and reconfigure. |
| `flash-id` | Read the iCE40 configuration flash's JEDEC ID. |
| `ice40-erase [gw0] yes` | Erase the start of the iCE40 configuration flash (and, with `gw0`, that blob slot) and hold the iCE40 in reset, so the next boot sees a new board: firmware 3.9.2+ loads image 0 from `gw0` at boot, or as soon as `gw0` is installed. |
| `emi [status\|start\|stop]`, `emi dac <all\|sine\|dc\|current\|off>`, `emi uart <1-14\|off>`, `emi net <mbit\|off>`, `emi log [n\|all]`, `emi fault <ice40\|ice40-stuck\|off>` | EMI test builds only (`make EMI_TEST=1`, never released). The build starts an activity loop 10 s after boot for emissions measurements: LA voltage 3.3 V, target power on at both eFuses, the DAC (`sine`: 1 kHz on 0-3.3 V; `dc`: 3 V on 0-5 V; `current`: 12 mA on the 4-20 mA output; `all`: those three together, on a pod with the quad DAC only; `off`; the default is `all` with the quad DAC, else `sine`; a digital-only board has no DAC or ADC and runs the LA captures only), and back-to-back deep ADC + LA and LA captures read back and discarded. `emi uart <la>` adds 115200 baud NMEA out of that LA pin (it drives a DUT pin). The Ethernet port streams raw frames while the loop runs (default 20 Mbit/s, up to 50; `emi net <mbit>` sets the rate, `off` stops it): 1514-byte frames, EtherType 0x88B6, to 01-80-C2-00-00-0F (an IEEE 802.1 link-local address: a switch never forwards it, so the stream loads the pod's cable and stops at the first switch instead of flooding the network behind it; a laptop cabled straight to the pod sees it, one behind a switch only sees the BPMON frames), each starting with `BPEMI`, a sequence number and the uptime (little-endian 32-bit), so `tcpdump ether proto 0x88b6` can count losses. It needs only an Ethernet link, no IP address or DHCP, and goes through a Wi-Fi bridge too. The sent and dropped counts and the link state are in the `emi` line and in the `emi_test` object of `emi_log` (`net_mbit`, `net_sent`, `net_drops`, `net_link`, `net_link_losses`). `emi` prints the state (also every 5 s, and as `emi_test` in the JSON `emi_log` reply, in short in `status`); `emi stop` turns the DAC and target power off and frees the pod for clients; a reset or power cycle starts the loop again. The status LED shows it, one color at a time, for a test lab to read at a glance: solid green until the loop starts; green blinking (1 s on, 1 s off) while it runs with no failure since power-on; blue, fast blink, while a function fails (a command or capture failed in the last 5 s, Ethernet has no link (after 10 s from the start) or could not send a frame, an eFuse tripped, the ESP32-C3 restarted) and the pod is getting itself running again; red, fast blink, when that failure lasted 60 s without a break or the recovery ran out of steps (a fail under the lab's criterion B; not latched: it turns blue if the pod comes back); blue blinking (1 s) once it runs again after any failure or after a restart that was not a power-on (watchdog, reset pin, software), until power is removed (`emi stop`/`emi start` do not clear it); the normal colors once stopped. The `emi` line and `emi_test` carry it as `led`, with `failed_since_power_on` and `reset_cause`. Every 5 s, from boot on and whether the loop runs or not, the monitor line also goes out of the Ethernet port as a broadcast frame on EtherType 0x88B6 starting with `BPMON` (plain text: `BPMON run 12, reset power-on, uptime 61234 ms, seq 98765, emi: running, led green, ...`, where `seq` is the stream's next sequence number), so a laptop on the cable reads it in Wireshark (`eth.type == 0x88b6`) or with `stm32h563/tools/emi/emi_watch.py`, which also flags lost frames, a stopped stream, reboots and the LED leaving green. Every run is kept in a run log in the W25Q (7,040 slots, about 6,000 records of history: 1,024 are kept erased ahead, at boot): a start record (firmware, reset cause), an event on every LED or link change and a checkpoint with the counters when a value that shows trouble changed, at most every 10 s, else once a minute; `emi log` prints its state and the last 20 records (`emi log <n>`: the last n, `emi log all`: every one) on the device log, the JSON command `emi_log` (docs/API.md) reads it out, and `stm32h563/tools/emi/emi_log_dump.py` downloads and summarizes it. While the loop runs, the pod gets itself running again when it stays stuck: captures failing (or one hung) for 10 s reload the iCE40 from its config flash (no write), again 10 s later, then the STM32 resets at 30 s and the loop starts 2 s after that boot, so it runs again well inside a minute; an Ethernet link down, or up while frames keep failing to go out, for 20 s gets a PHY reset; an eFuse that trips (FLT held 30 ms) counts as a failure and gets its supply turned off and on again 2 s later (6 times an hour at most; then that output stays off and counts as failing); with limits, each step logged as a `recovery` event (docs/API.md, `emi_log`). On a board with the ESP32-C3, a C3 restart while the loop runs (it stopped answering, rebooted or failed a setup step; not an access point refusing the association) is a failure (blue recovering for 5 s, then blue), an `ESP32-C3 restart` event in the run log, and `c3 restarts N` in the `emi` line (`c3_restarts` in `emi_test`). The EMI build never goes to safe mode after crashed boots: it keeps restarting instead. `emi fault ice40` holds the iCE40 in reset once to test it, `emi fault ice40-stuck` after every reload and across resets (to see the reset and the limits), `emi fault off` ends it (a power-off too). Port 80 serves a page for a test lab, `http://<the pod's address>/`, that shows the loop live and keeps a copy of the run log in the browser (docs/API.md, `emi_log`). Log writes and reads do not interrupt the outputs: the W25Q shares the PSRAM bus, but the sine plays from the gateware's BRAM table and the DC and 4-20 mA outputs are held by the quad DAC, so they keep running; a session only delays the next capture (measured 0.3 to 0.4 ms for one record, about 3 ms for a batch of 14, about 60 ms with a sector erase, once the 1,024 records erased at boot are used up). The `emi log` state shows the last and longest W25Q session and DAC gap (0 normally; only a deep DAC replay, which the loop never plays, would be stopped). The JSON `status` carries a short `emi_test` (`running`, `led`, `failed_since_power_on`, `captures`, `errors`, `net_link`, `run`); `emi_log` without `count` carries the whole object. |

#### I²C peripherals (development builds only)

The pod's own current monitor (`pod(0x41)` in `ina`) is JSON `power_status` `pod` in every build.

| Command | Description |
|---|---|
| `i2c-scan` | Scan the power/IO I²C bus (prints on the device log). |
| `ina` | Read the INA238 power monitors: `int(0x40)` internal 5 V, `ext(0x44)` external supply, and `pod(0x41)` when the pod's own monitor is fitted. |
| `expdump` | Dump the four TCA9554 expanders (LA pull-ups, eFuse control, DAC mux, analog switch). |

#### PSRAM, capture and iCE40 diagnostics (development builds only)

| Command | Description |
|---|---|
| `led <colour\|mask>` | Light the status LED in one colour for 5 s (`red`, `green`, `blue`, `cyan`, `magenta`, `yellow`, `white`, `off`, or a SET_LED mask `0`-`7`), then it shows the pod's state again. Checks the LED. |
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
`selftest`, `status` and `dfu` work, and a development build tests the iCE40 and PSRAM
(`psram-selftest`, `psram-test`, `flash-ice40`) before you power-cycle.

#### Wi-Fi bridge

BenchPod v4 has no radio: a small bridge box on the Ethernet port carries the pod's traffic to
Wi-Fi ([wifi-bridge/README.md](../wifi-bridge/README.md), protocol in
[wifi-bridge/docs/PROTOCOL.md](../wifi-bridge/docs/PROTOCOL.md)). From firmware 4.0.0 it is
driven over JSON: `wifi_set`, `wifi_clear`, `wifi_status` and `wifi_scan` go to the bridge when one
is in use (and the console's `wifi-set` and `wifi-show`), and [`wifi_bridge`](API.md#wifi_bridge--the-wi-fi-bridge-on-the-ethernet-port) updates,
reboots, resets it or installs its certificate. The console's `net-reset` clears its Wi-Fi.

Built-in Wi-Fi wins: while it is configured, the pod takes no address through a bridge and the
Wi-Fi commands act on the built-in Wi-Fi, even when it cannot join (`wifi_status` says
`bridge_ignored`). `wifi_clear` switches to the bridge. Pairing and bridge updates carry on either
way. The pod pairs with the bridge on its own the first time it sees it. If `wifi_status` reports
`the bridge is paired with another pod`, press PAIR on the bridge.

**Genuine bridges only** (firmware 3.10.7+). Every bridge (firmware 0.1.3+) has its own identity
key and, once provisioned, a certificate for it signed with our release key. When the pod pairs,
the bridge shows the certificate and signs the pairing with its identity key; the pod checks both
and only then sends it the Wi-Fi password (`wifi_set`) and stores it as its bridge. A bridge
without a valid certificate (not genuine, or not provisioned yet, or older bridge firmware) still
pairs, so its status, scan and bridge updates work, but `wifi_set` is refused with `the Wi-Fi
bridge has no identity certificate ...` or `the Wi-Fi bridge's identity check failed ...`. The pod
pairs on its own with any bridge, whether it has an address on Ethernet or not: after PAIR on the
bridge, with a replacement or after moving a bridge between pods, nothing is needed on the pod.
While its genuine bridge is on the port, HELLOs from another bridge are ignored.

Provisioning a bridge: `wifi_status` `bridge.identity` is its identity key (64 hex digits). Sign it
(the release workflow "Sign bridge identity", or `tools/fwsign.py sign-bridge-id` with the
developer key for a development pod) and install the 128-byte certificate over the USB link with
`{"cmd":"wifi_bridge","action":"cert","part":<1-4>,"hex":"<64 hex digits>"}`, a quarter at a time
(up to firmware 3.10.13 the console's `bridge cert <1-4> <hex>`).

**Bridge firmware.** The pod keeps the bridge's firmware in W25Q blob slot `bridge` (installed
like the other blobs, OTA target `bridge`; it must carry a good signed manifest). When the slot
holds a newer version than the bridge reports, the pod updates the bridge by itself (about a
minute, during which captures and uploads answer `BP0048`), and `wifi_status` `bridge.update`
shows the progress. A version that failed or rolled back is not tried again until the pod restarts
or `wifi_bridge` `update` asks for it.

### Log levels

Firmware 3.10.13+. Every background line on the console has a level, and the pod
prints the lines at or above its log level (`log`, or JSON `log_level`; default `info`):

| Level | What it prints |
|---|---|
| error | Faults only: a crash report, a failed iCE40, PSRAM, storage or identity check, out of memory. These always print, at every level. |
| warn | Also: something failed but the pod carries on, such as the first failure of a reconnect loop. |
| info | Also: boot lines and state changes (link up or down, an address, cloud connected or lost). The default. |
| debug | Also: detail for debugging. The `[cmd] <- "verb"` / `[cmd] -> ok ...` trace of every LAN, USB network and cloud command, each step and retry of the cloud, Wi-Fi and DHCP reconnect loops, cloud lease renewals, Wi-Fi signal (RSSI) misses, USB network link flaps, Ethernet error counters, capture and DAC progress lines. |

Retry loops are rate limited at every level: the first failure prints (for example
`[cloud] cannot connect: challenge connect failed`), then at most one line a minute
(`...: still failing (12 times)`), then one line when it works again (`[cloud] connected to
... (after 12 failed attempts)`). An unreachable cloud or a network without DHCP no longer
prints a burst of lines every 20-30 seconds. `[esp] SPI xfer error` is latched the same way.

Console command replies are not log lines: they print whatever the level. Firmware 3.10.12
and older has no levels and prints everything listed under `debug`.

### Moved to the JSON API

Firmware 3.9.0 removed these console commands; their JSON equivalents work over the
USB network adapter, the LAN and the cloud (`benchpod` 0.6.0+, the `embeddedci` SDK and
MCP server 2.12.0+ use them over `--connection usb`). Firmware 3.9.0 to 3.10.13 answered a
removed command with one line that said what to update (`la-voltage: moved to the network API
(JSON la_voltage) in firmware 3.9.0; ...`, and for the upload commands the `upload-begin error
...` shape older CLIs waited for). Firmware 4.0.0 answers them like any other unknown command:
`  unknown command 'la-voltage' (try 'help')`.

The commands that stay keep their names and reply formats: `status`, `ping`, `dfu`,
`identity`, `usb-network`, `lan-policy`, `lan-tls`, `log`, `selftest`, `reboot`, `wifi-set` and
`wifi-show`.

<a id="removed-in-firmware-400"></a>
#### Removed in firmware 4.0.0

Firmware 4.0.0 makes the console a recovery tool. These commands are gone from every build (they
answer `unknown command`); the JSON command does the same over the USB network adapter (CLI 0.10
and later use it). "USB link" means the JSON command runs only on a connection over the USB network
adapter (physical presence, like the console); Ethernet, Wi-Fi and the cloud get `BP0043`.

| Removed console command | JSON command |
|---|---|
| `wifi-scan` | `wifi_scan` |
| `wifi-clear` | `wifi_clear` (and the console's `net-reset`) |
| `identity-wipe <id\|unknown>` | `identity_wipe` with `confirm` (USB link) |
| `sig-policy [required]` | `sig_policy` |
| `rollback-floor [clear]` | `sig_policy` (`rollback_floor`; `clear_floor`, USB link) |
| `blobs` | `blob_status` |
| `flash-esp32` | `ota_begin` target `esp` (`benchpod install-blobs --force --only esp`), which writes the image straight into the Wi-Fi module |
| `cloud-ca`, `cloud-ca-clear` (`ca`, `ca-clear`) | `cloud_ca` (USB link or the cloud to clear; `ota_begin` target `ca` to install) |
| `cloud-proxy`, `cloud-proxy-set`, `cloud-proxy-clear` (`proxy*`) | `cloud_proxy` (USB link or the cloud to change) |
| `eth <stop\|start\|restart\|stats>` | `eth` (and the console's `net-reset`) |
| `bridge [scan\|update\|reboot\|cert\|factory-reset]` | `wifi_status` (`bridge`), `wifi_scan`, `wifi_bridge` (`cert`: USB link) |
| `usb-reattach` | `usb_reattach` |

These moved to development builds only: `uid`, `led`, `psram-selftest`, `psram-test`,
`flash-ice40`, `wifi-static`, `esp-reset-pulse`, `esp-mon` and `flash-esp32-sync` (with the
diagnostics that were already there).

#### Removed in firmware 3.9.0

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
| `can ...` | `can_start`, `can_write`, `can_read`, `can_status`, `can_respond`, `can_term`, `can_stop` |

### `net-reset` result

`net-reset` first prints `net-reset eth autonegotiation, link and DHCP again`, then wipes both
places Wi-Fi credentials can be: the pod's own config store, and the ESP32-C3's NVS partition
(erased through the C3's ROM loader and verified blank by MD5), as JSON
[`wifi_clear`](API.md#wifi_set--wifi_clear) does.

| Outcome | Output |
|---|---|
| Both wiped | `net-reset wifi cleared (pod flash and the ESP32-C3's NVS)` |
| Pod wiped, C3 not | `net-reset wifi cleared on the pod, but the ESP32-C3's NVS was not erased (<why>): run net-reset again, or install the esp image again` |
| No C3 on the board (digital v3r3) | `net-reset wifi cleared (pod flash; this BenchPod has no ESP32-C3)` |
| A Wi-Fi bridge in use | `net-reset wifi cleared on the Wi-Fi bridge` |

`<why>` is `safe mode: ...`, `busy: ...` (a capture, upload or update holds the shared bus), or
`the C3 did not answer its ROM loader or the erase did not verify (see log)`; the `[espflash]`
log lines above the result say which step failed. Installing the `esp` image again
(`benchpod install-blobs --force --only esp`) rewrites the whole C3 image, NVS region included.

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

The ROM bootloader writes any image, signed or not. That is on purpose: firmware 4.0.0 and later
take only signed images over the network, the USB network adapter and the cloud, so USB DFU is
the way to install firmware you built yourself, and it needs the pod on a USB cable. Holding
BOOT while tapping RESET also enters the ROM bootloader. (Firmware 3.10.7 to 3.10.13 refused
`dfu` under `sig-policy required` with `dfu error sig-policy is required ...`.)

---

## Talking to it from Go

Any pure-Go serial library works since the device is a normal serial port, with no
libusb/cgo dependency. For example, with `go.bug.st/serial`:

```go
port, err := serial.Open("/dev/cu.usbmodem1101", &serial.Mode{BaudRate: 115200})
// write: port.Write([]byte("status\n"))
// read lines until the "> " prompt, then look for "device : benchpod".
```

Discover the port by enumerating serial ports and matching USB VID `0x1209` and
PID `0xEC10` (the next firmware release after 4.0.4 and later) or VID `0x0483` and PID `0x5740`
(firmware up to and including 4.0.x; pre-release 3.9.0 builds used `0x1209:0x0001`); `enumerator.GetDetailedPortsList()` exposes the VID/PID. Or match
the platform path patterns in the table above. The UART bridge and the
CMSIS-DAP probe are not on the console; use the USB network adapter, the LAN or the cloud for those
(`uart_proxy_start`, `dap_start` in [API.md](API.md)).
