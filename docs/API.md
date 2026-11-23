# bench-pod-firmware TCP/JSON API

The firmware exposes a plain-text JSON API over a TCP socket. Commands are sent as single-line JSON objects and responses are returned as one or more single-line JSON objects. The same commands run over the embeddedci.com cloud link (the command channel and the byte tunnels, see [While a cloud job holds the pod](#while-a-cloud-job-holds-the-pod)), and the same socket also speaks [SCPI](#scpi-reference). The USB console has its own text commands: see [usb-serial-interface.md](usb-serial-interface.md).

---

## Transport

| Property | Value |
|---|---|
| Protocol | TCP |
| Port (Ethernet, Wi-Fi) | `8443`, TLS 1.2 (firmware 3.9.3+; from 4.0.0 the only port the JSON API answers on there by default); see [LAN TLS](#lan_tls--the-lan-api-over-tls) |
| Port (USB network adapter) | `8080` (plaintext) and `8443` (TLS) |
| Plaintext `8080` on Ethernet and Wi-Fi | Off by default from firmware 4.0.0 (`lan_tls` `"https_only":false` turns it on); the default port up to 3.10.13 |
| Framing | Newline-delimited (`\n`) — one JSON object per line |
| Encoding | UTF-8 |
| Concurrency | Up to 5 LAN connections at once; the commands that use the capture hardware take turns (a second one gets `BP0047`, `pod_busy`) |

Connect with any TCP client:

```bash
openssl s_client -connect <device-ip>:8443 -quiet     # the LAN (firmware 4.0.0 and later)
nc 10.237.x.y 8080                                    # the USB network adapter
```

**Over USB.** From firmware 3.9.0 the pod's USB port is also a network adapter (CDC-NCM): a computer on the cable reaches port `8080` at the pod's fixed `10.237.x.y` address with no Ethernet or Wi-Fi, and everything below works the same way. The link stays plaintext on `8080` in firmware 4.0.0: it is a point-to-point cable that never crosses a network, and the pod drops packets for its USB subnet that arrive on another interface. `8443` answers there too. See [USB network adapter](usb-serial-interface.md#usb-network-adapter). The `status` field `usb` gives both ends of that link.

The IP address is printed on the USB console (and the USART2 debug pins) at boot, and is also returned by the `status` command and the console `status` command. The pod also advertises itself over mDNS (on Ethernet and Wi-Fi unless the [LAN policy](#lan_policy--what-the-external-lan-may-do) is `off`; on the USB link always). The pod is IPv4-only: it answers an AAAA (IPv6 address) lookup of its `.local` name with an NSEC record saying it has only an A record, so a resolver that asks for both (for example `getaddrinfo` on macOS) gets the IPv4 address at once instead of waiting about 5 s for an IPv6 answer.

**Over TLS.** From firmware 3.9.3 the same API also listens on port `8443` with TLS 1.2. Everything below works the same way on it (JSON, SCPI when it is on, the raw modes). The pod's certificate is its own (generated on first boot, or one you install), so clients pin its SHA-256 on first use; `benchpod`, the `embeddedci` SDK and the MCP server do this for you, with no user action, and use `8443` by default when the pod has it. From firmware 4.0.0 [HTTPS-only](#lan_tls--the-lan-api-over-tls) is on by default: `8080` answers only on the USB network adapter (and, for SCPI only, on the LAN when SCPI is on). A plaintext JSON line that reaches `8080` from the LAN anyway (SCPI on) gets `BP0075` (`lan_plaintext_off`) and the connection closes.

**Latency.** Small replies (a JSON ack) go out as soon as they are ready. While a connection is a CMSIS-DAP probe (`dap_start`) the pod turns Nagle's algorithm off (`TCP_NODELAY`) on it, because flashing is a stream of small request and response packets. No client action is required.

**Line length.** A JSON command line is at most 1279 bytes; a longer one gets `"command too long"` and is dropped. (A SCPI line is at most 255 bytes.) Bulk data goes through the commands that switch a connection to raw bytes (`load_bin`, `dap_start`, `uart_proxy_start`) or through base64url chunks (`load`, `ota_data`).

---

## Security model

**Read this before putting a pod on a network you do not control.**

The pod has two command channels, and they have deliberately different trust models.

### Pairing: who may use the LAN API

**From firmware 4.0.1 the LAN API needs pairing.** A computer on Ethernet or Wi-Fi gets only
`ping`, `status`, `diag`, `cloud_status`, `identity_public` and `lan_tls` until it is paired; every other command
gets `BP0081` (`lan_not_paired`). Pairing puts the SHA-256 of the computer's TLS client key on the
pod's list ([`lan_clients`](#lan_clients--paired-computers)): `benchpod setup` does it over the
pod's USB-C cable, and embeddedci.com does it for a registered pod. The USB console, the USB
network adapter and the cloud never need pairing. Pairing is on by default, also on a pod updated
from older firmware; `lan_policy` `"pairing":false` (over USB or from the cloud) turns it off,
which is the behavior below. See `docs/design/lan-pairing.md`.

### Without pairing the LAN API is unauthenticated

With pairing off (or on firmware before 4.0.1), the JSON-over-TCP and SCPI interfaces on port
`8080` have **no authentication, no authorization, and no transport encryption** (from firmware 4.0.0 `8080` is off on Ethernet and
Wi-Fi by default, and SCPI is off everywhere by default). Port `8443` (firmware 3.9.3+) is the same API over
TLS: it adds **encryption and integrity** against the network path (a Wi-Fi bridge, a shared
switch), not authentication. Any client that can open a TCP connection to either port has the
full command set, which includes:

- driving the DAC and the analog outputs,
- switching target power on and off (`target_power`),
- driving the SWD probe — i.e. halting, reading, and reflashing an attached target,
- uploading firmware to the pod itself over OTA.

There is no login, no API key, and no allow-list. The USB network adapter (firmware 3.9.0+)
is the same tier as the LAN: port `8080` on it is this API. The LAN policy, though, is for the
external LAN only: the USB adapter stays on and open under every policy (physical presence). The
USB-CDC console is a separate, higher channel and assumes physical access.

**The LAN policy limits this on the external LAN** (Ethernet and Wi-Fi). `lan_policy` (below) is
`open` by default, which is everything above. `locked` keeps the external LAN to reads and
instrument control (tiers T0 and T1 of `src/cmd_tier.c`): persisted settings (Wi-Fi, cloud, DAC
limits, calibration, the policies) and firmware updates then need the cloud or USB. `off` refuses
TCP connections over Ethernet and Wi-Fi and stops the mDNS advertisement there. Only the cloud and the USB console can change it, from the web app's device
settings or `benchpod lan-policy set`.

**HTTPS-only** (`lan_tls` with `https_only`, default on from firmware 4.0.0, off before) keeps
plaintext `8080` off on the external LAN, so only TLS crosses Ethernet and Wi-Fi; the USB network
adapter keeps `8080`. **SCPI** (`lan_tls` with `scpi`, firmware 4.0.0, default off) is switched on
on purpose. Both are changed like the LAN policy: from the USB console, the USB network adapter
or the cloud, never from Ethernet or Wi-Fi (over TLS or not), since an active man-in-the-middle
could otherwise open plaintext or swap the certificate before a client pins it. See
[`lan_tls`](#lan_tls--the-lan-api-over-tls).

**Treat the LAN interface as a trusted-network-only interface.** Put the pod on a lab
VLAN or behind a firewall rule that admits only the hosts that need it. This is the
same posture as most bench instruments (a SCPI-over-LAN scope or DMM is also
unauthenticated), but it is a conscious choice, not an oversight — say so in your own
threat model rather than assuming the pod defends itself.

### The cloud channel *is* authenticated and encrypted

Nothing above applies to the EmbeddedCI cloud path. That channel is protected in both
directions:

| Property | How |
|---|---|
| Confidentiality + integrity | TLS (WSS). The pod dials **out**; it never accepts an inbound cloud connection, so it needs no port forwarded. |
| Server authentication | The server certificate chain is verified against **two Let's Encrypt roots embedded in the firmware** (ISRG Root X1 and X2), plus an SNI/CN-SAN hostname check. Verification is enforced, not advisory — a failed verify drops the link. The old unauthenticated bring-up mode was removed, and any stored `verify=0` from older firmware is upgraded on load so it cannot downgrade a link. |
| Device authentication | Ed25519 challenge–response. The pod fetches a server nonce, signs it with its device private key, and presents `device_id` + nonce + signature on the WebSocket upgrade; the server verifies against the public key registered for that device. |
| Key custody | The private key is generated on-device from the hardware TRNG at first boot, stored in its own flash sector, and **never leaves the unit** — the pod only ever signs. |

One caveat worth knowing: the pod has no RTC, so certificate **validity dates are not
checked** (`MBEDTLS_HAVE_TIME_DATE` is off). An otherwise-valid certificate from a
trusted root is accepted after expiry.

### Domain separation between the two channels

`identity_pop` (below) is reachable over the *unauthenticated* LAN API, so anyone on
the LAN can obtain a signature over a nonce of their choosing. That deliberately
cannot be replayed as a cloud login: LAN proofs are signed under the context
`benchpod-pop:v1`, cloud WebSocket authentication under `benchpod-ws-auth:v1`, and the
signed message is `context || 0x00 || nonce`. A signature collected from the LAN is
therefore not a valid cloud-auth signature, and vice versa.

---

## Request Format

Every request is a flat JSON object on a single line, terminated with `\n`.

```
{"cmd":"<command>", ...parameters...}\n
```

The `cmd` field is required. All other fields are command-specific and documented below. Unknown fields are silently ignored.

Two fields go with any command: `client_api`, the [client API level](#client-compatibility) the
client was built for, and `client`, its name and version for the refusal text. A client sends them
on every command:

```
{"cmd":"la_pins","client_api":1,"client":"sdk 3.0.6"}\n
```

---

## Response Format

### Success

```json
{"status":"ok","data":<value>}\n
```

`data` is command-specific. It may be `null`, a JSON object, or a JSON array.

### Error

```json
{"status":"error","code":"BP0031","args":{"la":4,"owner":"gpio","function":"gpio","instance":""}}\n
{"status":"error","message":"<description>"}\n
```

A refusal a client acts on carries a stable error code (`BP0001` and up) and the values that go
with it in `args` (left out when the code has none), and no `message` (firmware 4.0.0 and
later; firmware 3.10.8 to 3.10.13 sent a message next to some codes). The client renders the
code's message and its fix from the catalog: [`errors/errors.json`](../errors/errors.json),
generated from [`errors/catalog.toml`](../errors/catalog.toml), with each code's message template
(`{la}` is an arg), cause and fix for the SDK, CLI, MCP and the web app. Codes never change meaning
and are never reused; each is on `https://embeddedci.com/docs/errors#<code>`, and
[`errors/ERRORS.md`](../errors/ERRORS.md) lists them. One code also carries a `message`: `BP0030`
(see [Client compatibility](#client-compatibility)), for clients too old to know it.

The second form is a refusal without a code: a bad argument or a failure that only a person
reads (`"unknown waveform"`, `"capture failed"`). Clients show its text as it
is and do not match on it. Over the cloud command channel the envelope's `"error"` is the
message, or the code when there is none, with `"code"` and `"args"` beside it.

### Chunked data (multi-packet responses)

Commands that return sample arrays (`capture`, `measure`, `test`, `capture_dual`, `capture_read`, `la_capture`, `sensor_la`) send the data in several packets. The first packet uses `"status":"ok"` and the ones after it `"status":"chunk"`. Every packet has a `"more"` boolean that says whether more packets follow. The packet size follows the connection's send buffer (at most 256 samples), so do not count on a fixed size.

```json
{"status":"ok",    "bits":16, "adc_sample_rate_hz":400000, "b64":"AAABAP__...", "more":true}\n
{"status":"chunk", "b64":"NBL_f...", "more":true}\n
{"status":"chunk", "b64":"QBQ0Fz...", "more":false}\n
```

ADC samples are 16-bit counts (`"bits":16`); see [ADC sample values](#adc-sample-values). A client must read until it receives a packet with `"more":false` to know the transfer is complete. If the client stops reading for a few seconds the pod ends the transfer and closes the connection.

#### Base64 samples (the default)

The 16-bit samples of `capture`, `measure`, `test`, `capture_dual` and `capture_read` come as `"b64"`: unpadded base64url (RFC 4648 §5) of the little-endian `uint16` values. That is 8/3 bytes per sample instead of up to 6 as decimal, and it reads back about 6.7x faster (2M ADC samples: 4.2 s instead of 28 s over the LAN). `capture_dual`'s LA region still comes as `la_edges`. Firmware 4.0.0 made it the default; up to 3.10.13 a request had to say `"enc":"b64"`, which still works.

Add `"enc":"dec"` for the decimal `"data"` array instead (what firmware up to 3.10.13 sent by default):

```json
{"status":"ok",    "bits":16, "data":[32768,33012,33270,...], "more":true}\n
{"status":"chunk", "data":[33501,33790,34012,...], "more":false}\n
```

---

## Commands

Every command the firmware accepts is in this table. **Tier** decides who may run it (see
`src/cmd_tier.c` and `docs/design/policy-commands.md`): T0 reads, T1 instrument control, T2
persisted settings, T3 firmware. A locked LAN keeps T2 and T3 for the cloud and the USB console,
and a cloud tunnel never goes above the tier the server allowed its user (T2 and T3 need an
organization owner or admin). "T0/T2" commands read at T0 and change a setting at T2.
Commands that send several packets or switch the connection to raw bytes need a stream connection
(a LAN socket or a cloud byte tunnel). The single-reply cloud command channel refuses `capture`,
`capture_dual`, `capture_read`, `measure`, `test`, `load`, `load_bin`, `replay`,
`sensor_la`, `la_capture`, `dap_start`, `uart_proxy_start` and `speedtest` with
`command not supported over cloud channel`. The USB console does not run JSON commands at all.

| `cmd` | Tier | Purpose | `data` returned | Multi-packet |
|---|---|---|---|---|
| `ping` | T0 | Connectivity check | `"pong"` (string) | no |
| `status` | T0 | Firmware, network, board and capability info | object | no |
| `diag` | T0 | Heap, stack, network-stack and console diagnostics (firmware 3.10.9+) | object | no |
| `generate` | T1 | Start a DAC waveform | `null` or object | no |
| `capture` | T0 | ADC snapshot, up to 32768 samples | array of uint16 | yes |
| `measure` | T1 | DAC waveform + ADC capture in one command | array of uint16 | yes |
| `capture_dual` | T0 | Deep ADC + LA capture off one trigger | array of uint16, LA as edges | yes |
| `capture_read` | T0 | Resume the read-back of the last `capture_dual` | array of uint16 | yes |
| `la_capture` | T0 | Deep raw capture of all 14 LA channels | LA words as edges | yes |
| `test` | T0 | Synthetic sample pattern (no FPGA) | array of uint16 | yes |
| `load` | T1 | Upload a waveform for replay (base64url chunks) | object | no |
| `load_bin` | T1 | Upload a waveform as raw bytes (RAM or PSRAM) | object | no |
| `replay` | T1 | Play the recorded or uploaded trace out the DAC | object | no |
| `dac_stop` | T1 | Stop any running DAC output (parks it when DAC limits are set) | `null` or object | no |
| `dac_set` | T1 | Hold the DAC at a raw 8-bit level | `null` | no |
| `dac_limits` | T0/T2 | Read, set or clear the DAC output limits for an external output stage | object | no |
| `analog_path` | T1 | Apply a named analog path (flips mux + relays) | object | no |
| `dac_out` | T1 | Route a DAC output path + set a calibrated voltage | object | no |
| `current_out` | T1 | Hold a current on the 4-20 mA output (J9), in µA | object | no |
| `adc_read` | T1 | Route an ADC source + return a calibrated reading (mV) | object | no |
| `adc_range` | T1 | Read or switch the ADC input range (normal ±32.8 V / low ±5.46 V, analog module v3r4+), or run its self-test | object | no |
| `calibrate` | T0/T2 | Run, read or clear this pod's own calibration: the ADC offset (measured on the `current_in` input, J8; applies to every ADC source from 3.9.3) and, from 3.9.3, the DMM-guided DAC lines and ADC gains | object | no |
| `dac_mux` | T1 | Low-level DAC output mux (U55); prefer `dac_out` | object | no |
| `cal_switch` | T1 | Low-level calibration relays (U58); prefer `analog_path` | object | no |
| `dac_control_loop` | T1 | Arm (or disarm) the in-fabric closed-loop DAC | object | no |
| `dac_loop_input` | T1 | Retarget a running control loop's input | object | no |
| `dac_loop_probe` | T0 | One live operating point of the control loop | object | no |
| `fpga_image` | T1 | Switch the running iCE40 image (0 = loop, 1 = deep replay, 2 = sensor emulation) | object | no |
| `psram_ping` | T0 | iCE40 to PSRAM write-path check | object | no |
| `psram_recover` | T1 | Reboot the pod to recover an inoperable PSRAM datapath | object | no |
| `la_step` | T1 | Start a step pulse train on an LA pin (`la` with `steps` up to 3.10.13) | object | no |
| `la_pull` | T1 | Switch or read the LA pull resistors (up/down/off) (`la` up to 3.10.13) | object | no |
| `la_pins` | T0 | Report all 14 LA pins: function, gpio mode/level, pull, live levels | object | no |
| `gpio` | T1 | Configure / release / drive / read LA pins as GPIO | object | no |
| `la_dir` | T1 | Tell the pod which LA pins are DUT outputs, inputs or bidirectional | object | no |
| `lease_acquire` | T1 | Take the [pod lease](#pod-lease), or join the one held with the same id | object | no |
| `lease_renew` | T1 | Push the deadline of the [pod lease](#pod-lease) | object | no |
| `lease_release` | T1 | Free the [pod lease](#pod-lease) | object | no |
| `la_voltage` | T1 | Select the LA I/O-bank voltage (1.8/3.3 V); required before any LA op | object | no |
| `nrst` | T1 | Drive the dedicated target-reset line (v3 pods) | object | no |
| `usb_cc` | T0 | Read the USB-C CC lines: orientation + source current advertisement (v3 pods) | object | no |
| `usb_host` | T0/T2 | USB host mode: the pod hosts a board's USB port for a while and reads what enumerates (read: T0; `enabled`: T2) | object | no |
| `target_power` | T1 | Enable/disable a target-power eFuse | object | no |
| `target_status` | T0 | Read target-power eFuse state (enabled/fault/valid) | object | no |
| `power_status` | T0 | One-shot INA238 reading of both supply rails | object | no |
| `power_profile` | T1 | Record a current/voltage profile of one eFuse rail | object | on `stop` |
| `module` | T0/T2 | Digital v3r3: the analog module's state, and its ID and calibration EEPROM | object | no |
| `module_power` | T1 | Digital v3r3: switch the analog module's 5 V and the USB-A bridge port | object | no |
| `dap_start` | T1 | Turn the connection into a CMSIS-DAP probe | `"dap ready"` (string) | no |
| `uart_proxy_start` | T1 | Enter transparent UART bridge mode | `"uart ready"` (string) | no |
| `spi_start` | T1 | Claim four LA pins as an SPI master | object | no |
| `spi_stop` | T1 | End the SPI session | `"spi stopped"` | no |
| `spi_status` | T0 | The SPI session's pins, rate and mode | object | no |
| `spi_xfer` | T1 | Raw full-duplex SPI transfer | object | no |
| `spi_stream` | T1 | Send a staged PSRAM upload in one CS frame | object | no |
| `esp_flash` | T1 | Flash an ESP32-family target over its UART ROM bootloader | object | no |
| `i2c_start` | T1 | Claim two LA pins as an I2C master | object | no |
| `i2c_stop` | T1 | End the I2C session | `"i2c stopped"` | no |
| `i2c_status` | T0 | The I2C session's pins, rate and pull-ups | object | no |
| `i2c_xfer` | T1 | Write, read, or write then read an I2C device | object | no |
| `i2c_scan` | T1 | List the addresses that answer on the bus | object | no |
| `spi_flash` | T1 | SPI NOR flash operations (id, read, erase, program) | object | no |
| `sensor_define` | T1 | Upload and compile a sensor definition (chunked) | object | no |
| `sensor_start` | T1 | Start an emulated sensor instance (I2C, SPI, UART or analog) | object | no |
| `sensor_set` | T1 | Set an instance's readings | object | no |
| `sensor_stimulus` | T1 | Upload a stimulus (chunked) | object | no |
| `sensor_stimulus_start` | T1 | Play the stimulus | object | no |
| `sensor_stimulus_stop` | T1 | Stop the stimulus | object | no |
| `sensor_fault` | T1 | Inject a fault into an instance | object | no |
| `sensor_fault_clear` | T1 | Clear an instance's faults | object | no |
| `sensor_stop` | T1 | Stop an instance (or all) and forget its definition | object | no |
| `sensor_status` | T0 | Instances, their values, faults and bus counters (paged) | object | no |
| `sensor_regs` | T0 | Read an I2C/SPI instance's register image (base64) | object | no |
| `sensor_la` | T0 | Raw I2C-bus logic capture of an I2C instance | array of uint8 | yes |
| `can_start` | T1 | Bring up classic CAN or CAN FD (`can_config` up to 3.10.13) | object | no |
| `can_write` | T1 | Send one CAN frame | object | no |
| `can_read` | T0 | Drain up to 8 received CAN frames | object | no |
| `can_status` | T0 | CAN state and error counters | object | no |
| `can_term` | T1 | Switch the 120 Ω CAN termination | object | no |
| `can_respond` | T1 | Add or clear an automatic CAN reply rule | object | no |
| `can_stop` | T1 | Turn CAN off and its termination (`can_disable` up to 3.10.13) | object | no |
| `identity_public` | T0 | Get the device Ed25519 public key | object | no |
| `identity_pop` | T0 | Sign a nonce (proof of possession) | object | no |
| `identity_wipe` | T2 | Erase the device key and make a new one, over the USB link only (the console's `identity-wipe` up to 3.10.13) | object | no |
| `cloud_status` | T0 | The cloud link's state and stored settings | object | no |
| `cloud_set` | T2 | Store the cloud server and device id, then connect | object | no |
| `cloud_clear` | T2 | Forget the cloud settings and disconnect | `"cleared"` | no |
| `cloud_ca` | T0/T2 | Read or clear the company CA for the cloud link | object | no |
| `cloud_proxy` | T0/T2 | Read, set or clear the HTTP proxy for the cloud link | object | no |
| `wifi_status` | T0 | Wi-Fi state and transport counters | object | no |
| `wifi_scan` | T0 | Start a Wi-Fi scan, or read its networks | object | no |
| `wifi_set` | T2 | Store Wi-Fi credentials and connect | object | no |
| `wifi_clear` | T2 | Forget the Wi-Fi credentials (pod and ESP32-C3) | `"cleared"` | no |
| `wifi_bridge` | T2 | The Wi-Fi bridge on the Ethernet port: update, reboot, factory reset, identity certificate (the console's `bridge` up to 3.10.13) | object | no |
| `usb_reattach` | T1 | Drop off the USB bus and enumerate again (the console's `usb-reattach` up to 3.10.13) | object | no |
| `eth` | T0/T2 | Wired link control and diagnostics | string or object | no |
| `speedtest` | T1 | Cloud throughput probe over a tunnel | none (raw bytes) | no |
| `sig_policy` | T0/T2 | Read or set which signed updates the pod accepts; clear the rollback floor over the USB link | object | no |
| `lan_policy` | T0/T2 | Read or set what the LAN API may do | object | no |
| `usb_network` | T0/T2 | Read, or over the USB link set, the USB network adapter setting | object | no |
| `log_level` | T0 | Read or set the console log level | object | no |
| `lan_tls` | T0/T2 | The LAN API's TLS certificate (port 8443) and the HTTPS-only setting | object | no |
| `lan_clients` | T0/T2 | List, pair or remove the computers paired for the LAN API (changes over the USB link or the cloud) | object | no |
| `ota_begin` | T3 | Start a firmware or blob update | object | no |
| `ota_data` | T3 | Send one base64url chunk of the image | object | no |
| `ota_end` | T3 | Verify the staged image's SHA-256 (and signature) | object | no |
| `ota_status` | T0 | The update session's state | object | no |
| `ota_abort` | T3 | Drop the staged image | object | no |
| `ota_selftest` | T3 | Test the flash writer on a scratch sector | object | no |
| `ota_commit` | T3 | Write the verified image and reset (or write a blob) | object | no |
| `blob_status` | T0 | What each W25Q blob slot holds | object | no |
| `usb_flash` | T1 | Flash a board over USB: UF2 and drag-and-drop drives (DAPLink), USB DFU / STM32 DfuSe, Teensy, Arduino AVR and SAMD bootloaders, ESP32 | object | no |
| `xip` | T0/T2 | Code that runs from the W25Q: whether this pod can run it, and a self-test (`selftest`: T2) | object | no |
| `wiring_list` | T0 | The wiring profiles stored on the pod, and the default | object | no |
| `wiring_get` | T0 | One stored wiring profile, up to 600 bytes a call | object | no |
| `wiring_put` | T1 | Store a wiring profile, up to 600 bytes a call | object | no |
| `wiring_delete` | T1 | Delete a stored wiring profile | object | no |
| `wiring_default` | T1 | Read or set the default wiring profile | object | no |
| `emi_log` | T0 | EMI test build only: read the run log out of the W25Q | object | no |

Commands not in this table answer `"unknown cmd"`.

**Names and units (firmware 4.0.0).** A rate or frequency is in Hz and its name ends in `_hz`:
`sample_rate_hz` for a sample rate (`adc_sample_rate_hz` and `la_sample_rate_hz` where a capture
has both), `freq_hz` for a waveform, `clock_hz` for a bus clock. A voltage given in volts ends in
`_v` (`dac_out` `v`, `generate` `amplitude_v`); integer millivolt settings and readings end in
`mv` or `_mv` and microvolts in `uv`. A DAC code is `code16`. Durations end in their unit (`_ms`,
`_us`, `_s`). Firmware up to 3.10.13 used `freq`, `hz`, `rate_hz` and `*_rate_mhz` (in MHz) in
places; the [features table](#features-table) entry `json_names_4` marks the change.

<a id="gates-that-apply-to-every-command"></a>
### Gates that apply to every command

Before a command runs, the firmware checks these, in this order (`src/cmd_gate.c`). The first one
that applies answers with its code (or, for safe mode, its message) and the command does not run.

| Gate | Applies to | Refusal |
|---|---|---|
| [Client API level](#client-compatibility) | every command except `status`, `diag`, `ping`, `lease_release` and the update commands, from a client whose `client_api` (none = 0) is below the firmware's `min_client_api` | `BP0030` (`client_too_old`, with a `message` for old clients; firmware 4.0.0) |
| LAN policy `locked` | a T2 or T3 command on a LAN connection | `BP0040` (`lan_locked`, args `cmd`) |
| A cloud job holds the pod (lease) | anything but a light read, a `lease_*` command or [pod administration](#pod-administration-passes-a-lease) on a LAN connection | `BP0025` (`lease_busy`, holder `a cloud job`, purpose the cloud holder) |
| Cloud tunnel tier | a command above the tier the server allowed the tunnel's user (its `tunnel.open` `max_tier`; a tunnel opened without one gets T1 from firmware 4.0.0, everything up to 3.10.13) | `BP0045` (`role_required`, args `cmd`) |
| Safe mode | everything except network, status, identity, target power, `dac_limits` and CAN, while the iCE40/PSRAM are off | `safe mode: iCE40/PSRAM are off. Unplug and replug the pod` |
| Digital-only board | the analog commands (`generate`, `capture`, `measure`, `load`, `load_bin`, `replay`, `dac_limits`, `dac_set`, `dac_mux`, `cal_switch`, `analog_path`, `dac_out`, `current_out`, `adc_read`, `calibrate`, `dac_control_loop`, `dac_loop_probe`, `dac_loop_input`) and a `capture_dual` with ADC samples | `BP0046` (`no_analog_front_end`, args `cmd`; SCPI shows the text `this BenchPod has no analog front end (digital board): ...`) |
| [Pod lease](#pod-lease) held by someone else | anything but a light read, a `lease_*` command or [pod administration](#pod-administration-passes-a-lease), on any connection that does not own the lease (LAN, USB, the cloud) and without the held id in its `lease` field | `BP0025` (`lease_busy`, args `holder`, `purpose`, `left_s`) |
| [Pod lease](#pod-lease) stopped | the next command of each connection that owned it (once), and for 30 minutes any command whose `lease` field is the stopped id | `BP0026` (`lease_lost`, args `by`) |
| [DAC pinout](#dac-pinout-on-the-flat-cable) mismatch | the commands that drive the DAC (`generate`, `measure`, `replay`, `dac_set`, `dac_out`, `dac_control_loop`, `current_out` with `ua`, `calibrate` with `dac` or `adc`, `adc_range` with `test`) on a digital board and an analog module with different flat-cable pinouts | `BP0080` (`dac_pinout_mismatch`, args `cmd`, `digital`, `module`) |
| DAC output limits | commands that would move the DAC outside the limits | see [`dac_limits`](#dac_limits--dac-output-limits-for-an-external-output-stage) |

Light reads are the T0 commands that do not use the capture hardware: everything at T0 except
`capture`, `capture_dual`, `capture_read`, `la_capture`, `sensor_regs`, `sensor_la`,
`can_read`, `psram_ping` and `test`. Commands that use the capture hardware (PSRAM, ADC, LA) also
take turns with each other: a second one gets `BP0047` (`pod_busy`) until the first is done.

---

### `ping` — connectivity check

Verify the TCP connection and command path are working. No parameters. No side effects.

#### Request

```json
{"cmd":"ping"}
```

#### Response

```json
{"status":"ok","data":"pong"}
```

Use this as a lightweight keepalive or to confirm the firmware is reachable before issuing signal commands.

---

### `generate` — DAC waveform output

Start continuous waveform generation on the DAC (16 bits, clocked by the FPGA). Give the waveform in volts on a DAC output (`path`, `amplitude_v`, `offset_v`), or in microamps on the 4-20 mA output (`amplitude_ua`, `offset_ua`). The pod routes the output, turns the values into DAC codes with the same per-path calibration as [`dac_out`](#dac_out--route-an-output-path-and-set-a-calibrated-voltage) and [`current_out`](#current_out--hold-a-current-on-the-4-20-ma-output-j9), and builds the waveform at the DAC's full 16 bits. One step is about 50 µV on the 3v3 output.

Firmware up to 3.10.13 also took 8-bit levels (`amplitude` 0-127, `offset` 0-255, no calibration). Firmware 4.0.0 takes only this form: a request without `amplitude_v` or `amplitude_ua` is refused.

#### Request

```json
{"cmd":"generate","waveform":"sine","freq_hz":1000,"path":"3v3","amplitude_v":1.48,"offset_v":1.65}
{"cmd":"generate","waveform":"square","freq_hz":10,"amplitude_ua":4000,"offset_ua":12000}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `waveform` | string | yes | — | `"sine"`, `"square"` or `"sawtooth"` |
| `path` | string | with `amplitude_v` | — | The DAC output: `"3v3"`, `"5v"` or `"12v"`. The pod switches to it before the waveform starts. |
| `amplitude_v`, `offset_v` | number (V) | together | — | Peak and center in volts. A peak up to 5% of the output's range past what it can reach is clamped to that end (so 0 to 3.3 V works on 3v3, whose calibrated range starts a few mV above 0); further out is refused. |
| `amplitude_ua`, `offset_ua` | number (µA) | together, instead of `_v` | — | Peak and center on the 4-20 mA output (J9), which is routed instead. The trough may go down to 4000 µA (played as the live zero, like `current_out`); the top is the output's maximum. |
| `freq_hz` | number (Hz) | no | `1000` | Output frequency |
| `duration_ms` | integer (ms) | no | `0` | How long to generate. `0` = run until the next command |
| `sample_rate_hz` | number (Hz) | no | auto | DAC sample-clock rate. Omit it to auto-pick the highest rate that fits one period in the waveform buffer. A **lower** rate reduces high-frequency clock feedthrough on the analog output (cleaner scope trace) at the cost of fewer samples per period. One DAC sample takes divider + 51 cycles of the 48 MHz engine clock (divider ≥ 4), so the maximum is 48 MHz ÷ 55 ≈ 873 kS/s; the firmware snaps to the nearest achievable rate, clamps faster requests to the maximum and logs the actual rate. |
| `route` | boolean | no | `true` | `false` keeps the current analog switching (for example `cal1`, a DAC to ADC loopback, which routing the output would undo); the volts still use `path`'s calibration. |
| `on_capture` | boolean | no | `false` | Gateware v27+: wait and start the waveform at the next capture's t0 (the DAC co-trigger). The reply is then `{"cotrig":true}`, or `{"cotrig":false}` when the gateware cannot co-trigger and the waveform started right away. |

```
sine:     value = offset + amplitude × sin(2π × t/T)
square:   value = offset ± amplitude  (high for the first half-period, low for the second)
sawtooth: value = offset - amplitude … offset + amplitude  (linear ramp per period)
```

**Frequency resolution**

The firmware precomputes one full period in a sample buffer and the FPGA loops it. Period length in samples is:

```
period_samples = floor(sample_rate / freq_hz)
```

where `sample_rate = 48 MHz / (divider + 51)`: the DAC engine runs on the 48 MHz `clk48` domain and spends 51 clocks per sample on top of the divider (3 to fetch the sample, 48 to shift the 24-bit SPI frame). The divider is at least 4 (the DAC's minimum gap between frames), so the maximum is about 873 kS/s.

The waveform buffer holds 2048 16-bit samples. Lower frequencies need a lower sample rate: leave `sample_rate_hz` out and the firmware picks one that fits the period.

#### Response

```json
{"status":"ok","data":null}
```

With `on_capture` the reply is `{"status":"ok","data":{"cotrig":true}}`. Firmware up to 3.10.13 replied `{"bits":16}` to the calibrated form.

#### Examples

```json
{"cmd":"generate","waveform":"sine","freq_hz":100000,"path":"3v3","amplitude_v":1.2,"offset_v":1.65}
```
→ 100 kHz sine, 0.45 to 2.85 V on the 3v3 output, runs until the next command.

```json
{"cmd":"generate","waveform":"square","freq_hz":50000,"path":"5v","amplitude_v":2.5,"offset_v":2.5,"duration_ms":2000}
```
→ 50 kHz square wave, 0 to 5 V, for 2 seconds, then the DAC stops.

```json
{"cmd":"generate","waveform":"sine","freq_hz":100000,"path":"3v3","amplitude_v":1,"offset_v":1.65,"sample_rate_hz":1000000}
```
→ 100 kHz sine clocked at 1 MHz (10 samples/period). Coarser waveform but much lower clock feedthrough, useful for a cleaner scope trace on breadboard wiring.

#### Error cases

| Condition | Message |
|---|---|
| Unknown or missing `waveform` | `"unknown waveform"` |
| Neither `amplitude_v` nor `amplitude_ua` (for example the old 8-bit form) | `amplitude_v (with path and offset_v) or amplitude_ua (with offset_ua) is required` |
| `amplitude_v` without a 3v3/5v/12v `path` | `path must be 3v3\|5v\|12v with amplitude_v` |
| `amplitude_v` without `offset_v` (or `_ua`) | `offset_v is required with amplitude_v` |
| `amplitude_v` not above 0 | `amplitude_v must be above 0 V` |
| A peak more than 5% of the range past the output's range | `the peaks <lo> to <hi> mV are outside the 3v3 output's <min> to <max> mV` (µA on the 4-20 mA output) |
| `freq_hz` ≤ 0 | `"generate failed"` |
| DAC limits are set | the [`dac_limits`](#dac_limits--dac-output-limits-for-an-external-output-stage) refusal |

---

### `capture` — ADC snapshot

Capture a fixed number of ADC samples (16-bit MCP33131, through the iCE40 into PSRAM) and return them. The reply arrives once the capture is done; meanwhile other connections keep working, but a second capture waits its turn (`BP0047`). A capture also fills the RAM replay buffer, so `replay` can play it straight back.

#### Request

```json
{"cmd":"capture","samples":<count>,"sample_rate_hz":<Hz>}
```

| Field | Type | Required | Default | Max |
|---|---|---|---|---|
| `samples` | integer | no | `256` | `32768` |
| `sample_rate_hz` | number (Hz) | no | max (~400000) | — |
| `trigger`, `trigger_timeout_ms` | object, integer | no | none | see [Capture triggers](#capture-triggers) |
| `enc` | string | no | `"b64"` | `"dec"` for decimal `data`, see [Base64 samples](#base64-samples-the-default) |

For more than 32768 samples use `capture_dual` with `la_samples` 0 (millions of samples, read back from PSRAM).

`sample_rate_hz` sets the ADC sample clock. Omit it for the maximum ~400 kSPS rate; a **lower** rate stretches the capture window so a slow waveform fits in the 32768-sample buffer. For example, 32768 samples at ~400 kSPS spans ~82 ms, and `80000` (80 kS/s) spans ~410 ms — enough to see a motor current waveform or other low-frequency signal. The v2 MCP33131 ADC runs in the 24 MHz domain and its divider is floored at 60, so the maximum rate is 24 MHz ÷ 60 ≈ 400 kSPS; the firmware snaps to the nearest achievable divider and logs the actual rate.

#### Response

Chunked JSON array (see [Chunked data](#chunked-data-multi-packet-responses)):

```json
{"status":"ok",    "bits":16, "data":[32768,33012,...], "more":true}\n
{"status":"chunk", "data":[33270,33501,...], "more":false}\n
```

Each sample is an unsigned 16-bit ADC count (see [ADC sample values](#adc-sample-values)).

#### Examples

```json
{"cmd":"capture","samples":1024}
```

```json
{"cmd":"capture"}
```
→ 256 samples (default).

#### Error cases

| Condition | Message |
|---|---|
| `samples` = 0 or > 32768 | `BP0067` (`samples_out_of_range`, `max` 32768; `capture_dual` takes deep captures) |
| Another capture or upload is running | `BP0047` (`pod_busy`) |
| The capture did not arm | `"capture failed"` |
| The PSRAM read-back failed | `"capture read-back failed"` |

#### Loopback test

To verify the signal path, start a waveform then capture:

```json
{"cmd":"generate","waveform":"sine","freq_hz":1000,"path":"3v3","amplitude_v":1.2,"offset_v":1.65}
{"cmd":"capture","samples":4096}
```

The captured array should show a sinusoidal pattern at the expected period.

---

### `stream` (removed)

Firmware up to 3.10.13 had `stream`, an alias of [`capture`](#capture--adc-snapshot) without the
trigger option, kept for older clients. Firmware 4.0.0 removed it (`BP0002`): send `capture`.

---

### `measure` — simultaneous DAC generate + ADC capture (loopback)

Generates a waveform on the DAC and captures ADC samples in one command. The iCE40 starts both together and plays exactly one waveform period across the capture window (the period is `samples` long), so the capture is phase-locked to the stimulus. Designed for loopback / round-trip measurements where the DAC output is wired (directly or through a DUT) to the ADC input.

When the capture finishes the firmware stops the DAC before it returns the data.

#### Request

```json
{"cmd":"measure","waveform":"<type>","freq_hz":<Hz>,"amplitude":<0-127>,"offset":<0-255>,"samples":<count>,"sample_rate_hz":<Hz>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `waveform` | string | yes | — | `"sine"`, `"square"`, or `"sawtooth"` |
| `freq_hz` | number (Hz) | no | `1000` | DAC output frequency |
| `amplitude` | integer 0–127 | no | `127` | DAC half-scale amplitude |
| `offset` | integer 0–255 | no | `128` | DAC DC offset |
| `samples` | integer | no | `256` | Number of ADC samples to capture, and the waveform period (max `2048`) |
| `sample_rate_hz` | number (Hz) | no | auto | Sample-clock rate. Omit it and the rate is `freq_hz × samples`, so one period spans the capture. |
| `enc` | string | no | `"b64"` | `"dec"` for decimal `data` |

#### Response

Same chunked format as `capture`. The first packet uses `"status":"ok"` and any subsequent packets use `"status":"chunk"`; the last packet has `"more":false`.

```json
{"status":"ok",    "bits":16, "data":[33012,37950,42800,...], "more":true}\n
{"status":"chunk", "data":[50511,52633,53518,...], "more":false}\n
```

#### Examples

```json
{"cmd":"measure","waveform":"sine","freq_hz":100000,"amplitude":100,"offset":128,"samples":512}
```
→ Output a 100 kHz sine and return 512 ADC samples taken during the same window.

```json
{"cmd":"measure","waveform":"square","freq_hz":50000,"samples":1024}
```
→ Output a 50 kHz square wave (defaults: amplitude 127, offset 128) and capture 1024 samples.

```json
{"cmd":"measure","waveform":"sine"}
```
→ Minimal form: 1 kHz sine, 256 samples (all defaults).

#### Error cases

| Condition | Message |
|---|---|
| `waveform` field missing | `"missing waveform"` |
| Unknown waveform | `"unknown waveform (use sine, square or sawtooth)"` |
| `samples` = 0 or > 2048 | `"samples out of range"` |
| Invalid params (for example `freq_hz` ≤ 0) | `"measure start failed"` |
| Another capture or upload is running | `BP0047` (`pod_busy`) |
| The capture did not complete | `"capture failed"` |
| DAC limits are set | the [`dac_limits`](#dac_limits--dac-output-limits-for-an-external-output-stage) refusal |

#### `measure` vs `generate` + `capture`

| | `generate` + `capture` (separate) | `measure` (combined) |
|---|---|---|
| DAC start time | First | Synchronised with ADC |
| ADC start time | Second (after `generate` response round-trip) | Synchronised with DAC |
| Skew between DAC and ADC start | Tens of ms (TCP round-trip), or none with `generate` `on_capture` | None (one gateware command) |
| DAC keeps running after | Yes (until next `generate` or duration expires) | No (stopped automatically after capture) |
| Use case | Independent generation and capture, long-running signals | Loopback / round-trip measurements, paired waveform/capture |

---

### `capture_dual` — simultaneous ADC + LA capture (one trigger)

Arm the ADC and the raw 14-channel logic analyzer off a **single** trigger and stream both regions back. The ADC samples come first, then the LA region. Either count may be `0` to capture just the other, so this one verb also serves deep ADC-only and LA-only captures. Both regions live in the 8 MB PSRAM and are read back in chunks, so each is bounded by the PSRAM, not by a RAM buffer. This is the unified capture (gateware opcode `OP_CAPTURE` = `0x31`).

The capture stays in PSRAM after the reply: if the read-back stalls, resume it with [`capture_read`](#capture_read--resume-a-capture-read-back) instead of capturing again.

#### Request

```json
{"cmd":"capture_dual","adc_samples":<N>,"adc_sample_rate_hz":<Hz>,"la_samples":<M>,"la_sample_rate_hz":<Hz>}
```

| Field | Type | Required | Default | Notes |
|---|---|---|---|---|
| `adc_samples` | integer | no | `0` | ADC 16-bit samples. |
| `adc_sample_rate_hz` | number (Hz) | no | max | ADC sample clock (24 MHz domain, divider ≥ 60 → max ~0.4 MHz). |
| `la_samples` | integer | no | `0` | Raw LA words, 2 bytes each (bit `n-1` = LA`n`). |
| `la_sample_rate_hz` | number (Hz) | no | max | LA sample clock. |
| `stop_dac_after_us` | integer | no | `0` | Gateware v21+: stop a running DAC output this many µs after t0, so the window shows it switch off. `0` leaves it running. |
| `trigger`, `trigger_timeout_ms` | object, integer | no | none | see [Capture triggers](#capture-triggers) |
| `enc` | string | no | `"b64"` | `"dec"` for the ADC region as decimal `data` |

`adc_samples` and `la_samples` must not both be `0`. On gateware v22 and newer the firmware packs
the LA region, the ADC region and any resident deep-replay waveform into the 8 MB and refuses up
front when they do not fit or when the combined sample rate would overrun the PSRAM bus (see
[tri-capture-unified-psram.md](tri-capture-unified-psram.md)).

The raw LA (12 MS/s and below) follows the same rules as [`la_capture`](#la_capture--deep-raw-logic-analyzer-capture):
without `la_sample_rate_hz` it runs at the fastest rate that fits (3 MS/s at 1 000 000 samples), and an
explicit rate that does not fit is refused with both ways out. On gateware before v53, with ADC
samples in the same capture the raw LA runs at most 8 MS/s (the PSRAM writer starved the ADC at
12 MS/s, at any depth and ADC rate): a higher raw rate is refused there. Gateware v53 and later run
the full 12 MS/s with the ADC. Above 12 MS/s the LA records changes instead. `la_sample_rate_hz`
in the reply is the rate it ran at. Each stream takes at most 2 097 152 samples (4 MB); `la_capture` takes up to 4 194 304 LA
samples. (Before firmware 3.10.0 a deep `capture_dual` ran at the requested or the full 12 MS/s
and overflowed.)

#### Response

Chunked reply: the ADC region as unsigned 16-bit counts in `data`, then the LA region as **edges**. The **first** packet also carries the **achieved** rates so the client can build an aligned time-base (the gateware floors the ADC to a divider ≥ 60, so the actual rate can differ from the request):

```json
{"status":"ok","bits":16,"adc_sample_rate_hz":399361,"la_sample_rate_hz":6000000,"data":[...],"more":true}
{"status":"chunk","data":[...],"more":true}
{"status":"chunk","la":true,"la_edges":[[0,5],[812,7],[1630,5]],"la_upto":2048,"more":true}
{"status":"chunk","la":true,"la_edges":[[4100,1]],"la_upto":4096,"more":false}
```

Digital lines are mostly static, so the LA region is sent run-length coded: each `[index, word]`
pair in `la_edges` is an LA sample index and the 16-bit word that starts there, and the word holds
until the next pair. `la_upto` is the LA index the packets so far cover. An LA-only capture's first
packet carries `"la":true,"la_edges"` directly.

| Field | Meaning |
|---|---|
| `adc_sample_rate_hz` | Achieved ADC sample rate in Hz (`24 MHz ÷ divider`); omitted when `adc_samples` = 0. |
| `la_sample_rate_hz` | Achieved LA sample rate in Hz; omitted when `la_samples` = 0. |

#### Error cases

| Condition | Message |
|---|---|
| `adc_samples` and `la_samples` both 0 | `"adc_samples and la_samples are both 0"` |
| An explicit LA rate above 8 MS/s with the ADC on gateware before v53 | `BP0068` (`la_rate_above_ceiling`, args `rate_mhz`, `max_mhz`, `fit_mhz` (the fastest rate for this depth), `samples`) |
| An explicit LA rate too fast for the depth | `BP0069` (`la_rate_too_deep`, args `rate_mhz`, `samples`, `max_samples` (the most at this rate), `fit_mhz` (the fastest rate for this depth)) |
| More than 2 097 152 samples in a stream | `BP0070` (`capture_dual_too_deep`, args `la_max`, `adc_max`, `la_capture_max`) |
| LA + ADC + a resident deep-replay waveform exceed the PSRAM (gateware v22+) | `BP0072` (`psram_too_small`) |
| The combined rate overruns the PSRAM bus (gateware v22+) | `BP0073` (`psram_bandwidth`) |
| The PSRAM could not keep up during the capture (STATUS overflow bit) | `BP0071` (`capture_overflowed`, args `la_ksps`, `adc_ksps`) |
| A count exceeds its fixed region (gateware before v22) | `"samples out of range (per-stream PSRAM region limit)"` |
| Another capture or upload is running | `BP0047` (`pod_busy`) |
| ADC samples on the digital-only board | the [digital-board refusal](#gates-that-apply-to-every-command) |
| Capture did not arm | `"capture failed"` |

---

<a id="capture_read--resume-a-capture-read-back"></a>
### `capture_read` — resume a capture read-back

The samples of the last `capture_dual` (or `la_capture`) stay in PSRAM after the reply. If the
read-back stalls (a slow cloud link), `capture_read` streams the rest from a sample index instead
of capturing again. The reply has the same chunked format as `capture_dual`, starting at `offset`
(ADC indices first, then LA indices continuing after them).

```json
{"cmd":"capture_read","offset":131072}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `offset` | integer | no | `0` | Global sample index to resume from (`0` .. `adc_samples + la_samples`). |
| `enc` | string | no | `"b64"` | `"dec"` for decimal `data` |

An `offset` equal to the total sends one empty final packet (`{"status":"ok","data":[],"more":false}`).

| Condition | Message |
|---|---|
| No `capture_dual` / `la_capture` since boot, or a newer capture started | `"no capture to resume (run capture_dual first)"` |
| Something wrote over the capture's PSRAM regions since (an OTA staging, a `load_bin` with `"psram"`, a gateware reload, a SCPI or console capture) | `"capture data was overwritten by <what>; run the capture again"` |
| `offset` past the end | `"offset out of range"` |
| Another capture or upload is running | `BP0047` (`pod_busy`) |

---

<a id="capture-triggers"></a>
### Capture triggers

**Gateware v35 or newer.** A capture can be armed and then *wait in the fabric* for a
condition on one LA pin. The producers are loaded on the arm but do not sample until the
condition is seen; that cycle becomes **t0**, so the DAC co-trigger (`generate` with
`on_capture`) and `stop_dac_after_us` count from the trigger, not from the arm.

Accepted by `capture`, `capture_dual` and `la_capture`:

```json
{"cmd":"la_capture","samples":65536,"trigger":{"la":9,"edge":"rising"},"trigger_timeout_ms":5000}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `trigger.la` | integer | yes | — | LA channel 1..14 to watch. **Any** pin, whatever its function — triggers observe, they do not claim. |
| `trigger.edge` | string | no | `"rising"` | `rising`, `falling`, `high` or `low`. |
| `trigger.pre_samples` | integer | no | `0` | `la_capture` only, gateware v50+: also return up to this many samples from before the trigger (see [`la_capture`](#la_capture--deep-raw-logic-analyzer-capture)). |
| `trigger_timeout_ms` | integer | no | `10000` | 1..600000. How long to wait before giving up. |

Omitting `trigger` (or passing `null`) captures immediately, exactly as before.

#### Reply

The final chunk of the capture reply carries the outcome:

```json
{"status":"chunk","data":[…],"trigger":{"la":9,"edge":"rising","fired":true},"more":false}
```

The firmware turns the trigger **off** in the fabric after every capture, so a following
untriggered capture is unaffected.

#### Timeout

If the condition never arrives the capture is aborted and the command is refused with
`BP0038` (`trigger_timeout`; the `edge` arg matches the request):

```json
{"status":"error","code":"BP0038","args":{"la":9,"edge":"rising","timeout_ms":10000}}
```

A co-triggered DAC start staged for that capture is cancelled with it — the abort is an
untriggered t0 and would otherwise fire the waveform.

#### Error cases

| Condition | Message |
|---|---|
| the running gateware is older than v35 | `BP0017` (`gateware_too_old`, feature `capture triggers`, min 35) |
| `trigger` is not an object | `"trigger must be an object like {\"la\":1,\"edge\":\"rising\"}"` |
| `trigger.la` outside 1..14 | `"trigger la must be 1..14"` |
| `trigger.edge` not recognised | `"trigger edge must be rising, falling, high or low"` |
| `trigger_timeout_ms` outside 1..600000 | `"trigger_timeout_ms must be 1..600000"` |
| the condition never arrived | `BP0038` (`trigger_timeout`, args `la`, `edge`, `timeout_ms`) |

The [features table](#features-table) has `capture_trigger` (and `gpio_read`) for the running
gateware that supports this.

---

### Capabilities frame — ADC calibration fields

From firmware 3.10.11 the frame also carries `features_rev`, `gateware_version` (the
running gateware) and `fpga_image` (as in `status`), so the server takes the features from the
[features table](#features-table). Firmware 4.0.0+ adds `api` and `min_client_api`, the
[client API level](#client-compatibility). Firmware 4.0.0 sends no per-feature flags in it
(`ota`, `scope`, `dac`, `dac_control_loop`, `capture_b64`, `sensors`, `lan_tls` and the rest): the
server takes the features from the table. The frame keeps the values (versions, policies,
`ws_auth_v2`, the ADC and DAC scaling and ranges, `la_max_rate_hz`, `current_out_min_ua` /
`current_out_max_ua`, `board`, the uplink, the boot health) and the facts the table names with a
`frame` key: `analog`, `nrst_pin`, `sensors_digital`, `wifi_scan`, `pod_current`, `adc_low_range`.

When the pod connects to the cloud it advertises a `capabilities` frame. Alongside `adc_bits` / `adc_fullscale_mv` / `adc_channels` it ships the affine ADC front-end calibration as **integers** (newlib-nano's `printf` has no `%f`), which the server uses to scale raw capture counts to the true probe voltage instead of the naive count ÷ full-scale:

| Field | Meaning |
|---|---|
| `adc_cal_a_uv` | Affine intercept *a* in microvolts (`V = a + b·count`). From firmware 3.9.3 it includes this pod's own offset when the pod is calibrated, and from 3.9.4 the `ext` gain when one was measured with a DMM (see [`calibrate`](#calibrate--calibrate-the-adc-offset-on-the-current_in-input-j8)), and the frame is sent again after a `calibrate`, so the server picks the new fit up without a reconnect. |
| `adc_cal_b_nv` | Affine slope *b* in nanovolts per count. |
| `adc_cal_unwrap` | `true`: the front end is bipolar, so a 16-bit count `< 32768` is unwrapped (`count += 65536`) before scaling. |
| `adc_low_range` | `true` when the board has the ADC [low range](#adc_range--the-adc-input-range-analog-module-v3r4) (analog module v3r4 and later, firmware 3.9.5+). Analog boards only. |
| `adc_range` | `"normal"` or `"low"`: the range the pod is in, the one `adc_cal_a_uv` / `adc_cal_b_nv` are for (firmware 3.9.5+). The frame is sent again on every range change, so the server scales a capture with the range it was taken in. |
| `adc_range_fs_mv` | That range's input full scale in mV (±): about 32800 normal, 5460 low (firmware 3.9.5+). |

From firmware 3.9.3 the same three fields are also in the [`status`](#status) reply on an analog
board (from 3.9.5 with `adc_range` and `adc_range_fs_mv`) (on the LAN and through cloud tunnels, not on the cloud command channel), so a LAN client
scales captures like the server does.

The frame also says which interface carries the cloud link, read from the local address of its
TCP connection (so it is the interface in use, not only the preferred one):

| Field | Meaning |
|---|---|
| `uplink` | `"eth"`, `"wifi"`, `"usb"` (through `benchpod usb-bridge`, firmware 3.9.0+), `"wifi-bridge"` (the Ethernet port goes into a connected [Wi-Fi bridge](#wifi_set--wifi_clear) box), or `"unknown"`. Older firmware leaves the field out. The server stores it with each connection-history entry. |
| `uplink_rssi_dbm` | With `"wifi-bridge"` only: the bridge's Wi-Fi signal (dBm) when the link came up. |

The frame also carries the pod's MACs (firmware 3.10.8+), `eth_mac` and `wifi_mac`, each only when
known (as in [`status`](#status)), and not in the shortened form a frame takes when its boot-health texts
are cut. The server stores them for the web's Settings tab and keeps the last ones it saw.

---

### `la_capture` — deep raw logic-analyzer capture

Capture all 14 LA channels into PSRAM and stream them back. The reply uses the LA edge format of
[`capture_dual`](#capture_dual--simultaneous-adc--la-capture-one-trigger) (`"la":true`,
`la_edges`, `la_upto`); each word holds LA1 in bit 0 up to LA14 in bit 13. Needs `la_voltage`.

```json
{"cmd":"la_capture","samples":65536,"sample_rate_hz":2000000}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `samples` | integer | no | `256` | LA samples. The limit is the PSRAM left above a resident deep-replay waveform (up to the whole 8 MB, 2 bytes per sample). |
| `sample_rate_hz` | number (Hz) | no | max (12000000) | LA sample clock, up to 48000000 on gateware v50 and later. The pod runs the closest rate at or below it that it can reach (see below); above 12 MS/s, the nearest of 48, 24 and 16. |
| `stop_dac_after_us` | integer | no | `0` | Gateware v21+: stop a running DAC output this many µs after t0. |
| `trigger`, `trigger_timeout_ms` | object, integer | no | none | see [Capture triggers](#capture-triggers) |

The first reply packet carries `la_sample_rate_hz`, the **achieved** sample rate in Hz. Use it, not the
requested rate, as the time base:

```json
{"status":"ok","bits":16,"la_sample_rate_hz":2181818,"la":true,"la_edges":[[0,256],[11,0]],"la_upto":256,"more":true}
```

The achieved rate can be lower than the request for two reasons:

- The sample period is a whole number of 24 MHz clocks, so the rate is 24 MHz ÷ *n* (*n* ≥ 2,
  so at most 12 MS/s), rounded down from the request: 2.304 MHz runs at 24/11 = 2.18 MHz.
  Request a rate of that form (12, 8, 6, 4.8, 4, 3, 2.4, 2, … MHz) to get it exactly.
- Without `sample_rate_hz` a capture larger than the 96 KB burst buffer (about 49 000 samples)
  runs at the fastest rate that fits it, so no sample is dropped: 4.8 MS/s at 100 000 samples,
  3.4 MS/s at 300 000 and 3 MS/s from about 1 000 000. An explicit rate that does not fit the
  depth is refused (firmware 3.10.0 and later; it used to be slowed down) with both ways out in
  its args: `BP0069` with `"args":{"rate_mhz":"12.0","samples":1000000,"max_samples":65536,"fit_mhz":"3.0"}`
  (firmware up to 3.10.13 sent them as text: `"LA at 12.0 MS/s does not fit 1000000 samples: at
  most 65536 samples at 12.0 MS/s, or at most 3.0 MS/s for 1000000 samples ..."`).

Older firmware omitted `la_sample_rate_hz` from this reply, so hosts labeled the trace with the
requested rate.

#### Above 12 MS/s (gateware v50 and later)

A request above 12 MS/s runs at 48, 24 or 16 MS/s, the one nearest the request (`la_sample_rate_hz`
says which). The pod records only the moments a channel changes, at 48 MS/s, and sends them as
the same `la_edges` reply, so nothing changes for the host. The capabilities report the top rate
as `la_max_rate_hz` (48000000; 12000000 on older gateware, which runs such a request at 12 MS/s).

Its limit is how often the signals change, not the number of samples: about 6 million changes per
second, all channels together, plus a buffer of about 32 000 changes for bursts. UART, I2C, CAN
and SPI with gaps between transfers stay well below it. A signal that changes faster for longer
than the buffer lasts, such as continuous SPI at 10 MHz or more for over about 1.5 ms, ends the
capture early: the reply then holds the samples up to that point, and its first packet carries
`"truncated":"busy"` (or `"memory"` when the changes filled the free capture memory). Capture at
12 MS/s or lower for such signals. `capture_dual` takes the same rates for its LA part
(`la_sample_rate_hz` above 12000000); there, a capture that stops early fails with `"capture failed"`.

#### Pre-trigger (gateware v50 and later)

With `trigger.pre_samples`, the reply also holds up to that many samples from before the trigger,
at any rate up to 48 MS/s (the pod records the changes from the arm, so this always uses the same
method as above):

```json
{"cmd":"la_capture","samples":10000,"sample_rate_hz":24000000,"trigger":{"la":3,"edge":"falling","pre_samples":2000}}
```

The first reply packet carries `"pre_samples":N`, the samples actually returned before the
trigger: fewer than asked when the capture was armed only shortly before the trigger, or when the
signals changed often before it (the pod keeps about the last 16 000 changes before the trigger).
The trigger is at sample index N; `samples` more follow it.

#### Send and capture

`send` runs one transaction right after the capture is armed, in the same command, so a trigger on
the bus line catches it (this also works over the cloud, which runs one command at a time):

```json
{"cmd":"la_capture","samples":4096,"sample_rate_hz":12000000,"trigger":{"la":1,"edge":"falling"},
 "send":{"bus":"i2c","addr":72,"write":"Dw","read":2}}
```

| `send` | Needs | Result in the last reply packet |
|---|---|---|
| `{"bus":"i2c","addr":A,"write":"<b64url>","read":N}` | an [`i2c_start`](#i2c_start--claim-two-pins-as-an-i2c-master) session; write and read up to 32 bytes | `"send":{"bus":"i2c","ack":true,"rx":"ARc"}` as `i2c_xfer` reports it (`nack_at` on a NACK) |
| `{"bus":"spi","tx":"<b64url>"}` | an [`spi_start`](#spi_start--claim-four-pins-for-spi) session, CS not held; 1..32 bytes in one CS frame | `"send":{"bus":"spi","rx":"..."}` |
| `{"bus":"gpio","la":N,"level":0\|1}` | LA N already a gpio `output` or `open_drain` | `"send":{"bus":"gpio","la":5,"level":0}` |

Trigger on the line the transaction starts with: SDA falling (I2C START), CS falling (SPI), or the
gpio pin's edge. A transfer that fails reports `"send":{"bus":"i2c","error":"SCL held low"}`, and the
capture still returns. A `send` that does not validate (no session, bad fields) is refused before
anything is armed.

| Condition | Message |
|---|---|
| LA voltage not set | `BP0001` (`la_voltage_not_set`) |
| `samples` = 0 or too deep | `BP0067` (`samples_out_of_range`, args `max`); a resident deep-DAC waveform can be what limits it (`dac_stop` frees it) |
| An explicit `sample_rate_hz` (12 MS/s and below) too fast for the depth | `BP0069` (`la_rate_too_deep`, args `rate_mhz`, `samples`, `max_samples`, `fit_mhz`) |
| Another capture or upload is running | `BP0047` (`pod_busy`) |
| The capture did not arm | `"la capture failed"` |
| A pre-trigger capture changed faster than the pod can store | `"la capture failed: the signals changed faster than the pod can store at 48.0 MS/s (about 6 million changes per second, all channels together); capture at 12 MS/s or lower"` |
| `trigger.pre_samples` on gateware older than v50 | `BP0017` (feature `trigger pre_samples`, min 50) |

---

### `load` — upload a waveform for replay

Upload a host-supplied sample buffer into the device's shared waveform buffer so it can later be played out the DAC with `replay`. This is how you replay a **previously saved** trace: capture a run, save the returned array on the host, and when you want to play it back, upload it with `load` then `replay`.

Samples are 16-bit little-endian (2 bytes each). A RAM trace plays at most 2048 samples (4096 bytes, the DAC's BRAM); for longer traces use [`load_bin`](#load_bin--raw-binary-waveform-upload) with `"psram":true`. The trace is uploaded in chunks: each chunk's `data` is the raw sample bytes encoded as **base64url** (RFC 4648 §5, no padding), at most 239 characters (about 179 bytes) per chunk. Send the first chunk with `offset:0` (this claims the shared buffer), then successive chunks at increasing byte offsets; 150 bytes per chunk is a safe size.

The buffer is held (gated, like `capture`) from the first `offset:0` chunk until a `replay` ships it to the FPGA, the connection closes, or two minutes pass without a chunk or `replay`.

#### Request

```json
{"cmd":"load","offset":<byte offset>,"data":"<base64url>"}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `offset` | integer | no | `0` | Byte offset into the buffer. `0` starts a fresh upload and claims the buffer. |
| `data` | string (base64url) | yes | — | Raw sample bytes for this chunk. |

#### Response

```json
{"status":"ok","data":{"offset":0,"len":150,"total":150}}
```

`len` is the number of bytes decoded from this chunk; `total` is the trace length so far in 16-bit samples (the value to pass as `replay`'s `samples`, or just omit it to replay everything).

#### Error cases

| Condition | Message |
|---|---|
| `data` field missing | `"missing data"` |
| Chunk at `offset > 0` sent without a preceding `offset:0` on this connection | `"load not started"` |
| Another connection holds the buffer | `BP0047` (`pod_busy`) |
| `offset` past 4096 bytes | `"offset out of range"` |
| The chunk would take the trace past 4096 bytes (2048 samples, what a RAM `replay` plays) | `"load: the trace would be longer than 4096 bytes (2048 samples, the DAC's RAM replay limit); use load_bin with \"psram\":true for longer traces"` |
| Malformed base64url | `"invalid data"` |

---

<a id="load_bin--raw-binary-waveform-upload"></a>
### `load_bin` — raw binary waveform upload

Arms a raw upload: after the reply the connection's next `total` bytes are the waveform (16-bit
little-endian samples), with no JSON or base64 around them. Then the connection returns to JSON and
`replay` plays the upload. Needs a stream connection (LAN socket or cloud byte tunnel).

```json
{"cmd":"load_bin","total":8192}
{"cmd":"load_bin","total":4000000,"psram":true}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `total` | integer | yes | — | Bytes that follow. RAM: at most 4096 (2048 samples). PSRAM: at most 8 MB. |
| `psram` | boolean | no | `false` | Stage into PSRAM for a deep replay (needs the deep-replay gateware image, feature `dac_deep_replay`). |

Replies: `{"status":"ok","data":{"ready":8192}}` right away, then `{"status":"ok","data":{"total":4096}}`
(in samples) once the bytes are in. An upload that gets no bytes for 30 s is dropped.

| Condition | Message |
|---|---|
| Sent on the cloud command channel | `"load_bin needs a stream connection"` |
| `total` missing | `"missing total"` |
| `total` 0 or too large | `"total out of range"` |
| Another capture or upload is running | `BP0047` (`pod_busy`) |

---

### `replay` — play a recorded trace out the DAC

Play the trace back out the DAC. The trace is whichever was most recently put there: the last `capture` or `measure` (so you can capture a signal and immediately play it back), a host-uploaded trace from `load` or `load_bin` (so you can replay a previously saved run), or a deep PSRAM upload from `load_bin` with `"psram":true`. The waveform loops continuously until `dac_stop` (or the next `generate`/`measure`/`replay`).

#### Request

```json
{"cmd":"replay","samples":<count>,"sample_rate_hz":<Hz>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `samples` | integer | no | full recorded length | Number of samples to play, ≤ the recorded length. |
| `sample_rate_hz` | number (Hz) | no | max | DAC sample rate. Set this to the **same** rate the trace was captured at so the playback time base matches the recording. The firmware plays the nearest achievable rate (one sample takes divider + 51 cycles of the 48 MHz DAC clock from RAM, divider + 52 from PSRAM, divider ≥ 4). The maximum, also the default, is about 873 kS/s from RAM and 857 kS/s for a deep (PSRAM) replay; a faster request is clamped to it. The reply's `sample_rate_hz` is the rate it plays at. |
| `on_capture` | boolean | no | `false` | Gateware v27+: start at the next capture's t0 (the DAC co-trigger). |

#### Response

```json
{"status":"ok","data":{"samples":2048,"cotrig":false,"sample_rate_hz":80000}}
```

`cotrig` is `true` when the start waits for the next capture. `sample_rate_hz` is the sample rate the DAC plays at (S/s, rounded): the request snapped to the nearest achievable rate, or clamped to the maximum. Older firmware leaves it out and played a replay slower than requested (it left the 51-cycle frame overhead out of the rate: 80 kS/s played at 73.7 kS/s, 400 kS/s at 281 kS/s).

#### Error cases

| Condition | Message |
|---|---|
| Nothing has been captured or uploaded yet | `"nothing to replay"` |
| `samples` = 0 or greater than the recorded length | `"samples out of range"` |
| Another connection holds the buffer | `BP0047` (`pod_busy`) |
| FPGA load/start error, or a RAM trace longer than 2048 samples | `"replay failed"` |
| DAC limits are set | the [`dac_limits`](#dac_limits--dac-output-limits-for-an-external-output-stage) refusal |

#### Record → save → replay workflow

```json
{"cmd":"capture","samples":4096,"sample_rate_hz":80000}   // record a run; host saves the returned array
{"cmd":"replay","sample_rate_hz":80000}                   // play that same run straight back out the DAC
```

To replay a run saved earlier (e.g. a "normal" trace after a "fault" trace overwrote the live buffer), upload it first:

```json
{"cmd":"load","offset":0,"data":"<base64url chunk 0>"}
{"cmd":"load","offset":150,"data":"<base64url chunk 1>"}
...
{"cmd":"replay","sample_rate_hz":80000}
```

Note that the DAC outputs the recorded samples as a **voltage** waveform — replaying a current trace reproduces its shape (e.g. for a scope or downstream stage), not the original current.

---

### `dac_stop` — stop DAC output

Halt any running DAC output: a looped `replay`, a continuous `generate` (`duration_ms` 0), or a held constant. No parameters.

#### Request

```json
{"cmd":"dac_stop"}
```

#### Response

```json
{"status":"ok","data":null}
```

With DAC limits set (below), `dac_stop` also holds the DAC at the low-output end of the limits
with the path routed, and replies `{"parked_mv":3600}`.

---

### `dac_set` — hold a raw DAC level

Holds the DAC at a fixed 8-bit level (scaled to the 16-bit DAC), with no routing and no
calibration. For a calibrated voltage use [`dac_out`](#analog-paths--analog_path--dac_out--adc_read).

```json
{"cmd":"dac_set","value":128,"divider":240}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `value` | integer 0..255 | yes | — | The level. |
| `divider` | integer | no | `2` | DAC engine clock = 48 MHz ÷ divider. |

Reply `null`. Errors: `"missing value"`, `"value out of range"`, `"dac set failed"`, and the
`dac_limits` refusal when limits are set.

---

### `dac_limits` — DAC output limits for an external output stage

For a bench with a power module between a DAC output and the DUT, such as a solar simulator.
Some of those modules are **inverted**: 0 V on the DAC is their FULL output. The limits are
stored in flash, survive reboots, and are checked on every command that can move the DAC,
whichever way it arrives (LAN, cloud, USB console). The embeddedci.com server sets them from the
device wiring (output stage) when the pod connects and when the wiring changes.

#### Request

```json
{"cmd":"dac_limits"}
{"cmd":"dac_limits","path":"5v","inverted":true,"min_mv":1850,"max_mv":3600}
{"cmd":"dac_limits","enabled":false}
```

| Field | Type | Description |
|---|---|---|
| `path` | string | DAC output the module is on: `3v3`, `5v` or `12v`. Without it the request only reads. |
| `inverted` | bool | `true` = 0 V on the DAC is the module's full output. |
| `min_mv`, `max_mv` | integer | The DAC voltages the module may be given, in mV. |
| `enabled` | bool | `false` clears the limits. |

Setting does not move the DAC; the next `dac_stop`, and every boot, parks it at the
low-output end (`max_mv` when inverted, `min_mv` otherwise).

#### Response

```json
{"status":"ok","data":{"enabled":true,"path":"5v","inverted":true,"min_mv":1850,"max_mv":3600,"park_mv":3600}}
```

#### What is refused while limits are set

| Command | Refused when |
|---|---|
| `dac_out` | `v` on the limited path is outside `min_mv`..`max_mv` |
| `dac_control_loop` | `vmin`/`vmax` (16-bit codes) fall outside the limits; `in_trip` on an inverted stage (it trips to `vmin`, the highest output there) |
| `analog_path`, `dac_out`, `adc_read` | inverted stage only: the route disconnects the limited path (`off`, another DAC path, `cal2`, `cal1` unless the path is `5v`), which leaves the module input near 0 V |
| `generate`, `dac_set`, `load`, `load_bin`, `replay`, `measure`, `dac_mux`, `cal_switch` | always: they write raw DAC codes the limits can't be checked against |
| `current_out` | always, when it has `ua`: the 4-20 mA output shares the DAC, so setting a current moves the limited path. Without `ua` it only reads the range and is allowed |

Firmware before 3.9.0 applied the same rules to the USB console's analog commands and had
`dac-limits` there; from 3.9.0 the console has no analog commands.

Between reset and the moment boot parks the DAC (after the iCE40 is up), the DAC reads 0 V.
Hold the module off with its own enable pin for that window.

---

### Analog paths — `analog_path` / `dac_out` / `adc_read`

The v2 analog front end has a DAC that can drive one of several output SMAs and an
ADC that can read one of several sources, selected by an I²C mux (U55) and four
relays (U58). Rather than flipping those by hand, address the front end by
**named path**. Each name maps to one fully-specified switch state, defined once
in firmware (`analog_path_set`), so the host never has to know the mux/relay
encoding and can't wire a path two different ways.

| Path | Meaning | ADC sees |
|---|---|---|
| `off` | Everything parked | — |
| `dac_3v3` (`3v3`) | DAC → 3.3 V output SMA | — |
| `dac_5v` (`5v`) | DAC → 5 V output SMA | — |
| `dac_12v` (`12v`) | DAC → ±12 V **bipolar** differential output SMA | — |
| `adc_ext` (`ext`, `sma`) | ADC ← front-panel input SMA | ~1 MΩ high-Z, ÷12 (low range: 1.82 MΩ, ÷2) |
| `cal1` | Internal 5 V DAC → ADC loopback | 5 V path |
| `cal2` | Internal ±12 V differential DAC → ADC loopback | ±12 V path |
| `current_in` | ADC ← 4-20 mA measurement terminal (249 Ω to ground; 100 Ω behind a 30 mA limiter from analog module v3r5, which also takes the low range by default, see [`adc_read`](#adc_read--route-a-source-and-return-a-calibrated-reading)) | J8 |
| `current_out` | DAC → 4-20 mA output terminal (J9) only: the DAC voltage outputs are switched off, the ADC relays stay as they are | unchanged |

The DAC output paths also open all ADC relays (the ADC returns to the external
SMA), so driving `dac_12v` and reading the ADC is a self-contained loopback of
the front end.

#### `analog_path` — apply a path (routing only)

```json
{"cmd":"analog_path","path":"cal1"}
→ {"status":"ok","data":{"path":"cal1","u55":3,"u58":9,"range":"normal"}}

{"cmd":"analog_path","path":"ext","range":"low"}      // analog module v3r4+
→ {"status":"ok","data":{"path":"adc_ext","u55":0,"u58":32,"range":"low"}}
```

`range` (firmware 3.9.5+, optional) also sets the [ADC input range](#adc_range--the-adc-input-range-analog-module-v3r4)
after routing; without it the range stays as it is. `off` and `cal2` put the range back to
`normal`. The reply's `range` is the range in use.

#### `dac_out` — route an output path **and** set a calibrated voltage

`v` (volts; `volts` up to firmware 3.10.13) is optional (omit to only route). The firmware converts volts to the DAC8551's full
16-bit code using the baked-in per-path calibration, so one step is about 50 µV on `3v3`,
76 µV on `5v` and 0.37 mV on `12v`.

```json
{"cmd":"dac_out","path":"5v","v":2.5}
→ {"status":"ok","data":{"path":"dac_5v","mv":2500,"uv":2499985,"code16":32723}}
```

| Field | Meaning |
|---|---|
| `mv`, `uv` | The voltage the chosen code gives, in millivolts and in microvolts |
| `code16` | The 16-bit DAC code now held |

When only routing, the reply is `{"path":…,"mv":0}` with no `uv` or `code16`. Firmware up to
3.10.13 also replied `code`, the nearest 8-bit code (`-1` when only routing).

#### `current_out` — hold a current on the 4-20 mA output (J9)

J9 is an XTR116 two-wire transmitter. It is **loop powered**: an external floating supply drives the
loop and the pod only sets how much current flows. The request and the reply are in microamps.

```json
{"cmd":"current_out","ua":12000}
→ {"status":"ok","data":{"ua":12000,"code":32576,"min_ua":4016,"max_ua":20078}}

{"cmd":"current_out"}                 // the range only, nothing moves
→ {"status":"ok","data":{"min_ua":4016,"max_ua":20078}}
```

| Field | Meaning |
|---|---|
| `ua` | Request: the current to hold. Reply: the current the nearest DAC code gives |
| `code` | The 16-bit DAC code now held (0.245 µA per code) |
| `min_ua`, `max_ua` | What the output can do: the current at code 0 and at code 65535 |

The output has a fixed 4 mA live zero, so it cannot go below `min_ua` or above `max_ua`. There is no 0 mA and no 21 mA level. A request from 4000 µA up to `min_ua`
gives `min_ua`, so "4 mA" works. Anything else outside the range is refused, not clamped:

```json
{"cmd":"current_out","ua":2000}
→ {"status":"error","message":"current_out: 2000 uA is out of range. The output can do 4016 to
   20078 uA: it cannot go below the 4 mA live zero or above the top of the DAC"}
```

Nominally the current is `100 × (4.096 V / 102 kΩ + Vdac / 25.5 kΩ)` with
`Vdac = code / 65536 × 4.096 V`: 4016 to 20078 µA, the numbers in the examples here. A real
board is off by tens of µA, so the pod converts with a fit for its board revision, like the DAC
and ADC fits:

| Revision | `min_ua` | `max_ua` | From |
|---|---|---|---|
| v2 | 4016 | 20078 | the nominal values (not measured) |
| rev3 | 4055 | 20094 | one rev3 pod against a 6½-digit DMM: a line through 25 currents from 4.1 to 20 mA (within 1 µA), and 4.055 mA at the bottom, where the DAC cannot go lower |

Always take the range from the reply (or the capabilities frame), never from this table. The
fit is per revision, not per pod: the output has no per-pod calibration.

**The DAC is shared.** The transmitter follows the DAC on every analog path, and the 3.3 V, 5 V
and ±12 V outputs use the same DAC:

- `current_out` with `ua` switches the DAC voltage outputs off first (path `current_out`), so
  they do not follow the current. The ADC relays are not touched: a `current_in` or `ext`
  reading keeps working.
- A voltage output (`dac_out`, `generate`, `replay`, the control loop) also moves the loop
  current: about `4.02 mA + 0.0627 mA × code` for the 8-bit `code` that `dac_out` returns.
  Path `off` does not park the loop: DAC code 0 is `min_ua`.
- `dac_stop` stops a waveform but leaves the DAC where it was. Send `{"cmd":"current_out","ua":4000}`
  to go back to 4 mA.
- To play a waveform as a current, set a current first (that switches the voltage outputs off),
  then use `generate` or `load_bin` + `replay` with codes computed from `min_ua` / `max_ua`:
  `ua = min_ua + code16 × (max_ua − min_ua) / 65535`, or `generate` with `amplitude_ua` and
  `offset_ua`.
- While [DAC limits](#dac_limits--dac-output-limits-for-an-external-output-stage) are set,
  `current_out` with `ua` is refused.

**Wiring.** The transmitter does not source current. An external supply powers the loop:

```
supply +  →  J9 pin 1 (plus)
J9 pin 2 (minus)  →  receiver  →  supply −
```

The supply needs at least 8 V plus 20 mA times the receiver resistance (13 V for 249 Ω), and
at most 30 V. Pin 1 has a series diode, so a reversed supply gives no current and no damage.

**The loop supply must float.** J9 pin 2 is not ground: it sits up to 0.5 V below pod ground,
across the transmitter's internal sense resistor. That resistor is how the current is
regulated, so nothing else in the loop may connect to pod ground.

- Use a supply whose output is isolated from pod ground and from earth. The pod's own 5 V and
  0-20 V outputs cannot power the loop.
- Do not connect supply minus, or the low side of the receiver, to pod ground.
- J9 cannot be wired to the pod's own measurement terminal J8, because J8 is a resistor to pod ground (249 Ω, or 100 Ω from analog module v3r5).
- A target whose loop input is referenced to a ground it shares with the pod (through SWD or
  UART wiring) cannot be driven either. Use an isolated loop input on the target.
- If the loop does touch pod ground, the current bypasses the sense resistor, is no longer
  regulated, and sits near the transmitter's 32 mA limit regardless of the DAC.

To measure the loop current with the pod, put a resistor between J9 pin 2 and supply minus and
connect the ADC input SMA center to supply minus, with the SMA shell left open. `adc_read ext`
then reads a negative voltage, `-(R + 25 Ω) × I`. `hwe2e/benchpod_current_loop_hw_test.go` in
embeddedci-server does this.

The pod cannot see the loop: with no supply or an open loop, `current_out` still replies ok.

The [features table](#features-table) has `current_out` when the firmware supports it. The cloud
capabilities frame has the range as `current_out_min_ua` / `current_out_max_ua`.

#### `adc_read` — route a source **and** return a calibrated reading

`source` is `ext` (default) / `cal1` / `cal2` / `current_in`. The firmware routes the
source, waits ~20 ms for the relays to settle, averages a short 16-sample burst,
applies that source's ADC calibration, and returns millivolts plus the raw 16-bit
count and the burst's peak-to-peak spread in counts (`span`). The front-SMA input
is high-impedance (~1 MΩ); its ÷12 divider is in the shared analog front end and
already baked into the calibration, so `mv` is the true voltage at the input SMA
with no further scaling.

The shared front end is **inverting**, which puts 0 V at count ≈ 65538 — just past
the 16-bit top. A near-zero or negative input therefore wraps to a small count and
is **unwrapped** (`count += 65536`) before the fit is applied. That unwrap belongs
to the front end, so it happens for **every** source, and the burst is averaged on
the 16-bit circle *before* the unwrap — every source's quiet operating point sits
within a few counts of the wrap, so averaging raw counts first turns a burst that
straddles it into a plausible-looking reading wrong by tens of volts.

If the burst is wider than ~1024 counts the input is still **moving** (a DAC left
driving the node by a preceding `measure`/`generate`, say) and has no single
voltage, so `adc_read` returns an **error** naming the span rather than averaging
it. Stop the DAC (`dac_stop`) or let the node settle and read again.

`range` (firmware 3.9.5+, optional): `normal` or `low` switches the
[ADC input range](#adc_range--the-adc-input-range-analog-module-v3r4) before the reading
(`low` needs analog module v3r4+ and is refused for `cal2`). Without it the reading is taken in the
range the pod is in. From 3.9.5 the reply also says which:

| Field | Meaning |
|---|---|
| `range` | `normal` or `low`: the range the reading was taken in, and scaled with |
| `fullscale_mv` | That range's input full scale (±), from this pod's fit: about 32800 normal, 5460 low |
| `overrange` | `true` when a sample of the burst sat on one of the ADC's rails (code 0x7FFF or 0x8000): the input is past `fullscale_mv` and `mv` is only a bound. Use the normal range |

From firmware 3.10.7 the reply also has `uv`: the same reading in microvolts. `mv` is rounded to
a whole millivolt, which hides most of what the low range resolves (about 167 µV per count, and the
16-sample average goes below one count); use `uv` when you need it.

```json
{"cmd":"adc_read","source":"ext"}
→ {"status":"ok","data":{"source":"adc_ext","mv":11980,"uv":11980214,"count":63037,"span":4,"range":"normal",
   "fullscale_mv":32856,"overrange":false}}

{"cmd":"adc_read","source":"ext","range":"low"}       // ~167 uV per count instead of ~1 mV
→ {"status":"ok","data":{"source":"adc_ext","mv":1203,"uv":1202736,"count":58322,"span":9,"range":"low",
   "fullscale_mv":5462,"overrange":false}}

{"cmd":"adc_read","source":"cal1"}   // DAC still playing a waveform into the loopback
→ {"status":"error","message":"adc_read: input not settled on cal1 (51811 counts pk-pk
   over the 16-sample burst, limit 1024). Stop the DAC (dac_stop) or let the node settle"}
```

`current_in` is the 4-20 mA measurement terminal (J8). Its reply has two more fields:

- `ua`: the loop current in µA. The pod reads the voltage across its sense resistor, so
  `ua = uv × 1000 / sense_mohm`. On analog modules up to v3r4 that is 249 Ω: 4 mA is 996 mV
  and 20 mA is 4980 mV. From analog module v3r5 it is 100 Ω: 4 mA is 400 mV and 20 mA is 2 V.
- `sense_mohm` (firmware 3.10.13+): the resistance `ua` was computed with, in milliohms.
  249000 up to module v3r4. From v3r5 it is 100 Ω in parallel with the ADC input divider, whose
  current also comes from the loop: 99990 in the normal range, 99995 in the low range.
- `offset_mv`: this pod's calibration offset, which is already taken out of `mv` and `ua`
  (see [`calibrate`](#calibrate--calibrate-the-adc-offset-on-the-current_in-input-j8)). It is 0 on a pod that was
  never calibrated.

`count` is always raw.

**From analog module v3r5** (firmware 3.10.13+) the 4-20 mA input changes. No v3r5 module is built yet: the v3r5 values in this table and the next paragraph are calculated from the design and the part datasheets, not measured.

| | Module up to v3r4 | Module v3r5 and later |
|---|---|---|
| Sense resistor | 249 Ω | 100 Ω, 0.1 % |
| Protection | none in series | series diode and a 30 mA current limiter |
| Voltage the pod takes from the loop at 20 mA | about 5 V | about 4.3 V |
| Sense voltage at 4 / 20 mA | 0.996 / 4.98 V | 0.4 / 2.0 V |
| Range `current_in` reads in by default | the range in use (`normal` unless set) | `low` |
| Resolution | about 4 µA per count (`normal`) | about 1.7 µA per count (`low`), about 10 µA (`normal`) |
| Measurable current | 0 to about 21 mA (`low`), more in `normal` | 0 to the limiter's 28.5 to 31.5 mA |

The limiter's datasheet range is 28.5 to 31.5 mA (30 mA nominal, it differs per part), so by
design a supply wired straight to J8 cannot overload the resistor. A reading near 30 mA that does
not follow the loop is the limiter, not the transmitter. The diode blocks a reversed loop, so it
should read 0. The ADC reads at the resistor, after the diode and the limiter, so their drop does not enter
the reading.

On a v3r5 module, routing `current_in` without a `range` (here, in `analog_path` and in a
capture) switches the ADC to the [low range](#adc_range--the-adc-input-range-analog-module-v3r4)
by itself: 31.5 mA is 3.15 V, well inside its ±5.46 V. The next path that moves the relays (any
source or output but `current_out`) puts the range back to normal, so the ±12 V output is not
held to ±5 V afterwards. Ask for `"range":"normal"` to read `current_in` in the normal range, or
`"range":"low"` to keep the low range after it. The module revision comes from the module's
ID EEPROM (see [`module`](#module--the-analog-module-and-its-eeprom)), read at boot.

```json
{"cmd":"adc_read","source":"current_in"}
→ {"status":"ok","data":{"source":"current_in","mv":3985,"uv":3985402,"count":61562,"span":6,"range":"normal",
   "fullscale_mv":32856,"overrange":false,"offset_mv":10,"ua":16004,"sense_mohm":249000}}

{"cmd":"adc_read","source":"current_in"}   // analog module v3r5: the low range by default
→ {"status":"ok","data":{"source":"current_in","mv":1600,"uv":1600212,"count":55938,"span":5,"range":"low",
   "fullscale_mv":5462,"overrange":false,"offset_mv":10,"ua":16003,"sense_mohm":99995}}
```

Firmware up to 3.3.0 used a different name for this source. That name is no longer accepted.

#### `adc_range` — the ADC input range (analog module v3r4)

Analog module v3r4 adds a relay (K5) that swaps the bottom leg of the ADC input divider:

| Range | Divider | Full scale | Resolution | Input impedance |
|---|---|---|---|---|
| `normal` (default) | 910k / 82.5k, about ÷12 | ±32.8 V | ~1 mV per count | ~1 MΩ |
| `low` | 910k / 910k, ÷2 | ±5.46 V | ~167 µV per count | 1.82 MΩ |

The ADC has a gap of about 3 counts at its zero crossing: a signal crossing 0 V jumps by that much, and one resting at 0 V can flip across it, which looks like extra noise. Readings away from 0 V are not affected, and calibration does not remove it.

The low range gives about 6x the resolution on signals up to ±5 V. **Use 1x leads**: its
1.82 MΩ input does not match a 10x scope probe, which expects 1 MΩ.

The divider is in front of every ADC source, so the range applies to `ext`, `cal1` and
`current_in` (4-20 mA is 0.996 to 4.98 V, inside the low range; 0.4 to 2 V from module v3r5, which
reads `current_in` in the low range by default). `cal2` (±12 V) has no low range.
The range stays until it is changed, `analog_path` `off` or `cal2` is routed, or the pod restarts;
each of those puts it back to `normal`. Older boards (and modules before v3r4, whose EEPROM says a
revision below 3.4) have no K5: the range is always `normal` there and `low` is refused. The pod
reads the module revision at boot, so restart it after programming a module's EEPROM.

```json
{"cmd":"adc_range"}
→ {"status":"ok","data":{"range":"normal","supported":true,"fullscale_mv":32856,
   "normal_fullscale_mv":32856,"low_fullscale_mv":5462,"adc_cal_a_uv":65708158,"adc_cal_b_nv":-1002644}}

{"cmd":"adc_range","range":"low"}
→ {"status":"ok","data":{"range":"low","supported":true,"fullscale_mv":5462,
   "normal_fullscale_mv":32856,"low_fullscale_mv":5462,"adc_cal_a_uv":10923784,"adc_cal_b_nv":-166687}}

{"cmd":"adc_range","range":"low"}          // a board without K5
→ {"status":"error","message":"range: the low range needs analog module v3r4 or later"}
```

| Field | Meaning |
|---|---|
| `range` | The range in use |
| `supported` | The board has the low range |
| `fullscale_mv` | The full scale (±) of the range in use, from this pod's `ext` fit |
| `normal_fullscale_mv`, `low_fullscale_mv` | Both ranges' full scale (`low_fullscale_mv` only when supported) |
| `adc_cal_a_uv`, `adc_cal_b_nv` | The `ext` fit of the range in use, as in the [capabilities frame](#capabilities-frame--adc-calibration-fields): scale captures with it from now on |

Switching waits 10 ms for the relay (1.4 ms operate, 1.3 ms release) and the divider to settle
before it replies, so a capture sent next sees the new range. Captures are raw counts: the
capabilities frame and `status` carry the fit of the range in use (`adc_cal_a_uv`,
`adc_cal_b_nv`, `adc_range`), and the frame is sent again on every change. A sample on an ADC rail
(code 0x7FFF or 0x8000) is past the range's full scale. Set the range before starting the control
loop: its input map uses the fit of the range in use when it starts.

**Calibration.** The low range is the normal fit times the divider ratio
(82.5k/992.5k ÷ 0.5 ≈ 0.1662), offset included: the offset arises after the divider, so the
`calibrate` zero (J8 open) serves both ranges. The resistors' tolerance (1 %) leaves the
uncalibrated low range within a few percent. `{"cmd":"calibrate","adc":"cal1","range":"low"}`
measures this pod's low-range gain through the `cal1` loopback (the 5 V output at 1 V and 4 V, inside
the ±5.46 V range) and stores it; one gain serves every source, since K5 changes the same divider
for all of them. It needs what `adc` `cal1` needs (the offset and the 5 V output's DMM line), and
refuses a gain more than 5 % off. See [`calibrate`](#calibrate--calibrate-the-adc-offset-on-the-current_in-input-j8).

**Self-test.** `{"cmd":"adc_range","test":true}` drives the 5 V output into the `cal1` loopback at
1 V and 3.5 V (under 4 V, inside the low range) and reads each point in both ranges, each with its
own fit. A point passes when both readings agree with the set point and each other within 5 % +
30 mV: an uncalibrated low range passes, a relay that does not switch (6x off) does not. Takes
about 0.5 s, leaves the outputs off and the range `normal`, and is refused while
[DAC output limits](#dac_limits--dac-output-limits-for-an-external-output-stage) are set.

```json
{"cmd":"adc_range","test":true}
→ {"status":"ok","data":{"points":[{"set_mv":1000,"normal_mv":1001,"low_mv":999,"ok":true},
   {"set_mv":3500,"normal_mv":3502,"low_mv":3497,"ok":true}],"pass":true,"range":"normal"}}
```

**The ±12 V output in the low range** (firmware 3.10.7+). The ±12 V output reaches the ADC
through the front SMA when the two are cabled together (the `calibrate` `adc` `ext` setup), and
the low range reads only ±5.46 V. So while the ADC is in the low range, the ±12 V output stays
within ±5 V:

- `dac_out` on `12v` with more than 5 V either way is refused, as is any held level past it
  (an analog sensor's value, SCPI). ±5 V itself is allowed.
- A waveform, replay, raw code or control loop on the ±12 V output (`generate` on `12v` or with
  `"route":false` while it is routed, `dac_set`, `load`, `load_bin`, `replay`, `measure`,
  `dac_control_loop`) is refused: its samples are not checked one by one.
- Switching to the low range (`adc_range`, or `range` on `analog_path` and `adc_read`) is refused
  while the ±12 V output holds more than ±5 V, or after a waveform or loop played on it. Route
  `off` or hold a level within ±5 V first.

The other outputs are not affected (the 5 V output tops out at about 4.98 V), and neither is the
normal range: there the ±12 V output keeps its full range. Calibration is not affected: the low
range is calibrated through `cal1` (the 5 V output), and the `cal2` and `ext` steps run in the
normal range.

```json
{"cmd":"adc_range","range":"low"}
{"cmd":"dac_out","path":"12v","v":6}
→ {"status":"error","message":"refused: the ADC is in the low range (+-5.46 V full scale), and 6000 mV on the 12v output is past +-5000 mV. Switch the ADC to the normal range first: {\"cmd\":\"adc_range\",\"range\":\"normal\"}"}
```

The `adc_low_range` fact is in `hw` on a board with the low range, and `status` (LAN and
tunnels) has `adc_range` and `adc_range_fs_mv`. `cal_switch` reports K5 as `low_range`.

#### `calibrate` — calibrate the ADC offset on the `current_in` input (J8)

`current_in` is the 4-20 mA terminal J8: pin 1 goes through a 249 Ω resistor to pod ground and
pin 2 is ground. The pod reads the voltage across that resistor, so 4 mA is 996 mV and
20 mA is 4980 mV. From analog module v3r5 the resistor is 100 Ω behind a series diode and a
30 mA current limiter: 4 mA is 400 mV and 20 mA is 2 V (see [`adc_read`](#adc_read--route-a-source-and-return-a-calibrated-reading)). The
calibration is the same on both: the offset is measured with J8 open, at 0 V.

Every ADC source is scaled with a fit, `volts = a + b × count`. The fits are built into
the firmware and are the same on every pod. Each pod still has its own offset, from part
tolerances in the ADC front end: measured from +13 mV to -50 mV at the input on pods of one
batch. `calibrate` measures that offset on `current_in` and stores it on the pod.

All four ADC sources (`ext`, `current_in`, `cal1`, `cal2`) share the same front end after the
input relays, so the offset is the same for all of them. From firmware 3.9.3 the pod takes it
out of every source's fit: `adc_read`, captures (through `adc_cal_a_uv` in `status` and the
capabilities frame) and the control loop's input map. Older firmware applies it to
`current_in` only.

Disconnect J8 first. With nothing connected the terminal is at 0 V, so the pod needs no
reference.

| Request | What it does |
|---|---|
| `{"cmd":"calibrate","source":"current_in"}` | Calibrate `current_in` with J8 disconnected, store the result, return it |
| `{"cmd":"calibrate"}` | Return the stored calibration |
| `{"cmd":"calibrate","clear":true}` | Remove it and go back to the built-in fits (from 3.9.4 also the DMM parts below) |
| `{"cmd":"calibrate","clear":"offset"}` | Remove one part only: `offset`, `dac` or `adc` (firmware 3.9.4+; from 3.9.5 `adc` also clears the low-range gain) |
| `{"cmd":"calibrate","adc":"cal1","range":"low"}` | The [low range](#adc_range--the-adc-input-range-analog-module-v3r4)'s gain through the `cal1` loopback (analog module v3r4+, firmware 3.9.5+) |

```json
{"cmd":"calibrate","source":"current_in"}
→ {"status":"ok","data":{"source":"current_in","calibrated":true,"offset_mv":4,"offset_uv":4356,
   "a_uv":65828041,"b_nv":-1004471,"count":65535,"span":6,"samples":512}}

{"cmd":"calibrate"}
→ {"status":"ok","data":{"source":"current_in","calibrated":true,"offset_mv":4,"offset_uv":4356,
   "a_uv":65828041,"b_nv":-1004471}}

{"cmd":"calibrate","clear":true}
→ {"status":"ok","data":{"source":"current_in","calibrated":false,"offset_mv":0,"offset_uv":0,
   "a_uv":65832397,"b_nv":-1004471}}
```

| Field | Meaning |
|---|---|
| `calibrated` | `true` when this pod has its own calibration stored |
| `offset_mv` | The offset in mV, rounded. `adc_read` reports the same value |
| `offset_uv` | The offset in µV, as stored |
| `a_uv`, `b_nv` | The fit the pod now uses for `current_in`: `a` in µV and `b` in nV per count, the same units as `adc_cal_a_uv` / `adc_cal_b_nv` in the [capabilities frame](#capabilities-frame--adc-calibration-fields). Unwrap the count first, as for every source |
| `sense_mohm` | The resistance a `current_in` voltage is turned into the loop current with, in milliohms, for the range in use: 249000 up to analog module v3r4, from v3r5 99990 (normal) or 99995 (low). Loop current = volts × 1000 / `sense_mohm` (firmware 3.10.13+) |
| `count`, `span` | Raw mean count and peak-to-peak spread of the measurement (calibrate only) |
| `samples` | Samples averaged (calibrate only) |
| `dac`, `adc_gain_ppm`, `dmm` | The DMM-guided parts (firmware 3.9.4+), see [below](#dmm-guided-calibration-firmware-394) |
| `adc_comp` | The recorded frequency response of the ADC input (firmware 3.9.5+), `hf_dc_e4` and `c_top_ff`, or `null`. See [below](#adc-frequency-response-record-firmware-395) |
| `stored` | Where the calibration in use is kept (firmware 3.10.1+): `"module"` (also in the analog module's EEPROM, digital v3r3, see [`module`](#module--the-analog-module-and-its-eeprom)), `"pod"` (only the pod's flash) or `"none"` |

The measurement averages 32 bursts of 16 samples, the same burst `adc_read` takes. It
takes about 0.3 s and leaves the `current_in` path routed.

A reading outside ±150 mV is refused, because it means something is driving J8 (older
firmware: ±50 mV). The old calibration stays in place.

```json
{"cmd":"calibrate","source":"current_in"}   // a 4 mA loop is still connected
→ {"status":"error","message":"calibrate: current_in reads 996 mV, it must be within +/-150 mV of zero.
   Something is driving J8. Disconnect it and try again"}
```

The calibration is stored in flash. It survives a reboot and a firmware update. Only the
offset is calibrated: the gain uses the built-in fits (within 0.3 % on measured pods), because
calibrating it needs a reference voltage (see the DMM-guided calibration below). On `cal1` and
`cal2` the DAC output stages add a few mV of their own on top. The [features table](#features-table) has `calibrate`
when the firmware supports it.

##### DMM-guided calibration (firmware 3.9.4)

With a bench DMM the pod can also take out what is left after the offset: its own DAC offset and
gain per output path, and then the ADC gain per source. Every step is optional and saved on its
own; a path or source without one keeps the built-in fit. The [features table](#features-table) has `cal_dmm` when
the firmware takes these forms.

| Request | What it does |
|---|---|
| `{"cmd":"calibrate","dac":"5v","code1":13069,"uv1":995900,"code2":52374,"uv2":3992900}` | Store this pod's line for a DAC output path (`3v3`, `5v`, `12v`) from two points: the 16-bit code the pod drove and what the DMM read on that output's SMA, in µV |
| `{"cmd":"calibrate","adc":"cal1"}` | Measure and store an ADC source's gain (`cal1`, `cal2`, `ext`) through a loopback |

**DAC lines.** Drive each set point with [`dac_out`](#dac_out--route-an-output-path-and-set-a-calibrated-voltage)
and keep the `code16` of its reply, read the DMM, then send both points. Use points well inside
the range and away from the output floor near 0 V: 0.5 and 3.0 V on `3v3`, 1.0 and 4.0 V on `5v`,
-5 and +5 V on `12v`. Disconnect the DUT from the output first. From then on every DAC user
converts with this pod's line: `dac_out`, the calibrated `generate`, the DAC output limits. The
4-20 mA output keeps its own fit, and commands that take raw codes are not converted.

The pod refuses (and keeps the old line) when the codes are less than 16384 apart, when a reading
is more than 100 mV + 2 % from what the built-in line gives at that code (the DMM is on another
output or not on DC volts), or when the two readings give a gain more than 2 % off the built-in one.

```json
{"cmd":"calibrate","dac":"5v","code1":13069,"uv1":995900,"code2":52374,"uv2":12000}
→ {"status":"error","message":"calibrate: the DMM reads 12 mV where the 5v output gives about 4000 mV
   at code 52374. Is the DMM on the 5v SMA, set to DC volts? Nothing was stored"}
```

**ADC gains.** Once an output path has its DMM line it is a known source. The pod drives two set
points through a loopback and compares what the ADC reads (offset removed) with what the output
now puts out: `cal1` through the 5 V output (1 and 4 V), `cal2` through the ±12 V output (-5 and
+5 V), and `ext`, the front SMA, through an SMA cable from the ±12 V output to the ADC input (-5
and +5 V). `current_in` is scaled like `cal1` and follows its gain. A source then reads
`volts = g × (a + b × count - offset)`: 0 V stays 0 V, and the gain does not depend on the
offset, so either can be redone alone. The measurement takes about 0.7 s and switches the outputs
off afterwards. It needs the ADC offset calibrated and the driving path's DMM line, and refuses a
gain more than 2 % off (for `ext`: is the cable connected?). The capabilities frame is sent again
after an `ext` gain, so captures scale with it.

Both forms are refused while [DAC output limits](#dac_limits--dac-output-limits-for-an-external-output-stage)
are set: clear them, calibrate, set them again. The J8 offset does not move the DAC and stays allowed.

```json
{"cmd":"calibrate","adc":"cal1"}
→ {"status":"ok","data":{"source":"current_in","calibrated":true,"offset_mv":-50,"offset_uv":-50210,
   "a_uv":65758368,"b_nv":-1002716,
   "dac":{"3v3":null,"5v":{"a_uv":-609,"b_pv":76249841,"offset_uv":-3127,"gain_ppm":-996},"12v":null},
   "adc_gain_ppm":{"cal1":1850,"cal2":null,"ext":null},"dmm":true,
   "measured":{"v1_uv":995900,"v2_uv":3992900,"r1_uv":994100,"r2_uv":3985500,"residual_uv":1200}}}
```

| Field | Meaning |
|---|---|
| `dac.<path>` | This pod's line, `volts = a + b × code16` with `a_uv` in µV and `b_pv` in pV per 16-bit code; against the built-in line its `offset_uv` (at 0 V on `12v`, at code 0 on the others) and `gain_ppm`. `null`: the built-in line |
| `adc_gain_ppm.<source>` | The gain correction in ppm. `null`: the built-in gain |
| `range` | On a board with the [low range](#adc_range--the-adc-input-range-analog-module-v3r4) (firmware 3.9.5+): the range `a_uv` / `b_nv` are for, the range in use |
| `adc_low_gain_ppm` | The [low range](#adc_range--the-adc-input-range-analog-module-v3r4)'s gain correction in ppm, against the divider's nominal ratio; `null`: not measured. Only on a board with the low range (firmware 3.9.5+) |
| `dmm` | `true` when any DMM part is stored |
| `zero_count_mc` | The raw zero the offset is derived from: the mean count `current_in` read with J8 open, unwrapped, in thousandths of a count (firmware 3.9.4+, only when calibrated) |
| `measured` | After an `adc` run: what the output put out (`v1_uv`, `v2_uv`), what the ADC read before the gain (`r1_uv`, `r2_uv`) and the mean offset left after it (`residual_uv`) |

##### ADC frequency response record (firmware 3.9.5+)

```json
{"cmd":"calibrate","comp_hf_dc_e4":9678,"comp_c_top_ff":9360}
```

Records what a frequency sweep through the `cal1` loopback measured on this pod: the high-frequency
gain of the ADC input divider against its DC gain, times 10000 (`comp_hf_dc_e4`, 5000 to 15000), and
the fitted capacitance of the divider's top leg in femtofarads (`comp_c_top_ff`, 1000 to 65000).
Both go together. It is a record only: **no reading is scaled with it**, and DC accuracy does not
depend on it. Out of range is refused. `{"cmd":"calibrate","clear":"adc"}` and `"clear":true` remove it.

**What the pod stores** (firmware 3.9.4+): raw measurements, never a fit. The mean count with J8
open (the zero), the two codes and DMM readings per DAC output, and the two counts and input
voltages per ADC source. Every fit is derived from them at boot, with the general fits built into
that firmware: the offset is the general `cal1` fit at the zero count, a DAC line goes through its
two points, and a gain is the slope of its two points over the general fit's slope. So a general
fit measured again in a later firmware never changes what a stored value means, and J8 open keeps
reading 0 V.

A pod updated from firmware 3.4.0 to 3.9.3 keeps its offset: that firmware stored it in µV on the
general fit of its day, which the pod converts to the zero count once, exactly. Every save also
writes a copy in the old format (the offset in µV) that older firmware reads, so going back to
firmware 3.4.0 to 3.9.3 keeps the offset (the DMM parts are not used there). What the older
firmware then calibrates or clears is what a later update finds.


#### Loopback / self-cal examples

```json
// Self-cal the 5 V path (no external wiring): drive 2.5 V, read it back.
{"cmd":"dac_out","path":"5v","v":2.5}
{"cmd":"adc_read","source":"cal1"}          // → ~2500 mV

// Measure an external signal on the ADC input SMA (high-Z ÷12 front end):
{"cmd":"adc_read","source":"ext"}
```

The low-level `dac_mux` (U55 mux: `ctrl1_sel` 0=3V3/1=5V/2=12V/3=12V_ADC,
`ctrl2_sel` 0=12V_VMID/1=ADC_VMID/2=GND) and `cal_switch` (U58 relays: `cal1`,
`cal2`, `current_in`, `cal_path`; `cal1`+`cal2` are mutually exclusive) commands
remain available for diagnostics, but prefer the named paths above.

---

### `test` — synthetic diagnostic pattern

Builds a known 16-bit pattern in the STM32 (the FPGA, ADC and analog front end are not touched) and returns it in the same chunked format as `capture`. Useful for verifying TCP, JSON, chunking and base64 decoding on their own.

#### Request

```json
{"cmd":"test","pattern":"<sine|counter|ramp|const>","value":<0-65535>,"samples":<count>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `pattern` | string | no | `sine` (or `const` if `value` is given) | `"sine"`, `"counter"`, `"ramp"`, or `"const"` |
| `value` | integer 0-65535 | no | `65535` (used only for `const`) | Constant sample value |
| `samples` | integer | no | `256` | 1 to `4096` |
| `enc` | string | no | `"b64"` | `"dec"` for decimal `data` |

#### Patterns

| `pattern` | Output | Visual character |
|---|---|---|
| `sine` | One full sine period across `samples`, centred at 32768, amplitude 25000 | Smooth curve, easy to spot in a plot |
| `counter` | `0,1,2,…` (the sample index) | Each sample is unique, so dropped or reordered samples are easy to spot |
| `ramp` | Linear `0 → 65535` across all samples | Monotonic linear ramp |
| `const` | Every sample = `value` | Constant: verifies chunking and value flow |

#### Response

Same chunked format as `capture`: the first packet uses `"status":"ok"` (with `"bits":16`), the ones after it `"status":"chunk"`, and the last has `"more":false`.

#### Examples

```json
{"cmd":"test"}
```
→ 256-sample sine wave (default).

```json
{"cmd":"test","pattern":"counter","samples":1024}
```
→ 1024 samples `0,1,2,…,1023`.

```json
{"cmd":"test","value":65535}
```
→ 256 samples of `65535`.

#### Error cases

| Condition | Message |
|---|---|
| `samples` = 0 or > 4096 | `"samples out of range"` |
| Unknown `pattern` | `"unknown pattern (use sine\|counter\|ramp\|const)"` |
| Another capture or upload is running | `BP0047` (`pod_busy`) |

---

### `status` — firmware and connection info

Returns the firmware version, network and cloud state, board and gateware details, health
counters, and what a client needs to look the pod's features up in the
[features table](#features-table): `features_rev`, `fpga_image` and the hardware facts `hw`.

#### Request

```json
{"cmd":"status"}
```

No parameters. Firmware 3.10.11 to 3.10.13 took `"caps":false` to leave out the `caps` list
older clients read; firmware 4.0.0 has no such list and ignores `caps`, so a client may keep
sending it.

#### Response (abridged)

```json
{"status":"ok","data":{"device":"benchpod","version":"3.7.0","board":"stm32h563","net":"ready",
 "ip":"192.168.1.220","rssi_dbm":null,"wifi":"connected","cloud":"connected",
 "usb":{"net":true,"enabled":true,"link":true,"ip":"10.237.8.85","host_ip":"10.237.8.86","source":"default","source_ma":500},
 "adc_bits":16,"adc_fullscale_mv":4096,"adc_channels":1,"la_vccio_mv":3300,
 "adc_cal_a_uv":65758736,"adc_cal_b_nv":-1002644,"adc_cal_unwrap":true,
 "board_rev":"v3","nrst_pin":true,"flash_kb":2048,
 "sig_policy":"required","sig_keys":3,"lan_policy":"open",
 "lease":{"held":false,"holder":"","left_s":0},"gateware":47,"gateware_embedded":47,
 "psram":"ok","psram_ok":true,"reset":"power-on","last_crash":"","analog":true,
 "safe_mode":false,"safe_reason":"","features_rev":8,"api":2,"min_client_api":2,
 "fpga_image":"loop","hw":["wifi","usb_cc"]}}
```

#### Short form

The cloud command channel caps a reply's size (1791 bytes). From firmware 3.10.5 `status` sent
there is the short form, marked `"short_form":true`. It leaves out what a cloud client reads
elsewhere:

| Left out | Where to read it |
|---|---|
| `lan_tls` | the [`lan_tls`](#lan_tls--the-lan-api-over-tls) command |
| `lease` | the server, which owns the cloud lease |
| `local_lease` | the server, from the pod's `lease.local` frames (see [Pod lease](#pod-lease)) |
| `module` | the [`module`](#module--the-analog-module-and-its-eeprom) command |
| `adc_cal_a_uv`, `adc_cal_b_nv`, `adc_cal_unwrap`, `adc_range`, `adc_range_fs_mv` | the [capabilities frame](#capabilities-frame--adc-calibration-fields) |
| `stacks`, `lwip_mem_max`, `lwip_mem_size`, `pbuf_pool_max`, `pbuf_pool_size`, `malloc_failures`, `rx_dropped`, `tx_dropped`, `board_rev_mv` | [`diag`](#diag--heap-stack-and-network-diagnostics) (firmware 3.10.9+; earlier, the full form) |

It also cuts `last_crash` and `safe_reason` to 48 characters, writing any character JSON would
escape as `?`, and adds `"boot_health_cut":true` when it cut one; the capabilities frame has both
in full. Everything else, the facts and `usb` included, is the same as the full form.

LAN, USB and the cloud byte tunnels get the full form. A full form larger than the connection
takes in one piece (2047 bytes, for example with a long crash report) comes as the short form
too, so a LAN client that reads the calibration from `status` checks
`short_form` and reads them again (or warns) when it is set. Firmware 3.10.9 moved the
diagnostics to [`diag`](#diag--heap-stack-and-network-diagnostics) so a running pod's full form
stays a few hundred bytes under that limit; instead of the error `status too large for this connection` that firmware up to 3.10.4 sent.
Firmware 3.10.4's full `status` could be larger than the cloud channel takes (with the USB network
adapter up and a lease held), so connecting to it over the cloud failed with
`BP0054` (`cloud_reply_too_large`).

#### Response fields

| Field | Type | Description |
|---|---|---|
| `device` | string | Always `"benchpod"`. |
| `version` | string | Firmware version. |
| `board` | string | Board name this firmware was built for. |
| `net` | string | `"ready"` when the Ethernet or Wi-Fi interface has an address, else `"disconnected"`. |
| `ip` | string | The pod's IP (Ethernet preferred over Wi-Fi), or `"0.0.0.0"`. |
| `rssi_dbm` | number or `null` | The Wi-Fi bridge's signal (dBm) while `wifi` is `"bridge connected"`, else `null`; the built-in Wi-Fi's signal is `wifi_status` `rssi_dbm` (firmware 4.0.0). |
| `wifi` | string | Wi-Fi (ESP32-C3) state: `disabled`, `starting`, `waiting-slave`, `flashing-c3`, `connecting`, `connected` or `backoff`; `absent` on digital v3r3, which has no ESP32-C3 (firmware 3.9.5+). With a [Wi-Fi bridge](#wifi_set--wifi_clear) on the Ethernet port and no built-in Wi-Fi configured (`disabled` or `absent`), it reports the bridge instead: `"bridge "` and its state (`bridge connected`, `bridge joining`, `bridge unconfigured`, `bridge wait-pod`, `bridge auth-failed`, `bridge no-ap`). |
| `cloud` | string | Cloud link state: `disabled`, `waiting-wifi`, `connecting`, `connected` or `backoff`. |
| `short_form` | boolean | Present (`true`) only in the [short form](#short-form) (firmware 3.10.5+). |
| `eth_mac` | string | The Ethernet port's MAC, `"aa:bb:cc:dd:ee:ff"`: fixed per pod (made from the MCU's unique ID), the same after every update. For a network that hands out addresses only to registered devices. Full form only (the server has it from the capabilities frame). Firmware 3.10.8+. |
| `wifi_mac` | string or `null` | Pods with Wi-Fi only: the MAC the pod uses on Wi-Fi. With the built-in Wi-Fi, the ESP32-C3's station MAC, `null` until the C3 told it since power-on: a join or a [`wifi_scan`](#wifi_scan--the-networks-the-pods-wi-fi-sees) tells it, so a pod that never joined can report it. Through a [Wi-Fi bridge](#wifi_set--wifi_clear), the same as `eth_mac`: the bridge joins Wi-Fi with the pod's MAC. Full form only. Firmware 3.10.8+. |
| `c3_restarts` | number | Boards with the built-in Wi-Fi (ESP32-C3) only: how many times the pod restarted the C3 since power-on (firmware 3.10.1+). The count survives software and watchdog resets and OTA updates; a power-on or the reset pin sets it to 0. The causes are in [`wifi_status`](#wifi_status). |
| `usb` | object | The [USB network adapter](usb-serial-interface.md#usb-network-adapter) (firmware 3.9.0+): `net` is `true`, or `false` with `off_reason` `"setting"` (the [`usb_network`](#usb_network--the-usb-network-adapter-on-or-off) setting is off; the pod then enumerates as its console only) or `"wifi"` (Wi-Fi credentials are stored); `enabled` is the stored `usb_network` setting (up to firmware 3.10.13 `net` and `setting` were `"on"`/`"off"`); `link` is `true` when a computer has the adapter up; `ip` is the pod's address on it and `host_ip` the one its DHCP server gives the computer. Not counted in `net` or `ip`. `source` and `source_ma` are what the USB-C port powering the pod advertises (as [`usb_cc`](#usb_cc--usb-c-cc-line-state) `advertised`, `null` on a v2 pod): `"default"` (500 mA, a typical computer port) can be too little for the pod plus a DUT on the internal 5 V rail, so clients warn about it. |
| `adc_bits`, `adc_fullscale_mv`, `adc_channels` | integer | ADC format. |
| `la_vccio_mv` | integer | LA I/O voltage, `0` while unset: the bank (1800 or 3300), or `5000` in the 5 V mode of digital v3r4 (the bank is then at 3.3 V). |
| `adc_cal_a_uv`, `adc_cal_b_nv`, `adc_cal_unwrap` | integer, integer, bool | The front-SMA ADC fit captures are scaled with, this pod's calibrated offset included (analog board, firmware 3.9.3+). Same fields as the [capabilities frame](#capabilities-frame--adc-calibration-fields). Full form only: cloud clients read the fit from the capabilities. |
| `board_rev` | string | PCB revision, detected at boot from the revision straps: `"v2"`, `"v3"`, `"v3r2"` (analog v3r2), `"v3r4"` (digital v3r4, firmware 3.10.13+), or `"unknown"`. Gates the LA-bank 1.8 V setting, the dedicated NRST pin, and USB-C CC monitoring. Digital v3r3 carries the v3 strap and reports `"v3"`; the `module` field tells it apart. Digital v3r4 is reported only when both straps agree; otherwise the board runs as `"v3"`, exactly as older firmware runs it. |
| `nrst_pin` | boolean | `true` when the pod has the dedicated target-reset pin (v3+). When `false`, `nrst` and CMSIS-DAP `SWJ_PINS` reset requests are no-ops. |
| `flash_kb` | integer | The MCU's internal flash: `2048` or `1024`. Updaters check it before sending an image. |
| `sig_policy`, `sig_keys` | string, integer | The [signature policy](#sig_policy--which-firmware-and-blob-updates-the-pod-accepts) and how many release keys the firmware trusts. |
| `lan_policy` | string | The [LAN policy](#lan_policy--what-the-external-lan-may-do), for the external LAN (Ethernet and Wi-Fi); the USB network adapter stays on under every value. |
| `lan_pairing`, `lan_paired` | bool, bool or null | Firmware 4.0.1+: whether computers on Ethernet/Wi-Fi must be [paired](#lan_clients--paired-computers), and whether the asking connection is (`null`: it needs no pairing, as on USB, the USB link and the cloud). Both forms. |
| `lan_tls` | object | The [LAN API over TLS](#lan_tls--the-lan-api-over-tls) (firmware 3.9.3+): `port` (8443), `https_only` (default `true` from 4.0.0), `scpi` (4.0.0, default `false`), and `sha256` of the certificate in use (`""` when there is none). Full form only. |
| `lease` | object | Whether a cloud job holds the pod: `held`, `holder`, `left_s`. Full form only. |
| `local_lease` | object | The [pod lease](#pod-lease): `{"held":false}`, or `held`, `holder`, `purpose`, `left_s` and `mine` (this connection owns it). Never the lease id. Full form only. |
| `gateware`, `gateware_embedded` | integer | The gateware running in the iCE40 and the one this firmware embeds (`0` = unknown). They differ after a firmware update until the boot-time gateware update has run. `gateware` `0` with `psram_ok` `false` on a new board means a blank iCE40 config flash: from firmware 3.9.2 the pod loads image 0 from its `gw0` blob at boot or as soon as that blob is installed. |
| `loop_tripped` | boolean | The control loop's over-range trip is latched. |
| `uart_rx_overflow` | boolean | The UART proxy's receive FIFO overflowed. |
| `uart_rx_parity_error` | boolean | A frame of the current (or last) UART proxy session arrived with a wrong parity bit (gateware v56+). Full form only. |
| `psram`, `psram_ok` | string, boolean | The boot PSRAM self-test: `"ok"`, `"inoperable (...)"` or `"not run"`. When `psram_ok` is `false`, run [`psram_recover`](#psram_recover--reboot-to-recover-the-psram). |
| `heap_free`, `heap_min`, `stack_min` | integer | Heap and the lowest task's stack headroom in bytes. Per-task stacks and the other diagnostics are in [`diag`](#diag--heap-stack-and-network-diagnostics) (firmware 3.10.9+; up to 3.10.8 the full form had `stacks`, `lwip_mem_max`, `lwip_mem_size`, `pbuf_pool_max`, `pbuf_pool_size`, `malloc_failures`, `rx_dropped`, `tx_dropped` and `board_rev_mv`). |
| `reset`, `last_crash` | string | Why the pod last reset, and the last crash report (`""` when none). The short form cuts `last_crash` to 48 characters. |
| `analog` | boolean | `false` on the digital-only board (no DAC or ADC). `true` on digital v3r3 when the analog module was found at boot. |
| `module` | object | Digital v3r3 only (firmware 3.9.5+; left out on every other board): the [analog module](#analog-module-digital-v3r3) in short form, `present`, `analog`, `power`, `bridge_power`, `dac_pinout_ok` and `eeprom`. Full form only; on the cloud send [`module`](#module--the-analog-module-and-its-eeprom). |
| `safe_mode`, `safe_reason` | boolean, string | Safe mode after repeated crashes at boot, and what it turned off. See [Gates](#gates-that-apply-to-every-command). The short form cuts `safe_reason` to 48 characters. |
| `boot_health_cut` | boolean | Present (`true`) only when the short form cut `last_crash` or `safe_reason`. |
| `features_rev` | integer | The revision of the [features table](#features-table) this firmware is built from. |
| `api` | integer | Firmware 4.0.0+: the [client API level](#client-compatibility) this firmware speaks. |
| `min_client_api` | integer | Firmware 4.0.0+: the lowest `client_api` this firmware takes; a command from a client below it gets `BP0030`. |
| `fpga_image` | string | The running gateware image, `"loop"`, `"deep"`, `"emu"` or `"unknown"`. |
| `hw` | array of string | The hardware facts the features table needs: `wifi` (a built-in Wi-Fi or a Wi-Fi bridge), `usb_cc`, `pod_current`, `adc_low_range`, `vref_out` (digital v3r4: pin 21 switched and measured, 5 V I/O; firmware 3.10.13+). |

#### Removed in firmware 4.0.0

Firmware up to 3.10.13 also sent, for clients without the features table, a `caps` list (when the
request did not say `"caps":false`) and flags that were always `true`: `sensors_format`,
`ota_sig`, `sig_policy_cmd`, `lan_policy_cmd`, `tunnel_max_tier`, `lease_state`, `cloud_ca` and
`cloud_proxy`. Firmware 4.0.0 sends neither: every feature is in the table. A `caps` name maps to
a table entry by its `legacy_caps` (`features/features.toml`), for clients that still talk to
older firmware.

#### Features table

[`features/features.json`](../features/features.json) (generated from
[`features/features.toml`](../features/features.toml)) says which firmware and gateware have which
feature. A client keeps a copy, so a pod works without a network connection, and decides like this:

1. The pod sends `features_rev` (`status` with `"caps":false`, or the cloud capabilities frame): a
   feature is there when its `rev` is at most `features_rev`, the running `gateware` is at least
   the entry's `gateware`, the board matches (`board`: `any`, `analog` or `digital`, from `analog`)
   and every fact in `requires` holds (`hw`, `nrst_pin`, `sensors_digital`, or `fpga_image:<name>`).
2. No `features_rev` but a `caps` list (firmware up to 3.10.10): the list.
3. Neither: compare the firmware version with the entry's `firmware`.

A pod with a higher `features_rev` than the client's table has features the client does not
know yet: update the client.

#### Client compatibility

Firmware 4.0.0+. `features_rev` only adds: a client older than the pod misses the new
features and can warn. The client API level is for the other kind of change, one a client built
for an older level would misread: a reply that changes shape, a field that changes meaning or
units, or a refusal text replaced so clients that match on it break.

| Field | Where | Meaning |
|---|---|---|
| `api` | `status` with `"caps":false`, the cloud capabilities frame | The client API level of this firmware, from [`features/features.toml`](../features/features.toml). It goes up by one only for a change an older client would misread. |
| `min_client_api` | the same | The lowest level the firmware takes. It is raised only on purpose, in a release, and never above `api`. |
| `client_api` | every JSON command (request) | The level the client was built for. A command without it counts as 0. |
| `client` | every JSON command (request), optional | The client's name and version (`"sdk 4.0.0"`); the pod ignores it, and logs show it. |

The levels:

| `api` | Firmware | `min_client_api` | What changed |
|---|---|---|---|
| 1 | none released (development builds after 3.10.13) | 0 | The first level clients send. Firmware up to 3.10.13 announces no `api`: treat it as level 1. |
| 2 | 4.0.0 | 2 | Clients classify refusals by [error code](#error) only, never by their text. Every refusal a client acts on has a code, and a coded refusal carries no message: the client renders the catalog's. Firmware 4.0.0 refuses every client that does not send `"client_api":2` or higher. |

A command whose `client_api` is below `min_client_api` is refused before every other
[gate](#gates-that-apply-to-every-command), on the LAN, USB and both cloud paths:

```json
{"status":"error","code":"BP0030","args":{"client_api":1,"min_client_api":2,"api":2},"message":"Update the BenchPod SDK, MCP or CLI: this firmware needs client API 2 or newer (this client sent 1)"}
```

`status`, `diag`, `ping` and `lease_release` always run, so a client can see the levels, report
them and let go of the pod. So do the update commands (`ota_status`, `ota_begin`, `ota_data`,
`ota_end`, `ota_commit`, `ota_abort`, `ota_selftest`, `blob_status` and `identity_public`), so
an old client can still update the pod's firmware or blobs, or roll it back. These are the
commands every older update path sends (CLI `flash-self --network` and `install-blobs`, the desktop
app through its CLI, the server's cloud update): none takes a lease first. The CLI's
`install-blobs` from 0.9.0 on also sends `fpga_image` after a gateware blob, to load the new
gateware without a restart; firmware 4.0.0 refuses that (`BP0030`), and the CLI then says to
restart the pod, which loads it. SCPI and the USB
console have no check: people read them.

A client:

1. Sends `"client_api":<its level>` on every command. It is stateless, so this works the same on
   every transport, the cloud command channel included (the server passes the command on as it
   is). Older firmware ignores it. It counts toward the line limit (1279 bytes).
2. Reads `api` and `min_client_api` from `status` (`"caps":false`) or the capabilities frame.
   No `api` (firmware up to 3.10.13): treat it as level 1.
3. Stops, and says to update the client, when `api` is above its own level (the firmware has a
   change this client would misread) or `min_client_api` is above it (the pod refuses it).
4. Shows a `BP0030` refusal from its `message`: a client from before the codes shows that text,
   which says what to do.

Firmware 4.0.0 also changes defaults a client has to follow, though they do not change a reply's
shape (so they are not an `api` level):

- **The LAN is TLS only.** The JSON API answers on Ethernet and Wi-Fi on `8443` only; plaintext
  `8080` answers on the USB network adapter. A client connects to a LAN pod on `8443` and pins its
  certificate on first use (see [`lan_tls`](#lan_tls--the-lan-api-over-tls)); a plaintext-only
  client fails to connect there (the connection is reset), or gets `BP0075` when SCPI is on.
- **SCPI is off** until switched on (`lan_tls` `"scpi":true`, over USB or from the cloud).
- **Signatures are always required** for every update ([`sig_policy`](#sig_policy--which-firmware-and-blob-updates-the-pod-accepts)):
  install signed releases; self-built firmware goes in over USB DFU. `BP0074` for an attempt to
  loosen it.

---

### `diag` — heap, stack and network diagnostics

Firmware 3.10.9+. The pod's health counters, which `status` carried in its full form up to 3.10.8.
Older firmware answers `BP0002` (unknown command): read them from its `status` then.

```json
{"cmd":"diag"}
```

```json
{"status":"ok","data":{"heap_free":128120,"heap_min":126280,"stack_min":400,
 "lwip_mem_max":12288,"lwip_mem_size":65536,"pbuf_pool_max":1,"pbuf_pool_size":16,
 "malloc_failures":0,"rx_dropped":0,"tx_dropped":0,"board_rev_mv":2728,
 "stacks":{"usb":2792,"console":3680,"net":12048,"hw":6992,"IDLE":400}}}
```

| Field | Type | Description |
|---|---|---|
| `heap_free`, `heap_min` | integer | Free heap bytes now, and the lowest since boot. |
| `stack_min` | integer | The lowest free stack bytes of any task since boot; `stacks` says which. |
| `stacks` | object | Per task (every task, `IDLE` and the timer task included): the lowest free stack bytes since boot. |
| `lwip_mem_max`, `lwip_mem_size` | integer | The network stack's heap: the most used since boot, and its size. |
| `pbuf_pool_max`, `pbuf_pool_size` | integer | The packet buffer pool: the most used since boot, and its size. |
| `malloc_failures` | integer | Allocations that failed since boot. |
| `rx_dropped`, `tx_dropped` | integer | Console bytes dropped because a ring was full. |
| `board_rev_mv` | integer | Raw revision-strap voltage in millivolts (`-1` if not measured). |
| `board_rev2_mv` | integer | The second revision strap in millivolts. Only on boards whose first strap reads the digital v3r4 band (firmware 3.10.13+); absent on every older board. |

---

### Logic Analyzer (LA) GPIO channels

Every LA command (`la`, `gpio`, `la_capture`, `dap_start`, `uart_proxy_start`, `sensor_start`, `spi_start`, capture triggers) addresses the FPGA's **Logic Analyzer bank** by a **1-based LA channel index** (`LA1`..`LA14`). LA channel `N` is J1 pin `N` (see below).

These pins live on the **iCE40 FPGA**. The stepper pulse generator, the SWD engine, the UART, the I2C target and the SPI master run inside the gateware; the STM32 only sends high-level commands over SPI.

| LA channel | iCE40 SG48 pin | | LA channel | iCE40 SG48 pin |
|---|---|---|---|---|
| `1` | 42 | | `8`  | 32 |
| `2` | 43 | | `9`  | 31 |
| `3` | 37 | | `10` | 28 |
| `4` | 38 | | `11` | 26 |
| `5` | 36 | | `12` | 27 |
| `6` | 35 | | `13` | 25 |
| `7` | 34 | | `14` | 23 |

(See `ice40/vbench_pod.pcf`. The channel→pin map is fixed per gateware build.)

---

<a id="dut-header"></a>
### DUT pin header — J1 pinout

Everything the pod exposes to the device under test lands on the 2×11 header
**J1**. LA channel `N` is simply **J1 pin N**, so the channel numbers used by
`la`, `la_capture` and `dap_start` are the header's own numbering.

| Pin | Signal | Pin | Signal |
|---|---|---|---|
| 1 | `LA1` | 12 | `LA12` |
| 2 | `LA2` | 13 | `LA13` |
| 3 | `LA3` | 14 | `LA14` |
| 4 | `LA4` | 15 | `GND` |
| 5 | `LA5` | 16 | `GND` |
| 6 | `LA6` | 17 | `+3V3` |
| 7 | `LA7` | 18 | `+1V8` |
| 8 | `LA8` | 19 | `CAN+` |
| 9 | `LA9` | 20 | `CAN−` |
| 10 | `LA10` | 21 | `LA_VCCIO` **(v3)** — the LA bank rail, at whatever `la_voltage` selected |
| 11 | `LA11` | 22 | `NRST` **(v3)** — target reset, open-drain, pulled up to `LA_VCCIO` |

Every LA line has a 330 Ω series resistor and an ESD clamp between the header and
the FPGA. Pins **21 and 22 are new in v3**: earlier boards had no reset pin (the
reset line had to borrow an LA channel) and did not bring the bank rail out.

- **Reset (pin 22):** see [`nrst`](#nrst--drive-the-targets-reset-line). Used
  automatically by `dap_start` for connect-under-reset — nothing to configure.
- **`LA_VCCIO` (pin 21):** the same rail that feeds the LA bank buffers, so a DUT
  can be referenced to it. Select it with
  [`la_voltage`](#la_voltage--select-the-la-io-bank-voltage) *before* connecting a
  DUT — on a v3 pod it comes up at 3.3 V. On digital v3r4 pin 21 is also the pull-up rail, the
  pod measures it, and it goes to 5 V in [5 V I/O](https://embeddedci.com/docs/benchpod-targets/5v-rs485#5v-io).

---

### `la_step` — step a pulse train

Firmware up to 3.10.13 had one `la` command for the step train and the pull resistors, told apart
by their fields. Firmware 4.0.0 splits it: `la_step` runs a step train, and
[`la_pull`](#la_pull--la-pin-pull-resistors) switches and reads the pull resistors. `la` is gone
(`unknown cmd`). There is no active-drive action here (that is
[`gpio`](#gpio--drive-and-read-la-pins-as-gpio)).

#### Step a pulse train

Starts a **non-blocking** train of `steps` pulses on an LA channel: each pulse is high for `delay_us` microseconds then low for `delay_us` (one full step period is `2 × delay_us`). The FPGA runs the train autonomously; the command returns immediately. Poll completion via SCPI `DIGital:STEP:BUSY?`.

```json
{"cmd":"la_step","la":<1-14>,"steps":<n>,"delay_us":<us>,"dir_la":<1-14>,"direction":<0|1>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `la` | integer | yes | — | LA channel to pulse (1..14). |
| `steps` | integer | yes | — | Number of pulses. Range 1..65535. |
| `delay_us` | integer | yes | — | Microseconds per half-phase. Range 4..65535. |
| `dir_la` | integer | no | — | Optional direction channel driven (high/low) **before** stepping. |
| `direction` | integer | no | 0 | Level for `dir_la`: `1` = high, `0` = low. |

Response:

```json
{"status":"ok","data":{"la":1,"steps":200,"delay_us":500,"status":"started"}}
```

<a id="la_pull--la-pin-pull-resistors"></a>
### `la_pull` — LA pin pull resistors

`{"cmd":"la_pull","la":N,"pull":"up"|"down"|"off"}` switches a pull, `{"cmd":"la_pull","la":N}`
reads one, and `{"cmd":"la_pull"}` reads all of them as a bitmask. Firmware up to 3.10.13 did this
with the `la` command.

#### Switch (or query) an LA pin pull resistor

Engages or releases the pull resistor on an LA channel, in a direction: `up`, `down` or
`off`. The host names the direction only. The resistor value is reported, never chosen:
on boards with more than one value per channel the firmware picks it from the pin's
function. Which channels have a resistor, and which way each one can pull, depends on
the board. The reply and [`la_pins`](#la_pins--report-every-la-pin) say it per channel
(`dirs`). On boards before digital v3r4 each resistor pulls one way only (digital v3r4 pulls
both ways on every channel, see [below](#pull-resistors-on-digital-v3r4)):

| LA pin | Resistor | | LA pin | Resistor |
|---|---|---|---|---|
| 1 | 4.7k pull-**up** | | 5 | 10k pull-**up** |
| 2 | 4.7k pull-**up** | | 6 | 10k pull-**up** |
| 3 | 2.2k pull-**up** | | 7 | 10k pull-**down** |
| 4 | 2.2k pull-**up** | | 8 | 10k pull-**down** |

LA9-LA14 have none. Asking for a direction a channel can't pull is refused with
[BP0024](../errors/ERRORS.md) ("LA7 supports pull-down only on this board"), and `up` or
`down` on a channel without a resistor with [BP0023](../errors/ERRORS.md) ("LA9 has no
pull resistor on this board"). `off` is always accepted, also on a channel without one.

All of them default to **off** at boot.

> **Works at 1.8 V and 3.3 V.** The pull-ups go to the LA bank rail and the pull-downs to
> ground, so a pull-up holds the line at the bank voltage (1.8 V or 3.3 V, see
> [`la_voltage`](#la_voltage--select-the-la-io-bank-voltage)) and engaged pulls stay on when
> the bank switches. A bank voltage must be set first. Firmware before the `la_pull_1v8`
> feature allowed pulls only at a 3.3 V bank and released them when the bank switched to
> 1.8 V. (Pre-production v2 boards had their pull-ups on 3.3 V; they still need a 3.3 V bank.)
> Turning one *off* is always allowed.

```json
{"cmd": "la_pull", "la": 3, "pull": "up"}
```

- `la` (required for a per-pin op): LA pin `1`-`14`.
- `pull` (optional, firmware with the `la_pull_dir` feature): `"up"`, `"down"` or `"off"`.
- **Omit it** (a bare `{"cmd":"la_pull","la":N}`) to **query** the current state without
  changing it.
- `pullup` (`"on"`/`"off"`, the older form of `pull`) is gone in firmware 4.0.0: the pod does not
  read it, so such a request is a query. Firmware up to 3.10.13 took it, `on` engaging the
  channel's own direction.

Response:

```json
{"status": "ok", "data": {"la": 3, "state": "up", "dirs": ["up"], "pullups_available": true}}
```

| Field | Meaning |
|---|---|
| `state` | What the channel does now: `"up"`, `"down"` or `"off"`. |
| `dirs` | Every direction the channel can pull: `["up"]`, `["down"]`, `["up","down"]`, or `[]` without a resistor. |
| `pullups_available` | `true` when a pull can be engaged now (a bank voltage is set): at `false` every `up`/`down` request is refused. Firmware up to 3.10.13 sent `1`/`0`. |

Firmware up to 3.10.13 also replied `pullup` (`1` when engaged), `ohms` (the resistor's value)
and `pull` (the channel's default direction); [`la_pins`](#la_pins--report-every-la-pin) has the
resistor's value and direction. Firmware before the `la_pull_dir` feature takes `pullup` only.

#### Pull resistors on digital v3r4

Digital v3r4 (feature `la_pull_strength`, hardware fact `vref_out`; firmware 3.10.13+;
designed, not built yet) has two resistors on **every** channel, LA1-LA14: a 10k and a 2.2k, each switched by an I2C
expander that can pull up or down. So every channel pulls `up` or `down`, with 10k, 2.2k
or both (about 1.8k). The pull-ups go to J1 pin 21, which follows the I/O voltage (1.8 V,
3.3 V, or 5 V in [5 V I/O](https://embeddedci.com/docs/benchpod-targets/5v-rs485#5v-io)), so they work at every
voltage.

The pod picks the strength from the pin's function unless the host names one:

| Pin function | Strength |
|---|---|
| I2C (emulated sensor or I2C master), one-wire UART, gpio `open_drain` | 2.2k |
| Pod outputs (gpio output, UART TX, SPI, SWD) in 5 V mode: the pull-up lifts the line from 3.3 V to 5 V | 2.2k |
| Everything else (UART RX hold, straps, idle levels, RS-485 DE bias) | 10k |

```json
{"cmd": "la_pull", "la": 9, "pull": "down", "strength": "both"}
{"status": "ok", "data": {"la": 9, "state": "down",
  "dirs": ["up","down"], "strength": "both", "strengths": ["10k","2.2k","both"], "pullups_available": true}}
```

- `strength` (optional, digital v3r4): `"10k"`, `"2.2k"` or `"both"`. Without it the pod picks
  from the table above. Other boards ignore it, as older firmware does.
- The reply adds `strength` (`"none"` when off) and `strengths`.
- [`la_pins`](#la_pins--report-every-la-pin) entries leave `dirs` out on this board (every
  channel pulls both ways; the reply says it once as `"pull_dirs":["up","down"]`), so the
  14-channel list fits the cloud reply.
- `pullups_available` is `true` once an I/O voltage is set and pin 21 is up.
- When pin 21 goes down (shorted, overloaded, or held down) the pod holds both expanders in
  reset (every pull off) and refuses `up`/`down` with `BP0029`. When pin 21 is back it
  writes every pull back as the host left it.
- When the pod switches the DUT's power off (`target_power`), the pull-ups stay off until a
  rail is switched on again or the host sets a pull, so pin 21 does not feed an unpowered DUT.
  Pull-downs stay. A DUT that the pod never powers keeps its pulls.
- While the pod holds a pull off (the two cases above), the replies say what is on the wire
  (firmware 3.10.13+). `state` is what the channel does now, and the reply
  adds `"held"`: `"pin21_down"`, `"dut_off"` (pull-ups suspended) or `"fault"` (an expander
  write failed; the next pull the host turns on with `la_pull` tries again and puts them all back). `strength` stays
  the setting the pod puts back. In [`la_pins`](#la_pins--report-every-la-pin), `gpio` and
  `la_dir` entries, `on` is what is on the wire. Other boards never hold a pull, and their
  replies have no `held`.
- As on the other v3 boards, the pulls stay on when the bank switches; on v3r4 that includes
  switching to and from 5 V (pin 21 moves with it).

#### Read all pulls as a bitmask

Send `la_pull` with no channel:

```json
{"cmd": "la_pull"}
```

Response:

```json
{"status": "ok", "data": {"la_pullup_mask": 5, "pullups_available": true}}
```

`la_pullup_mask` bit `(la-1)` is set when LA`<la>`'s pull is engaged, whichever way it
pulls (here `0b00000101` = LA1 and LA3). `pullups_available` reports whether a pull can be engaged at
the current bank voltage (`true` = yes, at 1.8 V or 3.3 V; firmware before `la_pull_1v8`: 3.3 V only).

On digital v3r4 (firmware 3.10.13+), while the pod holds pulls off, the mask is what is on
the wire and the reply adds `held` (the reason, as in [the pull resistors on digital
v3r4](#pull-resistors-on-digital-v3r4)) and `asked_mask`, the pulls the host set, which the pod
puts back:

```json
{"status": "ok", "data": {"la_pullup_mask": 0, "pullups_available": false, "held": "pin21_down", "asked_mask": 5}}
```

#### Error cases

| Condition | Message |
|---|---|
| `la_step` without `steps` / `la` / `delay_us` | `"missing steps"` / `"missing la"` / `"missing delay_us"` |
| `steps` / `delay_us` not a plain non-negative integer | `"invalid steps"` / `"invalid delay_us"` |
| `steps` outside 1..65535 (the gateware step counter is 16-bit) | `"steps must be 1..65535 (the step counter is 16-bit): split a longer move into several trains"` |
| `delay_us` outside 4..65535 | `"delay_us must be 4..65535 (microseconds per half-phase: a step takes 2 x delay_us)"` |
| a step train is already running | `BP0066` (`step_busy`) |
| step `la` invalid or out of range | `"invalid args"` |
| `dir_la` invalid, or equal to `la` | `"invalid dir_la"` / `"dir_la must be a different pin than la"` |
| the step or direction pin belongs to another function | `BP0031` (`la_pin_in_use`, args `la`, `owner`, `function`, `instance`) |
| `la` outside 1-14 | `"LA<n> is not a pin (use 1..14)"` |
| `pull` not `up`, `down` or `off` | `"pull must be up, down or off"` |
| `up`/`down` on a channel without a resistor | BP0023 `"LA9 has no pull resistor on this board"` |
| a direction the channel's resistor can't pull | BP0024 `"LA7 supports pull-down only on this board"` (args `la`, `want`, `have`) |
| `up`/`down` with the LA bank at 1.8 V on a v2 board (pull-ups on 3.3 V) | `"this board's pull-ups go to 3.3 V; set the LA bank to 3.3 V first"` (firmware before `la_pull_1v8` refused this on every board) |
| pull-**down** enable on a pin whose function needs the line high | `BP0033` (`la_pull_down_conflict`; see [pin ownership](#la-pin-ownership-functions-and-conflicts)) |
| `strength` not `10k`, `2.2k` or `both` (digital v3r4) | `"strength must be 10k, 2.2k or both"` |
| `up`/`down` while J1 pin 21 is down (digital v3r4) | BP0029 `"pin 21 is down (120 mV): the pull resistors are off until it is back"` |
| `up`/`down` before an I/O voltage is set (digital v3r4) | `BP0001` (`la_voltage_not_set`) |
| the I2C write to a pull expander failed | `"pull expander write failed"` |

The step train also participates in **pin ownership**: a free step / `dir_la` pin is claimed
for the duration of the train (function `step` / `step_dir`) and released when the gateware
reports the train finished. A pin already configured as a **gpio output** is pulsed in place
without changing its ownership, and its commanded level returns afterwards.

---

<a id="la-pin-ownership"></a>
### LA pin ownership: functions and conflicts

Every one of LA1..LA14 has exactly **one function** at a time. Before this existed the
gateware's pin bank resolved overlaps *silently* by priority (SWD > stepper > I2C > UART >
static level), so a UART proxy and an emulated I2C sensor could be told to share a channel
and the loser simply did not work. The firmware now keeps a table and refuses the collision.

| Function | Set by | Released by |
|---|---|---|
| `none` | default — "LA mode": high-Z, observed by captures | — |
| `gpio` | [`gpio`](#gpio--use-la-pins-as-gpio) with mode `input` / `output` / `open_drain` | `gpio` mode `off` |
| `uart_rx`, `uart_tx`, `uart_hd`, `uart_de` | [`uart_proxy_start`](#uart_proxy_start--transparent-uart-bridge) | the proxy ending (`+++`, send failure, connection close, tunnel reset) |
| `swd_clk`, `swd_dio` | `dap_start` (see [dap-over-tunnel.md](dap-over-tunnel.md)) | SWD disarm, connection close, tunnel reset |
| `i2c_sda`, `i2c_scl` | [`sensor_start`](#sensor_start--start-an-instance) of an I2C sensor (shared by the I2C sensors on the same pair) | `sensor_stop` of the last instance on it, a reconfiguration away from the emulation image |
| `sensor_tx`, `sensor_rx`, `sensor_drdy`, `sensor_sck`, `sensor_mosi`, `sensor_miso`, `sensor_cs` | `sensor_start` of a UART sensor, a data-ready pin, the SPI sensor | `sensor_stop`, a reconfiguration away from the emulation image |
| `step`, `step_dir` | the [`la_step` train](#step-a-pulse-train) on a free pin | the train finishing (the firmware polls the gateware) |
| `spi_sck`, `spi_mosi`, `spi_miso`, `spi_cs` | [`spi_start`](#spi_start--claim-four-pins-for-spi) | `spi_stop`, gateware reconfiguration |
| `i2cm_sda`, `i2cm_scl` | [`i2c_start`](#i2c_start--claim-two-pins-as-an-i2c-master) | `i2c_stop`, gateware reconfiguration |

**Captures never own pins.** `la_capture`, `capture_dual`, `sensor_la` and capture triggers
observe all 14 channels whatever their function.

Claims **persist across client disconnects** for `gpio` and the emulated sensor — they are pod
state, like the LA voltage — and are cleared by a pod reboot. A gateware reconfiguration (an
image swap or `flash-ice40`) resets every engine in the fabric: the firmware re-applies the
`gpio` pins' output state and releases the UART / SWD / I2C / step functions so the table and
the wire agree again.

#### Conflict errors

Claiming a pin another function owns is refused with `BP0031` (`la_pin_in_use`):

```json
{"status":"error","code":"BP0031","args":{"la":4,"owner":"gpio","function":"gpio","instance":""}}
```

`owner` is what `la_pins` shows: the function (`gpio`, `uart_tx`, `swd_dio`, `step`, `spi_cs`,
`i2cm_sda`, ...), or for an emulated sensor's pin `sensor:<instance> (<function>)` with
`instance` set. In `la_pins` such a pin carries `"owner":"sensor:<instance>"`; on an I2C line
several sensors share, the first one and how many others: `"sensor:env+1"`. What frees the pin
follows from the function: `gpio` with `mode` `off` for a GPIO, `uart_proxy_stop` for the UART,
the end of the SWD session or the step train, `sensor_stop` for a sensor, `spi_stop` or
`i2c_stop` for the masters. The catalog's fix says it in each client's terms. SCPI and the USB
console get the text `pin conflict: LA<n> is in use by <owner>`.

When a request names several pins, the **first** conflict is reported and **nothing changes**.

Setting a level on a pin that is not a gpio output is refused with `BP0036`
(`la_not_gpio_output`, args `la` and `function`, `gpio input` for a GPIO input).

#### Pull compatibility

Engaged pulls are checked against pin functions in **both** directions: when claiming a function
on a pin whose pull is engaged, and when engaging a pull on a pin that already has a function.
On boards before digital v3r4 the resistors are fixed (LA1/LA2 4.7k up, LA3/LA4 2.2k up,
LA5/LA6 10k up, **LA7/LA8 10k down**, LA9-LA14 none), so only LA7/LA8 can have a pull-down
engaged. On digital v3r4 every channel can pull down, and the table applies to any channel
with its pull-down engaged.

| Function | Pull-up engaged | Pull-down engaged |
|---|---|---|
| `none`, gpio `input`, gpio `output`, `swd_clk`, `step`, `step_dir`, `spi_*` | ok | ok |
| gpio `open_drain` | ok | **conflict** — a released line would read low |
| `uart_rx`, `uart_tx` | ok | **conflict** — the line idles high |
| `i2c_sda`, `i2c_scl` | ok | **conflict** — an open-drain bus needs pull-ups |
| `swd_dio` | ok | **conflict** — SWDIO is pulled up when released |

```json
{"status":"error","code":"BP0032","args":{"la":7,"ohms":"10k","function":"uart_rx","reason":"the line idles high"}}
{"status":"error","code":"BP0033","args":{"la":7,"ohms":"10k","function":"i2c_sda","reason":"an open-drain bus needs pull-ups"}}
```

`BP0032` (`la_pull_down_engaged`) comes from a claim (`uart_proxy_start`, `dap_start`,
`sensor_start`, `gpio`), `BP0033` (`la_pull_down_conflict`) from `{"cmd":"la_pull","la":7,"pull":"down"}`
on a pin that is already in use.

---

### `la_pins` — report every LA pin

Reports the function, gpio mode/level and pull state of all 14 channels, plus the live pin
levels when the gateware can read them back.

```json
{"cmd":"la_pins"}
{"cmd":"la_pins","la":[1,2,3,4,5,6,7]}
{"cmd":"la_pins","from":10}
```

`la` (optional, firmware 3.10.13+): a pin or a list of pins. The reply then lists only
those pins' entries, in pin order; everything else is the same. Without `la` (or with `"all"`)
the reply lists all 14, as before. Older firmware ignores `la` and lists all 14, so a host that
asks for a subset keeps only the entries it asked for.

`from` (optional, firmware 4.0.1+, feature `la_pins_paged`): the first pin to list, 1 to 14,
normally the `next` of the previous reply. It combines with `la` (the pins of `la` from `from`
on). Older firmware ignores it and starts at LA1.

Response:

```json
{"status":"ok","data":{"pins":[
  {"la":1,"function":"none","gpio":null,"level":null,"pull":{"dir":"up","ohms":"4.7k","on":false,"dirs":["up"]}},
  {"la":2,"function":"gpio","gpio":"output","level":1,"pull":{"dir":"up","ohms":"4.7k","on":false,"dirs":["up"]}},
  {"la":9,"function":"uart_rx","gpio":null,"level":null,"pull":null}
],"levels":1234}}
```

| Field | Meaning |
|---|---|
| `function` | one of `none`, `gpio`, `uart_rx`, `uart_tx`, `swd_clk`, `swd_dio`, `i2c_sda`, `i2c_scl`, `step`, `step_dir`, `spi_sck`, `spi_mosi`, `spi_miso`, `spi_cs`, the `sensor_*` functions, `i2cm_sda`, `i2cm_scl` |
| `gpio` | `null`, or the gpio mode (`input` / `output` / `open_drain`) |
| `level` | the **commanded** level of a gpio `output` / `open_drain` pin, else `null` |
| `pull` | `null` on a channel without a resistor (LA9-LA14 today); otherwise `dir` (the way it pulls when on), `ohms`, `on` (engaged) and `dirs` (every direction it can pull, firmware with `la_pull_dir`) |
| `dir` | only when set with [`la_dir`](#la_dir--which-way-each-pins-dut-signal-goes): `dut_input`, `dut_output` or `bidir` |
| `warn` | only when non-zero, bits: 1 = a pull is on under a push-pull output (it only costs current; use `open_drain` or switch the pull off), 2 = the pod's target power is off while the pin is driven high or pulled up (this powers an unpowered DUT through its I/O pins; ignore it when the DUT has its own supply) |
| `levels` | live pin levels, bit `la-1`, read back from the gateware; `null` when the running image is older than **v35** |
| `pull_dirs` | digital v3r4 only: `["up","down"]`, every channel's pull directions (the entries leave `dirs` out) |
| `next` | only when the reply could not carry every pin asked for (firmware 4.0.1+): the first pin it left out. Ask again with `"from":<next>` (and the same `la`, if any) for the rest |

The full reply is about 1.3 kB. With a `dir` and a `warn` on every pin it can reach about
1.8 kB, and with an emulated sensor instance with a long name on every pin about 2.4 kB: more
than one cloud command reply holds (1791 bytes). Firmware 4.0.1 and later never refuses the listing
for its size: each reply carries the pins that fit in 1727 bytes (one cloud reply less 64 bytes)
and says `next`. A host reads the whole list like this:

```json
{"cmd":"la_pins"}
{"status":"ok","data":{"pins":[{"la":1,...},...,{"la":9,...}],"next":10,"levels":1234}}
{"cmd":"la_pins","from":10}
{"status":"ok","data":{"pins":[{"la":10,...},...,{"la":14,...}],"levels":1234}}
```

A reply without `next` is the last one. `levels` is read again for every reply. A reply that fits
whole is the same as before (no `next`), so most listings stay one round trip. Older firmware
answers `BP0053` (`reply_too_large`; `BP0054` over the cloud) instead; read it in two parts
there: `"la":[1,2,3,4,5,6,7]` and `"la":[8,9,10,11,12,13,14]` each fit with room to spare.

`la_pins` does **not** require the LA voltage: it reports state and reads nothing that would
drive a DUT line.

---

### `la_dir` — which way each pin's DUT signal goes

```json
{"cmd":"la_dir","la":[9,10],"dir":"dut_output"}
```

`la` is a pin, a list of pins or `"all"`; `dir` is `dut_input`, `dut_output`, `bidir` or `unknown`
(the default after a reboot). Hosts send it from the wiring profile. On a pin wired to a DUT
output the pod refuses every push-pull driver (a gpio `output`, UART TX, SPI SCK / MOSI / CS, SWD,
the step train, an emulated sensor's TX / MISO / data-ready), because two push-pull outputs would
drive against each other: `BP0034` (`la_dut_output`, args `la` and `function`). Open drain
(`open_drain`, I2C) and inputs are allowed. A pin that is driven can't be marked `dut_output`
until it is released (`BP0035`, `la_driven_pin`). `{"cmd":"la_dir"}` lists every pin. The reply is the `la_pins` entries of
the pins it changed.

### `gpio` — use LA pins as GPIO

Four shapes, distinguished by which fields are present. All of them require the LA voltage to
have been set (see [`la_voltage`](#la_voltage--select-the-la-io-bank-voltage)).

**Modes:** `input` (high-Z, level readable), `output` (push-pull, drives `level`),
`open_drain` (`level` 0 drives low, `level` 1 releases the line to high-Z), `off` (release the
pin back to `none`).

#### 1. Configure (claim) or release

```json
{"cmd":"gpio","la":5,"mode":"output","level":1}
{"cmd":"gpio","la":[5,6,7],"mode":"input"}
{"cmd":"gpio","la":5,"mode":"off"}
{"cmd":"gpio","la":"all","mode":"off"}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `la` | integer, array of integers, or `"all"` | yes | — | Channel(s) 1..14. `"all"` is allowed **only** with `"mode":"off"`. |
| `mode` | string | yes (here) | — | `input`, `output`, `open_drain` or `off`. |
| `level` | `0` or `1` | no | `0` for `output`, `1` (released) for `open_drain` | Not accepted with `input`. |

Switching between gpio modes on a pin that is already `gpio` is allowed. `"la":"all"` with
`"mode":"off"` releases **every** gpio pin and leaves other functions untouched (no error).
The whole request is validated first: on any error **nothing changes**.

#### 2. Set the level of pins that are already gpio outputs

```json
{"cmd":"gpio","la":[5,6],"level":0}
```

#### 3. Read the live levels

```json
{"cmd":"gpio"}
```

```json
{"status":"ok","data":{"levels":1234,"pins":[ …12 entries… ]}}
```

`levels` bit `la-1` is the live level of LA`<la>`, sampled in the fabric through a two-flop
synchroniser. This needs gateware **v35** or newer; older images answer `BP0017`
(`gateware_too_old`, feature `reading pin levels`, min 35).

#### Response (1 and 2)

The affected pins, in the same entry shape `la_pins` uses:

```json
{"status":"ok","data":{"pins":[{"la":5,"function":"gpio","gpio":"output","level":1,"pull":{"dir":"up","ohms":"10k","on":false,"dirs":["up"]}}]}}
```

#### Error cases

| Condition | Message |
|---|---|
| the LA voltage has not been set | `BP0001` (`la_voltage_not_set`) |
| neither `mode` nor `level` given, with a `la` | `"gpio needs a mode or a level"` |
| `mode` not recognised | `"mode must be input, output, open_drain or off"` |
| `level` not 0/1 | `"level must be 0 or 1"` |
| `la` not a number, list or `"all"` | `"la must be a pin number, a list of pin numbers, or \"all\""` |
| `la` out of range | `"la must be 1..14 (or \"all\" with \"mode\":\"off\")"` |
| `"la":"all"` with anything but `"mode":"off"` | `"\"la\":\"all\" is only allowed with \"mode\":\"off\""` |
| `level` with `"mode":"input"` | `"level applies to output and open_drain pins, not input"` |
| the pin belongs to another function | `BP0031` (see [above](#la-pin-ownership)) |
| a pull-down would fight the mode | `BP0032` (see [above](#la-pin-ownership)) |
| level set on a pin that is not a gpio output | `BP0036` (`la_not_gpio_output`) |
| `{"cmd":"gpio"}` read on gateware < v35 | `BP0017` (feature `reading pin levels`, min 35) |

#### SCPI

`DIGital:OUTPut <la>,<0|1>` goes through the same table: the pin becomes a **gpio output** at
that level, shows up in `la_pins`, and is released with `{"cmd":"gpio","la":<n>,"mode":"off"}`.
It requires the LA voltage, and raises a settings-conflict error if another function owns the
pin. `DIGital:STEP` follows the step-train rules above.

---

### `la_voltage` — select the LA I/O-bank voltage

The LA bank I/O voltage (iCE40 `VCCIO_0` — the supply for **all** of LA1..LA14) is
switched between **3.3 V** and **1.8 V** by an on-board TPS2116 power-mux (U8),
driven by `PG3` (select) with status on `PG4`. At boot the selection is **unset**. From
firmware 3.10.10 the first LA-bank operation (`la`, `la_capture`, `dap_start`, `uart_proxy_start`,
`sensor_start`, SCPI `DIG:OUTP` / `DIG:STEP`) then takes it from the wiring profile:
the `la_mv` of the profile a powered rail was switched on with (`target_power`
`profile`), else of the pod's default (active) profile. So the DUT's logic level is
still an explicit choice, made once in the profile instead of after every boot.
Without a profile that sets `la_mv` (1800 or 3300) the operation is refused with
`BP0001` (`la voltage not set`) until the host sets one, here or in the profile.
Nothing is driven before an LA operation asks for it: the profile's voltage is not
applied at boot.

```json
{"cmd": "la_voltage", "mv": 3300}
{"status": "ok", "data": {"mv": 3300, "st": 1, "readback_mv": 3300}}
```

- `mv` (optional): `1800` or `3300`. **Omit `mv`** to query the current state
  without changing it.
- `st`: the raw TPS2116 status pin (`PG4`) level (0/1).
- `readback_mv`: `st` decoded to a rail voltage — `3300` or `1800` — i.e. what the
  mux says it is *actually* passing, independent of what was requested. `0` on a
  v2 pod, where `st` is ambiguous (see below).

A bare `{"cmd": "la_voltage"}` reports `{"mv": 0, …}` before any voltage is
set (it does not take the profile's). `status` also carries the current selection as `la_vccio_mv` (0 = unset).

#### 5 V and pin 21 (digital board v3r4)

On digital v3r4 (feature `la_5v`, hardware fact `vref_out`; firmware 3.10.13+) `mv` also
takes **`5000`**: 5 V I/O. In short, the bank stays at 3.3 V, J1 pin 21 (the pull-up rail) goes
to 5 V, and outputs reach 5 V through their pull-up. What it means for your wiring is on the
[5 V page](https://embeddedci.com/docs/benchpod-targets/5v-rs485#5v-io). This section is the protocol. No v3r4 board is built yet, so its numbers come
from the design and the datasheets.

The pod checks pin 21 after switching. Below 4.3 V it goes back to the previous voltage and
refuses with `BP0028`. `la_mv: 5000` in a wiring profile works the same way. Every other board
refuses `5000` as before (`"la voltage must be 1800 or 3300 (mv)"`).

```json
{"cmd": "la_voltage", "mv": 5000}
{"status": "ok", "data": {"mv": 5000, "st": 1, "readback_mv": 3300,
  "vref_out": {"source": "5v", "set_mv": 5000, "mv": 4920, "state": "ok", "up": true}}}
```

- `mv`: the DUT's I/O voltage (`5000` in 5 V mode); `readback_mv` stays the bank (3300).
  `status.la_vccio_mv` reports the same `mv`.
- `vref_out` (digital v3r4 only): pin 21. `source` `bank` (follows the bank, 1.8 or 3.3 V, also
  at power-up) or `5v`; `set_mv` what it should be; `mv` what the pod measures (`-1` without a
  reading); `up` pin 21 is high enough for the pull resistors to work. `state`:
  - `ok`: within the band of its setting (5 V mode: 4.3 to 5.3 V).
  - `high`: more than 0.2 V above its setting. A DUT signal above the I/O voltage lifts pin 21
    through the pull resistors (5 V signals with the pod set to 1.8 or 3.3 V). Select 5 V, or
    remove the signal.
  - `low`: more than 0.3 V below its setting: something on pin 21 draws too much current.
  - `short`: under half its setting: pin 21 is shorted or held down. The pull resistors are
    off until it comes back.
  - `unknown`: no reading yet.
- After any bank change the pod waits 30 ms before it drives: the LA clamps' reference follows
  the bank through a filter. The 30 ms is calculated from that filter, to be checked on a board.

> **1.8 V requires a v3 pod.** On v2 boards the TPS2116's `MODE` pin is tied to
> GND, which puts the part in a mode with no manual 1.8 V selection at all: with
> the select line low it passes the *higher* of its two inputs (always 3.3 V), and
> with the select line high it **shuts down** and leaves the bank rail
> unpowered. Firmware before this release called that shutdown state "1.8 V" — it
> never was one. v3 ties `MODE` to +3V3, which enables the real manual selection,
> and the pod now reports its revision in `status.board_rev`. On a v2 pod
> `{"mv": 1800}` returns an error instead of silently cutting the DUT's I/O rail;
> `3300` works on both. The boot default is also different: on v3 the mux comes up
> at **3.3 V**, on v2 it comes up **unpowered**.

> **Pulls at 1.8 V.** On v3 boards before digital v3r4 the LA1-LA6 pull-ups go to the bank rail
> and the LA7/LA8 pull-downs to ground; on digital v3r4 every channel pulls both ways and the
> pull-ups go to J1 pin 21, which follows the I/O voltage (see
> [Pull resistors on digital v3r4](#pull-resistors-on-digital-v3r4)). So pulls work at every
> I/O voltage and engaged pulls stay on when the bank switches (firmware with the `la_pull_1v8`
> feature; older firmware refused pulls at 1.8 V and released them on the switch to 1.8 V). The
> pre-production v2 boards had their pull-ups on 3.3 V and cannot run the bank at 1.8 V anyway.

#### Error cases

| Condition | Message |
|---|---|
| `mv` not 1800 or 3300 | `"la voltage must be 1800 or 3300 (mv)"` (digital v3r4: `"la voltage must be 1800, 3300 or 5000 (mv)"`) |
| `mv: 5000` and pin 21 stayed below 4.3 V (digital v3r4) | `BP0028`: `"5 V did not come up on pin 21 (4120 mV); check the USB supply"` |
| `mv: 1800` on a v2 pod | `"1.8 V needs a v3 pod; this board is v2 (its TPS2116 has no 1.8 V setting)"` |
| any LA op before a voltage is set | `BP0001` (`la_voltage_not_set`) |
| a **change** while any LA pin has a function | `BP0037` (`la_voltage_pins_in_use`, args `pins`: `"LA3 (uart_rx), LA4 (uart_tx)"`) |

> **Not while pins are in use.** Switching the bank glitches every LA line, so a change is refused while any pin is owned by a function other than
> `none` (see [pin ownership](#la-pin-ownership)); the message names each pin and its function.
> Re-setting the voltage that is already selected is always allowed. The console `la_voltage`
> command applies the same rule.

---

### `nrst` — drive the target's reset line

**v3 pods only.** rev3 gave the pod a dedicated reset output: `/NRST_CONTROL` on
**pin-header J1 pin 22**, driven from the MCU through a 330 Ω series resistor and
pulled up on the board by 10 kΩ to `LA_VCCIO` — i.e. to whatever the LA bank is
switched to (1.8 V or 3.3 V), so it matches the DUT's own logic level.

Before rev3 there was no such pin: nRESET had to borrow a logic-analyzer channel,
which cost a channel and had to be named on every `dap_start`. **That is gone** —
`dap_start` no longer takes an `nreset` field, and clients no longer choose a
channel. Wire the DUT's reset to J1 pin 22 and it works everywhere.

`nreset` survives one level up, in the tooling, as a **yes/no** — `benchpod flash
--nreset`, the SDK's `flash(..., nreset=True)`, the flash dialog's checkbox — where
it now means "the target's reset is wired to that pin", and turns on
connect-under-reset. The pod itself is never told: it drives the pin whenever
CMSIS-DAP asks.

The pin is driven **open-drain**: asserted it pulls the DUT's reset net low
(~105 mV), released it is genuinely high-impedance and the board's pull-up (or the
DUT's own reset circuit) sets the level. It never drives high, so a 1.8 V DUT is
never pushed above its own rail.

```json
{"cmd": "nrst", "assert": true}   // hold the target in reset
{"cmd": "nrst", "assert": false}  // release
{"cmd": "nrst", "pulse_ms": 50}   // assert, wait 50 ms, release
{"cmd": "nrst"}                   // query
{"status": "ok", "data": {"supported": true, "asserted": false}}
```

| Field | Type | Required | Description |
|---|---|---|---|
| `assert` | integer | no | `1` holds reset asserted (low), `0` releases it. |
| `pulse_ms` | integer | no | Assert, hold for this many milliseconds, then release. Clamped to 1–1000. Takes precedence over `assert`. |

**You do not need this to flash.** `dap_start`'s CMSIS-DAP `SWJ_PINS` command
drives the same pin, so an OpenOCD/pyOCD *connect-under-reset* works with no extra
call. `nrst` is for test steps that reset the DUT outside a debug session — e.g.
power-on-reset behaviour, or recovering a target that wedged itself.

#### Error cases

| Condition | Message |
|---|---|
| pod is a v2 board | `"nrst pin needs a v3 pod"` |
| `pulse_ms` ≤ 0 | `"pulse_ms must be > 0"` |

---

### `usb_host` — USB host mode

Firmware `next` and later (feature `usb_host_mode`; older firmware answers `usb_host` as an
unknown command, except development builds made with `USB_HOST=1`). The pod's USB port becomes a
full-speed USB host for the board under test, for a limited time, and reports the device that
enumerates on it: IDs, strings, speed, how long enumeration took, the configuration and its
interfaces. [`usb_flash`](#usb_flash--flash-a-board-over-usb) flashes the board over it.

Which port:

| Pod | Port | How the board connects |
|---|---|---|
| digital v3r4 and later | `usb_a` (the default) | its normal USB cable in the pod's USB-A host port |
| older pods only (no USB-A host port) | `usb_c` | the pod's own USB-C connector, through a breakout: D+, D- and GND to the board, 5 V from a supply into the pod's VBUS (the pod gives the board no VBUS: a board powered from USB gets the same 5 V) |

`status` `board_rev` tells them apart (`"v3r4"` has the USB-A host port).

```json
{"cmd": "usb_host", "enabled": true, "port": "usb_c", "timeout_s": 120}
{"cmd": "usb_host"}
{"status": "ok", "data": {
  "mode": "host", "port": "usb_c", "remaining_s": 112, "host_ms": 8011,
  "enumerations": 1, "attached": true, "enumeration": "done", "connected": true, "line_speed": "full",
  "device": {
    "vid": 11914, "pid": 3, "speed": "full", "usb": "2.10", "release": "1.00",
    "class": 239, "subclass": 2, "protocol": 1, "ep0_size": 64, "configurations": 1,
    "manufacturer": "Raspberry Pi", "product": "Pico", "serial": "E6614C311B7A8B22",
    "descriptor_ms": 182, "mounted_ms": 231,
    "config": {
      "value": 1, "max_power_ma": 100, "self_powered": false, "remote_wakeup": false,
      "interface_count": 2, "truncated": false, "malformed": false,
      "interfaces": [
        {"number": 0, "alt": 0, "class": 2, "class_name": "cdc", "subclass": 2, "protocol": 0, "endpoints": 1},
        {"number": 1, "alt": 0, "class": 10, "class_name": "cdc_data", "subclass": 0, "protocol": 0, "endpoints": 2}
      ]
    }
  }
}}
{"cmd": "usb_host", "enabled": false}
```

| Argument | Description |
|---|---|
| `enabled` | `true` starts host mode (or renews its timeout when it already runs on that port), `false` stops it. Without it the command only reads the state. |
| `port` | `"usb_a"`: the USB-A host port, on pods that have one (the default there). `"usb_c"`: the pod's own USB-C connector, on any pod, for a bench rig whose adapter cable powers the pod and carries D+/D- to the board. Required on a pod without a USB-A host port. |
| `timeout_s` | 1 to 3600, default 60. When it runs out the pod goes back to being a USB device by itself, so a client that disappears never leaves it off USB. |

| Field | Description |
|---|---|
| `mode` | `"device"` (normal), `"starting"`, `"host"` or `"stopping"`. The switch happens right after the reply: poll until `"host"`. |
| `port` | The port host mode runs on, `"none"` in device mode. |
| `remaining_s` | Seconds until the timeout. |
| `host_ms` | Time in host mode so far. |
| `enumerations` | Devices that enumerated since host mode started (a replug counts again). |
| `attached` | A device is enumerated now. |
| `enumeration` | How far the current device got: `none` (nothing since host start or the last detach), `running` (its device descriptor arrived), `done` (enumerated), `stuck` (no enumeration 2 s after its device descriptor; the pod reads configuration descriptors of up to 1024 bytes, and a longer one stops here). A stuck device is reported until the next device sends its descriptor. |
| `connected` | Host mode: a device's pull-up is on the port, before any enumeration. `false` while a board draws power but never shows up means its USB lines do not reach the pod. |
| `line_speed` | `"full"` or `"low"`: the line the device pulls up (D+ or D-); `"none"` when nothing is connected. `"low"` from a full-speed board means D+ and D- are swapped. |
| `device` | The device that enumerated last; kept after it detaches (`attached` false) until the next one. |
| `device.vid`, `device.pid` | Vendor and product ID. |
| `device.speed` | `"full"` or `"low"`. |
| `device.usb`, `device.release` | `bcdUSB` and `bcdDevice` as `"major.minor"`. |
| `device.class`, `subclass`, `protocol` | From the device descriptor (0 when the interfaces say it). |
| `device.manufacturer`, `product`, `serial` | String descriptors in ASCII (other characters read `?`), up to 47 characters; `""` when the device has none. |
| `device.descriptor_ms`, `mounted_ms` | From host start (or the previous detach) to the device descriptor and to the configured device. |
| `device.config` | The first configuration: `max_power_ma` (what the device declares it draws), `self_powered`, `remote_wakeup`, `interface_count` (alternate settings included) and up to 8 `interfaces` with their class numbers and names. `truncated`: the descriptor was longer than the pod reads; `malformed`: a descriptor in it had a bad length, so the list stops there. |

While the pod hosts, it is **off the computer's USB**: no USB console and no USB network
adapter. Start host mode over the network (LAN or Wi-Fi) or the cloud; a request over the
pod's own USB link is refused (BP0076, `usb_host_over_usb`). Other refusals: BP0077
(`usb_host_port_unavailable`, args `port`: no such port on this pod), BP0078 (`usb_host_no_crystal`:
the crystal clock is not running), BP0079 (`usb_host_other_port`, args `port`: host mode already
runs on the other port), BP0006 (`missing_argument`, `port`) on a pod without a USB-A host
port.

### `usb_cc` — USB-C CC-line state

**v3 pods only.** The pod is a USB-C sink with the usual 5.1 kΩ Rd on each CC
line; rev3 added an ADC tap on both. An attached source pulls one of them up
through its Rp, so the voltage is the source's **current advertisement** and the
line that is live gives the **cable orientation**.

```json
{"cmd": "usb_cc"}
{"status": "ok", "data": {
  "supported": true, "cc1_mv": 1682, "cc2_mv": 3,
  "orientation": "cc1", "advertised": "3.0A", "advertised_ma": 3000
}}
```

| Field | Description |
|---|---|
| `supported` | `false` on a v2 pod (no CC taps). |
| `cc1_mv` / `cc2_mv` | Measured CC pin voltages in millivolts. |
| `orientation` | `"cc1"`, `"cc2"`, or `"none"` — which line the source is pulling up. |
| `advertised` | `"none"`, `"default"`, `"1.5A"` or `"3.0A"`. |
| `advertised_ma` | The same as a number: `0`, `500`, `1500` or `3000`. |

Bands are the USB Type-C sink-side vRd thresholds: below 200 mV nothing is
advertised, 200–660 mV is default USB power, 660–1230 mV is 1.5 A at 5 V, and
above 1230 mV is 3.0 A at 5 V.

> A legacy USB-A-to-C cable or a plain 5 V supply presents **no Rp at all**: both
> CC lines read ~0 mV and this reports `"none"` even though VBUS is live. That is
> correct — there is no advertisement to read. Treat `"none"` as "unknown, assume
> the minimum", not as "not powered".

This is **report-only**: nothing in the firmware gates on it. It is there so a
host can decide whether the upstream supply can carry what it is about to switch
on — the internal 5 V eFuse (`target_power` eFuse 1) feeds both the screw
terminal and, on v3, the new USB-A output socket.

#### Budget accounting

USB-C VBUS is the **only** source of the pod's `+5V` rail (through input eFuse
U57), and that rail feeds both the pod's own electronics *and* eFuse 1 — the
internal DUT rail that `power_status` measures. So the advertised budget has to
cover the internal rail's draw, and

```
headroom  ≤  advertised_ma − power_status.internal.current_ua/1000
```

is an **upper bound**, not a promise: the pod's own consumption comes out of the
same budget and is not measured (there is no shunt on the pod's supply, on any
board revision). The **external** rail (eFuse 2) is fed from screw terminal J4, a
separate supply, so it does not count against the USB-C budget at all.

The web UI renders exactly this on its Power tab (`UsbBudgetPanel`).

#### Error cases

| Condition | Message |
|---|---|
| pod is a v2 board | `"usb cc monitoring needs a v3 pod"` |
| ADC conversion failed on a v3 pod | `"usb cc read failed"` |

---

### `target_power` — enable/disable a target-power eFuse

Switches one of the two target-power eFuses on or off. This is how a host powers
the DUT before flashing/debugging it (the CLI's `flash --target-power`, for
example, enables an eFuse, lets the target boot, then connects over SWD).

| eFuse | Rail |
|---|---|
| `1` | internal 5 V |
| `2` | external 5 V..20 V |

State is **not** persisted: after a reset both eFuses return to off.

#### Request

```json
{"cmd":"target_power","efuse":<1|2>,"enabled":<true|false>,"delay_ms":<ms>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `efuse` | integer | yes | — | Which eFuse to switch: `1` (internal 5 V) or `2` (external). |
| `enabled` | boolean | yes | — | `true` switches the eFuse on, `false` off. Firmware up to 3.10.13 took `"state":1\|0`. |
| `delay_ms` | integer | no | `0` | Apply the change after this many milliseconds. `0` = immediately. The command **acks right away** and the eFuse switches later from the pod's poll loop — so the connection is free to do something else meanwhile (e.g. enter `uart_proxy_start` to capture the target's boot output as it powers up). A new `target_power` for the same eFuse replaces a still-pending delayed change. |
| `profile` | string | no | — | The wiring profile the board is powered on with (a name from `wiring_list`). The pod keeps it while the rail is on and `target_status` reports it, so every tool shows which wiring the DUT runs with, whoever turned it on. The tools set the profile's I/O voltage first; the pod does not. Ignored with `enabled` `false`. A bad name is refused with `BP0008`. |

#### Response

```json
{"status":"ok","data":{"efuse":1,"enabled":true,"delay_ms":0}}
```

| Field | Type | Description |
|---|---|---|
| `efuse` | integer | The eFuse that was switched (echoes the request). |
| `enabled` | boolean | The **requested** state (firmware up to 3.10.13: `1`/`0`). With `delay_ms` > 0 this is the state the eFuse will reach once the delay elapses. |
| `delay_ms` | integer | The delay applied (echoes the request; `0` when immediate). |

#### Error cases

| Condition | Message |
|---|---|
| `efuse` field missing | `"missing efuse"` |
| `enabled` missing or not `true`/`false` | `"enabled must be true or false"` |
| `efuse` not 1 or 2 | `"invalid efuse"` |

> **Scheduled power-on + UART capture.** `uart_proxy_start` turns the connection
> into a raw byte stream, so that connection cannot send a power-on *while*
> capturing. Schedule it instead (or send it from a second connection):
> `target_power` with `delay_ms` → `uart_proxy_start` → the eFuse switches mid-
> capture and you catch the boot banner.

> Read the eFuses' fault/valid status with `target_status`. (Firmware before 3.9.0 also had
> `power` and `pstat` on the USB console; a computer on the USB cable now uses these JSON
> commands over the [USB network adapter](usb-serial-interface.md#usb-network-adapter).)

---

### `target_status` — read target-power eFuse state

Reports the live enable/fault/valid state of both eFuses. On the STM32H563 pod both
eFuses' `V` (power-good) and `FLT` (fault) lines are wired to the TCA9554 power expander
at I2C `0x22`, so `status_supported` is always `true`.

#### Request

```json
{"cmd":"target_status"}
```

No parameters.

#### Response

```json
{"status":"ok","data":{"efuse1":{"enabled":true,"fault":false,"valid":true,"flt_pin":0,"reverse_blocks":0,"profile":null},"efuse2":{"enabled":false,"fault":false,"valid":false,"flt_pin":0,"reverse_blocks":0,"profile":null},"status_supported":true}}
```

| Field | Type | Description |
|---|---|---|
| `efuseN.enabled` | boolean | `true` if the eFuse EN output is currently driven on (firmware up to 3.10.13: `1`/`0`, as `fault` and `valid`). |
| `efuseN.fault` | boolean | `true` while the eFuse has a confirmed fault: `FLT` held for 30 ms and not reverse-current blocking (below). Firmware before 3.10.1: the `FLT` pin as read. |
| `efuseN.flt_pin` | integer | `1` if the `FLT` pin is asserted right now (firmware 3.10.1+). |
| `efuseN.reverse_blocks` | integer | How many times since boot a held `FLT` was reverse-current blocking, not a fault (firmware 3.10.1+). |
| `efuseN.valid` | boolean | `true` once the eFuse's input supply is valid and its inrush is done (the TPS259470A's `AUXOFF` pin). It is not output power good: no output fault takes it low. |
| `status_supported` | boolean | `true` — `V`/`FLT` are wired through the power expander. |

The TPS259470A pulls `FLT` low for real faults and also while it blocks reverse current: when
its output is more than 29 mV above its input it turns off its blocking FET, and turns it back on
only once the input is 104 mV above the output. With a light load, a dip of the pod's 5 V (a Wi-Fi
bridge on the USB-A port transmitting, for example) can hold `FLT` low for tens of ms while the
output stays up. So a held `FLT` (firmware 3.10.1+) is a fault only when the output's current
monitor shows the output collapsed (below 1 V) or carrying current (over 250 mA, the limits are
about 2 A and 3 A); otherwise it counts in `reverse_blocks`.

`FLT` is also sampled every 10 ms in the background, so a trip is reported asynchronously
(and over the cloud as an `efuse.event` frame) without polling this command. The eFuses
auto-retry after ~110 ms, and `FLT` stays asserted through the retry, so the sampling
interval never misses a trip.

---

From firmware 3.10.8 each eFuse object also has `"profile"`: the wiring profile named
when that rail was switched on (`target_power` `profile`), or `null` when it is off or none was
named.

### `power_status` — one-shot rail measurement

Reads both on-board INA238 current monitors (`0x40` = internal 5 V rail, `0x44` = external
rail) and reports each rail's bus voltage and current. Boards with the pod's own current monitor
(`0x41`, capability `pod_current`) add `pod`: what the pod itself draws from its 5 V input. This is the cheap "what is the DUT
drawing right now" call; for a waveform over time use
[`power_profile`](#power_profile--record-a-rail-current-profile).

#### Request

```json
{"cmd":"power_status"}
```

No parameters. (Treated as a high-frequency poll: it is not traced on the console.)

#### Response

```json
{"status":"ok","data":{"internal":{"ok":true,"bus_mv":5008,"current_ua":142300},
                       "external":{"ok":false,"bus_mv":0,"current_ua":0},
                       "pod":{"ok":true,"bus_mv":5012,"current_ua":231000,"total_ua":373300}}}
```

| Field | Type | Description |
|---|---|---|
| `<rail>.ok` | boolean | `false` when that INA238 did not answer (an unpopulated / unpowered rail). |
| `<rail>.bus_mv` | integer | Bus (DUT-side) voltage in mV. |
| `<rail>.current_ua` | integer | Current in µA, computed from the measured shunt voltage and the known shunt resistance — signed, so a back-feeding DUT reads negative. |
| `pod` | object | Only on boards with the pod's own monitor (`0x41`): `ok`, `bus_mv` and `current_ua` of the pod's own 5 V draw (the DUT's internal rail bypasses that shunt), and `total_ua` = pod + internal rail, which is what the USB-C input delivers (compare it with `status.usb.source_ma`). `total_ua` is left out when either read failed. The console `ina` line `pod(0x41)` (development builds) shows the same reading. |

Both reads are taken under the hardware lock that serialises this I2C bus against the console
`ina` command, the eFuse expander and the profile sampler.

---

<a id="power-profile"></a>
### `power_profile` — record a rail current profile

Samples one eFuse rail's INA238 continuously and returns the statistics (and optionally the
waveform) of the window. This is what turns "the board browns out sometimes" into a number:
inrush at power-on, sleep-current floors, the energy of one duty cycle.

#### Rail limits (rev3 hardware)

| | eFuse 1 — internal | eFuse 2 — external |
|---|---|---|
| Source | 5 V from USB-C | J4 terminal, 5–20 V |
| INA238 | `0x40`, 50 mΩ shunt | `0x44`, 30 mΩ shunt |
| Current LSB | 100 µA | 167 µA |
| Full scale | ±3.28 A | ±5.46 A |
| eFuse current limit (TPS259470A) | ≈ 2.0 A | ≈ 3.0 A |
| Over-voltage lockout | 5.7 V | ≈ 21.8 V |

The eFuses auto-retry after ~110 ms and blank faults for ~2.8 ms, so **a fast trip (< 1 ms)
will not appear in a ~1 kHz profile** — `stats.fault` is the reliable indicator that one
happened. `VBUS` is measured on the **DUT side** of the eFuse. (The ADC `current_in` input is a
4–20 mA terminal, 249 Ω or 100 Ω from analog module v3r5, *not* the DUT supply — do not use it for this.)

#### Start

```json
{"cmd":"power_profile","action":"start","efuse":1,"sample_rate_hz":1000,"max_duration_ms":60000,"keep_samples":2048}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `efuse` | `1` or `2` | no | `1` | Which rail to sample. |
| `sample_rate_hz` | integer | no | `500` | 100..500, whole numbers only. Accurate to ~200 Hz; above that the sampler (one register read per worker pass) beats against the pass interval and flattens near 365 Hz. `stats.sample_rate_hz` reports what was delivered, `stats.adc_sample_rate_hz` what the sensor was configured for. |
| `max_duration_ms` | integer | no | `60000` | 1..600000. The sampler stops by itself at the limit and marks the result `truncated`. |
| `keep_samples` | integer | no | `0` | 0..4096 samples to keep for the reply. `0` = statistics only. |

```json
{"status":"ok","data":{"started":true,"efuse":1,"sample_rate_hz":907}}
```

The INA238 is put into continuous shunt+bus mode with a conversion time × averaging that makes
one conversion period ≈ the sample period (gap-free). The hw worker reads **one** register per
pass — a shunt value every period, a bus voltage every ~10 ms, the last bus value held in
between — so the worker is never blocked. `ADC_CONFIG` is restored when the profile ends.

Statistics run over **every** raw sample. The samples kept for the reply are **bin-averaged**
down to `keep_samples`: when the store fills, neighbouring entries are averaged in pairs and
the bin size doubles, so every entry averages the same number of raw samples.

A `target_power` with `delay_ms` may fire while a profile is running — that is how you capture
inrush: start the profile, then schedule the eFuse on.

#### Stop and read the result

```json
{"cmd":"power_profile","action":"stop"}
```

Stops the sampler (if running) and returns the result using the same chunked framing as
`la_capture` — `"more":true` on every packet but the last:

```json
{"status":"ok","t_us":[0,1102,2204],"current_ua":[830,142300,141900],"bus_mv":[5008,4990,4992],"more":true}
{"status":"chunk","t_us":[…],"current_ua":[…],"bus_mv":[…],
 "stats":{"efuse":1,"sample_rate_hz":364,"adc_sample_rate_hz":907,"n":3640,"duration_ms":10000,
          "avg_ua":141200,"min_ua":830,"peak_ua":612400,
          "avg_mv":4995,"min_mv":4870,"max_mv":5012,
          "energy_uj":7052000,"charge_uc":1412000,
          "fault":false,"truncated":false},"more":false}
```

| Field | Meaning |
|---|---|
| `t_us` | Start of each bin, µs since the profile started. |
| `current_ua` / `bus_mv` | Bin-averaged current (µA, signed) and bus voltage (mV). |
| `stats.sample_rate_hz` | Samples per second actually **delivered** (`n` over `duration_ms`, measured). |
| `stats.adc_sample_rate_hz` | Conversion rate the current monitor was configured for — the ceiling `sample_rate_hz` works towards, not what arrived. |
| `stats.n` | Raw samples taken (not the number returned). |
| `stats.energy_uj` / `charge_uc` | ∫I·V·dt in µJ and ∫I·dt in µC over every raw sample. |
| `stats.fault` | An eFuse fault was seen at any point in the window. |
| `stats.truncated` | The sampler stopped itself at `max_duration_ms`. |

The result is kept until it is read or the next `start`. Errors: `"no power profile to stop"`
when none ran.

#### Status

```json
{"cmd":"power_profile","action":"status"}
{"status":"ok","data":{"running":true,"efuse":1,"elapsed_ms":4213,"n":3821}}
```

#### One-shot

Omit `action` and give `duration_ms`: the pod starts, waits (without blocking the worker) and
then sends the same reply `stop` would. Needs a streaming connection.

```json
{"cmd":"power_profile","efuse":1,"duration_ms":2000,"sample_rate_hz":500,"keep_samples":1024}
```

#### Error cases

| Condition | Message |
|---|---|
| a profile is already running | `"power profile already running on efuse 1"` |
| `efuse` not 1 or 2 | `"efuse must be 1 or 2"` |
| `sample_rate_hz` out of range | `"sample_rate_hz must be 100..500"` |
| `max_duration_ms` / `duration_ms` out of range | `"max_duration_ms must be 1..600000"` / `"duration_ms must be 1..600000"` |
| `keep_samples` > 4096 | `"keep_samples must be 0..4096"` |
| the rail's INA238 does not answer | `"power profile: the INA238 for efuse 1 (I2C 0x40) is not responding"` |
| not enough heap for `keep_samples` | `"not enough memory to keep N samples (M bytes); ask for fewer keep_samples"` |
| `action` not recognised | `"action must be start, stop or status"` |
| no `action` and no `duration_ms` | `"power_profile needs an action (start, stop, status) or duration_ms for a one-shot"` |
| `stop` with nothing to return | `"no power profile to stop"` |
| kept samples requested over the cloud **command** channel | `"the power profile kept N samples; read them over a stream connection (LAN or tunnel), or start with keep_samples 0"` |
| a one-shot over the cloud command channel | `"a one-shot power_profile waits for its result, which needs a stream connection; over the cloud command channel use action start, then stop"` |

The chunked result needs a streaming connection (LAN, or a cloud **tunnel**); the single-reply
cloud command channel can still `start`, `status` and `stop` a `keep_samples: 0` profile.

The [features table](#features-table) has `power_profile`.

---

### `uart_proxy_start` — transparent UART bridge

Bridges a DUT UART through two LA channels, turning the connection into a serial-over-TCP pipe (the network equivalent of the console's `uart-proxy`). The iCE40 gateware runs a soft UART on the chosen channels (**8N1** by default; 8E1, 8O1, 8N2 and the other parity and stop-bit combinations with gateware v56+). The LA pins run at the I/O voltage: 1.8 V or 3.3 V, and on digital v3r4 also 5 V (feature `la_5v`, see [5 V I/O](https://embeddedci.com/docs/benchpod-targets/5v-rs485#5v-io)). On older boards a 5 V UART needs a level shifter between the pod and the device. An RS-485 bus (Modbus RTU) needs an external RS-485 transceiver between the pod and the bus. The MCU forwards bytes both ways over SPI.

Like `dap_start`, this is **connection-scoped** and changes how the connection's bytes are interpreted:

1. Send `uart_proxy_start` as a normal JSON command. The pod arms the UART and replies `{"status":"ok","data":"uart ready"}` **while still in JSON mode**.
2. **Immediately after the ack the same TCP connection becomes a transparent byte stream:** everything you send is transmitted to the DUT, and everything the DUT sends is delivered back to you (asynchronously — no polling).

#### Request

```json
{"cmd":"uart_proxy_start","rx":<1-14>,"tx":<1-14>,"baud":115200,"parity":"none","stop_bits":1,
 "half_duplex":false,"entry":{"reset_ms":10,"tx_low":true,"hold_ms":10,"idle_ms":10,"hex":"00"}}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `rx` | integer | yes | — | LA channel the FPGA samples (wire the DUT's TX here). |
| `tx` | integer | yes | — | LA channel the FPGA drives (wire the DUT's RX here); distinct from `rx`, or the same channel with `half_duplex`. |
| `baud` | integer | no | 115200 | Line rate; the firmware computes the FPGA bit-period divisor. ≤115200 solid, higher best-effort. |
| `parity` | string | no | `"none"` | `"none"`, `"even"` or `"odd"`: a parity bit after the 8 data bits, sent and checked. A received frame with a wrong parity bit is still delivered, and `status` reports `uart_rx_parity_error: true` until the next session. Gateware v56+. |
| `stop_bits` | integer | no | `1` | `1` or `2`. The pod sends this many; when receiving it checks the first one, as most UARTs do. Gateware v56+. |
| `half_duplex` | boolean | no | `false` | A one-wire UART: `rx` and `tx` are the same LA channel. The pod only pulls the line low and lets the pull-up hold it high, so the device can answer on the same wire, and it does not read back its own bytes. Use LA1 to LA6 (the pod's pull-up is turned on for the session) or a pull-up on the board. Gateware v57+ (`uart_hd` in the [features table](#features-table)). |
| `de` | integer | no | none | An RS-485 transceiver's DE (tied to /RE) on this LA channel. The pod drives it high from the moment it has a byte to send, before the start bit, to the end of the last stop bit, and low otherwise, so the transceiver listens whenever the pod is not sending. A pin of its own, not `rx` or `tx`. Gateware v58+ (`uart_de` in the [features table](#features-table)). |
| `de_active_low` | boolean | no | `false` | Drive `de` low while sending, for a transceiver whose enable is active low. |
| `entry` | object | no | none | A boot-mode entry the pod runs before the ack, timed on the pod so no network delay sits between the reset and the first byte. See [boot-mode entry](#boot-mode-entry). Needs the pod's reset pin (`nrst_pin` in `caps`). |

#### Boot-mode entry

Some boot ROMs pick their mode from the level of the UART pin when the chip leaves reset, then
expect a byte soon after. `entry` runs that sequence on the pod:

1. With `tx_low`, drive TX low.
2. Hold the target in reset (the pod's reset pin) for `reset_ms`, then release it.
3. Wait `hold_ms` (TX still low with `tx_low`).
4. Arm the UART: TX is released and the line idles high.
5. Wait `idle_ms`, then send the `hex` bytes.
6. Reply `"uart ready"`; the connection carries raw bytes from here.

| Field | Type | Default | Description |
|---|---|---|---|
| `reset_ms` | integer | `10` | Reset hold, 0 to 1000 ms. |
| `tx_low` | boolean | `true` | Hold TX low across the reset release. `false` leaves TX idle high (a plain reset followed by a timed write). |
| `hold_ms` | integer | `10` | Time after the reset release before TX is released, 0 to 1000 ms. |
| `idle_ms` | integer | `10` | Time from TX release to the first byte, 0 to 1000 ms. |
| `hex` | string | none | Bytes sent right after `idle_ms`, as hex, at most 32 bytes. |

`{"entry":{}}` runs the sequence with the defaults and sends nothing. The target's own timing
requirements come from its datasheet; the pod's waits are in whole milliseconds.

#### Response

```json
{"status":"ok","data":"uart ready"}
```

After this line, the byte stream is raw UART (no more JSON until the connection returns to JSON mode, below).

#### Leaving the mode

There is no single "escape key" for a network client, so the proxy ends on either:

- **Closing the socket** (primary, recommended) — the pod auto-disarms the UART and releases the TX channel to high-Z.
- A **guard-timed `+++` escape** (Hayes convention): ≥1 s idle, then `+++`, then ≥1 s idle with no other bytes → the connection returns to JSON mode on the same socket. (The `+++` is also passed through to the DUT.)

There is **no inactivity timeout** — an idle serial console stays open.

#### Error cases

| Condition | `message` |
|---|---|
| `rx`/`tx` field missing | `"missing rx"` / `"missing tx"` |
| an LA channel is outside 1..14 | `"invalid uart args"` |
| `rx` and `tx` are the same channel without `half_duplex` | code `BP0018` |
| `half_duplex` with two different channels | code `BP0019` |
| `half_duplex` on gateware older than v57 | code `BP0017` |
| `entry` on a pod without the reset pin | code `BP0020` |
| an `entry` time out of range, or `hex` not hex or longer than 32 bytes | code `BP0021` |
| `de` is the `rx` or `tx` channel | code `BP0022` |
| `de` on gateware older than v58 | code `BP0017` |
| `parity` or `stop_bits` has another value | `"invalid uart frame: parity is none, even or odd; stop_bits is 1 or 2"` |
| parity or two stop bits on gateware older than v56 | `BP0017` (feature `parity and stop_bits`, min 56) |
| a UART proxy is already active (console or another connection) | `BP0049` (`uart_session_open`) |
| `rx` or `tx` belongs to another function | `BP0031` (see [pin ownership](#la-pin-ownership)) |
| `rx`/`tx` on a channel with its pull-**down** engaged (LA7 or LA8 before digital v3r4) | `BP0032`: the UART idles high |
| the iCE40 does not answer after arming | `"uart proxy failed: FPGA not responding (check gateware)"` |

`rx` and `tx` are **claimed** for the session (functions `uart_rx` / `uart_tx`, or `uart_hd` for the one pin of a half-duplex UART, and `uart_de` for `de`) and released
when the proxy ends, so nothing else can be armed onto them meanwhile.

> **RX pull hold.** The proxy holds the RX line high for the session so a DUT's floating TX
> pin does not decode as a flood of `0x00` before its firmware brings the UART up. On digital
> v3r4 this works on every channel (a 10k pull-up to J1 pin 21). On older boards it only
> happens on **LA1-LA6**, whose resistors pull **up** (to the bank rail, so at 1.8 V too;
> firmware before `la_pull_1v8` skipped it at a 1.8 V bank): LA7/LA8 pull *down* (engaging theirs
> would guarantee the very flood it prevents) and LA9-LA14 have no resistor. On the digital
> v3r4 board every channel pulls up (10k, 2.2k on a one-wire line). The previous state
> is restored when the proxy ends, and a failure to restore it is logged on the console.

---

## SPI master (flash an SPI device)

Gateware v45+ (`status` caps `spi_master`; `spi_stream` with it on firmware that has the command).  The iCE40 engine that runs SWD has a second job:
an SPI master on any four LA pins, used to read and program SPI NOR flash (W25Q, MX25, GD25,
IS25 and other 25-series parts) or to talk to any other SPI device.  One engine, so an SPI
session and an SWD session (`dap_start`) exclude each other.

Hold the DUT in reset while you flash its SPI flash (`{"cmd":"nrst","assert":true}`) so its own
controller does not drive the same wires, and release it afterwards.

Every command and reply fits one cloud frame, so all of this also works over the cloud
command channel.

### `spi_start` — claim four pins for SPI

```json
{"cmd":"spi_start","sck":3,"mosi":4,"miso":5,"cs":6,"clock_hz":1000000,"mode":0}
```

| Field | Meaning |
|---|---|
| `sck`, `mosi`, `miso`, `cs` | LA pins 1..14, all different |
| `clock_hz` | SCK rate in Hz, default 1000000.  The pod picks the nearest rate at or below it from 24 MHz / (2 * n), n = 2..63: 6 MHz down to 190 kHz |
| `mode` | 0 (default), 1, 2 or 3 (CPOL = mode / 2, CPHA = mode % 2).  Modes 1 and 2 need gateware v55 (firmware with I2C master support) |

Reply: `{"sck":3,"mosi":4,"miso":5,"cs":6,"clock_hz":1000000,"mode":0}` with the rate actually used.
CS is driven high (released) until a transfer.  Needs the LA voltage set first.

Errors: `BP0017` (the SPI master needs gateware v45, modes 1 and 2 v55), `BP0004` (an SPI session
is open), `BP0005` (the engine runs an SWD or I2C session), and the pin codes `BP0031` / `BP0032`.

### `spi_stop` / `spi_status`

`{"cmd":"spi_stop"}` releases the four pins (they go back to high-Z).
`{"cmd":"spi_status"}` returns `{"active":false}` or the session's pins, rate, mode and whether
CS is held.

### `spi_xfer` — raw full-duplex transfer

```json
{"cmd":"spi_xfer","tx":"nwAAAA","cs":"release"}
```

`tx` is 1..768 bytes, base64url.  The reply carries the bytes clocked in at the same positions:
`{"rx":"_-9AFw","cs":"released"}`.  CS is asserted before the first byte; `"cs":"hold"` keeps
it asserted for the next `spi_xfer`, so one transaction can span several commands.

### `spi_stream` — send a staged upload in one CS frame

For data too big for `spi_xfer`, such as an FPGA bitstream into its slave-SPI configuration port.
Stage the bytes in PSRAM first with `load_bin` and `"psram":true` (a stream connection), then:

```json
{"cmd":"spi_stream","len":262144,"head":"egAAAA","cs":"release"}
```

| Field | Meaning |
|---|---|
| `len` | bytes of the staged upload to send (0 .. its size; 0 sends only `head`) |
| `head` | optional, up to 64 bytes (base64url) sent first in the same frame, e.g. a command opcode |
| `cs` | `release` (default) or `hold` after the last byte |

CS is asserted once and stays asserted for the whole stream; between 512-byte chunks SCK pauses
(about 3 ms per chunk, so about 1.5 s for 256 KB at 6 MHz).  A held CS from an earlier
`spi_xfer` is released first, so the stream is a frame of its own.  The pod holds the PSRAM bus
and the heavy-operation gate while it runs: send it on the connection that did the upload, or
after that connection closed.  Reply: `{"sent":262144,"ms":1450,"cs":"released"}`.

Errors: `nothing staged: load_bin with "psram":true first`, `len is more than the staged upload`,
`busy` (a capture or another connection owns the gate), `spi_stream: SPI transfer failed after N
bytes`.

Example, an ECP5 (Lattice sysCONFIG slave SPI, write-only), with PROGRAMN and DONE on two more
LA pins as `gpio`: pulse PROGRAMN low, wait 50 ms, `spi_xfer` `xgAAAA` (ISC_ENABLE 0xC6 + 3 zero
bytes), `spi_stream` with `head` `egAAAA` (LSC_BITSTREAM_BURST 0x7A + 3 zero bytes) and `len` the
bitstream size, `spi_xfer` `JgAAAA` (ISC_DISABLE 0x26), then read DONE.

### `spi_flash` — SPI NOR flash operations

Standard 25-series commands with 3-byte addresses (the first 16 MB).

| Request | Reply |
|---|---|
| `{"cmd":"spi_flash","op":"id"}` | `{"id":"ef4017","present":true,"size":8388608,"status":0}` |
| `{"cmd":"spi_flash","op":"read","addr":0,"len":1024}` | `{"addr":0,"len":1024,"data":"<b64url>"}` (len 1..1024) |
| `{"cmd":"spi_flash","op":"erase","addr":0,"len":65536}` | `{"addr":0,"len":65536,"ms":152}` |
| `{"cmd":"spi_flash","op":"write","addr":0,"data":"<b64url>"}` | `{"addr":0,"len":768,"verified":true,"ms":9}` |
| `{"cmd":"spi_flash","op":"chip_erase"}` | `{"ms":21500}` |

- `present` is false when the ID reads all 00 or all FF: nothing is answering on those pins.
- `erase` erases every 4 KB sector the range touches (64 KB block erases where a whole block
  is covered), up to 1 MB per command.  The reply gives the range actually erased.
- `write` programs 1..768 bytes on an already-erased range and reads them back
  (`"verify":false` skips the read-back).  Page boundaries are handled.
- `chip_erase` can take minutes on a large part and can outlast the cloud reply timeout; over
  the cloud prefer `erase` in 1 MB steps.

Errors: `write enable did not stick: no flash answering, or it is write-protected`,
`flash stayed busy (WIP never cleared)`, `verify failed at 0x...: not erased, write-protected,
or a bad wire`, `no SPI session: send spi_start first`.  The pod does not clear block-protect
bits; a part that ships protected needs its status register written with `spi_xfer`.

A whole image: `spi_start`, `nrst` assert, `op:"id"`, `erase` the image's range in 1 MB steps,
`write` it in 768-byte chunks, `nrst` release, `spi_stop`.

## ESP32 targets (flash over the UART ROM bootloader)

`esp_flash` writes an image into an ESP32-family target's flash through the chip's own ROM
bootloader, the way `esptool` does, with no USB cable to the target. ESP32, ESP32-S2, S3, C2, C3,
C6 and H2; the ESP8266 is refused. Firmware 3.10.7 and later. Wiring, at 3.3 V:

| Target | Pod |
|---|---|
| U0TXD (TXD0) | an LA pin, `rx` |
| U0RXD (RXD0) | an LA pin, `tx` |
| IO0 (IO9 on the C3, C6 and C2) | an LA pin, `boot`, driven open-drain |
| EN | the pod's reset pin (`"en":"nrst"`, the default) or an LA pin, open-drain |
| GND | GND |

Stage the image in PSRAM first with `load_bin` and `"psram":true` (a stream connection, as for
`spi_stream`), then start the flash. It runs in the background: `start` and `sync` answer at once
and `status` reports progress. Meanwhile captures, uploads and updates answer busy.

```json
{"cmd":"esp_flash","op":"start","rx":3,"tx":4,"boot":5,"en":"nrst","offset":65536,"len":524288,
 "baud":460800,"flash_size":4194304,"run":true}
{"cmd":"esp_flash","op":"status"}
```

| Field | Meaning |
|---|---|
| `op` | `start` (write), `sync` (enter the loader and read the chip, flash untouched), `status`, `abort` |
| `rx`, `tx`, `boot` | LA pins (1..14), see the wiring above |
| `en` | `"nrst"` (default) or an LA pin |
| `offset` | flash address, a multiple of 4096 (`start` only, default 0) |
| `len` | bytes of the staged upload to write (`start` only) |
| `baud` | rate after the ROM's CHANGE_BAUDRATE, 9600..2000000 (default 460800; the loader starts at 115200) |
| `flash_size` | the target's flash size in bytes, a power of two (default 4 MB); `offset + len` must fit |
| `run` | `true` (default): reset into the new image at the end; `false`: stay in the loader |

Sequence: boot pin low, EN low 100 ms, EN released, 50 ms, SYNC (retried); the boot pin is
released once the ROM answers. Then CHANGE_BAUDRATE, SPI_ATTACH, SPI_SET_PARAMS, FLASH_BEGIN
(the ROM erases the region, seconds for a large image), the data in 1 KB blocks, the chip's MD5
of the region, FLASH_END. With `run` the pod pulses EN with the boot pin released. Every pin is
released at the end, whatever the result. With firmware 4.0.0+ the pod also holds `rx`
high with its pull-up for the run, where the channel has one (as the UART proxy does, see
[RX pull hold](#uart_proxy_start--transparent-uart-bridge)), and puts back what the channel did
before at the end. Write each region of a multi-file image (bootloader, partition table,
application) with its own `start`, `run` only on the last.

`start` and `sync` reply `{"started":true,"op":"start"}`. `status` (and `abort`) reply:

```json
{"running":false,"op":"start","phase":"done","done":512,"total":512,"chip":"ESP32-S3",
 "magic":"0x00000009","offset":65536,"len":524288,"baud":460800,"resends":0,"ms":21450,
 "md5":"9e107d9d372bb6826bd81d3542a419d6","result":"ok"}
```

`phase` is `enter`, `sync`, `detect`, `baud`, `attach`, `params`, `erase`, `write`, `verify`, `end`,
`reset` or `done` (after a failure: the step it failed in); `done`/`total` count 1 KB blocks. `result` is `null` while running, then `ok`
(the chip's MD5 matched), `unverified` (every block acknowledged, no MD5 came back), or the
failure: `no answer from the ROM loader` (wiring, straps, or the target is not powered),
`FLASH_BEGIN failed`, `a block was not acknowledged`, `MD5 mismatch`, `chip not supported
(ESP8266)`, `aborted`.

Errors at `start`: `nothing staged: load_bin with "psram":true first`, `len is more than the
staged upload`, `BP0047` (`pod_busy`, also while an ESP32 flash runs), `BP0049` (a UART
session is open), `BP0031` (a pin conflict), `this pod has no reset pin: wire EN to an LA pin and pass en`.

## I2C master

The pod is an I2C master on any two LA pins (firmware with gateware v55 and later): read and write
registers of a chip on the DUT's board, or scan the bus. The pins are open drain, so the bus needs
pull-ups. SWD, SPI and I2C sessions exclude each other: they share one engine in the FPGA. Every
command and reply fits one cloud frame.

### `i2c_start` — claim two pins as an I2C master

```json
{"cmd":"i2c_start","sda":1,"scl":2,"clock_hz":400000,"pullups":"pod"}
```

| Field | Meaning |
|---|---|
| `sda`, `scl` | LA pins 1..14, different |
| `clock_hz` | 10000..1000000 (Hz), default 100000. 100 kHz runs at 100 kHz, 400 kHz at about 364 kHz, 1 MHz at about 923 kHz: each speed keeps SCL low as long as its I2C mode needs. The pod counts SCL high only once SCL reads high, which lets a target stretch the clock, so the SCL rise time adds to each period: the bus never runs faster than the reply's `hz`, and somewhat slower with slow edges (with the pod's 4.7 kOhm pull-ups on a short cable, 400 kHz measured about 333 kHz and 1 MHz about 774 kHz). Stronger pull-ups bring it closer |
| `pullups` | `pod` (default): both pins need a pod pull-up (LA1..LA6 on today's boards) switched on (`{"cmd":"la_pull","la":1,"pull":"up"}`); they go to the LA bank rail, so they work at 1.8 V and 3.3 V (firmware before `la_pull_1v8`: 3.3 V only). `external`: the board has its own pull-ups |

The pod never switches a pull-up by itself; the error says which command does.
Reply: `{"sda":1,"scl":2,"clock_hz":363636,"pullups":"pod"}` with the rate actually used.

Errors: `BP0017` (the I2C master needs gateware v55), `BP0061` (`i2c_needs_pullups`: switch on
the pull-up, or pass `"pullups":"external"`), `BP0029` (pin 21 is down, so the pod's pull-ups are
off), `LA7 has no pod pull-up on this board...`, `BP0003` (an I2C session is open), `BP0005` (the
engine runs an SWD or SPI session), and the pin codes `BP0031` / `BP0032`.

### `i2c_stop` / `i2c_status`

`{"cmd":"i2c_stop"}` releases both pins. `{"cmd":"i2c_status"}` returns `{"active":false}` or the
session's pins, rate and pull-ups.

### `i2c_xfer` — write, read, or write then read

```json
{"cmd":"i2c_xfer","addr":72,"write":"Dw","read":2}
```

| Field | Meaning |
|---|---|
| `addr` | the 7-bit address, 0..127 |
| `write` | 0..128 bytes, base64url, sent after the address |
| `read` | 0..248 bytes to read; after a write, a repeated START comes first. The last byte is NACKed |

With neither, the pod sends the address alone (a probe). Reply:
`{"addr":72,"ack":true,"rx":"ARc"}`. When a byte is not acknowledged, `ack` is false and `nack_at`
says which: 0 the address, 1..N the Nth written byte, N+1 the address of the read. `rx` is empty
when the device did not answer the read.

Errors: `BP0062` (`i2c_session_not_open`), `BP0063` (`i2c_engine_reset`: a gateware reload
ended the session), `SCL held low: a target stretched the clock for
over 200 ms, or the bus is stuck; the pod released the bus`, `arbitration lost: another master
drove SDA low; the pod released the bus`.

### `i2c_scan` — which addresses answer

`{"cmd":"i2c_scan"}` probes 0x08..0x77 (`from` and `to` change the range) and returns
`{"from":8,"to":119,"found":[72,118]}`.

## Emulated sensors

The pod can **play sensors** for the DUT: I2C, SPI and UART sensors on LA channels, and analog
sensors on the analog outputs. What a sensor does is not built into the firmware: the host sends a
**sensor definition** (the format is documented in the public sensor library,
`embeddedci-com/benchpod-sensors`, `docs/format.md`), the pod compiles it and plays it. The pod
never stores definitions; they are gone after `sensor_stop`, a reboot or a firmware update.

A definition is JSON (clients convert YAML). **Key order matters**: `constants`, `functions`,
`fragments` and `let` refer to earlier entries, and the pod reads them in the order they appear
in the text. A converter must keep the source order of every object.

Up to **4 I2C sensors** (on one bus or on several), **1 SPI sensor**, **1 UART sensor** and **2
analog sensors** (one on the DAC voltage output, one on the 4-20 mA output) run at the same time.
Each one is an **instance** with a name the host picks: 1 to 16 characters of `a-z`, `0-9` and
`_` (usually the definition's `name`; the limit keeps `la_pins` and `sensor_status` inside the
cloud reply cap). At most 8 instances are defined at once.

Feature `sensors` in the [features table](#features-table) (definition format 1). `status` and
the cloud capabilities frame carry `"sensors_digital"` (the emulation image is installed or
running, so I2C, SPI and UART sensors work). Firmware up to 3.10.13 also sent `"sensors_format":1`
and, in the frame, `"sensors":true`. These commands replace the single-sensor `sensor_*`, `sensor_types` and `gps_*` commands
of firmware 3.10.1 and earlier (see [Removed in firmware 3.10.3](#removed-in-firmware-3103)).

Typical flow:

1. `sensor_define` (one or more chunks) for each instance.
2. `sensor_start` with its pins (`la_voltage` first, and pull-ups on I2C lines).
3. `sensor_set` to change readings, `sensor_stimulus` + `sensor_stimulus_start` to play values over
   time, `sensor_fault` to inject a fault.
4. `sensor_stop`.

### The emulation image

I2C, SPI and UART sensors need the **emulation** gateware image (image 2, gateware v54 and
later, W25Q slot `gw2`). The first `sensor_start` of such a sensor loads it when another image is
running (about 2 seconds; the reply says `"image_loaded":true`). Loading it is refused while a
DAC waveform plays (`generate`, `replay` or the control loop): `"loading the emulation image stops
the DAC waveform that is playing: dac_stop it first"`. Without the image installed:
`"the emulation image is not installed (W25Q slot gw2 is empty): install the gateware blobs"`.

While the emulation image runs, the closed DAC loop (`dac_control_loop`, `dac_loop_input`,
`dac_loop_probe`) and deep DAC replay (`load_bin` with `psram`, a deep `replay`) are not
available; they fail with `BP0059` (`fpga_image_sensors_first`, args `feature` and `image`: stop
the sensors, then switch to that image). Captures, the UART
proxy, SWD, GPIO, the SPI master, `generate` and shallow `replay` work in every image. The pod does
not swap back by itself: after `sensor_stop` the emulation image stays loaded until
[`fpga_image`](#fpga_image--switch-the-running-gateware-image) switches it, and `fpga_image` to
another image is refused while I2C, SPI or UART sensors run. Analog sensors work in every image
and never load one.

After any reconfiguration (an image reload, `flash-ice40`, the boot-time gateware update) the
running instances are armed again with their values and the register bytes the DUT wrote; when the
emulation image is no longer running, the I2C, SPI and UART instances stop and release their pins.

### `sensor_define` — upload and compile a definition

The JSON is sent base64 encoded (standard or URL alphabet, padding optional) in chunks: `offset` 0
first, then each next chunk at the offset where the previous one ended. The chunk that reaches
`total` compiles it. The cloud command channel takes a command of at most 1280 bytes, counting the
whole command text (`{"cmd":"sensor_define",...}` with every field); `data_b64` itself takes at
most 1240 characters. A chunk of 768 raw bytes (1024 base64 characters) always fits, with room for
the other fields, in a command of at most 1152 bytes, which is what the SDK sends.

```json
{"cmd":"sensor_define","instance":"env","offset":0,"total":3225,"data_b64":"eyJmb3JtYXQi..."}
```

| Field | Type | Required | Description |
|---|---|---|---|
| `instance` | string | yes | Instance name. Defining a running instance stops it first. |
| `offset` | integer | yes | Byte offset of this chunk in the JSON (before base64). |
| `total` | integer | yes | Size of the whole JSON, 1..32768. |
| `data_b64` | string | yes | This chunk, base64. |
| `dry_run` | bool | no | Compile and report, keep nothing (validation). Send it on the last chunk. |

One upload (a definition or a stimulus) is in flight at a time: a chunk at offset 0 starts a new
one and drops the old. An upload idle for 60 s is dropped.

Replies: `{"status":"ok","data":{"instance":"env","received":768}}` for a chunk, and for the last:

```json
{"instance":"env","kind":"i2c","name":"bme280","label":"Bosch BME280...","program_bytes":1648,
 "i2c":{"address":"0x76","alt_addresses":["0x77"]},
 "params":[{"key":"temperature_c","unit":"C","min":-100,"max":150,"default":25}]}
```

SPI definitions report `"spi":{"mode":3}`, UART `"uart":{"baud":9600,"format":"8N1"}`, analog
`"analog":{"output":"voltage","min":0,"max":3.3}`. `params` lists every param unless the reply
would not fit the cloud cap (16 params with long keys); it then ends early and the reply carries
`"params_cut":true` (`sensor_status` lists every value).

Errors:

| Condition | `message` |
|---|---|
| bad instance name | `"instance must be 1..16 characters of a-z, 0-9 and _"` |
| a chunk out of order | `"chunk at offset N, expected M (start again at offset 0)"` |
| `total` out of range | `"total must be 1..32768 bytes"` |
| not base64 / too long | `"data_b64 is not base64"`, `"data_b64 too long: at most 1240 characters per chunk"` |
| the definition does not compile | `"invalid definition: registers[4].s16be: unknown name 'ax' at 17"` (format.md section 10) |
| the heap is short | `"out of memory: compiling needs N bytes, M free"` |
| 8 instances defined | `"8 instances are defined already: sensor_stop one first"` |

Memory: the upload, the compiler's scratch (about 28 KB) and the program live on the heap only
while they are needed; a running instance holds its program (up to 8 KB, typically 1.5 to 3 KB),
about 4.5 KB of state, a 256-byte register image (I2C/SPI) or 2.5 KB of UART buffers.

### `sensor_start` — start an instance

```json
{"cmd":"sensor_start","instance":"env","address":"0x77","pins":{"sda":2,"scl":1,"drdy":7},
 "values":{"humidity_pct":40}}
```

| Field | Type | Required | Description |
|---|---|---|---|
| `instance` | string | yes | A defined instance. |
| `pins` | object | yes | I2C `sda`, `scl`, optional `drdy`; SPI `sclk`, `mosi`, `miso`, `cs`; UART `tx` (pod to DUT) and optional `rx` (DUT to pod); analog `output`: `"dac"` or `"current"`. Values are LA channels: `3` or `"LA3"`. The pod does not know wiring names: clients resolve them (the SDK and MCP take wiring names and send channels). |
| `address` | string | no | I2C: the definition's `address` or one of its `alt_addresses` (`"0x77"` or `119`). |
| `baud` | integer | no | UART: overrides the definition's baud rate (300..3000000). |
| `values` | object | no | Initial param values (otherwise the defaults), checked like `sensor_set`. |

Reply:

```json
{"instance":"env","slot":"i2c0","image":"emulation","image_loaded":true,"address":"0x77",
 "unavailable":["dac_control_loop","deep_replay"]}
```

`slot` is the gateware block the instance uses: `i2c0`..`i2c3`, `spi0`, `uart0`, `analog0` (the DAC
voltage output) or `analog1` (the 4-20 mA output). UART replies add `"baud"`. `unavailable` is
there while the emulation image runs.

Pins are claimed as LA functions with the instance as owner (`la_pins` shows
`"function":"i2c_sda","owner":"sensor:env"`, see
[LA pin ownership](#la-pin-ownership-functions-and-conflicts)). Several I2C instances may share
`sda`/`scl` when they use the same pair and different addresses. A pin conflict is refused as for
every LA owner.

What the gateware supports, and the limits that follow:

- **I2C**: 8- or 16-bit register addresses (`i2c.page` is the 256-aligned window), 16-bit word
  registers (`i2c.register_bytes: 2`: up to 128 registers, pointer bit 7 ignored, so `p` and
  `p + 0x80` are one register), 16-bit commands in any high byte (`register_address_bits: 16`
  without `page`: the pod lays out every answer, and the bytes a set command writes
  (`write_bytes`), plus one byte per high command byte, in the 256-byte image; a command the
  definition does not list reads whatever its high byte's layout points at), the conversion
  handshake (`trigger`, `busy`, `while_busy` flag, nack or stretch), commanded sensors, the FIFO
  register, `writable: false` bytes (the DUT's writes are counted but not stored), data ready on
  `drdy`. A target has 4 conversion times of at most 65535 us: a longer `time_us` / `busy_us` is
  cut to 65.535 ms, and a definition with more than 4 distinct times uses the longest set one for
  the rest. The pointer always auto-increments and wraps at the end of the window.
- **SPI**: modes 0..3, read and increment bits, address mask, 0..3 dummy bytes, the FIFO register
  and `writable: false`. The SPI target has no conversion handshake and no data-ready pin
  (`"drdy"` is refused: `"the emulated SPI target has no data-ready pin"`); SCK up to about 2 MHz.
- **UART**: 8N1 only (`"the emulated UART is 8N1 only (the definition asks for 7E1)"`); messages
  go out at their times, requests are matched byte by byte and answered after `delay_ms`.
- **FIFO**: one instance with a `fifo` runs at a time (the gateware has one 2047-byte FIFO); frames
  are pushed at `rate_hz` while `enabled`, up to `depth` frames. When it is full new frames are
  dropped (`overflow: drop_oldest` behaves like `drop_newest`: the gateware cannot drop the oldest
  frame). `fifo_level` in a register reads the frames waiting.
- **Data ready**: `on: conversion` asserts in the gateware at the end of each conversion;
  `on: sample` with each FIFO frame and each stimulus step; `on: rate` at `rate_hz`. While the
  definition's `enabled` condition is false the pin keeps its inactive level and never asserts (a
  held level is released). All targets share one pulse width: the last started instance's
  `pulse_us` (1..255).
- **Analog**: one instance per output. The DAC instance picks the smallest output path that covers
  its `range.max` (3v3, 5v, 12v); the current instance drives the 4-20 mA output. `excitation: adc`
  is refused for now (`"excitation: adc is not supported by the pod yet"`). An analog start is
  refused while a DAC waveform plays, and while an analog sensor plays an output, the commands that
  drive it (`generate`, `replay`, `load`, `load_bin`, `measure`, `dac_set`, `dac_out`,
  `dac_control_loop`, and `current_out`) are refused with `BP0065` (`output_used_by_sensor`, args
  `output` `dac` or `current` and `instance`). On a board where the
  4-20 mA output shares the DAC (no quad DAC), both outputs count as the DAC. `dac_stop` stops the
  analog instances on the DAC too (they stay defined).

Errors include `BP0055` (`sensor_instance_unknown`), `"env is running: sensor_stop it first"`,
`"an I2C sensor needs pins sda and scl"`, `"address 0x76 is not one of this sensor's addresses"`,
`"address 0x76 is used by sensor:env on this bus"`, `"all 4 I2C sensor slots are in use:
sensor_stop one first"`, `"the sensor FIFO is used by sensor:imu (one FIFO sensor at a time)"`,
`BP0001` (`la_voltage_not_set`), `BP0017` (I2C, SPI and UART sensors need gateware v54) and the
pin codes (`BP0031` and on).

### `sensor_set` — change readings

```json
{"cmd":"sensor_set","instance":"env","values":{"temperature_c":31.5,"humidity_pct":60}}
```

Every value is checked before any is applied: a value outside the param's range is refused with
its range (`"humidity_pct must be 0..100 %"`), an unknown key with `"env has no param 'x'"`.
Reply: `{"instance":"env","values":{"temperature_c":31.5,"humidity_pct":60}}` (the values as sent).
The DUT sees the new reading on its next read (I2C/SPI) or next message (UART); an analog output
moves on the next poll.

### `sensor_stimulus` — upload a stimulus

Same chunking as `sensor_define`, without `instance` (a stimulus may drive several instances; its
signals use `instance.param`, or `param` when only one instance runs). One stimulus is loaded at a
time; a new one replaces it. The last chunk's reply:
`{"program_bytes":412,"signals":["env.temperature_c"],"faults":2,"duration_s":95,"repeat":false}`.

```json
{"cmd":"sensor_stimulus","offset":0,"total":812,"data_b64":"..."}
{"cmd":"sensor_stimulus_start"}
{"cmd":"sensor_stimulus_start","at_s":30}
{"cmd":"sensor_stimulus_stop"}
```

`sensor_stimulus_start` checks that every signal and fault names a running instance and one of its
params (`"signal env.temp: env has no such param"`), then starts at t = 0 or `at_s`. Values are
applied `rate_hz` times a second (default 50, at most 100), clamped to the param ranges. It ends
after its last segment (signals hold their last value) unless it has `repeat: true`. Replies:
`{"running":true}` and `{"running":false}`; stopping switches its faults off. Stopping or
redefining an instance a running stimulus drives stops the stimulus.

### `sensor_fault` — inject a fault now

```json
{"cmd":"sensor_fault","instance":"env","type":"no_response","duration_s":2}
{"cmd":"sensor_fault","instance":"env","type":"corrupt","at":"0xFA","mask":"0x01"}
{"cmd":"sensor_fault_clear","instance":"env"}
```

Types and fields as in the format's faults section: `type`, `duration_s`, `param`, `value`, `at`,
`mask`, `rate_hz`; `at` and `mask` take integers or hex strings. Without `duration_s` a fault lasts
until `sensor_fault_clear`; `bus_stuck` needs a `duration_s` of at most 1 s. At most 4 faults are
on per instance. Reply: `{"instance":"env","fault":"no_response"}`; `sensor_fault_clear` replies
`{"instance":"env","faults":[]}` and also clears the faults a stimulus switched on.

| Type | What the pod does |
|---|---|
| `no_response` | I2C: the address is not acknowledged; SPI: MISO stays released; UART: no messages and no answers |
| `bus_stuck` | I2C: SDA held low (the gateware), released after `duration_s` |
| `stuck`, `offset`, `gain`, `corrupt`, `open`, `short` | in the engine: the next image, message or output carries it |
| `garbage` | UART: random bytes at `rate_hz` (default 100) instead of the messages |
| `drop` | UART: each message dropped with probability `value` |
| `baud_error` | UART: UART2 runs `value` percent off the baud rate |

A fault that does not apply to the kind is refused: `"fault bus_stuck does not apply to a uart
sensor"`.

### `sensor_stop`

`{"cmd":"sensor_stop","instance":"env"}` or `{"cmd":"sensor_stop","all":true}`. Releases the pins
and forgets the definition. Reply: `{"stopped":["env"]}` (every name for `all`). The emulation
image stays loaded.

### `sensor_status`

```json
{"image":"emulation","stimulus":{"loaded":true,"running":true,"t_s":12.4},
 "instances":[{"instance":"env","kind":"i2c","name":"bme280","slot":"i2c0","running":true,
   "address":"0x76","pins":{"sda":2,"scl":1},
   "values":{"temperature_c":25,"pressure_pa":101325,"humidity_pct":40},
   "faults":[{"type":"no_response","left_s":1.25}],
   "bus":{"writes":14,"transfers":null,"log_lost":0,"last_write":{"at":"0xF4","value":"0x27"}}}]}
```

`{"cmd":"sensor_status","instance":"env"}` reports one instance. Every reply fits the cloud reply
cap (1792 bytes): when the next instance would not fit, the list ends there and the reply carries
`"next":N`; ask again with `"first":N` for the rest. Six typical instances fit one reply; six with
16 long-named params each take two or three.

Per instance: `slot` as in `sensor_start` (empty when not running); `pins`; `values` (the params
as set); `faults`, a list of objects with `type`, the fields it was given (`param`, `value`, `at`,
`mask`, `rate_hz`), `"stimulus":true` when a stimulus switched it on, and `left_s` (null: until
cleared). I2C/SPI `bus`: `writes` (the DUT's data bytes), `transfers` (always `null`: the v54
gateware counts written bytes, not transfers), `log_lost` (writes that happened while more than 32
were pending, so the rules saw them only as "something was written"), `last_write`, `drdy` (asserts
the firmware sent), `fifo_frames` and `fifo_drops`. UART `bus`: `sent_bytes`, `received_bytes`,
`messages`, `answers`, `overruns` (a message or answer due while the last one was still going out:
the baud rate is too low for the rate and the messages), `dropped` (the `drop` fault), `late`
(epochs skipped because the pod was busy when they fell due, for example during a long capture:
the pod sends the newest one, as a busy receiver would). Analog instances report `output` (the value last driven, V or mA).

### `sensor_regs` — read an I2C/SPI image

`{"cmd":"sensor_regs","instance":"env","start":"0xF7","len":8}` returns the bytes the DUT reads now
(its own writes included) in **one reply** (not the chunked uint8 array of firmware 3.10 and
earlier: at most 256 bytes fit, and a single reply works over the cloud command channel):
`{"instance":"env","start":"0xF7","len":8,"data_b64":"gAAAgAAAgAA="}` (standard base64). `start`
0..255, `len` 1..256 (default the whole image).

### `sensor_la` — raw I2C bus capture

`{"cmd":"sensor_la","instance":"env","samples":1024,"sample_rate_hz":500000}` captures the two bus
pins of a running **I2C** instance. By default it **waits for the DUT's next transaction**: the
capture starts on the first SDA fall (on an idle bus that is a START) and keeps 1/8 of the samples
from before it, so the START itself is in the window. A DUT that reads its sensor every 200 ms is
then always caught, where a capture at once mostly missed it.

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `instance` | string | yes | | A running I2C instance. |
| `samples` | integer | no | 256 | Packed bytes to return, 1..4096 (4 samples per byte). |
| `sample_rate_hz` | number (Hz) | no | the highest rate | Sample rate. |
| `trigger` | string | no | `"start"` | `"start"`: wait for the bus's next START, as above. `"none"`: capture at once (the behavior of firmware 3.10.3 and earlier). |
| `timeout_ms` | integer | no | 2000 | How long `"start"` waits, 1..600000. |

While it waits the emulation keeps running (the DUT's reads and writes are answered as usual) and
the pod answers other commands; the ones that need the capture engine or its buffer (other captures, `sensor_regs`, `load`) answer `BP0047` (`pod_busy`). When nothing happens on
the bus in time the reply is `BP0039` (`sensor_trigger_timeout`, args `la`, `instance`,
`timeout_ms`), the sensor form of the other captures' `BP0038`.

The data comes back chunked like the other sample arrays (a stream connection: not over the cloud
command channel), one byte per four samples, oldest first in the high bits: bit7 SCL(s0), bit6
SDA(s0), bit5 SCL(s1) ... bit0 SDA(s3), the format of the earlier single-sensor `sensor_la`. The
first chunk carries `"la_sample_rate_hz"` (the rate the capture ran at) and, with `"start"`, the last chunk
carries `"trigger":{"la":2,"edge":"falling","fired":true}`. When the START came sooner after the
request than the pre-trigger part covers, the samples before the capture began repeat the level
the bus had then. The features table has `sensor_la_trigger` for firmware that takes `trigger` and
`timeout_ms`; older firmware ignores both fields and captures at once.

SPI and UART instances are refused
(`"sensor_la captures I2C sensors only; use la_capture for the pins of acc"`): capture their pins
with [`la_capture`](#la_capture--deep-raw-logic-analyzer-capture).

### Removed in firmware 3.10.3

User-defined sensors replaced the built-in single-sensor commands. Firmware 3.10.1 and earlier still
have them (their documentation is `docs/API.md` at the `v3.10.1` tag).

| Removed | Replaced by |
|---|---|
| `sensor_start` with `type`, `addr`, `sda`, `scl` (one built-in model: `bmp280`, `bme280`, `sht4x`, `mpu6050`) | [`sensor_define`](#sensor_define--upload-and-compile-a-definition) with the model's definition from the sensor library, then [`sensor_start`](#sensor_start--start-an-instance) with `instance`, `address` and `pins` |
| `sensor_set` without `instance` | `sensor_set` with `instance` |
| `sensor_stop` without `instance` | `sensor_stop` with `instance` |
| `sensor_status` (the one sensor and its bus counters) | `sensor_status` (every instance) |
| `sensor_regs` without `instance` (a chunked uint8 array) | `sensor_regs` with `instance` (one reply, `data_b64`) |
| `sensor_la` without `instance` | `sensor_la` with `instance` |
| `sensor_types` | the sensor library (`embeddedci-com/benchpod-sensors`): each definition lists its values and ranges |
| `gps_start`, `gps_set`, `gps_stop`, `gps_status` | the library's `gps_nmea` definition with `sensor_define`, `sensor_start` (a UART instance), `sensor_set` (the fix), `sensor_stop` and `sensor_status` |
| capability `i2c_sensor` | `sensors` |
| capability `sensor_types` | `sensors` (and `sensors_format`) |
| capability `gps` | `sensors` with `sensors_digital` (UART sensors need the emulation image) |

## Device Identity (Ed25519)

Each unit owns a stable **Ed25519 keypair** that serves as its device identity. The private key is generated on the device the first time it boots (seeded from the STM32 hardware TRNG) and persisted to a dedicated flash sector. It survives firmware updates and `wifi_clear`/`cloud_clear`. The device only ever **signs**; the private key never leaves the unit.

The firmware never replaces the key on its own. The one way to erase it and make a new one is [`identity_wipe`](#identity_wipe--erase-the-device-key-usb-link-only) over the USB link (physical presence), after which the pod must be registered again. Up to firmware 3.10.13 it was the USB console's `identity-wipe <short id>`, and the JSON command was always refused.

Two operations are exposed: fetch the **public key** (the identifier), and produce a **proof of possession** — a signature over a caller-supplied nonce that proves the unit holds the private key matching that public key.

### `identity_wipe` — erase the device key (USB link only)

```json
{"cmd":"identity_wipe","confirm":"a1b2c3"}
→ {"status":"ok","data":{"short_id":"d4e5f6"}}
```

`confirm` is the current short id (the six hex digits of `benchpod-a1b2c3`, from
`identity_public`), or `unknown` when the pod has no identity, so a command meant for another pod
fails. The pod erases the identity sector, makes a new key, replies with the new short id and logs
in to the cloud again; the server needs the pod registered again. It runs only on a connection over
the [USB network adapter](usb-serial-interface.md#usb-network-adapter): Ethernet, Wi-Fi and the
cloud get `BP0043` (`setting_device_only`, setting `identity_wipe`). A wrong token gets
`identity_wipe: <why>` with the token to use.

### Encoding of binary fields

All binary values — the public key, the nonce you send, and the signature returned — are **base64url without padding** (RFC 4648 §5; alphabet `A–Z a–z 0–9 - _`, no `=`).

| Field | Raw size | base64url length |
|---|---|---|
| Public key | 32 bytes | 43 chars |
| Signature | 64 bytes | 86 chars |
| Nonce (input) | 1–128 bytes | up to ~171 chars |

Language notes:

- **Go:** encode/decode with `base64.RawURLEncoding`. Verify a proof of possession with stock `crypto/ed25519` — no extra dependencies — but verify over the domain-separated message, not the bare nonce: `ed25519.Verify(pub, append([]byte("benchpod-pop:v1\x00"), nonce...), sig)`. See [Standard, interoperable Ed25519](#standard-interoperable-ed25519).
- **Python:** `base64.urlsafe_b64encode(...).rstrip(b"=")` to send; to decode a returned value, re-pad to a multiple of 4 with `=` before `base64.urlsafe_b64decode(...)`.

### Standard, interoperable Ed25519

The signature is **RFC 8032 Ed25519 (SHA-512)** — the same scheme Go's `crypto/ed25519` and standard JWT/JOSE `EdDSA` verifiers expect. It is **not** an encrypted blob: Ed25519 cannot encrypt. The "proof of possession" is a detached signature that you verify against the public key. No nonce state is kept on the device — every `identity_pop` is independent, so the caller is responsible for nonce freshness/uniqueness.

**The signed message is domain-separated — verify over the prefixed message, not the bare nonce.** The device signs

```
"benchpod-pop:v1" || 0x00 || <decoded nonce bytes>
```

That is, the ASCII context string, a single NUL byte, then your nonce. The NUL separator keeps the encoding unambiguous. Cloud WebSocket authentication uses the same construction with the context `benchpod-ws-auth:v1`, which is what stops a LAN-obtained proof from being replayed as a cloud login (see [Security model](#security-model)). A verifier that passes the bare nonce to `ed25519.Verify` will always fail.

### `identity_public` — fetch the device public key

No parameters.

#### Request

```json
{"cmd":"identity_public"}
```

#### Response

```json
{"status":"ok","data":{"public":"ue5UfSsOAhQYZ332ELokjOI8-uyjcbrd1ic3Nmypvzk"}}
```

| Field | Type | Description |
|---|---|---|
| `public` | string | The device Ed25519 public key, base64url (43 chars, decodes to 32 bytes) |

The same key is also printed on the UART debug console at boot (`[id] device public key (ed25519): …`).

### `identity_pop` — proof of possession

Sign a caller-supplied nonce with the device private key and return the signature.

#### Request

```json
{"cmd":"identity_pop","nonce":"<base64url-nonce>"}
```

| Field | Type | Required | Description |
|---|---|---|---|
| `nonce` | string | yes | Random challenge, base64url-encoded. Decoded length must be 1–128 bytes. |

#### Response

```json
{"status":"ok","data":{"signature":"hZ8…(86 chars)…Qq"}}
```

| Field | Type | Description |
|---|---|---|
| `signature` | string | Ed25519 signature over `"benchpod-pop:v1" \|\| 0x00 \|\| <decoded nonce>`, base64url (86 chars, decodes to 64 bytes) |

#### Verifying (Go)

```go
pub, _ := base64.RawURLEncoding.DecodeString(publicField)    // 32 bytes
sig, _ := base64.RawURLEncoding.DecodeString(signatureField) // 64 bytes

// The device signs the DOMAIN-SEPARATED message, not the bare nonce:
//     "benchpod-pop:v1" || 0x00 || nonce
msg := append([]byte("benchpod-pop:v1\x00"), nonceBytes...)  // nonceBytes = the raw nonce you sent
ok := ed25519.Verify(pub, msg, sig)
```

#### Error cases

| Condition | Message |
|---|---|
| `nonce` field missing | `"missing nonce"` |
| `nonce` not valid base64url, or decodes to 0 or > 128 bytes | `"invalid nonce"` |
| Identity not initialised (should not occur after boot) | `"identity not available"` |

### SCPI equivalents

The same socket also speaks SCPI (a connection is routed to SCPI when its first byte is not `{`). The identity operations are available as:

| SCPI | Equivalent JSON | Result |
|---|---|---|
| `SYSTem:IDENtity:PUBlic?` | `identity_public` | base64url public key (unquoted token) |
| `SYSTem:IDENtity:POP? "<base64url-nonce>"` | `identity_pop` | base64url signature (unquoted token) |

```
SYSTem:IDENtity:PUBlic?
ue5UfSsOAhQYZ332ELokjOI8-uyjcbrd1ic3Nmypvzk

SYSTem:IDENtity:POP? "Q2hhbGxlbmdlMTIz"
hZ8…(86 chars)…Qq
```

A bad or out-of-range nonce pushes SCPI error `-224 "Illegal parameter value"` (retrievable via `SYSTem:ERRor?`).

`SYSTem:IDENtity:POP?` returns the same domain-separated signature as the JSON `identity_pop` — verify it over `"benchpod-pop:v1" || 0x00 || <nonce>`.

### SCPI capture, replay & trace upload

The capture / replay / waveform-upload features have SCPI equivalents that reuse the standard `SOURce`/`SENSe`/`OUTPut`/`TRACe` subsystems. Bulk samples are returned as ASCII CSV (not a JSON array).

| SCPI | Equivalent JSON | Notes |
|---|---|---|
| `SENSe:SRATe <MHz>` / `?` | `capture` `sample_rate_hz` (SCPI keeps MHz) | ADC capture clock; `0` = max ~400 kSPS (24 MHz ÷ divider, divider ≥ 60). Lower stretches the window (e.g. `0.08` → 80 kS/s → 32768 samples ≈ 410 ms). |
| `SENSe:SWEep:POINts <n>` / `?` | `capture` `samples` | Default acquisition length for `READ?`. |
| `READ? [<n>]` | `capture` | Capture `n` (or the `POINts` default) samples at `SENSe:SRATe` and return CSV. Also loads the replay buffer (its first 2048 samples, the DAC BRAM size). |
| `TRACe[:DATA] <offset>,"<base64url>"` | `load` | Upload one chunk of a replay waveform: 16-bit little-endian samples (2 bytes each, as the JSON `load`), at most 4096 bytes. `offset` is in bytes. Send `offset` 0 first, then successive offsets (≤150 bytes/chunk to fit the 256-byte line). |
| `TRACe:POINts?` | — | Number of 16-bit samples currently loaded for replay. |
| `SOURce:FUNCtion USER` + `OUTPut ON` | `replay` | Play the captured/uploaded buffer out the DAC (loops). Playback rate = `SOURce:SRATe`. |
| `OUTPut OFF` | `dac_stop` | Stop the DAC (a looped replay or any waveform). |

`SOURce:FUNCtion` now accepts `USER` alongside `SINusoid`/`SQUare`/`RAMP`. `OUTPut ON` with `USER` selected and an empty buffer pushes `-200 "Execution error"`.

The replay buffer is the pod's one RAM sample buffer, shared with the JSON commands. A JSON `capture`, `test`, `load` or `load_bin` (to RAM) fills it, after which the SCPI trace is gone: `OUTPut ON` with `USER` pushes `-200` and `TRACe:POINts?` answers `0` until the next `READ?`, `MEASure?` or `TRACe:DATA`. The reverse holds too: after a SCPI capture or upload, a JSON `replay` of a RAM upload answers `nothing to replay`. `TRACe:DATA` takes the same capture lock as `READ?`, so it pushes `-200` while a capture or upload is running.

SCPI gets the same safe-mode and digital-board refusals as the JSON commands. In safe mode (the iCE40/PSRAM are off) `OUTPut`, `READ?`, `MEASure?`, `DIAGnostic:CAPture?`, `TRACe:DATA` and `DIGital:*` push `-240 "Hardware error"`; on the digital-only board `OUTPut ON`, `READ?`, `MEASure?`, `TRACe:DATA` and a `DIAGnostic:CAPture?` with ADC samples push `-241 "Hardware missing"`. `SYSTem:ERRor?` carries the same text as the JSON refusal. The `SOURce`/`SENSe` settings, `DIAGnostic:PATTern?`, identity, network and target power keep working. `OUTPut OFF` is refused in safe mode too (it maps to `dac_stop`), but works on the digital-only board.

```
# Record a slow trace, then replay it straight back out the DAC
SENSe:SRATe 0.08
SENSe:SWEep:POINts 4096
READ?                       → 142,139,141,...      (also fills the replay buffer)
SOURce:SRATe 0.08
SOURce:FUNCtion USER
OUTPut ON                   # DAC now loops the recorded trace
OUTPut OFF                  # stop

# Replay a previously saved trace (upload first, in base64url chunks)
TRACe:DATA 0,"<base64url chunk 0>"
TRACe:DATA 150,"<base64url chunk 1>"
TRACe:POINts?               → 150      (300 bytes = 150 samples)
SOURce:FUNCtion USER
OUTPut ON
```

---

## Closed-loop DAC, gateware images and PSRAM

The closed-loop DAC is described in full in [dac-control-loop.md](dac-control-loop.md); the
commands are summarized here. They need the analog front end and the loop gateware image
(feature `dac_control_loop`).

### `dac_control_loop` — arm the closed-loop DAC

The iCE40 reads the ADC every tick, looks the reading up in a 2048-point curve and moves the DAC
toward it. `dac_stop` disarms the loop.

```json
{"cmd":"dac_control_loop","k":32767,"vmin":0,"vmax":65535,"tick_div":64,"curve":"<b64url>","source":"adc"}
```

| Field | Type | Default | Description |
|---|---|---|---|
| `curve` | string (base64url) | the curve already loaded | 16-bit little-endian points; the firmware stretches a short curve to the 2048-entry table |
| `k` | integer | `8192` | Loop gain, Q15 (clamped to 32767) |
| `vmin`, `vmax` | integer 0..65535 | `0`, `65535` | Output clamp in DAC codes; `vmin > vmax` is refused |
| `tick_div` | integer | `64` | Loop tick divider (clamped to at least 8) |
| `source` | string | `"adc"` | Gateware v29+: `"adc"` (closed loop), `"fixed"` (hold `input`) or `"sweep"` (add `step` every tick) |
| `input`, `step` | integer | `0` | The fixed input and the sweep step for the open-loop sources |
| `in_mv_per_unit`, `in_mv_at_zero`, `in_min`, `in_max`, `in_trip` | number | none | Gateware v30+ input map in engineering units; `in_trip` parks the output at `vmin` past that level |

The reply echoes the effective values: `{"armed":true,"k":..,"vmin":..,"vmax":..,"tick_div":..,"curve_pts":..,"source":"adc","input":0,"step":0}`, plus `in_zero`, `in_gain`, `idx_max` and `trip_idx` when an input map is set.

Errors: `BP0017` (the control loop needs gateware v23, input sources v29, the input map v30),
`"unknown loop input source (use adc, fixed or sweep)"`, `"curve base64 decode failed"`, `"curve
needs at least one 16-bit point"`, the parameter checks (inverted window, zero-step sweep),
`BP0064` (`loop_sweep_with_map`), `BP0058` (`fpga_image_required`: the control loop needs the
`loop` image; `BP0059` on the emulation image) and `"control loop arm failed"`.

### `dac_loop_input` — retarget a running loop

Changes the loop's input source or value without re-arming or re-uploading the curve (gateware v29+).
Fields left out keep their value.

```json
{"cmd":"dac_loop_input","input":32768}
{"cmd":"dac_loop_input","source":"sweep","step":16}
```

Reply: `{"source":"fixed","input":32768,"step":0,"code16":<DAC code at the moment of the write>}` (`v` up to firmware 3.10.13). Poll
`dac_loop_probe` for the settled value. Errors: `"loop input sources need gateware v29+"`,
`"unknown loop input source (use adc, fixed or sweep)"`, the parameter checks, the image refusal
and `"loop input update failed"`.

### `dac_loop_probe` — one live operating point

```json
{"cmd":"dac_loop_probe"}
```

Reply: `{"i":<raw ADC count>,"in":<what the loop indexed with>,"source":"adc","code16":<DAC code>,"tripped":false}` (`v` up to firmware 3.10.13).
Plot against `in`, not `i`: in a fixed or sweep run the ADC is not in the path. `tripped` is the
latched over-range trip (the output then sits at `vmin` until the loop is disarmed). On the
deep-replay image it answers `BP0058` (`fpga_image_required`, image `loop`).

<a id="fpga_image--switch-the-running-gateware-image"></a>
### `fpga_image` — switch the running gateware image

The iCE40 has three images: `0` or `"loop"` (the closed-loop DAC), `1` or `"deep"` (deep DAC
replay from PSRAM) and, from gateware v54, `2` or `"emulation"` (the
[emulated sensors](#emulated-sensors): I2C and SPI targets, the sensor FIFO, UART2). Each lives in
its own W25Q slot (`gw0`, `gw1`, `gw2`). The switch reflashes the iCE40's configuration from the
slot and reconfigures it, which takes 2 to 3 seconds. The pod then announces its new capabilities
to the cloud, and puts an external output stage back on its park level when DAC limits are set.
`sensor_start` loads the emulation image by itself; nothing switches back by itself.

```json
{"cmd":"fpga_image","image":"emulation"}
{"cmd":"fpga_image"}
```

Reply: `{"image":2,"name":"emulation","version":54,"features":4}`; without `image` it reports the
running one. `features` are the FPGA_FEATURES bits: 1 control loop, 2 deep replay, 4 emulation.

Errors: `"image must be 0 (loop), 1 (deep) or 2 (emulation)"`, `BP0017` (the emulation image needs
gateware v54), `"image 2 is not installed on the pod (W25Q slot gw2 is empty): install the
gateware blobs"`, `BP0057` (`image_switch_sensors_running`, args `sensors`: `"env, imu"`), `"image
switch: reflash failed (CDONE never rose)"`, `"image switch: iCE40->PSRAM write inoperable after
retries (run psram_recover: the pod restarts and reloads the iCE40)"`, `"image switch failed"` and
`BP0047` (`pod_busy`).

### `psram_ping` — check the iCE40 to PSRAM write path

The iCE40 writes a known ramp through the real capture datapath into PSRAM and the STM32 reads it
back. A failure points at the iCE40 to PSRAM write path, not at the analog ADC.

```json
{"cmd":"psram_ping","count":16}
```

`count` is 4..256 (default 16). Reply: `{"pass":true,"count":16,"first_bad":-1}`.

<a id="psram_recover--reboot-to-recover-the-psram"></a>
### `psram_recover` — reboot to recover the PSRAM

When `status.psram_ok` is `false`, this acks with `{"recover":"rebooting"}` and reboots the pod
about 0.4 s later; the boot self-test reloads the iCE40 from its config flash and brings the
datapath back without a power cycle. The config flash is rewritten only when the image in it no
longer matches a gateware slot, so reboots do not wear it.

**The iCE40 recovers by itself.** When the pod is idle the firmware pings the iCE40 every 30 s.
One that answered since boot and stops answering (an ESD hit can hang or unconfigure it) is reloaded
from its config flash (no write) after 10 s, at most 6 times an hour; the DAC outputs are parked as
at boot. It never resets the STM32 for this, since that would also power-cycle the DUT. The device
log says what it did. Over the cloud the reply may not arrive before the reboot: treat "no reply, the pod
reconnects" as success and check `status.psram_ok` again.

---

## CAN

Classic CAN and CAN FD on FDCAN1 with the TCAN1044 transceiver, on J1 pins 19 (`CAN+`) and 20
(`CAN−`). Every CAN command is a single reply, so all of them work over the cloud command channel,
and they keep working in safe mode. Frame IDs and data bytes accept decimal or `0x` hex.

**CAN FD** (firmware 3.10.2 and later, capability `can_fd`): give `can_start` a `data_bitrate`.
Frames then carry up to 64 data bytes, and with a `data_bitrate` above `bitrate` their data phase
runs at the faster rate (bit-rate switching). Without `data_bitrate` the bus is classic CAN, as
before.

### `can_start` — bring CAN up

Firmware up to 3.10.13 called it `can_config` (and `can_stop` `can_disable`), like nothing else; 4.0.0 names them as `spi_start`/`spi_stop` and `i2c_start`/`i2c_stop`.

```json
{"cmd":"can_start","bitrate":500000,"mode":"normal","term":true}
```

| Field | Type | Default | Description |
|---|---|---|---|
| `bitrate` | integer | `500000` | Bit rate in bit/s (the arbitration rate on CAN FD); one that has no exact bit timing is refused |
| `data_bitrate` | integer | none | Turns CAN FD on: the data-phase bit rate in bit/s, from `bitrate` up to 8000000. Above `bitrate` frames switch to it after arbitration; equal to `bitrate` gives FD frames without switching. Rates with exact timing work, for example 1, 2, 4, 5 and 8 Mbit/s; 3 or 6 Mbit/s are refused |
| `mode` | string | `"normal"` | `normal`, `listen` (receive only), `internal` (internal loopback) or `external` (loopback through the transceiver); the loopback modes let a single pod test itself |
| `term` | boolean | `false` | Switch the 120 Ω termination on |

```json
{"cmd":"can_start","bitrate":500000,"data_bitrate":2000000,"mode":"normal","term":true}
```

Reply: `{"bitrate":500000,"data_bitrate":0,"fd":false,"mode":"normal","term":true}` (`data_bitrate`
is 0 on classic CAN). Errors: `"mode must be normal|internal|external|listen"`, `"unsupported
bitrate (no exact bit timing)"`, `"unsupported data_bitrate (no exact bit timing at 80 MHz; 1, 2,
4, 5 and 8 Mbit/s work)"`, `"data_bitrate must be from bitrate up to 8000000"`, `"can init
failed"`.

The pod sets the bit timing itself: a sample point of about 87.5 % on classic CAN; on CAN FD 80 %
in the arbitration phase and 80 % (up to 2 Mbit/s) or 75 % (above) in the data phase, with
transmitter delay compensation on, as CiA 601-3 recommends. A CAN FD peer should use the same
sample points.

### `can_write` — send one frame

```json
{"cmd":"can_write","id":291,"ext":false,"rtr":false,"data":[1,2,3]}
```

`id` is required; `ext` selects a 29-bit ID, `rtr` a remote frame, and `data` holds up to 8 bytes
on classic CAN, up to 64 on CAN FD. On a CAN FD bus a frame is an FD frame with bit-rate switching;
`"fd":false` sends a classic frame instead (to talk to a classic node on the same bus), `"brs":false`
keeps an FD frame's data phase at `bitrate`. CAN FD payloads are 0 to 8, 12, 16, 20, 24, 32, 48 or
64 bytes: another length is padded with zeros up to the next one. A remote frame (`rtr`) is always
classic, as CAN FD has none.

Reply: `{"id":291,"ext":false,"dlc":3,"fd":false,"brs":false}`, where `dlc` is the number of data
bytes sent (after padding). Errors: `"missing id"`, `"can not enabled"`, `"bus off"`, `"tx failed
(fifo full?)"`, `"data is longer than 64 bytes"`, `"more than 8 data bytes needs CAN FD (can_start
with data_bitrate)"`, `"more than 8 data bytes needs an FD frame"`, `"rtr needs a classic frame (CAN
FD has no remote frames)"`.

### `can_read` — drain received frames

```json
{"cmd":"can_read","max":8}
```

`max` is 1..8 (default 8). Reply: `{"frames":[{"id":291,"ext":false,"rtr":false,"fd":false,"brs":false,"dlc":3,"data":[1,2,3],"ts_ms":12345}],"overflow":0,"pending":0}`.
`ts_ms` is when the frame arrived, in milliseconds since boot (`ts` up to firmware 3.10.13).
`fd` and `brs` say whether the frame was a CAN FD frame and switched bit rate, `dlc` is the number
of data bytes. `overflow` counts frames dropped because the receive queue was full, `pending` the
frames still queued. A reply holds at most about five 64-byte FD frames (to fit a cloud reply), so
read again while `pending` is above 0. The queue holds about 50 classic frames or 13 64-byte FD
frames.

### `can_status` — state and counters

Reply: `{"enabled":true,"mode":"normal","bitrate":500000,"data_bitrate":0,"fd":false,"term":true,"tec":0,"rec":0,"bus_off":false,"error_passive":false,"rx_pending":0,"rx_overflow":0,"responder_rules":0,"responder_hits":0,"bus_off_recoveries":0}`.

### `can_term` — termination

`{"cmd":"can_term","on":true}` switches the 120 Ω termination, even while CAN is off. Reply
`{"term":true}`. Error: `"missing on"`.

### `can_respond` — automatic replies

Adds a rule so the pod answers a matching frame on its own (an ECU simulation), or clears all rules.
Up to 8 rules.

```json
{"cmd":"can_respond","match_id":2015,"ext":false,"reply_id":2024,"reply_ext":false,"reply_data":[2,80,3]}
{"cmd":"can_respond","clear":true}
```

`reply_rtr` sends a remote frame. `reply_data` holds up to 64 bytes; on a CAN FD bus the reply is an
FD frame with bit-rate switching unless `"reply_fd":false` (classic) or `"reply_brs":false`, as for
[`can_write`](#can_write--send-one-frame). An FD reply is only sent while the bus runs CAN FD.
Reply: `{"rule":0,"rules":1}`, or `{"rules":0}` after a clear. Errors: `"missing match_id"`,
`"missing reply_id"`, `"responder table full"`, `"invalid reply frame"`, and the `can_write` data
errors.

### `can_stop` — turn CAN off

Stops CAN and switches the 120 Ω termination off, so a closed bus leaves nothing on the wire.
Reply `{"enabled":false,"term":false}`. Before firmware 3.10.6 the termination stayed as it was
and the reply was `{"enabled":false}`; send `can_term` with `"on":false` as well for those.
`can_term` still switches the termination on with CAN off when you want it.

---

<a id="analog-module-digital-v3r3"></a>
## Analog module (digital v3r3)

From digital v3r3 the BenchPod is two boards: the digital core board and an analog module that
plugs into it over a 24-pin FFC. The module carries the DAC (a DAC8554, as on analog v3r2), the
ADC, the analog switching expander (U58) and a 24C32 EEPROM for its ID and calibration. Its I2C
devices sit on their own bus (the STM32's I2C2). Firmware 3.9.5 and later:

- recognises digital v3r3 at boot by its module-link pins (the revision strap is the v3 one),
- finds the module when it is plugged in (`MOD_PRESENT#` low) and U58 answers on the module bus.
  `status.analog` is then `true` and every analog command works as on analog v3r2, with the
  v3r2 calibration set until the module's own is used. Without a module, digital v3r3 behaves as
  the digital-only board. Detection runs at boot: restart the pod after plugging the module in,
- reads the module EEPROM at boot,
- reports the module and the two rails it switches in `status.module` and `module`.

The one-piece analog boards (analog v3r1, v3r2) and the earlier digital boards are not affected.

<a id="dac-pinout-on-the-flat-cable"></a>
### DAC pinout on the flat cable

The DAC's clock and data lines cross the flat cable. Digital v3r4 and analog module 3.5 swapped
them, so the clock sits next to a ground pin:

| Pin | Digital v3r3, analog module 3.4 and earlier | Digital v3r4+, analog module 3.5+ |
|---|---|---|
| 7 | DAC data | DAC clock |
| 8 | DAC clock | DAC data |

A digital board and a module from different sides of that change do not work together: the DAC
would get scrambled data. Nothing is damaged (both lines come from the digital board), and from
the next firmware release on the pod refuses every command that drives the DAC with `BP0080` and
sends the DAC nothing at all. Everything else (ADC, LA, relays, I2C) keeps working.
`status.module.dac_pinout_ok` is `false` on such a pair. The module's revision comes from its
EEPROM header: a module that was never programmed (`eeprom` `"blank"`), or whose header does not
read, counts as an older module.

The commands refused: `generate`, `measure`, `replay`, `dac_set`, `dac_out`,
`dac_control_loop`, `current_out` with `ua`, `calibrate` with `dac` or `adc`, and `adc_range`
with `test`. Anything else that would reach the DAC (an analog sensor, for example) fails as well,
because the firmware blocks the DAC frames themselves.

<a id="module--the-analog-module-and-its-eeprom"></a>
### `module`: the analog module and its EEPROM

```json
{"cmd":"module"}
```

```json
{"status":"ok","data":{"present":true,"analog":true,"power":true,"bridge_power":true,
 "dac_pinout_ok":true,"eeprom":"ok","type":"analog","rev":"1.0","serial":"AM-000123","layout":1,"cal_len":0,"cal_ok":false}}
```

| Field | Type | Meaning |
|---|---|---|
| `present` | boolean | A module is plugged in now (`MOD_PRESENT#`, read live). |
| `analog` | boolean | The module was found at boot and is the pod's analog front end. |
| `power`, `bridge_power` | boolean | The module's 5 V rail and the USB-A bridge port rail are on. See [`module_power`](#module_power--module-and-bridge-port-power). |
| `dac_pinout_ok` | boolean | `false` when the digital board and the module use different [DAC pinouts on the flat cable](#dac-pinout-on-the-flat-cable): the DAC is off (`BP0080`). Next firmware release. |
| `eeprom` | string | The EEPROM header: `"ok"`, `"blank"` (never written), `"bad magic"`, `"bad crc"`, `"bad length"`, or `"none"` (no EEPROM answered, or no module at boot). |
| `type`, `rev`, `serial` | string | With `eeprom` `"ok"`: the module type (`"analog"`), its hardware revision and serial number. |
| `layout` | integer | The header's layout version (1). |
| `cal_len`, `cal_ok` | integer, boolean | The stored calibration blob's length in bytes (0 = none), and whether its CRC matched when it was last read. |

`status.module` has the first six fields only.

The EEPROM forms need a module plugged in with its power on. Reads are T0, writes T2 (they persist
settings on the module):

| Request | Does |
|---|---|
| `{"cmd":"module","eeprom":"read","addr":256,"len":32}` | Raw read, 1 to 128 bytes. Reply `{"addr":256,"len":32,"data":"<hex>"}`. |
| `{"cmd":"module","eeprom":"write","addr":256,"data":"<hex>"}` | Raw write of 1 to 64 bytes into the calibration area, `0x100` to `0xfff`. Reply `{"addr":256,"len":N}`. |
| `{"cmd":"module","eeprom":"id","type":"analog","rev":"1.0","serial":"AM-000123"}` | Write the ID fields (`type` defaults to `"analog"`, `serial` up to 16 characters) and keep the calibration fields. Replies with the module object. |
| `{"cmd":"module","eeprom":"cal_commit","len":300}` | Record the first `len` bytes of the calibration area as the calibration blob: their length and CRC go into the header (0 clears it). Replies with the module object. |

Errors: `BP0056` (`no_module_link`, args `cmd`) on any other board, and
`module: no module plugged in, or module power is off` for the EEPROM forms.

EEPROM layout, version 1 (little endian):

| Address | Bytes | Content |
|---|---|---|
| `0x000` | 4 | Magic `"BPMD"` |
| `0x004` | 1 | Layout version (1) |
| `0x005` | 1 | Header length (32). A later layout may grow it; the header CRC is always its last two bytes. |
| `0x006` | 1 | Module type (1 = analog) |
| `0x007` | 2 | Hardware revision, major and minor |
| `0x009` | 1 | Reserved (0) |
| `0x00a` | 16 | Serial number, NUL padded |
| `0x01a` | 2 | Calibration length in bytes |
| `0x01c` | 2 | Calibration CRC-16/CCITT-FALSE |
| `0x01e` | 2 | Header CRC-16/CCITT-FALSE over bytes `0x000` to `0x01d` |
| `0x020` | 224 | Reserved |
| `0x100` | up to 3840 | Calibration blob: the module's ADC/DAC calibration (below). Its length and CRC are in the header. |

The calibration blob (firmware 3.10.1+), little endian:

| Offset | Bytes | Content |
|---|---|---|
| 0 | 4 | Magic `"ACAL"` |
| 4 | 2 | Record version (2) |
| 6 | 2 | Record length in bytes (116 for version 2) |
| 8 | 116 | The calibration record, the same bytes the pod keeps in its own flash |

The calibration belongs to the analog front end, so on digital v3r3 it lives on the module:

- every [`calibrate`](#calibrate--calibrate-the-adc-offset-on-the-current_in-input-j8) save (the J8 zero, the DMM-guided DAC
  lines and ADC gains, the frequency-response record, a clear) writes the pod's flash and then the
  module's blob. The header's calibration fields are dropped first, so a write cut short leaves no
  old blob that still checks out. When the module write fails the pod keeps the new calibration and
  `calibrate` answers `calibrate: saved on the pod, but not in the analog module's EEPROM (is the module
  plugged in and powered?)`;
- at boot the module's record is loaded when its blob is valid, so a module moved to another digital
  board brings its calibration along. A module with no blob yet (calibrated with older firmware, or
  never) gets the pod's record copied to it; an unreadable blob or one of another record version is
  left alone and the pod's record is used.

`calibrate` reports where the calibration in use is kept in `stored`.

<a id="module_power--module-and-bridge-port-power"></a>
### `module_power`: module and bridge port power

Digital v3r3 switches two 5 V rails: the analog module's (about 1.45 A limit) and the always-on
USB-A bridge port (about 1 A limit). Both are on from power-up.

```json
{"cmd":"module_power","module":false}
{"cmd":"module_power","bridge":true}
{"cmd":"module_power"}
```

Reply `{"module":true,"bridge":true}`, the state after the change. Turning the module off stops
the DAC and the +-14 V supply first. Turning it back on sets up U58 again (relays off), parks the
DAC outputs and switches the +-14 V supply on, as at boot; if U58 does not answer, the reply is
`module_power: module power is on, but U58 does not answer`. While the module is off, the analog
commands fail. Refused on other boards with `BP0056` (`no_module_link`).

---

## Network and cloud

The cloud and Wi-Fi settings are T2: on a locked LAN they need the cloud or the USB console. They
work in safe mode, so a pod can always be put back on the network.

### `cloud_status`

Reply: `{"state":"connected","last_error":"","configured":true,"host":"www.embeddedci.com","port":443,"tls":true,"verify":true,"device_id":"<uuid>"}`.
`state` is `disabled`, `waiting-wifi`, `connecting`, `connected` or `backoff`. `waiting-wifi` waits
for an Ethernet or Wi-Fi address, or for the USB network link with a proxy set (the link has no
router, so the cloud rides `benchpod usb-bridge` on the computer).

### `cloud_set` — store the cloud settings

`benchpod register` sends this once; the pod then connects on its own at every boot.

```json
{"cmd":"cloud_set","host":"www.embeddedci.com","port":443,"tls":true,"device_id":"<uuid>","enabled":true}
```

| Field | Type | Default | Description |
|---|---|---|---|
| `host` | string | required | Server host name |
| `device_id` | string | required | The id embeddedci.com gave the pod |
| `port` | integer | `443` with TLS, else `80` | |
| `tls` | boolean | `false` | Release firmware refuses `false` |
| `enabled` | boolean | `true` | |

TLS always verifies the server certificate and host name; a `verify` field is ignored. The pod
reconnects after the reply. Reply: the stored settings. Errors: `"missing host"`, `"host too long"`,
`"missing device_id"`, `"device_id too long"`, `"cloud_set: release firmware needs "tls":true
(plain ws is for development builds)"`, `"config save failed"`.

### `cloud_clear`

Forgets the cloud settings and disconnects. Reply `"cleared"`.

### `wifi_status`

Reply: `{"state":"connected","configured":true,"connected":true,"ssid":"lab","xacts":..,"pump_calls":..,"batch":[..],"settle_hit":..,"settle_miss":..,"xact_err":..,"c3_lost_noresp":0,"c3_lost_reboot":0,"c3_restarts":{"total":0,"no_response":0,"rebooted":0,"no_boot":0,"setup":0,"assoc":0,"flash":0,"boots":1},"c3_nvs":"none","rssi_dbm":-61}`. `rssi_dbm` (firmware 4.0.0) is the signal while joined, else `null`, as in the console's `wifi-show`.
`state` is the same as `status.wifi`; the counters describe the link to the ESP32-C3 and how
often it was lost (`c3_lost_*`: since this boot).

`c3_restarts` (firmware 3.10.1+) counts every time the pod restarted the C3 from reset since
power-on, by cause. The counts survive software and watchdog resets and OTA updates; a power-on
or the reset pin sets them to 0, and `boots` counts the boots they cover (1: this boot only).

| Cause | The pod restarted the C3 because |
|---|---|
| `no_response` | it stopped answering while the link was up (3 RSSI requests in a row) |
| `rebooted` | it announced a new boot in the middle of the session (it reset by itself) |
| `no_boot` | it never announced a boot after its reset was released |
| `setup` | a setup step (init, storage, mode, config, start, connect, MAC) failed or timed out |
| `assoc` | the access point did not take the association (wrong password, out of range) |
| `flash` | its automatic flash from the `esp` slot failed or could not be queued |

After the first restart the object adds `last` (the reason, at most 39 characters),
`last_cause`, `last_uptime_s` (the uptime of the boot that did it) and `last_this_boot`. The pod
also sends `c3_restarts` and `c3_restart_last` (the last cause) in the capabilities it sends
embeddedci.com on every connect. `c3_nvs` is the outcome of the last `wifi_clear` since boot on the ESP32-C3's
side (see below): `none` (no `wifi_clear` yet), `pending`, `erased` or `failed`.

### `wifi_scan` — the networks the pod's Wi-Fi sees

For a Wi-Fi picker: the networks in range, to choose the SSID for `wifi_set`. The pod scans with
its own Wi-Fi, or with a Wi-Fi bridge when one is plugged in, and answers in
the same format either way. Pods with the `wifi_scan` feature have it (firmware 3.9.4+).

A scan takes a few seconds, and the pod keeps answering other commands meanwhile, so it is two
steps: start it, then read it with the `id` the first reply gave.

```json
{"cmd":"wifi_scan"}
{"cmd":"wifi_scan","id":3}
```

The first form starts a scan (or joins the one already running) and answers at once:

```json
{"state":"scanning","id":3,"via":"built-in"}
```

The second form answers `"scanning"` again until the scan is over, then:

```json
{"state":"done","id":3,"via":"built-in",
 "networks":[{"ssid":"lab","rssi":-48,"channel":6,"auth":"wpa2/wpa3"},
             {"ssid":"guest","rssi":-71,"channel":11,"auth":"open"}],
 "truncated":false}
```

Read it every half second or so; a scan is normally done within 5 seconds. `networks` is strongest
first, one entry per network (the strongest access point of a network that has several), at most
20; a hidden network has an empty `ssid`. `rssi` is in dBm. `auth` is `open`, `wep`, `wpa`, `wpa2`,
`wpa/wpa2`, `wpa2-ent`, `wpa3`, `wpa2/wpa3`, `wapi`, `owe` or `other`. The list is cut to fit one
cloud reply; `truncated` is `true` when weaker networks were left out for that. With a Wi-Fi bridge,
the first form waits for the bridge and answers `"done"` (`"via":"wifi-bridge"`, `"id":0`) at once.

How the pod scans depends on its Wi-Fi state:

- **Joined:** the Wi-Fi radio scans alongside the connection; the pod stays joined, and traffic
  over Wi-Fi (the cloud link among it) pauses for moments while the radio visits other channels.
- **Not configured, or a failed join:** the radio is switched on for the scan alone and off again
  afterwards. USB networking is not affected.
- **Joining:** the scan waits for the join to finish (up to 30 seconds).

Errors: `"wifi_scan: <why>"`, for example `"wifi_scan: Wi-Fi is still connecting; try the scan
again in a moment"`, `"wifi_scan: the ESP32-C3 is being flashed or erased; try again when that is
done"`, or `"wifi_scan: no scan with that id; start one with {"cmd":"wifi_scan"}"` (an `id` the pod
never gave out, for example after a reboot). On digital v3r3, which has no ESP32-C3, the read form
fails at once with `"wifi_scan: no Wi-Fi module: this BenchPod has no built-in Wi-Fi"` (firmware
3.9.5+). With a bridge: `"Wi-Fi bridge: <why>"`. Up to firmware 3.10.13 the
[USB console](usb-serial-interface.md) had the same scan as `wifi-scan`.

### `wifi_set` / `wifi_clear`

```json
{"cmd":"wifi_set","ssid":"lab","password":"secret"}
{"cmd":"wifi_clear"}
```

Wired Ethernet stays the primary link; Wi-Fi is the fallback: with both up, `status.ip` is the
Ethernet address and outgoing traffic uses Ethernet. Wi-Fi and the [USB network
adapter](usb-serial-interface.md#usb-network-adapter) are one or the other: `wifi_set` turns USB
networking off (the reply may still go out over it), and `wifi_clear` turns it back on. An empty or
missing `password` is an open network. `wifi_set` replies `{"ssid":"lab"}` and reconnects after the reply; `wifi_clear`
replies `"cleared"` and drops Wi-Fi after the reply. Errors: `"missing ssid"`, `"ssid too long"`,
`"password too long"`, `"config save failed"`.

**Digital v3r3 has no ESP32-C3** (firmware 3.9.5+). `wifi_set` answers `"no Wi-Fi module: this
BenchPod has no built-in Wi-Fi"` and saves nothing, `wifi_clear` clears the pod's copy only
(`c3_nvs` stays `none`), USB networking stays on, `status.wifi` is `absent`, and the pod never
drives the C3's pins or tries to flash it. Credentials stored on an older board are ignored. Up to firmware 3.10.13 the
[USB console](usb-serial-interface.md) had the same operations as `wifi-set` / `wifi-clear`; from
4.0.0 its `net-reset` clears the Wi-Fi (and resets Ethernet) when no network reaches the pod.

**Where the credentials are kept.** In one place: the STM32's config store (two flash sectors,
A/B). The ESP32-C3 is put in RAM storage mode (`esp_wifi_set_storage(WIFI_STORAGE_RAM)`) before it
is given the Wi-Fi mode or the credentials, and it is given them again on every bring-up, so it
holds them only while it is powered and out of reset. If the C3 does not confirm RAM storage, the
pod does not send it the credentials and retries.

**What `wifi_clear` does.** Pod firmware up to 3.7.0 ran the C3 in the ESP-IDF default flash
storage mode, which wrote the SSID and password to the C3's own NVS partition as well. NVS only
marks a replaced or deleted entry as erased, so nothing short of erasing the sectors removes old
values. `wifi_clear` therefore wipes both copies:

1. At once, before the reply: the STM32's copy. A tombstone record is written, then the other slot
   and the legacy sector are erased, so a power loss part-way cannot bring an old config back.
2. After the reply has gone out and Wi-Fi is down (the reply may ride the Wi-Fi link): the C3's
   NVS partition (16 KB at `0x9000`, from the partition table of the image the pod last wrote into
   the C3, or of the copy older firmware kept in the W25Q) is
   erased through the C3's ROM loader and read back blank (MD5 of the region computed by the C3).
   This takes about 3 seconds, and like an `esp` install it stops a running DAC waveform (the W25Q
   shares the PSRAM bus). The C3's application is untouched; it recreates an empty NVS on its
   next start. The C3 then stays in reset and Wi-Fi stays off until the next `wifi_set`.

`wifi_status` reports step 2 as `c3_nvs`: `pending`, then `erased`, or `failed` if the C3 did not
answer its ROM loader, the erase did not verify, the pod does not know where the C3 image keeps its
NVS (no `esp` update since firmware that kept a copy, and no copy left), the pod is in safe
mode, or a capture, upload or update held the shared bus for a minute. After `failed`, send
`wifi_clear` again, or install the `esp` image again (`benchpod install-blobs --force --only esp`: it
rewrites the C3's whole image, NVS region included). `wifi_clear` does not touch the device identity key, the cloud configuration
(`cloud_clear`) or calibration data.

**With a Wi-Fi bridge.** BenchPod v4 has no radio; a bridge box on the
Ethernet port carries Wi-Fi ([wifi-bridge/docs/PROTOCOL.md](../wifi-bridge/docs/PROTOCOL.md)).
Built-in Wi-Fi wins: while it is configured, the pod takes no address through a bridge, the
commands here and `wifi_scan` go to the built-in Wi-Fi, and `wifi_status` adds
`"bridge_ignored":true`. This holds even when the built-in Wi-Fi cannot join. Send `wifi_clear`
to use the bridge. Pairing and bridge updates carry on either way. Without built-in Wi-Fi configured, while a bridge says HELLO
on the Ethernet port, `wifi_set` stores the credentials on the bridge (reply
`{"ssid":"lab","via":"wifi-bridge"}`, the pod's own flash, USB networking and the ESP32-C3 are left
alone), `wifi_clear` clears them there (reply `"cleared"`), and `wifi_status` describes the bridge:

```json
{"state":"connected","configured":true,"connected":true,"ssid":"lab","via":"wifi-bridge",
 "bridge":{"mac":"24:0a:c4:00:00:03","fw":"0.1.0","hw":"A","paired":true,"genuine":true,"problem":null,"update":"idle",
           "rssi_dbm":-58,"channel":6,"identity":"<64 hex>","image":"0.1.3","boot":"power-on","reboots":0,"emc":null,
           "reason":0,"uptime_s":125,"status_age_ms":3200,
           "counters":{"eth_rx":812,"wifi_tx":790,"wifi_tx_err":0,"wifi_rx":1204,"eth_tx":1150,
                       "eth_tx_err":0,"dropped":76}}}
```

`identity` is the bridge's identity key (`null` before bridge firmware 0.1.3), `image` the bridge
firmware the pod holds for it (`null` when slot `bridge` is empty), `boot` the bridge's last reset
reason, `reboots` the bridge restarts the pod saw and `emc` the EMC test radio state (`null`
normally); `boot`, `reboots` and `emc` come once the bridge has sent a status (firmware 4.0.0; the
console's `bridge` showed them before).

`state` is then the bridge's: `unconfigured`, `wait-pod`, `joining`, `connected`, `auth-failed` or
`no-ap`. The pod pairs with the bridge on first contact; `paired:false` with a `problem` such as
"the bridge is paired with another pod: press PAIR on the bridge" means the user has to press PAIR
on the bridge. `genuine` (firmware 3.10.7+) is `true` once the bridge showed a valid identity
certificate and signed the pairing with its identity key; `wifi_set` goes only to a genuine bridge,
else it fails with the `problem` text (`the Wi-Fi bridge has no identity certificate ...` or `the
Wi-Fi bridge's identity check failed ...`). The pod pairs again by itself after PAIR on the bridge
or with a replacement bridge. See [USB serial interface, Wi-Fi bridge](usb-serial-interface.md#wi-fi-bridge)
for provisioning a certificate. Errors from the bridge come back as `"Wi-Fi bridge: <code>: <detail>"`.
`bridge.update` says what the pod's automatic bridge firmware update is doing (`"idle"`,
`"updating 0.1.0 to 0.2.0: 45%"`, `"updated to 0.2.0"`, `"update to 0.2.0 failed: ..."`); the
image comes from W25Q blob slot `bridge` (OTA target `bridge`, listed by `blob_status` in
development builds).

> **After updating from 3.7.0 or older:** the update alone does not erase what the older firmware
> stored on the C3. It stays in the C3's NVS (unused) until the first `wifi_clear` or
> `esp` install. To keep Wi-Fi and still remove it: `wifi_clear`, then `wifi_set` again.

### `wifi_bridge` — the Wi-Fi bridge on the Ethernet port

```json
{"cmd":"wifi_bridge","action":"update"}         → {"update":"requested"}
{"cmd":"wifi_bridge","action":"reboot"}         → {"rebooting":true}
{"cmd":"wifi_bridge","action":"factory_reset"}  → {"factory_reset":true}
{"cmd":"wifi_bridge","action":"cert","part":1,"hex":"<64 hex digits>"} → {"part":1,"parts":1}
```

`update` installs the pod's bridge image now, whatever the versions (`wifi_status`
`bridge.update` follows it); `reboot` restarts the bridge; `factory_reset` makes it forget its
Wi-Fi and its pairing. `cert` stores the bridge's identity certificate, 128 bytes in four parts of
32 bytes (64 hex digits each), in any order; with the fourth the pod sends it to the bridge and
replies `"stored":true`, then checks the bridge's identity again (`genuine` in `wifi_status`).
`cert` runs over the USB link only (else `BP0043`). Without a bridge: `wifi_bridge: no Wi-Fi
bridge on the Ethernet port`; the bridge's own refusals come back as `Wi-Fi bridge: <code>: <detail>`.
The bridge's state is in [`wifi_status`](#wifi_status), its networks in
[`wifi_scan`](#wifi_scan--the-networks-the-pods-wi-fi-sees). Up to firmware 3.10.13 this was the USB
console's `bridge`.

### `usb_reattach` — enumerate on USB again

```json
{"cmd":"usb_reattach"}
→ {"status":"ok","data":{"reattach":true}}
```

About 0.4 s after the reply the pod drops off the USB bus for half a second and connects again, so
the computer enumerates the console and the network adapter anew: macOS ignores a network adapter
that appears while the screen is locked. Over the USB link itself the connection then drops;
connect again. Up to firmware 3.10.13 this was the USB console's `usb-reattach`.

### `eth` — wired link control and diagnostics

| Request | Tier | Reply |
|---|---|---|
| `{"cmd":"eth","action":"stats"}` | T0 | `{"phy_ok":true,"link":true,"aneg_done":true,"phy_mode":"100FD","mac_mode":"100FD","rx_good":..,"rx_crc":..,"tx_good":..,...}` |
| `{"cmd":"eth","action":"refclk"}` | T0 | `{"clock_hz":50000012,"ppm":0,"on_hsi":false}`: the PHY's 50 MHz RMII clock against the MCU crystal; drops the link for about 200 ms |
| `{"cmd":"eth","action":"stop"}`, `"start"`, `"restart"` (the default) | T2 | the action name; `start` and `restart` reset the PHY and get a new DHCP lease |
| `{"cmd":"eth","action":"speed","mbit":100,"duplex":"full"}` | T2 | `{"speed":100,"duplex":"full"}`; `mbit` `0` returns to autonegotiation, `duplex` defaults to half |
| `{"cmd":"eth","action":"loopback","mbit":100,"n":200}` | T2 | `{"mbit":100,"sent":200,"received":200,"intact":200,"corrupt":0,"crc":0,"align":0,"tx_fail":0}`: a PHY loopback test, `n` 1..1000 |

Errors: `"eth action must be stop|start|restart|stats|speed|refclk|loopback"`, `"eth speed needs
mbit (0 = autoneg, 10 or 100)"`, `"eth speed mbit must be 0 (autoneg), 10 or 100"`, `"eth loopback
mbit must be 10 or 100"`, `"eth loopback: timed out"`.

### `speedtest` — cloud throughput probe

Used by the server's speed test over a byte tunnel (`POST /api/benchpod/devices/{id}/speedtest`).

```json
{"cmd":"speedtest","dir":"up","bytes":1048576}
```

`bytes` defaults to 1 MiB and is capped at 64 MiB. `up` (the default): the pod sends `bytes`
synthetic bytes, then `{"speedtest":"done","bytes":N}`. `down`: the pod counts and discards the
next `bytes` bytes, acking every 8 KB with `{"speedtest":"ack","bytes":N}` and finishing with
`{"speedtest":"done","bytes":N}`.

---

## Firmware and blob updates (OTA)

An update is staged in PSRAM, checked (SHA-256, signature, fit) and only then written. The
firmware itself goes to the MCU flash; the blob targets go to slots in the W25Q flash: `gw0` and
`gw1` (the two iCE40 images) and `ca` (a company CA, see
[`cloud_ca`](#cloud_ca--cloud_proxy-company-ca-and-http-proxy-for-the-cloud-link)). Over the
cloud the server sends the image as WS `ota.*` frames. `esp`, the ESP32-C3 Wi-Fi image (boards with
the C3 only), is not kept on the pod: it is written from the staging straight into the C3 (see
`ota_commit`). Over the cloud the server sends the image as WS `ota.*` frames; over the LAN the image goes in base64url
`ota_data` chunks, also over the USB network adapter (the USB console's `upload-*` commands are gone
from firmware 3.9.0; see [usb-serial-interface.md](usb-serial-interface.md)). Most clients use `benchpod firmware install`
or the web app instead of these commands.

One transport holds an update session at a time. Every `ota_*` reply carries the session:

```json
{"status":"ok","data":{"state":"receiving","target":"firmware","received":65536,"size":563696,
 "frames_seen":0,"sig":"ok","sig_key":"<key id>","owner":"lan:1","error":""}}
```

`state` is `idle`, `receiving`, `verified`, `error`, `installed` or `applying` (an `esp` image is
being written into the ESP32-C3). `sig` is the signature check of
the last begin: `none`, `ok`, `format`, `unknown-key`, `signature`, `target` or `image`. `owner` is
`cloud`, `usb`, `lan:N` or `""`.

| Command | Fields | What it does |
|---|---|---|
| `ota_begin` | `size`, `sha256` (64 hex), optional `target` (`gw0`, `gw1`, `gw2`, `esp`, `bridge`, `xip`, `ca`, `tls`; none = firmware), `version`, `sig` (base64url signed manifest) | Starts a session. The signature is checked here against the [signature policy](#sig_policy--which-firmware-and-blob-updates-the-pod-accepts) (the `ca` and `tls` targets are configuration and are not signed; `bridge`, the Wi-Fi bridge's firmware, always needs a good signature because the bridge checks it again, and so does `esp` from firmware 3.10.7). From firmware 4.0.0 every code image (firmware, gateware, `esp`) needs a good signature on every transport, and a signed firmware below the rollback floor is refused (same section). `xip` is code the pod runs straight from its W25Q (blob `blob-xip.bin`, made by the same build as the firmware): it is listed by `blob_status` and accepted only on boards that can run it, and only the image the running firmware was built with (another one is refused with `ota_begin`'s `error` "this xip image was not built with this firmware"). |
| `ota_data` | `offset`, `data` (base64url) | Writes one chunk at a byte offset. |
| `ota_end` | none | Checks the SHA-256 (and, for the firmware, that the image fits this MCU's flash and can enforce the policy); `state` becomes `verified` or `error`. |
| `ota_status` | none | The session, from any transport (T0). |
| `ota_abort` | none | Drops the staged image. |
| `ota_selftest` | none | Tests the flash writer on a scratch sector (never the firmware); replies `{"selftest":"pass"}`. |
| `ota_commit` | none | Firmware: replies `{"committing":true}`, writes the image and resets. A blob: writes the slot and replies with the session. `esp`: replies with `state` `applying` and writes the image into the ESP32-C3 in the background (about 2 minutes, Wi-Fi down meanwhile; a commit sent over Wi-Fi may lose its reply), then `state` becomes `installed`, or `error` with the reason (the C3 is then left without a usable image). Poll `ota_status` until then: nothing may begin, abort or overwrite the session meanwhile, and captures and uploads answer busy. |

Refusals:

| Condition | Message |
|---|---|
| `size` or `sha256` missing | `"missing size/sha256"` |
| Unknown `target` | `BP0060` (`ota_unknown_target`) |
| `ca` from Ethernet or Wi-Fi (the USB network adapter may) | `BP0042` (`setting_cloud_or_usb`, setting `cloud_ca`) |
| Another transport holds the session | `BP0051` (`update_in_progress`, args `target`, `holder`). Within the LAN, a second connection is refused while the one holding the session is open; once it closes, the next LAN connection takes the session over. A session with no progress for 30 s may be replaced or aborted from any transport. The ota replies carry `"owner"`: `"cloud"`, `"usb"`, `"lan:N"` or `""` |
| A capture or upload is running | `BP0047` (`pod_busy`) |
| Safe mode turned the PSRAM off | `"safe mode: iCE40/PSRAM are off. Unplug and replug the pod"` (the gate) |
| Size, hash or chunk problems | `"bad size"`, `"bad sha256"`, `"chunk out of range"`, `"incomplete image"`, `"sha256 mismatch"`, `"invalid data"`, `"missing data"` |
| The policy refuses the signature | `"signature: <result>"` |
| The image could not enforce `required` | `"image cannot enforce signatures (policy required)"` |
| `ota_commit` before a successful `ota_end` | `"no verified image staged"` |
| Any `ota_*` that would change the session while an `esp` image is written into the C3 | `BP0048` (`storage_bus_busy`) |
| `esp` on a board without the ESP32-C3 | `"this BenchPod has no ESP32-C3: it takes no esp image"` |
| `ota_selftest` while the hardware is busy | `BP0047` (`pod_busy`); a failed test is `"ota selftest failed (see console log)"` |

### `blob_status` — what the W25Q slots hold

Reads the cached slot headers (no bus access).

```json
{"status":"ok","data":{"blobs":[
  {"name":"gw0","state":"ok","present":true,"size":135100,"version":47,"sha256":"<hex>"},
  {"name":"gw1","state":"ok","present":true,"size":135100,"version":47,"sha256":"<hex>"},
  {"name":"esp","state":"outdated","present":true,"size":1048576,"version":3,"sha256":"<hex>"}]}}
```

It lists the release slots: `gw0`, `gw1`, `gw2`, `esp`, `bridge` (the Wi-Fi bridge's firmware, which the pod keeps ready and relays when a bridge is plugged in and runs an older version) and, on boards that can run code from the W25Q, `xip` (that code: its `state` is `ok` only for the image built with the running firmware); the company CA is read with `cloud_ca`.

`state` compares the slot with what this firmware was built with: `ok` (exactly that blob),
`outdated` or `missing` (an installer should send it), or `unknown` (this build expects nothing in
particular).

`esp` is listed only on a board with the ESP32-C3 (digital v3r3 and later have none). It is not a
slot: it describes the image the C3 runs, as recorded when the pod last wrote one into it. On a pod
updated from firmware that kept a copy in the W25Q, that copy is reported until the first `esp`
update. `present` is false, and `state` `missing`, when the last write into the C3 failed, or when
the C3 does not start and the pod has no copy to reinstall it from.

### `usb_flash` — flash a board over USB

Firmware `next` and later (feature `usb_flash`). The pod flashes a board on its USB host port
([`usb_host`](#usb_host--usb-host-mode)) from a file uploaded with `load_bin` and `"psram": true`,
the way a computer would:

- `uf2`: UF2 bootloaders (Raspberry Pi Pico and Pico 2, Adafruit and Seeed boards, tinyuf2) take
  the UF2 file's blocks on the drive they show.
- `file`: drag-and-drop loaders such as DAPLink (NUCLEO, nRF DK, micro:bit) take a file copied onto
  their drive.
- `dfu`: USB DFU loaders, and ST's DfuSe in every STM32's ROM bootloader (BOOT0 high at reset) and
  boards such as the Black Pill. For DfuSe the pod reads the flash layout from the loader, erases
  the sectors the image covers and writes it at `address`.
- `teensy`: the Teensy loader (Teensy 3.x, 4.0, 4.1, MicroMod; press the button or use the 1200
  baud touch to start it). Blocks of 0xFF are skipped.
- `avr`: the Arduino AVR bootloaders over the board's USB serial port. The pod finds the one the
  board runs: Caterina (Leonardo, Micro, Pro Micro, Feather 32u4; a board running its sketch gets
  the 1200 baud touch first), optiboot and the older Nano bootloader (Uno, Nano, Pro Mini, after a
  DTR reset at 115200 then 57600 baud), and the Mega 2560 bootloader. Every page is read back and
  compared.
- `samd`: the Arduino SAMD bootloader (Zero, MKR, Nano 33 IoT, and the Adafruit and Seeed SAMD
  bootloaders, which answer the same way), after the 1200 baud touch. The pod reads the flash size
  from the chip, erases from `address` to the end, writes, reads back and compares, then resets the
  board. `address` is where the sketch starts: `0x2000` on a SAMD21, `0x4000` on a SAMD51. The
  bootloader below it is refused.
- `esp`: an ESP32 chip's ROM loader over the board's USB: its own USB serial port (C3, C6, S3, H2)
  or the USB-serial chip of a dev kit (CP210x, CH34x, FTDI). The pod resets the chip into its
  loader with DTR and RTS as esptool does, writes the image at `address` (0 for a merged image) and
  checks its MD5 on the chip. A dev kit's chip runs at 460800 baud for the write.

Serial ports: CDC ACM (a board's own USB), and the FTDI, CP210x and CH34x USB-serial chips.

The image for `dfu`, `teensy`, `avr`, `samd` and `esp` is a binary with its start `address` (the SDK
converts `.hex` and `.dfu` files). The flash runs in the background; poll `usb_flash` for its
state.

```json
{"cmd": "usb_host", "enabled": true, "port": "usb_a", "timeout_s": 120}
{"cmd": "usb_flash", "mode": "uf2", "len": 204800}
{"cmd": "usb_flash", "mode": "file", "name": "FIRMWARE.BIN", "len": 61440}
{"cmd": "usb_flash", "mode": "dfu", "address": "0x08000000", "len": 131072}
{"cmd": "usb_flash", "mode": "teensy", "address": 0, "len": 262144}
{"cmd": "usb_flash", "mode": "avr", "len": 10240}
{"cmd": "usb_flash", "mode": "samd", "address": "0x2000", "len": 51200}
{"cmd": "usb_flash", "mode": "esp", "address": 0, "len": 1048576}
{"cmd": "usb_flash"}
{"status": "ok", "data": {"running": false, "have_result": true, "mode": "file", "name": "FIRMWARE.BIN",
  "len": 61440, "address": 0, "protocol": "", "phase": "done", "done": 120, "total": 120, "result": "ok",
  "ok": true, "ms": 1840}}
{"cmd": "usb_flash", "abort": true}
```

| Argument | Description |
|---|---|
| `mode` | `uf2`: the file is a UF2 file; its 512-byte blocks are checked and written to the drive in order. `file`: any file (`.bin`, `.hex`), written to the drive's FAT12 or FAT16 file system as `name`. `dfu`: a binary for a USB DFU or STM32 DfuSe loader. `teensy`: a binary for the Teensy loader. `avr`, `samd`, `esp`: a binary for the board's bootloader on its USB serial port. Without `mode` the command reads the state. |
| `address` | `dfu` with a DfuSe loader: the flash address the image starts at (`"0x08000000"` for an STM32's flash). `teensy`: the loader address of the image (0 for a whole image). `avr`: the byte address (0 for a sketch). `samd`: the flash address (`0x2000` on a SAMD21). `esp`: the flash offset (0 for a merged image, `0x10000` for an application). A number or a `0x` string. |
| `block` | `teensy`: the loader's block size, 1024 (default) or 512 (Teensy LC). |
| `leave` | `dfu`, `teensy`, `avr`, `samd`, `esp`: `false` keeps the board in its loader at the end; by default it starts the new firmware. Some bootloaders (optiboot, Caterina) start the sketch on their own after a few seconds anyway. |
| `name` | With `mode` `file`: the file name, an 8.3 name such as `FIRMWARE.BIN`. A file of the same name is replaced. |
| `len` | Bytes of the uploaded file to write. |
| `abort` | `true` stops a running flash. |

| Field | Description |
|---|---|
| `running` | A flash is in progress. While it runs, captures, DAC playback, uploads and updates are refused with BP0048. |
| `protocol` | `avr`: the bootloader found, `avr109`, `stk500v1` or `stk500v2`. `samd`: `sam-ba`. `esp`: the chip, such as `ESP32-S3`. Empty otherwise. |
| `phase` | `uf2` and `file`: `start`, `uf2`, `directory`, `free space`, `data`, `fat`, `entry`, `done`. `dfu` and `teensy`: `start`, `descriptors`, `layout`, `erase`, `address`, `write`, `leave`, `done`. `avr` and `samd`: `start`, `touch`, `reattach`, `reset`, `sync`, `info`, `erase`, `write`, `verify`, `leave`, `done`. `esp`: `reset`, then the ESP32 flasher's phases (`sync`, `data`, `md5`, ...). |
| `done`, `total` | Sectors (`uf2`, `file`), blocks (`dfu`, `teensy`, `esp`) or bytes (`avr`, `samd`: written, then verified) of all. |
| `result`, `ok` | The outcome once it ended: `ok`, `aborted`, or what went wrong (the drive did not take a sector, the file is not a UF2 file, no FAT12/FAT16 file system, no room for the file, the board has no DFU interface, the image does not fit the loader's flash layout, the loader reported an error, the board is not a Teensy in its loader, no bootloader answered on the serial port, what the board holds is not the image, the image would overwrite the bootloader, ...). |

The pod keeps `usb_host` on while a flash runs. The flash logic of every mode but `esp` runs from
the pod's W25Q (`xip`), so it needs the `xip` blob of the running firmware; `esp` uses the same
flasher as `esp_flash`, and is refused while another ESP32 flash runs. Refusals: BP0084 (`usb_flash_refused`, arg
`why`: host mode off, no drive mounted, device or serial port attached, nothing staged, a bad mode, name or length), BP0083 (the W25Q
code), BP0048 (the storage bus is busy).

### `xip` — code that runs from the W25Q

Some rarely used code (the USB flash protocols, coming) runs straight from the pod's W25Q flash
instead of the MCU's own flash. It is the `xip` blob (`blob-xip.bin`), made by the same build as the
firmware and installed with the other blobs; the pod runs only the one built with its firmware.
While it runs, the pod's storage bus belongs to it: captures, DAC playback, uploads and updates
are refused with BP0048 until it ends.

```json
{"cmd": "xip"}
{"status": "ok", "data": {"supported": true, "mode": "mapped", "image": {"size": 124, "slot": "ok"}}}
{"cmd": "xip", "selftest": true}
{"status": "ok", "data": {"supported": true, "mode": "mapped", "image": {"size": 124, "slot": "ok"},
  "selftest": {"ok": true, "result": 2871906233, "expected": 2871906233, "begin_us": 412, "call_us": 38}}}
```

| Field | Description |
|---|---|
| `supported` | This pod can run code from its W25Q (boards whose W25Q chip-select can be memory-mapped). |
| `mode` | `mapped` (the CPU runs it in place), `ram` (development builds that copy it to RAM first) or `none`. |
| `image.size` | Bytes of W25Q code this firmware was built with. |
| `image.slot` | The `xip` blob slot against that image: `ok`, `outdated`, `missing` or `unknown` (as `blob_status`). |
| `selftest` | With `"selftest": true`: runs the image's self-test (constants read from the W25Q, a call back into the firmware). `ok` when `result` equals `expected`; `begin_us` is the time to check and map the image, `call_us` the run. |

Refusals: BP0083 (`xip_unavailable`, arg `why`) when the board or the slot cannot run the image,
BP0048 while something else holds the storage bus.

### Wiring profiles: `wiring_list`, `wiring_get`, `wiring_put`, `wiring_delete`, `wiring_default`

The pod keeps its wiring profiles (which board signal is on which LA channel, the I/O voltage, the
power rail) itself, so every connection sees the same ones: LAN, USB and the cloud, from any
computer (firmware 3.10.8 and later). Up to 16 named profiles of at most 4096 bytes of JSON each, 16 KB in all, and one of them
is the default (the one a tool uses when it names none). Names are 1 to 40 characters: lowercase
letters, digits and dashes, not starting with a dash. The SDK, CLI, MCP server and apps read and
write them for you (`bp.load_wiring`, `bp.save_wiring`, `benchpod wiring`); this is the protocol
underneath.

The pod treats a profile as opaque JSON: it only checks that it is one JSON object. The tools
validate the contents. Every save gives the profile the next revision number.

```json
{"cmd":"wiring_list"}
-> {"status":"ok","data":{"default":"k66","profiles":[{"name":"k66","rev":3,"bytes":812},
    {"name":"nrf52840-dk","rev":1,"bytes":640}],"max_profiles":16,"max_bytes":4096,"free_bytes":14920}}
```

`wiring_get` and `wiring_put` move a profile in pieces of at most 600 bytes (base64url, no padding),
so a piece always fits the cloud command channel. Read from `offset` 0 until `offset` plus the
piece's length reaches `total`; when `rev` changes between pieces, start again.

```json
{"cmd":"wiring_get","name":"k66","offset":0}
-> {"status":"ok","data":{"name":"k66","rev":3,"total":812,"offset":0,"data_b64":"eyJ2ZXJz..."}}

{"cmd":"wiring_put","name":"k66","offset":0,"total":812,"data_b64":"eyJ2ZXJz...","if_revision":3}
-> {"status":"ok","data":{"name":"k66","received":600,"total":812}}
{"cmd":"wiring_put","name":"k66","offset":600,"total":812,"data_b64":"..."}
-> {"status":"ok","data":{"name":"k66","rev":4,"default":true}}
```

Send the pieces in order, starting at `offset` 0; the save happens with the last one. One upload
runs at a time for the whole pod: a new piece at offset 0 replaces an unfinished upload.
`if_revision` (optional, on the first piece) is the revision you loaded, or 0 for a new name: when
the pod's copy moved on since then the save is refused with `BP0009` instead of overwriting someone
else's change. The first profile stored becomes the default.

```json
{"cmd":"wiring_delete","name":"nrf52840-dk"}  -> {"status":"ok","data":{"name":"nrf52840-dk","deleted":true}}
{"cmd":"wiring_default"}                       -> {"status":"ok","data":{"default":"k66"}}
{"cmd":"wiring_default","name":"nrf52840-dk"}  -> {"status":"ok","data":{"default":"nrf52840-dk"}}
```

The default profile cannot be deleted (`BP0013`): make another one the default first. Reading never
touches the pod's flash bus. Saving, deleting and changing the default do, so they are refused with
`BP0016` while the DAC runs or a capture holds the bus; stop the DAC first. A save survives a power
loss: the pod keeps two copies and writes the older one. Refusals: `BP0006` (a missing value),
`BP0007` (no such profile), `BP0008` (bad name), `BP0009` (revision conflict), `BP0010` (full),
`BP0011` (too large), `BP0012` (not a JSON object), `BP0014` (piece out of order), `BP0015`
(storage failed), see [error codes](https://embeddedci.com/docs/errors).

### `emi_log` — the EMI test build's run log

Only in the EMI test build (`make EMI_TEST=1`, never released); other builds answer `"unknown cmd"`.
That build keeps every run (one power-on) in the W25Q's free range `0x192000`-`0x200000` (440 KB,
7,040 records of 64 bytes; no blob slot uses it, so a slot added there later would erase it): a
`START` record at boot (firmware version, reset cause, core clock, DAC mode, gateware version), an
`EVENT` on every change (the status LED's state, the Ethernet link lost or back, the activity
started or stopped, a recovery step with its result, an ESP32-C3 restart with its cause) and a `CHECK` with the counters (captures, errors, frames sent and dropped, link
losses, the longest DAC gap a log write caused, 0 normally). A `CHECK` is written when a value that
shows trouble changed (errors, dropped frames, link losses, the LED, the link, the DAC gap, lost
records), at most every 10 s, and once a minute when nothing changed. A quiet hour takes about 60
records, an hour with trouble all the time at most 360. 16 sectors (1,024 records) are kept erased
ahead of the write position, erased at boot before the loop starts, so a lab day never erases flash
while it runs (unless it has trouble for more than about 2.8 hours in all); the ring keeps about
6,000 records of history, about 100 quiet hours.

```json
{"cmd":"emi_log"}
{"cmd":"emi_log","offset":0,"count":16}
{"cmd":"emi_log","after":345,"count":16}
```

Without `count` it answers the log's state and the loop's (`emi_test`, the full object; `status`
carries a short one); with `count` (at most `max_count`: 16, or 12 over the cloud, so the reply fits
a connection's 2 KB send buffer) it returns that many raw 64-byte slots from slot `offset` on,
base64url without padding, in slot order, instead of `emi_test`. With `after` (a sequence number)
in place of `offset`, the read starts at the first record newer than it, and `count` is cut to the
records there are (0 when there are none): a viewer that keeps a copy asks for `after` its newest
`seq` until `count` is 0. An `after` older than the oldest record starts at the oldest. `offset` in
the reply says where the read started; `uptime_ms` is the pod's uptime, to turn a record's
`uptime_ms` in this run into a time of day. Without `count` the reply also carries `last_error` (the
loop's last failure, empty when none) and `fw`:

```json
{"status":"ok","data":{"base":1646592,"record":64,"slots":7040,"valid":345,"run":12,
  "next_seq":346,"write_slot":345,"oldest_slot":512,"queued":0,"lost":0,"written":18,"erases":4,
  "write_errors":0,"session_us_last":273,"session_us_max":2969,"glitch_us_last":0,"glitch_us_max":0,
  "dac_stops":0,"max_count":16,"uptime_ms":4821930,"offset":0,"count":16,
  "data":"RUwBAg..."}}
```

`run` is this boot's run number; `queued` records wait in RAM for the next write, `lost` found no
room there. A slot is empty (all `0xFF`) or a record: `u16 magic 0x4C45` ("EL"), `u8 type` (1
START, 2 EVENT, 3 CHECK, 4 SUM), `u8 led` (1 starting, 2 green, 3 red, 4 blue, 5 blue recovering), `u32 seq` (grows across
runs: sort by it), `u32 run`, `u32 uptime_ms`, 44 bytes of payload, and a CRC-32 (zlib's) of the
first 60 bytes; the payloads are in `stm32h563/src/emi_log.c`. `stm32h563/tools/emi/emi_log_dump.py`
reads the whole log and prints it with a summary per run.

The W25Q shares its bus with the PSRAM: a read with `count` while the EMI loop runs waits for no
capture to be in flight (else `"busy: an EMI capture is using the bus, retry in a moment"`; the loop
then holds its next capture back for 1.5 s, so a retry finds the bus free). Log writes and reads do
not interrupt the outputs: the sine plays from the gateware's BRAM table and the 0-5 V DC and
4-20 mA outputs are held by the quad DAC, none of which use the bus, so the DAC keeps running
through a W25Q session. Only a deep DAC replay (from PSRAM) would be stopped and started again;
the EMI loop never plays one. A session delays the loop's next capture instead: `session_us_last`
and `session_us_max` are its time (measured on an analog v3r2 pod: 0.3 to 0.4 ms for one record,
about 3 ms for a batch of 14, about 60 ms when a sector erase comes with it (the W25Q's worst case
is 400 ms), which happens only once the 1,024 records erased at boot are used up). `glitch_us_last` and `glitch_us_max` are the
time the DAC was found stopped after a session (0 normally) and `dac_stops` how many sessions did.
The whole log (7,040 slots) downloads in about 10 s over the LAN while the loop runs.

**The LED's verdict.** The color is the verdict for a test lab: green passes every test; blue
passes criterion B (blue, fast blink: a function fails now and the pod is getting itself running
again; blue, 1 s blink: it runs again after a failure or restart); red, fast blink, fails: a
function has been failing for 60 s without a break, or the recovery below ran out of steps. Red is
not latched: a pod that comes back after a long burst test turns blue, so the lab reads the LED
60 s after each test. An eFuse trip or an iCE40 that dropped out go through this too (no red at
once), and `emi_test` and the `emi` line name the state (`led`: `green`, `blue (recovering)`,
`blue`, `red`).

**Recovery.** While the loop runs, the EMI build gets itself running again when it stays stuck
(`stm32h563/src/recovery.h`): captures that keep failing for 10 s (or one that has not finished
after 10 s) reload the iCE40 from its config flash (no write), a second time 10 s later, and when
that does not help, the STM32 resets (10 s after the last reload, 30 s after the failure began);
after that reset the loop starts 2 s into the boot instead of 10 s, so the pod runs again well
inside a minute. An Ethernet link that stays down for 20 s, or stays up while frames keep failing
to go out, gets a PHY reset and a new DHCP request, at most once a minute. An eFuse that trips is
a failure (blue recovering, then blue) and its supply is turned off and on again 2 s later, at most 6 times an
hour (`eFuse 1 tripped: power retry`); past that the output stays off and counts as failing (`eFuse 1 tripped: off (retry limit)`), so the LED turns red 60 s later, until `emi stop` or a power cycle. A trip is FLT held for 30 ms (3 samples 10 ms apart), so a glitch on the FLT line does not count. The EMI build never goes to safe mode after crashed boots. Limits: 6 iCE40 reloads an hour, 2 STM32 resets until the pod ran
10 minutes without a failure or was powered off and on (the count survives the resets), 10 PHY
resets an hour; past them the LED is red while the failure lasts. Each step is an `EVENT` (kind 6, `recovery`) whose text
says what it did and how it went, e.g. `iCE40 reload ok v51` or `STM32 reset (1 of 2)`; the next
run's `START` then has the reset cause `recovery reset`.

On a board with the built-in Wi-Fi (ESP32-C3, v3 boards) a C3 restart while the activity runs is a
failure too (blue recovering for 5 s, then blue): the pod restarts the C3 when it stops answering, reboots by
itself or fails a setup step (see [`wifi_status`](#wifi_status)), so an ESD hit that resets it
shows. Each one is an `EVENT` (kind 7, `ESP32-C3 restart`) whose text has the cause, e.g.
`C3 restart: rebooted`, and `emi_test` counts them (`c3_restarts`).

Things the pod counts that the LED does not show are `EVENT`s too (kind 8, `health`), checked once
a second and logged at most once per item every 10 s (eFuse reverse blocking: every 60 s) with the
count since the last one: an I2C bus clear, a capture read-back from the PSRAM that needed a retry
or failed, a CAN bus-off recovery, eFuse reverse-current blocking (`eFuse1 rev block +31`, see
`target_status`), the iCE40 losing its configuration (CDONE low) and the LA supply switching source
(the TPS2116's status pin). They do not change the LED.

The cloud link is not under test (a lab network may have no internet), so it never changes the
LED, and a drop that reconnects is only counted. A cloud link that is set up but stays down for
2 minutes is a `health` event (`cloud down 2 min: idle timeout (no pong)`), and so is its return
(`cloud back after 7 min`): the shared lab view stops updating meanwhile.

Every 10 minutes while the loop runs, also when nothing went wrong, a `SUM` record (type 4) keeps
the totals, so after a failure the log shows which one started climbing first: cloud drops and
the seconds the cloud was down this run, ESP32-C3 restarts, I2C bus clears, CAN bus-off
recoveries, PSRAM read-back retries and failures, iCE40 configuration losses this run, eFuse
reverse blocks per eFuse, Ethernet frames received with a CRC or alignment error and frames
missed for want of a receive buffer, and the Wi-Fi bridge's failed Wi-Fi transmits (its own
count). Layout after the common header: `u16` cloud drops, C3 restarts, I2C clears, CAN bus-off,
PSRAM fails, iCE40 losses at bytes 16-27, then `u32` PSRAM retries, eFuse 1 and 2 reverse blocks,
Ethernet CRC, Ethernet missed, bridge transmit errors and cloud-down seconds at bytes 28-55,
then `u16` Wi-Fi bridge restarts and silences this run at bytes 56-59.

With a Wi-Fi bridge on the Ethernet port (the lab's configuration 2: the bridge's EMC test build
puts the pod's frames on the air), the bridge is part of the equipment under test: once the pod
has seen one this run, its failures are the pod's, as `health` events and on the LED (blue
recovering, red after 60 s): a restart (`bridge restart: brownout`, the reason from the bridge's
next STATUS), no HELLO for 10 s (`bridge silent 10 s`, then `bridge back after 14 s`), and its radio
reporting that it is not transmitting (`bridge radio not transmitting`, then `bridge radio back`).
`wifi_status` `bridge` shows the bridge's last boot (`boot`), the reboots the pod saw (`reboots`) and the EMC radio (`emc`). A restart because the access
point refused the association is left out: that is the network, not the pod. The digital v3r3
and the v4 boards have no C3. A hang of the STM32 itself is the watchdog's
(about 30 s). The console's `emi fault` makes such failures on purpose to test it.

**The lab page.** The EMI build also answers HTTP on port 80 of every interface it has an address
on: `http://<the pod's address>/` is a page for a test lab that shows the loop's state live (the
LED's meaning, the counters, the last error) and keeps a copy of the run log in the browser. It
fetches the newest 500 records the first time (all of them with "Load full history"), then only
what is new every 5 s, also after the network or the pod was down, and stamps records of the
running boot with the laptop's clock; "Download CSV" and "Download all (JSON)" save the copy, the
JSON with a status sample every 5 s. The page is built in from `stm32h563/tools/emi/lab_page.html`
and uses nothing from the internet. Besides `GET /` there are only two read-only paths, the two
calls the page makes: `GET /api/status` (`{"cmd":"emi_log"}`) and `GET /api/log?after=<seq>`
(`{"cmd":"emi_log","after":<seq>,"count":16}`); each answers that command's reply line as JSON.
Anything else is refused (404, or 405 for a method other than GET). HTTP/1.0, one request per
connection; a request takes one of the pod's 5 LAN connection slots while it runs.

---

<a id="scpi-reference"></a>
## SCPI reference

**Off by default from firmware 4.0.0.** Switch it on with `{"cmd":"lan_tls","scpi":true}` (over
USB or from the cloud) or `lan-tls scpi on` on the USB console; see
[`lan_tls`](#lan_tls--the-lan-api-over-tls). While it is off, a connection that starts with a
SCPI line gets one line, `SCPI is off on this BenchPod: ...`, and is closed, and mDNS has no
`_scpi-raw._tcp`. The cloud byte tunnels are not affected. Up to 3.10.13 it was always on.

Port `8080` also speaks SCPI (IEEE 488.2, on libscpi): a connection whose first byte is not `{`
is a SCPI connection for its whole life. It stays on port `8080`, plaintext: with SCPI on, `8080`
also takes connections from Ethernet and Wi-Fi under HTTPS-only, for SCPI only (a JSON line there
gets `BP0075`). `8443` speaks it too, wrapped in TLS. Lines end with `\n`, are at most 255 bytes, and several
commands can share a line with `;`. Queries answer one line; bulk samples come as comma-separated
ASCII. Errors go to the error queue (16 deep): read them with `SYSTem:ERRor?`. The instrument
state (`SOURce`, `SENSe`, the trace) is shared by every SCPI connection. Upper-case letters are
the short form; `[...]` is optional; `#` is a number in the header (`OUTPut:POWer1`).

**VISA.** The pod is a raw socket instrument. Its VISA resource string is
`TCPIP::<address>::8080::SOCKET`: the LAN address, the pod's USB network address over USB
(`TCPIP::10.237.x.y::8080::SOCKET`, from `status.usb.ip`), or its mDNS host name
(`TCPIP::benchpod-a1b2c3.local::8080::SOCKET`). Set the termination character to `\n` for reads and
writes. From firmware 3.9.0 the pod also advertises `_scpi-raw._tcp` over mDNS on each interface (from 4.0.0 only while SCPI is on),
so VISA and LXI discovery tools list it. SCPI follows the same rules as the JSON API there: the
external LAN policy applies on Ethernet and Wi-Fi, not on the USB link.

### IEEE 488.2 common commands

| Command | Description |
|---|---|
| `*IDN?` | `EmbeddedCI,BenchPod,0,<firmware version>`, for example `EmbeddedCI,BenchPod,0,3.7.0` (the same version as JSON `status`) |
| `*RST` | Stop the DAC and restore the `SOURce`/`SENSe` defaults (sine, 1 kHz, amplitude 127, offset 128, 256 points, maximum rates) and empty the trace |
| `*CLS`, `*ESE`, `*ESE?`, `*ESR?`, `*OPC`, `*OPC?`, `*SRE`, `*SRE?`, `*STB?`, `*TST?`, `*WAI` | Standard status and synchronization commands (libscpi core) |

### System

| Command | Description |
|---|---|
| `SYSTem:ERRor[:NEXT]?` | Next error: `<code>,"<text>"`, or `0,"No error"` |
| `SYSTem:ERRor:COUNt?` | Errors in the queue |
| `SYSTem:VERSion?` | SCPI version |
| `SYSTem:PING?` | `PONG` |
| `SYSTem:WIFI:STATe?` | `READY` when an interface has an address, else `DISCONNECTED` |
| `SYSTem:WIFI:RSSI?` | Always pushes `-200` on this firmware (no RSSI) |
| `SYSTem:COMMunicate:LAN:IPADdress?` | The pod's IP |
| `SYSTem:IDENtity:PUBlic?`, `SYSTem:IDENtity:POP? "<nonce>"` | Device identity, see [SCPI equivalents](#scpi-equivalents) |

### Signal generator (DAC)

| Command | Parameter | Description |
|---|---|---|
| `[SOURce]:FUNCtion[:SHAPe] <shape>` / `?` | `SINusoid`, `SQUare`, `RAMP` or `USER` | Waveform; `USER` replays the trace |
| `[SOURce]:FREQuency <Hz>` / `?` | > 0, unit suffixes allowed; `MIN` 1, `MAX` 1e6, `DEF` 1000 | Frequency |
| `[SOURce]:VOLTage[:AMPLitude] <0-255>` / `?` | 8-bit half-scale amplitude | A level, not volts (the JSON `generate` takes volts from firmware 4.0.0; SCPI keeps the 8-bit level) |
| `[SOURce]:VOLTage:OFFSet <0-255>` / `?` | 8-bit offset | A level, as `AMPLitude` |
| `[SOURce]:SRATe <MHz>` / `?` | ≥ 0; `0` = auto | DAC sample clock, also the `USER` replay rate |
| `[SOURce]:DURation <ms>` / `?` | ≥ 0; `0` = until stopped | Output duration |
| `OUTPut[:STATe] <ON\|OFF>` / `?` | | `ON` starts the selected waveform (as `generate`, or `replay` for `USER`); `OFF` stops the DAC |

### Acquisition (ADC and LA)

| Command | Parameter | Description |
|---|---|---|
| `SENSe:SWEep:POINts <n>` / `?` | 1..4096 | Default `READ?` and `MEASure?` length |
| `SENSe:SRATe <MHz>` / `?` | ≥ 0; `0` = maximum (about 0.4) | ADC sample clock for `READ?` |
| `READ? [<n>]` | 1..4096 | Capture `n` ADC samples and return them as CSV; also fills the trace (first 2048 samples) |
| `MEASure?` | | Play one period of the `SOURce` waveform across a `POINts`-long capture (as `measure`) and return the ADC samples; also fills the trace |
| `DIAGnostic:CAPture? [<adc_n>][,<la_n>]` | each 0..128, not both 0; default 16,16 | One-trigger ADC + LA capture (ADC 100 kS/s, LA 1 MS/s): `adc_n` ADC counts, then `la_n` LA words, as CSV. Each LA word holds LA1..LA14 in its low 14 bits (bit 0 = LA1). |
| `DIAGnostic:PATTern? <pattern>[,<value>[,<n>]]` | `SINusoid`, `COUNter`, `RAMP` or `CONStant`; value 0..255 (for `CONStant`, default 255); n 1..4096 (default 256) | An 8-bit synthetic pattern as CSV (no hardware) |
| `TRACe[:DATA] <offset>,"<base64url>"` | byte offset 0..4096 | Upload one chunk of a replay trace (16-bit little-endian samples, at most 4096 bytes in total) |
| `TRACe:POINts?` | | Samples in the trace (bytes ÷ 2) |

### Digital (LA pins)

| Command | Parameter | Description |
|---|---|---|
| `DIGital:OUTPut <la>,<0\|1>` | LA 1..14 | Make the pin a gpio output at that level (the JSON `gpio` table applies) |
| `DIGital:STEP <la>,<steps>,<delay_us>[,<dir_la>[,<dir>]]` | as the JSON [`la_step`](#step-a-pulse-train) | Start a step pulse train (non-blocking) |
| `DIGital:STEP:BUSY?` | | `1` while a step train runs, else `0` |

### Target power

| Command | Description |
|---|---|
| `OUTPut:POWer<n>[:STATe] <ON\|OFF>` / `?` | eFuse `n` (1 = internal 5 V, 2 = external) on or off |
| `OUTPut:POWer<n>:FAULt?` | `1` when the eFuse reports a fault |
| `OUTPut:POWer<n>:VALid?` | `1` when the eFuse's input is valid |

### SCPI errors

| Code | When |
|---|---|
| `-200 Execution error` | The hardware is busy (a JSON capture or upload holds it), a capture or waveform failed, `OUTPut ON` with `USER` and an empty trace, `SYSTem:WIFI:RSSI?`, or a setter while a cloud job holds the pod (queries still answer) |
| `-221 Settings conflict` | DAC limits are set (`OUTPut ON`, `MEASure?`), or an LA pin belongs to another function (`DIGital:*`) |
| `-222 Data out of range` | A parameter outside the ranges above, or an eFuse other than 1 or 2 |
| `-224 Illegal parameter value` | Bad base64url (`TRACe:DATA`, `SYSTem:IDENtity:POP?`) or a bad `FREQuency` keyword |
| `-240 Hardware error` | Safe mode: the iCE40/PSRAM are off this boot, so `OUTPut`, `READ?`, `MEASure?`, `DIAG:CAP?`, `TRACe:DATA` and `DIGital:*` are refused (the error text is the JSON refusal) |
| `-241 Hardware missing` | The digital-only board has no analog front end: the analog commands are refused (`DIAG:CAP?` for the LA alone still works) |
| `-300 Device specific error` | `DIGital:STEP` while a step train is running |
| `-310 System error` | The device identity is not available |
| Other libscpi codes | Malformed input, for example `-113 Undefined header` |

---

## Signal Parameters Reference

### Waveform types

| `waveform` | Description |
|---|---|
| `"sine"` | Sinusoidal. Smooth output, good for frequency response testing |
| `"square"` | 50% duty-cycle square wave. Useful for rise/fall time and bandwidth tests |
| `"sawtooth"` | Linear ramp from `offset - amplitude` to `offset + amplitude` per period |

### Amplitude / offset levels (`measure` and SCPI)

`measure` and the SCPI `SOURce:VOLTage` commands take 8-bit levels: `amplitude` 0-127 and `offset`
0-255, played in the DAC's high byte with no calibration. `generate` takes volts or microamps
instead (firmware 4.0.0).

| Goal | `amplitude` | `offset` |
|---|---|---|
| Full-scale, centred | `127` | `128` |
| Half-scale, centred | `64` | `128` |
| Low-side only (0–127) | `64` | `63` |
| High-side only (128–255) | `64` | `192` |
| Minimum swing | `1` | any |

### ADC sample values

ADC samples are raw 16-bit counts (0–65535) from the MCP33131 ADC, with no calibration or
scaling applied. The front end in front of the ADC input SMA is bipolar and divides by about 12,
so the count is not a simple fraction of a reference. To convert, use the affine fit the pod
announces (`adc_cal_a_uv`, `adc_cal_b_nv`, `adc_cal_unwrap`, see
[Capabilities frame](#capabilities-frame--adc-calibration-fields)):

```
if adc_cal_unwrap and count < 32768: count += 65536
V = (adc_cal_a_uv + adc_cal_b_nv / 1000 × count) / 1e6
```

For a single calibrated reading of any source, use [`adc_read`](#adc_read--route-a-source-and-return-a-calibrated-reading).

---

## Error Reference

Coded refusals (firmware 4.0.0) carry `code` and `args` only; the message, cause and fix of each
are in [`errors/ERRORS.md`](../errors/ERRORS.md) and [`errors/errors.json`](../errors/errors.json).
The ones the commands above name:

| Code | Name | Args | Raised by |
|---|---|---|---|
| `BP0001` | `la_voltage_not_set` | | an LA command before the I/O voltage is set |
| `BP0002` | `unknown_command` | `cmd` | a `cmd` the firmware does not know |
| `BP0003`, `BP0004` | `i2c_session_open`, `spi_session_open` | | `i2c_start` / `spi_start` while a session is open |
| `BP0005` | `bus_engine_busy` | `want` | SWD, SPI or I2C while another of them runs |
| `BP0017` | `gateware_too_old` | `feature`, `min` | a feature the running gateware is too old for |
| `BP0025` | `lease_busy` | `holder`, `purpose`, `left_s` | a command while someone else holds the pod's lease (a cloud job: holder `a cloud job`) |
| `BP0030` | `client_too_old` | `client_api`, `min_client_api`, `api` | a client below `min_client_api` (it also carries a `message`) |
| `BP0031` | `la_pin_in_use` | `la`, `owner`, `function`, `instance` | claiming an LA pin another function holds ([pin ownership](#la-pin-ownership)) |
| `BP0032`, `BP0033` | `la_pull_down_engaged`, `la_pull_down_conflict` | `la`, `ohms`, `function`, `reason` | a function and an engaged pull-down that cannot share a line |
| `BP0034`, `BP0035` | `la_dut_output`, `la_driven_pin` | `la`, `function` | driving a pin `la_dir` marks as a DUT output, or marking a driven pin |
| `BP0036` | `la_not_gpio_output` | `la`, `function` | writing a level or a gpio send on a pin that is not a GPIO output |
| `BP0037` | `la_voltage_pins_in_use` | `pins` | `la_voltage` while pins are in use |
| `BP0038`, `BP0039` | `trigger_timeout`, `sensor_trigger_timeout` | `la`, `edge` or `instance`, `timeout_ms` | a [capture trigger](#capture-triggers) that never fired |
| `BP0040` | `lan_locked` | `cmd` | a T2 or T3 command on a locked LAN |
| `BP0041` to `BP0044` | `policy_not_from_lan`, `setting_cloud_or_usb`, `setting_device_only`, `policy_loosen_usb_only` | `setting` (and `now`) | a [pod policy](#pod-policies) or a trust setting changed from the wrong place |
| `BP0045` | `role_required` | `cmd` | a cloud tunnel without the role a command needs |
| `BP0046` | `no_analog_front_end` | `cmd` | an analog command on the digital board |
| `BP0047` | `pod_busy` | | a capture, measurement, upload or update while another one runs |
| `BP0048` | `storage_bus_busy` | `what` | a storage write while the shared bus is in use |
| `BP0049`, `BP0050` | `uart_session_open`, `swd_session_open` | | `uart_proxy_start` / `dap_start` while one runs |
| `BP0051` | `update_in_progress` | `target`, `holder` | an `ota_*` command while another transport holds the update |
| `BP0052` | `cloud_command_busy` | | a cloud command while the previous one runs |
| `BP0053`, `BP0054` | `reply_too_large`, `cloud_reply_too_large` | (`bytes`, `limit`) | a reply larger than the pod (or the cloud channel) can send |
| `BP0055` | `sensor_instance_unknown` | `instance` | a `sensor_*` command naming an instance that is not there |
| `BP0056` | `no_module_link` | `cmd` | `module` / `module_power` on a board without the module link |
| `BP0057` to `BP0059` | `image_switch_sensors_running`, `fpga_image_required`, `fpga_image_sensors_first` | `sensors` / `feature`, `image` | the [gateware images](#closed-loop-dac-gateware-images-and-psram) |
| `BP0060` | `ota_unknown_target` | `target` | an `ota_begin` target this firmware does not know |
| `BP0061` to `BP0063` | `i2c_needs_pullups`, `i2c_session_not_open`, `i2c_engine_reset` | (`la`) | the [I2C master](#i2c-master) |
| `BP0064` | `loop_sweep_with_map` | | `dac_control_loop` sweep with an input map |
| `BP0065` | `output_used_by_sensor` | `output`, `instance` | a DAC or current command while an analog emulated sensor plays that output |
| `BP0066` | `step_busy` | | an `la_step` train while one runs |
| `BP0067` | `samples_out_of_range` | `max` | `capture`, `capture_read`, `la_capture` and the others with `samples` outside 1 to `max` |
| `BP0068`, `BP0069` | `la_rate_above_ceiling`, `la_rate_too_deep` | `rate_mhz`, `fit_mhz`, `samples` (and `max_mhz` / `max_samples`) | an explicit raw LA rate the capture cannot run: the rates are strings with one decimal ("3.0"), in MS/s |
| `BP0070` | `capture_dual_too_deep` | `la_max`, `adc_max`, `la_capture_max` | `capture_dual` asking for more than a stream's region |
| `BP0071` | `capture_overflowed` | `la_ksps`, `adc_ksps` | a capture whose streams the PSRAM could not keep up with |
| `BP0072`, `BP0073` | `psram_too_small`, `psram_bandwidth` | | `capture_dual` whose regions do not fit the PSRAM, or whose rates overrun its bus |
| `BP0080` | `dac_pinout_mismatch` | `cmd`, `digital`, `module` | a command that drives the DAC on a digital board and an analog module with different [flat-cable pinouts](#dac-pinout-on-the-flat-cable) |

Refusals without a code keep `{"status":"error","message":...}`; the common ones:

| Message | Cause |
|---|---|
| `"missing cmd"` | Request JSON has no `"cmd"` field |
| `"command too long"` | A JSON line longer than 1279 bytes |
| `"command not supported over cloud channel"` | A streaming command on the single-reply cloud command channel (see [Commands](#commands)) |
| `"safe mode: ..."` | The safe-mode [gate](#gates-that-apply-to-every-command) |
| `"unknown waveform"` | `generate` `waveform` value not `sine`, `square`, or `sawtooth` |
| `"generate failed"` | Invalid `freq_hz` (<= 0) |
| `"capture failed"` | The capture did not arm or did not complete |
| `"invalid swd la channel"`, `"invalid uart args"` | a channel outside 1..14, or two equal channels |
| `"invalid definition: <path>: <message>"` | `sensor_define` whose definition does not compile (see [Emulated sensors](#emulated-sensors)) |
| `"capture data was overwritten by <what>; run the capture again"` | `capture_read` after something wrote over the capture's PSRAM regions |
| `"safe mode: the PSRAM is off this boot, so an update cannot be staged; ..."` | an update begin in a safe mode that turned the iCE40/PSRAM off |
| `BP0043` (`setting_device_only`) | `identity_wipe`, `sig_policy` `clear_floor` or `wifi_bridge` `cert` off the USB link |

Over SCPI the same refusals arrive in the error queue (`SYSTem:ERRor?`) with their text, for a
person: safe mode as `-240 Hardware error` and the digital-only board as `-241 Hardware missing`;
a pin conflict, a busy capture lock, or a trace that a JSON command replaced, as `-200 Execution
error`; and the DAC limits as `-221 Settings conflict` with the limits' reason. The USB console
prints its own text too.

---

## Session Example

```
$ nc 192.168.1.220 8080

{"cmd":"ping"}
{"status":"ok","data":"pong"}

{"cmd":"la_voltage","mv":3300}
{"status":"ok","data":{"mv":3300,"st":1,"readback_mv":3297}}

{"cmd":"target_power","efuse":1,"enabled":true}
{"status":"ok","data":{"efuse":1,"enabled":true,"delay_ms":0}}

{"cmd":"generate","waveform":"sine","freq_hz":1000,"path":"3v3","amplitude_v":1.2,"offset_v":1.65}
{"status":"ok","data":null}

{"cmd":"capture","samples":512}
{"status":"ok","bits":16,"data":[32768,33170,33571,...],"more":true}
{"status":"chunk","data":[...],"more":false}

{"cmd":"dac_stop"}
{"status":"ok","data":null}

{"cmd":"la_pull","la":1,"pull":"up"}
{"status":"ok","data":{"la":1,"state":"up","dirs":["up"],"pullups_available":true}}

{"cmd":"la_step","la":1,"steps":200,"delay_us":500}
{"status":"ok","data":{"la":1,"steps":200,"delay_us":500,"status":"started"}}

{"cmd":"identity_public"}
{"status":"ok","data":{"public":"ue5UfSsOAhQYZ332ELokjOI8-uyjcbrd1ic3Nmypvzk"}}

{"cmd":"cloud_set","host":"www.embeddedci.com","device_id":"<uuid>","tls":true}
{"status":"error","code":"BP0040","args":{"cmd":"cloud_set"}}
```

The last reply is what a pod with the LAN policy `locked` answers; on an `open` LAN it stores the
settings.

---

## Pod policies

See `docs/design/policy-commands.md` for the full rules.

### `sig_policy` — which firmware and blob updates the pod accepts

```json
{"cmd":"sig_policy"}
{"cmd":"sig_policy","set":"required"}
```

```json
{"status":"ok","data":{"policy":"required","enforces":true,"keys":3,"rollback_floor":null}}
```

**Firmware 4.0.0 and later install only signed images, and nothing turns that off.** The policy
is always `required`: an update of the firmware, the gateware (`gw0`, `gw1`, `gw2`) or the
ESP32-C3 image without a good signature from a key the firmware trusts is refused at
`ota_begin` with `signature: <result>`, on every path: Ethernet and Wi-Fi, the USB network
adapter and the cloud (WS `ota.begin`). Every published release and its blobs are signed, so
`benchpod flash-self --network` and the web app's install are not affected. A development build
(not `RELEASE=1`) also trusts the developer key, so a dev-signed image (`tools/fwsign.py`) goes in
over the network on a bench pod running a dev build; a release build trusts the release keys
only. `keys` counts the keys this build trusts. The `ca` and `tls` targets are configuration,
not code, and are not signed.

`"set":"required"` is accepted from every source (a server pushing it keeps working);
`"set":"audit"` or `"permissive"` is refused from every source, the USB console included, with
`BP0074` (`signatures_always_required`, args `setting` `sig_policy` and `want`). So is
`"lan_unsigned":true`, the 3.10.7 to 3.10.13 opt-out for Ethernet and Wi-Fi (setting
`lan_unsigned`); the reply has no `lan_unsigned` field any more. A pod updated from 3.10.x reads
its stored `audit` or `permissive` as `required` and stores `required` with the next policy save,
so a rollback to 3.10.x keeps it.

**Firmware you built yourself goes in over USB DFU**: `benchpod flash-self` (without
`--network`), which reboots the pod into the MCU's ROM bootloader with the console's `dfu`
command, or BOOT held while tapping RESET. The ROM bootloader writes any image; it needs the pod
on a USB cable. (3.10.7 to 3.10.13 refused `dfu` under `required`.)

Up to 3.10.13: `audit` (the default) accepted every update and reported its check, `permissive`
refused a bad signature, the cloud could only tighten the policy and the USB console set any
value; from 3.10.7 an update over Ethernet or Wi-Fi needed a good signature unless
`lan_unsigned` was on.

**ESP32-C3 images are always signed** (firmware 3.10.7+), like the Wi-Fi bridge's firmware: the
C3 sees the Wi-Fi password and every Wi-Fi frame. An `esp` image without a good signature gets
`signature: none (esp images must be signed)` from every transport.

**Rollback floor** (firmware 3.10.7+). `rollback_floor` is the oldest release (`"3.10.7"`, or
`null` for none, the default) whose signed firmware the pod still takes: a signed firmware image
older than that gets `rollback: <its release> is below this pod's floor <floor>`. The floor only
goes up, when a signed release marked as a floor release (a flag in its signed manifest) is
installed, so an older release with a known security problem cannot be put back later. No
published release sets it yet. `{"cmd":"sig_policy","clear_floor":true}` over the USB link lowers it
to none (Ethernet, Wi-Fi and the cloud get `BP0043`, setting `rollback_floor`); up to firmware
3.10.13 that was the USB console's `rollback-floor clear`.

### `lan_policy` — what the external LAN may do

```json
{"cmd":"lan_policy"}
{"cmd":"lan_policy","set":"locked"}
{"cmd":"lan_policy","pairing":false}
```

```json
{"status":"ok","data":{"policy":"locked","pairing":true}}
```

`pairing` (firmware 4.0.1+, default `true`): computers on the external LAN must be
[paired](#lan_clients--paired-computers). The policy below then applies to paired computers; an
unpaired one only reads status. Changed over USB (the console's `lan-pairing on|off`, or the USB
link) or from the cloud, never from Ethernet or Wi-Fi: `BP0041`.

The policy covers the **external LAN**: Ethernet and Wi-Fi. `open` (default), `locked` (a T2/T3
command there gets `BP0040`, `lan_locked`) or `off` (connections to
8080 over Ethernet or Wi-Fi are refused and open ones closed, no mDNS there). The [USB network
adapter](usb-serial-interface.md#usb-network-adapter) stays on and open under every policy: it
means physical presence. It is still LAN tier, so changing `lan_policy` stays off it too; the
exceptions are what needs someone at the pod: the company CA and the cloud proxy (see `cloud_ca` /
`cloud_proxy`), `identity_wipe`, the rollback floor and a Wi-Fi bridge certificate. The wire
values and the field name are unchanged. Set from the cloud or the USB console (`lan-policy
<value>`), never from the LAN or the USB network.

### `usb_network` — the USB network adapter on or off

```json
{"cmd":"usb_network"}
{"cmd":"usb_network","enabled":false}
```

```json
{"status":"ok","data":{"enabled":false}}
```

Firmware up to 3.10.13 took `"set":"on"|"off"` and replied `"setting":"on"|"off"`.

Whether the pod's USB port is also a [USB network
adapter](usb-serial-interface.md#usb-network-adapter) (default `on`). With `off` the pod
re-enumerates as its USB console only: the computer sees no network adapter at all. The setting is
stored, separate from `lan_policy` (which covers Ethernet and Wi-Fi), and survives a reboot and a
firmware update. Only the device changes it: the USB console (`usb-network on|off`, the way back
on) or a connection over the USB link itself (which can turn it off). Ethernet, Wi-Fi and the cloud
get `BP0043` (`setting_device_only`, setting `usb_network`); reading works
everywhere. The reply comes first; the pod re-enumerates about 0.4 s later. Stored Wi-Fi
credentials also turn the adapter off (status `usb.off_reason`).

### `log_level` — the console log level

Firmware 3.10.13+ (feature `log_level`).

```json
{"cmd":"log_level"}
{"cmd":"log_level","set":"debug"}
```

```json
{"status":"ok","data":{"level":"debug"}}
```

How much the pod prints on its USB console (and its USART2 pins): `error`, `warn`, `info` (the
default) or `debug`. The levels and what each prints are in [Log
levels](usb-serial-interface.md#log-levels); `debug` adds the per-command trace (`[cmd] <- ...`,
`[cmd] -> ok ...`) and the retry detail of the cloud, Wi-Fi and DHCP loops. Errors always print.
The setting is stored on the pod (it survives a reboot and a firmware update) and is the same one
as the console's `log` command. It changes only console output, so every connection may read and
set it (T0). Another value gets `log_level: set must be error, warn, info or debug`.

### `lan_clients` — paired computers

Firmware 4.0.1+. See `docs/design/lan-pairing.md`.

```json
{"cmd":"lan_clients"}
{"cmd":"lan_clients","offset":12}
{"cmd":"lan_clients","add":"ci-runner-1","spki_sha256":"3f0c…64 hex digits"}
{"cmd":"lan_clients","remove":"ci-runner-1"}
{"cmd":"lan_clients","remove":"3f0c9a1e"}
{"cmd":"lan_clients","clear":true}
```

```json
{"status":"ok","data":{"pairing":true,"count":2,"max":32,"offset":0,
 "clients":[{"name":"laptop","spki_sha256":"3f0c…"},{"name":"ci-runner-1","spki_sha256":"9b2e…"}]}}
```

A computer is known by `spki_sha256`: the SHA-256 of the DER SubjectPublicKeyInfo of the
certificate it presents on port `8443` (any self-signed certificate; the pod has no CA). It may
renew its certificate with the same key. Port `8443` asks every client for a certificate; one
without a certificate still connects, unpaired. mbedTLS checks the client's CertificateVerify, so a
client that sends a certificate holds its key.

- `add`: a name (1 to 23 printable characters, no quotes or backslashes) and the key hash. A key
  already paired gets the new name. Full: `BP0082` (`lan_clients_full`, args `max`).
- `remove`: every computer with that name, or the one key that starts with 8 or more hex digits
  (two matches remove neither).
- `clear`: forget them all.
- The list comes back after a change, 12 at a time (`offset` for the next 12; `count` is the total).
  `max` is 32, fewer when a large user certificate fills the record they share.

Changes come over the USB link or from the cloud (the console has `lan-clients remove` and
`lan-clients clear`), never from Ethernet or Wi-Fi, paired or not: `BP0041`. The list is stored with
the TLS certificate, so regenerating or replacing the certificate keeps it. Rolled back to firmware
4.0.0, the pod keeps its certificate (clients' pins stay valid) and forgets the list.

### `lan_tls` — the LAN API over TLS

Firmware 3.9.3+. See `docs/design/lan-tls.md`.

```json
{"cmd":"lan_tls"}
{"cmd":"lan_tls","https_only":false}
{"cmd":"lan_tls","scpi":true}
{"cmd":"lan_tls","regenerate":true}
```

```json
{"status":"ok","data":{"port":8443,"https_only":true,"scpi":false,"sessions":1,"origin":"generated",
 "key":"EC P-256","subject":"CN=benchpod-baea06","san":["benchpod-baea06","benchpod-baea06.local"],
 "not_after":"9999-12-31","sha256":"3f1c...e9"}}
```

The pod also serves this API on TCP `8443` with TLS 1.2 (ECDHE, AES-GCM or ChaCha20-Poly1305). A
full handshake takes the pod about 0.46 s with its EC key (about 1.1 s with an RSA 2048 one); it
resumes sessions (session tickets and a 4-entry session cache, one hour), which takes about 20 ms, so
clients that open a connection per command should keep the session. On
the first boot of firmware with LAN TLS it generates an ECDSA P-256 key and a self-signed
certificate for `benchpod-xxxxxx` and `benchpod-xxxxxx.local` (its mDNS name), valid to 9999 (the
pod has no clock), and keeps both in its internal config flash. `sha256` is the SHA-256 of the
certificate (DER), the value clients pin. `origin` is `generated` or `user`. `sessions` is the
number of TLS connections open now: at most 3 at once (each needs about 22 KB of RAM), a fourth is
refused at connect, and so is one the pod has no room for; plaintext connections do not count.
Firmware 3.9.3 to 3.9.6 allowed 2, so a UART session, a capture and one more command over TLS
could not run together there. When there is no certificate the reply has
`error` instead of the certificate fields.

| Field | Effect |
|---|---|
| `https_only` | `true`: plaintext connections to `8080` over Ethernet and Wi-Fi are refused and open ones closed, so only `8443` answers there. `8080` keeps answering on the [USB network adapter](usb-serial-interface.md#usb-network-adapter). Default `true` from firmware 4.0.0 (a pod updated from 3.10.x gets it too, whatever it had); `false` before. `false` turns plaintext JSON on `8080` on for the LAN again. Stored; survives reboots and updates. |
| `scpi` | `true`: SCPI answers on the pod's TCP connections (`8080` from every interface, for SCPI only on the LAN under HTTPS-only, and `8443`), and mDNS advertises `_scpi-raw._tcp`. Default `false` (firmware 4.0.0; always on before). `false` closes the plaintext LAN connections (under HTTPS-only) and ends open SCPI sessions at their next line. Stored. |
| `regenerate` | `true`: a new key and self-signed certificate. New connections get it; open ones keep the old one. Clients that pinned the old one must trust the new one (`benchpod lan-tls trust`). |

**Your own certificate.** Upload a PEM bundle, the certificate (optionally followed by its chain)
and its unencrypted private key (EC P-256/P-384 or RSA 2048-4096), with the upload path as target
`tls`: `ota_begin` with `"target":"tls"` and the bundle's `size` and `sha256`, `ota_data` chunks,
`ota_end` (the pod checks that both parse and that the key matches the certificate) and
`ota_commit` (stored and used for new connections; the reply's `state` is `installed`). At most
7.6 KB. `benchpod lan-tls set-cert --cert pod.pem --key pod.key` does all of it.

**Who may change it.** Reading works from anywhere. Changing `https_only` (on or off), `scpi`,
`regenerate` and the upload need the USB console (`lan-tls https-only on|off`, `lan-tls scpi
on|off`), the USB network adapter (`--connection usb`) or the cloud, the way the LAN policy is
changed (the cloud needs an organization owner or admin: T2).
The external LAN (Ethernet, Wi-Fi) gets `BP0042` (`setting_cloud_or_usb`, setting `lan_tls`), over
plaintext and over TLS alike: port `8443` has no client authentication, so a TLS connection does
not show who is on the other end. Firmware 3.9.3 to 3.10.6 let a LAN connection over TLS make all
of these changes and let plaintext LAN turn `https_only` on. Reading is T0; the changes
are T2 (so `locked` keeps them off the external LAN). mDNS: the `_benchpod._tcp` TXT record carries
`id`, `host`, `port=8080` (the plaintext port, which answers on the USB link), `tls=8443`,
`https_only=0|1` (`1`, the 4.0.0 default: the LAN takes only TLS) and `api=<n>` (the
[client API level](#client-compatibility), firmware 4.0.0). The SRV record stays on `8080`, so a
client that reads it tries `8443` first the way `benchpod` and the SDK do for `8080`.

**Pinning needs no user action.** The certificate is generated once and kept across reboots and
updates (and across a rollback to 3.10.x), so a client that pinned it on first use keeps
connecting. Firmware 4.0.0 does not change the certificate or how it is made. Only `regenerate`,
an uploaded certificate or a damaged stored one (the pod makes a new one at boot) replace it;
then a pinned client
refuses the pod until the user trusts the new one (`benchpod lan-tls trust`). Clients pin per
address (host:8443).

**Raw sockets and VISA.** `8443` speaks the same SCPI as `8080`, wrapped in TLS. VISA's raw socket
resource (`TCPIP::<host>::8080::SOCKET`) has no TLS: with SCPI on it answers on `8080` from the LAN too, or run a local TLS
wrapper and point VISA at it, for example with the pod's certificate from
`benchpod lan-tls show --pem > pod.pem`:

```bash
socat TCP-LISTEN:5025,reuseaddr,fork OPENSSL:benchpod-baea06.local:8443,cafile=pod.pem
```

then use `TCPIP::127.0.0.1::5025::SOCKET`.

### `cloud_ca` / `cloud_proxy`: company CA and HTTP proxy for the cloud link

See `docs/design/cloud-hardening.md` section 3.

```json
{"cmd":"cloud_ca"}
{"cmd":"cloud_ca","clear":true}
{"cmd":"cloud_proxy"}
{"cmd":"cloud_proxy","set":"proxy.corp:3128","user":"u","password":"p"}
{"cmd":"cloud_proxy","clear":true}
```

`cloud_ca` replies `{"present":true,"certs":[{"subject":"...","sha256":"<hex>"}]}`. A company CA is
installed with the upload path as target `ca` (`ota_begin` with `"target":"ca"` from the cloud, WS
`ota.begin`, `ota_begin` over the USB network adapter, or the USB console's `upload-begin ca` on
firmware before 3.9.0; `benchpod cloud ca set corp.pem` picks the way); it
must parse as X.509 and is trusted in addition to the built-in roots.
`cloud_proxy` replies `{"host":"...","port":3128,"auth":true}` or `{}`; the password is never shown.
The pod sends `CONNECT <server>:443` with the server's host name and, when set, Basic proxy auth.
Setting or clearing either reconnects the cloud. Reading is T0; setting and clearing are T2.

**Never from the external LAN.** Installing or clearing the CA and setting or clearing the proxy are
refused on an Ethernet or Wi-Fi TCP connection whatever the LAN policy (even `open`), with
`BP0042` (`setting_cloud_or_usb`, setting `cloud_ca` or `cloud_proxy`). Reading both (`{"cmd":"cloud_ca"}`,
`{"cmd":"cloud_proxy"}`) still works on the LAN. The reason: a LAN attacker who could install their
own CA and point the pod at their own proxy could terminate the pod's TLS with a certificate for the
real server name and relay the host-bound (v2) login, which defeats it.

**Allowed over the USB network adapter** (firmware 3.9.0+). A connection on the USB link is
physical presence, like the USB console (anything on that computer can open the console anyway),
so it may install or clear the CA and set or clear the proxy. This is how `benchpod cloud ca` and
`cloud proxy` (and `usb-bridge`) change them over USB. From firmware 4.0.0 the console has no CA or
proxy commands: the USB link and the cloud are the ways. Everything else on the USB link stays LAN
tier.

### While a cloud job holds the pod

When a cloud consumer (a CI run, the web app, the SDK) holds the pod's lease, LAN connections get
only light reads (`status`, `ping`, the `*_status` reads, policy reads) and
[pod administration](#pod-administration-passes-a-lease); everything else gets
`busy: a cloud job holds this pod (<holder>, <N> s left)`, and SCPI setters get -200. The pod ends
the lease itself at the announced expiry or when the cloud link drops. `status` shows
`"lease":{"held":...,"holder":"...","left_s":...}`.

<a id="pod-lease"></a>
## Pod lease

A client takes the pod for a while (a CI run, an SDK session) with `lease_acquire`, on any
transport: LAN, the USB network adapter, the USB console or the cloud. The pod keeps one lease in
RAM: an id, a holder label, a purpose and a deadline. A restart clears it. Feature `lease` in the
[features table](#features-table).

While the lease is held, only its **owners** drive the pod. An owner is a connection that sent
`lease_acquire` with the held id; other processes of the same run join by sending `lease_acquire`
with the same id. Every other connection, on any transport, gets light reads only (the T0
commands that do not use the capture hardware, see [Gates](#gates-that-apply-to-every-command));
anything else is refused with `BP0025`, and SCPI setters get -200. A connection that closes is no
longer an owner; the lease itself stays until it is released, runs out or is stopped.

**The `lease` field on any command.** A client that opens a connection per command proves
ownership on each command instead of joining first: any JSON command may carry
`"lease":"<id>"`.

```json
{"cmd":"la_dir","lease":"L-6f1c..."}
```

With the held id, the command runs as the owner's (and its connection becomes an owner). Another
id, or an id while nothing is held, counts as no id: refused with `BP0025` while a lease is held,
run as usual when the pod is free. A command never moves the deadline; only `lease_acquire`
with the held id and `lease_renew` do. Commands without the field work as before. The check runs
before the command starts, so the commands that switch the connection to raw bytes or chunks
(`dap_start`, `uart_proxy_start`, `load_bin`, the captures) take it on their start line; the bytes
that follow belong to that session. `status` with the held id reports `local_lease.mine: true`.

The `lease_*` commands always reach the pod's lease, whoever holds it, so a busy pod answers
with its holder.

<a id="pod-administration-passes-a-lease"></a>
**Pod administration passes a lease** (firmware 4.0.2+). The commands that say who may reach the
pod over the LAN do not drive the hardware, so they run while someone else holds the pod's lease
or a cloud job holds the pod: the owner at the USB cable, or over the cloud, can unpair a computer
that keeps a session open, pair a new one, or change the LAN policy. Every other gate still
applies: an unpaired LAN computer gets `BP0081`, a locked LAN `BP0040`, a cloud tunnel its user's
tier (`BP0045`), and the writes still come only from USB or the cloud (see each command). Before
4.0.2 their write forms got `BP0025` like any other command.

Commands that pass a lease: `lan_clients`, `lan_policy` and `lan_tls`.

`sig_policy` is not among them: it only matters for a firmware update, which needs the lease
anyway.

### `lease_acquire`

```json
{"cmd":"lease_acquire","id":"L-6f1c...","holder":"benchpod ci: ward","purpose":"process-node-hil","ttl_s":300}
```

| Parameter | Type | Required | Description |
|---|---|---|---|
| `id` | string | yes | 1 to 64 printable characters, no spaces, quotes or backslashes. The token that makes a connection an owner: keep it to the processes of the run. |
| `holder` | string | no | Who holds it, as people should see it (40 characters; quotes and backslashes become `?`). Default `unnamed`. |
| `purpose` | string | no | What it is for (64 characters). |
| `ttl_s` | integer | no | Seconds until it runs out, 10 to 1800 (clamped). Default 120. |

Free, or held with the same id: the pod takes it (or pushes its deadline) and this connection
becomes an owner. The holder and purpose stay those of the first `lease_acquire`.

```json
{"status":"ok","data":{"lease":{"id":"L-6f1c...","holder":"benchpod ci: ward","purpose":"process-node-hil","left_s":300,"source":"local"}}}
```

Held with another id, or a cloud job holds the pod (the server's lease, `status` `lease`; then
`purpose` is `cloud`):

```json
{"status":"error","code":"BP0025","args":{"holder":"benchpod ci: ward","purpose":"process-node-hil","left_s":245}}
```

A missing or malformed `id` is `BP0006` (`args.arg` `id`).

### `lease_renew`

```json
{"cmd":"lease_renew","id":"L-6f1c...","ttl_s":300}
```

Pushes the deadline of the held lease with this id to `ttl_s` (10 to 1800, default 120) from now
and answers like `lease_acquire`. It does not make the connection an owner. Another id while held:
`BP0025`. Nothing held (it ran out, was released, or the pod restarted): `BP0027`
(`args.id`). A stopped lease's id: `BP0026` (see below). Renew well before the deadline: a client that crashed loses the lease when it runs out.

### `lease_release`

```json
{"cmd":"lease_release","id":"L-6f1c..."}
```

Frees the held lease with this id at once: `{"status":"ok","data":{"released":true}}`. Nothing
held: `{"status":"ok","data":{"released":false}}`. Another id while held: `BP0025`.

### Stopped from the web app or the cloud

The server can stop the lease (frame `{"type":"lease.break","by":"<name>"}` on the cloud link).
Each connection that owned it then gets, for its next command only, instead of running it:

```json
{"status":"error","code":"BP0026","args":{"by":"<name>"}}
```

A stopped lease stays stopped: for 30 minutes after the stop, `lease_acquire`, `lease_renew` and
any command whose `lease` field carries its id get the same `BP0026` (a light read too), instead
of a new lease or `BP0027`. The pod remembers the last stopped id only. Other ids, and commands
without one, see a free pod. A lease that ran out without being stopped is not remembered: the
same id may be acquired again.

### In `status` and on the cloud link

`status` with `"caps":false` has `"local_lease":{"held":true,"holder":"...","purpose":"...","left_s":N,"mine":true}`
(`mine`: this connection owns it), or `"local_lease":{"held":false}`. It never shows the id.

While the cloud link is up, the pod tells the server about its lease on every change, when the
link comes up, and every 30 s while it is held:

```json
{"type":"lease.local","held":true,"holder":"benchpod ci: ward","purpose":"process-node-hil","expires_in_s":245}
{"type":"lease.local","held":false}
```

The cloud command channel is one connection for every cloud request, so a `lease_acquire` sent
there makes that channel an owner until the lease ends; the cloud byte tunnels are separate
connections, each an owner only while it is open.

---

## Implementation Notes

- **JSON parser limitations:** The firmware uses a minimal flat-JSON parser. Nested objects are not supported, except the `trigger` object of the captures; arrays only where a command documents one (`data` of `can_write`, `reply_data` of `can_respond`). Fields must be at the top level of the JSON object, and unknown fields are ignored.
- **No authentication:** Any TCP client that can reach port 8080 (or 8443, its TLS form) has the full command set (target power, SWD access to an attached target, and pod OTA) unless the LAN policy is `locked` or `off`. Restrict access at the firewall/VLAN level. The cloud channel is unaffected (TLS + Ed25519 device auth). See [Security model](#security-model).
- **Concurrent commands:** Up to 5 LAN connections are served at once. Commands run one at a time on one worker task, and the commands that use the capture hardware take turns: a second one gets `BP0047` (`pod_busy`) until the first one's reply is complete.
- **DAC runs until stopped:** A `generate` with `duration_ms` 0 runs the DAC until `dac_stop`, the next `generate`, `measure` or `replay`, or `OUTPut OFF` over SCPI.
- **Separate DAC and ADC sample-clock domains:** the DAC8551 engine runs on the 48 MHz `clk48` domain (SPI-limited) while the MCP33131 ADC runs in the 24 MHz domain with its divider floored at 60 (max ~400 kSPS). Each is selected per command via `sample_rate_hz`; the firmware snaps to the nearest achievable divider and logs the actual rate.
