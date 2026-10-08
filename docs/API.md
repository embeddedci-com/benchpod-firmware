# bench-pod-firmware TCP/JSON API

The firmware exposes a plain-text JSON API over a TCP socket. Commands are sent as single-line JSON objects and responses are returned as one or more single-line JSON objects. The same commands run over the embeddedci.com cloud link (the command channel and the byte tunnels, see [While a cloud job holds the pod](#while-a-cloud-job-holds-the-pod)), and the same socket also speaks [SCPI](#scpi-reference). The USB console has its own text commands: see [usb-serial-interface.md](usb-serial-interface.md).

---

## Transport

| Property | Value |
|---|---|
| Protocol | TCP |
| Default port | `8080` |
| Framing | Newline-delimited (`\n`) — one JSON object per line |
| Encoding | UTF-8 |
| Concurrency | Up to 5 LAN connections at once; the commands that use the capture hardware take turns (a second one gets `"busy"`) |

Connect with any TCP client:

```bash
nc <device-ip> 8080
```

The IP address is printed on the USB console (and the USART2 debug pins) at boot, and is also returned by the `status` command and the console `status` command. The pod also advertises itself over mDNS unless the [LAN policy](#lan_policy--what-the-lan-api-may-do) is `off`.

**Latency.** Small replies (a JSON ack) go out as soon as they are ready. While a connection is a CMSIS-DAP probe (`dap_start`) the pod turns Nagle's algorithm off (`TCP_NODELAY`) on it, because flashing is a stream of small request and response packets. No client action is required.

**Line length.** A JSON command line is at most 1279 bytes; a longer one gets `"command too long"` and is dropped. (A SCPI line is at most 255 bytes.) Bulk data goes through the commands that switch a connection to raw bytes (`load_bin`, `dap_start`, `uart_proxy_start`) or through base64url chunks (`load`, `ota_data`).

---

## Security model

**Read this before putting a pod on a network you do not control.**

The pod has two command channels, and they have deliberately different trust models.

### The LAN API is unauthenticated

The JSON-over-TCP and SCPI interfaces on port `8080` have **no authentication, no
authorization, and no transport encryption**. Any client that can open a TCP
connection to the pod has the full command set, which includes:

- driving the DAC and the analog outputs,
- switching target power on and off (`target_power`),
- driving the SWD probe — i.e. halting, reading, and reflashing an attached target,
- uploading firmware to the pod itself over OTA.

There is no login, no API key, and no allow-list. The USB-CDC console is equivalent and
assumes physical access.

**The LAN policy limits this.** `lan_policy` (below) is `open` by default, which is everything
above. `locked` keeps the LAN to reads and instrument control (tiers T0 and T1 of
`src/cmd_tier.c`): persisted settings (Wi-Fi, cloud, DAC limits, calibration, the policies) and
firmware updates then need the cloud or the USB console. `off` stops the TCP listener and the
mDNS advertisement. Only the cloud and the USB console can change it, from the web app's device
settings or `benchpod lan-policy set`.

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

---

## Response Format

### Success

```json
{"status":"ok","data":<value>}\n
```

`data` is command-specific. It may be `null`, a JSON object, or a JSON array.

### Error

```json
{"status":"error","message":"<description>"}\n
```

### Chunked data (multi-packet responses)

Commands that return sample arrays (`capture`, `stream`, `measure`, `test`, `capture_dual`, `capture_read`, `la_capture`, `sensor_regs`, `sensor_la`) send the data in several packets. The first packet uses `"status":"ok"` and the ones after it `"status":"chunk"`. Every packet has a `"more"` boolean that says whether more packets follow. The packet size follows the connection's send buffer (at most 256 samples), so do not count on a fixed size.

```json
{"status":"ok",    "bits":16, "data":[32768,33012,33270,...], "more":true}\n
{"status":"chunk", "data":[33501,33790,34012,...], "more":true}\n
{"status":"chunk", "data":[34190,34322,34410,...], "more":false}\n
```

ADC samples are 16-bit counts (`"bits":16`); see [ADC sample values](#adc-sample-values). A client must read until it receives a packet with `"more":false` to know the transfer is complete. If the client stops reading for a few seconds the pod ends the transfer and closes the connection.

#### Base64 samples (`"enc":"b64"`)

Add `"enc":"b64"` to `capture`, `stream`, `measure`, `test`, `capture_dual` or `capture_read` and the 16-bit samples come as `"b64"` instead of `"data"`: unpadded base64url (RFC 4648 §5) of the little-endian `uint16` values. That is 8/3 bytes per sample instead of up to 6, and it reads back about 6.7x faster (2M ADC samples: 4.2 s instead of 28 s over the LAN). Everything else in the packet stays the same, and `capture_dual`'s LA region still comes as `la_edges`. `status.caps` contains `"capture_b64"` when the firmware supports it. Without the key the reply is the decimal `"data"` array, so older clients keep working.

```json
{"status":"ok",    "bits":16, "adc_rate_hz":400000, "b64":"AAABAP__...", "more":true}\n
{"status":"chunk", "b64":"NBL_f...", "more":false}\n
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
`stream`, `capture_dual`, `capture_read`, `measure`, `test`, `load`, `load_bin`, `replay`,
`sensor_regs`, `sensor_la`, `la_capture`, `dap_start`, `uart_proxy_start` and `speedtest` with
`command not supported over cloud channel`. The USB console does not run JSON commands at all.

| `cmd` | Tier | Purpose | `data` returned | Multi-packet |
|---|---|---|---|---|
| `ping` | T0 | Connectivity check | `"pong"` (string) | no |
| `status` | T0 | Firmware, network, board and capability info | object | no |
| `generate` | T1 | Start a DAC waveform | `null` or object | no |
| `capture` | T0 | ADC snapshot, up to 32768 samples | array of uint16 | yes |
| `stream` | T0 | Same as `capture`, without a trigger | array of uint16 | yes |
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
| `calibrate` | T0/T2 | Run, read or clear this pod's own ADC calibration (the `current_in` input, J8) | object | no |
| `dac_mux` | T1 | Low-level DAC output mux (U55); prefer `dac_out` | object | no |
| `cal_switch` | T1 | Low-level calibration relays (U58); prefer `analog_path` | object | no |
| `dac_control_loop` | T1 | Arm (or disarm) the in-fabric closed-loop DAC | object | no |
| `dac_loop_input` | T1 | Retarget a running control loop's input | object | no |
| `dac_loop_probe` | T0 | One live operating point of the control loop | object | no |
| `fpga_image` | T1 | Switch the running iCE40 image (0 = loop, 1 = deep replay) | object | no |
| `psram_ping` | T0 | iCE40 to PSRAM write-path check | object | no |
| `psram_recover` | T1 | Reboot the pod to recover an inoperable PSRAM datapath | object | no |
| `la` | T1 | LA pin control: step pulse trains + pull-ups (LA1-8) | object | no |
| `la_pins` | T0 | Report all 14 LA pins: function, gpio mode/level, pull, live levels | object | no |
| `gpio` | T1 | Configure / release / drive / read LA pins as GPIO | object | no |
| `la_voltage` | T1 | Select the LA I/O-bank voltage (1.8/3.3 V); required before any LA op | object | no |
| `nrst` | T1 | Drive the dedicated target-reset line (v3 pods) | object | no |
| `usb_cc` | T0 | Read the USB-C CC lines: orientation + source current advertisement (v3 pods) | object | no |
| `target_power` | T1 | Enable/disable a target-power eFuse | object | no |
| `target_status` | T0 | Read target-power eFuse state (enabled/fault/valid) | object | no |
| `power_status` | T0 | One-shot INA238 reading of both supply rails | object | no |
| `power_profile` | T1 | Record a current/voltage profile of one eFuse rail | object | on `stop` |
| `dap_start` | T1 | Turn the connection into a CMSIS-DAP probe | `"dap ready"` (string) | no |
| `uart_proxy_start` | T1 | Enter transparent UART bridge mode | `"uart ready"` (string) | no |
| `spi_start` | T1 | Claim four LA pins as an SPI master | object | no |
| `spi_stop` | T1 | End the SPI session | `"spi stopped"` | no |
| `spi_status` | T0 | The SPI session's pins, rate and mode | object | no |
| `spi_xfer` | T1 | Raw full-duplex SPI transfer | object | no |
| `spi_stream` | T1 | Send a staged PSRAM upload in one CS frame | object | no |
| `spi_flash` | T1 | SPI NOR flash operations (id, read, erase, program) | object | no |
| `sensor_start` | T1 | Arm an emulated I2C sensor (BMP280) | object | no |
| `sensor_set` | T1 | Set the emulated sensor's readings | object | no |
| `sensor_stop` | T1 | Disarm the emulated sensor | `null` | no |
| `sensor_status` | T0 | Sensor + I2C-bus activity counters | object | no |
| `sensor_regs` | T0 | Read the emulated register image | array of uint8 | yes |
| `sensor_la` | T0 | Raw I2C-bus logic capture | array of uint8 | yes |
| `can_config` | T1 | Bring up classic CAN (FDCAN1 / TCAN1044) | object | no |
| `can_write` | T1 | Send one CAN frame | object | no |
| `can_read` | T0 | Drain up to 8 received CAN frames | object | no |
| `can_status` | T0 | CAN state and error counters | object | no |
| `can_term` | T1 | Switch the 120 Ω CAN termination | object | no |
| `can_respond` | T1 | Add or clear an automatic CAN reply rule | object | no |
| `can_disable` | T1 | Turn CAN off | object | no |
| `identity_public` | T0 | Get the device Ed25519 public key | object | no |
| `identity_pop` | T0 | Sign a nonce (proof of possession) | object | no |
| `identity_wipe` | T0 | Always refused here: only on the USB console | error | no |
| `cloud_status` | T0 | The cloud link's state and stored settings | object | no |
| `cloud_set` | T2 | Store the cloud server and device id, then connect | object | no |
| `cloud_clear` | T2 | Forget the cloud settings and disconnect | `"cleared"` | no |
| `cloud_ca` | T0/T2 | Read or clear the company CA for the cloud link | object | no |
| `cloud_proxy` | T0/T2 | Read, set or clear the HTTP proxy for the cloud link | object | no |
| `wifi_status` | T0 | Wi-Fi state and transport counters | object | no |
| `wifi_set` | T2 | Store Wi-Fi credentials and connect | object | no |
| `wifi_clear` | T2 | Forget the Wi-Fi credentials | `"cleared"` | no |
| `eth` | T0/T2 | Wired link control and diagnostics | string or object | no |
| `speedtest` | T1 | Cloud throughput probe over a tunnel | none (raw bytes) | no |
| `sig_policy` | T0/T2 | Read or set which signed updates the pod accepts | object | no |
| `lan_policy` | T0/T2 | Read or set what the LAN API may do | object | no |
| `ota_begin` | T3 | Start a firmware or blob update | object | no |
| `ota_data` | T3 | Send one base64url chunk of the image | object | no |
| `ota_end` | T3 | Verify the staged image's SHA-256 (and signature) | object | no |
| `ota_status` | T0 | The update session's state | object | no |
| `ota_abort` | T3 | Drop the staged image | object | no |
| `ota_selftest` | T3 | Test the flash writer on a scratch sector | object | no |
| `ota_commit` | T3 | Write the verified image and reset (or write a blob) | object | no |
| `blob_status` | T0 | What each W25Q blob slot holds | object | no |

Commands not in this table answer `"unknown cmd"`.

<a id="gates-that-apply-to-every-command"></a>
### Gates that apply to every command

Before a command runs, the firmware checks these, in this order (`src/cmd_gate.c`). The first one
that applies answers with its message and the command does not run.

| Gate | Applies to | Message |
|---|---|---|
| LAN policy `locked` | a T2 or T3 command on a LAN connection | `locked: <cmd> needs the cloud or the USB console` |
| A cloud job holds the pod (lease) | anything but a light read on a LAN connection | `busy: a cloud job holds this pod (<holder>, <N> s left)` |
| Cloud tunnel tier | a command above the tier the server allowed the tunnel's user | `forbidden: <cmd> needs an organization owner or admin` |
| Safe mode | everything except network, status, identity, target power, `dac_limits` and CAN, while the iCE40/PSRAM are off | `safe mode: iCE40/PSRAM are off. Unplug and replug the pod` |
| Digital-only board | the analog commands (`generate`, `capture`, `stream`, `measure`, `load`, `load_bin`, `replay`, `dac_limits`, `dac_set`, `dac_mux`, `cal_switch`, `analog_path`, `dac_out`, `current_out`, `adc_read`, `calibrate`, `dac_control_loop`, `dac_loop_probe`, `dac_loop_input`) and a `capture_dual` with ADC samples | `this BenchPod has no analog front end (digital board): no DAC, ADC or analog outputs. Restart the pod after fitting an analog add-on` |
| DAC output limits | commands that would move the DAC outside the limits | see [`dac_limits`](#dac_limits--dac-output-limits-for-an-external-output-stage) |

Light reads are the T0 commands that do not use the capture hardware: everything at T0 except
`capture`, `capture_dual`, `capture_read`, `stream`, `la_capture`, `sensor_regs`, `sensor_la`,
`can_read`, `psram_ping` and `test`. Commands that use the capture hardware (PSRAM, ADC, LA) also
take turns with each other: a second one gets `"busy"` until the first is done.

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

Start continuous waveform generation on the DAC (16-bit DAC8551, clocked by the iCE40). The waveform is built from 8-bit levels and scaled to the 16-bit DAC.

#### Request

```json
{"cmd":"generate","waveform":"<type>","freq":<Hz>,"amplitude":<0-127>,"offset":<0-255>,"duration_ms":<ms>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `waveform` | string | yes | — | Waveform shape: `"sine"`, `"square"`, or `"sawtooth"` |
| `freq` | number (Hz) | no | `1000` | Output frequency in Hz |
| `amplitude` | integer 0–127 | no | `127` | Half-scale amplitude. Peak-to-peak swing = `2 × amplitude` counts out of 255 |
| `offset` | integer 0–255 | no | `128` | DC offset (vertical centre). `128` = mid-scale |
| `duration_ms` | integer (ms) | no | `0` | How long to generate. `0` = run indefinitely until the next command |
| `sample_rate_mhz` | number (MHz) | no | auto | FPGA DAC sample-clock rate. Omit to auto-pick the highest rate that fits one period in the waveform buffer. A **lower** rate reduces high-frequency clock feedthrough on the analog output (cleaner scope trace) at the cost of fewer samples per period. The DAC engine runs at 48 MHz ÷ divider (divider ≥ 2); the firmware snaps to the nearest achievable divider and logs the actual rate. |
| `on_capture` | boolean | no | `false` | Gateware v27+: wait and start the waveform at the next capture's t0 (the DAC co-trigger). The reply is then `{"cotrig":true}`, or `{"cotrig":false}` when the gateware cannot co-trigger and the waveform started right away. |

**Amplitude and offset arithmetic**

The 8-bit level at each sample point is clamped to `[0, 255]` and then scaled to the 16-bit DAC:

```
sine:     value = offset + amplitude × sin(2π × t/T)
square:   value = offset ± amplitude  (high for first half-period, low for second)
sawtooth: value = offset - amplitude … offset + amplitude  (linear ramp per period)
```

With `amplitude=100, offset=128` the output swings between 28 and 228 (out of 255), centred at mid-scale.

**Frequency resolution**

The firmware precomputes one full period in a sample buffer and the iCE40 loops it. Period length in samples is:

```
period_samples = floor(sample_rate / freq)
```

where `sample_rate = 48 MHz / divider` — on gateware ≥ 13 the DAC8551 engine runs on the 48 MHz `clk48` domain (divider ≥ 2, so up to ~24 MSPS; the DAC8551 SPI caps the achievable rate below that).

The waveform buffer holds 2048 16-bit samples. Lower frequencies need a lower sample rate: leave `sample_rate_mhz` out and the firmware picks one that fits the period.

#### Response

```json
{"status":"ok","data":null}
```

With `on_capture` the reply is `{"status":"ok","data":{"cotrig":true}}`.

#### Examples

```json
{"cmd":"generate","waveform":"sine","freq":100000,"amplitude":100,"offset":128}
```
→ 100 kHz sine, ±100 counts around mid-scale, runs until next command.

```json
{"cmd":"generate","waveform":"square","freq":50000,"amplitude":64,"offset":128,"duration_ms":2000}
```
→ 50 kHz square wave for 2 seconds, then DAC stops.

```json
{"cmd":"generate","waveform":"sawtooth","freq":500000,"amplitude":127,"offset":127}
```
→ 500 kHz sawtooth, full-scale ramp 0 → 254.

```json
{"cmd":"generate","waveform":"sine","freq":100000,"sample_rate_mhz":1}
```
→ 100 kHz sine clocked at 1 MHz (10 samples/period). Coarser waveform but much lower clock feedthrough, useful for a cleaner scope trace on breadboard wiring.

#### Error cases

| Condition | Message |
|---|---|
| Unknown or missing `waveform` | `"unknown waveform"` |
| `freq` ≤ 0 or `amplitude` = 0 | `"generate failed"` |
| DAC limits are set | the [`dac_limits`](#dac_limits--dac-output-limits-for-an-external-output-stage) refusal |

---

### `capture` — ADC snapshot

Capture a fixed number of ADC samples (16-bit MCP33131, through the iCE40 into PSRAM) and return them. The reply arrives once the capture is done; meanwhile other connections keep working, but a second capture waits its turn (`"busy"`). A capture also fills the RAM replay buffer, so `replay` can play it straight back.

#### Request

```json
{"cmd":"capture","samples":<count>,"sample_rate_mhz":<MHz>}
```

| Field | Type | Required | Default | Max |
|---|---|---|---|---|
| `samples` | integer | no | `256` | `32768` |
| `sample_rate_mhz` | number (MHz) | no | max (~0.4) | — |
| `trigger`, `trigger_timeout_ms` | object, integer | no | none | see [Capture triggers](#capture-triggers) |
| `enc` | string | no | decimal | `"b64"`, see [Base64 samples](#base64-samples-encb64) |

For more than 32768 samples use `capture_dual` with `la_samples` 0 (millions of samples, read back from PSRAM).

`sample_rate_mhz` sets the ADC sample clock. Omit it for the maximum ~400 kSPS rate; a **lower** rate stretches the capture window so a slow waveform fits in the 32768-sample buffer. For example, 32768 samples at ~400 kSPS spans ~82 ms, and `0.08` (80 kS/s) spans ~410 ms — enough to see a motor current waveform or other low-frequency signal. The v2 MCP33131 ADC runs in the 24 MHz domain and its divider is floored at 60, so the maximum rate is 24 MHz ÷ 60 ≈ 400 kSPS; the firmware snaps to the nearest achievable divider and logs the actual rate.

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
| `samples` = 0 or > 32768 | `"samples out of range (use capture_dual for deep captures)"` |
| Another capture or upload is running | `"busy"` |
| The capture did not arm | `"capture failed"` |
| The PSRAM read-back failed | `"capture read-back failed"` |

#### Loopback test

To verify the signal path, start a waveform then capture:

```json
{"cmd":"generate","waveform":"sine","freq":1000,"amplitude":100,"offset":128}
{"cmd":"capture","samples":4096}
```

The captured array should show a sinusoidal pattern at the expected period.

---

### `stream` — ADC capture (alias of `capture`)

On this firmware `stream` is the same one-shot PSRAM capture as `capture`, without the trigger option: the capture runs in the gateware and the samples are read back once it is done. It is kept for older clients.

#### Request

```json
{"cmd":"stream","samples":<count>,"sample_rate_mhz":<MHz>}
```

| Field | Type | Required | Default | Max |
|---|---|---|---|---|
| `samples` | integer | no | `256` | `32768` |
| `sample_rate_mhz` | number (MHz) | no | max (~0.4) | — |
| `enc` | string | no | decimal | `"b64"` |

`sample_rate_mhz` has the same meaning as in `capture`.

#### Response

Same chunked format as `capture`:

```json
{"status":"ok",    "bits":16, "data":[...], "more":true}\n
{"status":"chunk", "data":[...], "more":false}\n
```

#### Example

```json
{"cmd":"stream","samples":4096}
```

#### Error cases

| Condition | Message |
|---|---|
| `samples` = 0 or > 32768 | `"samples out of range (use capture_dual for deep captures)"` |
| Another capture or upload is running | `"busy"` |
| The capture did not arm | `"capture failed"` |

---

### `measure` — simultaneous DAC generate + ADC capture (loopback)

Generates a waveform on the DAC and captures ADC samples in one command. The iCE40 starts both together and plays exactly one waveform period across the capture window (the period is `samples` long), so the capture is phase-locked to the stimulus. Designed for loopback / round-trip measurements where the DAC output is wired (directly or through a DUT) to the ADC input.

When the capture finishes the firmware stops the DAC before it returns the data.

#### Request

```json
{"cmd":"measure","waveform":"<type>","freq":<Hz>,"amplitude":<0-127>,"offset":<0-255>,"samples":<count>,"sample_rate_mhz":<MHz>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `waveform` | string | yes | — | `"sine"`, `"square"`, or `"sawtooth"` |
| `freq` | number (Hz) | no | `1000` | DAC output frequency |
| `amplitude` | integer 0–127 | no | `127` | DAC half-scale amplitude |
| `offset` | integer 0–255 | no | `128` | DAC DC offset |
| `samples` | integer | no | `256` | Number of ADC samples to capture, and the waveform period (max `2048`) |
| `sample_rate_mhz` | number (MHz) | no | auto | Sample-clock rate. Omit it and the rate is `freq × samples`, so one period spans the capture. |
| `enc` | string | no | decimal | `"b64"` |

#### Response

Same chunked format as `capture`. The first packet uses `"status":"ok"` and any subsequent packets use `"status":"chunk"`; the last packet has `"more":false`.

```json
{"status":"ok",    "bits":16, "data":[33012,37950,42800,...], "more":true}\n
{"status":"chunk", "data":[50511,52633,53518,...], "more":false}\n
```

#### Examples

```json
{"cmd":"measure","waveform":"sine","freq":100000,"amplitude":100,"offset":128,"samples":512}
```
→ Output a 100 kHz sine and return 512 ADC samples taken during the same window.

```json
{"cmd":"measure","waveform":"square","freq":50000,"samples":1024}
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
| Invalid params (for example `freq` ≤ 0) | `"measure start failed"` |
| Another capture or upload is running | `"busy"` |
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
{"cmd":"capture_dual","adc_samples":<N>,"adc_rate_mhz":<R>,"la_samples":<M>,"la_rate_mhz":<S>}
```

| Field | Type | Required | Default | Notes |
|---|---|---|---|---|
| `adc_samples` | integer | no | `0` | ADC 16-bit samples. |
| `adc_rate_mhz` | number (MHz) | no | max | ADC sample clock (24 MHz domain, divider ≥ 60 → max ~0.4 MHz). |
| `la_samples` | integer | no | `0` | Raw LA words, 2 bytes each (bit `n-1` = LA`n`). |
| `la_rate_mhz` | number (MHz) | no | max | LA sample clock. |
| `stop_dac_after_us` | integer | no | `0` | Gateware v21+: stop a running DAC output this many µs after t0, so the window shows it switch off. `0` leaves it running. |
| `trigger`, `trigger_timeout_ms` | object, integer | no | none | see [Capture triggers](#capture-triggers) |
| `enc` | string | no | decimal | `"b64"` for the ADC region |

`adc_samples` and `la_samples` must not both be `0`. On gateware v22 and newer the firmware packs
the LA region, the ADC region and any resident deep-replay waveform into the 8 MB and refuses up
front when they do not fit or when the combined sample rate would overrun the PSRAM bus (see
[tri-capture-unified-psram.md](tri-capture-unified-psram.md)).

#### Response

Chunked reply: the ADC region as unsigned 16-bit counts in `data`, then the LA region as **edges**. The **first** packet also carries the **achieved** rates so the client can build an aligned time-base (the gateware floors the ADC to a divider ≥ 60, so the actual rate can differ from the request):

```json
{"status":"ok","bits":16,"adc_rate_hz":399361,"la_rate_hz":6000000,"data":[...],"more":true}
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
| `adc_rate_hz` | Achieved ADC sample rate in Hz (`24 MHz ÷ divider`); omitted when `adc_samples` = 0. |
| `la_rate_hz` | Achieved LA sample rate in Hz; omitted when `la_samples` = 0. |

#### Error cases

| Condition | Message |
|---|---|
| `adc_samples` and `la_samples` both 0 | `"adc_samples and la_samples are both 0"` |
| LA + ADC + a resident deep-replay waveform exceed the PSRAM (gateware v22+) | `"capture too deep: LA+ADC+DAC exceed 8 MB PSRAM"` |
| The combined rate overruns the PSRAM bus (gateware v22+) | `"combined sample rate exceeds PSRAM bus bandwidth"` |
| A count exceeds its fixed region (gateware before v22) | `"samples out of range (per-stream PSRAM region limit)"` |
| Another capture or upload is running | `"busy"` |
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
| `enc` | string | no | decimal | `"b64"` |

An `offset` equal to the total sends one empty final packet (`{"status":"ok","data":[],"more":false}`).

| Condition | Message |
|---|---|
| No `capture_dual` / `la_capture` since boot, or a newer capture started | `"no capture to resume (run capture_dual first)"` |
| Something wrote over the capture's PSRAM regions since (an OTA staging, a `load_bin` with `"psram"`, a gateware reload, a SCPI or console capture) | `"capture data was overwritten by <what>; run the capture again"` |
| `offset` past the end | `"offset out of range"` |
| Another capture or upload is running | `"busy"` |

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

If the condition never arrives the capture is aborted and the command errors with a
machine-parseable prefix (the edge word matches the request):

```
trigger timeout: no rising edge on LA9 within 10000 ms
```

A co-triggered DAC start staged for that capture is cancelled with it — the abort is an
untriggered t0 and would otherwise fire the waveform.

#### Error cases

| Condition | Message |
|---|---|
| the running gateware is older than v35 | `"capture triggers need gateware v35 or newer (this pod runs vNN)"` |
| `trigger` is not an object | `"trigger must be an object like {\"la\":1,\"edge\":\"rising\"}"` |
| `trigger.la` outside 1..14 | `"trigger la must be 1..14"` |
| `trigger.edge` not recognised | `"trigger edge must be rising, falling, high or low"` |
| `trigger_timeout_ms` outside 1..600000 | `"trigger_timeout_ms must be 1..600000"` |
| the condition never arrived | `"trigger timeout: no <edge> edge on LA<n> within <N> ms"` |

`status.caps` contains `"capture_trigger"` (and `"gpio_read"`) when the running gateware
supports this; the cloud capabilities frame carries the same two flags as booleans.

---

### Capabilities frame — ADC calibration fields

When the pod connects to the cloud it advertises a `capabilities` frame. Alongside `adc_bits` / `adc_fullscale_mv` / `adc_channels` it ships the affine ADC front-end calibration as **integers** (newlib-nano's `printf` has no `%f`), which the server uses to scale raw capture counts to the true probe voltage instead of the naive count ÷ full-scale:

| Field | Meaning |
|---|---|
| `adc_cal_a_uv` | Affine intercept *a* in microvolts (`V = a + b·count`). |
| `adc_cal_b_nv` | Affine slope *b* in nanovolts per count. |
| `adc_cal_unwrap` | `true`: the front end is bipolar, so a 16-bit count `< 32768` is unwrapped (`count += 65536`) before scaling. |

---

### `la_capture` — deep raw logic-analyzer capture

Capture all 14 LA channels into PSRAM and stream them back. The reply uses the LA edge format of
[`capture_dual`](#capture_dual--simultaneous-adc--la-capture-one-trigger) (`"la":true`,
`la_edges`, `la_upto`); each word holds LA1 in bit 0 up to LA14 in bit 13. Needs `la_voltage`.

```json
{"cmd":"la_capture","samples":65536,"sample_rate_mhz":2}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `samples` | integer | no | `256` | LA samples. The limit is the PSRAM left above a resident deep-replay waveform (up to the whole 8 MB, 2 bytes per sample). |
| `sample_rate_mhz` | number (MHz) | no | max | LA sample clock. |
| `stop_dac_after_us` | integer | no | `0` | Gateware v21+: stop a running DAC output this many µs after t0. |
| `trigger`, `trigger_timeout_ms` | object, integer | no | none | see [Capture triggers](#capture-triggers) |

| Condition | Message |
|---|---|
| LA voltage not set | `"la voltage not set; set it with la_voltage (mv 1800 or 3300) first"` |
| `samples` = 0 or too deep | `"samples out of range"` |
| Another capture or upload is running | `"busy"` |
| The capture did not arm | `"la capture failed"` |

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
| Another connection holds the buffer | `"busy"` |
| `offset` past the 8192-byte staging buffer | `"offset out of range"` |
| Malformed base64url, or chunk overflows the buffer | `"invalid data"` |

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
| `psram` | boolean | no | `false` | Stage into PSRAM for a deep replay (needs the deep-replay gateware image, `status.caps` `"dac_deep_replay"`). |

Replies: `{"status":"ok","data":{"ready":8192}}` right away, then `{"status":"ok","data":{"total":4096}}`
(in samples) once the bytes are in. An upload that gets no bytes for 30 s is dropped.

| Condition | Message |
|---|---|
| Sent on the cloud command channel | `"load_bin needs a stream connection"` |
| `total` missing | `"missing total"` |
| `total` 0 or too large | `"total out of range"` |
| Another capture or upload is running | `"busy"` |

---

### `replay` — play a recorded trace out the DAC

Play the trace back out the DAC. The trace is whichever was most recently put there: the last `capture`, `stream` or `measure` (so you can capture a signal and immediately play it back), a host-uploaded trace from `load` or `load_bin` (so you can replay a previously saved run), or a deep PSRAM upload from `load_bin` with `"psram":true`. The waveform loops continuously until `dac_stop` (or the next `generate`/`measure`/`replay`).

#### Request

```json
{"cmd":"replay","samples":<count>,"sample_rate_mhz":<MHz>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `samples` | integer | no | full recorded length | Number of samples to play, ≤ the recorded length. |
| `sample_rate_mhz` | number (MHz) | no | max | DAC sample clock. Set this to the **same** rate the trace was captured at so the playback time-base matches the recording. |
| `on_capture` | boolean | no | `false` | Gateware v27+: start at the next capture's t0 (the DAC co-trigger). |

#### Response

```json
{"status":"ok","data":{"samples":2048,"cotrig":false}}
```

`cotrig` is `true` when the start waits for the next capture.

#### Error cases

| Condition | Message |
|---|---|
| Nothing has been captured or uploaded yet | `"nothing to replay"` |
| `samples` = 0 or greater than the recorded length | `"samples out of range"` |
| Another connection holds the buffer | `"busy"` |
| FPGA load/start error, or a RAM trace longer than 2048 samples | `"replay failed"` |
| DAC limits are set | the [`dac_limits`](#dac_limits--dac-output-limits-for-an-external-output-stage) refusal |

#### Record → save → replay workflow

```json
{"cmd":"capture","samples":4096,"sample_rate_mhz":0.08}   // record a run; host saves the returned array
{"cmd":"replay","sample_rate_mhz":0.08}                   // play that same run straight back out the DAC
```

To replay a run saved earlier (e.g. a "normal" trace after a "fault" trace overwrote the live buffer), upload it first:

```json
{"cmd":"load","offset":0,"data":"<base64url chunk 0>"}
{"cmd":"load","offset":150,"data":"<base64url chunk 1>"}
...
{"cmd":"replay","sample_rate_mhz":0.08}
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
| `dac_out` | `volts` on the limited path is outside `min_mv`..`max_mv` |
| `dac_control_loop` | `vmin`/`vmax` (16-bit codes) fall outside the limits; `in_trip` on an inverted stage (it trips to `vmin`, the highest output there) |
| `analog_path`, `dac_out`, `adc_read` | inverted stage only: the route disconnects the limited path (`off`, another DAC path, `cal2`, `cal1` unless the path is `5v`), which leaves the module input near 0 V |
| `generate`, `dac_set`, `load`, `load_bin`, `replay`, `measure`, `dac_mux`, `cal_switch` | always: they write raw DAC codes the limits can't be checked against |
| `current_out` | always, when it has `ua`: the 4-20 mA output shares the DAC, so setting a current moves the limited path. Without `ua` it only reads the range and is allowed |

The USB console applies the same rules to `dac`, `path`, `adc`, `current-out`, `dacraw`, `dacmux` and `calsw`;
`dac-limits` shows them and `dac-limits clear` removes them.

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
| `adc_ext` (`ext`, `sma`) | ADC ← front-panel input SMA | ~1 MΩ high-Z, ÷12 |
| `cal1` | Internal 5 V DAC → ADC loopback | 5 V path |
| `cal2` | Internal ±12 V differential DAC → ADC loopback | ±12 V path |
| `current_in` | ADC ← 4-20 mA measurement terminal (249 Ω to ground) | J8 |
| `current_out` | DAC → 4-20 mA output terminal (J9) only: the DAC voltage outputs are switched off, the ADC relays stay as they are | unchanged |

The DAC output paths also open all ADC relays (the ADC returns to the external
SMA), so driving `dac_12v` and reading the ADC is a self-contained loopback of
the front end.

#### `analog_path` — apply a path (routing only)

```json
{"cmd":"analog_path","path":"cal1"}
→ {"status":"ok","data":{"path":"cal1","u55":3,"u58":9}}
```

#### `dac_out` — route an output path **and** set a calibrated voltage

`volts` is optional (omit to only route). The firmware converts volts→DAC code
using the baked-in per-path calibration and returns the achieved millivolts and
the code used (`code` is `-1` when only routing).

```json
{"cmd":"dac_out","path":"5v","volts":2.5}
→ {"status":"ok","data":{"path":"dac_5v","mv":2502,"code":128}}
```

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
| rev3 | 4056 | 20094 | one rev3 pod: 4.056 mA at 8-bit code 0, 20.032 mA at 8-bit code 255 |

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
  `ua = min_ua + code16 × (max_ua − min_ua) / 65535`. `generate` takes 8-bit levels; a level is
  the high byte of the 16-bit code.
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
- J9 cannot be wired to the pod's own measurement terminal J8, because J8 is 249 Ω to pod ground.
- A target whose loop input is referenced to a ground it shares with the pod (through SWD or
  UART wiring) cannot be driven either. Use an isolated loop input on the target.
- If the loop does touch pod ground, the current bypasses the sense resistor, is no longer
  regulated, and sits near the transmitter's 32 mA limit regardless of the DAC.

To measure the loop current with the pod, put a resistor between J9 pin 2 and supply minus and
connect the ADC input SMA center to supply minus, with the SMA shell left open. `adc_read ext`
then reads a negative voltage, `-(R + 25 Ω) × I`. `hwe2e/benchpod_current_loop_hw_test.go` in
embeddedci-server does this.

The pod cannot see the loop: with no supply or an open loop, `current_out` still replies ok.

`status.caps` contains `"current_out"` when the firmware supports it. The cloud capabilities
frame has `"current_out":true` and the range as `current_out_min_ua` / `current_out_max_ua`.
On the USB console, `current-out 12` holds 12 mA (the console takes mA) and `current-out`
shows the range.

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

```json
{"cmd":"adc_read","source":"ext"}
→ {"status":"ok","data":{"source":"adc_ext","mv":11980,"count":63037,"span":4}}

{"cmd":"adc_read","source":"cal1"}   // DAC still playing a waveform into the loopback
→ {"status":"error","message":"adc_read: input not settled on cal1 (51811 counts pk-pk
   over the 16-sample burst, limit 1024). Stop the DAC (dac_stop) or let the node settle"}
```

`current_in` is the 4-20 mA measurement terminal (J8). Its reply has two more fields:

- `ua`: the loop current in µA. The pod reads the voltage across a 249 Ω resistor, so
  `ua = mv × 1000 / 249`. 4 mA is 996 mV and 20 mA is 4980 mV.
- `offset_mv`: this pod's calibration offset, which is already taken out of `mv` and `ua`
  (see [`calibrate`](#calibrate--calibrate-the-current_in-input-j8)). It is 0 on a pod that was
  never calibrated.

`count` is always raw.

```json
{"cmd":"adc_read","source":"current_in"}
→ {"status":"ok","data":{"source":"current_in","mv":3985,"count":61562,"span":6,"offset_mv":10,"ua":16004}}
```

Firmware up to 3.3.0 used a different name for this source. That name is no longer accepted.

#### `calibrate` — calibrate the `current_in` input (J8)

`current_in` is the 4-20 mA terminal J8: pin 1 goes through a 249 Ω resistor to pod ground and
pin 2 is ground. The pod reads the voltage across that resistor, so 4 mA is 996 mV and
20 mA is 4980 mV.

Every ADC source is scaled with a fit, `volts = a + b × count`. The fits are built into
the firmware and are the same on every pod. Each pod still has its own offset of a few mV
on `current_in`. `calibrate` measures that offset and stores it on the pod. From then on the pod
uses its own fit for `current_in`: the built-in one with the offset taken out of `a`.

Disconnect J8 first. With nothing connected the terminal is at 0 V, so the pod needs no
reference.

| Request | What it does |
|---|---|
| `{"cmd":"calibrate","source":"current_in"}` | Calibrate `current_in` with J8 disconnected, store the result, return it |
| `{"cmd":"calibrate"}` | Return the stored calibration |
| `{"cmd":"calibrate","clear":true}` | Remove it and go back to the built-in fit |

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
| `count`, `span` | Raw mean count and peak-to-peak spread of the measurement (calibrate only) |
| `samples` | Samples averaged (calibrate only) |

The measurement averages 32 bursts of 16 samples, the same burst `adc_read` takes. It
takes about 0.3 s and leaves the `current_in` path routed.

A reading outside ±50 mV is refused, because it means something is driving J8. The old
calibration stays in place.

```json
{"cmd":"calibrate","source":"current_in"}   // a 4 mA loop is still connected
→ {"status":"error","message":"calibrate: current_in reads 996 mV, it must be within +/-50 mV of zero.
   Something is driving J8. Disconnect it and try again"}
```

The calibration is stored in flash. It survives a reboot and a firmware update. Only the
offset of `current_in` is calibrated. The gain, and `ext`, `cal1` and `cal2`, use the built-in
fits: calibrating those needs a reference voltage. `status.caps` contains `"calibrate"`
when the firmware supports it, and the cloud capabilities frame has `"calibrate":true`.

On the USB console, `calibrate` shows the calibration, `calibrate current_in` runs it, and
`calibrate clear` removes it. `adc current_in` uses it.

#### Loopback / self-cal examples

```json
// Self-cal the 5 V path (no external wiring): drive 2.5 V, read it back.
{"cmd":"dac_out","path":"5v","volts":2.5}
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
| `enc` | string | no | decimal | `"b64"` |

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
| Another capture or upload is running | `"busy"` |

---

### `status` — firmware and connection info

Returns the firmware version, network and cloud state, board and gateware details, health
counters and the capability list. No parameters.

#### Request

```json
{"cmd":"status"}
```

#### Response (abridged)

```json
{"status":"ok","data":{"device":"benchpod","version":"3.7.0","board":"stm32h563","net":"ready",
 "ip":"192.168.1.220","rssi_dbm":null,"wifi":"connected","cloud":"connected",
 "adc_bits":16,"adc_fullscale_mv":4096,"adc_channels":1,"la_vccio_mv":3300,
 "board_rev":"v3","board_rev_mv":1650,"nrst_pin":true,"flash_kb":2048,
 "ota_sig":true,"sig_policy":"audit","sig_keys":3,"lan_policy":"open",
 "lease":{"held":false,"holder":"","left_s":0},"gateware":47,"gateware_embedded":47,
 "psram":"ok","psram_ok":true,"reset":"power-on","last_crash":"","analog":true,
 "safe_mode":false,"safe_reason":"","caps":["signal","gpio","power","swd","..."]}}
```

#### Response fields

| Field | Type | Description |
|---|---|---|
| `device` | string | Always `"benchpod"`. |
| `version` | string | Firmware version. |
| `board` | string | Board name this firmware was built for. |
| `net` | string | `"ready"` when the Ethernet or Wi-Fi interface has an address, else `"disconnected"`. |
| `ip` | string | The pod's IP (Ethernet preferred over Wi-Fi), or `"0.0.0.0"`. |
| `rssi_dbm` | `null` | Always `null` on this firmware; the Wi-Fi signal is in the console's `wifi-show`. |
| `wifi` | string | Wi-Fi (ESP32-C3) state: `disabled`, `starting`, `waiting-slave`, `flashing-c3`, `connecting`, `connected` or `backoff`. |
| `cloud` | string | Cloud link state: `disabled`, `waiting-wifi`, `connecting`, `connected` or `backoff`. |
| `rx_dropped`, `tx_dropped` | integer | Console bytes dropped because a ring was full. |
| `adc_bits`, `adc_fullscale_mv`, `adc_channels` | integer | ADC format. |
| `la_vccio_mv` | integer | LA bank voltage, `0` while unset. |
| `board_rev` | string | PCB revision, detected at boot from the `PA3` strap: `"v2"`, `"v3"`, or `"unknown"`. Gates the LA-bank 1.8 V setting, the dedicated NRST pin, and USB-C CC monitoring. |
| `board_rev_mv` | integer | Raw revision-strap voltage in millivolts (`-1` if not measured). Diagnostic only. |
| `nrst_pin` | boolean | `true` when the pod has the dedicated target-reset pin (v3+). When `false`, `nrst` and CMSIS-DAP `SWJ_PINS` reset requests are no-ops. |
| `flash_kb` | integer | The MCU's internal flash: `2048` or `1024`. Updaters check it before sending an image. |
| `ota_sig`, `sig_policy_cmd`, `lan_policy_cmd`, `tunnel_max_tier`, `lease_state`, `cloud_ca`, `cloud_proxy` | boolean | Always `true`: this firmware has signed updates, the policy commands, per-tunnel tiers, the lease state, and the company CA and proxy commands. Clients use them to tell older firmware apart. |
| `sig_policy`, `sig_keys` | string, integer | The [signature policy](#sig_policy--which-firmware-and-blob-updates-the-pod-accepts) and how many release keys the firmware trusts. |
| `lan_policy` | string | The [LAN policy](#lan_policy--what-the-lan-api-may-do). |
| `lease` | object | Whether a cloud job holds the pod: `held`, `holder`, `left_s`. |
| `lwip_mem_max`, `lwip_mem_size`, `pbuf_pool_max`, `pbuf_pool_size`, `malloc_failures` | integer | Network-stack and heap health counters. |
| `gateware`, `gateware_embedded` | integer | The gateware running in the iCE40 and the one this firmware embeds (`0` = unknown). They differ after a firmware update until the boot-time gateware update has run. |
| `loop_tripped` | boolean | The control loop's over-range trip is latched. |
| `uart_rx_overflow` | boolean | The UART proxy's receive FIFO overflowed. |
| `psram`, `psram_ok` | string, boolean | The boot PSRAM self-test: `"ok"`, `"inoperable (...)"` or `"not run"`. When `psram_ok` is `false`, run [`psram_recover`](#psram_recover--reboot-to-recover-the-psram). |
| `heap_free`, `heap_min`, `stack_min`, `stacks` | integer, object | Heap and per-task stack headroom in bytes. |
| `reset`, `last_crash` | string | Why the pod last reset, and the last crash report (`""` when none). |
| `analog` | boolean | `false` on the digital-only board (no DAC or ADC). |
| `safe_mode`, `safe_reason` | boolean, string | Safe mode after repeated crashes at boot, and what it turned off. See [Gates](#gates-that-apply-to-every-command). |
| `caps` | array of string | The capability list, below. |

#### `caps` values

| Value | Meaning |
|---|---|
| `signal`, `gpio`, `power`, `swd`, `i2c_sensor`, `uart`, `la`, `analyzer`, `command`, `tunnel`, `ota`, `la_pins`, `power_profile`, `capture_b64`, `can` | Always present on this firmware: signal generation, GPIO, target power, the CMSIS-DAP probe (`dap_start`), emulated I2C sensors, the UART bridge, the logic analyzer, the command channel and tunnels, OTA, per-pin functions (`la_pins`, `gpio`), [`power_profile`](#power_profile--record-a-rail-current-profile), [base64 samples](#base64-samples-encb64) and CAN. |
| `pod_current` | The pod's own current monitor (INA at 0x41) is fitted. |
| `analog`, `scope`, `dac_limits`, `calibrate`, `current_out` | The analog front end is fitted. |
| `dac`, `dac_dc`, `dac_replay` | The board's DAC features (analog board only). |
| `dac_deep_replay`, `dac_control_loop`, `dac_cotrig`, `dac_loop_sources`, `dac_loop_input_map` | Features of the **running** gateware image (analog board only): deep PSRAM replay, the closed-loop DAC, the co-trigger, loop input sources and the loop input map. |
| `gpio_read`, `capture_trigger` | The running gateware reads pin levels back and supports [capture triggers](#capture-triggers) (v35+). |
| `spi_master`, `spi_stream` | The running gateware has the SPI master (see [SPI master](#spi-master-flash-an-spi-device)). |
| `nrst_pin`, `usb_cc` | v3 board features. |

The gateware-dependent values can appear and disappear across an [`fpga_image`](#fpga_image--switch-the-running-gateware-image) swap.

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
  DUT — on a v3 pod it comes up at 3.3 V.

---

### `la` — logic-analyzer pin control

A single command for the LA bank. The action is inferred from the fields present:
**step** (when `steps` is given), **pull-up set** (when `pullup` is given), **pull-up
query** (a bare `la`), or **pull-up bitmask** (no `la`). It replaces the former
`gpio_set` / `gpio_step` / `pullup` / `pullup_status` commands. There is no active-drive
action — a pull-up, or its absence (which leaves the board's pull-down), sets a line's
idle level instead.

#### Step a pulse train

Starts a **non-blocking** train of `steps` pulses on an LA channel: each pulse is high for `delay_us` microseconds then low for `delay_us` (one full step period is `2 × delay_us`). The FPGA runs the train autonomously; the command returns immediately. Poll completion via SCPI `DIGital:STEP:BUSY?`.

```json
{"cmd":"la","la":<1-14>,"steps":<n>,"delay_us":<us>,"dir_la":<1-14>,"direction":<0|1>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `la` | integer | yes | — | LA channel to pulse (1..14). |
| `steps` | integer | yes | — | Number of pulses. Range 1..65535. **Its presence selects the step action.** |
| `delay_us` | integer | yes | — | Microseconds per half-phase. Range 4..65535. |
| `dir_la` | integer | no | — | Optional direction channel driven (high/low) **before** stepping. |
| `direction` | integer | no | 0 | Level for `dir_la`: `1` = high, `0` = low. |

Response:

```json
{"status":"ok","data":{"la":1,"steps":200,"delay_us":500,"status":"started"}}
```

#### Switch (or query) an LA pin pull-up

Connects/disconnects a pull-up resistor on an LA pin. Each pin's pull-up is gated by an
SN74LVC2G66 analog switch driven from a PCA9555 IO-expander line: **on** drives the ctrl
line high (switch closed, resistor in circuit); **off** sets it high-Z (a PCB pull-down
holds the switch open). Only **LA1–8** carry a bias network; the resistor and its
direction are fixed per pin:

| LA pin | Network | | LA pin | Network |
|---|---|---|---|---|
| 1 | 4.7k pull-**up** | | 5 | 10k pull-**up** |
| 2 | 4.7k pull-**up** | | 6 | 10k pull-**up** |
| 3 | 2.2k pull-**up** | | 7 | 10k pull-**down** |
| 4 | 2.2k pull-**up** | | 8 | 10k pull-**down** |

**LA7 and LA8 pull DOWN, not up.** They are driven by the same `pullup` field and the
same analog switch as LA1-LA6 — only the resistor's other end differs — so a client
that assumes every biased channel pulls up will drive an open-drain bus the wrong way.
The `pull` field in the response says which way each one goes. LA9-LA14 have no
network at all.

All of them default to **off (high-Z)** at boot.

> **Only usable at a 3.3 V LA bank.** The resistors are referenced to **+3V3**, not to
> the LA bank rail, so the firmware refuses to engage one while the bank is at 1.8 V
> (see [`la_voltage`](#la_voltage--select-the-la-io-bank-voltage)). Switching the bank
> down to 1.8 V also releases any pull-up that is currently on. Turning one *off* is
> always allowed.

```json
{"cmd": "la", "la": 3, "pullup": "on"}
```

- `la` (required for a per-pin op): LA pin `1`–`8`.
- `pullup` (optional): `"on"`/`"off"` (or `1`/`0`). **Omit `pullup`** — a bare
  `{"cmd":"la","la":N}` — to **query** the current state without changing it.

Response:

```json
{"status": "ok", "data": {"la": 3, "pullup": 1, "ohms": "2.2k", "pull": "up", "pullups_available": 1}}
```

`pullup` is `1` when the resistor is connected, `0` when high-Z. `pull` is `"up"` for
LA1-LA6 and `"down"` for LA7/LA8, and does not change with state. `pullups_available`
is `1` only while the LA bank is at 3.3 V — when it is `0`, every enable request on
this pod is refused until the bank is switched back.

#### Read all pull-ups as a bitmask

Send `la` with no channel:

```json
{"cmd": "la"}
```

Response:

```json
{"status": "ok", "data": {"la_pullup_mask": 5, "pullups_available": 1}}
```

`la_pullup_mask` bit `(la-1)` is set when LA`<la>`'s pull-up is on (here `0b00000101` =
LA1 and LA3 enabled). `pullups_available` reports whether a pull-up can be engaged at
the current bank voltage (`1` = bank at 3.3 V).

#### Error cases

| Condition | Message |
|---|---|
| step request without `la` / `delay_us` | `"missing la"` / `"missing delay_us"` |
| `steps` / `delay_us` not a plain non-negative integer | `"invalid steps"` / `"invalid delay_us"` |
| `steps` outside 1..65535 (the gateware step counter is 16-bit) | `"steps must be 1..65535 (the step counter is 16-bit): split a longer move into several trains"` |
| `delay_us` outside 4..65535 | `"delay_us must be 4..65535 (microseconds per half-phase: a step takes 2 x delay_us)"` |
| a step train is already running | `"busy"` |
| step `la` invalid or out of range | `"invalid args"` |
| `dir_la` invalid, or equal to `la` | `"invalid dir_la"` / `"dir_la must be a different pin than la"` |
| the step or direction pin belongs to another function | `"pin conflict: LA<n> is in use by <function>; <how to release>"` |
| pull-up `la` outside 1–8 (incl. LA9–12) | `"no pull-up on this la"` |
| pull-up enable with the LA bank at 1.8 V | `"pull-ups are 3V3-referenced; not available with the LA bank at 1.8 V"` |
| pull-**down** enable on a pin whose function needs the line high | `"pull conflict: LA7 is used by …"` (see [pin ownership](#la-pin-ownership-functions-and-conflicts)) |
| I2C write to the PCA9555 failed | `"pca9555 write failed"` |

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
| `uart_rx`, `uart_tx` | [`uart_proxy_start`](#uart_proxy_start--transparent-uart-bridge) | the proxy ending (`+++`, send failure, connection close, tunnel reset) |
| `swd_clk`, `swd_dio` | `dap_start` (see [dap-over-tunnel.md](dap-over-tunnel.md)) | SWD disarm, connection close, tunnel reset |
| `i2c_sda`, `i2c_scl` | [`sensor_start`](#sensor_start--arm-an-emulated-sensor) | `sensor_stop`, or a replacing `sensor_start` |
| `step`, `step_dir` | the [`la` step train](#step-a-pulse-train) on a free pin | the train finishing (the firmware polls the gateware) |
| `spi_sck`, `spi_mosi`, `spi_miso`, `spi_cs` | [`spi_start`](#spi_start--claim-four-pins-for-spi) | `spi_stop`, gateware reconfiguration |

**Captures never own pins.** `la_capture`, `capture_dual`, `sensor_la` and capture triggers
observe all 14 channels whatever their function.

Claims **persist across client disconnects** for `gpio` and the emulated sensor — they are pod
state, like the LA voltage — and are cleared by a pod reboot. A gateware reconfiguration (an
image swap or `flash-ice40`) resets every engine in the fabric: the firmware re-applies the
`gpio` pins' output state and releases the UART / SWD / I2C / step functions so the table and
the wire agree again.

#### Conflict errors

Claiming a pin another function owns fails with a machine-parseable prefix:

```
pin conflict: LA<n> is in use by <function>; <how to release>
```

| Owner | `<how to release>` |
|---|---|
| `gpio` | `release it with {"cmd":"gpio","la":<n>,"mode":"off"}` |
| `uart_rx` / `uart_tx` | `stop the uart proxy first` |
| `swd_clk` / `swd_dio` | `end the SWD session first` |
| `i2c_sda` / `i2c_scl` | `stop the sensor emulation first ({"cmd":"sensor_stop"})` |
| `step` / `step_dir` | `wait for the step train to finish` |
| `spi_*` | `stop the SPI session first ({"cmd":"spi_stop"})` |

When a request names several pins, the **first** conflict is reported and **nothing changes**.

Setting a level on a pin that is not a gpio output:

```
LA<n> is not a gpio output (function <function>[, gpio <mode>]); configure it with {"cmd":"gpio","la":<n>,"mode":"output"}
```

#### Pull compatibility

The fixed bias resistors (LA1/LA2 4.7k up, LA3/LA4 2.2k up, LA5/LA6 10k up, **LA7/LA8 10k
down**, LA9–LA14 none) are checked in **both** directions — when claiming a function on a pin
whose pull is engaged, and when engaging a pull on a pin that already has a function.

| Function | Pull-up engaged | Pull-down engaged (LA7/LA8) |
|---|---|---|
| `none`, gpio `input`, gpio `output`, `swd_clk`, `step`, `step_dir`, `spi_*` | ok | ok |
| gpio `open_drain` | ok | **conflict** — a released line would read low |
| `uart_rx`, `uart_tx` | ok | **conflict** — the line idles high |
| `i2c_sda`, `i2c_scl` | ok | **conflict** — an open-drain bus needs pull-ups |
| `swd_dio` | ok | **conflict** — SWDIO is pulled up when released |

```
pull conflict: LA7 has its 10k pull-down engaged, which uart_rx can't work with (the line idles high); disable it with {"cmd":"la","la":7,"pullup":"off"} or use another channel
pull conflict: LA7 is used by i2c_sda, which can't work with the 10k pull-down (an open-drain bus needs pull-ups)
```

The first form comes from a claim (`uart_proxy_start`, `dap_start`, `sensor_start`, `gpio`),
the second from `{"cmd":"la","la":7,"pullup":"on"}` on a pin that is already in use.

---

### `la_pins` — report every LA pin

Reports the function, gpio mode/level and pull state of all 14 channels, plus the live pin
levels when the gateware can read them back.

```json
{"cmd":"la_pins"}
```

Response:

```json
{"status":"ok","data":{"pins":[
  {"la":1,"function":"none","gpio":null,"level":null,"pull":{"dir":"up","ohms":"4.7k","on":false}},
  {"la":2,"function":"gpio","gpio":"output","level":1,"pull":{"dir":"up","ohms":"4.7k","on":false}},
  {"la":9,"function":"uart_rx","gpio":null,"level":null,"pull":null}
],"levels":1234}}
```

| Field | Meaning |
|---|---|
| `function` | one of `none`, `gpio`, `uart_rx`, `uart_tx`, `swd_clk`, `swd_dio`, `i2c_sda`, `i2c_scl`, `step`, `step_dir`, `spi_sck`, `spi_mosi`, `spi_miso`, `spi_cs` |
| `gpio` | `null`, or the gpio mode (`input` / `output` / `open_drain`) |
| `level` | the **commanded** level of a gpio `output` / `open_drain` pin, else `null` |
| `pull` | `null` on LA9–LA14; otherwise the fixed resistor and whether it is engaged |
| `levels` | live pin levels, bit `la-1`, read back from the gateware; `null` when the running image is older than **v35** |

The reply always lists all 14 entries and is about 1.3 kB. It fits the cloud command channel, but
it is the largest single-reply command the pod has.

`la_pins` does **not** require the LA voltage: it reports state and reads nothing that would
drive a DUT line.

---

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
synchroniser. This needs gateware **v35** or newer; older images answer:

```
reading pin levels needs gateware v35 or newer (this pod runs v34)
```

#### Response (1 and 2)

The affected pins, in the same entry shape `la_pins` uses:

```json
{"status":"ok","data":{"pins":[{"la":5,"function":"gpio","gpio":"output","level":1,"pull":{"dir":"up","ohms":"10k","on":false}}]}}
```

#### Error cases

| Condition | Message |
|---|---|
| the LA voltage has not been set | `"la voltage not set; set it with la_voltage (mv 1800 or 3300) first"` |
| neither `mode` nor `level` given, with a `la` | `"gpio needs a mode or a level"` |
| `mode` not recognised | `"mode must be input, output, open_drain or off"` |
| `level` not 0/1 | `"level must be 0 or 1"` |
| `la` not a number, list or `"all"` | `"la must be a pin number, a list of pin numbers, or \"all\""` |
| `la` out of range | `"la must be 1..14 (or \"all\" with \"mode\":\"off\")"` |
| `"la":"all"` with anything but `"mode":"off"` | `"\"la\":\"all\" is only allowed with \"mode\":\"off\""` |
| `level` with `"mode":"input"` | `"level applies to output and open_drain pins, not input"` |
| the pin belongs to another function | `"pin conflict: …"` (see [above](#la-pin-ownership)) |
| a pull-down would fight the mode | `"pull conflict: …"` (see [above](#la-pin-ownership)) |
| level set on a pin that is not a gpio output | `"LA<n> is not a gpio output (…)"` |
| `{"cmd":"gpio"}` read on gateware < v35 | `"reading pin levels needs gateware v35 or newer (this pod runs vNN)"` |

#### SCPI

`DIGital:OUTPut <la>,<0|1>` goes through the same table: the pin becomes a **gpio output** at
that level, shows up in `la_pins`, and is released with `{"cmd":"gpio","la":<n>,"mode":"off"}`.
It requires the LA voltage, and raises a settings-conflict error if another function owns the
pin. `DIGital:STEP` follows the step-train rules above.

---

### `la_voltage` — select the LA I/O-bank voltage

The LA bank I/O voltage (iCE40 `VCCIO_0` — the supply for **all** of LA1..LA14) is
switched between **3.3 V** and **1.8 V** by an on-board TPS2116 power-mux (U8),
driven by `PG3` (select) with status on `PG4`. At boot the selection is **unset**
and the firmware **refuses every LA-bank operation** (`la`, `la_capture`,
`dap_start`, `uart_proxy_start`, `sensor_start`) until the host picks a voltage —
so the DUT's logic level is always an explicit choice.

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
set. `status` also carries the current selection as `la_vccio_mv` (0 = unset).

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

> **v2.0.0 hardware limitation — now enforced in firmware.** The LA1–8 pull-ups (and
> the board pull-downs that hold their switches open) are referenced to **3V3**, not to
> the bank rail, so at 1.8 V a closed pull-up drives the DUT line ~1.5 V above its own
> rail. The firmware therefore **refuses to enable a pull-up while the bank is at
> 1.8 V**, and `la_voltage` with `mv: 1800` **releases any pull-up that is on** before
> switching the rail. `uart_proxy_start`'s automatic RX hold is skipped for the same
> reason at 1.8 V (a floating DUT TX may decode as `0x00` until the DUT drives it).
> Switch the bank to 3.3 V if you need a pull-up. (Hardware fix pending in v2.0.1.)

#### Error cases

| Condition | Message |
|---|---|
| `mv` not 1800 or 3300 | `"la voltage must be 1800 or 3300 (mv)"` |
| `mv: 1800` on a v2 pod | `"1.8 V needs a v3 pod; this board is v2 (its TPS2116 has no 1.8 V setting)"` |
| any LA op before a voltage is set | `"la voltage not set; set it with la_voltage (mv 1800 or 3300) first"` |
| a **change** while any LA pin has a function | `"la voltage can't change while pins are in use: LA3 (uart_rx), LA4 (uart_tx); stop them first"` |

> **Not while pins are in use.** Switching the bank glitches every LA line and strands the
> 3V3-referenced pulls, so a change is refused while any pin is owned by a function other than
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
{"cmd": "nrst", "assert": 1}      // hold the target in reset
{"cmd": "nrst", "assert": 0}      // release
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
{"cmd":"target_power","efuse":<1|2>,"state":<0|1>,"delay_ms":<ms>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `efuse` | integer | yes | — | Which eFuse to switch: `1` (internal 5 V) or `2` (external). |
| `state` | integer | yes | — | `1` (non-zero) enables the eFuse; `0` disables it. |
| `delay_ms` | integer | no | `0` | Apply the change after this many milliseconds. `0` = immediately. The command **acks right away** and the eFuse switches later from the pod's poll loop — so the connection is free to do something else meanwhile (e.g. enter `uart_proxy_start` to capture the target's boot output as it powers up). A new `target_power` for the same eFuse replaces a still-pending delayed change. |

#### Response

```json
{"status":"ok","data":{"efuse":1,"enabled":1,"delay_ms":0}}
```

| Field | Type | Description |
|---|---|---|
| `efuse` | integer | The eFuse that was switched (echoes the request). |
| `enabled` | integer | The **requested** state: `1` for on, `0` for off. With `delay_ms` > 0 this is the state the eFuse will reach once the delay elapses. |
| `delay_ms` | integer | The delay applied (echoes the request; `0` when immediate). |

#### Error cases

| Condition | Message |
|---|---|
| `efuse` field missing | `"missing efuse"` |
| `state` field missing | `"missing state"` |
| `efuse` not 1 or 2 | `"invalid efuse"` |

> **Scheduled power-on + UART capture.** `uart_proxy_start` turns the connection
> into a raw byte stream, so that connection cannot send a power-on *while*
> capturing. Schedule it instead (or send it from a second connection):
> `target_power` with `delay_ms` → `uart_proxy_start` → the eFuse switches mid-
> capture and you catch the boot banner.

> The USB console has `power <1|2> <on|off>` (no delay) and `pstat`; see
> [usb-serial-interface.md](usb-serial-interface.md). Read the eFuses'
> fault/valid status with `target_status`.

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
{"status":"ok","data":{"efuse1":{"enabled":1,"fault":0,"valid":1},"efuse2":{"enabled":0,"fault":0,"valid":0},"status_supported":true}}
```

| Field | Type | Description |
|---|---|---|
| `efuseN.enabled` | integer | `1` if the eFuse EN output is currently driven on. |
| `efuseN.fault` | integer | `1` if the eFuse signalled an over-current/fault (`FLT`, active-low on the wire). |
| `efuseN.valid` | integer | `1` if the eFuse output voltage is valid (`V`, power-good). |
| `status_supported` | boolean | `true` — `V`/`FLT` are wired through the power expander. |

`FLT` is also sampled every 10 ms in the background, so a trip is reported asynchronously
(and over the cloud as an `efuse.event` frame) without polling this command. The eFuses
auto-retry after ~110 ms, and `FLT` stays asserted through the retry, so the sampling
interval never misses a trip.

---

### `power_status` — one-shot rail measurement

Reads both on-board INA238 current monitors (`0x40` = internal 5 V rail, `0x44` = external
rail) and reports each rail's bus voltage and current. This is the cheap "what is the DUT
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
                       "external":{"ok":false,"bus_mv":0,"current_ua":0}}}
```

| Field | Type | Description |
|---|---|---|
| `<rail>.ok` | boolean | `false` when that INA238 did not answer (an unpopulated / unpowered rail). |
| `<rail>.bus_mv` | integer | Bus (DUT-side) voltage in mV. |
| `<rail>.current_ua` | integer | Current in µA, computed from the measured shunt voltage and the known shunt resistance — signed, so a back-feeding DUT reads negative. |

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
249 Ω 4–20 mA terminal, *not* the DUT supply — do not use it for this.)

#### Start

```json
{"cmd":"power_profile","action":"start","efuse":1,"rate_hz":1000,"max_duration_ms":60000,"keep_samples":2048}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `efuse` | `1` or `2` | no | `1` | Which rail to sample. |
| `rate_hz` | integer | no | `500` | 100..500, whole numbers only. Accurate to ~200 Hz; above that the sampler (one register read per worker pass) beats against the pass interval and flattens near 365 Hz. `stats.rate_hz` reports what was delivered, `stats.adc_rate_hz` what the sensor was configured for. |
| `max_duration_ms` | integer | no | `60000` | 1..600000. The sampler stops by itself at the limit and marks the result `truncated`. |
| `keep_samples` | integer | no | `0` | 0..4096 samples to keep for the reply. `0` = statistics only. |

```json
{"status":"ok","data":{"started":true,"efuse":1,"rate_hz":907}}
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
 "stats":{"efuse":1,"rate_hz":364,"adc_rate_hz":907,"n":3640,"duration_ms":10000,
          "avg_ua":141200,"min_ua":830,"peak_ua":612400,
          "avg_mv":4995,"min_mv":4870,"max_mv":5012,
          "energy_uj":7052000,"charge_uc":1412000,
          "fault":false,"truncated":false},"more":false}
```

| Field | Meaning |
|---|---|
| `t_us` | Start of each bin, µs since the profile started. |
| `current_ua` / `bus_mv` | Bin-averaged current (µA, signed) and bus voltage (mV). |
| `stats.rate_hz` | Samples per second actually **delivered** (`n` over `duration_ms`, measured). |
| `stats.adc_rate_hz` | Conversion rate the INA238 was configured for — the ceiling `rate_hz` works towards, not what arrived. |
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
{"cmd":"power_profile","efuse":1,"duration_ms":2000,"rate_hz":500,"keep_samples":1024}
```

#### Error cases

| Condition | Message |
|---|---|
| a profile is already running | `"power profile already running on efuse 1"` |
| `efuse` not 1 or 2 | `"efuse must be 1 or 2"` |
| `rate_hz` out of range | `"rate_hz must be 100..500"` |
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

`status.caps` contains `"power_profile"`.

---

### `uart_proxy_start` — transparent UART bridge

Bridges a DUT UART through two LA channels, turning the connection into a serial-over-TCP pipe (the network equivalent of the console's `uart-proxy`). The iCE40 gateware runs a soft **8N1 UART** on the chosen channels; the RP2350 forwards bytes both ways over SPI.

Like `dap_start`, this is **connection-scoped** and changes how the connection's bytes are interpreted:

1. Send `uart_proxy_start` as a normal JSON command. The pod arms the UART and replies `{"status":"ok","data":"uart ready"}` **while still in JSON mode**.
2. **Immediately after the ack the same TCP connection becomes a transparent byte stream:** everything you send is transmitted to the DUT, and everything the DUT sends is delivered back to you (asynchronously — no polling).

#### Request

```json
{"cmd":"uart_proxy_start","rx":<1-14>,"tx":<1-14>,"baud":115200}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `rx` | integer | yes | — | LA channel the FPGA samples (wire the DUT's TX here). |
| `tx` | integer | yes | — | LA channel the FPGA drives (wire the DUT's RX here); distinct from `rx`. |
| `baud` | integer | no | 115200 | Line rate; the firmware computes the FPGA bit-period divisor. 8N1, ≤115200 solid, higher best-effort. |

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
| an LA channel is outside 1..14 or the two are equal | `"invalid uart args"` |
| a UART proxy is already active (console or another connection) | `"uart busy"` |
| `rx` or `tx` belongs to another function | `"pin conflict: …"` (see [pin ownership](#la-pin-ownership)) |
| `rx`/`tx` on LA7 or LA8 with its 10k pull-**down** engaged | `"pull conflict: …"` — the UART idles high |
| the iCE40 does not answer after arming | `"uart proxy failed: FPGA not responding (check gateware)"` |

`rx` and `tx` are **claimed** for the session (functions `uart_rx` / `uart_tx`) and released
when the proxy ends, so nothing else can be armed onto them meanwhile.

> **RX pull hold.** The proxy holds the RX line high for the session so a DUT's floating TX
> pin does not decode as a flood of `0x00` before its firmware brings the UART up. This only
> happens on **LA1–LA6**, whose resistors pull **up**: LA7/LA8 pull *down* (engaging theirs
> would guarantee the very flood it prevents) and LA9–LA14 have no resistor. It is also
> skipped at a 1.8 V bank, where the 3V3-referenced pulls are unavailable. The previous state
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
{"cmd":"spi_start","sck":3,"mosi":4,"miso":5,"cs":6,"hz":1000000,"mode":0}
```

| Field | Meaning |
|---|---|
| `sck`, `mosi`, `miso`, `cs` | LA pins 1..14, all different |
| `hz` | SCK rate, default 1000000.  The pod picks the nearest rate at or below it from 24 MHz / (2 * n), n = 2..63: 6 MHz down to 190 kHz |
| `mode` | 0 (default) or 3 |

Reply: `{"sck":3,"mosi":4,"miso":5,"cs":6,"hz":1000000,"mode":0}` with the rate actually used.
CS is driven high (released) until a transfer.  Needs the LA voltage set first.

Errors: `SPI master needs gateware v45+`, `spi busy: send spi_stop first`,
`swd or spi busy: end the SWD session first`, and the usual `pin conflict:` / `pull conflict:`.

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

## Emulated I2C sensor

The pod can **pretend to be an I2C sensor** on two LA channels: the iCE40 FPGA
acts as an I2C **slave (target)** that the DUT's I2C **master** reads, while the
STM32 serves the register image. This lets a host-in-the-loop test present a
device (currently a **BMP280**) to a DUT and control what it reports — so the DUT
can be tested both with the sensor "present" and "absent" without touching real
hardware. The feature is advertised by the `"i2c_sensor"` capability in
`status`.

Only one emulated sensor is active at a time. It stays armed (the FPGA keeps
responding on the bus) until `sensor_stop`, a device reset, or a new
`sensor_start`. These commands are **TCP/JSON only** — they are not on the serial
console.

> **BMP280 basics.** 7-bit address `0x76` (default) or `0x77`; chip-id register
> `0xD0` reads `0x58`. A DUT driver writes `ctrl_meas` (`0xF4`) to start a
> conversion, polls the `status` register (`0xF3`) bit 3 while "measuring", then
> reads the 20-bit temperature/pressure ADC words from `0xF7`–`0xFC` and
> decompresses them with the calibration block at `0x88`–`0x9E`. The emulator
> fills all of these and mimics the ~5.5 ms conversion handshake.

### `sensor_start` — arm an emulated sensor

#### Request

```json
{"cmd":"sensor_start","type":"bmp280","addr":"0x76","sda":<1-14>,"scl":<1-14>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `type` | string | yes | — | Sensor model. Currently only `"bmp280"`. |
| `sda` | integer | yes | — | LA channel (1..14) for the I2C **SDA** line. |
| `scl` | integer | yes | — | LA channel (1..14) for the I2C **SCL** line. |
| `addr` | string | no | model default (`0x76`) | 7-bit I2C address as a hex/decimal string (e.g. `"0x77"`). |

#### Response

```json
{"status":"ok","data":{"type":"bmp280","addr":118,"sda":1,"scl":2}}
```

`addr` is the numeric 7-bit address (e.g. `118` = `0x76`).

#### Error cases

| Condition | Message |
|---|---|
| `type` field missing | `"missing type"` |
| `sda` or `scl` missing | `"missing sda/scl"` |
| `type` not a known model | `"unknown sensor type"` |
| bad channel / FPGA config failed | `"sensor start failed (bad channel?)"` |
| `sda` or `scl` belongs to another function | `"pin conflict: …"` (see [pin ownership](#la-pin-ownership)) |
| `sda`/`scl` on LA7 or LA8 with its 10k pull-**down** engaged | `"pull conflict: …"` — an open-drain bus needs pull-ups |

`sda` and `scl` are **claimed** (functions `i2c_sda` / `i2c_scl`) until `sensor_stop`. The
claim survives a client disconnect — the emulation is pod state. A `sensor_start` while a
sensor is running may reuse the running sensor's own pins; if the new claim is refused, the
**old sensor keeps running and nothing changes**.

### `sensor_set` — set the emulated readings

Updates the values the DUT will read back and rebuilds the register image. At
least one parameter is required; a sensor must be active.

#### Request

```json
{"cmd":"sensor_set","temperature_c":25.0,"pressure_pa":101325}
```

| Field | Type | Required | Description |
|---|---|---|---|
| `temperature_c` | number | no* | Temperature in °C. |
| `pressure_pa` | number | no* | Pressure in Pa (BMP280 range ~1000–200000). |

\* at least one of the two must be present.

#### Response

```json
{"status":"ok","data":{"type":"bmp280"}}
```

#### Error cases

| Condition | Message |
|---|---|
| no sensor armed | `"no sensor active"` |
| `temperature_c` not finite | `"temperature_c rejected"` |
| `pressure_pa` not finite / out of range | `"pressure_pa rejected"` |
| neither parameter present | `"no recognised parameters"` |

### `sensor_stop` — disarm the sensor

```json
{"cmd":"sensor_stop"}
```

Releases the SDA/SCL channels and stops responding on the bus. Response:
`{"status":"ok","data":null}`. (Safe to call when nothing is armed.)

### `sensor_status` — query sensor + bus activity

```json
{"cmd":"sensor_status"}
```

#### Response (inactive)

```json
{"status":"ok","data":{"active":false}}
```

#### Response (active)

```json
{"status":"ok","data":{"active":true,"type":"bmp280","addr":118,"transactions":5,"writes":3,"last_reg":244,"last_val":1}}
```

| Field | Type | Description |
|---|---|---|
| `active` | boolean | Whether a sensor is armed. |
| `type` | string | Model name (active only). |
| `addr` | integer | 7-bit address (active only). |
| `transactions` | integer | I2C transactions the FPGA has handled. |
| `writes` | integer | Register-write bytes the DUT has issued. |
| `last_reg` | integer | Address of the last register the DUT wrote (0–255). |
| `last_val` | integer | Value of that last write (0–255). |

`transactions` going up proves the DUT actually probed the emulated sensor —
useful to distinguish "DUT never tried" from "DUT tried and got the wrong data".

### `sensor_regs` — read the register image

Returns the emulated device's registers (firmware-loaded values plus anything the
DUT has written). Chunked array, same framing as `capture`.

#### Request

```json
{"cmd":"sensor_regs","start":"0xF7","len":6}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `start` | string | no | `0` | First register address (hex/decimal string), 0–255. |
| `len` | integer | no | `256` | Number of bytes; `start + len` must be ≤ 256. |

#### Response

```json
{"status":"ok","data":[247,0,0,201,63,160],"more":false}
```

#### Error cases

| Condition | Message |
|---|---|
| no sensor armed | `"no sensor active"` |
| `len` = 0 or `start + len > 256` | `"start/len out of range"` |
| another command holds the buffer | `"busy"` |
| FPGA read failed | `"read regs failed"` |

### `sensor_la` — raw I2C-bus logic capture

Captures the raw SDA/SCL line states on the emulated sensor's channels. Each
returned byte packs **four** `{SCL,SDA}` samples (oldest in the high nibble); an
idle bus reads `0xFF`. Chunked array, same framing as `capture`.

#### Request

```json
{"cmd":"sensor_la","samples":1024,"sample_rate_mhz":2.0}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `samples` | integer | no | `256` | Number of packed bytes (1..4096), **not** individual samples. |
| `sample_rate_mhz` | number | no | max | Capture clock; same semantics as `capture`. |

#### Error cases

| Condition | Message |
|---|---|
| `samples` = 0 or > 4096 | `"samples out of range"` |
| another command holds the buffer | `"busy"` |
| FPGA capture failed | `"la capture failed"` |

---

## Device Identity (Ed25519)

Each unit owns a stable **Ed25519 keypair** that serves as its device identity. The private key is generated on the device the first time it boots (seeded from the STM32 hardware TRNG) and persisted to a dedicated flash sector. It survives firmware updates and `wifi_clear`/`cloud_clear`. The device only ever **signs**; the private key never leaves the unit.

The firmware never replaces the key on its own. The one way to erase it and make a new one is the USB console's `identity-wipe <short id>` (physical presence; `benchpod identity wipe --connection usb`), after which the pod must be registered again. Over the LAN and the cloud, `{"cmd":"identity_wipe"}` is always refused with `identity_wipe: only on the pod's USB console (physical presence): benchpod identity wipe --connection usb`.

Two operations are exposed: fetch the **public key** (the identifier), and produce a **proof of possession** — a signature over a caller-supplied nonce that proves the unit holds the private key matching that public key.

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
| `SENSe:SRATe <MHz>` / `?` | `capture` `sample_rate_mhz` | ADC capture clock; `0` = max ~400 kSPS (24 MHz ÷ divider, divider ≥ 60). Lower stretches the window (e.g. `0.08` → 80 kS/s → 32768 samples ≈ 410 ms). |
| `SENSe:SWEep:POINts <n>` / `?` | `capture` `samples` | Default acquisition length for `READ?`. |
| `READ? [<n>]` | `capture` | Capture `n` (or the `POINts` default) samples at `SENSe:SRATe` and return CSV. Also loads the replay buffer (its first 2048 samples, the DAC BRAM size). |
| `TRACe[:DATA] <offset>,"<base64url>"` | `load` | Upload one chunk of a replay waveform: 16-bit little-endian samples (2 bytes each, as the JSON `load`), at most 4096 bytes. `offset` is in bytes. Send `offset` 0 first, then successive offsets (≤150 bytes/chunk to fit the 256-byte line). |
| `TRACe:POINts?` | — | Number of 16-bit samples currently loaded for replay. |
| `SOURce:FUNCtion USER` + `OUTPut ON` | `replay` | Play the captured/uploaded buffer out the DAC (loops). Playback rate = `SOURce:SRATe`. |
| `OUTPut OFF` | `dac_stop` | Stop the DAC (a looped replay or any waveform). |

`SOURce:FUNCtion` now accepts `USER` alongside `SINusoid`/`SQUare`/`RAMP`. `OUTPut ON` with `USER` selected and an empty buffer pushes `-200 "Execution error"`.

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
(`status.caps` `"dac_control_loop"`).

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

Errors: `"control loop needs gateware v23+"`, `"loop input sources need gateware v29+"`,
`"loop input map needs gateware v30+"`, `"unknown loop input source (use adc, fixed or sweep)"`,
`"curve base64 decode failed"`, `"curve needs at least one 16-bit point"`, the parameter checks
(inverted window, zero-step sweep), `"sweep cannot be combined with an input map on this
gateware: ..."`, `"control loop not in the running gateware image (switch with
{"cmd":"fpga_image","image":0})"` and `"control loop arm failed"`.

### `dac_loop_input` — retarget a running loop

Changes the loop's input source or value without re-arming or re-uploading the curve (gateware v29+).
Fields left out keep their value.

```json
{"cmd":"dac_loop_input","input":32768}
{"cmd":"dac_loop_input","source":"sweep","step":16}
```

Reply: `{"source":"fixed","input":32768,"step":0,"v":<DAC code at the moment of the write>}`. Poll
`dac_loop_probe` for the settled value. Errors: `"loop input sources need gateware v29+"`,
`"unknown loop input source (use adc, fixed or sweep)"`, the parameter checks, the image refusal
and `"loop input update failed"`.

### `dac_loop_probe` — one live operating point

```json
{"cmd":"dac_loop_probe"}
```

Reply: `{"i":<raw ADC count>,"in":<what the loop indexed with>,"source":"adc","v":<DAC code>,"tripped":false}`.
Plot against `in`, not `i`: in a fixed or sweep run the ADC is not in the path. `tripped` is the
latched over-range trip (the output then sits at `vmin` until the loop is disarmed). On the
deep-replay image it answers `"closed-loop DAC not in the running gateware image (switch with
{"cmd":"fpga_image","image":0})"`.

<a id="fpga_image--switch-the-running-gateware-image"></a>
### `fpga_image` — switch the running gateware image

The iCE40 has two images: `0` (the closed-loop DAC) and `1` (deep DAC replay from PSRAM). The
switch reflashes the iCE40's configuration and reconfigures it, which takes 2 to 3 seconds. The pod
then announces its new capabilities to the cloud, and puts an external output stage back on its
park level when DAC limits are set.

```json
{"cmd":"fpga_image","image":1}
```

Reply: `{"image":1,"version":47,"features":<feature bits>}`. Errors: `"image required (0=loop,
1=deep-replay)"`, `"image out of range (0=loop, 1=deep-replay)"`, `"image switch: reflash failed
(CDONE never rose)"`, `"image switch: iCE40->PSRAM write inoperable after retries (run
psram_recover to reboot+reflash)"`, `"image switch failed"` and `"busy"`.

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
about 0.4 s later; the boot self-test reflashes the iCE40 and brings the datapath back without a
power cycle. Over the cloud the reply may not arrive before the reboot: treat "no reply, the pod
reconnects" as success and check `status.psram_ok` again.

---

## CAN

Classic CAN on FDCAN1 with the TCAN1044 transceiver, on J1 pins 19 (`CAN+`) and 20 (`CAN−`).
Every CAN command is a single reply, so all of them work over the cloud command channel, and they
keep working in safe mode. Frame IDs and data bytes accept decimal or `0x` hex.

### `can_config` — bring CAN up

```json
{"cmd":"can_config","bitrate":500000,"mode":"normal","term":true}
```

| Field | Type | Default | Description |
|---|---|---|---|
| `bitrate` | integer | `500000` | Bit rate in bit/s; one that has no exact bit timing is refused |
| `mode` | string | `"normal"` | `normal`, `listen` (receive only), `internal` (internal loopback) or `external` (loopback through the transceiver); the loopback modes let a single pod test itself |
| `term` | boolean | `false` | Switch the 120 Ω termination on |

Reply: `{"bitrate":500000,"mode":"normal","term":true}`. Errors: `"mode must be
normal|internal|external|listen"`, `"unsupported bitrate (no exact bit timing)"`, `"can init failed"`.

### `can_write` — send one frame

```json
{"cmd":"can_write","id":291,"ext":false,"rtr":false,"data":[1,2,3]}
```

`id` is required; `ext` selects a 29-bit ID, `rtr` a remote frame, and `data` holds up to 8 bytes.
Reply: `{"id":291,"ext":false,"dlc":3}`. Errors: `"missing id"`, `"can not enabled"`, `"bus off"`,
`"tx failed (fifo full?)"`.

### `can_read` — drain received frames

```json
{"cmd":"can_read","max":8}
```

`max` is 1..8 (default 8). Reply: `{"frames":[{"id":291,"ext":false,"rtr":false,"dlc":3,"data":[1,2,3],"ts":12345}],"overflow":0}`.
`overflow` counts frames dropped because the receive queue was full.

### `can_status` — state and counters

Reply: `{"enabled":true,"mode":"normal","bitrate":500000,"term":true,"tec":0,"rec":0,"bus_off":false,"error_passive":false,"rx_pending":0,"rx_overflow":0,"responder_rules":0,"responder_hits":0,"bus_off_recoveries":0}`.

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

`reply_rtr` sends a remote frame. Reply: `{"rule":0,"rules":1}`, or `{"rules":0}` after a clear.
Errors: `"missing match_id"`, `"missing reply_id"`, `"responder table full"`.

### `can_disable` — turn CAN off

Reply `{"enabled":false}`.

---

## Network and cloud

The cloud and Wi-Fi settings are T2: on a locked LAN they need the cloud or the USB console. They
work in safe mode, so a pod can always be put back on the network.

### `cloud_status`

Reply: `{"state":"connected","last_error":"","configured":true,"host":"www.embeddedci.com","port":443,"tls":true,"verify":true,"device_id":"<uuid>"}`.
`state` is `disabled`, `waiting-wifi`, `connecting`, `connected` or `backoff`.

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

Reply: `{"state":"connected","configured":true,"connected":true,"ssid":"lab","xacts":..,"pump_calls":..,"batch":[..],"settle_hit":..,"settle_miss":..,"xact_err":..,"c3_lost_noresp":0,"c3_lost_reboot":0}`.
`state` is the same as `status.wifi`; the other counters describe the link to the ESP32-C3 and how
often it was lost.

### `wifi_set` / `wifi_clear`

```json
{"cmd":"wifi_set","ssid":"lab","password":"secret"}
{"cmd":"wifi_clear"}
```

Wired Ethernet stays the primary link; Wi-Fi is the fallback. An empty or missing `password` is an
open network. `wifi_set` replies `{"ssid":"lab"}` and reconnects after the reply; `wifi_clear`
replies `"cleared"`. Errors: `"missing ssid"`, `"ssid too long"`, `"password too long"`, `"config
save failed"`.

### `eth` — wired link control and diagnostics

| Request | Tier | Reply |
|---|---|---|
| `{"cmd":"eth","action":"stats"}` | T0 | `{"phy_ok":true,"link":true,"aneg_done":true,"phy_mode":"100FD","mac_mode":"100FD","rx_good":..,"rx_crc":..,"tx_good":..,...}` |
| `{"cmd":"eth","action":"refclk"}` | T0 | `{"hz":50000012,"ppm":0,"on_hsi":false}`: the PHY's 50 MHz RMII clock against the MCU crystal; drops the link for about 200 ms |
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
`gw1` (the two iCE40 images), `esp` (the ESP32-C3 Wi-Fi image) and `ca` (a company CA, see
[`cloud_ca`](#cloud_ca--cloud_proxy-company-ca-and-http-proxy-for-the-cloud-link)). Over the
cloud the server sends the image as WS `ota.*` frames; over the LAN the image goes in base64url
`ota_data` chunks; the USB console has `upload-*` (see
[usb-serial-interface.md](usb-serial-interface.md)). Most clients use `benchpod firmware install`
or the web app instead of these commands.

One transport holds an update session at a time. Every `ota_*` reply carries the session:

```json
{"status":"ok","data":{"state":"receiving","target":"firmware","received":65536,"size":563696,
 "frames_seen":0,"sig":"ok","sig_key":"<key id>","owner":"lan:1","error":""}}
```

`state` is `idle`, `receiving`, `verified`, `error` or `installed`. `sig` is the signature check of
the last begin: `none`, `ok`, `format`, `unknown-key`, `signature`, `target` or `image`. `owner` is
`cloud`, `usb`, `lan:N` or `""`.

| Command | Fields | What it does |
|---|---|---|
| `ota_begin` | `size`, `sha256` (64 hex), optional `target` (`gw0`, `gw1`, `esp`, `ca`; none = firmware), `version`, `sig` (base64url signed manifest) | Starts a session. The signature is checked here against the [signature policy](#sig_policy--which-firmware-and-blob-updates-the-pod-accepts) (the `ca` target is not signed). |
| `ota_data` | `offset`, `data` (base64url) | Writes one chunk at a byte offset. |
| `ota_end` | none | Checks the SHA-256 (and, for the firmware, that the image fits this MCU's flash and can enforce the policy); `state` becomes `verified` or `error`. |
| `ota_status` | none | The session, from any transport (T0). |
| `ota_abort` | none | Drops the staged image. |
| `ota_selftest` | none | Tests the flash writer on a scratch sector (never the firmware); replies `{"selftest":"pass"}`. |
| `ota_commit` | none | Firmware: replies `{"committing":true}`, writes the image and resets. A blob: writes the slot and replies with the session. |

Refusals:

| Condition | Message |
|---|---|
| `size` or `sha256` missing | `"missing size/sha256"` |
| Unknown `target` | `"unknown target"` |
| `ca` from the LAN | `"cloud_ca: change it from the cloud or the USB console"` |
| Another transport holds the session | `"busy: <target> update from <holder> is in progress"` (see [Error Reference](#error-reference)) |
| A capture or upload is running | `"busy: a capture or upload is running"` |
| Safe mode turned the PSRAM off | `"safe mode: iCE40/PSRAM are off. Unplug and replug the pod"` (the gate) |
| Size, hash or chunk problems | `"bad size"`, `"bad sha256"`, `"chunk out of range"`, `"incomplete image"`, `"sha256 mismatch"`, `"invalid data"`, `"missing data"` |
| The policy refuses the signature | `"signature: <result>"` |
| The image could not enforce `required` | `"image cannot enforce signatures (policy required)"` |
| `ota_commit` before a successful `ota_end` | `"no verified image staged"` |
| `ota_selftest` while the hardware is busy | `"busy"`, or `"ota selftest failed (see console log)"` |

### `blob_status` — what the W25Q slots hold

Reads the cached slot headers (no bus access).

```json
{"status":"ok","data":{"blobs":[
  {"name":"gw0","state":"ok","present":true,"size":135100,"version":47,"sha256":"<hex>"},
  {"name":"gw1","state":"ok","present":true,"size":135100,"version":47,"sha256":"<hex>"},
  {"name":"esp","state":"outdated","present":true,"size":1048576,"version":3,"sha256":"<hex>"}]}}
```

It lists the three release slots (`gw0`, `gw1`, `esp`); the company CA is read with `cloud_ca`.

`state` compares the slot with what this firmware was built with: `ok` (exactly that blob),
`outdated` or `missing` (an installer should send it), or `unknown` (this build expects nothing in
particular).

---

<a id="scpi-reference"></a>
## SCPI reference

Port `8080` also speaks SCPI (IEEE 488.2, on libscpi): a connection whose first byte is not `{`
is a SCPI connection for its whole life. Lines end with `\n`, are at most 255 bytes, and several
commands can share a line with `;`. Queries answer one line; bulk samples come as comma-separated
ASCII. Errors go to the error queue (16 deep): read them with `SYSTem:ERRor?`. The instrument
state (`SOURce`, `SENSe`, the trace) is shared by every SCPI connection. Upper-case letters are
the short form; `[...]` is optional; `#` is a number in the header (`OUTPut:POWer1`).

### IEEE 488.2 common commands

| Command | Description |
|---|---|
| `*IDN?` | `EmbeddedCI,BenchPod,0,0.2.0` (the last field is the SCPI layer's version, not the firmware's; read that with JSON `status`) |
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
| `[SOURce]:VOLTage[:AMPLitude] <0-255>` / `?` | 8-bit half-scale amplitude | As `generate` `amplitude` |
| `[SOURce]:VOLTage:OFFSet <0-255>` / `?` | 8-bit offset | As `generate` `offset` |
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
| `DIAGnostic:CAPture? [<adc_n>][,<la_n>]` | each 0..128, not both 0; default 16,16 | One-trigger ADC + LA capture (ADC 100 kS/s, LA 1 MS/s): `adc_n` ADC counts, then `la_n` LA words, as CSV. The LA words are masked to 12 bits (LA1..LA12). |
| `DIAGnostic:PATTern? <pattern>[,<value>[,<n>]]` | `SINusoid`, `COUNter`, `RAMP` or `CONStant`; value 0..255 (for `CONStant`, default 255); n 1..4096 (default 256) | An 8-bit synthetic pattern as CSV (no hardware) |
| `TRACe[:DATA] <offset>,"<base64url>"` | byte offset 0..4096 | Upload one chunk of a replay trace (16-bit little-endian samples, at most 4096 bytes in total) |
| `TRACe:POINts?` | | Samples in the trace (bytes ÷ 2) |

### Digital (LA pins)

| Command | Parameter | Description |
|---|---|---|
| `DIGital:OUTPut <la>,<0\|1>` | LA 1..14 | Make the pin a gpio output at that level (the JSON `gpio` table applies) |
| `DIGital:STEP <la>,<steps>,<delay_us>[,<dir_la>[,<dir>]]` | as the JSON [`la` step train](#step-a-pulse-train) | Start a step pulse train (non-blocking) |
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

### Amplitude / offset guide

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

All errors follow the format:

```json
{"status":"error","message":"<description>"}\n
```

| Message | Cause |
|---|---|
| `"missing cmd"` | Request JSON has no `"cmd"` field |
| `"unknown cmd"` | `cmd` value not recognised |
| `"command too long"` | A JSON line longer than 1279 bytes |
| `"busy"` | A command that uses the capture hardware while another one runs |
| `"command not supported over cloud channel"` | A streaming command on the single-reply cloud command channel (see [Commands](#commands)) |
| `"locked: <cmd> needs the cloud or the USB console"`, `"busy: a cloud job holds this pod (...)"`, `"forbidden: <cmd> needs an organization owner or admin"`, `"safe mode: ..."`, `"this BenchPod has no analog front end (digital board): ..."` | The [gates](#gates-that-apply-to-every-command) |
| `"la voltage not set; set it with la_voltage (mv 1800 or 3300) first"` | An LA command before `la_voltage` |
| `"unknown waveform"` | `generate` `waveform` value not `sine`, `square`, or `sawtooth` |
| `"generate failed"` | Invalid `freq` (≤ 0) or `amplitude` (= 0) |
| `"samples out of range"` | `samples` outside the command's range (see each command) |
| `"samples out of range (use capture_dual for deep captures)"` | `capture` / `stream` with more than 32768 samples |
| `"capture failed"` | The capture did not arm or did not complete |
| `"missing waveform"` | `measure` request without `waveform` field |
| `"unknown waveform (use sine, square or sawtooth)"` | `measure` with another waveform |
| `"measure start failed"` | `measure` invoked with invalid params |
| `"missing la"` | `la` step request without `la` field |
| `"missing state"` | `target_power` request without `state` field |
| `"missing efuse"` | `target_power` request without `efuse` field |
| `"invalid efuse"` | `target_power` `efuse` is not 1 or 2 |
| `"invalid swd la channel"` | `dap_start` channel outside 1..14 or both channels equal |
| `"missing rx"` / `"missing tx"` | `uart_proxy_start` request without `rx`/`tx` field |
| `"invalid uart args"` | `uart_proxy_start` channel outside 1..14 or `rx`==`tx` |
| `"uart busy"` | a UART proxy is already active (console or another connection) |
| `"missing type"` | `sensor_start` request without `type` field |
| `"missing sda/scl"` | `sensor_start` request without `sda`/`scl` field |
| `"unknown sensor type"` | `sensor_start` `type` is not a known model |
| `"sensor start failed (bad channel?)"` | `sensor_start` bad LA channel or FPGA config error |
| `"no sensor active"` | `sensor_set`/`sensor_regs` with no sensor armed |
| `"temperature_c rejected"` / `"pressure_pa rejected"` | `sensor_set` value not finite / out of range |
| `"no recognised parameters"` | `sensor_set` with neither `temperature_c` nor `pressure_pa` |
| `"start/len out of range"` | `sensor_regs` `len` = 0 or `start`+`len` > 256 |
| `"missing nonce"` | `identity_pop` request without `nonce` field |
| `"invalid nonce"` | `identity_pop` nonce is not valid base64url, or decodes to 0 or > 128 bytes |
| `"identity not available"` | Device identity not initialised (should not occur after boot) |
| `"capture data was overwritten by <what>; run the capture again"` | `capture_read` after an OTA staging, a `load_bin` with `"psram"`, a gateware reload or another (SCPI/console) capture wrote over the capture's PSRAM regions |
| `"busy: <target> update from <holder> is in progress"` | an `ota_*` command (or `upload-*` on the USB console) while another transport (`the cloud`, `the USB console`, `LAN connection N`) holds the update session. Within the LAN, a second connection is refused while the one holding the session is open; once it closes, the next LAN connection takes the session over and continues (one connection per command works). A session with no progress for 30 s may be replaced or aborted from any transport. `ota_status` and the other ota replies carry `"owner"`: `"cloud"`, `"usb"`, `"lan:N"` (the current LAN connection) or `""` |
| `"safe mode: the PSRAM is off this boot, so an update cannot be staged; ..."` | an update begin (cloud `ota.begin`, console `upload-begin`) in a safe mode that turned the iCE40/PSRAM off |
| `"busy: a capture, upload or update is using the PSRAM bus; try again when it ends"` | `cloud_ca` clear and `cloud_proxy` set/clear (and the console `ca-clear`, `proxy-set`, `proxy-clear`, `flash-esp32` and PSRAM diagnostics) while the shared PSRAM/W25Q bus is in use |
| `"cloud_ca: change it from the cloud or the USB console"`, `"cloud_proxy: ..."`, `"sig_policy: ..."`, `"lan_policy: ..."` | Changing the company CA, the proxy or a policy from a LAN connection (see [Pod policies](#pod-policies)) |
| `"identity_wipe: only on the pod's USB console (physical presence): benchpod identity wipe --connection usb"` | `identity_wipe` from anywhere but the USB console |

Three families carry a **machine-parseable prefix** — clients (the Python SDK among them) match
on the prefix and show the rest to the user:

| Prefix | Raised by | Details |
|---|---|---|
| `pin conflict: ` | any command claiming an LA pin (`gpio`, `uart_proxy_start`, `dap_start`, `sensor_start`, the `la` step train, SCPI `DIGital:*`) | [pin ownership](#la-pin-ownership) |
| `pull conflict: ` | the same claims, and `{"cmd":"la",…,"pullup":"on"}` | [pull compatibility](#la-pin-ownership) |
| `trigger timeout: ` | `capture`, `capture_dual`, `la_capture` with a `trigger` | [capture triggers](#capture-triggers) |

---

## Session Example

```
$ nc 192.168.1.220 8080

{"cmd":"ping"}
{"status":"ok","data":"pong"}

{"cmd":"la_voltage","mv":3300}
{"status":"ok","data":{"mv":3300,"st":1,"readback_mv":3297}}

{"cmd":"target_power","efuse":1,"state":1}
{"status":"ok","data":{"efuse":1,"enabled":1,"delay_ms":0}}

{"cmd":"generate","waveform":"sine","freq":1000,"amplitude":100,"offset":128}
{"status":"ok","data":null}

{"cmd":"capture","samples":512}
{"status":"ok","bits":16,"data":[32768,33170,33571,...],"more":true}
{"status":"chunk","data":[...],"more":false}

{"cmd":"dac_stop"}
{"status":"ok","data":null}

{"cmd":"la","la":1,"pullup":"on"}
{"status":"ok","data":{"la":1,"pullup":1,"ohms":"4.7k","pull":"up","pullups_available":1}}

{"cmd":"la","la":1,"steps":200,"delay_us":500}
{"status":"ok","data":{"la":1,"steps":200,"delay_us":500,"status":"started"}}

{"cmd":"identity_public"}
{"status":"ok","data":{"public":"ue5UfSsOAhQYZ332ELokjOI8-uyjcbrd1ic3Nmypvzk"}}

{"cmd":"cloud_set","host":"www.embeddedci.com","device_id":"<uuid>","tls":true}
{"status":"error","message":"locked: cloud_set needs the cloud or the USB console"}
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
{"status":"ok","data":{"policy":"audit","enforces":true,"keys":3}}
```

`audit` (default) accepts every update and reports its signature check; `permissive` refuses a
bad signature; `required` refuses anything not signed by a key the firmware trusts. Over the
cloud the policy can only get stricter (`sig_policy: only the USB console can loosen the policy
(now required)`); the LAN cannot change it; the USB console (`sig-policy <value>`) can set any
value.

### `lan_policy` — what the LAN API may do

```json
{"cmd":"lan_policy"}
{"cmd":"lan_policy","set":"locked"}
```

```json
{"status":"ok","data":{"policy":"locked"}}
```

`open` (default), `locked` (a T2/T3 command on the LAN gets `locked: <verb> needs the cloud or
the USB console`) or `off` (no TCP listener on 8080, no mDNS). Set from the cloud or the USB
console (`lan-policy <value>`), never from the LAN.

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
`ota.begin`, the USB console `upload-begin ca`, or `benchpod cloud ca set corp.pem` over USB); it
must parse as X.509 and is trusted in addition to the built-in roots.
`cloud_proxy` replies `{"host":"...","port":3128,"auth":true}` or `{}`; the password is never shown.
The pod sends `CONNECT <server>:443` with the server's host name and, when set, Basic proxy auth.
Setting or clearing either reconnects the cloud. Reading is T0; setting and clearing are T2.

**Never from the LAN.** Installing or clearing the CA and setting or clearing the proxy are refused
on a LAN TCP connection whatever the LAN policy (even `open`), with
`cloud_ca: change it from the cloud or the USB console` or
`cloud_proxy: change it from the cloud or the USB console`. Reading both (`{"cmd":"cloud_ca"}`,
`{"cmd":"cloud_proxy"}`) still works on the LAN. The reason: a LAN attacker who could install their
own CA and point the pod at their own proxy could terminate the pod's TLS with a certificate for the
real server name and relay the host-bound (v2) login, which defeats it.

### While a cloud job holds the pod

When a cloud consumer (a CI run, the web app, the SDK) holds the pod's lease, LAN connections get
only light reads (`status`, `ping`, the `*_status` reads, policy reads); everything else gets
`busy: a cloud job holds this pod (<holder>, <N> s left)`, and SCPI setters get -200. The pod ends
the lease itself at the announced expiry or when the cloud link drops. `status` shows
`"lease":{"held":...,"holder":"...","left_s":...}`.

---

## Implementation Notes

- **JSON parser limitations:** The firmware uses a minimal flat-JSON parser. Nested objects are not supported, except the `trigger` object of the captures; arrays only where a command documents one (`data` of `can_write`, `reply_data` of `can_respond`). Fields must be at the top level of the JSON object, and unknown fields are ignored.
- **No authentication:** Any TCP client that can reach port 8080 has the full command set (target power, SWD access to an attached target, and pod OTA) unless the LAN policy is `locked` or `off`. Restrict access at the firewall/VLAN level. The cloud channel is unaffected (TLS + Ed25519 device auth). See [Security model](#security-model).
- **Concurrent commands:** Up to 5 LAN connections are served at once. Commands run one at a time on one worker task, and the commands that use the capture hardware take turns: a second one gets `"busy"` until the first one's reply is complete.
- **DAC runs until stopped:** A `generate` with `duration_ms` 0 runs the DAC until `dac_stop`, the next `generate`, `measure` or `replay`, or `OUTPut OFF` over SCPI.
- **Separate DAC and ADC sample-clock domains:** the DAC8551 engine runs on the 48 MHz `clk48` domain (SPI-limited) while the MCP33131 ADC runs in the 24 MHz domain with its divider floored at 60 (max ~400 kSPS). Each is selected per command via `sample_rate_mhz`; the firmware snaps to the nearest achievable divider and logs the actual rate.
