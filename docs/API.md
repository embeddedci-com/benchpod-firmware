# bench-pod-firmware TCP/JSON API

The firmware exposes a plain-text JSON API over a TCP socket. Commands are sent as single-line JSON objects and responses are returned as one or more single-line JSON objects.

---

## Transport

| Property | Value |
|---|---|
| Protocol | TCP |
| Default port | `8080` |
| Framing | Newline-delimited (`\n`) — one JSON object per line |
| Encoding | UTF-8 |
| Concurrency | Single client at a time |

Connect with any TCP client:

```bash
nc <device-ip> 8080
```

The IP address is printed on the UART1 debug console at boot and is also returned by the `status` command.

**Latency.** The pod disables Nagle's algorithm (`TCP_NODELAY`) on every accepted connection, so small replies — a JSON ack, an SWD `c` sample byte — are sent immediately rather than batched. The RP2350↔ESP32 backhaul UART also auto-negotiates from 115200 up to 921600 baud at boot, so the per-command handshake cost is low. Both are transport-internal; no client action is required.

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

There is no login, no API key, and no allow-list. "Single client at a time" is a
dispatcher limitation, not a security control — it does not stop an attacker, it only
means they queue. The USB-CDC console is equivalent and assumes physical access.

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

Commands that return large arrays (`capture`, `stream`) send the data in multiple packets of up to 256 samples each. The first packet uses `"status":"ok"` and subsequent packets use `"status":"chunk"`. Every packet includes a `"more"` boolean that indicates whether additional packets follow.

```json
{"status":"ok",    "data":[142,139,141,...], "more":true}\n
{"status":"chunk", "data":[144,140,138,...], "more":true}\n
{"status":"chunk", "data":[135,137,140,...], "more":false}\n
```

A client must read until it receives a packet with `"more":false` to know the transfer is complete.

---

## Commands

| `cmd` | Purpose | `data` returned | Multi-packet |
|---|---|---|---|
| `ping` | Connectivity check | `"pong"` (string) | no |
| `status` | Firmware / WiFi info | object | no |
| `generate` | Start DAC waveform | `null` | no |
| `capture` | Blocking ADC snapshot | array of uint8 | yes |
| `stream` | Async ADC capture | array of uint8 | yes |
| `measure` | DAC + ADC loopback | array of uint8 | yes |
| `capture_dual` | Simultaneous ADC + raw LA capture off one trigger | array of uint16 | yes |
| `load` | Upload a waveform for replay (chunked) | object | no |
| `replay` | Play the recorded/uploaded trace out the DAC | object | no |
| `dac_stop` | Stop any running DAC output | `null` | no |
| `analog_path` | Apply a named analog path (flips mux + relays) | object | no |
| `dac_out` | Route a DAC output path + set a calibrated voltage | object | no |
| `adc_read` | Route an ADC source + return a calibrated reading (mV) | object | no |
| `dac_mux` | Low-level DAC output mux (U55) — prefer `dac_out` | object | no |
| `cal_switch` | Low-level calibration relays (U58) — prefer `analog_path` | object | no |
| `test` | Pico-side pattern (no FPGA) | array of uint8 | yes |
| `la` | Logic-analyzer pin control: step pulses + pull-ups (LA1–8) | object | no |
| `la_pins` | Report all 12 LA pins: function, gpio mode/level, pull, live levels | object | no |
| `gpio` | Configure / release / drive / read LA pins as GPIO | object | no |
| `la_voltage` | Select the LA I/O-bank voltage (1.8/3.3 V); required before any LA op | object | no |
| `nrst` | Drive the dedicated target-reset line (v3 pods) | object | no |
| `usb_cc` | Read the USB-C CC lines: orientation + source current advertisement (v3 pods) | object | no |
| `target_power` | Enable/disable a target-power eFuse | object | no |
| `target_status` | Read target-power eFuse state (enabled/fault/valid) | object | no |
| `power_status` | One-shot INA238 reading of both supply rails | object | no |
| `power_profile` | Record a current/voltage profile of one eFuse rail | object | on `stop` |
| `uart_proxy_start` | Enter transparent UART bridge mode | `"uart ready"` (string) | no |
| `sensor_start` | Arm an emulated I2C sensor (BMP280) | object | no |
| `sensor_set` | Set the emulated sensor's readings | object | no |
| `sensor_stop` | Disarm the emulated sensor | `null` | no |
| `sensor_status` | Sensor + I2C-bus activity counters | object | no |
| `sensor_regs` | Read the emulated register image | array of uint8 | yes |
| `sensor_la` | Raw I2C-bus logic capture | array of uint8 | yes |
| `identity_public` | Get the device Ed25519 public key | object | no |
| `identity_pop` | Sign a nonce (proof of possession) | object | no |

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

Start continuous waveform generation on the DAC output (SMA connector).

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
| `sample_rate_mhz` | number (MHz) | no | auto | FPGA DAC sample-clock rate. Omit to auto-pick the highest rate that fits the waveform in the 4096-sample buffer. A **lower** rate reduces high-frequency clock feedthrough on the analog output (cleaner scope trace) at the cost of fewer samples per period. Range ≈ `0.001`–`12` MHz (FPGA HFOSC is 24 MHz, divider 2–65535). The firmware snaps to the nearest achievable divider and logs the actual rate. |

**Amplitude and offset arithmetic**

The 8-bit DAC output value at each sample point is clamped to `[0, 255]`:

```
sine:     value = offset + amplitude × sin(2π × t/T)
square:   value = offset ± amplitude  (high for first half-period, low for second)
sawtooth: value = offset - amplitude … offset + amplitude  (linear ramp per period)
```

With `amplitude=100, offset=128` the output swings between 28 and 228 (out of 255), centred at mid-scale.

**Frequency resolution**

The firmware precomputes one full period in a sample buffer and loops it via DMA. Period length in samples is:

```
period_samples = floor(sample_rate / freq)
```

where `sample_rate = 48 MHz / divider` — on gateware ≥ 13 the DAC8551 engine runs on the 48 MHz `clk48` domain (divider ≥ 2, so up to ~24 MSPS; the DAC8551 SPI caps the achievable rate below that).

The maximum buffer size is 4096 samples, which at the ~24 MSPS ceiling limits the minimum frequency to approximately **5.9 kHz**. Below that the period is clamped to 4096 samples and the actual frequency will be higher than requested.

#### Response

```json
{"status":"ok","data":null}
```

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
→ 100 kHz sine clocked at 1 MHz (10 samples/period) instead of the default 12 MHz. Coarser waveform but ~12× lower clock feedthrough — useful for a cleaner scope trace on breadboard wiring.

#### Error cases

| Condition | Message |
|---|---|
| Unknown `waveform` value | `"unknown waveform"` |
| `freq` ≤ 0 or `amplitude` = 0 | `"generate failed"` |

---

### `capture` — blocking ADC snapshot

Capture a fixed number of ADC samples and return them. The command blocks the TCP connection until all samples are collected and sent.

#### Request

```json
{"cmd":"capture","samples":<count>,"sample_rate_mhz":<MHz>}
```

| Field | Type | Required | Default | Max |
|---|---|---|---|---|
| `samples` | integer | no | `256` | `32768` |
| `sample_rate_mhz` | number (MHz) | no | max (~0.4) | — |

`sample_rate_mhz` sets the ADC sample clock. Omit it for the maximum ~400 kSPS rate; a **lower** rate stretches the capture window so a slow waveform fits in the 32768-sample buffer. For example, 32768 samples at ~400 kSPS spans ~82 ms, and `0.08` (80 kS/s) spans ~410 ms — enough to see a motor current waveform or other low-frequency signal. The v2 MCP33131 ADC runs in the 24 MHz domain and its divider is floored at 60, so the maximum rate is 24 MHz ÷ 60 ≈ 400 kSPS; the firmware snaps to the nearest achievable divider and logs the actual rate.

#### Response

Chunked JSON array, 256 samples per packet:

```json
{"status":"ok",    "data":[142,139,141,144,139,...], "more":true}\n
{"status":"chunk", "data":[138,140,143,141,142,...], "more":false}\n
```

Each sample is an unsigned 8-bit integer (0–255) representing the raw ADC output.

If `samples` ≤ 256 the entire result fits in a single packet:

```json
{"status":"ok","data":[142,139,141,...],"more":false}\n
```

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
| `samples` = 0 or > 32768 | `"samples out of range"` |
| DMA error | `"capture failed"` |

#### Loopback test

To verify the signal path, start a waveform then capture:

```json
{"cmd":"generate","waveform":"sine","freq":1000000,"amplitude":100,"offset":128}
{"cmd":"capture","samples":4096}
```

The captured array should show a sinusoidal pattern at the expected period.

---

### `stream` — async ADC capture with immediate delivery

Like `capture`, but ADC data is delivered in 256-sample chunks as each DMA transfer completes rather than buffering the full capture first. Useful for larger captures where you want to start processing data before all samples arrive.

The firmware starts the first DMA transfer immediately and sends each chunk as soon as it is ready. The next DMA transfer is queued while the previous chunk is being sent over TCP.

#### Request

```json
{"cmd":"stream","samples":<count>,"sample_rate_mhz":<MHz>}
```

| Field | Type | Required | Default | Max |
|---|---|---|---|---|
| `samples` | integer | no | `256` | `32768` |
| `sample_rate_mhz` | number (MHz) | no | max (~0.4) | — |

`sample_rate_mhz` has the same meaning as in `capture` and applies to every chunk of the stream.

#### Response

Same chunked format as `capture`:

```json
{"status":"ok",    "data":[...256 samples...], "more":true}\n
{"status":"chunk", "data":[...256 samples...], "more":true}\n
{"status":"chunk", "data":[...256 samples...], "more":false}\n
```

#### Example

```json
{"cmd":"stream","samples":4096}
```

#### Error cases

| Condition | Message |
|---|---|
| `samples` = 0 or > 32768 | `"samples out of range"` |

#### `capture` vs `stream`

| | `capture` | `stream` |
|---|---|---|
| Execution | Blocks until done | Returns chunks as they arrive |
| Latency to first byte | After all samples collected | After first 256 samples |
| Max samples | 32768 | 32768 |
| Use case | Snapshot, loopback test | Larger captures, real-time inspection |

---

### `measure` — simultaneous DAC generate + ADC capture (loopback)

Generates a waveform on the DAC and simultaneously captures ADC samples in a single command. Both DMA channels start in the same `dma_start_channel_mask` call, so capture begins as the waveform is being emitted. Designed for loopback / round-trip measurements where the DAC output is wired (directly or through a DUT) to the ADC input.

After the ADC DMA finishes, the firmware automatically stops the DAC loop before returning the captured data.

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
| `samples` | integer | no | `256` | Number of ADC samples to capture (max `32768`) |
| `sample_rate_mhz` | number (MHz) | no | auto | Sample-clock rate for **both** the DAC and ADC. Omit to auto-pick. Same semantics and range as `generate`'s `sample_rate_mhz`. |

#### Response

Same chunked format as `capture`. The first packet uses `"status":"ok"` and any subsequent packets use `"status":"chunk"`; the last packet has `"more":false`.

```json
{"status":"ok",    "data":[129,148,167,184,...], "more":true}\n
{"status":"chunk", "data":[197,205,209,209,...], "more":false}\n
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
| `samples` = 0 or > 32768 | `"samples out of range"` |
| Unknown waveform / invalid params | `"measure start failed"` |
| ADC capture did not complete within 5 s | `"measure timeout"` |

#### `measure` vs `generate` + `capture`

| | `generate` + `capture` (separate) | `measure` (combined) |
|---|---|---|
| DAC start time | First | Synchronised with ADC |
| ADC start time | Second (after `generate` response round-trip) | Synchronised with DAC |
| Skew between DAC and ADC start | Tens of ms (TCP round-trip) | A few PIO cycles |
| DAC keeps running after | Yes (until next `generate` or duration expires) | No (stopped automatically after capture) |
| Use case | Independent generation and capture, long-running signals | Loopback / round-trip measurements, paired waveform/capture |

---

### `capture_dual` — simultaneous ADC + LA capture (one trigger)

Arm the ADC and the raw 12-channel logic analyzer off a **single** trigger and stream both regions back as one array. The ADC samples come first, then the LA words; the server splits the array at `adc_samples`. Either count may be `0` to capture just the other (so this one verb also serves ADC-only or LA-only). This is the unified capture (gateware opcode `OP_CAPTURE` = `0x31`); the standalone `OP_START_CAPTURE` (`0x20`, ADC-only) and `OP_LA_CAPTURE` (`0x69`, 24-bit deep LA) still exist.

#### Request

```json
{"cmd":"capture_dual","adc_samples":<N>,"adc_rate_mhz":<R>,"la_samples":<M>,"la_rate_mhz":<S>}
```

| Field | Type | Required | Default | Notes |
|---|---|---|---|---|
| `adc_samples` | integer | no | `0` | ADC 16-bit samples. `adc_samples + la_samples` must fit the read-back buffer (≤ `32768`). |
| `adc_rate_mhz` | number (MHz) | no | max | ADC sample clock (24 MHz domain, divider ≥ 60 → max ~0.4 MHz). |
| `la_samples` | integer | no | `0` | Raw 12-channel LA words (≤ `4161536`, the full 8 MB LA region). |
| `la_rate_mhz` | number (MHz) | no | max | LA sample clock. |

`adc_samples` and `la_samples` must not both be `0`.

#### Response

Chunked array of unsigned 16-bit counts (ADC counts, then LA words). The **first** packet also carries the **achieved** rates so the server can build an aligned time-base — the gateware floors the ADC to a divider ≥ 60, so the actual rate can differ from the request:

```json
{"status":"ok","adc_rate_hz":399361,"la_rate_hz":6000000,"data":[...],"more":true}
```

| Field | Meaning |
|---|---|
| `adc_rate_hz` | Achieved ADC sample rate in Hz (`24 MHz ÷ divider`); omitted when `adc_samples` = 0. |
| `la_rate_hz` | Achieved LA sample rate in Hz; omitted when `la_samples` = 0. |

#### Error cases

| Condition | Message |
|---|---|
| `adc_samples` and `la_samples` both 0 | `"adc_samples and la_samples are both 0"` |
| `adc_samples + la_samples` exceeds the read-back buffer | `"samples out of range (adc+la must fit the read-back buffer)"` |
| Capture did not arm | `"capture failed"` |

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
| `trigger.la` | integer | yes | — | LA channel 1..12 to watch. **Any** pin, whatever its function — triggers observe, they do not claim. |
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
| `trigger.la` outside 1..12 | `"trigger la must be 1..12"` |
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

### `load` — upload a waveform for replay

Upload a host-supplied sample buffer into the device's shared waveform buffer so it can later be played out the DAC with `replay`. This is how you replay a **previously saved** trace: capture a run, save the returned array on the host, and when you want to play it back, upload it with `load` then `replay`.

A full 4096-sample trace does not fit in a single command line (the device caps a JSON command at 256 bytes), so the trace is uploaded in chunks. Each chunk's `data` is the raw sample bytes encoded as **base64url** (RFC 4648 §5, no padding). Send the first chunk with `offset:0` — this claims the shared buffer — then successive chunks at increasing byte offsets. Keep each chunk to **≤150 sample bytes** (~200 base64url chars) so the whole command line stays under the 256-byte limit.

The buffer is held (gated, like `capture`) from the first `offset:0` chunk until a `replay` ships it to the FPGA or the connection closes.

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

`len` is the number of bytes decoded from this chunk; `total` is the running highest filled length (the value to pass as `replay`'s `samples`, or just omit it to replay everything).

#### Error cases

| Condition | Message |
|---|---|
| `data` field missing | `"missing data"` |
| Chunk at `offset > 0` sent without a preceding `offset:0` on this connection | `"load not started"` |
| Another connection holds the buffer | `"busy"` |
| `offset` past the 4096-byte buffer | `"offset out of range"` |
| Malformed base64url, or chunk overflows the buffer | `"invalid data"` |

---

### `replay` — play a recorded trace out the DAC

Play the sample buffer back out the DAC. The buffer holds whichever trace was most recently put there — either the last `capture`/`stream` (so you can capture a signal and immediately play it back), or a host-uploaded trace from `load` (so you can replay a previously saved run). The waveform loops continuously until `dac_stop` (or the next `generate`/`measure`/`replay`).

#### Request

```json
{"cmd":"replay","samples":<count>,"sample_rate_mhz":<MHz>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `samples` | integer | no | full recorded length | Number of samples to play, ≤ the recorded length. |
| `sample_rate_mhz` | number (MHz) | no | `12` (max) | DAC sample clock. Set this to the **same** rate the trace was captured at so the playback time-base matches the recording. |

#### Response

```json
{"status":"ok","data":{"samples":4096}}
```

#### Error cases

| Condition | Message |
|---|---|
| Nothing has been captured or uploaded yet | `"nothing to replay"` |
| `samples` = 0 or greater than the recorded length | `"samples out of range"` |
| Another connection holds the buffer | `"busy"` |
| FPGA load/start error | `"replay failed"` |

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
| `amp` | ADC ← amps screw terminal | J8 |

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

#### `adc_read` — route a source **and** return a calibrated reading

`source` is `ext` (default) / `cal1` / `cal2` / `amp`. The firmware routes the
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
   over the 16-sample burst, limit 1024) — stop the DAC (dac_stop) or let the node settle"}
```

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
`cal2`, `amp_measure`, `cal_path`; `cal1`+`cal2` are mutually exclusive) commands
remain available for diagnostics, but prefer the named paths above.

---

### `test` — pure-Pico diagnostic with a known data pattern

Generates a known sample pattern entirely on the RP2350 (no FPGA involved) and returns it in the same chunked array format as `capture` / `measure`. Useful for verifying TCP / JSON / chunking independent of the FPGA, ADC, and analog frontend.

#### Request

```json
{"cmd":"test","pattern":"<sine|counter|ramp|const>","value":<0-255>,"samples":<count>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `pattern` | string | no | `sine` (or `const` if `value` is given) | `"sine"`, `"counter"`, `"ramp"`, or `"const"` |
| `value` | integer 0-255 | no | `255` (used only for `const`) | Constant byte value |
| `samples` | integer | no | `256` | 1 to `4096` |

#### Patterns

| `pattern` | Output | Visual character |
|---|---|---|
| `sine` | One full sine period across `samples`, centred at 128, amplitude 100 | Smooth curve, easy to spot in a plot |
| `counter` | `0,1,2,…,255,0,1,…` (wraps every 256) | Each sample is unique within a 256-window — easy to spot dropped/reordered bytes |
| `ramp` | Linear `0 → 255` across all samples | Monotonic linear ramp |
| `const` | Every byte = `value` | Constant — verifies chunking and value flow |

#### Response

Same chunked format as `capture` / `measure`: 256 samples per chunk; first chunk uses `"status":"ok"`, subsequent ones use `"status":"chunk"`, last has `"more":false`.

#### Examples

```json
{"cmd":"test"}
```
→ 256-sample sine wave (default).

```json
{"cmd":"test","pattern":"counter","samples":1024}
```
→ 1024 samples `0,1,2,…`, wrapping at 256.

```json
{"cmd":"test","value":255}
```
→ 256 bytes of `255`. Old "all 1's" smoke test.

#### Error cases

| Condition | Message |
|---|---|
| `samples` = 0 or > 4096 | `"samples out of range"` |
| Unknown `pattern` | `"unknown pattern (use sine\|counter\|ramp\|const)"` |

---

### `status` — firmware and connection info

Returns current firmware version, WiFi connection state, and IP address. No parameters.

#### Request

```json
{"cmd":"status"}
```

#### Response

```json
{"status":"ok","data":{"version":"0.2.0","wifi":"ready","ip":"192.168.1.213","rssi_dbm":-57,"caps":["signal","gpio","power","swd","i2c_sensor","uart"]}}\n
```

#### Response fields

| Field | Type | Description |
|---|---|---|
| `version` | string | Firmware version string |
| `wifi` | string | WiFi/TCP state (see below) |
| `ip` | string | Station IP address, or `""` if not connected |
| `rssi_dbm` | integer or `null` | Received signal strength from the AP in dBm (e.g. `-57`). `null` when not associated. Higher (closer to 0) is stronger: roughly `≥ -50` excellent, `-60..-50` very good, `-70..-60` good, `-80..-70` fair, `< -80` weak. Querying this costs an AT round-trip (~30 ms) so it adds a small latency to each `status` call. |
| `board_rev` | string | PCB revision, detected at boot from the `PA3` strap: `"v2"`, `"v3"`, or `"unknown"`. Gates the three features that differ between the two boards — the LA-bank 1.8 V setting, the dedicated NRST pin, and USB-C CC monitoring. |
| `board_rev_mv` | integer | Raw revision-strap voltage in millivolts (`-1` if the strap was not measured — e.g. a v2 pod, where the pad is not connected). Diagnostic only. |
| `nrst_pin` | boolean | `true` when the pod has the dedicated target-reset pin (v3+). When `false`, `nrst` and CMSIS-DAP `SWJ_PINS` reset requests are no-ops. |
| `caps` | array of string | Capabilities this firmware exposes. `"swd"` = SWD debug-probe / flash mode via the pod's CMSIS-DAP probe (`dap_start`); `"i2c_sensor"` = emulated I2C sensors (`sensor_start`); `"uart"` = transparent UART bridge (`uart_proxy_start`); `"nrst_pin"` = dedicated target-reset pin (v3); `"usb_cc"` = USB-C CC monitoring (v3); `"la_pins"` = per-pin functions + the `gpio` command ([pin ownership](#la-pin-ownership)); `"power_profile"` = [`power_profile`](#power-profile); `"gpio_read"` = the gateware can read live pin levels back (v35+); `"capture_trigger"` = [triggered captures](#capture-triggers) (v35+). The last two depend on the **running** gateware image, so they can appear and disappear across an image swap. |

#### `wifi` state values

| Value | Meaning |
|---|---|
| `"disconnected"` | Not associated with an access point |
| `"connecting"` | Association in progress |
| `"connected"` | Associated with AP, no TCP server yet |
| `"ready"` | TCP server is up and accepting connections |

#### Example

```json
{"cmd":"status"}
```
```json
{"status":"ok","data":{"version":"0.2.0","wifi":"ready","ip":"192.168.1.213","rssi_dbm":-57,"caps":["signal","gpio","power","swd","i2c_sensor","uart"]}}
```

---

### Logic Analyzer (LA) GPIO channels

The digital-IO command — `la` — and the SWD probe (`dap_start`) operate on the FPGA's **Logic Analyzer GPIO bank**, addressed by a **1-based LA channel index** (`LA1`..`LA12`).

These pins physically live on the **iCE40 FPGA**, not on the RP2350. The stepper pulse generator and the SWD bit-banger run inside the FPGA gateware; the RP2350 only forwards high-level commands over SPI. **The host must supply LA channel numbers (1..12), not RP2350 GPIO numbers.**

| LA channel | iCE40 SG48 pin | | LA channel | iCE40 SG48 pin |
|---|---|---|---|---|
| `1` | 19 | | `7`  | 27 |
| `2` | 20 | | `8`  | 28 |
| `3` | 21 | | `9`  | 31 |
| `4` | 23 | | `10` | 32 |
| `5` | 25 | | `11` | 35 |
| `6` | 26 | | `12` | 34 |

(Pin mapping for the `vbench_pod_b` / RP2350B board — see `ice40/vbench_pod_b.pcf`. The channel→pin map is fixed per gateware build.)

> **Field name:** the channel field is `la` — a 1-based LA channel index (`1`..`12`), not an RP2350 GPIO number.

---

<a id="dut-header"></a>
### DUT pin header — J1 pinout

Everything the pod exposes to the device under test lands on the 2×11 header
**J1**. LA channel `N` is simply **J1 pin N**, so the channel numbers used by
`la`, `la_capture` and `dap_start` are the header's own numbering.

| Pin | Signal | Pin | Signal |
|---|---|---|---|
| 1 | `LA1` | 12 | `LA12` |
| 2 | `LA2` | 13 | `LA13` *(not exposed as an API channel)* |
| 3 | `LA3` | 14 | `LA14` *(not exposed as an API channel)* |
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
{"cmd":"la","la":<1-12>,"steps":<n>,"delay_us":<us>,"dir_la":<1-12>,"direction":<0|1>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `la` | integer | yes | — | LA channel to pulse (1..12). |
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
The `pull` field in the response says which way each one goes. LA9-LA12 have no
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
| `steps` ≤ 0 / `delay_us` ≤ 0 | `"invalid steps"` / `"invalid delay_us"` |
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

Every one of LA1..LA12 has exactly **one function** at a time. Before this existed the
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

**Captures never own pins.** `la_capture`, `capture_dual`, `sensor_la` and capture triggers
observe all 12 channels whatever their function.

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

When a request names several pins, the **first** conflict is reported and **nothing changes**.

Setting a level on a pin that is not a gpio output:

```
LA<n> is not a gpio output (function <function>[, gpio <mode>]); configure it with {"cmd":"gpio","la":<n>,"mode":"output"}
```

#### Pull compatibility

The fixed bias resistors (LA1/LA2 4.7k up, LA3/LA4 2.2k up, LA5/LA6 10k up, **LA7/LA8 10k
down**, LA9–LA12 none) are checked in **both** directions — when claiming a function on a pin
whose pull is engaged, and when engaging a pull on a pin that already has a function.

| Function | Pull-up engaged | Pull-down engaged (LA7/LA8) |
|---|---|---|
| `none`, gpio `input`, gpio `output`, `swd_clk`, `step`, `step_dir` | ok | ok |
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

Reports the function, gpio mode/level and pull state of all 12 channels, plus the live pin
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
| `function` | one of `none`, `gpio`, `uart_rx`, `uart_tx`, `swd_clk`, `swd_dio`, `i2c_sda`, `i2c_scl`, `step`, `step_dir` |
| `gpio` | `null`, or the gpio mode (`input` / `output` / `open_drain`) |
| `level` | the **commanded** level of a gpio `output` / `open_drain` pin, else `null` |
| `pull` | `null` on LA9–LA12; otherwise the fixed resistor and whether it is engaged |
| `levels` | live pin levels, bit `la-1`, read back from the gateware; `null` when the running image is older than **v35** |

The reply always lists all 12 entries and is ~1.1 kB — it fits the cloud command channel, but
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
| `la` | integer, array of integers, or `"all"` | yes | — | Channel(s) 1..12. `"all"` is allowed **only** with `"mode":"off"`. |
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
| `la` out of range | `"la must be 1..12 (or \"all\" with \"mode\":\"off\")"` |
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

The LA bank I/O voltage (iCE40 `VCCIO_0` — the supply for **all** of LA1..LA12) is
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

> **Scheduled power-on + UART capture.** Because the firmware dispatches one
> connection at a time and `uart_proxy_start` turns the connection into a raw
> byte stream, you cannot send a power-on *while* capturing. Instead schedule it:
> `target_power` with `delay_ms` → `uart_proxy_start` → the eFuse switches mid-
> capture and you catch the boot banner.

> The serial console exposes the same operation as
> `target-power <1|2> <on|off> [delay_ms]` — see
> [usb-serial-interface.md](usb-serial-interface.md). On RP2350B boards the
> eFuses also report fault/valid status; read it with `target_status`.

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
happened. `VBUS` is measured on the **DUT side** of the eFuse. (The ADC `amp` input is a
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
{"cmd":"uart_proxy_start","rx":<1-12>,"tx":<1-12>,"baud":115200}
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
| an LA channel is outside 1..12 or the two are equal | `"invalid uart args"` |
| a UART proxy is already active (console or another connection) | `"uart busy"` |
| `rx` or `tx` belongs to another function | `"pin conflict: …"` (see [pin ownership](#la-pin-ownership)) |
| `rx`/`tx` on LA7 or LA8 with its 10k pull-**down** engaged | `"pull conflict: …"` — the UART idles high |
| the iCE40 does not answer after arming | `"uart proxy failed: FPGA not responding (check gateware)"` |

`rx` and `tx` are **claimed** for the session (functions `uart_rx` / `uart_tx`) and released
when the proxy ends, so nothing else can be armed onto them meanwhile.

> **RX pull hold.** The proxy holds the RX line high for the session so a DUT's floating TX
> pin does not decode as a flood of `0x00` before its firmware brings the UART up. This only
> happens on **LA1–LA6**, whose resistors pull **up**: LA7/LA8 pull *down* (engaging theirs
> would guarantee the very flood it prevents) and LA9–LA12 have no resistor. It is also
> skipped at a 1.8 V bank, where the 3V3-referenced pulls are unavailable. The previous state
> is restored when the proxy ends, and a failure to restore it is logged on the console.

---

## Emulated I2C sensor

The pod can **pretend to be an I2C sensor** on two LA channels: the iCE40 FPGA
acts as an I2C **slave (target)** that the DUT's I2C **master** reads, while the
RP2350 serves the register image. This lets a host-in-the-loop test present a
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
{"cmd":"sensor_start","type":"bmp280","addr":"0x76","sda":<1-12>,"scl":<1-12>}
```

| Field | Type | Required | Default | Description |
|---|---|---|---|---|
| `type` | string | yes | — | Sensor model. Currently only `"bmp280"`. |
| `sda` | integer | yes | — | LA channel (1..12) for the I2C **SDA** line. |
| `scl` | integer | yes | — | LA channel (1..12) for the I2C **SCL** line. |
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

Each unit owns a stable **Ed25519 keypair** that serves as its device identity. The private key is generated on the device the first time it boots (seeded from the RP2350 hardware TRNG), persisted to a dedicated flash sector, and **never overwritten** thereafter — it survives firmware updates (`make flash`) and a WiFi factory-reset (`wifi-clear`). The device only ever **signs**; the private key never leaves the unit.

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
| `READ? [<n>]` | `capture` | Capture `n` (or the `POINts` default) samples at `SENSe:SRATe` and return CSV. Also loads the replay buffer. |
| `TRACe[:DATA] <offset>,"<base64url>"` | `load` | Upload one chunk of a replay waveform. Send `offset` 0 first, then successive offsets (≤150 bytes/chunk to fit the 256-byte line). |
| `TRACe:POINts?` | — | Number of samples currently loaded for replay. |
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
TRACe:POINts?               → 300
SOURce:FUNCtion USER
OUTPut ON
```

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

ADC samples are raw 8-bit values (0–255) from the AD9280. No calibration or scaling is applied. To convert to a physical voltage:

```
V = sample / 255 × V_ref
```

where `V_ref` is the AD9280 reference voltage (3.3 V on this board).

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
| `"unknown waveform"` | `waveform` value not `sine`, `square`, or `sawtooth` |
| `"generate failed"` | Invalid `freq` (≤ 0) or `amplitude` (= 0) |
| `"samples out of range"` | `samples` = 0 or > 32768 |
| `"capture failed"` | DMA setup or hardware error |
| `"missing waveform"` | `measure` request without `waveform` field |
| `"measure start failed"` | `measure` invoked with invalid params or unknown waveform |
| `"measure timeout"` | `measure` ADC DMA did not complete within 5 s |
| `"missing la"` | `la` step request without `la` field |
| `"missing state"` | `target_power` request without `state` field |
| `"missing efuse"` | `target_power` request without `efuse` field |
| `"invalid efuse"` | `target_power` `efuse` is not 1 or 2 |
| `"invalid swd la channel"` | `dap_start` channel outside 1..12 or both channels equal |
| `"missing rx"` / `"missing tx"` | `uart_proxy_start` request without `rx`/`tx` field |
| `"invalid uart args"` | `uart_proxy_start` channel outside 1..12 or `rx`==`tx` |
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

Three families carry a **machine-parseable prefix** — clients (the Python SDK among them) match
on the prefix and show the rest to the user:

| Prefix | Raised by | Details |
|---|---|---|
| `pin conflict: ` | any command claiming an LA pin (`gpio`, `uart_proxy_start`, `dap_start`, `sensor_start`, the `la` step train, SCPI `DIGital:*`) | [pin ownership](#la-pin-ownership) |
| `pull conflict: ` | the same claims, and `{"cmd":"la",…,"pullup":"on"}` | [pull compatibility](#la-pin-ownership) |
| `trigger timeout: ` | `capture`, `capture_dual`, `la_capture` with a `trigger` | [capture triggers](#capture-triggers) |

---

## Session Example

A complete session demonstrating all commands:

```
$ nc 192.168.1.213 8080

{"cmd":"ping"}
{"status":"ok","data":"pong"}

{"cmd":"status"}
{"status":"ok","data":{"version":"0.2.0","wifi":"ready","ip":"192.168.1.213","rssi_dbm":-57}}

{"cmd":"generate","waveform":"sine","freq":250000,"amplitude":100,"offset":128,"duration_ms":500}
{"status":"ok","data":null}

{"cmd":"capture","samples":512}
{"status":"ok","data":[128,151,172,190,204,214,219,219,214,204,190,172,...],"more":true}
{"status":"chunk","data":[148,127,106,86,68,53,42,35,32,33,38,47,59,...],"more":false}

{"cmd":"generate","waveform":"square","freq":1000000,"amplitude":64,"offset":128}
{"status":"ok","data":null}

{"cmd":"stream","samples":1024}
{"status":"ok","data":[192,192,192,191,193,192,64,63,64,65,63,64,...],"more":true}
{"status":"chunk","data":[192,193,191,192,64,63,65,64,192,193,191,...],"more":true}
{"status":"chunk","data":[192,192,192,64,64,63,64,192,192,192,64,63,...],"more":true}
{"status":"chunk","data":[192,191,193,64,64,65,63,192,191,193,64,64,...],"more":false}

{"cmd":"measure","waveform":"sine","freq":250000,"amplitude":100,"offset":128,"samples":256}
{"status":"ok","data":[128,148,168,186,202,214,221,224,221,213,201,185,...],"more":false}

{"cmd":"la","la":1,"pullup":"on"}
{"status":"ok","data":{"la":1,"pullup":1,"ohms":"4.7k"}}

{"cmd":"la","la":1,"steps":200,"delay_us":500}
{"status":"ok","data":{"la":1,"steps":200,"delay_us":500,"status":"started"}}

{"cmd":"identity_public"}
{"status":"ok","data":{"public":"ue5UfSsOAhQYZ332ELokjOI8-uyjcbrd1ic3Nmypvzk"}}

{"cmd":"identity_pop","nonce":"Q2hhbGxlbmdlMTIz"}
{"status":"ok","data":{"signature":"hZ8…(86 chars)…Qq"}}

{"cmd":"status"}
{"status":"ok","data":{"version":"0.2.0","wifi":"ready","ip":"192.168.1.213","rssi_dbm":-57}}
```

---

## Implementation Notes

- **JSON parser limitations:** The firmware uses a minimal flat-JSON parser. Nested objects and arrays in requests are not supported. All command fields must be at the top level of the JSON object.
- **No authentication:** Any TCP client that can reach port 8080 has the full command set — including target power, SWD access to an attached target, and pod OTA. Restrict access at the firewall/VLAN level. The cloud channel is unaffected (TLS + Ed25519 device auth). See [Security model](#security-model).
- **Concurrent commands:** A second TCP connection while one is active is handled by the ESP32 AT layer but not by the firmware dispatcher — only connection ID 0 is dispatched. Do not send a new command while a `capture` or `stream` response is in progress.
- **DAC runs until stopped:** A `generate` command with `duration_ms=0` runs the DAC indefinitely. Issuing another `generate` command replaces the running waveform immediately. There is currently no explicit `stop` command — send `generate` with `amplitude=0` workaround is not valid (returns error); instead send a new `generate` with a different `duration_ms` value, or rely on the previous `duration_ms` expiry.
- **Separate DAC and ADC sample-clock domains (v2):** the DAC8551 engine runs on the 48 MHz `clk48` domain (up to ~24 MSPS, SPI-limited) while the MCP33131 ADC runs in the 24 MHz domain with its divider floored at 60 (max ~400 kSPS). Each is selected per command via `sample_rate_mhz`; the firmware snaps to the nearest achievable divider and logs the actual rate.
