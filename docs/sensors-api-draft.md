## Emulated sensors

The pod can **play sensors** for the DUT: I2C, SPI and UART sensors on LA channels, and analog
sensors on the analog outputs. What a sensor does is not built into the firmware: the host sends a
**sensor definition** (the format is documented in the public sensor library,
`embeddedci-com/benchpod-sensors`, `docs/format.md`) and the pod compiles it and plays it. The pod
never stores definitions; they are gone after `sensor_stop`, a reboot or a firmware update.

Up to 4 I2C sensors (on one bus or on several), 1 SPI sensor, 1 UART sensor and the analog sensors
run at the same time. Each one is an **instance** with a name the host picks
(`[a-z0-9_]{1,32}`; usually the definition's `name`).

I2C, SPI and UART sensors need the **emulation** FPGA image. `sensor_start` loads it when another
image is running (about a second) and says so in its reply. While it runs, the closed DAC loop
(`dac_control_loop`) and deep DAC replay are not available; captures, the UART proxy, SWD, GPIO
and `generate` are. Analog sensors work in every image.

Capability in `status`: `"sensors"` (with `"sensors_format":1`). The commands below replace the
earlier single-sensor `sensor_*`, `sensor_types` and `gps_*` commands.

Typical flow:

1. `sensor_define` (one or more chunks) for each instance.
2. `sensor_start` with its pins (`set_la_voltage` first, and pull-ups on I2C lines).
3. `sensor_set` to change readings, `sensor_stimulus` + `sensor_stimulus_start` to play values over
   time, `sensor_fault` to inject a fault.
4. `sensor_stop`.

### `sensor_define` — upload and compile a definition

The definition is JSON (clients convert YAML). It is sent base64 encoded in chunks, so it fits a
cloud command (at most 1280 bytes): send `offset` 0 first, then each next chunk at the offset where
the previous one ended. The chunk that reaches `total` compiles it.

```json
{"cmd":"sensor_define","instance":"env","offset":0,"total":3225,"data_b64":"eyJmb3JtYXQi..."}
```

| Field | Type | Required | Description |
|---|---|---|---|
| `instance` | string | yes | Instance name. Defining a running instance stops it first. |
| `offset` | integer | yes | Byte offset of this chunk in the JSON (before base64). |
| `total` | integer | yes | Size of the whole JSON, at most 32768. |
| `data_b64` | string | yes | This chunk, base64 (standard or URL alphabet). |
| `dry_run` | bool | no | Compile and report, keep nothing (validation). Send it on the last chunk. |

Replies: `{"status":"ok","instance":"env","received":960}` for a chunk, and for the last one:

```json
{"status":"ok","instance":"env","kind":"i2c","name":"bme280","label":"...","program_bytes":1648,
 "params":[{"key":"temperature_c","unit":"C","min":-100,"max":150,"default":25}],
 "i2c":{"address":"0x76","alt_addresses":["0x77"]}}
```

A definition that does not compile: `{"status":"error","error":"invalid definition","detail":
"registers[4].s16be: unknown name 'ax' at 17"}`. Chunks out of order: `"chunk at offset N, expected
M"` (start again at 0).

### `sensor_start` — start an instance

```json
{"cmd":"sensor_start","instance":"env","address":"0x77","pins":{"sda":2,"scl":1,"drdy":7}}
```

| Field | Type | Required | Description |
|---|---|---|---|
| `instance` | string | yes | A defined instance. |
| `pins` | object | yes | I2C `sda`, `scl` and optional `drdy`; SPI `sclk`, `mosi`, `miso`, `cs`, optional `drdy`; UART `tx` (pod to DUT) and optional `rx` (DUT to pod); analog `output`: `"dac"` or `"current"`. Values are LA channels (1..14) or wiring names. |
| `address` | string | no | I2C: the definition's `address` or one of its `alt_addresses`. |
| `baud` | integer | no | UART: overrides the definition's baud rate. |
| `values` | object | no | Initial param values (otherwise the defaults). |

Reply: `{"status":"ok","instance":"env","slot":"i2c0","image":"emulation","image_loaded":true,
"unavailable":["dac_control_loop","deep_replay"]}`.

Several I2C instances may share `sda`/`scl` when their addresses differ. Pin conflicts are refused
as for every LA owner (`la_pins` shows instances as owner `sensor:<instance>`).

### `sensor_set` — change readings

```json
{"cmd":"sensor_set","instance":"env","values":{"temperature_c":31.5,"humidity_pct":60}}
```

A value outside the param's range is refused with its range. The DUT sees the new reading on its
next read (I2C/SPI) or next message (UART).

### `sensor_stimulus` — upload a stimulus

Same chunking as `sensor_define`, without `instance` (a stimulus may drive several instances; its
signals use `instance.param`, or `param` when only one instance runs). Only one stimulus is loaded at
a time.

```json
{"cmd":"sensor_stimulus","offset":0,"total":812,"data_b64":"..."}
{"cmd":"sensor_stimulus_start"}
{"cmd":"sensor_stimulus_stop"}
```

`sensor_stimulus_start` starts it at t = 0 (`"at_s"` to start elsewhere). It ends after its last
segment (signals hold their last value) unless it has `repeat: true`.

### `sensor_fault` — inject a fault now

```json
{"cmd":"sensor_fault","instance":"env","type":"no_response","duration_s":2}
{"cmd":"sensor_fault_clear","instance":"env"}
```

Types and fields as in the format's faults section. Without `duration_s` a fault lasts until
`sensor_fault_clear`.

### `sensor_stop`

`{"cmd":"sensor_stop","instance":"env"}` or `{"cmd":"sensor_stop","all":true}`. Releases the pins
and forgets the definition. The emulation image stays loaded until another feature needs a
different image.

### `sensor_status`

```json
{"status":"ok","image":"emulation","stimulus":{"loaded":true,"running":true,"t_s":12.4},
 "instances":[{"instance":"env","kind":"i2c","name":"bme280","slot":"i2c0","running":true,
   "address":"0x76","pins":{"sda":2,"scl":1},
   "values":{"temperature_c":25,"pressure_pa":101325,"humidity_pct":40},
   "faults":[],"bus":{"transfers":120,"writes":14,"last_write":{"at":"0xF4","value":"0x27"}}}]}
```

UART instances report `bus.sent_bytes`, `bus.received_bytes`, `bus.messages`, `bus.overruns`.

### `sensor_regs` — read an I2C/SPI image

`{"cmd":"sensor_regs","instance":"env","start":"0xF7","len":8}` returns the bytes the DUT reads now
(its own writes included), chunked like other sample arrays.

### `sensor_la` — raw bus capture

`{"cmd":"sensor_la","instance":"env","samples":4096}`: the instance's bus pins, captured.
