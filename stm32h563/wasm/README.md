# sensor_def.wasm

The BenchPod sensor definition engine (`../src/sensor_def*.c`, `../src/sensor_stim.c`) built as
WebAssembly, so the server, the web app and the desktop app check and preview definitions with
the same code the pod runs. The engine is unchanged: this directory adds a small wrapper with a
JSON ABI (`sd_wasm.c`), a freestanding libc shim (`libc/`) and the tests.

It lives next to the engine (`stm32h563/wasm/`) because it builds the engine's sources directly
and its golden test reads the engine's fixtures (`../test/fixtures/sensor_defs/`).

## Build and test

Needs Homebrew LLVM and LLD (`brew install llvm lld`) and, for the test, Node.

```bash
make          # build/sensor_def.wasm (about 190 KB, no imports)
make test     # shim math vs libm, then every golden call replayed in Node
make clean
```

`make test` runs three things:

1. `shim_math_test`: the shim's `round`, `llround`, `fmod` and `ldexp` against the host libm,
   bit for bit (edge cases and 2 million random doubles).
2. `golden_host`: `sd_wasm.c` built natively runs about 120 calls (the five library sensors at
   several times and param values, live register images, UART requests, an analog sensor,
   stimulus files, broken definitions) and writes each call and its result to
   `build/golden.jsonl`.
3. `test_wasm.mjs`: replays every line against `sensor_def.wasm` through `sensor_def.mjs` and
   checks the JSON is byte for byte the same. It also checks that every number in the output is
   printed the way JavaScript prints it.

The compile flags are `--target=wasm32 -nostdlib -ffreestanding -O2 -ffp-contract=off`; the link
is `wasm-ld --no-entry --gc-sections --strip-all`. The exports are named with
`__attribute__((export_name(...)))` in `sd_wasm.c`. `-ffp-contract=off` keeps the compiler from
fusing multiply and add, so the results round the same as on the pod and the host.

The server vendors a copy of `build/sensor_def.wasm` (embeddedci-server
`internal/sensordef/wasm/`, updated with `make sensordef-wasm` there).

## The libc shim

The engine needs `memcpy`, `memmove`, `memset`, `memcmp`, `memchr`, `strlen`, `strchr`, `strcmp`
and a few exact math functions. `floor`, `ceil`, `trunc`, `sqrt` and `fabs` are single WASM
instructions. `round` (half away from zero; WASM's own `nearest` rounds half to even), `llround`,
`fmod` and `ldexp` are exact algorithms in `libc/shim.c` (`fmod` and `ldexp` follow musl). The
transcendental functions are the engine's own (`sensor_def_math.c`).

## ABI (version 1)

Exports: `memory` and the functions below. Every call is stateless: it compiles the definition
again, runs it and writes one JSON document into the output buffer. The call returns the length
in bytes; `out_ptr()` is where it starts (UTF-8, valid until the next call). One instance runs
one call at a time; use one instance per thread or a pool.

| Export | Signature |
|---|---|
| `abi_version` | `() -> u32`, 1 |
| `sd_alloc` | `(n: u32) -> ptr`, 16-byte aligned, 0 when memory cannot grow |
| `sd_free` | `(ptr)`; memory comes back once every live allocation is freed |
| `out_ptr` | `() -> ptr` |
| `validate` | `(def, def_len) -> len` |
| `preview` | `(def, def_len, values, values_len, live, live_len, t_ms: f64, input: f64) -> len` |
| `uart_answer` | `(def, def_len, values, values_len, request, request_len, t_ms: f64) -> len` |
| `stimulus_validate` | `(stim, stim_len) -> len` |
| `stimulus_sample` | `(stim, stim_len, from_s: f64, to_s: f64, step_s: f64) -> len` |

A host copies each input into memory from `sd_alloc`, makes the call, reads the output, then
frees the inputs. `def` and `stim` are definition and stimulus JSON (format.md; YAML must be
converted to JSON first). `values` is an optional JSON object of param values,
`{"temperature_c": 30}` (pass 0, 0 for the defaults); a value outside the param's range is an
error, not clamped. `live` is an optional 256-byte register image (0, 0 for none).

### Errors

Every call answers `{"ok": false, ...}` on an error:

```json
{"ok": false, "error": "registers[3].s16be: unknown name 'acel_lsb' at 12", "code": -5,
 "path": "registers[3].s16be", "message": "unknown name 'acel_lsb'", "json_off": 812, "expr_off": 12}
```

`error` is the engine's message in the format.md style (`path: message`, plus ` at N` inside an
expression). `code` is an `SD_E_*` value from `sensor_def.h`. `json_off` is the byte offset in
the JSON given, `expr_off` the character offset inside the expression (-1 when none). Errors in
the call's own arguments use paths such as `values.temperature_c`, `live_regs` and `step_s`.

### validate

```json
{"ok": true, "kind": "i2c", "name": "bme280", "label": "Bosch BME280: ...", "program_bytes": 1656,
 "params": [{"key": "temperature_c", "unit": "C", "label": "Temperature", "min": -100, "max": 150, "default": 25}],
 "bus": {"address": 118, "alt": [119], "addr_bits": 8, "page": 0, "auto_increment": true, "wrap": 255,
         "register_bytes": 1, "command_map": false,
         "conversion": {"trigger": 244, "busy_at": 243, "busy_mask": 8, "while_busy": "flag", "us": 5500},
         "data_ready": null, "fifo": null, "commands": []},
 "dynamic": [247, 248], "read_only": [208]}
```

- I2C `bus`: `address`, `alt`, `addr_bits`, `page`, `auto_increment`, `wrap`, `register_bytes`
  (2: word registers, register p is image bytes 2p and 2p + 1), `command_map` (16-bit commands
  without a page) and, with the command map, `map`: one `{addr, at, span}` per address, the
  image bytes `at` .. `at + span - 1` serve addresses `addr` .. `addr + span - 1`. SPI `bus`:
  `spi_mode`, `read_bit`, `read_val`, `inc_bit` (null when none), `inc_val`, `address_mask`,
  `dummy_bytes`. Both: `conversion`, `data_ready` (`active_high`, `level`, `on`, `pulse_us`,
  `clear_on_read`), `fifo` (`at`, `frame_len`, `depth`, `drop`), `commands` (`at`, `busy_us`,
  `command`: the command's address).
- `dynamic`: image addresses rebuilt on every change: the bytes whose owner, a register or a
  command's answer, reads a param, `reg[]`, `t`, a state and so on (or has `reload: "always"`).
  A command-map sensor (an SHT3x: `register_address_bits: 16` without a page) lists its answers
  here. `read_only`: addresses the pod restores after a DUT write (`writable: false`).
- UART: `"uart": {"baud", "data_bits", "parity": "N"|"E"|"O", "stop_bits", "messages", "requests"}`.
- Analog: `"analog": {"output": "voltage"|"current", "excitation", "min", "max"}`.

### preview

What the sensor puts out at `t_ms` (milliseconds since the instance started) for `values`:

- I2C/SPI: `{"ok", "kind", "t_ms", "power_on", "image_b64", "changed", "dynamic", "read_only"}`.
  Without `live` the image starts from power on (`power_on: true`), else from `live` (what the
  DUT wrote); either way it is then rebuilt at `t_ms`. `image_b64` is the 256-byte image,
  `changed` the addresses the rebuild changed.
- UART: `{"ok", "kind", "t_ms", "epochs", "at_ms", "text", "bytes_b64", "next_ms"}`. It runs
  every epoch from 0 to `t_ms` (states advance) and returns the last one, due at `at_ms` (null
  when none is due yet). `text` has one character per byte (bytes 0x80 and up as `\u0080` to
  `ÿ`); `bytes_b64` is exact. At most 200000 epochs.
- Analog: `{"ok", "kind", "t_ms", "value", "unit": "V"|"mA"}`. `input` is the voltage the pod
  measures (excitation sensors), ignored otherwise.

### uart_answer

Feeds the request bytes to a UART sensor, all at `t_ms`, and lists every answer:
`{"ok": true, "answers": [{"after_byte": 13, "delay_ms": 0, "text": "+READ: -3.1\r\n", "bytes_b64": "..."}]}`.
`after_byte` is the index of the request byte that completed the match.

### stimulus_validate

`{"ok", "kind": "stimulus", "name" (null when the file has none), "program_bytes", "rate_hz",
"duration_s", "repeat", "signals": [{"name", "instance", "param"}], "faults": [...]}`. A fault
has `type`, `at_s`, `duration_s` (null: until cleared), `sensor`, `param`, `value` (null when
none), plus `rate_hz` for garbage and `at`, `mask` for corrupt.

### stimulus_sample

Values for a chart at `from_s + k * step_s` up to `to_s` (at most 10000 points):
`{"ok", "from_s", "to_s", "step_s", "t": [...], "signals": [{"name", "instance", "param",
"values": [...]}], "faults": [{"index", ...fault fields, "windows": [[on_s, off_s], [on_s, null]]}]}`.
The fault windows are where the player switches each fault on and off when stepped at these
times (null: still on at `to_s`).

### Numbers

Numbers are printed in the shortest form that reads back as the same double, the way
JavaScript's `String(number)` prints them. NaN and infinities print as `null`.

## sensor_def.mjs

An ES module without dependencies for the browser, Electron and Node:

```js
import { loadSensorDef } from "./sensor_def.mjs";
const sd = await loadSensorDef(new URL("./sensor_def.wasm", import.meta.url));
sd.validate(defJsonOrObject);                           // parsed validate result
sd.preview(def, { temperature_c: 30 }, { tMs: 1000 });  // adds .image / .bytes (Uint8Array)
sd.preview(def, null, { liveRegs: uint8ArrayOrBase64 });
sd.uartAnswer(def, null, "AT+READ?\r\n");               // a string goes as UTF-8
sd.stimulusValidate(stim);
sd.stimulusSample(stim, 0, 120, 0.5);
```

`loadSensorDef` takes a URL (fetched), a `Response`, the bytes or a compiled
`WebAssembly.Module`. Each method has a `...Raw` twin that returns the JSON text unparsed.
