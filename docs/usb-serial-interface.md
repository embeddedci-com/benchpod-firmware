# bench-pod-firmware USB Serial Console

The firmware exposes its interactive console over the STM32H563's **USB CDC-ACM**
port (ST's stock CDC-ACM class device), in addition to the UART debug
console. A host tool (e.g. a provisioning CLI) can open the resulting serial
port and drive commands such as `wifi-set`, `wifi-show`, and `dfu`.

This document describes the contract for talking to that interface. It does not
ship a CLI — it specifies what a CLI needs to know.

---

## Transport

| Property | Value |
|---|---|
| Class | USB CDC-ACM (virtual serial port) |
| USB VID/PID | `0x0483:0x5740` (STMicroelectronics CDC — the host's stock CDC-ACM driver binds) |
| Line settings | `115200 8N1` |
| Framing | Line-oriented; commands terminated with `\n` (or `\r`) |
| Encoding | ASCII/UTF-8 |

Because it is a standard CDC-ACM device, the host OS class driver creates a
serial port automatically — **no libusb, no custom driver, and no cgo** are
needed. The serial port appears as:

| OS | Device path |
|---|---|
| macOS | `/dev/tty.usbmodem*` (or `/dev/cu.usbmodem*`) |
| Linux | `/dev/ttyACM*` |
| Windows | `COMx` |

> CDC-ACM ignores the actual baud rate on the wire, but set `115200 8N1` anyway
> for compatibility with terminal tools.

The same console is **also** available on the UART debug pins; both run
simultaneously and mirror each other's output.

---

## Command protocol

The USB channel is the existing text console. Send a command line terminated by
`\n`; the device executes it and prints human-readable output, then redisplays
the prompt `"> "`.

### Behaviour a parser must account for

- **Character echo:** typed characters are echoed back as you send them.
- **Prompt delimiter:** after each command the device prints `"> "`. Read until
  the next prompt to capture a command's full output.
- **Interleaved logs:** boot messages and asynchronous AT-modem / WiFi events
  (e.g. reconnect attempts) are printed on the same channel and may appear
  between or during command output. Parse by scanning for the documented marker
  lines below rather than assuming the response is contiguous.
- **Quoting:** arguments with spaces must be double-quoted, e.g.
  `wifi-set "My SSID" "my password"`.
- **Password masking:** the WiFi password is never printed in clear text. It is
  shown as `*` in `wifi-show` and masked in AT-command logs (a failed join logs
  `AT+CWJAP="ssid","******"`).

### Command reference

The console runs `help` to print this list live on the device. All argument
tokens are whitespace-separated; quote any value containing spaces. On a usage
or argument error a command prints `usage: ...` or `ERROR: ...` and makes no
change.

#### System / provisioning

| Command | Description |
|---|---|
| `help` | List all commands. |
| `status` | Firmware version, WiFi state, IP, TCP port. |
| `wifi-set <ssid> <password>` | Save credentials to flash and (re)join the AP. Use quotes for values with spaces. |
| `wifi-show` | Show stored SSID (password masked), WiFi state, IP, RSSI. |
| `wifi-clear` | Erase stored credentials (reboot to fully apply). |
| `esp32-reconnect` | Retry ESP32 probe + WiFi join + TCP server bring-up. |
| `esp32-monitor [secs]` | EN-reset the modem and dump raw UART RX (1–30 s, default 3 s) — shows the ESP32 boot log. |
| `reboot` | Software reset via watchdog. |
| `dfu` | Reboot into the STM32 ROM USB DFU bootloader to reflash firmware (host: `dfu-util` / `make flash-dfu`). |

#### Target power & GPIO

| Command | Description |
|---|---|
| `target-power <1\|2> <on\|off> [delay_ms]` | Enable/disable a target eFuse (1 = internal 5 V, 2 = external). Optional `delay_ms` applies the change after a delay (fired from the poll loop); echoes `eFuse<N> ON/OFF` (or `eFuse<N> ON/OFF in <ms> ms`). |
| `target-status [1\|2]` | Show eFuse `enabled`/`fault`/`valid` (both if omitted). |
| `la-step <la 1..12> <steps> <delay_us> [dir_la] [dir]` | Emit `steps` pulses on an LA channel (non-blocking; optional direction channel). |
| `pullup [<1-8\|all> <on\|off>]` | Switch (or, with no args, report) an LA pin pull-up. LA1–8 only; off leaves the line on the board's pull-down. |

The `<la>` argument is a **Logic-Analyzer channel index (1..12)** driven by the iCE40 gateware — see the channel→pin table in [`docs/API.md`](API.md#logic-analyzer-la-gpio-channels). The stepper/SWD bit-banging now lives in the FPGA, not on RP2350 PIO. (The former `gpio-set` console command is gone — drive a line's idle level with `pullup` instead.)

#### FPGA (iCE40 over SPI0)

| Command | Description |
|---|---|
| `spi-ping` | Re-PING the iCE40 FPGA (3 attempts); prints gateware version on success. |
| `spi-status` | Read the FPGA `STATUS` byte (`DAC_RUN`/`CAP_BUSY`/`CAP_DONE`). |
| `flash-ice40 [verify\|diag]` | Program the config flash from the embedded bitstream. `verify` reads back + compares; `diag` probes the JEDEC ID. |

#### I²C peripherals (v2: INA238 ×2 / TCA9554 ×4; analog DAC = DAC8551)

| Command | Description |
|---|---|
| `i2c-scan` | Scan the I2C0 power/IO bus. |
| `ina` | Read both INA238 power monitors (int = 0x40 internal 5 V, ext = 0x44 external supply). |
| `expdump` | Dump the four TCA9554 expanders (LA pull-ups @0x20, eFuse ctrl @0x22, DAC mux @0x24, analog switch @0x26). |

The 16-bit **DAC8551** analog DAC is not an I²C part — it is clocked by the iCE40 over SPI and its output is routed by the TCA9554 mux (@0x24). See the analog commands below.

#### Analog (v2: DAC8551 / MCP33131)

| Command | Description |
|---|---|
| `dac <off\|3v3\|5v\|12v> [volts]` | Route the DAC8551 output path (mux via TCA9554) and hold a calibrated DC voltage. |
| `dacraw <0-255> [divider]` | Raw DAC8551 code, no routing/calibration; `divider` sets the FPGA DAC engine rate (48 MHz / divider). Debug. |
| `adc [ext\|cal1\|cal2\|amp]` | Route an MCP33131 ADC source and return a calibrated reading in mV (default `ext`). |
| `adcraw` | Single raw MCP33131 sample byte. Debug. |
| `measure` | Read the ADC input SMA in volts (= `adc ext`, ÷12 front-end). |

> **v2 analog.** The 16-bit DAC8551 is clocked by the iCE40 DAC sequencer on the 48 MHz `clk48` domain, so the FPGA-sequenced TCP commands `generate`, `replay`, and `measure` **are** available (unlike the retired RP2350B carrier, whose DC-only MCP4728 could not be clocked at waveform rates). The ADC is the 16-bit MCP33131D-10 SAR, sampled through the iCE40 into PSRAM.

#### Debug probe

| Command | Description |
|---|---|
| `dap-start <swclk la> <swdio la>` | Turn the console into the pod's **CMSIS-DAP** probe on the given LA channels (1..12), so host-side OpenOCD (`cmsis-dap` backend) can flash/debug an SWD target through the pod. Replies `dap ready`, then the connection carries length-framed CMSIS-DAP packets. Target reset is the pod's own NRST pin (J1 pin 22) — there is no `nreset` channel argument since rev3. |

#### Serial bridge

| Command | Description |
|---|---|
| `uart-proxy <rx la> <tx la> <baud>` | Open a transparent UART terminal to a DUT on the given LA channels (1..12) at `baud` (8N1). Exit with **Ctrl-]**. See [UART proxy mode](#uart-proxy-mode-uart-proxy). |

> `selftest` runs an STM32H563 silicon health check (HSE/PLL clocks, timer,
> SRAM walking-bit/address pattern, RNG).

### `wifi-set` result markers

After `wifi-set "<ssid>" "<password>"`, scan the output for:

| Outcome | Marker line (substring) |
|---|---|
| Credentials persisted | `[cfg] credentials written to flash` |
| Join succeeded | `[wifi] join OK  ip=<ip>` |
| Join failed (creds still saved) | `[wifi] join failed` |

A typical success transcript:

```
> wifi-set "MyNet" "s3cr3t"
[cfg] credentials written to flash
[wifi] joining "MyNet"...
[wifi] join OK  ip=192.168.1.42
>
```

The credentials are stored in flash and survive reboots and firmware updates
(the config sector is not part of the firmware image).

---

## Firmware update via `dfu`

Sending `dfu` reboots the device into the STM32H563's ROM USB **DFU** bootloader.
Flash a new image from the host with `dfu-util` (e.g. `make flash-dfu`, which runs
`dfu-util ... :leave` and auto-reboots into the app). The application does **not**
flash itself — it only hands off to the built-in ROM bootloader.

```
> dfu
entering DFU (USB bootloader) — flash with dfu-util / `benchpod flash-self`...
```

The CDC serial port disappears at this point and the STM32 ROM DFU device
(`0483:df11`) enumerates instead.

---

## UART proxy mode (`uart-proxy`)

`uart-proxy` turns the console into a **transparent UART terminal** to a DUT —
the bench-pod equivalent of `screen /dev/ttyUSB0`. The wire toggling runs in the
iCE40 gateware (a soft 8N1 UART on two LA channels); the firmware bridges your
keystrokes to the DUT and prints what the DUT sends back. It is the serial
equivalent of the TCP `uart_proxy_start` command.

### Entering the mode

```
uart-proxy <rx la> <tx la> <baud>\n
```

- `rx`, `tx` — **LA channel indices (1..12)**: `rx` is the pin the FPGA samples
  (wire the DUT's TX here), `tx` is the pin the FPGA drives (wire the DUT's RX
  here). They must be distinct.
- `baud` — any value; the firmware computes the FPGA bit-period divisor.
  Frame format is fixed at **8N1**. ≤115200 is solid; higher is best-effort.

| Outcome | Output | Meaning |
|---|---|---|
| Armed OK | `uart ready (press Ctrl-] to exit)` | **UART mode active** — see below |
| Already in use | `ERROR: uart busy` | another client/console owns the UART |
| Bad args | `ERROR: invalid uart args (...)` | back at `> ` prompt |
| Missing args | `usage: uart-proxy <rx la> <tx la> <baud>` | back at `> ` prompt |

A client should send `uart-proxy ...\n` and **read lines until it sees
`uart ready`** (treat any `ERROR:`/`usage:` line as failure).

### After `uart ready`: transparent stream

The line editor steps aside: every byte you type is transmitted to the DUT, and
every byte the DUT sends is printed. There is **no inactivity timeout** — a
serial console may sit idle waiting for the operator.

### Leaving the mode

Press **Ctrl-]** (0x1D, the telnet escape). Only your keystrokes are scanned for
it, so binary DUT output containing 0x1D is never mistaken for the exit. On exit
the UART is disarmed (the TX channel returns to high-Z) and the `> ` prompt
returns.

> The TCP form (`uart_proxy_start`, see [API.md](API.md#uart_proxy_start)) uses a
> different exit since a network client has no single "escape key": it ends on
> socket close, or on a guard-timed `+++` (Hayes-style) to return to JSON.

---

## Talking to it from Go

Any pure-Go serial library works since the device is a normal serial port — no
libusb/cgo dependency. For example, with `go.bug.st/serial`:

```go
port, err := serial.Open("/dev/tty.usbmodem1101", &serial.Mode{BaudRate: 115200})
// write: port.Write([]byte("wifi-set \"MyNet\" \"s3cr3t\"\n"))
// read lines until you see the "> " prompt, then scan for "[wifi] join OK"
//   or "[wifi] join failed".
```

Discover the port by enumerating serial ports and matching USB VID `0x2E8A`
(`enumerator.GetDetailedPortsList()` exposes the VID/PID), or by matching the
platform path patterns in the table above.
