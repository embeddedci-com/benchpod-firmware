# bench-pod calibration

DMM-referenced calibration of the RP2350B DAC and ADC, plus the v1 loopback
characterization tooling.

> **Status update — GPIB now runs Mac-direct (no Pi, no linux-gpib).**
> The original design (below) put the DMM on a Raspberry Pi behind linux-gpib
> because NI ships no Apple-Silicon driver. In practice linux-gpib's `ni_usb`
> kernel driver could **not** drive our (genuine) NI GPIB-USB-HS on the Pi. We
> replaced it with a pure-Python user-space driver, [`scripts/ni_gpib_usb.py`](scripts/ni_gpib_usb.py)
> (pyusb + libusb, no kernel module), which talks to the 34401A **directly from
> the Mac** — verified: `*IDN? -> HEWLETT-PACKARD,34401A,0,10-5-2` plus live DC
> reads. [`scripts/dmm.py`](scripts/dmm.py) now uses that backend. So the DMM,
> board console, and flashing can all live on the host; the Pi/HTTP bench-server
> split below is optional. Setup: `brew install libusb && pip install -r scripts/requirements-host.txt`.
> The `pi-gpib-setup.md` linux-gpib instructions are retained only for reference.

## Supported DMMs

`scripts/dmm.py` finds the meter on its own (`open_dmm()`, used by `dmm_read.py`): it tries
each driver in turn and takes the first meter that answers `*IDN?` with its model.

| Meter | Connection | Notes |
|---|---|---|
| HP/Agilent/Keysight 34401A | GPIB through an NI GPIB-USB-HS, `scripts/ni_gpib_usb.py` (pyusb) | GPIB address 22 by default (`--addr`) |
| OWON XDM2041, XDM1041 | its USB port, a USB serial port at 115200 8N1 (pyserial) | Found by its CH340 USB ID; `--port` or `DMM_PORT` for another port. Written from OWON's programming manual, not yet tried on a meter |

- `--dmm auto|34401a|owon|mock` (or `DMM=`) picks one when both are plugged in.
- Ranges are the largest voltage you expect: `--range 5.5` is the 34401A's 10 V range and the
  OWON's 5 V range (55,000 counts, reads up to 5.5 V). `--range 10` on the OWON is its 50 V
  range, a digit less.
- The OWON has no NPLC: `--nplc` 5 and above selects its slow rate, 0.5 to 5 medium, below that
  fast. After a range or rate change the driver waits 1.5 s and drops the first reading.
- Another meter is a class with `detect`, `idn`, `read_dc`, `set_range` and `close`, added to
  `DRIVERS` in `dmm.py`. `scripts/test_dmm.py` tests the drivers without hardware:
  `python3 -m unittest calibration/scripts/test_dmm.py`.

## Architecture

The DMM is GPIB-only and lives on a Raspberry Pi (with an NI GPIB-USB-HS). The
Pi runs a small **HTTP/JSON bench server** that owns both instruments and
exposes them over the network; the **cal client runs on any host** (e.g. the
Mac) and speaks only HTTP.

```
  Host (Mac)                         Raspberry Pi (on the network)
  ┌────────────────────┐   HTTP/JSON  ┌───────────────────────────────┐
  │ dac_cal.py         │ ───────────► │ bench_server.py (Flask)       │
  │ adc_cal.py         │ ◄─────────── │  ├─ dmm.py  → GPIB → 34401A    │
  │ cal_client.py      │  --host <ip> │  └─ board_console.py → serial │
  │ (stdlib only)      │              │                     → RP2350B │
  └────────────────────┘              └───────────────────────────────┘
```

Why an HTTP/JSON API (not raw serial-over-TCP): the DMM is GPIB-only on the Pi
(so *some* Pi-side service is required regardless), structured endpoints are
curl-testable and robust, and it makes the Pi a reusable bench head. The brains
(sweep + fit) stay on the client; the Pi is a thin hardware proxy.

> The board console moves to the Pi for this. Flashing (CMSIS-DAP/OpenOCD) can
> stay on the Mac — it's a separate USB link and isn't needed during cal.

## Endpoints

| Method | Path | Returns |
|---|---|---|
| GET | `/health` | `{ok, board, dmm}` |
| GET | `/dmm/idn` | `{idn}` |
| GET | `/dmm/dc?nplc=10&samples=1` | `{volts}` |
| POST | `/board/dac` `{code,channel}` | `{ok, raw}` |
| GET | `/board/adc?avg=32` | `{count, n, spread}` |
| POST | `/board/raw` `{line}` | `{out}` |

## Files

| File | Where | Purpose |
|---|---|---|
| `scripts/bench_server.py` | Pi | HTTP server owning the DMM + board (`--mock` for no hardware) |
| `scripts/dmm.py` | Mac | DMM drivers (34401A over the NI GPIB-USB-HS, OWON XDM over USB serial), `open_dmm()` auto-detection + `MockDMM` |
| `scripts/dmm_read.py` | Mac | Read the DMM from the command line: `idn`, `read [-n N] [--json] [--range 5.5]`, `watch`; `--dmm`, `--port` pick the meter. For calibration runs and sessions that read the DMM themselves |
| `scripts/test_dmm.py` | Mac | Host tests for `dmm.py` (a simulated OWON, the range mapping, detection order) |
| `scripts/adc_comp_sweep.py` | Mac | ADC input divider compensation (R90/C140 vs R91/C72): sine sweep 200 Hz-150 kHz through the cal1 loopback with the SDK, no cable; fits the top-leg C and the HF/DC gain, exits 1 when it is off by more than 5 %. `--fit <json>` refits a saved sweep. `--range low` sweeps the ÷2 low range (analog module v3r4+, relay K5: 910k ‖ 56.4 pF over 910k ‖ ~55 pF, sine kept under 4 V). Rev3 and later |
| `scripts/board_console.py` | Pi | RP2350B console (pyserial) + `MockBoard` |
| `scripts/cal_client.py` | Host | HTTP client + `linfit()` (stdlib only) |
| `scripts/dac_cal.py` | Host | DAC + lower-range ADC cal (board sweep) |
| `scripts/adc_cal.py` | Host | full 0-5 V ADC cal (external source) |
| `pi-gpib-setup.md` | — | linux-gpib + NI-HS one-time setup |
| `scripts/dac_adc_*.py`, `adc_noise.py` | Host | v1 direct-serial loopback tools (retired RP2350B hardware) |

## Quickstart

```bash
# Pi — validate the pipeline with no hardware:
python3 scripts/bench_server.py --mock

# Host — run against it (mock board reproduces the v1 DAC->ADC transfer):
python3 scripts/dac_cal.py --host <pi-ip>
```

With hardware, follow `pi-gpib-setup.md`, then:

```bash
python3 scripts/bench_server.py --board-port /dev/ttyUSB0 --gpib-addr 22
python3 scripts/dac_cal.py --host <pi-ip>     # -> dac_cal.json
python3 scripts/adc_cal.py --host <pi-ip>     # -> adc_cal.json
```

## Outputs

Each cal writes a JSON file with the fit coefficients:

- **DAC**: `V = a + b·code` → command a voltage with `code = (V − a)/b`.
- **ADC**: `V = a + b·count` (lower range from `dac_cal`, full 0-5 V from `adc_cal`).

Averaging the ADC's boundary dither gives sub-LSB resolution in the fit.

## Caveats (v1)

- Absolute accuracy is bounded by the MCP4728's **VDD reference** — the cal is
  valid at that moment/temperature; VDD drift shifts it. (v2's 4.096 V reference
  makes a DMM cal stick.)
- `dac_cal` exercises only the lower ~half of the ADC range (DAC tops ~3.1 V);
  `adc_cal` + an external 0-5 V source covers the rest.
- The Pi-side scripts are written but **untested against real hardware** (built
  before the GPIB adapter shipped); validate with `--mock` first, then on the
  bench. SCPI node names in `dmm.py` may need minor tweaks per instrument.
