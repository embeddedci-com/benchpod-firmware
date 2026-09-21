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
| `scripts/dmm.py` | Pi | 34401A GPIB driver (linux-gpib) + `MockDMM` |
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
