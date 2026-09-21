# CMSIS-DAP over the cloud tunnel (fast SWD/flash over the internet)

## Problem (the retired per-bit path)

The original flash/debug path tunnelled OpenOCD `remote_bitbang` over TCP: the
host (OpenOCD) emitted **one command byte per SWCLK edge** and **blocked on every
`c` sample** to read the SWD `ACK` before deciding the next bit. Each of those
read-backs was a full internet round-trip. Over a 30–100 ms link a single 32-bit
transfer cost several RTTs, so flashing tens of KB was effectively unusable. That
`swd_start` path has since been **retired** in favour of the CMSIS-DAP transport
described below.

Batching the *bytes* doesn't help: the round-trips are data-dependent (OpenOCD
must see each ACK), so the bits and the decisions both have to cross the wire.

## Fix: raise the protocol boundary to the DAP transfer

Stop shipping wire bits. Ship **DAP transfers**, and execute the SWD line
protocol — including ACK/WAIT retries — **on the pod**, microseconds from the
pins. The host batches a list of transfers; the pod runs them all locally and
returns one response. That is exactly what **CMSIS-DAP** is (`DAP_Transfer` /
`DAP_TransferBlock`), so we make the pod a real CMSIS-DAP probe and let pyOCD
drive it. pyOCD brings every vendor's flash algorithm (via CMSIS-Packs), so
multi-vendor support comes for free and lives *above* our transport.

```
pytest / CI
  └─ pyOCD                         vendor flash algos, DAP batching, posted-read/RDBUFF
       └─ BenchPodDAPInterface     a pyOCD Interface: ships CMSIS-DAP packets over our transport
            └─ length-framed tunnel (our transport)   [len_lo][len_hi][CMSIS-DAP packet]
                 └─ embeddedci cloud relay            transparent byte pipe — NO server change
                      └─ RP2350 firmware
                           ├─ PROTO_DAP reassembly    (command_handler.c)
                           ├─ dap_process()           CMSIS-DAP command processor (dap.c)
                           └─ swd_ll                   SWD line protocol over remote_bitbang
                                └─ fpga_swd_feed()      EXISTING path → iCE40 swd_engine.v → SWD wire
```

Nothing in the iCE40 gateware or the server changes. The pod drives the wire
through the *existing* `fpga_swd_feed`/`fpga_swd_read` SPI path; only the RP
firmware gains a CMSIS-DAP brain, and the host gains a thin pyOCD probe.

## Wire transport (ours)

A connection enters DAP mode exactly like SWD mode does today:

1. Host sends JSON `{"cmd":"dap_start","swclk":<la>,"swdio":<la>}`.

   > Since rev3 there is **no `nreset` field**. nRESET is the pod's own
   > `/NRST_CONTROL` pin (J1 pin 22 — see the [DUT header pinout](API.md#dut-header)),
   > driven by `swd_ll_nreset()` → `nrst_ctrl.c`, not by an LA channel. CMSIS-DAP
   > `SWJ_PINS` reaches it directly, so connect-under-reset needs no extra setup.
   > An `nreset` sent by an older client is accepted and ignored rather than
   > rejected, so a stale call still flashes. Host-side tooling keeps an `nreset`
   > *flag* (is the target's reset wired to that pin?) to gate
   > connect-under-reset — it just never reaches the pod.
2. Firmware arms the SWD engine (`fpga_swd_arm`, the shared FPGA SWD engine),
   acks `{"status":"ok","data":"dap ready"}\n` **while still in JSON mode**, then
   flips the connection to `PROTO_DAP`.
3. From then on the byte stream carries **length-framed CMSIS-DAP packets**:

   ```
   request   (host → pod):  [len_lo][len_hi][ CMSIS-DAP command bytes (len) ]
   response  (pod → host):  [len_lo][len_hi][ CMSIS-DAP response bytes (len) ]
   ```

   `len` is little-endian u16, ≤ `DAP_PACKET_SIZE` (256). A **zero-length frame**
   (`00 00`) leaves DAP mode and disarms — the connection returns to JSON.

The frame length is the only thing we add on top of standard CMSIS-DAP; the
payload is a verbatim CMSIS-DAP v1 command/response. So the same firmware could
later be exposed to OpenOCD's `cmsis-dap` driver or probe-rs.

## Firmware: supported CMSIS-DAP commands (SWD only)

| ID | Command | Notes |
|----|---------|-------|
| 0x00 | `DAP_Info` | caps=SWD, packet size 256, count 1, empty strings |
| 0x01 | `DAP_HostStatus` | LED — acked, ignored |
| 0x02 | `DAP_Connect` | SWD only (port 1); JTAG rejected |
| 0x03 | `DAP_Disconnect` | |
| 0x04 | `DAP_TransferConfigure` | idle cycles, wait-retry, match-retry |
| 0x05 | `DAP_Transfer` | posted AP reads + trailing RDBUFF, match value/mask |
| 0x06 | `DAP_TransferBlock` | block read primes + final RDBUFF; block write |
| 0x08 | `DAP_WriteABORT` | |
| 0x09 | `DAP_Delay` | |
| 0x0A | `DAP_ResetTarget` | returns "no device-specific sequence" |
| 0x10 | `DAP_SWJ_Pins` | nRESET (bit7) via the FPGA reset line |
| 0x11 | `DAP_SWJ_Clock` | acked (rate is SPI-bound, see Perf) |
| 0x12 | `DAP_SWJ_Sequence` | line reset / JTAG→SWD select |
| 0x13 | `DAP_SWD_Configure` | turnaround period, data phase |
| 0x1D | `DAP_SWD_Sequence` | generic in/out sequences |

Unknown commands return `0xFF` (DAP_Invalid), matching the spec.

The SWD line layer (`swd_ll.c`) implements one transfer as **two** `fpga_swd_feed`
phases, because the FPGA's feed→sample model can't branch mid-feed:

- **Phase 1**: packet request + turnaround + 3 ACK samples → read ACK.
- **Phase 2** (branch on ACK): read 32+parity, or write 32+parity, or the
  WAIT/FAULT dummy/data phase per `DAP_SWD_Configure`.

This keeps WAIT handling correct (no spurious data phase) and matches the ARM
CMSIS-DAP `SWD_Transfer` reference, just emitting `remote_bitbang` bytes
(`d/e/f/g/O/o/c/s/r`) instead of toggling GPIO.

## Host: pyOCD probe

`embeddedci.benchpod.dap`:

- `BenchPodDAPInterface(pyocd…Interface)` — `write(cmd)` length-frames the packet
  onto the transport `RawLink`; `read()` reads one framed response. Reports
  `is_bulk=True`, `packet_size=256`, `packet_count=1` (strict request/response).
- `dap_session(transport, swclk, swdio, target=…)` — constructs
  `DAPAccessCMSISDAP(None, interface=…)` → `CMSISDAPProbe` → pyOCD `Session`.
  pyOCD's whole CMSIS-DAP stack (DAP_TransferBlock batching, posted reads,
  WAIT retries, flash loaders) rides on top **unchanged**.
- A `pyocd.probe` entry-point plugin (`benchpod:<connection>`) so the `pyocd`
  CLI works too.

Pytest usage:

```python
def test_flash(benchpod):
    benchpod.flash_pyocd(firmware="fw.elf", target="stm32h5",
                         swclk=11, swdio=12, nreset=True)
```

GitHub Actions is unchanged: the existing OIDC → cloud-tunnel path carries the
framed packets; CI just `pip install 'embeddedci[pyocd]'` and calls the above.

## Performance

The internet now sees **one round-trip per DAP command**, not per bit:

- `DAP_TransferBlock` writes ~250 words (≈1 KB) of flash in **one** packet →
  one RTT instead of thousands.
- Writes have no return data, so the host can keep several block packets in
  flight (future: raise `packet_count`) to hide RTT almost entirely.

The remaining floor is **local, on the pod**: the FPGA is a 1 MHz-SPI
remote_bitbang decoder (~8 µs/byte, ~1.6 ms per 32-bit transfer for the two
feed phases). That's unchanged by this work and is bench-local. To go faster you
raise the RP↔FPGA SPI baud or add a "shift N bits" FPGA op — a separate,
optional optimization that does **not** affect internet latency.

## What this is NOT

- SWD/DAP is ARM-Cortex (CoreSight) only. ESP32 (Xtensa / RISC-V) and RISC-V
  parts need a different transport — out of scope here.
- No gateware change, no server change. Firmware + host library only.
