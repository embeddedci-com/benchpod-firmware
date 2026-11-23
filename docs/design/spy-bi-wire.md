# Spy-Bi-Wire (MSP430): spec and groundwork

Status: groundwork, 2026-11-23. Nothing here has run on hardware. Written from the TI documents
listed below before any code (the five existing `prog_flash` protocols were first written from memory
and needed a datasheet review afterwards). Every sequence and timing cites a section, table or source
function. Anything the documents leave unclear is marked **UNCLEAR** and collected in section 13.

Code: `stm32h563/src/prog_sbw.c` (not in the build yet), host test `stm32h563/test/test_prog_sbw.c`.
Timing decision and the bench measurement: section 14 on.

## 0. Sources

| Tag | Document | Used for |
|---|---|---|
| [S320] | SLAU320AJ, "MSP430 Programming With the JTAG Interface", July 2010, rev. May 2021 | SBW slots, JTAG instructions, flows, lock, FRAM, device tables |
| [S445] | SLAU445I, "MSP430FR4xx and MSP430FR2xx Family User's Guide", rev. March 2019 | SYS: JTAG mailbox, lock signatures, SYSCFG0, pin notes |
| [S367] | SLAU367P, "MSP430FR58xx, FR59xx, FR6xx Family User's Guide", rev. April 2020 | same for FR5xxx/FR6xxx: lock with password, FRCTL |
| [DS2433] | MSP430FR2433 datasheet SLASE59F, Table 5-24 and Table 6-6 | SBW timing, FR2xx/FR4xx family |
| [DS4133] | MSP430FR4133 datasheet SLAS865F, section 8.12.10.1 | same |
| [DS5994] | MSP430FR5994 datasheet SLASE54D, section 8.12.13.1 | SBW timing, FR5xxx/FR6xxx |
| [DSG2553] | MSP430G2553 datasheet SLAS735J, "JTAG and Spy-Bi-Wire Interface" | SBW timing, F2xx (flash) |
| [DS5529] | MSP430F5529 datasheet SLAS590P, section 8.49 | SBW timing, F5xx (flash) |
| [REP] | TI's reference source for [S320], `slau320.zip`, folder `Replicator430FR` | exact delays and shift sequences. Cited as `file:function` |

All were downloaded from ti.com on 2026-11-23 (`ti.com/lit/ug/slau320aj`, `ti.com/lit/zip/slau320`,
`ti.com/lit/ds/symlink/<part>`). [S320] defers the actual code to [REP] ("See the reference functions
... in the MSP430 Replicator project"), so [REP] is part of the specification where [S320] shows
only a figure. Where [S320] and a datasheet disagree, the stricter value is the design value and the
difference is listed in section 13.

## 1. Scope

- Pins: **SBWTCK** = the chip's TEST pin, **SBWTDIO** = its RST/NMI pin [S320 2.2.1.3]. The pod's own
  reset pin is not used. In `prog_flash` terms: `clk` = TEST, `data` = RST/SBWTDIO.
- Devices: every MSP430 with a TEST pin and SBW. First target: FRAM parts with CPUXv2 (FR2xx, FR4xx,
  FR5xxx, FR6xxx; JTAG IDs 0x98, 0x99, 0x91, section 8). Flash parts (F1xx/F2xx/F4xx, JTAG ID 0x89 and
  F5xx/F6xx) are listed in section 12 and are not part of the first code.
- 4-wire JTAG on the same chips is not covered (it needs four pins; the pod has `jtag_seq.v` for
  generic JTAG already).

## 2. Electrical and board notes

- TEST has an internal pulldown of 20 to 50 kohm (FR2433/FR4133/FR5994) or 25 to 90 kohm (G2553)
  [DS2433 Table 5-24, DSG2553]. The pin is low in normal operation and "SBW and any JTAG access is
  disabled while TEST is low" [S320 2.3.1.1]. Plugging in the programmer can produce stray rising
  edges that leave the entry logic confused: drive TEST low before powering the chip and repeat the
  entry two to three times [S320 2.3.1.1 note, REP `MAX_ENTRY_TRY` = 4].
- RST/NMI normally has a pullup (47 kohm or internal) and a small capacitor. The capacitor on RST
  "should not exceed 1.1 nF" for SBW with TI tools on FR2xx/FR4xx [S445 SYS chapter, "Connection of
  Unused Pins" Table 1-4, note (2)] and 2.2 nF on FR5xxx/FR6xxx [S367, same table, note]. A larger capacitor slows SBWTDIO and breaks the slot timing.
  UNCLEAR: the pod's strength on RST (10 kohm pullup always on while a session runs, see
  `prog_wire.c`) against that capacitor has not been checked.
- SBWTDIO is bidirectional and the chip has a bus keeper on it [S320 2.2.3.4]. The pod must release
  it in the TDO slot (section 5).
- I/O voltage: the pod's LA voltage must equal the chip's VCC (the SBW pins are VCC-level). Supply
  stays with the DUT or the pod's target power rail; this protocol never changes it.

## 3. Timing limits

From the datasheets, "over recommended operating conditions":

| Parameter | FR2433 / FR4133 | FR5994 | G2553 | F5529 | Source |
|---|---|---|---|---|---|
| fSBW, input frequency | 0 to 10 MHz | 0 to 10 MHz | 0 to 20 MHz | 0 to 20 MHz | T5-24, 8.12.13.1, JTAG and SBW table, 8.49 |
| tSBW,Low, SBWTCK low pulse | **0.028 to 15 us** | 0.04 to 15 us | 0.025 to 15 us | 0.025 to 15 us | same |
| tSU,SBWTDIO (before falling edge, TMS and TDI slot) | 4 ns | not given | not given | not given | [DS2433 T5-24] |
| tHD,SBWTDIO (after rising edge, TMS and TDI slot) | **19 ns** | not given | not given | not given | [DS2433 T5-24] |
| tValid,SBWTDIO (chip drives TDO, after falling edge, TDO slot) | max **31 ns** | not given | not given | not given | [DS2433 T5-24] |
| tSBW,En, TEST high to first clock edge accepted | **110 us** | 110 us | 1 us | 1 us | same, footnote (1) |
| tSBW,Ret, return to normal operation after TEST low | 15 to 100 us | 15 to 100 us | 15 to 100 us | 15 to 100 us | same |

From [S320]:

- "The low phase of the clock signal supplied on SBWTCK must not be longer than **7 us**. If the low
  phase is longer, the SBW logic is deactivated, and it must be activated again" [S320 2.2.3.4 note].
  This is stricter than the datasheets' 15 us. The design limit is 7 us (section 13, item 1).
- Exit SBW (and 4-wire JTAG) by holding TEST low for **more than 100 us** [S320 2.3.1.1]. This
  agrees with tSBW,Ret max 100 us. [REP] waits 15 ms (`StopJtag`).
- The datasheet timing diagram also names tEN,SBWTDIO (the chip's time to start driving) and
  tSBW,High but gives no value for either [DS2433 Figure after Table 5-24]. UNCLEAR (item 2).
- TI's own driver spends about 280 ns per half slot: "NOP 5 cycles (delay at 18 MHz)" between the
  steps of each macro [S320 2.2.3.2 to 2.2.3.4]. It is an example, not a limit.

## 4. Entry

SBW is off while TEST is low. To select it: TEST goes high; after tSBW,En the SBWTDIO line is held
high and "the first clock is applied on SBWTCK. After this clock, the normal SBW timings are applied
starting with the TMS slot" [S320 2.3.1.1]. Holding SBWTDIO low at that first clock selects the 4-wire
JTAG mode instead. While SBW is active the chip's reset is held inactive (RST/NMI is the data line).

[S320 Figure 2-13] shows four cases (SBW or 4-wire, RST high or low at entry) as waveforms without
numbers. The numbers come from [REP `JTAGfunc430FR.c`]:

`EntrySequences_RstHigh_SBW` (RST high: the device starts running user code and may be in a state
that blocks JTAG; used for normal entry):

| Step | TEST | RST | Wait after |
|---|---|---|---|
| 1 | low | (as is) | 4 ms ("reset TEST logic") |
| 2 | low | high | none |
| 3 | high | high | 20 ms ("activate TEST logic") |
| 4 | high | high | 60 us |
| 5 (the first clock, "phase 2") | low | high | 1 us |
| 6 ("phase 4") | high | high | 60 us, then 5 ms |

`EntrySequences_RstLow_SBW` (RST held low: the boot code stops, user code does not run; used for the
JTAG mailbox erase and to recover chips that disable JTAG from user code):

| Step | TEST | RST | Wait after |
|---|---|---|---|
| 1 | low | (as is) | 1 ms |
| 2 | low | low | 50 ms |
| 3 | high | low | 100 ms |
| 4 | high | high | 40 us |
| 5 | low | high | 1 us |
| 6 | high | high | 40 us, then 5 ms |

[REP] disables interrupts around steps 5 and 6 (`_DINT`) so the 1 us low is exact. The first clock
(step 5) is a TEST low pulse of about 1 us with SBWTDIO high.

After entry the TAP is in an unknown state: send `ResetTAP` (section 7.1), then read the JTAG ID.
If no valid ID comes back, release TEST (low for at least 15 ms [REP `StopJtag`]) and repeat, up to
4 times [REP `GetCoreID`, `MAX_ENTRY_TRY`]. After four failures TI tries the "magic pattern" (hold
the chip in reset, send 0xA55A through the JTAG mailbox, section 10); that variant exists in [REP]
only for LPMx.5 recovery.

UNCLEAR (item 3): no document gives a tolerance for the 1 us first clock. The datasheets' tSBW,Low
(28 ns to 15 us) is the only low-pulse limit and probably covers it.

## 5. The SBW bit: three slots

One JTAG clock (TCK) is three SBWTCK cycles, the **TMS slot**, the **TDI slot** and the **TDO slot**
[S320 2.2.3.1, Figure 2-8, Figure 2-10]. SBWTCK idles high between slots. The chip captures TMS and
TDI on the **falling edge** of SBWTCK in their slots; the internal TCK is generated in the TDI slot
("the internal signal TCK is generated automatically in every TDI_SLOT", [S320 2.3.1.2]). TDO is the
chip's output in the TDO slot.

| Slot | Pod does | Chip does | Edges (every slot: SBWTCK high, low phase, high) |
|---|---|---|---|
| TMS | drives SBWTDIO = TMS | captures TMS at the falling edge | set the level while SBWTCK is high (at least tSU = 4 ns before the fall), fall, hold the level at least tHD = 19 ns after the rise, rise |
| TDI | drives SBWTDIO = TDI | captures TDI at the falling edge (and TCK is generated) | same |
| TDO | releases SBWTDIO after the rising edge of the TDI slot; reads it; drives it again after the rising edge of the TDO slot | drives TDO "during the low time of the SBWTCK cycle" only, valid at most 31 ns after the fall; releases at the rise | release, fall, **read at the end of the low phase**, rise, drive again |

[S320 2.2.3.4]: "The master should release control of the SBWTDIO line based off of the rising edge of
SBWTCK of the TDI cycle ... The next falling edge of SBWTCK triggers the slave to start driving the
bus. The slave only drives the SBWTDIO line during the low time ... The master should not enable its
drivers until the slave has released the SBWTDIO line. Therefore, the master could use the rising edge
of the SBWTCK signal as a trigger point to enable its driver." So the line is never driven by both:
the pod owns it in the TMS and TDI slots and while SBWTCK is high; the chip owns it only while
SBWTCK is low in the TDO slot.

TI's macros (all steps in order, [S320 2.2.3.2 to 2.2.3.4], source in [REP `LowLevelFunc430Xv2.h`]):

| Macro | Steps |
|---|---|
| TMSH, TDIH | SBWTDIO high; delay; SBWTCK low; delay; SBWTCK high |
| TMSL, TDIL | SBWTDIO low; delay; SBWTCK low; delay; SBWTCK high |
| TMSLDH | SBWTDIO low; delay; SBWTCK low; delay; **SBWTDIO high**; SBWTCK high |
| TDO_RD | SBWTDIO hi-Z; delay; SBWTCK low; delay; **read SBWTDIO**; delay; SBWTCK high; SBWTDIO driven again |
| TDOsbw | SBWTDIO hi-Z; delay; SBWTCK low; delay; SBWTCK high; SBWTDIO driven again (no read) |

In the macros one ends with SBWTCK high and the next starts by changing SBWTDIO, so the hold time tHD
(19 ns) is just the instruction time of TI's code (about 55 ns at 18 MHz). The chip's TDO bit read in the TDO slot of one cycle is the bit the TAP
would show at that TCK, which makes the DR readout one bit "earlier" than TDI: a BYPASS register
returns the TDI stream delayed by one cycle (0xAAAA in, 0x5555 out, [S320 2.4.3]). The TDO slot of a
cycle is read before the TAP applies that cycle's TMS/TDI, as in 4-wire JTAG ("data on TDI and data
on TDO are aligned", [S320 2.2.3.1]); [REP `AllShifts`] reads TDO in every shift cycle
(`TMSx_TDIx_TDOrd`) and calls the last shift cycle's TMS high.

### 5.1 TCLK in SBW

The CPU clock (TCLK) is the **TDI value while the TAP is in Run-Test/Idle** [S320 2.2.2.1.5, 2.2.3.5].
In SBW, "the generation of a complete TCLK clock cycle requires two TDI slots, one of which sets the
TCLK signal and one of which clears it. In each case, the SBWTDIO signal must be set low in the TMS
slot to keep the TAP controller from leaving the Run-Test/Idle state" [S320 2.2.3.5.1, Figure 2-11].
The TDI/TCLK latch follows its input in Run-Test/Idle, so SBWTDIO must hold TCLK's current level
while the TMS slot's low phase runs, otherwise a TCLK edge appears in the TMS slot: that is what TMSLDH
does ("brings it back high after the falling edge before the TMS slot ends"). From [REP
`LowLevelFunc430Xv2.c`]:

| Call | TMS slot | TDI slot | After |
|---|---|---|---|
| `SetTCLK_sbw` (TCLK was 1) | TMSLDH | SBWTDIO high (TDIH) | TCLK = 1 |
| `SetTCLK_sbw` (TCLK was 0) | TMSL | TDIH | TCLK = 1 |
| `ClrTCLK_sbw` (TCLK was 1) | TMSLDH | SBWTDIO low (TDIL) | TCLK = 0 |
| `ClrTCLK_sbw` (TCLK was 0) | TMSL | TDIL | TCLK = 0 |

The TDO slot follows as TDOsbw (no read). Every IR or DR scan saves TCLK first and restores it in
its last cycle (the TDI value of the final "back to Run-Test/Idle" cycle, `TCLK_saved`) so the scan
does not disturb TCLK ([REP `AllShifts`], [S320 Figure 2-3]).

UNCLEAR (item 4): [S320] describes the latch only in words and Figure 2-9. The exact moment TCLK
changes (TDI slot falling edge, or following SBWTDIO in the TMS slot) is inferred from Figure 2-11 and
the macro structure.

## 6. Tools the protocol needs from the bit layer

Everything above reduces to these primitives, which `prog_sbw.c` keeps behind a small interface so a
gateware transport can replace them (sections 15 and 16):

1. `cycle(tms, tdi, tms_dh, read)`: one JTAG clock (3 slots); `tms_dh` selects TMSLDH; `read`
   returns the TDO bit.
2. `pins(test, rst)`: both pins driven to a level (entry and exit; waits between steps).
3. `release_rst()`: RST hi-Z (pullup).
4. `wait_us(n)`.

## 7. JTAG access over the slots

### 7.1 TAP navigation (counts in SBW cycles)

A "cycle" is one JTAG clock with TMS and TDI given. TDI is `TCLK_saved` (the TDI level to keep TCLK)
whenever the TAP leaves or enters Run-Test/Idle; elsewhere TDI is don't-care and [REP] sends 1.

- **ResetTAP** [REP `ResetTAP`, SBW branch; [S320 2.3.1.2]]: 6 cycles TMS=1 (TDI=1) reach Test-Logic-
  Reset, then 1 cycle TMS=0 reaches Run-Test/Idle: 7 cycles. A fuse check (toggling TMS twice) is **not**
  needed on FRAM and 5xx/6xx parts: they have a software lock instead of a fuse [S320 2.3.1.2 note].
- **IR_Shift(8 bits)** from Run-Test/Idle [REP `IR_Shift`]: TMS = 1, 1, 0, 0 (Select-DR, Select-IR,
  Capture-IR, Shift-IR; the first cycle's TDI = TCLK_saved), then 8 cycles with TMS=0 except the
  last with TMS=1 (Exit1-IR), then TMS=1 (Update-IR) and TMS=0 (Run-Test/Idle, TDI = TCLK_saved): 14
  cycles. The instruction goes in LSB first of the value in [S320 Table 2-5] (the [REP] constants
  are those values bit-reversed because `AllShifts` sends MSB first; [S320 2.2.4]: "All instructions ...
  are transferred LSB first"). TDO returns the **JTAG ID** during every IR shift, "regardless of the
  8-bit instruction", MSB first [S320 2.2.2.1.1]. The new instruction takes effect at Update-IR
  [S320 2.2.4 note].
- **DR_Shift16**: TMS = 1, 0, 0 (Select-DR, Capture-DR, Shift-DR), 16 cycles (last TMS=1), TMS=1, TMS=0:
  21 cycles. MSB first both ways ([S320 2.2.2.1.2], Figure 2-4: 0x158B in, 0x55AA out).
- **DR_Shift20**: 25 cycles, TDI = the 20-bit value MSB first (bit 19 first, [S320 Figure 2-5]); TDO
  comes out as bits 15 to 0 first and bits 19 to 16 last, so the host rebuilds `((raw << 16) + (raw >> 4))
  & 0xFFFFF` ([S320 2.2.2.1.3] note, [REP `AllShifts`]). UNCLEAR (item 5): the TDI side is shown only
  by example (Figure 2-5, input 0x12568).
- **TCLK edge** (`SetTCLK`, `ClrTCLK`): 1 cycle (section 5.1).

### 7.2 Instructions used [S320 Table 2-5]

| Name | Value (LSB first) | Name | Value |
|---|---|---|---|
| IR_ADDR_16BIT | 0x83 | IR_CNTRL_SIG_16BIT | 0x13 |
| IR_ADDR_CAPTURE | 0x84 | IR_CNTRL_SIG_CAPTURE | 0x14 |
| IR_DATA_TO_ADDR | 0x85 | IR_CNTRL_SIG_RELEASE | 0x15 |
| IR_DATA_16BIT | 0x41 | IR_JMB_EXCHANGE | 0x61 |
| IR_DATA_QUICK | 0x43 | IR_BYPASS | 0xFF |

Not in [S320 Table 2-5] but used by [REP] for CPUXv2 (original values in the `JTAGfunc430FR.h`
comments): IR_DATA_CAPTURE 0x42, IR_COREIP_ID 0x17, IR_DEVICE_ID 0x87, IR_TEST_REG 0x2A,
IR_TEST_3V_REG 0x2F. "Do not write any unlisted values to the JTAG instruction register" [S320 2.2.4
note]: these five are not in the table; [S320 2.3.2.2.1] and Figure 2-15 name `IR_COREIP_ID` and
`IR_DEVICE_ID` (UNCLEAR item 6: the table of values omits them).

### 7.3 JTAG control signal register (CPUXv2) [S320 Table 2-7]

| Bit | Name | Meaning |
|---|---|---|
| 0 | R/W | 1 = read |
| 3 | WAIT | read only: CPU clock stopped |
| 4 | BYTE | 1 = byte access |
| 7 | INSTR_LOAD | read only: instruction fetch state |
| 8 | CPUSUSP | write 1 to suspend; read: 1 = pipeline empty |
| 9 | TCE0 | read: 1 = CPU synchronized to JTAG |
| 10 | TCE1 | write 1: JTAG takes control of the CPU |
| 11 | POR | write 1: power-on reset |
| 12, 13 | RELEASE_LBYTE | which low-byte bits the CPU controls |

Words used by [REP] and [S320 2.3.2.2.1, 2.3.3.1, 2.3.3.2]: 0x1501 (TCE1 + CPUSUSP + read: take
control), 0x0501 (read, CPUSUSP), 0x0500 (write), 0x0511 (byte read), 0x0510 (byte write), 0x0C01
(CPUSUSP released, POR), 0x0401 (POR released), 0x1400 (release low byte).

## 8. JTAG ID and device identification

- The JTAG ID is the byte returned by any IR shift [S320 2.2.2.1.1]. CPUXv2 values [S320 Table 2-16,
  2-15 and the notes under Table 2-14]: **0x98** FR2xx and FR4xx, **0x99** FR5041/5043 and FR59xx,
  FR60xx, FR68xx, FR69xx (and some F67xx), **0x91** FR57xx and many F5xx/F6xx, **0x89** F1xx, F2xx, F4xx
  and some F5xx/F6xx. [REP `GetCoreID`] accepts 0x91, 0x99, 0x98 as valid.
- A line with nothing on it reads all ones (the pod's pullup) or all zeros: "no answer".
- `IR_COREIP_ID` + DR16 returns the core ID (zero = not valid); `IR_DEVICE_ID` + DR20 returns a
  pointer to the device descriptor [S320 2.3.2.2.1 `GetCoreipIdXv2`; [REP] `GetCoreipIdXv2`].
  The device ID word is at **pointer + 4** [REP `GetDevice_430Xv2`]; the datasheet/TLV shows it at
  0x1A04 (low byte) and 0x1A05 (high byte) for all FRxx in [S320 Table 2-16], e.g. FR2433 = 0x40,
  0x82. UNCLEAR (item 7): [S320] does not say the pointer is always 0x1A00.
- The pod reports the ID as `ctx->id = { JTAG ID, device ID low, device ID high }`.

## 9. Taking control: sync, POR, halt

[S320 2.3.2.2.1 Figure 2-15] and [REP `GetDevice_430Xv2`, `SyncJtag_AssertPor`, `ExecutePOR_430Xv2`]:

1. Entry (section 4), ResetTAP, JTAG ID check (up to 4 entries).
2. Lock check (section 10). A locked device stops here.
3. Core ID and device pointer (section 8). Core ID 0 is an error.
4. **Sync**: `IR_CNTRL_SIG_16BIT`, `DR16(0x1501)`; check the JTAG ID again; poll `IR_CNTRL_SIG_CAPTURE`,
   `DR16(0)` until bit 9 (TCE0) is set, at most 50 polls ([REP]); else error.
5. **POR, FRAM variant** (the CPU must end in the "Full-Emulation-State", checked as
   `DR16(0) & 0x0301`) [S320 2.3.2.2.3; REP `ExecutePOR_430Xv2`]:
   `ClrTCLK, SetTCLK` (empty the pipe); `IR_CNTRL_SIG_16BIT`; `DR16(0x0C01)`; `DR16(0x0401)`;
   `IR_DATA_16BIT`; `ClrTCLK SetTCLK ClrTCLK SetTCLK`; `DR16(0x0004)` (safe PC); `ClrTCLK SetTCLK`;
   `IR_DATA_CAPTURE`; `ClrTCLK SetTCLK ClrTCLK SetTCLK`; `IR_CNTRL_SIG_16BIT`; `DR16(0x0501)`; `ClrTCLK
   SetTCLK`.
6. **Hold the watchdog**: write 0x5A80 to WDTCTL: **0x01CC** for JTAG ID 0x98, **0x015C** otherwise
   [REP `ExecutePOR_430Xv2`]. For JTAG ID 0x91 and 0x99 also write 0x3FFF to 0x0006 and 0x0008
   ("initialize test memory ... MAB is +2 after sync", [S320 2.3.2.2.3 FRAM], [REP]).
7. Re-check `DR16(0) & 0x0301` (Full-Emulation-State).

[S320]'s text of the FRAM POR flow is partly lost in the PDF text (flow chart); the order above is
[REP]'s, which the text agrees with where it is readable.

For the CPU "halt": CPUXv2 has no HaltCPU flow like the 1xx/2xx/4xx devices. The CPU is kept
in the Full-Emulation-State (CPUSUSP = 1, pipeline empty) and every memory access sets the control
register around the access (section 11). No user code runs between accesses.

## 10. JTAG lock, fuse and mailbox

- **Detect**: `IR_CNTRL_SIG_CAPTURE`, `DR16(0xAAAA)`; the device is protected if TDO returns 0x5555
  (BYPASS: TDI delayed one clock). Retry up to three times, "first trial could be wrong" [S320 2.4.3;
  REP `IsLockKeyProgrammed`]. Only BYPASS and `IR_JMB_EXCHANGE` work on a protected device [S445
  1.11.1, S367 1.13.1]. The JTAG ID is still returned by every IR shift (so "locked" is distinct from "no
  answer").
- **Lock signature**: JTAG Signature 1 at 0xFF80 and 2 at 0xFF82 (FR2xx/FR4xx and FR5xxx/FR6xxx). Takes
  effect at the next BOR. The rules differ by document:
  - [S445 1.11.1] (FR2xx/FR4xx): locked if both signatures are neither 0x0000 nor 0xFFFF; unlock only by
    BSL, or by the JTAG mailbox `User_Code_Erase`.
  - [S367 1.13.1, 1.13.2] (FR5xxx/FR6xxx): 0x5555 in both = locked without password; 0xAAAA in the first
    and a length in the second = locked with password (words at 0xFF88).
  - [S320 Table 2-11]: FR2xx/FR4xx "writing the pattern 0x55555555 at 0xFF80" (REP `ProgramLockKey`
    writes 0x5555, 0x5555), with a note that any pattern other than 0x00000000 locks. UNCLEAR (item 8).
  Locking is permanent unless unlocked as below; the pod never writes a lock signature.
- **Unlock/mass erase by mailbox** (FR2xx/FR4xx: the only JTAG unlock; FR5xxx/FR6xxx: full erase)
  [S320 2.4.4.2 and Table 2-11; REP `EraseFRAMViaBootCode_430Xv2`]:
  1. TEST low, RST low, 200 ms. RST-low entry sequence (section 4). ResetTAP.
  2. `IR_JMB_EXCHANGE`; poll `DR16(0)` until IN0RDY (bit 0), at most 3000 polls; `DR16(0x0011)` (32-bit mode,
     INREQ); `DR16(0xA55A)` (stop after boot code, "magic pattern"); `DR16(0x1A1A)` (`User_Code_Erase`).
  3. TEST low, RST high, 200 ms. RST-high entry. ResetTAP. Wait 60 ms ("until erase is done").
  4. `SyncJtag_AssertPor` (section 9). Writes to the signatures are gone: the erase sets main FRAM to 0xFF
     and clears the signatures (UNCLEAR item 9: [S320] does not state how long the boot code erase takes;
     [REP] waits 200 + 60 ms).
  5. Mailbox bits [REP `JTAGfunc430FR.h`]: control word INREQ = 0x0001, OUTREQ = 0x0004, JMB32B = 0x0010;
     status IN0RDY = 0x0001, OUT1RDY = 0x0008.
  The password unlock for FR5xxx/FR6xxx devices (`0x1E1E`, 1.2 s window, [S320 2.4.4.1]) is not
  implemented. Total erase of an IPE-protected device uses 0x1B1B [S320 2.4.6]: also not implemented.
- A BOR through JTAG: `IR_TEST_REG`, `DR16(0x0200)`, wait 5 ms: the chip leaves JTAG control [REP
  `ReleaseDevice_430Xv2`]. The signatures take effect after it.

## 11. FRAM: write protection, read, write, erase

FRAM "behaves like standard SRAM"; reads are destructive but written back by the controller; "erases
are not required before writing" [S320 2.3.8, 2.3.8.1]. "If a flash-style erase needs to be executed,
a dummy erase can be done by using the write function to write 0xFF to all locations" (2.3.8.2).

**Write protection must be off first** [S320 2.4.5, 2.4.7; REP `DisableMpu_430Xv2`,
`DisableFramWprod_430Xv2`]:
- JTAG ID 0x98 (FR2xx/FR4xx): SYSCFG0 at **0x0160**: read; clear bit 0 (PFWP, main FRAM) and bit 1 (DFWP, info
  FRAM) and write the password 0xA5 in the high byte ([S445 1.16.1 SYSCFG0, Table 1-24]; "both bits
  default to 1" after a PUC and writes to FRAM are then protected [S445 FRAM controller chapter, write
  protection]). Read back: `& 0x3` must be 0.
- Other FRxx: MPUCTL0 at **0x05A0**: if bit 0 (MPUENA) is set and the MPU is not locked, write 0xA500. If
  MPULOCK is set the BOR dance of [S320 2.4.5] is needed (not implemented). FR59xx/FR60xx (JTAG ID 0x99) also have
  a write-protect bit 0 in the register at **0x0140** ([REP `FR59xx_FR60xx_LOCKREGISTER`]; the WPROT
  bit of the FRAM controller [S367 FRAM controller chapter] may be the same feature; UNCLEAR item 10: the register name/address is
  not in [S367] text).
- A mass erase by mailbox is a BOR: protection is back after it.

**Read a word** (`ReadMem_430Xv2`, [S320 2.3.3.1]; the check `IR_CNTRL_SIG_CAPTURE, DR16(0) & 0x0301`
runs first):
`ClrTCLK; IR_CNTRL_SIG_16BIT; DR16(0x0501); IR_ADDR_16BIT; DR20(addr); IR_DATA_TO_ADDR; SetTCLK; ClrTCLK;
DR16(0) -> data; SetTCLK; ClrTCLK; SetTCLK`.

**Write a word** (`WriteMem_430Xv2`, [S320 2.3.3.2], same check first):
`ClrTCLK; IR_CNTRL_SIG_16BIT; DR16(0x0500); IR_ADDR_16BIT; DR20(addr); SetTCLK; IR_DATA_TO_ADDR; DR16(data);
ClrTCLK; IR_CNTRL_SIG_16BIT; DR16(0x0501); SetTCLK; ClrTCLK; SetTCLK`.
Byte access: 0x0511 / 0x0510 in place of the word values. The last `SetTCLK ClrTCLK SetTCLK` lets the CPU
drive the right MAB again.

**Cost** (cycles, section 7.1): one `WriteMem` = 35 (check) + 150 = **185 cycles**; one `ReadMem` = **185**.

**Faster paths** (not in the first code; all UNCLEAR until measured, items 11 and 12):
- `ReadMemQuick`: set the PC, then per word `SetTCLK; ClrTCLK; DR16(0)` = 23 cycles ([S320 2.3.3.3.1], [REP
  `ReadMemQuick_430Xv2`]). Needs `SetPC_430Xv2` (the MOVA instruction injection, [S320 2.3.2.2.2]).
- Quick write: [S320 Table 2-16] says "Data Quick Access to FRAM: READ/WRITE" but 2.3.3.3.2 says "for
  the MSP430Xv2 architecture ... there is no specific implementation of a quick write operation".
- TI's own FRAM write loads a 120-byte funclet into RAM and feeds words through the JTAG mailbox
  (`WriteFram_430Xv2`, `FramWrite.c`): per word `DR16(0)` (poll IN0RDY), `DR16(control)`, `DR16(data)` =
  63 cycles, about three times fewer than `WriteMem`. The funclet is a binary blob from [REP].

**Erase**:
- A range ("segment", the host picks the size): write 0xFFFF to every word of the range [S320 2.3.8.2].
- Whole chip: the mailbox mass erase of section 10. It also unlocks a locked FR2xx/FR4xx.
- Erase check: PSA (`VerifyPSA`, [S320 2.3.7]). Not used; the pod reads back.

## 12. Leaving, and what flash parts add

**Release** [S320 2.3.2.2.4, 2.3.2.1.6; REP `ReleaseDevice_430Xv2` with `V_RESET`]:
`IR_CNTRL_SIG_16BIT; DR16(0x0C01); DR16(0x0401); IR_CNTRL_SIG_RELEASE`. The CPU starts at the reset
vector. A true power-on or a BOR (`IR_TEST_REG`/`DR16(0x0200)`, 5 ms) boots through the boot code and
calibrates the chip; a JTAG POR does not [S320 2.3.2.2.4]. The pod then leaves SBW: TEST low for more than
100 us (tSBW,Ret) and releases SBWTDIO. **TEST must end LOW**: TEST high enables SBW again and disables the RST
function [S320 2.3.1.1]. "Not recommended to release the device ... during an erase-program-verify cycle"
[S320 2.3.2.2.5]: leave only after the verify. Keeping the chip in reset instead: release as above, then drive
RST low before TEST goes low. (`prog_wire_close` drives clk high on close: it would re-enable SBW on the chip.
The pod's wiring layer needs a way to leave the clock low; see section 17.)

**What F1xx/F2xx/F4xx and F5xx/F6xx flash parts would add** (JTAG ID 0x89 and some 0x91/0x98/0x99 F5xx):
- Xv1 parts (0x89): different CPU control (HaltCPU/ReleaseCPU flows with `0x2409`/`0x2401`, [S320 2.3.2.1]),
  different control register (Table 2-6), JTAG fuse check at entry [S320 2.3.1.2, ResetTAP with the SBW
  side effect: two more dummy TCK], programming through the flash controller registers (FCTL1 to FCTL3).
- **Flash timing from TCLK**: F1xx/F2xx/F4xx need "a TCLK frequency of 350 kHz +/- 100 kHz while the erase
  or programming cycle is being executed" [S320 2.3.4.1, 2.3.5.1, 2.3.5.1.2]. Over SBW TCLK comes from TDI
  slots: "TCLK strobes" put many TCLK cycles in one TDI slot [S320 2.2.3.5.2, Figure 2-12; REP
  Replicator430 `TCLKstrobes`]; "not applicable for the F5xx or F6xx devices" (internal MODOSC).
  This needs sub-microsecond control of SBWTDIO inside a TDI slot's low phase: a gateware job (section 15).
- F5xx/F6xx: block write through the flash controller with the CPU executing a RAM funclet [S320 2.3.4.2],
  JTAG lock key at 0x17FC in the BSL memory (SYSBSLPE must be cleared first, [S320 2.4.2.1]), erase
  via the flash controller. No high voltage needed.
- F1xx/F2xx/F4xx fuse blowing needs 6 to 7 V on TEST [DSG2553 JTAG Fuse table]: out of scope, never.

## 13. Unclear points and conflicts

1. **tSBW,Low maximum: 7 us [S320 2.2.3.4] against 15 us [every datasheet above].** Design limit 7 us; 15 us
   is what the silicon is specified for. The bench must find out which (section 18) before the 7 us is
   relaxed.
2. tEN,SBWTDIO (time for the chip to start driving TDO after the falling edge) and tSBW,High are named in the
   datasheet figure but have no value. Only tValid (31 ns max) is given.
3. Tolerance of the 1 us first-clock low pulse in the entry sequence (section 4).
4. When exactly the TDI/TCLK latch changes (section 5.1).
5. DR_Shift20 TDI bit order is shown only by an example.
6. IR values for `IR_COREIP_ID`, `IR_DEVICE_ID`, `IR_TEST_REG`, `IR_DATA_CAPTURE` are not in [S320 Table 2-5];
   they are in [REP] only.
7. Whether the device-ID pointer is always 0x1A00.
8. FR2xx/FR4xx lock signature rule: three documents, three rules (section 10).
9. Boot-code erase time.
10. Address/name of the FR59xx/FR60xx write-protect register.
11. Quick write on FRAM: capability table against the implementation note.
12. TCLK limits against FRAM access: [DS5994] gives 4 MHz and 100 ns per TCLK phase with FRAM access. A TCLK
    cycle over SBW is at least two cycles of three slots, far slower, so this limit is not a concern; it was
    not checked for the other families.
13. Whether the bus keeper still holds the TDO level after the chip releases SBWTDIO at the TDO slot's
    rising edge ([S320 2.2.3.4] speaks of the keeper only for the master's release). Matters for
    sampling after the rising edge instead of before it (section 14).
