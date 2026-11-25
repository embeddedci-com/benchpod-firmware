# Spy-Bi-Wire (MSP430): spec and groundwork

Status: groundwork, 2026-11-23. Nothing here has run on hardware. Written from the TI documents
listed below before any code (the five existing `prog_flash` protocols were first written from memory
and needed a datasheet review afterwards). Every sequence and timing cites a section, table or source
function. Anything the documents leave unclear is marked **UNCLEAR** and collected in section 13.

Status 2026-11-25: integrated into the firmware build (option a of section 17, inside the `xip_prog` image, 15,916 of 20,480 bytes
after), proto `sbw`, id 5, both prog_wire gaps closed (section 17), gateware v52+ required. Host tests green; **still not run
on hardware** (bench plan: section 18).

Code: `stm32h563/src/prog_sbw.c`, host test `stm32h563/test/test_prog_sbw.c`.
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

## 14. The pod's bit-bang byte mode against SBW

What the byte mode is (read from `ice40/src/swd_engine.v`, `signal_engine.c` `fpga_swd_feed`,
`prog_wire.c`; none of it measured yet, `prog_wire.h`: "edge timing not measured yet"):

- `fpga_swd_arm(clk, data)` puts the SWD engine in remote_bitbang mode. Each byte of a `SWD_FEED` is one
  command: `d e f g` set (SWCLK, SWDIO) = (0,0) (0,1) (1,0) (1,1) **both at once**; `O`/`o` drive or release
  SWDIO; `c` samples SWDIO (2-flop synchronised) into the reply.
- A command takes effect when its byte has been received: **one SPI byte time after the previous one**. The
  SPI link is 250 MHz / 128 = 1.95 MHz at boot and 250 MHz / 64 = 3.9 MHz once the gateware is v52 or newer
  (`signal_engine.c` `spi_link_for_version`; the pod runs v61). One byte is 8 SPI clocks: **4.1 us at
  the boot rate, 2.05 us at the fast rate**, if the STM32 SPI master leaves no gap between bytes of a
  burst (UNCLEAR, to measure; the H5's master inter-data idleness is not set in `signal_engine.c`).
- A feed is cut into bursts: `fpga_swd_feed` sends up to 256 bytes per SPI transaction, stops before a
  `c` that would overflow the caller's reply space, and `prog_wire.c` flushes at 192 bytes
  (`FEED_MAX`), or when a clock-high `set()` finds 176 queued (`FEED_SOFT`), or before a `c` when
  `nsamples + nfeed >= 128`. **Between two bursts the pins sit still for tens of microseconds** (CS
  release, DMA setup, reading the reply). The old rule "keep the clock high at every flush" covers the
  first two; **the `c` guard flushes before the `c`, that is, with SBWTCK low in a TDO slot.** The SBW bit
  layer must therefore do its own accounting and flush at cycle boundaries (`prog_sbw.c` does: it flushes
  before a cycle when more than 96 bytes are queued).

### 14.1 One SBW cycle in bytes

| Slot | Bytes | Low phase (fall to rise) |
|---|---|---|
| TMS | `set(1,tms)` setup, `set(0,tms)` fall, `set(1,tms)` rise | 1 byte time |
| TMS, TMSLDH shape | `set(1,0)`, `set(0,0)`, `set(0,1)`, `set(1,1)` | 2 byte times |
| TDI | same as TMS | 1 byte time |
| TDO, with read | `o`, `set(0,x)` fall, `c`, `set(1,x)` rise, `O` | **2 byte times** (the `c` is a byte of its own) |
| TDO, no read | `o`, fall, rise, `O` | 1 byte time |

A byte holds clock and data together, so a data change needs its own byte *after* the rising edge (hold
19 ns) and *before* the falling edge (setup 4 ns): that is why each of the TMS and TDI slots is three
bytes and not two. Data and clock never change in the same byte except at a fall in the TDO slot (data
released, level irrelevant).

| Per cycle | Boot link (4.1 us/byte) | Fast link (2.05 us/byte) |
|---|---|---|
| bytes | 11 with a read, 10 without | same |
| time | 45 / 41 us | **22.5 / 20.5 us** |
| longest low phase (TDO with read, TMSLDH) | **8.2 us** | **4.1 us** |
| JTAG cycles per second | about 22k | about 44k |

Against the limits (section 3): at the fast link the longest low phase, 4.1 us, is under both the 7 us
of [S320] and the 15 us of the datasheets, with 2.9 us (7 us) of room for jitter. At the boot link it is 8.2 us:
over [S320]'s 7 us, under the datasheets' 15 us. TMS and TDI slots, and the entry first clock (2.05 us, TI uses
1 us), are 1 byte time. The minimum, 28 to 40 ns, is no problem.

Sample timing: the `c` byte lands one byte time after the fall (2 us); the chip has driven TDO since 31 ns
after the fall and drives until the rise, so the sample is inside the valid window by a wide margin.
Sampling after the rise would depend on the bus keeper (item 13) and is not used.

### 14.2 Cost of the flows (fast link, 22.5 us per cycle; the boot link doubles it)

| Operation | Cycles | Time |
|---|---|---|
| `IR_Shift` | 14 | 0.3 ms |
| `DR_Shift16` | 21 | 0.5 ms |
| `ReadMem` or `WriteMem` (one word) | 185 | 4.2 ms |
| 16 KB FR2433: write and read back | 16 384 words x 185 | about 70 s |
| 256 KB FR5994 | 262 144 words x 185 | about 18 min (with the mailbox funclet and quick reads: about 4 min) |
| the same with a gateware engine at 1 us per cycle (section 15) | | 3 s for 16 KB, 50 s for 256 KB |

Flush overhead (one SPI transaction and a reply read per 96 to 176 bytes) adds roughly 10 to 15 %; not
measured. Slow for large parts, fine for LaunchPad-class parts (FR2433, 16 KB).

### 14.3 Which sequences need a guaranteed short low phase

All of them: every slot is a low phase, and the limit is on every one. In order of risk: TDO slot with read and
TMSLDH (2 byte times), TMS/TDI (1), the entry first clock (1). Keeping SBWTCK high between feeds is
necessary (a gap is tens of microseconds) and the layer keeps to it, but it is **not sufficient** alone:

1. Inside a burst the byte spacing must be uniform. The SPI master, DMA and the FPGA's `spi_slave` decide
   that; a single stretched byte of 3 us would make a 2-byte low phase 5 us at the fast link, and of 5 us would
   exceed 7 us.
2. Inside a burst nothing else may stall the pod's SPI: other transactions to the FPGA from another task (LA
   capture, status polling, PSRAM streaming) would cut the burst. The prog job runs on the hw worker; whether
   that is exclusive on the SPI link is the question to measure (section 18, M2).
3. A burst cut in a low phase is unrecoverable: the chip leaves SBW after 7 us (or 15) and the rest of the
   sequence goes to a chip that is not listening. The layer then reads a wrong ID, so a failed answer must
   mean "retry from entry", not "no chip".

Whether the TCLK strobes of flash parts (section 12) are possible in this mode: no, they need edges 1 us
apart on SBWTDIO inside a TDI slot's low phase (the byte mode's best is 2 us per edge at the fast rate:
about 250 kHz period against the 250 to 450 kHz required). Flash parts need the engine.

## 15. What a gateware slot engine would add

Over the byte mode:

1. **A guaranteed low phase**: the low phase is `half` clocks of the 24 MHz engine clock, fixed by a register
   and bounded by the 7-bit field (half <= 127 is 5.3 us, under 7 us whatever the firmware does).
   No SPI jitter, no flush race.
2. **Sub-microsecond slots**: with `half = 4` (167 ns) a slot is 333 ns and a cycle 1 us (SBWTCK 3 MHz): 22x
   faster; this is what makes 256 KB parts reasonable and the mailbox or quick paths cheap.
3. **Setup and hold by construction**: the data changes half a slot after the rising edge (hold =
   `half` x 41.7 ns, at least 83 ns) and half a slot before the falling edge.
4. **Entry/exit timers in the engine** (1 us first clock, 100 us exit) so the firmware does not time them.
5. TCLK strobes for flash parts (later), and the `DAP_JTAG_Sequence`-shaped interface so OpenOCD or
   mspdebug-style host tools could drive SBW through the pod's CMSIS-DAP (they do not need it today; the pod's
   own routines do).

Sketch, next to `ice40/src/jtag_seq.v` (105 lines; this would be a similar size; NOT implemented, gateware is
held back until the SWIM engine lands in `swd_engine.v`):

```verilog
module sbw_seq (
    input  wire       clk, rst,
    input  wire       on,         // armed in SBW mode (SPI_ARM flags bit 4, v6x+)
    input  wire       arm,        // 1-cycle: (re)armed: SBWTCK high, SBWTDIO released
    input  wire [6:0] half,       // clk per SBWTCK half period, 2..127
    input  wire       dio_s,      // synchronised SBWTDIO (pad)
    input  wire [7:0] t_rd,       // queue head, as in jtag_seq
    input  wire       qready,
    output reg        take, push,
    output wire [7:0] pdata,
    output wire       busy,
    output reg        tck,        // SBWTCK (TEST): rests HIGH, unlike jtag_seq's TCK
    output reg        dio_o,      // SBWTDIO level
    output reg        dio_oe      // SBWTDIO driven
);
```

SPI_ARM: `sck_ch` = TEST, `mosi_ch` = `miso_ch` = RST/SBWTDIO (one bidirectional pad as in the I2C master's SDA),
`cs_ch` unused. Queue (SWD_FEED bytes), all fields bit 0 first:

| Token | Bytes | Meaning |
|---|---|---|
| SEQ | n in bits 5:0 with bits 7:6 = 00 (cycles, 0 = 64), flags byte, then ceil(n/8) TDI bytes | n JTAG cycles; flags bit 0 = TMS level for all of them (as `DAP_JTAG_Sequence`), bit 1 = capture TDO, bit 2 = TMSLDH slot shape (TCLK handling), bit 3 = TDI slots hold SBWTDIO as TCLK_saved |
| PINS | `0x40`, levels | SBWTCK, SBWTDIO level and drive (entry, exit, mailbox steps) |
| WAIT | `0x80`, count lo, count hi | n x 1 us with the pins as they are |

A capturing SEQ appends ceil(n/8) TDO bytes to the reply buffer (jtag_seq packing, where the last byte's TDO bits are
its top k bits). Cycle: TMS slot: SBWTCK high `half` (data setup), low `half`, rise; TDI slot the same; TDO slot:
release `half` after the rise, low `half` with the sample at the end (2-flop synchroniser, 83 ns before: still
inside the chip's drive window), rise, drive again `half` after the rise. Idle: SBWTCK high, so a queue
underrun between tokens is harmless (a burst cut in the byte mode is not). Like JTAG and I2C it fits the loop
and deep images only, not the full emulation image.

## 16. Recommendation

1. **Build the bit-bang transport first** (it is written, section 14 says the fast link is inside the limits on
   paper, and the bench decides), and **require gateware v52+ for SBW** (the fast SPI link). At the boot rate the
   TDO slot is 8.2 us: over [S320]'s 7 us.
2. Keep the SBW bit layer behind the small interface of section 6 so the engine can replace it without touching
   the JTAG, FRAM or lock code.
3. **Decide on the engine after the bench measurements** of section 18: if M1 shows the longest SBWTCK low phase
   (TDO slot with read) at most 5 us at the fast link with the pod busy, and M5 shows the real chip tolerates at
   least 2 us more than that, ship the byte mode for FR2xx/FR4xx-size parts and document the speed (70 s for 16 KB);
   build the engine when a 128 KB or larger FRAM part, a flash part, or the speed matters, or if M1 or M5 fail.
   The engine is not needed for correctness on the small parts and is needed for flash parts and big FRAM.
4. Gateware waits for the SWIM engine session (same `swd_engine.v` queue and SPI_ARM flags) and for the bench
   numbers.

## 17. Where the code lives

Measured at -Os (`arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -Os -DPROG_XIP=1`, the flags `prog_*.o` get in the
Makefile; text+rodata of the object, 2026-11-23):

| Object | Bytes |
|---|---|
| prog_updi | 3 207 |
| prog_pic | 2 755 |
| prog_c2 | 2 648 |
| prog_tpi | 1 865 |
| prog_pdi | 2 493 |
| xip_prog_mod | 161 |
| **the xip_prog image today** | **13 129** |
| **prog_sbw (FRAM parts, this change)** | **2 752** |
| image with SBW | 15 881 of the 20 KB region (4.1 KB left) |

The earlier estimate was about 8 KB for the JTAG routines; the FRAM-only code is a third of that because it
uses only the documented word access (`ReadMem`, `WriteMem`) and one mailbox erase. **It fits in the existing
`xip_prog` image**, which has room for 7 KB. What it does not include and what each would add (estimates, not
measured): quick reads and the mailbox write funclet (0.5 to 1 KB), password unlock and the MPU BOR dance (0.7 KB),
F5xx/F6xx flash parts (flash controller and a RAM funclet, 2 to 3 KB), F1xx/F2xx/F4xx (different CPU control, TCLK
strobes, 3 to 4 KB), and SWIM or cJTAG firmware later. The full MSP430 set would not fit next to the five.

Options (a prog_flash job holds one session and runs one protocol, so protocols never need to coexist in RAM):

| Option | Effect |
|---|---|
| a. **SBW FRAM into `xip_prog` now** (a `dispatch` case, the enum and metadata entry, the Makefile object list, the linker script's object list) | cheapest, 4.1 KB headroom afterwards. **Recommended for the FRAM scope.** |
| b. Third XIP image `prog2` (slot `xip_prog2`, own table, hash record, `xip_finish.py` and `check_xip_refs.py` entries, `XIP_IMG_PROG2`, dispatch by protocol id), same 20 KB overlay region | when flash parts, the funclet paths or SWIM/cJTAG firmware arrive and the 4.1 KB is gone; move SBW there at that time (the code does not change) |
| c. Internal flash | spends the code budget XIP saved |
| d. Grow the region beyond 20 KB | the region is RAM shared with the usb image, and static RAM is nearly full |

(Both gaps below are closed since 2026-11-25, see section 19.) The wiring layer had two gaps, both in `prog_wire.c`: (1) `prog_wire_close()` leaves the clock
**high**, which on an MSP430 re-enables SBW and disables its RST function (section 12): SBW needs a way to leave TEST low
(an ops hook, or `close` not touching the clock when the protocol says so); `prog_sbw_call(LEAVE)` leaves it low
and the host test checks that; (2) the `c` guard flush described in section 14.

## 18. What the bench must measure

All with the pod's LA (14 channels, observes pins whoever owns them) on SBWTCK and SBWTDIO, and the MSP430
LaunchPad's chip (cut the eZ-FET jumpers, TEST and RST wired to two pod channels, section 2), under the hardware
lock. Order:

| # | Measurement | Why | Pass |
|---|---|---|---|
| M1 | Feed 192-byte bursts that repeat the bytes of a TDO slot (`o`, fall, `c`, rise, `O`) and of plain slots, at both link rates, 100 000 cycles, with idle pod and with Ethernet and USB traffic and a DAP session running; LA at the highest rate; histogram of the SBWTCK low width and of the high width at burst boundaries | the one number the decision rests on: longest low phase, byte spacing jitter, gaps inside a burst | longest low <= 5 us (TDO with read), fast link, loaded |
| M2 | Same capture while other tasks use the SPI link to the FPGA (LA capture, status poll, PSRAM streaming, the web UI) | whether a burst can be cut by other SPI users | no burst is cut |
| M3 | `prog_sbw` against the real chip: ID read (JTAG ID 0x98 for FR2433 and device ID 0x40 0x82), lock check, 1 KB write and verify, 1000 times | functional proof of all slots at the real timing, entry sequence, tolerance of the 2 us first clock | 0 errors in 1000 runs; error pattern if not (which slot) |
| M4 | Stretch the low phase on purpose (add n extra no-op bytes between the fall and the rise) and find the width at which the chip stops answering | resolves 7 us ([S320]) against 15 us (datasheets) on the real silicon | width found; compare with M1 |
| M5 | Same as M3 with the boot link (gateware v51 or force `/128`) | confirms the "needs v52+" rule | fails or passes as predicted; decides if the boot rate is allowed |
| M6 | Sample after the rise instead of before it | bus keeper (item 13) | informational |
| M7 | `DATA_QUICK` write to FRAM, and the mailbox funclet write | the faster write paths (items 11 and 12) | informational |
| M8 | Wiring: RST pin loading (10 kohm pod pullup, LaunchPad's RST capacitor) and the pod's drive into it; TEST pulldown against the clock edges | section 2 | clean edges on the LA |

### 18.1 Bench results (2026-11-25)

MSP-EXP430FR2433 LaunchPad, J101 jumpers removed, TEST/SBWTCK on LA4, RST/SBWTDIO on LA3, target 3.3 V from the pod,
gateware v68, network link. Functional runs only (M3 style); the timing captures M1, M2, M4 and M5 are not done yet.

| Build | Run | Result |
|---|---|---|
| 81e0116 (before the rebases) | ID read | JTAG ID 0x98, device ID 0x8240 (MSP430FR2433) after the device-pointer and quick-read fixes |
| 81e0116 | full program memory (15 232 bytes at 0xC400), write + verify | 9 of 9 passed on the final build; earlier builds of the same day failed 5 of 9 with no cause found, which led to the re-entry recovery |
| 480223a (rebased on main with named fields, #271) | ID read | ok, 0.3 s |
| 480223a | full program memory, write + verify, 3 runs | 3 of 3 passed: 70.2 s, 70.0 s, 70.5 s (about 217 bytes/s) |
| 480223a | blink (32 bytes + reset vector), run after | ok, 1.4 s, LED1 blinks |

Open: M3's 1000-run count, M1/M2/M4/M5 timing, and whether the re-entry recovery ever fired (status does not report it).

## 19. What waits

- Gateware (`ice40/`): not touched. A slot engine (section 15) waits for the SWIM session's change to `swd_engine.v` and for M1/M4.
- DONE 2026-11-25: `prog_sbw` is in the build: enum `PROG_PROTO_SBW` (5), the `prog_flash.c` metadata entry, the dispatch case,
  `command_handler_prog.c` (`proto` "sbw", refused below gateware v52), Makefile/linker/`check_xip_refs.py` lists. A `prog2`
  image only when flash parts or the faster paths come.
- DONE 2026-11-25: `prog_wire` changes. `prog_wire_ops_t.mode()` takes `PROG_WIRE_CLK_LOW_AT_CLOSE` (close leaves TEST low) and
  `PROG_WIRE_STRICT_LOW` (the automatic flushes never run while the clock is low: the frame fails with a protocol error instead
  of being cut); `prog_sbw` sets both in ENTER. Host test: the flags are set (`test_enter`); `prog_wire.c` itself only builds
  for the pod.
- Flash parts (Xv1 0x89, F5xx/F6xx flash controller): after FRAM works on the bench.
- The faster FRAM write paths (mailbox funclet, quick access): after M7.
- Password unlock (FR5xxx/FR6xxx), IPE total erase, LPMx.5 recovery (magic pattern): later, on demand.
