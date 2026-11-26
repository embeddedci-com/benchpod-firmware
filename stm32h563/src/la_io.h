#ifndef LA_IO_H
#define LA_IO_H

/* la_io: the LA I/O bank (TPS2116 mux), 5 V I/O and pin 21 (digital v4).

   The LA bank VCCIO (iCE40 bank 0 — supplies ALL of LA1..LA14) is switched
   between 3.3 V and 1.8 V by a TPS2116 power-mux (U8): PG3 (LA_VCCIO_SWITCH)
   drives PR1 (the select), PG4 (LA_VCCIO_ST) reads the open-drain ST output.
   PR1 and ST each have a 100 k pull-up to +3V3 (R151/R152).

   The MODE pin (U8 pin 5) moved from GND to +3V3 in rev3, which changes the
   part's behaviour completely — from the datasheet truth table:

     v3, MODE >= 1 V  (manual mode)
        PR1 high  ->  ST high  ->  VOUT = VIN1 = +3.3 V
        PR1 low   ->  ST low   ->  VOUT = VIN2 = +1.8 V
     v2, MODE <= 0.35 V
        PR1 high  ->  ST low   ->  VOUT = Hi-Z (SHUTDOWN)
        PR1 low   ->            ->  VOUT = the HIGHER input = +3.3 V

   So on v2 there is no 1.8 V setting at all: what the firmware used to call
   "1.8 V" (PG3 high) actually shut the mux down and left the bank rail floating
   on its own decoupling.  la_vccio_set_mv() refuses 1800 on a v2 board, and the
   PG3 polarity for 3.3 V is inverted between the two revisions.  On v3 the Hi-Z
   power-on default (PR1 pulled high) is a genuine +3.3 V, and ST is a real
   readback of which input is live.

   The level MUST be chosen by the host before ANY LA-bank operation (la /
   la_capture / dap_start / uart_proxy / i2c-sensor); until then the bank is at
   an undefined level for the DUT and those commands are refused.  Selection is
   tracked in software (LA_VCCIO_UNSET at boot) and enforced independently of the
   hardware default so the host makes an explicit choice.

   Two voltages, and they differ only in 5 V mode (digital v4, board_caps()->la_5v), where
   the bank stays at 3.3 V and pin 21 goes to 5 V:
     la_vccio_get_mv()  the bank: what the FPGA pins, the clamps and the pull gates see
                        (UNSET, 1V8 or 3V3).  Ask this for "is a voltage chosen" and for the
                        pin 21 setpoint.
     la_io_get_mv()     the DUT's I/O voltage (UNSET, 1V8, 3V3 or 5V0): what status and
                        la_voltage report, and what pull strengths and pin checks follow.

   Everything here runs on the hw worker (hw_worker.h): the one task that touches the bank,
   pin 21 and the pulls. */
#define LA_VCCIO_UNSET 0
#define LA_VCCIO_1V8   1800
#define LA_VCCIO_3V3   3300
/* 5 V I/O, digital v4 only (board_caps()->la_5v): the bank at 3.3 V, pin 21 at 5 V. */
#define LA_VCCIO_5V0   5000
/* After a bank change on a board with the LA clamps (board_caps()->la_clamps).  Calculated from the
   clamp reference's filter; to be checked on the first board. */
#define LA_VCCIO_SETTLE_CLAMPS_MS 30

/* Configure PG3 (Hi-Z -> the hardware default for this revision) and PG4
   (input); software state = unset. */
void la_vccio_init(void);
/* Select the LA bank voltage; mv must be LA_VCCIO_1V8 or LA_VCCIO_3V3.
   Drives PG3 with the revision's polarity and settles the rail.
   Returns 0 on success, -1 on an invalid millivolt value, -2 for 1800 on a v2
   board (the mux cannot produce 1.8 V there — see above).  Digital v4 also takes
   LA_VCCIO_5V0 (-3: 5 V did not come up on pin 21; the previous setting is restored);
   every other board refuses 5000 with -1, as before. */
int  la_vccio_set_mv(int mv);
/* The DUT's I/O voltage: la_vccio_get_mv(), or LA_VCCIO_5V0 in 5 V mode.  The bank itself (what
   the FPGA and the pull gates see) is la_vccio_get_mv(). */
int  la_io_get_mv(void);
/* Current selection: LA_VCCIO_UNSET (not chosen yet), LA_VCCIO_1V8, or LA_VCCIO_3V3. */
int  la_vccio_get_mv(void);
/* Raw TPS2116 status pin (PG4/ST) level: 0 or 1. */
int  la_vccio_status_pin(void);
/* ST decoded to a rail voltage on v3 (LA_VCCIO_3V3 or LA_VCCIO_1V8); 0 on v2,
   where ST low is ambiguous between "1.8 V" and "shut down". */
int  la_vccio_readback_mv(void);

/* 5 V I/O (digital v4): the bank to 3.3 V, then pin 21 to 5 V, checked on the pin 21 sense.
   0 ok (also when it already is), -3 5 V did not come up (the previous setting is restored), or
   la_vccio_set_mv()'s error for the 3.3 V step.  la_vccio_set_mv(LA_VCCIO_5V0) calls it on a
   board with board_caps()->la_5v; exposed for the host test. */
int  la_io_set_5v(void);

#endif /* LA_IO_H */
