#ifndef BOARD_CAPS_H
#define BOARD_CAPS_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * board_caps — what this board has, decided once from the revision straps.
 *
 * The one revision gate.  Code for hardware that only some boards have asks board_caps() and
 * never compares board_rev_get() itself, so adding a board revision is one change here
 * (board_caps_for_rev) instead of version checks spread over the firmware.  Only board_rev.c,
 * this file and the calibration-set selection in cal_data.c name a revision; `make test` fails
 * on board_rev_get / board_rev_is_v3 / BOARD_REV_V* anywhere else in src/
 * (test/check_board_rev_gates.py).
 *
 * Every revision field is false (zero) before board_caps_init() runs, which is how a v2 board ran
 * (board_rev_is_v3() was false before board_rev_init() too), so an early caller gets the most
 * conservative behavior.  board_rev.c already turns any doubt about the straps into
 * BOARD_REV_V3, and board_rev_init() latches the capabilities itself.
 *
 * The analog part (board_caps()->analog) is not a function of the revision: whether the analog
 * front end is fitted, and where, is decided later at boot from I2C probes after the buses are up
 * (board_variant.h), and the module's facts come from its EEPROM (module_link.h).
 * board_variant_init() fills it once the module EEPROM has been read, after board_rev_init(), and
 * board_caps_init() keeps it.  Code asks board_caps()->analog and never board_variant's decision or
 * the module EEPROM getters itself; `make test` fails on those anywhere else
 * (test/check_board_rev_gates.py).
 * ==========================================================================*/

/* The calibration set a revision's analog section was fitted on (cal_data.h). */
typedef enum {
    BOARD_CAL_V2 = 0,   /* v2, and any revision this firmware does not know */
    BOARD_CAL_V3,       /* v3, analog-v3r1, the digital boards (v3r4 included) */
    BOARD_CAL_V3R2,     /* analog v3r2 (and the analog module, set by main.c) */
} board_cal_set_t;

/* The analog front end and the analog module (board_variant.h, module_link.h). */
typedef struct {
    /* The analog front end is fitted: on the board, or as the plug-in module.  True until
       board_variant_init() decides, so nothing is hidden on an analog board by asking early. */
    bool present;
    /* It is the plug-in analog module of digital v3r3. */
    bool module;
    /* Module EEPROM facts (false without the module).  The ADC low range: K5 on U58 P5. */
    bool adc_low_range;
    /* The 4-20 mA input's sense resistor (R81) is 100 ohm; false is 249 ohm. */
    bool current_in_100r;
} board_analog_caps_t;

/* What board_caps()->analog holds before board_variant_init(): analog, on the board. */
#define BOARD_ANALOG_CAPS_UNPROBED ((board_analog_caps_t){ .present = true })

typedef struct {
    /* ---- v3 and later (every revision from the PA3 strap on) ---- */
    /* The LA-bank mux runs in manual mode (MODE tied high): PR1 picks 3.3 V or 1.8 V, and its
       status pin tells which.  False (v2, MODE grounded): only 3.3 V is reachable, with PR1 low,
       and PR1 high shuts the bank down; the status pin is ambiguous; the pull-ups go to +3V3, so
       a pull is refused unless the bank is at 3.3 V.  la_io.c, pull_tca9554.c. */
    bool la_1v8;
    /* PF4 NRST_CTRL, the dedicated open-drain DUT reset pin.  nrst_ctrl.c. */
    bool nrst_pin;
    /* The USB-C CC1/CC2 sense taps on the MCU's ADC.  usb_cc.c. */
    bool usb_cc;

    /* ---- analog v3r2 ---- */
    /* The on-board analog section has the quad DAC (a channel per output) instead of the single
       DAC and its output muxes.  Only meaningful when the analog front end is on the board
       (dac_quad_for_board(); the analog module has the quad DAC on any revision). */
    bool dac_quad;
    /* The calibration set (cal_data_use()).  main.c switches to BOARD_CAL_V3R2 when the analog
       front end is the module. */
    board_cal_set_t cal_set;

    /* ---- digital v3r4 ---- */
    /* J1 pin 21 is VREF_OUT: PG5 VREF_SEL (follow the bank / 5 V input), PG6 VREF_5V_EN (the 5 V
       switch) and the PF11 VREF_SENSE divider (one third of pin 21).  vref_out.h. */
    bool vref_out;
    /* 5 V I/O: the bank stays at 3.3 V and pin 21 (the pull-up rail) goes to 5 V. */
    bool la_5v;
    /* LSF0108 clamps on the LA lines and NRST: wait for their reference after a bank change. */
    bool la_clamps;
    /* Two TCA6416A pull expanders on I2C0 (U54 0x21: LA1-LA7, LA9; U18 0x20: LA8, LA10-LA14;
       a 10k and a 2.2k per channel), reset on PG7 PULL_RST: every channel pulls up or down with
       10k, 2.2k or both.  False: the TCA9554 at 0x20.  Read by la_pull_init() (it picks the pull
       backend, la_pull.h) and pull_exp_init(). */
    bool pull_expanders;
    /* PC9 USB_SEL: the USB switch between USB-C and the USB-A host port (low = USB-C, the
       board's default on its pull-down).  Nothing drives it yet: kept for the planned USB
       host support, which switches it on demand only. */
    bool usb_sel;
    /* The DAC clock is on the module flat cable's pin 7 and its data on pin 8 (dac_link.h); older
       digital boards have them the other way round.  Only matters on a two-board pod. */
    bool cn1_dac_swap;
    /* The quad bus chip-selects are on PB6 (/ICE_SS, the config flash) and PC11 (/ICE_SS_PSRAM),
       pins that can also carry OCTOSPI1_NCS; false: PE3 and PE4.  qspi_cs.h. */
    bool qspi_cs_ncs_pins;

    /* ---- the analog front end (not from the revision: board_caps_set_analog) ---- */
    board_analog_caps_t analog;
} board_caps_t;

/* The capabilities of a revision (a BOARD_REV_* value), with analog BOARD_ANALOG_CAPS_UNPROBED.
   Pure (host tests). */
board_caps_t board_caps_for_rev(int rev);

/* Latch the capabilities of a revision.  board_rev_init() calls it once it has the revision;
   host tests call it directly.  The analog part is kept as it is. */
void board_caps_init(int rev);

/* Latch the analog part.  board_variant.c calls it at boot (board_variant_init) and again when the
   module's power comes back on and its EEPROM was read again (module_power_set). */
void board_caps_set_analog(board_analog_caps_t analog);

/* The latched capabilities (the revision fields all false before board_caps_init()). Never NULL. */
const board_caps_t *board_caps(void);

#endif /* BOARD_CAPS_H */
