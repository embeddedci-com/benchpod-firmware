#ifndef BOARD_CAPS_H
#define BOARD_CAPS_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * board_caps — what this board has, decided once from the revision straps.
 *
 * The one revision gate for hardware that only some boards have.  Code for such hardware asks
 * board_caps() and never compares board_rev_get() itself, so adding a board revision is one
 * change here (board_caps_for_rev) instead of version checks spread over the firmware.
 *
 * Every field is false (zero) on the boards that existed before digital v3r4, and before
 * board_caps_init() runs, so an early caller or an unrecognised board gets exactly the behavior
 * the firmware had before v3r4.  board_rev.c already turns any doubt about the straps into
 * BOARD_REV_V3.
 *
 * Older gates stay where they are (board_rev_is_v3() for the v3 features, board_variant.h for the
 * analog front end, module_link.h for the module link); this struct is for what is new.
 * ==========================================================================*/

typedef struct {
    /* J1 pin 21 is VREF_OUT: PG5 VREF_SEL (follow the bank / 5 V input), PG6 VREF_5V_EN (the 5 V
       switch) and the PA0 VREF_SENSE divider (one third of pin 21).  vref_out.h. */
    bool vref_out;
    /* 5 V I/O: the bank stays at 3.3 V and pin 21 (the pull-up rail) goes to 5 V. */
    bool la_5v;
    /* LSF0108 clamps on the LA lines and NRST: wait for their reference after a bank change. */
    bool la_clamps;
    /* Two TCA6416A pull expanders on I2C0 (0x20 2.2k, 0x21 10k), reset on PG7 PULL_RST: every
       channel pulls up or down with 10k, 2.2k or both.  False: the TCA9554 driver at 0x20. */
    bool pull_expanders;
    /* PC9 USB_SEL: the USB switch between USB-C and the USB-A host port (low = USB-C, the
       board's default on its pull-down).  Nothing drives it yet. */
    bool usb_sel;
    /* The LA series resistors, in ohms (100 on v3r4, 330 before). */
    uint16_t la_series_ohms;
} board_caps_t;

/* The capabilities of a revision (a BOARD_REV_* value).  Pure (host tests). */
board_caps_t board_caps_for_rev(int rev);

/* Latch the capabilities of a revision: board_caps_init(board_rev_get()), once, right after
   board_rev_init(). */
void board_caps_init(int rev);

/* The latched capabilities (all false before board_caps_init()). Never NULL. */
const board_caps_t *board_caps(void);

#endif /* BOARD_CAPS_H */
