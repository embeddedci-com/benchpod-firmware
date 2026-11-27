/* board_caps.c — see board_caps.h. */
#include "board_caps.h"
#include "board_rev.h"

static board_caps_t s_caps = { .analog = { .present = true } };   /* BOARD_ANALOG_CAPS_UNPROBED */

board_caps_t board_caps_for_rev(int rev)
{
    board_caps_t c = { 0 };
    c.analog = BOARD_ANALOG_CAPS_UNPROBED;
    /* v3 and later (rev >= BOARD_REV_V3): the old board_rev_is_v3() gate. */
    if (rev >= BOARD_REV_V3) {
        c.la_1v8   = true;
        c.nrst_pin = true;
        c.usb_cc   = true;
    }
    if (rev == BOARD_REV_V3R2) c.dac_quad = true;
    /* Digital v4 is a v3 board: the v3 set, as on the digital boards before it.  Any other
       revision (unknown, or newer than this firmware) gets the v2 set, as before. */
    c.cal_set = (rev == BOARD_REV_V3R2) ? BOARD_CAL_V3R2
              : (rev == BOARD_REV_V3 || rev == BOARD_REV_V4) ? BOARD_CAL_V3 : BOARD_CAL_V2;
    if (rev == BOARD_REV_V4) {
        c.vref_out       = true;
        c.la_5v          = true;
        c.la_clamps      = true;
        c.pull_expanders = true;
        c.usb_sel        = true;
        c.supply_3v3_1v8      = true;
        c.cn1_dac_swap   = true;
        c.qspi_cs_ncs_pins = true;
        /* c.power_cycle stays false: the PWR_CYCLE circuit (PC10) is not built or tested yet. */
    }
    return c;
}

void board_caps_init(int rev)
{
    const board_analog_caps_t analog = s_caps.analog;   /* not from the revision */
    s_caps = board_caps_for_rev(rev);
    s_caps.analog = analog;
}

void board_caps_set_analog(board_analog_caps_t analog) { s_caps.analog = analog; }

const board_caps_t *board_caps(void) { return &s_caps; }
