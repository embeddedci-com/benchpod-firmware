/* board_caps.c — see board_caps.h. */
#include "board_caps.h"
#include "board_rev.h"

static board_caps_t s_caps;

board_caps_t board_caps_for_rev(int rev)
{
    board_caps_t c = { 0 };
    /* v3 and later (rev >= BOARD_REV_V3): the old board_rev_is_v3() gate. */
    if (rev >= BOARD_REV_V3) {
        c.la_1v8   = true;
        c.nrst_pin = true;
        c.usb_cc   = true;
    }
    if (rev == BOARD_REV_V3R2) c.dac_quad = true;
    /* Digital v3r4 is a v3 board: the v3 set, as on the digital boards before it.  Any other
       revision (unknown, or newer than this firmware) gets the v2 set, as before. */
    c.cal_set = (rev == BOARD_REV_V3R2) ? BOARD_CAL_V3R2
              : (rev == BOARD_REV_V3 || rev == BOARD_REV_V3R4) ? BOARD_CAL_V3 : BOARD_CAL_V2;
    if (rev == BOARD_REV_V3R4) {
        c.vref_out       = true;
        c.la_5v          = true;
        c.la_clamps      = true;
        c.pull_expanders = true;
        c.usb_sel        = true;
    }
    return c;
}

void board_caps_init(int rev) { s_caps = board_caps_for_rev(rev); }

const board_caps_t *board_caps(void) { return &s_caps; }
