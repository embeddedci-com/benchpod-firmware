/* board_caps.c — see board_caps.h. */
#include "board_caps.h"
#include "board_rev.h"

static board_caps_t s_caps;

board_caps_t board_caps_for_rev(int rev)
{
    board_caps_t c = { 0 };
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
