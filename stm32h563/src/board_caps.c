/* board_caps.c — see board_caps.h. */
#include "board_caps.h"
#include "board_rev.h"

static board_caps_t s_caps = { .la_series_ohms = 330 };

board_caps_t board_caps_for_rev(int rev)
{
    board_caps_t c = { .la_series_ohms = 330 };
    if (rev == BOARD_REV_V3R4) {
        c.vref_out       = true;
        c.la_5v          = true;
        c.la_clamps      = true;
        c.pull_expanders = true;
        c.usb_sel        = true;
        c.la_series_ohms = 100;
    }
    return c;
}

void board_caps_init(void) { s_caps = board_caps_for_rev(board_rev_get()); }

const board_caps_t *board_caps(void) { return &s_caps; }
