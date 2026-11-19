/* la_pull.c: the LA pull resistors (i2c_bus.h declares them for now): the TCA9554 at 0x20 on the
   boards before digital v3r4, the two pull expanders on v3r4 (pull_exp.h).  Moved out of
   i2c_bus.c unchanged, so the host tests can build it (test/test_ch_pins.c). */
#include "i2c_bus.h"
#include "tca9554.h"
#include "signal_engine.h"   /* la_vccio_get_mv: a pull needs a selected bank voltage */
#include "board_rev.h"       /* board_rev_is_v3: v2 pull-ups go to +3V3 */
#include "board_caps.h"
#include "la_pins.h"         /* la_pull_strength_for, la_pins_note_pull */
#include "pull_exp.h"        /* digital v3r4: the TCA6416A pull expanders */
#include "vref_out.h"        /* vref_out_rail_up: a held pull with pin 21 up is a failed write */

/* ---- LA pull-up control via TCA9554 @ 0x20 ----------------------------
   LA<n> (1..8) -> expander pin (U54: CTRL1=P2, CTRL2=P3, CTRL3=P1, CTRL4=P0,
   CTRL5=P7, CTRL6=P6, CTRL7=P5, CTRL8=P4).  Enabling drives the ctrl line high,
   closing the analog switch that connects the pull-up resistor.

   On v3 boards before digital v3r4 the LA1-6 pull-ups go to LA_VCCIO (the bank rail) and
   the LA7/8 pull-downs to GND, so every pull works at 1.8 V and at 3.3 V.  Digital v3r4 has
   pull expanders instead (pull_exp.h): every channel pulls up or down.  On v2 the
   pull-ups went to +3V3; v2 cannot run the bank at 1.8 V at all, but the gate
   stays for it (see i2c_bus.h).  la_pullups_available() is the one place that
   decides this; everything else asks it. */
static const int8_t s_la_tca_pin[9] = { -1, 2, 3, 1, 0, 7, 6, 5, 4 };
static uint8_t s_la_pu_shadow;

/* ---- digital v3r4: the two TCA6416A expanders (pull_exp.h) ----
   Each entry point below checks board_caps()->pull_expanders first and, on every other board,
   runs the TCA9554 code exactly as before.  The pulls go to J1 pin 21, which follows the bank
   (or 5 V), so on v3r4 they work at every I/O voltage, but only while pin 21 is up. */

int la_pull_engage(uint8_t la, la_pull_dir_t dir, int str)
{
    if (!board_caps()->pull_expanders) return -1;
    if (la < 1 || la > LA_PINS_COUNT) return -1;
    if (dir != LA_PULL_NONE) {
        if (la_vccio_get_mv() == LA_VCCIO_UNSET) return LA_PULLUP_ERR_VOLTAGE;
        /* After a failed write (pin 21 up), this request tries the expanders again. */
        if (!pull_exp_recover()) return vref_out_rail_up() ? -1 : LA_PULLUP_ERR_RAIL;
        if (str <= 0) str = la_pull_strength_for(la, la_io_get_mv());
    }
    pull_exp_dir_t d = dir == LA_PULL_UP ? PULL_EXP_UP : dir == LA_PULL_DOWN ? PULL_EXP_DOWN : PULL_EXP_OFF;
    int rc = pull_exp_set(la, d, (pull_str_t)str);
    if (rc == PULL_EXP_ERR_RAIL) return LA_PULLUP_ERR_RAIL;
    if (rc != 0) return -1;
    la_pins_note_pull(la, dir, pull_str_ohms(pull_exp_str(la)));
    return 0;
}

bool la_pullups_available(void)
{
    if (board_caps()->pull_expanders) return la_vccio_get_mv() != LA_VCCIO_UNSET && pull_exp_live();
    int mv = la_vccio_get_mv();
    if (mv == LA_VCCIO_3V3) return true;
    return mv == LA_VCCIO_1V8 && board_rev_is_v3();
}

void la_pullups_all_off(void)
{
    if (board_caps()->pull_expanders) {
        pull_exp_all_off();
        for (unsigned la = 1; la <= LA_PINS_COUNT; la++) la_pins_note_pull(la, LA_PULL_NONE, NULL);
        return;
    }
    for (uint8_t la = 1; la <= 8; la++) {
        if ((s_la_pu_shadow >> (la - 1)) & 1u) (void)pca9555_set_la_pullup(la, false);
    }
}

int pca9555_set_la_pullup(uint8_t la_pin, bool enable)
{
    /* v3r4: "on" pulls up, with the strength the pin's function asks for. */
    if (board_caps()->pull_expanders) return la_pull_engage(la_pin, enable ? LA_PULL_UP : LA_PULL_NONE, 0);
    if (la_pin < 1 || la_pin > 8) return -1;
    /* Turning a pull OFF is always safe; only closing the switch is gated (no bank
       voltage selected, or a v2 board's +3V3 pull-ups with the bank at 1.8 V). */
    if (enable && !la_pullups_available()) return LA_PULLUP_ERR_VOLTAGE;
    uint8_t pin = (uint8_t)s_la_tca_pin[la_pin];
    if (tca9554_set_dir(I2C_ADDR_TCA9554_LA, pin, true) != 0) return -1;
    if (tca9554_write_pin(I2C_ADDR_TCA9554_LA, pin, enable) != 0) return -1;
    if (enable) s_la_pu_shadow |= (uint8_t)(1u << (la_pin - 1));
    else        s_la_pu_shadow &= (uint8_t)~(1u << (la_pin - 1));
    return 0;
}

bool pca9555_la_pullup_enabled(uint8_t la_pin)
{
    if (board_caps()->pull_expanders) return pull_exp_dir(la_pin) != PULL_EXP_OFF;
    if (la_pin < 1 || la_pin > 8) return false;
    return (s_la_pu_shadow >> (la_pin - 1)) & 1u;
}

bool la_pull_live(uint8_t la_pin)
{
    if (board_caps()->pull_expanders) return pull_exp_live_dir(la_pin, NULL) != PULL_EXP_OFF;
    return pca9555_la_pullup_enabled(la_pin);
}

/* Which way each channel pulls and its value: la_pins.c (la_pull_dir, la_pull_ohms), one table
   per board revision.  This file only knows which expander pin switches which channel. */

