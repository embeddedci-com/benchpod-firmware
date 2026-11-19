/* pull_tca9554.c: the LA pull backend of the boards before digital v3r4 (la_pull.h).
 *
 * One TCA9554 at 0x20 (U54 on the schematic; older code called it pca9555): LA<n> (1..8) -> expander pin
 * (CTRL1=P2, CTRL2=P3, CTRL3=P1, CTRL4=P0, CTRL5=P7, CTRL6=P6, CTRL7=P5, CTRL8=P4).  Enabling drives
 * the ctrl line high, closing the analog switch that connects the channel's one resistor (la_pull.c
 * has the table: which way it pulls and its value).  LA9..LA14 have none.
 *
 * On v3 the LA1-6 pull-ups go to LA_VCCIO (the bank rail) and the LA7/8 pull-downs to GND, so every
 * pull works at 1.8 V and at 3.3 V.  On v2 the pull-ups went to +3V3: with the bank at 1.8 V a
 * closed one drives the DUT line about 1.5 V above its own rail, back-feeding the DUT and the
 * iCE40's 1.8 V bank.  v2 cannot run the bank at 1.8 V at all (la_vccio_set_mv refuses it), but the
 * gate stays: closing a switch needs available() (a bank voltage selected, and 3.3 V on v2), and
 * la_vccio_set_mv(1800) drops every pull on v2 (la_pull_all_off).  Opening one is always allowed.
 */
#include "la_pull.h"
#include "i2c_bus.h"         /* I2C_ADDR_TCA9554_LA */
#include "tca9554.h"
#include "signal_engine.h"   /* la_vccio_get_mv: a pull needs a selected bank voltage */
#include "board_rev.h"       /* board_rev_is_v3: v2 pull-ups go to +3V3 */

static const int8_t s_tca_pin[9] = { -1, 2, 3, 1, 0, 7, 6, 5, 4 };
static uint8_t      s_on;   /* bit la-1: the switch is closed (what the host asked for) */

static bool tca_available(void)
{
    int mv = la_vccio_get_mv();
    if (mv == LA_VCCIO_3V3) return true;
    return mv == LA_VCCIO_1V8 && board_rev_is_v3();
}

static int tca_set(unsigned la, la_pull_dir_t dir, pull_str_t str)
{
    (void)str;   /* one fixed resistor per channel */
    if (la < 1 || la > 8) return LA_PULL_ERR_IO;
    bool enable = dir != LA_PULL_NONE;
    if (enable && !tca_available()) return LA_PULL_ERR_BANK;
    uint8_t pin = (uint8_t)s_tca_pin[la];
    if (tca9554_set_dir(I2C_ADDR_TCA9554_LA, pin, true) != 0) return LA_PULL_ERR_IO;
    if (tca9554_write_pin(I2C_ADDR_TCA9554_LA, pin, enable) != 0) return LA_PULL_ERR_IO;
    if (enable) s_on |= (uint8_t)(1u << (la - 1));
    else        s_on &= (uint8_t)~(1u << (la - 1));
    return 0;
}

static la_pull_dir_t tca_asked(unsigned la, pull_str_t *str)
{
    *str = PULL_STR_NONE;
    if (la < 1 || la > 8 || !((s_on >> (la - 1)) & 1u)) return LA_PULL_NONE;
    return la_pull_caps(la).dflt ? (la_pull_dir_t)la_pull_caps(la).dflt : LA_PULL_NONE;
}

static bool tca_live(unsigned la)
{
    pull_str_t s;
    return tca_asked(la, &s) != LA_PULL_NONE;
}

static void tca_all_off(void)
{
    for (unsigned la = 1; la <= 8; la++)
        if ((s_on >> (la - 1)) & 1u) (void)tca_set(la, LA_PULL_NONE, PULL_STR_NONE);
}

const la_pull_backend_t la_pull_backend_tca9554 = {
    .rail_pin21 = false,
    .set        = tca_set,
    .asked      = tca_asked,
    .live       = tca_live,
    .available  = tca_available,
    .all_off    = tca_all_off,
};
