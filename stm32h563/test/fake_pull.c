/* fake_pull.c: see fake_pull.h. */
#include "fake_pull.h"
#include <string.h>

la_pull_dir_t fake_pull_dir[LA_PULL_CHANNELS + 1];
pull_str_t    fake_pull_str[LA_PULL_CHANNELS + 1];
bool          fake_pull_available = true;
bool          fake_pull_held_off;
int           fake_pull_set_rc;
unsigned      fake_pull_sets;

void fake_pull_reset(void) {
    memset(fake_pull_dir, 0, sizeof(fake_pull_dir));
    memset(fake_pull_str, 0, sizeof(fake_pull_str));
    fake_pull_available = true;
    fake_pull_held_off = false;
    fake_pull_set_rc = 0;
    fake_pull_sets = 0;
}

static int f_set(unsigned la, la_pull_dir_t dir, pull_str_t str) {
    fake_pull_sets++;
    if (fake_pull_set_rc) return fake_pull_set_rc;
    if (dir != LA_PULL_NONE && !fake_pull_available) return LA_PULL_ERR_BANK;
    fake_pull_dir[la] = dir;
    fake_pull_str[la] = dir == LA_PULL_NONE ? PULL_STR_NONE : str;
    return 0;
}
static la_pull_dir_t f_asked(unsigned la, pull_str_t *str) { *str = fake_pull_str[la]; return fake_pull_dir[la]; }
static bool f_live(unsigned la) { return !fake_pull_held_off && fake_pull_dir[la] != LA_PULL_NONE; }
static bool f_available(void) { return fake_pull_available; }
static void f_all_off(void) {
    memset(fake_pull_dir, 0, sizeof(fake_pull_dir));
    memset(fake_pull_str, 0, sizeof(fake_pull_str));
}
static const char *f_held(void) {
    if (!fake_pull_held_off) return NULL;
    for (unsigned la = 1; la <= LA_PULL_CHANNELS; la++) if (fake_pull_dir[la] != LA_PULL_NONE) return "dut_off";
    return NULL;
}

const la_pull_backend_t la_pull_backend_tca9554 = {
    .rail_pin21 = false, .set = f_set, .asked = f_asked, .live = f_live,
    .available = f_available, .all_off = f_all_off,
};
const la_pull_backend_t la_pull_backend_exp = {
    .rail_pin21 = true, .set = f_set, .asked = f_asked, .live = f_live,
    .available = f_available, .all_off = f_all_off, .held = f_held,
};
