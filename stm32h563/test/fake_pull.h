/* fake_pull: both LA pull backends (la_pull.h) as RAM, for the host tests that link the pin table
   and la_pull.c but not the pull hardware.  la_pull.c (the tables, the reporting, the hold) is the
   real one; a test calls board_caps_init() then la_pull_init() to pick the board. */
#ifndef FAKE_PULL_H
#define FAKE_PULL_H
#include "la_pull.h"

extern la_pull_dir_t fake_pull_dir[LA_PULL_CHANNELS + 1];   /* what la_pull_set asked for */
extern pull_str_t    fake_pull_str[LA_PULL_CHANNELS + 1];   /* the strength as passed (NONE = by function) */
extern bool          fake_pull_available;                   /* la_pull_available (default true) */
extern bool          fake_pull_held_off;                    /* every asked pull off the wire (v3r4 hold) */
extern int           fake_pull_set_rc;                      /* nonzero: la_pull_set fails with it */
extern unsigned      fake_pull_sets;                        /* la_pull_set calls that reached the backend */
void fake_pull_reset(void);                                 /* everything off, defaults back */
#endif
