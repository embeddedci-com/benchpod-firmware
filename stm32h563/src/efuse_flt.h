/* efuse_flt.h: is a held FLT a real eFuse fault, or the TPS259470A blocking reverse current?
   See target_power_set_flt_check. */
#ifndef EFUSE_FLT_H
#define EFUSE_FLT_H

#include <stdbool.h>

#define EFUSE_FLT_COLLAPSED_MV 1000    /* below this the output is down: a fault */
#define EFUSE_FLT_LOAD_UA      250000  /* above this the output carries current: an overload */

/* The rule, from the output's voltage and current (host-tested). */
bool efuse_flt_is_fault(int efuse, int out_mv, int out_ua);

/* Registers the check with target_power (after target_power_init and the INA probe). */
void efuse_flt_init(void);

#endif
