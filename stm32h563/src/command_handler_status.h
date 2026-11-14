#ifndef COMMAND_HANDLER_STATUS_H
#define COMMAND_HANDLER_STATUS_H

/*
 * The `status` reply's short form (command_handler_status.c): the bounds test_ch_status.c checks.
 */

#include "bp_limits.h"

/* The short form cuts last_crash and safe_reason to this many characters. */
#define STATUS_SHORT_TEXT_MAX  48u
/* The worst-case short form leaves at least this many bytes of BP_CLOUD_REPLY_MAX free, so the
   next field added to status cannot silently break `status` over the cloud. */
#define STATUS_SHORT_MARGIN    200u

#endif /* COMMAND_HANDLER_STATUS_H */
