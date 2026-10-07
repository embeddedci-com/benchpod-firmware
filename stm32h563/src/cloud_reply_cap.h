#ifndef CLOUD_REPLY_CAP_H
#define CLOUD_REPLY_CAP_H

/*
 * cloud_reply_cap — captures the reply of a command that arrived as a cloud command.request
 * (command_handler_dispatch_cloud): the handlers write it with at_send_data as for any client,
 * and net_server routes CH_CLOUD_CONN's bytes here instead of to a socket.
 *
 * A reply longer than the capture used to be cut at BP_CLOUD_REPLY_MAX - 1 bytes and sent on as
 * if whole: the server got JSON with no end. Now an overflow is remembered and the caller gets a
 * clear "too large" error with the size instead.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bp_limits.h"

#define CLOUD_REPLY_CAP_MAX  BP_CLOUD_REPLY_MAX   /* including the NUL */

void cloud_reply_cap_begin(void);
/* Bytes from the handler; ignored while no capture is running. */
void cloud_reply_cap_append(const uint8_t *buf, size_t len);
/* End the capture: the reply without its trailing newline into out (NUL-terminated), or an
   error reply when it overflowed or there was none. Returns the length (0 only when out_cap
   cannot hold even the error). */
size_t cloud_reply_cap_end(char *out, size_t out_cap);

#endif /* CLOUD_REPLY_CAP_H */
