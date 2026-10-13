#ifndef CONSOLE_MOVED_H
#define CONSOLE_MOVED_H

#include <stddef.h>

/* Console commands firmware 3.9.0 moved to the JSON API (docs/usb-serial-interface.md, "Moved to
 * the JSON API"). An older benchpod CLI or embeddedci SDK still sends them over the USB console;
 * the console answers with this line instead of "unknown command", so the user learns what to
 * update. Pure: host-tested (test/test_console_moved.c). */

/* The JSON command(s) that replaced console command `verb`, or NULL when it was not moved. */
const char *console_moved_to(const char *verb);

/* The reply line for a moved command (no leading spaces, no line end); "" when not moved. */
size_t console_moved_line(const char *verb, char *out, size_t cap);

#endif /* CONSOLE_MOVED_H */
