#ifndef USB_CHECK_H
#define USB_CHECK_H

/* usb_check — checks of the board's own USB firmware (USB_HOST=1 builds).
 *
 * The pod hosts the board (usb_host.h) and powers it through target-power eFuse 1, so it measures
 * what a PC cannot: enumeration time after power-on, current against bMaxPower and in suspend,
 * bus resets, short VBUS dropouts. One check runs at a time in the background, in steps on the
 * hw worker (usb_check_poll), like usb_flash. The logic is usb_check_core.h; the bus actions
 * are usb_bus.h. Every check that switches eFuse 1 puts it back as it found it, also when it
 * is aborted or host mode ends. */

#include <stdbool.h>
#include <stddef.h>

#include "usb_check_core.h"

/* 0 = started; -1 = refused (BP0087 `why`); -2 = the board is not powered from eFuse 1 (BP0088). */
int  usb_check_start(const uc_args_t *a, const char **why);
void usb_check_poll(void);
void usb_check_abort(void);
bool usb_check_busy(void);
/* The state and the last result as JSON; 0 when it does not fit `cap`. */
size_t usb_check_reply(char *out, size_t cap);

#endif /* USB_CHECK_H */
