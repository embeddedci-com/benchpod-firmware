#ifndef USB_CC_H
#define USB_CC_H

#include <stdbool.h>

/* ============================================================================
 * usb_cc — USB-C CC-line monitoring (rev3+).
 *
 * The pod is a plain USB-C sink: USBC1's CC1/CC2 each carry a 5.1 k Rd to GND
 * (R76/R75).  An attached source pulls one of them up through its Rp, and the
 * resulting divider voltage is the source's current advertisement.  Each line
 * is tapped into an ADC pin through 10 k with 10 n to GND, and the ADC input is
 * high-Z, so the pin reads the CC voltage directly.
 *
 * Thresholds are the USB Type-C sink-side vRd bands (spec Table "Voltage on
 * sink CC pins"), split at the midpoint of the gaps between them:
 *
 *     < 200 mV   nothing attached, or this is the unused CC line, or a
 *                powered-cable Ra (vRa max is 150 mV)
 *   200..660 mV  Default USB power   (nominal 410 mV) — 500/900 mA
 *   660..1230 mV 1.5 A @ 5 V         (nominal 920 mV)
 *      > 1230 mV 3.0 A @ 5 V         (nominal 1680 mV)
 *
 * Whichever line is above the floor also gives the cable orientation.  Note a
 * legacy USB-A-to-C cable or a dumb 5 V supply presents no Rp at all: both CC
 * lines read ~0 and this reports "none" even though VBUS is live.  That is
 * correct — there is no advertisement to report.
 *
 * This is REPORT-ONLY.  Nothing in the firmware gates on it; the host decides
 * what to do with the number (see docs/API.md, `usb_cc`).
 * ==========================================================================*/

#define USB_CC_ORIENT_NONE 0
#define USB_CC_ORIENT_CC1  1
#define USB_CC_ORIENT_CC2  2

typedef struct {
    bool supported;      /* false on v2 — the CC taps do not exist            */
    int  cc1_mv;         /* CC1 pin voltage, -1 if unread                     */
    int  cc2_mv;         /* CC2 pin voltage, -1 if unread                     */
    int  orientation;    /* USB_CC_ORIENT_*                                   */
    int  advertised_ma;  /* 0 (none), 500, 1500 or 3000                       */
    const char *advertised; /* "none" / "default" / "1.5A" / "3.0A"           */
} usb_cc_t;

/* Configure the CC ADC pins.  Safe (and a no-op) on v2. */
void usb_cc_init(void);

/* True on rev3 and later. */
bool usb_cc_supported(void);

/* Sample both CC lines and classify.  Returns 0 on success, -1 if unsupported
   or the ADC failed (*out is still filled with supported/-1 values).
   Takes ~1 ms (8 averaged conversions per line); call it from the hw worker. */
int usb_cc_read(usb_cc_t *out);

#endif /* USB_CC_H */
