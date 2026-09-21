/* usb_cc.c — see usb_cc.h.  Two averaged ADC reads plus a threshold table. */
#include "usb_cc.h"
#include "mcu_adc.h"
#include "board_rev.h"
#include "stm32h5xx_hal.h"
#include "board_pins.h"

/* Sink-side vRd band edges, at the midpoints of the spec's gaps. */
#define CC_FLOOR_MV      200    /* below: open, unused line, or a cable Ra */
#define CC_DEFAULT_MAX   660    /* 0.25..0.61 V band -> default USB power  */
#define CC_1A5_MAX      1230    /* 0.70..1.16 V band -> 1.5 A             */
                                /* above: 1.31..2.04 V band -> 3.0 A      */

/* The CC taps sit behind 10 k / 10 n and pick up the pod's own switching
   noise; average enough samples to make the band decision stable. */
#define CC_SAMPLES         8

void usb_cc_init(void)
{
    if (!board_rev_is_v3()) return;
    /* mcu_adc_init() already put PC0/PC2 in analog mode; nothing else to do.
       Kept as an explicit entry point so main()'s boot sequence reads in the
       same order as the hardware it brings up. */
    USB_CC_CLK_EN();
}

bool usb_cc_supported(void) { return board_rev_is_v3(); }

static void classify(int mv, int *ma, const char **name)
{
    if (mv < CC_FLOOR_MV)         { *ma = 0;    *name = "none";    }
    else if (mv < CC_DEFAULT_MAX) { *ma = 500;  *name = "default"; }
    else if (mv < CC_1A5_MAX)     { *ma = 1500; *name = "1.5A";    }
    else                          { *ma = 3000; *name = "3.0A";    }
}

int usb_cc_read(usb_cc_t *out)
{
    if (out == NULL) return -1;

    out->supported     = usb_cc_supported();
    out->cc1_mv        = -1;
    out->cc2_mv        = -1;
    out->orientation   = USB_CC_ORIENT_NONE;
    out->advertised_ma = 0;
    out->advertised    = "none";
    if (!out->supported) return -1;

    int cc1 = 0, cc2 = 0;
    if (mcu_adc_read_mv_avg(USB_CC1_ADC_CH, CC_SAMPLES, &cc1) != 0) return -1;
    if (mcu_adc_read_mv_avg(USB_CC2_ADC_CH, CC_SAMPLES, &cc2) != 0) return -1;
    out->cc1_mv = cc1;
    out->cc2_mv = cc2;

    /* The live line is the higher of the two, and only if it clears the floor —
       the other one is left at ~0 by our own Rd (or at vRa by a powered cable). */
    int live = (cc1 >= cc2) ? cc1 : cc2;
    if (live >= CC_FLOOR_MV)
        out->orientation = (cc1 >= cc2) ? USB_CC_ORIENT_CC1 : USB_CC_ORIENT_CC2;

    classify(live, &out->advertised_ma, &out->advertised);
    return 0;
}
