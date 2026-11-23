/* usb_bus.c — see usb_bus.h. USB_HOST=1 builds only. */
#include "usb_bus.h"

#include "tusb.h"
#include "host/hcd.h"   /* hcd_event_device_attach */
#include "stm32h5xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#define SUSPRDY_MS     10u   /* SUSPRDY must follow SUSPEN within this */
#define SOF_WATCH_MS   5u    /* frame number frozen this long = SOFs stopped */
#define RESUME_MS      21u   /* K signaling: the spec asks 20 ms */
#define RESUME_SOF_MS  5u    /* then the SOFs must count again within this */
#define RECOVERY_MS    10u   /* TRSMRCY: no traffic right after a resume */

static volatile uint8_t s_state;    /* usb_bus_state_t */
static uint8_t  s_op, s_step;
static bool     s_suspended;
static uint16_t s_fn;
static uint32_t s_t;
static usb_bus_info_t s_info;

/* THE resume register order, in one place. Stage 0 starts the resume signaling (L2RES, held
   RESUME_MS as ST's HAL_HCD_ResumePort does), stage 1 ends it and lets the SOFs run again, stage 2
   clears the bits and flags without any signaling (the clean-up of a bus that is not suspended, or
   a role change). Bench-verified on an Uno R3 (16U2): SOFs restart, the board answers, 5/5. */
static void resume_regs(unsigned stage)
{
    switch (stage) {
    case 0:   /* resume signaling while the suspend state is still set, as the HAL does for a wake-up */
        USB_DRD_FS->CNTR |= USB_CNTR_L2RES;
        break;
    default:  /* stages 1 and 2 */
        USB_DRD_FS->CNTR &= ~USB_CNTR_L2RES;
        USB_DRD_FS->CNTR &= ~USB_CNTR_SUSPEN;   /* the hardware already cleared it during the K state */
        /* Bench-found (Uno R3, 2026-11-23): the SOFs restart only once the SUSP and WKUP flags are
           cleared too. The host driver masks those interrupts, so nobody else clears them; without
           this the frame number stays frozen and the port is dead until host mode restarts.
           ISTR is rc_w0: writing 0 clears a flag, writing 1 leaves it. */
        USB_DRD_FS->ISTR = ~(uint32_t)(USB_ISTR_SUSP | USB_ISTR_WKUP);
        break;
    }
}

static uint16_t frame_now(void) { return (uint16_t)(USB_DRD_FS->FNR & USB_FNR_FN); }

int usb_bus_request(usb_bus_op_t op)
{
    int rc = -1;
    taskENTER_CRITICAL();
    if (s_state != USB_BUS_BUSY &&
        !((op == USB_BUS_OP_SUSPEND || op == USB_BUS_OP_RESET) && s_suspended)) {
        s_op = (uint8_t)op;
        s_step = 0;
        s_state = USB_BUS_BUSY;
        rc = 0;
    }
    taskEXIT_CRITICAL();
    return rc;
}

usb_bus_state_t usb_bus_state(usb_bus_info_t *info)
{
    taskENTER_CRITICAL();
    usb_bus_state_t st = (usb_bus_state_t)s_state;
    if (info) *info = s_info;
    taskEXIT_CRITICAL();
    return st;
}

bool usb_bus_suspended(void) { return s_suspended; }

void usb_bus_role_changed(void)
{
    if (s_suspended || s_state == USB_BUS_BUSY) resume_regs(2);   /* never leave SUSPEN or L2RES set */
    s_suspended = false;
    s_state = USB_BUS_IDLE;
}

static void done(bool ok) { s_state = ok ? USB_BUS_OK : USB_BUS_FAIL; }

void usb_bus_pump(void)
{
    if (s_state != USB_BUS_BUSY) return;
    const uint32_t now = HAL_GetTick();
    switch (s_op) {
    case USB_BUS_OP_SUSPEND:
        if (s_step == 0) {
            s_info.susprdy = s_info.sof_stopped = s_info.sof_resumed = false;
            USB_DRD_FS->CNTR |= USB_CNTR_SUSPEN;
            s_t = now;
            s_step = 1;
        } else if (s_step == 1) {
            if (USB_DRD_FS->CNTR & USB_CNTR_SUSPRDY) {
                s_info.susprdy = true;
                s_suspended = true;
                s_fn = frame_now();
                s_t = now;
                s_step = 2;
            } else if (now - s_t > SUSPRDY_MS) {
                USB_DRD_FS->CNTR &= ~USB_CNTR_SUSPEN;
                done(false);
            }
        } else if (now - s_t >= SOF_WATCH_MS) {
            s_info.sof_stopped = frame_now() == s_fn;
            done(true);
        }
        break;
    case USB_BUS_OP_RESUME:
        /* Not suspended (a check that left early, a suspend that failed): no signaling, but SUSPEN
           and L2RES are cleared and the frame counter is watched all the same. */
        if (s_step == 0 && !s_suspended) {
            s_info.sof_resumed = false;
            resume_regs(2);
            s_fn = frame_now();
            s_t = now;
            s_step = 2;
        } else if (s_step == 0) {
            resume_regs(0);
            s_t = now;
            s_step = 1;
        } else if (s_step == 1) {
            if (now - s_t >= RESUME_MS) {
                resume_regs(1);
                s_fn = frame_now();
                s_t = now;
                s_step = 2;
            }
        } else if (s_step == 2) {
            if (now - s_t >= RESUME_SOF_MS) {
                s_info.sof_resumed = frame_now() != s_fn;
                s_suspended = false;
                s_t = now;
                s_step = 3;
            }
        } else if (now - s_t >= RECOVERY_MS) {
            done(s_info.sof_resumed);
        }
        break;
    default:   /* USB_BUS_OP_RESET */
        /* An attach for the mounted device: TinyUSB drops it, resets the bus and enumerates again. */
        hcd_event_device_attach(0, false);
        done(true);
        break;
    }
}
