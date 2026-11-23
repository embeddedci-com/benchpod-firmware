#ifndef USB_BUS_H
#define USB_BUS_H

/* usb_bus — bus-level actions on the pod's USB host port for usb_check (USB_HOST=1 builds):
 * suspend, resume and a port reset of the hosted board.
 *
 * TinyUSB's host has no suspend or resume, and its stm32_fsdev driver never enables the wake-up
 * interrupt. The peripheral does it in hardware: setting SUSPEN in USB_CNTR stops the SOFs (the
 * board enters suspend after 3 ms of bus idle) and SUSPRDY tells when that took effect; resume
 * drives K signaling with L2RES for 20 ms, then the SOFs start again. This runs from the pod's own
 * code on the usb task (usb_bus_pump), so the pinned TinyUSB submodule stays unpatched. A port
 * reset is TinyUSB's own re-enumeration: an attach event for the mounted device makes it drop the
 * device, reset the bus and enumerate again.
 *
 * The hw worker posts one request and polls its state; the usb task runs it in steps that never
 * wait (usb_bus_pump, every pass of the host loop). */

#include <stdbool.h>
#include <stdint.h>

typedef enum { USB_BUS_OP_SUSPEND = 1, USB_BUS_OP_RESUME, USB_BUS_OP_RESET } usb_bus_op_t;
typedef enum { USB_BUS_IDLE = 0, USB_BUS_BUSY, USB_BUS_OK, USB_BUS_FAIL } usb_bus_state_t;

typedef struct {
    bool susprdy;       /* SUSPRDY came up after SUSPEN */
    bool sof_stopped;   /* the frame number stood still for 5 ms in suspend */
    bool sof_resumed;   /* it counted again after the resume */
} usb_bus_info_t;

/* Post a request. 0 = accepted; -1 = one is running, or the bus is in the wrong state (a suspend
   while suspended, a reset while suspended). hw worker only. */
int usb_bus_request(usb_bus_op_t op);
/* IDLE before any request; BUSY while it runs; OK or FAIL stays until the next request. */
usb_bus_state_t usb_bus_state(usb_bus_info_t *info);
bool usb_bus_suspended(void);
/* usb task, every pass of its host loop. */
void usb_bus_pump(void);
/* The usb task started or left host mode: forget everything. */
void usb_bus_role_changed(void);

#endif /* USB_BUS_H */
