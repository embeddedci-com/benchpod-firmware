#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

/* USB device (TinyUSB on USB_DRD_FS): the CDC-ACM console and a CDC-NCM network
 * adapter. TinyUSB runs on its own task; everything below is safe to call from
 * other tasks, and the console calls also from an ISR or before the scheduler. */

/* Clocks, pins and TinyUSB, then the usb task. Call before vTaskStartScheduler. */
void MX_USB_DEVICE_Init(void);

/* Disconnect from the bus for 0.5 s and reconnect, once the console has sent what it
   holds: the host enumerates the device again (console and network adapter). Any task. */
void usb_reattach_request(void);

/* ---- console (CDC-ACM) ---- */
/* True once the host has configured the device. */
int  cdc_if_connected(void);
/* Queue bytes for the host (ring-buffered, drops on a full ring). Returns bytes taken. */
int  cdc_if_write(const uint8_t *buf, uint16_t len);

/* ---- network (CDC-NCM), for the net task ---- */
#define USB_NET_FRAME_MAX  1514

/* The NCM data interface is up (host configured it and selected the data alt) and USB
   networking is on. */
bool usb_net_link_up(void);
/* USB networking on/off (on at boot). Off reports the link down to the host and drops
   frames; the console is unaffected. Any task. */
void usb_net_set_enabled(bool on);
bool usb_net_enabled(void);
/* The stored USB network setting (pod_policy_usb_net); off, the device enumerates without the
   NCM function. */
bool usb_net_setting_on(void);
/* The adapter is on: the setting is on and no Wi-Fi is configured. */
bool usb_net_on(void);
/* The oldest received frame, or false when none is waiting. Release it when done. */
bool usb_net_rx_peek(const uint8_t **frame, uint16_t *len);
void usb_net_rx_release(void);
/* A free transmit slot (USB_NET_FRAME_MAX bytes), waiting up to wait_ms for one while
   all are queued; NULL when none freed in time. */
uint8_t *usb_net_tx_slot(uint32_t wait_ms);
/* Queue the slot from usb_net_tx_slot() with `len` bytes of frame. */
void usb_net_tx_commit(uint16_t len);

/* MAC addresses from the UID: the pod's netif and the host's NCM function. */
void usb_net_dev_mac(uint8_t mac[6]);
void usb_net_host_mac(uint8_t mac[6]);

typedef struct {
    uint32_t rx_frames, rx_drops, tx_frames, tx_full;
} usb_net_stats_t;
void usb_net_stats(usb_net_stats_t *out);

#endif /* USB_DEVICE_H */
