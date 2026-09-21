#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H

#include "usbd_cdc.h"

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* True once the host has configured the device (VCP open-able). */
int  cdc_if_connected(void);

/* Queue bytes for transmission to the host (best-effort, ring-buffered).
   Returns the number of bytes accepted (may be < len if the ring is full). */
int  cdc_if_write(const uint8_t *buf, uint16_t len);

#endif /* USBD_CDC_IF_H */
