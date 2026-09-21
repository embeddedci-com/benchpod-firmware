#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Initialise the USB device, register the CDC class + interface, and start. */
void MX_USB_DEVICE_Init(void);

#endif /* USB_DEVICE_H */
