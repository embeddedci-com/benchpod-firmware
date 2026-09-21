/*
 * usb_device.c — bring up the USB device core + CDC class.
 */
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_DRD_FS;   /* defined in usbd_conf.c */

void MX_USB_DEVICE_Init(void)
{
    if (USBD_Init(&hUsbDeviceFS, &CDC_Desc, DEVICE_FS) != USBD_OK) return;
    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK) return;
    if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK) return;
    USBD_Start(&hUsbDeviceFS);

    /* Force the host to re-enumerate after a bootloader (DFU) hand-off.  The ROM
       USB bootloader leaves D+ pulled up / enumerated; when we jump to the app
       the host keeps that stale enumeration and our CDC virtual COM port never
       appears until a full power-cycle.  Pulsing the D+ pull-up off→on makes the
       host see a disconnect + reconnect and enumerate the app cleanly. */
    HAL_PCD_DevDisconnect(&hpcd_USB_DRD_FS);
    HAL_Delay(50);
    HAL_PCD_DevConnect(&hpcd_USB_DRD_FS);
}
