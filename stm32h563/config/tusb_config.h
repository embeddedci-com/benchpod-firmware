/*
 * tusb_config.h — TinyUSB device configuration: USB_DRD_FS (full speed) with
 * CDC-ACM (the console) + CDC-NCM (the pod's API as a USB network adapter).
 *
 * Every TinyUSB call runs on the usb task (usb_device.c); the console and the
 * net task only touch rings and slots there, never TinyUSB itself.
 */
#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#define CFG_TUSB_MCU             OPT_MCU_STM32H5
#define CFG_TUSB_OS              OPT_OS_FREERTOS
#define CFG_TUSB_DEBUG           0
#define CFG_TUSB_MEM_ALIGN       __attribute__((aligned(4)))

#define BOARD_TUD_RHPORT         0
#define CFG_TUD_ENABLED          1
#define CFG_TUD_MAX_SPEED        OPT_MODE_FULL_SPEED
#define CFG_TUD_ENDPOINT0_SIZE   64

#define CFG_TUD_CDC              1
#define CFG_TUD_NCM              1

/* CDC-ACM console FIFOs (the console keeps its own 2 KB TX ring in front). */
#define CFG_TUD_CDC_RX_BUFSIZE   256
#define CFG_TUD_CDC_TX_BUFSIZE   256
#define CFG_TUD_CDC_EP_BUFSIZE   64

/* NCM transfer blocks. CDC-NCM 1.0 Table 6-4: 2048 is the minimum. 3200 holds two
   full frames, so one transfer carries two TCP segments; two IN NTBs keep the IN
   endpoint busy while the next one fills (ncm.h: up to 50% more on full speed). */
#define CFG_TUD_NCM_OUT_NTB_MAX_SIZE 3200
#define CFG_TUD_NCM_IN_NTB_MAX_SIZE  3200
#define CFG_TUD_NCM_OUT_NTB_N        1
#define CFG_TUD_NCM_IN_NTB_N         2

#endif /* TUSB_CONFIG_H */
