/*
 * tusb_config.h — TinyUSB on USB_DRD_FS (full speed). Device: CDC-ACM (the console) +
 * CDC-NCM (the pod's API as a USB network adapter). Host (usb_host.h, on demand only): the
 * same port hosts one DUT, enumeration only so far. One role at a time: the usb task tears
 * one stack down before it starts the other (usb_device.c).
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


/* Host (USB_HOST=1 builds only): one device straight on the port, no hub, mass storage and serial class drivers (the pod reads
   the other descriptors itself). The enumeration buffer holds a configuration descriptor: a longer
   one still enumerates, only the interface list in usb_host is cut. */
#if defined(BENCHPOD_USB_HOST)
#define BOARD_TUH_RHPORT         0
#define CFG_TUH_ENABLED          1
#define CFG_TUH_MAX_SPEED        OPT_MODE_FULL_SPEED
#define CFG_TUH_ENUMERATION_BUFSIZE 512
#define CFG_TUH_DEVICE_MAX       1
#define CFG_TUH_HUB              0
#define CFG_TUH_ENDPOINT_MAX     4
#define CFG_TUH_MSC              1   /* a USB drive to flash (usb_msc.h, usb_flash.h) */
#define CFG_TUH_CDC              1   /* a USB serial port to flash over (usb_serial.h, usb_flash.h) */
#define CFG_TUH_CDC_FTDI         1   /* boards with a USB-serial chip instead of native USB */
#define CFG_TUH_CDC_CP210X       1
#define CFG_TUH_CDC_CH34X        1
#define CFG_TUH_CDC_RX_BUFSIZE   256
#define CFG_TUH_CDC_TX_BUFSIZE   256
/* 115200 8N1 when a serial port mounts (the CH34x driver takes it for its init too); the DTR/RTS
   lines stay low (no reset) until a flash sets them. */
#define CFG_TUH_CDC_LINE_CODING_ON_ENUM       { 115200, CDC_LINE_CODING_STOP_BITS_1, CDC_LINE_CODING_PARITY_NONE, 8 }
#endif

#endif /* TUSB_CONFIG_H */
