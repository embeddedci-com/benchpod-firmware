/*
 * usbd_conf.h — ST USB Device library configuration for the bench-pod
 * (STM32H563 USB_DRD_FS, single CDC-ACM virtual COM port).
 */
#ifndef USBD_CONF_H
#define USBD_CONF_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32h5xx_hal.h"

/* Device-library limits. */
#define USBD_MAX_NUM_INTERFACES        1U
#define USBD_MAX_NUM_CONFIGURATION     1U
#define USBD_MAX_STR_DESC_SIZ          512U
#define USBD_DEBUG_LEVEL               0U
#define USBD_SELF_POWERED              1U
#define DEVICE_FS                      0

/* Memory management: the library uses a single static allocation. */
void *USBD_static_malloc(uint32_t size);
void  USBD_static_free(void *p);

#define USBD_malloc         USBD_static_malloc
#define USBD_free           USBD_static_free
#define USBD_memset         memset
#define USBD_memcpy         memcpy
#define USBD_Delay          HAL_Delay

/* Debug macros (no-ops). */
#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)

#endif /* USBD_CONF_H */
