/**
  ******************************************************************************
  * @file    LwIP/LwIP_TCP_Echo_Server/LWIP/Target/ethernetif.h
  * @author  MCD Application Team
  * @brief   Header for ethernetif.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__


#include "lwip/err.h"
#include "lwip/netif.h"

/* Exported types ------------------------------------------------------------*/
err_t ethernetif_init(struct netif *netif);
void ethernetif_input(struct netif *netif);
void ethernet_link_check_state(struct netif *netif);
/* Manual/self-heal link control (see ethernetif.c). Both must run on the net task
   (they touch HAL_ETH + lwIP netif state). */
void ethernetif_stop(struct netif *netif);
void ethernetif_phy_restart(struct netif *netif);
/* Force 10/100 Mbit (mbit = 10 or 100, full = duplex) or restore autoneg (mbit = 0).
   A debug aid for a suspect wired link: see the comment in ethernetif.c. */
int  ethernetif_force_speed(struct netif *netif, int mbit, int full);
/* Measure the PHY's RMII reference clock on PA1 (nominally 50 MHz) against the MCU crystal.
   0 = ok and *hz_out is set, -1 = no edges / failed. Drops the link for ~200 ms. */
int  ethernetif_measure_refclk(struct netif *netif, uint32_t *hz_out);

/* PHY near-end loopback test (see ethernetif.c). */
typedef struct {
  int      mbit;       /* 10 or 100 */
  uint32_t sent;       /* frames handed to the MAC */
  uint32_t tx_fail;    /* frames the MAC refused / timed out */
  uint32_t received;   /* frames that came back */
  uint32_t intact;     /* ... with the payload byte-for-byte as sent */
  uint32_t corrupt;    /* ... that came back but differ */
  uint32_t crc;        /* MAC CRC errors during the test (MMC delta) */
  uint32_t align;      /* MAC alignment errors during the test (MMC delta) */
} eth_loopback_result_t;
int  ethernetif_loopback_test(struct netif *netif, int mbit, uint32_t n, eth_loopback_result_t *r);

#include "eth_diag.h"
/* Wired-link diagnostics (net task only). */
void ethernetif_diag_refresh(struct netif *netif, eth_diag_t *d);
#endif
