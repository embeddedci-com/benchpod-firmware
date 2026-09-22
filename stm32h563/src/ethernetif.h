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

#include "eth_diag.h"
/* Wired-link diagnostics (net task only). */
void ethernetif_diag_refresh(struct netif *netif, eth_diag_t *d);
#endif
