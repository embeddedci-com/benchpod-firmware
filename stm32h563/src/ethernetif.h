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
#endif
