/**
  ******************************************************************************
  * @file    LwIP/LwIP_TCP_Echo_Server/LWIP/Target/ethernetif.c
  * @author  MCD Application Team
  * @brief   This file implements Ethernet network interface drivers for lwIP
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

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "lwip/netif.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "board_uid.h"
#include "lan8742.h"
#include "board_pins.h"
#include "eth_diag.h"
#include <stdio.h>
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'

#define ETH_DMA_TRANSMIT_TIMEOUT                (20U)

/* One full Ethernet frame per RX buffer (was 1000 < a 1514-byte frame, which split every
   full packet across TWO DMA buffers — halving effective RX buffering and adding reassembly).
   A bigger pool (16 vs 12) leaves headroom above ETH_RX_DESC_CNT(4) DMA-armed buffers for an
   8-segment TCP window (TCP_WND) held by lwIP, so a cloud download isn't RX-starved into drops.
   RAM: +~12 KB. */
#define ETH_RX_BUFFER_SIZE            1536U
#define ETH_RX_BUFFER_CNT             16U
#define ETH_TX_BUFFER_MAX             ((ETH_TX_DESC_CNT) * 2U)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*
@Note: This interface is implemented to operate in zero-copy mode only:
        - Rx Buffers will be allocated from LwIP stack Rx memory pool,
          then passed to ETH HAL driver.
        - Tx Buffers will be allocated from LwIP stack memory heap,
          then passed to ETH HAL driver.

@Notes:
  1.a. ETH DMA Rx descriptors must be contiguous, the default count is 4,
       to customize it please redefine ETH_RX_DESC_CNT in ETH GUI (Rx Descriptor Length)
       so that updated value will be generated in stm32xxxx_hal_conf.h
  1.b. ETH DMA Tx descriptors must be contiguous, the default count is 4,
       to customize it please redefine ETH_TX_DESC_CNT in ETH GUI (Tx Descriptor Length)
       so that updated value will be generated in stm32xxxx_hal_conf.h

  2.a. Rx Buffers number: ETH_RX_BUFFER_CNT must be greater than ETH_RX_DESC_CNT.
  2.b. Rx Buffers must have the same size: ETH_RX_BUFFER_SIZE, this value must
       passed to ETH DMA in the init field (heth.Init.RxBuffLen)
*/
typedef enum
{
  RX_ALLOC_OK       = 0x00,
  RX_ALLOC_ERROR    = 0x01
} RxAllocStatusTypeDef;

typedef struct
{
  struct pbuf_custom pbuf_custom;
  uint8_t buff[(ETH_RX_BUFFER_SIZE + 31) & ~31] __ALIGNED(32);
} RxBuff_t;

ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

/* Memory Pool Declaration */
LWIP_MEMPOOL_DECLARE(RX_POOL, ETH_RX_BUFFER_CNT, sizeof(RxBuff_t), "Zero-copy RX PBUF pool");



/* Variable Definitions */
static uint8_t RxAllocStatus;
static volatile uint32_t s_rx_alloc_fail;   /* RX pool empty events (eth_diag) */
static uint32_t s_rx_missed, s_rx_missed_ovf;  /* accumulated: DMACMFCR clears on read */
static volatile uint32_t s_rx_frames, s_rx_bcast;  /* every received frame (the MMC counts unicast only) */

_Static_assert(ETH_DIAG_MACCR_DM == ETH_MACCR_DM && ETH_DIAG_MACCR_FES == ETH_MACCR_FES,
               "eth_diag MACCR bits must match the CMSIS definitions");

/* Global Ethernet handle*/
ETH_HandleTypeDef EthHandle;
ETH_TxPacketConfig TxConfig;


/* Private function prototypes -----------------------------------------------*/
u32_t sys_now(void);

int32_t ETH_PHY_IO_Init(void);
int32_t ETH_PHY_IO_DeInit (void);
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_IO_GetTick(void);

lan8742_Object_t LAN8742;
lan8742_IOCtx_t  LAN8742_IOCtx = {ETH_PHY_IO_Init,
                               ETH_PHY_IO_DeInit,
                               ETH_PHY_IO_WriteReg,
                               ETH_PHY_IO_ReadReg,
                               ETH_PHY_IO_GetTick};


/* Private functions ---------------------------------------------------------*/
void pbuf_free_custom(struct pbuf *p);
/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
  * @brief In this function, the hardware should be initialized.
  * Called from ethernetif_init().
  *
  * @param netif the already initialized lwip network interface structure
  *        for this ethernetif
  */
/* The MAC address, unique per pod: a locally administered unicast address (first byte 0x02)
   whose other five bytes hash the STM32's 96-bit unique device ID.  A fixed address made every
   pod on a LAN share one MAC, so two pods fought over a single DHCP lease.  Stable across
   reboots and reflashes (the UID is factory-programmed).
   The UID is read as whole words through board_uid.h. */
static void board_eth_mac(uint8_t mac[6])
{
  uint32_t w[3];
  board_uid_words(w);   /* whole words: see board_uid.h */
  uint32_t h = 2166136261u;                    /* FNV-1a over the 12 UID bytes */
  for (int i = 0; i < 3; i++) {
    for (int b = 0; b < 4; b++) {
      h ^= (w[i] >> (8 * b)) & 0xFFu;
      h *= 16777619u;
    }
  }
  uint32_t h2 = (h ^ 0x45u) * 16777619u;       /* one more round for the fifth byte */
  mac[0] = 0x02;
  mac[1] = (uint8_t)(h >> 24);
  mac[2] = (uint8_t)(h >> 16);
  mac[3] = (uint8_t)(h >> 8);
  mac[4] = (uint8_t)h;
  mac[5] = (uint8_t)(h2 >> 24);
}

static void low_level_init(struct netif *netif)
{
  static uint8_t macaddress[6];   /* HAL keeps the pointer: must outlive this call */
  board_eth_mac(macaddress);

  EthHandle.Instance = ETH;
  EthHandle.Init.MACAddr = macaddress;
  EthHandle.Init.MediaInterface = HAL_ETH_RMII_MODE;
  EthHandle.Init.RxDesc = DMARxDscrTab;
  EthHandle.Init.TxDesc = DMATxDscrTab;
  EthHandle.Init.RxBuffLen = ETH_RX_BUFFER_SIZE;

  /* configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
  HAL_ETH_Init(&EthHandle);

  /* set MAC hardware address length */
  netif->hwaddr_len = ETH_HWADDR_LEN;

  /* set MAC hardware address */
  for (int i = 0; i < 6; i++)
    netif->hwaddr[i] = macaddress[i];

  /* maximum transfer unit */
  netif->mtu = ETH_MAX_PAYLOAD;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  /* NETIF_FLAG_IGMP lets LwIP join the mDNS multicast group (224.0.0.251). */
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP;

  /* Let the MAC receive multicast frames so mDNS queries (dst 01:00:5E:00:00:FB)
     actually reach LwIP — the default filter after HAL_ETH_Init drops them. */
  {
    ETH_MACFilterConfigTypeDef filter = {0};
    HAL_ETH_GetMACFilterConfig(&EthHandle, &filter);
    filter.PassAllMulticast = ENABLE;
    HAL_ETH_SetMACFilterConfig(&EthHandle, &filter);
  }

  /* Initialize the RX POOL */
  LWIP_MEMPOOL_INIT(RX_POOL);

  /* Set Tx packet config common parameters */
  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

  /* Set PHY IO functions */
  LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);

  /* Initialize the LAN8742 ETH PHY */
  if(LAN8742_Init(&LAN8742) != LAN8742_STATUS_OK)
  {
    netif_set_link_down(netif);
    netif_set_down(netif);
    return;
  }

  /* Belt-and-suspenders MDIO software reset (BCR bit 15) after the hardware nRST
     pulse in HAL_ETH_MspInit.  A software NVIC reboot (console `reboot`, a
     watchdog/fault reset, a DFU `:leave`) must recover the link WITHOUT a power
     cycle; issuing the PHY's own reset here makes bring-up self-sufficient even
     if the board's nRST alone leaves the PHY in a wedged state (it clears the
     autoneg/PLL state the hardware pulse might not).  The bit self-clears when
     the reset completes; autoneg is re-driven by ethernet_link_check_state. */
  {
    uint32_t bcr = 0;
    if (HAL_ETH_WritePHYRegister(&EthHandle, LAN8742.DevAddr,
                                 LAN8742_BCR, LAN8742_BCR_SOFT_RESET) == HAL_OK) {
      uint32_t t0 = HAL_GetTick();
      do {
        if (HAL_ETH_ReadPHYRegister(&EthHandle, LAN8742.DevAddr,
                                    LAN8742_BCR, &bcr) != HAL_OK) break;
      } while ((bcr & LAN8742_BCR_SOFT_RESET) && (HAL_GetTick() - t0 < 1000u));
    }
  }

  ethernet_link_check_state(netif);
}

/**
  * @brief This function should do the actual transmission of the packet. The packet is
  * contained in the pbuf that is passed to the function. This pbuf
  * might be chained.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
  * @return ERR_OK if the packet could be sent
  *         an err_t value if the packet couldn't be sent
  *
  * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
  *       strange results. You might consider waiting for space in the DMA queue
  *       to become available since the stack doesn't retry to send a packet
  *       dropped because of memory failure (except for the TCP timers).
  */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  uint32_t i = 0U;
  struct pbuf *q = NULL;
  err_t errval = ERR_OK;
  ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT] = {0};

  memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));

  for(q = p; q != NULL; q = q->next)
  {
    if(i >= ETH_TX_DESC_CNT)
      return ERR_IF;

    Txbuffer[i].buffer = q->payload;
    Txbuffer[i].len = q->len;

    if(i>0)
    {
      Txbuffer[i-1].next = &Txbuffer[i];
    }

    if(q->next == NULL)
    {
      Txbuffer[i].next = NULL;
    }

    i++;
  }

  TxConfig.Length = p->tot_len;
  TxConfig.TxBuffer = Txbuffer;
  TxConfig.pData = p;

  if( HAL_ETH_Transmit(&EthHandle, &TxConfig, ETH_DMA_TRANSMIT_TIMEOUT)== HAL_OK )
  {
    errval = ERR_OK;
  }
  else
  {
    errval = ERR_IF;
  }

  return errval;
}


/**
  * @brief Should allocate a pbuf and transfer the bytes of the incoming
  * packet from the interface into the pbuf.
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return a pbuf filled with the received packet (including MAC header)
  *         NULL on memory error
  */
static struct pbuf * low_level_input(struct netif *netif)
{
  struct pbuf *p = NULL;

  if(RxAllocStatus == RX_ALLOC_OK)
  {
    HAL_ETH_ReadData(&EthHandle, (void **)&p);
  }
  if (p != NULL)
  {
    const uint8_t *dst = (const uint8_t *)p->payload;
    s_rx_frames++;
    if (p->len >= 6 && (dst[0] & dst[1] & dst[2] & dst[3] & dst[4] & dst[5]) == 0xFF) s_rx_bcast++;
  }
  return p;

}

/**
  * @brief This function is the ethernetif_input task, it is processed when a packet
  * is ready to be read from the interface. It uses the function low_level_input()
  * that should handle the actual reception of bytes from the network
  * interface. Then the type of the received packet is determined and
  * the appropriate input function is called.
  *
  * @param netif the lwip network interface structure for this ethernetif
  */
void ethernetif_input(struct netif *netif)
{
  struct pbuf *p = NULL;

    do
    {
      p = low_level_input( netif );
      if (p != NULL)
      {
        if (netif->input( p, netif) != ERR_OK )
        {
          pbuf_free(p);
        }
      }

    } while(p!=NULL);

}

/**
  * @brief Should be called at the beginning of the program to set up the
  * network interface. It calls the function low_level_init() to do the
  * actual setup of the hardware.
  *
  * This function should be passed as a parameter to netif_add().
  *
  * @param netif the lwip network interface structure for this ethernetif
  * @return ERR_OK if the loopif is initialized
  *         ERR_MEM if private data couldn't be allocated
  *         any other err_t on error
  */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}

/**
  * @brief  Custom Rx pbuf free callback
  * @param  pbuf: pbuf to be freed
  * @retval None
  */
void pbuf_free_custom(struct pbuf *p)
{
  struct pbuf_custom* custom_pbuf = (struct pbuf_custom*)p;
  LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);
   /* If the Rx Buffer Pool was exhausted, signal the ethernetif_input task to
   * call HAL_ETH_GetRxDataBuffer to rebuild the Rx descriptors. */
  if (RxAllocStatus == RX_ALLOC_ERROR)
  {
    RxAllocStatus = RX_ALLOC_OK;
  }
}

/**
  * @brief  Returns the current time in milliseconds
  *         when LWIP_TIMERS == 1 and NO_SYS == 1
  * @param  None
  * @retval Current Time value
  */
u32_t sys_now(void)
{
  return HAL_GetTick();
}
/*******************************************************************************
                       Ethernet MSP Routines
*******************************************************************************/
/**
  * @brief  Initializes the ETH MSP.
  * @param  heth: ETH handle
  * @retval None
  */
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};
 if(heth->Instance==ETH)
 {
   /* USER CODE BEGIN ETH_MspInit 0 */

   /* USER CODE END ETH_MspInit 0 */
   /* SBS clock — HAL_ETH_Init uses HAL_SBS_ETHInterfaceSelect() to pick RMII. */
   __HAL_RCC_SBS_CLK_ENABLE();

   /* Peripheral clock enable */
   __HAL_RCC_ETH_CLK_ENABLE();
   __HAL_RCC_ETHTX_CLK_ENABLE();
   __HAL_RCC_ETHRX_CLK_ENABLE();

   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOG_CLK_ENABLE();
   __HAL_RCC_GPIOE_CLK_ENABLE();

   /* Reset the LAN8742 PHY (active-low nRST on PE2): hold low, release, settle
      before any MDIO access.  (Not present on the Nucleo; our board wires it.) */
   GPIO_InitStruct.Pin = RMII_NRST_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(RMII_NRST_PORT, &GPIO_InitStruct);
   HAL_GPIO_WritePin(RMII_NRST_PORT, RMII_NRST_PIN, GPIO_PIN_RESET);
   HAL_Delay(5);
   HAL_GPIO_WritePin(RMII_NRST_PORT, RMII_NRST_PIN, GPIO_PIN_SET);
   HAL_Delay(50);
   /**ETH GPIO Configuration
   PC1     ------> ETH_MDC
   PA1     ------> ETH_REF_CLK
   PA2     ------> ETH_MDIO
   PA7     ------> ETH_CRS_DV
   PC4     ------> ETH_RXD0
   PC5     ------> ETH_RXD1
   PB15     ------> ETH_TXD1
   PG11     ------> ETH_TX_EN
   PG13     ------> ETH_TXD0
   */
   /* VERY_HIGH, not LOW: RMII runs at 50 MHz and the PHY samples TXD0/TXD1/TX_EN within a
      20 ns period (LAN8742 REF_CLK-out mode: ~4 ns setup). The LOW slew rate this was
      generated with left transmit timing marginal on every board and broke it outright on
      one: 100M transmit failed (PHY loopback returned 0 of 200 frames) while 10M, where each
      dibit is held for 10 clocks, and receive, where these pins are inputs, both worked. */
   GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_15;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
   HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

   /* ETH interrupt Init */
   HAL_NVIC_SetPriority(ETH_IRQn, 7, 0);
   HAL_NVIC_EnableIRQ(ETH_IRQn);
   /* USER CODE BEGIN ETH_MspInit 1 */

   /* USER CODE END ETH_MspInit 1 */
  }
}

/*******************************************************************************
                       PHI IO Functions
*******************************************************************************/
/**
  * @brief  Initializes the MDIO interface GPIO and clocks.
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_Init(void)
{
  /* We assume that MDIO GPIO configuration is already done
     in the ETH_MspInit() else it should be done here
  */

  /* Configure the MDIO Clock */
  HAL_ETH_SetMDIOClockRange(&EthHandle);

  return 0;
}

/**
  * @brief  De-Initializes the MDIO interface .
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_DeInit (void)
{
  return 0;
}

/**
  * @brief  Read a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  pRegVal: pointer to hold the register value
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
  if(HAL_ETH_ReadPHYRegister(&EthHandle, DevAddr, RegAddr, pRegVal) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

/**
  * @brief  Write a value to a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  RegVal: Value to be written
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
  if(HAL_ETH_WritePHYRegister(&EthHandle, DevAddr, RegAddr, RegVal) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

/**
  * @brief  Get the time in millisecons used for internal PHY driver process.
  * @retval Time value
  */
int32_t ETH_PHY_IO_GetTick(void)
{
  return HAL_GetTick();
}

/**
  * @brief
  * @retval None
  */
void ethernet_link_check_state(struct netif *netif)
{
  ETH_MACConfigTypeDef MACConf = {0};
  int32_t PHYLinkState = 0U;
  uint32_t linkchanged = 0U, speed = 0U, duplex =0U;

  PHYLinkState = LAN8742_GetLinkState(&LAN8742);

  if(netif_is_link_up(netif) && (PHYLinkState <= LAN8742_STATUS_LINK_DOWN))
  {
    HAL_ETH_Stop(&EthHandle);
    netif_set_down(netif);
    netif_set_link_down(netif);
  }
  else if(!netif_is_link_up(netif) && (PHYLinkState > LAN8742_STATUS_LINK_DOWN))
  {
    switch (PHYLinkState)
    {
    case LAN8742_STATUS_100MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case LAN8742_STATUS_100MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_100M;
      linkchanged = 1;
      break;
    case LAN8742_STATUS_10MBITS_FULLDUPLEX:
      duplex = ETH_FULLDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    case LAN8742_STATUS_10MBITS_HALFDUPLEX:
      duplex = ETH_HALFDUPLEX_MODE;
      speed = ETH_SPEED_10M;
      linkchanged = 1;
      break;
    default:
      break;
    }

    if(linkchanged)
    {
      /* Get MAC Config MAC */
      HAL_ETH_GetMACConfig(&EthHandle, &MACConf);
      MACConf.DuplexMode = duplex;
      MACConf.Speed = speed;
      HAL_ETH_SetMACConfig(&EthHandle, &MACConf);
      HAL_ETH_Start(&EthHandle);
      netif_set_up(netif);
      netif_set_link_up(netif);
      uint32_t anlpar = 0;
      (void)HAL_ETH_ReadPHYRegister(&EthHandle, LAN8742.DevAddr, LAN8742_ANLPAR, &anlpar);
      printf("[net] eth link up: %s%s%s, partner 0x%04lx\r\n",
             speed == ETH_SPEED_100M ? "100M" : "10M",
             duplex == ETH_FULLDUPLEX_MODE ? " full" : " HALF",
             duplex == ETH_FULLDUPLEX_MODE ? "" : " duplex (a full-duplex switch port would mean a mismatch)",
             (unsigned long)anlpar);
    }
  }

}

/* Administratively halt the wired MAC and mark the interface down. Paired with
   ethernetif_phy_restart() to bring it back. Used by the `eth` console/JSON
   command (manual stop) — while stopped, net_server.c skips the link check so the
   interface stays down until an explicit start. */
void ethernetif_stop(struct netif *netif)
{
  HAL_ETH_Stop(&EthHandle);
  netif_set_down(netif);
  netif_set_link_down(netif);
}

/* Issue the LAN8742's own soft reset (BCR bit 15), then mark the link down so the
   next ethernet_link_check_state() re-negotiates and restarts the MAC from
   scratch. This clears a wedged PHY autoneg/PLL state that a stuck DHCP or a flaky
   link can leave behind — a full recovery WITHOUT a power cycle. Used by the DHCP
   self-heal escalation and the `eth start`/`eth restart` command. */
void ethernetif_phy_restart(struct netif *netif)
{
  HAL_ETH_Stop(&EthHandle);
  uint32_t bcr = 0;
  if (HAL_ETH_WritePHYRegister(&EthHandle, LAN8742.DevAddr,
                               LAN8742_BCR, LAN8742_BCR_SOFT_RESET) == HAL_OK) {
    uint32_t t0 = HAL_GetTick();
    do {
      if (HAL_ETH_ReadPHYRegister(&EthHandle, LAN8742.DevAddr,
                                  LAN8742_BCR, &bcr) != HAL_OK) break;
    } while ((bcr & LAN8742_BCR_SOFT_RESET) && (HAL_GetTick() - t0 < 1000u));
  }
  netif_set_down(netif);
  netif_set_link_down(netif);
}

/* PHY near-end loopback: the LAN8742 returns every frame the MAC transmits straight back
   to the MAC from its DIGITAL side, so the frame crosses the RMII transmit lines (TXD0,
   TXD1, TX_EN), the PHY's digital logic and the RMII receive lines, and never reaches the
   PHY's analog front end, the magnetics, the RJ45 or the cable.

   That splits a link that fails at 100M and works at 10M in two:
     - loopback also fails at 100M  -> RMII lines or the PHY's digital side. At 100M every
       50 MHz clock carries new data; at 10M each dibit is held for 10 clocks, so a marginal
       TXD/TX_EN joint or timing is fatal at 100M and invisible at 10M.
     - loopback is clean at 100M    -> the digital path is fine; the fault is analog
       (RBIAS, analog supply decoupling, magnetics, jack).

   Each frame is addressed to our own MAC with a local-experimental ethertype, carries its
   sequence number and a bit-toggling pattern, and is compared byte for byte on return.
   The link is down for the duration; the PHY is soft-reset (autoneg) afterwards. */
#define ETH_LB_ETHERTYPE  0x88B5u
#define ETH_LB_PAYLOAD    1000u
#define ETH_LB_FRAME      (14u + ETH_LB_PAYLOAD)

static uint8_t eth_lb_byte(uint32_t seq, uint32_t i)
{
  return (uint8_t)((i * 7u + seq) ^ ((i & 1u) ? 0xAAu : 0x55u));
}

int ethernetif_loopback_test(struct netif *netif, int mbit, uint32_t n, eth_loopback_result_t *r)
{
  if (!r) return -1;
  memset(r, 0, sizeof(*r));
  r->mbit = mbit;

  HAL_ETH_Stop(&EthHandle);
  netif_set_down(netif);
  netif_set_link_down(netif);

  uint32_t bcr = LAN8742_BCR_LOOPBACK | LAN8742_BCR_DUPLEX_MODE |
                 (mbit == 100 ? LAN8742_BCR_SPEED_SELECT : 0u);
  if (HAL_ETH_WritePHYRegister(&EthHandle, LAN8742.DevAddr, LAN8742_BCR, bcr) != HAL_OK) {
    ethernetif_phy_restart(netif);
    return -1;
  }
  HAL_Delay(20);

  ETH_MACConfigTypeDef mac = {0};
  HAL_ETH_GetMACConfig(&EthHandle, &mac);
  mac.DuplexMode = ETH_FULLDUPLEX_MODE;
  mac.Speed      = (mbit == 100) ? ETH_SPEED_100M : ETH_SPEED_10M;
  HAL_ETH_SetMACConfig(&EthHandle, &mac);
  HAL_ETH_Start(&EthHandle);

  /* Drain anything already queued so it is not counted as a loopback frame. */
  for (struct pbuf *q; (q = low_level_input(netif)) != NULL; ) pbuf_free(q);

  uint32_t crc0 = ETH->MMCRCRCEPR, align0 = ETH->MMCRAEPR;

  for (uint32_t seq = 0; seq < n; seq++) {
    struct pbuf *p = pbuf_alloc(PBUF_RAW, ETH_LB_FRAME, PBUF_RAM);
    if (!p) { r->tx_fail++; continue; }
    uint8_t *f = (uint8_t *)p->payload;
    memcpy(f, netif->hwaddr, 6);
    memcpy(f + 6, netif->hwaddr, 6);
    f[12] = (uint8_t)(ETH_LB_ETHERTYPE >> 8);
    f[13] = (uint8_t)(ETH_LB_ETHERTYPE & 0xFFu);
    for (uint32_t i = 0; i < ETH_LB_PAYLOAD; i++) f[14 + i] = eth_lb_byte(seq, i);
    f[14] = (uint8_t)(seq >> 8);
    f[15] = (uint8_t)(seq & 0xFFu);

    r->sent++;
    if (low_level_output(netif, p) != ERR_OK) r->tx_fail++;
    pbuf_free(p);

    /* A 1 KB frame takes ~0.8 ms at 10M; give it 5 ms to come back. */
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < 5u) {
      struct pbuf *q = low_level_input(netif);
      if (!q) continue;
      const uint8_t *g = (const uint8_t *)q->payload;
      if (q->len >= 14u && g[12] == (uint8_t)(ETH_LB_ETHERTYPE >> 8) &&
          g[13] == (uint8_t)(ETH_LB_ETHERTYPE & 0xFFu)) {
        r->received++;
        bool ok = (q->tot_len >= ETH_LB_FRAME) && (q->len >= ETH_LB_FRAME) &&
                  g[14] == (uint8_t)(seq >> 8) && g[15] == (uint8_t)(seq & 0xFFu);
        for (uint32_t i = 2; ok && i < ETH_LB_PAYLOAD; i++)
          if (g[14 + i] != eth_lb_byte(seq, i)) ok = false;
        if (ok) r->intact++; else r->corrupt++;
        pbuf_free(q);
        break;
      }
      pbuf_free(q);
    }
  }

  r->crc   = ETH->MMCRCRCEPR - crc0;
  r->align = ETH->MMCRAEPR   - align0;

  HAL_ETH_Stop(&EthHandle);
  ethernetif_phy_restart(netif);     /* soft reset: loopback off, autoneg back on */
  return 0;
}

/* Measure the RMII reference clock the PHY drives into PA1 (nominally 50 MHz) against the
   MCU's own 25 MHz crystal, and report it in Hz.

   Why this is worth a command: 100BASE-TX needs the transmit reference within +/-50 ppm,
   while 10BASE-T tolerates +/-100 ppm and in practice much more. A PHY whose reference is
   off therefore fails at 100M and works at 10M, and it fails on TRANSMIT only, because the
   receive path recovers its clock from the wire. That is indistinguishable from damaged
   magnetics by looking at the MAC's counters, but it is obvious here.

   Method: PA1 is temporarily re-muxed from ETH_REF_CLK to TIM5_CH2, TIM5 counts its edges
   for a gate timed by the DWT cycle counter (the CPU clock, so the MCU crystal), and the
   pins and the link are put back. The link drops for the ~200 ms this takes.

   It measures the two oscillators AGAINST EACH OTHER, so compare a suspect pod with a
   known-good one rather than reading one number in isolation. If the MCU fell back to the
   internal HSI (no crystal) the reference is worthless: the caller reports that. */
int ethernetif_measure_refclk(struct netif *netif, uint32_t *hz_out)
{
  if (!hz_out) return -1;
  *hz_out = 0;

  HAL_ETH_Stop(&EthHandle);
  netif_set_down(netif);
  netif_set_link_down(netif);

  /* PA1: ETH_REF_CLK (AF11) -> TIM5_CH2 (AF2). The PHY keeps driving it either way.
     NOT TIM2 (AF1): TIM2 is the firmware's free-running microsecond clock (port/pico_compat.c),
     and reprogramming it here stopped every deadline, ping and timeout until reboot. */
  GPIO_InitTypeDef g = {0};
  g.Pin       = GPIO_PIN_1;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &g);

  __HAL_RCC_TIM5_CLK_ENABLE();
  TIM5->CR1   = 0;
  TIM5->PSC   = 0;
  TIM5->ARR   = 0xFFFFFFFFu;
  TIM5->CCMR1 = (1u << 8);                    /* CC2S = 01: IC2 on TI2 (PA1), no filter */
  TIM5->CCER  = 0;                            /* TI2 rising edge */
  TIM5->SMCR  = (6u << TIM_SMCR_TS_Pos) |     /* TS = 110: trigger = TI2FP2 */
                (7u << TIM_SMCR_SMS_Pos);     /* SMS = 111: external clock mode 1 */
  TIM5->EGR   = TIM_EGR_UG;
  TIM5->CNT   = 0;
  TIM5->CR1   = TIM_CR1_CEN;

  /* ~200 ms gate: 10 M edges at 50 MHz, so one count of quantisation is 0.1 ppm. */
  const uint32_t gate_cycles = SystemCoreClock / 5u;
  uint32_t t0 = DWT->CYCCNT;
  uint32_t c0 = TIM5->CNT;
  while ((DWT->CYCCNT - t0) < gate_cycles) { /* busy-wait: the gate must not be preempted */ }
  uint32_t c1      = TIM5->CNT;
  uint32_t elapsed = DWT->CYCCNT - t0;

  TIM5->CR1  = 0;
  TIM5->SMCR = 0;
  __HAL_RCC_TIM5_CLK_DISABLE();

  g.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &g);

  uint32_t edges = c1 - c0;
  if (elapsed == 0) { ethernetif_phy_restart(netif); return -1; }
  *hz_out = (uint32_t)(((uint64_t)edges * (uint64_t)SystemCoreClock) / (uint64_t)elapsed);

  ethernetif_phy_restart(netif);              /* re-negotiate, DHCP re-acquires */
  return (edges == 0) ? -1 : 0;               /* no edges = the PHY is not clocking PA1 */
}

/* Force the PHY's link mode instead of letting it autonegotiate, or put it back on
   autoneg (mbit = 0).  A DEBUG AID: 10BASE-T swings ~5x the voltage of 100BASE-TX,
   runs at a quarter of the symbol rate and tolerates a far looser reference clock, so
   a link that is lossy at 100M and clean at 10M points at the analog path (magnetics,
   RJ45, the PHY's 25 MHz reference) rather than the MAC, the driver or the network.

   A forced link does NOT autonegotiate, so a switch port that does will fall back to
   parallel detection and pick HALF duplex — which is why half is the sane default here.
   Forcing full against such a port is a duplex mismatch and produces late collisions
   and its own losses, so it stays opt-in.

   Returns 0 on success, -1 if the MDIO write failed. Net task only. */
int ethernetif_force_speed(struct netif *netif, int mbit, int full)
{
  uint32_t bcr = (mbit == 0) ? (LAN8742_BCR_AUTONEGO_EN | LAN8742_BCR_RESTART_AUTONEGO)
                             : ((mbit == 100 ? LAN8742_BCR_SPEED_SELECT : 0u) |
                                (full        ? LAN8742_BCR_DUPLEX_MODE  : 0u));
  HAL_ETH_Stop(&EthHandle);
  if (HAL_ETH_WritePHYRegister(&EthHandle, LAN8742.DevAddr, LAN8742_BCR, bcr) != HAL_OK) {
    netif_set_down(netif);
    netif_set_link_down(netif);
    return -1;
  }
  /* Mark the link down so the next ethernet_link_check_state() re-reads the PHY and
     reconfigures the MAC for whatever it now reports (the LAN8742 driver derives the
     mode from BCR when autoneg is off, so a forced link comes back up by itself). */
  netif_set_down(netif);
  netif_set_link_down(netif);
  return 0;
}

void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
  struct pbuf_custom *p = LWIP_MEMPOOL_ALLOC(RX_POOL);
  if (p)
  {
    /* Get the buff from the struct pbuf address. */
    *buff = (uint8_t *)p + offsetof(RxBuff_t, buff);
    p->custom_free_function = pbuf_free_custom;
    /* Initialize the struct pbuf.
    * This must be performed whenever a buffer's allocated because it may be
    * changed by lwIP or the app, e.g., pbuf_free decrements ref. */
    pbuf_alloced_custom(PBUF_RAW, 0, PBUF_REF, p, *buff, ETH_RX_BUFFER_SIZE);
  }
  else
  {
    if (RxAllocStatus != RX_ALLOC_ERROR) s_rx_alloc_fail++;
    RxAllocStatus = RX_ALLOC_ERROR;
    *buff = NULL;
  }
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
  struct pbuf **ppStart = (struct pbuf **)pStart;
  struct pbuf **ppEnd = (struct pbuf **)pEnd;
  struct pbuf *p = NULL;

  /* Get the struct pbuf from the buff address. */
  p = (struct pbuf *)(buff - offsetof(RxBuff_t, buff));
  p->next = NULL;
  p->tot_len = 0;
  p->len = Length;

  /* Chain the buffer. */
  if (!*ppStart)
  {
    /* The first buffer of the packet. */
    *ppStart = p;
  }
  else
  {
    /* Chain the buffer to the end of the packet. */
    (*ppEnd)->next = p;
  }
  *ppEnd  = p;

  /* Update the total length of all the buffers of the chain. Each pbuf in the chain should have its tot_len
   * set to its own length, plus the length of all the following pbufs in the chain. */
  for (p = *ppStart; p != NULL; p = p->next)
  {
    p->tot_len += Length;
  }
}

void HAL_ETH_TxFreeCallback(uint32_t * buff)
{
  pbuf_free((struct pbuf *)buff);
}

/* ETH global interrupt — MspInit enables ETH_IRQn, so it must be serviced. */
void ETH_IRQHandler(void)
{
  HAL_ETH_IRQHandler(&EthHandle);
}



/* Fill `d` from the PHY (MDIO), the MAC and its MMC counters. Net task only: MDIO is not locked. */
void ethernetif_diag_refresh(struct netif *netif, eth_diag_t *d)
{
  uint32_t v = 0;
  memset(d, 0, sizeof(*d));
  d->phy_ok = true;
  /* BSR's link bit latches low: the first read reports a drop since the last read, the second now. */
  if (HAL_ETH_ReadPHYRegister(&EthHandle, LAN8742.DevAddr, LAN8742_BSR, &v) != HAL_OK ||
      HAL_ETH_ReadPHYRegister(&EthHandle, LAN8742.DevAddr, LAN8742_BSR, &v) != HAL_OK) d->phy_ok = false;
  d->bsr = (uint16_t)v;
  if (HAL_ETH_ReadPHYRegister(&EthHandle, LAN8742.DevAddr, LAN8742_PHYSCSR, &v) != HAL_OK) d->phy_ok = false;
  d->physcsr = (uint16_t)v;
  if (HAL_ETH_ReadPHYRegister(&EthHandle, LAN8742.DevAddr, LAN8742_ANLPAR, &v) != HAL_OK) d->phy_ok = false;
  d->anlpar = (uint16_t)v;
  if (HAL_ETH_ReadPHYRegister(&EthHandle, LAN8742.DevAddr, LAN8742_SECR, &v) != HAL_OK) d->phy_ok = false;
  d->symbol_errors = (uint16_t)v;

  d->maccr = ETH->MACCR;
  d->netif_link = netif_is_link_up(netif);
  d->rx_frames = s_rx_frames;
  d->rx_bcast = s_rx_bcast;
  d->rx_good = ETH->MMCRUPGR;
  d->rx_crc = ETH->MMCRCRCEPR;
  d->rx_align = ETH->MMCRAEPR;
  d->tx_good = ETH->MMCTPCGR;
  d->tx_col_single = ETH->MMCTSCGPR;
  d->tx_col_multi = ETH->MMCTMCGPR;
  uint32_t mfc = ETH->DMACMFCR;   /* clears on read */
  s_rx_missed += mfc & ETH_DMACMFCR_MFC;
  if (mfc & ETH_DMACMFCR_MFCO) s_rx_missed_ovf++;
  d->rx_missed = s_rx_missed;
  d->rx_missed_ovf = s_rx_missed_ovf;
  d->rx_alloc_fail = s_rx_alloc_fail;
  d->rx_alloc_stuck = (RxAllocStatus == RX_ALLOC_ERROR);
}
