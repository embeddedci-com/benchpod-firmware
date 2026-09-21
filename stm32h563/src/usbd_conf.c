/*
 * usbd_conf.c — ST USB Device library low-level layer for STM32H563 USB_DRD_FS.
 *
 * Maps the USBD core's USBD_LL_* calls onto the HAL PCD driver and routes the
 * HAL PCD callbacks back into the core.  Also brings up the USB clock (HSI48 +
 * CRS trim from USB SOF), VDDUSB, GPIO PA11/PA12 and the NVIC.
 *
 * NOTE: ICACHE is intentionally left DISABLED (we never enable it) — the ST USB
 * stack's unique-ID read trips a known ICACHE bug on H5.
 */
#include "usbd_core.h"
#include "usbd_def.h"
#include "usbd_cdc.h"
#include "stm32h5xx_hal.h"

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* ===== HAL PCD MSP (clock / GPIO / NVIC) ================================== */

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcdHandle)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef pclk = {0};
    RCC_OscInitTypeDef osc = {0};
    RCC_CRSInitTypeDef crs = {0};

    if (pcdHandle->Instance != USB_DRD_FS) return;

    /* HSI48 on (USB 48 MHz reference). */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State = RCC_HSI48_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&osc);

    /* USB kernel clock = HSI48. */
    pclk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    pclk.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    HAL_RCCEx_PeriphCLKConfig(&pclk);

    /* Trim HSI48 to USB SOF via the clock-recovery system. */
    __HAL_RCC_CRS_CLK_ENABLE();
    crs.Prescaler = RCC_CRS_SYNC_DIV1;
    crs.Source = RCC_CRS_SYNC_SOURCE_USB;
    crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    crs.ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT;
    crs.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
    crs.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
    HAL_RCCEx_CRSConfig(&crs);

    /* USB transceiver supply. */
    HAL_PWREx_EnableVddUSB();

    /* PA11 = USB DM, PA12 = USB DP (AF10). */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF10_USB;
    HAL_GPIO_Init(GPIOA, &gpio);

    __HAL_RCC_USB_CLK_ENABLE();

    HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcdHandle)
{
    if (pcdHandle->Instance != USB_DRD_FS) return;
    __HAL_RCC_USB_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
    HAL_NVIC_DisableIRQ(USB_DRD_FS_IRQn);
}

/* ===== HAL PCD callbacks -> USBD core ===================================== */

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup);
}
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum,
                         hpcd->OUT_ep[epnum].xfer_buff);
}
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum,
                        hpcd->IN_ep[epnum].xfer_buff);
}
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, USBD_SPEED_FULL);
    USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
}
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
}
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum);
}
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
}
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
}

/* ===== USBD core -> HAL PCD =============================================== */

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
    hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
    hpcd_USB_DRD_FS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;

    hpcd_USB_DRD_FS.pData = pdev;
    pdev->pData = &hpcd_USB_DRD_FS;

    if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK) {
        return USBD_FAIL;
    }

    /* Packet memory layout (single-buffered): EP0 ctrl, CDC bulk IN/OUT, cmd. */
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x00U, PCD_SNG_BUF, 0x18U);
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x80U, PCD_SNG_BUF, 0x58U);
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, CDC_IN_EP,  PCD_SNG_BUF, 0x98U);
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, CDC_OUT_EP, PCD_SNG_BUF, 0xD8U);
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, CDC_CMD_EP, PCD_SNG_BUF, 0x118U);
    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    HAL_StatusTypeDef st = HAL_PCD_DeInit(pdev->pData);
    return (st == HAL_OK) ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    return (HAL_PCD_Start(pdev->pData) == HAL_OK) ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    return (HAL_PCD_Stop(pdev->pData) == HAL_OK) ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                  uint8_t ep_type, uint16_t ep_mps)
{
    return (HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type) == HAL_OK)
               ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return (HAL_PCD_EP_Close(pdev->pData, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return (HAL_PCD_EP_Flush(pdev->pData, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return (HAL_PCD_EP_SetStall(pdev->pData, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return (HAL_PCD_EP_ClrStall(pdev->pData, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
    if ((ep_addr & 0x80U) == 0x80U) return hpcd->IN_ep[ep_addr & 0x7FU].is_stall;
    return hpcd->OUT_ep[ep_addr & 0x7FU].is_stall;
}
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    return (HAL_PCD_SetAddress(pdev->pData, dev_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                    uint8_t *pbuf, uint32_t size)
{
    return (HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size) == HAL_OK)
               ? USBD_OK : USBD_FAIL;
}
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                          uint8_t *pbuf, uint32_t size)
{
    return (HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size) == HAL_OK)
               ? USBD_OK : USBD_FAIL;
}
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

/* ===== misc =============================================================== */

void *USBD_static_malloc(uint32_t size)
{
    (void)size;
    /* Sized for the single CDC class handle. */
    static uint32_t mem[(sizeof(USBD_CDC_HandleTypeDef) / 4) + 1];
    return mem;
}
void USBD_static_free(void *p) { (void)p; }

void USB_DRD_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_DRD_FS);
}
