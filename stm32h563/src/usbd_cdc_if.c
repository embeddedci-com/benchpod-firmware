/*
 * usbd_cdc_if.c — CDC-ACM interface: host RX -> console ring, console TX ring
 * -> host.  TX is ring-buffered and pumped from the TransmitCplt callback so
 * console output isn't dropped under load (as long as the host is draining).
 */
#include "usbd_cdc_if.h"
#include "console_io.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

#define APP_RX_DATA_SIZE   64
#define TX_RING_SIZE       2048   /* must be a power of two */

static uint8_t  rx_packet[APP_RX_DATA_SIZE];
static uint8_t  tx_ring[TX_RING_SIZE];
static volatile uint16_t tx_head, tx_tail;
static volatile uint8_t  tx_busy;

/* Line coding (host queries/sets it; we just store it). */
static uint8_t line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 }; /* 115200 8N1 */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    CDC_Init_FS, CDC_DeInit_FS, CDC_Control_FS, CDC_Receive_FS, CDC_TransmitCplt_FS
};

int cdc_if_connected(void)
{
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}

/* Send the next contiguous chunk from the ring if the endpoint is idle. */
static void tx_pump(void)
{
    if (tx_busy || tx_head == tx_tail || !cdc_if_connected()) return;

    uint16_t tail = tx_tail;
    uint16_t contig = (tx_head > tail) ? (tx_head - tail)
                                       : (TX_RING_SIZE - tail);
    if (contig > APP_RX_DATA_SIZE) contig = APP_RX_DATA_SIZE;

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &tx_ring[tail], contig);
    if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK) {
        tx_busy = 1;
        tx_tail = (uint16_t)((tail + contig) & (TX_RING_SIZE - 1));
    }
}

int cdc_if_write(const uint8_t *buf, uint16_t len)
{
    if (!cdc_if_connected()) return 0;
    int accepted = 0;
    /* Enqueue + kick atomically vs the USB completion ISR (which also pumps). */
    HAL_NVIC_DisableIRQ(USB_DRD_FS_IRQn);
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((tx_head + 1) & (TX_RING_SIZE - 1));
        if (next == tx_tail) break;   /* ring full — drop the rest */
        tx_ring[tx_head] = buf[i];
        tx_head = next;
        accepted++;
    }
    tx_pump();
    HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);
    return accepted;
}

static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_ring, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx_packet);
    tx_head = tx_tail = 0;
    tx_busy = 0;
    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void) { return USBD_OK; }

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    (void)length;
    switch (cmd) {
    case CDC_SET_LINE_CODING:
        if (pbuf) memcpy(line_coding, pbuf, sizeof(line_coding));
        break;
    case CDC_GET_LINE_CODING:
        if (pbuf) memcpy(pbuf, line_coding, sizeof(line_coding));
        break;
    default:
        break;
    }
    return USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
    for (uint32_t i = 0; i < *Len; i++) {
        console_io_rx_push(Buf[i]);
    }
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    (void)Buf; (void)Len; (void)epnum;
    tx_busy = 0;
    tx_pump();
    return USBD_OK;
}
