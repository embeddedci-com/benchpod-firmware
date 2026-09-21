/*
 * esp_hosted_spi.c — ESP-Hosted SPI transport to the ESP32-C3 (U4).
 *
 * STM32 = SPI master on SPI4 (PE12/13/14, AF5) + software CS (PB2). The ESP32
 * raises HANDSHAKE (PB0) when ready for a transaction and DATA_READY (PB1) when
 * it has a frame for us. Frames use the esp-hosted 12-byte header
 * (esp_hosted_frame.*). Polled from the net task; no IRQs.  Each pass runs a
 * bounded BATCH of transactions (esp_hosted_pump.*) rather than a single one,
 * so a backlog is not rationed to one frame per 1 ms net_poll() tick.
 *
 * ⚠ NOT hardware-verified: SPI mode/clock, the HANDSHAKE/DATA_READY polarities,
 *   the per-transaction CS framing, and the reset timing are bench bring-up
 *   items and must match the slave's menuconfig. Buffer size + framing are
 *   fixed by the protocol and unit-tested (esp_hosted_frame).
 */
#include "esp_hosted_spi.h"
#include "esp_hosted_pump.h"
#include "board_pins.h"
#include "xfer.h"
#include "dma_wait.h"

#include "stm32h5xx_hal.h"
#include "FreeRTOSConfig.h"
#include "pico/time.h"        /* sleep_ms / get_absolute_time for reset timing */

#include <string.h>
#include <stdio.h>

/* esp-hosted priv-interface constants (from common/transport/esp_hosted_transport.h). */
#define ESP_HOSTED_PKT_TYPE_EVENT   0x33   /* priv_pkt_type for an event frame   */
#define ESP_HOSTED_EVENT_INIT       0x22   /* event_type: slave bootup announce  */

#define ESP_SPI_XFER_TIMEOUT_MS     50
#define TX_SLOTS                    6      /* pending outbound frames            */

static SPI_HandleTypeDef hspi_esp;

/* ---- SPI4 DMA transfer backend ------------------------------------------
   Each esp-hosted transaction moves a full ESP_HOSTED_SPI_BUF_SIZE (1600 B)
   buffer.  Doing that as a blocking HAL_SPI_TransmitReceive spun the net task
   for the entire clocking; over GPDMA the task blocks on a completion semaphore
   and the CPU is free for lwIP/TLS meanwhile.  SPI4 DMA completes through the
   SPI EOT interrupt, so SPI4_IRQn is enabled + routed below.  Set
   ESP_SPI_USE_DMA=0 to force the blocking transfer. */
#ifndef ESP_SPI_USE_DMA
#define ESP_SPI_USE_DMA   1
#endif

static DMA_HandleTypeDef hdma_esp_tx, hdma_esp_rx;
static dma_waiter_t      s_esp_waiter;
static bool              s_esp_dma_ok;

static int  esp_start_txrx(void *c, const uint8_t *tx, uint8_t *rx, size_t n) {
    (void)c; dma_wait_arm(&s_esp_waiter);
    return HAL_SPI_TransmitReceive_DMA(&hspi_esp, (uint8_t *)tx, rx, (uint16_t)n) == HAL_OK ? 0 : -1;
}
static int  esp_wait(void *c, uint32_t to)  { (void)c; return dma_wait_block(&s_esp_waiter, to); }
static void esp_abort(void *c)              { (void)c; HAL_SPI_Abort(&hspi_esp); }
static int  esp_block_txrx(void *c, const uint8_t *tx, uint8_t *rx, size_t n, uint32_t to) {
    (void)c; return HAL_SPI_TransmitReceive(&hspi_esp, (uint8_t *)tx, rx, (uint16_t)n, to) == HAL_OK ? XFER_OK : XFER_ERR;
}
static bool esp_dma_ready(void *c)          { (void)c; return s_esp_dma_ok && dma_sched_ready(); }

static const xfer_backend_t s_esp_xfer = {
    .start_txrx = esp_start_txrx, .wait = esp_wait, .abort = esp_abort,
    .block_txrx = esp_block_txrx, .dma_ready = esp_dma_ready,
    .dma_min = 64u, .timeout_ms = ESP_SPI_XFER_TIMEOUT_MS,
};

static esp_hosted_frame_cb_t s_data_cb;     /* ESP_STA_IF  */
static esp_hosted_frame_cb_t s_serial_cb;   /* ESP_SERIAL_IF */

static bool     s_running = false;          /* reset released */
static bool     s_ready   = false;          /* boot event seen */
static uint16_t s_seq;

/* TX ring of pre-framed packets (header already prepended). */
static struct { uint8_t buf[ESP_HOSTED_SPI_BUF_SIZE]; uint16_t len; } s_tx[TX_SLOTS];
static uint8_t  s_tx_head, s_tx_tail;       /* head==tail -> empty */

/* Transport counters, zeroed at each esp_hosted_spi_start(). */
static esp_hosted_pump_stats_t s_pump_stats;

/* Static transaction buffers (avoid 3.2 KB on the net-task stack). */
static uint8_t  s_xfer_tx[ESP_HOSTED_SPI_BUF_SIZE];
static uint8_t  s_xfer_rx[ESP_HOSTED_SPI_BUF_SIZE];

static inline bool tx_empty(void) { return s_tx_head == s_tx_tail; }
static inline bool tx_full(void)  { return (uint8_t)(s_tx_head + 1) % TX_SLOTS == s_tx_tail; }

/* ---- GPIO / SPI bring-up -------------------------------------------------- */

static void esp_gpio_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* SPI4 SCK/MISO/MOSI — alternate function, high speed. */
    g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = ESP_SPI_AF;
    g.Pin = ESP_SPI_SCK_PIN;  HAL_GPIO_Init(ESP_SPI_SCK_PORT,  &g);
    g.Pin = ESP_SPI_MISO_PIN; HAL_GPIO_Init(ESP_SPI_MISO_PORT, &g);
    g.Pin = ESP_SPI_MOSI_PIN; HAL_GPIO_Init(ESP_SPI_MOSI_PORT, &g);

    /* CS / EN / BOOT — push-pull outputs. CS idle high, EN low (held in reset),
       BOOT high (normal boot once released). */
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = 0;
    g.Pin = ESP_CS_PIN;   HAL_GPIO_Init(ESP_CS_PORT,   &g);
    HAL_GPIO_WritePin(ESP_CS_PORT, ESP_CS_PIN, GPIO_PIN_SET);
    g.Pin = ESP_EN_PIN;   HAL_GPIO_Init(ESP_EN_PORT,   &g);
    HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_RESET);   /* hold reset */
    g.Pin = ESP_BOOT_PIN; HAL_GPIO_Init(ESP_BOOT_PORT, &g);
    HAL_GPIO_WritePin(ESP_BOOT_PORT, ESP_BOOT_PIN, GPIO_PIN_SET); /* normal boot */

    /* HANDSHAKE / DATA_READY — inputs from the slave. */
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = ESP_HOSTED_HS_ACTIVE_HIGH ? GPIO_PULLDOWN : GPIO_PULLUP;
    g.Pin = ESP_HANDSHAKE_PIN; HAL_GPIO_Init(ESP_HANDSHAKE_PORT, &g);
    g.Pull = ESP_HOSTED_DR_ACTIVE_HIGH ? GPIO_PULLDOWN : GPIO_PULLUP;
    g.Pin = ESP_DATAREADY_PIN; HAL_GPIO_Init(ESP_DATAREADY_PORT, &g);
}

int esp_hosted_spi_init(void) {
    esp_gpio_init();

    __HAL_RCC_SPI4_CLK_ENABLE();
    memset(&hspi_esp, 0, sizeof(hspi_esp));
    hspi_esp.Instance               = ESP_SPI;
    hspi_esp.Init.Mode              = SPI_MODE_MASTER;
    hspi_esp.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi_esp.Init.DataSize          = SPI_DATASIZE_8BIT;
#if ESP_HOSTED_SPI_MODE == 0
    hspi_esp.Init.CLKPolarity = SPI_POLARITY_LOW;  hspi_esp.Init.CLKPhase = SPI_PHASE_1EDGE;
#elif ESP_HOSTED_SPI_MODE == 1
    hspi_esp.Init.CLKPolarity = SPI_POLARITY_LOW;  hspi_esp.Init.CLKPhase = SPI_PHASE_2EDGE;
#elif ESP_HOSTED_SPI_MODE == 2
    hspi_esp.Init.CLKPolarity = SPI_POLARITY_HIGH; hspi_esp.Init.CLKPhase = SPI_PHASE_1EDGE;
#else
    hspi_esp.Init.CLKPolarity = SPI_POLARITY_HIGH; hspi_esp.Init.CLKPhase = SPI_PHASE_2EDGE;
#endif
    hspi_esp.Init.NSS               = SPI_NSS_SOFT;          /* CS is the PB2 GPIO */
    hspi_esp.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;  /* conservative — tune on bench */
    hspi_esp.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi_esp.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi_esp.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi_esp.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(&hspi_esp) != HAL_OK) {
        printf("[esp] SPI4 init failed\n");
        return -1;
    }

#if ESP_SPI_USE_DMA
    if (dma_wait_channel_init(&hdma_esp_rx, GPDMA1_Channel2, GPDMA1_REQUEST_SPI4_RX,
                              DMA_PERIPH_TO_MEMORY, GPDMA1_Channel2_IRQn) == 0 &&
        dma_wait_channel_init(&hdma_esp_tx, GPDMA1_Channel3, GPDMA1_REQUEST_SPI4_TX,
                              DMA_MEMORY_TO_PERIPH, GPDMA1_Channel3_IRQn) == 0) {
        __HAL_LINKDMA(&hspi_esp, hdmarx, hdma_esp_rx);
        __HAL_LINKDMA(&hspi_esp, hdmatx, hdma_esp_tx);
        dma_wait_setup(&s_esp_waiter, ESP_SPI);
        HAL_NVIC_SetPriority(SPI4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(SPI4_IRQn);
        s_esp_dma_ok = true;
        printf("[esp] SPI4 DMA enabled (GPDMA1 ch2=rx ch3=tx)\n");
    } else {
        printf("[esp] SPI4 DMA init failed — using blocking transfers\n");
    }
#endif

    s_running = false; s_ready = false;
    s_tx_head = s_tx_tail = 0;
    printf("[esp] SPI4 transport configured (ESP32-C3 held in reset)\n");
    return 0;
}

void esp_hosted_spi_start(void) {
    if (s_running) return;
    /* Reset pulse: EN low (held by init), BOOT high already; release EN. */
    HAL_GPIO_WritePin(ESP_BOOT_PORT, ESP_BOOT_PIN, GPIO_PIN_SET);   /* normal boot */
    HAL_GPIO_WritePin(ESP_EN_PORT,   ESP_EN_PIN,   GPIO_PIN_RESET);
    sleep_ms(10);
    HAL_GPIO_WritePin(ESP_EN_PORT,   ESP_EN_PIN,   GPIO_PIN_SET);   /* release reset */
    s_running = true; s_ready = false;
    s_tx_head = s_tx_tail = 0;
    memset(&s_pump_stats, 0, sizeof(s_pump_stats));
    printf("[esp] ESP32-C3 reset released — awaiting boot event\n");
}

void esp_hosted_spi_stop(void) {
    HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_RESET);     /* hold reset */
    s_running = false; s_ready = false;
    s_tx_head = s_tx_tail = 0;
}

bool esp_hosted_spi_ready(void) { return s_ready; }

/* True once the SPI4 GPDMA channels inited OK (Wi-Fi transactions use DMA). */
bool esp_hosted_spi_dma_active(void) { return s_esp_dma_ok; }

void esp_hosted_spi_set_data_cb(esp_hosted_frame_cb_t cb)   { s_data_cb = cb; }
void esp_hosted_spi_set_serial_cb(esp_hosted_frame_cb_t cb) { s_serial_cb = cb; }

/* ---- TX ------------------------------------------------------------------- */

int esp_hosted_spi_send(uint8_t if_type, uint8_t if_num,
                        const uint8_t *payload, uint16_t len) {
    if (!s_running) return -1;
    if (tx_full()) return -1;
    uint8_t *slot = s_tx[s_tx_head].buf;
    size_t n = esp_hosted_frame_build(slot, ESP_HOSTED_SPI_BUF_SIZE,
                                      if_type, if_num, 0, 0, s_seq++, payload, len);
    if (n == 0) return -1;   /* too large for one transfer */
    s_tx[s_tx_head].len = (uint16_t)n;
    s_tx_head = (uint8_t)((s_tx_head + 1) % TX_SLOTS);
    return 0;
}

/* ---- RX demux ------------------------------------------------------------- */

static void handle_rx_frame(const esp_hosted_rx_t *rx) {
    switch (rx->if_type) {
    case ESP_STA_IF:
    case ESP_AP_IF:
        if (s_data_cb) s_data_cb(rx);
        break;
    case ESP_SERIAL_IF:
        if (s_serial_cb) s_serial_cb(rx);
        break;
    case ESP_PRIV_IF:
        /* Boot announce: priv_pkt_type == EVENT and first payload byte == INIT.
           (Capabilities/config TLVs in the payload are parsed later — TODO.) */
        if (rx->priv_pkt_type == ESP_HOSTED_PKT_TYPE_EVENT &&
            rx->payload_len >= 1 && rx->payload[0] == ESP_HOSTED_EVENT_INIT) {
            if (!s_ready) printf("[esp] slave boot event — link up\n");
            s_ready = true;
        }
        break;
    default:
        break;
    }
}

/* ---- polled transaction --------------------------------------------------- */

static inline bool hs_active(void) {
    GPIO_PinState v = HAL_GPIO_ReadPin(ESP_HANDSHAKE_PORT, ESP_HANDSHAKE_PIN);
    return ESP_HOSTED_HS_ACTIVE_HIGH ? (v == GPIO_PIN_SET) : (v == GPIO_PIN_RESET);
}
static inline bool dr_active(void) {
    GPIO_PinState v = HAL_GPIO_ReadPin(ESP_DATAREADY_PORT, ESP_DATAREADY_PIN);
    return ESP_HOSTED_DR_ACTIVE_HIGH ? (v == GPIO_PIN_SET) : (v == GPIO_PIN_RESET);
}

/* Bounded wait for HANDSHAKE to come back mid-batch: the slave drops it for a
   moment after each transaction while it re-arms.  Only ever called when we
   already know more work is pending, and never on the first transaction of a
   pass, so an idle link costs nothing. */
#ifndef ESP_HOSTED_HS_SETTLE_US
#define ESP_HOSTED_HS_SETTLE_US   200u
#endif

static bool esp_hs_settle(void *ctx) {
    (void)ctx;
    uint32_t start = time_us_32();
    while ((uint32_t)(time_us_32() - start) < ESP_HOSTED_HS_SETTLE_US) {
        if (hs_active()) return true;
        busy_wait_us(2);
    }
    return hs_active();
}

/* One full-duplex transaction: clock the next queued TX frame (or a zero-padded
   dummy) out, demux whatever came back. */
static int esp_do_xact(void *ctx) {
    (void)ctx;
    bool have_tx = !tx_empty();

    memset(s_xfer_tx, 0, sizeof(s_xfer_tx));
    if (have_tx) memcpy(s_xfer_tx, s_tx[s_tx_tail].buf, s_tx[s_tx_tail].len);

    HAL_GPIO_WritePin(ESP_CS_PORT, ESP_CS_PIN, GPIO_PIN_RESET);
    int st = xfer_txrx(&s_esp_xfer, s_xfer_tx, s_xfer_rx, ESP_HOSTED_SPI_BUF_SIZE);
    HAL_GPIO_WritePin(ESP_CS_PORT, ESP_CS_PIN, GPIO_PIN_SET);
    if (st != XFER_OK) {
        printf("[esp] SPI xfer error %d\n", st);
        return -1;   /* leave the TX frame queued for retry */
    }

    if (have_tx) s_tx_tail = (uint8_t)((s_tx_tail + 1) % TX_SLOTS);

    esp_hosted_rx_t rx;
    if (esp_hosted_frame_parse(s_xfer_rx, ESP_HOSTED_SPI_BUF_SIZE, &rx) == 0 &&
        rx.payload_len > 0) {
        handle_rx_frame(&rx);
    }
    return 0;
}

static bool esp_hs_active_cb(void *ctx)   { (void)ctx; return hs_active(); }
static bool esp_dr_active_cb(void *ctx)   { (void)ctx; return dr_active(); }
static bool esp_tx_pending_cb(void *ctx)  { (void)ctx; return !tx_empty(); }

static const esp_hosted_pump_backend_t s_esp_pump = {
    .hs_active  = esp_hs_active_cb,
    .dr_active  = esp_dr_active_cb,
    .tx_pending = esp_tx_pending_cb,
    .xact       = esp_do_xact,
    .hs_settle  = esp_hs_settle,
    .ctx        = NULL,
    .max_batch  = 0,   /* ESP_HOSTED_PUMP_MAX_BATCH */
    .stats      = &s_pump_stats,
};

const esp_hosted_pump_stats_t *esp_hosted_spi_stats(void) { return &s_pump_stats; }

void esp_hosted_spi_poll(void) {
    if (!s_running) return;
    esp_hosted_pump(&s_esp_pump);
}

/* ---- SPI4 DMA IRQs -------------------------------------------------------
   GPDMA channels finish the block + arm the SPI EOT interrupt; SPI4_IRQHandler
   services EOT and calls the shared HAL_SPI_*CpltCallback (dma_wait.c). */
#if ESP_SPI_USE_DMA
void GPDMA1_Channel2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_esp_rx); }
void GPDMA1_Channel3_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_esp_tx); }
void SPI4_IRQHandler(void)            { HAL_SPI_IRQHandler(&hspi_esp); }
#endif
