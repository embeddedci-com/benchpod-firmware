/*
 * esp_rom_flash.c — program the on-board ESP32-C3 (U4) via its ROM serial
 * bootloader from the STM32 host.  See esp_rom_flash.h for the rationale (no
 * programming header exists on the board — the C3's UART0 + straps reach only
 * the STM32, so the pod must flash it itself, mirroring flash-ice40).
 *
 * Wiring (netlist vbench-pod.net):
 *   USART1 TX PA9  -> C3 U0RXD (pin 30)     BOOT PF12 -> C3 IO9 (download strap)
 *   USART1 RX PA10 <- C3 U0TXD (pin 31)     EN   PF11 -> C3 EN  (reset, act-low)
 *
 * This file is the hardware half: straps, USART1 on interrupts (RX into a ring, TX from the
 * job's frame buffer), the W25Q image source, and the background flash the hw worker drives.
 * The SLIP protocol and the write sequence are in esp_rom_job.c.
 */
#include "esp_rom_flash.h"
#include "esp_rom_job.h"
#include "esp_part_table.h"
#include "board_pins.h"
#include "watchdog.h"
#include "blob_store.h"
#include "w25q.h"

#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bp_log.h"

#include <string.h>
#include <stdio.h>

#define ESP_UART_FLASH_BAUD 115200u

/* Above configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5): FreeRTOS critical sections do not
   mask it, so no byte is lost while a task holds one.  The handler calls no FreeRTOS API. */
#define ESP_UART_IRQ_PRIO   4

static UART_HandleTypeDef s_uart;

/* ---- USART1 on interrupts ---------------------------------------------------- */

#define RX_RING 1024u                       /* power of two */
static volatile uint8_t  s_rx[RX_RING];
static volatile uint32_t s_rx_head;         /* written by the ISR   */
static volatile uint32_t s_rx_tail;         /* written by the task  */
static volatile uint32_t s_rx_overruns;     /* UART overruns + ring full: should stay 0 */
static const uint8_t * volatile s_tx_p;
static volatile uint32_t s_tx_left;

void USART1_IRQHandler(void)
{
    uint32_t isr = ESP_UART->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        if (isr & USART_ISR_ORE) s_rx_overruns++;
        ESP_UART->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    }
    while (ESP_UART->ISR & USART_ISR_RXNE_RXFNE) {
        uint8_t b = (uint8_t)(ESP_UART->RDR & 0xFF);
        uint32_t h = s_rx_head;
        if (h - s_rx_tail < RX_RING) { s_rx[h & (RX_RING - 1)] = b; s_rx_head = h + 1; }
        else s_rx_overruns++;
    }
    if ((ESP_UART->CR1 & USART_CR1_TXEIE_TXFNFIE) && (ESP_UART->ISR & USART_ISR_TXE_TXFNF)) {
        if (s_tx_left) { ESP_UART->TDR = *s_tx_p++; s_tx_left--; }
        if (!s_tx_left) ESP_UART->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
    }
}

static bool scheduler_running(void)
{
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

/* Wait without spinning the CPU (the hw worker runs at a low priority). */
static void sleep_ms_rtos(uint32_t ms)
{
    if (scheduler_running()) vTaskDelay(pdMS_TO_TICKS(ms) ? pdMS_TO_TICKS(ms) : 1);
    else HAL_Delay(ms);
}

static void uart_init(uint32_t baud)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* Pin USART1's kernel clock to PCLK2 (reset default, but be explicit — H5
       kernel-clock muxes have bitten this port before on SPI1). */
    RCC_PeriphCLKInitTypeDef p = {0};
    p.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    p.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    HAL_RCCEx_PeriphCLKConfig(&p);

    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH; g.Alternate = ESP_UART_GPIO_AF;
    g.Pin = ESP_UART_TX_PIN | ESP_UART_RX_PIN;
    HAL_GPIO_Init(ESP_UART_GPIO_PORT, &g);

    memset(&s_uart, 0, sizeof(s_uart));
    s_uart.Instance          = ESP_UART;
    s_uart.Init.BaudRate     = baud;
    s_uart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_uart.Init.StopBits     = UART_STOPBITS_1;
    s_uart.Init.Parity       = UART_PARITY_NONE;
    s_uart.Init.Mode         = UART_MODE_TX_RX;
    s_uart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    s_uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    HAL_UART_Init(&s_uart);

    s_rx_tail = s_rx_head;
    s_tx_left = 0;
    ESP_UART->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    HAL_NVIC_SetPriority(USART1_IRQn, ESP_UART_IRQ_PRIO, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    ESP_UART->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
}

static void uart_deinit(void)
{
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    ESP_UART->CR1 &= ~(USART_CR1_RXNEIE_RXFNEIE | USART_CR1_TXEIE_TXFNFIE);
    s_tx_left = 0;
    HAL_UART_DeInit(&s_uart);
    __HAL_RCC_USART1_CLK_DISABLE();
}

static int uart_rx_byte(void)
{
    uint32_t t = s_rx_tail;
    if (t == s_rx_head) return -1;
    int b = s_rx[t & (RX_RING - 1)];
    s_rx_tail = t + 1;
    return b;
}

static void uart_flush_rx(void) { s_rx_tail = s_rx_head; }

/* esp_rom_io_t.send: drop stale input, then hand the frame to the TX interrupt.  The job keeps
   `frame` untouched until its next send, which comes after this one is answered or timed out. */
static uint32_t uart_send(const uint8_t *frame, uint32_t n)
{
    uint32_t t0 = HAL_GetTick();
    while (s_tx_left && HAL_GetTick() - t0 < 500u) sleep_ms_rtos(1);   /* never in practice */
    uart_flush_rx();
    s_tx_p = frame;
    s_tx_left = n;
    ESP_UART->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    return n * 10u * 1000u / ESP_UART_FLASH_BAUD + 2u;
}

static uint32_t now_ms(void) { return HAL_GetTick(); }

/* ---- EN / BOOT strap control ---------------------------------------------- */

static void strap_gpio_init(void)
{
    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = ESP_EN_PIN;   HAL_GPIO_Init(ESP_EN_PORT,   &g);
    g.Pin = ESP_BOOT_PIN; HAL_GPIO_Init(ESP_BOOT_PORT, &g);
}

/* Reset into the ROM serial bootloader: BOOT(IO9)=0 latched at an EN reset. */
static void strap_enter_download(void)
{
    HAL_GPIO_WritePin(ESP_BOOT_PORT, ESP_BOOT_PIN, GPIO_PIN_RESET);   /* IO9 low   */
    HAL_GPIO_WritePin(ESP_EN_PORT,   ESP_EN_PIN,   GPIO_PIN_RESET);   /* hold rst  */
    sleep_ms_rtos(50);                                                /* drain EN RC */
    HAL_GPIO_WritePin(ESP_EN_PORT,   ESP_EN_PIN,   GPIO_PIN_SET);     /* release   */
    sleep_ms_rtos(200);                                               /* ROM boot  */
    uart_flush_rx();                                                  /* toss banner */
}

/* Reset into the flashed application: BOOT(IO9)=1 latched at an EN reset. */
static void strap_boot_app(void)
{
    HAL_GPIO_WritePin(ESP_BOOT_PORT, ESP_BOOT_PIN, GPIO_PIN_SET);     /* IO9 high  */
    HAL_GPIO_WritePin(ESP_EN_PORT,   ESP_EN_PIN,   GPIO_PIN_RESET);
    sleep_ms_rtos(50);
    HAL_GPIO_WritePin(ESP_EN_PORT,   ESP_EN_PIN,   GPIO_PIN_SET);
}

static void strap_hold_reset(void)
{
    strap_gpio_init();
    HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_RESET);
}

/* Straps + UART into the ROM loader; esp_rom_job takes it from there. */
static void rom_enter(void)
{
    strap_gpio_init();
    uart_init(ESP_UART_FLASH_BAUD);
    strap_enter_download();
}

/* Run a job to its end on the calling task (short jobs only: sync, the NVS erase). */
static int run_blocking(esp_rom_job_t *j)
{
    while (esp_rom_job_step(j)) {
        watchdog_heartbeat(WD_TASK_WORKER, "worker");
        sleep_ms_rtos(1);
    }
    return j->result;
}

/* ---- the image source -------------------------------------------------------- */

/* One block straight from the slot, the bus taken for that read only.  The masters were
   quiesced when the flash started and the busy gate keeps them stopped, so the gateware does
   not need the bus between blocks; commands that touch it in between run on this same worker
   and so never overlap a read. */
static int slot_read(void *ctx, uint32_t off, uint8_t *buf, uint32_t n)
{
    w25q_open();
    int rc = blob_store_read(ctx, off, buf, n);
    w25q_close();
    return rc;
}

/* The same read for code that already holds the bus (the partition-table lookup). */
static int slot_read_held(void *ctx, uint32_t off, uint8_t *buf, uint32_t n)
{
    return blob_store_read(ctx, off, buf, n);
}

/* ---- public: wiring test -------------------------------------------------- */

int esp_rom_flash_sync(uint32_t *chip_magic_out)
{
    if (esp_rom_flash_busy()) return -1;
    log_printf("[espflash] entering C3 download mode (BOOT=0, EN pulse)...\n");
    rom_enter();
    static esp_rom_io_t io;
    io = (esp_rom_io_t){ uart_send, uart_rx_byte, now_ms, NULL, NULL };
    static esp_rom_job_t j;
    esp_rom_job_start_sync(&j, &io);
    int rc = run_blocking(&j);
    uart_deinit();
    if (rc != ESP_ROM_OK) {
        log_printf("[espflash] SYNC failed — no response from C3 ROM (check EN/BOOT/UART wiring)\n");
        strap_hold_reset();
        return -1;
    }
    if (chip_magic_out) *chip_magic_out = j.chip_magic;
    return 0;   /* left in download mode, powered: the caller decides the next step */
}

/* ---- public: background flash from the W25Q slot ------------------------------- */

static esp_rom_job_t     s_job;
static esp_rom_io_t      s_io;
static volatile bool     s_active;
static esp_flash_done_fn s_done;
static uint32_t          s_t0;

bool esp_rom_flash_busy(void) { return s_active; }

bool esp_rom_flash_progress(uint32_t *done, uint32_t *total)
{
    if (!s_active) return false;
    if (done)  *done  = esp_rom_job_blocks_done(&s_job);
    if (total) *total = s_job.blocks;
    return true;
}

int esp_rom_flash_start(esp_flash_done_fn done)
{
    if (s_active) return ESP_FLASH_START_BUSY;
    if (!blob_store_present(BLOB_ESP)) {
        log_printf("[espflash] no ESP32-C3 image in the W25Q (slot esp is empty): install blob esp\n");
        return ESP_FLASH_START_NO_IMAGE;
    }
    w25q_session_open();   /* quiesces the gateware's PSRAM masters, which stay stopped (gate) */
    int intact = blob_store_verify(BLOB_ESP) == 0;
    w25q_session_close();
    if (!intact) {
        log_printf("[espflash] slot esp does not match its checksum: not flashed\n");
        return ESP_FLASH_START_BAD_SLOT;
    }
    s_active = true;       /* from here the busy gate holds off every PSRAM-bus user */
    s_done = done;
    s_t0 = HAL_GetTick();
    s_rx_overruns = 0;
    rom_enter();
    s_io = (esp_rom_io_t){ uart_send, uart_rx_byte, now_ms, slot_read, (void *)(uintptr_t)BLOB_ESP };
    esp_rom_job_start(&s_job, &s_io, blob_store_info(BLOB_ESP)->len, 0);
    return 0;
}

void esp_rom_flash_poll(void)
{
    if (!s_active) return;
    if (esp_rom_job_step(&s_job)) return;
    int rc = s_job.result;
    uart_deinit();
    unsigned secs = (unsigned)((HAL_GetTick() - s_t0) / 1000u);
    if (rc >= 0) {
        log_printf("[espflash] done (%s, %lu block(s) resent, %lu MD5 retries, %lu UART overruns, %u s)"
                   " — resetting C3 into application\n", esp_rom_result_str(rc),
                   (unsigned long)s_job.resends, (unsigned long)s_job.md5_retries,
                   (unsigned long)s_rx_overruns, secs);
        strap_boot_app();
    } else {
        log_printf("[espflash] failed: %s (%lu block(s) resent, %lu UART overruns, %u s); "
                   "C3 left in reset\n", esp_rom_result_str(rc), (unsigned long)s_job.resends,
                   (unsigned long)s_rx_overruns, secs);
        strap_hold_reset();
    }
    esp_flash_done_fn done = s_done;
    s_done = NULL;
    s_active = false;
    if (done) done(rc);
}

/* ---- public: wipe the C3's NVS partition ----------------------------------- */

int esp_rom_flash_erase_nvs(void)
{
    uint32_t off = 0, size = 0;
    int rc = -1;
    if (esp_rom_flash_busy()) {
        log_printf("[espflash] a C3 flash is running: NVS not erased\n");
        return -1;           /* the C3 is the flash's: leave its straps alone */
    }
    if (!blob_store_present(BLOB_ESP)) {
        log_printf("[espflash] no ESP32-C3 image in the W25Q (slot esp is empty): "
                   "cannot locate the C3's NVS partition\n");
        goto out;
    }
    /* The slot is checked like a flash-esp32 would: a corrupt table must not choose what to
       erase (esp_part_find_nvs refuses the bootloader and the table, not the application). */
    w25q_session_open();
    bool found = blob_store_verify(BLOB_ESP) == 0
              && esp_part_find_nvs_rd(slot_read_held, (void *)(uintptr_t)BLOB_ESP, &off, &size);
    w25q_session_close();
    if (!found) {
        log_printf("[espflash] no usable NVS partition in the W25Q C3 image's partition table\n");
        goto out;
    }
    rom_enter();
    {
        static esp_rom_io_t io;
        io = (esp_rom_io_t){ uart_send, uart_rx_byte, now_ms, NULL, NULL };   /* NULL: erase */
        static esp_rom_job_t j;
        esp_rom_job_start(&j, &io, size, off);
        rc = run_blocking(&j) == ESP_ROM_OK ? 0 : -1;   /* an unverified erase is not "blank" */
    }
    uart_deinit();
    if (rc == 0) log_printf("[espflash] C3 NVS erased and verified blank (0x%06lx, %lu bytes)\n",
                            (unsigned long)off, (unsigned long)size);
out:
    strap_hold_reset();      /* in reset either way */
    return rc;
}

void esp_uart_monitor(uint32_t ms)
{
    if (esp_rom_flash_busy()) { log_printf("[esp-mon] a C3 flash is running\n"); return; }
    uart_init(ESP_UART_FLASH_BAUD);     /* USART1 only — does not touch EN/BOOT */
    log_printf("[esp-mon] C3 UART0 for %lu ms:\n", (unsigned long)ms);
    uint32_t end = HAL_GetTick() + ms;
    char line[160]; int li = 0;
    while ((int32_t)(HAL_GetTick() - end) < 0) {
        int c = uart_rx_byte();
        if (c < 0) { sleep_ms_rtos(1); continue; }
        if (c == '\n' || li >= (int)sizeof(line) - 1) {
            line[li] = '\0';
            if (li) log_printf("[c3] %s\n", line);
            li = 0;
        } else if (c != '\r') {
            line[li++] = (c >= 32 && c < 127) ? (char)c : '.';
        }
    }
    if (li) { line[li] = '\0'; log_printf("[c3] %s\n", line); }
    log_printf("[esp-mon] done\n");
    uart_deinit();
}
