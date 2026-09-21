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
 * Implements the esptool SLIP command protocol, ROM-loader variant (no stub
 * upload): SYNC, SPI_ATTACH, FLASH_BEGIN, FLASH_DATA, FLASH_END, SPI_FLASH_MD5.
 * All UART I/O is raw-register polled (robust against overrun); HAL is used only
 * to program the baud generator.  Verbose progress via printf to the console.
 *
 * ⚠ NOT hardware-verified — strap/reset timing, USART1 kernel clock, and framing
 *   are bench bring-up items.  Prove the wiring with esp_rom_flash_sync() first.
 */
#include "esp_rom_flash.h"
#include "board_pins.h"

#include "stm32h5xx_hal.h"
#include "mbedtls/md5.h"

#include <string.h>
#include <stdio.h>

/* ---- ROM protocol constants ---------------------------------------------- */
#define ESP_SYNC            0x08
#define ESP_READ_REG        0x0A
#define ESP_SPI_ATTACH      0x0D
#define ESP_FLASH_BEGIN     0x02
#define ESP_FLASH_DATA      0x03
#define ESP_FLASH_END       0x04
#define ESP_SPI_FLASH_MD5   0x13

#define ESP_CHECKSUM_MAGIC  0xEF
#define FLASH_WRITE_SIZE    0x400u          /* ROM-loader FLASH_DATA block size  */
#define ROM_STATUS_BYTES    2               /* ROM (no stub) trailing status len */

/* SLIP framing bytes. */
#define SLIP_END            0xC0
#define SLIP_ESC            0xDB
#define SLIP_ESC_END        0xDC
#define SLIP_ESC_ESC        0xDD

/* Chip-detect magic register (all ESP32 targets). */
#define CHIP_DETECT_MAGIC_REG   0x40001000u
#define ESP_UART_FLASH_BAUD     115200u

/* Static I/O scratch (kept off the caller's stack). */
static uint8_t  s_cmd[8 + 16 + FLASH_WRITE_SIZE];         /* header+data          */
static uint8_t  s_slip[2 * (8 + 16 + FLASH_WRITE_SIZE) + 2]; /* worst-case encoded */
static uint8_t  s_resp[256];                              /* decoded response     */
static uint8_t  s_block[16 + FLASH_WRITE_SIZE];           /* FLASH_DATA packet    */

static UART_HandleTypeDef s_uart;

/* ---- raw USART1 I/O -------------------------------------------------------- */

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
}

static void uart_deinit(void)
{
    HAL_UART_DeInit(&s_uart);
    __HAL_RCC_USART1_CLK_DISABLE();
}

static void uart_putc(uint8_t b)
{
    while (!(ESP_UART->ISR & USART_ISR_TXE_TXFNF)) { }
    ESP_UART->TDR = b;
}

static void uart_tx(const uint8_t *p, uint32_t n)
{
    while (n--) uart_putc(*p++);
}

static void uart_tx_done(void)
{
    while (!(ESP_UART->ISR & USART_ISR_TC)) { }
}

/* Read one byte, clearing overrun; returns byte 0..255 or -1 on timeout. */
static int uart_getc(uint32_t deadline)
{
    for (;;) {
        uint32_t isr = ESP_UART->ISR;
        if (isr & USART_ISR_ORE) ESP_UART->ICR = USART_ICR_ORECF;
        if (isr & USART_ISR_RXNE_RXFNE) return (int)(ESP_UART->RDR & 0xFF);
        if ((int32_t)(HAL_GetTick() - deadline) >= 0) return -1;
    }
}

static void uart_flush_rx(void)
{
    uint32_t end = HAL_GetTick() + 30;
    while ((int32_t)(HAL_GetTick() - end) < 0) {
        uint32_t isr = ESP_UART->ISR;
        if (isr & USART_ISR_ORE) ESP_UART->ICR = USART_ICR_ORECF;
        if (isr & USART_ISR_RXNE_RXFNE) { (void)ESP_UART->RDR; end = HAL_GetTick() + 30; }
    }
}

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
    HAL_Delay(50);                                                    /* drain EN RC */
    HAL_GPIO_WritePin(ESP_EN_PORT,   ESP_EN_PIN,   GPIO_PIN_SET);     /* release   */
    HAL_Delay(200);                                                   /* ROM boot  */
    uart_flush_rx();                                                  /* toss banner */
}

/* Reset into the flashed application: BOOT(IO9)=1 latched at an EN reset. */
static void strap_boot_app(void)
{
    HAL_GPIO_WritePin(ESP_BOOT_PORT, ESP_BOOT_PIN, GPIO_PIN_SET);     /* IO9 high  */
    HAL_GPIO_WritePin(ESP_EN_PORT,   ESP_EN_PIN,   GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(ESP_EN_PORT,   ESP_EN_PIN,   GPIO_PIN_SET);
}

/* ---- SLIP command / response --------------------------------------------- */

static void slip_put(uint8_t **w, uint8_t b)
{
    if (b == SLIP_END) { *(*w)++ = SLIP_ESC; *(*w)++ = SLIP_ESC_END; }
    else if (b == SLIP_ESC) { *(*w)++ = SLIP_ESC; *(*w)++ = SLIP_ESC_ESC; }
    else *(*w)++ = b;
}

/* Send a command frame: 8-byte header (dir=0, op, size, checksum) + data. */
static void esp_send(uint8_t op, const uint8_t *data, uint16_t dlen, uint32_t checksum)
{
    uint8_t *h = s_cmd;
    h[0] = 0x00; h[1] = op;
    h[2] = (uint8_t)dlen; h[3] = (uint8_t)(dlen >> 8);
    h[4] = (uint8_t)checksum;        h[5] = (uint8_t)(checksum >> 8);
    h[6] = (uint8_t)(checksum >> 16); h[7] = (uint8_t)(checksum >> 24);
    if (dlen) memcpy(h + 8, data, dlen);

    uint8_t *w = s_slip;
    *w++ = SLIP_END;
    for (uint32_t i = 0; i < 8u + dlen; i++) slip_put(&w, s_cmd[i]);
    *w++ = SLIP_END;

    uart_flush_rx();
    uart_tx(s_slip, (uint32_t)(w - s_slip));
    uart_tx_done();
}

/* Read one SLIP frame into s_resp; returns decoded length or -1 on timeout. */
static int slip_read(uint32_t timeout_ms)
{
    uint32_t deadline = HAL_GetTick() + timeout_ms;
    int c;
    /* find frame start */
    do { c = uart_getc(deadline); if (c < 0) return -1; } while (c != SLIP_END);
    /* collect until closing END */
    uint32_t n = 0;
    for (;;) {
        c = uart_getc(deadline);
        if (c < 0) return -1;
        if (c == SLIP_END) { if (n == 0) continue; return (int)n; }  /* skip empty */
        if (c == SLIP_ESC) {
            c = uart_getc(deadline);
            if (c < 0) return -1;
            c = (c == SLIP_ESC_END) ? SLIP_END : (c == SLIP_ESC_ESC) ? SLIP_ESC : c;
        }
        if (n < sizeof(s_resp)) s_resp[n++] = (uint8_t)c;
    }
}

/* Send a command and wait for its matching response. On success returns 0 and
 * (optionally) the 4-byte value and a pointer/len to the payload (minus status).
 * `payload`/`plen` may be NULL. */
static int esp_command(uint8_t op, const uint8_t *data, uint16_t dlen, uint32_t checksum,
                       uint32_t *value_out, const uint8_t **payload, int *plen,
                       uint32_t timeout_ms)
{
    esp_send(op, data, dlen, checksum);
    uint32_t deadline = HAL_GetTick() + timeout_ms;
    for (;;) {
        int32_t remain = (int32_t)(deadline - HAL_GetTick());
        if (remain <= 0) return -1;
        int n = slip_read((uint32_t)remain);
        if (n < 0) return -1;
        if (n < 8 + ROM_STATUS_BYTES) continue;         /* runt / not a response  */
        if (s_resp[0] != 0x01 || s_resp[1] != op) continue; /* not our response   */
        uint16_t size = (uint16_t)(s_resp[2] | (s_resp[3] << 8));
        if ((uint32_t)(8 + size) > (uint32_t)n || size < ROM_STATUS_BYTES) continue;
        const uint8_t *body = s_resp + 8;
        if (body[size - ROM_STATUS_BYTES] != 0)         /* status byte: 0 = OK    */
            return -(int)(100 + body[size - ROM_STATUS_BYTES + 1]);  /* -100-errcode */
        if (value_out) *value_out = (uint32_t)s_resp[4] | ((uint32_t)s_resp[5] << 8) |
                                    ((uint32_t)s_resp[6] << 16) | ((uint32_t)s_resp[7] << 24);
        if (payload) *payload = body;
        if (plen)    *plen = (int)size - ROM_STATUS_BYTES;
        return 0;
    }
}

static void put_le32(uint8_t *p, uint32_t v)
{ p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

/* ---- SYNC + chip detect --------------------------------------------------- */

static int esp_sync(void)
{
    uint8_t d[36];
    d[0] = 0x07; d[1] = 0x07; d[2] = 0x12; d[3] = 0x20;
    memset(d + 4, 0x55, 32);
    for (int attempt = 0; attempt < 10; attempt++) {
        if (esp_command(ESP_SYNC, d, sizeof(d), 0, NULL, NULL, NULL, 120) == 0) {
            uart_flush_rx();     /* ROM emits several SYNC replies — drain the rest */
            return 0;
        }
    }
    return -1;
}

static int esp_read_reg(uint32_t addr, uint32_t *val)
{
    uint8_t d[4]; put_le32(d, addr);
    return esp_command(ESP_READ_REG, d, 4, 0, val, NULL, NULL, 200);
}

/* ---- public: wiring test -------------------------------------------------- */

int esp_rom_flash_sync(uint32_t *chip_magic_out)
{
    printf("[espflash] entering C3 download mode (BOOT=0, EN pulse)...\n");
    strap_gpio_init();
    uart_init(ESP_UART_FLASH_BAUD);
    strap_enter_download();

    if (esp_sync() != 0) {
        printf("[espflash] SYNC failed — no response from C3 ROM (check EN/BOOT/UART wiring)\n");
        HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_RESET);
        uart_deinit();
        return -1;
    }
    uint32_t magic = 0;
    (void)esp_read_reg(CHIP_DETECT_MAGIC_REG, &magic);
    printf("[espflash] SYNC ok — C3 ROM responding (chip magic 0x%08lx)\n", (unsigned long)magic);
    if (chip_magic_out) *chip_magic_out = magic;

    /* leave in download mode, powered — caller decides next step */
    uart_deinit();
    return 0;
}

/* ---- public: full program ------------------------------------------------- */

int esp_rom_flash_program(const uint8_t *data, size_t len, uint32_t offset)
{
    if (!data || len == 0) return -1;

    printf("[espflash] programming %u bytes at 0x%06lx\n", (unsigned)len, (unsigned long)offset);
    strap_gpio_init();
    uart_init(ESP_UART_FLASH_BAUD);
    strap_enter_download();

    if (esp_sync() != 0) {
        printf("[espflash] SYNC failed\n");
        goto fail;
    }
    uint32_t magic = 0;
    (void)esp_read_reg(CHIP_DETECT_MAGIC_REG, &magic);
    printf("[espflash] C3 ROM synced (magic 0x%08lx)\n", (unsigned long)magic);

    /* SPI_ATTACH(0): default flash pins. Non-fatal — C3 flash is usually already
       attached; FLASH_BEGIN/DATA + MD5 are the real correctness gates. */
    {
        uint8_t a[8] = {0};
        if (esp_command(ESP_SPI_ATTACH, a, sizeof(a), 0, NULL, NULL, NULL, 1000) != 0)
            printf("[espflash] SPI_ATTACH warning (continuing)\n");
    }

    uint32_t num_blocks = (uint32_t)((len + FLASH_WRITE_SIZE - 1) / FLASH_WRITE_SIZE);
    {
        /* 20 bytes: the C3 ROM sets SPI_FLASH_SUPPORTS_ENCRYPTED_FLASH, so esptool
           appends a 5th word (encrypted=0) for non-stub FLASH_BEGIN. */
        uint8_t b[20];
        put_le32(b + 0, (uint32_t)len);         /* erase size          */
        put_le32(b + 4, num_blocks);            /* num packets         */
        put_le32(b + 8, FLASH_WRITE_SIZE);      /* packet size         */
        put_le32(b + 12, offset);               /* flash offset        */
        put_le32(b + 16, 0);                    /* not encrypted       */
        printf("[espflash] FLASH_BEGIN (%lu blocks, erasing)...\n", (unsigned long)num_blocks);
        /* ROM may erase the whole region up front (esptool budgets ~30 s/MB). */
        if (esp_command(ESP_FLASH_BEGIN, b, sizeof(b), 0, NULL, NULL, NULL, 40000) != 0) {
            printf("[espflash] FLASH_BEGIN failed\n");
            goto fail;
        }
    }

    for (uint32_t seq = 0; seq < num_blocks; seq++) {
        uint32_t off = seq * FLASH_WRITE_SIZE;
        uint32_t n = (len - off < FLASH_WRITE_SIZE) ? (uint32_t)(len - off) : FLASH_WRITE_SIZE;
        uint8_t *pk = s_block;                  /* separate from esp_send's s_cmd */
        put_le32(pk + 0, FLASH_WRITE_SIZE);     /* data length (padded)           */
        put_le32(pk + 4, seq);
        put_le32(pk + 8, 0);
        put_le32(pk + 12, 0);
        memcpy(pk + 16, data + off, n);
        if (n < FLASH_WRITE_SIZE) memset(pk + 16 + n, 0xFF, FLASH_WRITE_SIZE - n);

        uint32_t checksum = ESP_CHECKSUM_MAGIC;
        for (uint32_t i = 0; i < FLASH_WRITE_SIZE; i++) checksum ^= pk[16 + i];

        /* FLASH_DATA payload = 16-byte sub-header + padded block; the command
           checksum covers only the block bytes (esptool convention). */
        if (esp_command(ESP_FLASH_DATA, pk, (uint16_t)(16 + FLASH_WRITE_SIZE),
                        checksum, NULL, NULL, NULL, 3000) != 0) {
            printf("[espflash] FLASH_DATA failed at block %lu/%lu\n",
                   (unsigned long)seq, (unsigned long)num_blocks);
            goto fail;
        }
        if ((seq & 0x3F) == 0 || seq == num_blocks - 1)
            printf("[espflash]  block %lu/%lu\n", (unsigned long)(seq + 1),
                   (unsigned long)num_blocks);
    }

    /* Verify with on-chip MD5 over the written region [offset, offset+len). */
    {
        uint8_t want[16], want_hex[33];
        mbedtls_md5(data, len, want);
        for (int i = 0; i < 16; i++)
            snprintf((char *)want_hex + i * 2, 3, "%02x", want[i]);

        uint8_t m[16];
        put_le32(m + 0, offset); put_le32(m + 4, (uint32_t)len);
        put_le32(m + 8, 0);      put_le32(m + 12, 0);
        const uint8_t *got = NULL; int glen = 0;
        if (esp_command(ESP_SPI_FLASH_MD5, m, sizeof(m), 0, NULL, &got, &glen, 10000) != 0
            || glen < 32) {
            printf("[espflash] MD5 read failed\n");
            goto fail;
        }
        if (memcmp(got, want_hex, 32) != 0) {
            printf("[espflash] MD5 MISMATCH — flash corrupt\n");
            printf("[espflash]   want %.32s\n", (const char *)want_hex);
            printf("[espflash]   got  %.32s\n", (const char *)got);
            goto fail;
        }
        printf("[espflash] MD5 verified (%.32s)\n", (const char *)want_hex);
    }

    /* FLASH_END: stay in loader (param 1) — we do our own clean strap reset. */
    {
        uint8_t e[4]; put_le32(e, 1);
        (void)esp_command(ESP_FLASH_END, e, sizeof(e), 0, NULL, NULL, NULL, 1000);
    }

    printf("[espflash] done — resetting C3 into application\n");
    strap_boot_app();
    uart_deinit();
    return 0;

fail:
    HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_RESET);   /* leave off */
    uart_deinit();
    return -1;
}

void esp_rom_flash_power_off(void)
{
    strap_gpio_init();
    HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_RESET);
}

void esp_uart_monitor(uint32_t ms)
{
    uart_init(ESP_UART_FLASH_BAUD);     /* USART1 only — does not touch EN/BOOT */
    printf("[esp-mon] C3 UART0 for %lu ms:\n", (unsigned long)ms);
    uint32_t end = HAL_GetTick() + ms;
    char line[160]; int li = 0;
    while ((int32_t)(HAL_GetTick() - end) < 0) {
        int c = uart_getc(HAL_GetTick() + 20);
        if (c < 0) continue;
        if (c == '\n' || li >= (int)sizeof(line) - 1) {
            line[li] = '\0';
            if (li) printf("[c3] %s\n", line);
            li = 0;
        } else if (c != '\r') {
            line[li++] = (c >= 32 && c < 127) ? (char)c : '.';
        }
    }
    if (li) { line[li] = '\0'; printf("[c3] %s\n", line); }
    printf("[esp-mon] done\n");
    uart_deinit();
}
