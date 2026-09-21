/*
 * console_io.c — USART2 + USB-CDC byte multiplexer and printf retarget.
 */
#include "console_io.h"
#include "usbd_cdc_if.h"
#include "stm32h5xx_hal.h"
#include "board_pins.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <stdio.h>

extern UART_HandleTypeDef huart2;

#define RX_RING_SIZE  1024   /* power of two */
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile uint32_t rx_drops;

/* TX ring, drained by the USART2 TXE interrupt.  console_io_write() used to
   call HAL_UART_Transmit(..., HAL_MAX_DELAY) — a blocking poll that stalls the
   calling task until every byte is clocked out, and hangs FOREVER on a wedged
   UART (no host reading, XOFF, floating pin).  Now writes enqueue here and
   return immediately; the ISR feeds TDR from the ring.  On a full ring bytes are
   dropped (counted) rather than blocking — same policy as the RX ring and the
   USB-CDC TX ring.  Single-producer (console_io_write, serialised by s_io_mtx) /
   single-consumer (the ISR), so head/tail need no extra locking. */
#define TX_RING_SIZE  2048   /* power of two */
static volatile uint8_t  tx_ring[TX_RING_SIZE];
static volatile uint16_t tx_head, tx_tail;
static volatile uint32_t tx_drops;
static bool              s_tx_irq_ready;   /* USART2 IRQ up: use the ring */

/* Serializes console_io_write() so two tasks' printf/output streams never
   interleave byte-for-byte on the UART/USB (and never race the HAL UART state).
   Taken only once the scheduler is running and not from an ISR — before that,
   output is single-threaded and locking would be unsafe/pointless. */
static SemaphoreHandle_t s_io_mtx;

static inline bool io_lock_active(void)
{
    return s_io_mtx != NULL &&
           xTaskGetSchedulerState() == taskSCHEDULER_RUNNING &&
           !xPortIsInsideInterrupt();
}
static inline void io_lock(void)   { if (io_lock_active()) xSemaphoreTake(s_io_mtx, portMAX_DELAY); }
static inline void io_unlock(void) { if (io_lock_active()) xSemaphoreGive(s_io_mtx); }

void console_io_init(void)
{
    /* Make printf() unbuffered.  Otherwise newlib block-buffers stdout, so the
       prompt/newline/backspace (printed via printf) flush at a different time
       than the char echo and command output (written directly via
       console_io_write) — that reordering made the "> " prompt appear merged
       with the next line and made backspace's "\b \b" erase never show (the
       char WAS deleted from the line buffer, it just wasn't drawn). */
    setvbuf(stdout, NULL, _IONBF, 0);

    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;
    if (s_io_mtx == NULL) s_io_mtx = xSemaphoreCreateMutex();
    /* Enable USART2 RX interrupt (raw-register drain).  TX is now interrupt-
       driven (TXE) rather than HAL-blocking — enabled per-write below. */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    HAL_NVIC_SetPriority(CONSOLE_UART_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CONSOLE_UART_IRQn);
    s_tx_irq_ready = true;   /* from here on, TX goes through the ring + ISR */
}

/* Enqueue into the TX ring and arm the TXE interrupt.  Drops on a full ring. */
static void uart_tx_enqueue(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((tx_head + 1) & (TX_RING_SIZE - 1));
        if (next == tx_tail) { tx_drops++; break; }   /* full — drop the rest */
        tx_ring[tx_head] = buf[i];
        tx_head = next;
    }
    if (tx_head != tx_tail) __HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
}

uint32_t console_tx_dropped(void) { return tx_drops; }

void console_io_write(const uint8_t *buf, size_t len)
{
    io_lock();
    if (huart2.Instance != NULL) {
        if (s_tx_irq_ready) {
            uart_tx_enqueue(buf, len);   /* non-blocking; ISR drains to TDR */
        } else {
            /* Pre-init (very early boot logs): the ISR isn't draining yet, so
               fall back to the blocking poll to avoid losing them. */
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
        }
    }
    cdc_if_write(buf, (uint16_t)len);
    io_unlock();
}

void console_io_rx_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((rx_head + 1) & (RX_RING_SIZE - 1));
    if (next == rx_tail) { rx_drops++; return; }   /* full — drop */
    rx_ring[rx_head] = byte;
    rx_head = next;
}

uint32_t console_rx_dropped(void) { return rx_drops; }

int console_io_getc(void)
{
    if (rx_head == rx_tail) return -1;
    uint8_t c = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1) & (RX_RING_SIZE - 1));
    return c;
}

bool console_io_rx_available(void) { return rx_head != rx_tail; }

/* printf / stdout / stderr -> both console sinks. */
int _write(int file, char *ptr, int len)
{
    (void)file;
    /* ONLCR: emit a CR before any bare LF, so device-log printf()s that end in
       "\n" (not "\r\n") don't stair-step across the terminal (each line starting
       at the column where the previous one ended).  Runs of non-LF bytes go out
       in one write; a lone "\n" becomes "\r\n"; an existing "\r\n" is untouched.
       Only printf() goes through here — console command output and char echo call
       console_io_write() directly and already use "\r\n". */
    static uint8_t last = 0;
    static const uint8_t crlf[2] = { '\r', '\n' };
    int start = 0;
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n' && last != '\r') {
            if (i > start) console_io_write((const uint8_t *)(ptr + start), (size_t)(i - start));
            console_io_write(crlf, 2);
            start = i + 1;
        }
        last = (uint8_t)ptr[i];
    }
    if (len > start) console_io_write((const uint8_t *)(ptr + start), (size_t)(len - start));
    return len;
}

/* USART2: drain RX into the ring (clears RXNE) and feed TX from the ring on TXE;
   clear overrun if set. */
void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        console_io_rx_push((uint8_t)(huart2.Instance->RDR & 0xFFU));
    }
    /* TXE fires whenever TDR is empty, so act only while the interrupt is armed
       (i.e. we still have queued bytes).  Write one byte per TXE; disarm when the
       ring drains so an idle-empty TDR can't storm the CPU. */
    if (__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_TXE) &&
        __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE)) {
        if (tx_head != tx_tail) {
            huart2.Instance->TDR = tx_ring[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1) & (TX_RING_SIZE - 1));
        }
        if (tx_head == tx_tail) {
            __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
        }
    }
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
    }
}
