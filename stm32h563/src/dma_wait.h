/*
 * dma_wait.h — FreeRTOS-aware GPDMA completion for the SPI/XSPI DMA transfer
 * paths, plus a channel-init helper and the shared HAL completion callbacks.
 *
 * Why this exists: on the STM32H5 a SPI/XSPI DMA transfer does NOT finish in the
 * GPDMA channel ISR — the channel-complete handler merely enables the
 * peripheral's EOT/TC interrupt, and the *peripheral* IRQ (SPIx_IRQn /
 * OCTOSPI1_IRQn) is what finally invokes HAL_SPI_/HAL_XSPI_*CpltCallback.  So a
 * DMA-driven transfer needs BOTH the GPDMA channel IRQ and the peripheral IRQ
 * enabled and routed to their HAL handlers.  The shared cplt/error callbacks
 * live here and demux by peripheral instance to the waiting driver.
 *
 * The wait itself blocks the calling task on a binary semaphore (given from the
 * completion ISR), so the CPU is free for other tasks while the controller moves
 * the bytes — instead of the old spin-in-HAL.  Before the scheduler is running
 * the transfer policy never picks DMA (see dma_sched_ready), so dma_wait_block's
 * poll fallback is only a defensive path.
 */
#ifndef DMA_WAIT_H
#define DMA_WAIT_H

#include "stm32h5xx_hal.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct dma_waiter {
    void             *instance;  /* SPIx / OCTOSPI1 base — matched in HAL cb */
    SemaphoreHandle_t sem;       /* binary; given from the completion ISR    */
    volatile int      result;    /* XFER_OK / XFER_ERR, set by the ISR       */
    volatile bool     done;      /* set by the ISR (poll-fallback reads it)  */
} dma_waiter_t;

/* Create the waiter's semaphore and register it so the shared HAL SPI/XSPI
   completion callbacks can find it by `instance`.  Call once at driver init. */
void dma_wait_setup(dma_waiter_t *w, void *instance);

/* Clear completion state before starting a DMA transfer (drains any stale
   signal so the following dma_wait_block can't return early). */
void dma_wait_arm(dma_waiter_t *w);

/* Block until the in-flight transfer signals completion, or timeout.  Returns
   XFER_OK / XFER_TIMEOUT / XFER_ERR (values from xfer.h). */
int dma_wait_block(dma_waiter_t *w, uint32_t timeout_ms);

/* True when a DMA transfer may be started right now: the RTOS scheduler is
   running and we are not inside an ISR.  Drivers use this as their xfer_backend
   dma_ready() hook so init-time transfers stay blocking. */
bool dma_sched_ready(void);

/* Configure one GPDMA1 channel for a byte-wide peripheral<->memory transfer and
   enable its NVIC line at a FreeRTOS-safe priority.  `dir` is
   DMA_MEMORY_TO_PERIPH (TX) or DMA_PERIPH_TO_MEMORY (RX).  Returns 0 on success,
   -1 on HAL_DMA_Init failure. */
int dma_wait_channel_init(DMA_HandleTypeDef *hdma, DMA_Channel_TypeDef *chan,
                          uint32_t request, uint32_t dir, IRQn_Type irqn);

#endif /* DMA_WAIT_H */
