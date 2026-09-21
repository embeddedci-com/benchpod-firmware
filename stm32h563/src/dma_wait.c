/*
 * dma_wait.c — see dma_wait.h.  Completion waiter + shared HAL SPI/XSPI DMA
 * callbacks (routed by peripheral instance) + GPDMA channel init helper.
 */
#include "dma_wait.h"
#include "xfer.h"      /* XFER_OK / XFER_TIMEOUT / XFER_ERR */

#include "task.h"

/* ---- waiter registry ----------------------------------------------------- */
/* One entry per DMA-driven peripheral (SPI1, SPI4, OCTOSPI1). */
#define DMA_WAIT_MAX 4
static dma_waiter_t *s_reg[DMA_WAIT_MAX];
static int           s_reg_n;

static dma_waiter_t *find_waiter(const void *instance)
{
    for (int i = 0; i < s_reg_n; i++)
        if (s_reg[i]->instance == instance) return s_reg[i];
    return NULL;
}

void dma_wait_setup(dma_waiter_t *w, void *instance)
{
    w->instance = instance;
    w->result   = XFER_OK;
    w->done     = false;
    if (w->sem == NULL) w->sem = xSemaphoreCreateBinary();
    for (int i = 0; i < s_reg_n; i++) if (s_reg[i] == w) return;  /* idempotent */
    if (s_reg_n < DMA_WAIT_MAX) s_reg[s_reg_n++] = w;
}

bool dma_sched_ready(void)
{
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING &&
           !xPortIsInsideInterrupt();
}

void dma_wait_arm(dma_waiter_t *w)
{
    w->result = XFER_OK;
    w->done   = false;
    /* Drain a stale give so the next take blocks until THIS transfer signals. */
    if (w->sem) (void)xSemaphoreTake(w->sem, 0);
}

int dma_wait_block(dma_waiter_t *w, uint32_t timeout_ms)
{
    if (dma_sched_ready() && w->sem) {
        if (xSemaphoreTake(w->sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
            return w->result;
        return XFER_TIMEOUT;
    }
    /* Pre-scheduler defensive poll (the policy avoids DMA here anyway). */
    uint32_t start = HAL_GetTick();
    while (!w->done) {
        if ((HAL_GetTick() - start) >= timeout_ms) return XFER_TIMEOUT;
    }
    return w->result;
}

/* Signal from the completion ISR: latch result, wake the waiter. */
static void dma_wait_signal(dma_waiter_t *w, int result)
{
    if (!w) return;
    w->result = result;
    w->done   = true;
    if (w->sem && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(w->sem, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

static void signal_instance(const void *instance, int result)
{
    dma_wait_signal(find_waiter(instance), result);
}

/* ---- shared HAL completion callbacks (override the HAL weak defaults) ------
   These fire in the peripheral IRQ (SPI EOT / XSPI TC) after the GPDMA channel
   IRQ armed it — one definition serves every SPI/XSPI instance, demuxed by the
   handle's Instance. */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h) { signal_instance(h->Instance, XFER_OK); }
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *h)   { signal_instance(h->Instance, XFER_OK); }
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *h)   { signal_instance(h->Instance, XFER_OK); }
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *h)    { signal_instance(h->Instance, XFER_ERR); }

void HAL_XSPI_RxCpltCallback(XSPI_HandleTypeDef *h) { signal_instance(h->Instance, XFER_OK); }
void HAL_XSPI_TxCpltCallback(XSPI_HandleTypeDef *h) { signal_instance(h->Instance, XFER_OK); }
void HAL_XSPI_ErrorCallback(XSPI_HandleTypeDef *h)  { signal_instance(h->Instance, XFER_ERR); }

/* ---- GPDMA channel init -------------------------------------------------- */
int dma_wait_channel_init(DMA_HandleTypeDef *hdma, DMA_Channel_TypeDef *chan,
                          uint32_t request, uint32_t dir, IRQn_Type irqn)
{
    __HAL_RCC_GPDMA1_CLK_ENABLE();

    hdma->Instance                 = chan;
    hdma->Init.Request             = request;
    hdma->Init.BlkHWRequest        = DMA_BREQ_SINGLE_BURST;
    hdma->Init.Direction           = dir;
    hdma->Init.SrcInc              = (dir == DMA_MEMORY_TO_PERIPH) ? DMA_SINC_INCREMENTED
                                                                   : DMA_SINC_FIXED;
    hdma->Init.DestInc             = (dir == DMA_MEMORY_TO_PERIPH) ? DMA_DINC_FIXED
                                                                   : DMA_DINC_INCREMENTED;
    hdma->Init.SrcDataWidth        = DMA_SRC_DATAWIDTH_BYTE;
    hdma->Init.DestDataWidth       = DMA_DEST_DATAWIDTH_BYTE;
    hdma->Init.Priority            = DMA_HIGH_PRIORITY;
    hdma->Init.SrcBurstLength      = 1;
    hdma->Init.DestBurstLength     = 1;
    hdma->Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT0;
    hdma->Init.TransferEventMode   = DMA_TCEM_BLOCK_TRANSFER;
    hdma->Init.Mode                = DMA_NORMAL;
    if (HAL_DMA_Init(hdma) != HAL_OK) return -1;

    /* Channel runs unprivileged-safe; harmless if the config is ignored. */
    (void)HAL_DMA_ConfigChannelAttributes(hdma, DMA_CHANNEL_NPRIV);

    HAL_NVIC_SetPriority(irqn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(irqn);
    return 0;
}
