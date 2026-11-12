/*
 * can_bus.c — classic CAN and CAN FD over FDCAN1 (TCAN1044 transceiver U10).
 *
 * Single-node board: `normal` mode needs an external node to ACK; the loopback
 * modes (internal / external) let one pod self-test TX+RX end to end (see
 * can_bus.h).  Received frames are pushed from the FDCAN RX-FIFO0 interrupt into
 * a lock-free single-producer/single-consumer ring, drained by can_rx_pop().
 *
 * FDCAN kernel clock, selected in can_configure() via RCCEx: HSE (25 MHz) for
 * classic CAN, PLL2Q (80 MHz from HSE) for CAN FD (see can_timing.h).
 */
#include "can_bus.h"
#include "can_timing.h"
#include "can_ring.h"
#include "board_pins.h"
#include "stm32h5xx_hal.h"
#include "bp_log.h"
#include <string.h>
#include <stdio.h>

static FDCAN_HandleTypeDef hfdcan;
static bool     s_enabled;
static bool     s_term;
static can_mode_t s_mode;
static uint32_t s_bitrate;
static uint32_t s_data_bitrate;   /* 0 = classic */
static bool     s_fd;             /* FD frames accepted (FDOE) */
static bool     s_brs;            /* bit-rate switching on (BRSE) */

/* SPSC RX ring (can_ring.h): the FDCAN IT0 ISR is the only producer,
   can_rx_pop()/can_rx_peek() the only consumer.  Variable-length records:
   1 KB holds 51 classic 8-byte frames or 13 64-byte FD frames (the fixed
   32-slot ring it replaced was 768 B, classic only). */
#define CAN_RX_RING_BYTES  1024u
static uint8_t           s_ring_buf[CAN_RX_RING_BYTES] __attribute__((aligned(4)));
static can_ring_t        s_ring;
static volatile uint32_t s_rx_overflow;

/* Autonomous responder rules (matched in the RX ISR). */
typedef struct {
    bool        used;
    uint32_t    match_id;
    bool        match_ext;
    can_frame_t reply;
} can_rule_t;
static can_rule_t        s_rules[CAN_RESPONDER_MAX];
static volatile uint32_t s_resp_hits;

/* Bus-off recovery (see can_bus.h).  Counted from the ISR and the task paths;
   logged from task context only. */
static volatile uint32_t s_busoff_recoveries;
static uint32_t          s_busoff_logged;

_Static_assert(CAN_REG_PSR_BO == FDCAN_PSR_BO, "PSR.BO bit");
_Static_assert(CAN_REG_CCCR_INIT == FDCAN_CCCR_INIT, "CCCR.INIT bit");

/* Mask the FDCAN IT0 interrupt (RX + responder + bus-off) around task-side
   work that touches the same state.  Returns whether it was enabled. */
static bool can_irq_mask(void)
{
    bool was = NVIC_GetEnableIRQ(CAN_IRQn) != 0u;
    HAL_NVIC_DisableIRQ(CAN_IRQn);
    __DSB();
    __ISB();
    return was;
}

static void can_irq_restore(bool was)
{
    if (was) HAL_NVIC_EnableIRQ(CAN_IRQn);
}

/* Start a bus-off recovery if needed.  Caller has masked the IRQ or IS the
   IRQ. */
static bool can_busoff_recover_locked(void)
{
    if (!s_enabled) return false;
    if (!can_busoff_recover_regs(&hfdcan.Instance->CCCR, hfdcan.Instance->PSR)) return false;
    s_busoff_recoveries++;
    return true;
}

/* Task-side check + log.  Returns true if the core is bus-off right now (a
   recovery may have just been started, or be under way). */
static bool can_busoff_poll(void)
{
    if (!s_enabled) return false;
    bool was = can_irq_mask();
    (void)can_busoff_recover_locked();
    bool bo = (hfdcan.Instance->PSR & FDCAN_PSR_BO) != 0u;
    can_irq_restore(was);
    uint32_t n = s_busoff_recoveries;
    if (n != s_busoff_logged) {
        log_printf("[can] bus-off: recovery started (%lu since boot), rejoining after 129x11 recessive bits\n",
               (unsigned long)n);
        s_busoff_logged = n;
    }
    return bo;
}

/* ---- bit timing: can_timing.c ------------------------------------------ */

static uint32_t hal_mode(can_mode_t m)
{
    switch (m) {
        case CAN_MODE_LOOPBACK_INTERNAL: return FDCAN_MODE_INTERNAL_LOOPBACK;
        case CAN_MODE_LOOPBACK_EXTERNAL: return FDCAN_MODE_EXTERNAL_LOOPBACK;
        case CAN_MODE_LISTEN:            return FDCAN_MODE_BUS_MONITORING;
        case CAN_MODE_NORMAL:
        default:                         return FDCAN_MODE_NORMAL;
    }
}

/* ---- termination (PA4 -> U24) -------------------------------------------- */

int can_set_term(bool on)
{
    HAL_GPIO_WritePin(CAN_TERM_PORT, CAN_TERM_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    s_term = on;
    return 0;
}

bool can_get_term(void) { return s_term; }

/* ---- autonomous responder ----------------------------------------------- */

/* Check a frame and round an FD length up to the next valid one, zero-padding
   the data.  Returns the DLC code, or -1 if the frame is invalid. */
static int can_frame_normalize(can_frame_t *f)
{
    if (f->fd && f->rtr) return -1;            /* CAN FD has no remote frames */
    if (!f->fd) f->brs = false;
    int code = can_len_to_dlc(f->dlc, f->fd);
    if (code < 0) return -1;
    uint8_t len = can_dlc_to_len((uint32_t)code);
    if (len > f->dlc) memset(&f->data[f->dlc], 0, (size_t)(len - f->dlc));
    f->dlc = len;
    return code;
}

int can_responder_add(uint32_t match_id, bool match_ext, const can_frame_t *reply)
{
    if (!reply) return -2;
    can_frame_t rep = *reply;
    if (can_frame_normalize(&rep) < 0) return -2;
    for (uint32_t r = 0; r < CAN_RESPONDER_MAX; r++) {
        if (!s_rules[r].used) {
            s_rules[r].match_id  = match_id;
            s_rules[r].match_ext = match_ext;
            s_rules[r].reply     = rep;
            s_rules[r].used      = true;   /* set last: the ISR reads `used` first */
            return (int)r;
        }
    }
    return -1;   /* table full */
}

void can_responder_clear(void)
{
    for (uint32_t r = 0; r < CAN_RESPONDER_MAX; r++) s_rules[r].used = false;
    s_resp_hits = 0;
}

int can_responder_count(void)
{
    int n = 0;
    for (uint32_t r = 0; r < CAN_RESPONDER_MAX; r++) if (s_rules[r].used) n++;
    return n;
}

void can_bus_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    CAN_TERM_CLK_EN();
    HAL_GPIO_WritePin(CAN_TERM_PORT, CAN_TERM_PIN, GPIO_PIN_RESET);  /* off first */
    gpio.Pin   = CAN_TERM_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAN_TERM_PORT, &gpio);
    s_term    = false;
    s_enabled = false;
    can_ring_init(&s_ring, s_ring_buf, sizeof(s_ring_buf));
    can_ring_reset(&s_ring);
    s_rx_overflow = 0;
    can_responder_clear();
    log_printf("[can] init: FDCAN1 PD1/PD0, term=PA4 (off), core idle\n");
}

/* ---- bring-up / teardown ------------------------------------------------- */

static int can_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    CAN_GPIO_CLK_EN();
    gpio.Pin       = CAN_TX_PIN | CAN_RX_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = CAN_GPIO_AF;
    HAL_GPIO_Init(CAN_GPIO_PORT, &gpio);
    return 0;
}

void can_disable(void)
{
    if (s_enabled) {
        HAL_NVIC_DisableIRQ(CAN_IRQn);
        HAL_FDCAN_Stop(&hfdcan);
        HAL_FDCAN_DeInit(&hfdcan);
        s_enabled = false;
    }
    s_fd = s_brs = false;
    s_data_bitrate = 0;
    can_ring_reset(&s_ring);
    log_printf("[can] disabled\n");
}

/* PLL2 for the CAN FD kernel clock: HSE 25 MHz / 5 = 5 MHz, x80 = 400 MHz VCO,
   Q / 5 = 80 MHz.  Nothing else uses PLL2.  Only the Q output is enabled. */
static int can_kernel_clock(bool fd)
{
    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    if (!fd) {
        pclk.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
    } else {
        pclk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL2Q;
        pclk.PLL2.PLL2Source      = RCC_PLL2_SOURCE_HSE;
        pclk.PLL2.PLL2M           = 5;
        pclk.PLL2.PLL2N           = 80;
        pclk.PLL2.PLL2P           = 2;
        pclk.PLL2.PLL2Q           = 5;
        pclk.PLL2.PLL2R           = 2;
        pclk.PLL2.PLL2RGE         = RCC_PLL2_VCIRANGE_2;      /* 4..8 MHz input */
        pclk.PLL2.PLL2VCOSEL      = RCC_PLL2_VCORANGE_WIDE;   /* 192..836 MHz */
        pclk.PLL2.PLL2FRACN       = 0;
        pclk.PLL2.PLL2ClockOut    = RCC_PLL2_DIVQ;
    }
    return (HAL_RCCEx_PeriphCLKConfig(&pclk) == HAL_OK) ? 0 : -1;
}

int can_configure(uint32_t bitrate, uint32_t data_bitrate, can_mode_t mode, bool term)
{
    const bool fd  = data_bitrate != 0u;
    const bool brs = fd && data_bitrate > bitrate;
    const uint32_t fclk = fd ? CAN_FD_KERNEL_HZ : CAN_CLASSIC_KERNEL_HZ;
    can_timing_t nt = {0}, dt = {0};

    if (fd && (data_bitrate < bitrate || data_bitrate > CAN_FD_DATA_MAX_BPS)) {
        log_printf("[can] data bitrate %lu bit/s outside %lu..%lu\n",
               (unsigned long)data_bitrate, (unsigned long)bitrate,
               (unsigned long)CAN_FD_DATA_MAX_BPS);
        return -4;
    }
    if (can_timing_nominal(fclk, bitrate, fd, &nt) != 0) {
        log_printf("[can] no bit timing for %lu bit/s @ %lu Hz kernel\n",
               (unsigned long)bitrate, (unsigned long)fclk);
        return -1;
    }
    if (brs) {
        if (can_timing_data(fclk, data_bitrate, &dt) != 0) {
            log_printf("[can] no data bit timing for %lu bit/s @ %lu Hz kernel\n",
                   (unsigned long)data_bitrate, (unsigned long)fclk);
            return -3;
        }
    } else {
        /* No switching: the data phase runs at the nominal rate.  The DBTP
           fields are narrower than NBTP, so only copy what fits (unused for
           classic, and for FD without BRS the core uses the nominal timing). */
        dt.presc = (nt.presc <= 32u) ? nt.presc : 32u;
        dt.seg1  = (nt.seg1 <= 32u) ? nt.seg1 : 32u;
        dt.seg2  = (nt.seg2 <= 16u) ? nt.seg2 : 16u;
        dt.sjw   = (nt.sjw <= 16u) ? nt.sjw : 16u;
        if (dt.sjw > dt.seg2) dt.sjw = dt.seg2;
    }

    /* Re-configuration: tear the old instance down first (idempotent). */
    can_disable();

    /* HSE is already on (PLL1 source). */
    if (can_kernel_clock(fd) != 0) return -2;
    __HAL_RCC_FDCAN_CLK_ENABLE();
    /* DeInit/Init leaves TEC/REC alone (only a peripheral reset or bus-off
       recovery clears them), so one session that ended error passive made the
       next start error passive too. Each configure starts from a clean core. */
    __HAL_RCC_FDCAN_FORCE_RESET();
    __HAL_RCC_FDCAN_RELEASE_RESET();

    can_gpio_init();

    hfdcan.Instance                  = CAN_FD;
    hfdcan.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan.Init.FrameFormat          = !fd ? FDCAN_FRAME_CLASSIC
                                     : (brs ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_FD_NO_BRS);
    hfdcan.Init.Mode                 = hal_mode(mode);
    hfdcan.Init.AutoRetransmission   = ENABLE;
    hfdcan.Init.TransmitPause        = DISABLE;
    hfdcan.Init.ProtocolException    = DISABLE;
    hfdcan.Init.NominalPrescaler     = nt.presc;
    hfdcan.Init.NominalSyncJumpWidth = nt.sjw;
    hfdcan.Init.NominalTimeSeg1      = nt.seg1;
    hfdcan.Init.NominalTimeSeg2      = nt.seg2;
    hfdcan.Init.DataPrescaler        = dt.presc;
    hfdcan.Init.DataSyncJumpWidth    = dt.sjw;
    hfdcan.Init.DataTimeSeg1         = dt.seg1;
    hfdcan.Init.DataTimeSeg2         = dt.seg2;
    hfdcan.Init.StdFiltersNbr        = 0;       /* no explicit filters — global accept */
    hfdcan.Init.ExtFiltersNbr        = 0;
    hfdcan.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hfdcan) != HAL_OK) return -2;

    /* Accept every non-matching std/ext frame into RX FIFO0 (no filter table);
       reject remote frames only if the caller never wants them — keep them. */
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan,
            FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        return -2;
    }

    /* Transmitter delay compensation: at fast data rates the transceiver loop
       delay (TCAN1044: up to ~175 ns TXD->RXD) is a large part of a bit, so the
       core would see its own bits late and flag bit errors.  TDC measures the
       delay on every FD frame and checks each bit at delay + TDCO instead. */
    if (brs && dt.tdco != 0u) {
        if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan, dt.tdco, 0) != HAL_OK ||
            HAL_FDCAN_EnableTxDelayCompensation(&hfdcan) != HAL_OK) {
            return -2;
        }
    }

    if (HAL_FDCAN_Start(&hfdcan) != HAL_OK) return -2;

    /* New-message-in-FIFO0 + bus-off interrupts -> IT0 line (ILS default). */
    if (HAL_FDCAN_ActivateNotification(&hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
        return -2;
    }
    HAL_NVIC_SetPriority(CAN_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN_IRQn);

    s_enabled      = true;
    s_mode         = mode;
    s_bitrate      = bitrate;
    s_data_bitrate = data_bitrate;
    s_fd           = fd;
    s_brs          = brs;
    can_ring_reset(&s_ring);
    s_rx_overflow = 0;
    can_set_term(term);

    log_printf("[can] up: %lu bit/s mode=%s presc=%lu seg1=%lu seg2=%lu sjw=%lu sp=%lu term=%d\n",
           (unsigned long)bitrate, can_mode_name(mode),
           (unsigned long)nt.presc, (unsigned long)nt.seg1, (unsigned long)nt.seg2,
           (unsigned long)nt.sjw, (unsigned long)can_timing_sample_permille(&nt), term ? 1 : 0);
    if (fd) {
        log_printf("[can] fd: data %lu bit/s brs=%d presc=%lu seg1=%lu seg2=%lu sjw=%lu sp=%lu tdco=%lu\n",
               (unsigned long)data_bitrate, brs ? 1 : 0,
               (unsigned long)dt.presc, (unsigned long)dt.seg1, (unsigned long)dt.seg2,
               (unsigned long)dt.sjw, (unsigned long)can_timing_sample_permille(&dt),
               (unsigned long)dt.tdco);
    }
    return 0;
}

/* ---- transmit ------------------------------------------------------------ */

/* Build a Tx header + queue a normalized frame (can_frame_normalize).  No
   enabled/bus-off checks (callers do those) so it is safe to call from the RX
   ISR (responder path).  Returns 0, -1 if the TX FIFO is full / HAL rejected
   it, -2 if the frame does not fit the bus (FD frame on a classic bus). */
static int can_tx_raw(const can_frame_t *f)
{
    if (f->fd && !s_fd) return -2;
    int code = can_len_to_dlc(f->dlc, f->fd);
    if (code < 0) return -2;
    FDCAN_TxHeaderTypeDef h = {0};
    h.Identifier          = f->ext ? (f->id & 0x1FFFFFFFu) : (f->id & 0x7FFu);
    h.IdType              = f->ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    h.TxFrameType         = f->rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    h.DataLength          = (uint32_t)code;   /* DLC code (H5 HAL: 0..15) */
    h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch       = (f->fd && f->brs && s_brs) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    h.FDFormat            = f->fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    h.MessageMarker       = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan) == 0) return -1;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &h, (uint8_t *)f->data) != HAL_OK) return -1;
    return 0;
}

int can_tx(const can_frame_t *in)
{
    if (!s_enabled) return -1;
    if (!in) return -4;
    can_frame_t fr = *in;
    if (can_frame_normalize(&fr) < 0) return -4;
    if (fr.fd && !s_fd) return -4;
    const can_frame_t *f = &fr;

    /* Bus-off: start the recovery (if the ISR has not) and refuse this frame;
       the core rejoins on its own and the next call goes through. */
    if (can_busoff_poll()) return -2;

    /* The RX ISR's responder also queues frames (can_tx_raw).  Two writers of
       the TX FIFO put index must not interleave, so mask the ISR while the
       task side checks the free level and adds its frame. */
    bool was = can_irq_mask();
    int rc = can_tx_raw(f);
    can_irq_restore(was);
    if (rc == -2) return -4;
    return (rc == 0) ? 0 : -3;
}

/* ---- receive (ISR producer / can_rx_pop consumer) ------------------------ */

void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan);
}

/* PSR.BO changed (IR.BO).  Recover right away so the autonomous responder
   keeps answering without waiting for a host poll.  No printf in an ISR: the
   task paths log the counter. */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *h, uint32_t its)
{
    if (h->Instance != CAN_FD) return;
    if ((its & FDCAN_IR_BO) == 0u) return;
    (void)can_busoff_recover_locked();
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h, uint32_t its)
{
    if (h->Instance != CAN_FD) return;
    if ((its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    /* Drain the hardware FIFO into the software ring while messages are queued. */
    while (HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0) > 0) {
        FDCAN_RxHeaderTypeDef rh;
        can_frame_t fr;
        if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &rh, fr.data) != HAL_OK) break;

        fr.ext = (rh.IdType == FDCAN_EXTENDED_ID);
        fr.id  = rh.Identifier;
        fr.rtr = (rh.RxFrameType == FDCAN_REMOTE_FRAME);
        fr.fd  = (rh.FDFormat == FDCAN_FD_CAN);
        fr.brs = fr.fd && (rh.BitRateSwitch == FDCAN_BRS_ON);
        fr.dlc = can_dlc_to_len(rh.DataLength);   /* classic 9..15 -> 8 bytes */
        if (!fr.fd && fr.dlc > 8) fr.dlc = 8;
        fr.ts  = HAL_GetTick();          /* device ms — jitter-free, ISR-stamped */
        if (!can_ring_push(&s_ring, &fr)) s_rx_overflow++;   /* ring full: drop */
        const can_frame_t *slot = &fr;

        /* Autonomous responder: reply immediately to a matching request. */
        for (uint32_t r = 0; r < CAN_RESPONDER_MAX; r++) {
            if (s_rules[r].used &&
                s_rules[r].match_ext == slot->ext &&
                s_rules[r].match_id  == slot->id) {
                if (can_tx_raw(&s_rules[r].reply) == 0) s_resp_hits++;
                break;
            }
        }
    }
}

int can_rx_peek(can_frame_t *out)
{
    if (!out) return 0;
    return can_ring_peek(&s_ring, out) ? 1 : 0;
}

int can_rx_pop(can_frame_t *out, int max)
{
    if (!out || max <= 0) return 0;
    (void)can_busoff_poll();   /* poll path: backstop for a missed bus-off IRQ */
    int n = 0;
    while (n < max && can_ring_pop(&s_ring, &out[n])) n++;
    return n;
}

/* ---- status -------------------------------------------------------------- */

int can_get_status(can_status_t *out)
{
    if (!out) return -1;
    memset(out, 0, sizeof(*out));
    out->enabled = s_enabled;
    out->mode    = s_mode;
    out->bitrate = s_bitrate;
    out->data_bitrate = s_data_bitrate;
    out->term    = s_term;
    out->rx_pending      = can_ring_count(&s_ring);
    out->rx_overflow     = s_rx_overflow;
    out->responder_rules = (uint32_t)can_responder_count();
    out->responder_hits  = s_resp_hits;

    if (s_enabled) {
        (void)can_busoff_poll();
        FDCAN_ErrorCountersTypeDef ec;
        FDCAN_ProtocolStatusTypeDef ps;
        if (HAL_FDCAN_GetErrorCounters(&hfdcan, &ec) == HAL_OK) {
            out->tec = (uint8_t)ec.TxErrorCnt;
            out->rec = (uint8_t)ec.RxErrorCnt;
        }
        if (HAL_FDCAN_GetProtocolStatus(&hfdcan, &ps) == HAL_OK) {
            out->bus_off       = ps.BusOff ? true : false;
            out->error_passive = ps.ErrorPassive ? true : false;
        }
    }
    out->bus_off_recoveries = s_busoff_recoveries;
    return 0;
}

/* ---- name helpers -------------------------------------------------------- */

int can_mode_from_name(const char *name, can_mode_t *out)
{
    if (!name || !out) return -1;
    if      (!strcmp(name, "normal"))   *out = CAN_MODE_NORMAL;
    else if (!strcmp(name, "internal")) *out = CAN_MODE_LOOPBACK_INTERNAL;
    else if (!strcmp(name, "external")) *out = CAN_MODE_LOOPBACK_EXTERNAL;
    else if (!strcmp(name, "listen"))   *out = CAN_MODE_LISTEN;
    else return -1;
    return 0;
}

const char *can_mode_name(can_mode_t mode)
{
    switch (mode) {
        case CAN_MODE_NORMAL:            return "normal";
        case CAN_MODE_LOOPBACK_INTERNAL: return "internal";
        case CAN_MODE_LOOPBACK_EXTERNAL: return "external";
        case CAN_MODE_LISTEN:            return "listen";
        default:                         return "?";
    }
}
