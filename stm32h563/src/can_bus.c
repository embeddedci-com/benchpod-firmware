/*
 * can_bus.c — classic CAN over FDCAN1 (TCAN1044 transceiver U10).
 *
 * Single-node board: `normal` mode needs an external node to ACK; the loopback
 * modes (internal / external) let one pod self-test TX+RX end to end (see
 * can_bus.h).  Received frames are pushed from the FDCAN RX-FIFO0 interrupt into
 * a lock-free single-producer/single-consumer ring, drained by can_rx_pop().
 *
 * FDCAN kernel clock = HSE (25 MHz), selected in can_configure() via RCCEx.
 */
#include "can_bus.h"
#include "board_pins.h"
#include "stm32h5xx_hal.h"
#include <string.h>
#include <stdio.h>

/* HSE crystal feeding the FDCAN kernel clock (see main.c SystemClock_Config). */
#define CAN_KERNEL_CLK_HZ  25000000u

static FDCAN_HandleTypeDef hfdcan;
static bool     s_enabled;
static bool     s_term;
static can_mode_t s_mode;
static uint32_t s_bitrate;

/* SPSC RX ring: the FDCAN IT0 ISR is the only producer (advances `head`), and
   can_rx_pop() the only consumer (advances `tail`).  Power-of-two size so the
   mask wraps cheaply.  volatile indices give the required visibility. */
#define CAN_RX_RING  32u   /* must be a power of two */
static can_frame_t     s_ring[CAN_RX_RING];
static volatile uint32_t s_rx_head;
static volatile uint32_t s_rx_tail;
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
        printf("[can] bus-off: recovery started (%lu since boot), rejoining after 129x11 recessive bits\n",
               (unsigned long)n);
        s_busoff_logged = n;
    }
    return bo;
}

/* ---- bit timing ---------------------------------------------------------- */

/* Solve nominal bit timing for `bitrate` from a `fclk`-Hz kernel clock, aiming
   for an 87.5%-ish sample point.  Picks the largest total time-quanta count
   (8..25) that yields an integer prescaler in [1,512]; writes prescaler/seg1/
   seg2/sjw.  Returns 0, or -1 if no exact timing exists for this rate. */
static int can_solve_timing(uint32_t fclk, uint32_t bitrate,
                            uint32_t *presc, uint32_t *seg1,
                            uint32_t *seg2, uint32_t *sjw)
{
    if (bitrate == 0) return -1;
    for (uint32_t total = 25; total >= 8; total--) {
        uint32_t denom = bitrate * total;
        if (denom == 0 || (fclk % denom) != 0) continue;
        uint32_t p = fclk / denom;
        if (p < 1 || p > 512) continue;
        /* sample point ~87.5%: tseg1 spans sync(1)+prop+phase1. */
        uint32_t t1 = (total * 7u) / 8u;      /* 0.875 * total, floored */
        if (t1 < 2) t1 = 2;
        if (t1 > total - 2) t1 = total - 2;   /* leave >=1 for seg2 (+sync) */
        uint32_t s1 = t1 - 1;                  /* NominalTimeSeg1 excludes sync */
        uint32_t s2 = total - 1 - s1;          /* remaining quanta */
        if (s1 < 2 || s2 < 2 || s2 > 128 || s1 > 256) continue;
        *presc = p;
        *seg1  = s1;
        *seg2  = s2;
        *sjw   = (s2 < 4) ? s2 : 4;
        return 0;
    }
    return -1;
}

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

int can_responder_add(uint32_t match_id, bool match_ext, const can_frame_t *reply)
{
    if (!reply || reply->dlc > 8) return -1;
    for (uint32_t r = 0; r < CAN_RESPONDER_MAX; r++) {
        if (!s_rules[r].used) {
            s_rules[r].match_id  = match_id;
            s_rules[r].match_ext = match_ext;
            s_rules[r].reply     = *reply;
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
    s_rx_head = s_rx_tail = s_rx_overflow = 0;
    can_responder_clear();
    printf("[can] init: FDCAN1 PD1/PD0, term=PA4 (off), core idle\n");
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
    s_rx_head = s_rx_tail = 0;
    printf("[can] disabled\n");
}

int can_configure(uint32_t bitrate, can_mode_t mode, bool fd, bool term)
{
    uint32_t presc = 0, seg1 = 0, seg2 = 0, sjw = 0;
    (void)fd;   /* classic CAN only for now — FD reserved */

    if (can_solve_timing(CAN_KERNEL_CLK_HZ, bitrate, &presc, &seg1, &seg2, &sjw) != 0) {
        printf("[can] no bit timing for %lu bit/s @ %lu Hz kernel\n",
               (unsigned long)bitrate, (unsigned long)CAN_KERNEL_CLK_HZ);
        return -1;
    }

    /* Re-configuration: tear the old instance down first (idempotent). */
    can_disable();

    /* FDCAN kernel clock = HSE (25 MHz).  HSE is already on (PLL1 source). */
    RCC_PeriphCLKInitTypeDef pclk = {0};
    pclk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    pclk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) return -2;
    __HAL_RCC_FDCAN_CLK_ENABLE();

    can_gpio_init();

    hfdcan.Instance                  = CAN_FD;
    hfdcan.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    hfdcan.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    hfdcan.Init.Mode                 = hal_mode(mode);
    hfdcan.Init.AutoRetransmission   = ENABLE;
    hfdcan.Init.TransmitPause        = DISABLE;
    hfdcan.Init.ProtocolException    = DISABLE;
    hfdcan.Init.NominalPrescaler     = presc;
    hfdcan.Init.NominalSyncJumpWidth = sjw;
    hfdcan.Init.NominalTimeSeg1      = seg1;
    hfdcan.Init.NominalTimeSeg2      = seg2;
    hfdcan.Init.DataPrescaler        = presc;   /* unused (classic) but must be valid */
    hfdcan.Init.DataSyncJumpWidth    = (sjw < 16) ? sjw : 16;
    hfdcan.Init.DataTimeSeg1         = (seg1 < 32) ? seg1 : 31;
    hfdcan.Init.DataTimeSeg2         = (seg2 < 16) ? seg2 : 15;
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

    if (HAL_FDCAN_Start(&hfdcan) != HAL_OK) return -2;

    /* New-message-in-FIFO0 + bus-off interrupts -> IT0 line (ILS default). */
    if (HAL_FDCAN_ActivateNotification(&hfdcan,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
        return -2;
    }
    HAL_NVIC_SetPriority(CAN_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN_IRQn);

    s_enabled = true;
    s_mode    = mode;
    s_bitrate = bitrate;
    s_rx_head = s_rx_tail = s_rx_overflow = 0;
    can_set_term(term);

    printf("[can] up: %lu bit/s mode=%s presc=%lu seg1=%lu seg2=%lu sjw=%lu term=%d\n",
           (unsigned long)bitrate, can_mode_name(mode),
           (unsigned long)presc, (unsigned long)seg1, (unsigned long)seg2,
           (unsigned long)sjw, term ? 1 : 0);
    return 0;
}

/* ---- transmit ------------------------------------------------------------ */

/* Build a classic-CAN Tx header + queue the frame.  No enabled/bus-off checks
   (callers do those) so it is safe to call from the RX ISR (responder path).
   Returns 0, or -1 if the TX FIFO is full / HAL rejected it. */
static int can_tx_raw(const can_frame_t *f)
{
    FDCAN_TxHeaderTypeDef h = {0};
    h.Identifier          = f->ext ? (f->id & 0x1FFFFFFFu) : (f->id & 0x7FFu);
    h.IdType              = f->ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    h.TxFrameType         = f->rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    h.DataLength          = f->dlc;   /* classic 0..8: DLC code == byte count */
    h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch       = FDCAN_BRS_OFF;
    h.FDFormat            = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    h.MessageMarker       = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan) == 0) return -1;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &h, (uint8_t *)f->data) != HAL_OK) return -1;
    return 0;
}

int can_tx(const can_frame_t *f)
{
    if (!s_enabled) return -1;
    if (!f || f->dlc > 8) return -3;

    /* Bus-off: start the recovery (if the ISR has not) and refuse this frame;
       the core rejoins on its own and the next call goes through. */
    if (can_busoff_poll()) return -2;

    /* The RX ISR's responder also queues frames (can_tx_raw).  Two writers of
       the TX FIFO put index must not interleave, so mask the ISR while the
       task side checks the free level and adds its frame. */
    bool was = can_irq_mask();
    int rc = can_tx_raw(f);
    can_irq_restore(was);
    return (rc == 0) ? 0 : -3;
}

/* ---- receive (ISR producer / can_rx_pop consumer) ------------------------ */

/* DLC code -> byte count (classic: 0..8 == identity; FD codes clamp to 8 since
   we never enable FD, but keep the table correct for safety). */
static uint8_t dlc_to_bytes(uint32_t dlc)
{
    static const uint8_t t[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    uint8_t n = t[dlc & 0xF];
    return (n > 8) ? 8 : n;
}

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
        uint8_t data[8] = {0};
        if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &rh, data) != HAL_OK) break;

        uint32_t head = s_rx_head;
        uint32_t next = (head + 1u) & (CAN_RX_RING - 1u);
        if (next == s_rx_tail) {          /* ring full — drop this frame */
            s_rx_overflow++;
            continue;
        }
        can_frame_t *slot = &s_ring[head];
        slot->ext = (rh.IdType == FDCAN_EXTENDED_ID);
        slot->id  = rh.Identifier;
        slot->rtr = (rh.RxFrameType == FDCAN_REMOTE_FRAME);
        slot->dlc = dlc_to_bytes(rh.DataLength);
        slot->ts  = HAL_GetTick();        /* device ms — jitter-free, ISR-stamped */
        memcpy(slot->data, data, sizeof(slot->data));
        s_rx_head = next;                 /* publish AFTER the slot is written */

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

int can_rx_pop(can_frame_t *out, int max)
{
    if (!out || max <= 0) return 0;
    (void)can_busoff_poll();   /* poll path: backstop for a missed bus-off IRQ */
    int n = 0;
    while (n < max && s_rx_tail != s_rx_head) {
        out[n++] = s_ring[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1u) & (CAN_RX_RING - 1u);
    }
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
    out->term    = s_term;
    uint32_t pending = (s_rx_head - s_rx_tail) & (CAN_RX_RING - 1u);
    out->rx_pending      = pending;
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
