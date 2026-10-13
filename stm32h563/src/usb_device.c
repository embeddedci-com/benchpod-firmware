/*
 * usb_device.c — TinyUSB on USB_DRD_FS: the CDC-ACM console + the CDC-NCM network
 * adapter, all driven from one usb task.
 *
 * Only the usb task (and the USB interrupt) call into TinyUSB. The rest of the
 * firmware meets it through two buffers:
 *  - the console TX ring: any task, an ISR or early boot code writes it
 *    (console_io.c), the usb task drains it into the CDC FIFO;
 *  - the network frame slots: the usb task copies received frames in and the net
 *    task (which owns lwIP, NO_SYS) takes them out; the net task copies outgoing
 *    frames in and the usb task hands them to the NCM driver.
 * Each is single-producer / single-consumer per direction, so indexes need only
 * a barrier, except the console ring, whose producers are serialized by a critical
 * section (an ISR may log).
 *
 * NOTE: ICACHE stays DISABLED (we never enable it); see the UID read note in board_uid.h.
 */
#include "usb_device.h"
#include "console_io.h"
#include "board_uid.h"

#include "tusb.h"
#include "device/usbd_pvt.h"   /* usbd_defer_func: wake the usb task */
#include "stm32h5xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* ---- clocks, pins, interrupt ----------------------------------------------------------- */
static void usb_hw_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef pclk = {0};
    RCC_OscInitTypeDef osc = {0};
    RCC_CRSInitTypeDef crs = {0};

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

    /* At or below configMAX_SYSCALL_INTERRUPT_PRIORITY (5): the handler posts to a
       FreeRTOS queue. TinyUSB enables the IRQ itself. */
    HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 6, 0);
}

void USB_DRD_FS_IRQHandler(void)
{
    tud_int_handler(BOARD_TUD_RHPORT);
}

/* ---- critical section usable from a task, an ISR or before the scheduler -------------- */
static inline uint32_t crit_enter(void)
{
    if (xPortIsInsideInterrupt()) return taskENTER_CRITICAL_FROM_ISR();
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) { taskENTER_CRITICAL(); return 0; }
    __disable_irq();
    return 0;
}

static inline void crit_exit(uint32_t saved)
{
    if (xPortIsInsideInterrupt()) { taskEXIT_CRITICAL_FROM_ISR(saved); return; }
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) { taskEXIT_CRITICAL(); return; }
    __enable_irq();
}

/* ---- console ---------------------------------------------------------------------------- */
#define TX_RING_SIZE  2048   /* power of two */
static uint8_t           s_con_ring[TX_RING_SIZE];
static volatile uint16_t s_con_head, s_con_tail;
static volatile bool     s_mounted;

int cdc_if_connected(void) { return s_mounted; }

int cdc_if_write(const uint8_t *buf, uint16_t len)
{
    if (!s_mounted) return 0;
    int accepted = 0;
    uint32_t saved = crit_enter();
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((s_con_head + 1) & (TX_RING_SIZE - 1));
        if (next == s_con_tail) break;   /* ring full: drop the rest */
        s_con_ring[s_con_head] = buf[i];
        s_con_head = next;
        accepted++;
    }
    crit_exit(saved);
    return accepted;
}

static void console_tx_pump(void)
{
    bool wrote = false;
    while (s_con_head != s_con_tail) {
        uint16_t tail = s_con_tail, head = s_con_head;
        uint32_t contig = (head > tail) ? (uint32_t)(head - tail) : (uint32_t)(TX_RING_SIZE - tail);
        uint32_t room = tud_cdc_write_available();
        if (room == 0) break;
        if (contig > room) contig = room;
        uint32_t n = tud_cdc_write(&s_con_ring[tail], contig);
        if (n == 0) break;
        __DMB();
        s_con_tail = (uint16_t)((tail + n) & (TX_RING_SIZE - 1));
        wrote = true;
    }
    if (wrote) tud_cdc_write_flush();
}

void tud_cdc_rx_cb(uint8_t itf)
{
    (void)itf;
    uint8_t buf[64];
    uint32_t n;
    while ((n = tud_cdc_read(buf, sizeof(buf))) > 0) {
        /* The USART2 RX interrupt pushes into the same ring. */
        taskENTER_CRITICAL();
        for (uint32_t i = 0; i < n; i++) console_io_rx_push(buf[i]);
        taskEXIT_CRITICAL();
    }
}

/* ---- network frame slots ---------------------------------------------------------------- */
#define RX_SLOTS 3
#define TX_SLOTS 3

typedef struct {
    uint16_t len;
    uint8_t  data[USB_NET_FRAME_MAX];
} __attribute__((aligned(4))) net_slot_t;

static net_slot_t        s_rx[RX_SLOTS], s_tx[TX_SLOTS];
static volatile uint8_t  s_rx_head, s_rx_tail;   /* usb task fills head, net task frees tail */
static volatile uint8_t  s_tx_head, s_tx_tail;   /* net task fills head, usb task sends tail */
static bool              s_rx_renew;             /* usb task: ask the driver for the next frame */
static bool              s_rx_stalled;           /* usb task: a frame waits for a free slot */
static volatile bool     s_net_enabled = true;   /* off while Wi-Fi is provisioned (usb_net_set_enabled) */
static usb_net_stats_t   s_stats;
static TaskHandle_t      s_usb_task;
static volatile bool     s_kick;                 /* a wake-up is queued for the usb task */
static TaskHandle_t volatile s_tx_waiter;        /* the net task waits for a free transmit slot */

#define NEXT(i, n) ((uint8_t)(((i) + 1) % (n)))

bool usb_net_link_up(void) { return s_net_enabled && s_mounted && !tud_suspended(); }

bool usb_net_enabled(void) { return s_net_enabled; }

/* The NCM function stays enumerated either way (the console shares the device); off
   tells the host the cable is out, so it drops the address and its route. */
void usb_net_set_enabled(bool on)
{
    if (s_net_enabled == on) return;
    s_net_enabled = on;
    tud_network_link_state(BOARD_TUD_RHPORT, on);   /* deferred onto the usb task */
}

/* The NCM driver's link state at (re)enumeration. */
bool tud_network_default_link_state_cb(void) { return s_net_enabled; }

bool usb_net_rx_peek(const uint8_t **frame, uint16_t *len)
{
    if (s_rx_tail == s_rx_head) return false;
    __DMB();
    *frame = s_rx[s_rx_tail].data;
    *len   = s_rx[s_rx_tail].len;
    return true;
}

void usb_net_rx_release(void)
{
    if (s_rx_tail == s_rx_head) return;
    __DMB();
    s_rx_tail = NEXT(s_rx_tail, RX_SLOTS);
}

static void kick_cb(void *param) { (void)param; s_kick = false; }

/* Wake the usb task now rather than at its next 1 ms poll: an event in TinyUSB's queue
   ends its wait. One wake-up in flight is enough. */
static void usb_task_kick(void)
{
    if (s_kick || !s_usb_task) return;
    s_kick = true;
    usbd_defer_func(kick_cb, NULL, false);
}

uint8_t *usb_net_tx_slot(uint32_t wait_ms)
{
    if (NEXT(s_tx_head, TX_SLOTS) != s_tx_tail) return s_tx[s_tx_head].data;
    s_stats.tx_full++;
    if (wait_ms == 0) return NULL;
    /* The usb task notifies us as soon as it hands a slot to the NCM driver. */
    s_tx_waiter = xTaskGetCurrentTaskHandle();
    TickType_t t0 = xTaskGetTickCount(), limit = pdMS_TO_TICKS(wait_ms);
    while (NEXT(s_tx_head, TX_SLOTS) == s_tx_tail) {
        TickType_t gone = xTaskGetTickCount() - t0;
        if (gone >= limit) { s_tx_waiter = NULL; return NULL; }
        usb_task_kick();
        ulTaskNotifyTake(pdTRUE, limit - gone);
    }
    s_tx_waiter = NULL;
    return s_tx[s_tx_head].data;
}

void usb_net_tx_commit(uint16_t len)
{
    s_tx[s_tx_head].len = len;
    __DMB();
    s_tx_head = NEXT(s_tx_head, TX_SLOTS);
    usb_task_kick();
}

/* Runs inside the NCM driver (usb task). Returning false keeps the frame in the
   driver until tud_network_recv_renew() is called again. */
bool tud_network_recv_cb(const uint8_t *src, uint16_t size)
{
    if (size == 0 || size > USB_NET_FRAME_MAX || !s_net_enabled) {
        s_stats.rx_drops++;
        s_rx_renew = true;
        return true;   /* consumed (dropped) */
    }
    if (NEXT(s_rx_head, RX_SLOTS) == s_rx_tail) {
        s_rx_stalled = true;   /* the net task has not caught up: retry once a slot frees */
        return false;
    }
    net_slot_t *s = &s_rx[s_rx_head];
    memcpy(s->data, src, size);
    s->len = size;
    __DMB();
    s_rx_head = NEXT(s_rx_head, RX_SLOTS);
    s_stats.rx_frames++;
    s_rx_renew = true;
    return true;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg)
{
    const net_slot_t *s = (const net_slot_t *)ref;
    (void)arg;
    memcpy(dst, s->data, s->len);
    return s->len;
}

void tud_network_init_cb(void)
{
    /* The host (re)selected the interface: drop frames queued for it. The receive
       slots belong half to the net task; any left there are delivered as usual. */
    s_tx_tail = s_tx_head;
    s_rx_renew = s_rx_stalled = false;
}

static void net_pump(void)
{
    /* host -> pod: keep pulling datagrams out of the NTB while slots are free */
    if (s_rx_stalled && NEXT(s_rx_head, RX_SLOTS) != s_rx_tail) {
        s_rx_stalled = false;
        s_rx_renew = true;
    }
    while (s_rx_renew) {
        s_rx_renew = false;
        tud_network_recv_renew();
    }
    /* pod -> host */
    while (s_tx_tail != s_tx_head) {
        net_slot_t *s = &s_tx[s_tx_tail];
        __DMB();
        if (!tud_network_can_xmit(s->len)) break;
        tud_network_xmit(s, 0);
        s_stats.tx_frames++;
        s_tx_tail = NEXT(s_tx_tail, TX_SLOTS);
        TaskHandle_t w = s_tx_waiter;
        if (w) xTaskNotifyGive(w);
    }
}

void usb_net_stats(usb_net_stats_t *out) { *out = s_stats; }

/* Locally administered MACs from the UID: 02:... for the pod, 06:... for the host. */
static void uid_mac(uint8_t first, uint8_t mac[6])
{
    uint32_t uid[3];
    board_uid_words(uid);
    uint32_t a = uid[0] ^ uid[2], b = uid[1];
    mac[0] = first;
    mac[1] = (uint8_t)(b >> 8);
    mac[2] = (uint8_t)(a >> 24);
    mac[3] = (uint8_t)(a >> 16);
    mac[4] = (uint8_t)(a >> 8);
    mac[5] = (uint8_t)a;
}

void usb_net_dev_mac(uint8_t mac[6])  { uid_mac(0x02, mac); }
void usb_net_host_mac(uint8_t mac[6]) { uid_mac(0x06, mac); }

/* ---- the usb task ----------------------------------------------------------------------- */
void tud_mount_cb(void)   { s_mounted = true; }
void tud_umount_cb(void)  { s_mounted = false; }

static volatile bool s_reattach;

void usb_reattach_request(void) { s_reattach = true; }

/* Drop off the bus and come back: the host enumerates the device again, as after
   unplugging the cable. */
static void usb_reattach(void)
{
    s_reattach = false;
    for (int i = 0; i < 20 && s_con_head != s_con_tail; i++) {   /* the reply first */
        tud_task_ext(1, false);
        console_tx_pump();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(500));
    tud_connect();
}

static void usb_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_reattach) usb_reattach();
        /* Waits up to 1 ms for a USB event, so the console ring and the transmit slots,
           which other tasks fill without waking us, are drained at least every tick. */
        tud_task_ext(1, false);
        console_tx_pump();
        net_pump();
    }
}

void MX_USB_DEVICE_Init(void)
{
    usb_hw_init();

    tusb_rhport_init_t dev_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO };
    if (!tusb_init(BOARD_TUD_RHPORT, &dev_init)) return;

    /* Force the host to re-enumerate after a bootloader (DFU) hand-off. The ROM USB
       bootloader leaves D+ pulled up / enumerated; when we jump to the app the host
       keeps that stale enumeration and the console never appears until a full power
       cycle. Pulsing the D+ pull-up off -> on makes the host see a disconnect +
       reconnect and enumerate the app cleanly. */
    tud_disconnect();
    HAL_Delay(50);
    tud_connect();

    /* Above the net task (+2): the console and the frame slots must keep moving while
       the net task runs a TLS handshake. Its work per wake-up is short. */
    xTaskCreate(usb_task, "usb", 768, NULL, tskIDLE_PRIORITY + 3, &s_usb_task);
}
