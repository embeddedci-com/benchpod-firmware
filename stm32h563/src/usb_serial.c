/* usb_serial.c — see usb_serial.h. USB_HOST=1 builds only. */
#include "usb_serial.h"

#include "tusb.h"
#include "FreeRTOS.h"
#include "task.h"

#define RX_SIZE  2048u   /* powers of two: the indices run free and wrap */
#define TX_SIZE  1024u
#define NO_PORT  0xFFu

/* Rings: head is the producer's, tail the consumer's (RX: usb task in, worker out; TX: the
   reverse). Each index has one writer. */
static uint8_t           s_rx[RX_SIZE], s_tx[TX_SIZE];
static volatile uint32_t s_rx_head, s_rx_tail, s_tx_head, s_tx_tail;

static volatile uint8_t  s_idx = NO_PORT;
static volatile uint32_t s_mounts;
static volatile bool     s_bridge;

enum { REQ_IDLE = 0, REQ_QUEUED, REQ_ISSUED, REQ_DONE };
enum { KIND_LINES = 0, KIND_BAUD };
static volatile uint8_t  s_state, s_kind;
static volatile bool     s_ok;
static volatile uint16_t s_line_state;
static volatile uint32_t s_rate;

static void fence(void) { __atomic_thread_fence(__ATOMIC_SEQ_CST); }

void tuh_cdc_mount_cb(uint8_t idx)
{
    /* tuh_cdc_itf_get_info reports every port as class CDC, the vendor chips (FTDI, CP210x, CH34x)
       too; the subclass is the interface's own, and cdc_host opens CDC ports only with ACM. */
    tuh_itf_info_t info;
    bool acm = tuh_cdc_itf_get_info(idx, &info) &&
               info.desc.bInterfaceSubClass == CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL;
    s_bridge = !acm;
    s_idx = idx;
    s_mounts++;
}

void tuh_cdc_umount_cb(uint8_t idx)
{
    if (s_idx != idx) return;
    s_idx = NO_PORT;
    if (s_state == REQ_QUEUED || s_state == REQ_ISSUED) { s_ok = false; s_state = REQ_DONE; }
}

bool     usb_serial_ready(void)     { return s_idx != NO_PORT; }
uint32_t usb_serial_mounts(void)    { return s_mounts; }
bool     usb_serial_is_bridge(void) { return s_bridge; }

static void on_req_done(tuh_xfer_t *x)
{
    s_ok = x->result == XFER_RESULT_SUCCESS;
    s_state = REQ_DONE;
}

void usb_serial_pump(void)
{
    uint8_t i = s_idx;
    if (i == NO_PORT || !tuh_cdc_mounted(i)) {
        s_tx_tail = s_tx_head;   /* nothing to send it to */
        if (s_state == REQ_QUEUED) { s_ok = false; s_state = REQ_DONE; }
        return;
    }
    /* RX: TinyUSB's FIFO into the ring. A full ring leaves the bytes in the FIFO, which holds
       the board back (no new IN transfer), so nothing is dropped. */
    uint8_t b[64];
    for (;;) {
        uint32_t room = RX_SIZE - (s_rx_head - s_rx_tail);
        uint32_t avail = tuh_cdc_read_available(i);
        if (!avail || !room) break;
        uint32_t want = avail < sizeof(b) ? avail : sizeof(b);
        if (want > room) want = room;
        uint32_t got = tuh_cdc_read(i, b, want);
        if (!got) break;
        uint32_t h = s_rx_head;
        for (uint32_t k = 0; k < got; k++) s_rx[(h + k) & (RX_SIZE - 1)] = b[k];
        fence();
        s_rx_head = h + got;
    }
    /* TX: the ring into TinyUSB's FIFO, then send it. */
    uint32_t pending = s_tx_head - s_tx_tail;
    if (pending) {
        uint32_t t = s_tx_tail;
        uint32_t chunk = TX_SIZE - (t & (TX_SIZE - 1));
        if (chunk > pending) chunk = pending;
        uint32_t w = tuh_cdc_write(i, &s_tx[t & (TX_SIZE - 1)], chunk);
        fence();
        s_tx_tail = t + w;
        (void)tuh_cdc_write_flush(i);
    }
    /* A line request. */
    if (s_state == REQ_QUEUED) {
        bool started = s_kind == KIND_LINES
            ? tuh_cdc_set_control_line_state(i, s_line_state, on_req_done, 0)
            : tuh_cdc_set_baudrate(i, s_rate, on_req_done, 0);
        if (started) s_state = REQ_ISSUED;   /* else busy: try again on the next pass */
    }
}

int usb_serial_write(const void *buf, uint32_t n, uint32_t timeout_ms)
{
    const uint8_t *p = buf;
    const TickType_t t0 = xTaskGetTickCount();
    while (n) {
        if (s_idx == NO_PORT) return -1;
        uint32_t h = s_tx_head;
        uint32_t room = TX_SIZE - (h - s_tx_tail);
        if (!room) {
            if (xTaskGetTickCount() - t0 > pdMS_TO_TICKS(timeout_ms)) return -1;
            vTaskDelay(1);
            continue;
        }
        uint32_t k = n < room ? n : room;
        for (uint32_t j = 0; j < k; j++) s_tx[(h + j) & (TX_SIZE - 1)] = p[j];
        fence();
        s_tx_head = h + k;
        p += k;
        n -= k;
    }
    return 0;
}

int usb_serial_getc(void)
{
    uint32_t t = s_rx_tail;
    if (t == s_rx_head) return -1;
    fence();
    uint8_t c = s_rx[t & (RX_SIZE - 1)];
    s_rx_tail = t + 1;
    return c;
}

uint32_t usb_serial_read(uint8_t *buf, uint32_t n, uint32_t timeout_ms)
{
    uint32_t got = 0;
    const TickType_t t0 = xTaskGetTickCount();
    while (got < n) {
        int c = usb_serial_getc();
        if (c >= 0) { buf[got++] = (uint8_t)c; continue; }
        if (xTaskGetTickCount() - t0 > pdMS_TO_TICKS(timeout_ms)) break;
        vTaskDelay(1);
    }
    return got;
}

void usb_serial_rx_clear(void) { s_rx_tail = s_rx_head; }

static int request(uint8_t kind, uint32_t timeout_ms)
{
    if (s_idx == NO_PORT) return -1;
    if (s_state == REQ_QUEUED || s_state == REQ_ISSUED) return -1;
    s_kind = kind;
    s_ok = false;
    fence();
    s_state = REQ_QUEUED;
    const TickType_t t0 = xTaskGetTickCount();
    while (s_state != REQ_DONE) {
        if (xTaskGetTickCount() - t0 > pdMS_TO_TICKS(timeout_ms)) {
            /* The request holds values, not a buffer: one still in flight completes on its own. */
            taskENTER_CRITICAL();
            if (s_state == REQ_QUEUED) s_state = REQ_IDLE;
            taskEXIT_CRITICAL();
            return -1;
        }
        vTaskDelay(1);
    }
    s_state = REQ_IDLE;
    return s_ok ? 0 : -1;
}

int usb_serial_lines(bool dtr, bool rts, uint32_t timeout_ms)
{
    s_line_state = (uint16_t)((dtr ? CDC_CONTROL_LINE_STATE_DTR : 0) | (rts ? CDC_CONTROL_LINE_STATE_RTS : 0));
    return request(KIND_LINES, timeout_ms);
}

int usb_serial_baud(uint32_t baud, uint32_t timeout_ms)
{
    s_rate = baud;
    return request(KIND_BAUD, timeout_ms);
}
