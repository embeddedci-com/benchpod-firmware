/* usb_serial.c — see usb_serial.h. USB_HOST=1 builds only. */
#include "usb_serial.h"

#include "tusb.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

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
static volatile uint16_t s_vid;            /* the mounted port's USB vendor ID (the chip's name) */

enum { REQ_IDLE = 0, REQ_QUEUED, REQ_ISSUED, REQ_DONE };
enum { KIND_LINES = 0, KIND_BAUD, KIND_CODING };
static volatile uint8_t  s_state, s_kind;
static volatile uint8_t  s_bits, s_par, s_stop;   /* KIND_CODING (with s_rate) */
static volatile bool     s_ok;
static volatile uint16_t s_line_state;
static volatile uint32_t s_rate;

static void fence(void) { __atomic_thread_fence(__ATOMIC_SEQ_CST); }

/* The console (usb_serial_con.h): a heap ring while it is open (pvPortMalloc at open, freed at
   close). The ring is touched from the usb task (pump) and the hw worker, always inside a
   critical section; close takes the pointer out in one, so the pump never sees a freed ring. */
#define CON_RX_SIZE 4096u
static usb_ring_t        s_con_ring;
static volatile bool     s_con_open;     /* the console is open */
static volatile bool     s_con_bound;    /* ... and bound to a mounted port of its kind */
static bool              s_con_bridge;   /* the kind of port it opened (a chip, or the board's own CDC) */
static uint32_t          s_con_mounts;   /* the mount it is bound to */
static uint32_t          s_con_baud, s_con_rx_total, s_con_tx_total;
static uint8_t           s_con_bits, s_con_par, s_con_stop;
static bool              s_con_dtr, s_con_rts;

void tuh_cdc_mount_cb(uint8_t idx)
{
    /* tuh_cdc_itf_get_info reports every port as class CDC, the vendor chips (FTDI, CP210x, CH34x)
       too; the subclass is the interface's own, and cdc_host opens CDC ports only with ACM. */
    tuh_itf_info_t info;
    bool acm = tuh_cdc_itf_get_info(idx, &info) &&
               info.desc.bInterfaceSubClass == CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL;
    s_bridge = !acm;
    uint16_t vid = 0, pid = 0;
    (void)tuh_vid_pid_get(info.daddr, &vid, &pid);
    s_vid = vid;
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
    uint8_t b[64];
    if (s_con_open) {
        /* The console (usb_serial command): everything goes into its ring, and what does not fit is
           counted as dropped. Until the console is bound to this port the bytes stay in the FIFO. */
        uint32_t budget = 1024;   /* per pass, so the usb task stays live under a flood */
        while (s_con_bound && budget) {
            uint32_t avail = tuh_cdc_read_available(i);
            if (!avail) break;
            uint32_t got = tuh_cdc_read(i, b, avail < sizeof(b) ? avail : sizeof(b));
            if (!got) break;
            budget = got < budget ? budget - got : 0;
            taskENTER_CRITICAL();
            if (s_con_ring.buf) (void)usb_ring_push(&s_con_ring, b, got);
            taskEXIT_CRITICAL();
            s_con_rx_total += got;
        }
    } else
    /* RX: TinyUSB's FIFO into the ring. A full ring leaves the bytes in the FIFO, which holds
       the board back (no new IN transfer), so nothing is dropped. */
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
        bool started;
        if (s_kind == KIND_LINES) {
            started = tuh_cdc_set_control_line_state(i, s_line_state, on_req_done, 0);
        } else if (s_kind == KIND_CODING) {
            cdc_line_coding_t c = { .bit_rate = s_rate, .stop_bits = s_stop, .parity = s_par, .data_bits = s_bits };
            started = tuh_cdc_set_line_coding(i, &c, on_req_done, 0);
        } else {
            started = tuh_cdc_set_baudrate(i, s_rate, on_req_done, 0);
        }
        if (started) s_state = REQ_ISSUED;   /* else busy: try again on the next pass */
    }
}

int usb_serial_write(const void *buf, uint32_t n, uint32_t timeout_ms)
{
    return usb_serial_write_some(buf, n, timeout_ms) == n ? 0 : -1;
}

uint32_t usb_serial_write_some(const void *buf, uint32_t n, uint32_t timeout_ms)
{
    const uint8_t *p = buf;
    const uint32_t total = n;
    const TickType_t t0 = xTaskGetTickCount();
    while (n) {
        if (s_idx == NO_PORT) break;
        uint32_t h = s_tx_head;
        uint32_t room = TX_SIZE - (h - s_tx_tail);
        if (!room) {
            if (xTaskGetTickCount() - t0 > pdMS_TO_TICKS(timeout_ms)) break;
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
    return total - n;
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

int usb_serial_coding(uint32_t baud, uint8_t data_bits, uint8_t parity, uint8_t stop_bits, uint32_t timeout_ms)
{
    s_rate = baud;
    s_bits = data_bits;
    s_par = parity;
    s_stop = stop_bits;
    return request(KIND_CODING, timeout_ms);
}

/* ---- the console ------------------------------------------------------------------------- */

#define CON_LINE_MS 200u

static void con_apply(void)
{
    /* A port that does not take a line request still streams: the result is not an error. */
    (void)usb_serial_coding(s_con_baud, s_con_bits, s_con_par, s_con_stop, CON_LINE_MS);
    (void)usb_serial_lines(s_con_dtr, s_con_rts, CON_LINE_MS);
}

static void con_set_from(const usb_serial_args_t *a)
{
    if (a->has_baud) s_con_baud = a->baud;
    if (a->has_dtr)  s_con_dtr = a->dtr;
    if (a->has_rts)  s_con_rts = a->rts;
    if (a->has_frame) { s_con_bits = a->data_bits; s_con_par = a->parity; s_con_stop = a->stop_bits; }
}

bool usb_serial_con_is_open(void) { return s_con_open; }

int usb_serial_con_open(const usb_serial_args_t *a)
{
    if (s_idx == NO_PORT) return -1;
    if (!s_con_open) {
        uint8_t *mem = pvPortMalloc(CON_RX_SIZE);
        if (!mem) return -2;
        usb_ring_init(&s_con_ring, mem, CON_RX_SIZE);   /* the pump does not look before s_con_open */
        s_con_rx_total = s_con_tx_total = 0;
        s_con_baud = 115200u;
        s_con_dtr = s_con_rts = true;
        s_con_bits = 8; s_con_par = USB_SER_PARITY_NONE; s_con_stop = USB_SER_STOP_1;
        con_set_from(a);
        s_con_bridge = s_bridge;
        usb_serial_rx_clear();
        s_con_mounts = s_mounts;
        s_con_bound = false;
        fence();
        s_con_open = true;
    } else {
        con_set_from(a);
    }
    con_apply();
    s_con_bound = true;
    return 0;
}

void usb_serial_con_close(void)
{
    uint8_t *mem;
    taskENTER_CRITICAL();
    s_con_open = false;
    s_con_bound = false;
    mem = s_con_ring.buf;
    s_con_ring.buf = NULL;
    taskEXIT_CRITICAL();
    vPortFree(mem);
}

void usb_serial_con_set(const usb_serial_args_t *a)
{
    if (!s_con_open) return;
    con_set_from(a);
    if (s_con_bound && s_idx != NO_PORT) {
        if (a->has_baud || a->has_frame) (void)usb_serial_coding(s_con_baud, s_con_bits, s_con_par, s_con_stop, CON_LINE_MS);
        if (a->has_dtr || a->has_rts) (void)usb_serial_lines(s_con_dtr, s_con_rts, CON_LINE_MS);
    }
}

void usb_serial_con_poll(void)
{
    if (!s_con_open) return;
    if (s_idx == NO_PORT) { s_con_bound = false; return; }
    if (s_con_bound && s_mounts == s_con_mounts) return;
    if (s_bridge != s_con_bridge) { s_con_bound = false; return; }   /* another kind of port: not ours */
    s_con_mounts = s_mounts;
    con_apply();
    s_con_bound = true;
}

bool usb_serial_con_present(void) { return s_con_open && s_con_bound && s_idx != NO_PORT; }

uint32_t usb_serial_con_read(uint8_t *buf, uint32_t n)
{
    uint32_t k = 0;
    taskENTER_CRITICAL();
    if (s_con_ring.buf) k = usb_ring_pop(&s_con_ring, buf, n);
    taskEXIT_CRITICAL();
    return k;
}

uint32_t usb_serial_con_write(const void *buf, uint32_t n, uint32_t timeout_ms)
{
    uint32_t k = usb_serial_write_some(buf, n, timeout_ms);
    s_con_tx_total += k;
    return k;
}

void usb_serial_con_state(usb_serial_state_t *st)
{
    memset(st, 0, sizeof(*st));
    st->open = s_con_open;
    st->baud = s_con_baud;
    st->dtr = s_con_dtr;
    st->rts = s_con_rts;
    st->data_bits = s_con_open ? s_con_bits : 8;
    st->stop_bits = s_con_stop;
    st->parity = s_con_par;
    st->bridge = usb_serial_chip_name(s_bridge, s_vid);
    st->port_present = usb_serial_con_present() || (!s_con_open && s_idx != NO_PORT);
    st->tx_total = s_con_tx_total;
    st->rx_total = s_con_rx_total;
    taskENTER_CRITICAL();
    if (s_con_ring.buf) { st->rx_buffered = usb_ring_avail(&s_con_ring); st->rx_dropped = s_con_ring.dropped; }
    taskEXIT_CRITICAL();
}
