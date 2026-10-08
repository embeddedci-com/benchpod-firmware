/*
 * test_hw_worker.c — host tests for the hw worker's queue handoff (src/hw_worker.c), above all
 * the connection teardown that is never dropped: a close or tunnel reset that finds the queue
 * full is parked, the slot stays busy, bytes for that slot are refused until it ran, and the
 * worker applies it only after everything queued before it. Also the single-slot cloud reply and
 * hw_worker_can_accept.
 *
 * hw_worker.c is included (its statics and worker steps are reached directly); shim_worker holds
 * the FreeRTOS and HAL headers, and the queue is a bounded FIFO here.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/hw_worker.c"

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

/* ---- fake queue ------------------------------------------------------------------------ */
struct shim_queue { size_t depth, item, head, n; unsigned char *buf; };

QueueHandle_t xQueueCreate(UBaseType_t depth, UBaseType_t item_size) {
    struct shim_queue *q = calloc(1, sizeof(*q));
    q->depth = depth; q->item = item_size; q->buf = calloc(depth, item_size);
    return q;
}
BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t wait) {
    (void)wait;
    if (q->n == q->depth) return pdFALSE;
    memcpy(q->buf + ((q->head + q->n) % q->depth) * q->item, item, q->item);
    q->n++;
    return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t wait) {
    (void)wait;
    if (q->n == 0) return pdFALSE;
    memcpy(item, q->buf + q->head * q->item, q->item);
    q->head = (q->head + 1) % q->depth;
    q->n--;
    return pdTRUE;
}
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q) { return q->n; }
UBaseType_t uxQueueSpacesAvailable(QueueHandle_t q) { return q->depth - q->n; }

/* ---- what the worker calls: an event log --------------------------------------------- */
#define EV_MAX 256
static char s_ev[EV_MAX][24];
static int  s_ev_n;
static void ev(const char *fmt, int id) {
    if (s_ev_n < EV_MAX) snprintf(s_ev[s_ev_n++], sizeof(s_ev[0]), fmt, id);
}
static void ev_reset(void) { s_ev_n = 0; }
static int ev_find(const char *e) {
    for (int i = 0; i < s_ev_n; i++) if (strcmp(s_ev[i], e) == 0) return i;
    return -1;
}

void command_handler_process(int conn_id, const uint8_t *buf, size_t len) {
    (void)len;
    char tag[24];
    snprintf(tag, sizeof(tag), "bytes%d:%c", conn_id, buf[0]);
    if (s_ev_n < EV_MAX) strcpy(s_ev[s_ev_n++], tag);
}
void command_handler_conn_closed(int conn_id)  { ev("closed%d", conn_id); }
void command_handler_tunnel_reset(int conn_id) { ev("reset%d", conn_id); }
size_t command_handler_dispatch_cloud(const char *json, char *out, size_t cap) {
    return (size_t)snprintf(out, cap, "{\"echo\":%s}", json);
}
void console_run_line(const char *line) { (void)line; ev("console%d", 0); }
void command_handler_poll(void) {}
void signal_engine_poll(void) {}
void target_power_poll(void) {}
void watchdog_heartbeat(int task, const char *name) { (void)task; (void)name; }
void watchdog_test_hang_point(int task) { (void)task; }
void boot_deferred_hw_init(void) {}
bool heavy_or_claimed(void) { return false; }
int  esp_rom_flash_from_slot(void) { return 0; }
void esp_wifi_ctrl_flash_done(bool ok) { (void)ok; }
int  ota_target_from_name(const char *n) { (void)n; return 0; }
void ota_refuse_for(ota_owner_t who, const char *why) { (void)who; (void)why; }
const char *ota_begin_gate(ota_owner_t who) { (void)who; return NULL; }
int  ota_begin_owned_b64(ota_owner_t who, uint32_t size, const char *sha, ota_target_t t, uint32_t v,
                         const char *sig_b64) {
    (void)who; (void)size; (void)sha; (void)t; (void)v; (void)sig_b64; return 0;
}
int  ota_data_by(ota_owner_t who, uint32_t off, const uint8_t *b, uint32_t len) { (void)who; (void)off; (void)b; (void)len; return 0; }
int  ota_end_by(ota_owner_t who) { (void)who; return 0; }
int  ota_abort_by(ota_owner_t who) { (void)who; return 0; }
const char *ota_busy_replace_for(ota_owner_t who) { (void)who; return NULL; }
const char *ota_busy_for(ota_owner_t who) { (void)who; return NULL; }
int  ota_commit(void) { return 0; }
void ota_watchdog(uint32_t now) { (void)now; }
uint32_t HAL_GetTick(void) { return 0; }
size_t xPortGetFreeHeapSize(void) { return 0; }
size_t xPortGetMinimumEverFreeHeapSize(void) { return 0; }
BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack, void *arg,
                       UBaseType_t prio, void *handle) {
    (void)fn; (void)name; (void)stack; (void)arg; (void)prio; (void)handle; return pdPASS;
}
void vTaskDelay(TickType_t t) { (void)t; }
volatile uint32_t g_malloc_failures;

/* One pass of worker_task's loop body (the queue part). */
static void worker_step(void) {
    static cmd_work_t w;
    for (int i = 0; i < HW_WORK_QUEUE_DEPTH; i++) {
        if (xQueueReceive(s_q, &w, 0) != pdTRUE) break;
        handle_work(&w);
    }
    apply_parked_teardowns();
}

static void drain(void) { for (int i = 0; i < 8; i++) worker_step(); }

static void fill(int conn_id, int n) {
    for (int i = 0; i < n; i++) {
        uint8_t b = (uint8_t)('a' + (i % 26));
        CHECK(hw_worker_submit_bytes(conn_id, &b, 1), "fill %d/%d refused", i, n);
    }
}

/* ---- tests ----------------------------------------------------------------------------- */

static void test_close_fits(void) {
    ev_reset();
    fill(3, 2);
    hw_worker_submit_closed(3);
    CHECK(hw_worker_conn_busy(3), "slot not busy while its close is queued");
    CHECK(s_parked[3] == 0, "parked although the queue had room");
    worker_step();
    CHECK(!hw_worker_conn_busy(3), "slot still busy after the close ran");
    CHECK(ev_find("bytes3:a") == 0 && ev_find("bytes3:b") == 1 && ev_find("closed3") == 2,
          "order: %s %s %s", s_ev[0], s_ev[1], s_ev[2]);
}

static void test_close_parked_when_full(void) {
    ev_reset();
    fill(3, HW_WORK_QUEUE_DEPTH);                  /* a bulk upload filled the queue */
    hw_worker_submit_closed(3);                    /* the FIN right behind it */
    CHECK(s_parked[3] == PEND_CLOSED, "close not parked: %u", s_parked[3]);
    CHECK(hw_worker_conn_busy(3), "slot not busy while the close is parked");
    /* bytes for that slot are refused until the close ran (the caller retries) */
    uint8_t x = 'Z';
    CHECK(!hw_worker_submit_bytes(3, &x, 1), "bytes queued behind a parked close");
    /* a later reset for the same slot parks too, even with room, so it stays behind the close */
    worker_step();                                 /* drains the 16 bytes; queue empty -> applies */
    CHECK(ev_find("closed3") == HW_WORK_QUEUE_DEPTH, "close not after the queued bytes (at %d)",
          ev_find("closed3"));
    CHECK(!hw_worker_conn_busy(3) && s_parked[3] == 0, "close not applied");
    CHECK(hw_worker_submit_bytes(3, &x, 1), "bytes refused after the close ran");
    drain();
}

static void test_parked_close_then_reset(void) {
    ev_reset();
    fill(7, HW_WORK_QUEUE_DEPTH);
    hw_worker_submit_closed(7);
    /* one item drained: the queue has room now, but the reset must not overtake the close */
    cmd_work_t w;
    CHECK(xQueueReceive(s_q, &w, 0) == pdTRUE, "receive");
    handle_work(&w);
    hw_worker_submit_tunnel_reset(7);
    CHECK(s_parked[7] == (PEND_CLOSED | PEND_RESET), "reset not parked behind the close: %u", s_parked[7]);
    worker_step();
    int c = ev_find("closed7"), r = ev_find("reset7");
    CHECK(c >= 0 && r == c + 1, "close %d, reset %d (want reset right after close)", c, r);
    CHECK(!hw_worker_conn_busy(7), "slot busy after both ran");
}

static void test_parked_waits_for_older_work(void) {
    ev_reset();
    fill(2, HW_WORK_QUEUE_DEPTH);
    hw_worker_submit_closed(2);
    /* the worker can only take part of the queue in one pass: the parked close waits */
    cmd_work_t w;
    for (int i = 0; i < 4; i++) { CHECK(xQueueReceive(s_q, &w, 0) == pdTRUE, "receive"); handle_work(&w); }
    apply_parked_teardowns();
    CHECK(ev_find("closed2") < 0, "parked close ran before the older queued work");
    CHECK(hw_worker_conn_busy(2), "slot freed early");
    /* other slots keep flowing meanwhile */
    uint8_t y = 'y';
    CHECK(hw_worker_submit_bytes(5, &y, 1), "another slot refused");
    drain();
    CHECK(ev_find("closed2") > ev_find("bytes5:y"), "close not after everything queued before it ran");
}

static void test_out_of_range_id(void) {
    ev_reset();
    hw_worker_submit_closed(HW_PEND_IDS + 2);      /* not tracked, still delivered */
    CHECK(!hw_worker_conn_busy(HW_PEND_IDS + 2), "untracked id reads busy");
    worker_step();
    CHECK(ev_find("closed18") >= 0, "untracked close not delivered");
    hw_worker_submit_tunnel_reset(-1);
    worker_step();
    CHECK(ev_find("reset-1") >= 0, "reset for -1 not delivered");
    /* a full queue drops an untracked teardown (nowhere to park it) without touching others */
    fill(1, HW_WORK_QUEUE_DEPTH);
    hw_worker_submit_closed(HW_PEND_IDS);
    for (int i = 0; i < HW_PEND_IDS; i++) CHECK(s_parked[i] == 0, "slot %d parked", i);
    drain();
}

static void test_cloud_slot(void) {
    char id[BP_TUNNEL_ID_MAX], reply[256];
    size_t n = 0;
    CHECK(!hw_worker_take_cloud_reply(id, sizeof(id), reply, sizeof(reply), &n), "reply before any command");
    CHECK(hw_worker_submit_cloud("r1", "{\"cmd\":\"ping\"}"), "first cloud command refused");
    CHECK(!hw_worker_submit_cloud("r2", "{\"cmd\":\"status\"}"), "second accepted while one is in flight");
    worker_step();
    CHECK(!hw_worker_submit_cloud("r2", "{\"cmd\":\"status\"}"), "accepted while the reply waits");
    CHECK(hw_worker_take_cloud_reply(id, sizeof(id), reply, sizeof(reply), &n), "no reply");
    CHECK(strcmp(id, "r1") == 0 && strcmp(reply, "{\"echo\":{\"cmd\":\"ping\"}}") == 0 && n == strlen(reply),
          "reply %s '%s'", id, reply);
    CHECK(hw_worker_submit_cloud("r2", "{\"cmd\":\"status\"}"), "slot not freed by the take");
    worker_step();
    /* a reply cut to the caller's buffer stays terminated */
    char small[8];
    CHECK(hw_worker_take_cloud_reply(id, sizeof(id), small, sizeof(small), &n) && n == 7 && small[7] == '\0',
          "cut reply n=%zu", n);
    /* a full queue frees the slot again instead of wedging it */
    fill(4, HW_WORK_QUEUE_DEPTH);
    CHECK(!hw_worker_submit_cloud("r3", "{}"), "accepted into a full queue");
    drain();
    CHECK(hw_worker_submit_cloud("r3", "{}"), "slot wedged after a full-queue refusal");
    drain();
    CHECK(hw_worker_take_cloud_reply(id, sizeof(id), reply, sizeof(reply), &n), "r3 reply");
}

static void test_can_accept(void) {
    drain();
    CHECK(hw_worker_can_accept(0) && hw_worker_can_accept(1), "empty queue");
    CHECK(hw_worker_can_accept((size_t)HW_WORK_DATA_MAX * HW_WORK_QUEUE_DEPTH), "exactly the queue");
    CHECK(!hw_worker_can_accept((size_t)HW_WORK_DATA_MAX * HW_WORK_QUEUE_DEPTH + 1), "one byte over");
    fill(6, HW_WORK_QUEUE_DEPTH - 1);
    CHECK(hw_worker_can_accept(HW_WORK_DATA_MAX) && !hw_worker_can_accept(HW_WORK_DATA_MAX + 1),
          "one slot left");
    drain();
}

int main(void) {
    hw_worker_init();
    CHECK(s_q != NULL, "queue not created");
    test_close_fits();
    test_close_parked_when_full();
    test_parked_close_then_reset();
    test_parked_waits_for_older_work();
    test_out_of_range_id();
    test_cloud_slot();
    test_can_accept();
    if (failures) { printf("test_hw_worker: %d FAILED\n", failures); return 1; }
    printf("test_hw_worker: all passed\n");
    return 0;
}
