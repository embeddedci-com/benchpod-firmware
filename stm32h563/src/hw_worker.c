#include "hw_worker.h"
#include "command_handler.h"
#include "signal_engine.h"
#include "target_power.h"
#include "console.h"
#include "watchdog.h"
#include "sys_health.h"
#include "bp_limits.h"
#include "ota.h"
#include "stm32h5xx_hal.h"   /* HAL_GetTick for the OTA staging watchdog */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>
#include <stdio.h>

/* HW_WORK_DATA_MAX / HW_WORK_QUEUE_DEPTH live in hw_worker.h so net_server.c can
   static_assert the "a full TCP window fits the queue" invariant against TCP_WND. */

enum { WK_BYTES = 0, WK_CONSOLE, WK_CLOUD, WK_CLOSED, WK_TUNNEL_RESET,
       WK_OTA_BEGIN, WK_OTA_DATA, WK_OTA_END, WK_OTA_ABORT, WK_OTA_COMMIT };

typedef struct {
    uint8_t  kind;
    int16_t  conn_id;
    uint16_t len;
    uint32_t u32;                         /* OTA: size (begin) / offset (data)   */
    uint8_t  data[HW_WORK_DATA_MAX + 1];  /* +1 so data[len]='\0' is always in bounds */
    char     req_id[BP_TUNNEL_ID_MAX];    /* cloud command / OTA sha256 hex       */
} cmd_work_t;

static QueueHandle_t s_q;

/* Scratch work item for producers (each producer runs on its own task; the queue
   copies by value, but the staging buffer must not be shared — use a local). */

/* ---- cloud command reply handoff (worker -> net), single slot ------------- */
static volatile bool s_cloud_pending;    /* a cloud command is in flight       */
static volatile bool s_cloud_ready;      /* its reply is ready to be taken      */
static char          s_cloud_req_id[BP_TUNNEL_ID_MAX];
static char          s_cloud_reply[BP_CLOUD_REPLY_MAX];
static size_t        s_cloud_reply_len;

/* ---- producers ------------------------------------------------------------ */

static bool submit_u(uint8_t kind, int conn_id, const uint8_t *buf, size_t len,
                     const char *req_id, uint32_t u32, TickType_t wait) {
    if (!s_q) return false;
    cmd_work_t w;
    w.kind    = kind;
    w.conn_id = (int16_t)conn_id;
    w.u32     = u32;
    if (len > HW_WORK_DATA_MAX) len = HW_WORK_DATA_MAX;
    w.len = (uint16_t)len;
    if (buf && len) memcpy(w.data, buf, len);
    w.req_id[0] = '\0';
    if (req_id) { strncpy(w.req_id, req_id, sizeof(w.req_id) - 1); w.req_id[sizeof(w.req_id) - 1] = '\0'; }
    return xQueueSend(s_q, &w, wait) == pdTRUE;
}

static bool submit(uint8_t kind, int conn_id, const uint8_t *buf, size_t len,
                   const char *req_id, TickType_t wait) {
    return submit_u(kind, conn_id, buf, len, req_id, 0, wait);
}

bool hw_worker_submit_bytes(int conn_id, const uint8_t *buf, size_t len) {
    return submit(WK_BYTES, conn_id, buf, len, NULL, 0);
}

/* True iff `len` bytes can be enqueued RIGHT NOW as ceil(len/HW_WORK_DATA_MAX)
   work items without blocking.  net_server uses this for all-or-nothing pbuf
   acceptance so a fast bulk upload (e.g. a deep-replay load_bin) is never
   partially consumed and truncated when the queue momentarily fills. */
bool hw_worker_can_accept(size_t len) {
    if (!s_q) return false;
    size_t segs = (len + (HW_WORK_DATA_MAX - 1)) / HW_WORK_DATA_MAX;
    if (segs == 0) segs = 1;
    return uxQueueSpacesAvailable(s_q) >= segs;
}

void hw_worker_submit_closed(int conn_id) {
    submit(WK_CLOSED, conn_id, NULL, 0, NULL, 0);
}

void hw_worker_submit_tunnel_reset(int conn_id) {
    submit(WK_TUNNEL_RESET, conn_id, NULL, 0, NULL, 0);
}

bool hw_worker_submit_cloud(const char *req_id, const char *command_json) {
    if (s_cloud_pending || s_cloud_ready) return false;   /* one at a time */
    s_cloud_pending = true;
    if (!submit(WK_CLOUD, -1, (const uint8_t *)command_json,
                strlen(command_json), req_id, 0)) {
        s_cloud_pending = false;
        return false;
    }
    return true;
}

bool hw_worker_submit_console(const char *line) {
    return submit(WK_CONSOLE, -1, (const uint8_t *)line, strlen(line), NULL,
                  pdMS_TO_TICKS(100));
}

bool hw_worker_submit_ota_begin(uint32_t size, const char *sha256_hex) {
    return submit_u(WK_OTA_BEGIN, -1, (const uint8_t *)sha256_hex,
                    strlen(sha256_hex), NULL, size, 0);
}
bool hw_worker_submit_ota_data(uint32_t offset, const uint8_t *buf, size_t len) {
    return submit_u(WK_OTA_DATA, -1, buf, len, NULL, offset, 0);
}
void hw_worker_submit_ota_end(void)    { submit(WK_OTA_END, -1, NULL, 0, NULL, 0); }
void hw_worker_submit_ota_abort(void)  { submit(WK_OTA_ABORT, -1, NULL, 0, NULL, 0); }
void hw_worker_submit_ota_commit(void) { submit(WK_OTA_COMMIT, -1, NULL, 0, NULL, 0); }

bool hw_worker_take_cloud_reply(char *req_id, size_t req_id_cap,
                                char *reply, size_t reply_cap, size_t *reply_len) {
    if (!s_cloud_ready) return false;
    strncpy(req_id, s_cloud_req_id, req_id_cap - 1);
    req_id[req_id_cap - 1] = '\0';
    size_t n = s_cloud_reply_len < reply_cap - 1 ? s_cloud_reply_len : reply_cap - 1;
    memcpy(reply, s_cloud_reply, n);
    reply[n] = '\0';
    *reply_len = n;
    s_cloud_ready   = false;   /* free the slot for the next cloud command */
    s_cloud_pending = false;
    return true;
}

/* ---- worker task ---------------------------------------------------------- */

static void handle_work(cmd_work_t *w) {
    switch (w->kind) {
    case WK_BYTES:
        command_handler_process(w->conn_id, w->data, w->len);
        break;
    case WK_CONSOLE:
        w->data[w->len] = '\0';
        console_run_line((char *)w->data);
        break;
    case WK_CLOUD: {
        w->data[w->len] = '\0';
        s_cloud_reply_len = command_handler_dispatch_cloud(
            (const char *)w->data, s_cloud_reply, sizeof(s_cloud_reply));
        strncpy(s_cloud_req_id, w->req_id, sizeof(s_cloud_req_id) - 1);
        s_cloud_req_id[sizeof(s_cloud_req_id) - 1] = '\0';
        s_cloud_ready = true;   /* net task frames it as command.response */
        break;
    }
    case WK_CLOSED:
        command_handler_conn_closed(w->conn_id);
        break;
    case WK_TUNNEL_RESET:
        command_handler_tunnel_reset(w->conn_id);
        break;
    case WK_OTA_BEGIN:
        w->data[w->len] = '\0';           /* sha256 hex string */
        ota_begin(w->u32, (const char *)w->data);
        break;
    case WK_OTA_DATA:
        ota_data(w->u32, w->data, w->len);
        break;
    case WK_OTA_END:
        ota_end();
        break;
    case WK_OTA_ABORT:
        ota_abort();
        break;
    case WK_OTA_COMMIT:
        ota_commit();                     /* does not return on success */
        break;
    default:
        break;
    }
}

static void worker_task(void *arg) {
    (void)arg;
    static cmd_work_t w;   /* static: the item is ~1.6 KB, keep it off the stack */
    for (;;) {
        watchdog_heartbeat(WD_TASK_WORKER, "worker");
        /* Drain a bounded batch so the periodic polls still run promptly even
           under a burst of inbound commands. */
        for (int i = 0; i < HW_WORK_QUEUE_DEPTH; i++) {
            if (xQueueReceive(s_q, &w, 0) != pdTRUE) break;
            handle_work(&w);
        }
        command_handler_poll();   /* async captures + paced bulk sends -> conn_tx rings */
        signal_engine_poll();     /* stop timed waveforms when their duration expires  */
        target_power_poll();      /* deferred eFuse enable + fault/EN change events     */
        ota_watchdog(HAL_GetTick()); /* drop a staging session that stopped receiving   */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void hw_worker_init(void) {
    s_q = xQueueCreate(HW_WORK_QUEUE_DEPTH, sizeof(cmd_work_t));
    if (!s_q) { printf("[hw] worker queue alloc failed\n"); return; }
    TaskHandle_t h = NULL;
    if (xTaskCreate(worker_task, "hw", 3072, NULL,
                    tskIDLE_PRIORITY + 1, &h) != pdPASS) {
        printf("[hw] worker task create failed\n");
        return;
    }
    sys_health_register("hw", h);   /* stack headroom telemetry */
}
