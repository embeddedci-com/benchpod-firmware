/* usb_check.c — see usb_check.h. USB_HOST=1 builds only: binds the job (usb_check_core.c) to the
   pod. The job object lives on the heap (static RAM is full) and stays until the next check
   starts, so its result can be polled. */
#include "usb_check.h"
#include "usb_bus.h"
#include "usb_host.h"
#include "usb_msc.h"
#include "usb_flash.h"
#include "target_power.h"
#include "power_profile.h"
#include "ina238.h"
#include "i2c_bus.h"
#include "bp_log.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

#define EFUSE            1
#define MONITOR_ADDR     I2C_ADDR_INA238_INTERNAL
#define STATUS_TIMEOUT_MS 300u

static uc_job_t *s_job;
static uint8_t   s_status[2];   /* GET_STATUS answer (usb_ctrl_io keeps the buffer while in flight) */

static uint32_t op_now(void *c) { (void)c; return xTaskGetTickCount() * portTICK_PERIOD_MS; }

static void op_host(void *c, uc_host_t *h)
{
    (void)c;
    usb_host_info_t hi;
    usb_host_info(&hi);
    h->hosting = hi.mode == USB_HOST_MODE_HOST;
    h->connected = hi.connected;
    h->attached = hi.attached;
    h->enumeration = (uint8_t)hi.enumeration;
    h->enumerations = hi.enumerations;
    h->descriptors = hi.descriptors;
    h->vid = hi.vid;
    h->pid = hi.pid;
    h->cfg_valid = hi.config.valid;
    h->max_power_ma = hi.config.max_power_ma;
    h->cfg_attrs = hi.config.attributes;
}

static bool op_efuse_on(void *c)
{
    (void)c;
    bool en = false;
    (void)target_power_get_cached(EFUSE, &en, NULL);
    return en;
}

static int op_efuse_set(void *c, bool on) { (void)c; return target_power_enable(EFUSE, on); }

static int op_current(void *c, int32_t *ua) { (void)c; return ina238_read_shunt_ua(MONITOR_ADDR, ua); }

static int op_bus_request(void *c, int op) { (void)c; return usb_bus_request((usb_bus_op_t)op); }

static int op_bus_state(void *c, uc_bus_info_t *info)
{
    (void)c;
    usb_bus_info_t bi;
    usb_bus_state_t st = usb_bus_state(&bi);
    info->susprdy = bi.susprdy;
    info->sof_stopped = bi.sof_stopped;
    info->sof_resumed = bi.sof_resumed;
    return (int)st;
}

static int op_get_status(void *c)
{
    (void)c;
    static const uint8_t setup[8] = { 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00 };
    uint32_t actual = 0;
    return usb_ctrl_io(setup, s_status, &actual, STATUS_TIMEOUT_MS) == 0 && actual == 2 ? 0 : -1;
}

static const uc_ops_t k_ops = { NULL, op_now, op_host, op_efuse_on, op_efuse_set, op_current,
                                op_bus_request, op_bus_state, op_get_status };

bool usb_check_busy(void) { return s_job && s_job->running; }

int usb_check_start(const uc_args_t *a, const char **why)
{
    *why = NULL;
    if (usb_check_busy()) { *why = "a USB check is already running"; return -1; }
    if (usb_flash_busy()) { *why = "a USB flash is running (usb_flash)"; return -1; }
    usb_host_info_t hi;
    usb_host_info(&hi);
    if (hi.mode != USB_HOST_MODE_HOST) { *why = "USB host mode is not on (usb_host)"; return -1; }
    const bool needs_efuse = a->test != UC_RESET;
    const bool needs_device = a->test != UC_ENUMERATE;
    const bool needs_monitor = a->test == UC_POWER || a->test == UC_SUSPEND;
    if (needs_device && !hi.attached) { *why = "no USB device is attached"; return -1; }
    if (needs_monitor) {
        if (power_profile_running()) { *why = "a power profile is running (power_profile)"; return -1; }
        if (ina238_chip(MONITOR_ADDR) == INA_CHIP_NONE) { *why = "eFuse 1 has no current monitor on this pod"; return -1; }
    }
    if (needs_efuse && !op_efuse_on(NULL)) return -2;
    if (usb_bus_suspended()) { *why = "the bus is still suspended from an earlier check"; return -1; }
    if (!s_job) {
        s_job = pvPortMalloc(sizeof(*s_job));
        if (!s_job) { *why = "out of memory"; return -1; }
    }
    if (needs_monitor) (void)ina238_set_adcrange_fine(MONITOR_ADDR);
    uc_job_init(s_job, a, &k_ops);
    return 0;
}

void usb_check_poll(void)
{
    if (!usb_check_busy()) return;
    /* Keep host mode on while the check runs (its timeout would hand the port back). */
    usb_host_info_t hi;
    usb_host_info(&hi);
    if (hi.mode == USB_HOST_MODE_HOST && hi.remaining_s < 15u) (void)usb_host_start(hi.port, 30u);
    if (uc_job_step(s_job) == 0)
        LOG_I("[usbcheck] %s: %s%s%s (%lu ms)\n", uc_test_name(s_job->a.test), s_job->r.pass ? "pass" : "fail",
              s_job->r.error[0] ? " " : "", s_job->r.error, (unsigned long)s_job->r.ms);
}

void usb_check_abort(void)
{
    if (usb_check_busy()) uc_job_abort(s_job);
}

size_t usb_check_reply(char *out, size_t cap)
{
    if (!s_job) {
        static const char idle[] = "{\"running\":false,\"have_result\":false}";
        if (cap < sizeof(idle)) return 0;
        memcpy(out, idle, sizeof(idle));
        return sizeof(idle) - 1;
    }
    return uc_reply_json(s_job, out, cap);
}
