/* Host test for usb_check_core.c: the argument parser, the statistics, the verdicts and the job
   state machine against a simulated board with a fake clock. */
#include "usb_check_core.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

/* ---- simulated board ----------------------------------------------------------------------- */
typedef struct {
    uint32_t now;
    bool     hosting, efuse, powered_by_efuse;
    bool     never_connects, stuck, stays_short;
    uint32_t survive_below_ms;
    uint32_t connect_ms, desc_ms, mount_ms;
    uint32_t power_since, off_at;
    bool     enumerating, got_conn, got_desc, got_mount;
    bool     connected, attached;
    uint16_t enumerations, descriptors, pid;
    int32_t  run_ua, suspend_ua;
    int      read_fail;
    bool     suspended, sof_ok, resume_ok, dead;
    int      bus_op, bus_busy;
    uint32_t bus_done_at;
    uint32_t reenum_ms, reenum_at;
    bool     reenum_pending, efuse_fail;
    int      efuse_sets, status_calls;
    uint16_t max_power_ma;
    uint8_t  attrs;
    bool     cfg_valid;
} sim_t;

static void sim_update(sim_t *s)
{
    if (s->reenum_pending && s->now >= s->reenum_at) {
        s->reenum_pending = false;
        s->connected = s->attached = true;
        s->enumerations++;
        s->descriptors++;
        s->dead = false;
    }
    if (s->enumerating && s->efuse && s->powered_by_efuse) {
        uint32_t el = s->now - s->power_since;
        if (!s->never_connects) {
            if (!s->got_conn && el >= s->connect_ms) { s->got_conn = true; s->connected = true; }
            if (s->got_conn && !s->got_desc && el >= s->desc_ms) { s->got_desc = true; s->descriptors++; }
            if (s->got_desc && !s->got_mount && !s->stuck && el >= s->mount_ms) {
                s->got_mount = true; s->attached = true; s->enumerations++; s->enumerating = false;
            }
        }
    }
}

static uint32_t o_now(void *c) { return ((sim_t *)c)->now; }
static void o_host(void *c, uc_host_t *h)
{
    sim_t *s = c;
    sim_update(s);
    memset(h, 0, sizeof(*h));
    h->hosting = s->hosting;
    h->connected = s->connected;
    h->attached = s->attached;
    h->enumeration = (s->stuck && s->got_desc && !s->attached && s->now - s->power_since > s->desc_ms + 2000u)
                     ? UC_ENUM_STUCK : 0;
    h->enumerations = s->enumerations;
    h->descriptors = s->descriptors;
    h->vid = 0x1234;
    h->pid = s->pid;
    h->cfg_valid = s->cfg_valid;
    h->max_power_ma = s->max_power_ma;
    h->cfg_attrs = s->attrs;
}
static bool o_efuse_on(void *c) { return ((sim_t *)c)->efuse; }
static int o_efuse_set(void *c, bool on)
{
    sim_t *s = c;
    if (s->efuse_fail) return -1;
    s->efuse_sets++;
    if (!on && s->efuse) {
        s->off_at = s->now;
        s->efuse = false;
        if (s->powered_by_efuse && !(s->stays_short)) { s->connected = s->attached = false; }
    } else if (on && !s->efuse) {
        s->efuse = true;
        if (s->powered_by_efuse) {
            bool survived = s->stays_short && s->now - s->off_at < s->survive_below_ms;
            if (!survived) {
                s->connected = s->attached = false;
                s->enumerating = true;
                s->got_conn = s->got_desc = s->got_mount = false;
                s->power_since = s->now;
            }
        }
    }
    return 0;
}
static int o_current(void *c, int32_t *ua)
{
    sim_t *s = c;
    if (s->read_fail) return -1;
    *ua = s->suspended ? s->suspend_ua : s->run_ua;
    return 0;
}
static int o_bus_req(void *c, int op)
{
    sim_t *s = c;
    if (s->bus_busy) return -1;
    if (op == UC_BUS_RESET) {
        if (s->suspended) return -1;
        s->attached = false;
        s->reenum_pending = true;
        s->reenum_at = s->now + s->reenum_ms;
        return 0;   /* the usb task issues it at once: no state to poll */
    }
    s->bus_op = op;
    s->bus_busy = 1;
    s->bus_done_at = s->now + (op == UC_BUS_SUSPEND ? 15u : op == UC_BUS_RESUME ? 40u : 1u);
    return 0;
}
static int o_bus_state(void *c, uc_bus_info_t *i)
{
    sim_t *s = c;
    memset(i, 0, sizeof(*i));
    if (!s->bus_busy) return UC_BUS_IDLE;
    if (s->now < s->bus_done_at) return UC_BUS_BUSY;
    s->bus_busy = 0;
    if (s->bus_op == UC_BUS_SUSPEND) { s->suspended = true; i->susprdy = true; i->sof_stopped = s->sof_ok; }
    if (s->bus_op == UC_BUS_RESUME) {
        s->suspended = false;
        i->sof_resumed = s->resume_ok;
        if (!s->resume_ok) { s->dead = true; return UC_BUS_FAIL; }
    }
    return UC_BUS_OK;
}
static int o_status(void *c)
{
    sim_t *s = c;
    s->status_calls++;
    return (s->attached && !s->dead && !s->suspended) ? 0 : -1;
}

static const uc_ops_t k_ops = { NULL, o_now, o_host, o_efuse_on, o_efuse_set, o_current, o_bus_req, o_bus_state, o_status };

static void sim_init(sim_t *s)
{
    memset(s, 0, sizeof(*s));
    s->hosting = s->efuse = s->powered_by_efuse = true;
    s->connected = s->attached = true;
    s->connect_ms = 40; s->desc_ms = 180; s->mount_ms = 260;
    s->enumerations = 1; s->descriptors = 1; s->pid = 0x5740;
    s->run_ua = 45000; s->suspend_ua = 800;
    s->sof_ok = s->resume_ok = true;
    s->reenum_ms = 300;
    s->max_power_ma = 100; s->cfg_valid = true; s->attrs = 0x80;
    s->survive_below_ms = 30;
}

/* Run a job with the clock advancing 1 ms per step; returns false when it did not finish. */
static bool run(sim_t *s, uc_job_t *j, const uc_args_t *a)
{
    uc_ops_t ops = k_ops;
    ops.ctx = s;
    uc_job_init(j, a, &ops);
    for (int n = 0; n < 2000000; n++) {
        if (!uc_job_step(j)) return true;
        s->now++;
    }
    return false;
}

static int parse(const char *json, uc_args_t *a, char *why)
{
    why[0] = 0;
    return uc_parse_args(json, a, why, 96);
}

static void test_parse(void)
{
    uc_args_t a;
    char why[96];
    CHECK(parse("{\"cmd\":\"usb_check\",\"test\":\"enumerate\"}", &a, why) == 0);
    CHECK(a.test == UC_ENUMERATE && a.count == 10 && a.off_ms == 500 && a.timeout_ms == 5000);
    CHECK(parse("{\"test\":\"enumerate\",\"count\":3,\"off_ms\":100,\"timeout_ms\":2000}", &a, why) == 0);
    CHECK(a.count == 3 && a.off_ms == 100 && a.timeout_ms == 2000);
    CHECK(parse("{\"test\":\"enumerate\",\"off_ms\":10}", &a, why) != 0 && strstr(why, "off_ms"));
    CHECK(parse("{\"test\":\"enumerate\",\"count\":0}", &a, why) != 0);
    CHECK(parse("{\"test\":\"enumerate\",\"count\":201}", &a, why) != 0);
    CHECK(parse("{\"test\":\"enumerate\",\"count\":-1}", &a, why) != 0);
    CHECK(parse("{\"test\":\"enumerate\",\"count\":\"x\"}", &a, why) != 0);
    CHECK(parse("{\"test\":\"bogus\"}", &a, why) != 0 && strstr(why, "test must be"));
    CHECK(parse("{\"cmd\":\"usb_check\"}", &a, why) != 0);
    CHECK(parse("{\"test\":\"power\"}", &a, why) == 0 && a.duration_ms == 3000);
    CHECK(parse("{\"test\":\"power\",\"duration_ms\":100}", &a, why) != 0);
    CHECK(parse("{\"test\":\"suspend\"}", &a, why) == 0 && a.duration_ms == 1000 && a.settle_ms == 200);
    CHECK(parse("{\"test\":\"suspend\",\"settle_ms\":5,\"duration_ms\":500}", &a, why) != 0);
    CHECK(parse("{\"test\":\"reset\"}", &a, why) == 0 && a.count == 20);
    CHECK(parse("{\"test\":\"replug\"}", &a, why) == 0 && a.count == 10 && a.n_drops == 4 &&
          a.drops[0] == 5 && a.drops[3] == 500);
    CHECK(parse("{\"test\":\"replug\",\"count\":2,\"off_ms\":[3, 40]}", &a, why) == 0 && a.n_drops == 2 &&
          a.drops[0] == 3 && a.drops[1] == 40 && a.count == 2);
    CHECK(parse("{\"test\":\"replug\",\"off_ms\":7}", &a, why) == 0 && a.n_drops == 1 && a.drops[0] == 7);
    CHECK(parse("{\"test\":\"replug\",\"off_ms\":[1,2,3,4,5,6,7]}", &a, why) != 0);
    CHECK(parse("{\"test\":\"replug\",\"off_ms\":[0]}", &a, why) != 0);
    CHECK(parse("{\"test\":\"replug\",\"off_ms\":[]}", &a, why) != 0);
    CHECK(parse("{\"test\":\"replug\",\"count\":101}", &a, why) != 0);
    CHECK(uc_test_from_name("suspend") == UC_SUSPEND && uc_test_from_name("x") == -1 && uc_test_from_name(NULL) == -1);
}

static void test_stats_verdicts(void)
{
    uc_stat_t s = {0};
    CHECK(uc_stat_avg(&s) == 0);
    uc_stat_add(&s, 10); uc_stat_add(&s, 30); uc_stat_add(&s, 21);
    CHECK(s.min == 10 && s.max == 30 && s.n == 3 && uc_stat_avg(&s) == 20);
    CHECK(uc_power_verdict(50000, 60000, 100) == UC_V_OK);
    CHECK(uc_power_verdict(100001, 100001, 100) == UC_V_AVG_OVER);
    CHECK(uc_power_verdict(90000, 125000, 100) == UC_V_OK);
    CHECK(uc_power_verdict(90000, 125001, 100) == UC_V_PEAK_OVER);
    CHECK(uc_power_verdict(10, 10, 0) == UC_V_NO_LIMIT);
    CHECK(uc_suspend_verdict(2500, false) == UC_V_OK);
    CHECK(uc_suspend_verdict(2501, false) == UC_V_AVG_OVER);
    CHECK(uc_suspend_verdict(90000, true) == UC_V_SKIPPED);
    CHECK(uc_suspend_verdict(-50, false) == UC_V_OK);
}

static uc_args_t args(const char *json)
{
    uc_args_t a; char why[96];
    CHECK(parse(json, &a, why) == 0);
    return a;
}

static uc_job_t g_job;

static void test_enumerate(void)
{
    sim_t s; sim_init(&s);
    uc_args_t a = args("{\"test\":\"enumerate\",\"count\":5,\"off_ms\":100}");
    CHECK(run(&s, &g_job, &a));
    CHECK(g_job.r.pass && g_job.r.ok == 5 && g_job.r.error[0] == 0);
    CHECK(g_job.r.connect.min >= 40 && g_job.r.connect.max <= 45);
    CHECK(g_job.r.descriptor.min >= 180 && g_job.r.descriptor.max <= 185);
    CHECK(g_job.r.mount.min >= 260 && g_job.r.mount.max <= 265);
    CHECK(s.efuse && s.efuse_sets == 10);
    char out[1800];
    size_t n = uc_reply_json(&g_job, out, sizeof(out));
    CHECK(n > 0 && strstr(out, "\"pass\":true") && strstr(out, "\"mount_ms\":{\"min\":260"));
    CHECK(strstr(out, "\"running\":false") && strstr(out, "\"have_result\":true"));

    /* never connects */
    sim_init(&s); s.never_connects = true;
    a = args("{\"test\":\"enumerate\",\"count\":2,\"off_ms\":100,\"timeout_ms\":600}");
    CHECK(run(&s, &g_job, &a));
    CHECK(!g_job.r.pass && g_job.r.no_connect == 2 && g_job.r.ok == 0 && s.efuse);
    CHECK(g_job.r.mount.n == 0);
    n = uc_reply_json(&g_job, out, sizeof(out));
    CHECK(n > 0 && strstr(out, "\"mount_ms\":null") && strstr(out, "\"no_connect\":2"));

    /* stuck: a descriptor, no mount */
    sim_init(&s); s.stuck = true;
    a = args("{\"test\":\"enumerate\",\"count\":1,\"off_ms\":100,\"timeout_ms\":9000}");
    CHECK(run(&s, &g_job, &a));
    CHECK(!g_job.r.pass && g_job.r.stuck == 1);

    /* slower than the timeout: connect seen, no mount */
    sim_init(&s); s.mount_ms = 3000;
    a = args("{\"test\":\"enumerate\",\"count\":1,\"off_ms\":100,\"timeout_ms\":1000}");
    CHECK(run(&s, &g_job, &a));
    CHECK(!g_job.r.pass && g_job.r.timeout == 1);

    /* the board has its own power: it stays connected with the rail off */
    sim_init(&s); s.powered_by_efuse = false;
    a = args("{\"test\":\"enumerate\",\"count\":3,\"off_ms\":100}");
    CHECK(run(&s, &g_job, &a));
    CHECK(!g_job.r.pass && !strcmp(g_job.r.error, "not_powered_by_efuse1") && s.efuse);

    /* an eFuse write that fails */
    sim_init(&s); s.efuse_fail = true;
    CHECK(run(&s, &g_job, &a));
    CHECK(!strcmp(g_job.r.error, "efuse_error"));

    /* host mode ends mid-run: the rail comes back */
    sim_init(&s);
    a = args("{\"test\":\"enumerate\",\"count\":5,\"off_ms\":500}");
    uc_ops_t ops = k_ops; ops.ctx = &s;
    uc_job_init(&g_job, &a, &ops);
    for (int i = 0; i < 100; i++) { uc_job_step(&g_job); s.now++; }
    CHECK(!s.efuse);   /* in the off phase */
    s.hosting = false;
    CHECK(uc_job_step(&g_job) == 0);
    CHECK(!strcmp(g_job.r.error, "host_stopped") && s.efuse);
}

static void test_abort(void)
{
    sim_t s; sim_init(&s);
    uc_args_t a = args("{\"test\":\"replug\",\"count\":5}");
    uc_ops_t ops = k_ops; ops.ctx = &s;
    uc_job_init(&g_job, &a, &ops);
    for (int i = 0; i < 3; i++) { uc_job_step(&g_job); s.now++; }
    uc_job_abort(&g_job);
    CHECK(uc_job_step(&g_job) == 0 && !g_job.running);
    CHECK(!strcmp(g_job.r.error, "aborted") && !g_job.r.pass && s.efuse);
    CHECK(uc_job_step(&g_job) == 0);
}

static void test_power(void)
{
    sim_t s; sim_init(&s);
    uc_args_t a = args("{\"test\":\"power\",\"duration_ms\":500}");
    CHECK(run(&s, &g_job, &a));
    CHECK(g_job.r.pass && g_job.r.verdict == UC_V_OK && g_job.r.limit_ma == 100);
    CHECK(g_job.r.samples > 100 && g_job.r.sum_ua / (int64_t)g_job.r.samples == 45000);
    char out[1800];
    CHECK(uc_reply_json(&g_job, out, sizeof(out)) > 0 && strstr(out, "\"avg_ua\":45000") && strstr(out, "\"limit_ma\":100"));

    sim_init(&s); s.run_ua = 140000;
    CHECK(run(&s, &g_job, &a));
    CHECK(!g_job.r.pass && g_job.r.verdict == UC_V_AVG_OVER);

    sim_init(&s); s.max_power_ma = 0;
    CHECK(run(&s, &g_job, &a));
    CHECK(!g_job.r.pass && g_job.r.verdict == UC_V_NO_LIMIT);

    sim_init(&s); s.attached = false;
    CHECK(run(&s, &g_job, &a));
    CHECK(!strcmp(g_job.r.error, "no_device"));

    sim_init(&s); s.read_fail = 1;
    CHECK(run(&s, &g_job, &a));
    CHECK(!strcmp(g_job.r.error, "monitor_error"));
}

static void test_suspend(void)
{
    sim_t s; sim_init(&s);
    uc_args_t a = args("{\"test\":\"suspend\",\"settle_ms\":50,\"duration_ms\":300}");
    CHECK(run(&s, &g_job, &a));
    CHECK(g_job.r.pass && g_job.r.suspend_ok && g_job.r.resume_ok && g_job.r.answers && !g_job.r.recovered);
    CHECK(g_job.r.verdict == UC_V_OK && g_job.r.sum_ua / (int64_t)g_job.r.samples == 800);
    CHECK(!s.suspended);
    char out[1800];
    CHECK(uc_reply_json(&g_job, out, sizeof(out)) > 0 && strstr(out, "\"resume\":\"signaling\"") &&
          strstr(out, "\"current\":\"ok\""));

    /* too much suspend current */
    sim_init(&s); s.suspend_ua = 9000;
    CHECK(run(&s, &g_job, &a));
    CHECK(!g_job.r.pass && g_job.r.verdict == UC_V_AVG_OVER && g_job.r.answers);

    /* self-powered: no limit */
    sim_init(&s); s.suspend_ua = 9000; s.attrs = 0xC0;
    CHECK(run(&s, &g_job, &a));
    CHECK(g_job.r.pass && g_job.r.self_powered && g_job.r.verdict == UC_V_SKIPPED);

    /* the SOFs never stop */
    sim_init(&s); s.sof_ok = false;
    CHECK(run(&s, &g_job, &a));
    CHECK(!g_job.r.pass && !g_job.r.suspend_ok && g_job.r.verdict == UC_V_NOT_MEASURED && !s.suspended);

    /* the resume fails and the device is dead: a bus reset brings it back */
    sim_init(&s); s.resume_ok = false;
    CHECK(run(&s, &g_job, &a));
    CHECK(!g_job.r.pass && !g_job.r.resume_ok && !g_job.r.answers && g_job.r.recovered);
    CHECK(uc_reply_json(&g_job, out, sizeof(out)) > 0 && strstr(out, "\"recovered_by_reset\":true") &&
          strstr(out, "\"resume\":\"failed\""));

    /* abort while suspended: the bus is resumed */
    sim_init(&s);
    uc_ops_t ops = k_ops; ops.ctx = &s;
    uc_job_init(&g_job, &a, &ops);
    for (int i = 0; i < 100; i++) { uc_job_step(&g_job); s.now++; }
    CHECK(s.suspended);
    uc_job_abort(&g_job);
    CHECK(uc_job_step(&g_job) == 0 && s.bus_busy && s.bus_op == UC_BUS_RESUME);
}

static void test_reset(void)
{
    sim_t s; sim_init(&s);
    uc_args_t a = args("{\"test\":\"reset\",\"count\":6}");
    CHECK(run(&s, &g_job, &a));
    CHECK(g_job.r.pass && g_job.r.ok == 6 && g_job.r.mount.min >= 300 && g_job.r.mount.max <= 305);
    char out[1800];
    CHECK(uc_reply_json(&g_job, out, sizeof(out)) > 0 && strstr(out, "\"reenumerate_ms\":{\"min\":300"));

    sim_init(&s); s.reenum_ms = 6000;
    a = args("{\"test\":\"reset\",\"count\":2,\"timeout_ms\":1000}");
    CHECK(run(&s, &g_job, &a));
    CHECK(!g_job.r.pass && g_job.r.timeout == 2);

    sim_init(&s); s.attached = false;
    CHECK(run(&s, &g_job, &a));
    CHECK(!strcmp(g_job.r.error, "no_device"));
}

static void test_replug(void)
{
    sim_t s; sim_init(&s);
    /* nothing survives a drop: every length re-enumerates */
    uc_args_t a = args("{\"test\":\"replug\",\"count\":3,\"off_ms\":[5,100]}");
    CHECK(run(&s, &g_job, &a));
    CHECK(g_job.r.pass && g_job.r.nlen == 2);
    CHECK(g_job.r.len[0].off_ms == 5 && g_job.r.len[0].reenumerated == 3 && g_job.r.len[0].failed == 0);
    CHECK(g_job.r.len[1].off_ms == 100 && g_job.r.len[1].reenumerated == 3);
    CHECK(g_job.done == 6 && g_job.total == 6 && s.efuse);

    /* a board with bulk capacitance rides through a short drop */
    sim_init(&s); s.stays_short = true; s.survive_below_ms = 30;
    a = args("{\"test\":\"replug\",\"count\":2,\"off_ms\":[5,100]}");
    CHECK(run(&s, &g_job, &a));
    CHECK(g_job.r.len[0].stayed == 2 && g_job.r.len[0].reenumerated == 0);
    CHECK(g_job.r.len[1].reenumerated == 2 && g_job.r.pass);

    /* it stays mounted for the host but is dead */
    sim_init(&s); s.stays_short = true; s.dead = true;
    a = args("{\"test\":\"replug\",\"count\":2,\"off_ms\":[5]}");
    CHECK(run(&s, &g_job, &a));
    CHECK(!g_job.r.pass && g_job.r.len[0].failed == 2);

    /* it never comes back */
    sim_init(&s); s.never_connects = true;
    a = args("{\"test\":\"replug\",\"count\":1,\"off_ms\":[5,20],\"timeout_ms\":800}");
    CHECK(run(&s, &g_job, &a));
    CHECK(!g_job.r.pass && g_job.r.len[0].failed == 1 && g_job.r.len[1].failed == 1);

    /* the reply fits the cloud reply budget with six lengths */
    sim_init(&s);
    a = args("{\"test\":\"replug\",\"count\":1,\"off_ms\":[1,2,3,4,5,6]}");
    CHECK(run(&s, &g_job, &a));
    char out[1728];
    size_t n = uc_reply_json(&g_job, out, sizeof(out));
    CHECK(n > 0 && n < 900);
    CHECK(uc_reply_json(&g_job, out, 40) == 0);
}

int main(void)
{
    test_parse();
    test_stats_verdicts();
    test_enumerate();
    test_abort();
    test_power();
    test_suspend();
    test_reset();
    test_replug();
    if (failures) { printf("%d failure(s)\n", failures); return 1; }
    printf("test_usb_check: ok\n");
    return 0;
}
