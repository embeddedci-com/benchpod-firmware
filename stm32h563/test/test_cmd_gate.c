/*
 * test_cmd_gate.c — host tests for the checks dispatch_line() runs before a command reaches its
 * handler (src/cmd_gate.c): the locked-LAN tier gate, the cloud-job lease gate, the tunnel max
 * tier, safe mode and the digital-only board, their order, and the exact refusal texts (clients
 * and the CLI match on their first word).
 */
#include "cmd_gate.h"

/* A test build with the minimum raised, as a release would set it in features.toml. */
#define BP_MIN_CLIENT_API_OVERRIDE 2u
#include "bp_features.h"
#include "bp_errors.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

/* An open pod on a full board, nothing in force: everything passes. */
static cmd_gate_ctx_t open_ctx(policy_src_t src) {
    cmd_gate_ctx_t c = {
        .src = src, .lan_policy = POD_LAN_OPEN, .lease_active = false, .lease_left_s = 0,
        .lease_holder = "", .tunnel_max_tier = -1, .skip_hw = false, .has_analog = true,
    };
    return c;
}

static char s_why[112];

static const char *gate(const char *json, const cmd_gate_ctx_t *c) {
    /* the same two steps dispatch_line takes: the verb, then its tier with the arguments */
    char cmd[32] = {0};
    const char *p = strstr(json, "\"cmd\":\"");
    if (p) {
        p += 7;
        size_t n = strcspn(p, "\"");
        if (n >= sizeof(cmd)) n = sizeof(cmd) - 1;
        memcpy(cmd, p, n);
    }
    memset(s_why, 0, sizeof(s_why));
    return cmd_gate_check(cmd_find(cmd), cmd, json, cmd_tier(cmd, json), c, s_why, sizeof(s_why));
}

/* r was noted with code (bp_err_noted, what send_error sends) and its first arg is arg0. */
static const bp_err_arg_t *g_args;
static bool coded(const char *r, bp_errcode_t want, const char *arg0) {
    bp_errcode_t code = BP_E_NONE;
    size_t n = 0;
    g_args = r ? bp_err_noted(r, &code, &n) : NULL;
    return g_args && code == want && n > 0 && g_args[0].is_str && strcmp(g_args[0].s, arg0) == 0;
}
static bool code_is(const char *r, bp_errcode_t want) {
    bp_errcode_t code = BP_E_NONE;
    return r && bp_err_noted(r, &code, NULL) && code == want;
}
static const char *arg_s(size_t i) { return g_args && g_args[i].is_str ? g_args[i].s : NULL; }

static int starts(const char *s, const char *prefix) {
    return s && strncmp(s, prefix, strlen(prefix)) == 0;
}

static void test_open_pod_passes(void) {
    static const policy_src_t srcs[] = { POLICY_SRC_USB, POLICY_SRC_CLOUD, POLICY_SRC_LAN };
    static const char *const lines[] = {
        "{\"cmd\":\"ping\"}", "{\"cmd\":\"generate\",\"wave\":\"sine\"}",
        "{\"cmd\":\"cloud_set\",\"host\":\"x\"}", "{\"cmd\":\"ota_begin\",\"size\":1}",
        "{\"cmd\":\"capture_dual\",\"adc_samples\":100}", "{\"cmd\":\"no_such_verb\"}",
    };
    for (size_t s = 0; s < 3; s++) {
        cmd_gate_ctx_t c = open_ctx(srcs[s]);
        for (size_t i = 0; i < sizeof(lines) / sizeof(lines[0]); i++)
            CHECK(gate(lines[i], &c) == NULL, "src %d refused %s: %s", (int)srcs[s], lines[i], s_why);
    }
}

static void test_locked_lan(void) {
    cmd_gate_ctx_t c = open_ctx(POLICY_SRC_LAN);
    c.lan_policy = POD_LAN_LOCKED;
    CHECK(gate("{\"cmd\":\"ping\"}", &c) == NULL, "T0 refused");
    CHECK(gate("{\"cmd\":\"generate\"}", &c) == NULL, "T1 refused");
    const char *r = gate("{\"cmd\":\"cloud_set\",\"host\":\"x\"}", &c);
    CHECK(coded(r, BP_E_LAN_LOCKED, "cloud_set"), "T2: %s", r);
    CHECK(coded(gate("{\"cmd\":\"ota_begin\"}", &c), BP_E_LAN_LOCKED, "ota_begin"), "T3: %s", s_why);
    /* an unknown verb is T3, so it is locked too (not "unknown cmd" first) */
    CHECK(code_is(gate("{\"cmd\":\"no_such_verb\"}", &c), BP_E_LAN_LOCKED), "unknown verb: %s", s_why);
    /* argument-dependent tiers: a dac_limits read is fine, a write is T2 */
    CHECK(gate("{\"cmd\":\"dac_limits\"}", &c) == NULL, "dac_limits read refused: %s", s_why);
    CHECK(code_is(gate("{\"cmd\":\"dac_limits\",\"path\":\"dac\",\"max_mv\":100}", &c), BP_E_LAN_LOCKED),
          "dac_limits write: %s", s_why);
    /* "off" refuses the same set (a LAN command can only arrive while the listener is down
       mid-switch, but the gate must not treat it as open) */
    c.lan_policy = POD_LAN_OFF;
    CHECK(code_is(gate("{\"cmd\":\"cloud_set\"}", &c), BP_E_LAN_LOCKED), "off: %s", s_why);
    /* the lock is for the LAN only */
    cmd_gate_ctx_t u = open_ctx(POLICY_SRC_USB);
    u.lan_policy = POD_LAN_LOCKED;
    CHECK(gate("{\"cmd\":\"ota_begin\"}", &u) == NULL, "USB locked out");
    cmd_gate_ctx_t cl = open_ctx(POLICY_SRC_CLOUD);
    cl.lan_policy = POD_LAN_LOCKED;
    CHECK(gate("{\"cmd\":\"ota_begin\"}", &cl) == NULL, "cloud locked out");
    /* the policy is for the external LAN: a connection on the USB network adapter stays open */
    cmd_gate_ctx_t un = open_ctx(POLICY_SRC_LAN);
    un.usb_link = true;
    for (int p = POD_LAN_LOCKED; p <= POD_LAN_OFF; p++) {
        un.lan_policy = (pod_lan_policy_t)p;
        CHECK(gate("{\"cmd\":\"cloud_set\"}", &un) == NULL, "USB network locked by policy %d: %s", p, s_why);
    }
}

static void test_lease(void) {
    cmd_gate_ctx_t c = open_ctx(POLICY_SRC_LAN);
    c.lease_active = true;
    c.lease_left_s = 42;
    c.lease_holder = "CI: org/repo";
    CHECK(gate("{\"cmd\":\"status\"}", &c) == NULL, "light read refused: %s", s_why);
    const char *r = gate("{\"cmd\":\"generate\"}", &c);
    CHECK(coded(r, BP_E_LEASE_BUSY, CMD_GATE_CLOUD_LEASE_HOLDER), "T1: %s", r);
    CHECK(arg_s(1) && strcmp(arg_s(1), "CI: org/repo") == 0, "lease purpose: %s", arg_s(1));
    /* dispatch_line sends this one refusal as lease_busy (BP0025) by its start */
    CHECK(coded(r, BP_E_LEASE_BUSY, CMD_GATE_CLOUD_LEASE_HOLDER), "cloud lease: %s", r);
    /* a T0 read that takes the capture hardware is not light */
    CHECK(code_is(gate("{\"cmd\":\"capture\",\"samples\":10}", &c), BP_E_LEASE_BUSY), "capture: %s", s_why);
    /* no holder label: "cloud" */
    c.lease_holder = "";
    CHECK(strstr(gate("{\"cmd\":\"generate\"}", &c), "(cloud, 42 s left)") != NULL, "label: %s", s_why);
    c.lease_holder = NULL;
    CHECK(strstr(gate("{\"cmd\":\"generate\"}", &c), "(cloud, 42 s left)") != NULL, "NULL label: %s", s_why);
    /* the lease verbs reach their handler, which answers busy with the cloud holder */
    CHECK(gate("{\"cmd\":\"lease_acquire\",\"id\":\"L-1\"}", &c) == NULL, "lease_acquire: %s", s_why);
    CHECK(gate("{\"cmd\":\"lease_release\",\"id\":\"L-1\"}", &c) == NULL, "lease_release: %s", s_why);
    /* the lease does not touch USB or the cloud */
    cmd_gate_ctx_t u = open_ctx(POLICY_SRC_USB);
    u.lease_active = true;
    CHECK(gate("{\"cmd\":\"generate\"}", &u) == NULL, "USB blocked by the lease");
    cmd_gate_ctx_t cl = open_ctx(POLICY_SRC_CLOUD);
    cl.lease_active = true;
    CHECK(gate("{\"cmd\":\"generate\"}", &cl) == NULL, "cloud blocked by the lease");
    /* order: on a locked LAN a T2 command says "locked", not "busy" */
    c.lan_policy = POD_LAN_LOCKED;
    CHECK(code_is(gate("{\"cmd\":\"cloud_set\"}", &c), BP_E_LAN_LOCKED), "order: %s", s_why);
}

static void test_tunnel_max_tier(void) {
    cmd_gate_ctx_t c = open_ctx(POLICY_SRC_CLOUD);
    c.tunnel_max_tier = 1;                        /* an org member: T0 + T1 */
    CHECK(gate("{\"cmd\":\"generate\"}", &c) == NULL, "T1 refused");
    const char *r = gate("{\"cmd\":\"cloud_proxy\",\"set\":\"x\"}", &c);
    CHECK(coded(r, BP_E_ROLE_REQUIRED, "cloud_proxy"), "T2: %s", r);
    CHECK(code_is(gate("{\"cmd\":\"ota_begin\"}", &c), BP_E_ROLE_REQUIRED), "T3: %s", s_why);
    c.tunnel_max_tier = 0;
    CHECK(code_is(gate("{\"cmd\":\"generate\"}", &c), BP_E_ROLE_REQUIRED), "T1 at max 0: %s", s_why);
    CHECK(gate("{\"cmd\":\"status\"}", &c) == NULL, "T0 at max 0: %s", s_why);
    c.tunnel_max_tier = 3;
    CHECK(gate("{\"cmd\":\"ota_begin\"}", &c) == NULL, "T3 at max 3: %s", s_why);
    /* not a tunnel (-1): no tier cap at all */
    c.tunnel_max_tier = -1;
    CHECK(gate("{\"cmd\":\"no_such_verb\"}", &c) == NULL, "non-tunnel capped");
}

static void test_safe_mode(void) {
    cmd_gate_ctx_t c = open_ctx(POLICY_SRC_USB);
    c.skip_hw = true;
    static const char *const ok[] = { "ping", "status", "cloud_set", "wifi_set", "eth",
                                      "target_power", "identity_wipe", "can_write", "dac_limits" };
    for (size_t i = 0; i < sizeof(ok) / sizeof(ok[0]); i++) {
        char line[64];
        snprintf(line, sizeof(line), "{\"cmd\":\"%s\"}", ok[i]);
        CHECK(gate(line, &c) == NULL, "%s refused in safe mode: %s", ok[i], s_why);
    }
    const char *r = gate("{\"cmd\":\"la_capture\"}", &c);
    CHECK(r && strcmp(r, "safe mode: iCE40/PSRAM are off. Unplug and replug the pod") == 0, "la: %s", r);
    CHECK(gate("{\"cmd\":\"ota_begin\"}", &c) != NULL, "OTA in safe mode");
    CHECK(gate("{\"cmd\":\"dap_start\"}", &c) != NULL, "SWD in safe mode");
    /* order: a tunnel tier refusal comes before safe mode */
    cmd_gate_ctx_t t = open_ctx(POLICY_SRC_CLOUD);
    t.skip_hw = true;
    t.tunnel_max_tier = 0;
    CHECK(starts(gate("{\"cmd\":\"la_capture\",\"samples\":1}", &t), "safe mode:"), "T0 la_capture: %s", s_why);
    CHECK(code_is(gate("{\"cmd\":\"generate\"}", &t), BP_E_ROLE_REQUIRED), "order: %s", s_why);
}

static void test_digital_board(void) {
    cmd_gate_ctx_t c = open_ctx(POLICY_SRC_LAN);
    c.has_analog = false;
    static const char *const analog[] = { "generate", "capture", "replay", "dac_out",
                                          "current_out", "adc_read", "calibrate", "dac_control_loop" };
    for (size_t i = 0; i < sizeof(analog) / sizeof(analog[0]); i++) {
        char line[64];
        snprintf(line, sizeof(line), "{\"cmd\":\"%s\"}", analog[i]);
        const char *r = gate(line, &c);
        CHECK(starts(r, "this BenchPod has no analog front end (digital board)"), "%s: %s", analog[i], r);
        CHECK(coded(r, BP_E_NO_ANALOG_FRONT_END, analog[i]), "%s: BP0046", analog[i]);
    }
    CHECK(gate("{\"cmd\":\"dac_stop\"}", &c) == NULL, "dac_stop refused (cleanup must work)");
    CHECK(gate("{\"cmd\":\"la_capture\"}", &c) == NULL, "LA refused on the digital board");
    /* capture_dual: LA-only is fine, any ADC samples is not */
    CHECK(gate("{\"cmd\":\"capture_dual\",\"la_samples\":1000}", &c) == NULL, "LA-only capture_dual");
    CHECK(gate("{\"cmd\":\"capture_dual\",\"adc_samples\":0,\"la_samples\":10}", &c) == NULL, "adc_samples 0");
    CHECK(gate("{\"cmd\":\"capture_dual\",\"adc_samples\":5}", &c) != NULL, "ADC capture_dual allowed");
    /* order: safe mode wins over the board check */
    c.skip_hw = true;
    CHECK(starts(gate("{\"cmd\":\"generate\"}", &c), "safe mode:"), "order: %s", s_why);
    /* dac_limits needs analog even though it runs in safe mode */
    c.skip_hw = false;
    CHECK(gate("{\"cmd\":\"dac_limits\"}", &c) != NULL, "dac_limits on a digital board");
}

static void test_lists_name_real_verbs(void) {
    /* A typo in either list would silently fail open (analog) or closed (safe mode). Every verb
       there must be one dispatch_line handles, which test_cmd_tier pins to cmd_tier's table. */
    static const char *const probe[] = {
        "ping", "status", "cloud_set", "cloud_status", "cloud_clear", "wifi_set", "wifi_status",
        "wifi_clear", "eth", "speedtest", "la_voltage", "usb_cc", "nrst", "target_power",
        "target_status", "power_status", "power_profile", "identity_public", "identity_pop",
        "identity_wipe", "dac_limits", "can_config", "can_write", "can_read", "can_status",
        "can_term", "can_respond", "can_disable", "generate", "capture", "stream", "measure", "load",
        "load_bin", "replay", "dac_set", "dac_mux", "cal_switch", "analog_path", "dac_out",
        "current_out", "adc_read", "calibrate", "dac_control_loop", "dac_loop_probe",
        "dac_loop_input",
    };
    int hw_ok = 0, analog = 0;
    for (size_t i = 0; i < sizeof(probe) / sizeof(probe[0]); i++) {
        CHECK(cmd_tier_known(probe[i]), "%s is not a known verb", probe[i]);
        hw_ok += cmd_gate_ok_without_hw(probe[i]);
        analog += cmd_gate_needs_analog(probe[i]);
    }
    CHECK(hw_ok == 28 && analog == 19, "list sizes changed: %d without hw, %d analog (update this test)",
          hw_ok, analog);
    CHECK(!cmd_gate_ok_without_hw("ota_begin") && !cmd_gate_needs_analog("la_capture"), "spot");
}

/* cmd_gate_device: the safe-mode and digital-board checks alone, as SCPI asks them with the JSON
   verb its command stands for. Same refusal texts as through cmd_gate_check, no tier or lease. */
static void test_device_gate(void) {
    CHECK(cmd_gate_device("capture", NULL, false, true) == NULL, "full board, no safe mode");
    const char *r = cmd_gate_device("capture", NULL, true, true);
    CHECK(r && strcmp(r, "safe mode: iCE40/PSRAM are off. Unplug and replug the pod") == 0, "safe: %s", r);
    CHECK(cmd_gate_device("target_power", NULL, true, true) == NULL, "power in safe mode");
    CHECK(starts(cmd_gate_device("measure", NULL, false, false), "this BenchPod has no analog front end"),
          "digital measure");
    CHECK(cmd_gate_device("la", NULL, false, false) == NULL, "digital la");
    CHECK(cmd_gate_device("capture_dual", "{\"adc_samples\":0}", false, false) == NULL, "LA-only dual");
    CHECK(cmd_gate_device("capture_dual", "{\"adc_samples\":16}", false, false) != NULL, "ADC dual");
    CHECK(cmd_gate_device("capture_dual", NULL, false, false) == NULL, "dual without json");
    CHECK(starts(cmd_gate_device("generate", NULL, true, false), "safe mode:"), "order");
    /* the same answer cmd_gate_check gives once the policy gates pass */
    cmd_gate_ctx_t c = open_ctx(POLICY_SRC_LAN);
    c.skip_hw = true;
    CHECK(gate("{\"cmd\":\"capture\"}", &c) == cmd_gate_device("capture", "{\"cmd\":\"capture\"}", true, true),
          "check vs device");
}

static void test_scpi_line_writes(void) {
    CHECK(!cmd_gate_scpi_line_writes("*IDN?"), "query");
    CHECK(!cmd_gate_scpi_line_writes("MEAS:VOLT?;:SYST:ERR?"), "two queries");
    CHECK(cmd_gate_scpi_line_writes("SOUR:VOLT 1.0"), "write");
    CHECK(cmd_gate_scpi_line_writes("MEAS:VOLT?;SOUR:VOLT 1"), "query then write");
    CHECK(cmd_gate_scpi_line_writes("SOUR:VOLT 1;MEAS:VOLT?"), "write then query");
    CHECK(cmd_gate_scpi_line_writes(""), "empty line");
    CHECK(cmd_gate_scpi_line_writes("*IDN?;"), "trailing empty part");
}

/* The client API check (features.toml min_client_api, BP0030 client_too_old). */
static int client_ok(const char *json, uint32_t min_api, int32_t *sent) {
    char cmd[32] = {0};
    const char *p = strstr(json, "\"cmd\":\"");
    if (p) {
        p += 7;
        size_t n = strcspn(p, "\"");
        if (n >= sizeof(cmd)) n = sizeof(cmd) - 1;
        memcpy(cmd, p, n);
    }
    return cmd_gate_client_api(cmd_find(cmd), json, min_api, sent);
}

static void test_client_api(void) {
    int32_t sent = -1;
    CHECK(BP_MIN_CLIENT_API == 2u, "the override did not take: %u", (unsigned)BP_MIN_CLIENT_API);

    /* min_client_api 0 (today): nothing is refused, a client that sends nothing included. */
    CHECK(client_ok("{\"cmd\":\"generate\",\"wave\":\"sine\"}", 0, &sent) && sent == 0, "min 0 refused");
    CHECK(client_ok("{\"cmd\":\"no_such_verb\"}", 0, &sent), "min 0 refused an unknown verb");

    /* Raised: below it is refused, at or above it runs. */
    const uint32_t min = BP_MIN_CLIENT_API;
    CHECK(!client_ok("{\"cmd\":\"generate\",\"wave\":\"sine\"}", min, &sent) && sent == 0, "no client_api ran");
    CHECK(!client_ok("{\"cmd\":\"generate\",\"client_api\":1}", min, &sent) && sent == 1, "client_api 1 ran (sent %d)", (int)sent);
    CHECK(client_ok("{\"cmd\":\"generate\",\"client_api\":2}", min, &sent) && sent == 2, "client_api 2 refused");
    CHECK(client_ok("{\"client_api\":7,\"cmd\":\"uart_open\"}", min, &sent) && sent == 7, "client_api 7 refused");
    CHECK(!client_ok("{\"cmd\":\"no_such_verb\",\"client_api\":1}", min, &sent), "an unknown verb ran");
    /* Not a number, negative, or only nested (a carried profile or definition): 0. */
    CHECK(!client_ok("{\"cmd\":\"generate\",\"client_api\":\"abc\"}", min, &sent) && sent == 0, "text counted");
    CHECK(!client_ok("{\"cmd\":\"generate\",\"client_api\":-3}", min, &sent) && sent == 0, "negative counted");
    CHECK(!client_ok("{\"cmd\":\"generate\",\"client_api\":2x}", min, &sent) && sent == 0, "2x counted");
    CHECK(!client_ok("{\"cmd\":\"wiring_put\",\"profile\":{\"client_api\":9}}", min, &sent) && sent == 0,
          "a nested client_api counted (sent %d)", (int)sent);
    CHECK(client_ok("{\"cmd\":\"generate\",\"client_api\":99999999999}", min, &sent) && sent == INT32_MAX,
          "a huge client_api: sent %d", (int)sent);

    /* What a client needs to find out and say so, and to let go of the pod: always. */
    static const char *const any[] = { "status", "diag", "ping", "lease_release" };
    for (size_t i = 0; i < sizeof(any) / sizeof(any[0]); i++) {
        char line[64];
        snprintf(line, sizeof(line), "{\"cmd\":\"%s\"}", any[i]);
        CHECK(client_ok(line, min, &sent), "%s refused for an old client", any[i]);
        CHECK(client_ok(line, UINT32_MAX, &sent), "%s refused at the highest minimum", any[i]);
    }
    /* And nothing else: the list stays short. */
    size_t n = 0, flagged = 0;
    const cmd_desc_t *rows = cmd_table_rows(&n);
    for (size_t i = 0; i < n; i++) {
        if (!cmd_has(&rows[i], CMD_F_ANY_CLIENT)) continue;
        flagged++;
        int known = 0;
        for (size_t j = 0; j < sizeof(any) / sizeof(any[0]); j++)
            known |= strcmp(rows[i].key.name, any[j]) == 0;
        CHECK(known, "%s runs for any client: add it to the docs and this list", rows[i].key.name);
    }
    CHECK(flagged == sizeof(any) / sizeof(any[0]), "%zu rows run for any client", flagged);
    CHECK(!client_ok("{\"cmd\":\"lease_acquire\",\"ttl_s\":60}", min, &sent), "lease_acquire ran");
}

int main(void) {
    test_open_pod_passes();
    test_locked_lan();
    test_lease();
    test_tunnel_max_tier();
    test_device_gate();
    test_safe_mode();
    test_digital_board();
    test_lists_name_real_verbs();
    test_scpi_line_writes();
    test_client_api();
    if (failures) { printf("test_cmd_gate: %d FAILED\n", failures); return 1; }
    printf("test_cmd_gate: all passed\n");
    return 0;
}
