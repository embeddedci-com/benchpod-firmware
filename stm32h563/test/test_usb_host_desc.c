/* Host test for usb_host_desc.c (configuration and string descriptors from an untrusted DUT)
   and usb_host_reply.c (the usb_host reply and its worst-case size). */
#include "usb_host.h"
#include "usb_host_desc.h"
#include "reply_budget.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

/* A CDC ACM device: config + IAD + 2 interfaces with their class descriptors + 3 endpoints. */
static const uint8_t cdc_cfg[] = {
    9, 2, 75, 0, 2, 1, 0, 0x80, 50,                 /* config: 75 bytes, 2 itf, bus powered, 100 mA */
    8, 11, 0, 2, 2, 2, 1, 0,                        /* IAD */
    9, 4, 0, 0, 1, 2, 2, 1, 0,                      /* itf 0: CDC ACM, 1 ep */
    5, 0x24, 0, 0x10, 0x01,                         /* CS header */
    5, 0x24, 1, 0, 1,                               /* call management */
    4, 0x24, 2, 2,                                  /* ACM */
    5, 0x24, 6, 0, 1,                               /* union */
    7, 5, 0x81, 3, 8, 0, 16,                        /* notify ep */
    9, 4, 1, 0, 2, 0x0A, 0, 0, 0,                   /* itf 1: CDC data, 2 eps */
    7, 5, 0x02, 2, 64, 0, 0,
    7, 5, 0x82, 2, 64, 0, 0,
};

static void test_cdc(void)
{
    usb_host_config_t c;
    CHECK(sizeof(cdc_cfg) == 75);
    usb_host_parse_config(cdc_cfg, sizeof(cdc_cfg), &c);
    CHECK(c.valid && !c.truncated && !c.malformed);
    CHECK(c.value == 1 && c.max_power_ma == 100 && c.num_itf == 2 && c.itf_count == 2);
    CHECK(c.itf[0].cls == 2 && c.itf[0].subclass == 2 && c.itf[0].protocol == 1 && c.itf[0].endpoints == 1);
    CHECK(c.itf[1].number == 1 && c.itf[1].cls == 0x0A && c.itf[1].endpoints == 2);
    CHECK(!strcmp(usb_host_class_name(2), "cdc") && !strcmp(usb_host_class_name(0xFF), "vendor"));
    CHECK(usb_host_class_name(0x42) == NULL);
}

static void test_hostile(void)
{
    usb_host_config_t c;
    uint8_t b[96];

    /* Not a config descriptor, too short, NULL. */
    usb_host_parse_config(NULL, 9, &c);           CHECK(!c.valid);
    usb_host_parse_config(cdc_cfg, 8, &c);        CHECK(!c.valid);
    memcpy(b, cdc_cfg, 9); b[1] = 4;
    usb_host_parse_config(b, 9, &c);              CHECK(!c.valid);

    /* A zero-length descriptor inside: must stop, not loop. */
    memcpy(b, cdc_cfg, sizeof(cdc_cfg));
    b[17] = 0;                                    /* the first interface's bLength */
    usb_host_parse_config(b, sizeof(cdc_cfg), &c);
    CHECK(c.valid && c.malformed && c.itf_count == 0);

    /* A length that runs past wTotalLength (no truncation): malformed. */
    memcpy(b, cdc_cfg, sizeof(cdc_cfg));
    b[2] = 20;                                    /* wTotalLength 20: cuts the first interface */
    usb_host_parse_config(b, sizeof(cdc_cfg), &c);
    CHECK(c.valid && c.malformed && c.itf_count == 0);

    /* wTotalLength longer than what the pod has: truncated, not malformed, interfaces so far. */
    usb_host_parse_config(cdc_cfg, 40, &c);
    CHECK(c.valid && c.truncated && !c.malformed && c.itf_count == 1);

    /* More interfaces than the list holds: counted, list capped. */
    uint8_t many[9 + 20 * 9];
    memset(many, 0, sizeof(many));
    memcpy(many, cdc_cfg, 9);
    many[2] = (uint8_t)sizeof(many); many[3] = 0; many[4] = 20;
    for (int i = 0; i < 20; i++) {
        uint8_t *d = many + 9 + i * 9;
        d[0] = 9; d[1] = 4; d[2] = (uint8_t)i; d[5] = 0xFF;
    }
    usb_host_parse_config(many, sizeof(many), &c);
    CHECK(c.valid && !c.malformed && c.itf_count == 20 && c.itf[USB_HOST_ITF_MAX - 1].number == USB_HOST_ITF_MAX - 1);

    /* Random bytes: never reads past the buffer (run under ASan in CI's sanitizer build). */
    srand(1);
    for (int n = 0; n < 20000; n++) {
        size_t len = (size_t)(rand() % 64);
        uint8_t *r = malloc(len ? len : 1);
        for (size_t i = 0; i < len; i++) r[i] = (uint8_t)rand();
        if (len >= 2) { r[1] = (n & 1) ? 2 : 3; if (n & 2) r[0] = 9; }
        usb_host_parse_config(r, len, &c);
        char s[USB_HOST_STR_MAX];
        usb_host_string_ascii(r, len, s, sizeof(s));
        CHECK(strlen(s) < sizeof(s));
        free(r);
    }
}

static void test_strings(void)
{
    char s[USB_HOST_STR_MAX];
    const uint8_t pico[] = { 10, 3, 'P', 0, 'i', 0, 'c', 0, 'o', 0 };
    CHECK(usb_host_string_ascii(pico, sizeof(pico), s, sizeof(s)) && !strcmp(s, "Pico"));
    /* Non-ASCII and JSON specials become '?'; an odd bLength drops the half character. */
    const uint8_t odd[] = { 9, 3, 'a', 0, '"', 0, 0xE9, 0x00, '\\' };
    CHECK(usb_host_string_ascii(odd, sizeof(odd), s, sizeof(s)) && !strcmp(s, "a??"));
    /* bLength longer than the buffer: the buffer wins. */
    const uint8_t lie[] = { 200, 3, 'x', 0 };
    CHECK(usb_host_string_ascii(lie, sizeof(lie), s, sizeof(s)) && !strcmp(s, "x"));
    /* Cut at the output size. */
    uint8_t lng[2 + 2 * 100];
    lng[0] = sizeof(lng) & 0xFF; lng[1] = 3;
    for (int i = 0; i < 100; i++) { lng[2 + 2 * i] = 'A'; lng[3 + 2 * i] = 0; }
    lng[0] = 202 & 0xFF;
    CHECK(usb_host_string_ascii(lng, sizeof(lng), s, sizeof(s)) && strlen(s) == USB_HOST_STR_MAX - 1);
    /* Not a string descriptor. */
    const uint8_t dev[] = { 18, 1, 0, 2 };
    CHECK(!usb_host_string_ascii(dev, sizeof(dev), s, sizeof(s)) && s[0] == '\0');
}

static void test_reply(void)
{
    char out[BP_CLOUD_REPLY_MAX];
    usb_host_info_t i;
    memset(&i, 0, sizeof(i));
    size_t n = usb_host_reply_json(&i, out, sizeof(out));
    CHECK(n > 0 && !strcmp(out, "{\"mode\":\"device\",\"port\":\"none\",\"remaining_s\":0,\"host_ms\":0,"
                                "\"enumerations\":0,\"attached\":false,\"enumeration\":\"none\","
                                "\"connected\":false,\"line_speed\":\"none\"}"));

    /* Worst case: every number at its widest, full strings, a full interface list. */
    i.mode = USB_HOST_MODE_STOPPING; i.port = USB_HOST_PORT_USB_A;
    i.remaining_s = 4294967295u; i.host_ms = 4294967295u; i.enumerations = 65535;
    i.attached = i.seen = true; i.speed = 1;
    i.enumeration = USB_HOST_ENUM_RUNNING;   /* the longest state name */
    i.connected = true;
    i.vid = i.pid = 65535; i.bcd_usb = i.bcd_device = 0xFFFF;
    i.dev_class = i.dev_subclass = i.dev_protocol = i.ep0_size = i.num_configs = 255;
    i.descriptor_ms = i.mounted_ms = 4294967295u;
    memset(i.manufacturer, 'M', USB_HOST_STR_MAX - 1);
    memset(i.product, 'P', USB_HOST_STR_MAX - 1);
    memset(i.serial, 'S', USB_HOST_STR_MAX - 1);
    i.config.valid = i.config.truncated = i.config.malformed = true;
    i.config.value = 255; i.config.max_power_ma = 510; i.config.attributes = 0xE0;
    i.config.itf_count = 255;
    for (int k = 0; k < USB_HOST_ITF_MAX; k++) {
        i.config.itf[k] = (usb_host_itf_t){ 255, 255, 0xFE, 255, 255, 255 };   /* longest class name */
    }
    n = usb_host_reply_json(&i, out, sizeof(out));
    CHECK(n > 0);
    /* send_ok_large wraps it: {"status":"ok","data":<payload>} plus the newline. */
    size_t wire = n + strlen("{\"status\":\"ok\",\"data\":}") + 1;
    CHECK(reply_budget_ok("usb_host (worst case)", wire, REPLY_LIMIT_CLOUD, REPLY_BUDGET_MARGIN));
    CHECK(strstr(out, "\"speed\":\"low\"") && strstr(out, "\"usb\":\"ff.ff\"") &&
          strstr(out, "\"self_powered\":true") && strstr(out, "\"class_name\":\"app_specific\""));

    /* Too small a buffer: 0, never a cut reply. */
    CHECK(usb_host_reply_json(&i, out, 100) == 0);

    /* The enumeration state: a device descriptor without a mount is stuck after USB_HOST_STUCK_MS. */
    CHECK(usb_host_enum_state(false, false, false, 0) == USB_HOST_ENUM_NONE);
    CHECK(usb_host_enum_state(true, false, true, 10) == USB_HOST_ENUM_RUNNING);
    CHECK(usb_host_enum_state(true, false, true, USB_HOST_STUCK_MS) == USB_HOST_ENUM_STUCK);
    CHECK(usb_host_enum_state(true, true, false, 99999) == USB_HOST_ENUM_DONE);
    CHECK(usb_host_enum_state(true, false, false, 99999) == USB_HOST_ENUM_NONE);   /* detached */
    CHECK(!strcmp(usb_host_enum_name(USB_HOST_ENUM_STUCK), "stuck"));

    CHECK(usb_host_port_from_name("usb_c") == USB_HOST_PORT_USB_C);
    CHECK(usb_host_port_from_name("usb_a") == USB_HOST_PORT_USB_A);
    CHECK(usb_host_port_from_name("usb") == USB_HOST_PORT_NONE);
    CHECK(usb_host_port_from_name(NULL) == USB_HOST_PORT_NONE);
}

int main(void)
{
    test_cdc();
    test_hostile();
    test_strings();
    test_reply();
    printf("test_usb_host_desc: %s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
