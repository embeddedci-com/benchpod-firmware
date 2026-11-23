/* Host test for usb_hid_core: the report ring (order, wrap, full, long reports) and the JSON of the
   usb_hid replies (the descriptors share the reply, paging with an offset, nothing past the cap). */
#include "usb_hid_core.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static void test_ring(void)
{
    usb_hid_rep_t slots[4];
    usb_hid_ring_t r;
    usb_hid_ring_init(&r, slots, 4);
    uint8_t kb[8] = { 0, 0, 4, 0, 0, 0, 0, 0 };
    for (int i = 0; i < 4; i++) { kb[2] = (uint8_t)(4 + i); CHECK(usb_hid_ring_push(&r, (uint32_t)(10 * i), 0, kb, 8)); }
    CHECK(!usb_hid_ring_push(&r, 99, 0, kb, 8) && r.dropped == 1 && r.count == 4);
    usb_hid_rep_t e;
    CHECK(usb_hid_ring_pop(&r, &e) && e.ms == 0 && e.data[2] == 4 && e.len == 8);
    CHECK(usb_hid_ring_push(&r, 50, 1, kb, 8));          /* wraps */
    for (int i = 1; i < 4; i++) CHECK(usb_hid_ring_pop(&r, &e) && e.ms == (uint32_t)(10 * i) && e.data[2] == 4 + i);
    CHECK(usb_hid_ring_pop(&r, &e) && e.ms == 50 && e.inst == 1);
    CHECK(!usb_hid_ring_pop(&r, &e));
    uint8_t big[200];
    memset(big, 0xAB, sizeof(big));
    CHECK(usb_hid_ring_push(&r, 1, 0, big, sizeof(big)));
    CHECK(usb_hid_ring_pop(&r, &e) && e.len == USB_HID_REPORT_MAX);   /* cut, not overrun */
    CHECK(usb_hid_ring_push(&r, 2, 0, NULL, 0) && usb_hid_ring_pop(&r, &e) && e.len == 0);
}

static void test_hex(void)
{
    char o[8];
    const uint8_t b[] = { 0x05, 0xAB, 0xff };
    CHECK(usb_hid_hex(b, 3, o, sizeof(o)) == 6 && strcmp(o, "05abff") == 0);
    CHECK(usb_hid_hex(b, 3, o, 6) == 0);            /* no room for the NUL */
    CHECK(usb_hid_hex(b, 0, o, sizeof(o)) == 0 && o[0] == '\0');
}

static void test_reports_json(void)
{
    usb_hid_rep_t slots[32];
    usb_hid_ring_t r;
    usb_hid_ring_init(&r, slots, 32);
    uint8_t kb[8] = { 0, 0, 0x0b, 0, 0, 0, 0, 0 };
    for (int i = 0; i < 3; i++) usb_hid_ring_push(&r, (uint32_t)(100 + i), 0, kb, 8);
    char out[1728];
    size_t n = usb_hid_reports_json(&r, 2, out, sizeof(out));
    CHECK(n == strlen(out) && n > 0);
    CHECK(strstr(out, "{\"ms\":100,\"instance\":0,\"data\":\"00000b0000000000\"}") != NULL);
    CHECK(strstr(out, "\"count\":2,\"dropped\":0}") != NULL && r.count == 1);
    /* 32 maximum-size reports into a small buffer: stops before it overflows, valid JSON. */
    uint8_t big[64];
    memset(big, 0x11, sizeof(big));
    for (int i = 0; i < 32; i++) usb_hid_ring_push(&r, (uint32_t)i, 1, big, 64);
    n = usb_hid_reports_json(&r, 32, out, sizeof(out));
    CHECK(n > 0 && n < sizeof(out) && out[n - 1] == '}' && strstr(out, "\"count\":") != NULL);
    CHECK(r.count > 0);                              /* what did not fit stays in the ring */
    CHECK(usb_hid_reports_json(&r, 1, out, 20) == 0);
}

static void test_list_json(void)
{
    uint8_t d0[63], d1[900];
    for (unsigned i = 0; i < sizeof(d0); i++) d0[i] = (uint8_t)i;
    for (unsigned i = 0; i < sizeof(d1); i++) d1[i] = (uint8_t)(i * 3);
    usb_hid_itf_t it[2] = { { 0, 0, 1, sizeof(d0), d0 }, { 1, 1, 0, sizeof(d1), d1 } };
    char out[1728];
    size_t n = usb_hid_list_json(it, 2, true, 0, out, sizeof(out));
    CHECK(n > 0 && n == strlen(out) && n < sizeof(out));
    CHECK(strstr(out, "\"protocol\":\"keyboard\",\"descriptor_len\":63,\"descriptor_offset\":0,\"truncated\":false") != NULL);
    CHECK(strstr(out, "\"protocol\":\"none\",\"descriptor_len\":900,\"descriptor_offset\":0,\"truncated\":true") != NULL);
    n = usb_hid_list_json(&it[1], 1, true, 600, out, sizeof(out));
    CHECK(n > 0 && strstr(out, "\"descriptor_offset\":600") != NULL && strstr(out, "\"truncated\":false") != NULL);
    n = usb_hid_list_json(&it[1], 1, true, 5000, out, sizeof(out));   /* past the end: empty */
    CHECK(n > 0 && strstr(out, "\"descriptor_offset\":900") != NULL && strstr(out, "\"descriptor\":\"\"") != NULL);
    CHECK(usb_hid_list_json(NULL, 0, false, 0, out, sizeof(out)) > 0 && strstr(out, "\"interfaces\":[]") != NULL);
    CHECK(usb_hid_list_json(it, 2, true, 0, out, 40) == 0);
    /* a descriptor that could not be read */
    usb_hid_itf_t none = { 0, 0, 2, 0, NULL };
    n = usb_hid_list_json(&none, 1, true, 0, out, sizeof(out));
    CHECK(n > 0 && strstr(out, "\"protocol\":\"mouse\",\"descriptor_len\":0") != NULL);
    /* every cap from 0 up: never past the buffer */
    for (size_t cap = 0; cap < 400; cap++) {
        char small[400];
        memset(small, 0x7E, sizeof(small));
        size_t w = usb_hid_list_json(it, 2, true, 0, small, cap);
        CHECK(w < cap || w == 0);
        CHECK(small[cap] == 0x7E || cap == sizeof(small));
    }
}

int main(void)
{
    test_ring();
    test_hex();
    test_reports_json();
    test_list_json();
    printf("test_usb_hid: %s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
