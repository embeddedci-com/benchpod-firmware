/* Host test for usb_serial_con.c: the receive ring, the usb_serial arguments, the refusal rules,
   the state reply, base64 and the `until` match of the USB serial console. */
#include "usb_serial_con.h"
#include "b64url.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static void test_ring(void)
{
    uint8_t mem[8], out[16];
    usb_ring_t r;
    usb_ring_init(&r, mem, sizeof(mem));
    CHECK(usb_ring_avail(&r) == 0 && usb_ring_pop(&r, out, 4) == 0);
    CHECK(usb_ring_push(&r, (const uint8_t *)"abcde", 5) == 5);
    CHECK(usb_ring_pop(&r, out, 3) == 3 && !memcmp(out, "abc", 3));
    /* wraps: 3 left room for 6 more (2 + 6 = 8 stored) */
    CHECK(usb_ring_push(&r, (const uint8_t *)"fghijk", 6) == 6);
    CHECK(usb_ring_avail(&r) == 8 && r.dropped == 0);
    /* full: nothing fits, all counted, what is buffered stays */
    CHECK(usb_ring_push(&r, (const uint8_t *)"xyz", 3) == 0 && r.dropped == 3);
    CHECK(usb_ring_pop(&r, out, 16) == 8 && !memcmp(out, "defghijk", 8));
    /* partly fits */
    CHECK(usb_ring_push(&r, (const uint8_t *)"0123456789", 10) == 8 && r.dropped == 5);
    /* indices running past 2^32 still work */
    r.head = r.tail = 0xFFFFFFFCu;
    CHECK(usb_ring_push(&r, (const uint8_t *)"wxyzuv", 6) == 6);
    CHECK(usb_ring_avail(&r) == 6 && usb_ring_pop(&r, out, 6) == 6 && !memcmp(out, "wxyzuv", 6));
}

static usb_serial_arg_err_t parse(const char *j, usb_serial_args_t *a) { return usb_serial_args_parse(j, a); }

static void test_args(void)
{
    usb_serial_args_t a;
    CHECK(parse("{\"cmd\":\"usb_serial\"}", &a) == USB_SER_ARG_OK && usb_serial_args_is_read(&a));
    CHECK(parse("{\"cmd\":\"usb_serial\",\"open\":true,\"baud\":115200,\"dtr\":true,\"rts\":false}", &a) == USB_SER_ARG_OK);
    CHECK(a.has_open && a.open && a.baud == 115200 && a.has_dtr && a.dtr && a.has_rts && !a.rts && !a.has_frame);
    CHECK(!usb_serial_args_is_read(&a));
    CHECK(parse("{\"cmd\":\"usb_serial\",\"open\":false}", &a) == USB_SER_ARG_OK && a.has_open && !a.open);
    CHECK(parse("{\"cmd\":\"usb_serial\",\"parity\":\"even\",\"stop_bits\":2}", &a) == USB_SER_ARG_OK);
    CHECK(a.has_frame && a.data_bits == 8 && a.parity == USB_SER_PARITY_EVEN && a.stop_bits == USB_SER_STOP_2);
    CHECK(parse("{\"cmd\":\"usb_serial\",\"data_bits\":7,\"parity\":\"odd\"}", &a) == USB_SER_ARG_OK);
    CHECK(a.data_bits == 7 && a.parity == USB_SER_PARITY_ODD && a.stop_bits == USB_SER_STOP_1);
    CHECK(parse("{\"cmd\":\"usb_serial\",\"baud\":9600}", &a) == USB_SER_ARG_OK && !a.has_frame && a.baud == 9600);
    CHECK(parse("{\"open\":\"yes\"}", &a) == USB_SER_ARG_OPEN);
    CHECK(parse("{\"baud\":299}", &a) == USB_SER_ARG_BAUD);
    CHECK(parse("{\"baud\":3000001}", &a) == USB_SER_ARG_BAUD);
    CHECK(parse("{\"baud\":\"fast\"}", &a) == USB_SER_ARG_BAUD);
    CHECK(parse("{\"baud\":-9600}", &a) == USB_SER_ARG_BAUD);
    CHECK(parse("{\"dtr\":1}", &a) == USB_SER_ARG_DTR);
    CHECK(parse("{\"rts\":\"x\"}", &a) == USB_SER_ARG_RTS);
    CHECK(parse("{\"data_bits\":9}", &a) == USB_SER_ARG_DATA_BITS);
    CHECK(parse("{\"data_bits\":4}", &a) == USB_SER_ARG_DATA_BITS);
    CHECK(parse("{\"parity\":\"mark\"}", &a) == USB_SER_ARG_PARITY);
    CHECK(parse("{\"stop_bits\":3}", &a) == USB_SER_ARG_STOP_BITS);
    for (int e = USB_SER_ARG_OPEN; e <= USB_SER_ARG_STOP_BITS; e++)
        CHECK(strstr(usb_serial_arg_err_text((usb_serial_arg_err_t)e), "usb_serial:") != NULL);
}

static void test_plan(void)
{
    usb_serial_args_t rd, op, cl, set;
    parse("{}", &rd);
    parse("{\"open\":true}", &op);
    parse("{\"open\":false}", &cl);
    parse("{\"baud\":9600}", &set);
    /* a state read and a close always go */
    CHECK(usb_serial_plan(&rd, false, false, false, true) == USB_SER_GO);
    CHECK(usb_serial_plan(&cl, false, false, false, true) == USB_SER_GO);
    CHECK(usb_serial_plan(&cl, true, false, false, true) == USB_SER_GO);
    /* open: the order of the refusals */
    CHECK(usb_serial_plan(&op, false, true, true, false) == USB_SER_GO);
    CHECK(usb_serial_plan(&op, false, true, true, true) == USB_SER_FLASHING);
    CHECK(usb_serial_plan(&op, false, false, false, true) == USB_SER_FLASHING);
    CHECK(usb_serial_plan(&op, false, false, false, false) == USB_SER_NO_HOST);
    CHECK(usb_serial_plan(&op, false, true, false, false) == USB_SER_NO_PORT);
    CHECK(usb_serial_plan(&op, true, true, true, false) == USB_SER_GO);   /* open again: new settings */
    /* settings need an open console, and survive a port that is away */
    CHECK(usb_serial_plan(&set, false, true, true, false) == USB_SER_NOT_OPEN);
    CHECK(usb_serial_plan(&set, true, true, true, false) == USB_SER_GO);
    CHECK(usb_serial_plan(&set, true, true, false, false) == USB_SER_GO);
    CHECK(usb_serial_refusal_why(USB_SER_NO_HOST)[0] && usb_serial_refusal_why(USB_SER_NO_PORT)[0] &&
          usb_serial_refusal_why(USB_SER_FLASHING)[0]);
}

static void test_state(void)
{
    usb_serial_state_t s = { .open = true, .baud = 115200, .dtr = true, .rts = false, .data_bits = 8,
                             .parity = USB_SER_PARITY_NONE, .stop_bits = USB_SER_STOP_1, .bridge = "cdc",
                             .rx_buffered = 12, .rx_dropped = 3, .tx_total = 40, .rx_total = 5000,
                             .port_present = true };
    char out[256];
    size_t n = usb_serial_state_json(&s, out, sizeof(out));
    CHECK(n == strlen(out) && n > 0);
    CHECK(!strcmp(out, "{\"open\":true,\"baud\":115200,\"frame\":\"8N1\",\"dtr\":true,\"rts\":false,\"bridge\":\"cdc\","
                       "\"rx_buffered\":12,\"rx_dropped\":3,\"tx_total\":40,\"rx_total\":5000,\"port_present\":true}"));
    CHECK(usb_serial_state_json(&s, out, 40) == 0);   /* does not fit */
    char f[4];
    usb_serial_frame_str(7, USB_SER_PARITY_EVEN, USB_SER_STOP_2, f);
    CHECK(!strcmp(f, "7E2"));
    usb_serial_frame_str(8, USB_SER_PARITY_ODD, USB_SER_STOP_1, f);
    CHECK(!strcmp(f, "8O1"));
    CHECK(!strcmp(usb_serial_chip_name(false, 0x0403), "cdc"));
    CHECK(!strcmp(usb_serial_chip_name(true, 0x0403), "ftdi"));
    CHECK(!strcmp(usb_serial_chip_name(true, 0x10C4), "cp210x"));
    CHECK(!strcmp(usb_serial_chip_name(true, 0x1A86), "ch34x"));
    CHECK(!strcmp(usb_serial_chip_name(true, 0x067B), "pl2303"));
    CHECK(!strcmp(usb_serial_chip_name(true, 0x1234), "bridge"));
}

static void test_data(void)
{
    uint8_t out[32];
    size_t n = 99;
    CHECK(usb_serial_b64_decode("aGVsbG8=", out, sizeof(out), &n) == 0 && n == 5 && !memcmp(out, "hello", 5));
    CHECK(usb_serial_b64_decode("aGVsbG8", out, sizeof(out), &n) == 0 && n == 5);          /* no padding */
    CHECK(usb_serial_b64_decode("-_-_", out, sizeof(out), &n) == 0 && n == 3);             /* URL alphabet */
    uint8_t a[3], b[3];
    size_t na, nb;
    CHECK(usb_serial_b64_decode("+/+/", a, 3, &na) == 0 && usb_serial_b64_decode("-_-_", b, 3, &nb) == 0 &&
          na == 3 && nb == 3 && !memcmp(a, b, 3));                                         /* standard = URL */
    CHECK(usb_serial_b64_decode("", out, sizeof(out), &n) == 0 && n == 0);
    CHECK(usb_serial_b64_decode("a", out, sizeof(out), &n) != 0);                          /* impossible length */
    CHECK(usb_serial_b64_decode("aGVs bG8", out, sizeof(out), &n) != 0);                   /* white space */
    CHECK(usb_serial_b64_decode("aGVsbG8=", out, 4, &n) != 0);                             /* too long for the buffer */
    /* the reply's base64url comes back through the decoder */
    char enc[16];
    const uint8_t raw[] = { 0, 0xFF, 0xFE, 'A', 10 };
    CHECK(b64url_encode(raw, sizeof(raw), enc, sizeof(enc)) > 0);
    CHECK(usb_serial_b64_decode(enc, out, sizeof(out), &n) == 0 && n == sizeof(raw) && !memcmp(out, raw, n));

    const uint8_t d[] = "boot ok\r\n> ";
    CHECK(usb_serial_ends_with(d, 11, (const uint8_t *)"> ", 2));
    CHECK(!usb_serial_ends_with(d, 11, (const uint8_t *)"ok", 2));
    CHECK(!usb_serial_ends_with(d, 1, (const uint8_t *)"ab", 2));
    CHECK(!usb_serial_ends_with(d, 11, (const uint8_t *)"", 0));
}

int main(void)
{
    test_ring();
    test_args();
    test_plan();
    test_state();
    test_data();
    if (failures) { printf("test_usb_serial_con: %d FAILED\n", failures); return 1; }
    printf("test_usb_serial_con: all passed\n");
    return 0;
}
