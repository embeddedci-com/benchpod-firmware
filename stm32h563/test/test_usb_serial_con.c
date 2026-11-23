/* Host test for usb_serial_con.c: the receive ring, the uart_proxy_start port usb arguments and
   the state reply of the USB serial console. */
#include "usb_serial_con.h"

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
    /* nothing given: 8N1, nothing set (the console opens at 115200 with DTR and RTS high) */
    CHECK(parse("{\"cmd\":\"uart_proxy_start\",\"port\":\"usb\"}", &a) == USB_SER_ARG_OK);
    CHECK(!a.has_baud && !a.has_dtr && !a.has_rts && !a.has_frame && a.data_bits == 8);
    CHECK(parse("{\"cmd\":\"uart_proxy_start\",\"port\":\"usb\",\"baud\":115200,\"dtr\":true,\"rts\":false}", &a) == USB_SER_ARG_OK);
    CHECK(a.baud == 115200 && a.has_dtr && a.dtr && a.has_rts && !a.rts && !a.has_frame);
    CHECK(parse("{\"cmd\":\"usb_serial\",\"parity\":\"even\",\"stop_bits\":2}", &a) == USB_SER_ARG_OK);
    CHECK(a.has_frame && a.data_bits == 8 && a.parity == USB_SER_PARITY_EVEN && a.stop_bits == USB_SER_STOP_2);
    CHECK(parse("{\"cmd\":\"usb_serial\",\"data_bits\":7,\"parity\":\"odd\"}", &a) == USB_SER_ARG_OK);
    CHECK(a.data_bits == 7 && a.parity == USB_SER_PARITY_ODD && a.stop_bits == USB_SER_STOP_1);
    CHECK(parse("{\"cmd\":\"usb_serial\",\"baud\":9600}", &a) == USB_SER_ARG_OK && !a.has_frame && a.baud == 9600);
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
    for (int e = USB_SER_ARG_BAUD; e <= USB_SER_ARG_STOP_BITS; e++)
        CHECK(strstr(usb_serial_arg_err_text((usb_serial_arg_err_t)e), "uart_proxy_start port usb:") != NULL);
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
    /* a closed console reports DTR and RTS low, whatever was set before */
    s.open = false; s.dtr = true; s.rts = true;
    CHECK(usb_serial_state_json(&s, out, sizeof(out)) > 0 &&
          strstr(out, "\"open\":false") && strstr(out, "\"dtr\":false,\"rts\":false"));
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

int main(void)
{
    test_ring();
    test_args();
    test_state();
    if (failures) { printf("test_usb_serial_con: %d FAILED\n", failures); return 1; }
    printf("test_usb_serial_con: all passed\n");
    return 0;
}
