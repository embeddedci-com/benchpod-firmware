/*
 * command_handler_bus_send.c — send and capture: la_capture's optional "send".
 *
 *   {"cmd":"la_capture","samples":4096,"sample_rate_mhz":12,"trigger":{"la":1,"edge":"falling"},
 *    "send":{"bus":"i2c","addr":72,"write":"Dw","read":2}}
 *   "send":{"bus":"spi","tx":"nwAAAA"}
 *   "send":{"bus":"gpio","la":5,"level":0}
 *
 * The capture and the transaction are one command, so they also work over the cloud, which runs
 * one command at a time.  bus_send_prepare() validates it before the capture is armed (on an
 * error nothing is armed), bus_send_run() runs it right after the arm, and the result rides the
 * capture's last reply frame as "send":{...}, next to "trigger".  The bus needs its session
 * (i2c_start / spi_start) or, for gpio, a pin that is already a gpio output or open_drain.
 */
#include "command_handler.h"
#include "command_handler_internal.h"
#include "signal_engine.h"
#include "la_pins.h"
#include "bp_json.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { SEND_NONE = 0, SEND_I2C, SEND_SPI, SEND_GPIO } send_kind_t;

static send_kind_t s_kind;
static unsigned    s_gpio_la, s_gpio_level;
static bool        s_have_result;
static char        s_result[BUS_SEND_RESULT_MAX];

bool bus_send_prepare(int conn_id, const char *json) {
    s_kind = SEND_NONE;
    s_have_result = false;
    char probe[8];
    if (!bp_json_get(json, "send", probe, sizeof(probe)) || strcmp(probe, "null") == 0) return true;
    char obj[256], bus[8], err[160];
    if (!bp_json_object(json, "send", obj, sizeof(obj)) || !bp_json_get(obj, "bus", bus, sizeof(bus))) {
        send_error(conn_id, "send must be an object like {\"bus\":\"i2c\",\"addr\":72,\"read\":2}");
        return false;
    }
    if (strcmp(bus, "i2c") == 0) {
        if (i2c_capture_prepare(obj, err, sizeof(err)) != 0) { send_error(conn_id, err); return false; }
        s_kind = SEND_I2C;
    } else if (strcmp(bus, "spi") == 0) {
        if (spi_capture_prepare(obj, err, sizeof(err)) != 0) { send_error(conn_id, err); return false; }
        s_kind = SEND_SPI;
    } else if (strcmp(bus, "gpio") == 0) {
        char s[8];
        long la = bp_json_get(obj, "la", s, sizeof(s)) ? strtol(s, NULL, 10) : 0;
        long lv = bp_json_get(obj, "level", s, sizeof(s)) ? strtol(s, NULL, 10) : -1;
        la_gpio_mode_t m = la_pins_gpio((unsigned)la);
        if (la < 1 || la > (long)LA_PINS_COUNT || (lv != 0 && lv != 1)) {
            send_error(conn_id, "send: gpio needs la 1..14 and level 0 or 1"); return false;
        }
        if (la_pins_fn((unsigned)la) != LA_FN_GPIO || (m != LA_GPIO_OUTPUT && m != LA_GPIO_OPEN_DRAIN)) {
            snprintf(err, sizeof(err), "send: LA%ld is not a gpio output or open_drain; configure it with "
                     "{\"cmd\":\"gpio\",\"la\":%ld,\"mode\":\"output\"} first", la, la);
            send_error(conn_id, err); return false;
        }
        s_gpio_la = (unsigned)la; s_gpio_level = (unsigned)lv;
        s_kind = SEND_GPIO;
    } else {
        send_error(conn_id, "send bus must be i2c, spi or gpio");
        return false;
    }
    return true;
}

void bus_send_run(void) {
    switch (s_kind) {
    case SEND_I2C: i2c_capture_run(s_result, sizeof(s_result)); break;
    case SEND_SPI: spi_capture_run(s_result, sizeof(s_result)); break;
    case SEND_GPIO:
        la_pins_set_level(s_gpio_la, (uint8_t)s_gpio_level);
        (void)fpga_la_set(s_gpio_la, la_pins_drive(s_gpio_la));
        snprintf(s_result, sizeof(s_result), "{\"bus\":\"gpio\",\"la\":%u,\"level\":%u}", s_gpio_la, s_gpio_level);
        break;
    default: return;
    }
    s_kind = SEND_NONE;
    s_have_result = true;
}

void bus_send_cancel(void) { s_kind = SEND_NONE; s_have_result = false; }

size_t bus_send_take(char *out, size_t cap) {
    if (!s_have_result) return 0;
    s_have_result = false;
    int n = snprintf(out, cap, ",\"send\":%s", s_result);
    return (n > 0 && (size_t)n < cap) ? (size_t)n : 0;
}
