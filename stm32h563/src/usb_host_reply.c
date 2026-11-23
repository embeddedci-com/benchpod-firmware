/* usb_host_reply.c — names and the usb_host command's reply (usb_host.h). Pure: host tests
   build it with the worst case to check the reply budget. */
#include "usb_host.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

const char *usb_host_port_name(usb_host_port_t port)
{
    return port == USB_HOST_PORT_USB_A ? "usb_a" : port == USB_HOST_PORT_USB_C ? "usb_c" : "none";
}

usb_host_port_t usb_host_port_from_name(const char *name)
{
    if (!name) return USB_HOST_PORT_NONE;
    if (!strcmp(name, "usb_a")) return USB_HOST_PORT_USB_A;
    if (!strcmp(name, "usb_c")) return USB_HOST_PORT_USB_C;
    return USB_HOST_PORT_NONE;
}

const char *usb_host_enum_name(usb_host_enum_t e)
{
    switch (e) {
    case USB_HOST_ENUM_RUNNING: return "running";
    case USB_HOST_ENUM_DONE:    return "done";
    case USB_HOST_ENUM_STUCK:   return "stuck";
    default:                    return "none";
    }
}

usb_host_enum_t usb_host_enum_state(bool seen, bool attached, bool open, uint32_t since_ms)
{
    if (attached) return USB_HOST_ENUM_DONE;
    if (seen && open) return since_ms >= USB_HOST_STUCK_MS ? USB_HOST_ENUM_STUCK : USB_HOST_ENUM_RUNNING;
    return USB_HOST_ENUM_NONE;
}

const char *usb_host_mode_name(usb_host_mode_t mode)
{
    switch (mode) {
    case USB_HOST_MODE_STARTING: return "starting";
    case USB_HOST_MODE_HOST:     return "host";
    case USB_HOST_MODE_STOPPING: return "stopping";
    default:                     return "device";
    }
}

typedef struct { char *p; size_t cap, len; bool over; } jbuf_t;

static void jp(jbuf_t *b, const char *fmt, ...)
{
    if (b->over) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b->p + b->len, b->cap - b->len, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= b->cap - b->len) { b->over = true; return; }
    b->len += (size_t)n;
}

/* BCD 0x0210 -> "2.10" */
static void bcd(char out[8], uint16_t v)
{
    snprintf(out, 8, "%x.%02x", (unsigned)(v >> 8), (unsigned)(v & 0xFF));
}

size_t usb_host_reply_json(const usb_host_info_t *i, char *out, size_t cap)
{
    if (!out || cap == 0) return 0;
    jbuf_t b = { out, cap, 0, false };
    jp(&b, "{\"mode\":\"%s\",\"port\":\"%s\",\"remaining_s\":%lu,\"host_ms\":%lu,"
           "\"enumerations\":%u,\"attached\":%s,\"enumeration\":\"%s\"",
       usb_host_mode_name(i->mode), usb_host_port_name(i->port), (unsigned long)i->remaining_s,
       (unsigned long)i->host_ms, (unsigned)i->enumerations, i->attached ? "true" : "false",
       usb_host_enum_name(i->enumeration));
    if (i->seen) {
        char usb[8], rel[8];
        bcd(usb, i->bcd_usb);
        bcd(rel, i->bcd_device);
        jp(&b, ",\"device\":{\"vid\":%u,\"pid\":%u,\"speed\":\"%s\",\"usb\":\"%s\",\"release\":\"%s\","
               "\"class\":%u,\"subclass\":%u,\"protocol\":%u,\"ep0_size\":%u,\"configurations\":%u,"
               "\"manufacturer\":\"%s\",\"product\":\"%s\",\"serial\":\"%s\","
               "\"descriptor_ms\":%lu,\"mounted_ms\":%lu",
           (unsigned)i->vid, (unsigned)i->pid, i->speed == 0 ? "full" : i->speed == 1 ? "low" : "high",
           usb, rel, (unsigned)i->dev_class, (unsigned)i->dev_subclass, (unsigned)i->dev_protocol,
           (unsigned)i->ep0_size, (unsigned)i->num_configs,
           i->manufacturer, i->product, i->serial,
           (unsigned long)i->descriptor_ms, (unsigned long)i->mounted_ms);
        const usb_host_config_t *c = &i->config;
        if (c->valid) {
            jp(&b, ",\"config\":{\"value\":%u,\"max_power_ma\":%u,\"self_powered\":%s,"
                   "\"remote_wakeup\":%s,\"interface_count\":%u,\"truncated\":%s,\"malformed\":%s,"
                   "\"interfaces\":[",
               (unsigned)c->value, (unsigned)c->max_power_ma,
               (c->attributes & 0x40) ? "true" : "false", (c->attributes & 0x20) ? "true" : "false",
               (unsigned)c->itf_count, c->truncated ? "true" : "false", c->malformed ? "true" : "false");
            unsigned n = c->itf_count < USB_HOST_ITF_MAX ? c->itf_count : USB_HOST_ITF_MAX;
            for (unsigned k = 0; k < n; k++) {
                const usb_host_itf_t *t = &c->itf[k];
                const char *name = usb_host_class_name(t->cls);
                jp(&b, "%s{\"number\":%u,\"alt\":%u,\"class\":%u,\"class_name\":\"%s\","
                       "\"subclass\":%u,\"protocol\":%u,\"endpoints\":%u}",
                   k ? "," : "", (unsigned)t->number, (unsigned)t->alt, (unsigned)t->cls,
                   name ? name : "", (unsigned)t->subclass, (unsigned)t->protocol, (unsigned)t->endpoints);
            }
            jp(&b, "]}");
        }
        jp(&b, "}");
    }
    jp(&b, "}");
    return b.over ? 0 : b.len;
}
