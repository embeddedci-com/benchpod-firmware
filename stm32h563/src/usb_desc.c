/*
 * usb_desc.c — USB descriptors: a composite device with the CDC-ACM console
 * (interfaces 0-1) and a CDC-NCM network adapter (interfaces 2-3).
 *
 * NCM is the one USB network class with an in-box driver on macOS, Linux and
 * Windows (10 1903+). Windows only binds it on its own when the device names it
 * through a Microsoft OS 2.0 descriptor (compatible ID "WINNCM"), so the device
 * reports USB 2.1 and a BOS descriptor that points at that set.
 *
 * SPIKE: VID 0x1209 PID 0x0001 is the pid.codes test ID. A product needs its own.
 */
#include "tusb.h"
#include "usb_device.h"
#include "board_uid.h"

#include <string.h>

#define USB_VID   0x1209
#define USB_PID   0x0001
#define USB_BCD   0x0100

enum { ITF_CDC = 0, ITF_CDC_DATA, ITF_NCM, ITF_NCM_DATA, ITF_TOTAL };

enum { STR_LANGID = 0, STR_MANUFACTURER, STR_PRODUCT, STR_SERIAL,
       STR_CDC, STR_NCM, STR_NCM_MAC, STR_COUNT };

#define EP_CDC_NOTIF  0x81
#define EP_CDC_OUT    0x02
#define EP_CDC_IN     0x82
#define EP_NCM_NOTIF  0x83
#define EP_NCM_OUT    0x04
#define EP_NCM_IN     0x84

#define VENDOR_REQUEST_MICROSOFT  1

/* ---- device ---------------------------------------------------------------------------- */
static const tusb_desc_device_t s_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0210,             /* 2.1: the host asks for the BOS */
    /* IAD composite: class 0xEF / 0x02 / 0x01. */
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = USB_BCD,
    .iManufacturer      = STR_MANUFACTURER,
    .iProduct           = STR_PRODUCT,
    .iSerialNumber      = STR_SERIAL,
    .bNumConfigurations = 1,
};

uint8_t const *tud_descriptor_device_cb(void) { return (uint8_t const *)&s_device; }

/* ---- configuration --------------------------------------------------------------------- */
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_CDC_NCM_DESC_LEN)

static const uint8_t s_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 500),
    TUD_CDC_DESCRIPTOR(ITF_CDC, STR_CDC, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 64),
    TUD_CDC_NCM_DESCRIPTOR(ITF_NCM, STR_NCM, STR_NCM_MAC, EP_NCM_NOTIF, 64, EP_NCM_OUT, EP_NCM_IN,
                           64, CFG_TUD_NET_MTU, 50,
                           (uint8_t)(NCM_NETWORK_CAPS_ETH_FILTER | NCM_NETWORK_CAPS_NTB_INPUT_SIZE)),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return s_config;
}

/* ---- BOS + Microsoft OS 2.0 (Windows binds UsbNcm.sys to the NCM function) ------------- */
#define MS_OS_20_DESC_LEN  (10 + 8 + 8 + 20)
#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

static const uint8_t s_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT),
};

uint8_t const *tud_descriptor_bos_cb(void) { return s_bos; }

static const uint8_t s_ms_os_20[MS_OS_20_DESC_LEN] = {
    /* set header: length, type, Windows 8.1+, total length */
    U16_TO_U8S_LE(10), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),
    /* configuration subset header: configuration index 0 */
    U16_TO_U8S_LE(8), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0,
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 10),
    /* function subset header: the NCM function starts at its control interface */
    U16_TO_U8S_LE(8), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), ITF_NCM, 0,
    U16_TO_U8S_LE(8 + 20),
    /* compatible ID: WINNCM */
    U16_TO_U8S_LE(20), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W', 'I', 'N', 'N', 'C', 'M', 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
};

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req)
{
    if (stage != CONTROL_STAGE_SETUP) return true;
    /* wIndex 7 = MS_OS_20_DESCRIPTOR_INDEX */
    if (req->bRequest == VENDOR_REQUEST_MICROSOFT && req->wIndex == 7)
        return tud_control_xfer(rhport, req, (void *)(uintptr_t)s_ms_os_20, sizeof(s_ms_os_20));
    return false;   /* stall anything else */
}

/* ---- strings --------------------------------------------------------------------------- */
static char s_serial[13];    /* 12 hex digits, as the ST stack derived them from the UID */
static char s_host_mac[13];  /* the host side's MAC for the NCM function */

static void hex_u32(char *out, uint32_t v, int digits)
{
    for (int i = 0; i < digits; i++) {
        uint8_t nib = (uint8_t)(v >> 28);
        out[i] = (char)(nib < 10 ? '0' + nib : 'A' + nib - 10);
        v <<= 4;
    }
}

static void build_strings(void)
{
    if (s_serial[0]) return;
    uint32_t uid[3];
    board_uid_words(uid);
    hex_u32(&s_serial[0], uid[0] + uid[2], 8);
    hex_u32(&s_serial[8], uid[1], 4);

    uint8_t mac[6];
    usb_net_host_mac(mac);
    for (int i = 0; i < 6; i++) hex_u32(&s_host_mac[2 * i], (uint32_t)mac[i] << 24, 2);
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    static uint16_t desc[32];
    const char *s;
    build_strings();
    switch (index) {
    case STR_LANGID:
        desc[1] = 0x0409;
        desc[0] = (uint16_t)((TUSB_DESC_STRING << 8) | 4);
        return desc;
    case STR_MANUFACTURER: s = "EmbeddedCI"; break;
    case STR_PRODUCT:      s = "BenchPod"; break;
    case STR_SERIAL:       s = s_serial; break;
    case STR_CDC:          s = "BenchPod console"; break;
    case STR_NCM:          s = "BenchPod network"; break;
    case STR_NCM_MAC:      s = s_host_mac; break;
    default:               return NULL;
    }
    size_t n = strlen(s);
    if (n > 31) n = 31;
    for (size_t i = 0; i < n; i++) desc[1 + i] = (uint8_t)s[i];
    desc[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return desc;
}
