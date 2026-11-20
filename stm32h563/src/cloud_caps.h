#ifndef CLOUD_CAPS_H
#define CLOUD_CAPS_H

/*
 * cloud_caps: the pod's capabilities, gathered once (cloud_caps_collect, cloud_caps_hw.c) and
 * written two ways here: the `capabilities` frame the pod sends the server on every connect (and
 * after a gateware swap), and the caps[] list of the `status` reply, the only capability source a
 * direct LAN or USB client sees. Both come from one cloud_caps_t, so they cannot disagree about
 * what this pod can do. The formatting is host-tested (test_cloud_caps.c), including the worst
 * case of the frame.
 *
 * The frame must fit one WS frame (cl_ws_send's scratch). A frame that did not fit used to be
 * dropped, and a dropped capabilities frame failed the connect, so the pod reconnected in a loop.
 * The fixed feature fields are bounded by the format; only the boot-health text (safe-mode reason,
 * last crash) is free-form. When the whole frame would not fit, those texts are shortened (and
 * "boot_health_cut":true says so); the feature fields are never dropped.
 */

#include <stdbool.h>
#include <stddef.h>

#include "bp_json.h"
#include "bp_limits.h"

/* The largest capabilities payload: cl_ws_send's scratch minus the WS header. */
#define CLOUD_CAPS_MAX        (BP_WS_FRAME_MAX - BP_WS_HEADER_MAX)
/* Free-form boot-health text is cut to this many characters when the frame runs long. */
#define CLOUD_CAPS_TEXT_SHORT 80u

typedef struct {
    const char   *device_id;
    const char   *firmware_version;
    unsigned long flash_kb;
    const char   *sig_policy;
    const char   *lan_policy;
    bool          analog;
    int           adc_bits, adc_fullscale_mv, adc_channels;
    long          adc_cal_a_uv, adc_cal_b_nv;   /* the ext fit of the range in use */
    bool          adc_low_range;  /* the ADC low range (analog module v3r4, K5): adc_range */
    const char   *adc_range;      /* "normal" / "low": the range adc_cal_* and adc_range_fs_mv are for */
    long          adc_range_fs_mv; /* that range's input full scale, mV (+-) */
    bool          dac_ac, dac_replay, dac_dc;
    int           dac_bits, dac_replay_bits, dac_fullscale_mv, dac_channels;
    bool          deep_replay;
    unsigned long replay_max_samples;
    bool          control_loop, loop_sources, loop_input_map, cotrig;
    bool          generate16;     /* generate's calibrated form: volts/uA at 16 bits (wave16.h) */
    bool          gpio_read, capture_trigger, spi_master;
    bool          uart_frame;     /* uart_proxy_start parity / stop_bits (gateware v56+) */
    unsigned long la_max_rate_hz;   /* LA top rate: 48 MHz with the change capture (gw >= v50), else 12 */
    bool          sensors_digital; /* the emulation image (gateware v54+) is installed or running: I2C/SPI/UART sensors */
    /* The table clients look features up in (features/features.toml): the revision this firmware
       is built from (BP_FEATURES_REV), the running gateware and its image ("loop", "deep", "emu"
       or "unknown"). */
    unsigned      features_rev;
    /* The client API level (BP_API) and the lowest client_api the pod takes (BP_MIN_CLIENT_API),
       announced next to features_rev (features.toml, cmd_gate_client_api). */
    unsigned      api;
    unsigned      min_client_api;
    unsigned      gateware_version;
    const char   *fpga_image;
    bool          nrst_pin;
    bool          usb_cc;         /* the USB-C CC monitor (rev3+): status caps[] only */
    bool          pod_current;
    /* J1 pin 21 is switched and sensed, and 5 V I/O exists (digital v3r4, board_caps.h): the hw
       fact vref_out. */
    bool          vref_out;
    /* No Wi-Fi to scan: no ESP32-C3 (digital v3r3) and no Wi-Fi bridge build. Drops wifi_scan. */
    bool          no_wifi_scan;
    long          current_out_min_ua, current_out_max_ua;
    const char   *board;
    /* The interface the cloud link runs over: "eth", "wifi", "usb", "wifi-bridge" or "unknown"
       (NULL = "unknown").
       Only the cloud client knows it (cloud_caps_collect leaves it NULL); the status reply's
       "net"/"wifi" fields describe the interfaces, not which one carries the cloud. */
    const char   *uplink;
    /* With "wifi-bridge" (the link runs over Ethernet into a connected Wi-Fi
       bridge box), the bridge's RSSI is sent as "uplink_rssi_dbm". */
    bool          has_uplink_rssi;
    int           uplink_rssi_dbm;
    /* Boot health. */
    bool          safe_mode;
    const char   *safe_reason;    /* "" when not in safe mode */
    const char   *reset_cause;
    const char   *last_crash;     /* "none" after a clean start */
    uint32_t      boot_id;        /* random per boot: the server's key for one boot's crash */
    uint32_t      unclean_resets; /* crashes + watchdog resets since the last frame went out */
    bool          c3;             /* the board has the built-in ESP32-C3 (Wi-Fi): the two below */
    uint32_t      c3_restarts;    /* C3 restarts since power-on (c3_restarts.h) */
    const char   *c3_restart_last;/* the last one's cause name ("no_response", ...), NULL if none */
    /* The MACs a network admin registers before giving the pod an address, "aa:bb:cc:dd:ee:ff".
       eth_mac: the Ethernet port's ("" when unknown). wifi_mac: the address the pod uses on
       Wi-Fi: the C3's station MAC once the C3 told it (a join or a scan since power-on), or the
       Ethernet MAC through a Wi-Fi bridge, which joins with the pod's MAC. "" = not known yet.
       has_wifi: the pod has Wi-Fi at all (a C3, or a bridge plugged in). */
    char          eth_mac[18];
    char          wifi_mac[18];
    bool          has_wifi;
} cloud_caps_t;

/* Build the frame into out (cap >= CLOUD_CAPS_MAX + 1 recommended). Returns its length, or 0
   when even the shortened form does not fit `cap`. */
size_t cloud_caps_build(const cloud_caps_t *c, char *out, size_t cap);

/* The `status` reply's ,"caps":[...] list from the same values. FROZEN: no new names (clients
   take new features from features/features.toml; tools/gen_features.py --check fails on one). */
void cloud_caps_emit_list(bp_emit_t *e, const cloud_caps_t *c);

/* What a `status` with "caps":false gets instead of that list: ,"features_rev":N,"api":A,
   "min_client_api":M,"fpga_image":"...","hw":[...], the hardware facts the table's `requires`
   names. */
void cloud_caps_emit_facts(bp_emit_t *e, const cloud_caps_t *c);

/* The running image's name from the gateware's feature bits (FPGA_FEATURE_*). */
const char *cloud_caps_image_name(unsigned gateware_version, unsigned feature_bits);

/* Fill *c from the drivers (firmware only, cloud_caps_hw.c). device_id is left NULL: only the
   cloud client has it. */
void cloud_caps_collect(cloud_caps_t *c);

/* The pod's Wi-Fi MAC into out ("" while not known, see cloud_caps_t.wifi_mac); returns whether
   the pod has Wi-Fi at all (firmware only, cloud_caps_hw.c). */
bool cloud_caps_wifi_mac(char out[18]);

#endif /* CLOUD_CAPS_H */
