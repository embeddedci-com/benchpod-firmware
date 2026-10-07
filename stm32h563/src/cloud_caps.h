#ifndef CLOUD_CAPS_H
#define CLOUD_CAPS_H

/*
 * cloud_caps: the one place that builds the `capabilities` frame the pod sends the server on
 * every connect (and after a gateware swap). cloud_client.c gathers the values; this file only
 * formats them, so a host test can build the worst case (test_cloud_caps.c).
 *
 * The frame must fit one WS frame (cl_ws_send's scratch). A frame that did not fit used to be
 * dropped, and a dropped capabilities frame failed the connect, so the pod reconnected in a loop.
 * The fixed feature fields are bounded by the format; only the boot-health text (safe-mode reason,
 * last crash) is free-form. When the whole frame would not fit, those texts are shortened (and
 * "boot_health_cut":true says so); the feature fields are never dropped.
 */

#include <stdbool.h>
#include <stddef.h>

#include "bp_limits.h"

/* The largest capabilities payload: cl_ws_send's scratch minus the WS header. */
#define CLOUD_CAPS_MAX        (BP_WS_FRAME_MAX - BP_WS_HEADER_MAX)
/* Free-form boot-health text is cut to this many characters when the frame runs long. */
#define CLOUD_CAPS_TEXT_SHORT 96u

typedef struct {
    const char   *device_id;
    const char   *firmware_version;
    unsigned long flash_kb;
    const char   *sig_policy;
    const char   *lan_policy;
    bool          analog;
    int           adc_bits, adc_fullscale_mv, adc_channels;
    long          adc_cal_a_uv, adc_cal_b_nv;
    bool          dac_ac, dac_replay, dac_dc;
    int           dac_bits, dac_replay_bits, dac_fullscale_mv, dac_channels;
    bool          deep_replay;
    unsigned long replay_max_samples;
    bool          control_loop, loop_sources, loop_input_map, cotrig;
    bool          gpio_read, capture_trigger, spi_master;
    bool          nrst_pin;
    bool          pod_current;
    long          current_out_min_ua, current_out_max_ua;
    const char   *board;
    /* Boot health. */
    bool          safe_mode;
    const char   *safe_reason;    /* "" when not in safe mode */
    const char   *reset_cause;
    const char   *last_crash;     /* "none" after a clean start */
} cloud_caps_t;

/* Build the frame into out (cap >= CLOUD_CAPS_MAX + 1 recommended). Returns its length, or 0
   when even the shortened form does not fit `cap`. */
size_t cloud_caps_build(const cloud_caps_t *c, char *out, size_t cap);

#endif /* CLOUD_CAPS_H */
