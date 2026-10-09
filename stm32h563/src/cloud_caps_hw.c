/*
 * cloud_caps_hw.c: gathers the pod's capabilities (cloud_caps.h) from the drivers, once, for both
 * the cloud `capabilities` frame and the `status` reply's caps[] (they used to gather them
 * separately and could drift).
 */
#include "cloud_caps.h"

#include "board_info.h"
#include "board_variant.h"
#include "boot_guard.h"
#include "cal_data.h"        /* ADC_CAL_EXT: the front-SMA cal shipped in capabilities */
#include "current_out.h"
#include "fault.h"
#include "flash_layout.h"
#include "fpga_config.h"     /* FPGA_DAC_REPLAY_MAX_SAMPLES */
#include "fw_sign.h"
#include "ina238.h"
#include "nrst_ctrl.h"
#include "pod_policy.h"
#include "signal_engine.h"
#include "usb_cc.h"
#include "version.h"

#include <math.h>            /* lround: integer-scale the cal (nano printf has no %f) */
#include <string.h>

void cloud_caps_collect(cloud_caps_t *c) {
    memset(c, 0, sizeof(*c));
    /* The cached gateware version is current: the hw worker re-reads it after boot bring-up, the
       boot gateware update and every image swap, and re-announces when it is done. Reading it
       over SPI1 here would race the worker, which drives SPI1 without hw_lock. */
    signal_engine_caps_t caps;
    signal_engine_caps(&caps);
    /* A digital-only board has the gateware's DAC/ADC engines but no DAC or ADC behind them
       (board_variant.h): none of the analog features there. */
    const bool analog = board_has_analog();
    c->firmware_version = FIRMWARE_VERSION;
    c->flash_kb = (unsigned long)(flash_layout_size() / 1024u);
    c->sig_policy = fw_sign_policy_name(fw_sign_policy());
    c->lan_policy = pod_policy_lan_name(pod_policy_lan());
    c->analog = analog;
    c->adc_bits = ADC_BITS;
    c->adc_fullscale_mv = ADC_FULLSCALE_MV;
    c->adc_channels = ADC_CHANNELS;
    /* The affine front-end fit of the front-SMA path the scope capture uses (ADC_CAL_EXT, what
       handle_adc_read applies), so the server scales raw counts to the probe voltage. Shipped as
       integers (a in microvolts, b in nanovolts per count): newlib-nano's printf has no %f, and a
       %g once sent a malformed frame. The server divides back. */
    c->adc_cal_a_uv = lround((double)ADC_CAL_EXT.a * 1000000.0);
    c->adc_cal_b_nv = lround((double)ADC_CAL_EXT.b * 1000000000.0);
    c->dac_ac = analog && DAC_AC;
    c->dac_replay = analog && DAC_REPLAY;
    c->dac_dc = analog && DAC_DC;
    c->dac_bits = DAC_BITS;
    c->dac_replay_bits = DAC_REPLAY_BITS;
    c->dac_fullscale_mv = DAC_FULLSCALE_MV;
    c->dac_channels = DAC_CHANNELS;
    /* Deep replay (a waveform streamed out of PSRAM, past the 4 KB DAC BRAM) and the depth that
       allows, so the server can offer full-length replay and clamp requests. */
    c->deep_replay = analog && caps.deep_replay;
    c->replay_max_samples = c->deep_replay ? (unsigned long)FPGA_DAC_REPLAY_MAX_SAMPLES
                                           : (unsigned long)SIGNAL_MAX_SAMPLES;
    c->control_loop = analog && caps.control_loop;
    c->loop_sources = analog && caps.loop_sources;
    c->loop_input_map = analog && caps.loop_input_map;
    c->cotrig = analog && caps.cotrig;
    c->gpio_read = caps.gpio_read;
    c->capture_trigger = caps.capture_trigger;
    c->spi_master = caps.spi_master;
    c->gps = caps.uart2;
    c->nrst_pin = nrst_ctrl_supported();     /* the DUT reset pin (rev3+): hold reset for SPI/SWD */
    c->usb_cc = usb_cc_supported();
    c->pod_current = ina_pod_present();      /* the pod's own current monitor (0x41) */
    /* The 4-20 mA output's range, so the server can turn a waveform in mA into DAC codes with the
       pod's own numbers (current_out.h). */
    c->current_out_min_ua = current_out_min_ua();
    c->current_out_max_ua = current_out_max_ua();
    c->board = BOARD_NAME;
    /* Boot health: last_crash is "none" and safe_reason "" after a clean start, which clears an
       old warning on the server. */
    c->safe_mode = boot_guard_safe_mode();
    c->safe_reason = boot_guard_reason();
    c->reset_cause = fault_last_reset_str();
    c->last_crash = fault_last_crash_str();
    c->boot_id = fault_boot_id();
    c->unclean_resets = fault_unclean_resets();
}
