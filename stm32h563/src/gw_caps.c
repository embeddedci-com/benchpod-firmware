/* gw_caps.c: see gw_caps.h. */
#include "gw_caps.h"
#include "signal_engine.h"   /* *_MIN_GW, FPGA_FEATURE_*, the running version and features */
#include "la_rate.h"         /* EXACT_CAP_DIVIDER_MIN_GW, RELOAD_WIRE_MIN_GW, LA_CHANGE_MIN_GW */

gw_caps_t gw_caps_for(uint8_t v, uint8_t f)
{
    gw_caps_t c;
    c.dac_deep_replay     = v >= DAC_DEEP_REPLAY_MIN_GW;
    c.concurrent_psram    = v >= CONCURRENT_PSRAM_MIN_GW;
    c.dac_capture_stop    = v >= DAC_CAPTURE_STOP_MIN_GW;
    c.capture_bases       = v >= CAPTURE_BASES_MIN_GW;
    c.dac_control_loop    = v >= DAC_CONTROL_LOOP_MIN_GW;
    c.dac_cotrig          = v >= DAC_COTRIG_MIN_GW;
    c.dac_loop_source     = v >= DAC_LOOP_SOURCE_MIN_GW;
    c.dac_loop_inmap      = v >= DAC_LOOP_INMAP_MIN_GW;
    c.exact_cap_divider   = v >= EXACT_CAP_DIVIDER_MIN_GW;
    c.gpio_get            = v >= GPIO_GET_MIN_GW;
    c.capture_trigger     = v >= CAPTURE_TRIGGER_MIN_GW;
    c.capture_opcode_only = v >= CAPTURE_OPCODE_ONLY_MIN_GW;
    c.reload_wire         = v >= RELOAD_WIRE_MIN_GW;
    c.spi_master          = v >= SPI_MASTER_MIN_GW;
    c.swd_queue           = v >= SWD_QUEUE_MIN_GW;
    c.dac_ctrl            = v >= DAC_CTRL_MIN_GW;
    c.la_change           = v >= LA_CHANGE_MIN_GW;
    c.spi_fast_link       = v >= SPI_FAST_LINK_MIN_GW;
    c.dual_la_full_rate   = v >= DUAL_LA_FULL_RATE_MIN_GW;
    c.emulation           = v >= EMULATION_MIN_GW;
    c.spi_modes12         = v >= SPI_MODES12_MIN_GW;
    c.i2c_master          = v >= I2C_MASTER_MIN_GW;
    c.uart_frame          = v >= UART_FRAME_MIN_GW;
    c.uart_half_duplex    = v >= UART_HALFDUPLEX_MIN_GW;
    c.uart_de             = v >= UART_DE_MIN_GW;
    c.uart_config2        = v >= UART_CONFIG2_MIN_GW;
    c.img_control_loop    = (f & FPGA_FEATURE_CONTROL_LOOP) != 0;
    c.img_deep_replay     = (f & FPGA_FEATURE_DEEP_REPLAY) != 0;
    c.img_emulation       = (f & FPGA_FEATURE_EMULATION) != 0;
    return c;
}

static gw_caps_t s_caps;
static bool      s_filled;
static uint8_t   s_ver, s_feat;   /* what s_caps was filled from */

const gw_caps_t *gw_caps(void)
{
    const uint8_t v = signal_engine_fpga_version();
    const uint8_t f = signal_engine_fpga_features_cached();
    if (!s_filled || v != s_ver || f != s_feat) {
        s_caps   = gw_caps_for(v, f);
        s_ver    = v;
        s_feat   = f;
        s_filled = true;
    }
    return &s_caps;
}
