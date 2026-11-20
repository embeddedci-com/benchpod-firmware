#ifndef GW_CAPS_H
#define GW_CAPS_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * gw_caps: what the running iCE40 gateware can do, from its version and FPGA_FEATURES byte.
 *
 * Every gateware gate reads a field here instead of comparing signal_engine_fpga_version()
 * against a *_MIN_GW constant at the call site, so a gate is named once (gw_caps_for) and a new
 * gateware feature is one field.  The *_MIN_GW constants stay where they are (signal_engine.h,
 * la_rate.h): the error texts print them, and the pure rate maths in la_rate.c takes the
 * version number.
 *
 * gw_caps() is filled from the cached version and FPGA_FEATURES reading, and filled again on the
 * first call after either changes: boot, an image swap or `flash-ice40`
 * (command_handler_on_gateware_reconfigured), a `status` that re-reads the version.  Keyed on the
 * two bytes rather than on those paths, so no path that sets the version can leave it stale.
 * Otherwise it is two compares and a pointer.
 *
 * The image bits are the LAST FPGA_FEATURES reading (signal_engine_fpga_features_cached()).
 * Code that must know the image that runs right now (signal_engine_has_control_loop(),
 * signal_engine_image_kind()) still reads FPGA_FEATURES live.
 * ==========================================================================*/

typedef struct {
    /* ---- by gateware version ---- */
    bool dac_deep_replay;      /* DAC_DEEP_REPLAY_MIN_GW     v17 */
    bool concurrent_psram;     /* CONCURRENT_PSRAM_MIN_GW    v18 */
    bool dac_capture_stop;     /* DAC_CAPTURE_STOP_MIN_GW    v21 */
    bool capture_bases;        /* CAPTURE_BASES_MIN_GW       v22 (and the FPGA_FEATURES opcode) */
    bool dac_control_loop;     /* DAC_CONTROL_LOOP_MIN_GW    v23 */
    bool dac_cotrig;           /* DAC_COTRIG_MIN_GW          v27 */
    bool dac_loop_source;      /* DAC_LOOP_SOURCE_MIN_GW     v29 */
    bool dac_loop_inmap;       /* DAC_LOOP_INMAP_MIN_GW      v30 */
    bool exact_cap_divider;    /* EXACT_CAP_DIVIDER_MIN_GW   v32 */
    bool gpio_get;             /* GPIO_GET_MIN_GW            v35 */
    bool capture_trigger;      /* CAPTURE_TRIGGER_MIN_GW     v35 */
    bool capture_opcode_only;  /* CAPTURE_OPCODE_ONLY_MIN_GW v40 */
    bool reload_wire;          /* RELOAD_WIRE_MIN_GW         v40 */
    bool spi_master;           /* SPI_MASTER_MIN_GW          v45 */
    bool swd_queue;            /* SWD_QUEUE_MIN_GW           v45 */
    bool dac_ctrl;             /* DAC_CTRL_MIN_GW            v49 */
    bool la_change;            /* LA_CHANGE_MIN_GW           v50 */
    bool spi_fast_link;        /* SPI_FAST_LINK_MIN_GW       v52 */
    bool dual_la_full_rate;    /* DUAL_LA_FULL_RATE_MIN_GW   v53 */
    bool emulation;            /* EMULATION_MIN_GW           v54 (the image exists, not that it runs) */
    bool spi_modes12;          /* SPI_MODES12_MIN_GW         v55 */
    bool i2c_master;           /* I2C_MASTER_MIN_GW          v55 */
    bool uart_frame;           /* UART_FRAME_MIN_GW          v56 */
    bool uart_half_duplex;     /* UART_HALFDUPLEX_MIN_GW     v57 */
    bool uart_de;              /* UART_DE_MIN_GW             v58 */
    bool uart_config2;         /* UART_CONFIG2_MIN_GW        v60 */
    /* ---- the running image (FPGA_FEATURES bits, gateware v22+) ---- */
    bool img_control_loop;     /* FPGA_FEATURE_CONTROL_LOOP */
    bool img_deep_replay;      /* FPGA_FEATURE_DEEP_REPLAY */
    bool img_emulation;        /* FPGA_FEATURE_EMULATION */
} gw_caps_t;

/* The capabilities of a gateware version and FPGA_FEATURES byte.  Pure (host tests). */
gw_caps_t gw_caps_for(uint8_t version, uint8_t features);

/* The running gateware's capabilities.  Never NULL. */
const gw_caps_t *gw_caps(void);

#endif /* GW_CAPS_H */
