#ifndef DAC_LIMITS_H
#define DAC_LIMITS_H

#include <stdbool.h>
#include <stdint.h>

/* ---- DAC output limits for an external output stage ---------------------------------------
 *
 * Some benches put a power module between a DAC output and the DUT, e.g. a solar simulator whose
 * input is INVERTED: 0 V on the DAC is its FULL output (~45 V), and the DAC must stay inside a
 * window (e.g. 1.85..3.60 V) to keep it below a chosen voltage. These limits are stored on the
 * pod and checked on every command that can move the DAC or its routing, whichever way it
 * arrives (LAN, cloud tunnel, cloud command, USB console). The server and web page check the
 * same limits; this is the layer nothing can go around.
 *
 * While limits are set:
 *   - dac_out on the limited path must stay inside [min_mv, max_mv].
 *   - dac_control_loop must clamp inside the window (vmin/vmax as 16-bit codes, converted with
 *     the path's calibration), and an inverted stage may not arm in_trip (the gateware trips
 *     TO vmin, which is the HIGHEST output there).
 *   - For an inverted stage, a route that disconnects the limited path is refused (off, another
 *     DAC path, cal2, cal1 unless the path is 5v): a disconnected input sits near 0 V.
 *   - Commands that write raw DAC codes or flip the mux by hand are refused: generate, dac_set,
 *     load, load_bin, replay, measure, dac_mux, cal_switch (and the console's dacraw/dacmux).
 *   - current_out with a current is refused: the 4-20 mA output has no DAC of its own, so setting
 *     a current moves the limited path. Reading its range (no `ua`) is allowed. The route
 *     current_out switches the DAC outputs off, so it follows the rule for `off` above.
 *   - dac_stop parks the DAC at the low-output end (max_mv when inverted, min_mv otherwise) with
 *     the path routed, and so does boot, as soon as the analog front end and iCE40 are up.
 *
 * What firmware can't cover: the time from reset until boot parks the DAC, when the DAC reads
 * 0 V. Hold the module off with its own enable pin for that.
 *
 * Stored in its own power-loss-safe A/B store (config_store.h), sectors s118/s119
 * (0x1EC000 / 0x1EE000), carved from the top of FLASH_BLOBS.
 * ------------------------------------------------------------------------------------------*/

#define DAC_LIMITS_SLOT_A_OFFSET  0x1EE000u
#define DAC_LIMITS_SLOT_B_OFFSET  0x1EC000u
#define DAC_LIMITS_RECORD_MAGIC   0x4D494C44u   /* "DLIM" */
#define DAC_LIMITS_VERSION        1u

/* Paths, as the DAC_CAL index (cal_data.h): 0 = 3v3, 1 = 5v, 2 = 12v. */
typedef struct {
    uint8_t enabled;
    uint8_t path;       /* DAC_CAL index */
    uint8_t inverted;   /* 1 = 0 V on the DAC is the module's FULL output */
    uint8_t reserved;
    int32_t min_mv;     /* DAC window on `path`, millivolts */
    int32_t max_mv;
} dac_limits_t;

/* The active limits (all zero = none). Loaded by dac_limits_load(). */
const dac_limits_t *dac_limits_get(void);

/* Load from flash into the active copy. 0 = limits loaded, -1 = none stored. */
int dac_limits_load(void);

/* Validate, store and activate. NULL = done, else why it was refused. */
const char *dac_limits_set(const dac_limits_t *in);

/* Remove the limits (flash and active copy). 0 on success. */
int dac_limits_clear(void);

/* Replace the active copy without touching flash (host tests). */
void dac_limits_set_active(const dac_limits_t *in);

/* Path name for a DAC_CAL index ("3v3" / "5v" / "12v"), and back (-1 = not a DAC path). */
const char *dac_limits_path_name(int path);
int dac_limits_path_index(const char *name);

/* The DAC voltage Stop and boot hold (low-output end of the window), in volts. */
float dac_limits_park_volts(void);

/* The 8-bit dac_set_constant code for the park voltage, rounded INTO the window. */
uint8_t dac_limits_park_code(void);

/* Check one JSON command (cmd = its "cmd" field). NULL = allowed, else the refusal. Pure. */
const char *dac_limits_check_command(const char *cmd, const char *json);

/* The same checks for the USB console's words: a raw DAC/mux command, a route by name, or a held
   voltage on a named path. NULL = allowed. */
const char *dac_limits_check_raw(const char *what);
const char *dac_limits_check_route(const char *path);
const char *dac_limits_check_volts(const char *path, float volts);

/* Setting a current on the 4-20 mA output (current_out.h). It shares the DAC with the limited
   path, so it is refused while limits are set. NULL = allowed. */
const char *dac_limits_check_current_out(void);

/* Hardware (dac_limits_hw.c, not in the host tests): hold the DAC at the park level and route the
   limited path. 0 = parked, 1 = no limits set, -1 = failed. Needs the iCE40 up. */
int dac_limits_park_now(void);

#endif /* DAC_LIMITS_H */
