#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================================================================
 * I2C0 power/IO bus — STM32 I2C1 on PB8 (SCL) / PB9 (SDA), 100 kHz.
 *
 * Devices (7-bit addresses decoded from the netlist):
 *   0x40  INA238  — INTERNAL power-connector current (50 mΩ shunt)   [U29]
 *   0x44  INA238  — EXTERNAL power-connector current (30 mΩ shunt)   [U33]
 *   0x20  TCA9554 — logic-analyzer pull-up control (CTRL1..8)        [U54]
 *   0x22  TCA9554 — eFuse control + INA ALERT (EN/V/FLT)            [U56]
 *   0x24  TCA9554 — DAC v2 analog mux                                [U55]
 *   0x26  TCA9554 — analog/cal switching                             [U58]
 *
 * (The MCP4728 I2C DAC of the RP2350B board is gone — DAC/ADC is iCE40-only.)
 * ========================================================================== */

#define I2C_ADDR_INA238_INTERNAL  0x40u
#define I2C_ADDR_INA238_EXTERNAL  0x44u
#define I2C_ADDR_TCA9554_LA       0x20u   /* LA pull-up control      */
#define I2C_ADDR_TCA9554_PWR      0x22u   /* eFuse enable/status     */
#define I2C_ADDR_TCA9554_DACMUX   0x24u
#define I2C_ADDR_TCA9554_ANASW    0x26u

/* Initialise (or re-initialise) I2C1.  Safe to call repeatedly.
   Returns 0 on success, <0 on HAL init failure. */
int  i2c_bus_init(void);
bool i2c_bus_ready(void);

/* Print bus state + a 0x08..0x77 scan, naming the known devices. */
void i2c_bus_status(void);

/* ---- Low-level transfers (return 0 on success, <0 on NACK/timeout) ---- */
int  i2c_bus_write(uint8_t addr, const uint8_t *buf, size_t len);
int  i2c_bus_read(uint8_t addr, uint8_t *buf, size_t len);
int  i2c_bus_write_read(uint8_t addr, const uint8_t *wbuf, size_t wlen,
                        uint8_t *rbuf, size_t rlen);
bool i2c_bus_probe(uint8_t addr);   /* true if the address ACKs */

/* ---- LA pull-up control (LA1..8) via TCA9554 @ 0x20 (CTRL1..8) ----------
   Compatibility names for the ported command_handler.c (PCA9555 on RP2350).

   HARDWARE LIMIT (v2.0.0): the LA1-8 pull-up resistors — and the board pull-downs
   that hold the analog switches open — are referenced to +3V3, NOT to the LA bank
   rail.  With the bank at 1.8 V a closed pull-up drives the DUT line ~1.5 V above
   its own rail, back-feeding the DUT and the iCE40's 1.8 V bank.  So the pull-ups
   are only ever engaged while the bank is at 3.3 V: enabling one at 1.8 V is
   REFUSED (LA_PULLUP_ERR_VOLTAGE) and la_vccio_set_mv(1800) drops any that are
   already on (la_pullups_all_off).  Disabling is always allowed. */

/* pca9555_set_la_pullup: 0 = ok, LA_PULLUP_ERR_VOLTAGE = refused because the LA
   bank is not at 3.3 V, -1 = bad pin or I2C failure. */
#define LA_PULLUP_ERR_VOLTAGE  (-2)
int  pca9555_set_la_pullup(uint8_t la_pin, bool enable);   /* la_pin 1..8 */
bool pca9555_la_pullup_enabled(uint8_t la_pin);
/* Fixed resistance on LA<n>, e.g. "4.7k"; "" for a channel with no network.
   The value alone does not say which way it pulls — ask pca9555_la_pull_is_down. */
const char *pca9555_la_pullup_ohms(uint8_t la_pin);
/* True for the channels whose resistor pulls DOWN rather than up (LA7/LA8).
   Everything here is named "pullup" for historical reasons; LA1-LA6 pull up
   with 4.7k (LA1/2), 2.2k (LA3/4) and 10k (LA5/6), while LA7/LA8 pull DOWN
   with 10k.  Enabling any of them closes the same kind of analog switch. */
bool pca9555_la_pull_is_down(uint8_t la_pin);
/* True when the LA bank voltage currently allows a pull-up to be engaged (3.3 V). */
bool la_pullups_available(void);
/* Force every LA pull-up off (used when the bank drops to 1.8 V). */
void la_pullups_all_off(void);

/* ---- Analog path switching (v2) -----------------------------------------
 * Two TCA9554 expanders route the DAC and switch the ADC input:
 *
 *   U55 @ 0x24 (DACMUX): drives two TMUX1104 4:1 muxes.
 *     CTRL1 (P0=EN, P1=A0, P2=A1): U47.D=BUFFER_OUT -> sel 0..3 =
 *          S1 TO_3V3 / S2 TO_5V / S3 TO_12V / S4 TO_12V_ADC.
 *     CTRL2 (P3=EN, P4=A0, P5=A1): U48.D=BUFFER_VMID -> sel 0..2 =
 *          S1 TO_12V_VMID / S2 TO_ADC_VMID / S3 GND.
 *
 *   U58 @ 0x26 (ANASW): drives 4 relays via a TPL7407 low-side driver
 *     (bit high = relay energised):
 *     P0 CAL1 (5V DAC path -> ADC), P1 CAL2 (12V DAC path -> ADC),
 *     P2 AMP_MEASURE (ADC <- amps screw terminal), P3 CAL_PATH (ADC <- cal path).
 *     CAL1 and CAL2 must never be enabled together.
 *
 * Sel encodings (match the TMUX1104 A1:A0 truth table). */
#define DACMUX_CTRL1_3V3      0u   /* S1 */
#define DACMUX_CTRL1_5V       1u   /* S2 */
#define DACMUX_CTRL1_12V      2u   /* S3 */
#define DACMUX_CTRL1_12V_ADC  3u   /* S4 */
#define DACMUX_CTRL2_12V_VMID 0u   /* S1 */
#define DACMUX_CTRL2_ADC_VMID 1u   /* S2 */
#define DACMUX_CTRL2_GND      2u   /* S3 */

/* Configure U55/U58 control pins as outputs, all driven low (everything off:
   no relay energised, no DAC routed to an SMA). Called at boot. */
void analog_switch_init(void);

/* DAC-path mux (U55). ``sel`` 0..3 (see DACMUX_* above). Returns 0 / <0 on I2C err. */
int  dacmux_set_ctrl1(bool en, uint8_t sel);
int  dacmux_set_ctrl2(bool en, uint8_t sel);
int  dacmux_read(uint8_t *out_reg);           /* current U55 output register */

/* Calibration switching (U58). Rejects cal1 && cal2 (returns -2). */
int  calsw_set(bool cal1, bool cal2, bool amp_measure, bool cal_path);
int  calsw_read(uint8_t *out_reg);            /* current U58 output register */
void analog_switch_dump(void);                /* diagnostic: dump U55/U58 registers */

/* ---- High-level analog paths (SINGLE SOURCE OF TRUTH) --------------------
 * One named path == one fully-specified switch state (U55 mux + U58 relays),
 * so the console, JSON API, and SDK never hand-flip switches or drift apart.
 * Routing is verified against the vbench-pod netlist + bench measurements:
 *
 *   path       U55 ctrl1      U55 ctrl2       U58 relays (cal1,cal2,amp,path)  ADC sees
 *   off        off            off             0,0,0,0                          -
 *   dac_3v3    en,3V3         off             0,0,0,0                          -
 *   dac_5v     en,5V          off             0,0,0,0                          -
 *   dac_12v    en,12V         en,12V_VMID     0,0,0,0                          -   (±12V diff)
 *   adc_ext    (unchanged)    (unchanged)     0,0,0,0                          RF4 SMA (÷12, ~1MΩ)
 *   cal1       en,5V          off             1,0,0,1                          5V DAC via K1
 *   cal2       en,12V_ADC     en,ADC_VMID     0,1,0,1                          ±12V diff via K2/U51
 *   amp        (unchanged)    (unchanged)     0,0,1,0                          J8 screw terminal
 *
 * The DAC output paths also open all ADC relays (ADC returns to the external
 * SMA), so `dac_12v` + read ADC is a self-contained loopback of the front end.
 * adc_ext / amp only move relays, leaving any DAC output running. */
typedef enum {
    ANALOG_PATH_OFF = 0,
    ANALOG_PATH_DAC_3V3,
    ANALOG_PATH_DAC_5V,
    ANALOG_PATH_DAC_12V,
    ANALOG_PATH_ADC_EXT,
    ANALOG_PATH_CAL1,
    ANALOG_PATH_CAL2,
    ANALOG_PATH_AMP,
    ANALOG_PATH__COUNT
} analog_path_t;

/* Apply a path (sets all needed switches atomically). Returns 0 / <0 on I2C err. */
int          analog_path_set(analog_path_t path);
/* Canonical name for a path (e.g. "dac_5v"); "" if out of range. */
const char  *analog_path_name(analog_path_t path);
/* Parse a user name/alias into a path. Accepts canonical names plus aliases
   3v3/5v/12v (= dac_*), ext/sma (= adc_ext). Returns 0 / <0 if unknown. */
int          analog_path_from_name(const char *name, analog_path_t *out);

#endif /* I2C_BUS_H */
