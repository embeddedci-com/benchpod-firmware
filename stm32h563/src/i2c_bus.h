#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================================================================
 * I2C0 power/IO bus — STM32 I2C1 on PB8 (SCL) / PB9 (SDA), 100 kHz.
 *
 * Devices (7-bit addresses decoded from the netlist):
 *   0x40  INA238  — INTERNAL power-connector current (50 mΩ shunt, 30 mΩ on INA226 boards) [U29]
 *   0x44  INA238  — EXTERNAL power-connector current (30 mΩ shunt, 20 mΩ on INA226 boards) [U33]
 *   0x41  INA226  — the pod's own 5 V current (30 mΩ shunt after the input eFuse, the DUT's
 *                   internal rail bypasses it); only on boards that have it          [U35]
 *   0x20  TCA9554 — logic-analyzer pull-up control (CTRL1..8)        [U54]
 *   0x22  TCA9554 — eFuse control + INA ALERT (EN/V/FLT)            [U56]
 *   0x24  TCA9554 — DAC v2 analog mux                                [U55]
 *   0x26  TCA9554 — analog/cal switching                             [U58]
 *
 * (The MCP4728 I2C DAC of the RP2350B board is gone — DAC/ADC is iCE40-only.)
 *
 * Module bus — STM32 I2C2 on PF1 (SCL) / PF0 (SDA), 100 kHz, digital v3r3 only (4.7k pull-ups
 * on the digital board, the plug-in analog module on the far end of the FFC, module_link.h):
 *   0x26  TCA9554 — analog/cal switching, as U58 above                [module U58]
 *   0x50  24C32   — module ID + calibration EEPROM, WP tied low       [module U61]
 * There is no U55 on the module.  The one-piece analog boards route PF0/PF1 to a header (H2)
 * with pull-ups and nothing else, so the bus is only brought up on digital v3r3.
 * ========================================================================== */

#define I2C_ADDR_INA238_INTERNAL  0x40u
#define I2C_ADDR_INA238_EXTERNAL  0x44u
#define I2C_ADDR_INA_POD          0x41u   /* pod's own current, not on every board */
#define I2C_ADDR_TCA9554_LA       0x20u   /* LA pull-up control      */
#define I2C_ADDR_TCA9554_PWR      0x22u   /* eFuse enable/status     */
#define I2C_ADDR_TCA9554_DACMUX   0x24u
#define I2C_ADDR_TCA9554_ANASW    0x26u
#define I2C_ADDR_MOD_EEPROM       0x50u   /* module bus only */

typedef enum {
    I2C_BUS_MAIN = 0,      /* I2C1, PB8/PB9: every board */
    I2C_BUS_MODULE,        /* I2C2, PF0/PF1: digital v3r3's module link */
    I2C_BUS__COUNT
} i2c_bus_id_t;

/* Initialise (or re-initialise) I2C1.  Safe to call repeatedly.  Runs a bus
   clear (see below) before the peripheral is set up.
   Returns 0 on success, <0 on HAL init failure. */
int  i2c_bus_init(void);
bool i2c_bus_ready(void);

/* Bus clear + peripheral re-init, from the task that owns the bus (the hw
   worker).  Bounded: well under 1 ms.  Returns the i2c_busclear_run() result
   (>=0 = SCL pulses it took, <0 = the bus is still stuck).  Transactions call
   this on their own after a BUSY/timeout/bus error or when SDA reads low. */
int  i2c_bus_clear_and_reinit(void);
/* Number of bus clears run since boot (the boot-time one included). */
uint32_t i2c_bus_clear_count(void);

/* ---- Bus clear (NXP UM10204 3.1.16) -------------------------------------
 * An MCU reset in the middle of a read (IWDG, crash, DFU, OTA reboot) with the
 * pod still powered can leave a slave half way through a byte, driving SDA low
 * and waiting for clocks that never come.  The I2C peripheral cannot start a
 * transfer on that bus, and the eFuse/power expander sits on it, so it stays
 * dead until a power cycle.  The cure: drive SCL by hand until the slave lets
 * go of SDA, then send a STOP.
 *
 * The sequence is pure logic over these pin callbacks so the host tests can run
 * it against a simulated slave.  Both lines are open drain: "high" means
 * release (the pull-up takes it high), "low" means drive low. */
#define I2C_BUSCLEAR_MAX_PULSES  16   /* 9 is enough for one byte + ACK */
#define I2C_BUSCLEAR_SCL_WAIT    20   /* half periods to wait for a stretched SCL */

#define I2C_BUSCLEAR_SCL_STUCK  (-1)  /* something holds SCL low */
#define I2C_BUSCLEAR_SDA_STUCK  (-2)  /* SDA still low after the pulse budget */

typedef struct {
    void (*set_scl)(void *ctx, bool high);
    void (*set_sda)(void *ctx, bool high);
    bool (*get_scl)(void *ctx);
    bool (*get_sda)(void *ctx);
    void (*delay)(void *ctx);        /* half an SCL period (~5 us = 100 kHz) */
    void *ctx;
} i2c_busclear_io_t;

/* Release SCL and wait (bounded) for it to read high: a slave may stretch. */
static inline bool i2c_busclear_scl_high(const i2c_busclear_io_t *io)
{
    io->set_scl(io->ctx, true);
    for (int i = 0; i < I2C_BUSCLEAR_SCL_WAIT; i++) {
        io->delay(io->ctx);
        if (io->get_scl(io->ctx)) return true;
    }
    return false;
}

/* Run the bus clear.  SDA only ever changes while SCL is low, except for the
   final STOP (SDA rising while SCL is high), so it never makes a START.
   Always ends with a STOP, even on a bus that looked idle: a slave that was
   mid-write holds SDA high but is still waiting for bits, and the STOP resets
   it.  Returns the number of SCL pulses clocked (>=0), or
   I2C_BUSCLEAR_SCL_STUCK / I2C_BUSCLEAR_SDA_STUCK.  Leaves both lines released. */
static inline int i2c_busclear_run(const i2c_busclear_io_t *io)
{
    int pulses = 0;

    io->set_sda(io->ctx, true);
    if (!i2c_busclear_scl_high(io)) return I2C_BUSCLEAR_SCL_STUCK;

    for (;;) {
        /* Clock until the slave releases SDA (it reads SDA after SCL rises). */
        while (!io->get_sda(io->ctx)) {
            if (pulses >= I2C_BUSCLEAR_MAX_PULSES) return I2C_BUSCLEAR_SDA_STUCK;
            io->set_scl(io->ctx, false);
            io->delay(io->ctx);
            if (!i2c_busclear_scl_high(io)) return I2C_BUSCLEAR_SCL_STUCK;
            pulses++;
        }

        /* STOP: SCL low, SDA low, SCL high, then SDA high while SCL is high.
           The SCL rise here is one more clock for the slave, so the total is
           at most MAX_PULSES + 1; the check below keeps a flapping SDA from
           looping forever. */
        if (pulses > I2C_BUSCLEAR_MAX_PULSES) return I2C_BUSCLEAR_SDA_STUCK;
        io->set_scl(io->ctx, false);
        io->delay(io->ctx);
        io->set_sda(io->ctx, false);
        io->delay(io->ctx);
        if (!i2c_busclear_scl_high(io)) {
            io->set_sda(io->ctx, true);
            return I2C_BUSCLEAR_SCL_STUCK;
        }
        pulses++;
        io->set_sda(io->ctx, true);
        io->delay(io->ctx);
        if (io->get_sda(io->ctx)) return pulses;
        /* A slave in a read drove its next data bit (a 0) on that falling
           edge, so SDA did not rise and there was no STOP.  Keep clocking. */
    }
}

/* Print bus state + a 0x08..0x77 scan, naming the known devices (the module bus too, when up). */
void i2c_bus_status(void);

/* ---- Low-level transfers (return 0 on success, <0 on NACK/timeout) ---- */
int  i2c_bus_write(uint8_t addr, const uint8_t *buf, size_t len);
int  i2c_bus_read(uint8_t addr, uint8_t *buf, size_t len);
int  i2c_bus_write_read(uint8_t addr, const uint8_t *wbuf, size_t wlen,
                        uint8_t *rbuf, size_t rlen);
bool i2c_bus_probe(uint8_t addr);   /* true if the address ACKs */

/* The same on a given bus.  i2c_bus_init_on(I2C_BUS_MODULE) brings up I2C2; until then every
   transfer on it fails (-1 / false) without touching the pins.  write_read takes a one-byte
   register pointer or a two-byte memory address (24C32) as one repeated-start transfer. */
int  i2c_bus_init_on(i2c_bus_id_t bus);
bool i2c_bus_ready_on(i2c_bus_id_t bus);
int  i2c_bus_write_on(i2c_bus_id_t bus, uint8_t addr, const uint8_t *buf, size_t len);
int  i2c_bus_read_on(i2c_bus_id_t bus, uint8_t addr, uint8_t *buf, size_t len);
int  i2c_bus_write_read_on(i2c_bus_id_t bus, uint8_t addr, const uint8_t *wbuf, size_t wlen,
                           uint8_t *rbuf, size_t rlen);
bool i2c_bus_probe_on(i2c_bus_id_t bus, uint8_t addr);

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
 *     P2 current_in (ADC <- 4-20 mA terminal J8; net AMP_MEASURE in the schematic), P3 CAL_PATH (ADC <- cal path).
 *     CAL1 and CAL2 must never be enabled together.
 *
 * Sel encodings (match the TMUX1104 A1:A0 truth table). */
#define DACMUX_CTRL1_3V3      0u   /* S1 */
#define DACMUX_CTRL1_5V       1u   /* S2 */
#define DACMUX_CTRL1_12V      2u   /* S3 */
#define DACMUX_CTRL1_12V_ADC  3u   /* S4 */
#define DACMUX_CTRL2_12V_VMID 0u   /* S1 */
#define DACMUX_CTRL2_ADC_VMID 1u   /* S2 */
#define DACMUX_CTRL2_S3       2u   /* S3: not connected on the board (no ground here) */

/* The bus U55/U58 are on: I2C_BUS_MAIN on the one-piece analog boards, I2C_BUS_MODULE for the
   analog module of digital v3r3.  Set at boot before analog_switch_init(). */
void         analog_switch_set_bus(i2c_bus_id_t bus);
i2c_bus_id_t analog_switch_bus(void);

/* Configure U55/U58 control pins as outputs, all driven low (everything off:
   no relay energised, no DAC routed to an SMA). Called at boot. */
void analog_switch_init(void);

/* DAC-path mux (U55). ``sel`` 0..3 (see DACMUX_* above). Returns 0 / <0 on I2C err. */
int  dacmux_set_ctrl1(bool en, uint8_t sel);
int  dacmux_set_ctrl2(bool en, uint8_t sel);
int  dacmux_read(uint8_t *out_reg);           /* current U55 output register */

/* U58's output register for a relay state: P0 cal1, P1 cal2, P2 current_in, P3 cal_path, and on
   analog v3r2 P4 = ANA14_EN (the +-14 V supply, TPS65131 U49).  ana14 is false on v2/v3, which
   gives exactly the pre-v3r2 byte.  Pure (analog_path.c). */
uint8_t analog_calsw_reg(bool cal1, bool cal2, bool current_in, bool cal_path, bool ana14);

/* Analog v3r2 only: switch the +-14 V supply of the analog stages (U58 P4; off at power-up, 100k
   pull-down).  -1 on other boards, which never drive P4.  The relays keep their state. */
int  ana14_set(bool on);
bool ana14_is_on(void);

/* Calibration switching (U58). Rejects cal1 && cal2 (returns -2). */
int  calsw_set(bool cal1, bool cal2, bool current_in, bool cal_path);
int  calsw_read(uint8_t *out_reg);            /* current U58 output register */
void analog_switch_dump(void);                /* diagnostic: dump U55/U58 registers */

/* ---- High-level analog paths (SINGLE SOURCE OF TRUTH) --------------------
 * One named path == one fully-specified switch state (U55 mux + U58 relays),
 * so the console, JSON API, and SDK never hand-flip switches or drift apart.
 * Routing is verified against the vbench-pod netlist + bench measurements:
 *
 *   path       U55 ctrl1      U55 ctrl2       U58 relays (cal1,cal2,cur,path)  ADC sees
 *   off        off            off             0,0,0,0                          -
 *   dac_3v3    en,3V3         off             0,0,0,0                          -
 *   dac_5v     en,5V          off             0,0,0,0                          -
 *   dac_12v    en,12V         en,12V_VMID     0,0,0,0                          -   (±12V diff)
 *   adc_ext    (unchanged)    (unchanged)     0,0,0,0                          RF4 SMA (÷12, ~1MΩ)
 *   cal1       en,5V          off             1,0,0,1                          5V DAC via K1
 *   cal2       en,12V_ADC     en,ADC_VMID     0,1,0,1                          ±12V diff via K2/U51
 *   current_in (unchanged)    (unchanged)     0,0,1,0                          J8 screw terminal
 *   current_out off           off             (unchanged)                      (unchanged)
 *
 * The DAC output paths also open all ADC relays (ADC returns to the external
 * SMA), so `dac_12v` + read ADC is a self-contained loopback of the front end.
 * adc_ext / current_in only move relays, leaving any DAC output running.
 * current_out is the 4-20 mA output (J9): the XTR116 follows the DAC buffer on every
 * path, so the path only switches the voltage outputs off (current_out.h).
 *
 * Analog v3r2 (dac_quad.h) has no U55: dacmux_* return -1 there.  The relays are as above, and
 * the DAC8554 channel replaces the mux: dac_3v3 = A, dac_5v and cal1 = B, dac_12v and cal2 = C
 * (channel C drives the +-12 V SMA and the cal2 stage together), current_out = D.  The voltage
 * channel the DAC leaves is parked at 0 V (dac_quad_park_code), and current_out leaves the
 * voltage outputs alone. */
typedef enum {
    ANALOG_PATH_OFF = 0,
    ANALOG_PATH_DAC_3V3,
    ANALOG_PATH_DAC_5V,
    ANALOG_PATH_DAC_12V,
    ANALOG_PATH_ADC_EXT,
    ANALOG_PATH_CAL1,
    ANALOG_PATH_CAL2,
    ANALOG_PATH_CURRENT_IN,
    ANALOG_PATH_CURRENT_OUT,
    ANALOG_PATH__COUNT
} analog_path_t;

/* One path's switch state (analog_path.c, pure): write U55 (`mux`) with ctrl1/ctrl2, then U58
   (`relays`).  quad = analog v3r2, which has no U55, so mux is false there.  0 / -1 bad path. */
typedef struct {
    bool    mux;
    bool    ctrl1_en;  uint8_t ctrl1_sel;
    bool    ctrl2_en;  uint8_t ctrl2_sel;
    bool    relays;
    bool    cal1, cal2, current_in, cal_path;
} analog_switches_t;
int analog_path_switches(analog_path_t path, bool quad, analog_switches_t *out);

/* Apply a path (sets all needed switches atomically). Returns 0 / <0 on I2C err. */
int          analog_path_set(analog_path_t path);
/* Canonical name for a path (e.g. "dac_5v"); "" if out of range. */
const char  *analog_path_name(analog_path_t path);
/* Parse a user name/alias into a path. Accepts canonical names plus aliases
   3v3/5v/12v (= dac_*), ext/sma (= adc_ext). Returns 0 / <0 if unknown. */
int          analog_path_from_name(const char *name, analog_path_t *out);

#endif /* I2C_BUS_H */
