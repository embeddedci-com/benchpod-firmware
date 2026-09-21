/*
 * i2c_bus.c — STM32 I2C1 master for the power/IO bus (PB8 SCL / PB9 SDA).
 *
 * The I2C1 kernel clock is sourced from HSI (64 MHz) so the TIMINGR value below
 * is independent of the PLL/SYSCLK configuration.  TIMINGR = 0xF0420F17 gives a
 * ~100 kHz SCL (PRESC=15 -> 250 ns tick; SCLL=24, SCLH=16 ticks; SDADEL=2,
 * SCLDEL=5) — conservative for bring-up; can be raised to 400 kHz once proven.
 */
#include "i2c_bus.h"
#include "tca9554.h"
#include "stm32h5xx_hal.h"
#include "board_pins.h"
#include "pico_compat.h"
#include "signal_engine.h"   /* la_vccio_get_mv — the pull-ups are 3V3-referenced */

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

#define I2C_TIMING_100K   0xF0420F17u
#define I2C_TIMEOUT_MS    20u

static I2C_HandleTypeDef hi2c;
static bool s_ready = false;

/* ---- transaction atomicity (fix for the I2C-on-the-worker-task regression) ---
 * All I2C now runs on the hw worker task, which the higher-priority net task can
 * preempt.  A cloud TLS handshake (mbedTLS bignum) hogs the net task for tens of
 * ms; if that lands mid-I2C-transaction on the worker, the blocking HAL_I2C poll
 * (20 ms wall-clock timeout) trips and leaves the peripheral half-done/BUSY-stuck
 * — which cascaded into "all I2C dead" until a lucky recovery.  Suspending the
 * scheduler around each transaction makes it atomic w.r.t. task switching (a few
 * hundred µs — SysTick still runs, so HAL_GetTick timeouts and a genuinely stuck
 * bus still bail out), so it can never be preempted mid-transfer. */
static inline void i2c_enter(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) vTaskSuspendAll();
}
static inline void i2c_leave(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED) xTaskResumeAll();
}

/* Clear a wedged peripheral (stuck BUSY / half-finished transfer) with a PE
   disable→enable toggle — keeps the pin/timing config, resets the state machine.
   Belt-and-suspenders: with i2c_enter() preventing mid-transfer preemption a wedge
   should no longer form, but this self-heals any that slips through. */
static void i2c_recover(void) {
    __HAL_I2C_DISABLE(&hi2c);
    for (volatile int i = 0; i < 1000; i++) { __NOP(); }
    __HAL_I2C_ENABLE(&hi2c);
}

int i2c_bus_init(void)
{
    GPIO_InitTypeDef gp = {0};
    RCC_PeriphCLKInitTypeDef pclk = {0};

    s_ready = false;

    /* I2C1 kernel clock = HSI (64 MHz). */
    pclk.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    pclk.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&pclk) != HAL_OK) {
        printf("[i2c] ERROR: kernel clock config failed\n");
        return -1;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* PB8/PB9 = I2C1 SCL/SDA, AF4, open-drain (board has external pull-ups). */
    gp.Pin = PWR_I2C_SCL_PIN | PWR_I2C_SDA_PIN;
    gp.Mode = GPIO_MODE_AF_OD;
    gp.Pull = GPIO_NOPULL;
    gp.Speed = GPIO_SPEED_FREQ_LOW;
    gp.Alternate = PWR_I2C_AF;
    HAL_GPIO_Init(GPIOB, &gp);

    hi2c.Instance = PWR_I2C;
    hi2c.Init.Timing = I2C_TIMING_100K;
    hi2c.Init.OwnAddress1 = 0;
    hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c.Init.OwnAddress2 = 0;
    hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c) != HAL_OK) {
        printf("[i2c] ERROR: I2C1 init failed\n");
        return -1;
    }
    HAL_I2CEx_ConfigAnalogFilter(&hi2c, I2C_ANALOGFILTER_ENABLE);

    s_ready = true;
    printf("[i2c] I2C1 ~100 kHz  SCL=PB8 SDA=PB9\n");
    return 0;
}

bool i2c_bus_ready(void) { return s_ready; }

int i2c_bus_write(uint8_t addr, const uint8_t *buf, size_t len)
{
    i2c_enter();
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c, (uint16_t)(addr << 1),
                                                   (uint8_t *)buf, (uint16_t)len,
                                                   I2C_TIMEOUT_MS);
    if (st != HAL_OK) i2c_recover();
    i2c_leave();
    return (st == HAL_OK) ? 0 : -1;
}

int i2c_bus_read(uint8_t addr, uint8_t *buf, size_t len)
{
    i2c_enter();
    HAL_StatusTypeDef st = HAL_I2C_Master_Receive(&hi2c, (uint16_t)(addr << 1),
                                                  buf, (uint16_t)len, I2C_TIMEOUT_MS);
    if (st != HAL_OK) i2c_recover();
    i2c_leave();
    return (st == HAL_OK) ? 0 : -1;
}

int i2c_bus_write_read(uint8_t addr, const uint8_t *wbuf, size_t wlen,
                       uint8_t *rbuf, size_t rlen)
{
    /* All bus devices use a single register-pointer byte, so a repeated-start
       Mem_Read covers them; fall back to discrete transfers otherwise. */
    if (wlen == 1) {
        i2c_enter();
        HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c, (uint16_t)(addr << 1),
                                                wbuf[0], I2C_MEMADD_SIZE_8BIT,
                                                rbuf, (uint16_t)rlen, I2C_TIMEOUT_MS);
        if (st != HAL_OK) i2c_recover();
        i2c_leave();
        return (st == HAL_OK) ? 0 : -1;
    }
    if (i2c_bus_write(addr, wbuf, wlen) != 0) return -1;
    return i2c_bus_read(addr, rbuf, rlen);
}

bool i2c_bus_probe(uint8_t addr)
{
    i2c_enter();
    HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c, (uint16_t)(addr << 1), 2, I2C_TIMEOUT_MS);
    if (st != HAL_OK) i2c_recover();
    i2c_leave();
    return st == HAL_OK;
}

/* ---- LA pull-up control via TCA9554 @ 0x20 ----------------------------
   LA<n> (1..8) -> expander pin (U54: CTRL1=P2, CTRL2=P3, CTRL3=P1, CTRL4=P0,
   CTRL5=P7, CTRL6=P6, CTRL7=P5, CTRL8=P4).  Enabling drives the ctrl line high,
   closing the analog switch that connects the pull-up resistor.

   The resistors hang off +3V3, not off the LA bank rail, so they are usable ONLY
   while the bank is at 3.3 V (see i2c_bus.h).  la_pullups_available() is the one
   place that decides this; everything else asks it. */
static const int8_t s_la_tca_pin[9] = { -1, 2, 3, 1, 0, 7, 6, 5, 4 };
static uint8_t s_la_pu_shadow;

bool la_pullups_available(void)
{
    return la_vccio_get_mv() == LA_VCCIO_3V3;
}

void la_pullups_all_off(void)
{
    for (uint8_t la = 1; la <= 8; la++) {
        if ((s_la_pu_shadow >> (la - 1)) & 1u) (void)pca9555_set_la_pullup(la, false);
    }
}

int pca9555_set_la_pullup(uint8_t la_pin, bool enable)
{
    if (la_pin < 1 || la_pin > 8) return -1;
    /* Turning a pull-up OFF is always safe (and is how we get back to a legal
       state); only closing the switch onto the 3V3 resistor is gated. */
    if (enable && !la_pullups_available()) return LA_PULLUP_ERR_VOLTAGE;
    uint8_t pin = (uint8_t)s_la_tca_pin[la_pin];
    if (tca9554_set_dir(I2C_ADDR_TCA9554_LA, pin, true) != 0) return -1;
    if (tca9554_write_pin(I2C_ADDR_TCA9554_LA, pin, enable) != 0) return -1;
    if (enable) s_la_pu_shadow |= (uint8_t)(1u << (la_pin - 1));
    else        s_la_pu_shadow &= (uint8_t)~(1u << (la_pin - 1));
    return 0;
}

bool pca9555_la_pullup_enabled(uint8_t la_pin)
{
    if (la_pin < 1 || la_pin > 8) return false;
    return (s_la_pu_shadow >> (la_pin - 1)) & 1u;
}

/* Fixed bias resistor per LA channel, indexed by LA number (CTRL1..CTRL8 map
   one-for-one onto LA1..LA8). LA1-LA6 pull UP to +3V3; LA7/LA8 pull DOWN. */
static const char *const s_la_pull_ohms[9] = {
    "", "4.7k", "4.7k", "2.2k", "2.2k", "10k", "10k", "10k", "10k",
};

const char *pca9555_la_pullup_ohms(uint8_t la_pin)
{
    if (la_pin < 1 || la_pin > 8) return "";
    return s_la_pull_ohms[la_pin];
}

bool pca9555_la_pull_is_down(uint8_t la_pin)
{
    return la_pin == 7 || la_pin == 8;
}

/* ---- Analog path switching: U55 @ 0x24 (DAC mux) + U58 @ 0x26 (cal) -------
   Register map (TCA9554): 0x01 output, 0x03 config (1=input, 0=output).
   Shadows track the output registers so a single-bit change is a read-modify-
   write in RAM + one bus write. */
#define TCA_REG_OUTPUT  0x01u
#define TCA_REG_CONFIG  0x03u
#define U55_OUTPUT_MASK 0x3Fu   /* P0..P5 used */
#define U58_OUTPUT_MASK 0x0Fu   /* P0..P3 used */

static uint8_t s_u55_out;       /* DACMUX shadow */
static uint8_t s_u58_out;       /* ANASW  shadow */

void analog_switch_init(void)
{
    /* Drive the output register LOW *before* switching pins to outputs, so the
       TCA9554's 0xFF power-on output default never briefly energises a relay or
       routes the DAC to an unknown SMA. */
    s_u55_out = 0x00; s_u58_out = 0x00;
    tca9554_write_reg(I2C_ADDR_TCA9554_DACMUX, TCA_REG_OUTPUT, 0x00);
    tca9554_write_reg(I2C_ADDR_TCA9554_DACMUX, TCA_REG_CONFIG, (uint8_t)~U55_OUTPUT_MASK);
    tca9554_write_reg(I2C_ADDR_TCA9554_ANASW,  TCA_REG_OUTPUT, 0x00);
    tca9554_write_reg(I2C_ADDR_TCA9554_ANASW,  TCA_REG_CONFIG, (uint8_t)~U58_OUTPUT_MASK);
}

int dacmux_set_ctrl1(bool en, uint8_t sel)
{
    if (sel > 3u) return -1;
    /* P0=EN, P1=A0, P2=A1 */
    s_u55_out = (uint8_t)((s_u55_out & ~0x07u)
                | (en ? 0x01u : 0u)
                | ((sel & 1u) ? 0x02u : 0u)
                | ((sel & 2u) ? 0x04u : 0u));
    return tca9554_write_reg(I2C_ADDR_TCA9554_DACMUX, TCA_REG_OUTPUT, s_u55_out);
}

int dacmux_set_ctrl2(bool en, uint8_t sel)
{
    if (sel > 3u) return -1;
    /* P3=EN, P4=A0, P5=A1 */
    s_u55_out = (uint8_t)((s_u55_out & ~0x38u)
                | (en ? 0x08u : 0u)
                | ((sel & 1u) ? 0x10u : 0u)
                | ((sel & 2u) ? 0x20u : 0u));
    return tca9554_write_reg(I2C_ADDR_TCA9554_DACMUX, TCA_REG_OUTPUT, s_u55_out);
}

int dacmux_read(uint8_t *out_reg)
{
    return tca9554_read_reg(I2C_ADDR_TCA9554_DACMUX, TCA_REG_OUTPUT, out_reg);
}

int calsw_set(bool cal1, bool cal2, bool amp_measure, bool cal_path)
{
    if (cal1 && cal2) return -2;   /* mutually exclusive: never both DAC rails */
    s_u58_out = (uint8_t)((cal1 ? 0x01u : 0u)
                | (cal2 ? 0x02u : 0u)
                | (amp_measure ? 0x04u : 0u)
                | (cal_path ? 0x08u : 0u));
    /* Re-assert the direction (P0..P3 = outputs) in case the power-on config
       write didn't take — otherwise the output write drives nothing and the
       relays never energise. */
    (void)tca9554_write_reg(I2C_ADDR_TCA9554_ANASW, TCA_REG_CONFIG, (uint8_t)~U58_OUTPUT_MASK);
    return tca9554_write_reg(I2C_ADDR_TCA9554_ANASW, TCA_REG_OUTPUT, s_u58_out);
}

int calsw_read(uint8_t *out_reg)
{
    return tca9554_read_reg(I2C_ADDR_TCA9554_ANASW, TCA_REG_OUTPUT, out_reg);
}

/* Diagnostic: dump both expanders' input/output/config registers. */
void analog_switch_dump(void)
{
    static const struct { const char *n; uint8_t a; uint8_t mask; } chips[] = {
        { "U55 dacmux@0x24", I2C_ADDR_TCA9554_DACMUX, U55_OUTPUT_MASK },
        { "U58 calsw @0x26", I2C_ADDR_TCA9554_ANASW,  U58_OUTPUT_MASK },
    };
    for (unsigned i = 0; i < 2; i++) {
        uint8_t in = 0xEE, out = 0xEE, cfg = 0xEE;
        int err = tca9554_read_reg(chips[i].a, 0x00u, &in)
                | tca9554_read_reg(chips[i].a, TCA_REG_OUTPUT, &out)
                | tca9554_read_reg(chips[i].a, TCA_REG_CONFIG, &cfg);
        printf("[exp] %s: in=0x%02x out=0x%02x cfg=0x%02x %s\r\n",
               chips[i].n, in, out, cfg,
               err ? "(I2C ERR)"
                   : (cfg == (uint8_t)~chips[i].mask ? "dir OK" : "*** dir WRONG ***"));
    }
}

/* ---- High-level analog paths — the ONE place switch states are defined ----
   Every layer (console dac/adc/measure, JSON analog_path, SDK) routes through
   here, so a path can never be wired two different ways in two places. */
int analog_path_set(analog_path_t path)
{
    int r = 0;
    switch (path) {
    case ANALOG_PATH_OFF:                                    /* all quiet */
        r |= dacmux_set_ctrl1(false, 0);
        r |= dacmux_set_ctrl2(false, 0);
        r |= calsw_set(false, false, false, false);
        break;
    case ANALOG_PATH_DAC_3V3:                                /* DAC -> 3V3 SMA */
        r |= dacmux_set_ctrl1(true, DACMUX_CTRL1_3V3);
        r |= dacmux_set_ctrl2(false, 0);
        r |= calsw_set(false, false, false, false);
        break;
    case ANALOG_PATH_DAC_5V:                                 /* DAC -> 5V SMA */
        r |= dacmux_set_ctrl1(true, DACMUX_CTRL1_5V);
        r |= dacmux_set_ctrl2(false, 0);
        r |= calsw_set(false, false, false, false);
        break;
    case ANALOG_PATH_DAC_12V:                                /* DAC -> ±12V diff SMA */
        r |= dacmux_set_ctrl1(true, DACMUX_CTRL1_12V);
        r |= dacmux_set_ctrl2(true, DACMUX_CTRL2_12V_VMID);
        r |= calsw_set(false, false, false, false);
        break;
    case ANALOG_PATH_ADC_EXT:                                /* ADC <- RF4 SMA (÷12) */
        r |= calsw_set(false, false, false, false);          /* leave DAC mux as-is */
        break;
    case ANALOG_PATH_CAL1:                                   /* 5V DAC -> ADC (K1) */
        r |= dacmux_set_ctrl1(true, DACMUX_CTRL1_5V);
        r |= dacmux_set_ctrl2(false, 0);
        r |= calsw_set(true, false, false, true);
        break;
    case ANALOG_PATH_CAL2:                                   /* ±12V diff -> ADC (K2/U51) */
        r |= dacmux_set_ctrl1(true, DACMUX_CTRL1_12V_ADC);
        r |= dacmux_set_ctrl2(true, DACMUX_CTRL2_ADC_VMID);
        r |= calsw_set(false, true, false, true);
        break;
    case ANALOG_PATH_AMP:                                    /* ADC <- amps terminal */
        r |= calsw_set(false, false, true, false);           /* leave DAC mux as-is */
        break;
    default:
        return -1;
    }
    return r;
}

static const struct { const char *name; analog_path_t path; } s_path_tbl[] = {
    { "off",     ANALOG_PATH_OFF     },
    { "dac_3v3", ANALOG_PATH_DAC_3V3 }, { "3v3", ANALOG_PATH_DAC_3V3 },
    { "dac_5v",  ANALOG_PATH_DAC_5V  }, { "5v",  ANALOG_PATH_DAC_5V  },
    { "dac_12v", ANALOG_PATH_DAC_12V }, { "12v", ANALOG_PATH_DAC_12V },
    { "adc_ext", ANALOG_PATH_ADC_EXT }, { "ext", ANALOG_PATH_ADC_EXT },
                                        { "sma", ANALOG_PATH_ADC_EXT },
    { "cal1",    ANALOG_PATH_CAL1    },
    { "cal2",    ANALOG_PATH_CAL2    },
    { "amp",     ANALOG_PATH_AMP     },
};

const char *analog_path_name(analog_path_t path)
{
    /* canonical names only (first hit per enum in the table) */
    static const char *canon[ANALOG_PATH__COUNT] = {
        "off", "dac_3v3", "dac_5v", "dac_12v", "adc_ext", "cal1", "cal2", "amp",
    };
    return (path < ANALOG_PATH__COUNT) ? canon[path] : "";
}

int analog_path_from_name(const char *name, analog_path_t *out)
{
    if (!name || !out) return -1;
    for (unsigned i = 0; i < sizeof(s_path_tbl) / sizeof(s_path_tbl[0]); i++) {
        if (strcmp(name, s_path_tbl[i].name) == 0) { *out = s_path_tbl[i].path; return 0; }
    }
    return -1;
}

void i2c_bus_status(void)
{
    if (!s_ready) { printf("[i2c] bus not initialised\n"); return; }
    printf("[i2c] scan:");
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (i2c_bus_probe(a)) {
            const char *name = "";
            switch (a) {
            case I2C_ADDR_INA238_INTERNAL: name = "INA238(int)"; break;
            case I2C_ADDR_INA238_EXTERNAL: name = "INA238(ext)"; break;
            case I2C_ADDR_TCA9554_LA:      name = "TCA9554(LA)"; break;
            case I2C_ADDR_TCA9554_PWR:     name = "TCA9554(pwr)"; break;
            case I2C_ADDR_TCA9554_DACMUX:  name = "TCA9554(dacmux)"; break;
            case I2C_ADDR_TCA9554_ANASW:   name = "TCA9554(anasw)"; break;
            default: break;
            }
            printf(" 0x%02x%s%s", a, name[0] ? "=" : "", name);
        }
    }
    printf("\n");
}
