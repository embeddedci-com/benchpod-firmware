/* test_la_io.c: the LA I/O bank (la_io.c) on every board revision: the voltage selection, the
   mux polarity, the ST readback, the settle, and 5 V I/O with pin 21 (digital v4), with host
   stand-ins for the HAL (shim_la_io/), pin 21 (vref_out) and the pulls. */
#include "la_io.h"
#include "board_caps.h"
#include "board_rev.h"
#include "vref_out.h"
#include "la_pull.h"
#include "pico_compat.h"
#include "stm32h5xx_hal.h"   /* shim_la_io */
#include "board_pins.h"      /* shim_la_io */

#include <stdio.h>

static int checks, fails;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                                                   printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- fakes ---------------------------------------------------------------------------- */
GPIO_TypeDef shim_gpiog;
static vref_src_t g_src = VREF_SRC_BANK;
static int g_vref_mv = 5000;     /* what pin 21 measures once it is on 5 V */
static int g_src_writes;
static vref_src_t g_src_at_bank_write = VREF_SRC_BANK;   /* pin 21's source when PG3 was last written */
static int g_sel = -1;           /* PG3's written level */
static uint32_t g_sel_mode = 99; /* PG3's mode */
static int g_st = 1;             /* PG4 (ST) */
static unsigned g_delay_ms, g_sleep_ms, g_sleeps;
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState s) {
    if (port == LA_VCCIO_SW_PORT && pin == LA_VCCIO_SW_PIN) { g_sel = (int)s; g_src_at_bank_write = g_src; }
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint32_t pin) {
    (void)port;
    return (pin == LA_VCCIO_ST_PIN && g_st) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) {
    if (port == LA_VCCIO_SW_PORT && init->Pin == LA_VCCIO_SW_PIN) g_sel_mode = init->Mode;
}
void HAL_Delay(uint32_t ms) { g_delay_ms += ms; }
void sleep_ms(uint32_t ms) { g_sleep_ms += ms; g_sleeps++; }

int vref_out_set_source(vref_src_t src) { g_src = src; g_src_writes++; return 0; }
vref_src_t vref_out_source(void) { return g_src; }
int vref_out_measure_mv(void) { return g_src == VREF_SRC_5V ? g_vref_mv : 3300; }

static int g_pulls_off;
void la_pull_all_off(void) { g_pulls_off++; }

static void board(int rev) {
    board_caps_init(rev);
    la_vccio_init();
    g_src = VREF_SRC_BANK; g_vref_mv = 5000; g_src_writes = 0;
    g_delay_ms = g_sleep_ms = g_sleeps = 0; g_pulls_off = 0; g_sel = -1;
}

/* ---- tests ---------------------------------------------------------------------------- */
static void test_init(void) {
    board(BOARD_REV_V3);
    CHECK(la_vccio_get_mv() == LA_VCCIO_UNSET && la_io_get_mv() == LA_VCCIO_UNSET, "unset at boot");
    CHECK(g_sel_mode == GPIO_MODE_INPUT, "PG3 Hi-Z at boot (the pull-up decides)");
    CHECK(la_vccio_set_mv(1234) == -1 && la_vccio_get_mv() == LA_VCCIO_UNSET, "1234 mV refused");
    CHECK(la_vccio_set_mv(LA_VCCIO_UNSET) == -1, "0 mV refused");
}

/* v2: MODE grounded, only 3.3 V with PR1 low; 1.8 V and 5 V refused, ST means nothing. */
static void test_v2(void) {
    board(BOARD_REV_V2);
    CHECK(la_vccio_set_mv(LA_VCCIO_1V8) == -2 && la_vccio_get_mv() == LA_VCCIO_UNSET, "v2: 1.8 V refused");
    CHECK(g_sel == -1 && g_pulls_off == 0, "v2: a refused 1.8 V touches nothing");
    CHECK(la_vccio_set_mv(LA_VCCIO_3V3) == 0, "v2: 3.3 V");
    CHECK(g_sel == GPIO_PIN_RESET && g_sel_mode == GPIO_MODE_OUTPUT_PP, "v2: 3.3 V is PR1 low");
    CHECK(la_vccio_get_mv() == LA_VCCIO_3V3 && la_io_get_mv() == LA_VCCIO_3V3, "v2: both 3.3 V");
    CHECK(g_delay_ms == 2 && g_sleeps == 0, "v2: the 2 ms busy wait (no clamps)");
    CHECK(la_vccio_set_mv(LA_VCCIO_5V0) == -1 && la_io_get_mv() == LA_VCCIO_3V3, "v2: 5 V refused");
    g_st = 0;
    CHECK(la_vccio_readback_mv() == 0, "v2: no readback");
    g_st = 1;
}

/* v3 and v3r2: manual mode, PR1 high = 3.3 V, low = 1.8 V, ST reads it back. */
static void test_v3(int rev, const char *name) {
    board(rev);
    CHECK(la_vccio_set_mv(LA_VCCIO_1V8) == 0 && g_sel == GPIO_PIN_RESET, "%s: 1.8 V is PR1 low", name);
    CHECK(la_vccio_get_mv() == LA_VCCIO_1V8 && la_io_get_mv() == LA_VCCIO_1V8, "%s: both 1.8 V", name);
    CHECK(g_pulls_off == 0, "%s: the pulls follow the bank to 1.8 V", name);
    CHECK(la_vccio_set_mv(LA_VCCIO_3V3) == 0 && g_sel == GPIO_PIN_SET, "%s: 3.3 V is PR1 high", name);
    CHECK(g_delay_ms == 4 && g_sleeps == 0, "%s: 2 ms per change", name);
    g_st = 0;
    CHECK(la_vccio_readback_mv() == LA_VCCIO_1V8 && la_vccio_status_pin() == 0, "%s: ST low", name);
    g_st = 1;
    CHECK(la_vccio_readback_mv() == LA_VCCIO_3V3 && la_vccio_status_pin() == 1, "%s: ST high", name);
    CHECK(la_vccio_set_mv(LA_VCCIO_5V0) == -1 && la_io_get_mv() == LA_VCCIO_3V3, "%s: 5 V refused", name);
    CHECK(g_src_writes == 0, "%s: pin 21 never touched", name);
}

/* Digital v4: the clamps' settle, and 5 V I/O on pin 21. */
static void test_v4(void) {
    board(BOARD_REV_V4);
    const unsigned settle = LA_VCCIO_SETTLE_CLAMPS_MS + 1u;
    CHECK(la_vccio_set_mv(LA_VCCIO_1V8) == 0 && g_sleep_ms == settle && g_delay_ms == 0, "v4: the clamp settle");

    /* 1.8 V -> 5 V: the bank to 3.3 V first, then pin 21 to 5 V */
    g_sleep_ms = 0;
    CHECK(la_vccio_set_mv(LA_VCCIO_5V0) == 0, "v4: 5 V");
    CHECK(la_vccio_get_mv() == LA_VCCIO_3V3 && la_io_get_mv() == LA_VCCIO_5V0, "v4: bank 3.3 V, I/O 5 V");
    CHECK(g_src == VREF_SRC_5V && g_sel == GPIO_PIN_SET, "v4: pin 21 on 5 V, the bank at 3.3 V");
    CHECK(g_src_at_bank_write == VREF_SRC_BANK, "v4: the bank moved before pin 21 went to 5 V");
    CHECK(g_sleep_ms == 2 * settle, "v4: settled after the bank and after pin 21");

    /* again: nothing moves */
    g_sleep_ms = 0; g_src_writes = 0;
    CHECK(la_io_set_5v() == 0 && g_sleep_ms == 0 && g_src_writes == 0, "v4: 5 V again is a no-op");

    /* 5 V -> 3.3 V: pin 21 back on the bank before the bank moves */
    CHECK(la_vccio_set_mv(LA_VCCIO_3V3) == 0 && g_src == VREF_SRC_BANK, "v4: back to the bank");
    CHECK(g_src_at_bank_write == VREF_SRC_BANK, "v4: pin 21 left 5 V before the bank was written");
    CHECK(la_vccio_get_mv() == LA_VCCIO_3V3 && la_io_get_mv() == LA_VCCIO_3V3, "v4: both 3.3 V");

    /* 5 V that does not come up: the previous setting comes back */
    CHECK(la_vccio_set_mv(LA_VCCIO_1V8) == 0, "v4: 1.8 V");
    g_vref_mv = VREF_5V_MIN_MV - 1;
    CHECK(la_vccio_set_mv(LA_VCCIO_5V0) == -3, "v4: 5 V missing");
    CHECK(g_src == VREF_SRC_BANK && la_vccio_get_mv() == LA_VCCIO_1V8 && la_io_get_mv() == LA_VCCIO_1V8,
          "v4: back on 1.8 V");
    g_vref_mv = VREF_5V_MIN_MV;
    CHECK(la_vccio_set_mv(LA_VCCIO_5V0) == 0 && la_io_get_mv() == LA_VCCIO_5V0, "v4: 5 V at the minimum");

    /* from nothing chosen: a failure leaves it unset */
    board(BOARD_REV_V4);
    g_vref_mv = 4000;
    CHECK(la_vccio_set_mv(LA_VCCIO_5V0) == -3, "v4: 5 V missing from unset");
    CHECK(la_vccio_get_mv() == LA_VCCIO_UNSET && la_io_get_mv() == LA_VCCIO_UNSET && g_src == VREF_SRC_BANK,
          "v4: back to unset");
    CHECK(g_pulls_off == 0, "v4: the pulls stay");
}

int main(void)
{
    test_init();
    test_v2();
    test_v3(BOARD_REV_V3, "v3");
    test_v3(BOARD_REV_V3R2, "v3r2");
    test_v4();
    printf("test_la_io: %d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
