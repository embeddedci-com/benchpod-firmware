/* test_dut_supply.c — host test for dut_supply.c: gated on board_caps()->supply_3v3_1v8, both rails on
   at boot (level before mode), switchable.  Also the PE3/PE4 split with qspi_cs.c: on digital v4
   (both revision straps agree) they are the DUT supply enables and the quad bus chip-selects are
   on PB6/PC11; on every other revision (v3r3 and older, unknown, a misread) they stay the
   chip-selects and the DUT supply code never touches a pin. */
#include "dut_supply.h"
#include "board_caps.h"
#include "board_rev.h"
#include "stm32h5xx_hal.h"
#include "board_pins.h"
#include "qspi_cs.h"

#include <stdarg.h>
#include <stdio.h>

static int failures;
#define CHECK(c) do { if(!(c)){printf("FAIL %s:%d  %s\n",__FILE__,__LINE__,#c);failures++;} } while(0)

GPIO_TypeDef shim_gpiob, shim_gpioc, shim_gpioe;
static GPIO_TypeDef *s_init_port;
static uint32_t s_level;      /* output level bits */
static int      s_init_at;    /* HAL_GPIO_Init call order: how many writes had happened */
static int      s_writes;
static int      s_inits;
static uint32_t s_init_pins;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState s)
{
    (void)port;
    s_writes++;
    if (s == GPIO_PIN_SET) s_level |= pin; else s_level &= ~pin;
}
static uint32_t s_tick;
uint32_t HAL_GetTick(void) { return s_tick; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    s_inits++;
    s_init_port = port;
    s_init_at = s_writes;
    s_init_pins = init->Pin;
}

static void reset_qspi_cs(void)
{
    qspi_cs = (qspi_cs_pins_t){ GPIOE, GPIO_PIN_3, GPIOE, GPIO_PIN_4 };   /* the power-on state */
}

int main(void)
{
    /* Every revision but v4 (BOARD_REV_UNKNOWN is also what a failed strap read latches; 1, 6
       and 99 stand for a misread): PE3/PE4 stay the chip-selects, nothing touched, nothing present. */
    static const int old[] = { BOARD_REV_UNKNOWN, BOARD_REV_V2, BOARD_REV_V3, BOARD_REV_V3R2, 1, 6, 99, -1 };
    for (unsigned i = 0; i < sizeof old / sizeof old[0]; i++) {
        reset_qspi_cs();
        board_caps_init(old[i]);
        qspi_cs_init();
        CHECK(qspi_cs.flash_port == GPIOE && qspi_cs.flash_pin == GPIO_PIN_3);
        CHECK(qspi_cs.psram_port == GPIOE && qspi_cs.psram_pin == GPIO_PIN_4);
        dut_supply_init();
        CHECK(!dut_supply_present());
        CHECK(s_writes == 0 && s_inits == 0);
        CHECK(dut_supply_set(DUT_SUPPLY_3V3, false) == -1 && dut_supply_set(DUT_SUPPLY_1V8, false) == -1);
        CHECK(dut_supply_schedule(DUT_SUPPLY_3V3, true, 0) == -1);   /* target_power answers BP0092 */
        dut_supply_poll();
        CHECK(s_writes == 0 && s_inits == 0);
        CHECK(!dut_supply_get(DUT_SUPPLY_3V3) && !dut_supply_get(DUT_SUPPLY_1V8));
    }

    /* v4: the chip-selects leave PE3/PE4 (before dut_supply_init, as in main.c), so the DUT
       enables there share no pin with them. */
    reset_qspi_cs();
    board_caps_init(BOARD_REV_V4);
    qspi_cs_init();
    CHECK(qspi_cs.flash_port == GPIOB && qspi_cs.flash_pin == GPIO_PIN_6);
    CHECK(qspi_cs.psram_port == GPIOC && qspi_cs.psram_pin == GPIO_PIN_11);
    CHECK(DUT_3V3_EN_PORT == GPIOE && DUT_1V8_EN_PORT == GPIOE);
    CHECK(DUT_3V3_EN_PIN != DUT_1V8_EN_PIN);
    CHECK(qspi_cs.flash_port != GPIOE && qspi_cs.psram_port != GPIOE);

    /* v4: both on, levels written before the pins become outputs. */
    dut_supply_init();
    CHECK(dut_supply_present());
    CHECK(s_inits == 1 && s_init_at == 2);
    CHECK(s_init_port == GPIOE && s_init_pins == (GPIO_PIN_3 | GPIO_PIN_4));   /* PE4 3V3, PE3 1V8 */
    CHECK((s_level & DUT_3V3_EN_PIN) && (s_level & DUT_1V8_EN_PIN));
    CHECK(dut_supply_get(DUT_SUPPLY_3V3) && dut_supply_get(DUT_SUPPLY_1V8));

    /* Independent switching. */
    CHECK(dut_supply_set(DUT_SUPPLY_1V8, false) == 0);
    CHECK(!dut_supply_get(DUT_SUPPLY_1V8) && dut_supply_get(DUT_SUPPLY_3V3));
    CHECK(!(s_level & DUT_1V8_EN_PIN) && (s_level & DUT_3V3_EN_PIN));
    CHECK(dut_supply_set(DUT_SUPPLY_3V3, false) == 0);
    CHECK(!(s_level & DUT_3V3_EN_PIN));
    CHECK(dut_supply_set(DUT_SUPPLY_3V3, true) == 0 && (s_level & DUT_3V3_EN_PIN));
    CHECK(dut_supply_set((dut_supply_t)5, true) == -1);

    /* A delayed switch waits for its time, and a newer command replaces it. */
    CHECK(dut_supply_set(DUT_SUPPLY_1V8, true) == 0);
    CHECK(dut_supply_schedule(DUT_SUPPLY_1V8, false, 500) == 0);
    CHECK(dut_supply_get(DUT_SUPPLY_1V8));
    s_tick = 499; dut_supply_poll();
    CHECK(dut_supply_get(DUT_SUPPLY_1V8));
    s_tick = 500; dut_supply_poll();
    CHECK(!dut_supply_get(DUT_SUPPLY_1V8) && !(s_level & DUT_1V8_EN_PIN));
    CHECK(dut_supply_schedule(DUT_SUPPLY_1V8, true, 100) == 0);
    CHECK(dut_supply_set(DUT_SUPPLY_1V8, false) == 0);          /* now: cancels the pending on */
    s_tick = 700; dut_supply_poll();
    CHECK(!dut_supply_get(DUT_SUPPLY_1V8));
    CHECK(dut_supply_schedule(DUT_SUPPLY_3V3, false, 0) == 0 && !dut_supply_get(DUT_SUPPLY_3V3));
    CHECK(dut_supply_schedule(DUT_SUPPLY_3V3, true, 10) == 0);
    s_tick = 0xFFFFFFF0u; dut_supply_poll();                   /* a wrapping tick must not fire early */
    CHECK(!dut_supply_get(DUT_SUPPLY_3V3));

    if (failures) { printf("test_dut_supply: %d FAILED\n", failures); return 1; }
    printf("test_dut_supply: all passed\n");
    return 0;
}
