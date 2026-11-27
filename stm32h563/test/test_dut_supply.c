/* test_dut_supply.c — host test for dut_supply.c: gated on board_caps()->supply_3v3_1v8, both rails on
   at boot (level before mode), switchable. */
#include "dut_supply.h"
#include "board_caps.h"
#include "board_rev.h"
#include "stm32h5xx_hal.h"

#include <stdarg.h>
#include <stdio.h>

static int failures;
#define CHECK(c) do { if(!(c)){printf("FAIL %s:%d  %s\n",__FILE__,__LINE__,#c);failures++;} } while(0)

GPIO_TypeDef shim_gpiod;
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
    (void)port;
    s_inits++;
    s_init_at = s_writes;
    s_init_pins = init->Pin;
}

int main(void)
{
    /* An older board: nothing touched, nothing present. */
    board_caps_init(BOARD_REV_V3);
    dut_supply_init();
    CHECK(!dut_supply_present());
    CHECK(s_writes == 0 && s_inits == 0);
    CHECK(dut_supply_set(DUT_SUPPLY_3V3, false) == -1);
    CHECK(!dut_supply_get(DUT_SUPPLY_3V3));

    /* v4: both on, levels written before the pins become outputs. */
    board_caps_init(BOARD_REV_V4);
    dut_supply_init();
    CHECK(dut_supply_present());
    CHECK(s_inits == 1 && s_init_at == 2);
    CHECK(s_init_pins == (GPIO_PIN_3 | GPIO_PIN_4));
    CHECK((s_level & GPIO_PIN_3) && (s_level & GPIO_PIN_4));
    CHECK(dut_supply_get(DUT_SUPPLY_3V3) && dut_supply_get(DUT_SUPPLY_1V8));

    /* Independent switching. */
    CHECK(dut_supply_set(DUT_SUPPLY_1V8, false) == 0);
    CHECK(!dut_supply_get(DUT_SUPPLY_1V8) && dut_supply_get(DUT_SUPPLY_3V3));
    CHECK(!(s_level & GPIO_PIN_4) && (s_level & GPIO_PIN_3));
    CHECK(dut_supply_set(DUT_SUPPLY_3V3, false) == 0);
    CHECK(!(s_level & GPIO_PIN_3));
    CHECK(dut_supply_set(DUT_SUPPLY_3V3, true) == 0 && (s_level & GPIO_PIN_3));
    CHECK(dut_supply_set((dut_supply_t)5, true) == -1);

    /* A delayed switch waits for its time, and a newer command replaces it. */
    CHECK(dut_supply_set(DUT_SUPPLY_1V8, true) == 0);
    CHECK(dut_supply_schedule(DUT_SUPPLY_1V8, false, 500) == 0);
    CHECK(dut_supply_get(DUT_SUPPLY_1V8));
    s_tick = 499; dut_supply_poll();
    CHECK(dut_supply_get(DUT_SUPPLY_1V8));
    s_tick = 500; dut_supply_poll();
    CHECK(!dut_supply_get(DUT_SUPPLY_1V8) && !(s_level & GPIO_PIN_4));
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
