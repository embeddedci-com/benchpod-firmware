/* test_dut_rails.c — host test for dut_rails.c: gated on board_caps()->dut_rails, both rails on
   at boot (level before mode), switchable, names parse. */
#include "dut_rails.h"
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
    dut_rails_init();
    CHECK(!dut_rails_present());
    CHECK(s_writes == 0 && s_inits == 0);
    CHECK(dut_rails_set(DUT_RAIL_3V3, false) == -1);
    CHECK(!dut_rails_get(DUT_RAIL_3V3));

    /* v4: both on, levels written before the pins become outputs. */
    board_caps_init(BOARD_REV_V4);
    dut_rails_init();
    CHECK(dut_rails_present());
    CHECK(s_inits == 1 && s_init_at == 2);
    CHECK(s_init_pins == (GPIO_PIN_3 | GPIO_PIN_4));
    CHECK((s_level & GPIO_PIN_3) && (s_level & GPIO_PIN_4));
    CHECK(dut_rails_get(DUT_RAIL_3V3) && dut_rails_get(DUT_RAIL_1V8));

    /* Independent switching. */
    CHECK(dut_rails_set(DUT_RAIL_1V8, false) == 0);
    CHECK(!dut_rails_get(DUT_RAIL_1V8) && dut_rails_get(DUT_RAIL_3V3));
    CHECK(!(s_level & GPIO_PIN_4) && (s_level & GPIO_PIN_3));
    CHECK(dut_rails_set(DUT_RAIL_3V3, false) == 0);
    CHECK(!(s_level & GPIO_PIN_3));
    CHECK(dut_rails_set(DUT_RAIL_3V3, true) == 0 && (s_level & GPIO_PIN_3));
    CHECK(dut_rails_set((dut_rail_t)5, true) == -1);

    /* Names. */
    CHECK(dut_rail_from_name("j1_3v3") == DUT_RAIL_3V3);
    CHECK(dut_rail_from_name("j1_1v8") == DUT_RAIL_1V8);
    CHECK(dut_rail_from_name("3v3") == -1 && dut_rail_from_name("") == -1 && dut_rail_from_name(NULL) == -1);
    CHECK(dut_rail_name(DUT_RAIL_3V3)[3] == '3' && dut_rail_name(DUT_RAIL_1V8)[3] == '1');

    if (failures) { printf("test_dut_rails: %d FAILED\n", failures); return 1; }
    printf("test_dut_rails: all passed\n");
    return 0;
}
