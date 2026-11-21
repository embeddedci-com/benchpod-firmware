/* qspi_cs.c — see qspi_cs.h. */
#include "qspi_cs.h"
#include "board_caps.h"

/* PE3 / PE4: every board up to digital v3r3, and the state before qspi_cs_init(). */
qspi_cs_pins_t qspi_cs = { GPIOE, GPIO_PIN_3, GPIOE, GPIO_PIN_4 };

void qspi_cs_init(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
    if (board_caps()->qspi_cs_ncs_pins) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        qspi_cs.flash_port = GPIOB;   /* PB6, pin 136 */
        qspi_cs.flash_pin  = GPIO_PIN_6;
        qspi_cs.psram_port = GPIOC;   /* PC11, pin 112 */
        qspi_cs.psram_pin  = GPIO_PIN_11;
    }
}
