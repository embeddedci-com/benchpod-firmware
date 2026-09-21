/*
 * board_pins.h — vbench-pod STM32H563ZIT6 pin map
 *
 * Derived from the board netlist (bench-pod-firmware/vbench-pod.net, MCU = U5).
 * This is the single source of truth for which STM32 pin carries which signal.
 * Phase 1 only uses the console UART; the remaining groups are captured here so
 * later phases (iCE40 SPI, I2C, RMII, USB, XSPI PSRAM) wire against this header.
 *
 * Net name (from schematic) is noted next to each pin.
 */
#ifndef BOARD_PINS_H
#define BOARD_PINS_H

/* ---------------------------------------------------------------------------
 * Console UART  — USART2 on PD5/PD6 (AF7)   [/USART2_TX, /USART2_RX]
 * ------------------------------------------------------------------------- */
#define CONSOLE_UART                 USART2
#define CONSOLE_UART_BAUD            115200u
#define CONSOLE_UART_CLK_ENABLE()    __HAL_RCC_USART2_CLK_ENABLE()
#define CONSOLE_UART_CLK_DISABLE()   __HAL_RCC_USART2_CLK_DISABLE()
#define CONSOLE_UART_IRQn            USART2_IRQn
#define CONSOLE_UART_GPIO_PORT       GPIOD
#define CONSOLE_UART_GPIO_CLK_EN()   __HAL_RCC_GPIOD_CLK_ENABLE()
#define CONSOLE_UART_TX_PIN          GPIO_PIN_5     /* PD5 */
#define CONSOLE_UART_RX_PIN          GPIO_PIN_6     /* PD6 */
#define CONSOLE_UART_GPIO_AF         GPIO_AF7_USART2

/* ---------------------------------------------------------------------------
 * iCE40 operational SPI link (signal engine) — SPI1   [/ICE_STM32_*]
 *   PA5 SCK, PA6 MISO, PB5 MOSI (AF5), PB10 CS0 (GPIO)
 *   PG1 = ICE_STM32_OTHER0 = BUSY/done (FPGA low while busy, rising edge = done)
 *   PG0 = ICE_STM32_OTHER1 = flash-bus ownership (STM32 asserts to make the
 *         iCE40 tristate the shared quad-SPI bus so the STM32 can drive it)
 *   PF15 = ICE_STM32_OTHER2 = spare
 * ------------------------------------------------------------------------- */
#define ICE_SPI                      SPI1
#define ICE_SPI_SCK_PORT             GPIOA
#define ICE_SPI_SCK_PIN              GPIO_PIN_5
#define ICE_SPI_MISO_PORT            GPIOA
#define ICE_SPI_MISO_PIN             GPIO_PIN_6
#define ICE_SPI_MOSI_PORT            GPIOB
#define ICE_SPI_MOSI_PIN             GPIO_PIN_5
#define ICE_SPI_AF                   GPIO_AF5_SPI1
#define ICE_SPI_CS_PORT              GPIOB
#define ICE_SPI_CS_PIN               GPIO_PIN_10   /* /ICE_STM32_CS0 */
#define ICE_BUSY_PORT                GPIOG         /* /ICE_STM32_OTHER0 (PG1) */
#define ICE_BUSY_PIN                 GPIO_PIN_1
#define ICE_BUSY_EXTI_IRQn           EXTI1_IRQn
#define ICE_FLASH_OWN_PORT           GPIOG         /* /ICE_STM32_OTHER1 (PG0) */
#define ICE_FLASH_OWN_PIN            GPIO_PIN_0
#define ICE_SPARE_PORT               GPIOF         /* /ICE_STM32_OTHER2 (PF15) */
#define ICE_SPARE_PIN                GPIO_PIN_15

/* ---------------------------------------------------------------------------
 * Shared quad-SPI bus (XSPI) — iCE40 cfg-flash (W25Q64, U2) + PSRAM (APS6404L,
 * U7) + iCE40 (U1).  Two chip-selects.   [/ICE_SCLK, /ICE_SPI_SO/SI, /ICE_IO2/3]
 *   PF10 SCLK, PF8 IO0, PF9 IO1, PF7 IO2, PF6 IO3
 *   PE3 = /ICE_SS (cfg-flash CS),  PE4 = /ICE_SS_PSRAM (PSRAM CS)
 *   PF13 = /ICE_CRESET,  PF14 = /ICE_CDONE
 * ------------------------------------------------------------------------- */
#define XSPI_SCLK_PORT               GPIOF
#define XSPI_SCLK_PIN                GPIO_PIN_10
#define XSPI_IO0_PORT                GPIOF
#define XSPI_IO0_PIN                 GPIO_PIN_8
#define XSPI_IO1_PORT                GPIOF
#define XSPI_IO1_PIN                 GPIO_PIN_9
#define XSPI_IO2_PORT                GPIOF
#define XSPI_IO2_PIN                 GPIO_PIN_7
#define XSPI_IO3_PORT                GPIOF
#define XSPI_IO3_PIN                 GPIO_PIN_6
#define ICE_FLASH_CS_PORT            GPIOE         /* /ICE_SS  (PE3) */
#define ICE_FLASH_CS_PIN             GPIO_PIN_3
#define PSRAM_CS_PORT                GPIOE         /* /ICE_SS_PSRAM (PE4) */
#define PSRAM_CS_PIN                 GPIO_PIN_4
#define ICE_CRESET_PORT              GPIOF         /* /ICE_CRESET (PF13) */
#define ICE_CRESET_PIN               GPIO_PIN_13
#define ICE_CDONE_PORT               GPIOF         /* /ICE_CDONE (PF14) */
#define ICE_CDONE_PIN                GPIO_PIN_14

/* ---------------------------------------------------------------------------
 * I2C0 power/IO bus — STM32 I2C1 on PB8/PB9 (AF4)   [/I2C0_SCL, /I2C0_SDA]
 *   Devices: INA238 x2 (U29/U33), TCA9554 x4 (U54/U55/U56/U58)
 *   PA8 = /EFUSE_INT (TCA9554 U56 INT)
 * ------------------------------------------------------------------------- */
#define PWR_I2C                      I2C1
#define PWR_I2C_SCL_PORT             GPIOB
#define PWR_I2C_SCL_PIN              GPIO_PIN_8
#define PWR_I2C_SDA_PORT             GPIOB
#define PWR_I2C_SDA_PIN              GPIO_PIN_9
#define PWR_I2C_AF                   GPIO_AF4_I2C1
#define EFUSE_INT_PORT               GPIOA
#define EFUSE_INT_PIN                GPIO_PIN_8

/* ---------------------------------------------------------------------------
 * RMII Ethernet — LAN8742A (U6), REF_CLK supplied by PHY into PA1 (AF11)
 *   PA1 REF_CLK, PA2 MDIO, PC1 MDC, PA7 CRS_DV, PC4 RXD0, PC5 RXD1,
 *   PG13 TXD0, PB15 TXD1, PG11 TX_EN, PE2 PHY nRST (GPIO)
 * ------------------------------------------------------------------------- */
#define RMII_NRST_PORT               GPIOE
#define RMII_NRST_PIN                GPIO_PIN_2

/* ---------------------------------------------------------------------------
 * USB-FS (CDC) — USB_DRD_FS on PA11 (D-) / PA12 (D+)   [/P_USB_D-, /P_USB_D+]
 * ------------------------------------------------------------------------- */
/* handled by the USB device stack; pins fixed by silicon */

/* ---------------------------------------------------------------------------
 * LA bank Vio control
 *   PG3 = /LA_VCCIO_SWITCH (-> TPS2116 mux),  PG4 = /LA_VCCIO_ST (status)
 * ------------------------------------------------------------------------- */
#define LA_VCCIO_SW_PORT             GPIOG
#define LA_VCCIO_SW_PIN              GPIO_PIN_3
#define LA_VCCIO_ST_PORT             GPIOG
#define LA_VCCIO_ST_PIN              GPIO_PIN_4

/* ---------------------------------------------------------------------------
 * CAN — TCAN1044 transceiver (U10) on FDCAN1.   [/FDCAN1_TX, /FDCAN1_RX]
 *   PD1 = FDCAN1_TX, PD0 = FDCAN1_RX (AF9).
 *   PA4 = /CAN_TERM_CTRL — drives one half of U24 (SN74LVC2G66) to switch the
 *         120 Ω bus termination in/out (high = termination engaged).
 *   The transceiver's STB (standby) pin is strapped by a resistor, not the MCU.
 *   Bus screw terminal = CAN+ / CAN-.  See can_bus.c.
 * ------------------------------------------------------------------------- */
#define CAN_FD                       FDCAN1
#define CAN_GPIO_PORT                GPIOD
#define CAN_GPIO_CLK_EN()            __HAL_RCC_GPIOD_CLK_ENABLE()
#define CAN_TX_PIN                   GPIO_PIN_1    /* PD1 -> U10 TXD */
#define CAN_RX_PIN                   GPIO_PIN_0    /* PD0 <- U10 RXD */
#define CAN_GPIO_AF                  GPIO_AF9_FDCAN1
#define CAN_IRQn                     FDCAN1_IT0_IRQn
#define CAN_TERM_PORT                GPIOA         /* /CAN_TERM_CTRL (PA4) */
#define CAN_TERM_PIN                 GPIO_PIN_4
#define CAN_TERM_CLK_EN()            __HAL_RCC_GPIOA_CLK_ENABLE()

/* ---------------------------------------------------------------------------
 * ESP32-C3-MINI-1 (U4) Wi-Fi co-processor — esp-hosted SPI link
 *   STM32 is SPI master; ESP32 is the Wi-Fi NIC running esp-hosted slave fw.
 *   SPI4: SCK PE12, MISO PE13, MOSI PE14 (AF5).  CS PB2 is a software GPIO.
 *   Slave-driven status lines:  HANDSHAKE PB0 (slave ready for a transaction),
 *   DATA_READY PB1 (slave has a frame to send).  Host-driven control:
 *   EN PF11 (reset, active-low: low = held in reset), BOOT PF12 (IO9 strap;
 *   high = normal boot, low = serial download for flashing).
 *   UART fallback (unused for SPI): USART1 TX PA9 / RX PA10 (AF7).
 * ------------------------------------------------------------------------- */
#define ESP_SPI                      SPI4
#define ESP_SPI_SCK_PORT             GPIOE
#define ESP_SPI_SCK_PIN              GPIO_PIN_12
#define ESP_SPI_MISO_PORT            GPIOE
#define ESP_SPI_MISO_PIN             GPIO_PIN_13
#define ESP_SPI_MOSI_PORT            GPIOE
#define ESP_SPI_MOSI_PIN             GPIO_PIN_14
#define ESP_SPI_AF                   GPIO_AF5_SPI4
#define ESP_CS_PORT                  GPIOB         /* /ESP32_CS  (PB2, GPIO)   */
#define ESP_CS_PIN                   GPIO_PIN_2
#define ESP_HANDSHAKE_PORT           GPIOB         /* /ESP32_HANDSHAKE (PB0)   */
#define ESP_HANDSHAKE_PIN            GPIO_PIN_0
#define ESP_DATAREADY_PORT           GPIOB         /* /ESP32_DATAREADY (PB1)   */
#define ESP_DATAREADY_PIN            GPIO_PIN_1
#define ESP_EN_PORT                  GPIOF         /* /ESP32_EN  (PF11, reset) */
#define ESP_EN_PIN                   GPIO_PIN_11
#define ESP_BOOT_PORT                GPIOF         /* /ESP32_BOOT (PF12, IO9)  */
#define ESP_BOOT_PIN                 GPIO_PIN_12
/* USART1 to the ESP32-C3 ROM UART (RXD0/TXD0) — used only to flash the C3's
 * serial download bootloader (esp_rom_flash.c).  Not used by the SPI transport. */
#define ESP_UART                     USART1
#define ESP_UART_GPIO_PORT           GPIOA
#define ESP_UART_TX_PIN              GPIO_PIN_9    /* PA9  -> ESP32 RXD0 (U4 pin30) */
#define ESP_UART_RX_PIN              GPIO_PIN_10   /* PA10 <- ESP32 TXD0 (U4 pin31) */
#define ESP_UART_GPIO_AF             GPIO_AF7_USART1

/* ---------------------------------------------------------------------------
 * Board revision strap (rev3+)   [Net-(U5-PA3), MCU pin 37]
 *   PA3 sits on a 1 k (R164, to +3V3) / 10 k (R163, to GND) divider, so a rev3
 *   board holds it at ~3.0 V.  On v2 the pin is a NC pad.  Detection is a
 *   digital pull-down test (NC collapses to 0, the divider does not), then an
 *   ADC read of the strap voltage so a future revision can be told apart by its
 *   divider ratio.  See board_rev.c.
 *   PA3 = ADC12_INP15 (DS14258 Table 12, "Pin/ball definition").
 * ------------------------------------------------------------------------- */
#define BOARD_REV_PORT               GPIOA
#define BOARD_REV_PIN                GPIO_PIN_3
#define BOARD_REV_CLK_EN()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define BOARD_REV_ADC_CH             ADC_CHANNEL_15
/* Nominal strap voltage of a rev3 board: 3300 * 10k/(10k+1k). */
#define BOARD_REV_V3_NOMINAL_MV      3000

/* ---------------------------------------------------------------------------
 * USB-C CC monitoring (rev3+)   [/CC_SENSE1 PC0, /CC_SENSE2 PC2]
 *   The pod is a USB-C sink: R76/R75 are the 5.1 k Rd pull-downs on USBC1's
 *   CC1/CC2.  Each CC line is tapped into an ADC pin through 10 k (R178/R179)
 *   with 10 n to GND (C259/C258) — the ADC input is high-Z, so the series
 *   resistor drops nothing and the pin sees the true CC voltage.  The level
 *   encodes the source's Rp advertisement (default / 1.5 A / 3.0 A) and which
 *   of the two lines is live gives the cable orientation.  See usb_cc.c.
 *   PC0 = ADC12_INP10, PC2 = ADC12_INP12 (DS14258 Table 12).
 * ------------------------------------------------------------------------- */
#define USB_CC1_PORT                 GPIOC
#define USB_CC1_PIN                  GPIO_PIN_0
#define USB_CC1_ADC_CH               ADC_CHANNEL_10
#define USB_CC2_PORT                 GPIOC
#define USB_CC2_PIN                  GPIO_PIN_2
#define USB_CC2_ADC_CH               ADC_CHANNEL_12
#define USB_CC_CLK_EN()              __HAL_RCC_GPIOC_CLK_ENABLE()

/* ---------------------------------------------------------------------------
 * Target NRST control (rev3+)   [/NRST_CONTROL, PF4 -> R174 330 R -> J1 pin 22]
 *   The DUT's reset line reaches the pod on pin-header J1 pin 22, pulled up by
 *   R7 (10 k) to LA_VCCIO — i.e. to whichever level the LA bank is switched to,
 *   1.8 V or 3.3 V.  PF4 must therefore NEVER drive high: a 3.3 V push-pull
 *   high would back-drive a 1.8 V DUT's reset net through the 330 R.  The pin is
 *   configured OPEN-DRAIN and only ever pulled low (asserted ~= LA_VCCIO *
 *   330/10330 = 105 mV); released it is Hi-Z and R7 restores the DUT's own
 *   level.  See nrst_ctrl.c.
 *   On v2 this pad is NC — nrst_ctrl reports unsupported there.
 * ------------------------------------------------------------------------- */
#define NRST_CTRL_PORT               GPIOF
#define NRST_CTRL_PIN                GPIO_PIN_4
#define NRST_CTRL_CLK_EN()           __HAL_RCC_GPIOF_CLK_ENABLE()

#endif /* BOARD_PINS_H */
