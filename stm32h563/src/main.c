/*
 * main.c — bench-pod STM32H563 firmware, Phase 1 boot skeleton.
 *
 * Brings up the clock tree (HSI -> PLL1 -> 250 MHz), the USART2 console, and
 * FreeRTOS, then runs a banner task.  Later phases add the iCE40 signal engine,
 * I2C devices, USB-CDC console, LwIP Ethernet server and XSPI PSRAM.
 *
 * Clock: 25 MHz HSE crystal (PH0/PH1) -> PLL1 -> 250 MHz SYSCLK.  USB (48 MHz
 * via HSI48+CRS) and RMII (50 MHz from the PHY) are brought up in later phases.
 */
#include "stm32h5xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board_pins.h"
#include "pico_compat.h"
#include "signal_engine.h"
#include "i2c_bus.h"
#include "ina238.h"
#include "tca9554.h"
#include "target_power.h"
#include "mcu_adc.h"
#include "board_rev.h"
#include "usb_cc.h"
#include "nrst_ctrl.h"
#include "can_bus.h"
#include "console_io.h"
#include "console.h"
#include "usb_device.h"
#include "net_server.h"
#include "device_identity.h"
#include "hw_lock.h"
#include "psram.h"
#include "dfu_boot.h"
#include "fault.h"
#include "watchdog.h"
#include "sys_health.h"
#include "hw_worker.h"
#include "version.h"

#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart2;

static void SystemClock_Config(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);

/* (printf retarget _write lives in console_io.c and fans out to USART2 + USB.) */

/* --- console task: line editor + periodic module polls ------------------- */
static void console_task(void *arg)
{
    (void)arg;
    console_init();
    for (;;) {
        watchdog_heartbeat(WD_TASK_CONSOLE, "console");
        /* Line editing only — complete lines are executed on the hw worker task,
           which also runs signal_engine_poll() / target_power_poll(). */
        console_poll();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/* --- net task: LwIP (NO_SYS=1) bring-up + poll loop ---------------------- */
static void net_task(void *arg)
{
    (void)arg;
    net_init();
    for (;;) {
        watchdog_heartbeat(WD_TASK_NET, "net");
        net_poll();
        watchdog_service();   /* refresh IWDG only if BOTH tasks are alive */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

int main(void)
{
    /* Must run before any clock/peripheral setup: if a prior `dfu` command armed
       the reboot flag, this jumps to the ROM USB DFU bootloader and never
       returns.  See dfu_boot.c. */
    dfu_boot_check();

    HAL_Init();
    SystemClock_Config();
    /* Latch the reset cause + any surviving crash record before anything can
       overwrite the .noinit slot, and clear the RCC flags. */
    fault_boot_init();
    MX_USART2_UART_Init();
    pico_compat_init();
    console_io_init();

    printf("\r\n\r\n");
    printf("========================================\r\n");
    printf(" bench-pod-firmware v%s\r\n", FIRMWARE_VERSION);
    printf(" STM32H563ZIT6  |  console: USB-CDC + USART2 PD5/PD6 @ %lu\r\n",
           (unsigned long)CONSOLE_UART_BAUD);
    printf(" sysclk=%lu Hz  FreeRTOS=%s\r\n",
           (unsigned long)HAL_RCC_GetSysClockFreq(), tskKERNEL_VERSION_NUMBER);
    printf(" reset: %s   last crash: %s\r\n",
           fault_last_reset_str(), fault_last_crash_str());
    printf("========================================\r\n");

    /* Bring up the hardware subsystems. */
    hw_lock_init();         /* serializes console + TCP command dispatch */

    /* Identify the PCB revision FIRST: it decides the TPS2116 LA-bank mux
       polarity (inverted between v2 and v3), whether the dedicated NRST pin
       exists, and whether the USB-C CC taps exist.  Everything below that
       touches those must already know the answer.  Cheap — one GPIO test and a
       few ADC conversions on the MCU's own ADC1. */
    (void)mcu_adc_init();   /* MCU ADC1: revision strap + USB-C CC sense */
    board_rev_init();       /* latches v2/v3 from the PA3 strap             */
    nrst_ctrl_init();       /* PF4 open-drain, released (no-op on v2)       */
    usb_cc_init();          /* CC1/CC2 analog pins   (no-op on v2)          */
    /* Bring the I2C bus + expander outputs up FIRST, before the slower SPI/XSPI
       init, so every TCA9554 output is driven to its safe (OFF) state as early as
       possible.  Both expanders power up with pins high-Z and output latch 0xFF,
       so until firmware drives them a DUT rail (eFuse EN) or a cal relay
       (U58→U53→K1..K4) can float active.  target_power_init() goes FIRST — a
       stray DUT power rail is the worst of the two — then the analog relays.
       (The complete fix for the pre-firmware window is a hardware pulldown on
       each EN / U53 IN net.) */
    i2c_bus_init();         /* I2C1 power/IO bus  */
    target_power_init();    /* eFuse EN driven OFF ASAP (glitch-sensitive) */
    analog_switch_init();   /* U55 DAC mux + U58 cal switching, all off */
    can_bus_init();         /* FDCAN1 term GPIO safe (core stays down until can_config) */
    signal_engine_init();   /* SPI1 to the iCE40 */
    /* Bring up the OCTOSPI/XSPI unconditionally: the same bus hosts the PSRAM AND
       the iCE40 config flash, and the config flash must be reachable (flash-ice40)
       even when the FPGA is unconfigured.  Gating this behind fpga_is_v2() was a
       bootstrap deadlock — the bus that flashes the FPGA only came up once the
       FPGA was already running.  psram_init() sets up the XSPI peripheral before
       it probes the PSRAM; then release the bus so the iCE40 can configure from
       its (now flashable) config flash. */
    (void)psram_init();
    psram_bus_release();
    /* Layered PSRAM datapath boot self-test: STM32<->PSRAM, iCE40->PSRAM write, and
       (if that fails) whether the iCE40 even reaches the shared /CE net — so a
       shared-bus write fault is flagged as such, never mis-read as a dead ADC.
       Console 'psram-selftest' re-runs it once the USB console is up.  The _recovery
       variant auto-reflashes the iCE40 if the write layer is faulted, so a reboot (the
       cloud `psram_recover` command) self-heals a wedged pod without a power-cycle. */
    psram_boot_selftest_with_recovery();
    i2c_bus_status();       /* scan + name known devices */
    device_identity_init(); /* Ed25519 identity (internal flash + RNG) */
    MX_USB_DEVICE_Init();   /* USB-CDC virtual COM port */

    /* The net task runs the lwIP poll loop AND the mbedTLS handshake for the
       outbound WSS cloud client; TLS (ASN.1/bignum) is stack-heavy, so give it
       a generous 16 KB stack. */
    TaskHandle_t console_h = NULL, net_h = NULL;
    if (xTaskCreate(console_task, "console", 768, NULL,
                    tskIDLE_PRIORITY + 1, &console_h) != pdPASS ||
        xTaskCreate(net_task, "net", 4096, NULL,
                    tskIDLE_PRIORITY + 2, &net_h) != pdPASS) {
        printf("[fatal] task create failed\r\n");
        fault_sw_panic(FAULT_SW_TASKCREATE, "main");
    }
    /* Register handles for stack/heap headroom telemetry (status/selftest). */
    sys_health_register("console", console_h);
    sys_health_register("net", net_h);

    /* The hw worker owns all instrument hardware + command execution; the console
       and net tasks feed it work and drain its replies.  Create it after the
       subsystems it drives are initialised. */
    hw_worker_init();

    /* Arm the IWDG last, just before handing control to the scheduler: if the
       scheduler never starts (out of heap) the watchdog still resets the pod.
       The net task refreshes it once both tasks are cycling. */
    watchdog_init();

    vTaskStartScheduler();

    /* Only reached if the scheduler could not start (out of heap). */
    Error_Handler();
    return 0;
}

/* --- clock tree: HSI 64 MHz -> PLL1 -> 250 MHz SYSCLK -------------------- */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* VOS0 is required for SYSCLK > 200 MHz. */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) { }

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;                   /* 25 MHz crystal on PH0/PH1 */
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
    osc.PLL.PLLM = 5;                           /* 25/5 = 5 MHz PLL input */
    osc.PLL.PLLN = 100;                         /* 5 * 100 = 500 MHz VCO */
    osc.PLL.PLLP = 2;                           /* 500/2 = 250 MHz */
    osc.PLL.PLLQ = 2;
    osc.PLL.PLLR = 2;
    osc.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;       /* 4..8 MHz input */
    osc.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE; /* 192..836 MHz VCO */
    osc.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;        /* HCLK = 250 MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    clk.APB3CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* --- USART2 console ------------------------------------------------------ */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = CONSOLE_UART;
    huart2.Init.BaudRate = CONSOLE_UART_BAUD;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};
    if (huart->Instance == CONSOLE_UART) {
        CONSOLE_UART_GPIO_CLK_EN();
        CONSOLE_UART_CLK_ENABLE();
        gpio.Pin = CONSOLE_UART_TX_PIN | CONSOLE_UART_RX_PIN;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = CONSOLE_UART_GPIO_AF;
        HAL_GPIO_Init(CONSOLE_UART_GPIO_PORT, &gpio);
    }
}

void Error_Handler(void)
{
    /* Record the reason and reset instead of spinning forever with IRQs off. HAL
       and SystemClock_Config call this on init failure — often before the IWDG is
       armed — so a bare while(1) here is a silent, unrecoverable hang. fault_sw_panic
       only writes the .noinit crash record and issues NVIC_SystemReset (no clock or
       peripheral dependency), so it is safe even this early; the next boot reports
       "last crash: error-handler". */
    fault_sw_panic(FAULT_SW_ERRHANDLER, "error-handler");
}

/* --- FreeRTOS hooks ------------------------------------------------------ */
/* These used to end in Error_Handler()'s while(1); now they record the reason in
   the crash log and reset, so the pod recovers and the next boot reports why. */
void vApplicationMallocFailedHook(void)
{
    printf("[fatal] malloc failed\r\n");
    fault_sw_panic(FAULT_SW_MALLOC, "heap");
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    printf("[fatal] stack overflow in task %s\r\n", name ? name : "?");
    fault_sw_panic(FAULT_SW_STACKOVF, name);
}

void vAssertCalled(const char *file, int line)
{
    printf("[assert] %s:%d\r\n", file ? file : "?", line);
    fault_sw_panic(FAULT_SW_ASSERT, file);
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    printf("[hal-assert] %s:%lu\r\n", (char *)file, (unsigned long)line);
    Error_Handler();
}
#endif
