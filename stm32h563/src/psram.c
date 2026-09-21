/*
 * psram.c — APS6404L QSPI PSRAM driver for the STM32H5 OCTOSPI1.
 *
 * The PSRAM and the iCE40 config flash share the quad bus (PF6..PF10); each has
 * its own GPIO chip-select straight to the chip (PE4 = PSRAM, PE3 = flash).
 * PE4 is NOT an OCTOSPI hardware-NCS pin, so we cannot use memory-mapped mode
 * (which auto-toggles the controller NCS).  Instead the OCTOSPI drives only
 * SCLK + IO0..3 and the STM32 frames every transaction by toggling PE4 as a
 * GPIO, breaking long reads/writes into chunks under the APS6404 tCEM (~8 us
 * max CS-low time).  PE3 (flash CS) is held high so the flash stays off the bus.
 *
 * Shared-bus arbitration with the iCE40 (which writes captures into the PSRAM):
 * psram_bus_acquire() drives PG0 high + restores the OCTOSPI/CS pins;
 * psram_bus_release() tristates them + drops PG0 so the iCE40 can drive.
 *
 * Command set: reset 0x66/0x99, read-ID 0x9F (single SPI); QPI enable 0x35;
 * then quad read 0xEB (6 dummy) / quad write 0x38 (QPI).  ⚠ The exact dummy
 * count, tCEM chunk size and QPI framing need bench validation against the
 * APS6404L datasheet.
 */
#include "psram.h"
#include "board_pins.h"
#include "ice40_flash.h"
#include "pico_compat.h"
#include "stm32h5xx_hal.h"
#include "xfer.h"        /* XFER_OK */
#include "dma_wait.h"    /* GPDMA completion for the read data phase */
#include <string.h>
#include <stdio.h>

#define PSRAM_AF_CLK   GPIO_AF9_OCTOSPI1
#define PSRAM_AF_IO    GPIO_AF10_OCTOSPI1
/* Read data phase is DMA (fast FIFO drain), so a big burst clears in ~20 us of
   CS-low — at/under the ~34 us the old 64-B CPU path already ran safely, but
   moving 8x the data per HAL_XSPI_Command.  Writes are CPU/blocking + diagnostic
   only, so they keep the small 64-B burst.  Operation clock is raised to 50 MHz
   after init (shortens CS-low → tCEM margin; the rate is clock-independent — it's
   the per-chunk overhead that dominates, see psram_bench). */
#define TCEM_CHUNK_WRITE   64u    /* CPU write burst — keep CS-low short          */
#define PSRAM_READ_CHUNK  512u    /* DMA read burst (tuned via psram-bench)       */
#define PSRAM_OP_PRESCALER  4u    /* 250/(4+1) = 50 MHz operation clock (0xEB ok) */
static uint32_t s_tcem_chunk = PSRAM_READ_CHUNK;   /* read burst (psram-chunk)    */
#define QREAD_DUMMY    6u      /* APS6404 fast-quad-read (0xEB) wait cycles */

static XSPI_HandleTypeDef hxspi;

/* ---- read DMA (GPDMA1 ch4) ------------------------------------------------
   The read data phase was CPU-bound: HAL_XSPI_Receive drains the FIFO in a byte
   polling loop (~0.37 us/byte, clock-independent → ~2.7 MB/s ceiling).  DMA
   drains it at the bus rate instead, and (bonus) shortens CS-low time so bigger
   tCEM-safe chunks fit.  Only the read (0xEB) uses DMA; the rare writes and the
   tiny init reads (ID/reset, < PSRAM_DMA_MIN) stay blocking, as does everything
   before the RTOS scheduler starts (dma_sched_ready()==false). */
#ifndef PSRAM_USE_DMA
#define PSRAM_USE_DMA   1
#endif
#define PSRAM_DMA_MIN   32u    /* reads smaller than this stay blocking */

static DMA_HandleTypeDef hdma_psram;
static dma_waiter_t      s_psram_waiter;
static bool              s_psram_dma_ok;

void *psram_xspi(void) { return &hxspi; }

#define CS_LOW()   HAL_GPIO_WritePin(PSRAM_CS_PORT, PSRAM_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(PSRAM_CS_PORT, PSRAM_CS_PIN, GPIO_PIN_SET)

/* ---- pin setup -------------------------------------------------------- */
static void octospi_pins(int af)   /* SCLK + IO0..3 as AF (drive) or analog (Hi-Z) */
{
    GPIO_InitTypeDef g = {0};
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    if (af) { g.Mode = GPIO_MODE_AF_PP; g.Alternate = PSRAM_AF_CLK; }
    else      g.Mode = GPIO_MODE_ANALOG;
    g.Pin = XSPI_SCLK_PIN; HAL_GPIO_Init(XSPI_SCLK_PORT, &g);
    if (af) g.Alternate = PSRAM_AF_IO;
    g.Pin = XSPI_IO0_PIN; HAL_GPIO_Init(XSPI_IO0_PORT, &g);
    g.Pin = XSPI_IO1_PIN; HAL_GPIO_Init(XSPI_IO1_PORT, &g);
    g.Pin = XSPI_IO2_PIN; HAL_GPIO_Init(XSPI_IO2_PORT, &g);
    g.Pin = XSPI_IO3_PIN; HAL_GPIO_Init(XSPI_IO3_PORT, &g);
}

void psram_bus_acquire(void)
{
    GPIO_InitTypeDef g = {0};
    HAL_GPIO_WritePin(ICE_FLASH_OWN_PORT, ICE_FLASH_OWN_PIN, GPIO_PIN_SET);  /* PG0=1 */
    octospi_pins(1);
    /* PSRAM CS (PE4) as a driven GPIO, idle high (deselected). */
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_HIGH;
    CS_HIGH();
    g.Pin = PSRAM_CS_PIN; HAL_GPIO_Init(PSRAM_CS_PORT, &g);
    CS_HIGH();
}

void psram_bus_release(void)
{
    GPIO_InitTypeDef g = {0};
    octospi_pins(0);                                   /* SCLK/IO -> Hi-Z */
    g.Mode = GPIO_MODE_ANALOG; g.Pull = GPIO_NOPULL;
    g.Pin = PSRAM_CS_PIN; HAL_GPIO_Init(PSRAM_CS_PORT, &g);  /* PSRAM CS -> Hi-Z */
    HAL_GPIO_WritePin(ICE_FLASH_OWN_PORT, ICE_FLASH_OWN_PIN, GPIO_PIN_RESET); /* PG0=0 */
}

/* Park the PSRAM chip-select DRIVEN HIGH (deselected) with SCLK/IO left tristated.  Used
 * during an iCE40 WARMBOOT: the config flash and PSRAM share SCLK/SO/SI (see the .pcf),
 * so if the PSRAM /CS floats low the initialised PSRAM also answers the iCE40's 0x03
 * config-read and corrupts the bitstream (no sync word -> warmboot fails).  The iCE40's
 * own psram_cs pad is Hi-Z in config mode, so the STM32 must hold /CS high here. */
void psram_cs_park_high(void)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = PSRAM_CS_PIN; HAL_GPIO_Init(PSRAM_CS_PORT, &g);
    HAL_GPIO_WritePin(PSRAM_CS_PORT, PSRAM_CS_PIN, GPIO_PIN_SET);   /* PSRAM /CS = high */
}

/* ---- boot /CE-net probe (iCE40-reaches-PSRAM self-test) ------------------
   Release the STM32's drive of the shared bus and hand it to the iCE40 (PG0=0),
   but keep PE4 (PSRAM /CS, on the shared /CE net) as an INPUT with a pull-up.
   With the iCE40 told to force its /CS low (CMD_PSRAM_CS), the STM32 can then sense
   whether that drive reaches the net: read 0 => the iCE40 pulls /CE low (pad45
   reaches the net); read 1 => the pull-up wins, the iCE40's drive never arrives
   (pad45 -> U7 /CE open) — the "iCE40 can't write PSRAM" root cause. */
void psram_ce_probe_begin(void)
{
    GPIO_InitTypeDef g = {0};
    octospi_pins(0);                                                          /* SCLK/IO -> Hi-Z */
    HAL_GPIO_WritePin(ICE_FLASH_OWN_PORT, ICE_FLASH_OWN_PIN, GPIO_PIN_RESET); /* PG0=0: iCE40 owns bus */
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = PSRAM_CS_PIN; HAL_GPIO_Init(PSRAM_CS_PORT, &g);                   /* PE4 -> input + pull-up */
}
int  psram_ce_probe_read(void)   /* 1 = net high (iCE40 does NOT reach it), 0 = net low (reaches it) */
{
    return (HAL_GPIO_ReadPin(PSRAM_CS_PORT, PSRAM_CS_PIN) == GPIO_PIN_SET) ? 1 : 0;
}
/* Read all six shared PSRAM-bus lines as GPIO inputs (call between
   psram_ce_probe_begin/_end, with the iCE40 forcing its known pattern).
   Bit layout: b0=/CS(PE4) b1=SCLK b2=IO0 b3=IO1 b4=IO2 b5=IO3. */
uint8_t psram_bus_read_lines(void)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = XSPI_SCLK_PIN; HAL_GPIO_Init(XSPI_SCLK_PORT, &g);
    g.Pin = XSPI_IO0_PIN;  HAL_GPIO_Init(XSPI_IO0_PORT,  &g);
    g.Pin = XSPI_IO1_PIN;  HAL_GPIO_Init(XSPI_IO1_PORT,  &g);
    g.Pin = XSPI_IO2_PIN;  HAL_GPIO_Init(XSPI_IO2_PORT,  &g);
    g.Pin = XSPI_IO3_PIN;  HAL_GPIO_Init(XSPI_IO3_PORT,  &g);
    uint8_t v = 0;
    if (HAL_GPIO_ReadPin(PSRAM_CS_PORT, PSRAM_CS_PIN)) v |= 0x01;
    if (HAL_GPIO_ReadPin(XSPI_SCLK_PORT, XSPI_SCLK_PIN)) v |= 0x02;
    if (HAL_GPIO_ReadPin(XSPI_IO0_PORT,  XSPI_IO0_PIN))  v |= 0x04;
    if (HAL_GPIO_ReadPin(XSPI_IO1_PORT,  XSPI_IO1_PIN))  v |= 0x08;
    if (HAL_GPIO_ReadPin(XSPI_IO2_PORT,  XSPI_IO2_PIN))  v |= 0x10;
    if (HAL_GPIO_ReadPin(XSPI_IO3_PORT,  XSPI_IO3_PIN))  v |= 0x20;
    return v;
}

void psram_ce_probe_end(void)
{
    psram_bus_acquire();   /* restore: PG0=1, OCTOSPI AF, PE4 driven high */
}

/* ---- low-level OCTOSPI indirect transfer (GPIO-CS framed) -------------
 * The data phase below is intentionally BLOCKING, not GPDMA — unlike SPI1/SPI4.
 * Every psram_read/psram_write call is split by the tCEM loop into ≤TCEM_CHUNK
 * (64-byte) CS-low bursts, so the largest transfer the HAL ever sees is 64 B.
 * At the 25 MHz quad clock a 64-byte burst is ~5 µs, and HAL_XSPI_Receive already
 * moves the whole burst in ONE call (there is no per-byte HAL overhead like the
 * SPI1 read-loop had).  Driving each 64-byte burst over DMA would add an ISR +
 * completion-semaphore + context-switch (~a few µs) PER burst — comparable to
 * the transfer itself — making the deep read-back slower for no throughput gain.
 * DMA cannot span bursts either: the CS is a GPIO toggled between chunks (PE4 is
 * not the controller NCS), which DMA can't drive.  So blocking is the correct
 * choice here; the tCEM chunk is below the DMA break-even.  (If TCEM_CHUNK could
 * ever grow far past ~256 B this decision should be revisited.) */
static int xspi_xfer(uint8_t instr, uint32_t inst_mode, uint32_t addr_mode,
                     uint32_t addr, uint32_t data_mode, uint32_t dummy,
                     uint8_t *buf, uint32_t len, int is_read)
{
    XSPI_RegularCmdTypeDef c = {0};
    c.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    c.Instruction = instr;
    c.InstructionMode = inst_mode;
    c.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    c.AddressMode = addr_mode;
    c.Address = addr;
    c.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    c.DataMode = data_mode;
    c.DataLength = len;
    c.DummyCycles = dummy;

    CS_LOW();
    HAL_StatusTypeDef s = HAL_XSPI_Command(&hxspi, &c, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
    if (s == HAL_OK && len) {
        if (is_read && s_psram_dma_ok && len >= PSRAM_DMA_MIN && dma_sched_ready()) {
            dma_wait_arm(&s_psram_waiter);
            if (HAL_XSPI_Receive_DMA(&hxspi, buf) == HAL_OK) {
                if (dma_wait_block(&s_psram_waiter, 100) != XFER_OK) {
                    HAL_XSPI_Abort(&hxspi);
                    s = HAL_ERROR;
                }
            } else {   /* couldn't start DMA — fall back to blocking */
                s = HAL_XSPI_Receive(&hxspi, buf, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
            }
        } else {
            s = is_read ? HAL_XSPI_Receive(&hxspi, buf, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)
                        : HAL_XSPI_Transmit(&hxspi, buf, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
        }
    }
    CS_HIGH();
    return (s == HAL_OK) ? 0 : -1;
}

static int cmd_simple(uint8_t instr)   /* single-line, instruction only */
{
    return xspi_xfer(instr, HAL_XSPI_INSTRUCTION_1_LINE, HAL_XSPI_ADDRESS_NONE,
                     0, HAL_XSPI_DATA_NONE, 0, NULL, 0, 0);
}

/* Reset the PSRAM to STANDARD-SPI standby (the fresh-power-up state), WITHOUT re-entering
 * QPI — the QPI-then-single-line 0x66/0x99 sequence.  Used BEFORE an iCE40 warmboot: the
 * config flash and PSRAM share SCK/SO/SI (see the .pcf), and a QPI-mode PSRAM can mis-latch
 * stray clocks during the reconfig and drive IO0 (= SPI_SO, the line the config controller
 * reads the bitstream on) — which is why cold-boot (PSRAM idle in SPI) works but a runtime
 * warmboot fails.  In standard SPI the PSRAM ignores partial/deselected traffic.  Call
 * psram_init() after the warmboot to put it back in QPI for the gateware. */
void psram_reset_to_spi(void)
{
    psram_bus_acquire();                            /* STM32 owns the bus; OCTOSPI AF; /CS driven */
    xspi_xfer(0x66, HAL_XSPI_INSTRUCTION_4_LINES, HAL_XSPI_ADDRESS_NONE, 0,
              HAL_XSPI_DATA_NONE, 0, NULL, 0, 0);   /* QPI RSTEN (recovers a QPI-stuck part) */
    xspi_xfer(0x99, HAL_XSPI_INSTRUCTION_4_LINES, HAL_XSPI_ADDRESS_NONE, 0,
              HAL_XSPI_DATA_NONE, 0, NULL, 0, 0);   /* QPI RST  */
    sleep_ms(1);
    cmd_simple(0x66);                               /* SPI RSTEN */
    cmd_simple(0x99);                               /* SPI RST  */
    sleep_ms(1);
    /* left in standard SPI mode (no 0x35) */
}

int psram_read(uint32_t addr, uint8_t *buf, uint32_t len)   /* QPI 0xEB, chunked */
{
    for (uint32_t off = 0; off < len; off += s_tcem_chunk) {
        uint32_t n = (len - off < s_tcem_chunk) ? (len - off) : s_tcem_chunk;
        if (xspi_xfer(0xEB, HAL_XSPI_INSTRUCTION_4_LINES, HAL_XSPI_ADDRESS_4_LINES,
                      addr + off, HAL_XSPI_DATA_4_LINES, QREAD_DUMMY,
                      buf + off, n, 1) != 0)
            return -1;
    }
    return 0;
}

int psram_write(uint32_t addr, const uint8_t *buf, uint32_t len)  /* QPI 0x38; bus must be acquired */
{
    for (uint32_t off = 0; off < len; off += TCEM_CHUNK_WRITE) {
        uint32_t n = (len - off < TCEM_CHUNK_WRITE) ? (len - off) : TCEM_CHUNK_WRITE;
        if (xspi_xfer(0x38, HAL_XSPI_INSTRUCTION_4_LINES, HAL_XSPI_ADDRESS_4_LINES,
                      addr + off, HAL_XSPI_DATA_4_LINES, 0,
                      (uint8_t *)(buf + off), n, 0) != 0)
            return -1;
    }
    return 0;
}

int psram_init(void)
{
    RCC_PeriphCLKInitTypeDef pclk = {0};
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* PG0 (bus-own) + PE3 (flash CS) as GPIO outputs; take the bus, deselect flash. */
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = ICE_FLASH_OWN_PIN; HAL_GPIO_Init(ICE_FLASH_OWN_PORT, &g);
    HAL_GPIO_WritePin(ICE_FLASH_OWN_PORT, ICE_FLASH_OWN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ICE_FLASH_CS_PORT, ICE_FLASH_CS_PIN, GPIO_PIN_SET);
    g.Pin = ICE_FLASH_CS_PIN; HAL_GPIO_Init(ICE_FLASH_CS_PORT, &g);
    HAL_GPIO_WritePin(ICE_FLASH_CS_PORT, ICE_FLASH_CS_PIN, GPIO_PIN_SET);

    psram_bus_acquire();   /* OCTOSPI pins -> AF, PSRAM CS -> driven high */

    /* OCTOSPI1 kernel clock = HCLK (250 MHz). */
    pclk.PeriphClockSelection = RCC_PERIPHCLK_OSPI;
    pclk.OspiClockSelection = RCC_OSPICLKSOURCE_HCLK;
    HAL_RCCEx_PeriphCLKConfig(&pclk);
    __HAL_RCC_OSPI1_CLK_ENABLE();

    hxspi.Instance = OCTOSPI1;
    hxspi.Init.FifoThresholdByte = 4;
    hxspi.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
    hxspi.Init.MemoryType = HAL_XSPI_MEMTYPE_APMEM;
    /* APS6404L is 64 Mbit = 8 MBYTES.  The HAL_XSPI_SIZE_* macros are in mega-BITS
       (HAL_XSPI_SIZE_8MB == 8 Mbit == 1 MByte!), so the old HAL_XSPI_SIZE_8MB set
       DEVSIZE for a 1 MByte device — any STM32 XSPI access at/above 0x100000 then
       HAL-errored (and poisoned the peripheral so later reads failed too).  This was
       the real "iCE40 high-address write bug": the iCE40 masters its own QPI writes so
       it always reached the full 8 MB, but the STM32 read-back (and the no-write
       sentinel write) were capped at 1 MB.  64 Mbit = 8 MByte = the true device size. */
    hxspi.Init.MemorySize = HAL_XSPI_SIZE_64MB;   /* 64 Mbit = 8 MByte (APS6404L) */
    hxspi.Init.ChipSelectHighTimeCycle = 1;
    hxspi.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
    hxspi.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
    /* SCLK = 250 MHz / (prescaler+1).  The APS6404L caps Read (0x03) and Read ID
       (0x9F) at 33 MHz (tCLK >= 30.3 ns, datasheet Table 10) — and 0x9F has ZERO
       wait cycles, so an over-fast clock samples SO before the PSRAM drives it
       and reads 0xFF.  /10 = 25 MHz stays in spec for every command we issue
       (0xEB/0x38 allow 84-133 MHz) with margin for the shared, series-R bus. */
    hxspi.Init.ClockPrescaler = 9;
    hxspi.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
    hxspi.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
    hxspi.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
    if (HAL_XSPI_Init(&hxspi) != HAL_OK) {
        printf("[psram] OCTOSPI init failed\n");
        return -1;
    }
    printf("[psram] SCLK %lu kHz (HCLK %lu kHz / %lu) [0x9F/0x03 max 33 MHz]\n",
           (unsigned long)(HAL_RCC_GetHCLKFreq() / (hxspi.Init.ClockPrescaler + 1) / 1000),
           (unsigned long)(HAL_RCC_GetHCLKFreq() / 1000),
           (unsigned long)(hxspi.Init.ClockPrescaler + 1));

    /* Reset the device to SPI standby REGARDLESS of its current mode.  A DFU
       reflash resets the STM32 but NOT the PSRAM (it stays on +3V3), so if any
       prior firmware ever entered QPI (0x35) the device is still latched in QPI
       across every reboot — and single-line 0x66/0x99 are not understood in QPI,
       so the ID read would stay 0xFF forever.  Issue RSTEN/RST in QUAD framing
       first (recovers a QPI-stuck device, datasheet Fig.18), then in single-line
       framing (the normal power-up SPI case).  A quad command sent to an SPI-mode
       device is just an aborted partial command on CS-high — harmless. */
    xspi_xfer(0x66, HAL_XSPI_INSTRUCTION_4_LINES, HAL_XSPI_ADDRESS_NONE, 0,
              HAL_XSPI_DATA_NONE, 0, NULL, 0, 0);   /* QPI RSTEN */
    xspi_xfer(0x99, HAL_XSPI_INSTRUCTION_4_LINES, HAL_XSPI_ADDRESS_NONE, 0,
              HAL_XSPI_DATA_NONE, 0, NULL, 0, 0);   /* QPI RST  */
    sleep_ms(1);
    cmd_simple(0x66);                               /* SPI RSTEN */
    cmd_simple(0x99);                               /* SPI RST  */
    sleep_ms(1);

    uint8_t id[8] = {0};
    if (xspi_xfer(0x9F, HAL_XSPI_INSTRUCTION_1_LINE, HAL_XSPI_ADDRESS_1_LINE,
                  0, HAL_XSPI_DATA_1_LINE, 0, id, sizeof(id), 1) != 0) {
        printf("[psram] ID read failed\n");
        return -1;
    }
    printf("[psram] ID: %02x %02x %02x %02x %02x %02x %02x %02x\n",
           id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
    if (id[0] != 0x0D || id[1] != 0x5D) {
        printf("[psram] unexpected ID (want MFID 0x0D / KGD 0x5D) — check CS/bus/power\n");
        return -1;
    }
    if (cmd_simple(0x35) != 0) return -1;   /* enter QPI */

#if PSRAM_USE_DMA
    /* GPDMA for the read data phase.  XSPI DMA completes through the OCTOSPI1 TC
       interrupt (the GPDMA channel IRQ only arms it), so OCTOSPI1_IRQn is enabled
       + routed to HAL_XSPI_IRQHandler below.  Idempotent (psram_init may re-run
       via psram-test / psram_diag). */
    if (!s_psram_dma_ok) {
        if (dma_wait_channel_init(&hdma_psram, GPDMA1_Channel4, GPDMA1_REQUEST_OCTOSPI1,
                                  DMA_PERIPH_TO_MEMORY, GPDMA1_Channel4_IRQn) == 0) {
            __HAL_LINKDMA(&hxspi, hdmarx, hdma_psram);
            dma_wait_setup(&s_psram_waiter, OCTOSPI1);
            HAL_NVIC_SetPriority(OCTOSPI1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
            HAL_NVIC_EnableIRQ(OCTOSPI1_IRQn);
            s_psram_dma_ok = true;
            printf("[psram] read DMA enabled (GPDMA1 ch4)\n");
        } else {
            printf("[psram] read DMA init failed — using blocking reads\n");
        }
    }
#endif

    /* Raise the operation clock now that ID (0x9F, ≤33 MHz) + QPI are done.  0xEB
       reads / 0x38 writes are rated well above this.  Doesn't speed the read up
       (overhead-bound), but halves the DMA transfer time → shorter CS-low. */
    hxspi.Init.ClockPrescaler = PSRAM_OP_PRESCALER;
    if (HAL_XSPI_Init(&hxspi) != HAL_OK) {
        printf("[psram] WARN: op-clock re-init failed, staying at init clock\n");
    }

    printf("[psram] APS6404L up (QPI, GPIO-CS indirect, %lu MB, SCLK %lu MHz, "
           "read %luB DMA%s)\n",
           (unsigned long)(PSRAM_SIZE >> 20),
           (unsigned long)(HAL_RCC_GetHCLKFreq() / (hxspi.Init.ClockPrescaler + 1) / 1000000u),
           (unsigned long)s_tcem_chunk, s_psram_dma_ok ? "" : " (DMA off)");
    return 0;
}

/* ---- read-DMA IRQs (see psram_init) -------------------------------------- */
#if PSRAM_USE_DMA
void GPDMA1_Channel4_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_psram); }
void OCTOSPI1_IRQHandler(void)        { HAL_XSPI_IRQHandler(&hxspi); }
#endif

/* ---- bus/CS diagnostics ---------------------------------------------- */

static int rd(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1 : 0;
}

/* Sample the shared quad bus + PSRAM CS as plain GPIO inputs with the given
   pull, then report the levels.  With a pull-down, any line that still reads 1
   is being ACTIVELY DRIVEN high (by the iCE40, or the PSRAM if selected). */
static void diag_sample_bus(const char *when, uint32_t pull)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_INPUT; g.Pull = pull; g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = XSPI_SCLK_PIN; HAL_GPIO_Init(XSPI_SCLK_PORT, &g);
    g.Pin = XSPI_IO0_PIN;  HAL_GPIO_Init(XSPI_IO0_PORT, &g);
    g.Pin = XSPI_IO1_PIN;  HAL_GPIO_Init(XSPI_IO1_PORT, &g);
    g.Pin = XSPI_IO2_PIN;  HAL_GPIO_Init(XSPI_IO2_PORT, &g);
    g.Pin = XSPI_IO3_PIN;  HAL_GPIO_Init(XSPI_IO3_PORT, &g);
    for (volatile int i = 0; i < 1000; i++) { __NOP(); }   /* let pulls settle */
    printf("[psram-diag] bus (%s): SCLK=%d IO0=%d IO1=%d IO2=%d IO3=%d\n", when,
           rd(XSPI_SCLK_PORT, XSPI_SCLK_PIN), rd(XSPI_IO0_PORT, XSPI_IO0_PIN),
           rd(XSPI_IO1_PORT, XSPI_IO1_PIN),   rd(XSPI_IO2_PORT, XSPI_IO2_PIN),
           rd(XSPI_IO3_PORT, XSPI_IO3_PIN));
}

/* Force the iCE40 into reset (CRESET=PF13 low) so every iCE40 pin — including
   psram_cs (pad 45) on the CE net — is guaranteed Hi-Z. */
static void hold_ice40_reset(void)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);
    g.Pin = ICE_CRESET_PIN; HAL_GPIO_Init(ICE_CRESET_PORT, &g);
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_RESET);
    sleep_ms(2);
}

/* One-shot, all-in-one diagnostic for the '0xFF ID' failure.  In one run it
   reports: (0) arbitration pins, (1) shared-bus health via the config-flash ID
   (same OCTOSPI + SCLK/IO0/IO1 as the PSRAM — only the CS pin differs), (2) who
   drives the data bus iCE40-running vs -in-reset, (3) PE4->CE net continuity,
   and (4) the PSRAM ID with the iCE40 held off the bus.

   How to read the result:
   - flash-id != EF..  => shared bus/OCTOSPI broken (not PSRAM-specific).
   - flash-id OK, CE(PE4) reads 0 => PE4 pin/trace to the CE net is open.
   - flash-id OK, CE(PE4) reads 1, ID still 0xFF => CE reaches PE4 but the PSRAM
     never answers: R16/CE-to-U7 open, U7 power, or a dead/cold-joint U7. */
int psram_diag(void)
{
    GPIO_InitTypeDef g = {0};
    uint8_t fid[3] = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    printf("[psram-diag] PG0(bus_own)=%d CRESET(PF13)=%d CDONE(PF14)=%d\n",
           rd(ICE_FLASH_OWN_PORT, ICE_FLASH_OWN_PIN),
           rd(ICE_CRESET_PORT, ICE_CRESET_PIN),
           rd(ICE_CDONE_PORT, ICE_CDONE_PIN));

    /* (0) Who drives the bus with the iCE40 still running from boot. */
    diag_sample_bus("iCE40 running, pull-down", GPIO_PULLDOWN);

    /* (1) Shared-bus health: read the config-flash JEDEC ID (EF 40 17) over the
       SAME OCTOSPI + SCLK/IO0/IO1 the PSRAM uses.  Self-contained (acquires and
       releases the bus, holds/releases the iCE40 internally). */
    if (ice40_flash_read_id(fid) == 0 && fid[0] == 0xEF)
        printf("[psram-diag] bus health: flash-id = %02x %02x %02x  => OCTOSPI + shared SCLK/IO0/IO1 OK\n",
               fid[0], fid[1], fid[2]);
    else
        printf("[psram-diag] bus health: flash-id = %02x %02x %02x  => SHARED BUS/OCTOSPI PROBLEM (not PSRAM-specific)\n",
               fid[0], fid[1], fid[2]);

    hold_ice40_reset();

    /* (2) Data-bus drivers with the iCE40 now off the bus. */
    diag_sample_bus("iCE40 in reset, pull-down", GPIO_PULLDOWN);

    /* (3) PE4->CE net continuity.  CE (U7.1) has R17 (10k) pull-up to 3V3 via
       R16.  Read PE4 as input+pull-down: 1 => PE4 reaches the pulled-up CE net
       (trace to the net is intact — meter R16->U7.1 to confirm CE reaches the
       chip); 0 => PE4 pin/trace to the CE net is OPEN. */
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLDOWN; g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = PSRAM_CS_PIN; HAL_GPIO_Init(PSRAM_CS_PORT, &g);
    for (volatile int i = 0; i < 4000; i++) { __NOP(); }
    int ce_pd = rd(PSRAM_CS_PORT, PSRAM_CS_PIN);
    printf("[psram-diag] CE(PE4) input+pull-down = %d  => %s\n", ce_pd,
           ce_pd ? "reaches the CE pull-up net (PE4 OK; meter R16->U7.1)"
                 : "OPEN — PE4 pin/trace not tied to the CE net");

    /* (4) Probe the PSRAM ID with the iCE40 guaranteed off the bus. */
    printf("[psram-diag] probing PSRAM ID (iCE40 held in reset)...\n");
    int rc = psram_init();          /* prints SCLK + the 8 ID bytes */

    /* (5) If the ID is good, run the quad write/read pattern test WHILE the iCE40
       is still held in reset.  psram-test runs it with the iCE40 running; doing
       it here isolates whether a data-path corruption (e.g. one IO lane) is the
       iCE40 driving the bus or a genuine OCTOSPI/wiring issue. */
    if (rc == 0) {
        printf("[psram-diag] ID ok — quad pattern test (iCE40 still in reset)...\n");
        rc = psram_test();
    }

    /* Hand the bus back and release the iCE40 (PG0 low BEFORE CRESET high, else
       the iCE40 wakes into a bus it can't read its own config flash on). */
    psram_bus_release();
    HAL_GPIO_WritePin(ICE_CRESET_PORT, ICE_CRESET_PIN, GPIO_PIN_SET);
    printf("[psram-diag] iCE40 released (reconfigures from flash)\n");
    return rc;
}

int psram_test(void)
{
    uint8_t wbuf[256], rbuf[256];
    for (int i = 0; i < 256; i++) wbuf[i] = (uint8_t)(i ^ 0xA5);
    if (psram_write(0, wbuf, sizeof(wbuf)) != 0) { printf("[psram] write failed\n"); return -1; }
    memset(rbuf, 0, sizeof(rbuf));
    if (psram_read(0, rbuf, sizeof(rbuf)) != 0) { printf("[psram] read failed\n"); return -1; }
    if (memcmp(wbuf, rbuf, sizeof(wbuf)) != 0) {
        int first = -1, nbad = 0;
        for (int i = 0; i < 256; i++)
            if (wbuf[i] != rbuf[i]) { if (first < 0) first = i; nbad++; }
        printf("[psram] pattern mismatch: %d/256 differ, first@%d\n", nbad, first);
        printf("[psram]  wrote:");
        for (int i = 0; i < 16; i++) printf(" %02x", wbuf[i]);
        printf("\n[psram]  read :");
        for (int i = 0; i < 16; i++) printf(" %02x", rbuf[i]);
        printf("\n[psram]  xor  :");   /* set bits => which lanes flipped */
        for (int i = 0; i < 16; i++) printf(" %02x", (uint8_t)(wbuf[i] ^ rbuf[i]));
        printf("\n");
        return -1;
    }
    printf("[psram] pattern test ok\n");
    return 0;
}

/* ---- read-throughput benchmark + runtime clock (console psram-bench/psram-clk)
   PSRAM is the real capture data path (iCE40 streams samples in, the STM32 reads
   them back over OCTOSPI), so this measures the actual read-back rate and checks
   the data is still correct at the current clock. */
int psram_bench(uint32_t total_kb)
{
    static uint8_t buf[4096];
    if (total_kb == 0) total_kb = 256;
    uint32_t total = total_kb * 1024u;
    uint32_t sclk  = HAL_RCC_GetHCLKFreq() / (hxspi.Init.ClockPrescaler + 1);

    psram_bus_acquire();

    /* Integrity: write a known 4 KB pattern, read it back, verify — so a fast but
       corrupt read (e.g. an over-clocked bus) shows up as integrity=FAIL. */
    for (int i = 0; i < 4096; i++) buf[i] = (uint8_t)(i * 31 + 7);
    bool ok = (psram_write(0, buf, 4096) == 0);
    memset(buf, 0, 4096);
    ok = ok && (psram_read(0, buf, 4096) == 0);
    for (int i = 0; ok && i < 4096; i++) if (buf[i] != (uint8_t)(i * 31 + 7)) ok = false;

    /* Speed: read `total` bytes sequentially (the normal 64-byte tCEM path). */
    uint64_t t0 = time_us_64();
    uint32_t done = 0;
    int rc = 0;
    while (done < total) {
        uint32_t n = (total - done < sizeof(buf)) ? (total - done) : sizeof(buf);
        if (psram_read(done % PSRAM_SIZE, buf, n) != 0) { rc = -1; break; }
        done += n;
    }
    uint64_t t1 = time_us_64();
    psram_bus_release();

    if (rc != 0) { printf("[psram-bench] read failed\n"); return -1; }
    uint32_t us   = (uint32_t)(t1 - t0); if (!us) us = 1;
    uint32_t kbps = (uint32_t)(((uint64_t)total * 1000u) / us);
    printf("[psram-bench] SCLK %lu MHz quad, %u-B chunks: integrity=%s, "
           "read %lu KB in %lu us => %lu.%02lu MB/s\n",
           (unsigned long)(sclk / 1000000u), (unsigned)s_tcem_chunk, ok ? "OK" : "FAIL",
           (unsigned long)total_kb, (unsigned long)us,
           (unsigned long)(kbps / 1000), (unsigned long)((kbps % 1000) / 10));
    return ok ? 0 : -1;
}

/* Re-init OCTOSPI at a new clock prescaler (SCLK = HCLK/(presc+1)); returns SCLK
   Hz or 0.  For sweeping the PSRAM read clock from the console (psram-clk). */
uint32_t psram_set_prescaler(uint32_t presc)
{
    if (presc > 255) return 0;
    psram_bus_acquire();
    hxspi.Init.ClockPrescaler = presc;
    HAL_StatusTypeDef s = HAL_XSPI_Init(&hxspi);
    psram_bus_release();
    if (s != HAL_OK) return 0;
    return HAL_RCC_GetHCLKFreq() / (presc + 1);
}

/* Set the tCEM CS-low burst size (bytes).  Bigger chunks amortise the per-chunk
   HAL_XSPI_Command overhead (the real bottleneck) but must stay under the ~8 µs
   APS6404 tCEM at the current SCLK — psram_bench's integrity check catches an
   over-long burst.  Returns the value set (clamped 1..1024). */
uint32_t psram_set_chunk(uint32_t n)
{
    if (n < 1)    n = 1;
    if (n > 1024) n = 1024;
    s_tcem_chunk = n;
    return n;
}
