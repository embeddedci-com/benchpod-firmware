/* See mock_flash.h. Implements the <hardware/flash.h> + <hardware/sync.h> shims and
   flash_read_checked() (as if the NMI hook were installed: an ECC hit reads as -1). */
#include "mock_flash.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "flash_compat.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t mock_flash[MOCK_FLASH_BYTES] = {0};
uint8_t mock_flash_ecc_bad[MOCK_FLASH_BYTES / MOCK_QW] = {0};
uint8_t mock_flash_programmed[MOCK_FLASH_BYTES / MOCK_QW] = {0};
int     mock_flash_erases, mock_flash_programs, mock_flash_double_programs;
int     mock_flash_ecc_reads, mock_flash_irq_depth;
int     mock_flash_cut_after;
jmp_buf mock_flash_power_jmp;

static uint32_t s_rng = 12345u;
static uint8_t rnd(void) { s_rng = s_rng * 1103515245u + 12345u; return (uint8_t)(s_rng >> 16); }

static uint32_t idx(uint32_t off, size_t n)
{
    if (off < MOCK_FLASH_BASE_OFF || off + n > MOCK_FLASH_BASE_OFF + MOCK_FLASH_BYTES) {
        fprintf(stderr, "mock_flash: access 0x%06x+%zu outside the modelled tail\n", off, n);
        abort();
    }
    return off - MOCK_FLASH_BASE_OFF;
}

void mock_flash_reset(void)
{
    memset(mock_flash, 0xFF, sizeof(mock_flash));
    memset(mock_flash_ecc_bad, 0, sizeof(mock_flash_ecc_bad));
    memset(mock_flash_programmed, 0, sizeof(mock_flash_programmed));
    mock_flash_erases = mock_flash_programs = mock_flash_double_programs = 0;
    mock_flash_ecc_reads = mock_flash_irq_depth = 0;
    mock_flash_cut_after = 0;
}

void mock_flash_poke(uint32_t off, const void *data, size_t n)
{
    uint32_t i = idx(off, n);
    memcpy(mock_flash + i, data, n);
    for (uint32_t q = i / MOCK_QW; q <= (i + n - 1) / MOCK_QW; q++) mock_flash_programmed[q] = 1;
}

uint8_t *mock_flash_ptr(uint32_t off) { return mock_flash + idx(off, 1); }

/* Returns 1 if the power is cut on this step. */
static int step(void)
{
    if (mock_flash_cut_after <= 0) return 0;
    return --mock_flash_cut_after == 0;
}

static void power_lost(void)
{
    mock_flash_irq_depth = 0;   /* the "reboot" resets the CPU */
    longjmp(mock_flash_power_jmp, 1);
}

void flash_range_erase(uint32_t offset, size_t count)
{
    uint32_t i = idx(offset, count);
    if (i % FLASH_SECTOR_SIZE || count % FLASH_SECTOR_SIZE) { fprintf(stderr, "mock_flash: unaligned erase\n"); abort(); }
    if (step()) {
        /* Interrupted erase: every cell anywhere between old and erased; many
           quad-words fail ECC, some read back as old data, some as erased. */
        for (uint32_t q = i / MOCK_QW; q < (i + count) / MOCK_QW; q++) {
            uint8_t r = rnd() % 3;
            if (r == 0) { memset(mock_flash + q * MOCK_QW, 0xFF, MOCK_QW); mock_flash_programmed[q] = 0; mock_flash_ecc_bad[q] = 0; }
            else if (r == 1) { for (int b = 0; b < (int)MOCK_QW; b++) mock_flash[q * MOCK_QW + b] |= rnd(); mock_flash_ecc_bad[q] = 1; mock_flash_programmed[q] = 1; }
            /* r == 2: untouched */
        }
        power_lost();
    }
    memset(mock_flash + i, 0xFF, count);
    memset(mock_flash_ecc_bad + i / MOCK_QW, 0, count / MOCK_QW);
    memset(mock_flash_programmed + i / MOCK_QW, 0, count / MOCK_QW);
    mock_flash_erases++;
}

void flash_range_program(uint32_t offset, const uint8_t *data, size_t count)
{
    uint32_t i = idx(offset, count);
    if (i % MOCK_QW) { fprintf(stderr, "mock_flash: unaligned program\n"); abort(); }
    for (size_t p = 0; p < count; p += MOCK_QW) {
        uint32_t q = (uint32_t)(i + p) / MOCK_QW;
        uint8_t qw[MOCK_QW];
        size_t n = count - p < MOCK_QW ? count - p : MOCK_QW;
        memset(qw, 0xFF, sizeof(qw));
        memcpy(qw, data + p, n);
        if (step()) {
            /* Torn quad-word program: some bits made it, ECC is wrong. */
            for (int b = 0; b < (int)MOCK_QW; b++) mock_flash[q * MOCK_QW + b] &= (qw[b] | rnd());
            mock_flash_ecc_bad[q] = 1;
            mock_flash_programmed[q] = 1;
            power_lost();
        }
        if (mock_flash_programmed[q]) { mock_flash_double_programs++; mock_flash_ecc_bad[q] = 1; }
        for (int b = 0; b < (int)MOCK_QW; b++) mock_flash[q * MOCK_QW + b] &= qw[b];
        mock_flash_programmed[q] = 1;
        mock_flash_programs++;
    }
}

uint32_t save_and_disable_interrupts(void) { return (uint32_t)mock_flash_irq_depth++; }
void restore_interrupts(uint32_t status) { mock_flash_irq_depth = (int)status; }

int flash_read_checked(uint32_t offset, void *dst, size_t n)
{
    uint32_t i = idx(offset, n);
    for (uint32_t q = i / MOCK_QW; n && q <= (i + n - 1) / MOCK_QW; q++) {
        if (mock_flash_ecc_bad[q]) { mock_flash_ecc_reads++; return -1; }
    }
    memcpy(dst, mock_flash + i, n);
    return 0;
}

int flash_ecc_nmi_absorb(void) { return 0; }
