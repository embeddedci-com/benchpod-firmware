/* mock_w25q.c — see mock_w25q.h. Implements w25q.h. */
#include "mock_w25q.h"
#include "w25q.h"
#include <string.h>

uint8_t mock_w25q[MOCK_W25Q_BYTES];
uint8_t mock_w25q_id[3];
int     mock_w25q_ops;
int     mock_w25q_bad_programs;
int     mock_w25q_cut_after;
int     mock_w25q_fail_read;
int     mock_w25q_open_depth;
jmp_buf mock_w25q_power_jmp;

void mock_w25q_reset(void)
{
    memset(mock_w25q, 0xFF, sizeof(mock_w25q));
    mock_w25q_id[0] = 0xEF; mock_w25q_id[1] = 0x40; mock_w25q_id[2] = 0x17;
    mock_w25q_ops = 0;
    mock_w25q_bad_programs = 0;
    mock_w25q_cut_after = 0;
    mock_w25q_fail_read = 0;
    mock_w25q_open_depth = 0;
}

/* One flash operation: may be the one the power cut lands on. */
static void step(void)
{
    mock_w25q_ops++;
    if (mock_w25q_cut_after > 0 && --mock_w25q_cut_after == 0) longjmp(mock_w25q_power_jmp, 1);
}

int  w25q_open(void)  { mock_w25q_open_depth++; return 0; }
void w25q_close(void) { mock_w25q_open_depth--; }
void w25q_cs_output(void) {}
void w25q_wake(void) {}

int w25q_read_id(uint8_t id[3]) { memcpy(id, mock_w25q_id, 3); return 0; }

uint32_t w25q_capacity(const uint8_t id[3])
{
    if (id[0] != 0xEF) return 0;
    if (id[2] == 0x17) return 8u * 1024u * 1024u;
    if (id[2] == 0x18) return 16u * 1024u * 1024u;
    return 0;
}

int w25q_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (mock_w25q_fail_read || addr + len > MOCK_W25Q_BYTES) return -1;
    memcpy(buf, mock_w25q + addr, len);
    return 0;
}

static int erase(uint32_t addr, uint32_t size)
{
    if (addr % size || addr + size > MOCK_W25Q_BYTES) return -1;
    step();
    memset(mock_w25q + addr, 0xFF, size);
    return 0;
}

int w25q_erase_sector(uint32_t addr) { return erase(addr, W25Q_SECTOR); }
int w25q_erase_block(uint32_t addr)  { return erase(addr, W25Q_BLOCK); }

int w25q_program(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (addr + len > MOCK_W25Q_BYTES) return -1;
    while (len) {
        uint32_t room = W25Q_PAGE - (addr % W25Q_PAGE);
        uint32_t n = len < room ? len : room;
        step();
        for (uint32_t i = 0; i < n; i++) {
            if ((uint8_t)(buf[i] & ~mock_w25q[addr + i])) mock_w25q_bad_programs++;
            mock_w25q[addr + i] &= buf[i];
        }
        addr += n; buf += n; len -= n;
    }
    return 0;
}
