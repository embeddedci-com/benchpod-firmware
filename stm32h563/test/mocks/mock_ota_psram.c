/*
 * mock_ota_psram.c — host-test stand-in for the PSRAM staging buffer and the bus arbitration
 * ota.c runs on.
 *
 * ota.c's job is bookkeeping over a big external buffer: track what has arrived, refuse what does
 * not fit, and hash back exactly what was staged. All of that is testable on the host once the
 * buffer is plain RAM — which is what this provides. It also counts acquire/release so a test can
 * assert the shared bus is never left held, and can be told to fail a write or a read so the
 * error paths are exercised without real hardware faults.
 */
#include "mock_ota_psram.h"

#include <string.h>

uint8_t mock_psram[MOCK_PSRAM_BYTES];
int mock_psram_acquired;       /* current depth: must be 0 between operations */
int mock_psram_acquire_count;
int mock_psram_fail_write_at = -1;  /* offset whose write fails, or -1 */
int mock_psram_fail_read_at  = -1;  /* offset whose read fails, or -1 */
int mock_quiesce_calls;

void mock_ota_psram_reset(void) {
    memset(mock_psram, 0, sizeof(mock_psram));
    mock_psram_acquired = 0;
    mock_psram_acquire_count = 0;
    mock_psram_fail_write_at = -1;
    mock_psram_fail_read_at = -1;
    mock_quiesce_calls = 0;
}

void psram_bus_acquire(void) { mock_psram_acquired++; mock_psram_acquire_count++; }
void psram_bus_release(void) { mock_psram_acquired--; }

int psram_write(uint32_t addr, const uint8_t *buf, uint32_t len) {
    if (mock_psram_fail_write_at >= 0 &&
        addr <= (uint32_t)mock_psram_fail_write_at &&
        (uint32_t)mock_psram_fail_write_at < addr + len) {
        return -1;
    }
    if (addr + len > MOCK_PSRAM_BYTES) return -1;
    memcpy(&mock_psram[addr], buf, len);
    return 0;
}

int psram_read(uint32_t addr, uint8_t *buf, uint32_t len) {
    if (mock_psram_fail_read_at >= 0 &&
        addr <= (uint32_t)mock_psram_fail_read_at &&
        (uint32_t)mock_psram_fail_read_at < addr + len) {
        return -1;
    }
    if (addr + len > MOCK_PSRAM_BYTES) return -1;
    memcpy(buf, &mock_psram[addr], len);
    return 0;
}

/* ota.c stops the gateware's PSRAM masters for the duration of the session. */
void signal_engine_quiesce_psram_masters(void) { mock_quiesce_calls++; }
