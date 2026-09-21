#ifndef MOCK_OTA_PSRAM_H
#define MOCK_OTA_PSRAM_H

#include <stdint.h>

/* Large enough for the OTA images the tests stage (the real part is 8 MB). */
#define MOCK_PSRAM_BYTES (256u * 1024u)

extern uint8_t mock_psram[MOCK_PSRAM_BYTES];
extern int mock_psram_acquired;         /* nesting depth; 0 when the bus is free */
extern int mock_psram_acquire_count;    /* total acquires, for "the bus was used" checks */
extern int mock_psram_fail_write_at;    /* make the write covering this offset fail (-1 = never) */
extern int mock_psram_fail_read_at;     /* make the read covering this offset fail (-1 = never) */
extern int mock_quiesce_calls;

void mock_ota_psram_reset(void);

#endif /* MOCK_OTA_PSRAM_H */
