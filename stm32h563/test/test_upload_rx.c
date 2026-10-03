/* Host test for upload_rx: the raw chunk receive behind the USB console upload-data command. */
#include "upload_rx.h"
#include <stdio.h>
#include <string.h>

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL %s:%d: ", __func__, __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

int main(void) {
    static upload_rx_t u;
    uint8_t data[512];
    for (int i = 0; i < 512; i++) data[i] = (uint8_t)(i * 7 + 3);   /* includes '\r' and '\n' */
    char args[64];

    /* Known answer: the standard CRC-32 of "123456789" is cbf43926. */
    CHECK(upload_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u, "crc32 known answer");

    snprintf(args, sizeof(args), "4096 512 %08x", (unsigned)upload_crc32(data, 512));
    CHECK(upload_rx_start(&u, args, 100) == 0 && u.active, "start");
    for (int i = 0; i < 511; i++) CHECK(!upload_rx_byte(&u, data[i], 100), "complete early at %d", i);
    CHECK(upload_rx_byte(&u, data[511], 101), "not complete after 512 bytes");
    CHECK(!u.active && u.offset == 4096 && upload_rx_crc_ok(&u), "chunk not accepted");
    CHECK(!upload_rx_byte(&u, 0x55, 102), "bytes after completion consumed");

    snprintf(args, sizeof(args), "0 16 %08x", (unsigned)upload_crc32(data, 16));
    upload_rx_start(&u, args, 0);
    for (int i = 0; i < 16; i++) upload_rx_byte(&u, i == 5 ? (uint8_t)(data[i] ^ 1) : data[i], 0);
    CHECK(!upload_rx_crc_ok(&u), "a corrupted chunk passed its CRC");

    /* Timeout: stalls stop receiving; progress keeps it alive. */
    upload_rx_start(&u, "0 4 0", 1000);
    upload_rx_byte(&u, 1, 2500);
    CHECK(!upload_rx_timed_out(&u, 4000), "timed out while bytes were arriving");
    CHECK(upload_rx_timed_out(&u, 4600) && !u.active, "a stalled chunk did not time out");
    CHECK(!upload_rx_timed_out(&u, 9000), "timed out twice");

    /* Bad arguments. */
    CHECK(upload_rx_start(&u, "0 513 0", 0) != 0, "a chunk over the limit was accepted");
    CHECK(upload_rx_start(&u, "0 0 0", 0) != 0, "an empty chunk was accepted");
    CHECK(upload_rx_start(&u, "abc", 0) != 0, "garbage accepted");
    CHECK(upload_rx_start(&u, "0 4", 0) != 0, "missing crc accepted");
    CHECK(upload_rx_start(&u, "0 4 1f junk", 0) != 0, "trailing junk accepted");
    CHECK(!u.active, "left receiving after bad arguments");
    CHECK(upload_rx_start(&u, "0x100 4 deadbeef", 0) == 0 && u.offset == 0x100 && u.crc == 0xDEADBEEFu,
          "hex offset / crc");

    if (fails) { printf("test_upload_rx: %d FAILED\n", fails); return 1; }
    printf("test_upload_rx: all passed\n");
    return 0;
}
