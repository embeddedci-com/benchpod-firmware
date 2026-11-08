/*
 * test_emi_log.c — host unit test for the EMI test build's run log (emi_log.h): record encode and
 * decode, CRC and blank detection, the newest-record scan, the ring wrap and erase rules, spoiled
 * slots, and the walk back for `emi log <n>`. The flash is a RAM array with NOR semantics
 * (programming only clears bits, erase sets a sector to 0xFF).
 */
#include "emi_log.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if(!(c)){printf("FAIL %s:%d  %s\n",__FILE__,__LINE__,#c);failures++;} } while(0)

#define BASE  EMI_LOG_BASE
static uint8_t flash[EMI_LOG_END - EMI_LOG_BASE];
static int     erases, fail_program_at = -1;

static int f_read(void *ctx, uint32_t addr, uint8_t *buf, uint32_t len) {
    (void)ctx;
    memcpy(buf, flash + (addr - BASE), len);
    return 0;
}
static int f_program(void *ctx, uint32_t addr, const uint8_t *buf, uint32_t len) {
    (void)ctx;
    for (uint32_t i = 0; i < len; i++) flash[addr - BASE + i] &= buf[i];   /* NOR: 1 -> 0 only */
    if ((int)addr == fail_program_at) flash[addr - BASE + 5] ^= 0x10;     /* a lost bit */
    return 0;
}
static int f_erase(void *ctx, uint32_t addr) {
    (void)ctx;
    CHECK((addr - BASE) % EMI_LOG_SECTOR == 0);
    memset(flash + (addr - BASE), 0xFF, EMI_LOG_SECTOR);
    erases++;
    return 0;
}
static const emi_log_io_t IO = { f_read, f_program, f_erase, NULL };

static void wipe(void) { memset(flash, 0xFF, sizeof(flash)); erases = 0; fail_program_at = -1; }

static emi_rec_t checkpoint(uint32_t up) {
    emi_rec_t r;
    memset(&r, 0, sizeof(r));
    r.type = EMI_REC_CHECKPOINT;
    r.led = 2;
    r.uptime_ms = up;
    r.u.cp.captures = up / 100u;
    return r;
}

static void test_encode_decode(void) {
    emi_rec_t a = { .type = EMI_REC_RUN_START, .led = 1, .seq = 7, .run = 3, .uptime_ms = 1234 };
    strcpy(a.u.start.fw, "3.9.6");
    strcpy(a.u.start.reset, "power-on");
    a.u.start.core_mhz = 250;
    a.u.start.dac = 4;
    a.u.start.net_mbit = 20;
    a.u.start.gw_version = 48;
    uint8_t buf[EMI_LOG_REC];
    emi_log_encode(&a, buf);
    CHECK(buf[0] == 'E' && buf[1] == 'L');
    emi_rec_t b;
    CHECK(emi_log_decode(buf, &b));
    CHECK(b.type == EMI_REC_RUN_START && b.seq == 7 && b.run == 3 && b.uptime_ms == 1234);
    CHECK(!strcmp(b.u.start.fw, "3.9.6") && !strcmp(b.u.start.reset, "power-on"));
    CHECK(b.u.start.core_mhz == 250 && b.u.start.dac == 4 && b.u.start.net_mbit == 20);
    CHECK(b.u.start.gw_version == 48);

    /* A known CRC (zlib.crc32 of the first 60 bytes): the host tools check the same one. */
    emi_rec_t z;
    memset(&z, 0, sizeof(z));
    z.type = EMI_REC_EVENT;
    emi_log_encode(&z, buf);
    uint32_t crc = (uint32_t)buf[60] | (uint32_t)buf[61] << 8 | (uint32_t)buf[62] << 16 | (uint32_t)buf[63] << 24;
    CHECK(crc == 0xA17A5D28u);   /* python3 -c "import zlib;print(hex(zlib.crc32(b'EL\\x02'+bytes(57))))" */

    /* Every single-bit flip is caught. */
    emi_log_encode(&a, buf);
    for (int i = 0; i < 64 * 8; i++) {
        buf[i / 8] ^= (uint8_t)(1u << (i % 8));
        CHECK(!emi_log_decode(buf, &b));
        buf[i / 8] ^= (uint8_t)(1u << (i % 8));
    }
    /* A long text is cut, NUL-terminated. */
    emi_rec_t e;
    memset(&e, 0, sizeof(e));
    e.type = EMI_REC_EVENT;
    e.u.ev.kind = EMI_EV_LED;
    e.u.ev.from = 2;
    e.u.ev.to = 3;
    memset(e.u.ev.text, 'x', sizeof(e.u.ev.text));   /* no NUL at all */
    emi_log_encode(&e, buf);
    CHECK(emi_log_decode(buf, &b));
    CHECK(strlen(b.u.ev.text) == sizeof(b.u.ev.text) - 1);

    uint8_t blank[EMI_LOG_REC];
    memset(blank, 0xFF, sizeof(blank));
    CHECK(emi_log_blank(blank));
    CHECK(!emi_log_decode(blank, &b));

    char line[200];
    b.seq = 9; b.run = 2; b.uptime_ms = 12345;
    emi_log_format(&b, line, sizeof(line));
    CHECK(strstr(line, "#9 run 2 +12.3s EVENT led green -> red") == line);
}

static void test_empty_scan_and_first_run(void) {
    wipe();
    static emi_log_t l;
    CHECK(emi_log_scan(&l, &IO, BASE, EMI_LOG_SECTORS) == 0);
    CHECK(!l.any && l.run == 1 && l.next_seq == 1 && l.write_slot == 0);
    CHECK(l.slots == 110u * 64u);
    CHECK(l.ready == l.slots);
    CHECK(emi_log_prepare(&l, 256) == 0);   /* all empty: nothing to erase */
    emi_rec_t r = checkpoint(1000);
    CHECK(emi_log_append(&l, &r) == 0);
    CHECK(r.seq == 1 && r.run == 1 && l.valid == 1);

    /* The next boot finds it and starts run 2 after it. */
    static emi_log_t m;
    CHECK(emi_log_scan(&m, &IO, BASE, EMI_LOG_SECTORS) == 0);
    CHECK(m.any && m.newest_seq == 1 && m.run == 2 && m.next_seq == 2 && m.write_slot == 1);
    CHECK(m.valid == 1);
}

/* Many runs on a small ring: it wraps, erases the oldest sector, and every reboot finds the newest
   record wherever it is. */
static void test_wrap(void) {
    wipe();
    static emi_log_t l;
    const uint32_t sectors = 4;   /* 256 slots */
    uint32_t seq_expected = 1;
    for (uint32_t run = 1; run <= 40; run++) {
        CHECK(emi_log_scan(&l, &IO, BASE, sectors) == 0);
        CHECK(l.run == run);
        CHECK(l.next_seq == seq_expected);
        CHECK(emi_log_prepare(&l, 64) >= 0);
        CHECK(l.ready >= 64);
        for (uint32_t i = 0; i < 37; i++) {   /* 37 records a run: not a sector multiple */
            emi_rec_t r = checkpoint(i * 1000u);
            if (l.ready == 0) CHECK(emi_log_prepare(&l, 1) == 1);
            CHECK(emi_log_append(&l, &r) == 0);
            CHECK(r.seq == seq_expected);
            seq_expected++;
        }
        CHECK(l.valid <= l.slots - 1u);
    }
    CHECK(erases > 0);
    /* Walking from the oldest slot meets the records in sequence order, ending at the newest. */
    uint32_t prev = 0, count = 0;
    const uint32_t start = emi_log_oldest_slot(&l);
    for (uint32_t i = 0; i < l.slots; i++) {
        const uint32_t slot = (start + i) % l.slots;
        if (!emi_log_slot_good(&l, slot)) continue;
        emi_rec_t r;
        CHECK(emi_log_decode(flash + slot * EMI_LOG_REC, &r));
        CHECK(r.seq > prev);
        prev = r.seq;
        count++;
    }
    CHECK(count == l.valid);
    CHECK(prev == seq_expected - 1u);
    /* At least three sectors' worth of history survive every wrap. */
    CHECK(l.valid >= 3u * 64u - 64u);
}

/* The prepare never erases the newest records, even when asked for everything. */
static void test_prepare_keeps_newest(void) {
    wipe();
    static emi_log_t l;
    CHECK(emi_log_scan(&l, &IO, BASE, 3) == 0);
    for (uint32_t i = 0; i < 3u * 64u - 10u; i++) {   /* nearly full */
        emi_rec_t r = checkpoint(i);
        CHECK(emi_log_append(&l, &r) == 0);
    }
    for (int boot = 0; boot < 6; boot++) {
        CHECK(emi_log_scan(&l, &IO, BASE, 3) == 0);
        const uint32_t newest = l.newest_seq;
        CHECK(emi_log_prepare(&l, 100000) >= 0);   /* capped at all but one sector */
        CHECK(l.ready <= l.slots);
        static emi_log_t m;
        CHECK(emi_log_scan(&m, &IO, BASE, 3) == 0);
        CHECK(m.any && m.newest_seq == newest);
        for (int i = 0; i < 50; i++) {
            emi_rec_t r = checkpoint((uint32_t)i);
            if (l.ready == 0) CHECK(emi_log_prepare(&l, 1) == 1);
            CHECK(emi_log_append(&l, &r) == 0);
        }
    }
}

/* A program that does not read back (power lost, a bad bit) spoils that slot only: the record
   goes to the next slot, and the scan skips the spoiled one. */
static void test_spoiled_slot(void) {
    wipe();
    static emi_log_t l;
    CHECK(emi_log_scan(&l, &IO, BASE, 4) == 0);
    for (int i = 0; i < 5; i++) { emi_rec_t r = checkpoint((uint32_t)i); CHECK(emi_log_append(&l, &r) == 0); }
    fail_program_at = (int)(BASE + 5 * EMI_LOG_REC);
    emi_rec_t r = checkpoint(99);
    CHECK(emi_log_append(&l, &r) == 0);
    CHECK(l.write_errors == 1 && l.write_slot == 7 && r.seq == 6);
    fail_program_at = -1;

    static emi_log_t m;
    CHECK(emi_log_scan(&m, &IO, BASE, 4) == 0);
    CHECK(m.newest_seq == 6 && m.write_slot == 7 && m.valid == 6);

    /* A spoiled slot ahead of the write position in its own sector: the write moves on to the
       next sector instead of erasing this one (which holds the newest records). */
    wipe();
    CHECK(emi_log_scan(&l, &IO, BASE, 4) == 0);
    for (int i = 0; i < 3; i++) { emi_rec_t q = checkpoint((uint32_t)i); CHECK(emi_log_append(&l, &q) == 0); }
    memset(flash + 10 * EMI_LOG_REC, 0x00, EMI_LOG_REC);   /* garbage at slot 10 */
    CHECK(emi_log_scan(&l, &IO, BASE, 4) == 0);
    CHECK(l.write_slot == 3 && l.ready == 7);
    CHECK(emi_log_prepare(&l, 64) == 0);   /* sector 1 is empty: just skip there */
    CHECK(l.write_slot == 64 && l.ready >= 64);
    CHECK(l.valid == 3);
}

static void test_slot_back(void) {
    wipe();
    static emi_log_t l;
    CHECK(emi_log_scan(&l, &IO, BASE, 2) == 0);
    for (int i = 0; i < 100; i++) { emi_rec_t r = checkpoint((uint32_t)i); CHECK(emi_log_append(&l, &r) == 0); }
    CHECK(emi_log_slot_back(&l, 1) == 99);
    CHECK(emi_log_slot_back(&l, 10) == 90);
    CHECK(emi_log_slot_back(&l, 100) == 0);
    CHECK(emi_log_slot_back(&l, 101) == l.slots);   /* fewer: walk from the oldest */
    CHECK(emi_log_oldest_slot(&l) == 0);
}

static void test_checkpoint_due(void) {
    emi_cp_sig_t a, b;
    memset(&a, 0, sizeof(a));
    a.led = 2; a.running = 1; a.link = 1;
    b = a;
    CHECK(!emi_log_checkpoint_due(&a, &b, 0));
    CHECK(!emi_log_checkpoint_due(&a, &b, EMI_LOG_CHECKPOINT_MIN_MS));        /* nothing changed */
    CHECK(!emi_log_checkpoint_due(&a, &b, EMI_LOG_HEARTBEAT_MS - 1));
    CHECK(emi_log_checkpoint_due(&a, &b, EMI_LOG_HEARTBEAT_MS));              /* the heartbeat */
    b.errors = 1;
    CHECK(!emi_log_checkpoint_due(&a, &b, EMI_LOG_CHECKPOINT_MIN_MS - 1));    /* not yet */
    CHECK(emi_log_checkpoint_due(&a, &b, EMI_LOG_CHECKPOINT_MIN_MS));
    b = a; b.net_drops = 3;     CHECK(emi_log_checkpoint_due(&a, &b, EMI_LOG_CHECKPOINT_MIN_MS));
    b = a; b.link_losses = 1;   CHECK(emi_log_checkpoint_due(&a, &b, EMI_LOG_CHECKPOINT_MIN_MS));
    b = a; b.led = 3;           CHECK(emi_log_checkpoint_due(&a, &b, EMI_LOG_CHECKPOINT_MIN_MS));
    b = a; b.link = 0;          CHECK(emi_log_checkpoint_due(&a, &b, EMI_LOG_CHECKPOINT_MIN_MS));
    b = a; b.glitch_us_max = 5; CHECK(emi_log_checkpoint_due(&a, &b, EMI_LOG_CHECKPOINT_MIN_MS));
    b = a; b.rec_dropped = 1;   CHECK(emi_log_checkpoint_due(&a, &b, EMI_LOG_CHECKPOINT_MIN_MS));
}

/* emi_log_slot_after: a viewer's "everything after seq N", across a wrap and a spoiled slot. */
static void test_slot_after(void) {
    wipe();
    emi_log_t l;
    CHECK(emi_log_scan(&l, &IO, BASE, 3) == 0);
    uint32_t newer;
    CHECK(emi_log_slot_after(&l, 0, &newer) == l.write_slot && newer == 0);   /* empty */
    CHECK(emi_log_prepare(&l, 2 * EMI_LOG_PER_SECTOR) >= 0);
    for (int i = 0; i < 10; i++) { emi_rec_t r = checkpoint((uint32_t)i); CHECK(emi_log_append(&l, &r) == 0); }
    /* seq 1..10 in slots 0..9 */
    CHECK(emi_log_slot_after(&l, 10, &newer) == l.write_slot && newer == 0);  /* has them all */
    CHECK(emi_log_slot_after(&l, 99, &newer) == l.write_slot && newer == 0);  /* ahead: none */
    CHECK(emi_log_slot_after(&l, 7, &newer) == 7 && newer == 3);
    CHECK(emi_log_slot_after(&l, 0, &newer) == 0 && newer == 10);
    /* Wrap: 3 sectors, write until the first sector was erased and reused. */
    for (int i = 0; i < 3 * (int)EMI_LOG_PER_SECTOR; i++) {
        if (l.ready == 0) CHECK(emi_log_prepare(&l, EMI_LOG_PER_SECTOR) >= 0);
        emi_rec_t r = checkpoint((uint32_t)i);
        CHECK(emi_log_append(&l, &r) == 0);
    }
    const uint32_t newest = l.next_seq - 1u;
    uint32_t slot = emi_log_slot_after(&l, newest - 5, &newer);
    CHECK(newer == 5);
    emi_rec_t r;
    uint8_t buf[EMI_LOG_REC];
    f_read(NULL, emi_log_slot_addr(&l, slot), buf, EMI_LOG_REC);
    CHECK(emi_log_decode(buf, &r) && r.seq == newest - 4);
    /* Older than everything still there: the oldest record. */
    slot = emi_log_slot_after(&l, 1, &newer);
    CHECK(slot == emi_log_oldest_slot(&l) && newer == l.valid);
    f_read(NULL, emi_log_slot_addr(&l, slot), buf, EMI_LOG_REC);
    CHECK(emi_log_decode(buf, &r) && r.seq == newest - l.valid + 1);
}

static void test_summary_round_trip(void) {
    emi_rec_t a, b;
    memset(&a, 0, sizeof(a));
    a.type = EMI_REC_SUMMARY;
    a.seq = 77; a.run = 3; a.uptime_ms = 600000;
    a.u.sum.cloud_drops = 8; a.u.sum.c3_restarts = 2; a.u.sum.i2c_clears = 1; a.u.sum.can_busoff = 0;
    a.u.sum.psram_fail = 0; a.u.sum.cdone_lost = 1; a.u.sum.psram_retry = 4;
    a.u.sum.rev_block[0] = 31; a.u.sum.rev_block[1] = 0; a.u.sum.eth_crc = 5; a.u.sum.eth_missed = 0;
    a.u.sum.bridge_tx_err = 977642; a.u.sum.cloud_down_s = 125;
    a.u.sum.bridge_restarts = 2; a.u.sum.bridge_silences = 1;
    uint8_t raw[EMI_LOG_REC];
    emi_log_encode(&a, raw);
    CHECK(emi_log_decode(raw, &b));
    CHECK(b.type == EMI_REC_SUMMARY && b.seq == 77 && b.run == 3);
    CHECK(memcmp(&a.u.sum, &b.u.sum, sizeof(a.u.sum)) == 0);
    char line[320];
    CHECK(emi_log_format(&b, line, sizeof(line)) > 0);
    CHECK(strstr(line, "#77 run 3 +600.0s SUM cloud drops 8 (down 125 s), c3 restarts 2, i2c clears 1") == line);
    CHECK(strstr(line, "eFuse rev blocks 31/0, eth crc 5 missed 0, bridge tx err 977642, bridge restarts 2 silent 1") != NULL);
    raw[2] = 5;   /* an unknown type stays unknown */
    { uint32_t c = 0xFFFFFFFFu; for (int i = 0; i < 60; i++) { c ^= raw[i]; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))); } c = ~c; raw[60] = (uint8_t)c; raw[61] = (uint8_t)(c >> 8); raw[62] = (uint8_t)(c >> 16); raw[63] = (uint8_t)(c >> 24); }
    CHECK(!emi_log_decode(raw, &b));
}

int main(void) {
    test_summary_round_trip();
    test_encode_decode();
    test_empty_scan_and_first_run();
    test_wrap();
    test_prepare_keeps_newest();
    test_spoiled_slot();
    test_slot_back();
    test_checkpoint_due();
    test_slot_after();
    if (failures) { printf("test_emi_log: %d FAILED\n", failures); return 1; }
    printf("test_emi_log: all passed\n");
    return 0;
}
