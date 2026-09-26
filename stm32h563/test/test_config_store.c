/* Host test for the power-loss-safe A/B config stores (src/config_store.c,
   src/cloud_config.c) on a RAM flash model (mocks/mock_flash.c). */
#include "config_store.h"
#include "cloud_config.h"
#include "mocks/mock_flash.h"

#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static config_t wifi(const char *ssid, const char *pw)
{
    config_t c;
    memset(&c, 0, sizeof(c));
    c.magic = CONFIG_MAGIC; c.version = CONFIG_VERSION;
    strncpy(c.ssid, ssid, sizeof(c.ssid) - 1);
    strncpy(c.password, pw, sizeof(c.password) - 1);
    return c;
}

/* The SSID the loader sees, "" for none. */
static const char *loaded_ssid(void)
{
    static config_t c;
    if (config_load(&c) != 0) return "";
    return c.ssid;
}

static uint32_t hdr_seq(uint32_t slot) { uint32_t v; memcpy(&v, mock_flash_ptr(slot + 4), 4); return v; }
static uint32_t hdr_magic(uint32_t slot) { uint32_t v; memcpy(&v, mock_flash_ptr(slot), 4); return v; }

/* Run config_save with a power cut after `cut` steps. Returns 1 if the cut happened. */
static int save_with_cut(const config_t *c, int cut)
{
    mock_flash_cut_after = cut;
    if (setjmp(mock_flash_power_jmp) == 0) {
        config_save(c);
        mock_flash_cut_after = 0;
        return 0;
    }
    mock_flash_cut_after = 0;
    return 1;
}

static int clear_with_cut(int cut)
{
    mock_flash_cut_after = cut;
    if (setjmp(mock_flash_power_jmp) == 0) {
        config_clear();
        mock_flash_cut_after = 0;
        return 0;
    }
    mock_flash_cut_after = 0;
    return 1;
}

static void test_round_trip_and_alternation(void)
{
    mock_flash_reset();
    CHECK(config_load(NULL) == -1, "blank flash: no config");

    config_t a = wifi("net-a", "pw-a"), b = wifi("net-b", "pw-b"), c = wifi("net-c", "pw-c");
    CHECK(config_save(&a) == 0, "save a");
    CHECK(strcmp(loaded_ssid(), "net-a") == 0, "load a, got '%s'", loaded_ssid());
    config_t got;
    CHECK(config_load(&got) == 0 && memcmp(&got, &a, sizeof(a)) == 0, "round trip is byte-exact");
    CHECK(hdr_magic(CONFIG_SLOT_A_OFFSET) == CONFIG_RECORD_MAGIC && hdr_seq(CONFIG_SLOT_A_OFFSET) == 1,
          "first save lands in slot A with seq 1");
    CHECK(hdr_magic(CONFIG_SLOT_B_OFFSET) == 0xFFFFFFFFu, "slot B untouched");

    CHECK(config_save(&b) == 0, "save b");
    CHECK(hdr_seq(CONFIG_SLOT_B_OFFSET) == 2, "second save in slot B, seq 2");
    CHECK(hdr_seq(CONFIG_SLOT_A_OFFSET) == 1, "slot A keeps the previous record");
    CHECK(strcmp(loaded_ssid(), "net-b") == 0, "load b");

    CHECK(config_save(&c) == 0, "save c");
    CHECK(hdr_seq(CONFIG_SLOT_A_OFFSET) == 3, "third save back in slot A, seq 3");
    CHECK(strcmp(loaded_ssid(), "net-c") == 0, "load c");

    int erases = mock_flash_erases;
    CHECK(config_save(&c) == 0, "save identical");
    CHECK(mock_flash_erases == erases, "identical save does not touch flash");

    /* magic/version are filled in by save even if the caller forgot them */
    config_t d = wifi("net-d", "");
    d.magic = 0; d.version = 0;
    CHECK(config_save(&d) == 0 && strcmp(loaded_ssid(), "net-d") == 0, "save fills magic/version");

    CHECK(mock_flash_double_programs == 0, "no quad-word programmed twice");
    CHECK(mock_flash_irq_depth == 0, "interrupts restored");
    CHECK(mock_flash_ecc_reads == 0, "no ECC reads in normal operation");
}

/* Power cut at every step of a save: the loader sees the old config or the new one,
   never nothing and never garbage; the next save then succeeds. */
static void test_torn_saves(void)
{
    config_t a = wifi("old-net", "old-pw"), b = wifi("new-net", "new-pw"), c = wifi("next-net", "x");
    for (int first_in_b = 0; first_in_b < 2; first_in_b++) {
        int total = 0;
        for (int cut = 1; ; cut++) {
            mock_flash_reset();
            if (first_in_b) { config_t z = wifi("older", "z"); config_save(&z); }  /* a lands in B */
            config_save(&a);
            int torn = save_with_cut(&b, cut);
            const char *s = loaded_ssid();
            if (!torn) { total = cut - 1; CHECK(strcmp(s, "new-net") == 0, "uncut save loads new"); break; }
            CHECK(strcmp(s, "old-net") == 0 || strcmp(s, "new-net") == 0,
                  "cut at step %d (%s): loaded '%s'", cut, first_in_b ? "into A" : "into B", s);
            CHECK(save_with_cut(&c, 0) == 0 && strcmp(loaded_ssid(), "next-net") == 0,
                  "cut at step %d: the next save recovers", cut);
            CHECK(mock_flash_double_programs == 0, "cut at step %d: no double program", cut);
        }
        CHECK(total >= 3, "a save takes erase + payload + header steps (%d)", total);
    }

    /* Payload tear: zero ECC reads by the loader (the no-hook crash is impossible here). */
    mock_flash_reset();
    config_save(&a);
    save_with_cut(&b, 3);   /* erase, payload QW 1, cut in payload QW 2 */
    int e = mock_flash_ecc_reads;
    CHECK(strcmp(loaded_ssid(), "old-net") == 0, "payload tear keeps old");
    CHECK(mock_flash_ecc_reads == e, "payload tear: the loader never reads the torn quad-word");

    /* Header tear: the loader hits the ECC error, treats the slot as invalid. */
    mock_flash_reset();
    config_save(&a);
    int steps = 1 + (int)((sizeof(config_t) + 15) / 16) + 1;
    CHECK(save_with_cut(&b, steps) == 1, "cut on the header program");
    e = mock_flash_ecc_reads;
    CHECK(strcmp(loaded_ssid(), "old-net") == 0, "header tear keeps old");
    CHECK(mock_flash_ecc_reads > e, "header tear is seen as an ECC error (NMI hook needed on HW)");
}

static void test_crc_rejection(void)
{
    config_t a = wifi("net-a", "pw"), b = wifi("net-b", "pw");
    mock_flash_reset();
    config_save(&a);
    config_save(&b);   /* b in slot B, seq 2 */
    mock_flash_ptr(CONFIG_SLOT_B_OFFSET + 16 + 10)[0] ^= 0x01;   /* bit flip in the payload */
    CHECK(strcmp(loaded_ssid(), "net-a") == 0, "corrupt newest -> previous slot, got '%s'", loaded_ssid());

    mock_flash_ptr(CONFIG_SLOT_A_OFFSET + 8)[0] ^= 0x01;          /* corrupt the len/version word */
    CHECK(config_load(NULL) == -1, "both corrupt, no legacy -> none");

    /* A corrupt newest slot is overwritten by the next save (it is not "the newest"). */
    config_t c = wifi("net-c", "pw");
    CHECK(config_save(&c) == 0 && strcmp(loaded_ssid(), "net-c") == 0, "save over corrupt slots");

    /* Seq comparison survives wrap-around. */
    mock_flash_reset();
    config_save(&a);
    config_save(&b);
    /* rewrite slot A's record with seq 0xFFFFFFFF and slot B's with 0 (0 is newer) */
    for (int s = 0; s < 2; s++) {
        uint32_t off = s ? CONFIG_SLOT_B_OFFSET : CONFIG_SLOT_A_OFFSET;
        uint8_t rec[16 + sizeof(config_t)];
        memcpy(rec, mock_flash_ptr(off), sizeof(rec));
        uint32_t seq = s ? 0u : 0xFFFFFFFFu, vl;
        memcpy(&vl, rec + 8, 4);
        uint32_t crc = ab_crc32(ab_crc32(ab_crc32(0, &seq, 4), &vl, 4), rec + 16, sizeof(config_t));
        memcpy(rec + 4, &seq, 4);
        memcpy(rec + 12, &crc, 4);
        memset(mock_flash_ptr(off), 0xFF, 8192);
        mock_flash_poke(off, rec, sizeof(rec));
    }
    CHECK(strcmp(loaded_ssid(), "net-b") == 0, "seq 0 is newer than 0xFFFFFFFF");
    CHECK(config_save(&c) == 0 && hdr_seq(CONFIG_SLOT_A_OFFSET) == 1, "save after wrap goes to slot A, seq 1");
    CHECK(strcmp(loaded_ssid(), "net-c") == 0, "after wrap");

    /* ECC error on the only slot: rejected, no crash. */
    mock_flash_reset();
    config_save(&a);
    mock_flash_ecc_bad[(CONFIG_SLOT_A_OFFSET - MOCK_FLASH_BASE_OFF) / 16 + 2] = 1;
    CHECK(config_load(NULL) == -1, "ECC error in the payload -> invalid");
}

static void test_legacy_migration(void)
{
    config_t old = wifi("legacy-net", "legacy-pw");
    mock_flash_reset();
    mock_flash_poke(CONFIG_FLASH_OFFSET, &old, sizeof(old));
    config_t got;
    CHECK(config_load(&got) == 0 && memcmp(&got, &old, sizeof(old)) == 0, "legacy layout still loads");

    config_t leg_wrongver = old; leg_wrongver.version = 99;
    mock_flash_reset();
    mock_flash_poke(CONFIG_FLASH_OFFSET, &leg_wrongver, sizeof(leg_wrongver));
    CHECK(config_load(NULL) == -1, "legacy with another schema version is ignored");

    mock_flash_reset();
    mock_flash_poke(CONFIG_FLASH_OFFSET, &old, sizeof(old));
    /* Re-saving the same config (e.g. a boot-time migration) writes the new format. */
    CHECK(config_save(&old) == 0, "save of the legacy config migrates it");
    CHECK(hdr_magic(CONFIG_SLOT_A_OFFSET) == CONFIG_RECORD_MAGIC, "now in slot A");
    CHECK(strcmp(loaded_ssid(), "legacy-net") == 0, "migrated config loads");
    config_t n = wifi("new-net", "pw");
    CHECK(config_save(&n) == 0 && strcmp(loaded_ssid(), "new-net") == 0, "new record beats legacy");

    /* A torn first save (legacy pod) falls back to the legacy copy. */
    for (int cut = 1; cut <= 13; cut++) {
        mock_flash_reset();
        mock_flash_poke(CONFIG_FLASH_OFFSET, &old, sizeof(old));
        int torn = save_with_cut(&n, cut);
        const char *s = loaded_ssid();
        CHECK(strcmp(s, torn ? "legacy-net" : "new-net") == 0 || strcmp(s, "new-net") == 0,
              "torn migration at step %d: got '%s'", cut, s);
    }

    /* ECC error in the legacy sector: rejected, no crash. */
    mock_flash_reset();
    mock_flash_poke(CONFIG_FLASH_OFFSET, &old, sizeof(old));
    mock_flash_ecc_bad[(CONFIG_FLASH_OFFSET - MOCK_FLASH_BASE_OFF) / 16 + 3] = 1;
    CHECK(config_load(NULL) == -1, "torn legacy sector -> none");
}

static void test_clear(void)
{
    config_t old = wifi("legacy-net", "pw"), a = wifi("net-a", "pw"), b = wifi("net-b", "pw");
    mock_flash_reset();
    mock_flash_poke(CONFIG_FLASH_OFFSET, &old, sizeof(old));
    config_save(&a);
    config_save(&b);
    config_clear();
    CHECK(config_load(NULL) == -1, "cleared");
    CHECK(*mock_flash_ptr(CONFIG_FLASH_OFFSET) == 0xFF, "clear erases the legacy sector");
    int wiped = 1;
    for (uint32_t i = 16; i < 16 + sizeof(config_t); i++) {
        if (*mock_flash_ptr(CONFIG_SLOT_A_OFFSET + i) != 0xFF && *mock_flash_ptr(CONFIG_SLOT_B_OFFSET + i) != 0xFF) wiped = 0;
    }
    CHECK(wiped, "no config payload survives a clear");
    CHECK(config_save(&a) == 0 && strcmp(loaded_ssid(), "net-a") == 0, "save after clear");

    /* A clear cut at any step: the loader sees b (not yet cleared) or nothing; never
       the older a or the legacy config. */
    for (int cut = 1; ; cut++) {
        mock_flash_reset();
        mock_flash_poke(CONFIG_FLASH_OFFSET, &old, sizeof(old));
        config_save(&a);
        config_save(&b);
        int torn = clear_with_cut(cut);
        const char *s = loaded_ssid();
        CHECK(strcmp(s, "net-b") == 0 || s[0] == '\0', "clear cut at step %d: got '%s'", cut, s);
        if (!torn) break;
    }
}

static void test_cloud_store(void)
{
    mock_flash_reset();
    CHECK(cloud_config_load(NULL) == -1, "cloud: blank");
    cloud_config_t c;
    memset(&c, 0, sizeof(c));
    strcpy(c.host, "api.embeddedci.com");
    strcpy(c.device_id, "0123e4567-e89b-12d3-a456-426614174000");
    c.port = 443; c.tls = 1; c.enabled = 1; c.verify = 1;
    CHECK(cloud_config_save(&c) == 0, "cloud: save");
    cloud_config_t got;
    CHECK(cloud_config_load(&got) == 0 && strcmp(got.host, c.host) == 0 && got.port == 443 &&
          got.magic == CLOUD_CONFIG_MAGIC, "cloud: round trip");
    CHECK(hdr_magic(CLOUD_CONFIG_SLOT_A_OFFSET) == CLOUD_CONFIG_RECORD_MAGIC, "cloud: slot A");
    CHECK(hdr_magic(CONFIG_SLOT_A_OFFSET) == 0xFFFFFFFFu, "cloud save does not touch the Wi-Fi slots");

    /* Legacy cloud config (verify=0 persisted by old firmware gets upgraded). */
    mock_flash_reset();
    cloud_config_t leg = c;
    leg.magic = CLOUD_CONFIG_MAGIC; leg.version = CLOUD_CONFIG_VERSION; leg.verify = 0;
    mock_flash_poke(CLOUD_CONFIG_FLASH_OFFSET, &leg, sizeof(leg));
    CHECK(cloud_config_load(&got) == 0 && got.verify == 1 && strcmp(got.device_id, c.device_id) == 0,
          "cloud: legacy loads, verify upgraded");

    /* Wi-Fi clear leaves the cloud config alone, and vice versa. */
    mock_flash_reset();
    config_t w = wifi("net", "pw");
    config_save(&w);
    cloud_config_save(&c);
    config_clear();
    CHECK(cloud_config_load(NULL) == 0, "wifi clear keeps the cloud config");
    config_save(&w);
    cloud_config_clear();
    CHECK(cloud_config_load(NULL) == -1 && config_load(NULL) == 0, "cloud clear keeps the Wi-Fi config");

    /* Torn cloud save keeps the old one. */
    cloud_config_t c2 = c;
    strcpy(c2.host, "other.example.com");
    for (int cut = 1; cut <= 12; cut++) {
        mock_flash_reset();
        cloud_config_save(&c);
        mock_flash_cut_after = cut;
        volatile int torn = 0;
        if (setjmp(mock_flash_power_jmp) == 0) cloud_config_save(&c2); else torn = 1;
        mock_flash_cut_after = 0;
        CHECK(cloud_config_load(&got) == 0, "cloud cut at %d: still configured", cut);
        CHECK(strcmp(got.host, torn ? c.host : c2.host) == 0 || strcmp(got.host, c2.host) == 0,
              "cloud cut at %d: host '%s'", cut, got.host);
    }
}

int main(void)
{
    test_round_trip_and_alternation();
    test_torn_saves();
    test_crc_rejection();
    test_legacy_migration();
    test_clear();
    test_cloud_store();
    if (fails) { printf("test_config_store: %d FAILED\n", fails); return 1; }
    printf("test_config_store: all passed\n");
    return 0;
}
