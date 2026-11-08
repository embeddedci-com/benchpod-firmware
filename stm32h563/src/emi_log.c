/*
 * emi_log.c — the EMI test build's run log: record format and ring logic (emi_log.h).
 */
#include "emi_log.h"

#include <stdio.h>
#include <string.h>

/* zlib's CRC-32 (Python: zlib.crc32), bitwise: 60 bytes per record. */
static uint32_t crc32_le(const uint8_t *p, size_t n) {
    uint32_t crc = 0xFFFFFFFFu;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
/* A fixed-size text field: copied up to n bytes, NUL-padded; read back NUL-terminated. */
static void put_str(uint8_t *p, const char *s, size_t n) {
    memset(p, 0, n);
    if (s) for (size_t i = 0; i < n && s[i]; i++) p[i] = (uint8_t)s[i];
}
static void get_str(const uint8_t *p, char *out, size_t n) {
    size_t i = 0;
    for (; i < n - 1 && p[i]; i++) out[i] = (char)p[i];
    out[i] = '\0';
}

/* Layout (little-endian):
     0 u16 magic  2 u8 type  3 u8 led  4 u32 seq  8 u32 run  12 u32 uptime_ms
    16 payload (44 bytes)    60 u32 crc32 of bytes 0-59
   RUN_START:  16 fw[16] 32 reset[16] 48 u16 core_mhz 50 u8 dac 51 u8 net_mbit 52 u32 gw_version
   EVENT:      16 u8 kind 17 u8 from 18 u8 to 20 u32 captures 24 u32 errors 28 u32 net_sent
               32 text[28]
   CHECKPOINT: 16 u32 captures 20 errors 24 net_sent 28 net_drops 32 link_losses 36 rec_dropped
               40 glitch_us_max 44 u8 dac 45 net_mbit 46 running 47 link 48 text[12] */
void emi_log_encode(const emi_rec_t *r, uint8_t out[EMI_LOG_REC]) {
    memset(out, 0, EMI_LOG_REC);
    put16(out, EMI_LOG_MAGIC);
    out[2] = r->type;
    out[3] = r->led;
    put32(out + 4, r->seq);
    put32(out + 8, r->run);
    put32(out + 12, r->uptime_ms);
    switch (r->type) {
    case EMI_REC_RUN_START:
        put_str(out + 16, r->u.start.fw, 16);
        put_str(out + 32, r->u.start.reset, 16);
        put16(out + 48, r->u.start.core_mhz);
        out[50] = r->u.start.dac;
        out[51] = r->u.start.net_mbit;
        put32(out + 52, r->u.start.gw_version);
        break;
    case EMI_REC_EVENT:
        out[16] = r->u.ev.kind;
        out[17] = r->u.ev.from;
        out[18] = r->u.ev.to;
        put32(out + 20, r->u.ev.captures);
        put32(out + 24, r->u.ev.errors);
        put32(out + 28, r->u.ev.net_sent);
        put_str(out + 32, r->u.ev.text, 28);
        break;
    case EMI_REC_CHECKPOINT:
        put32(out + 16, r->u.cp.captures);
        put32(out + 20, r->u.cp.errors);
        put32(out + 24, r->u.cp.net_sent);
        put32(out + 28, r->u.cp.net_drops);
        put32(out + 32, r->u.cp.link_losses);
        put32(out + 36, r->u.cp.rec_dropped);
        put32(out + 40, r->u.cp.glitch_us_max);
        out[44] = r->u.cp.dac;
        out[45] = r->u.cp.net_mbit;
        out[46] = r->u.cp.running;
        out[47] = r->u.cp.link;
        put_str(out + 48, r->u.cp.text, 12);
        break;
    case EMI_REC_SUMMARY:
        put16(out + 16, r->u.sum.cloud_drops);
        put16(out + 18, r->u.sum.c3_restarts);
        put16(out + 20, r->u.sum.i2c_clears);
        put16(out + 22, r->u.sum.can_busoff);
        put16(out + 24, r->u.sum.psram_fail);
        put16(out + 26, r->u.sum.cdone_lost);
        put32(out + 28, r->u.sum.psram_retry);
        put32(out + 32, r->u.sum.rev_block[0]);
        put32(out + 36, r->u.sum.rev_block[1]);
        put32(out + 40, r->u.sum.eth_crc);
        put32(out + 44, r->u.sum.eth_missed);
        put32(out + 48, r->u.sum.bridge_tx_err);
        put32(out + 52, r->u.sum.cloud_down_s);
        break;
    }
    put32(out + 60, crc32_le(out, 60));
}

bool emi_log_checkpoint_due(const emi_cp_sig_t *last, const emi_cp_sig_t *now, uint32_t since_ms) {
    if (since_ms >= EMI_LOG_HEARTBEAT_MS) return true;
    if (since_ms < EMI_LOG_CHECKPOINT_MIN_MS) return false;
    return last->errors != now->errors || last->net_drops != now->net_drops ||
           last->link_losses != now->link_losses || last->rec_dropped != now->rec_dropped ||
           last->glitch_us_max != now->glitch_us_max || last->led != now->led ||
           last->dac != now->dac || last->running != now->running || last->link != now->link;
}

bool emi_log_blank(const uint8_t in[EMI_LOG_REC]) {
    for (unsigned i = 0; i < EMI_LOG_REC; i++) if (in[i] != 0xFF) return false;
    return true;
}

bool emi_log_decode(const uint8_t in[EMI_LOG_REC], emi_rec_t *r) {
    if (get16(in) != EMI_LOG_MAGIC) return false;
    if (get32(in + 60) != crc32_le(in, 60)) return false;
    if (in[2] < EMI_REC_RUN_START || in[2] > EMI_REC_SUMMARY) return false;
    memset(r, 0, sizeof(*r));
    r->type = in[2];
    r->led = in[3];
    r->seq = get32(in + 4);
    r->run = get32(in + 8);
    r->uptime_ms = get32(in + 12);
    switch (r->type) {
    case EMI_REC_RUN_START:
        get_str(in + 16, r->u.start.fw, 16);
        get_str(in + 32, r->u.start.reset, 16);
        r->u.start.core_mhz = get16(in + 48);
        r->u.start.dac = in[50];
        r->u.start.net_mbit = in[51];
        r->u.start.gw_version = get32(in + 52);
        break;
    case EMI_REC_EVENT:
        r->u.ev.kind = in[16];
        r->u.ev.from = in[17];
        r->u.ev.to = in[18];
        r->u.ev.captures = get32(in + 20);
        r->u.ev.errors = get32(in + 24);
        r->u.ev.net_sent = get32(in + 28);
        get_str(in + 32, r->u.ev.text, 28);
        break;
    case EMI_REC_SUMMARY:
        r->u.sum.cloud_drops = get16(in + 16);
        r->u.sum.c3_restarts = get16(in + 18);
        r->u.sum.i2c_clears = get16(in + 20);
        r->u.sum.can_busoff = get16(in + 22);
        r->u.sum.psram_fail = get16(in + 24);
        r->u.sum.cdone_lost = get16(in + 26);
        r->u.sum.psram_retry = get32(in + 28);
        r->u.sum.rev_block[0] = get32(in + 32);
        r->u.sum.rev_block[1] = get32(in + 36);
        r->u.sum.eth_crc = get32(in + 40);
        r->u.sum.eth_missed = get32(in + 44);
        r->u.sum.bridge_tx_err = get32(in + 48);
        r->u.sum.cloud_down_s = get32(in + 52);
        break;
    default:
        r->u.cp.captures = get32(in + 16);
        r->u.cp.errors = get32(in + 20);
        r->u.cp.net_sent = get32(in + 24);
        r->u.cp.net_drops = get32(in + 28);
        r->u.cp.link_losses = get32(in + 32);
        r->u.cp.rec_dropped = get32(in + 36);
        r->u.cp.glitch_us_max = get32(in + 40);
        r->u.cp.dac = in[44];
        r->u.cp.net_mbit = in[45];
        r->u.cp.running = in[46];
        r->u.cp.link = in[47];
        get_str(in + 48, r->u.cp.text, 12);
        break;
    }
    return true;
}

/* status_led_emi_t, by value (status_led.h); this file does not include it, so the host tests
   need no LED code. */
const char *emi_log_led_name(uint8_t led) {
    switch (led) {
    case 1:  return "starting";
    case 2:  return "green";
    case 3:  return "red";
    case 4:  return "blue";
    case 5:  return "recovering";
    default: return "off";
    }
}

static const char *dac_name(uint8_t d) {
    static const char *const k[] = { "sine", "dc", "current", "off", "all" };
    return d < 5 ? k[d] : "?";
}

size_t emi_log_format(const emi_rec_t *r, char *buf, size_t cap) {
    int n = snprintf(buf, cap, "#%lu run %lu +%lu.%lus ", (unsigned long)r->seq,
                     (unsigned long)r->run, (unsigned long)(r->uptime_ms / 1000u),
                     (unsigned long)(r->uptime_ms % 1000u / 100u));
    if (n < 0 || (size_t)n >= cap) return 0;
    int m = 0;
    switch (r->type) {
    case EMI_REC_RUN_START:
        m = snprintf(buf + n, cap - (size_t)n,
                     "START fw %s reset %s core %u MHz dac %s net %u Mbit/s gw %lu",
                     r->u.start.fw, r->u.start.reset, (unsigned)r->u.start.core_mhz,
                     dac_name(r->u.start.dac), (unsigned)r->u.start.net_mbit,
                     (unsigned long)r->u.start.gw_version);
        break;
    case EMI_REC_EVENT: {
        char what[40];
        switch (r->u.ev.kind) {
        case EMI_EV_LED:
            snprintf(what, sizeof(what), "led %s -> %s", emi_log_led_name(r->u.ev.from),
                     emi_log_led_name(r->u.ev.to));
            break;
        case EMI_EV_LINK_LOST: snprintf(what, sizeof(what), "ethernet link lost"); break;
        case EMI_EV_LINK_BACK: snprintf(what, sizeof(what), "ethernet link back"); break;
        case EMI_EV_STARTED:   snprintf(what, sizeof(what), "activity started"); break;
        case EMI_EV_STOPPED:   snprintf(what, sizeof(what), "activity stopped"); break;
        case EMI_EV_C3_RESTART: snprintf(what, sizeof(what), "ESP32-C3 restart"); break;
        case EMI_EV_HEALTH:    snprintf(what, sizeof(what), "health"); break;
        default:               snprintf(what, sizeof(what), "event %u", (unsigned)r->u.ev.kind); break;
        }
        if (r->u.ev.kind == EMI_EV_RECOVERY)   /* the text is the step itself (emi_log_dump.py) */
            m = snprintf(buf + n, cap - (size_t)n, "EVENT recovery: %s, captures %lu, errors %lu",
                         r->u.ev.text, (unsigned long)r->u.ev.captures, (unsigned long)r->u.ev.errors);
        else
            m = snprintf(buf + n, cap - (size_t)n, "EVENT %s, captures %lu, errors %lu, sent %lu%s%s",
                         what, (unsigned long)r->u.ev.captures, (unsigned long)r->u.ev.errors,
                         (unsigned long)r->u.ev.net_sent, r->u.ev.text[0] ? ", last: " : "",
                         r->u.ev.text);
        break;
    }
    case EMI_REC_SUMMARY:
        m = snprintf(buf + n, cap - (size_t)n,
                     "SUM cloud drops %u (down %lu s), c3 restarts %u, i2c clears %u, can bus-off %u, "
                     "psram retry %lu fail %u, iCE40 lost %u, eFuse rev blocks %lu/%lu, eth crc %lu "
                     "missed %lu, bridge tx err %lu",
                     (unsigned)r->u.sum.cloud_drops, (unsigned long)r->u.sum.cloud_down_s,
                     (unsigned)r->u.sum.c3_restarts, (unsigned)r->u.sum.i2c_clears,
                     (unsigned)r->u.sum.can_busoff, (unsigned long)r->u.sum.psram_retry,
                     (unsigned)r->u.sum.psram_fail, (unsigned)r->u.sum.cdone_lost,
                     (unsigned long)r->u.sum.rev_block[0], (unsigned long)r->u.sum.rev_block[1],
                     (unsigned long)r->u.sum.eth_crc, (unsigned long)r->u.sum.eth_missed,
                     (unsigned long)r->u.sum.bridge_tx_err);
        break;
    default:
        m = snprintf(buf + n, cap - (size_t)n,
                     "CHECK led %s, %s, dac %s, captures %lu, errors %lu, sent %lu, drops %lu, "
                     "link %s, link losses %lu, glitch max %lu us, lost records %lu%s%s",
                     emi_log_led_name(r->led), r->u.cp.running ? "running" : "stopped",
                     dac_name(r->u.cp.dac), (unsigned long)r->u.cp.captures,
                     (unsigned long)r->u.cp.errors, (unsigned long)r->u.cp.net_sent,
                     (unsigned long)r->u.cp.net_drops, r->u.cp.link ? "up" : "down",
                     (unsigned long)r->u.cp.link_losses, (unsigned long)r->u.cp.glitch_us_max,
                     (unsigned long)r->u.cp.rec_dropped, r->u.cp.text[0] ? ", last: " : "",
                     r->u.cp.text);
        break;
    }
    if (m < 0) return (size_t)n;
    size_t len = (size_t)n + (size_t)m;
    return len < cap ? len : cap - 1;
}

/* ---- the ring ---------------------------------------------------------------------------- */

static bool bit(const uint8_t *map, uint32_t i) { return (map[i >> 3] >> (i & 7u)) & 1u; }
static void bit_set(uint8_t *map, uint32_t i, bool v) {
    if (v) map[i >> 3] |= (uint8_t)(1u << (i & 7u));
    else   map[i >> 3] &= (uint8_t)~(1u << (i & 7u));
}

uint32_t emi_log_slot_addr(const emi_log_t *l, uint32_t slot) { return l->base + slot * EMI_LOG_REC; }
bool emi_log_slot_good(const emi_log_t *l, uint32_t slot) { return bit(l->good, slot % l->slots); }

/* Count the empty slots from the write position on. */
static void count_ready(emi_log_t *l) {
    l->ready = 0;
    while (l->ready < l->slots && bit(l->blank, (l->write_slot + l->ready) % l->slots)) l->ready++;
}

int emi_log_scan(emi_log_t *l, const emi_log_io_t *io, uint32_t base, uint32_t sectors) {
    memset(l, 0, sizeof(*l));
    l->io = *io;
    l->base = base;
    if (sectors > EMI_LOG_MAX_SLOTS / EMI_LOG_PER_SECTOR) sectors = EMI_LOG_MAX_SLOTS / EMI_LOG_PER_SECTOR;
    if (sectors < 2) return -1;   /* the ring keeps one sector of records while it erases another */
    l->slots = sectors * EMI_LOG_PER_SECTOR;
    uint32_t newest_slot = 0;
    uint8_t chunk[4u * EMI_LOG_REC];   /* small: this runs on the hw worker's stack */
    for (uint32_t c = 0; c < l->slots / 4u; c++) {
        if (io->read(io->ctx, base + c * sizeof(chunk), chunk, sizeof(chunk)) != 0) return -2;
        for (uint32_t k = 0; k < 4u; k++) {
            const uint32_t slot = c * 4u + k;
            const uint8_t *p = chunk + k * EMI_LOG_REC;
            emi_rec_t r;
            if (emi_log_blank(p)) {
                bit_set(l->blank, slot, true);
            } else if (emi_log_decode(p, &r)) {
                bit_set(l->good, slot, true);
                l->valid++;
                if (!l->any || r.seq > l->newest_seq) {
                    l->any = true;
                    l->newest_seq = r.seq;
                    l->newest_run = r.run;
                    newest_slot = slot;
                }
            }
        }
    }
    l->write_slot = l->any ? (newest_slot + 1u) % l->slots : 0u;
    l->next_seq = l->any ? l->newest_seq + 1u : 1u;
    l->run = l->any ? l->newest_run + 1u : 1u;
    count_ready(l);
    l->ok = true;
    return 0;
}

static int erase_sector(emi_log_t *l, uint32_t sector) {
    const uint32_t addr = l->base + sector * EMI_LOG_SECTOR;
    if (l->io.erase_sector(l->io.ctx, addr) != 0) return -2;
    l->erases++;
    for (uint32_t k = 0; k < EMI_LOG_PER_SECTOR; k++) {
        const uint32_t slot = sector * EMI_LOG_PER_SECTOR + k;
        if (bit(l->good, slot)) l->valid--;
        bit_set(l->good, slot, false);
        bit_set(l->blank, slot, true);
    }
    return 0;
}

int emi_log_prepare(emi_log_t *l, uint32_t min_ready) {
    if (!l->ok) return -1;
    const uint32_t max_ready = l->slots - EMI_LOG_PER_SECTOR;
    if (min_ready > max_ready) min_ready = max_ready;
    int erased = 0;
    for (uint32_t guard = 0; l->ready < min_ready && guard <= l->slots / EMI_LOG_PER_SECTOR + 1u; guard++) {
        const uint32_t stop = (l->write_slot + l->ready) % l->slots;   /* the first slot in use */
        const uint32_t sector = stop / EMI_LOG_PER_SECTOR;
        /* The slot in use is in the write position's own sector, after records of this sector
           (a slot spoiled by a lost program): erasing the sector would erase the newest records,
           so continue at the start of the next sector. The cap above keeps the sector of the
           newest record when the write position is at a sector start. */
        if (sector == l->write_slot / EMI_LOG_PER_SECTOR && l->write_slot % EMI_LOG_PER_SECTOR != 0) {
            l->write_slot = (sector + 1u) * EMI_LOG_PER_SECTOR % l->slots;
            count_ready(l);
            continue;
        }
        int rc = erase_sector(l, sector);
        if (rc) return rc;
        erased++;
        count_ready(l);
    }
    return erased;
}

int emi_log_append(emi_log_t *l, emi_rec_t *r) {
    if (!l->ok) return -2;
    for (int tries = 0; tries < 2; tries++) {
        if (l->ready == 0) return -1;
        const uint32_t slot = l->write_slot;
        r->seq = l->next_seq;
        r->run = l->run;
        uint8_t buf[EMI_LOG_REC], back[EMI_LOG_REC];
        emi_log_encode(r, buf);
        const uint32_t addr = emi_log_slot_addr(l, slot);
        int prc = l->io.program(l->io.ctx, addr, buf, EMI_LOG_REC);
        int rrc = prc ? -1 : l->io.read(l->io.ctx, addr, back, EMI_LOG_REC);
        bit_set(l->blank, slot, false);
        l->write_slot = (slot + 1u) % l->slots;
        l->ready--;
        if (!prc && !rrc && !memcmp(buf, back, EMI_LOG_REC)) {
            bit_set(l->good, slot, true);
            l->valid++;
            l->next_seq++;
            l->any = true;
            l->newest_seq = r->seq;
            l->newest_run = r->run;
            return 0;
        }
        l->write_errors++;   /* the slot is spoiled; the next one gets the record */
    }
    return -2;
}

uint32_t emi_log_oldest_slot(const emi_log_t *l) {
    return (l->write_slot + l->ready) % l->slots;
}

uint32_t emi_log_slot_after(const emi_log_t *l, uint32_t after, uint32_t *newer) {
    const uint32_t newest = l->any ? l->next_seq - 1u : 0u;
    uint32_t n = (l->any && (int32_t)(newest - after) > 0) ? newest - after : 0u;
    if (n > l->valid) n = l->valid;
    *newer = n;
    if (!n) return l->write_slot;
    uint32_t slot = emi_log_slot_back(l, n);
    if (slot >= l->slots) slot = emi_log_oldest_slot(l);
    return slot;
}

uint32_t emi_log_slot_back(const emi_log_t *l, uint32_t n) {
    const uint32_t span = l->slots - l->ready;   /* the slots from the oldest to the newest */
    uint32_t found = 0;
    for (uint32_t i = 1; i <= span; i++) {
        const uint32_t slot = (l->write_slot + l->slots - i) % l->slots;
        if (bit(l->good, slot) && ++found == n) return slot;
    }
    return l->slots;
}
