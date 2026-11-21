/* Host test for ser_flash (xip_mod_ser_flash.c, ser_flash_names.c) against simulated bootloaders on
   a USB serial port: optiboot (STK500v1, Uno), ATmegaBOOT behind a CH340 at 57600 (old Nano), the
   Mega 2560's STK500v2 bootloader, Caterina (AVR109, Leonardo: 1200 baud touch, comes back with the
   bootloader's PID), and the Arduino SAMD bootloader (SAM-BA with X/Y, Zero: 1200 baud touch).
   Then refusals and faults, and random replies. */
#include "ser_flash.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

/* ---- simulated board ----------------------------------------------------------------------- */

enum { DEV_OPTIBOOT, DEV_V2, DEV_CATERINA, DEV_SAMBA, DEV_SILENT, DEV_RANDOM };

#define FLASH_MAX (256u * 1024u)

typedef struct {
    int      kind;
    uint8_t  flash[FLASH_MAX];
    uint32_t flash_size;
    uint8_t  sig[3];
    uint32_t rate;                /* the rate the bootloader listens at (0 = native USB: any) */
    bool     bridge;
    /* USB */
    uint16_t vid, pid, boot_pid;
    uint32_t mounts;
    bool     mounted;
    bool     in_loader;           /* AVR: reset into the bootloader / SAMD, AVR109: after the touch */
    bool     touch_works;
    /* host side line state */
    bool     dtr, rts;
    uint32_t baud;
    /* parser */
    uint8_t  in[8192];
    uint32_t in_len;
    uint8_t  out[8192];
    uint32_t out_len, out_pos;
    uint32_t addr;                /* AVR word address / SAM-BA */
    uint8_t  ram[0x10000];        /* SAM-BA: board RAM from 0x20000000 */
    uint32_t y_src;
    uint32_t s_left, s_addr;      /* SAM-BA: raw bytes still expected after S */
    bool     samba_xy;
    bool     started;             /* the bootloader started the sketch (Q, leave, E, AIRCR) */
    bool     erased;
    uint32_t fault_at;            /* write a wrong byte at this flash address (0 = none) */
    /* the image */
    const uint8_t *img;
    uint32_t img_len;
    uint32_t clock;
    uint32_t seed;
} sim_t;

static uint32_t rnd(sim_t *d) { d->seed = d->seed * 1103515245u + 12345u; return d->seed >> 8; }

static void emit(sim_t *d, const void *p, uint32_t n)
{
    if (d->out_len + n > sizeof(d->out)) return;
    memcpy(d->out + d->out_len, p, n);
    d->out_len += n;
}
static void emit1(sim_t *d, uint8_t b) { emit(d, &b, 1); }

static void flash_write(sim_t *d, uint32_t at, const uint8_t *p, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
        if (at + i < d->flash_size) {
            uint8_t v = p[i];
            if (d->fault_at && at + i == d->fault_at) v ^= 0x5A;
            d->flash[at + i] = v;
        }
}

static bool hears(sim_t *d) { return d->rate == 0 || d->baud == d->rate; }

/* STK500v1: commands end in CRC_EOP; returns the bytes consumed, 0 when incomplete. */
static uint32_t v1_parse(sim_t *d, const uint8_t *b, uint32_t n)
{
    if (n < 1) return 0;
    uint32_t need;
    switch (b[0]) {
    case '0': case 'u': case 'P': case 'Q': need = 2; break;
    case 'U': need = 4; break;
    case 'd': if (n < 4) return 0; need = 5 + (((uint32_t)b[1] << 8) | b[2]); break;
    case 't': need = 5; break;
    default: return 1;   /* noise: dropped */
    }
    if (n < need) return 0;
    if (b[need - 1] != 0x20) return need;   /* no CRC_EOP: no answer */
    switch (b[0]) {
    case '0': case 'P': emit1(d, 0x14); emit1(d, 0x10); break;
    case 'u': emit1(d, 0x14); emit(d, d->sig, 3); emit1(d, 0x10); break;
    case 'Q': emit1(d, 0x14); emit1(d, 0x10); d->started = true; d->in_loader = false; break;
    case 'U': d->addr = b[1] | ((uint32_t)b[2] << 8); emit1(d, 0x14); emit1(d, 0x10); break;
    case 'd': {
        uint32_t len = ((uint32_t)b[1] << 8) | b[2];
        flash_write(d, d->addr * 2, b + 4, len);
        emit1(d, 0x14); emit1(d, 0x10);
        break;
    }
    case 't': {
        uint32_t len = ((uint32_t)b[1] << 8) | b[2];
        emit1(d, 0x14);
        for (uint32_t i = 0; i < len; i++) emit1(d, d->addr * 2 + i < d->flash_size ? d->flash[d->addr * 2 + i] : 0xFF);
        emit1(d, 0x10);
        break;
    }
    }
    return need;
}

static void v2_reply(sim_t *d, uint8_t seq, const uint8_t *body, uint32_t n)
{
    uint8_t h[5] = { 0x1B, seq, (uint8_t)(n >> 8), (uint8_t)n, 0x0E };
    uint8_t ck = 0;
    for (int i = 0; i < 5; i++) ck ^= h[i];
    for (uint32_t i = 0; i < n; i++) ck ^= body[i];
    emit(d, h, 5); emit(d, body, n); emit1(d, ck);
}

static uint32_t v2_parse(sim_t *d, const uint8_t *b, uint32_t n)
{
    if (n < 1) return 0;
    if (b[0] != 0x1B) return 1;
    if (n < 5) return 0;
    uint32_t len = ((uint32_t)b[2] << 8) | b[3];
    if (n < 6 + len) return 0;
    uint8_t ck = 0;
    for (uint32_t i = 0; i < 5 + len; i++) ck ^= b[i];
    if (b[4] != 0x0E || ck != b[5 + len]) return 6 + len;
    const uint8_t *m = b + 5;
    uint8_t seq = b[1];
    static uint8_t r[1024];
    switch (m[0]) {
    case 0x01: { static const uint8_t s[] = { 0x01, 0x00, 8, 'A','V','R','I','S','P','_','2' }; v2_reply(d, seq, s, sizeof(s)); break; }
    case 0x1B: r[0] = 0x1B; r[1] = 0; r[2] = d->sig[m[4] % 3]; r[3] = 0; v2_reply(d, seq, r, 4); break;
    case 0x10: case 0x11: case 0x06:
        if (m[0] == 0x06) d->addr = ((((uint32_t)m[1] << 24) | ((uint32_t)m[2] << 16) | ((uint32_t)m[3] << 8) | m[4]) & 0x7FFFFFFFu);
        if (m[0] == 0x11) { d->started = true; d->in_loader = false; }
        r[0] = m[0]; r[1] = 0; v2_reply(d, seq, r, 2);
        break;
    case 0x13: {
        uint32_t sz = ((uint32_t)m[1] << 8) | m[2];
        flash_write(d, d->addr * 2, m + 10, sz);
        d->addr += sz / 2;
        r[0] = 0x13; r[1] = 0; v2_reply(d, seq, r, 2);
        break;
    }
    case 0x14: {
        uint32_t sz = ((uint32_t)m[1] << 8) | m[2];
        r[0] = 0x14; r[1] = 0;
        for (uint32_t i = 0; i < sz; i++) r[2 + i] = d->addr * 2 + i < d->flash_size ? d->flash[d->addr * 2 + i] : 0xFF;
        r[2 + sz] = 0;
        d->addr += sz / 2;
        v2_reply(d, seq, r, sz + 3);
        break;
    }
    default: r[0] = m[0]; r[1] = 0xC0; v2_reply(d, seq, r, 2); break;
    }
    return 6 + len;
}

static uint32_t a109_parse(sim_t *d, const uint8_t *b, uint32_t n)
{
    if (n < 1) return 0;
    switch (b[0]) {
    case 'S': emit(d, "CATERIN", 7); return 1;
    case 's': emit1(d, d->sig[2]); emit1(d, d->sig[1]); emit1(d, d->sig[0]); return 1;
    case 'b': emit1(d, 'Y'); emit1(d, 0); emit1(d, 128); return 1;
    case 'P': case 'L': emit1(d, '\r'); return 1;
    case 'E': emit1(d, '\r'); d->started = true; return 1;
    case 'e': memset(d->flash, 0xFF, d->flash_size - 4096); d->erased = true; emit1(d, '\r'); return 1;
    case 'A': if (n < 3) return 0; d->addr = ((uint32_t)b[1] << 8) | b[2]; emit1(d, '\r'); return 3;
    case 'B': {
        if (n < 4) return 0;
        uint32_t len = ((uint32_t)b[1] << 8) | b[2];
        if (n < 4 + len) return 0;
        if (d->addr * 2 + len > d->flash_size - 4096) return 4 + len;   /* Caterina refuses its own area: no answer */
        flash_write(d, d->addr * 2, b + 4, len);
        d->addr += len / 2;
        emit1(d, '\r');
        return 4 + len;
    }
    case 'g': {
        if (n < 4) return 0;
        uint32_t len = ((uint32_t)b[1] << 8) | b[2];
        for (uint32_t i = 0; i < len; i++) emit1(d, d->flash[(d->addr * 2 + i) % d->flash_size]);
        d->addr += len / 2;
        return 4;
    }
    default: emit1(d, '?'); return 1;
    }
}

static uint32_t hexv(const char *p, const char **end)
{
    uint32_t v = 0;
    while ((*p >= '0' && *p <= '9') || (*p >= 'A' && *p <= 'F') || (*p >= 'a' && *p <= 'f')) {
        v = v * 16 + (uint32_t)(*p <= '9' ? *p - '0' : (*p | 0x20) - 'a' + 10);
        p++;
    }
    *end = p;
    return v;
}

static uint8_t *samba_mem(sim_t *d, uint32_t a)
{
    if (a < d->flash_size) return &d->flash[a];
    if (a >= 0x20000000u && a < 0x20000000u + sizeof(d->ram)) return &d->ram[a - 0x20000000u];
    return NULL;
}

static uint32_t samba_parse(sim_t *d, const uint8_t *b, uint32_t n)
{
    if (d->s_left) {
        uint32_t k = n < d->s_left ? n : d->s_left;
        for (uint32_t i = 0; i < k; i++) { uint8_t *m = samba_mem(d, d->s_addr + i); if (m) *m = b[i]; }
        d->s_addr += k;
        d->s_left -= k;
        return k;
    }
    uint32_t e = 0;
    while (e < n && b[e] != '#') e++;
    if (e == n) return n > 64 ? n : 0;   /* no '#' yet */
    char cmd[80];
    uint32_t cl = e < sizeof(cmd) - 1 ? e : sizeof(cmd) - 1;
    memcpy(cmd, b, cl); cmd[cl] = 0;
    const char *p = cmd + 1, *q;
    uint32_t a = hexv(p, &q), v = 0;
    if (*q == ',') v = hexv(q + 1, &q);
    switch (cmd[0]) {
    case 'N': emit(d, "\n\r", 2); break;
    case 'V':
        if (d->samba_xy) emit(d, "v2.0 [Arduino:XYZ] Mar 19 2018 09:45:14\n\r", 41);
        else emit(d, "v1.1 Nov 1 2016 12:00:00\n\r", 26);
        break;
    case 'w': {
        uint32_t w = 0;
        if (a == 0x41002018u) w = 0x10010305u;                      /* SAMD21G18A */
        else if (a == 0x41004008u) w = (3u << 16) | (d->flash_size / 64);   /* 64-byte pages */
        emit(d, &w, 4);
        break;
    }
    case 'X':
        if (d->samba_xy) { for (uint32_t i = a; i < d->flash_size; i++) d->flash[i] = 0xFF; d->erased = true; emit(d, "X\n\r", 3); }
        break;
    case 'S': d->s_addr = a; d->s_left = v; break;
    case 'Y':
        if (!d->samba_xy) break;
        if (v == 0) d->y_src = a;
        else {
            for (uint32_t i = 0; i < v; i++) {
                uint8_t *src = samba_mem(d, d->y_src + i);
                if (src && a + i < d->flash_size) flash_write(d, a + i, src, 1);
            }
        }
        emit(d, "Y\n\r", 3);
        break;
    case 'R':
        for (uint32_t i = 0; i < v; i++) { uint8_t *m = samba_mem(d, a + i); emit1(d, m ? *m : 0); }
        break;
    case 'W':
        if (a == 0xE000ED0Cu && v == 0x05FA0004u) { d->started = true; d->in_loader = false; }
        break;
    default: break;
    }
    return e + 1;
}

static void feed(sim_t *d)
{
    for (;;) {
        uint32_t used = 0;
        if (!d->in_len) return;
        if (d->kind == DEV_RANDOM) {
            uint32_t k = rnd(d) % 12;
            for (uint32_t i = 0; i < k; i++) emit1(d, (uint8_t)rnd(d));
            if (rnd(d) % 4 == 0) emit(d, "\n\r", 2);
            if (rnd(d) % 8 == 0) emit1(d, 0x14);
            used = d->in_len;
        } else if (!d->in_loader || !hears(d) || d->kind == DEV_SILENT) {
            used = d->in_len;
        } else if (d->kind == DEV_OPTIBOOT) used = v1_parse(d, d->in, d->in_len);
        else if (d->kind == DEV_V2) used = v2_parse(d, d->in, d->in_len);
        else if (d->kind == DEV_CATERINA) used = a109_parse(d, d->in, d->in_len);
        else if (d->kind == DEV_SAMBA) used = samba_parse(d, d->in, d->in_len);
        if (!used) return;
        memmove(d->in, d->in + used, d->in_len - used);
        d->in_len -= used;
    }
}

/* ---- ops ------------------------------------------------------------------------------------ */

static int o_write(void *io, const uint8_t *buf, uint32_t n)
{
    sim_t *d = io;
    if (!d->mounted) return -1;
    if (d->in_len + n > sizeof(d->in)) { feed(d); if (d->in_len + n > sizeof(d->in)) d->in_len = 0; }
    uint32_t off = 0;
    while (off < n) {
        uint32_t k = n - off;
        if (k > sizeof(d->in) - d->in_len) k = sizeof(d->in) - d->in_len;
        memcpy(d->in + d->in_len, buf + off, k);
        d->in_len += k;
        off += k;
        feed(d);
    }
    return 0;
}

static uint32_t o_read(void *io, uint8_t *buf, uint32_t n, uint32_t ms)
{
    sim_t *d = io;
    uint32_t avail = d->out_len - d->out_pos;
    uint32_t k = n < avail ? n : avail;
    memcpy(buf, d->out + d->out_pos, k);
    d->out_pos += k;
    if (d->out_pos == d->out_len) d->out_pos = d->out_len = 0;
    if (k < n) d->clock += ms;
    return k;
}

static void o_rx_clear(void *io) { sim_t *d = io; d->out_pos = d->out_len = 0; }

static int o_lines(void *io, bool dtr, bool rts)
{
    sim_t *d = io;
    if (!d->mounted) return -1;
    bool was = d->dtr;
    d->dtr = dtr; d->rts = rts;
    /* The 1200 baud touch: a native-USB sketch resets into its bootloader and enumerates again. */
    if (was && !dtr && d->baud == 1200 && d->touch_works && d->boot_pid) {
        d->pid = d->boot_pid;
        d->mounts++;
        d->in_loader = true;
        d->in_len = 0;
        return 0;
    }
    /* DTR asserted resets an AVR (the auto-reset capacitor): its bootloader runs. */
    if (!was && dtr && (d->kind == DEV_OPTIBOOT || d->kind == DEV_V2 || d->kind == DEV_SILENT || d->kind == DEV_RANDOM)) {
        d->in_loader = true;
        d->in_len = 0;
    }
    return 0;
}

static int o_baud(void *io, uint32_t baud)
{
    sim_t *d = io;
    if (!d->mounted) return -1;
    d->baud = baud;
    return 0;
}

static bool o_port(void *io, uint32_t *mounts, uint16_t *vid, uint16_t *pid)
{
    sim_t *d = io;
    *mounts = d->mounts; *vid = d->vid; *pid = d->pid;
    return d->mounted;
}

static int o_src(void *io, uint32_t off, uint8_t *buf, uint32_t n)
{
    sim_t *d = io;
    if (off > d->img_len || n > d->img_len - off) return -1;
    memcpy(buf, d->img + off, n);
    return 0;
}

static void o_delay(void *io, uint32_t ms) { sim_t *d = io; d->clock += ms; }

/* ---- runs ----------------------------------------------------------------------------------- */

static sim_t g_dev;
static ser_flash_ctx_t g_ctx;
static uint8_t g_img[FLASH_MAX];

static sim_t *board(int kind, uint16_t vid, uint16_t pid, uint32_t flash, const uint8_t sig[3])
{
    sim_t *d = &g_dev;
    memset(d, 0, sizeof(*d));
    d->kind = kind; d->vid = vid; d->pid = pid; d->flash_size = flash;
    if (sig) memcpy(d->sig, sig, 3);
    memset(d->flash, 0xFF, sizeof(d->flash));
    d->mounted = true; d->mounts = 1; d->baud = 115200;
    d->touch_works = true;
    d->samba_xy = true;
    return d;
}

static void make_image(uint32_t len, uint32_t seed)
{
    for (uint32_t i = 0; i < len; i++) { seed = seed * 1664525u + 1013904223u; g_img[i] = (uint8_t)(seed >> 24); }
}

static int run(sim_t *d, int mode, uint32_t len, uint32_t address, uint32_t max_steps)
{
    d->img = g_img; d->img_len = len;
    ser_flash_ctx_t *c = &g_ctx;
    memset(c, 0, sizeof(*c));
    c->ops = (ser_flash_ops_t){ d, o_write, o_read, o_rx_clear, o_lines, o_baud, o_port, o_src, o_delay };
    c->mode = (uint8_t)mode;
    c->len = len;
    c->address = address;
    c->leave = true;
    int rc = SER_FLASH_RUNNING;
    for (uint32_t i = 0; i < max_steps && rc == SER_FLASH_RUNNING; i++) rc = ser_flash_step(c);
    return rc;
}

static bool flash_is_image(sim_t *d, uint32_t at, uint32_t len) { return memcmp(d->flash + at, g_img, len) == 0; }

static const uint8_t SIG_328P[3] = { 0x1E, 0x95, 0x0F };
static const uint8_t SIG_2560[3] = { 0x1E, 0x98, 0x01 };
static const uint8_t SIG_32U4[3] = { 0x1E, 0x95, 0x87 };

static void t_optiboot(void)
{
    sim_t *d = board(DEV_OPTIBOOT, 0x2341, 0x0043, 32768, SIG_328P);   /* Uno R3: 16U2, CDC ACM */
    make_image(10000, 1);
    int rc = run(d, SER_FLASH_AVR, 10000, 0, 100000);
    CHECK(rc == SER_FLASH_DONE);
    CHECK(g_ctx.proto == SER_PROTO_STK500V1);
    CHECK(g_ctx.page == 128 && g_ctx.flash_size == 32768);
    CHECK(flash_is_image(d, 0, 10000));
    CHECK(d->flash[10000] == 0xFF);   /* the last page padded with 0xFF */
    CHECK(d->started);
}

static void t_old_nano(void)
{
    sim_t *d = board(DEV_OPTIBOOT, 0x1A86, 0x7523, 32768, SIG_328P);   /* CH340, ATmegaBOOT at 57600 */
    d->rate = 57600; d->bridge = true;
    make_image(30720, 2);
    int rc = run(d, SER_FLASH_AVR, 30720, 0, 200000);
    CHECK(rc == SER_FLASH_DONE);
    CHECK(g_ctx.proto == SER_PROTO_STK500V1 && g_ctx.baud == 57600);
    CHECK(flash_is_image(d, 0, 30720));
}

static void t_mega(void)
{
    sim_t *d = board(DEV_V2, 0x2341, 0x0042, 262144, SIG_2560);         /* Mega 2560 R3 */
    make_image(140000, 3);   /* past 128 KB: the extended address */
    int rc = run(d, SER_FLASH_AVR, 140000, 0, 400000);
    CHECK(rc == SER_FLASH_DONE);
    CHECK(g_ctx.proto == SER_PROTO_STK500V2);
    CHECK(g_ctx.page == 256);
    CHECK(flash_is_image(d, 0, 140000));
    CHECK(d->started);
}

static void t_caterina(void)
{
    sim_t *d = board(DEV_CATERINA, 0x2341, 0x8036, 32768, SIG_32U4);    /* Leonardo running a sketch */
    d->boot_pid = 0x0036;
    make_image(20000, 4);
    int rc = run(d, SER_FLASH_AVR, 20000, 0, 100000);
    CHECK(rc == SER_FLASH_DONE);
    CHECK(g_ctx.proto == SER_PROTO_AVR109);
    CHECK(d->pid == 0x0036 && d->erased && d->started);
    CHECK(flash_is_image(d, 0, 20000));
}

static void t_caterina_in_loader(void)
{
    sim_t *d = board(DEV_CATERINA, 0x2341, 0x0036, 32768, SIG_32U4);    /* already in its bootloader */
    d->in_loader = true;
    make_image(1000, 5);
    CHECK(run(d, SER_FLASH_AVR, 1000, 0, 100000) == SER_FLASH_DONE);
    CHECK(flash_is_image(d, 0, 1000));
}

static void t_caterina_too_big(void)
{
    sim_t *d = board(DEV_CATERINA, 0x2341, 0x8036, 32768, SIG_32U4);
    d->boot_pid = 0x0036;
    make_image(30000, 6);   /* over 28 KB: into Caterina */
    CHECK(run(d, SER_FLASH_AVR, 30000, 0, 100000) == SER_FLASH_E_FIT);
}

static void t_reattach_timeout(void)
{
    sim_t *d = board(DEV_CATERINA, 0x2341, 0x8036, 32768, SIG_32U4);
    d->boot_pid = 0x0036; d->touch_works = false;
    make_image(1000, 7);
    CHECK(run(d, SER_FLASH_AVR, 1000, 0, 100000) == SER_FLASH_E_REATTACH);
}

static void t_samd(void)
{
    sim_t *d = board(DEV_SAMBA, 0x2341, 0x804D, 262144, NULL);          /* Zero running a sketch */
    d->boot_pid = 0x004D;
    make_image(50001, 8);
    int rc = run(d, SER_FLASH_SAMD, 50001, 0x2000, 100000);
    CHECK(rc == SER_FLASH_DONE);
    CHECK(g_ctx.proto == SER_PROTO_SAMBA);
    CHECK(g_ctx.page == 64 && g_ctx.flash_size == 262144 && g_ctx.chip_id == 0x10010305u);
    CHECK(flash_is_image(d, 0x2000, 50001));
    CHECK(d->started);
    CHECK(d->flash[0x1FFF] == 0xFF);   /* the bootloader area was not touched */
}

static void t_samd_in_loader(void)
{
    sim_t *d = board(DEV_SAMBA, 0x03EB, 0x6124, 262144, NULL);          /* unknown IDs, already in SAM-BA */
    d->in_loader = true;
    make_image(4096, 9);
    CHECK(run(d, SER_FLASH_SAMD, 4096, 0x2000, 100000) == SER_FLASH_DONE);
    CHECK(flash_is_image(d, 0x2000, 4096));
}

static void t_samd_refusals(void)
{
    sim_t *d = board(DEV_SAMBA, 0x2341, 0x804D, 262144, NULL);
    d->boot_pid = 0x004D;
    make_image(4096, 10);
    CHECK(run(d, SER_FLASH_SAMD, 4096, 0, 100000) == SER_FLASH_E_FIT);        /* the bootloader */
    d = board(DEV_SAMBA, 0x2341, 0x804D, 262144, NULL);
    d->boot_pid = 0x004D;
    CHECK(run(d, SER_FLASH_SAMD, 4096, 0x2010, 100000) == SER_FLASH_E_ARGS);  /* not on a page */
    d = board(DEV_SAMBA, 0x2341, 0x804D, 65536, NULL);
    d->boot_pid = 0x004D;
    make_image(65536, 10);
    CHECK(run(d, SER_FLASH_SAMD, 65536, 0x2000, 100000) == SER_FLASH_E_FIT);  /* past the end */
    d = board(DEV_SAMBA, 0x2341, 0x804D, 262144, NULL);
    d->boot_pid = 0x004D; d->samba_xy = false;
    CHECK(run(d, SER_FLASH_SAMD, 4096, 0x2000, 100000) == SER_FLASH_E_UNSUPPORTED);
}

static void t_no_loader(void)
{
    sim_t *d = board(DEV_SILENT, 0x0403, 0x6001, 32768, SIG_328P);      /* FTDI, nothing answers */
    make_image(1000, 11);
    CHECK(run(d, SER_FLASH_AVR, 1000, 0, 100000) == SER_FLASH_E_NO_LOADER);
    d = board(DEV_SILENT, 0x10C4, 0xEA60, 32768, NULL);
    CHECK(run(d, SER_FLASH_SAMD, 1000, 0x2000, 100000) == SER_FLASH_E_NO_LOADER);
}

static void t_verify(void)
{
    sim_t *d = board(DEV_OPTIBOOT, 0x2341, 0x0043, 32768, SIG_328P);
    d->fault_at = 5000;
    make_image(10000, 12);
    CHECK(run(d, SER_FLASH_AVR, 10000, 0, 100000) == SER_FLASH_E_VERIFY);
    CHECK(!d->started);
    d = board(DEV_SAMBA, 0x2341, 0x804D, 262144, NULL);
    d->boot_pid = 0x004D; d->fault_at = 0x2000 + 7777;
    CHECK(run(d, SER_FLASH_SAMD, 20000, 0x2000, 100000) == SER_FLASH_E_VERIFY);
    CHECK(!d->started);
}

static void t_args(void)
{
    sim_t *d = board(DEV_OPTIBOOT, 0x2341, 0x0043, 32768, SIG_328P);
    make_image(100, 13);
    CHECK(run(d, SER_FLASH_AVR, 100, 1, 1000) == SER_FLASH_E_ARGS);   /* odd address */
    CHECK(run(d, SER_FLASH_AVR, 0, 0, 1000) == SER_FLASH_E_ARGS);
    CHECK(run(d, SER_FLASH_AVR, 100, 0xFFFFFFF0u, 1000) == SER_FLASH_E_ARGS);
    d->mounted = false;
    CHECK(run(d, SER_FLASH_AVR, 100, 0, 1000) == SER_FLASH_E_IO);
    d = board(DEV_OPTIBOOT, 0x2341, 0x0043, 32768, SIG_328P);
    make_image(32768, 13);
    CHECK(run(d, SER_FLASH_AVR, 32768, 0, 100000) == SER_FLASH_E_FIT);   /* over optiboot */
}

static void t_random(void)
{
    for (uint32_t i = 0; i < 2000; i++) {
        sim_t *d = board(DEV_RANDOM, (uint16_t)(0x2341 + (i % 3)), (uint16_t)(i * 7919u), 32768, NULL);
        d->seed = i * 2654435761u + 1;
        d->in_loader = true;
        d->boot_pid = (uint16_t)(i & 0x7FFF);
        make_image(3000, i);
        int rc = run(d, (i & 1) ? SER_FLASH_SAMD : SER_FLASH_AVR, 3000, (i & 1) ? 0x2000 : 0, 200000);
        CHECK(rc != SER_FLASH_RUNNING);
        CHECK(rc != SER_FLASH_DONE || g_ctx.proto != SER_PROTO_NONE);
        CHECK(ser_flash_rc_str(rc)[0] != '?');
    }
}

static void t_names(void)
{
    for (int p = SER_PH_START; p <= SER_PH_DONE; p++) CHECK(ser_flash_phase_str(p)[0] != '?');
    for (int r = SER_FLASH_E_UNSUPPORTED; r <= SER_FLASH_RUNNING; r++) CHECK(ser_flash_rc_str(r)[0] != '?');
    CHECK(!strcmp(ser_flash_proto_str(SER_PROTO_STK500V2), "stk500v2"));
}

int main(void)
{
    t_optiboot();
    t_old_nano();
    t_mega();
    t_caterina();
    t_caterina_in_loader();
    t_caterina_too_big();
    t_reattach_timeout();
    t_samd();
    t_samd_in_loader();
    t_samd_refusals();
    t_no_loader();
    t_verify();
    t_args();
    t_random();
    t_names();
    if (failures) { printf("test_ser_flash: %d failures\n", failures); return 1; }
    printf("test_ser_flash: all passed\n");
    return 0;
}
