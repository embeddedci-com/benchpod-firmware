/* xip_mod_ctrl_flash.c — flashing over USB control transfers: DFU/DfuSe and Teensy HalfKay
 * (ctrl_flash.h).
 *
 * Runs from the W25Q (xip_image.h): no static data, no callbacks; the ops in the context are
 * internal functions. Also built into the host tests.
 *
 * Every step does at most one control transfer of the protocol (plus a short delay), so a loader
 * that is busy for seconds (a sector erase) never holds the hw worker: the step returns and the
 * next one asks again. */
#include "ctrl_flash.h"

#include <string.h>

/* DFU 1.1 requests and states */
#define DFU_DNLOAD     1
#define DFU_GETSTATUS  3
#define DFU_CLRSTATUS  4
#define DFU_ABORT      6
#define ST_IDLE        2
#define ST_DNLOAD_SYNC 3
#define ST_DNBUSY      4
#define ST_DNLOAD_IDLE 5
#define ST_MANIFEST_SYNC 6
#define ST_MANIFEST    7
#define ST_ERROR       10

#define DFUSE_SET_ADDR 0x21
#define DFUSE_ERASE    0x41

#define STATUS_TIMEOUT_MS  20000u   /* a busy loader (the longest STM32 sector erase is ~4 s) */
#define TEENSY_FIRST_MS    45000u   /* HalfKay erases on the first block */
#define TEENSY_BLOCK_MS    5000u
#define TEENSY_VID 0x16C0u
#define TEENSY_PID 0x0478u

static uint32_t rd16(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8); }
static uint32_t rd24(const uint8_t *p) { return rd16(p) | ((uint32_t)p[2] << 16); }

static void setup(uint8_t s[8], uint8_t type, uint8_t req, uint16_t value, uint16_t index, uint16_t len)
{
    s[0] = type; s[1] = req;
    s[2] = (uint8_t)value; s[3] = (uint8_t)(value >> 8);
    s[4] = (uint8_t)index; s[5] = (uint8_t)(index >> 8);
    s[6] = (uint8_t)len;   s[7] = (uint8_t)(len >> 8);
}

static int finish(ctrl_flash_ctx_t *c, int rc)
{
    c->result = rc;
    c->phase = CTRL_PH_DONE;
    return rc;
}

static int ctrl(ctrl_flash_ctx_t *c, uint8_t type, uint8_t req, uint16_t value, uint16_t index,
                uint8_t *data, uint16_t len, uint32_t *actual)
{
    uint8_t s[8];
    uint32_t a = 0;
    setup(s, type, req, value, index, len);
    int rc = c->ops.ctrl(c->ops.io, s, data, &a);
    if (actual) *actual = a;
    return rc;
}

/* ---- DFU ----------------------------------------------------------------------------------- */

static int dnload(ctrl_flash_ctx_t *c, uint16_t block, uint8_t *data, uint16_t len)
{
    if (ctrl(c, 0x21, DFU_DNLOAD, block, c->itf, data, len, NULL) != 0) return -1;
    c->waiting = true;
    c->waited = 0;
    return 0;
}

/* One GETSTATUS while a DNLOAD is in progress. 1 = still busy (the step returns), 0 = done,
   < 0 = a ctrl_flash_rc_t error. */
static int poll_status(ctrl_flash_ctx_t *c)
{
    uint8_t st[6];
    uint32_t got = 0;
    if (ctrl(c, 0xA1, DFU_GETSTATUS, 0, c->itf, st, 6, &got) != 0 || got < 6) return CTRL_FLASH_E_IO;
    c->dfu_status = st[0];
    uint32_t poll = rd24(st + 1), state = st[4];
    if (st[0] != 0 || state == ST_ERROR) {
        (void)ctrl(c, 0x21, DFU_CLRSTATUS, 0, c->itf, NULL, 0, NULL);
        return CTRL_FLASH_E_STATUS;
    }
    if (state == ST_DNLOAD_IDLE || state == ST_IDLE) { c->waiting = false; return 0; }
    if (state == ST_DNBUSY || state == ST_DNLOAD_SYNC || state == ST_MANIFEST_SYNC || state == ST_MANIFEST) {
        if (poll > 200u) poll = 200u;           /* ask again sooner than a slow loader says */
        if (poll == 0) poll = 1;
        c->ops.delay_ms(c->ops.io, poll);
        c->waited += poll;
        if (c->waited > STATUS_TIMEOUT_MS) return CTRL_FLASH_E_TIMEOUT;
        return 1;
    }
    return CTRL_FLASH_E_STATUS;
}

static int dfu_descriptors(ctrl_flash_ctx_t *c)
{
    uint8_t *b = c->buf;
    uint32_t got = 0;
    if (ctrl(c, 0x80, 6, 0x0200, 0, b, 9, &got) != 0 || got < 9 || b[1] != 2) return CTRL_FLASH_E_IO;
    uint32_t total = rd16(b + 2);
    if (total > CTRL_FLASH_BUF) total = CTRL_FLASH_BUF;
    if (total < 9u) return CTRL_FLASH_E_NO_DFU;
    if (ctrl(c, 0x80, 6, 0x0200, 0, b, (uint16_t)total, &got) != 0 || got < 9) return CTRL_FLASH_E_IO;
    if (got < total) total = got;
    bool in_dfu = false, found = false;
    uint32_t xfer = 0, bcd = 0;
    for (uint32_t off = b[0]; off + 2 <= total;) {
        uint32_t dl = b[off], type = b[off + 1];
        if (dl < 2 || off + dl > total) break;
        if (type == 4 && dl >= 9) {
            in_dfu = !found && b[off + 5] == 0xFE && b[off + 6] == 0x01 && b[off + 7] == 0x02 && b[off + 3] == 0;
            if (in_dfu) { c->itf = b[off + 2]; c->i_string = b[off + 8]; found = true; }
        } else if (type == 0x21 && in_dfu && dl >= 7) {
            xfer = rd16(b + off + 5);
            bcd = dl >= 9 ? rd16(b + off + 7) : 0x0100u;
        }
        off += dl;
    }
    if (!found || xfer == 0) return CTRL_FLASH_E_NO_DFU;
    /* DfuSe places block n at pointer + (n - 2) * the device's transfer size: blocks must be that
       size, or the pointer set before each one. */
    c->xfer = (uint16_t)(xfer > CTRL_FLASH_BUF ? CTRL_FLASH_BUF : xfer);
    c->dfuse = bcd == 0x011Au;
    c->addr_each = c->dfuse && xfer > CTRL_FLASH_BUF;
    return 0;
}

static uint32_t hex_at(const char *s, const char **end)
{
    uint32_t v = 0;
    while ((*s >= '0' && *s <= '9') || (*s >= 'a' && *s <= 'f') || (*s >= 'A' && *s <= 'F')) {
        uint32_t d = (uint32_t)(*s <= '9' ? *s - '0' : (*s | 0x20) - 'a' + 10);
        v = (v << 4) | d;
        s++;
    }
    *end = s;
    return v;
}

/* "@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg" -> erasable and writable regions. */
static int dfuse_layout(ctrl_flash_ctx_t *c, const char *s)
{
    c->nregions = 0;
    if (*s != '@') return CTRL_FLASH_E_LAYOUT;
    for (const char *p = s; (p = strstr(p, "/0x")) != NULL;) {
        const char *q;
        uint32_t addr = hex_at(p + 3, &q);
        if (*q != '/') return CTRL_FLASH_E_LAYOUT;
        q++;
        for (;;) {
            uint32_t count = 0, size = 0;
            while (*q >= '0' && *q <= '9') count = count * 10u + (uint32_t)(*q++ - '0');
            if (*q++ != '*') return CTRL_FLASH_E_LAYOUT;
            while (*q >= '0' && *q <= '9') size = size * 10u + (uint32_t)(*q++ - '0');
            char unit = *q++;
            if (unit == 'K') size *= 1024u; else if (unit == 'M') size *= 1024u * 1024u;
            else if (unit != 'B' && unit != ' ') return CTRL_FLASH_E_LAYOUT;
            char kind = *q++;
            uint32_t bits = (kind >= 'a' && kind <= 'g') ? (uint32_t)(kind - 'a' + 1) : 0;
            if (count == 0 || size == 0) return CTRL_FLASH_E_LAYOUT;
            if ((bits & 6u) == 6u && c->nregions < CTRL_FLASH_MAX_REGIONS) {   /* erasable + writable */
                c->region[c->nregions].start = addr;
                c->region[c->nregions].count = count;
                c->region[c->nregions].size = size;
                c->nregions++;
            }
            addr += count * size;
            if (*q != ',') break;
            q++;
        }
        p = q;
    }
    return c->nregions ? 0 : CTRL_FLASH_E_LAYOUT;
}

/* The sector containing addr: its start in *start, size in *size. false if none. */
static bool sector_of(const ctrl_flash_ctx_t *c, uint32_t addr, uint32_t *start, uint32_t *size)
{
    for (unsigned i = 0; i < c->nregions; i++) {
        uint32_t base = c->region[i].start, sz = c->region[i].size;
        if (addr >= base && addr - base < c->region[i].count * sz) {
            *start = base + (addr - base) / sz * sz;
            *size = sz;
            return true;
        }
    }
    return false;
}

static int dfu_layout_step(ctrl_flash_ctx_t *c)
{
    if (!c->i_string) return finish(c, CTRL_FLASH_E_LAYOUT);
    uint8_t *b = c->buf;
    uint32_t got = 0;
    if (ctrl(c, 0x80, 6, (uint16_t)(0x0300u | c->i_string), 0x0409, b, 255, &got) != 0 || got < 2 || b[1] != 3)
        return finish(c, CTRL_FLASH_E_IO);
    uint32_t n = b[0] < got ? b[0] : got;
    char *s = (char *)(c->buf + 512);   /* ASCII copy, NUL-terminated */
    uint32_t o = 0;
    for (uint32_t i = 2; i + 1 < n && o < 255u; i += 2) s[o++] = (char)(b[i + 1] ? '?' : b[i]);
    s[o] = '\0';
    int rc = dfuse_layout(c, s);
    if (rc != 0) return finish(c, rc);
    /* The whole image must be inside erasable, writable flash. */
    for (uint32_t a = c->address; a < c->address + c->len;) {
        uint32_t st, sz;
        if (!sector_of(c, a, &st, &sz)) return finish(c, CTRL_FLASH_E_LAYOUT);
        a = st + sz;
    }
    uint32_t st = 0, sz = 0;
    (void)sector_of(c, c->address, &st, &sz);   /* found: the loop above checked every sector */
    c->erase_addr = st;
    c->phase = CTRL_PH_ERASE;
    return CTRL_FLASH_RUNNING;
}

static uint8_t *put_cmd(ctrl_flash_ctx_t *c, uint8_t cmd, uint32_t addr)
{
    uint8_t *b = c->buf;
    b[0] = cmd; b[1] = (uint8_t)addr; b[2] = (uint8_t)(addr >> 8);
    b[3] = (uint8_t)(addr >> 16); b[4] = (uint8_t)(addr >> 24);
    return b;
}

static int dfu_step(ctrl_flash_ctx_t *c)
{
    if (c->waiting) {
        int r = poll_status(c);
        if (r < 0) return finish(c, c->phase == CTRL_PH_LEAVE ? CTRL_FLASH_DONE : r);
        if (r > 0) return CTRL_FLASH_RUNNING;
        if (c->phase == CTRL_PH_LEAVE) return finish(c, CTRL_FLASH_DONE);
    }
    switch (c->phase) {
    case CTRL_PH_ERASE:
        if (c->erase_addr >= c->address + c->len) { c->phase = CTRL_PH_ADDR; return CTRL_FLASH_RUNNING; }
        if (dnload(c, 0, put_cmd(c, DFUSE_ERASE, c->erase_addr), 5) != 0) return finish(c, CTRL_FLASH_E_IO);
        {
            uint32_t st, sz;
            if (!sector_of(c, c->erase_addr, &st, &sz)) return finish(c, CTRL_FLASH_E_LAYOUT);
            c->erase_addr = st + sz;
        }
        return CTRL_FLASH_RUNNING;
    case CTRL_PH_ADDR:
        if (dnload(c, 0, put_cmd(c, DFUSE_SET_ADDR, c->address), 5) != 0) return finish(c, CTRL_FLASH_E_IO);
        c->phase = CTRL_PH_WRITE;
        c->block_no = 2;
        return CTRL_FLASH_RUNNING;
    case CTRL_PH_WRITE: {
        if (c->pos >= c->len) {
            c->phase = CTRL_PH_LEAVE;
            if (!c->leave) return finish(c, CTRL_FLASH_DONE);
            if (c->dfuse) {   /* the pointer back to the start, then leave */
                if (dnload(c, 0, put_cmd(c, DFUSE_SET_ADDR, c->address), 5) != 0) return finish(c, CTRL_FLASH_E_IO);
                int r;   /* its status is read here, before the zero-length request */
                do { r = poll_status(c); } while (r > 0);
                if (r < 0) return finish(c, r);
            }
            /* A zero-length DNLOAD: the loader manifests and starts the firmware; it may drop off
               the bus before it answers, which is fine. */
            if (dnload(c, c->dfuse ? 0 : (uint16_t)c->block_no, NULL, 0) != 0) return finish(c, CTRL_FLASH_DONE);
            return CTRL_FLASH_RUNNING;
        }
        if (c->addr_each && !c->addr_sent) {
            if (dnload(c, 0, put_cmd(c, DFUSE_SET_ADDR, c->address + c->pos), 5) != 0) return finish(c, CTRL_FLASH_E_IO);
            c->addr_sent = true;
            c->block_no = 2;
            return CTRL_FLASH_RUNNING;
        }
        uint32_t n = c->len - c->pos < c->xfer ? c->len - c->pos : c->xfer;
        if (c->ops.src(c->ops.io, c->pos, c->buf, n) != 0) return finish(c, CTRL_FLASH_E_SRC);
        if (dnload(c, (uint16_t)c->block_no, c->buf, (uint16_t)n) != 0) return finish(c, CTRL_FLASH_E_IO);
        c->pos += n;
        c->block_no++;
        c->addr_sent = false;
        c->done++;
        return CTRL_FLASH_RUNNING;
    }
    default:
        return finish(c, CTRL_FLASH_E_ARGS);
    }
}

static int dfu_start(ctrl_flash_ctx_t *c)
{
    int rc = dfu_descriptors(c);
    if (rc != 0) return finish(c, rc);
    /* Get the loader to dfuIDLE: clear an error, abort a half-done transfer. */
    uint8_t st[6];
    uint32_t got = 0;
    if (ctrl(c, 0xA1, DFU_GETSTATUS, 0, c->itf, st, 6, &got) != 0 || got < 6) return finish(c, CTRL_FLASH_E_IO);
    if (st[4] == ST_ERROR) (void)ctrl(c, 0x21, DFU_CLRSTATUS, 0, c->itf, NULL, 0, NULL);
    else if (st[4] != ST_IDLE) (void)ctrl(c, 0x21, DFU_ABORT, 0, c->itf, NULL, 0, NULL);
    c->total = (c->len + c->xfer - 1u) / c->xfer;
    c->block_no = 0;
    c->phase = c->dfuse ? CTRL_PH_LAYOUT : CTRL_PH_WRITE;
    return CTRL_FLASH_RUNNING;
}

/* ---- HalfKay ------------------------------------------------------------------------------- */

static bool all_ff(const uint8_t *p, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) if (p[i] != 0xFF) return false;
    return true;
}

static int halfkay_step(ctrl_flash_ctx_t *c)
{
    const uint32_t blk = c->block, rep = 64u + blk;
    if (c->phase == CTRL_PH_LEAVE) {
        memset(c->buf, 0, rep);
        c->buf[0] = c->buf[1] = c->buf[2] = 0xFF;   /* the reboot request */
        (void)ctrl(c, 0x21, 0x09, 0x0200, 0, c->buf, (uint16_t)rep, NULL);
        return finish(c, CTRL_FLASH_DONE);
    }
    /* Skip blocks that are all 0xFF (erased already), but never the first. */
    for (unsigned skips = 0; c->pos < c->len && skips < 64u; skips++) {
        uint32_t n = c->len - c->pos < blk ? c->len - c->pos : blk;
        memset(c->buf + 64, 0xFF, blk);
        if (c->ops.src(c->ops.io, c->pos, c->buf + 64, n) != 0) return finish(c, CTRL_FLASH_E_SRC);
        if (c->pos == 0 || !all_ff(c->buf + 64, blk)) break;
        c->pos += blk;
        c->done++;
    }
    if (c->pos >= c->len) {
        c->phase = CTRL_PH_LEAVE;
        if (!c->leave) return finish(c, CTRL_FLASH_DONE);
        return CTRL_FLASH_RUNNING;
    }
    uint32_t addr = c->address + c->pos;
    memset(c->buf, 0, 64);
    c->buf[0] = (uint8_t)addr; c->buf[1] = (uint8_t)(addr >> 8); c->buf[2] = (uint8_t)(addr >> 16);
    if (ctrl(c, 0x21, 0x09, 0x0200, 0, c->buf, (uint16_t)rep, NULL) != 0) {
        /* The loader is busy erasing (the first block) or writing: try again shortly. */
        c->ops.delay_ms(c->ops.io, 10);
        c->waited += 10;
        if (c->waited > (c->pos == 0 ? TEENSY_FIRST_MS : TEENSY_BLOCK_MS)) return finish(c, CTRL_FLASH_E_TIMEOUT);
        return CTRL_FLASH_RUNNING;
    }
    c->waited = 0;
    c->pos += blk;
    c->done++;
    return CTRL_FLASH_RUNNING;
}

static int halfkay_start(ctrl_flash_ctx_t *c)
{
    if (c->ops.vid != TEENSY_VID || c->ops.pid != TEENSY_PID) return finish(c, CTRL_FLASH_E_NOT_TEENSY);
    if (c->block == 0) c->block = 1024;
    if ((c->block != 1024 && c->block != 512) || c->address + c->len > 0x1000000u) return finish(c, CTRL_FLASH_E_ARGS);
    c->total = (c->len + c->block - 1u) / c->block;
    c->phase = CTRL_PH_WRITE;
    return CTRL_FLASH_RUNNING;
}

/* ---- entry ---------------------------------------------------------------------------------- */

int ctrl_flash_step(ctrl_flash_ctx_t *c)
{
    if (c->phase == CTRL_PH_DONE) return c->result;
    if (c->phase == CTRL_PH_START) {
        c->result = CTRL_FLASH_RUNNING;
        c->done = c->total = c->pos = 0;
        c->waiting = false;
        c->waited = 0;
        if (c->len == 0) return finish(c, CTRL_FLASH_E_ARGS);
        if (c->mode == CTRL_FLASH_HALFKAY) return halfkay_start(c);
        if (c->mode == CTRL_FLASH_DFU) return dfu_start(c);
        return finish(c, CTRL_FLASH_E_ARGS);
    }
    if (c->mode == CTRL_FLASH_HALFKAY) return halfkay_step(c);
    if (c->phase == CTRL_PH_LAYOUT) return dfu_layout_step(c);
    return dfu_step(c);
}
