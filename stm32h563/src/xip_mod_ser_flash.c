/* xip_mod_ser_flash.c — flashing over a USB serial port: the Arduino AVR bootloaders (AVR109,
 * STK500v1, STK500v2) and the Arduino SAMD bootloader (SAM-BA with Arduino's X/Y commands)
 * (ser_flash.h).
 *
 * Runs from the W25Q (xip_image.h): no static data, no callbacks; the ops in the context are
 * internal functions. Also built into the host tests.
 *
 * Every step does one exchange with the bootloader (a page, a 4 KB chunk, or a short command
 * sequence), so the hw worker keeps answering between steps. */
#include "ser_flash.h"

#include <string.h>

#define REPLY_MS        300u
#define SYNC_MS         200u
#define PAGE_MS         1000u
#define ERASE_AVR_MS    10000u
#define ERASE_SAMD_MS   30000u
#define REATTACH_MS     10000u
#define V1_SYNC_TRIES   8u
#define OTHER_TRIES     5u

/* STK500v1 */
#define STK_OK          0x10
#define STK_INSYNC      0x14
#define CRC_EOP         0x20

/* STK500v2 */
#define V2_START        0x1B
#define V2_TOKEN        0x0E
#define V2_SIGN_ON      0x01
#define V2_LOAD_ADDRESS 0x06
#define V2_ENTER_PROG   0x10
#define V2_LEAVE_PROG   0x11
#define V2_PROGRAM_FLASH 0x13
#define V2_READ_FLASH   0x14
#define V2_READ_SIG     0x1B

/* SAM-BA: the board RAM a 4 KB chunk goes through, and the chunk size. */
#define SAMBA_RAM       0x20005000u
#define SAMBA_CHUNK     4096u
#define SAMBA_READ      1024u
#define SAMD_DID        0x41002018u
#define SAMD_NVM_PARAM  0x41004008u

/* AVR probes, in order, for a board whose bootloader is not known from its IDs. */
#define NPROBES 4

static void probe_get(uint8_t i, uint8_t *proto, uint32_t *baud)
{
    switch (i) {
    case 0:  *proto = SER_PROTO_STK500V1; *baud = 115200; break;   /* optiboot: Uno, new Nano */
    case 1:  *proto = SER_PROTO_STK500V2; *baud = 115200; break;   /* Mega 2560 */
    case 2:  *proto = SER_PROTO_STK500V1; *baud = 57600;  break;   /* ATmegaBOOT: old Nano, Pro Mini */
    default: *proto = SER_PROTO_AVR109;   *baud = 57600;  break;   /* a 32u4 already in Caterina */
    }
}

/* AVR signature -> flash size, the smallest bootloader it carries, page size. */
static bool avr_chip(const uint8_t s[3], uint32_t *flash, uint32_t *boot, uint32_t *page)
{
    if (s[0] != 0x1E) return false;
    uint16_t id = (uint16_t)((s[1] << 8) | s[2]);
    switch (id) {
    case 0x950F: case 0x9514: case 0x9516:            /* ATmega328P, 328, 328PB */
        *flash = 32768; *boot = 512; *page = 128; return true;
    case 0x9406: case 0x940B:                         /* ATmega168, 168P */
        *flash = 16384; *boot = 512; *page = 128; return true;
    case 0x930A: case 0x930F: case 0x9307:            /* ATmega88, 88P, 8 */
        *flash = 8192; *boot = 512; *page = 64; return true;
    case 0x9801:                                      /* ATmega2560 */
        *flash = 262144; *boot = 8192; *page = 256; return true;
    case 0x9703:                                      /* ATmega1280 */
        *flash = 131072; *boot = 8192; *page = 256; return true;
    case 0x9587:                                      /* ATmega32U4 (Caterina 4 KB) */
        *flash = 32768; *boot = 4096; *page = 128; return true;
    case 0x9705:                                      /* ATmega1284P */
        *flash = 131072; *boot = 1024; *page = 256; return true;
    case 0x960A:                                      /* ATmega644P */
        *flash = 65536; *boot = 1024; *page = 256; return true;
    default: return false;
    }
}

/* Boards that run a sketch with native USB and enter their bootloader on the 1200 baud touch
   (Arduino, Adafruit, SparkFun, Seeed: sketch PIDs have bit 15 set, or SparkFun's pairs). */
static bool app_needs_touch(uint16_t vid, uint16_t pid)
{
    if (vid == 0x2341 || vid == 0x2A03 || vid == 0x239A || vid == 0x2886) return (pid & 0x8000) != 0;
    if (vid == 0x1B4F) return pid == 0x9204 || pid == 0x9206;
    return false;
}

static bool caterina_boot(uint16_t vid, uint16_t pid)
{
    if (vid == 0x2341 || vid == 0x2A03)
        return pid == 0x0036 || pid == 0x0037 || pid == 0x003C || pid == 0x0041;
    if (vid == 0x1B4F) return pid == 0x9203 || pid == 0x9205;
    if (vid == 0x239A) return pid == 0x000C || pid == 0x000D || pid == 0x000E;
    return false;
}

static int finish(ser_flash_ctx_t *c, int rc)
{
    c->result = rc;
    c->phase = SER_PH_DONE;
    return rc;
}

static uint32_t rd32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Send `out` (after dropping what came before), then read exactly `inn` bytes. */
static bool xchg(ser_flash_ctx_t *c, const uint8_t *out, uint32_t outn, uint8_t *in, uint32_t inn, uint32_t ms)
{
    c->ops.rx_clear(c->ops.io);
    if (outn && c->ops.write(c->ops.io, out, outn) != 0) return false;
    return c->ops.read(c->ops.io, in, inn, ms) == inn;
}

static bool expect(ser_flash_ctx_t *c, const uint8_t *want, uint32_t n, uint32_t ms)
{
    uint8_t got[4];
    if (n > sizeof(got) || c->ops.read(c->ops.io, got, n, ms) != n) return false;
    return memcmp(got, want, n) == 0;
}

/* The image: n bytes at off into dst, 0xFF up to pad. */
static bool image(ser_flash_ctx_t *c, uint32_t off, uint8_t *dst, uint32_t n, uint32_t pad)
{
    if (n && c->ops.src(c->ops.io, off, dst, n) != 0) return false;
    if (pad > n) memset(dst + n, 0xFF, pad - n);
    return true;
}

/* ---- STK500v1 ------------------------------------------------------------------------------ */

/* cmd (with its CRC_EOP) -> STK_INSYNC, `rn` bytes into resp, STK_OK. */
static bool v1(ser_flash_ctx_t *c, const uint8_t *cmd, uint32_t n, uint8_t *resp, uint32_t rn, uint32_t ms)
{
    static const uint8_t insync[1] = { STK_INSYNC }, ok[1] = { STK_OK };
    c->ops.rx_clear(c->ops.io);
    if (c->ops.write(c->ops.io, cmd, n) != 0) return false;
    if (!expect(c, insync, 1, ms)) return false;
    if (rn && c->ops.read(c->ops.io, resp, rn, ms) != rn) return false;
    return expect(c, ok, 1, ms);
}

static bool v1_addr(ser_flash_ctx_t *c, uint32_t byte_addr)
{
    uint32_t w = byte_addr >> 1;
    uint8_t cmd[4] = { 'U', (uint8_t)w, (uint8_t)(w >> 8), CRC_EOP };
    return v1(c, cmd, sizeof(cmd), NULL, 0, REPLY_MS);
}

/* ---- STK500v2 ------------------------------------------------------------------------------ */

/* One message: body -> answer (into ans, at most ans_max). The answer's length, -1 on a bad
   frame or a status other than STATUS_CMD_OK. */
static int v2(ser_flash_ctx_t *c, const uint8_t *body, uint32_t n, uint8_t *ans, uint32_t ans_max, uint32_t ms)
{
    uint8_t h[5] = { V2_START, ++c->seq, (uint8_t)(n >> 8), (uint8_t)n, V2_TOKEN };
    uint8_t ck = 0;
    for (uint32_t i = 0; i < 5; i++) ck ^= h[i];
    for (uint32_t i = 0; i < n; i++) ck ^= body[i];
    c->ops.rx_clear(c->ops.io);
    if (c->ops.write(c->ops.io, h, 5) != 0 || c->ops.write(c->ops.io, body, n) != 0 ||
        c->ops.write(c->ops.io, &ck, 1) != 0) return -1;
    uint8_t r[5];
    if (c->ops.read(c->ops.io, r, 5, ms) != 5) return -1;
    if (r[0] != V2_START || r[1] != c->seq || r[4] != V2_TOKEN) return -1;
    uint32_t len = ((uint32_t)r[2] << 8) | r[3];
    if (len < 2 || len > ans_max) return -1;
    if (c->ops.read(c->ops.io, ans, len, ms) != len) return -1;
    uint8_t rck;
    if (c->ops.read(c->ops.io, &rck, 1, ms) != 1) return -1;
    uint8_t x = 0;
    for (uint32_t i = 0; i < 5; i++) x ^= r[i];
    for (uint32_t i = 0; i < len; i++) x ^= ans[i];
    if (x != rck || ans[0] != body[0] || ans[1] != 0x00) return -1;
    return (int)len;
}

static bool v2_addr(ser_flash_ctx_t *c, uint32_t byte_addr)
{
    uint32_t w = byte_addr >> 1;
    if (c->flash_size > 131072u) w |= 0x80000000u;   /* the extended address flag, as avrdude sends it */
    uint8_t b[5] = { V2_LOAD_ADDRESS, (uint8_t)(w >> 24), (uint8_t)(w >> 16), (uint8_t)(w >> 8), (uint8_t)w };
    uint8_t a[4];
    return v2(c, b, sizeof(b), a, sizeof(a), REPLY_MS) == 2;
}

/* ---- AVR109 -------------------------------------------------------------------------------- */

static bool cr(ser_flash_ctx_t *c, uint32_t ms)
{
    static const uint8_t r[1] = { '\r' };
    return expect(c, r, 1, ms);
}

static bool a109_addr(ser_flash_ctx_t *c, uint32_t byte_addr)
{
    uint32_t w = byte_addr >> 1;
    uint8_t cmd[3] = { 'A', (uint8_t)(w >> 8), (uint8_t)w };
    c->ops.rx_clear(c->ops.io);
    return c->ops.write(c->ops.io, cmd, 3) == 0 && cr(c, REPLY_MS);
}

/* ---- SAM-BA -------------------------------------------------------------------------------- */

static char *hex8(char *p, uint32_t v)
{
    static const char d[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
    for (int i = 7; i >= 0; i--) *p++ = d[(v >> (i * 4)) & 0xF];
    return p;
}

/* "<op><a>[,<b>]#" */
static uint32_t samba_cmd(char *out, char op, uint32_t a, bool has_b, uint32_t b)
{
    char *p = out;
    *p++ = op;
    p = hex8(p, a);
    if (has_b) { *p++ = ','; p = hex8(p, b); }
    *p++ = '#';
    return (uint32_t)(p - out);
}

static bool samba_ack(ser_flash_ctx_t *c, char op, uint32_t ms)
{
    uint8_t w[3] = { (uint8_t)op, '\n', '\r' };
    uint8_t got[3];
    return c->ops.read(c->ops.io, got, 3, ms) == 3 && memcmp(got, w, 3) == 0;
}

static bool samba_word(ser_flash_ctx_t *c, uint32_t addr, uint32_t *v)
{
    char cmd[24];
    uint32_t n = samba_cmd(cmd, 'w', addr, true, 4);
    uint8_t r[4];
    if (!xchg(c, (const uint8_t *)cmd, n, r, 4, REPLY_MS)) return false;
    *v = rd32le(r);
    return true;
}

/* The version line: Arduino's fast commands are listed in its "[Arduino:XYZ]" tag. */
static bool samba_fast(const uint8_t *v, uint32_t n)
{
    static const char tag[9] = { '[','A','r','d','u','i','n','o',':' };
    for (uint32_t i = 0; i + sizeof(tag) <= n; i++) {
        if (memcmp(v + i, tag, sizeof(tag)) != 0) continue;
        bool x = false, y = false;
        for (uint32_t j = i + sizeof(tag); j < n && v[j] != ']'; j++) {
            if (v[j] == 'X') x = true;
            if (v[j] == 'Y') y = true;
        }
        return x && y;
    }
    return false;
}

/* ---- the steps ----------------------------------------------------------------------------- */

static int step_start(ser_flash_ctx_t *c)
{
    if (c->len == 0 || c->len > 0x1000000u || c->address > 0xFFFFFFFFu - c->len) return finish(c, SER_FLASH_E_ARGS);
    if (c->mode == SER_FLASH_AVR && (c->address & 1u)) return finish(c, SER_FLASH_E_ARGS);
    uint32_t mounts; uint16_t vid, pid;
    if (!c->ops.port(c->ops.io, &mounts, &vid, &pid)) return finish(c, SER_FLASH_E_IO);
    c->total = c->len;
    if (app_needs_touch(vid, pid)) { c->phase = SER_PH_TOUCH; return SER_FLASH_RUNNING; }
    if (c->mode == SER_FLASH_SAMD) {
        c->proto = SER_PROTO_SAMBA;
        c->attempt = 0;
        c->phase = SER_PH_SYNC;
    } else if (caterina_boot(vid, pid)) {
        c->proto = SER_PROTO_AVR109;
        c->attempt = 0;
        c->phase = SER_PH_SYNC;
    } else {
        c->probe = 0;
        c->phase = SER_PH_RESET;
    }
    return SER_FLASH_RUNNING;
}

static int step_touch(ser_flash_ctx_t *c)
{
    uint16_t vid, pid;
    if (!c->ops.port(c->ops.io, &c->mounts, &vid, &pid)) return finish(c, SER_FLASH_E_IO);
    /* Open at 1200 baud and drop DTR: the sketch's USB code resets into the bootloader. The board
       may already be gone when the last request goes out, so errors do not count here. */
    (void)c->ops.lines(c->ops.io, true, true);
    (void)c->ops.baud(c->ops.io, 1200);
    (void)c->ops.lines(c->ops.io, false, false);
    c->waited = 0;
    c->phase = SER_PH_REATTACH;
    return SER_FLASH_RUNNING;
}

static int step_reattach(ser_flash_ctx_t *c)
{
    uint32_t m; uint16_t vid, pid;
    if (c->ops.port(c->ops.io, &m, &vid, &pid) && m != c->mounts) {
        c->ops.delay_ms(c->ops.io, 300);   /* let the bootloader settle */
        (void)c->ops.baud(c->ops.io, c->mode == SER_FLASH_SAMD ? 115200 : 57600);
        (void)c->ops.lines(c->ops.io, true, true);
        c->proto = c->mode == SER_FLASH_SAMD ? SER_PROTO_SAMBA : SER_PROTO_AVR109;
        c->mounts = m;
        c->attempt = 0;
        c->phase = SER_PH_SYNC;
        return SER_FLASH_RUNNING;
    }
    if (c->waited >= REATTACH_MS)
        return finish(c, c->touch_fallback ? SER_FLASH_E_NO_LOADER : SER_FLASH_E_REATTACH);
    c->ops.delay_ms(c->ops.io, 100);
    c->waited += 100;
    return SER_FLASH_RUNNING;
}

/* AVR, unknown bootloader: set the probe's rate and reset the board with DTR, as avrdude's
   arduino programmer does. */
static int step_reset(ser_flash_ctx_t *c)
{
    uint8_t proto; uint32_t baud;
    probe_get(c->probe, &proto, &baud);
    c->proto = proto;
    c->baud = baud;
    if (c->ops.baud(c->ops.io, baud) != 0) return finish(c, SER_FLASH_E_IO);
    if (c->ops.lines(c->ops.io, false, false) != 0) return finish(c, SER_FLASH_E_IO);
    c->ops.delay_ms(c->ops.io, 250);
    if (c->ops.lines(c->ops.io, true, true) != 0) return finish(c, SER_FLASH_E_IO);
    c->ops.delay_ms(c->ops.io, 50);
    c->ops.rx_clear(c->ops.io);
    c->attempt = 0;
    c->phase = SER_PH_SYNC;
    return SER_FLASH_RUNNING;
}

static int sync_failed(ser_flash_ctx_t *c)
{
    if (c->proto == SER_PROTO_SAMBA) {
        if (c->mounts == 0) {   /* not touched yet: maybe a sketch that knows the touch */
            c->touch_fallback = true;
            c->phase = SER_PH_TOUCH;
            return SER_FLASH_RUNNING;
        }
        return finish(c, SER_FLASH_E_NO_LOADER);
    }
    if (c->proto == SER_PROTO_AVR109 && c->mounts != 0) return finish(c, SER_FLASH_E_NO_LOADER);
    if (c->probe + 1u < NPROBES && c->mounts == 0) {
        c->probe++;
        c->phase = SER_PH_RESET;
        return SER_FLASH_RUNNING;
    }
    return finish(c, SER_FLASH_E_NO_LOADER);
}

static int step_sync(ser_flash_ctx_t *c)
{
    bool ok = false;
    uint32_t tries = OTHER_TRIES;
    switch (c->proto) {
    case SER_PROTO_STK500V1: {
        static const uint8_t cmd[2] = { '0', CRC_EOP };
        uint8_t r[2];
        tries = V1_SYNC_TRIES;
        ok = xchg(c, cmd, 2, r, 2, SYNC_MS) && r[0] == STK_INSYNC && r[1] == STK_OK;
        break;
    }
    case SER_PROTO_STK500V2: {
        static const uint8_t b[1] = { V2_SIGN_ON };
        int n = v2(c, b, 1, c->buf, 64, REPLY_MS);
        ok = n >= 3;
        break;
    }
    case SER_PROTO_AVR109: {
        static const uint8_t cmd[1] = { 'S' };
        uint8_t id[7];
        ok = xchg(c, cmd, 1, id, 7, REPLY_MS);
        for (uint32_t i = 0; ok && i < 7; i++) ok = id[i] >= 0x20 && id[i] < 0x7F;
        break;
    }
    case SER_PROTO_SAMBA: {
        static const uint8_t n_cmd[2] = { 'N', '#' }, v_cmd[2] = { 'V', '#' };
        uint8_t r[2];
        if (!xchg(c, n_cmd, 2, r, 2, REPLY_MS) || r[0] != '\n' || r[1] != '\r') break;
        c->ops.rx_clear(c->ops.io);
        if (c->ops.write(c->ops.io, v_cmd, 2) != 0) return finish(c, SER_FLASH_E_IO);
        uint32_t n = c->ops.read(c->ops.io, c->buf, 96, 500);
        if (n < 4) break;
        if (!samba_fast(c->buf, n)) return finish(c, SER_FLASH_E_UNSUPPORTED);
        ok = true;
        break;
    }
    default:
        return finish(c, SER_FLASH_E_ARGS);
    }
    if (ok) { c->phase = SER_PH_INFO; return SER_FLASH_RUNNING; }
    if (++c->attempt < tries) return SER_FLASH_RUNNING;
    return sync_failed(c);
}

static int check_fit(ser_flash_ctx_t *c, uint32_t boot)
{
    if (c->page == 0 || c->page > SER_FLASH_BUF) return finish(c, SER_FLASH_E_REPLY);
    if (c->address % c->page) return finish(c, SER_FLASH_E_ARGS);
    if (c->flash_size && (c->address + c->len > c->flash_size - boot)) return finish(c, SER_FLASH_E_FIT);
    return SER_FLASH_RUNNING;
}

static int step_info(ser_flash_ctx_t *c)
{
    uint32_t boot = 0;
    switch (c->proto) {
    case SER_PROTO_STK500V1: {
        static const uint8_t sig_cmd[2] = { 'u', CRC_EOP }, prog[2] = { 'P', CRC_EOP };
        if (!v1(c, sig_cmd, 2, c->sig, 3, REPLY_MS)) return finish(c, SER_FLASH_E_REPLY);
        if (!avr_chip(c->sig, &c->flash_size, &boot, &c->page)) { c->flash_size = 0; c->page = 128; }
        if (!v1(c, prog, 2, NULL, 0, REPLY_MS)) return finish(c, SER_FLASH_E_REPLY);
        break;
    }
    case SER_PROTO_STK500V2: {
        for (uint8_t i = 0; i < 3; i++) {
            uint8_t b[6] = { V2_READ_SIG, 0x04, 0x30, 0x00, i, 0x00 };
            uint8_t a[8];
            if (v2(c, b, sizeof(b), a, sizeof(a), REPLY_MS) < 3) return finish(c, SER_FLASH_E_REPLY);
            c->sig[i] = a[2];
        }
        static const uint8_t enter[12] = { V2_ENTER_PROG, 200, 100, 25, 32, 0, 0x53, 3, 0xAC, 0x53, 0, 0 };
        uint8_t a[4];
        if (v2(c, enter, sizeof(enter), a, sizeof(a), REPLY_MS) != 2) return finish(c, SER_FLASH_E_REPLY);
        if (!avr_chip(c->sig, &c->flash_size, &boot, &c->page)) { c->flash_size = 0; c->page = 256; }
        break;
    }
    case SER_PROTO_AVR109: {
        static const uint8_t s_cmd[1] = { 's' }, b_cmd[1] = { 'b' }, p_cmd[1] = { 'P' };
        uint8_t s[3], b[3];
        if (!xchg(c, s_cmd, 1, s, 3, REPLY_MS)) return finish(c, SER_FLASH_E_REPLY);
        c->sig[0] = s[2]; c->sig[1] = s[1]; c->sig[2] = s[0];   /* AVR109 sends it last byte first */
        if (!xchg(c, b_cmd, 1, b, 3, REPLY_MS) || b[0] != 'Y') return finish(c, SER_FLASH_E_REPLY);
        uint32_t block = ((uint32_t)b[1] << 8) | b[2];
        uint32_t page;
        if (!avr_chip(c->sig, &c->flash_size, &boot, &page)) c->flash_size = 0;
        c->page = block > 256u ? 256u : block;   /* Caterina says 128 */
        c->ops.rx_clear(c->ops.io);
        if (c->ops.write(c->ops.io, p_cmd, 1) != 0 || !cr(c, REPLY_MS)) return finish(c, SER_FLASH_E_REPLY);
        break;
    }
    case SER_PROTO_SAMBA: {
        uint32_t param;
        if (!samba_word(c, SAMD_DID, &c->chip_id) || !samba_word(c, SAMD_NVM_PARAM, &param))
            return finish(c, SER_FLASH_E_REPLY);
        uint32_t psz = (param >> 16) & 7u, nvmp = param & 0xFFFFu;
        c->page = 8u << psz;
        c->flash_size = c->page * nvmp;
        uint32_t cpu = c->chip_id >> 28;
        if (cpu == 1) boot = 0x2000;          /* SAMD21 / SAMR21 / SAML21: Cortex-M0+, 8 KB bootloader */
        else if (cpu == 6) boot = 0x4000;     /* SAMD51 / SAME5x: Cortex-M4, 16 KB bootloader */
        else return finish(c, SER_FLASH_E_REPLY);
        if (c->flash_size == 0 || c->page > SAMBA_CHUNK) return finish(c, SER_FLASH_E_REPLY);
        c->boot_end = boot;
        if (c->address < boot) return finish(c, SER_FLASH_E_FIT);
        if (c->address % c->page) return finish(c, SER_FLASH_E_ARGS);
        if (c->address + c->len > c->flash_size) return finish(c, SER_FLASH_E_FIT);
        c->pos = 0;
        c->phase = SER_PH_ERASE;
        return SER_FLASH_RUNNING;
    }
    default:
        return finish(c, SER_FLASH_E_ARGS);
    }
    int rc = check_fit(c, boot);
    if (rc != SER_FLASH_RUNNING) return rc;
    c->pos = 0;
    c->phase = c->proto == SER_PROTO_AVR109 ? SER_PH_ERASE : SER_PH_WRITE;
    return SER_FLASH_RUNNING;
}

static int step_erase(ser_flash_ctx_t *c)
{
    if (c->proto == SER_PROTO_AVR109) {
        static const uint8_t e[1] = { 'e' };
        c->ops.rx_clear(c->ops.io);
        if (c->ops.write(c->ops.io, e, 1) != 0 || !cr(c, ERASE_AVR_MS)) return finish(c, SER_FLASH_E_REPLY);
    } else {
        char cmd[16];
        uint32_t n = samba_cmd(cmd, 'X', c->address, false, 0);
        c->ops.rx_clear(c->ops.io);
        if (c->ops.write(c->ops.io, (const uint8_t *)cmd, n) != 0 || !samba_ack(c, 'X', ERASE_SAMD_MS))
            return finish(c, SER_FLASH_E_REPLY);
    }
    c->phase = SER_PH_WRITE;
    return SER_FLASH_RUNNING;
}

static int step_write(ser_flash_ctx_t *c)
{
    uint32_t left = c->len - c->pos;
    uint32_t addr = c->address + c->pos;
    if (c->proto == SER_PROTO_SAMBA) {
        uint32_t n = left < SAMBA_CHUNK ? left : SAMBA_CHUNK;
        uint32_t size = (n + c->page - 1) / c->page * c->page;
        char cmd[24];
        uint32_t k = samba_cmd(cmd, 'S', SAMBA_RAM, true, size);
        c->ops.rx_clear(c->ops.io);
        if (c->ops.write(c->ops.io, (const uint8_t *)cmd, k) != 0) return finish(c, SER_FLASH_E_IO);
        for (uint32_t off = 0; off < size; off += SAMBA_READ) {
            uint32_t piece = size - off < SAMBA_READ ? size - off : SAMBA_READ;
            uint32_t have = off < n ? (n - off < piece ? n - off : piece) : 0;
            if (!image(c, c->pos + off, c->buf, have, piece)) return finish(c, SER_FLASH_E_SRC);
            if (c->ops.write(c->ops.io, c->buf, piece) != 0) return finish(c, SER_FLASH_E_IO);
        }
        k = samba_cmd(cmd, 'Y', SAMBA_RAM, true, 0);
        if (c->ops.write(c->ops.io, (const uint8_t *)cmd, k) != 0 || !samba_ack(c, 'Y', REPLY_MS))
            return finish(c, SER_FLASH_E_REPLY);
        k = samba_cmd(cmd, 'Y', addr, true, size);
        c->ops.rx_clear(c->ops.io);
        if (c->ops.write(c->ops.io, (const uint8_t *)cmd, k) != 0 || !samba_ack(c, 'Y', 5000))
            return finish(c, SER_FLASH_E_REPLY);
        c->pos += n;
    } else {
        uint32_t n = left < c->page ? left : c->page;
        uint8_t *data = c->buf + 16;
        if (!image(c, c->pos, data, n, c->page)) return finish(c, SER_FLASH_E_SRC);
        uint32_t p = c->page;
        bool ok;
        if (c->proto == SER_PROTO_STK500V1) {
            uint8_t *h = data - 4;
            h[0] = 'd'; h[1] = (uint8_t)(p >> 8); h[2] = (uint8_t)p; h[3] = 'F';
            data[p] = CRC_EOP;
            ok = v1_addr(c, addr) && v1(c, h, p + 5, NULL, 0, PAGE_MS);
        } else if (c->proto == SER_PROTO_STK500V2) {
            uint8_t *h = data - 10;
            h[0] = V2_PROGRAM_FLASH; h[1] = (uint8_t)(p >> 8); h[2] = (uint8_t)p;
            h[3] = 0xC1; h[4] = 10; h[5] = 0x40; h[6] = 0x4C; h[7] = 0x20; h[8] = 0; h[9] = 0;
            uint8_t a[4];
            ok = v2_addr(c, addr) && v2(c, h, p + 10, a, sizeof(a), PAGE_MS) == 2;
        } else {
            uint8_t *h = data - 4;
            h[0] = 'B'; h[1] = (uint8_t)(p >> 8); h[2] = (uint8_t)p; h[3] = 'F';
            ok = a109_addr(c, addr);
            if (ok) {
                c->ops.rx_clear(c->ops.io);
                ok = c->ops.write(c->ops.io, h, p + 4) == 0 && cr(c, PAGE_MS);
            }
        }
        if (!ok) return finish(c, SER_FLASH_E_REPLY);
        c->pos += n;
    }
    c->done = c->pos;
    if (c->pos >= c->len) { c->pos = 0; c->done = 0; c->phase = SER_PH_VERIFY; }
    return SER_FLASH_RUNNING;
}

static int step_verify(ser_flash_ctx_t *c)
{
    uint32_t left = c->len - c->pos;
    uint32_t addr = c->address + c->pos;
    uint32_t unit = c->proto == SER_PROTO_SAMBA ? SAMBA_READ : c->page;
    uint32_t n = left < unit ? left : unit;
    uint8_t *want = c->buf;
    uint8_t *got = c->buf + 512;   /* pages are at most 512 bytes for AVR; SAM-BA reads use buf below */
    bool ok;
    if (c->proto == SER_PROTO_SAMBA) {
        /* 1 KB read: the image goes to buf[0..1023], the board's bytes compared as they come. */
        char cmd[24];
        uint32_t k = samba_cmd(cmd, 'R', addr, true, n);
        if (!image(c, c->pos, want, n, n)) return finish(c, SER_FLASH_E_SRC);
        c->ops.rx_clear(c->ops.io);
        if (c->ops.write(c->ops.io, (const uint8_t *)cmd, k) != 0) return finish(c, SER_FLASH_E_IO);
        uint8_t piece[32];
        for (uint32_t off = 0; off < n; off += sizeof(piece)) {
            uint32_t m = n - off < sizeof(piece) ? n - off : sizeof(piece);
            if (c->ops.read(c->ops.io, piece, m, PAGE_MS) != m) return finish(c, SER_FLASH_E_REPLY);
            if (memcmp(piece, want + off, m) != 0) return finish(c, SER_FLASH_E_VERIFY);
        }
        ok = true;
    } else {
        if (c->page > 512) return finish(c, SER_FLASH_E_REPLY);
        if (!image(c, c->pos, want, n, n)) return finish(c, SER_FLASH_E_SRC);
        uint32_t p = c->page;
        if (c->proto == SER_PROTO_STK500V1) {
            uint8_t cmd[5] = { 't', (uint8_t)(p >> 8), (uint8_t)p, 'F', CRC_EOP };
            ok = v1_addr(c, addr) && v1(c, cmd, sizeof(cmd), got, p, PAGE_MS);
        } else if (c->proto == SER_PROTO_STK500V2) {
            uint8_t b[4] = { V2_READ_FLASH, (uint8_t)(p >> 8), (uint8_t)p, 0x20 };
            int len = v2_addr(c, addr) ? v2(c, b, sizeof(b), got, p + 3, PAGE_MS) : -1;
            ok = len == (int)(p + 3);
            if (ok) memmove(got, got + 2, p);
        } else {
            uint8_t cmd[4] = { 'g', (uint8_t)(p >> 8), (uint8_t)p, 'F' };
            ok = a109_addr(c, addr) && xchg(c, cmd, 4, got, p, PAGE_MS);
        }
        if (!ok) return finish(c, SER_FLASH_E_REPLY);
        if (memcmp(got, want, n) != 0) return finish(c, SER_FLASH_E_VERIFY);
    }
    c->pos += n;
    c->done = c->pos;
    if (c->pos >= c->len) c->phase = SER_PH_LEAVE;
    return SER_FLASH_RUNNING;
}

static int step_leave(ser_flash_ctx_t *c)
{
    switch (c->proto) {
    case SER_PROTO_STK500V1: {
        static const uint8_t q[2] = { 'Q', CRC_EOP };
        if (c->leave && !v1(c, q, 2, NULL, 0, REPLY_MS)) return finish(c, SER_FLASH_E_REPLY);
        break;
    }
    case SER_PROTO_STK500V2: {
        static const uint8_t b[3] = { V2_LEAVE_PROG, 1, 1 };
        uint8_t a[4];
        if (c->leave && v2(c, b, sizeof(b), a, sizeof(a), REPLY_MS) != 2) return finish(c, SER_FLASH_E_REPLY);
        break;
    }
    case SER_PROTO_AVR109: {
        static const uint8_t l[1] = { 'L' }, e[1] = { 'E' };
        c->ops.rx_clear(c->ops.io);
        if (c->ops.write(c->ops.io, l, 1) != 0 || !cr(c, REPLY_MS)) return finish(c, SER_FLASH_E_REPLY);
        if (c->leave) {
            c->ops.rx_clear(c->ops.io);
            if (c->ops.write(c->ops.io, e, 1) != 0 || !cr(c, REPLY_MS)) return finish(c, SER_FLASH_E_REPLY);
        }
        break;
    }
    case SER_PROTO_SAMBA: {
        if (c->leave) {
            /* SCB->AIRCR = VECTKEY | SYSRESETREQ: the board resets into the new sketch (no reply). */
            char cmd[24];
            uint32_t n = samba_cmd(cmd, 'W', 0xE000ED0Cu, true, 0x05FA0004u);
            if (c->ops.write(c->ops.io, (const uint8_t *)cmd, n) != 0) return finish(c, SER_FLASH_E_IO);
        }
        break;
    }
    default:
        break;
    }
    c->done = c->total;
    return finish(c, SER_FLASH_DONE);
}

int ser_flash_step(ser_flash_ctx_t *c)
{
    switch (c->phase) {
    case SER_PH_START:    return step_start(c);
    case SER_PH_TOUCH:    return step_touch(c);
    case SER_PH_REATTACH: return step_reattach(c);
    case SER_PH_RESET:    return step_reset(c);
    case SER_PH_SYNC:     return step_sync(c);
    case SER_PH_INFO:     return step_info(c);
    case SER_PH_ERASE:    return step_erase(c);
    case SER_PH_WRITE:    return step_write(c);
    case SER_PH_VERIFY:   return step_verify(c);
    case SER_PH_LEAVE:    return step_leave(c);
    default:              return c->result;
    }
}
