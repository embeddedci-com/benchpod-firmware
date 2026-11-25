/* prog_desc.c: see prog_desc.h. Internal flash (the request parsing is not part of the XIP image). */
#include "prog_desc.h"
#include "prog_pic.h"
#include "prog_avr12.h"
#include "prog_c2.h"
#include "bp_json.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool get_u32(const char *json, const char *key, uint32_t *out)
{
    char s[16];
    if (!bp_json_get(json, key, s, sizeof(s))) return false;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 0);
    if (!end || *end != '\0') return false;
    *out = (uint32_t)v;
    return true;
}

static bool has(const char *json, const char *key)
{
    char t[24];
    return bp_json_get(json, key, t, sizeof(t));
}

static int fail(char *err, size_t cap, const char *what, const char *key)
{
    if (err && cap) snprintf(err, cap, "%s %s", what, key);
    return -1;
}

/* A boolean field: absent = false; true/false/1/0 (bp_json_flag: 1, t, T, y, Y are true). */
static bool flag(const char *json, const char *key) { return bp_json_flag(json, key); }

/* Reject a key that belongs to another protocol. */
static int only_for(const char *json, const char *proto, const char *want,
                    const char *const *keys, size_t n, char *err, size_t cap)
{
    if (strcmp(proto, want) == 0) return 0;
    for (size_t i = 0; i < n; i++)
        if (has(json, keys[i])) {
            if (err && cap) snprintf(err, cap, "%s is for %s only", keys[i], want);
            return -1;
        }
    return 0;
}

int prog_desc_parse(const char *json, const char *proto, prog_desc_t *d, char *err, size_t cap)
{
    static const char *const k_pic[] = { "cmd_set", "pc", "ee_erase", "cp_high", "q10", "id_split",
                                         "key_flip", "cp_bit" };
    static const char *const k_updi[] = { "flash_base" };
    static const char *const k_tpi[] = { "write_words", "page_erase" };
    static const char *const k_c2[] = { "fpdat", "no_setup" };
    memset(d, 0, sizeof(*d));
    if (only_for(json, proto, "pic",  k_pic,  sizeof(k_pic) / sizeof(k_pic[0]), err, cap) ||
        only_for(json, proto, "updi", k_updi, 1, err, cap) ||
        only_for(json, proto, "tpi",  k_tpi,  2, err, cap) ||
        only_for(json, proto, "c2",   k_c2,   2, err, cap))
        return -1;

    bool legacy = has(json, "flags");
    uint32_t v;
    bool named = false;
    uint32_t f = 0;

    if (strcmp(proto, "updi") == 0) {
        if (has(json, "flash_base")) {
            if (!get_u32(json, "flash_base", &v)) return fail(err, cap, "invalid", "flash_base");
            f = v; named = true;
        }
    } else if (strcmp(proto, "pic") == 0) {
        char s[8];
        if (has(json, "cmd_set")) {
            bp_json_get(json, "cmd_set", s, sizeof(s));
            if (strcmp(s, "8bit") == 0) f |= PIC_F_SET8;
            else if (strcmp(s, "6bit") != 0) return fail(err, cap, "invalid", "cmd_set (6bit or 8bit)");
            named = true;
        }
        if (has(json, "pc")) {
            bp_json_get(json, "pc", s, sizeof(s));
            if (strcmp(s, "byte") == 0) f |= PIC_F_BYTEPC;
            else if (strcmp(s, "word") != 0) return fail(err, cap, "invalid", "pc (word or byte)");
            named = true;
        }
        static const struct { const char *key; uint32_t bit; } k_bool[] = {
            { "ee_erase", PIC_F_HAS_EE }, { "cp_high", PIC_F_CP_HIGH }, { "q10", PIC_F_Q10 },
            { "id_split", PIC_F_ID_SPLIT }, { "key_flip", PIC_F_KEY_FLIP } };
        for (size_t i = 0; i < sizeof(k_bool) / sizeof(k_bool[0]); i++)
            if (has(json, k_bool[i].key)) { if (flag(json, k_bool[i].key)) f |= k_bool[i].bit; named = true; }
        if (has(json, "cp_bit")) {
            if (!get_u32(json, "cp_bit", &v) || v > 15) return fail(err, cap, "invalid", "cp_bit (0..15)");
            f |= PIC_F_CP_BIT(v); named = true;
        }
        if ((f & PIC_F_BYTEPC) && !(f & PIC_F_SET8)) return fail(err, cap, "pc byte needs", "cmd_set 8bit");
        if ((f & PIC_F_Q10) && !(f & PIC_F_BYTEPC)) return fail(err, cap, "q10 needs", "pc byte");
    } else if (strcmp(proto, "tpi") == 0) {
        if (has(json, "write_words")) {
            if (!get_u32(json, "write_words", &v) || (v != 1 && v != 2 && v != 4))
                return fail(err, cap, "invalid", "write_words (1, 2 or 4)");
            f |= v;   /* TPI_F_WORDS_MASK holds the count; 1 reads the same as 0 */
            named = true;
        }
        if (has(json, "page_erase")) { if (flag(json, "page_erase")) f |= TPI_F_PAGE_ERASE; named = true; }
    } else if (strcmp(proto, "c2") == 0) {
        if (has(json, "fpdat")) {
            if (!get_u32(json, "fpdat", &v) || v > 255) return fail(err, cap, "invalid", "fpdat (0..255)");
            f |= v; named = true;
        }
        if (has(json, "no_setup")) { if (flag(json, "no_setup")) f |= C2_FLAG_NO_SETUP; named = true; }
    }

    if (named && legacy) return fail(err, cap, "flags cannot be mixed with the named fields of", proto);
    if (legacy) {
        if (!get_u32(json, "flags", &v)) return fail(err, cap, "invalid", "flags");
        f = v;
    }
    d->flags = f;
    d->named = named;
    return 0;
}
