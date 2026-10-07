/*
 * psram_regions.c — staleness of the last capture in PSRAM (see psram_regions.h).
 */
#include "psram_regions.h"

#include <stddef.h>

#define REGIONS_MAX 2

static struct { uint32_t base, len; } s_r[REGIONS_MAX];
static int         s_n;
static const char *s_stale;

void psram_regions_track(uint32_t base0, uint32_t len0, uint32_t base1, uint32_t len1)
{
    s_n = 0;
    if (len0) { s_r[s_n].base = base0; s_r[s_n].len = len0; s_n++; }
    if (len1) { s_r[s_n].base = base1; s_r[s_n].len = len1; s_n++; }
    s_stale = NULL;
}

void psram_regions_forget(void)
{
    s_n = 0;
    s_stale = NULL;
}

void psram_regions_dirty(uint32_t base, uint32_t len, const char *why)
{
    if (len == 0 || s_stale) return;
    uint64_t end = (uint64_t)base + len;
    for (int i = 0; i < s_n; i++) {
        uint64_t rend = (uint64_t)s_r[i].base + s_r[i].len;
        if (base < rend && s_r[i].base < end) {
            s_stale = why ? why : "another PSRAM write";
            return;
        }
    }
}

void psram_regions_dirty_all(const char *why)
{
    if (s_n && !s_stale) s_stale = why ? why : "another PSRAM write";
}

const char *psram_regions_stale(void)
{
    return s_stale;
}
