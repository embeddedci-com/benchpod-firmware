/* xspi_guard.c — see xspi_guard.h. */
#include "xspi_guard.h"

#include <stddef.h>

static const xspi_guard_hooks_t *s_hooks;   /* non-NULL: the W25Q is mapped */
static uint16_t s_depth;                    /* nested enter() calls */
static xspi_guard_stats_t s_stats;

void xspi_guard_set_mapped(const xspi_guard_hooks_t *hooks)
{
    s_hooks = hooks;
    s_depth = 0;
}

bool xspi_guard_mapped(void) { return s_hooks != NULL; }

xspi_guard_tok_t xspi_guard_enter(void)
{
    xspi_guard_tok_t t = { false, true };
    if (!s_hooks) return t;
    if (s_depth++ == 0) {
        if (s_hooks->unmap() != 0) {
            s_stats.unmap_failures++;
            t.ok = false;
            return t;               /* still counted: exit() balances it */
        }
        s_stats.unmaps++;
        t.unmapped = true;
    }
    return t;
}

void xspi_guard_exit(xspi_guard_tok_t t)
{
    if (!s_hooks) return;
    if (s_depth) s_depth--;
    if (t.unmapped && s_depth == 0 && s_hooks->map() != 0) s_stats.map_failures++;
}

xspi_guard_stats_t xspi_guard_stats(void)
{
    xspi_guard_stats_t s = s_stats;
    s.depth = s_depth;
    return s;
}
