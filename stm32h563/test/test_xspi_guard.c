/* Host test for xspi_guard.c: with nothing mapped the guard does nothing; while mapped, the
   outermost enter/exit pair unmaps and maps once; failures are reported and counted. */
#include "xspi_guard.h"

#include <stdio.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static int unmaps, maps, fail_unmap, fail_map;
static int f_unmap(void) { unmaps++; return fail_unmap ? -1 : 0; }
static int f_map(void)   { maps++;   return fail_map ? -1 : 0; }
static const xspi_guard_hooks_t hooks = { f_unmap, f_map };

static void reset(void) { unmaps = maps = fail_unmap = fail_map = 0; }

int main(void)
{
    /* Not mapped: no hooks called, always ok. */
    xspi_guard_tok_t t = xspi_guard_enter();
    CHECK(t.ok && !t.unmapped);
    xspi_guard_exit(t);
    CHECK(!xspi_guard_mapped());

    /* Mapped: one unmap/map per outermost pair, none for nested ones. */
    reset();
    xspi_guard_set_mapped(&hooks);
    CHECK(xspi_guard_mapped());
    t = xspi_guard_enter();
    CHECK(t.ok && t.unmapped && unmaps == 1 && maps == 0);
    xspi_guard_tok_t inner = xspi_guard_enter();
    CHECK(inner.ok && !inner.unmapped && unmaps == 1);
    CHECK(xspi_guard_stats().depth == 2);
    xspi_guard_exit(inner);
    CHECK(maps == 0);
    xspi_guard_exit(t);
    CHECK(maps == 1 && xspi_guard_stats().depth == 0);

    /* Back to back. */
    for (int i = 0; i < 5; i++) xspi_guard_exit(xspi_guard_enter());
    CHECK(unmaps == 6 && maps == 6 && xspi_guard_stats().unmaps == 6);

    /* Unmap fails: the caller is told not to send, nothing is mapped again, depth balances. */
    fail_unmap = 1;
    t = xspi_guard_enter();
    CHECK(!t.ok && !t.unmapped);
    xspi_guard_exit(t);
    CHECK(maps == 6 && xspi_guard_stats().depth == 0 && xspi_guard_stats().unmap_failures == 1);

    /* Map fails: counted (the session treats it as fatal). */
    fail_unmap = 0; fail_map = 1;
    xspi_guard_exit(xspi_guard_enter());
    CHECK(xspi_guard_stats().map_failures == 1);

    /* Unmapped for good: the guard is a pass-through again. */
    reset();
    xspi_guard_set_mapped(NULL);
    xspi_guard_exit(xspi_guard_enter());
    CHECK(unmaps == 0 && maps == 0 && !xspi_guard_mapped());

    printf("test_xspi_guard: %s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
