/*
 * reply_budget.h: how close a reply's worst case comes to the limit of the channel it goes out on.
 *
 * A reply that outgrows its limit fails as "reply too large" (cloud_reply_cap.c, send_emitted),
 * and the first field that tips it over is usually added for one board, far from the test that
 * measured the worst case. So every reply with a fixed limit and no cut of its own keeps
 * REPLY_BUDGET_MARGIN bytes free at its worst case, and the tests print every budget in one
 * format ("reply budget: ...") so a run shows them side by side.
 *
 * Replies that cut themselves to fit are not budgeted here: sensor_status and wifi_scan page or
 * truncate, the capabilities frame drops boot health, a sensor define reply drops params, CAN
 * replies leave frames queued. Their own tests check the cut.
 *
 * A reply already over its budget is listed with reply_budget_known(): the test records the
 * size measured when it was listed and fails when it grows, until it is fixed.
 */
#ifndef REPLY_BUDGET_H
#define REPLY_BUDGET_H

#include <stdio.h>
#include <stddef.h>
#include "bp_limits.h"

/* Bytes a worst-case reply keeps free under its limit: room for a few more fields. */
#define REPLY_BUDGET_MARGIN 64u

/* A single-reply command over the cloud: the reply and its newline in BP_CLOUD_REPLY_MAX with the
   NUL (cloud_reply_cap.h). The handlers' own reply buffers are the same size. */
#define REPLY_LIMIT_CLOUD   (BP_CLOUD_REPLY_MAX - 1u)

static inline void reply_budget_print(const char *what, size_t len, size_t limit, size_t margin) {
    long free_b = (long)limit - (long)len;
    printf("  reply budget: %-44s %5zu of %5zu bytes, %5ld free (want >= %zu)\n",
           what, len, limit, free_b, margin);
}

/* True when `len` (the reply with its newline) leaves `margin` bytes under `limit`. */
static inline int reply_budget_ok(const char *what, size_t len, size_t limit, size_t margin) {
    reply_budget_print(what, len, limit, margin);
    return len + margin <= limit;
}

/* A reply known to be over its budget: true while it is no larger than when it was listed. */
static inline int reply_budget_known(const char *what, size_t len, size_t limit, size_t listed) {
    reply_budget_print(what, len, limit, REPLY_BUDGET_MARGIN);
    printf("  reply budget: %-44s KNOWN over budget (listed at %zu bytes); it must not grow\n", what, listed);
    return len <= listed;
}

#endif /* REPLY_BUDGET_H */
