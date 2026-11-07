/*
 * test_recovery.c — host unit test for the recovery ladder's decisions (recovery.h): the order of
 * the steps, their spacing, and the limits that stop a broken pod from cycling forever.
 */
#include "recovery.h"

#include <stdio.h>

static int failures;
#define CHECK(c) do { if(!(c)){printf("FAIL %s:%d  %s\n",__FILE__,__LINE__,#c);failures++;} } while(0)

/* Run the loop every 100 ms from t0 to t1 with captures failing since `fail_start` (0 = never
   failing), counting the actions taken. */
typedef struct { int reloads, resets, eth; uint32_t first_reload, first_reset; } tally_t;

static tally_t run(recovery_t *r, uint32_t t0, uint32_t t1, uint32_t fail_start, uint32_t link_down_start,
                   uint32_t *resets) {
    tally_t t = {0};
    for (uint32_t now = t0; now < t1; now += 100) {
        uint32_t fail = fail_start && now >= fail_start ? now - fail_start : 0;
        uint32_t down = link_down_start && now >= link_down_start ? now - link_down_start : 0;
        recovery_action_t a = recovery_poll(r, now, fail, down, resets);
        if (a == RECOVERY_NONE) continue;
        recovery_done(r, a, now, resets);
        if (a == RECOVERY_ICE40_RELOAD) { if (!t.reloads) t.first_reload = now; t.reloads++; }
        if (a == RECOVERY_MCU_RESET) { if (!t.resets) t.first_reset = now; t.resets++; }
        if (a == RECOVERY_ETH_RESTART) t.eth++;
    }
    return t;
}

static void test_healthy_does_nothing(void) {
    recovery_t r;
    uint32_t resets = 0;
    recovery_init(&r, 0);
    tally_t t = run(&r, 0, 3600000, 0, 0, &resets);
    CHECK(t.reloads == 0 && t.resets == 0 && t.eth == 0);
    /* A short failure (under RECOVERY_FAIL_MS) is left alone: the LED shows it, it recovers. */
    t = run(&r, 3600000, 3600000 + 9000, 3600000, 0, &resets);
    CHECK(t.reloads == 0 && t.resets == 0);
}

static void test_ladder_order(void) {
    recovery_t r;
    uint32_t resets = 0;
    recovery_init(&r, 0);
    /* Captures fail from 1000 ms on, for 5 minutes: reload at +10 s, again 30 s later, then one
       STM32 reset 30 s after that. Then the reset is "done": the caller resets and a new boot
       begins; here the same recovery_t keeps going, so check the limit too. */
    tally_t t = run(&r, 0, 300000, 1000, 0, &resets);
    CHECK(t.reloads == 2);
    CHECK(t.first_reload == 1000 + RECOVERY_FAIL_MS);
    CHECK(t.resets == RECOVERY_RESETS_MAX);
    CHECK(t.first_reset == t.first_reload + 2 * RECOVERY_RELOAD_GAP_MS);
    CHECK(resets == RECOVERY_RESETS_MAX);
    CHECK(recovery_exhausted(&r, 300000, 299000, resets));
    CHECK(!recovery_exhausted(&r, 300100, 299100, resets));   /* once per episode */
}

static void test_resets_limited_across_boots(void) {
    /* Each boot is a new recovery_t; only `resets` survives (noinit). */
    uint32_t resets = 0;
    int total = 0;
    for (int boot = 0; boot < 6; boot++) {
        recovery_t r;
        recovery_init(&r, 0);
        tally_t t = run(&r, 0, 200000, 1000, 0, &resets);
        total += t.resets;
        if (t.resets) {
            /* this boot ended in its reset */
        }
    }
    CHECK(total == RECOVERY_RESETS_MAX);
    /* A healthy stretch clears the count; a shorter one does not. */
    recovery_t r;
    recovery_init(&r, 0);
    run(&r, 0, RECOVERY_HEALTHY_MS - 1000, 0, 0, &resets);
    CHECK(resets == RECOVERY_RESETS_MAX);
    run(&r, RECOVERY_HEALTHY_MS - 1000, RECOVERY_HEALTHY_MS + 1000, 0, 0, &resets);
    CHECK(resets == 0);
}

static void test_reload_hourly_limit(void) {
    /* Failures that come back again and again: each episode gets its reloads, but never more
       than RECOVERY_RELOADS_PER_HOUR in an hour. With resets used up, nothing else happens. */
    recovery_t r;
    uint32_t resets = RECOVERY_RESETS_MAX;
    recovery_init(&r, 0);
    int reloads = 0;
    for (uint32_t ep = 0; ep < 12; ep++) {
        uint32_t t0 = ep * 240000u;   /* every 4 minutes: 2 min failing, 2 min working */
        reloads += run(&r, t0, t0 + 120000, t0 + 1, 0, &resets).reloads;
        run(&r, t0 + 120000, t0 + 240000, 0, 0, &resets);
    }
    /* 12 episodes in 48 minutes: at most 6 reloads. */
    CHECK(reloads == RECOVERY_RELOADS_PER_HOUR);
    CHECK(resets == RECOVERY_RESETS_MAX);
}

static void test_ethernet(void) {
    recovery_t r;
    uint32_t resets = 0;
    recovery_init(&r, 0);
    /* Cable out for 10 minutes: a PHY restart at +20 s, then one a minute; never an STM32 reset. */
    tally_t t = run(&r, 0, 600000, 0, 1, &resets);
    CHECK(t.resets == 0 && t.reloads == 0);
    CHECK(t.eth == RECOVERY_ETH_PER_HOUR);
    /* Out for the rest of the hour: no more than the hourly limit. */
    t = run(&r, 600000, 3500000, 0, 1, &resets);
    CHECK(t.eth == 0);
    /* Short drops (under RECOVERY_LINK_DOWN_MS) are left alone. */
    recovery_init(&r, 0);
    t = run(&r, 0, 19000, 0, 1, &resets);
    CHECK(t.eth == 0);
}

int main(void) {
    test_healthy_does_nothing();
    test_ladder_order();
    test_resets_limited_across_boots();
    test_reload_hourly_limit();
    test_ethernet();
    if (failures) { printf("test_recovery: %d FAILED\n", failures); return 1; }
    printf("test_recovery: all passed\n");
    return 0;
}
