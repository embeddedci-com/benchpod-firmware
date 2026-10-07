/*
 * test_bus_owner.c — host tests for the shared-bus owner token and depth (src/bus_owner.c).
 */
#include "bus_owner.h"

#include <stdio.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static int task_a, task_b;   /* owner tokens: any distinct addresses */

static void test_single_hold_switches_twice(void)
{
    bus_owner_t b = {0};
    CHECK(bus_owner_take(&b, &task_a) == 1);   /* free -> ours: switch the pins */
    CHECK(b.depth == 1 && b.owner == &task_a);
    CHECK(bus_owner_give(&b, &task_a) == 1);   /* outermost give: back to the iCE40 */
    CHECK(b.depth == 0 && b.owner == 0);
    CHECK(b.violations == 0 && b.unpaired == 0);
}

/* The bug this exists for: an inner w25q_open/close inside an outer holder released the bus. */
static void test_nested_close_keeps_the_bus(void)
{
    bus_owner_t b = {0};
    CHECK(bus_owner_take(&b, &task_a) == 1);   /* outer: e.g. ota_commit */
    CHECK(bus_owner_take(&b, &task_a) == 0);   /* inner: w25q_open */
    CHECK(bus_owner_give(&b, &task_a) == 0);   /* inner close: still held, pins stay */
    CHECK(b.depth == 1);
    CHECK(bus_owner_give(&b, &task_a) == 1);   /* outer release hands it back */
    CHECK(b.violations == 0);
}

static void test_other_owner_is_a_violation(void)
{
    bus_owner_t b = {0};
    bus_owner_take(&b, &task_a);
    CHECK(bus_owner_take(&b, &task_b) == 0);   /* counted, not a pin switch */
    CHECK(b.violations == 1);
    CHECK(b.owner == &task_a);                 /* the first holder stays the owner */
    CHECK(bus_owner_give(&b, &task_b) == 0);   /* a give by the non-holder: counted, balances */
    CHECK(b.violations == 2);
    CHECK(bus_owner_give(&b, &task_a) == 1);
}

static void test_unpaired_give_hands_back(void)
{
    bus_owner_t b = {0};
    CHECK(bus_owner_give(&b, &task_a) == 1);   /* "hand the bus to the iCE40" with nothing held */
    CHECK(b.unpaired == 1 && b.violations == 0 && b.depth == 0);
}

static void test_handover_drops_and_reports_leaks(void)
{
    bus_owner_t b = {0};
    CHECK(bus_owner_handover(&b) == 0);        /* nothing held: the normal case */
    CHECK(b.violations == 0);
    bus_owner_take(&b, &task_a);
    bus_owner_take(&b, &task_a);               /* a take that was never given back */
    CHECK(bus_owner_handover(&b) == 2);
    CHECK(b.violations == 1 && b.depth == 0 && b.owner == 0);
    CHECK(bus_owner_take(&b, &task_b) == 1);   /* free again: the next take switches */
}

int main(void)
{
    test_single_hold_switches_twice();
    test_nested_close_keeps_the_bus();
    test_other_owner_is_a_violation();
    test_unpaired_give_hands_back();
    test_handover_drops_and_reports_leaks();
    if (failures) { printf("test_bus_owner: %d FAILURE(S)\n", failures); return 1; }
    printf("test_bus_owner: all passed\n");
    return 0;
}
