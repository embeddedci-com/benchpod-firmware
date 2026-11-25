/*
 * test_ice40_switch.c: host tests for src/ice40_switch.c, the decision whether a gateware image
 * switch loads anything, and the switch counters in status / fpga_image.
 */
#include <stdio.h>
#include <string.h>
#include "ice40_switch.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static void test_plan(void) {
    /* The image that runs, answering, healthy: skip. */
    for (int img = 0; img < 3; img++)
        CHECK(ice40_switch_plan(img, img, true, true, true, false) == ICE40_SW_SKIP, "running image %d not skipped", img);
    /* Another image: load. */
    CHECK(ice40_switch_plan(1, 0, true, true, true, false) == ICE40_SW_LOAD, "0 -> 1 skipped");
    CHECK(ice40_switch_plan(0, 2, true, true, true, false) == ICE40_SW_LOAD, "2 -> 0 skipped");
    /* Force always loads. */
    CHECK(ice40_switch_plan(0, 0, true, true, true, true) == ICE40_SW_LOAD, "force skipped");
    /* No proof: features read failed, CDONE low, or the PSRAM write path is broken (a reload is
       how it recovers): load. */
    CHECK(ice40_switch_plan(0, -1, true, true, true, false) == ICE40_SW_LOAD, "unknown running image skipped");
    CHECK(ice40_switch_plan(0, 0, false, true, true, false) == ICE40_SW_LOAD, "CDONE low skipped");
    CHECK(ice40_switch_plan(1, 1, true, false, true, false) == ICE40_SW_LOAD, "PSRAM wedged skipped");
    /* The running gateware is not the firmware's (a downgrade, a new blob): reload the kind that
       runs from its slot, which is what `benchpod install-blobs` asks for. */
    CHECK(ice40_switch_plan(0, 0, true, true, false, false) == ICE40_SW_LOAD, "stale gateware skipped");
}

static void test_counters(void) {
    ice40_switch_reset_stats();
    ice40_switch_note_skip();
    ice40_switch_note_switch(); ice40_switch_note_reflash(); ice40_switch_note_erases(27);
    ice40_switch_note_switch(); ice40_switch_note_failed();
    ice40_switch_note_slave(true); ice40_switch_note_slave(true); ice40_switch_note_slave(false);
    ice40_switch_note_time_ms(52);
    const ice40_switch_stats_t *st = ice40_switch_stats();
    CHECK(st->switches == 2 && st->skipped == 1 && st->reflashes == 1 && st->erases == 27 && st->failed == 1,
          "counters %u %u %u %u %u", st->switches, st->skipped, st->reflashes, st->erases, st->failed);
    char buf[200];
    bp_emit_t e;
    bp_emit_init(&e, buf, sizeof(buf));
    ice40_switch_emit(&e);
    CHECK(bp_emit_ok(&e), "emit overflowed");
    CHECK(!strcmp(buf, "\"fpga_switch\":{\"switches\":2,\"skipped\":1,\"failed\":1,\"slave_loads\":2,"
                       "\"slave_fails\":1,\"reflashes\":1,\"erases\":27,\"last_ms\":52}"),
          "emit: %s", buf);
    /* The longest it gets (every counter at its maximum) fits the fpga_image reply's room. */
    ice40_switch_reset_stats();
    ice40_switch_note_erases(4294967295u);
    for (int i = 0; i < 3; i++) { ice40_switch_note_skip(); }
    ice40_switch_note_time_ms(4294967295u);
    bp_emit_init(&e, buf, sizeof(buf));
    ice40_switch_emit(&e);
    CHECK(bp_emit_ok(&e) && strstr(buf, "\"erases\":4294967295"), "max emit: %s", buf);
    ice40_switch_reset_stats();
}

int main(void) {
    test_plan();
    test_counters();
    if (fails) { printf("test_ice40_switch: %d FAILED\n", fails); return 1; }
    printf("test_ice40_switch: OK\n");
    return 0;
}
