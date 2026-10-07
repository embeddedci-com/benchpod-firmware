/* Host unit test for dac_rearm.c: the stop-before-rearm sequence signal_engine.c runs before a
 * deep replay (always), a closed-loop arm and a BRAM waveform load (when STATUS says running).
 *
 * The fake fabric below keeps what the gateware keeps: the DAC engine's `running` (STATUS bit 0,
 * which takes a few STATUS reads to drop after STOP_DAC here), the deep-replay mode LEVEL that
 * only STOP_DAC clears, and a staged co-trigger that STOP_DAC cancels.  Each op is logged as one
 * letter (S = STATUS read, P = STOP_DAC, C = DAC_ARM_ON_CAPTURE) so the order is checked, not
 * only the end state: the stop must come before the start, and a co-trigger must be staged
 * again AFTER the stop, or the next start runs at once instead of at the capture's t0.
 */
#include "dac_rearm.h"
#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static struct {
    bool running;         /* engine running (STATUS DAC_RUN) */
    bool mode;            /* dac_psram_mode level: high = reader keeps its address and FIFO */
    bool cotrig;          /* dac_pend */
    int  stop_lag;        /* STATUS reads after STOP_DAC that still show DAC_RUN */
    int  lag_left;
    bool stopping;        /* STOP_DAC sent, DAC_RUN not dropped yet */
    bool stuck;          /* DAC_RUN never clears */
    bool status_bad;      /* STATUS read fails (bit 7 set) */
    char log[64];
} f;

static void fab_reset(void) { memset(&f, 0, sizeof f); }
static void logc(char c) { size_t n = strlen(f.log); if (n + 1 < sizeof f.log) f.log[n] = c; }

static int fake_status(uint8_t *st) {
    logc('S');
    if (f.status_bad) { *st = 0xFF; return -1; }
    if (f.stopping) {
        if (f.lag_left > 0) f.lag_left--;
        else { f.running = false; f.stopping = false; }
    }
    *st = f.running ? DAC_REARM_STATUS_DAC_RUN : 0u;
    return 0;
}
static void fake_stop(void) {
    logc('P');
    f.mode = false; f.cotrig = false;
    if (f.running && !f.stuck) { f.stopping = true; f.lag_left = f.stop_lag; }
}
static bool fake_arm(void) { logc('C'); f.cotrig = true; return true; }

static const dac_rearm_ops_t ops = { fake_status, fake_stop, fake_arm };

/* What signal_engine.c's dac_replay_psram now does, on the fake: quiesce(force), then
   START_DAC_PSRAM.  Returns whether the reader got the clean (mode low) edge it reloads on. */
static bool replay_start(bool cotrig_pending) {
    if (dac_rearm_quiesce(&ops, true, cotrig_pending) != 0) return false;
    bool clean = !f.mode;
    logc('R');
    f.mode = true;
    if (!f.cotrig) f.running = true;   /* a staged co-trigger defers the engine start to t0 */
    return clean;
}

int main(void) {
    /* 1. Idle, not forced: one STATUS read, no stop, no extra bus time. */
    fab_reset();
    CHECK(dac_rearm_quiesce(&ops, false, false) == 0, "idle: rc 0");
    CHECK(strcmp(f.log, "S") == 0, "idle: expected one STATUS read, got \"%s\"", f.log);

    /* 2. Running, not forced: stop, then poll until DAC_RUN clears. */
    fab_reset(); f.running = true; f.stop_lag = 2;
    CHECK(dac_rearm_quiesce(&ops, false, false) == 0, "running: rc 0");
    CHECK(strcmp(f.log, "SPSSS") == 0, "running: expected SPSSS, got \"%s\"", f.log);
    CHECK(!f.running && !f.mode, "running: DAC must be idle after quiesce");

    /* 3. Forced while STATUS says idle: a co-trigger-staged deep replay has the reader
          streaming (mode high) with DAC_RUN 0, so the stop must still go out. */
    fab_reset(); f.mode = true;
    CHECK(dac_rearm_quiesce(&ops, true, false) == 0, "forced idle: rc 0");
    CHECK(strcmp(f.log, "PS") == 0, "forced idle: expected PS, got \"%s\"", f.log);
    CHECK(!f.mode, "forced idle: mode level must drop");

    /* 4. A staged co-trigger is staged again AFTER the stop and after DAC_RUN clears. */
    fab_reset(); f.running = true; f.stop_lag = 1; f.cotrig = true;
    CHECK(dac_rearm_quiesce(&ops, true, true) == 0, "cotrig: rc 0");
    CHECK(strcmp(f.log, "PSSC") == 0, "cotrig: expected PSSC, got \"%s\"", f.log);
    CHECK(f.cotrig, "cotrig: co-trigger must still be staged");

    /* 5. DAC_RUN stuck: bounded poll, error, co-trigger still restored. */
    fab_reset(); f.running = true; f.stuck = true;
    CHECK(dac_rearm_quiesce(&ops, false, true) == -1, "stuck: rc -1");
    CHECK(strlen(f.log) == 2u + DAC_REARM_MAX_POLLS + 1u && f.log[1] == 'P'
          && f.log[strlen(f.log) - 1] == 'C',
          "stuck: expected S P %u*S C, got \"%s\"", DAC_REARM_MAX_POLLS, f.log);

    /* 6. Unreadable STATUS: not forced = treat as idle (the old fpga_load_wave rule);
          forced = stop anyway, and report that idle could not be confirmed. */
    fab_reset(); f.status_bad = true;
    CHECK(dac_rearm_quiesce(&ops, false, false) == 0 && strcmp(f.log, "S") == 0,
          "bad status, not forced: no stop, got \"%s\"", f.log);
    fab_reset(); f.status_bad = true;
    CHECK(dac_rearm_quiesce(&ops, true, false) == -1 && f.log[0] == 'P',
          "bad status, forced: stop then -1, got \"%s\"", f.log);

    /* 7. The bug: a second deep replay on top of a running one.  Every start must see the
          mode level low first, so the reader reloads base/len and flushes its FIFO. */
    fab_reset(); f.stop_lag = 1;
    CHECK(replay_start(false), "first replay: clean start");
    CHECK(replay_start(false), "re-arm while running: reader must see a clean start");
    CHECK(strcmp(f.log, "PSRPSSR") == 0, "re-arm: expected PSRPSSR, got \"%s\"", f.log);

    /* 8. Re-arm a co-triggered replay while the previous co-triggered one is still staged. */
    fab_reset();
    f.cotrig = true; CHECK(replay_start(true), "staged replay: clean start");
    f.cotrig = true; CHECK(replay_start(true), "re-staged replay: clean start");
    CHECK(f.cotrig && !f.running, "re-staged replay must still wait for the capture's t0");
    CHECK(strcmp(f.log, "PSCRPSCR") == 0, "re-staged: expected PSCRPSCR, got \"%s\"", f.log);

    if (fails == 0) printf("PASS test_dac_rearm\n");
    else            printf("FAIL test_dac_rearm: %d failure(s)\n", fails);
    return fails ? 1 : 0;
}
