/*
 * test_target_power.c — host unit test for the eFuse glitch-free bring-up.
 *
 * A register-file mock of the TCA9554 (behind the i2c_bus API) lets us pin down
 * the property that matters for the power-on 5V glitch: target_power_init() must
 * drive BOTH eFuse EN latches LOW (write the OUTPUT register to a safe value)
 * BEFORE it switches the EN pins to outputs (CONFIG) — and if that safe write
 * fails it must NOT enable the outputs, or it would drive the TCA9554's 0xFF
 * power-on latch default HIGH = eFuse ON.  Also checks enable/disable set the
 * right EN bit without disturbing the other eFuse.
 */
#include "target_power.h"
#include "efuse_flt.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if(!(c)){printf("FAIL %s:%d  %s\n",__FILE__,__LINE__,#c);failures++;} } while(0)

/* ---- TCA9554 register-file mock (only addr 0x22 is used) ----------------- */
#define REG_INPUT   0x00u
#define REG_OUTPUT  0x01u
#define REG_CONFIG  0x03u

static uint8_t regs[4];
static struct { uint8_t reg, val; } wlog[32];
static int     wlog_n;
static int     fail_write_from;   /* -1 = never; else fail the Nth write onward (1-based) */
static int     write_count;

static void mock_reset(void)
{
    /* TCA9554 power-on defaults: all inputs, output latch 0xFF. */
    regs[REG_INPUT] = 0x00; regs[REG_OUTPUT] = 0xFF; regs[0x02] = 0x00; regs[REG_CONFIG] = 0xFF;
    wlog_n = 0; write_count = 0; fail_write_from = -1;
}

int i2c_bus_write(uint8_t addr, const uint8_t *buf, size_t len)
{
    (void)addr;
    if (len == 2) {
        write_count++;
        if (fail_write_from >= 0 && write_count >= fail_write_from) return -1;
        regs[buf[0] & 3u] = buf[1];
        if (wlog_n < 32) { wlog[wlog_n].reg = buf[0]; wlog[wlog_n].val = buf[1]; wlog_n++; }
    }
    return 0;
}

int i2c_bus_write_read(uint8_t addr, const uint8_t *wbuf, size_t wlen,
                       uint8_t *rbuf, size_t rlen)
{
    (void)addr;
    if (wlen == 1 && rlen == 1) rbuf[0] = regs[wbuf[0] & 3u];
    return 0;
}

/* ---- stubs for the rest of target_power.c's externals ------------------- */
void     fpga_set_led(uint8_t mask) { (void)mask; }
static uint64_t s_now_us;
/* the output's current monitor, for efuse_flt.c */
static int s_out_mv = 5100, s_out_ua = 0, s_ina_rc = 0;
int ina238_read(uint8_t addr, int *bus_mv, int *shunt_uv, int *current_ua)
{
    (void)addr; *bus_mv = s_out_mv; *shunt_uv = 0; *current_ua = s_out_ua; return s_ina_rc;
}
uint64_t time_us_64(void) { return s_now_us; }

/* ---- tests -------------------------------------------------------------- */


/* FLT (active-low on P5 for eFuse 1) counts only after FLT_CONFIRM_SAMPLES 10 ms samples in a
   row: a single-sample glitch is no trip, a held FLT is. */
static int s_events;
static void on_event(int efuse, bool enabled, bool fault) { (void)enabled; if (efuse == 1 && fault) s_events++; }
static void poll_at(uint64_t ms, bool flt1)
{
    regs[REG_INPUT] = (uint8_t)(0xFFu & ~(flt1 ? (1u << 5) : 0u));   /* FLT high = clear */
    s_now_us = ms * 1000u;
    target_power_poll();
}
static void test_flt_debounce(void)
{
    mock_reset();
    target_power_init();
    target_power_set_event_cb(on_event);
    s_events = 0;
    bool en = false, flt = false;
    uint64_t t = 1000;
    poll_at(t += 10, false);
    poll_at(t += 10, true);                      /* one glitch */
    (void)target_power_get_cached(1, &en, &flt);
    CHECK(!flt);
    poll_at(t += 10, false);
    poll_at(t += 10, true);
    poll_at(t += 10, true);                      /* two in a row: still a glitch */
    poll_at(t += 10, false);
    (void)target_power_get_cached(1, &en, &flt);
    CHECK(!flt && s_events == 0);
    poll_at(t += 10, true);
    poll_at(t += 10, true);
    poll_at(t += 10, true);                      /* held: a trip */
    (void)target_power_get_cached(1, &en, &flt);
    CHECK(flt && s_events == 1);
    poll_at(t += 10, true);                      /* still held: one event, not one per sample */
    CHECK(s_events == 1);
    poll_at(t += 10, false);
    (void)target_power_get_cached(1, &en, &flt);
    CHECK(!flt);
    target_power_set_event_cb(NULL);
}


/* Core invariant: OUTPUT latch driven safe (0x00) BEFORE CONFIG enables outputs. */
static void test_init_safe_ordering(void)
{
    mock_reset();
    target_power_init();
    CHECK(wlog_n == 2);                                    /* exactly OUTPUT then CONFIG */
    CHECK(wlog[0].reg == REG_OUTPUT && wlog[0].val == 0x00);
    CHECK(wlog[1].reg == REG_CONFIG && wlog[1].val == 0x7E); /* P7,P0 output; rest input */
    CHECK((regs[REG_OUTPUT] & 0x81u) == 0x00u);           /* both EN latches low */
    CHECK((regs[REG_CONFIG] & 0x81u) == 0x00u);           /* both EN pins are outputs */
}

/* If the safe OUTPUT write fails, CONFIG must NOT be written — the EN pins stay
   inputs (high-Z, POR) rather than being driven off a 0xFF latch = eFuse ON. */
static void test_init_aborts_if_output_write_fails(void)
{
    mock_reset();
    fail_write_from = 1;                 /* fail the very first write (the OUTPUT preset) */
    target_power_init();
    for (int i = 0; i < wlog_n; i++) CHECK(wlog[i].reg != REG_CONFIG);
    CHECK((regs[REG_CONFIG] & 0x81u) == 0x81u);   /* P7,P0 still inputs (POR 0xFF) */
}

/* enable/disable toggle the correct EN bit and don't disturb the other eFuse. */
static void test_enable_disable(void)
{
    mock_reset();
    target_power_init();

    CHECK(target_power_enable(1, true)  == 0);
    CHECK((regs[REG_OUTPUT] & 0x80u) == 0x80u);   /* eF1 EN=P7 high */

    CHECK(target_power_enable(2, true)  == 0);
    CHECK((regs[REG_OUTPUT] & 0x01u) == 0x01u);   /* eF2 EN=P0 high */
    CHECK((regs[REG_OUTPUT] & 0x80u) == 0x80u);   /* eF1 still on */

    CHECK(target_power_enable(1, false) == 0);
    CHECK((regs[REG_OUTPUT] & 0x80u) == 0x00u);   /* eF1 off */
    CHECK((regs[REG_OUTPUT] & 0x01u) == 0x01u);   /* eF2 unaffected */

    CHECK(target_power_enable(3, true)  != 0);    /* invalid eFuse index rejected */
}

/* The rule: output collapsed or carrying current = a fault; up with no current = reverse blocking. */
static void test_flt_rule(void)
{
    CHECK(!efuse_flt_is_fault(1, 5100, 0));        /* reverse-current blocking, no load */
    CHECK(!efuse_flt_is_fault(1, 5050, -2000));    /* reverse current, briefly */
    CHECK(efuse_flt_is_fault(1, 4200, 2000000));   /* in current limit at 2 A */
    CHECK(efuse_flt_is_fault(2, 300, 0));          /* short or shutdown: output down */
}

/* With the check registered, a held FLT with the output up and no current is no trip (counted as
   reverse blocking); when the output then collapses while FLT stays held, it is. */
static void test_flt_reverse_blocking(void)
{
    mock_reset();
    target_power_init();
    efuse_flt_init();
    target_power_set_event_cb(on_event);
    s_events = 0; s_out_mv = 5100; s_out_ua = 0; s_ina_rc = 0;
    bool en = false, flt = false;
    uint64_t t = 100000;   /* after test_flt_debounce: the poll paces itself on the clock */
    for (int i = 0; i < 8; i++) poll_at(t += 10, true);
    (void)target_power_get_cached(1, &en, &flt);
    CHECK(!flt && s_events == 0 && target_power_reverse_blocks(1) == 1);
    s_out_mv = 200;                                 /* the output collapses: a real fault */
    for (int i = 0; i < 10; i++) poll_at(t += 10, true);
    (void)target_power_get_cached(1, &en, &flt);
    CHECK(flt && s_events == 1);
    poll_at(t += 10, false);
    s_out_mv = 5100;
    s_ina_rc = -1;                                  /* the monitor does not answer: a fault */
    for (int i = 0; i < 3; i++) poll_at(t += 10, true);
    (void)target_power_get_cached(1, &en, &flt);
    CHECK(flt && s_events == 2);
    s_ina_rc = 0;
    target_power_set_flt_check(NULL);
    target_power_set_event_cb(NULL);
}

int main(void)
{
    test_flt_debounce();
    test_flt_rule();
    test_flt_reverse_blocking();
    test_init_safe_ordering();
    test_init_aborts_if_output_write_fails();
    test_enable_disable();

    if (failures) { printf("FAIL — %d target_power checks failed\n", failures); return 1; }
    printf("PASS — all target_power tests\n");
    return 0;
}
