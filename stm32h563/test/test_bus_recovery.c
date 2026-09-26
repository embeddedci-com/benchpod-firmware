/* Host test for the bus-recovery logic: the I2C bus clear (i2c_bus.h) against a
   simulated slave, the TRNG retry policy (rng.h) and FDCAN bus-off recovery
   register logic (can_bus.h). */
#include "i2c_bus.h"
#include "rng.h"
#include "can_bus.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

/* ---- simulated I2C bus + one slave --------------------------------------- */

typedef enum { SL_IDLE, SL_READ, SL_WRITE } slave_mode_t;

typedef struct {
    /* master side: true = released */
    bool m_scl, m_sda;
    /* slave */
    slave_mode_t mode;
    int  bit;              /* READ: 0..7 data bit being driven, 8 = ACK slot */
    uint8_t data;          /* READ: byte it sends, repeated for every byte */
    bool s_sda_low;        /* slave drives SDA low */
    int  scl_hold;         /* slave holds SCL low for this many more delays (-1 = forever) */
    bool sda_stuck;        /* SDA shorted low (not a slave, a fault) */
    /* observed */
    bool scl, sda;         /* line levels */
    int  rises;            /* SCL rising edges */
    int  stops, starts;
    int  delays;
} bus_t;

static bool line_scl(const bus_t *b) { return b->m_scl && b->scl_hold == 0; }
static bool line_sda(const bus_t *b) { return b->m_sda && !b->s_sda_low && !b->sda_stuck; }

static void slave_drive(bus_t *b)
{
    if (b->mode == SL_READ) {
        b->s_sda_low = (b->bit < 8) ? !((b->data >> (7 - b->bit)) & 1u) : false;
    } else {
        b->s_sda_low = false;
    }
}

/* Recompute the lines and let the slave react to edges. */
static void bus_update(bus_t *b)
{
    bool scl0 = b->scl, sda0 = b->sda;
    bool scl1 = line_scl(b);

    if (scl0 && !scl1) {                        /* SCL falling: slave shifts */
        if (b->mode == SL_READ) {
            b->bit++;
            if (b->bit > 8) b->bit = 0;
        } else if (b->mode == SL_WRITE) {
            /* was ACKing a byte; releases SDA and waits for data bits */
            b->mode = SL_IDLE;                   /* for this model: then listens */
        }
        slave_drive(b);
    }
    bool sda1 = line_sda(b);
    if (!scl0 && scl1) {                        /* SCL rising: slave samples */
        b->rises++;
        if (b->mode == SL_READ && b->bit == 8 && sda1) {  /* master NACK */
            b->mode = SL_IDLE;
            slave_drive(b);
            sda1 = line_sda(b);
        }
    }
    if (scl0 && scl1 && sda0 != sda1) {         /* SDA edge while SCL high */
        if (sda1) {
            b->stops++;
            b->mode = SL_IDLE;
            slave_drive(b);
            sda1 = line_sda(b);
        } else {
            b->starts++;
        }
    }
    b->scl = scl1;
    b->sda = sda1;
}

static void io_set_scl(void *c, bool h) { bus_t *b = c; b->m_scl = h; bus_update(b); }
static void io_set_sda(void *c, bool h) { bus_t *b = c; b->m_sda = h; bus_update(b); }
static bool io_get_scl(void *c) { return ((bus_t *)c)->scl; }
static bool io_get_sda(void *c) { return ((bus_t *)c)->sda; }
static void io_delay(void *c)
{
    bus_t *b = c;
    b->delays++;
    if (b->scl_hold > 0) b->scl_hold--;
    bus_update(b);
}

static void bus_init(bus_t *b)
{
    memset(b, 0, sizeof(*b));
    b->m_scl = b->m_sda = true;
    slave_drive(b);
    b->scl = line_scl(b);
    b->sda = line_sda(b);
}

static int run(bus_t *b)
{
    i2c_busclear_io_t io = { io_set_scl, io_set_sda, io_get_scl, io_get_sda, io_delay, b };
    return i2c_busclear_run(&io);
}

static void test_i2c_busclear(void)
{
    bus_t b;

    /* Idle bus: no clocks needed, but still a STOP (and nothing else). */
    bus_init(&b);
    int rc = run(&b);
    CHECK(rc == 1);
    CHECK(b.stops == 1 && b.starts == 0);
    CHECK(b.scl && b.sda);

    /* Reset in the middle of a read: every data byte, every bit position the
       MCU could have died at.  The slave must end idle with the bus released,
       via a STOP, without the recovery ever making a START. */
    static const uint8_t bytes[] = { 0x00, 0xFF, 0x55, 0xAA, 0x7F, 0x80, 0x01, 0xFE };
    int worst = 0;
    for (unsigned i = 0; i < sizeof(bytes); i++) {
        for (int bit = 0; bit <= 8; bit++) {
            bus_init(&b);
            b.mode = SL_READ;
            b.data = bytes[i];
            b.bit = bit;
            slave_drive(&b);
            b.sda = line_sda(&b);
            rc = run(&b);
            if (rc < 0 || b.mode != SL_IDLE || b.starts != 0 || !b.scl || !b.sda || b.stops < 1)
                printf("  read slave data=0x%02x bit=%d: rc=%d mode=%d starts=%d stops=%d\n",
                       bytes[i], bit, rc, b.mode, b.starts, b.stops);
            CHECK(rc >= 0);
            CHECK(rc == b.rises);
            CHECK(b.mode == SL_IDLE);
            CHECK(b.starts == 0);
            CHECK(b.stops >= 1);
            CHECK(b.scl && b.sda);
            CHECK(rc <= I2C_BUSCLEAR_MAX_PULSES + 1);
            if (rc > worst) worst = rc;
        }
    }
    printf("  read-slave recovery: worst case %d SCL pulses\n", worst);
    CHECK(worst <= 18);

    /* Reset while the slave ACKs a written byte (SDA low for one clock). */
    bus_init(&b);
    b.mode = SL_WRITE;
    b.s_sda_low = true;
    b.sda = line_sda(&b);
    rc = run(&b);
    CHECK(rc == 2);          /* one clock ends the ACK, one for the STOP */
    CHECK(b.stops == 1 && b.starts == 0 && b.scl && b.sda);

    /* The slave stretches SCL for a few half periods: still fine. */
    bus_init(&b);
    b.scl_hold = 3;
    b.scl = line_scl(&b);
    rc = run(&b);
    CHECK(rc == 1);
    CHECK(b.stops == 1);

    /* SCL held low for good: give up, bounded. */
    bus_init(&b);
    b.scl_hold = -1;
    b.scl = line_scl(&b);
    rc = run(&b);
    CHECK(rc == I2C_BUSCLEAR_SCL_STUCK);
    CHECK(b.delays <= I2C_BUSCLEAR_SCL_WAIT);

    /* SDA shorted low: give up after the pulse budget, lines released. */
    bus_init(&b);
    b.sda_stuck = true;
    b.sda = line_sda(&b);
    rc = run(&b);
    CHECK(rc == I2C_BUSCLEAR_SDA_STUCK);
    CHECK(b.rises == I2C_BUSCLEAR_MAX_PULSES);
    CHECK(b.m_scl && b.m_sda);
    CHECK(b.starts == 0);
}

/* ---- TRNG retry policy ---------------------------------------------------- */

typedef struct {
    int fail_first;    /* this many reads fail before any succeed */
    int fail_every;    /* if >0, the reads from index fail_from fail for good */
    int fail_from;
    int reads, resets;
    uint32_t next;
} rng_mock_t;

static int mock_word(void *c, uint32_t *out)
{
    rng_mock_t *m = c;
    int i = m->reads++;
    if (i < m->fail_first) return -1;
    if (m->fail_every && i >= m->fail_from) return -1;
    *out = m->next;
    m->next += 0x01010101u;
    return 0;
}
static void mock_reset(void *c) { ((rng_mock_t *)c)->resets++; }

static void test_rng_policy(void)
{
    uint8_t buf[32];
    uint32_t errors;

    /* Healthy source: filled, no resets. */
    rng_mock_t m = { .next = 0x11223344u };
    errors = 0;
    memset(buf, 0xEE, sizeof(buf));
    CHECK(rng_fill_policy(buf, sizeof(buf), mock_word, mock_reset, &m, 4, &errors) == 0);
    CHECK(m.reads == 8 && m.resets == 0 && errors == 0);
    CHECK(buf[0] == 0x44 && buf[3] == 0x11);

    /* Odd length: the tail uses part of one word. */
    m = (rng_mock_t){ .next = 0xA1B2C3D4u };
    CHECK(rng_fill_policy(buf, 5, mock_word, mock_reset, &m, 4, NULL) == 0);
    CHECK(m.reads == 2);

    /* Transient seed error: reset + retry, then good data. */
    m = (rng_mock_t){ .fail_first = 2, .next = 0x55555555u };
    errors = 0;
    CHECK(rng_fill_policy(buf, sizeof(buf), mock_word, mock_reset, &m, 4, &errors) == 0);
    CHECK(m.resets == 2 && errors == 2);
    CHECK(buf[0] == 0x55);

    /* Dead source: fails, and the buffer is wiped (never half-filled). */
    m = (rng_mock_t){ .fail_every = 1, .fail_from = 3, .next = 0x12345678u };
    errors = 0;
    memset(buf, 0xEE, sizeof(buf));
    CHECK(rng_fill_policy(buf, sizeof(buf), mock_word, mock_reset, &m, 4, &errors) == -1);
    CHECK(errors == 4 && m.resets == 4);
    int nonzero = 0;
    for (unsigned i = 0; i < sizeof(buf); i++) nonzero |= buf[i];
    CHECK(nonzero == 0);

    /* Dead from the first read. */
    m = (rng_mock_t){ .fail_first = 1000 };
    CHECK(rng_fill_policy(buf, 4, mock_word, mock_reset, &m, RNG_WORD_ATTEMPTS, NULL) == -1);
    CHECK(m.reads == RNG_WORD_ATTEMPTS);
}

/* ---- FDCAN bus-off ------------------------------------------------------- */

static void test_can_busoff(void)
{
    volatile uint32_t cccr;

    /* Healthy: nothing to do. */
    cccr = 0;
    CHECK(!can_busoff_recover_regs(&cccr, 0));
    CHECK(cccr == 0);

    /* Stopped by software (not bus-off): INIT set, BO clear -> leave alone. */
    cccr = CAN_REG_CCCR_INIT | 0x2u;
    CHECK(!can_busoff_recover_regs(&cccr, 0));
    CHECK(cccr == (CAN_REG_CCCR_INIT | 0x2u));

    /* Bus-off: the core set INIT; clearing it starts recovery, other bits kept. */
    cccr = CAN_REG_CCCR_INIT | 0x100u;
    CHECK(can_busoff_recover_regs(&cccr, CAN_REG_PSR_BO | 0x7u));
    CHECK(cccr == 0x100u);

    /* Recovery under way (INIT clear, BO still set until 129x11 bits):
       a second call must not count again. */
    CHECK(!can_busoff_recover_regs(&cccr, CAN_REG_PSR_BO));
    CHECK(cccr == 0x100u);

    /* Recovered, then bus-off again later: counts again. */
    cccr = CAN_REG_CCCR_INIT;
    CHECK(can_busoff_recover_regs(&cccr, CAN_REG_PSR_BO));
    CHECK(cccr == 0);
}

int main(void)
{
    test_i2c_busclear();
    test_rng_policy();
    test_can_busoff();
    if (failures) { printf("test_bus_recovery: %d FAILURE(S)\n", failures); return 1; }
    printf("test_bus_recovery: all passed\n");
    return 0;
}
