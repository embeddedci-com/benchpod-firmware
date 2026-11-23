/* prog_job.c — the prog_flash background job (prog_flash.h): pins, the staged image, and the
 * protocol stepped one bounded call per hw worker loop. Runs on the hw worker only. */
#include "prog_flash.h"
#include "bp_errors.h"
#include "command_handler_internal.h"   /* command_handler_psram_stage */
#include "signal_engine.h"              /* signal_engine_quiesce_psram_masters */
#include "la_pins.h"
#include "nrst_ctrl.h"
#include "psram.h"
#include "bp_log.h"
#include "dap.h"
#include "stm32h5xx_hal.h"

#include <stdio.h>
#include <string.h>

typedef enum { J_IDLE, J_ENTER, J_ERASE, J_WRITE, J_LEAVE } jst_t;

/* The working state, only while a job runs: in the DAP's idle scratch (dap_scratch), so the job
   costs almost no RAM of its own. What status reports after the job is copied to `r`. */
typedef struct {
    prog_params_t   p;
    prog_wire_ops_t ops;
    prog_ctx_t      ctx;
    uint32_t        base;       /* PSRAM address of the staged upload */
    uint32_t        t0;
    uint32_t        wire[PROG_WIRE_MEM / 4u];
} work_t;

static jst_t   s_st;
static work_t *wk;
static struct {
    const prog_proto_t *proto;
    const char *phase;
    uint32_t    done, total, at_addr, ms;
    int         rc;
    uint8_t     op, id[8], id_len;
    bool        have_result;
} r;

#define j      (*wk)
#define s_ctx  (wk->ctx)

static int src_read(void *job, uint32_t off, uint8_t *buf, uint32_t n)
{
    (void)job;
    if (off + n > j.p.len) return -1;
    psram_bus_acquire();
    int rc = psram_read(j.base + off, buf, n);
    psram_bus_release();
    return rc < 0 ? -1 : 0;
}

bool prog_flash_busy(void) { return s_st != J_IDLE; }

static void release_pins(void)
{
    prog_wire_close(&j.ops);
    if (!j.p.proto->uart) {
        la_pins_release_fn(LA_FN_SWD_CLK);
        la_pins_release_fn(LA_FN_SWD_DIO);
    }
    if (j.p.proto->uses_nrst) nrst_ctrl_assert(false);
}

int prog_flash_start(const prog_params_t *p, char *err, size_t cap)
{
    if (s_st != J_IDLE) { bp_note_pod_busy(err, cap); return -1; }
    const prog_proto_t *pr = p->proto;
    /* A DAP, SPI or I2C session holds the engine (and the DAP its scratch, which the job borrows). */
    if (fpga_swd_armed() || la_pins_mask_fn(LA_FN_SWD_CLK) || la_pins_mask_fn(LA_FN_JTAG_TCK) ||
        la_pins_mask_fn(LA_FN_SPI_SCK) || la_pins_mask_fn(LA_FN_I2CM_SCL)) {
        bp_note_bus_engine_busy(err, cap, "prog_flash"); return -1;
    }
    size_t scratch_len = 0;
    void *scratch = dap_scratch(&scratch_len);
    _Static_assert(sizeof(work_t) <= 1024u, "work_t must fit the DAP scratch");
    if (scratch_len < sizeof(work_t)) { snprintf(err, cap, "no room for the job"); return -1; }
    if (!(pr->mems & (1u << p->mem))) { snprintf(err, cap, "%s cannot write that memory", pr->name); return -1; }
    if (pr->uses_nrst && !nrst_ctrl_supported()) {
        snprintf(err, cap, "this pod has no reset pin, which %s needs", pr->name); return -1;
    }
    uint32_t base = 0, staged = 0;
    if (p->op == PROG_OP_START) {
        if (p->len == 0) { snprintf(err, cap, "len must be more than 0"); return -1; }
        if (!command_handler_psram_stage(&base, &staged)) {
            snprintf(err, cap, "nothing staged: load_bin with \"psram\":true first"); return -1;
        }
        if (p->len > staged) { snprintf(err, cap, "len is more than the staged upload"); return -1; }
    }

    wk = scratch;
    memset(wk, 0, sizeof(*wk));
    j.p = *p;
    if (pr->uart) {
        if (prog_wire_open_uart(&j.ops, j.wire, p->data, err, cap) != 0) {
            if (!err[0]) bp_note_uart_session_open(err, cap);
            return -1;
        }
    } else {
        if (p->clk == p->data) { snprintf(err, cap, "clk and data must be different pins"); return -1; }
        uint8_t c = p->clk, d = p->data;
        if (!la_pins_check_claim(LA_FN_SWD_CLK, LA_GPIO_NONE, &c, 1, 0, la_pull_mask_now(), err, cap) ||
            !la_pins_check_claim(LA_FN_SWD_DIO, LA_GPIO_NONE, &d, 1, 0, la_pull_mask_now(), err, cap))
            return -1;
        la_pins_claim(LA_FN_SWD_CLK, LA_GPIO_NONE, 0, &c, 1);
        la_pins_claim(LA_FN_SWD_DIO, LA_GPIO_NONE, 0, &d, 1);
        int rc = prog_wire_open_2w(&j.ops, j.wire, p->clk, p->data);
        if (rc != 0) {
            la_pins_release_fn(LA_FN_SWD_CLK);
            la_pins_release_fn(LA_FN_SWD_DIO);
            if (rc == -2) bp_note_bus_engine_busy(err, cap, "prog_flash");
            else          snprintf(err, cap, "invalid clk or data pin");
            return -1;
        }
    }
    if (p->op == PROG_OP_START) signal_engine_quiesce_psram_masters();

    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.w = &j.ops;
    s_ctx.mem = p->mem; s_ctx.erase = p->erase;
    s_ctx.addr = p->addr; s_ctx.len = p->len; s_ctx.page = p->page;
    s_ctx.baud = p->baud; s_ctx.flags = p->flags;
    s_ctx.src = src_read; s_ctx.job = &j;
    s_ctx.phase = "enter";
    j.base = base;
    j.t0 = HAL_GetTick();
    memset(&r, 0, sizeof(r));
    r.proto = pr; r.op = p->op;
    s_st = J_ENTER;
    log_printf("[prog] %s %s: clk LA%u data LA%u, %lu bytes at 0x%06lx\n", pr->name,
               p->op == PROG_OP_START ? "start" : "info", p->clk, p->data,
               (unsigned long)p->len, (unsigned long)p->addr);
    return 0;
}

static void finish(int rc)
{
    r.rc = rc;
    r.have_result = true;
    r.ms = HAL_GetTick() - j.t0;
    r.phase = s_ctx.phase;
    r.done = s_ctx.done; r.total = s_ctx.total; r.at_addr = s_ctx.at_addr;
    r.id_len = s_ctx.id_len > sizeof(r.id) ? (uint8_t)sizeof(r.id) : s_ctx.id_len;
    memcpy(r.id, s_ctx.id, r.id_len);
    log_printf("[prog] %s: %s (%lu ms)\n", j.p.proto->name, prog_rc_str(rc), (unsigned long)r.ms);
    release_pins();
    s_st = J_IDLE;
    wk = NULL;                       /* the scratch goes back to the DAP */
}

/* After a failure the protocol still gets its leave() (pins and chip back to normal); status
   reports the phase it failed in. */
static void fail(int rc)
{
    const char *at = s_ctx.phase;
    j.p.proto->leave(&s_ctx, false);
    s_ctx.phase = at;
    finish(rc);
}

void prog_flash_poll(void)
{
    if (s_st == J_IDLE) return;
    const prog_proto_t *pr = j.p.proto;
    int rc;
    switch (s_st) {
    case J_IDLE:
        return;
    case J_ENTER:
        rc = pr->enter(&s_ctx);
        if (rc == PROG_RUNNING) return;
        if (rc < 0) { fail(rc); return; }
        if (j.p.expect_len && (s_ctx.id_len < j.p.expect_len ||
                               memcmp(s_ctx.id, j.p.expect_id, j.p.expect_len) != 0)) {
            fail(PROG_E_WRONG_CHIP); return;
        }
        if (j.p.op == PROG_OP_INFO) { s_st = J_LEAVE; return; }
        s_ctx.done = s_ctx.total = 0;
        s_st = (j.p.erase != PROG_ERASE_NONE) ? J_ERASE : J_WRITE;
        return;
    case J_ERASE:
        s_ctx.phase = "erase";
        rc = pr->erase(&s_ctx);
        if (rc == PROG_RUNNING) return;
        if (rc < 0) { fail(rc); return; }
        s_ctx.done = s_ctx.total = 0;
        s_st = J_WRITE;
        return;
    case J_WRITE:
        rc = pr->write(&s_ctx);
        if (rc == PROG_RUNNING) return;
        if (rc < 0) { fail(rc); return; }
        s_st = J_LEAVE;
        return;
    case J_LEAVE:
        s_ctx.phase = "leave";
        pr->leave(&s_ctx, j.p.run);
        s_ctx.phase = "done";
        finish(PROG_OK);
        return;
    }
}

void prog_flash_abort(void)
{
    if (s_st == J_IDLE) return;
    log_printf("[prog] aborted\n");
    fail(PROG_E_ABORTED);
}

const char *prog_rc_str(int rc)
{
    switch (rc) {
    case PROG_OK:            return "ok";
    case PROG_E_NO_ANSWER:   return "no answer (wiring, power or I/O voltage)";
    case PROG_E_WRONG_CHIP:  return "wrong chip (the ID is not expect_id)";
    case PROG_E_LOCKED:      return "locked (erase the chip to unlock it)";
    case PROG_E_VERIFY:      return "verify failed";
    case PROG_E_TIMEOUT:     return "timeout (the chip stayed busy)";
    case PROG_E_PROTOCOL:    return "protocol error (an answer the protocol does not allow)";
    case PROG_E_ARGS:        return "memory, address, length or page size not valid here";
    case PROG_E_SRC:         return "could not read the staged image";
    case PROG_E_ABORTED:     return "aborted";
    case PROG_E_UNSUPPORTED: return "not supported (this chip, memory or protocol)";
    default:                 return "error";
    }
}

void prog_flash_status(prog_status_t *st)
{
    memset(st, 0, sizeof(*st));
    st->running = s_st != J_IDLE;
    st->have_result = r.have_result;
    if (!r.proto) return;
    st->op = r.op == PROG_OP_START ? "start" : "info";
    st->proto = r.proto->name;
    if (st->running) {
        st->phase = s_ctx.phase;
        st->done = s_ctx.done; st->total = s_ctx.total;
        st->id_len = s_ctx.id_len > sizeof(st->id) ? (uint8_t)sizeof(st->id) : s_ctx.id_len;
        memcpy(st->id, s_ctx.id, st->id_len);
        st->ms = HAL_GetTick() - j.t0;
        return;
    }
    st->phase = r.phase;
    st->done = r.done; st->total = r.total; st->at_addr = r.at_addr;
    st->id_len = r.id_len;
    memcpy(st->id, r.id, r.id_len);
    st->rc = r.rc;
    st->result = prog_rc_str(r.rc);
    st->ms = r.ms;
}
