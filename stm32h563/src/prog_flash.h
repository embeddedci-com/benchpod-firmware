#ifndef PROG_FLASH_H
#define PROG_FLASH_H

/* prog_flash — flash a chip over its own programming pins: AVR UPDI, PIC ICSP (low-voltage entry),
 * SiLabs C2, AVR TPI, XMEGA PDI. Plan: embeddedci-server private-docs/pin-programmers/plan.md.
 *
 * The host stages the image in PSRAM (load_bin with "psram"); `prog_flash` (command_handler_prog.c)
 * starts a background job on the hw worker that runs one protocol (prog_<proto>.c) over prog_wire.
 * Each protocol is pure logic over prog_wire_ops_t and the job's memory callbacks, so host tests
 * run it against a model of the chip.
 *
 * The pod knows no device tables: it reads and reports the chip's ID; the host passes the
 * geometry (page size, memory base) and expect_id.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "prog_wire.h"

typedef enum {
    PROG_MEM_FLASH = 0,
    PROG_MEM_EEPROM,
    PROG_MEM_CONFIG,   /* PIC config words, AVR fuses, C2 lock byte */
    PROG_MEM_USER,     /* AVR user row, PIC user ID */
    PROG_MEM_COUNT
} prog_mem_t;

typedef enum {
    PROG_ERASE_NONE = 0,
    PROG_ERASE_CHIP,   /* whole chip before writing (also unlocks a locked chip) */
    PROG_ERASE_PAGE,   /* only the pages written */
} prog_erase_t;

typedef enum {
    PROG_OK = 0,
    PROG_RUNNING = 1,
    PROG_E_NO_ANSWER = -1,  /* nothing answered: wiring, power, I/O voltage */
    PROG_E_WRONG_CHIP = -2, /* the ID is not expect_id */
    PROG_E_LOCKED = -3,     /* read/write refused by the chip's lock: erase to unlock */
    PROG_E_VERIFY = -4,     /* read back differs (status: at_addr) */
    PROG_E_TIMEOUT = -5,    /* the chip stayed busy (NVM, erase) */
    PROG_E_PROTOCOL = -6,   /* an answer the protocol does not allow (parity, framing, bad ack) */
    PROG_E_ARGS = -7,       /* mem/addr/len/page not valid for this protocol */
    PROG_E_SRC = -8,        /* reading the staged image / writing the read-back area failed */
    PROG_E_ABORTED = -9,
    PROG_E_UNSUPPORTED = -10, /* this chip or memory is not handled (yet) */
} prog_rc_t;

/* What the job asks of a protocol. The job owns phases and progress; a protocol call does a bounded
   amount of work (one page, one erase poll) and returns PROG_RUNNING to be called again, PROG_OK when
   finished, or a prog_rc_t error. */
typedef struct {
    /* in */
    const prog_wire_ops_t *w;
    uint8_t   mem;            /* prog_mem_t */
    uint8_t   erase;          /* prog_erase_t */
    uint32_t  addr;           /* in the memory's own address space (bytes; PIC: bytes = 2 * word address) */
    uint32_t  len;            /* bytes */
    uint16_t  page;           /* the chip's write page/row in bytes (from the host's table) */
    uint32_t  baud;           /* UPDI: link rate after entry (0 = the protocol's default) */
    uint32_t  flags;          /* protocol-specific (prog_<proto>.h), e.g. PIC command set */
    /* image access: the job's buffers over the PSRAM staging area */
    int (*src)(void *job, uint32_t off, uint8_t *buf, uint32_t n);   /* staged bytes */
    int (*dst)(void *job, uint32_t off, const uint8_t *buf, uint32_t n); /* read-back area */
    void *job;
    /* out */
    uint8_t   id[8];          /* the chip's ID bytes as read (protocol order) */
    uint8_t   id_len;
    uint32_t  done, total;    /* progress in pages (or blocks) */
    uint32_t  at_addr;        /* PROG_E_VERIFY: first differing address */
    const char *phase;        /* "enter", "id", "erase", "write", "verify", "read", "leave" */
    /* protocol state: each protocol casts this to its own struct; keep it <= PROG_STATE_BYTES */
    uint8_t   state[192] __attribute__((aligned(4)));
} prog_ctx_t;
#define PROG_STATE_BYTES 192u

typedef struct {
    const char *name;        /* "updi", "pic", "c2", "tpi", "pdi" */
    bool        uart;        /* true: one pin over the UART engine (UPDI); false: clk + data */
    bool        uses_nrst;   /* true: the pod's reset pin is part of the wiring (PIC MCLR, TPI RESET) */
    uint8_t     mems;        /* bit mask of prog_mem_t it supports */
    /* Enter programming mode and read the ID into ctx->id. */
    int (*enter)(prog_ctx_t *ctx);
    int (*erase)(prog_ctx_t *ctx);        /* the erase ctx->erase asks for */
    int (*write)(prog_ctx_t *ctx);        /* next page of [addr, addr+len) from src, verified */
    int (*read)(prog_ctx_t *ctx);         /* next block of [addr, addr+len) to dst */
    /* Leave programming mode; run = let the chip start its program. Always called, also after errors. */
    void (*leave)(prog_ctx_t *ctx, bool run);
} prog_proto_t;

/* The table (prog_flash.c): one entry per protocol built in. */
const prog_proto_t *prog_proto_find(const char *name);
size_t prog_proto_count(void);
const prog_proto_t *prog_proto_at(size_t i);

/* One line per protocol; each prog_<proto>.c defines its symbol. */
extern const prog_proto_t prog_proto_updi;
extern const prog_proto_t prog_proto_pic;
extern const prog_proto_t prog_proto_c2;
extern const prog_proto_t prog_proto_tpi;
extern const prog_proto_t prog_proto_pdi;

/* ---- the job (prog_flash.c), on the hw worker ---- */

typedef enum { PROG_OP_INFO = 0, PROG_OP_START } prog_op_t;

typedef struct {
    const prog_proto_t *proto;
    uint8_t  op;              /* prog_op_t */
    uint8_t  clk, data;       /* LA channels (UPDI: data only) */
    uint8_t  mem, erase;
    uint32_t addr, len;
    uint16_t page;
    uint32_t baud, flags;
    uint8_t  expect_id[8];    /* compared with the chip's ID when expect_len > 0 */
    uint8_t  expect_len;
    bool     run;             /* let the chip start its program at the end */
    uint8_t  reset;           /* the chip's reset: 0 = the pod's reset pin, else an LA pin (open drain) */
} prog_params_t;

/* Claims the pins, checks the staged upload, starts the job. 0 = started; < 0 = err says why. */
int  prog_flash_start(const prog_params_t *p, char *err, size_t cap);
void prog_flash_poll(void);     /* hw worker loop: one bounded step */
void prog_flash_abort(void);
bool prog_flash_busy(void);

typedef struct {
    bool        running, have_result;
    const char *op, *proto, *phase, *result;   /* result NULL while running */
    uint32_t    done, total, at_addr, ms;
    uint8_t     id[8], id_len;
    int         rc;
} prog_status_t;
void prog_flash_status(prog_status_t *st);
const char *prog_rc_str(int rc);

#endif /* PROG_FLASH_H */
