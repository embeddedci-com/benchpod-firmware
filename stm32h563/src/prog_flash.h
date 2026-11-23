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

/* What the pod knows about a protocol without running it: the request checks in prog_flash and the
   job read these. Internal flash (prog_flash.c). The protocol code itself runs from the W25Q
   (xip_image.h, image "prog"): internal code reaches it only through prog_xip_call(). */
typedef struct {
    const char *name;        /* "updi", "pic", "c2", "tpi", "pdi" */
    bool        uart;        /* true: one pin over the UART engine (UPDI); false: clk + data */
    bool        uses_nrst;   /* true: the pod's reset pin is part of the wiring (PIC MCLR, TPI RESET) */
    uint8_t     mems;        /* bit mask of prog_mem_t it supports */
    uint8_t     id;          /* prog_proto_id_t: the protocol's number in the XIP dispatch */
} prog_proto_info_t;

typedef enum { PROG_PROTO_UPDI = 0, PROG_PROTO_PIC, PROG_PROTO_C2, PROG_PROTO_TPI, PROG_PROTO_PDI,
               PROG_PROTO_COUNT } prog_proto_id_t;

/* The metadata, once per protocol: used by prog_flash.c's table and by the protocol's own source. */
#define PROG_INFO_UPDI .name = "updi", .uart = true,  .uses_nrst = false, .id = PROG_PROTO_UPDI, \
    .mems = (1u<<PROG_MEM_FLASH)|(1u<<PROG_MEM_EEPROM)|(1u<<PROG_MEM_CONFIG)|(1u<<PROG_MEM_USER)
#define PROG_INFO_PIC  .name = "pic",  .uart = false, .uses_nrst = true,  .id = PROG_PROTO_PIC, \
    .mems = (1u<<PROG_MEM_FLASH)|(1u<<PROG_MEM_EEPROM)|(1u<<PROG_MEM_CONFIG)|(1u<<PROG_MEM_USER)
#define PROG_INFO_C2   .name = "c2",   .uart = false, .uses_nrst = false, .id = PROG_PROTO_C2, \
    .mems = (1u<<PROG_MEM_FLASH)|(1u<<PROG_MEM_CONFIG)
#define PROG_INFO_TPI  .name = "tpi",  .uart = false, .uses_nrst = true,  .id = PROG_PROTO_TPI, \
    .mems = (1u<<PROG_MEM_FLASH)|(1u<<PROG_MEM_CONFIG)
#define PROG_INFO_PDI  .name = "pdi",  .uart = false, .uses_nrst = false, .id = PROG_PROTO_PDI, \
    .mems = (1u<<PROG_MEM_FLASH)|(1u<<PROG_MEM_EEPROM)|(1u<<PROG_MEM_CONFIG)|(1u<<PROG_MEM_USER)

/* What a protocol does, by operation: the `op` of prog_xip_call(). The job owns the sequence. */
typedef enum {
    PROG_XOP_ENTER = 0,   /* enter programming mode and read the ID into ctx->id */
    PROG_XOP_ERASE,       /* the erase ctx->erase asks for */
    PROG_XOP_WRITE,       /* next page of [addr, addr+len) from src, verified */
    PROG_XOP_READ,        /* next block of [addr, addr+len) to dst */
    PROG_XOP_LEAVE,       /* leave programming mode, the chip stays in reset or idle */
    PROG_XOP_LEAVE_RUN,   /* leave programming mode and let the chip start its program */
} prog_xop_t;

/* Run one operation of protocol `proto` (prog_proto_id_t): the protocol code in the "prog" XIP
   image, so a session for that image must be active (the job holds it from start to end). Returns
   what the protocol call returns (PROG_RUNNING, PROG_OK, prog_rc_t error); leave returns PROG_OK.
   Defined in prog_job.c; on the pod it calls the image's table entry XIP_PROG_FN_CALL. */
int prog_xip_call(prog_ctx_t *ctx, unsigned proto, unsigned op);

/* The metadata table (prog_flash.c): one entry per protocol built in. */
const prog_proto_info_t *prog_proto_find(const char *name);
size_t prog_proto_count(void);
const prog_proto_info_t *prog_proto_at(size_t i);

/* Each prog_<proto>.c defines its entry, which the XIP image's dispatch (xip_mod_prog.c) calls:
   `op` is a prog_xop_t. Plain C, so host tests link the sources directly. */
int prog_updi_call(prog_ctx_t *ctx, unsigned op);
int prog_pic_call(prog_ctx_t *ctx, unsigned op);
int prog_c2_call(prog_ctx_t *ctx, unsigned op);
int prog_tpi_call(prog_ctx_t *ctx, unsigned op);
int prog_pdi_call(prog_ctx_t *ctx, unsigned op);

/* Host tests (and nothing on the pod) call a protocol's steps through this struct: every
   prog_<proto>.c defines one with PROG_PROTO() unless the firmware build says PROG_XIP (the
   function pointers would sit inside the XIP image, which the rules in xip_image.h forbid). */
typedef struct {
    const char *name;
    bool        uart;
    bool        uses_nrst;
    uint8_t     mems;
    uint8_t     id;
    int (*enter)(prog_ctx_t *ctx);
    int (*erase)(prog_ctx_t *ctx);
    int (*write)(prog_ctx_t *ctx);
    int (*read)(prog_ctx_t *ctx);
    void (*leave)(prog_ctx_t *ctx, bool run);
} prog_proto_t;

/* At the end of prog_<proto>.c: the XIP entry (always) and the test struct (host builds). */
#define PROG_PROTO(nm, fenter, ferase, fwrite, fread, fleave, INFO)                              \
    int prog_##nm##_call(prog_ctx_t *c, unsigned op)                                             \
    {                                                                                            \
        switch (op) {                                                                            \
        case PROG_XOP_ENTER:     return fenter(c);                                               \
        case PROG_XOP_ERASE:     return ferase(c);                                               \
        case PROG_XOP_WRITE:     return fwrite(c);                                               \
        case PROG_XOP_READ:      return fread(c);                                                \
        case PROG_XOP_LEAVE:     fleave(c, false); return PROG_OK;                               \
        case PROG_XOP_LEAVE_RUN: fleave(c, true);  return PROG_OK;                               \
        default:                 return PROG_E_ARGS;                                             \
        }                                                                                        \
    }                                                                                            \
    PROG_PROTO_STRUCT(nm, fenter, ferase, fwrite, fread, fleave, INFO)
#if defined(PROG_XIP)
#define PROG_PROTO_STRUCT(nm, fenter, ferase, fwrite, fread, fleave, ...)
#else
#define PROG_PROTO_STRUCT(nm, fenter, ferase, fwrite, fread, fleave, ...)                       \
    const prog_proto_t prog_proto_##nm = { __VA_ARGS__, .enter = fenter, .erase = ferase,               \
        .write = fwrite, .read = fread, .leave = fleave };
#endif
extern const prog_proto_t prog_proto_updi, prog_proto_pic, prog_proto_c2, prog_proto_tpi, prog_proto_pdi;

/* ---- the job (prog_flash.c), on the hw worker ---- */

typedef enum { PROG_OP_INFO = 0, PROG_OP_START } prog_op_t;

typedef struct {
    const prog_proto_info_t *proto;
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
