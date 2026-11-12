/* ============================================================================
 * sensor_def_cc.h — the sensor definition compiler's internals, shared by
 * sensor_def_compile.c (the definition's structure) and sensor_def_expr.c
 * (expressions and templates).  Not a public API.
 * ========================================================================== */
#pragma once

#include "sensor_def_int.h"

/* dependency bits of an expression: any of them makes a register entry dynamic */
#define DEP_PARAM 0x01u
#define DEP_REG   0x02u
#define DEP_TIME  0x04u
#define DEP_STATE 0x08u
#define DEP_WRITE 0x10u
#define DEP_COUNT 0x20u
#define DEP_FIFO  0x40u
#define DEP_INPUT 0x80u

enum { SYM_PARAM = 1, SYM_CONST, SYM_LET, SYM_STATE, SYM_FUNC, SYM_FRAG };

typedef struct {
    uint16_t name;      /* JSON offset of the name's characters */
    uint8_t  len;
    uint8_t  kind;      /* SYM_* */
    uint8_t  index;
    uint8_t  deps;
    uint8_t  nargs;
    uint8_t  let_hi;    /* functions: lets it reads, +1 (0 none); lets: its own index + 1 */
    uint64_t cval;      /* constants: the value */
} sd_sym_t;

/* AST */
enum { N_NUM = 1, N_STR, N_CSTR, N_VAR, N_LOCAL, N_UN, N_BIN, N_AND, N_OR, N_COND, N_BUILTIN,
       N_CALL, N_FRAG, N_SOLVE, N_TABLE, N_REG, N_HOLE };
enum { TY_NUM = 0, TY_STR, TY_ANY };
typedef struct {
    uint8_t  kind, op, type, depth;
    uint16_t kid, next;     /* first child, next sibling (0 = none; node 0 is unused) */
    uint16_t src;           /* offset in the expression text */
    uint16_t aux;           /* builtin id, function, slot, string length */
    union { double num; uint64_t cval; uint32_t soff; } u;
} sd_node_t;

#define SD_CC_NODES  256u
#define SD_CC_SYMS   200u
#define SD_CC_RANGES 512u
#define SD_CC_MAXDEPTH 40u   /* AST depth (emit and fold recurse over it) */
#define SD_CC_MAXNEST  20u   /* parser nesting: parentheses, calls, operators (C stack) */

typedef struct { uint16_t code_lo, code_hi, json; } sd_range_t;
#define SD_CC_SRC (SD_MAX_EXPR + 8u)

/* what an expression may use where it is compiled */
#define CTX_NO_TIME   0x01u   /* constants: no t, params, lets, ... */
#define CTX_NO_LETS   0x02u   /* state expressions: no global lets */
#define CTX_LETS_BELOW 0x04u  /* a let: only lets before it */
#define CTX_FUNC      0x08u   /* inside a function body: any let (checked at the call) */
#define CTX_ANALOG    0x10u   /* `input` allowed */

typedef struct {
    uint16_t name; uint8_t len;
} sd_local_t;

typedef struct {
    sdj_t    j;
    uint32_t root;
    /* program being built */
    uint8_t *p;
    uint32_t cap, len;
    sdp_hdr_t *h;
    /* work arena pieces */
    sd_sym_t  *syms;  uint16_t nsyms;
    sd_node_t *nodes; uint16_t nnodes;
    sd_range_t *ranges; uint16_t nranges;
    char      *src;   uint32_t src_cap;     /* the decoded expression text */
    uint32_t   src_len;
    sd_inst_t *dry;                          /* for the trial run */
    /* the expression being compiled */
    uint32_t   json_off;                     /* JSON offset of the value */
    uint32_t   src_base;                     /* offset of this part inside the value's text */
    uint8_t    ctx;
    uint8_t    let_limit;                    /* CTX_LETS_BELOW: lets [0, let_limit) */
    const sd_local_t *locals; uint8_t nlocals;
    uint8_t    func_limit, frag_limit;           /* callable: functions/fragments below these */
    uint8_t    const_limit;                      /* constants section: earlier constants only */
    uint8_t    deps;                         /* collected */
    uint8_t    let_hi;                       /* collected: highest let read + 1 */
    sd_err_t  *err;
    bool       failed;
} sd_cc_t;

/* errors: record the first one */
void cc_fail(sd_cc_t *c, int code, uint32_t json_off, int32_t expr_off, const char *fmt, const char *arg);

/* emit into the program */
bool     cc_emit(sd_cc_t *c, const void *d, size_t n);
bool     cc_emit8(sd_cc_t *c, uint8_t v);
bool     cc_emit16(sd_cc_t *c, uint16_t v);
uint32_t cc_align(sd_cc_t *c);                       /* pad to 8; returns the offset */
uint32_t cc_alloc(sd_cc_t *c, size_t n);             /* zeroed, aligned record; 0 on failure */
uint16_t cc_string(sd_cc_t *c, const char *s, size_t n);   /* a string record (u8 len + chars) */
void     cc_range(sd_cc_t *c, uint32_t code_lo, uint32_t json_off);

/* symbols */
bool cc_valid_name(const sd_cc_t *c, uint32_t off, uint32_t len);   /* [A-Za-z_][A-Za-z0-9_]{0,31} */
int  cc_sym_find(const sd_cc_t *c, const char *name, size_t len);   /* index or -1 */

/* Compile the JSON value at json_off (a string, a number or a boolean) as an
   expression; returns the code offset (0 on failure).  deps/let_hi land in c. */
uint16_t cc_expr(sd_cc_t *c, uint32_t json_off, uint8_t ctx);
/* The same for a template (literal text with {expr:fmt} parts). */
uint16_t cc_template(sd_cc_t *c, uint32_t json_off, uint8_t ctx);
/* Constant-evaluate an expression value (constants section): number or string. */
bool cc_const_value(sd_cc_t *c, uint32_t json_off, uint64_t *out);
/* Compile the body of a function (args, let, return) / fragment (args, text). */
bool cc_function(sd_cc_t *c, uint32_t obj, sdp_func_t *out, bool fragment, uint8_t *let_hi, uint8_t *deps);
