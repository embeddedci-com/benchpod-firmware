/* ============================================================================
 * sensor_def_int.h — internals shared by the sensor definition engine's files:
 * the compiled program layout, the VM's opcodes and values, the bounded JSON
 * reader and the deterministic math.  Not a public API.
 * ========================================================================== */
#pragma once

#include "sensor_def.h"

#include <string.h>

/* Same float math everywhere: no fused multiply-add in the engine. */
#if defined(__clang__)
#pragma STDC FP_CONTRACT OFF
#endif

#if defined(__GNUC__) || defined(__clang__)
#define SD_NOINLINE __attribute__((noinline))
#else
#define SD_NOINLINE
#endif

/* ---- variable slots (sd_inst_t.vars) ---- */
#define SDV_PARAM   0u
#define SDV_LET     (SDV_PARAM + SD_MAX_PARAMS)
#define SDV_STATE   (SDV_LET + SD_MAX_LETS)
#define SDV_BUILTIN (SDV_STATE + SD_MAX_STATES)
enum { SDB_T = 0, SDB_UTC, SDB_DT, SDB_WRITTEN, SDB_VALUE, SDB_COUNT, SDB_FIFO, SDB_I, SDB_INPUT,
       SDB_N };
#define SDV_N (SDV_BUILTIN + SDB_N)
#define SDV_LAST_OUT SDV_N      /* analog: the last output (a stuck fault holds it) */
_Static_assert(SDV_N + 1 <= SD_MAX_PARAMS + SD_MAX_LETS + SD_MAX_STATES + 12, "vars too small");

/* ---- values: a double, or a string NaN-boxed into one (an sNaN pattern arithmetic never makes) ---- */
#define SDS_TAG_PROG    0x7FF4u   /* bits 63..48: string in the program */
#define SDS_TAG_SCRATCH 0x7FF5u   /* string in the instance's scratch */
static inline uint64_t sdv_num(double d) {
    uint64_t u; memcpy(&u, &d, 8);
    if ((u & 0x7FF0000000000000ull) == 0x7FF0000000000000ull && (u & 0x000FFFFFFFFFFFFFull))
        u = 0x7FF8000000000000ull;   /* one canonical NaN, never a string tag */
    return u;
}
static inline double sd_inf(void) { uint64_t u = 0x7FF0000000000000ull; double d; memcpy(&d, &u, 8); return d; }
static inline double sd_nan(void) { uint64_t u = 0x7FF8000000000000ull; double d; memcpy(&d, &u, 8); return d; }
static inline double sdv_d(uint64_t u) { double d; memcpy(&d, &u, 8); return d; }
static inline bool sdv_is_str(uint64_t u) { return ((u >> 48) & 0xFFFEu) == SDS_TAG_PROG; }
static inline uint64_t sdv_str(unsigned tag, uint32_t off, uint32_t len) {
    return ((uint64_t)tag << 48) | ((uint64_t)(len & 0xFFFFu) << 32) | off;
}
static inline uint32_t sdv_str_off(uint64_t u) { return (uint32_t)u; }
static inline uint32_t sdv_str_len(uint64_t u) { return (uint32_t)(u >> 32) & 0xFFFFu; }

/* ---- program layout ----
   Every record starts 8-byte aligned; offsets are from the start of the program. */
#define SDP_MAGIC   0x4453u   /* "SD" */
#define SDP_VERSION 1u

/* entry types */
enum { SDT_U8 = 0, SDT_S8, SDT_U16BE, SDT_U16LE, SDT_S16BE, SDT_S16LE, SDT_U24BE, SDT_U24LE,
       SDT_S24BE, SDT_S24LE, SDT_U32BE, SDT_U32LE, SDT_S32BE, SDT_S32LE, SDT_BYTES, SDT_RAW,
       SDT_N };
/* entry flags */
#define SDE_DYN       0x01u   /* rebuilt on every change */
#define SDE_RO        0x02u   /* writable: false: the pod restores it */
#define SDE_CMD       0x04u   /* a commands entry */

typedef struct {
    uint8_t  at;        /* image index */
    uint8_t  type;      /* SDT_* */
    uint8_t  len;       /* bytes it covers (BYTES/RAW: count) */
    uint8_t  flags;
    uint16_t code;      /* numeric: code; BYTES: u16 code[len]; RAW: bytes */
    uint16_t pad;
    uint32_t busy_us;   /* commands */
} sdp_entry_t;

typedef struct {
    uint16_t key, unit, label;  /* string records: u8 len + chars */
    uint16_t pad;
    double   min, max, def;
} sdp_param_t;

typedef struct { uint16_t init, update; int16_t follow; uint16_t pad; } sdp_state_t;
typedef struct { uint16_t code; uint8_t nargs, nlets; } sdp_func_t;   /* functions and fragments */
typedef struct { uint16_t when; uint8_t reset; uint8_t pad; sdp_entry_t set; } sdp_rule_t;

enum { SDEND_NONE = 0, SDEND_NMEA, SDEND_CRLF, SDEND_LF };
typedef struct { uint16_t every, enabled, repeat, text; uint8_t end, pad[3]; } sdp_msg_t;

typedef struct {
    uint8_t  line;       /* 1: match_line */
    uint8_t  mlen;       /* bytes of match (or line length) */
    uint8_t  text;       /* 1: the answer is a template, else a byte list */
    uint8_t  alen;       /* answer bytes (byte list) */
    uint16_t match;      /* bytes then a wildcard bitmap ((mlen + 7) / 8) / line chars */
    uint16_t answer;     /* template code / u16 code[alen] */
    uint8_t  end;
    uint8_t  pad;
    uint16_t delay_ms;
} sdp_req_t;

typedef struct {
    uint16_t magic;
    uint8_t  version, kind;
    uint32_t size;
    uint16_t name, label;        /* string records */
    uint8_t  n_params, n_lets, n_states, n_funcs, n_frags, n_entries, n_rules, n_msgs, n_reqs;
    uint8_t  fill;
    int8_t   utc_param;          /* param index of utc_start, -1 none */
    uint8_t  n_cmds;
    uint16_t params, lets, states, funcs, frags, entries, owner, rules, msgs, reqs;
    /* i2c / spi */
    uint8_t  addr7, n_alt, alt[3], addr_bits, auto_inc, wrap;
    uint16_t page;
    uint8_t  spi_mode, read_bit, read_val, inc_bit, inc_val, has_inc, address_mask, dummy_bytes;
    uint8_t  has_conv, conv_trigger, conv_busy_at, conv_busy_mask, while_busy;
    uint32_t conv_us;
    uint8_t  has_drdy, drdy_high, drdy_level, drdy_on;
    uint32_t drdy_pulse_us;
    int16_t  drdy_clear;
    uint16_t drdy_rate, drdy_enabled;     /* code, 0 none */
    uint8_t  has_fifo, fifo_at, fifo_len, fifo_drop_newest;
    uint16_t fifo_depth, fifo_frame, fifo_rate, fifo_enabled;   /* frame: u16 code[fifo_len] */
    /* uart */
    uint32_t baud;
    uint8_t  data_bits, parity, stop_bits;
    /* analog */
    uint8_t  current, excitation;
    uint16_t value;
    double   range_min, range_max;
} sdp_hdr_t;

static inline const sdp_hdr_t *sdp_hdr(const uint8_t *p) { return (const sdp_hdr_t *)(const void *)p; }
/* A string record: u8 length then the characters. */
static inline const char *sdp_str(const uint8_t *p, uint16_t off, uint8_t *len) {
    if (!off) { *len = 0; return ""; }
    *len = p[off];
    return (const char *)p + off + 1;
}

/* ---- opcodes ---- */
enum {
    OP_RET = 0,
    OP_I8,       /* i8 */
    OP_I16,      /* i16 */
    OP_F64,      /* 8 bytes */
    OP_STR,      /* u8 len, chars (the string value points at them) */
    OP_VAR,      /* u8 slot */
    OP_LOCAL,    /* u8 index in the frame */
    OP_REG,      /* pop addr: the live byte */
    OP_REG16BE, OP_REG16LE,
    OP_NEG, OP_NOT, OP_BNOT,
    OP_ADD, OP_SUB, OP_MUL, OP_DIV, OP_MOD, OP_POW, OP_SHL, OP_SHR, OP_BAND, OP_BXOR, OP_BOR,
    OP_LT, OP_LE, OP_GT, OP_GE, OP_EQ, OP_NE,
    OP_JZ,       /* i16: pop, jump if false */
    OP_JMP,      /* i16 */
    OP_ANDJ,     /* i16: if top false: replace with 0 and jump, else pop */
    OP_ORJ,      /* i16: if top true: replace with 1 and jump, else pop */
    OP_BOOL,     /* top -> 0 / 1 */
    OP_BUILTIN,  /* u8 id, u8 argc */
    OP_CALL,     /* u8 function */
    OP_FRAG,     /* u8 fragment */
    OP_SOLVE,    /* u8 function, u8 placeholder position (0xFF: one-argument form) */
    OP_CTAB,     /* u8 n, then n * 8 bytes; pop index */
    OP_TAB,      /* u8 n; pop index, then n values */
    OP_TXT,      /* u8 len, chars: append */
    OP_TXTV,     /* u8 flags, u8 width, u8 prec, u8 conv: pop, append formatted */
    OP_N
};
/* OP_TXTV flags / conv */
#define SDF_MINUS 0x01u
#define SDF_PLUS  0x02u
#define SDF_ZERO  0x04u
#define SDF_PREC  0x08u
enum { SDC_DEFAULT = 0, SDC_D, SDC_X, SDC_XU, SDC_F };

/* built-in functions */
enum {
    BI_ABS = 0, BI_MIN, BI_MAX, BI_CLAMP, BI_ROUND, BI_FLOOR, BI_CEIL, BI_TRUNC, BI_SQRT, BI_EXP,
    BI_LOG, BI_LOG10, BI_POW, BI_SIN, BI_COS, BI_TAN, BI_ASIN, BI_ACOS, BI_ATAN, BI_ATAN2, BI_RAD,
    BI_DEG, BI_IDIV, BI_IMOD, BI_U8, BI_S8, BI_U16, BI_S16, BI_U24, BI_S24, BI_U32, BI_S32,
    BI_SAT_U8, BI_SAT_S8, BI_SAT_U16, BI_SAT_S16, BI_SAT_U24, BI_SAT_S24, BI_SAT_U32, BI_SAT_S32,
    BI_BIT, BI_BITS, BI_BYTE, BI_CRC8, BI_CRC16, BI_SUM8, BI_XOR8, BI_RANDOM, BI_HHMMSS,
    BI_DDMMYY, BI_NMEA_LAT, BI_NMEA_LON, BI_N
};
typedef struct { const char *name; uint8_t min_args, max_args; bool str_result; } sd_builtin_t;
extern const sd_builtin_t sd_builtins[BI_N];
int sd_builtin_find(const char *name, size_t len);   /* -1 none */

/* ---- VM (sensor_def_vm.c) ---- */
/* Evaluate code at offset `code` (an expression or a template).  A template writes
   to in->out.  *result gets the expression's value (string or number). */
int  sd_vm_eval(sd_inst_t *in, uint16_t code, uint64_t *result);
int  sd_vm_num(sd_inst_t *in, uint16_t code, double *result);   /* a number result */
void sd_vm_begin(sd_inst_t *in);         /* new evaluation: steps 0, scratch empty */
/* The pure operations, shared with the compiler's constant folding. */
double sd_op_binary(int op, double a, double b);
double sd_op_unary(int op, double a);
int    sd_builtin_num(int id, const double *a, unsigned n, double *out);   /* numeric built-ins */
int64_t sd_to_i64(double v);
double sd_sat(double v, int bits, bool is_signed);    /* round half away, clamp */
double sd_wrap(double v, int bits, bool is_signed);   /* truncate, wrap */
/* Number formatting for templates: returns chars written (no NUL), out of space -> cap+1. */
size_t sd_fmt_num(double v, uint8_t flags, uint8_t width, uint8_t prec, uint8_t conv, char *out,
                  size_t cap);
void   sd_set_err(sd_inst_t *in, int code, const char *msg);

/* ---- instance helpers (sensor_def_inst.c) ---- */
/* t, utc, dt, the states (in order) and the lets (in order), for the current time. */
int  sd_prepare(sd_inst_t *in);
static inline const sdp_hdr_t *sd_h(const sd_inst_t *in) { return sdp_hdr(in->prog); }
static inline uint16_t sdp_code_at(const uint8_t *p, uint16_t arr, unsigned i) {
    return (uint16_t)(p[arr + 2u * i] | (p[arr + 2u * i + 1u] << 8));
}

/* ---- deterministic math (sensor_def_math.c) ---- */
double sd_exp(double x);
double sd_log(double x);
double sd_pow(double x, double y);
double sd_sin(double x);
double sd_cos(double x);
double sd_tan(double x);
double sd_atan(double x);
double sd_atan2(double y, double x);
double sd_asin(double x);
double sd_acos(double x);
#define SD_PI 3.14159265358979323846

/* Exact helpers (no rounding differences between platforms). */
double sd_parse_number(const char *s, size_t n, size_t *used);   /* used = 0: not a number */

/* ---- bounded JSON reader (sensor_def_json.c) ----
   Validate once, then walk values by offset without copying or allocating. */
typedef struct { const char *s; uint32_t n; } sdj_t;
#define SDJ_MAX_DEPTH 24
int  sdj_validate(const char *s, size_t n, uint32_t *err_off, const char **why);
uint32_t sdj_ws(const sdj_t *j, uint32_t o);          /* skip whitespace */
char     sdj_type(const sdj_t *j, uint32_t o);        /* '{' '[' '"' 'n'(number) 't' 'f' 'z'(null) */
uint32_t sdj_skip(const sdj_t *j, uint32_t o);        /* offset after the value */
/* Iterate an object: *it starts at the '{' offset; returns false at the end. */
bool sdj_obj_next(const sdj_t *j, uint32_t *it, uint32_t *key, uint32_t *val);
bool sdj_arr_next(const sdj_t *j, uint32_t *it, uint32_t *val);
uint32_t sdj_count(const sdj_t *j, uint32_t o);       /* members / elements */
bool sdj_key_is(const sdj_t *j, uint32_t key, const char *name);
bool sdj_find(const sdj_t *j, uint32_t obj, const char *name, uint32_t *val);
/* The raw characters of a string value or key (between the quotes; escapes left in). */
const char *sdj_raw(const sdj_t *j, uint32_t o, uint32_t *len, bool *has_escape);
/* Decode a string (escapes resolved, \u to UTF-8); false if it does not fit. */
bool sdj_str(const sdj_t *j, uint32_t o, char *out, size_t cap, size_t *len);
bool sdj_num(const sdj_t *j, uint32_t o, double *v);
/* The path of the value or key at `off`, from the document's root: "registers[3].s16be". */
void sdj_path_at(const sdj_t *j, uint32_t root, uint32_t off, char *out, size_t cap);
