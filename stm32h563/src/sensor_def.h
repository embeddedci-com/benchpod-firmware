/* ============================================================================
 * sensor_def.h — the engine that plays user-defined BenchPod sensors.
 *
 * A sensor definition (format: benchpod-sensors docs/format.md) arrives as JSON.
 * sd_compile() checks it and turns it into a compact program (at most
 * SD_PROG_MAX bytes) in a buffer the caller owns; sd_inst_init() starts an
 * instance of that program.  An instance then builds I2C/SPI register images,
 * UART epochs and answers, or an analog output, from its params.  A stimulus
 * file compiles the same way (sd_stim_compile) into a player that yields param
 * values and fault events over time.
 *
 * The engine is pure C11: no RTOS, HAL or heap, no globals except const
 * tables, and the same double math everywhere (its own transcendental
 * functions, fixed iteration counts), so the pod, a browser (WASM) and the
 * server compute the same bytes.  Nothing here talks to hardware; the manager
 * that loads images into the gateware sits on top.
 *
 * Time is milliseconds since the instance started (int64_t), set by the caller.
 * ========================================================================== */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- limits (format.md section 1; the compiler reports the one that was hit) ---- */
#define SD_JSON_MAX        32768u   /* bytes of definition JSON */
#define SD_PROG_MAX        8192u    /* bytes of compiled program per instance */
#define SD_COMPILE_WORK    20480u   /* scratch sd_compile needs (caller provided) */
#define SD_MAX_PARAMS      16u
#define SD_MAX_CONSTS      64u
#define SD_MAX_FUNCS       16u
#define SD_MAX_FRAGS       16u
#define SD_MAX_LETS        64u
#define SD_MAX_STATES      16u
#define SD_MAX_ENTRIES     255u     /* register + command entries */
#define SD_MAX_RULES       32u      /* on_write rules */
#define SD_MAX_MSGS        32u      /* UART messages */
#define SD_MAX_REQS        32u      /* UART requests */
#define SD_MAX_ARGS        8u       /* arguments of a function or fragment */
#define SD_MAX_FLETS       16u      /* let entries inside one function */
#define SD_MAX_EXPR        1024u    /* characters of one expression or template */
#define SD_MAX_MATCH       64u      /* bytes of a request match / line */
#define SD_SOLVE_MAX_ITER  64u      /* bisection steps of one solve() */
#define SD_STEP_LIMIT      200000u  /* VM steps per rebuild, epoch, answer or evaluation */
#define SD_STACK           96u      /* VM value stack */
#define SD_FRAMES          24u      /* VM call depth (functions + fragments + solve) */
#define SD_SCRATCH         768u     /* bytes of string scratch per evaluation */
#define SD_REGIMAGE        256u

/* How many sensors a pod runs at once (format.md section 9); the manager enforces it. */
#define SD_MAX_I2C_INST    4u
#define SD_MAX_SPI_INST    1u
#define SD_MAX_UART_INST   1u
#define SD_MAX_ANALOG_INST 4u

typedef enum { SD_KIND_NONE = 0, SD_KIND_I2C, SD_KIND_SPI, SD_KIND_UART, SD_KIND_ANALOG } sd_kind_t;

/* ---- errors ---- */
enum {
    SD_OK = 0,
    SD_E_JSON = -1,       /* not valid JSON */
    SD_E_SCHEMA = -2,     /* a missing, unknown or wrong-typed key */
    SD_E_LIMIT = -3,      /* a limit above was hit */
    SD_E_EXPR = -4,       /* an expression does not compile */
    SD_E_NAME = -5,       /* unknown or duplicate name */
    SD_E_EVAL = -6,       /* an evaluation failed (type, step limit, stack) */
    SD_E_SOLVE = -7,      /* solve(): not monotonic, bad range */
    SD_E_SPACE = -8,      /* the program buffer, scratch or an output is too small */
    SD_E_ARG = -9,        /* a bad argument to an API call */
    SD_E_RANGE = -10,     /* a param value outside its range */
};

typedef struct {
    int      code;          /* SD_E_* */
    uint32_t json_off;      /* byte offset in the JSON of the value at fault */
    int32_t  expr_off;      /* character offset inside the expression, -1 if none */
    char     path[72];      /* "registers[3].s16be" */
    char     msg[96];       /* "unknown name 'acel_lsb'" */
} sd_err_t;

/* "registers[3].s16be: unknown name 'acel_lsb' at 12" (format.md section 10). */
size_t sd_err_format(const sd_err_t *e, char *out, size_t cap);

/* ---- compile ---- */
/* Compile a definition.  prog: the program buffer (8-byte aligned, at least
   SD_PROG_MAX is plenty), work: scratch of at least SD_COMPILE_WORK bytes
   (8-byte aligned), discarded afterwards.  *prog_len gets the program size.
   The compiler also starts the program once at its defaults, so a definition
   that compiles also evaluates. */
int sd_compile(const char *json, size_t json_len, void *prog, size_t prog_cap,
               void *work, size_t work_cap, size_t *prog_len, sd_err_t *err);

/* ---- instance ---- */
typedef struct {
    const char *key;  uint8_t key_len;
    const char *unit; uint8_t unit_len;
    const char *label; uint8_t label_len;
    double min, max, def;
} sd_param_info_t;

/* Fault types (format.md section 8). */
typedef enum {
    SD_FAULT_NONE = 0, SD_FAULT_STUCK, SD_FAULT_OFFSET, SD_FAULT_GAIN, SD_FAULT_NO_RESPONSE,
    SD_FAULT_CORRUPT, SD_FAULT_BUS_STUCK, SD_FAULT_GARBAGE, SD_FAULT_DROP, SD_FAULT_BAUD_ERROR,
    SD_FAULT_OPEN, SD_FAULT_SHORT,
} sd_fault_type_t;

typedef struct {
    sd_fault_type_t type;
    double   at_s;            /* stimulus time it starts (0 = at once) */
    double   duration_s;      /* < 0: until cleared */
    bool     has_value;
    double   value;
    double   rate_hz;         /* garbage */
    int16_t  at;              /* corrupt: register, -1 none */
    uint8_t  mask;            /* corrupt: XOR mask */
    char     sensor[33];      /* instance name, "" = the only one */
    char     param[33];       /* param key, "" = all */
} sd_fault_t;

/* Which kinds a fault type applies to (format.md section 8). */
bool sd_fault_allowed(sd_fault_type_t type, sd_kind_t kind);
const char *sd_fault_name(sd_fault_type_t type);

/* Per-message UART bookkeeping. */
typedef struct {
    int64_t  next_ms;         /* when it is due next */
    uint32_t count;           /* times sent */
} sd_msg_state_t;

/* An instance: the program it runs plus all of its run-time state.  Treat the
   fields as private; the size is fixed so a manager can allocate them statically. */
typedef struct sd_inst {
    const uint8_t *prog;
    uint32_t prog_len;
    uint64_t vars[SD_MAX_PARAMS + SD_MAX_LETS + SD_MAX_STATES + 12];
    double   raw[SD_MAX_PARAMS];          /* the params as set; vars hold them after faults */
    double   gain[SD_MAX_PARAMS], offset[SD_MAX_PARAMS], stuck_val[SD_MAX_PARAMS];
    uint16_t stuck;                       /* bit per param */
    bool     frozen;                      /* a stuck fault without a param */
    bool     started;
    double   follow_last[SD_MAX_STATES];
    int64_t  now_ms, state_ms;
    sd_msg_state_t msg[SD_MAX_MSGS];
    uint32_t req_count[SD_MAX_REQS];
    /* UART request matcher */
    uint8_t  rx_win[SD_MAX_MATCH]; uint8_t rx_win_len;
    uint8_t  rx_line[SD_MAX_MATCH]; uint8_t rx_line_len; bool rx_line_over;
    int64_t  rx_last_ms;
    /* analog open/short fault: 0 none, 1 open, 2 short */
    uint8_t  force_out; bool force_has_val; double force_val;
    uint8_t  img_base[SD_REGIMAGE];       /* the live image a rebuild reads (reg[]) */
    /* corrupt fault: what is XOR-ed into the image now */
    int16_t  corrupt_at; uint8_t corrupt_mask;
    int16_t  corrupt_applied_at; uint8_t corrupt_applied_mask;
    /* VM */
    uint64_t stack[SD_STACK];
    uint16_t sp;
    uint8_t  depth;
    uint8_t  scratch[SD_SCRATCH];
    uint16_t scratch_top;
    uint32_t steps;
    uint32_t max_steps_seen;             /* largest step count of one evaluation, for budgets */
    const uint8_t *regsrc;                /* reg[] reads this image */
    char    *out; size_t out_cap, out_len; bool out_over;
    sd_err_t err;                         /* the last run-time error */
} sd_inst_t;

int  sd_inst_init(sd_inst_t *in, const void *prog, size_t prog_len, sd_err_t *err);
sd_kind_t sd_kind(const sd_inst_t *in);
const char *sd_name(const sd_inst_t *in, size_t *len);
const char *sd_label(const sd_inst_t *in, size_t *len);           /* "" when it has none */
unsigned sd_n_params(const sd_inst_t *in);
int  sd_param_index(const sd_inst_t *in, const char *key);          /* -1 unknown */
int  sd_param_info(const sd_inst_t *in, unsigned i, sd_param_info_t *out);
/* Set param i.  clamp=false: SD_E_RANGE outside [min, max] (the old value stays);
   clamp=true (stimulus values): clamped into the range. */
int  sd_set_param(sd_inst_t *in, unsigned i, double v, bool clamp);
double sd_get_param(const sd_inst_t *in, unsigned i);                /* as set */
void sd_set_time_ms(sd_inst_t *in, int64_t now_ms);                  /* t for the next rebuild */
/* Fault on/off.  Value faults (stuck, offset, gain, corrupt, open, short) act in
   the engine; bus faults are for the manager and only validated here. */
int  sd_set_fault(sd_inst_t *in, const sd_fault_t *f, bool on);
void sd_clear_faults(sd_inst_t *in);
const sd_err_t *sd_last_error(const sd_inst_t *in);
uint32_t sd_max_steps(const sd_inst_t *in);   /* most VM steps one evaluation took */

/* ---- I2C / SPI register images ---- */
typedef struct {
    uint8_t changed[SD_REGIMAGE / 8];   /* bit per byte that differs from the live image */
    uint16_t n_changed;
    uint8_t lo, hi;                      /* first and last changed byte (n_changed > 0) */
    bool    reset;                       /* load the whole image (power on / reset) */
} sd_img_result_t;

/* The power-on image: fill, every entry, then the data entries once more against
   that image (reg[] sees the power-on configuration).  res->reset is set. */
int sd_regs_power_on(sd_inst_t *in, uint8_t img[SD_REGIMAGE], sd_img_result_t *res);
/* Rebuild from the live image (what the DUT wrote): img = live, then the dynamic
   and writable:false entries are rewritten.  For a param change, a stimulus step
   or a time tick.  img may equal live. */
int sd_regs_rebuild(sd_inst_t *in, const uint8_t live[SD_REGIMAGE], uint8_t img[SD_REGIMAGE],
                    sd_img_result_t *res);
/* After a DUT write (live already holds it): on_write rules, then a rebuild.
   addr/value: the register and byte written, or -1 when the gateware only
   reported that a write happened (then `written`/`value` read -1). */
int sd_regs_after_write(sd_inst_t *in, const uint8_t live[SD_REGIMAGE], int addr, int value,
                        uint8_t img[SD_REGIMAGE], sd_img_result_t *res);

typedef struct { uint8_t at; uint32_t busy_us; uint16_t addr; } sd_command_t;   /* addr: the command */
/* Command map: the image bytes of one address range (an entry's address and what follows). */
typedef struct { uint16_t addr; uint8_t at, span; } sd_map_block_t;

typedef struct {
    /* i2c */
    uint8_t  address, n_alt, alt[3];
    uint8_t  addr_bits;          /* 8 | 16 */
    uint16_t page;
    bool     auto_increment;
    uint8_t  wrap;
    /* spi */
    uint8_t  spi_mode, read_bit, read_val, inc_bit, inc_val, address_mask, dummy_bytes;
    bool     has_inc_bit;
    /* conversion (both) */
    bool     has_conversion;
    uint8_t  conv_trigger, conv_busy_at, conv_busy_mask, while_busy;   /* 0 flag 1 nack 2 stretch */
    uint32_t conv_us;
    /* data-ready pin */
    bool     has_drdy;
    uint8_t  drdy_active_high, drdy_level, drdy_on;  /* on: 0 conversion 1 sample 2 rate */
    uint32_t drdy_pulse_us;
    int16_t  drdy_clear_on_read;                    /* -1 none */
    /* fifo */
    bool     has_fifo;
    uint8_t  fifo_at, fifo_frame_len, fifo_drop_newest;
    uint16_t fifo_depth;
    uint8_t  n_commands;
    /* i2c pointer mode */
    uint8_t  register_bytes;     /* 1 bytes, 2 word registers (pointer p = image bytes 2p, 2p+1) */
    bool     command_map;        /* 16-bit commands in any high byte (no page) */
    uint8_t  n_map_blocks;       /* command map: sd_map_block() 0 .. n - 1 */
} sd_bus_info_t;

int sd_bus_info(const sd_inst_t *in, sd_bus_info_t *out);
/* The bytes of writable:false entries (a bit per byte): the gateware stores no DUT write there. */
int sd_regs_ro_map(const sd_inst_t *in, uint8_t ro[SD_REGIMAGE / 8]);
/* The bytes rebuilt on every change (a bit per byte): those whose owning entry, a register or a
   command's answer, reads a param, reg[], t, a state, ... (or says reload: always). */
int sd_regs_dyn_map(const sd_inst_t *in, uint8_t dyn[SD_REGIMAGE / 8]);
int sd_command(const sd_inst_t *in, unsigned i, sd_command_t *out);
int sd_map_block(const sd_inst_t *in, unsigned i, sd_map_block_t *out);
/* Data-ready rate (Hz, 0 = none) and enable, from the live image. */
int sd_drdy_eval(sd_inst_t *in, const uint8_t live[SD_REGIMAGE], double *rate_hz, bool *enabled);
/* FIFO: rate, enable and one frame (fifo_frame_len bytes) for the current values. */
int sd_fifo_eval(sd_inst_t *in, const uint8_t live[SD_REGIMAGE], double *rate_hz, bool *enabled);
int sd_fifo_frame(sd_inst_t *in, const uint8_t live[SD_REGIMAGE], uint8_t *out, size_t cap,
                  size_t *len);
void sd_set_fifo_level(sd_inst_t *in, unsigned frames);

/* ---- UART ---- */
typedef struct { uint32_t baud; uint8_t data_bits, parity /* 'N','E','O' */, stop_bits; } sd_uart_info_t;
int sd_uart_info(const sd_inst_t *in, sd_uart_info_t *out);
/* When the next epoch is due (ms since start), -1 when no message is defined. */
int64_t sd_uart_next_ms(const sd_inst_t *in);
/* Build the epoch due at sd_uart_next_ms(): every message due then, in file
   order (t = that time), and schedule the next ones.  *at_ms gets its time. */
int sd_uart_epoch(sd_inst_t *in, char *out, size_t cap, size_t *len, int64_t *at_ms);
/* Feed one byte the DUT sent at now_ms.  Returns 1 with the answer in out
   (*len bytes, to be sent after *delay_ms), 0 when nothing matched yet, < 0 on error. */
int sd_uart_rx(sd_inst_t *in, int64_t now_ms, uint8_t byte, uint8_t *out, size_t cap,
               size_t *len, uint32_t *delay_ms);

/* ---- analog ---- */
typedef struct { bool current; bool excitation; double min, max; } sd_analog_info_t;
int sd_analog_info(const sd_inst_t *in, sd_analog_info_t *out);
/* The output (V, or mA for current) for the current params; `input` is the voltage
   the pod measures (excitation: adc), else ignored.  Clamped to the range. */
int sd_analog_eval(sd_inst_t *in, double input, double *out);

/* ---- stimulus (format.md sections 7, 8) ---- */
#define SD_STIM_MAX        8192u    /* bytes of compiled stimulus */
#define SD_STIM_SIGNALS    16u
#define SD_STIM_FAULTS     32u
#define SD_STIM_SEGMENTS   256u     /* segments of all signals, repeats included */
#define SD_STIM_DEPTH      4u       /* nested repeats */

int sd_stim_compile(const char *json, size_t json_len, void *prog, size_t prog_cap,
                    size_t *prog_len, sd_err_t *err);

typedef struct {
    const uint8_t *prog;
    double   t_last;
    uint32_t active;          /* bit per fault: on at the last step */
    bool     stepped;
} sd_stim_t;

typedef enum { SD_EV_VALUE = 0, SD_EV_FAULT_ON, SD_EV_FAULT_OFF } sd_stim_ev_kind_t;
typedef struct { sd_stim_ev_kind_t kind; uint8_t index; double value; } sd_stim_event_t;

int      sd_stim_init(sd_stim_t *s, const void *prog, size_t prog_len);
unsigned sd_stim_n_signals(const sd_stim_t *s);
/* Signal i's name; *inst_len > 0 when it has an instance prefix ("env.temperature_c":
   inst "env", param "temperature_c"). */
int      sd_stim_signal(const sd_stim_t *s, unsigned i, const char **inst, size_t *inst_len,
                        const char **param, size_t *param_len);
unsigned sd_stim_n_faults(const sd_stim_t *s);
int      sd_stim_fault(const sd_stim_t *s, unsigned i, sd_fault_t *out);
double   sd_stim_rate_hz(const sd_stim_t *s);
double   sd_stim_duration_s(const sd_stim_t *s);     /* one pass */
bool     sd_stim_repeats(const sd_stim_t *s);
/* Signal i at time t (seconds since the stimulus started). */
double   sd_stim_value(const sd_stim_t *s, unsigned i, double t);
/* Step to time t: one SD_EV_VALUE per signal, then the faults that switched on
   or off since the previous step.  Returns the number of events (<= max). */
int      sd_stim_step(sd_stim_t *s, double t, sd_stim_event_t *ev, unsigned max);

/* A single fault as JSON ({"type":"stuck","param":...}), for faults applied through the API. */
int sd_fault_from_json(const char *json, size_t len, sd_fault_t *out, sd_err_t *err);
