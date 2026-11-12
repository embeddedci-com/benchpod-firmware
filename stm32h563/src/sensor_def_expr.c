/* ============================================================================
 * sensor_def_expr.c — expressions and templates of sensor definitions:
 * lexer, precedence parser to a small AST (constants folded as it is built),
 * name resolution with dependency tracking, and bytecode emission.
 *
 * Precedence, high to low (format.md 2.2): calls, indexing; ! ~ unary -;
 * ** (right); * / %; + -; << >>; &; ^; |; < <= > >=; == !=; &&; ||; ?:.
 * Within the spec's rows the C order holds (& before ^ before |, relational
 * before equality, && before ||).  Nesting is bounded (SD_CC_MAXDEPTH), so a
 * hostile expression cannot exhaust the C stack.
 * ========================================================================== */
#include "sensor_def_cc.h"

#include <math.h>

/* ---- lexer ---- */
enum { TK_END = 0, TK_NUM, TK_STR, TK_ID, TK_OP, TK_BAD };
enum { P_POW = 256, P_SHL, P_SHR, P_LE, P_GE, P_EQ, P_NE, P_AND, P_OR };

typedef struct {
    int      k;
    uint32_t at, len;    /* in c->src */
    double   num;
    int      op;
} tok_t;

typedef struct {
    sd_cc_t *c;
    uint32_t pos, end;   /* in c->src */
    tok_t    t;          /* current token */
    bool     have;
    int      nest;
} ps_t;

static bool idstart(char ch) { return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '_'; }
static bool idchar(char ch) { return idstart(ch) || (ch >= '0' && ch <= '9'); }

static void perr(ps_t *ps, uint32_t at, const char *fmt, const char *arg) {
    cc_fail(ps->c, SD_E_EXPR, ps->c->json_off, (int32_t)(ps->c->src_base + at), fmt, arg);
}

static void lex(ps_t *ps) {
    const char *s = ps->c->src;
    uint32_t i = ps->pos;
    while (i < ps->end && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) i++;
    tok_t t = { TK_END, i, 0, 0, 0 };
    if (i >= ps->end) { ps->t = t; ps->pos = i; ps->have = true; return; }
    char ch = s[i];
    if ((ch >= '0' && ch <= '9') || (ch == '.' && i + 1 < ps->end && s[i + 1] >= '0' && s[i + 1] <= '9')) {
        size_t used = 0;
        t.num = sd_parse_number(s + i, ps->end - i, &used);
        t.k = used ? TK_NUM : TK_BAD;
        t.len = used ? (uint32_t)used : 1;
        if (used && i + used < ps->end && idchar(s[i + used])) t.k = TK_BAD;   /* 12abc */
    } else if (idstart(ch)) {
        uint32_t j = i;
        while (j < ps->end && idchar(s[j])) j++;
        t.k = TK_ID; t.len = j - i;
    } else if (ch == '\'') {
        uint32_t j = i + 1;
        while (j < ps->end && s[j] != '\'') j++;
        if (j >= ps->end) { t.k = TK_BAD; t.len = 1; }
        else { t.k = TK_STR; t.len = j + 1 - i; }
    } else {
        static const struct { const char *s; int op; } two[] = {
            { "**", P_POW }, { "<<", P_SHL }, { ">>", P_SHR }, { "<=", P_LE }, { ">=", P_GE },
            { "==", P_EQ }, { "!=", P_NE }, { "&&", P_AND }, { "||", P_OR } };
        t.k = TK_OP; t.len = 1; t.op = (unsigned char)ch;
        for (unsigned k = 0; k < sizeof(two) / sizeof(two[0]); k++)
            if (i + 1 < ps->end && s[i] == two[k].s[0] && s[i + 1] == two[k].s[1]) { t.op = two[k].op; t.len = 2; }
        if (t.len == 1 && !strchr("()[],?:!~+-*/%&^|<>", ch)) t.k = TK_BAD;
    }
    ps->t = t;
    ps->pos = i + t.len;
    ps->have = true;
}

static tok_t *peek(ps_t *ps) { if (!ps->have) lex(ps); return &ps->t; }
static void next(ps_t *ps) { ps->have = false; }
static bool is_op(ps_t *ps, int op) { tok_t *t = peek(ps); return t->k == TK_OP && t->op == op; }
static bool expect(ps_t *ps, int op, const char *what) {
    if (is_op(ps, op)) { next(ps); return true; }
    perr(ps, peek(ps)->at, "expected %s", what);
    return false;
}

/* What the token after the current one is (one-token lookahead past the current). */
static tok_t peek2(ps_t *ps) {
    ps_t save = *ps;
    peek(&save);
    next(&save);
    return *peek(&save);
}

/* ---- AST ---- */
static uint16_t node(ps_t *ps, int kind, uint32_t src) {
    sd_cc_t *c = ps->c;
    if (c->nnodes >= SD_CC_NODES) { perr(ps, src, "expression too large", NULL); return 0; }
    uint16_t n = c->nnodes++;
    memset(&c->nodes[n], 0, sizeof(sd_node_t));
    c->nodes[n].kind = (uint8_t)kind;
    c->nodes[n].src = (uint16_t)src;
    return n;
}
#define ND(i) (ps->c->nodes[(i)])

static bool add_kid(ps_t *ps, uint16_t parent, uint16_t kid) {
    sd_node_t *p = &ND(parent);
    if (!kid) return false;
    if (!p->kid) p->kid = kid;
    else { uint16_t k = p->kid; while (ND(k).next) k = ND(k).next; ND(k).next = kid; }
    if (ND(kid).depth + 1 > p->depth) p->depth = (uint8_t)(ND(kid).depth + 1);
    if (p->depth > SD_CC_MAXDEPTH) { perr(ps, p->src, "expression nested too deep", NULL); return false; }
    return true;
}

static uint16_t num_node(ps_t *ps, double v, uint32_t src) {
    uint16_t n = node(ps, N_NUM, src);
    if (n) ND(n).u.num = v;
    return n;
}
static bool is_num(ps_t *ps, uint16_t n) { return ND(n).kind == N_NUM; }

/* ---- folding ---- */
static int binop_of(int tok) {
    switch (tok) {
    case '+': return OP_ADD; case '-': return OP_SUB; case '*': return OP_MUL; case '/': return OP_DIV;
    case '%': return OP_MOD; case P_POW: return OP_POW; case P_SHL: return OP_SHL; case P_SHR: return OP_SHR;
    case '&': return OP_BAND; case '^': return OP_BXOR; case '|': return OP_BOR; case '<': return OP_LT;
    case P_LE: return OP_LE; case '>': return OP_GT; case P_GE: return OP_GE; case P_EQ: return OP_EQ;
    case P_NE: return OP_NE;
    default: return -1;
    }
}

static uint16_t mk_bin(ps_t *ps, int op, uint16_t a, uint16_t b, uint32_t src) {
    if (!a || !b) return 0;
    bool streq = op == OP_EQ || op == OP_NE;
    if (!streq && (ND(a).type == TY_STR || ND(b).type == TY_STR)) {
        perr(ps, src, "a string in arithmetic", NULL);
        return 0;
    }
    if (is_num(ps, a) && is_num(ps, b)) {
        ND(a).u.num = sd_op_binary(op, ND(a).u.num, ND(b).u.num);
        return a;
    }
    uint16_t n = node(ps, N_BIN, src);
    if (!n) return 0;
    ND(n).op = (uint8_t)op;
    ND(n).type = TY_NUM;
    if (!add_kid(ps, n, a) || !add_kid(ps, n, b)) return 0;
    return n;
}

/* ---- name resolution ---- */
static bool name_eq(const sd_cc_t *c, uint16_t json_name, uint8_t len, const char *s, size_t n) {
    return len == n && memcmp(c->j.s + json_name, s, n) == 0;
}

int cc_sym_find(const sd_cc_t *c, const char *name, size_t len) {
    for (uint16_t i = 0; i < c->nsyms; i++)
        if (name_eq(c, c->syms[i].name, c->syms[i].len, name, len)) return i;
    return -1;
}

static const struct { const char *name; uint8_t slot; uint8_t deps; } k_vars[] = {
    { "t", SDB_T, DEP_TIME }, { "utc", SDB_UTC, DEP_TIME | DEP_PARAM }, { "dt", SDB_DT, DEP_TIME },
    { "written", SDB_WRITTEN, DEP_WRITE }, { "value", SDB_VALUE, DEP_WRITE },
    { "count", SDB_COUNT, DEP_COUNT }, { "fifo_level", SDB_FIFO, DEP_FIFO }, { "i", SDB_I, DEP_COUNT },
    { "input", SDB_INPUT, DEP_INPUT },
};

/* A printable copy of a name for messages. */
static const char *nm(ps_t *ps, uint32_t at, uint32_t len, char *buf, size_t cap) {
    size_t n = len < cap - 1 ? len : cap - 1;
    memcpy(buf, ps->c->src + at, n);
    buf[n] = '\0';
    return buf;
}

static uint16_t parse_ternary(ps_t *ps);
static uint16_t parse_call(ps_t *ps, const tok_t *id, bool holes_ok, int *hole_pos);

static uint16_t resolve(ps_t *ps, const tok_t *t) {
    sd_cc_t *c = ps->c;
    const char *s = c->src + t->at;
    char buf[40];
    /* locals: function arguments and lets */
    for (int k = (int)c->nlocals - 1; k >= 0; k--)
        if (name_eq(c, c->locals[k].name, c->locals[k].len, s, t->len)) {
            uint16_t n = node(ps, N_LOCAL, t->at);
            if (n) { ND(n).aux = (uint16_t)k; ND(n).type = TY_ANY; }
            return n;
        }
    int si = cc_sym_find(c, s, t->len);
    if (si >= 0) {
        const sd_sym_t *y = &c->syms[si];
        if (c->ctx & CTX_NO_TIME && y->kind != SYM_CONST) {
            perr(ps, t->at, "a constant can only use numbers and earlier constants ('%s')", nm(ps, t->at, t->len, buf, sizeof(buf)));
            return 0;
        }
        switch (y->kind) {
        case SYM_CONST: {
            if (c->ctx & CTX_NO_TIME && y->index >= c->const_limit) {
                perr(ps, t->at, "constant '%s' is defined later", nm(ps, t->at, t->len, buf, sizeof(buf)));
                return 0;
            }
            if (sdv_is_str(y->cval)) {
                uint16_t n = node(ps, N_CSTR, t->at);
                if (n) { ND(n).u.cval = y->cval; ND(n).type = TY_STR; }
                return n;
            }
            return num_node(ps, sdv_d(y->cval), t->at);
        }
        case SYM_PARAM: {
            uint16_t n = node(ps, N_VAR, t->at);
            if (n) { ND(n).aux = (uint16_t)(SDV_PARAM + y->index); ND(n).type = TY_NUM; }
            c->deps |= DEP_PARAM;
            return n;
        }
        case SYM_STATE: {
            uint16_t n = node(ps, N_VAR, t->at);
            if (n) { ND(n).aux = (uint16_t)(SDV_STATE + y->index); ND(n).type = TY_NUM; }
            c->deps |= DEP_STATE | DEP_TIME;
            return n;
        }
        case SYM_LET: {
            if (c->ctx & CTX_NO_LETS) {
                perr(ps, t->at, "state expressions cannot use lets ('%s')", nm(ps, t->at, t->len, buf, sizeof(buf)));
                return 0;
            }
            if ((c->ctx & CTX_LETS_BELOW) && y->index >= c->let_limit) {
                perr(ps, t->at, "let '%s' is used before it is set", nm(ps, t->at, t->len, buf, sizeof(buf)));
                return 0;
            }
            uint16_t n = node(ps, N_VAR, t->at);
            if (n) { ND(n).aux = (uint16_t)(SDV_LET + y->index); ND(n).type = TY_ANY; }
            /* function bodies compile before the lets: assume the let is dynamic */
            c->deps |= (c->ctx & CTX_FUNC) ? (uint8_t)(DEP_PARAM | DEP_REG | DEP_TIME) : y->deps;
            if ((uint8_t)(y->index + 1) > c->let_hi) c->let_hi = (uint8_t)(y->index + 1);
            return n;
        }
        default:
            perr(ps, t->at, "'%s' is a function: call it", nm(ps, t->at, t->len, buf, sizeof(buf)));
            return 0;
        }
    }
    for (unsigned k = 0; k < sizeof(k_vars) / sizeof(k_vars[0]); k++)
        if (strlen(k_vars[k].name) == t->len && memcmp(k_vars[k].name, s, t->len) == 0) {
            if (c->ctx & CTX_NO_TIME) {
                perr(ps, t->at, "a constant cannot use '%s'", k_vars[k].name);
                return 0;
            }
            if (k_vars[k].slot == SDB_INPUT && !(c->ctx & CTX_ANALOG)) {
                perr(ps, t->at, "'input' needs kind analog with excitation: adc", NULL);
                return 0;
            }
            uint16_t n = node(ps, N_VAR, t->at);
            if (n) { ND(n).aux = (uint16_t)(SDV_BUILTIN + k_vars[k].slot); ND(n).type = TY_NUM; }
            c->deps |= k_vars[k].deps;
            return n;
        }
    perr(ps, t->at, "unknown name '%s'", nm(ps, t->at, t->len, buf, sizeof(buf)));
    return 0;
}

/* ---- parser ---- */
static uint16_t parse_primary(ps_t *ps) {
    sd_cc_t *c = ps->c;
    tok_t t = *peek(ps);
    char buf[40];
    if (c->failed) return 0;
    switch (t.k) {
    case TK_NUM: next(ps); return num_node(ps, t.num, t.at);
    case TK_STR: {
        next(ps);
        if (t.len - 2 > 255) { perr(ps, t.at, "string longer than 255", NULL); return 0; }
        uint16_t n = node(ps, N_STR, t.at);
        if (n) { ND(n).u.soff = t.at + 1; ND(n).aux = (uint16_t)(t.len - 2); ND(n).type = TY_STR; }
        return n;
    }
    case TK_ID: {
        next(ps);
        const char *s = c->src + t.at;
        if (is_op(ps, '(')) return parse_call(ps, &t, false, NULL);
        if (t.len == 1 && s[0] == '_') { perr(ps, t.at, "'_' only stands for the solved argument in solve()", NULL); return 0; }
        static const struct { const char *n; uint8_t op; } regs[] = {
            { "reg", OP_REG }, { "reg16be", OP_REG16BE }, { "reg16le", OP_REG16LE } };
        for (unsigned k = 0; k < 3; k++)
            if (strlen(regs[k].n) == t.len && memcmp(regs[k].n, s, t.len) == 0) {
                if (!is_op(ps, '[')) { perr(ps, t.at, "expected '[' after %s", regs[k].n); return 0; }
                if (c->ctx & CTX_NO_TIME) { perr(ps, t.at, "a constant cannot read registers", NULL); return 0; }
                next(ps);
                uint16_t a = parse_ternary(ps);
                if (!a || !expect(ps, ']', "']'")) return 0;
                uint16_t n = node(ps, N_REG, t.at);
                if (!n || !add_kid(ps, n, a)) return 0;
                ND(n).op = regs[k].op;
                c->deps |= DEP_REG;
                return n;
            }
        return resolve(ps, &t);
    }
    case TK_OP:
        if (t.op == '(') {
            next(ps);
            if (++ps->nest > (int)SD_CC_MAXNEST) { perr(ps, t.at, "expression nested too deep", NULL); return 0; }
            uint16_t e = parse_ternary(ps);
            ps->nest--;
            if (!e || !expect(ps, ')', "')'")) return 0;
            return e;
        }
        if (t.op == '[') {
            next(ps);
            if (++ps->nest > (int)SD_CC_MAXNEST) { perr(ps, t.at, "expression nested too deep", NULL); return 0; }
            uint16_t n = node(ps, N_TABLE, t.at);
            if (!n) return 0;
            unsigned cnt = 0;
            bool allnum = true;
            if (!is_op(ps, ']')) {
                for (;;) {
                    uint16_t e = parse_ternary(ps);
                    if (!e || !add_kid(ps, n, e)) return 0;
                    if (!is_num(ps, e)) allnum = false;
                    if (++cnt > 255) { perr(ps, t.at, "a table has at most 255 entries", NULL); return 0; }
                    if (is_op(ps, ',')) { next(ps); continue; }
                    break;
                }
            }
            if (!expect(ps, ']', "']' after the table")) return 0;
            if (cnt == 0) { perr(ps, t.at, "empty table", NULL); return 0; }
            if (!expect(ps, '[', "'[index]' after a table")) return 0;
            uint16_t ix = parse_ternary(ps);
            if (!ix || !expect(ps, ']', "']'")) return 0;
            ps->nest--;
            if (ND(ix).type == TY_STR) { perr(ps, t.at, "a string as a table index", NULL); return 0; }
            if (allnum && is_num(ps, ix)) {   /* fold */
                int64_t i = sd_to_i64(ND(ix).u.num);
                uint16_t k = ND(n).kid;
                for (int64_t q = 0; q < i && k; q++) k = ND(k).next;
                return num_node(ps, (i >= 0 && k) ? ND(k).u.num : 0.0, t.at);
            }
            if (!allnum && cnt > 32) { perr(ps, t.at, "a table of expressions has at most 32 entries", NULL); return 0; }
            if (!add_kid(ps, n, ix)) return 0;
            ND(n).aux = (uint16_t)cnt;
            ND(n).op = allnum ? 1 : 0;
            ND(n).type = TY_ANY;
            return n;
        }
        break;
    case TK_END: perr(ps, t.at, "unexpected end of the expression", NULL); return 0;
    default: break;
    }
    perr(ps, t.at, "unexpected '%s'", nm(ps, t.at, t.len ? t.len : 1, buf, sizeof(buf)));
    return 0;
}

static uint16_t parse_unary(ps_t *ps);

static uint16_t parse_pow(ps_t *ps) {
    uint16_t a = parse_unary(ps);
    if (!a) return 0;
    if (is_op(ps, P_POW)) {
        uint32_t at = peek(ps)->at;
        next(ps);
        if (++ps->nest > (int)SD_CC_MAXNEST) { perr(ps, at, "expression nested too deep", NULL); return 0; }
        uint16_t b = parse_pow(ps);   /* right associative */
        ps->nest--;
        return mk_bin(ps, OP_POW, a, b, at);
    }
    return a;
}

static uint16_t parse_unary(ps_t *ps) {
    tok_t t = *peek(ps);
    if (t.k == TK_OP && (t.op == '!' || t.op == '~' || t.op == '-' || t.op == '+')) {
        next(ps);
        if (++ps->nest > (int)SD_CC_MAXNEST) { perr(ps, t.at, "expression nested too deep", NULL); return 0; }
        uint16_t a = parse_unary(ps);
        ps->nest--;
        if (!a) return 0;
        if (t.op == '+') {
            if (ND(a).type == TY_STR) { perr(ps, t.at, "a string in arithmetic", NULL); return 0; }
            return a;
        }
        int op = t.op == '!' ? OP_NOT : (t.op == '~' ? OP_BNOT : OP_NEG);
        if (ND(a).type == TY_STR) { perr(ps, t.at, "a string in arithmetic", NULL); return 0; }
        if (is_num(ps, a)) { ND(a).u.num = sd_op_unary(op, ND(a).u.num); return a; }
        uint16_t n = node(ps, N_UN, t.at);
        if (!n || !add_kid(ps, n, a)) return 0;
        ND(n).op = (uint8_t)op;
        ND(n).type = TY_NUM;
        return n;
    }
    return parse_primary(ps);
}

/* Binary operators by precedence (higher binds tighter): precedence climbing, so the C stack
   grows with the operators actually nested, not with the number of levels. */
static int prec_of(const tok_t *t) {
    if (t->k != TK_OP) return -1;
    switch (t->op) {
    case P_OR: return 1;
    case P_AND: return 2;
    case P_EQ: case P_NE: return 3;
    case '<': case P_LE: case '>': case P_GE: return 4;
    case '|': return 5;
    case '^': return 6;
    case '&': return 7;
    case P_SHL: case P_SHR: return 8;
    case '+': case '-': return 9;
    case '*': case '/': case '%': return 10;
    default: return -1;
    }
}

static SD_NOINLINE uint16_t mk_logic(ps_t *ps, bool is_or, uint16_t a, uint16_t b, uint32_t at) {
    if (!a || !b) return 0;
    if (ND(a).type == TY_STR || ND(b).type == TY_STR) { perr(ps, at, "a string as a condition", NULL); return 0; }
    if (is_num(ps, a)) {
        bool ta = ND(a).u.num != 0;
        if (is_or ? ta : !ta) { ND(a).u.num = is_or ? 1 : 0; return a; }
        if (is_num(ps, b)) { ND(b).u.num = ND(b).u.num != 0; return b; }
    }
    uint16_t n = node(ps, is_or ? N_OR : N_AND, at);
    if (!n || !add_kid(ps, n, a) || !add_kid(ps, n, b)) return 0;
    ND(n).type = TY_NUM;
    return n;
}

static uint16_t parse_bin(ps_t *ps, int min_prec) {
    uint16_t a = parse_pow(ps);
    for (;;) {
        if (!a) return 0;
        tok_t t = *peek(ps);
        int p = prec_of(&t);
        if (p < min_prec || p < 0) return a;
        next(ps);
        if (++ps->nest > (int)SD_CC_MAXNEST) { perr(ps, t.at, "expression nested too deep", NULL); return 0; }
        uint16_t b = parse_bin(ps, p + 1);   /* left associative */
        ps->nest--;
        if (t.op == P_OR || t.op == P_AND) a = mk_logic(ps, t.op == P_OR, a, b, t.at);
        else a = mk_bin(ps, binop_of(t.op), a, b, t.at);
    }
}

static uint16_t parse_ternary(ps_t *ps) {
    uint16_t c0 = parse_bin(ps, 1);
    if (!c0 || !is_op(ps, '?')) return c0;
    uint32_t at = peek(ps)->at;
    next(ps);
    if (++ps->nest > (int)SD_CC_MAXNEST) { perr(ps, at, "expression nested too deep", NULL); return 0; }
    uint16_t a = parse_ternary(ps);
    if (!a || !expect(ps, ':', "':' of '?:'")) return 0;
    uint16_t b = parse_ternary(ps);
    ps->nest--;
    if (!b) return 0;
    if (ND(c0).type == TY_STR) { perr(ps, at, "a string as a condition", NULL); return 0; }
    if (is_num(ps, c0)) return ND(c0).u.num != 0 ? a : b;
    uint16_t n = node(ps, N_COND, at);
    if (!n || !add_kid(ps, n, c0) || !add_kid(ps, n, a) || !add_kid(ps, n, b)) return 0;
    ND(n).type = ND(a).type == ND(b).type ? ND(a).type : TY_ANY;
    return n;
}

/* A built-in call with only numbers: its value (kept out of parse_call, which recurses). */
static SD_NOINLINE uint16_t fold_builtin(ps_t *ps, uint16_t n, int bi, unsigned argc, uint32_t at) {
    double a[64], r;
    unsigned q = 0;
    for (uint16_t k = ND(n).kid; k && q < 64; k = ND(k).next) a[q++] = ND(k).u.num;
    if (sd_builtin_num(bi, a, argc, &r) == SD_OK) return num_node(ps, r, at);
    return n;
}

/* A call: built-in, solve(), user function or fragment.  The '(' is next. */
static uint16_t parse_call(ps_t *ps, const tok_t *id, bool holes_ok, int *hole_pos) {
    sd_cc_t *c = ps->c;
    const char *s = c->src + id->at;
    char buf[40];
    next(ps);   /* '(' */
    if (++ps->nest > (int)SD_CC_MAXNEST) { perr(ps, id->at, "expression nested too deep", NULL); return 0; }
    bool is_solve = id->len == 5 && memcmp(s, "solve", 5) == 0;
    int bi = is_solve ? -1 : sd_builtin_find(s, id->len);
    int si = -1;
    const sd_sym_t *y = NULL;
    if (!is_solve && bi < 0) {
        si = cc_sym_find(c, s, id->len);
        if (si < 0 || (c->syms[si].kind != SYM_FUNC && c->syms[si].kind != SYM_FRAG)) {
            perr(ps, id->at, si < 0 ? "unknown function '%s'" : "'%s' is not a function",
                 nm(ps, id->at, id->len, buf, sizeof(buf)));
            return 0;
        }
        y = &c->syms[si];
        if (c->ctx & CTX_NO_TIME) { perr(ps, id->at, "a constant cannot call '%s'", nm(ps, id->at, id->len, buf, sizeof(buf))); return 0; }
        if ((y->kind == SYM_FUNC && y->index >= c->func_limit) || (y->kind == SYM_FRAG && y->index >= c->frag_limit)) {
            perr(ps, id->at, "'%s' is not defined before this (no recursion)", nm(ps, id->at, id->len, buf, sizeof(buf)));
            return 0;
        }
    }
    uint16_t n = node(ps, is_solve ? N_SOLVE : (bi >= 0 ? N_BUILTIN : (y->kind == SYM_FUNC ? N_CALL : N_FRAG)), id->at);
    if (!n) return 0;
    unsigned argc = 0;
    bool allnum = true;
    if (is_solve) {
        /* solve(f, target, lo, hi) or solve(f(_, x...), target, lo, hi) */
        if (c->ctx & CTX_NO_TIME) { perr(ps, id->at, "a constant cannot use solve()", NULL); return 0; }
        tok_t f = *peek(ps);
        tok_t f2 = peek2(ps);
        int fs = f.k == TK_ID ? cc_sym_find(c, c->src + f.at, f.len) : -1;
        if (fs < 0 || c->syms[fs].kind != SYM_FUNC) {
            perr(ps, f.at, "solve() needs a function: solve(f, target, lo, hi) or solve(f(_, x), ...)", NULL);
            return 0;
        }
        const sd_sym_t *fy = &c->syms[fs];
        if (fy->index >= c->func_limit) { perr(ps, f.at, "'%s' is not defined before this (no recursion)", nm(ps, f.at, f.len, buf, sizeof(buf))); return 0; }
        int pos = -1;
        if (f2.k == TK_OP && f2.op == '(') {
            next(ps);
            peek(ps);   /* the '(' parse_call consumes */
            uint16_t call = parse_call(ps, &f, true, &pos);
            if (!call) return 0;
            if (pos < 0) { perr(ps, f.at, "solve(): mark the solved argument with '_'", NULL); return 0; }
            /* move the call's fixed arguments to the solve node */
            uint16_t k = ND(call).kid;
            while (k) { uint16_t nx = ND(k).next; ND(k).next = 0; if (!add_kid(ps, n, k)) return 0; k = nx; }
            ND(n).op = (uint8_t)pos;
        } else {
            next(ps);
            if (fy->nargs != 1) { perr(ps, f.at, "solve(%s, ...): use solve(f(_, x), ...) for a function of several arguments", nm(ps, f.at, f.len, buf, sizeof(buf))); return 0; }
            ND(n).op = 0xFF;
        }
        ND(n).aux = fy->index;
        c->deps |= fy->deps;
        if (fy->let_hi > c->let_hi) c->let_hi = fy->let_hi;
        for (int k = 0; k < 3; k++) {
            if (!expect(ps, ',', "',' in solve(f, target, lo, hi)")) return 0;
            uint16_t e = parse_ternary(ps);
            if (!e || !add_kid(ps, n, e)) return 0;
            if (ND(e).type == TY_STR) { perr(ps, ND(e).src, "a string in solve()", NULL); return 0; }
        }
        if (!expect(ps, ')', "')'")) return 0;
        ps->nest--;
        ND(n).type = TY_NUM;
        return n;
    }
    if (!is_op(ps, ')')) {
        for (;;) {
            tok_t a = *peek(ps);
            uint16_t e = 0;
            if (a.k == TK_ID && a.len == 1 && c->src[a.at] == '_') {
                tok_t a2 = peek2(ps);
                if (!holes_ok || !(a2.k == TK_OP && (a2.op == ',' || a2.op == ')'))) {
                    perr(ps, a.at, "'_' only stands for the solved argument in solve()", NULL);
                    return 0;
                }
                if (*hole_pos >= 0) { perr(ps, a.at, "solve(): only one '_'", NULL); return 0; }
                next(ps);
                *hole_pos = (int)argc;
                argc++;
                allnum = false;
            } else {
                e = parse_ternary(ps);
                if (!e || !add_kid(ps, n, e)) return 0;
                if (!is_num(ps, e)) allnum = false;
                argc++;
            }
            if (argc > 64) { perr(ps, a.at, "too many arguments", NULL); return 0; }
            if (is_op(ps, ',')) { next(ps); continue; }
            break;
        }
    }
    if (!expect(ps, ')', "')' or ','")) return 0;
    ps->nest--;
    if (bi >= 0) {
        const sd_builtin_t *b = &sd_builtins[bi];
        if (argc < b->min_args || argc > b->max_args) {
            perr(ps, id->at, "wrong number of arguments to '%s'", b->name);
            return 0;
        }
        for (uint16_t k = ND(n).kid; k; k = ND(k).next)
            if (ND(k).type == TY_STR) { perr(ps, ND(k).src, "a string as a number argument", NULL); return 0; }
        ND(n).aux = (uint16_t)bi;
        ND(n).type = b->str_result ? TY_STR : TY_NUM;
        if (allnum && !b->str_result) return fold_builtin(ps, n, bi, argc, id->at);
        return n;
    }
    if (argc != y->nargs) { perr(ps, id->at, "'%s' takes a different number of arguments", nm(ps, id->at, id->len, buf, sizeof(buf))); return 0; }
    ND(n).aux = y->index;
    ND(n).type = y->kind == SYM_FRAG ? TY_STR : TY_ANY;
    c->deps |= y->deps;
    if (y->let_hi > c->let_hi) c->let_hi = y->let_hi;
    if ((c->ctx & CTX_LETS_BELOW) && y->let_hi > c->let_limit) {
        perr(ps, id->at, "'%s' reads a let that is set after this one", nm(ps, id->at, id->len, buf, sizeof(buf)));
        return 0;
    }
    return n;
}

/* ---- emission ---- */
static bool emit_jump(sd_cc_t *c, uint8_t op, uint32_t *patch) {
    if (!cc_emit8(c, op)) return false;
    *patch = c->len;
    return cc_emit16(c, 0);
}
static bool patch_to_here(sd_cc_t *c, uint32_t patch) {
    int32_t off = (int32_t)c->len - (int32_t)(patch + 2);
    if (off > 32767) { cc_fail(c, SD_E_LIMIT, c->json_off, -1, "expression code too long", NULL); return false; }
    c->p[patch] = (uint8_t)(off & 0xFF);
    c->p[patch + 1] = (uint8_t)((off >> 8) & 0xFF);
    return true;
}

static bool emit_num(sd_cc_t *c, double v) {
    if (v == floor(v) && v >= -128 && v <= 127 && !(v == 0 && 1.0 / v < 0)) return cc_emit8(c, OP_I8) && cc_emit8(c, (uint8_t)(int8_t)v);
    if (v == floor(v) && v >= -32768 && v <= 32767 && !(v == 0 && 1.0 / v < 0)) return cc_emit8(c, OP_I16) && cc_emit16(c, (uint16_t)(int16_t)v);
    return cc_emit8(c, OP_F64) && cc_emit(c, &v, 8);
}

/* A string constant: copy its chars (already in the program) after an OP_STR. */
static SD_NOINLINE bool emit_cstr(sd_cc_t *c, uint64_t cval) {
    uint32_t off = sdv_str_off(cval), len = sdv_str_len(cval);
    char tmp[256];
    memcpy(tmp, c->p + off, len);
    return cc_emit8(c, OP_STR) && cc_emit8(c, (uint8_t)len) && cc_emit(c, tmp, len);
}

static bool emit_node(sd_cc_t *c, uint16_t n) {
    sd_node_t *d = &c->nodes[n];
    uint16_t k;
    switch (d->kind) {
    case N_NUM: return emit_num(c, d->u.num);
    case N_STR:
        return cc_emit8(c, OP_STR) && cc_emit8(c, (uint8_t)d->aux) && cc_emit(c, c->src + d->u.soff, d->aux);
    case N_CSTR: return emit_cstr(c, d->u.cval);
    case N_VAR: return cc_emit8(c, OP_VAR) && cc_emit8(c, (uint8_t)d->aux);
    case N_LOCAL: return cc_emit8(c, OP_LOCAL) && cc_emit8(c, (uint8_t)d->aux);
    case N_UN: return emit_node(c, d->kid) && cc_emit8(c, d->op);
    case N_BIN: return emit_node(c, d->kid) && emit_node(c, c->nodes[d->kid].next) && cc_emit8(c, d->op);
    case N_REG: return emit_node(c, d->kid) && cc_emit8(c, d->op);
    case N_AND: case N_OR: {
        uint32_t p;
        if (!emit_node(c, d->kid) || !emit_jump(c, d->kind == N_AND ? OP_ANDJ : OP_ORJ, &p)) return false;
        if (!emit_node(c, c->nodes[d->kid].next) || !cc_emit8(c, OP_BOOL)) return false;
        return patch_to_here(c, p);
    }
    case N_COND: {
        uint16_t a = c->nodes[d->kid].next, b = c->nodes[a].next;
        uint32_t p1, p2;
        if (!emit_node(c, d->kid) || !emit_jump(c, OP_JZ, &p1) || !emit_node(c, a) || !emit_jump(c, OP_JMP, &p2))
            return false;
        if (!patch_to_here(c, p1) || !emit_node(c, b)) return false;
        return patch_to_here(c, p2);
    }
    case N_BUILTIN: case N_CALL: case N_FRAG: {
        unsigned argc = 0;
        for (k = d->kid; k; k = c->nodes[k].next) { if (!emit_node(c, k)) return false; argc++; }
        if (d->kind == N_BUILTIN) return cc_emit8(c, OP_BUILTIN) && cc_emit8(c, (uint8_t)d->aux) && cc_emit8(c, (uint8_t)argc);
        return cc_emit8(c, d->kind == N_CALL ? OP_CALL : OP_FRAG) && cc_emit8(c, (uint8_t)d->aux);
    }
    case N_SOLVE:
        for (k = d->kid; k; k = c->nodes[k].next) if (!emit_node(c, k)) return false;
        return cc_emit8(c, OP_SOLVE) && cc_emit8(c, (uint8_t)d->aux) && cc_emit8(c, d->op);
    case N_TABLE: {
        unsigned cnt = d->aux;
        uint16_t ix = d->kid;
        for (unsigned q = 0; q < cnt; q++) ix = c->nodes[ix].next;
        if (d->op) {   /* constant table */
            if (!emit_node(c, ix) || !cc_emit8(c, OP_CTAB) || !cc_emit8(c, (uint8_t)cnt)) return false;
            k = d->kid;
            for (unsigned q = 0; q < cnt; q++) { if (!cc_emit(c, &c->nodes[k].u.num, 8)) return false; k = c->nodes[k].next; }
            return true;
        }
        k = d->kid;
        for (unsigned q = 0; q < cnt; q++) { if (!emit_node(c, k)) return false; k = c->nodes[k].next; }
        return emit_node(c, ix) && cc_emit8(c, OP_TAB) && cc_emit8(c, (uint8_t)cnt);
    }
    default:
        cc_fail(c, SD_E_EXPR, c->json_off, -1, "internal: bad node", NULL);
        return false;
    }
}

/* Parse + emit the expression text c->src[a, b) (no OP_RET).  Returns the root type, -1 on failure. */
static int compile_span(sd_cc_t *c, uint32_t a, uint32_t b, bool emit, uint16_t *root_out) {
    ps_t ps = { c, a, b, { 0 }, false, 0 };
    c->nnodes = 1;   /* node 0 = none */
    uint16_t root = parse_ternary(&ps);
    if (!root || c->failed) { if (!c->failed) perr(&ps, a, "bad expression", NULL); return -1; }
    if (peek(&ps)->k != TK_END) { perr(&ps, peek(&ps)->at, "unexpected text after the expression", NULL); return -1; }
    if (root_out) *root_out = root;
    if (emit && !emit_node(c, root)) return -1;
    return c->nodes[root].type;
}

/* Load the expression text of a JSON value into c->src.  Numbers and booleans become their text. */
static bool load_src(sd_cc_t *c, uint32_t json_off, bool *is_number, double *num) {
    char t = sdj_type(&c->j, json_off);
    c->json_off = sdj_ws(&c->j, json_off);
    c->src_base = 0;
    *is_number = false;
    if (t == 'n') {
        if (!sdj_num(&c->j, json_off, num)) { cc_fail(c, SD_E_SCHEMA, c->json_off, -1, "bad number", NULL); return false; }
        *is_number = true;
        return true;
    }
    if (t == 't' || t == 'f') { *num = t == 't' ? 1 : 0; *is_number = true; return true; }
    if (t != '"') { cc_fail(c, SD_E_SCHEMA, c->json_off, -1, "expected an expression (a string or a number)", NULL); return false; }
    size_t n;
    if (!sdj_str(&c->j, json_off, c->src, c->src_cap, &n) || n > SD_MAX_EXPR) {
        cc_fail(c, SD_E_LIMIT, c->json_off, -1, "expression longer than 1024 characters", NULL);
        return false;
    }
    c->src_len = (uint32_t)n;
    return true;
}

/* Emit a value expression without OP_RET.  Returns its type, -1 on failure. */
static int emit_value(sd_cc_t *c, uint32_t json_off, uint8_t ctx) {
    bool isn;
    double v;
    c->ctx = ctx;
    if (!load_src(c, json_off, &isn, &v)) return -1;
    if (isn) return emit_num(c, v) ? TY_NUM : -1;
    return compile_span(c, 0, c->src_len, true, NULL);
}

uint16_t cc_expr(sd_cc_t *c, uint32_t json_off, uint8_t ctx) {
    uint32_t start = c->len;
    if (start > 0xFFFF) { cc_fail(c, SD_E_LIMIT, json_off, -1, "program larger than 8 KB", NULL); return 0; }
    if (emit_value(c, json_off, ctx) < 0 || !cc_emit8(c, OP_RET)) return 0;
    cc_range(c, start, json_off);
    return (uint16_t)start;
}

bool cc_const_value(sd_cc_t *c, uint32_t json_off, uint64_t *out) {
    bool isn;
    double v;
    c->ctx = CTX_NO_TIME;
    if (!load_src(c, json_off, &isn, &v)) return false;
    if (isn) { *out = sdv_num(v); return true; }
    uint16_t root;
    if (compile_span(c, 0, c->src_len, false, &root) < 0) return false;
    sd_node_t *d = &c->nodes[root];
    if (d->kind == N_NUM) { *out = sdv_num(d->u.num); return true; }
    if (d->kind == N_CSTR) { *out = d->u.cval; return true; }
    if (d->kind == N_STR) {
        uint16_t rec = cc_string(c, c->src + d->u.soff, d->aux);
        if (!rec) return false;
        *out = sdv_str(SDS_TAG_PROG, (uint32_t)rec + 1u, d->aux);
        return true;
    }
    cc_fail(c, SD_E_EXPR, c->json_off, -1, "a constant must work out to a number or a string", NULL);
    return false;
}

/* ---- templates: "text {expr} more {expr:fmt}" ---- */
static bool parse_fmt(const char *s, uint32_t n, uint8_t f[4]) {
    uint32_t i = 0;
    f[0] = f[1] = f[2] = 0;
    f[3] = SDC_DEFAULT;
    while (i < n && (s[i] == '-' || s[i] == '+' || s[i] == '0')) {
        f[0] |= s[i] == '-' ? SDF_MINUS : (s[i] == '+' ? SDF_PLUS : SDF_ZERO);
        i++;
    }
    unsigned w = 0;
    while (i < n && s[i] >= '0' && s[i] <= '9') { w = w * 10u + (unsigned)(s[i] - '0'); if (w > 64) return false; i++; }
    f[1] = (uint8_t)w;
    if (i < n && s[i] == '.') {
        i++;
        unsigned p = 0;
        if (i >= n || s[i] < '0' || s[i] > '9') return false;
        while (i < n && s[i] >= '0' && s[i] <= '9') { p = p * 10u + (unsigned)(s[i] - '0'); if (p > 15) return false; i++; }
        f[0] |= SDF_PREC;
        f[2] = (uint8_t)p;
    }
    if (i + 1 != n) return false;
    switch (s[i]) {
    case 'd': f[3] = SDC_D; break;
    case 'x': f[3] = SDC_X; break;
    case 'X': f[3] = SDC_XU; break;
    case 'f': f[3] = SDC_F; break;
    default: return false;
    }
    return true;
}

static bool emit_txt(sd_cc_t *c, const char *s, uint32_t n) {
    while (n) {
        uint32_t k = n > 255 ? 255 : n;
        if (!cc_emit8(c, OP_TXT) || !cc_emit8(c, (uint8_t)k) || !cc_emit(c, s, k)) return false;
        s += k;
        n -= k;
    }
    return true;
}

static bool emit_template_body(sd_cc_t *c, uint32_t json_off, uint8_t ctx) {
    bool isn;
    double v;
    c->ctx = ctx;
    if (sdj_type(&c->j, json_off) != '"') { cc_fail(c, SD_E_SCHEMA, sdj_ws(&c->j, json_off), -1, "expected text", NULL); return false; }
    if (!load_src(c, json_off, &isn, &v)) return false;
    const char *s = c->src;
    uint32_t n = c->src_len, i = 0, lit = 0;
    char litbuf[256];
    while (i < n) {
        if (s[i] == '{' && i + 1 < n && s[i + 1] == '{') { if (lit == sizeof(litbuf)) { if (!emit_txt(c, litbuf, lit)) return false; lit = 0; } litbuf[lit++] = '{'; i += 2; continue; }
        if (s[i] == '}' && i + 1 < n && s[i + 1] == '}') { if (lit == sizeof(litbuf)) { if (!emit_txt(c, litbuf, lit)) return false; lit = 0; } litbuf[lit++] = '}'; i += 2; continue; }
        if (s[i] == '}') { cc_fail(c, SD_E_EXPR, c->json_off, (int32_t)i, "'}' without '{' (write '}}' for one)", NULL); return false; }
        if (s[i] != '{') {
            if (lit == sizeof(litbuf)) { if (!emit_txt(c, litbuf, lit)) return false; lit = 0; }
            litbuf[lit++] = s[i++];
            continue;
        }
        if (lit) { if (!emit_txt(c, litbuf, lit)) return false; lit = 0; }
        /* find the matching '}' outside quotes and brackets; note the last top-level ':' */
        uint32_t a = i + 1, j = a, colon = 0;
        int depth = 0;
        bool q = false;
        for (; j < n; j++) {
            char ch = s[j];
            if (q) { if (ch == '\'') q = false; continue; }
            if (ch == '\'') q = true;
            else if (ch == '(' || ch == '[') depth++;
            else if (ch == ')' || ch == ']') depth--;
            else if (ch == ':' && depth == 0) colon = j;
            else if (ch == '}' && depth <= 0) break;
        }
        if (j >= n) { cc_fail(c, SD_E_EXPR, c->json_off, (int32_t)i, "'{' without '}'", NULL); return false; }
        uint8_t fmt[4] = { 0, 0, 0, SDC_DEFAULT };
        uint32_t e = j;
        if (colon && colon + 1 < j && s[colon + 1] != ' ' && parse_fmt(s + colon + 1, j - colon - 1, fmt)) e = colon;
        if (compile_span(c, a, e, true, NULL) < 0) return false;
        if (!cc_emit8(c, OP_TXTV) || !cc_emit(c, fmt, 4)) return false;
        i = j + 1;
    }
    if (lit && !emit_txt(c, litbuf, lit)) return false;
    return true;
}

uint16_t cc_template(sd_cc_t *c, uint32_t json_off, uint8_t ctx) {
    uint32_t start = c->len;
    if (start > 0xFFFF) { cc_fail(c, SD_E_LIMIT, json_off, -1, "program larger than 8 KB", NULL); return 0; }
    if (!emit_template_body(c, json_off, ctx) || !cc_emit8(c, OP_RET)) return 0;
    cc_range(c, start, json_off);
    return (uint16_t)start;
}

/* ---- functions and fragments ---- */
bool cc_valid_name(const sd_cc_t *c, uint32_t off, uint32_t len) {
    if (len == 0 || len > 32 || !idstart(c->j.s[off])) return false;
    for (uint32_t i = 1; i < len; i++) if (!idchar(c->j.s[off + i])) return false;
    return true;
}

bool cc_function(sd_cc_t *c, uint32_t obj, sdp_func_t *out, bool fragment, uint8_t *let_hi, uint8_t *deps) {
    sd_local_t locals[SD_MAX_ARGS + SD_MAX_FLETS];
    uint8_t nl = 0;
    uint32_t it = obj, k, v, args = 0, lets = 0, ret = 0, text = 0;
    bool has_args = false, has_lets = false, has_ret = false, has_text = false;
    while (sdj_obj_next(&c->j, &it, &k, &v)) {
        if (sdj_key_is(&c->j, k, "args")) { args = v; has_args = true; }
        else if (!fragment && sdj_key_is(&c->j, k, "let")) { lets = v; has_lets = true; }
        else if (!fragment && sdj_key_is(&c->j, k, "return")) { ret = v; has_ret = true; }
        else if (fragment && sdj_key_is(&c->j, k, "text")) { text = v; has_text = true; }
        else if (sdj_key_is(&c->j, k, "description")) { }
        else { cc_fail(c, SD_E_SCHEMA, k, -1, "unknown key", NULL); return false; }
    }
    if (fragment ? !has_text : !has_ret) {
        cc_fail(c, SD_E_SCHEMA, obj, -1, fragment ? "a fragment needs 'text'" : "a function needs 'return'", NULL);
        return false;
    }
    if (has_args) {
        if (sdj_type(&c->j, args) != '[') { cc_fail(c, SD_E_SCHEMA, args, -1, "args: expected a list of names", NULL); return false; }
        uint32_t ai = args, av;
        while (sdj_arr_next(&c->j, &ai, &av)) {
            uint32_t len;
            bool esc;
            if (sdj_type(&c->j, av) != '"') { cc_fail(c, SD_E_SCHEMA, av, -1, "expected a name", NULL); return false; }
            const char *r = sdj_raw(&c->j, av, &len, &esc);
            uint32_t off = (uint32_t)(r - c->j.s);
            if (esc || !cc_valid_name(c, off, len)) { cc_fail(c, SD_E_NAME, av, -1, "bad argument name", NULL); return false; }
            if (nl >= SD_MAX_ARGS) { cc_fail(c, SD_E_LIMIT, av, -1, "more than 8 arguments", NULL); return false; }
            for (uint8_t q = 0; q < nl; q++)
                if (locals[q].len == len && memcmp(c->j.s + locals[q].name, r, len) == 0) {
                    cc_fail(c, SD_E_NAME, av, -1, "duplicate argument", NULL); return false;
                }
            locals[nl].name = (uint16_t)off;
            locals[nl].len = (uint8_t)len;
            nl++;
        }
    }
    out->nargs = nl;
    out->nlets = 0;
    uint32_t start = c->len;
    if (start > 0xFFFF) { cc_fail(c, SD_E_LIMIT, obj, -1, "program larger than 8 KB", NULL); return false; }
    out->code = (uint16_t)start;
    c->locals = locals;
    c->nlocals = nl;
    c->deps = 0;
    c->let_hi = 0;
    uint8_t ctx = CTX_FUNC;
    bool ok = true;
    if (fragment) {
        ok = emit_template_body(c, text, ctx) && cc_emit8(c, OP_RET);
        if (ok) cc_range(c, start, text);
    } else {
        if (has_lets) {
            if (sdj_type(&c->j, lets) != '[') { cc_fail(c, SD_E_SCHEMA, lets, -1, "let: expected a list", NULL); ok = false; }
            uint32_t li = lets, lv;
            while (ok && sdj_arr_next(&c->j, &li, &lv)) {
                uint32_t lk, lval, lit2 = lv;
                if (sdj_type(&c->j, lv) != '{' || sdj_count(&c->j, lv) != 1) {
                    cc_fail(c, SD_E_SCHEMA, lv, -1, "a let entry is one 'name: expression'", NULL); ok = false; break;
                }
                sdj_obj_next(&c->j, &lit2, &lk, &lval);
                uint32_t len;
                bool esc;
                const char *r = sdj_raw(&c->j, lk, &len, &esc);
                uint32_t off = (uint32_t)(r - c->j.s);
                if (esc || !cc_valid_name(c, off, len)) { cc_fail(c, SD_E_NAME, lk, -1, "bad name", NULL); ok = false; break; }
                if (nl >= SD_MAX_ARGS + SD_MAX_FLETS) { cc_fail(c, SD_E_LIMIT, lk, -1, "more than 16 let entries in a function", NULL); ok = false; break; }
                uint32_t code_lo = c->len;
                if (emit_value(c, lval, ctx) < 0) { ok = false; break; }
                cc_range(c, code_lo, lval);
                c->ctx = ctx;
                locals[nl].name = (uint16_t)off;
                locals[nl].len = (uint8_t)len;
                nl++;
                c->nlocals = nl;
                out->nlets++;
            }
        }
        if (ok) {
            uint32_t code_lo = c->len;
            ok = emit_value(c, ret, ctx) >= 0 && cc_emit8(c, OP_RET);
            if (ok) cc_range(c, code_lo, ret);
        }
    }
    c->locals = NULL;
    c->nlocals = 0;
    *let_hi = c->let_hi;
    *deps = c->deps;
    return ok && !c->failed;
}
