/* ============================================================================
 * sensor_def_uart.c — UART sensors: periodic messages and request answers.
 *
 * Messages: each has its own schedule (every_ms, truncated to whole
 * milliseconds, at least 1); messages due at the same moment form one epoch
 * and go out together in file order, with t = that moment.  States and lets
 * are computed once per epoch.  `repeat: n` sends a message n times with
 * i = 0 .. n-1; `count` is how many times it was sent before.
 *
 * Requests: the bytes the DUT sends are matched as they arrive.  A byte request
 * matches the last bytes received ("*" = any byte); a line request matches a
 * whole line without its CR/LF.  A gap of more than 100 ms starts over.
 * ========================================================================== */
#include "sensor_def_int.h"

#include <math.h>

#define RX_GAP_MS 100

static bool is_uart(const sd_inst_t *in) { return in && in->prog && sd_h(in)->kind == SD_KIND_UART; }

int sd_uart_info(const sd_inst_t *in, sd_uart_info_t *out) {
    if (!is_uart(in) || !out) return SD_E_ARG;
    const sdp_hdr_t *h = sd_h(in);
    out->baud = h->baud;
    out->data_bits = h->data_bits;
    out->parity = h->parity;
    out->stop_bits = h->stop_bits;
    return SD_OK;
}

int64_t sd_uart_next_ms(const sd_inst_t *in) {
    if (!is_uart(in) || !sd_h(in)->n_msgs) return -1;
    int64_t t = INT64_MAX;
    for (unsigned i = 0; i < sd_h(in)->n_msgs; i++) if (in->msg[i].next_ms < t) t = in->msg[i].next_ms;
    return t;
}

static const char k_hex[] = "0123456789ABCDEF";

/* Append the end of a message that started at out[start]. */
static void put_end(sd_inst_t *in, size_t start, uint8_t end) {
    char tail[5];
    size_t n = 0;
    if (end == SDEND_NMEA) {
        uint8_t cs = 0;
        size_t from = start;
        if (from < in->out_len && (in->out[from] == '$' || in->out[from] == '!')) from++;
        for (size_t i = from; i < in->out_len; i++) cs ^= (uint8_t)in->out[i];
        tail[n++] = '*'; tail[n++] = k_hex[cs >> 4]; tail[n++] = k_hex[cs & 15]; tail[n++] = '\r'; tail[n++] = '\n';
    } else if (end == SDEND_CRLF) {
        tail[n++] = '\r'; tail[n++] = '\n';
    } else if (end == SDEND_LF) {
        tail[n++] = '\n';
    }
    for (size_t i = 0; i < n; i++) {
        if (in->out_len >= in->out_cap) { in->out_over = true; return; }
        in->out[in->out_len++] = tail[i];
    }
}

static int clamp_int(double v, int lo, int hi) {
    if (!(v == v)) return lo;
    v = trunc(v);
    return v < lo ? lo : (v > hi ? hi : (int)v);
}

int sd_uart_epoch(sd_inst_t *in, char *out, size_t cap, size_t *len, int64_t *at_ms) {
    if (!is_uart(in) || !out || !len) return SD_E_ARG;
    const sdp_hdr_t *h = sd_h(in);
    *len = 0;
    if (!h->n_msgs) return SD_E_ARG;
    int64_t T = sd_uart_next_ms(in);
    if (at_ms) *at_ms = T;
    in->err.code = 0;
    in->now_ms = T;
    sd_vm_begin(in);
    in->regsrc = NULL;
    int rc = SD_OK;
    if (!in->frozen || !in->started) rc = sd_prepare(in);
    if (rc) return rc;
    in->out = out;
    in->out_cap = cap;
    in->out_len = 0;
    in->out_over = false;
    const sdp_msg_t *m = (const sdp_msg_t *)(const void *)(in->prog + h->msgs);
    for (unsigned i = 0; i < h->n_msgs && !rc; i++) {
        if (in->msg[i].next_ms != T) continue;
        double p, en = 1, rep = 1;
        if ((rc = sd_vm_num(in, m[i].every, &p))) break;
        in->msg[i].next_ms = T + clamp_int(p, 1, 86400000);
        if (m[i].enabled && (rc = sd_vm_num(in, m[i].enabled, &en))) break;
        if (en == 0) continue;
        if (m[i].repeat && (rc = sd_vm_num(in, m[i].repeat, &rep))) break;
        int n = clamp_int(rep, 0, 64);
        for (int r = 0; r < n && !rc; r++) {
            in->vars[SDV_BUILTIN + SDB_I] = sdv_num(r);
            in->vars[SDV_BUILTIN + SDB_COUNT] = sdv_num((double)in->msg[i].count);
            size_t start = in->out_len;
            rc = sd_vm_eval(in, m[i].text, NULL);
            if (!rc) put_end(in, start, m[i].end);
            in->msg[i].count++;
        }
    }
    *len = in->out_len;
    in->out = NULL;
    if (rc) return rc;
    if (in->out_over) { sd_set_err(in, SD_E_SPACE, "the epoch does not fit the buffer"); return SD_E_SPACE; }
    return SD_OK;
}

static int answer(sd_inst_t *in, unsigned i, int64_t now_ms, uint8_t *out, size_t cap, size_t *len,
                  uint32_t *delay_ms) {
    const sdp_hdr_t *h = sd_h(in);
    const sdp_req_t *r = (const sdp_req_t *)(const void *)(in->prog + h->reqs) + i;
    in->err.code = 0;
    in->now_ms = now_ms;
    sd_vm_begin(in);
    in->regsrc = NULL;
    int rc = SD_OK;
    if (!in->frozen || !in->started) rc = sd_prepare(in);
    if (rc) return rc;
    in->vars[SDV_BUILTIN + SDB_COUNT] = sdv_num((double)in->req_count[i]);
    in->vars[SDV_BUILTIN + SDB_I] = sdv_num(0);
    in->out = (char *)out;
    in->out_cap = cap;
    in->out_len = 0;
    in->out_over = false;
    if (r->text) {
        rc = sd_vm_eval(in, r->answer, NULL);
        if (!rc) put_end(in, 0, r->end);
    } else {
        for (unsigned k = 0; k < r->alen && !rc; k++) {
            double v;
            rc = sd_vm_num(in, sdp_code_at(in->prog, r->answer, k), &v);
            if (rc) break;
            if (in->out_len >= cap) { in->out_over = true; break; }
            out[in->out_len++] = (uint8_t)sd_sat(v, 8, false);
        }
        if (!rc) put_end(in, 0, r->end);
    }
    *len = in->out_len;
    in->out = NULL;
    if (rc) return rc;
    if (in->out_over) { sd_set_err(in, SD_E_SPACE, "the answer does not fit the buffer"); return SD_E_SPACE; }
    in->req_count[i]++;
    *delay_ms = r->delay_ms;
    return 1;
}

int sd_uart_rx(sd_inst_t *in, int64_t now_ms, uint8_t byte, uint8_t *out, size_t cap, size_t *len,
               uint32_t *delay_ms) {
    if (!is_uart(in) || !out || !len || !delay_ms) return SD_E_ARG;
    const sdp_hdr_t *h = sd_h(in);
    *len = 0;
    *delay_ms = 0;
    if (in->rx_last_ms >= 0 && now_ms - in->rx_last_ms > RX_GAP_MS) {
        in->rx_win_len = 0;
        in->rx_line_len = 0;
        in->rx_line_over = false;
    }
    in->rx_last_ms = now_ms;
    const sdp_req_t *r = (const sdp_req_t *)(const void *)(in->prog + h->reqs);
    if (byte == '\r' || byte == '\n') {
        if (in->rx_line_len && !in->rx_line_over) {
            for (unsigned i = 0; i < h->n_reqs; i++) {
                if (!r[i].line || r[i].mlen != in->rx_line_len) continue;
                if (memcmp(in->prog + r[i].match, in->rx_line, r[i].mlen) != 0) continue;
                in->rx_line_len = 0;
                in->rx_win_len = 0;
                return answer(in, i, now_ms, out, cap, len, delay_ms);
            }
        }
        in->rx_line_len = 0;
        in->rx_line_over = false;
    } else if (in->rx_line_len < SD_MAX_MATCH) {
        in->rx_line[in->rx_line_len++] = byte;
    } else {
        in->rx_line_over = true;
    }
    if (in->rx_win_len == SD_MAX_MATCH) {
        memmove(in->rx_win, in->rx_win + 1, SD_MAX_MATCH - 1u);
        in->rx_win_len--;
    }
    in->rx_win[in->rx_win_len++] = byte;
    for (unsigned i = 0; i < h->n_reqs; i++) {
        if (r[i].line || r[i].mlen > in->rx_win_len) continue;
        const uint8_t *m = in->prog + r[i].match;
        const uint8_t *wild = m + r[i].mlen;
        const uint8_t *tail = in->rx_win + in->rx_win_len - r[i].mlen;
        bool ok = true;
        for (unsigned k = 0; k < r[i].mlen && ok; k++)
            if (!(wild[k / 8] & (1u << (k % 8))) && tail[k] != m[k]) ok = false;
        if (!ok) continue;
        in->rx_win_len = 0;
        in->rx_line_len = 0;
        return answer(in, i, now_ms, out, cap, len, delay_ms);
    }
    return 0;
}
