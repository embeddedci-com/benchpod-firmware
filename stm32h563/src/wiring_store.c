/* wiring_store.c: see wiring_store.h. */
#include "wiring_store.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "blob_store.h"
#include "mbedtls/sha256.h"
#include "w25q.h"
#include "bp_log.h"

static uint8_t       *s_doc;      /* heap, s_idx->doc_len bytes; NULL when empty */
/* The index lives on the heap too (about 900 bytes): RAM has no room for it in .bss. Until
   wiring_store_load allocates it, s_empty stands in. */
static const wiring_index_t s_empty = { .def = WIRING_NO_DEFAULT };   /* flash, not RAM */
static wiring_index_t *s_idx;                                         /* NULL until loaded */
static blob_id_t      s_cur = BLOB_WIRING_A;   /* the slot holding the newest copy */
static uint32_t       s_seq;      /* its sequence number (0: no copy yet) */

const wiring_index_t *wiring_store_index(void) { return s_idx ? s_idx : &s_empty; }
const uint8_t        *wiring_store_doc(void)   { return s_doc; }

/* Read a present slot's data into a fresh heap buffer and parse it. Bus held. */
static bool load_slot(blob_id_t id, uint8_t **out, wiring_index_t *idx) {
    const blob_info_t *in = blob_store_info(id);
    *out = NULL;
    if (!in || !in->present || in->len == 0 || in->len > WIRING_DOC_MAX) return false;
    if (blob_store_verify(id) != 0) {
        LOG_W("[wiring] slot %s fails its hash: ignored\n", blob_name(id));
        return false;
    }
    uint8_t *buf = pvPortMalloc(in->len);
    if (!buf) return false;
    for (uint32_t off = 0; off < in->len; off += 256u) {
        uint32_t n = in->len - off < 256u ? in->len - off : 256u;
        if (blob_store_read((void *)(uintptr_t)id, off, buf + off, n) != 0) { vPortFree(buf); return false; }
    }
    if (!wiring_doc_parse(buf, in->len, idx)) {
        LOG_W("[wiring] slot %s holds no valid profiles: ignored\n", blob_name(id));
        vPortFree(buf);
        return false;
    }
    *out = buf;
    return true;
}

void wiring_store_load(void) {
    if (!s_idx) {
        wiring_index_t *idx = pvPortMalloc(sizeof(*idx));
        if (!idx) { LOG_E("[wiring] out of memory: no wiring profiles\n"); return; }
        s_idx = idx;
    }
    (void)wiring_doc_parse(NULL, 0, s_idx);   /* empty until a copy loads */
    if (w25q_open() != 0) { LOG_W("[wiring] W25Q does not answer: no wiring profiles\n"); return; }
    const blob_id_t ids[2] = { BLOB_WIRING_A, BLOB_WIRING_B };
    int best = -1;
    uint32_t best_seq = 0;
    for (int i = 0; i < 2; i++) {
        const blob_info_t *in = blob_store_info(ids[i]);
        if (in && in->present && in->version > best_seq) { best = i; best_seq = in->version; }
    }
    /* Newest first; fall back to the other copy when the newest does not load. */
    for (int pass = 0; pass < 2 && best >= 0; pass++) {
        int i = pass == 0 ? best : 1 - best;
        uint8_t *buf;
        wiring_index_t idx;
        if (load_slot(ids[i], &buf, &idx)) {
            s_doc = buf;
            *s_idx = idx;
            s_cur = ids[i];
            s_seq = blob_store_info(ids[i])->version;
            break;
        }
    }
    w25q_close();
    LOG_I("[wiring] %u profile(s)%s%s\n", (unsigned)s_idx->count,
          s_idx->def != WIRING_NO_DEFAULT ? ", default " : "",
          s_idx->def != WIRING_NO_DEFAULT ? s_idx->e[s_idx->def].name : "");
}

static int mem_src(void *ctx, uint32_t off, uint8_t *buf, uint32_t n) {
    memcpy(buf, (const uint8_t *)ctx + off, n);
    return 0;
}

wiring_store_rc_t wiring_store_apply(const wiring_op_t *op, wiring_result_t *why) {
    if (!s_idx) return WIRING_STORE_IO;   /* never loaded (out of memory) */
    *why = wiring_doc_check(s_idx, op);
    if (*why != WIRING_OK) return WIRING_STORE_REFUSED;
    size_t need = wiring_doc_size_after(s_idx, op);
    if (need > WIRING_DOC_MAX) { *why = WIRING_E_FULL; return WIRING_STORE_REFUSED; }
    uint8_t *out = pvPortMalloc(need);
    if (!out) return WIRING_STORE_IO;
    size_t n = wiring_doc_build(s_doc, s_idx, op, out, need);
    wiring_index_t idx;
    if (n == 0 || !wiring_doc_parse(out, n, &idx)) { vPortFree(out); return WIRING_STORE_IO; }
    uint8_t sha[32];
    mbedtls_sha256(out, n, sha, 0);
    /* The slot that does not hold the newest copy; the first save goes to A. */
    blob_id_t target = s_seq == 0 ? BLOB_WIRING_A : (s_cur == BLOB_WIRING_A ? BLOB_WIRING_B : BLOB_WIRING_A);
    if (w25q_session_open() != 0) { vPortFree(out); return WIRING_STORE_IO; }
    int rc = blob_store_write(target, (uint32_t)n, s_seq + 1u, sha, mem_src, out);
    w25q_session_close();
    if (rc != 0) {
        LOG_E("[wiring] writing slot %s failed (%d): profiles unchanged\n", blob_name(target), rc);
        vPortFree(out);
        return WIRING_STORE_IO;
    }
    if (s_doc) vPortFree(s_doc);
    s_doc = out;
    *s_idx = idx;
    s_cur = target;
    s_seq++;
    return WIRING_STORE_OK;
}
