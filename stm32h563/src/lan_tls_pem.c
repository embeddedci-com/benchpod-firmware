/*
 * lan_tls_pem.c — split an uploaded PEM bundle into key and certificates (lan_tls_pem.h).
 */
#include "lan_tls_pem.h"

#include <stdbool.h>
#include <string.h>

/* The line starting at p (up to end): its length without the line break, and where the next
   line starts. */
static size_t line_at(const char *p, const char *end, const char **next)
{
    const char *q = p;
    while (q < end && *q != '\n' && *q != '\r') q++;
    size_t n = (size_t)(q - p);
    while (q < end && (*q == '\n' || *q == '\r')) q++;
    *next = q;
    return n;
}

static bool starts(const char *p, size_t n, const char *s)
{
    size_t k = strlen(s);
    return n >= k && memcmp(p, s, k) == 0;
}

/* Append n bytes and a newline to buf (cap bytes, *used so far). false = no room. */
static bool put(char *buf, size_t cap, size_t *used, const char *p, size_t n)
{
    if (*used + n + 2 > cap) return false;   /* + newline + the final NUL */
    memcpy(buf + *used, p, n);
    *used += n;
    buf[(*used)++] = '\n';
    return true;
}

const char *lan_tls_pem_split(const char *in, size_t len,
                              char *key, size_t key_cap, size_t *key_len,
                              char *cert, size_t cert_cap, size_t *cert_len)
{
    size_t ku = 0, cu = 0;
    int keys = 0, certs = 0;
    const char *p = in, *end = in + len;
    enum { NONE, IN_KEY, IN_CERT, IN_OTHER } in_block = NONE;
    char label[48] = "";

    while (p < end) {
        const char *next;
        size_t n = line_at(p, end, &next);
        if (in_block == NONE) {
            if (starts(p, n, "-----BEGIN ") && n > 16 && memcmp(p + n - 5, "-----", 5) == 0) {
                size_t ln = n - 16;   /* between "-----BEGIN " and "-----" */
                if (ln >= sizeof(label)) ln = sizeof(label) - 1;
                memcpy(label, p + 11, ln);
                label[ln] = '\0';
                if (strcmp(label, "CERTIFICATE") == 0) {
                    in_block = IN_CERT;
                    certs++;
                    if (!put(cert, cert_cap, &cu, p, n)) return "the certificates are too large";
                } else if (strstr(label, "PRIVATE KEY")) {
                    if (strstr(label, "ENCRYPTED")) return "the private key is encrypted; upload it without a passphrase";
                    if (++keys > 1) return "more than one private key";
                    in_block = IN_KEY;
                    if (!put(key, key_cap, &ku, p, n)) return "the private key is too large";
                } else {
                    in_block = IN_OTHER;
                }
            }
        } else {
            bool is_end = starts(p, n, "-----END ");
            if (in_block == IN_KEY) {
                if (starts(p, n, "Proc-Type:") && n >= 19 && memcmp(p + n - 9, "ENCRYPTED", 9) == 0)
                    return "the private key is encrypted; upload it without a passphrase";
                if (!put(key, key_cap, &ku, p, n)) return "the private key is too large";
            } else if (in_block == IN_CERT) {
                if (!put(cert, cert_cap, &cu, p, n)) return "the certificates are too large";
            }
            if (is_end) in_block = NONE;
        }
        p = next;
    }
    if (in_block != NONE) return "a PEM block has no END line";
    if (keys == 0) return "no private key in the upload";
    if (certs == 0) return "no certificate in the upload";
    key[ku++] = '\0';
    cert[cu++] = '\0';
    *key_len = ku;
    *cert_len = cu;
    return NULL;
}
