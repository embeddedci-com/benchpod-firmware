#ifndef B64URL_H
#define B64URL_H

#include <stdint.h>
#include <stddef.h>

/* ---- base64url (RFC 4648 §5), no padding ---------------------------------
 *
 * Alphabet: A-Z a-z 0-9 '-' '_'.  No '=' padding is emitted or required.
 * Used to carry binary key material / nonces / signatures over the text-based
 * SCPI and JSON socket protocols.  Decoding is tolerant of '=' padding if a
 * peer includes it, but rejects any other out-of-alphabet byte.
 * ---------------------------------------------------------------------------*/

/* Number of output chars an n-byte input encodes to (no padding). */
#define B64URL_ENCODED_LEN(n)  (((n) * 4 + 2) / 3)
/* Max bytes a c-char (unpadded) input can decode to. */
#define B64URL_DECODED_MAX(c)  (((c) * 3) / 4)

/* Encode n input bytes into out as a NUL-terminated base64url string.
   Requires out_cap >= B64URL_ENCODED_LEN(n) + 1.
   Returns the number of characters written (excluding the NUL), or 0 on
   insufficient capacity. */
size_t b64url_encode(const uint8_t *in, size_t n, char *out, size_t out_cap);

/* Decode a NUL-terminated base64url string into out.
   Writes the decoded length to *out_len.  Returns 0 on success, -1 on a
   malformed input (bad character or impossible length) or insufficient
   out_cap. */
int b64url_decode(const char *in, uint8_t *out, size_t out_cap, size_t *out_len);

#endif /* B64URL_H */
