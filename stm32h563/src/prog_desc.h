#ifndef PROG_DESC_H
#define PROG_DESC_H

/* prog_desc: the typed chip descriptor of `prog_flash` info/start.
 *
 * Until firmware 4.0.5 the host packed a chip's protocol options into one integer, `flags`
 * (PIC command set bits, UPDI flash base address, TPI word count ...). Since 4.0.6 the host sends
 * named fields instead (features.prog_named); this module checks them and turns them into the
 * ctx->flags / code-protect address the protocols read, so the protocol code is the same for both.
 * `flags` and `cp_addr` are still accepted for one release (a host that does not know the named
 * fields), but not together with a named field of the same request.
 *
 * Named fields (JSON keys of the request), by protocol:
 *   updi: flash_base (u32)   the chip's flash in the UPDI data space (0 = the SIB's default)
 *   pic:  cmd_set "6bit"|"8bit", pc "word"|"byte", ee_erase, cp_high, q10, id_split, key_flip
 *         (booleans), cp_bit (0..15); cp_addr (u32, the byte address of the config word
 *         with the code-protect bit) stays a field of its own, read by the request handler
 *   tpi:  write_words (1, 2 or 4), page_erase (bool)
 *   c2:   fpdat (1..255, the FPDAT register address; 0 = by device ID), no_setup (bool)
 * Pure C over a key lookup, so host tests link it directly. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t flags;      /* what the protocol reads from ctx->flags */
    bool     named;      /* the request used named fields */
} prog_desc_t;

/* Parse the descriptor of `proto` ("updi", "pic", "c2", "tpi", "pdi") from the request. Returns 0,
   or -1 with a message in err (an unknown value, a field of another protocol, flags mixed with
   named fields). `json` is the whole request. */
int prog_desc_parse(const char *json, const char *proto, prog_desc_t *d, char *err, size_t cap);

#endif
