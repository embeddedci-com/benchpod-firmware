#ifndef XSPI_GUARD_H
#define XSPI_GUARD_H

/* xspi_guard — keeps the quad bus's ordinary (indirect) commands safe while the W25Q is
 * memory-mapped for code execution (XIP, xip_session.h).
 *
 * OCTOSPI1 serves the PSRAM and the W25Q through one set of registers. Memory-mapped mode lets the
 * CPU run code straight from the W25Q, but an indirect command (a PSRAM read, a W25Q status read)
 * needs the controller out of that mode, and the W25Q's hardware chip-select (PB6 as
 * OCTOSPI1_NCS) back to a GPIO held high, or the flash would answer every PSRAM command too.
 *
 * Every indirect command goes through xspi_guard_enter()/xspi_guard_exit() (psram.c xspi_xfer,
 * w25q.c fxfer; test/check_xspi_guard.py fails the build on a HAL_XSPI_* call anywhere else).
 * With nothing mapped they cost a flag test. While mapped, enter() unmaps and exit() maps again
 * before returning, so code running from the W25Q can call an internal function that reads the
 * PSRAM and still have its next instruction there. Both must run from internal flash or RAM
 * (they do: this file is never placed in the XIP region).
 *
 * The mapping itself belongs to whoever registered the hooks (xip_session.c); this file only
 * counts and calls them. Pure (host tests): no HAL here.
 */

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int  (*unmap)(void);   /* leave memory-mapped mode; NCS pin to GPIO high. 0 = ok */
    int  (*map)(void);     /* enter memory-mapped mode again. 0 = ok */
} xspi_guard_hooks_t;

/* Install the hooks and mark the W25Q mapped (hooks != NULL), or clear both (NULL). The mapper
   calls this after it mapped, and with NULL after it unmapped for good. */
void xspi_guard_set_mapped(const xspi_guard_hooks_t *hooks);
bool xspi_guard_mapped(void);

/* Around one indirect command. enter() returns a token for exit(); nested pairs only unmap and map
   once (the outermost pair). Returns false from enter when the unmap failed: the caller must not
   send its command (and still calls exit with the token). */
typedef struct { bool unmapped; bool ok; } xspi_guard_tok_t;
xspi_guard_tok_t xspi_guard_enter(void);
void xspi_guard_exit(xspi_guard_tok_t t);

/* Counters for diag: unmaps done, map failures (code still running from the W25Q would then
   fault on its next fetch, so a failed map is a hard error the session handles). */
typedef struct { uint32_t unmaps, map_failures, unmap_failures; uint16_t depth; } xspi_guard_stats_t;
xspi_guard_stats_t xspi_guard_stats(void);

#endif /* XSPI_GUARD_H */
