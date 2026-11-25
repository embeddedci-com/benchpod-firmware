/*
 * dap_id.h — read the IDs of the chip on the wire for `dap_start` with "op":"id".
 *
 * The pod arms the SWD or JTAG engine on the caller's pins (command_handler_dap.c does the
 * gates and pin claims), calls these, disarms again and replies one JSON object.  Nothing here
 * halts, resets or writes target memory: SWD touches DP/AP registers only (CTRL/STAT power-up
 * request, SELECT, CSW, TAR), and the power-up request is cleared at the end.  swd_ll / jtag_ll
 * are the only hardware access, so the host tests run it against a fake DP and a fake TAP chain.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define DAP_ID_MAX_READS   8
#define DAP_ID_MAX_IDCODES 8
#define DAP_ID_WAIT_MS     200u   /* how long a missing answer is retried when wait_ms is absent */

typedef struct {
    bool     found;                       /* a DPIDR answered */
    bool     has_targetid, has_apidr, has_cpuid;
    uint32_t dpidr, targetid, apidr, cpuid;
    size_t   n_reads;
    uint32_t addr[DAP_ID_MAX_READS];
    uint32_t value[DAP_ID_MAX_READS];
    bool     value_ok[DAP_ID_MAX_READS];  /* false: the read faulted or got no ACK (null) */
} dap_id_swd_t;

/* "read":["0x...",...] of a command: the addresses into addr[] (cap entries).  Returns the
   count (0 when the key is absent), or -1 when the list is longer than cap or an entry is not a
   number. */
int    dap_id_parse_reads(const char *json, uint32_t *addr, size_t cap);

/* Read DPIDR (retried for wait_ms), TARGETID (DP version 2+), AP0's IDR, and, on a MEM-AP,
   CPUID plus each of addr[0..n).  The SWD engine must be armed. */
void   dap_id_swd(uint32_t wait_ms, const uint32_t *addr, size_t n, dap_id_swd_t *r);

/* The IDCODEs of a JTAG chain from nbits captured with TDI = 1 after Test-Logic-Reset
   (bit 0 of tdo[0] first), TDO-nearest first; a 1-bit bypass device (bit 0 = 0) has none.
   Stops at an all-ones word (the chain's end) or 32 zero bits in a row (TDO stuck low).
   Returns how many went into id[0..cap). */
size_t dap_id_jtag_parse(const uint8_t *tdo, size_t nbits, uint32_t *id, size_t cap);

/* Scan the chain (retried for wait_ms until an IDCODE shows up); the JTAG engine must be armed. */
size_t dap_id_jtag(uint32_t wait_ms, uint32_t *id, size_t cap);

/* The reply object, "0x%08x" lower case.  Return the length, or 0 when cap is too small. */
size_t dap_id_fmt_swd(char *out, size_t cap, const dap_id_swd_t *r);
size_t dap_id_fmt_jtag(char *out, size_t cap, const uint32_t *id, size_t n);
