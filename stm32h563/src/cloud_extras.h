#ifndef CLOUD_EXTRAS_H
#define CLOUD_EXTRAS_H

/*
 * cloud_extras — what a corporate network needs for the cloud link (cloud-hardening.md section 3):
 *
 *   company CA   extra trusted roots (a TLS-inspecting proxy re-signs HTTPS with them), PEM, in
 *                the W25Q slot "ca"; installed through the upload path (OTA target "ca")
 *   HTTP proxy   host, port and optional user/password, in the W25Q slot "proxy"
 *
 * Both are read once at boot (worker, bus free) and cached in RAM; the cloud client (net task)
 * takes the cached copies when it connects. A change makes the cloud client reconnect.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CLOUD_CA_MAX_BYTES     16384u
#define CLOUD_PROXY_HOST_MAX   64
#define CLOUD_PROXY_USER_MAX   48
#define CLOUD_PROXY_PASS_MAX   64

typedef struct {
    char     host[CLOUD_PROXY_HOST_MAX];   /* "" = no proxy */
    uint16_t port;
    char     user[CLOUD_PROXY_USER_MAX];   /* "" = no Proxy-Authorization */
    char     pass[CLOUD_PROXY_PASS_MAX];
} cloud_proxy_t;

/* Boot, on the hw worker with the bus free: read both slots into RAM. Always marks the module
   ready, also when the W25Q is missing (then neither is set). */
void cloud_extras_load(void);
/* The cloud client waits for this before its first connect (up to a timeout). */
bool cloud_extras_ready(void);
/* Safe mode skipped the hardware bring-up: no W25Q reads, nothing extra. */
void cloud_extras_mark_ready(void);

/* ---- company CA ---------------------------------------------------------- */
/* Check a PEM blob holds at least one X.509 certificate. NULL = fine, else why not. */
const char *cloud_extras_ca_check(const uint8_t *pem, size_t len);
/* The slot changed (installed or cleared): reload the RAM copy and reconnect the cloud. Worker. */
void cloud_extras_ca_changed(void);
/* Remove the company CA. Worker. NULL = done, else why not. */
const char *cloud_extras_ca_clear(void);
/* Net task: the PEM for TLS when a company CA is in use: the embedded roots + the company CA,
   NUL-terminated, freshly allocated into *out; the caller frees it with cloud_extras_free().
   0 with *out NULL = no company CA (use cloud_ca_pem as it is); -1 = out of memory (retry later,
   and never fall back to the roots alone for it: that would lock out a TLS-inspecting network). */
int   cloud_extras_ca_pem(char **out, size_t *len_with_nul);
/* Net task: TLS refused the combined PEM. Parse the company CA again: when it is corrupt it is
   dropped (cloud_extras_ca_error says why) and true is returned, so the next connect uses the
   built-in roots; false = it parses (or memory is short), so the refusal was out of memory. */
bool  cloud_extras_ca_tls_refused(void);
/* Why an installed company CA is not in use, NULL when none is installed or it is in use. */
#define CLOUD_CA_ERR_HASH   "corrupt (slot hash mismatch)"
#define CLOUD_CA_ERR_PARSE  "unparsable (not a valid PEM X.509 certificate)"
const char *cloud_extras_ca_error(void);
#if !defined(BENCHPOD_RELEASE)
/* Development builds: clear one bit in the installed company CA's slot data (the slot hash then
   fails) and reload, to try the fallback on hardware (console "ca-damage"). NULL = done. */
const char *cloud_extras_ca_damage(void);
#endif
void  cloud_extras_free(void *p);
/* Describe the installed company CA as JSON array items {"subject":..,"sha256":..} into out;
   returns the number of certificates (0 = none). Worker. */
int cloud_extras_ca_describe(char *out, size_t cap);

/* ---- proxy --------------------------------------------------------------- */
/* Set ("host:port", user/pass may be NULL or "") or clear (spec NULL). Worker. NULL = done. */
const char *cloud_extras_proxy_set(const char *spec, const char *user, const char *pass);
/* A copy of the proxy settings (host "" = none). Any task. */
void cloud_extras_proxy_get(cloud_proxy_t *out);

#endif /* CLOUD_EXTRAS_H */
