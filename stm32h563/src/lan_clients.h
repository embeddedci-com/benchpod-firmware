#ifndef LAN_CLIENTS_H
#define LAN_CLIENTS_H
/*
 * lan_clients — the computers paired with this pod for the LAN API (docs/design/lan-pairing.md).
 *
 * A computer is known by the SHA-256 of its TLS client certificate's public key (the DER
 * SubjectPublicKeyInfo), so it may renew its certificate with the same key. Pairing adds that hash
 * and a name; it happens over USB (the console, or JSON on the USB network adapter) or from the
 * cloud, never from the external LAN. A TLS connection on 8443 whose client key is on the list is
 * "paired" (cmd_gate.h).
 *
 * This file is the list itself: pure, no flash, no locking, host-tested. lan_tls.c keeps the one
 * list in RAM, stores it in its flash record and serializes access.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LAN_CLIENTS_MAX      32
#define LAN_CLIENT_NAME_MAX  24    /* with the terminating NUL: 23 characters */
#define LAN_CLIENT_FP_LEN    32    /* SHA-256 */

typedef struct {
    uint8_t fp[LAN_CLIENT_FP_LEN];
    char    name[LAN_CLIENT_NAME_MAX];
} lan_client_t;
_Static_assert(sizeof(lan_client_t) == 56, "lan_client_t is stored as is");

typedef struct {
    uint8_t      n;
    uint8_t      cap;                       /* how many fit in the flash record (<= MAX) */
    lan_client_t c[LAN_CLIENTS_MAX];
} lan_clients_t;

void lan_clients_init(lan_clients_t *l, uint8_t cap);

/* Index of fp, or -1. */
int  lan_clients_find(const lan_clients_t *l, const uint8_t fp[LAN_CLIENT_FP_LEN]);

/* A name is 1-23 printable ASCII characters, no quotes or backslashes (it goes into JSON as is). */
bool lan_clients_name_ok(const char *name);

/* Parse 64 hex digits (either case). */
bool lan_clients_fp_from_hex(const char *hex, uint8_t fp[LAN_CLIENT_FP_LEN]);
void lan_clients_fp_to_hex(const uint8_t fp[LAN_CLIENT_FP_LEN], char out[2 * LAN_CLIENT_FP_LEN + 1]);

typedef enum {
    LAN_CLIENTS_ADDED = 0,
    LAN_CLIENTS_RENAMED,   /* the key was already on the list: its name is now `name` */
    LAN_CLIENTS_SAME,      /* already on the list with that name: nothing changed */
    LAN_CLIENTS_FULL,
    LAN_CLIENTS_BAD_NAME,
} lan_clients_add_t;

lan_clients_add_t lan_clients_add(lan_clients_t *l, const uint8_t fp[LAN_CLIENT_FP_LEN], const char *name);

/* Remove the entries `what` names: an exact name (every entry with it), or 8 to 64 hex digits that
   start one fingerprint (a prefix that matches two entries removes neither: -2). Returns how many
   were removed (0 = none matched), or -2 when a hex prefix is ambiguous. */
int  lan_clients_remove(lan_clients_t *l, const char *what);

void lan_clients_clear(lan_clients_t *l);

/* Stored form: n (1 byte) then n entries. Returns the size; 0 if it does not fit in cap bytes. */
size_t lan_clients_pack(const lan_clients_t *l, uint8_t *out, size_t cap);
/* false when the bytes are not a valid list (bad count, a bad name, more than l->cap entries). */
bool   lan_clients_unpack(lan_clients_t *l, const uint8_t *in, size_t len);
size_t lan_clients_packed_size(uint8_t n);

#endif /* LAN_CLIENTS_H */
