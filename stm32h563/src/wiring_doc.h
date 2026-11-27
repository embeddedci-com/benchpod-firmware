#ifndef WIRING_DOC_H
#define WIRING_DOC_H
/*
 * wiring_doc: the pod's wiring profiles as one document in the W25Q slot WIRING (wiring_store.h).
 * Pure: no I/O, host-tested (test/test_wiring_doc.c).
 *
 * The pod stores each profile as opaque JSON text with a name and a revision. It does not validate a
 * profile: the hosts (SDK, CLI, MCP, server, apps) do, except the target-power supply ("supply" or the older "efuse": wiring_doc_json_supply,
 * checked on save). The key it reads is la_mv, the I/O voltage
 * (wiring_doc_json_la_mv), so an LA operation can take it from the profile when none is set. The pod only checks that the
 * text is one JSON object (wiring_doc_json_object_ok), so a reply that carries it stays valid.
 *
 * Document layout (little-endian):
 *   "BPWR" | version (1) | count | default index (0xFF = none) | 0
 *   count x { name_len (1) | name | revision (4) | json_len (2) | json }
 * An empty or unreadable slot is an empty document: no profiles, no default.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "supply.h"

#define WIRING_MAX_PROFILES   16u
#define WIRING_NAME_MAX       40u        /* characters, [a-z0-9-], first a letter or digit */
#define WIRING_PROFILE_MAX    4096u      /* bytes of one profile's JSON */
#define WIRING_DOC_HDR        8u
#define WIRING_DOC_MAX        (16u * 1024u)   /* kept in RAM (heap) and in each 32 KB slot */
#define WIRING_NO_DEFAULT     0xFFu

typedef struct {
    char     name[WIRING_NAME_MAX + 1];
    uint32_t rev;
    uint16_t len;     /* json bytes */
    uint32_t off;     /* where the json starts in the document */
} wiring_entry_t;

typedef struct {
    uint8_t        count;
    uint8_t        def;          /* index of the default profile, WIRING_NO_DEFAULT = none */
    uint32_t       doc_len;      /* bytes of the whole document (0 when empty) */
    wiring_entry_t e[WIRING_MAX_PROFILES];
} wiring_index_t;

typedef enum { WIRING_OP_PUT, WIRING_OP_DELETE, WIRING_OP_DEFAULT } wiring_op_kind_t;

typedef struct {
    wiring_op_kind_t kind;
    const char      *name;
    const uint8_t   *json;       /* PUT */
    uint16_t         json_len;   /* PUT */
} wiring_op_t;

/* The profile's target-power supply (supply.h): the "supply" key ("5v", "external", "3v3", "1v8") and/or
   the older "efuse" key (1 or 2), top level only.  Both may be there when they name the same supply.
   The one place a profile's supply is read.  WIRING_SUPPLY_NONE: neither key (or null); _OK: *out set;
   _BAD: a value that is not a supply; _CONFLICT: supply and efuse disagree.  For _BAD and _CONFLICT
   bad gets the text for the error (BP0089).  A 3v3 or 1v8 still needs board_caps()->supply_3v3_1v8:
   the caller checks (supply_is_j1). */
typedef enum { WIRING_SUPPLY_NONE = 0, WIRING_SUPPLY_OK, WIRING_SUPPLY_BAD, WIRING_SUPPLY_CONFLICT } wiring_supply_result_t;
wiring_supply_result_t wiring_doc_json_supply(const uint8_t *json, size_t len, supply_t *out, char *bad, size_t cap);

/* Outcome of a change, before anything is written. */
typedef enum {
    WIRING_OK = 0,
    WIRING_E_NAME,          /* the name breaks the naming rule */
    WIRING_E_NOT_FOUND,     /* DELETE / DEFAULT of a name the pod does not have */
    WIRING_E_FULL,          /* PUT of a new name with WIRING_MAX_PROFILES already, or past WIRING_DOC_MAX */
    WIRING_E_TOO_LARGE,     /* PUT of more than WIRING_PROFILE_MAX bytes */
    WIRING_E_DEFAULT,       /* DELETE of the default profile */
    WIRING_E_NOT_OBJECT,    /* PUT of text that is not one JSON object */
} wiring_result_t;

/* Parse a document into idx. Returns false for a corrupt document (idx is then empty: the caller
   treats the slot as empty). A zero-length document is a valid empty one. */
bool wiring_doc_parse(const uint8_t *doc, size_t len, wiring_index_t *idx);

bool wiring_name_ok(const char *name);
int  wiring_index_find(const wiring_index_t *idx, const char *name);   /* -1 = none */

/* Whether text is exactly one JSON object (whitespace around it allowed): balanced braces and
   brackets outside strings, terminated strings, valid escapes. Not a full validator: enough that
   splicing it into a reply cannot break the reply's JSON. */
bool wiring_doc_json_object_ok(const uint8_t *text, size_t len);

/* The profile's top-level "la_mv" (the board's I/O voltage) when it is 1800, 3300 or 5000 (5 V I/O,
   digital v4; the caller drops it on other boards), else 0 (absent,
   null, another value, or a key of that name nested in another object). json need not be
   NUL-terminated. */
uint16_t wiring_doc_json_la_mv(const uint8_t *json, size_t len);

/* Check an op against idx without building anything. */
wiring_result_t wiring_doc_check(const wiring_index_t *idx, const wiring_op_t *op);

/* Build the document after op into out (cap bytes). old is the current document that idx was
   parsed from. Returns the new length, 0 when out is too small. Call wiring_doc_check first.
   PUT gives the profile revision old + 1 (1 when new); the first profile becomes the default. */
size_t wiring_doc_build(const uint8_t *old, const wiring_index_t *idx, const wiring_op_t *op,
                        uint8_t *out, size_t cap);

/* The document's size after op (for the caller's buffer). */
size_t wiring_doc_size_after(const wiring_index_t *idx, const wiring_op_t *op);

#endif /* WIRING_DOC_H */
