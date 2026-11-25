#ifndef BOOT_POLICY_H
#define BOOT_POLICY_H

#include <stdint.h>

/* boot_policy: which subsystems a safe-mode boot turns off (pure logic, host-tested).
 *
 * Safe mode used to turn off networking AND the iCE40/PSRAM whatever broke, so a pod that
 * crash-looped in the iCE40 bring-up vanished from the network and nobody could see why.
 * Now it turns off only what failed, so the pod stays visible whenever it can:
 *   - the task that crashed ("net" or "hw", from the fault record) names the culprit;
 *   - otherwise a subsystem whose bring-up started but never finished (a hang: IWDG);
 *   - otherwise, a failure before either started or somewhere else: both off.
 * Safe boots keep what earlier safe boots turned off, so a second culprit adds to the set
 * instead of the pod alternating between two crash loops. */

#define BOOT_SUB_NET 0x1u   /* networking: lwIP, Ethernet/Wi-Fi, cloud */
#define BOOT_SUB_HW  0x2u   /* iCE40 link, PSRAM, gateware */
#define BOOT_SUB_ALL (BOOT_SUB_NET | BOOT_SUB_HW)

/* The subsystems the last failed boot points at (0: none identified).  crash_task is the
   fault record's task name ("" when the last reset was not a crash). */
uint32_t boot_policy_culprit(uint32_t started, uint32_t done, const char *crash_task);

/* What a safe-mode boot turns off, given what the previous boot turned off (prev_off). */
uint32_t boot_policy_off(uint32_t prev_off, uint32_t culprit);

/* "network", "iCE40/PSRAM" or "network and iCE40/PSRAM". */
const char *boot_policy_off_str(uint32_t off);


/* The version alone misses a new image of the SAME gateware version (a dev rebuild, or the
   emulation image of a later build of v54): it would never be written while that image kind
   runs. So the firmware also records WHICH image it wrote to the iCE40's config flash: the slot
   it came from and that slot's sha256 and length (the slot header, blob_store.h), in a small
   record next to the image (GW_IDENT_OFF, ice40_flash.h). Written right after every write of the
   config flash from a slot, erased before any write of it, so it never names an image that is
   not there. */
#define GW_IDENT_MAGIC    0x49574742u   /* "BGWI" */
#define GW_IDENT_VERSION  1u

typedef struct {
    uint32_t magic;          /* GW_IDENT_MAGIC */
    uint16_t version;        /* GW_IDENT_VERSION */
    uint16_t slot;           /* blob_id_t the image was copied from */
    uint32_t len;            /* image bytes */
    uint8_t  sha256[32];     /* of the image, from the slot header */
    uint32_t crc;            /* CRC-32 of everything above */
} gw_ident_t;
_Static_assert(sizeof(gw_ident_t) == 48, "gw_ident_t is stored raw in the W25Q");

/* Fill a record for the image of `slot` (len bytes, sha256). */
void gw_ident_make(gw_ident_t *r, uint16_t slot, uint32_t len, const uint8_t sha256[32]);
/* 1 when r is a whole record (magic, version, CRC); 0 for a blank (erased) or torn one. */
int  gw_ident_valid(const gw_ident_t *r);

typedef enum {
    GW_IDENT_NONE,      /* the running kind's slot does not hold what this firmware expects:
                           nothing to compare against, leave the iCE40 alone */
    GW_IDENT_UNKNOWN,   /* no valid record (first boot with this check, or a write was cut) */
    GW_IDENT_SAME,      /* the record names exactly the slot's image */
    GW_IDENT_DIFFERS,   /* the record names another image */
} gw_ident_match_t;

/* How the record `rec` (NULL or invalid = unknown) relates to the image in `slot` (len, sha256).
   slot_ok 0 = the slot does not hold the blob this firmware expects (GW_IDENT_NONE). */
gw_ident_match_t gw_ident_match(const gw_ident_t *rec, int slot_ok, uint16_t slot, uint32_t len,
                                const uint8_t sha256[32]);

typedef enum {
    GW_ACT_NONE,      /* leave the iCE40 alone */
    GW_ACT_WRITE,     /* write image *img from its slot (and record it) */
    GW_ACT_COMPARE,   /* no record yet: read-compare the config flash with image *img's slot;
                         record it when equal, write it when not. One pass, then recorded: a
                         blank record never makes every boot rewrite the flash. */
} gw_action_t;


/* From firmware 4.0.6 an image switch loads the image straight from its slot (SPI slave mode)
   and never writes the config flash, so offset 0, the image the iCE40 powers up with, always
   holds the loop image (slot gw0). What to do so that holds and the running gateware matches
   this firmware:
     - boot: GW_ACT_WRITE writes gw0 to offset 0, GW_ACT_COMPARE read-compares offset 0 with gw0
       first (no record yet), GW_ACT_NONE leaves offset 0 alone. boot_ident is offset 0's record
       against slot gw0 (gw_ident_match). A pod coming from an older firmware that last switched
       to another image gets gw0 written once.
     - load: the image to load from its slot afterwards (no write), -1 for none. The running kind
       is kept: after a write or compare (both reconfigure the iCE40 from offset 0, so loop runs)
       a deep or emulation image is loaded again, and a running image whose version is not this
       firmware's is loaded from its slot.
   An iCE40 that never configured gets gw0 written. Nothing is touched while the iCE40 does not
   answer (running 0) or the firmware does not know its own gateware (embedded 0). */
typedef struct {
    gw_action_t boot;
    int         load;
} gw_plan_t;
gw_plan_t boot_policy_gateware_plan(uint8_t running, uint8_t embedded, int running_kind,
                                    int configured, gw_ident_match_t boot_ident);
#endif /* BOOT_POLICY_H */
