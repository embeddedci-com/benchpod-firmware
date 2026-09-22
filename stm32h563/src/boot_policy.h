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

/* A firmware update does not touch the iCE40's own config flash, so a pod keeps its old
   gateware until something reprograms it. Which embedded image to load so the gateware
   matches this firmware, or -1 to leave it: the running version (`running`, 0 = the iCE40
   did not answer) differs from the embedded one (`embedded`, 0 = unknown at build time).
   Keeps the kind of image that is running: 1 when it is the deep-replay image, else 0. */
int boot_policy_gateware_image(uint8_t running, uint8_t embedded, int running_is_deep);

#endif /* BOOT_POLICY_H */
