#ifndef VERSION_H
#define VERSION_H

/* Single source of truth for the firmware version string.  Reported by the
   console/JSON `status`, the boot banner, and the cloud `capabilities` frame (so
   the server can record which build a device is running and decide whether to
   push an OTA update). */
#define FIRMWARE_VERSION "3.1.1"

#endif /* VERSION_H */
