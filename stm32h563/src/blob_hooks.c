/*
 * blob_hooks.c — what the pod does when OTA installs a blob into its W25Q slot (blob_store.h).
 */
#include "blob_store.h"
#include "signal_engine.h"
#include "esp_wifi_ctrl.h"
#include "cloud_extras.h"
#include "dac_quad.h"
#include "i2c_bus.h"
#include <stdio.h>
#include "bp_log.h"

void blob_store_on_installed(blob_id_t id)
{
    LOG_I("[blob] %s installed\n", blob_name(id));
    switch (id) {
    case BLOB_GW0:
    case BLOB_GW1:
    case BLOB_GW2:
        /* The running gateware may now be updatable to the version this firmware expects
           (the firmware was installed before its gateware), or this is a new image of the same
           version for the kind that runs (its identity differs from the recorded one). */
        signal_engine_gateware_update();
        /* A new deep or emulation image of the kind that runs: load it from its slot (no flash
           write; gw0 is handled above, it is the image the iCE40 boots from). */
        {
            const int kind = id == BLOB_GW1 ? 1 : id == BLOB_GW2 ? 2 : 0;
            if (kind != 0 && signal_engine_image_kind() == kind && blob_state(id) == BLOB_STATE_OK)
                (void)fpga_warmboot_ex((uint8_t)kind, true, NULL);
        }
        /* A blank iCE40 booted without gateware, so boot could not park the DAC8554 or switch the
           +-14 V supply on (analog v3r2).  Do it now that the gateware runs; once on, a later
           install must not park outputs that are in use. */
        if (dac_quad_present() && !ana14_is_on()) dac_quad_boot();
        break;
    case BLOB_ESP:
        esp_wifi_ctrl_image_installed();
        break;
    case BLOB_XIP:
        /* The ESP32-C3's ROM-loader code is in the xip image: an automatic C3 flash that was
           refused for lack of it earns another try now (esp_wifi_ctrl.c). */
        esp_wifi_ctrl_image_installed();
        break;
    case BLOB_CA:
        cloud_extras_ca_changed();   /* reload it and reconnect the cloud with it */
        break;
    default:
        break;
    }
}
