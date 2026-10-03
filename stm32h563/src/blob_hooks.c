/*
 * blob_hooks.c — what the pod does when OTA installs a blob into its W25Q slot (blob_store.h).
 */
#include "blob_store.h"
#include "signal_engine.h"
#include <stdio.h>

void blob_store_on_installed(blob_id_t id)
{
    printf("[blob] %s installed\n", blob_name(id));
    switch (id) {
    case BLOB_GW0:
    case BLOB_GW1:
        /* The running gateware may now be updatable to the version this firmware expects
           (the firmware was installed before its gateware). */
        signal_engine_gateware_update();
        break;
    default:
        break;
    }
}
