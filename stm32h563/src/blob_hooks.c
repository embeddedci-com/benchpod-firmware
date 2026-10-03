/*
 * blob_hooks.c — what the pod does when OTA installs a blob into its W25Q slot (blob_store.h).
 */
#include "blob_store.h"
#include <stdio.h>

void blob_store_on_installed(blob_id_t id)
{
    printf("[blob] %s installed\n", blob_name(id));
}
