/* prog_flash.c — the protocol metadata table (prog_flash.h). The job and the command come in the
   framework; the protocols themselves run from the "prog" XIP image (xip_prog_mod.c). */
#include "prog_flash.h"
#include <string.h>

static const prog_proto_info_t k_protos[PROG_PROTO_COUNT] = {
    [PROG_PROTO_UPDI] = { PROG_INFO_UPDI },
    [PROG_PROTO_PIC]  = { PROG_INFO_PIC },
    [PROG_PROTO_C2]   = { PROG_INFO_C2 },
    [PROG_PROTO_TPI]  = { PROG_INFO_TPI },
    [PROG_PROTO_PDI]  = { PROG_INFO_PDI },
};

size_t prog_proto_count(void) { return PROG_PROTO_COUNT; }
const prog_proto_info_t *prog_proto_at(size_t i) { return i < PROG_PROTO_COUNT ? &k_protos[i] : NULL; }

const prog_proto_info_t *prog_proto_find(const char *name) {
    for (size_t i = 0; i < PROG_PROTO_COUNT; i++)
        if (name && strcmp(k_protos[i].name, name) == 0) return &k_protos[i];
    return NULL;
}
