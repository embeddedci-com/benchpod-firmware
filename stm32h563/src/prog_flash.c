/* prog_flash.c — the protocol table (prog_flash.h). The job and the command come in the framework. */
#include "prog_flash.h"
#include <string.h>

static const prog_proto_t *const k_protos[] = {
    &prog_proto_updi, &prog_proto_pic, &prog_proto_c2, &prog_proto_tpi, &prog_proto_pdi,
};

size_t prog_proto_count(void) { return sizeof(k_protos) / sizeof(k_protos[0]); }
const prog_proto_t *prog_proto_at(size_t i) { return i < prog_proto_count() ? k_protos[i] : NULL; }

const prog_proto_t *prog_proto_find(const char *name) {
    for (size_t i = 0; i < prog_proto_count(); i++)
        if (name && strcmp(k_protos[i]->name, name) == 0) return k_protos[i];
    return NULL;
}
