/* prog_c2.c — Silicon Labs C2 (EFM8, C8051F) for prog_flash (prog_flash.h).
 * STUB: every call answers PROG_E_UNSUPPORTED until the protocol lands. */
#include "prog_flash.h"

static int stub(prog_ctx_t *ctx) { (void)ctx; return PROG_E_UNSUPPORTED; }
static void stub_leave(prog_ctx_t *ctx, bool run) { (void)ctx; (void)run; }

const prog_proto_t prog_proto_c2 = {
    .name = "c2", .uart = false, .uses_nrst = false,
    .mems = (1u<<PROG_MEM_FLASH)|(1u<<PROG_MEM_CONFIG),
    .enter = stub, .erase = stub, .write = stub, .read = stub, .leave = stub_leave,
};
