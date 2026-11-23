/* prog_pic.c — PIC ICSP low-voltage entry (PIC16F1xxx, PIC16F18xxx, PIC18F Q) for prog_flash (prog_flash.h).
 * STUB: every call answers PROG_E_UNSUPPORTED until the protocol lands. */
#include "prog_flash.h"

static int stub(prog_ctx_t *ctx) { (void)ctx; return PROG_E_UNSUPPORTED; }
static void stub_leave(prog_ctx_t *ctx, bool run) { (void)ctx; (void)run; }

const prog_proto_t prog_proto_pic = {
    .name = "pic", .uart = false, .uses_nrst = true,
    .mems = (1u<<PROG_MEM_FLASH)|(1u<<PROG_MEM_EEPROM)|(1u<<PROG_MEM_CONFIG)|(1u<<PROG_MEM_USER),
    .enter = stub, .erase = stub, .write = stub, .read = stub, .leave = stub_leave,
};
