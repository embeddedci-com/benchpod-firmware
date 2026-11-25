/* xip_prog_mod.c — runs from the W25Q (xip_image.h): the prog image's table and dispatch.
 *
 * The prog image holds the six pin programmers of prog_flash (prog_updi.c, prog_pic.c, prog_c2.c,
 * prog_tpi.c, prog_pdi.c, prog_sbw.c, linked into .xip_prog by name) and this file. It takes the same RAM
 * region as the usb image: a prog_flash job holds a session for the whole job and the other image
 * is not loaded meanwhile. The protocols are pure logic over prog_ctx_t and its callbacks, with no
 * static state; internal code starts one operation at a time through XIP_PROG_FN_CALL
 * (prog_xip_call in prog_job.c). */
#include "xip_image.h"
#include "prog_flash.h"

#include <stdint.h>

static const char k_probe[] = "BenchPod XIP self-test: code and constants from the W25Q";

/* Internal flash (the veneer test): FNV-1a over a byte. */
uint32_t xip_core_fnv_step(uint32_t h, uint8_t b);

static uint32_t selftest(uint32_t seed)
{
    uint32_t h = 2166136261u ^ seed;
    for (unsigned i = 0; i < sizeof(k_probe) - 1u; i++) h = xip_core_fnv_step(h, (uint8_t)k_probe[i]);
    return h;
}

/* One call of one protocol (prog_xop_t); the protocol's own return value. */
static int dispatch(prog_ctx_t *ctx, unsigned proto, unsigned op)
{
    switch (proto) {
    case PROG_PROTO_UPDI: return prog_updi_call(ctx, op);
    case PROG_PROTO_PIC:  return prog_pic_call(ctx, op);
    case PROG_PROTO_C2:   return prog_c2_call(ctx, op);
    case PROG_PROTO_TPI:  return prog_tpi_call(ctx, op);
    case PROG_PROTO_PDI:  return prog_pdi_call(ctx, op);
    case PROG_PROTO_SBW:  return prog_sbw_call(ctx, op);
    default:              return PROG_E_ARGS;
    }
}

__attribute__((section(".xip_prog.table"), used))
const xip_table_t xip_prog_table = {
    XIP_TABLE_MAGIC, XIP_PROG_ABI, XIP_PROG_FN_COUNT,
    { [XIP_PROG_FN_SELFTEST] = (const void *)selftest,
      [XIP_PROG_FN_CALL] = (const void *)dispatch },
};
