/* Host test for prog_pdi.c against a bit-level model of the chip (prog_sim.h). STUB. */
#include "prog_flash.h"
#include "prog_sim.h"
#include <stdio.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

int main(void) {
    CHECK(prog_proto_pdi.name != NULL);
    printf("test_prog_pdi: %s\n", failures ? "FAILED" : "ok");
    return failures ? 1 : 0;
}
