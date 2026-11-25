/* Host test for prog_desc.c: the named chip fields of prog_flash turn into the same ctx->flags the
   packed `flags` integer of firmware 4.0.5 and older carried, for every combination the host's
   device table (embeddedci-python prog-devices/) uses. */
#include "prog_desc.h"
#include "prog_pic.h"
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static void ok(const char *proto, const char *json, uint32_t flags, bool named)
{
    prog_desc_t d;
    char err[64] = "";
    int rc = prog_desc_parse(json, proto, &d, err, sizeof(err));
    if (rc != 0 || d.flags != flags || d.named != named) {
        printf("FAIL %s %s: rc=%d flags=0x%x (want 0x%x) named=%d err=%s\n", proto, json, rc,
               (unsigned)d.flags, (unsigned)flags, d.named, err);
        failures++;
    }
}

static void bad(const char *proto, const char *json, const char *msg)
{
    prog_desc_t d;
    char err[64] = "";
    int rc = prog_desc_parse(json, proto, &d, err, sizeof(err));
    if (rc == 0 || !strstr(err, msg)) {
        printf("FAIL %s %s: rc=%d err='%s' (want '%s')\n", proto, json, rc, err, msg);
        failures++;
    }
}

int main(void)
{
    /* nothing sent: no flags, not named */
    ok("pic", "{\"cmd\":\"prog_flash\"}", 0, false);
    ok("updi", "{\"cmd\":\"prog_flash\"}", 0, false);
    ok("pdi", "{\"cmd\":\"prog_flash\"}", 0, false);
    /* the old packed integer still works */
    ok("pic", "{\"flags\":1796}", 1796, false);
    ok("updi", "{\"flags\":8388608}", 8388608u, false);
    ok("tpi", "{\"flags\":8}", 8, false);

    /* UPDI: the three flash bases of the table */
    ok("updi", "{\"flash_base\":8388608}", 8388608u, true);
    ok("updi", "{\"flash_base\":32768}", 32768u, true);
    ok("updi", "{\"flash_base\":\"0x4000\"}", 16384u, true);

    /* PIC: the eight combinations of the table (flags -> named fields) */
    ok("pic", "{\"cmd_set\":\"8bit\"}", 1, true);                                   /* PIC16F184xx */
    ok("pic", "{\"cmd_set\":\"6bit\",\"ee_erase\":true,\"cp_bit\":0}", 4, true);
    ok("pic", "{\"cmd_set\":\"6bit\",\"ee_erase\":false}", 0, true);
    ok("pic", "{\"cmd_set\":\"6bit\",\"id_split\":true}", 32, true);                /* PIC16F170x */
    ok("pic", "{\"cmd_set\":\"6bit\",\"ee_erase\":true,\"id_split\":true,\"cp_bit\":3}",
       4 | 32 | PIC_F_CP_BIT(3), true);
    ok("pic", "{\"cmd_set\":\"8bit\",\"pc\":\"byte\",\"cp_bit\":0}", 3, true);       /* PIC18 Q */
    ok("pic", "{\"cmd_set\":\"8bit\",\"pc\":\"byte\",\"q10\":true}", 19, true);      /* PIC18 Q10 */
    ok("pic", "{\"cmd_set\":\"8bit\",\"pc\":\"byte\",\"cp_bit\":1}", 3 | PIC_F_CP_BIT(1), true);
    ok("pic", "{\"cmd_set\":\"6bit\",\"key_flip\":true,\"cp_high\":true}", 64 | 8, true);
    ok("pic", "{\"cp_addr\":65558}", 0, false);          /* cp_addr is read by the request handler */

    /* TPI */
    ok("tpi", "{\"write_words\":2}", 2, true);
    ok("tpi", "{\"write_words\":4}", 4, true);
    ok("tpi", "{\"write_words\":1}", 1, true);          /* same as 0 in prog_tpi.c */
    ok("tpi", "{\"page_erase\":true}", 8, true);
    ok("tpi", "{\"write_words\":2,\"page_erase\":true}", 10, true);

    /* C2 */
    ok("c2", "{\"fpdat\":173}", 173, true);
    ok("c2", "{\"no_setup\":true}", 0x100, true);

    /* refusals */
    bad("pic", "{\"cmd_set\":\"7bit\"}", "cmd_set");
    bad("pic", "{\"pc\":\"word2\"}", "pc");
    bad("pic", "{\"cp_bit\":16}", "cp_bit");
    bad("pic", "{\"cmd_set\":\"6bit\",\"pc\":\"byte\"}", "pc byte");
    bad("pic", "{\"cmd_set\":\"8bit\",\"q10\":true}", "q10");
    bad("pic", "{\"cmd_set\":\"8bit\",\"flags\":1}", "cannot be mixed");
    bad("pic", "{\"flags\":\"zz\"}", "flags");
    bad("updi", "{\"cmd_set\":\"8bit\"}", "pic only");
    bad("pdi", "{\"flash_base\":1}", "updi only");
    bad("pic", "{\"write_words\":2}", "tpi only");
    bad("tpi", "{\"write_words\":3}", "write_words");
    bad("tpi", "{\"write_words\":8}", "write_words");
    bad("c2", "{\"fpdat\":256}", "fpdat");
    bad("updi", "{\"flash_base\":\"x\"}", "flash_base");

    printf(failures ? "FAILED: %d\n" : "prog_desc: all passed\n", failures);
    return failures ? 1 : 0;
}
