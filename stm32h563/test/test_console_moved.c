/* test_console_moved.c — the console's reply for commands firmware 3.9.0 moved to the JSON API. */
#include "console_moved.h"

#include <stdio.h>
#include <string.h>

static int s_fail;
#define CHECK(c, ...) do { if (!(c)) { s_fail++; printf("FAIL %s:%d  ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

int main(void)
{
    char line[256];
    CHECK(console_moved_to("la-voltage") && !strcmp(console_moved_to("la-voltage"), "la_voltage"), "la-voltage");
    CHECK(!strcmp(console_moved_to("power"), "target_power"), "power");
    CHECK(!strcmp(console_moved_to("measure"), "adc_read"), "measure");
    /* uploads go over the network now (USB network adapter); recovery is dfu */
    CHECK(!strcmp(console_moved_to("upload-begin"), "ota_begin"), "upload-begin");
    CHECK(!strcmp(console_moved_to("upload-data"), "ota_data"), "upload-data");
    /* the kept recovery commands are not moved: older CLIs keep using them */
    const char *kept[] = { "status", "dfu", "identity", "identity-wipe", "lan-policy", "sig-policy",
                           "wifi-set", "wifi-clear", "blobs", "help", "flash-ice40" };
    for (size_t i = 0; i < sizeof(kept) / sizeof(kept[0]); i++)
        CHECK(console_moved_to(kept[i]) == NULL, "%s must stay on the console", kept[i]);
    CHECK(console_moved_to(NULL) == NULL, "NULL");

    size_t n = console_moved_line("la-voltage", line, sizeof(line));
    CHECK(n == strlen(line) && n > 0, "length %zu", n);
    const char *head = "la-voltage: moved to the network API (JSON la_voltage)";
    CHECK(!strncmp(line, head, strlen(head)), "line: %s", line);
    CHECK(strstr(line, "0.6.0") && strstr(line, "2.12.0"), "names the versions: %s", line);
    /* an old CLI treats "unknown command" as "this firmware is too old"; the line must not say it */
    CHECK(!strstr(line, "unknown command"), "no unknown command: %s", line);
    CHECK(console_moved_line("status", line, sizeof(line)) == 0 && line[0] == '\0', "not moved");
    CHECK(console_moved_line("power", line, 20) == 19 && strlen(line) == 19, "cut to fit: %s", line);

    if (s_fail) { printf("test_console_moved: %d FAILED\n", s_fail); return 1; }
    printf("test_console_moved: all passed\n");
    return 0;
}
