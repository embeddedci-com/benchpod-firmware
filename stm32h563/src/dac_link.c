/* dac_link.c — see dac_link.h. */
#include "dac_link.h"
#include "bp_json.h"
#include "cmd_opcodes.h"

#include <string.h>

bool dac_link_mismatch(bool link_board, bool present, bool digital_swap, bool module_swap)
{
    return link_board && present && digital_swap != module_swap;
}

/* A JSON value that is present and not false/0 (bp_json_get leaves strings unquoted). */
static bool truthy(const char *json, const char *key)
{
    char v[16] = {0};
    if (!json || !bp_json_get(json, key, v, sizeof(v)) || !v[0]) return false;
    return strcmp(v, "false") != 0 && strcmp(v, "0") != 0 && strcmp(v, "null") != 0;
}

static bool has(const char *json, const char *key)
{
    char v[16] = {0};
    return json && bp_json_get(json, key, v, sizeof(v));
}

bool dac_link_command_drives_dac(const char *cmd, const char *json)
{
    if (!cmd) return false;
    static const char *const always[] = {
        "generate", "measure", "replay", "dac_set", "dac_out", "dac_control_loop",
    };
    for (size_t i = 0; i < sizeof(always) / sizeof(always[0]); i++)
        if (!strcmp(cmd, always[i])) return true;
    if (!strcmp(cmd, "current_out")) return has(json, "ua");          /* bare: reads the range */
    if (!strcmp(cmd, "calibrate")) return has(json, "dac") || has(json, "adc");   /* not j8 */
    if (!strcmp(cmd, "adc_range")) return truthy(json, "test");       /* drives the 5 V output */
    return false;
}

bool dac_link_opcode_drives_dac(uint8_t opcode)
{
    switch (opcode) {
    case CMD_START_DAC:
    case CMD_START_DAC_PSRAM:
    case CMD_START_DAC_LOOP:
    case CMD_DAC_CTRL:
        return true;
    default:
        return false;
    }
}
