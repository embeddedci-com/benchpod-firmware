/* ctrl_flash_names.c — the names ctrl_flash.h reports. Internal flash (command replies). */
#include "ctrl_flash.h"

const char *ctrl_flash_rc_str(int rc)
{
    switch (rc) {
    case CTRL_FLASH_RUNNING:      return "running";
    case CTRL_FLASH_DONE:         return "ok";
    case CTRL_FLASH_E_IO:         return "a USB control transfer failed";
    case CTRL_FLASH_E_SRC:        return "reading the uploaded file failed";
    case CTRL_FLASH_E_NO_DFU:     return "the board has no DFU interface in DFU mode";
    case CTRL_FLASH_E_STATUS:     return "the DFU loader reported an error";
    case CTRL_FLASH_E_LAYOUT:     return "the image does not fit the loader's flash layout";
    case CTRL_FLASH_E_NOT_TEENSY: return "the board is not a Teensy in its loader";
    case CTRL_FLASH_E_TIMEOUT:    return "the loader stayed busy";
    case CTRL_FLASH_E_ARGS:       return "bad length, address or block size";
    default:                      return "?";
    }
}

const char *ctrl_flash_phase_str(int phase)
{
    static const char *const k[] = { "start", "descriptors", "layout", "erase", "address", "write",
                                      "leave", "done" };
    return (phase >= 0 && phase < (int)(sizeof(k) / sizeof(k[0]))) ? k[phase] : "?";
}
