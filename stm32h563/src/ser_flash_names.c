/* ser_flash_names.c — the names ser_flash.h reports. Internal flash (command replies). */
#include "ser_flash.h"

const char *ser_flash_rc_str(int rc)
{
    switch (rc) {
    case SER_FLASH_RUNNING:       return "running";
    case SER_FLASH_DONE:          return "ok";
    case SER_FLASH_E_IO:          return "the USB serial port failed or went away";
    case SER_FLASH_E_SRC:         return "reading the uploaded file failed";
    case SER_FLASH_E_NO_LOADER:   return "no bootloader answered";
    case SER_FLASH_E_REPLY:       return "the bootloader answered something unexpected";
    case SER_FLASH_E_VERIFY:      return "what the board holds is not the image";
    case SER_FLASH_E_FIT:         return "the image does not fit the chip or would overwrite the bootloader";
    case SER_FLASH_E_REATTACH:    return "the board did not come back as its bootloader";
    case SER_FLASH_E_ARGS:        return "bad length or address";
    case SER_FLASH_E_UNSUPPORTED: return "the bootloader lacks the fast write commands";
    default:                      return "?";
    }
}

const char *ser_flash_phase_str(int phase)
{
    static const char *const k[] = { "start", "touch", "reattach", "reset", "sync", "info", "erase",
                                      "write", "verify", "leave", "done" };
    return (phase >= 0 && phase < (int)(sizeof(k) / sizeof(k[0]))) ? k[phase] : "?";
}

const char *ser_flash_proto_str(int proto)
{
    switch (proto) {
    case SER_PROTO_AVR109:   return "avr109";
    case SER_PROTO_STK500V1: return "stk500v1";
    case SER_PROTO_STK500V2: return "stk500v2";
    case SER_PROTO_SAMBA:    return "sam-ba";
    default:                 return "";
    }
}
