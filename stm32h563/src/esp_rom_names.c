/* esp_rom_names.c — the names esp_rom_job.h reports (result, chip, phase). Internal flash: the
   command replies and the console use them; internal code may not call into the W25Q image. */
#include "esp_rom_job.h"

const char *esp_rom_result_str(int result)
{
    switch (result) {
    case ESP_ROM_OK:         return "verified";
    case ESP_ROM_UNVERIFIED: return "written, MD5 not read back";
    case ESP_ROM_ERR_SYNC:   return "no answer from the ROM loader";
    case ESP_ROM_ERR_BEGIN:  return "FLASH_BEGIN failed";
    case ESP_ROM_ERR_READ:   return "reading the image failed";
    case ESP_ROM_ERR_DATA:   return "a block was not acknowledged";
    case ESP_ROM_ERR_MD5:    return "MD5 mismatch";
    case ESP_ROM_ERR_CHIP:   return "chip not supported (ESP8266)";
    case ESP_ROM_ERR_BAUD:   return "could not change the baud rate";
    default:                 return "bad arguments";
    }
}

const char *esp_rom_chip_name(uint32_t magic)
{
    switch (magic) {
    case ESP_MAGIC_ESP32:   return "ESP32";
    case 0x000007C6u:       return "ESP32-S2";
    case 0x00000009u:       return "ESP32-S3";
    case 0x6921506Fu: case 0x1B31506Fu: case 0x4881606Fu: case 0x4361606Fu:
                            return "ESP32-C3";
    case 0x6F51306Fu: case 0x7C41A06Fu:
                            return "ESP32-C2";
    case 0x2CE0806Fu:       return "ESP32-C6";
    case 0xD7B73E80u:       return "ESP32-H2";
    case ESP_MAGIC_ESP8266: return "ESP8266";
    default:                return "unknown";
    }
}

const char *esp_rom_phase_str(esp_rom_phase_t phase)
{
    switch (phase) {
    case ESP_ROM_IDLE:   return "idle";
    case ESP_ROM_SYNC:   return "sync";
    case ESP_ROM_MAGIC:  return "detect";
    case ESP_ROM_BAUD:   return "baud";
    case ESP_ROM_ATTACH: return "attach";
    case ESP_ROM_PARAMS: return "params";
    case ESP_ROM_BEGIN:  return "erase";
    case ESP_ROM_DATA:   return "write";
    case ESP_ROM_MD5:    return "verify";
    case ESP_ROM_END:    return "end";
    default:             return "done";
    }
}
