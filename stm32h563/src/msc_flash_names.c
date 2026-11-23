/* msc_flash_names.c — the names msc_flash.h reports, and 8.3 names. Internal flash (the command
   replies use them; internal code may not call into the W25Q image). */
#include "msc_flash.h"

#include <string.h>

const char *msc_flash_rc_str(int rc)
{
    switch (rc) {
    case MSC_FLASH_RUNNING:   return "running";
    case MSC_FLASH_DONE:      return "ok";
    case MSC_FLASH_E_IO:      return "the drive did not take a sector read or write";
    case MSC_FLASH_E_SRC:     return "reading the uploaded file failed";
    case MSC_FLASH_E_NOT_UF2: return "the file is not a UF2 file";
    case MSC_FLASH_E_SECTOR:  return "the drive's sectors are not 512 bytes";
    case MSC_FLASH_E_FS:      return "the drive has no FAT12, FAT16, FAT32 or exFAT file system";
    case MSC_FLASH_E_FULL:    return "the drive has no room for the file";
    case MSC_FLASH_E_ARGS:    return "bad mode, name or length";
    case MSC_FLASH_E_ABORTED: return "aborted";
    case MSC_FLASH_E_EXFAT:   return "reading back from an exFAT drive is not supported";
    case MSC_FLASH_E_VERIFY:  return "the drive's flash contents differ from the file";
    case MSC_FLASH_E_NOFILE:  return "the drive has no CURRENT.UF2";
    case MSC_FLASH_E_LOADER:  return "the loader reported a failure";
    default:                  return "?";
    }
}

const char *msc_flash_phase_str(int phase)
{
    static const char *const k[] = { "start", "uf2", "directory", "free space", "data", "fat",
                                      "entry", "done", "new directory block", "old file", "fsinfo",
                                      "find file", "read file", "verify", "wait for drive",
                                      "bitmap", "fat", "old file", "entry", "old file" };
    return (phase >= 0 && phase < (int)(sizeof(k) / sizeof(k[0]))) ? k[phase] : "?";
}

bool msc_flash_name83(const char *name, char out[11])
{
    memset(out, ' ', 11);
    if (!name || !name[0]) return false;
    const char *dot = strrchr(name, '.');
    size_t base = dot ? (size_t)(dot - name) : strlen(name);
    size_t ext = dot ? strlen(dot + 1) : 0;
    if (base == 0 || base > 8 || ext > 3) return false;
    for (size_t i = 0; i < base + (dot ? 1 + ext : 0); i++) {
        char ch = name[i];
        if (dot && name + i == dot) continue;
        if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
        if (!((ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '_' || ch == '-')) return false;
        if (name + i < (dot ? dot : name + base)) out[i] = ch;
        else out[8 + (size_t)(name + i - dot - 1)] = ch;
    }
    return true;
}
