/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 LoBo (https://github.com/loboris)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
    Boot configuration used by Kboot is stored in one SPI Flash sector (4KB) at fixed Flash address (0x00004000).
    One backup configuration sector is also used for security reasons,
    stored in one SPI Flash sector (4KB) at fixed Flash address (0x00005000).

    The configuration sector consists of 8 application entries occupying 32 bytes each.
    After the last application entry the config flags entry is placed, one 32bit value.

    Configuration sector layout:

    | ---------------------------------------------- |
    | From   | To    | Length | Comment              |
    | ---------------------------------------------- |
    | 0x000  | 0x01F | 32     | application entry #0 |
    | 0x020  | 0x03F | 32     | application entry #1 |
    | 0x040  | 0x05F | 32     | application entry #2 |
    | 0x060  | 0x07F | 32     | application entry #3 |
    | 0x080  | 0x09F | 32     | application entry #4 |
    | 0x0A0  | 0x0BF | 32     | application entry #5 |
    | 0x0C0  | 0x0DF | 32     | application entry #6 |
    | 0x0E0  | 0x0FF | 32     | application entry #7 |
    | 0x100  | 0x103 |  4     | config flags         |
    | 0x104  | 0x11F |  4     | reserved             |
    | 0x120  | 0x11F |  4     | not used, user data  |
    | ---------------------------------------------- |

    The format of each _application entry_ in configuration sector is as follows:

    | ------------------------------------------------------------------------------------------------- |
    | Offset | Length | Comment                                                                         |
    | ------------------------------------------------------------------------------------------------- |
    |  0     |  4     | Configuration entry ID (bits 4-31) + entry flags (bits 0-4)                     |
    |  4     |  4     | Application address in SPI Flash                                                |
    |  8     |  4     | Application size in SPI Flash. This is the size of the application's *.bin file |
    | 12     |  4     | Application's CRC32 value (32bits)                                              |
    | 16     | 16     | Null terminated application name or description                                 |
    | ------------------------------------------------------------------------------------------------- |

    Notes:
    Configuration entry ID must have a value of 0x5AA5D0C0 for entry to be recognized as valid.
    Application addres must be in range 0x10000 ~ 0x800000 ( 64KB ~ 8MB ).
    Application size must be in range 0x4000 ~ 0x300000 ( 16KB ~ 3MB ).
    Application CRC32 value is used only if CRC32 flag is set.

    If config flags at offset 0x100 in configuration sector is set to configuration entry ID (0x5AA5D0C0),
    interractive mode will be disabled, boot Pin will not be checked and nothing will be printed during the boot process.

    Configuration entry flags:

    | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
    | Bit | Comment                                                                                                                                                      |
    | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
    | 0 | 'Active' flag, if set the application will be loaded and executed.
    |                    If multiple entries have active flag set, the first one will be loaded ad executed                                                              |
    | 1 |  'CRC32' flag, if set the application's CRC32 value will be calculated and compared with the value in the configuration entry                                  |
    | 2 | 'AES256' flag, if set the application's AES256 hash value will be calculated and compared with the value stored in flash after the application code (SHA_HASH) |
    | 3 |   'Size' flag, if set the application size specified in configuration entry must match the application size present in application's Flash block               |
    | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |

    Warning: all 32-bit values in the configuration sector entries must be written in big-endian format.
 */

/*
    Each application stored in SPI Flash has the following format:

    | ----------------------------------------------------------------------------- |
    | Byte offset  | Size     | Content                                             |
    | ----------------------------------------------------------------------------- |
    | 0            | 1        | AES cipher flag, for use with **Kboot** must be `0` |
    | 1            | 4        | Application code size                               |
    | 5            | APP_SIZE | Application code                                    |
    | APP_SIZE + 5 | 32       | Application SHA256 hash                             |
    | ----------------------------------------------------------------------------- |

    All applications flashed with **ktool.py** or **kflash.py** have such format.
 */

#include "mpconfigport.h"

#if MICROPY_PY_USE_OTA

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "w25qxx.h"
#include "sha256_hard.h"
#include "spi.h"
#include "syslog.h"
#include "mphalport.h"

#include "py/runtime.h"
#include "extmod/vfs.h"
#include "py/stream.h"


#define BOOT_CONFIG_ADDR        0x00004000  // main boot config sector in flash at 52K
#define BOOT_CONFIG_ITEMS       8           // number of handled config entries
#define BOOT_CONFIG_ITEM_SIZE   32          // size of config entry
#define BOOT_CONFIG_SECTOR_SIZE 4096
#define BOOT_MAIN_SECTOR        1
#define BOOT_BACKUP_SECTOR      2

#define MAGIC_ID                0x5AA5D0C0
#define MAGIC_ID_FLAG           0xA55A60C0
#define MAGIC_ID_MASK           0xFFFFFFF0
#define CFG_APP_FLAG_ACTIVE     0x00000001
#define CFG_APP_FLAG_CRC32      0x00000002
#define CFG_APP_FLAG_SHA256     0x00000004
#define CFG_APP_FLAG_SIZE       0x00000008

#define DEFAULT_APP_ADDRESS     0x00010000
#define BOOT_ENTRY_NAME_LEN     16


typedef struct _ota_entry_t {
    uint32_t id_flags;
    uint32_t address;
    uint32_t size;
    uint32_t crc32;
    char     name[BOOT_ENTRY_NAME_LEN];
} __attribute__((packed, aligned(4))) ota_entry_t;


static uint8_t config_sector[BOOT_CONFIG_SECTOR_SIZE];
static uint8_t config_loaded = 0;
static const char* TAG = "[OTA]";

/*
//-------------------------------------------------
static void _swap_endianess(uint8_t *buff, int len)
{
    uint32_t val_in, val;
    for (int i=0; i<len; i++) {
        val_in = *((uint32_t *)(buff + (i*4)));
        val = (val_in >> 24) & 0x000000ff;
        val |= (val_in >> 8) & 0x0000ff00;
        val |= (val_in << 8) & 0x00ff0000;
        val |= (val_in << 24) & 0xff000000;
        *((uint32_t *)(buff + (i*4))) = val;
    }
}
*/

/*
 * Get uint32_t value from Flash
 * 8-bit Flash pointer is used, so the byte order must be swapped
 * This is used when reading from not 4-byte aligned locations
 * XIP must be already enabled!
 */
//-----------------------------------------
static uint32_t flash2uint32(uint32_t addr)
{
    uint32_t val = w25qxx_flash_ptr[addr];
    val += w25qxx_flash_ptr[addr+1] << 8;
    val += w25qxx_flash_ptr[addr+2] << 16;
    val += w25qxx_flash_ptr[addr+3] << 24;
    return val;
}

//--------------------------------------------
static uint32_t swap_endian32(uint32_t val_in)
{
    uint32_t val = (val_in >> 24) & 0x000000ff;
    val |= (val_in >> 8) & 0x0000ff00;
    val |= (val_in << 8) & 0x00ff0000;
    val |= (val_in << 24) & 0xff000000;
    return val;
}

// Swap endianness of 32-bit values in currently loaded config sector
//---------------------------------
static void config_swap_endianess()
{
    // Swap endianness of all 32-bit entries
    ota_entry_t *entry;
    for (int i=0; i <= BOOT_CONFIG_ITEMS; i++) {
        entry = (ota_entry_t *)(config_sector + (i*BOOT_CONFIG_ITEM_SIZE));
        entry->id_flags = swap_endian32(entry->id_flags);
        entry->address = swap_endian32(entry->address);
        entry->size = swap_endian32(entry->size);
        entry->crc32 = swap_endian32(entry->crc32);
    }
}

// Read config sector into buffer
//------------------------------------------------
static bool read_config(uint8_t n_conf, bool swap)
{
    if (config_loaded == n_conf) return true;

    config_loaded = 0;
    enum w25qxx_status_t res;
    bool curr_spi_check = w25qxx_spi_check;
    w25qxx_spi_check = true;
    res = w25qxx_read_data(BOOT_CONFIG_ADDR + ((n_conf-1) * BOOT_CONFIG_SECTOR_SIZE ), config_sector, BOOT_CONFIG_SECTOR_SIZE);
    w25qxx_spi_check = curr_spi_check;

    if (res != W25QXX_OK) return false;

    if (swap) config_swap_endianess();

    config_loaded = n_conf;
    return true;
}

// Backup main config sector to backup config sector
//------------------------------
static bool backup_boot_sector()
{
    bool curr_spi_check = w25qxx_spi_check;
    w25qxx_spi_check = true;
    config_loaded = 0; // force load
    if (!read_config(BOOT_MAIN_SECTOR, false)) {
        w25qxx_spi_check = curr_spi_check;
        LOGW(TAG, "Error reading main config sector");
        return false;
    }

    enum w25qxx_status_t res;
    res = w25qxx_write_data(BOOT_CONFIG_ADDR + BOOT_CONFIG_SECTOR_SIZE, config_sector, BOOT_CONFIG_SECTOR_SIZE);
    config_swap_endianess();

    if (res != W25QXX_OK) {
        w25qxx_spi_check = curr_spi_check;
        LOGW(TAG, "Error saving backup config sector");
        return false;
    }
    w25qxx_spi_check = curr_spi_check;
    return true;
}

// Restore main config sector from backup
//-------------------------------
static bool restore_boot_sector()
{
    bool curr_spi_check = w25qxx_spi_check;
    w25qxx_spi_check = true;
    config_loaded = 0; // force load

    if (!read_config(BOOT_BACKUP_SECTOR, false)) {
        w25qxx_spi_check = curr_spi_check;
        return false;
    }

    enum w25qxx_status_t res;
    res = w25qxx_write_data(BOOT_CONFIG_ADDR, config_sector, BOOT_CONFIG_SECTOR_SIZE);
    if (res != W25QXX_OK) {
        w25qxx_spi_check = curr_spi_check;
        config_loaded = 0;
        return false;
    }
    w25qxx_spi_check = curr_spi_check;

    config_swap_endianess();
    config_loaded = BOOT_MAIN_SECTOR;

    return true;
}

// Write modified boot sector from buffer to Flash
//-----------------------------
static bool write_boot_sector()
{
    // Save modified boot sector
    config_swap_endianess();
    config_loaded = 0;
    bool curr_spi_check = w25qxx_spi_check;
    w25qxx_spi_check = true;
    enum w25qxx_status_t res = w25qxx_write_data(BOOT_CONFIG_ADDR, config_sector, BOOT_CONFIG_SECTOR_SIZE);
    w25qxx_spi_check = curr_spi_check;
    if (res != W25QXX_OK) {
        LOGW(TAG, "Error writing config sector");
        // Try to restore backup sector
        bool f = restore_boot_sector();
        if (f) {
            LOGW(TAG, "Config sector restored");
        }
        else {
            LOGE(TAG, "Config sector NOT restored");
        }
        return false;
    }
    return true;
}

/*
 * Calculate the firmware's SHA256 hash and get the stored SHA256
 * Complete firmware, including 5-byte prefix and 32-byte SHA hash, is expected at flash address
 * The hash is 32 bytes long and written after the firmware's code
 * The hash is calculated over 5-byte prefix and firmware's code !
 * by Ktool or other application
 */
//-----------------------------------------------------------------------------
static void calc_app_sha256(uint32_t address, uint8_t *hash, uint8_t *app_hash)
{
    w25qxx_enable_xip_mode();

    memset(hash, 0, SHA256_HASH_LEN);
    memset(app_hash, 0, SHA256_HASH_LEN);

    sha256_hard_context_t context;
    uint8_t buffer[1024] = {0};
    int size = flash2uint32(address+1) + 5;
    int sz;
    uint32_t idx = 0;
    uint32_t hash_offset = address + size;

    sha256_hard_init(&context, size);

    while (size > 0) {
        sz = (size >= 1024) ? 1024 : size;
        for (int n=0; n<sz; n++) {
            buffer[n] = w25qxx_flash_ptr[address + idx + n];
        }
        sha256_hard_update(&context, buffer, sz);
        idx += sz;
        size -= sz;
    }

    sha256_hard_final(&context, hash);

    // get the application's SHA256 hash
    for (int n=0; n<SHA256_HASH_LEN; n++) {
        app_hash[n] = w25qxx_flash_ptr[hash_offset + n];
    }

    w25qxx_disable_xip_mode();
}

/*
 * Check the firmware's SHA256 hash
 * Complete firmware, including 5-byte prefix and 32-byte SHA hash, is expected at flash address
 * The hash is 32 bytes long and written after the firmware's code
 * The hash is calculated over 5-byte prefix and firmware's code !
 * by Ktool or other application
 */
//--------------------------------------------
static bool check_app_sha256(uint32_t address)
{
    uint8_t hash[SHA256_HASH_LEN];
    uint8_t app_hash[SHA256_HASH_LEN];

    calc_app_sha256(address, hash, app_hash);

    // Check the hashes
    for (int idx=0; idx<SHA256_HASH_LEN; idx++) {
        if (hash[idx] != app_hash[idx]) {
            /*
            char hashhex[SHA256_HASH_LEN*2+1];
            for (int i=0; i<SHA256_HASH_LEN; i++) {
                sprintf(hashhex+(i*2), "%02X", hash[i]);
            }
            printf("[%s] <>\r\n", hashhex);
            for (int i=0; i<SHA256_HASH_LEN; i++) {
                sprintf(hashhex+(i*2), "%02X", app_hash[i]);
            }
            printf("[%s]\r\n", hashhex);
            */
            return false;
        }
    }
    return true;
}

// Calculate firmwares's CRC32 value
//-------------------------------------------------------------
static uint32_t calc_app_crc32(uint32_t address, uint32_t size)
{
    uint32_t crc = 0xFFFFFFFF;
    uint32_t byte, mask;

    w25qxx_enable_xip_mode();

    // Read from flash and update CRC32 value
    for (uint32_t n = 5; n < (size+5); n++) {
        byte = w25qxx_flash_ptr[address + n];
        crc = crc ^ byte;
        for (uint32_t j = 0; j < 8; j++) {
            mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320 & mask);
        }
    }

    w25qxx_disable_xip_mode();

    return (~crc);
}

// Check the firmware's CRC32 value
//--------------------------------------------------------
static bool app_crc32(ota_entry_t *entry, uint32_t *crc32)
{
    *crc32 = calc_app_crc32(entry->address, entry->size);
    if (*crc32 != entry->crc32) return false;
    return true;
}

// Get the size of the firmware at Flash address
//--------------------------------------------------
static uint32_t get_fw_flash_size(uint32_t address)
{
    w25qxx_enable_xip_mode();
    uint32_t app_size = flash2uint32(address+1);
    w25qxx_disable_xip_mode();
    return app_size;
}

// Check the config sector entry
//----------------------------------------------------------
static uint8_t check_entry(ota_entry_t *entry, char *status)
{
    uint8_t stat = 0;
    if (status) status[0] = '\0';
    if ((entry->id_flags & MAGIC_ID_MASK) != MAGIC_ID) return false;

    uint32_t app_size = get_fw_flash_size(entry->address);
    uint8_t flags = entry->id_flags & 0x0f;

    if (app_size != entry->size) {
        if (status) strcat(status, "Size mismatch! ");
        stat |= 1;
    }
    if (flags & CFG_APP_FLAG_SIZE) {
        if (app_size != entry->size) {
            stat |= 2;
            return stat;
        }
    }

    if (flags & CFG_APP_FLAG_CRC32) {
        uint32_t crc32;
        if (!app_crc32(entry, &crc32)) {
            if (status) strcat(status, "CRC32 Error ");
            stat |= 4;
            return stat;
        }
    }
    if (flags & CFG_APP_FLAG_SHA256) {
        if (!check_app_sha256(entry->address)) {
            if (status) strcat(status, "SHA256 Error ");
            stat |= 8;
            return stat;
        }
    }
    if (status) strcat(status, "Check OK ");

    return stat;
}

// Copy the firmware from one Flash address to another
// Complete firmware, including 5-byte prefix and 32-byte SHA hash, is copied
// We assume that the source and destination address are aligned to 4K
//----------------------------------------------------------------------------------
static bool firmware_copy(uint32_t src, uint32_t dest, uint32_t size, bool progress)
{
    uint8_t buffer[w25qxx_FLASH_SECTOR_SIZE];
    enum w25qxx_status_t res;
    uint32_t idx=0, sz=0;
    uint32_t total_size = size + 37; // add 5 byte prefix and 32 byte sha256 hash
    float prog = 0.0;
    // Align the size to sector size (4K)
    if (total_size % w25qxx_FLASH_SECTOR_SIZE) {
        total_size /= w25qxx_FLASH_SECTOR_SIZE;
        total_size *= w25qxx_FLASH_SECTOR_SIZE;
        total_size += w25qxx_FLASH_SECTOR_SIZE;
    }
    LOGD(TAG, "Firmware copy: total_size=%u (0x%08X, sectors=%u [%04X])",
            total_size, total_size, total_size/w25qxx_FLASH_SECTOR_SIZE, total_size/w25qxx_FLASH_SECTOR_SIZE);
    uint32_t copy_size = total_size;

    bool curr_spi_check = w25qxx_spi_check;
    w25qxx_spi_check = true;

    while (total_size > 0) {
        sz = (total_size > w25qxx_FLASH_SECTOR_SIZE) ? w25qxx_FLASH_SECTOR_SIZE : total_size;
        res = w25qxx_read_data(src + idx, buffer, sz);
        if (res != W25QXX_OK) break;

        res = w25qxx_write_data(dest + idx, buffer, sz);
        if (res != W25QXX_OK) break;

        if (progress) {
            prog = ((float)(copy_size - total_size) / (float)copy_size) * 100.0;
            mp_printf(&mp_plat_print, "%.2f%%  \r", prog);
        }

        total_size -= sz;
        idx += sz;
        mp_hal_wdt_reset();
    }
    w25qxx_spi_check = curr_spi_check;
    if (progress) {
        mp_printf(&mp_plat_print, "%s\r\n", (total_size == 0) ? "Finished" : "Failed  ");
    }

    if (total_size > 0) return false;
    return true;
}

// Write the firmware from file to Flash
// We assume that the destination address is aligned to 4K
//-----------------------------------------------------------------------------------
static bool firmware_write(mp_obj_t ffd, uint32_t dest, uint32_t size, bool progress)
{
    uint8_t buffer[w25qxx_FLASH_SECTOR_SIZE] = {0xFF};
    uint8_t fwhash[SHA256_HASH_LEN];
    sha256_hard_context_t context;
    enum w25qxx_status_t res;
    uint32_t idx=0, sz=0, rd;
    uint32_t to_read = size;
    float prog = 0.0;
    bool first_sect = true;
    bool f;

    LOGD(TAG, "Firmware write: file_size=%u (0x%08X, sectors=%u [%04X])",
            size, size, size/w25qxx_FLASH_SECTOR_SIZE, size/w25qxx_FLASH_SECTOR_SIZE);
    uint32_t copy_size = size;

    sha256_hard_init(&context, size+5); // init sha256 calculation
    bool curr_spi_check = w25qxx_spi_check;
    w25qxx_spi_check = true;

    while (to_read > 0) {
        f = true;
        if (first_sect) {
            first_sect = false;
            buffer[0] = 0;
            *(uint32_t *)(buffer+1) = size;
            sz = w25qxx_FLASH_SECTOR_SIZE - 5;
            rd = mp_stream_posix_read((void *)ffd, buffer+5, sz);
            f = (rd == sz);
            to_read -= sz;
            if (f) sha256_hard_update(&context, buffer, sz+5);
        }
        else {
            sz = (to_read > w25qxx_FLASH_SECTOR_SIZE) ? w25qxx_FLASH_SECTOR_SIZE : to_read;
            if (sz < w25qxx_FLASH_SECTOR_SIZE) memset(buffer, 0xFF, w25qxx_FLASH_SECTOR_SIZE);
            rd = mp_stream_posix_read((void *)ffd, buffer, sz);
            f = (rd == sz);
            if (f) {
                sha256_hard_update(&context, buffer, sz);
                to_read -= sz;
                if (to_read == 0) {
                    // Last data, add SHA256 hash
                    LOGD(TAG, "FW write: Last sector %u (%08X), size=%d", idx, idx, sz);
                    sha256_hard_final(&context, fwhash);
                    if ((sz + SHA256_HASH_LEN) > w25qxx_FLASH_SECTOR_SIZE) {
                        LOGE(TAG, "FW write: No space for SHA32 hash at %u (%d)", idx, w25qxx_FLASH_SECTOR_SIZE-sz-SHA256_HASH_LEN);
                        f = false;
                        break;
                    }
                    else memcpy(buffer+sz, fwhash, SHA256_HASH_LEN);
                }
            }
        }
        if (!f) {
            LOGD(TAG, "FW write: read error at %u (%d <> %d)", idx, rd, sz);
            break;
        }

        // Write the full sector from 'buffer' to the Flash
        res = w25qxx_write_data(dest + idx, buffer, w25qxx_FLASH_SECTOR_SIZE);
        if (res != W25QXX_OK) {
            LOGD(TAG, "FW write: write error at %u (size=%d)", idx, sz);
            f = false;
            break;
        }

        if (progress) {
            prog = ((float)(copy_size - to_read) / (float)copy_size) * 100.0;
            mp_printf(&mp_plat_print, "%08X: %.2f%%  \r", idx, prog);
        }

        idx += w25qxx_FLASH_SECTOR_SIZE; // next sector
        mp_hal_wdt_reset();
    }

    if (progress) {
        if (f) mp_printf(&mp_plat_print, "%08X: 100%%  \r\n", idx);
        else mp_printf(&mp_plat_print, "\r\n");
    }
    w25qxx_spi_check = curr_spi_check;
    if (progress) {
        mp_printf(&mp_plat_print, "%s\r\n", (f) ? "Finished" : "Failed");
    }

    return f;
}

// Get the 1st active firmware from the main config sector
//----------------------------
static int config_get_active()
{
    if (!read_config(BOOT_MAIN_SECTOR, true)) return -1;

    ota_entry_t *entry;
    uint8_t stat, flags;

    for (int i=0; i < BOOT_CONFIG_ITEMS; i++) {
        entry = (ota_entry_t *)(config_sector + (i*BOOT_CONFIG_ITEM_SIZE));
        if ((entry->id_flags & MAGIC_ID_MASK) == MAGIC_ID) {
            flags = entry->id_flags & 0x0f;
            if (flags & CFG_APP_FLAG_ACTIVE) {
                stat = check_entry(entry, NULL);
                if ((stat&0x7E) == 0) return i;
            }
        }
    }
    return -1;
}

// Set the active main config sector entry
// Other entries, if active, are marked as inactive
//--------------------------------------------
static bool config_set_active(uint8_t n_entry)
{
    if (!backup_boot_sector()) return false;

    ota_entry_t *entry;
    uint8_t stat, flags;
    bool f = false;
    char status[32];

    for (int i=0; i < BOOT_CONFIG_ITEMS; i++) {
        entry = (ota_entry_t *)(config_sector + (i*BOOT_CONFIG_ITEM_SIZE));
        if ((entry->id_flags & MAGIC_ID_MASK) == MAGIC_ID) {
            flags = entry->id_flags & 0x0f;
            if (i == n_entry) {
                if ((flags & CFG_APP_FLAG_ACTIVE) == 0) {
                    stat = check_entry(entry, status);
                    if ((stat&0x7E) != 0) {
                        if (strlen(status) > 0) status[strlen(status)-1] = '\0';
                        LOGW(TAG, "Found config entry, but check failed [%s]", status);
                        break;
                    }
                    entry->id_flags |= CFG_APP_FLAG_ACTIVE;
                    f = true;
                    LOGD(TAG, "Config entry #%d set active", i);
                    break;
                }
            }
        }
    }
    if (f) {
        // mark other entries as not active
        for (int i=0; i < BOOT_CONFIG_ITEMS; i++) {
            entry = (ota_entry_t *)(config_sector + (i*BOOT_CONFIG_ITEM_SIZE));
            if ((entry->id_flags & MAGIC_ID_MASK) == MAGIC_ID) {
                flags = entry->id_flags & 0x0f;
                if ((i != n_entry) && (flags & CFG_APP_FLAG_ACTIVE)) {
                    entry->id_flags &= ~CFG_APP_FLAG_ACTIVE;
                    LOGD(TAG, "Config entry #%d set inactive", i);
                }
            }
        }
        // Save modified boot sector
        return write_boot_sector();
    }
    return false;
}

// Enable or disable the Kboot interactive mode
//------------------------------------------------------
static bool config_set_interactive(bool enable, int pin)
{
    if (!backup_boot_sector()) return false;

    uint32_t *mboot_flag = (uint32_t *)(config_sector + (BOOT_CONFIG_ITEMS*BOOT_CONFIG_ITEM_SIZE));

    if (enable) {
        if ((pin >= 0) && (pin < 48) && (pin != 16)) *mboot_flag = MAGIC_ID_FLAG | (pin & 0x3f);
        else *mboot_flag = 0;
    }
    else *mboot_flag = MAGIC_ID;

    // Save modified boot sector
    return write_boot_sector();
}

//--------------------------------------------------------------------------------------
STATIC mp_obj_t mod_ota_list(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_print, ARG_sect };
    const mp_arg_t allowed_args[] = {
       { MP_QSTR_print,      MP_ARG_BOOL, { .u_bool = false } },
       { MP_QSTR_sect,       MP_ARG_INT,  { .u_int = BOOT_MAIN_SECTOR } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    uint8_t sect = args[ARG_sect].u_int & 3;
    if ((sect != BOOT_MAIN_SECTOR) && (sect != BOOT_BACKUP_SECTOR)) {
        mp_raise_ValueError("Wrong boot sector requested");
    }

    if (!read_config(sect, true)) {
        mp_raise_msg(&mp_type_OSError, "Error reading config sector");
    }

    uint8_t flags;
    ota_entry_t *entry;
    char sflags[32];
    char status[32];
    uint8_t stat;
    int n_entries = 0;
    mp_obj_t entry_tuple[6];

    for (int i=0; i < (BOOT_CONFIG_ITEMS-1); i++) {
        entry = (ota_entry_t *)(config_sector + (i*BOOT_CONFIG_ITEM_SIZE));
        if ((entry->id_flags & MAGIC_ID_MASK) == MAGIC_ID) n_entries++;
    }
    mp_obj_t config_tuple[n_entries+1];
    n_entries = 0;

    if (args[ARG_print].u_bool) mp_printf(&mp_plat_print, "%s boot sector entries:\r\n", (sect == BOOT_MAIN_SECTOR) ? "Main" : "Backup");

    for (int i=0; i < BOOT_CONFIG_ITEMS; i++) {
        entry = (ota_entry_t *)(config_sector + (i*BOOT_CONFIG_ITEM_SIZE));
        if (i < (BOOT_CONFIG_ITEMS-1)) {
            if ((entry->id_flags & MAGIC_ID_MASK) == MAGIC_ID) {
                flags = entry->id_flags & 0x0f;
                sflags[0] = '\0';
                if (flags & CFG_APP_FLAG_ACTIVE) strcat(sflags, "ACTIVE+");
                if (flags & CFG_APP_FLAG_CRC32) strcat(sflags, "CRC32+");
                if (flags & CFG_APP_FLAG_SHA256) strcat(sflags, "SHA256+");
                if (flags & CFG_APP_FLAG_SIZE) strcat(sflags, "SIZE+");
                if (strlen(sflags) > 0) sflags[strlen(sflags)-1] = '\0';
                stat = check_entry(entry, status);
                if (strlen(status) > 0) status[strlen(status)-1] = '\0';

                entry_tuple[0] = mp_obj_new_int(i);
                entry_tuple[1] = mp_obj_new_int(flags);
                entry_tuple[2] = mp_obj_new_int(entry->address);
                entry_tuple[3] = mp_obj_new_int(entry->size);
                entry_tuple[4] = mp_obj_new_str(entry->name, strlen(entry->name));
                entry_tuple[5] = mp_obj_new_int(stat);

                config_tuple[n_entries] = mp_obj_new_tuple(6, entry_tuple);
                n_entries++;
                if (args[ARG_print].u_bool)
                    mp_printf(&mp_plat_print, "%d: flags=[%s], addr=0x%08X, size=%u (%u), '%s', %s\r\n",
                            i, sflags, entry->address, entry->size, get_fw_flash_size(entry->address), entry->name, status);
            }
        }
        else {
            config_tuple[n_entries] = mp_obj_new_bool( ((entry->id_flags & MAGIC_ID_MASK) != MAGIC_ID) );
            if (args[ARG_print].u_bool) {
                if ((entry->id_flags & MAGIC_ID_MASK) == MAGIC_ID)
                    mp_printf(&mp_plat_print, "Interactive mode disabled\r\n", entry->id_flags);
                else
                    mp_printf(&mp_plat_print, "Interactive mode enabled\r\n", entry->id_flags);
            }
        }
    }
    return mp_obj_new_tuple(n_entries+1, config_tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_ota_list_obj, 0, mod_ota_list);

//---------------------------------------------------------------------------------------
STATIC mp_obj_t mod_ota_clone(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_dest, ARG_address, ARG_active, ARG_progress };
    const mp_arg_t allowed_args[] = {
       { MP_QSTR_dest,       MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
       { MP_QSTR_address,    MP_ARG_REQUIRED | MP_ARG_INT, { .u_int = 0 } },
       { MP_QSTR_active,                       MP_ARG_BOOL, { .u_bool = false } },
       { MP_QSTR_progress,                     MP_ARG_BOOL, { .u_bool = false } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Read the main config sector and backup it
    if (!backup_boot_sector()) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "Error reading config sector"));
    }

    int dest = args[ARG_dest].u_int;
    if ((dest < 0) || (dest > (BOOT_CONFIG_ITEMS-1))) {
        mp_raise_ValueError("Wrong dest index");
    }
    uint32_t dest_address = (uint32_t)args[ARG_address].u_int & 0xFFFFF000;
    uint32_t src_address, src_end_address, src_size, dest_end_address;
    ota_entry_t *active_entry = NULL;

    ota_entry_t default_entry = {0};
    default_entry.id_flags = (uint32_t)(MAGIC_ID + CFG_APP_FLAG_ACTIVE + CFG_APP_FLAG_SHA256);
    default_entry.address = DEFAULT_APP_ADDRESS;
    default_entry.size = get_fw_flash_size(DEFAULT_APP_ADDRESS);
    sprintf(default_entry.name, "MicroPython");

    int src = config_get_active();
    if (src >= 0) active_entry = (ota_entry_t *)(config_sector + (src*BOOT_CONFIG_ITEM_SIZE));
    else active_entry = &default_entry;
    src_address = active_entry->address;
    src_size = get_fw_flash_size(active_entry->address);
    src_end_address = ((src_address + src_size) & 0xFFFFF000) + 0x1000;
    dest_end_address = ((dest_address + src_size) & 0xFFFFF000) + 0x1000;

    if (src == dest) {
        mp_raise_ValueError("Source and destination equal!");
    }
    // Check if valid destination Flash area is selected
    if ( ((dest_address >= MICRO_PY_FLASHFS_START_ADDRESS) || (dest_end_address >= MICRO_PY_FLASHFS_START_ADDRESS)) ||
         ((dest_address < DEFAULT_APP_ADDRESS) || (dest_end_address < DEFAULT_APP_ADDRESS)) ||
         ((dest_address >= src_address) && (dest_address <= src_end_address)) ||
         ((dest_end_address >= src_address) && (dest_end_address <= src_end_address)) ) {
        mp_raise_ValueError("Wrong destination address!");
    }

    // Copy firmware
    LOGD(TAG, "Firmware copy: %d -> %d: 0x%08X -> 0x%08X, size=%u", src, dest, src_address, dest_address, src_size);
    if (firmware_copy(src_address, dest_address, src_size, args[ARG_progress].u_bool)) {
        // Set the destination entry data
        ota_entry_t *dest_entry = (ota_entry_t *)(config_sector + (dest*BOOT_CONFIG_ITEM_SIZE));
        dest_entry->id_flags = (uint32_t)(MAGIC_ID + CFG_APP_FLAG_SHA256 + ((args[ARG_active].u_bool) ? CFG_APP_FLAG_ACTIVE : 0));
        dest_entry->address = dest_address;
        dest_entry->size = src_size;
        dest_entry->crc32 = 0;
        memcpy(dest_entry->name, active_entry->name, BOOT_ENTRY_NAME_LEN);
        LOGD(TAG, "Adding boot entry #%d: %08X, %08X, %u", dest, dest_entry->id_flags, dest_entry->address, dest_entry->size);

        if (!write_boot_sector()) {
            mp_raise_msg(&mp_type_OSError, "Error saving config sector.");
        }
        LOGD(TAG, "Boot entry #%d saved.", dest);
    }
    else {
        mp_raise_msg(&mp_type_OSError, "Error while copying firmware.");
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_ota_clone_obj, 2, mod_ota_clone);

//---------------------------------------------------------------------------------------------
STATIC mp_obj_t mod_ota_fw_fromfile(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_dest, ARG_address, ARG_file, ARG_name, ARG_active, ARG_progress };
    const mp_arg_t allowed_args[] = {
       { MP_QSTR_dest,       MP_ARG_REQUIRED | MP_ARG_INT,  { .u_int = 0 } },
       { MP_QSTR_address,    MP_ARG_REQUIRED | MP_ARG_INT,  { .u_int = 0 } },
       { MP_QSTR_file,       MP_ARG_REQUIRED | MP_ARG_OBJ,  { .u_obj = mp_const_none } },
       { MP_QSTR_name,                         MP_ARG_OBJ,  { .u_obj = mp_const_none } },
       { MP_QSTR_active,                       MP_ARG_BOOL, { .u_bool = false } },
       { MP_QSTR_progress,                     MP_ARG_BOOL, { .u_bool = false } },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // Read the main config sector and backup it
    if (!backup_boot_sector()) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError, "Error reading config sector"));
    }

    uint32_t active_address, active_end_address, active_size, dest_end_address;
    ota_entry_t *active_entry = NULL;
    uint32_t dest_address = (uint32_t)args[ARG_address].u_int & 0xFFFFF000;
    int dest = args[ARG_dest].u_int;
    if ((dest < 0) || (dest > (BOOT_CONFIG_ITEMS-1))) {
        mp_raise_ValueError("Wrong dest index");
    }

    char entry_name[BOOT_ENTRY_NAME_LEN] = {'\0'};
    char *fname = NULL;
    mp_obj_t dest_file = mp_const_none;
    int fsize = 0;

    if (mp_obj_is_str(args[ARG_name].u_obj)) {
        char *ename = (char *)mp_obj_str_get_str(args[ARG_file].u_obj);
        snprintf(entry_name, BOOT_ENTRY_NAME_LEN, "%s", ename);
    }
    else sprintf(entry_name, "MicroPython");

    if (mp_obj_is_str(args[ARG_file].u_obj)) {
        fname = (char *)mp_obj_str_get_str(args[ARG_file].u_obj);
        mp_obj_t fargs[2];
        fargs[0] = mp_obj_new_str(fname, strlen(fname));
        fargs[1] = mp_obj_new_str("rb", 2);
        dest_file = mp_vfs_open(2, fargs, (mp_map_t*)&mp_const_empty_map);
        if (!dest_file) {
            mp_raise_msg(&mp_type_OSError, "Error opening firmware file");
        }
        // Get file size
        fsize = mp_stream_posix_lseek((void *)dest_file, 0, SEEK_END);
        int at_start = mp_stream_posix_lseek((void *)dest_file, 0, SEEK_SET);
        if ((fsize < (512*1024)) || (at_start != 0)) {
            mp_stream_close(dest_file);
            mp_raise_ValueError("Wrong file size");
        }
    }
    else {
        mp_raise_ValueError("File name not provided");
    }

    // Get current firmware information
    ota_entry_t default_entry = {0};
    default_entry.id_flags = (uint32_t)(MAGIC_ID + CFG_APP_FLAG_ACTIVE + CFG_APP_FLAG_SHA256);
    default_entry.address = DEFAULT_APP_ADDRESS;
    default_entry.size = get_fw_flash_size(DEFAULT_APP_ADDRESS);
    sprintf(default_entry.name, "MicroPython");

    int src = config_get_active();
    if (src == dest) {
        mp_raise_ValueError("Source and destination equal!");
    }

    if (src >= 0) active_entry = (ota_entry_t *)(config_sector + (src*BOOT_CONFIG_ITEM_SIZE));
    else active_entry = &default_entry;
    active_address = active_entry->address;
    active_size = get_fw_flash_size(active_entry->address);
    active_end_address = ((active_address + active_size) & 0xFFFFF000) + 0x1000;

    dest_end_address = ((dest_address + fsize) & 0xFFFFF000) + 0x1000;

    // Check if valid destination Flash area is selected
    if ( ((dest_address >= MICRO_PY_FLASHFS_START_ADDRESS) || (dest_end_address >= MICRO_PY_FLASHFS_START_ADDRESS)) ||
         ((dest_address < DEFAULT_APP_ADDRESS) || (dest_end_address < DEFAULT_APP_ADDRESS)) ||
         ((dest_address >= active_address) && (dest_address <= active_end_address)) ||
         ((dest_end_address >= active_address) && (dest_end_address <= active_end_address)) ) {
        mp_raise_ValueError("Wrong destination address!");
    }

    // Write the firmware from file
    LOGD(TAG, "Firmware write: %d: 0x%08X, size=%u", dest, dest_address, fsize);
    if (firmware_write(dest_file, dest_address, fsize, args[ARG_progress].u_bool)) {
        mp_stream_close(dest_file);
        // Set the destination entry data
        ota_entry_t *dest_entry = (ota_entry_t *)(config_sector + (dest*BOOT_CONFIG_ITEM_SIZE));
        dest_entry->id_flags = (uint32_t)(MAGIC_ID + CFG_APP_FLAG_SHA256 + ((args[ARG_active].u_bool) ? CFG_APP_FLAG_ACTIVE : 0));
        dest_entry->address = dest_address;
        dest_entry->size = fsize;
        dest_entry->crc32 = 0;
        memcpy(dest_entry->name, entry_name, BOOT_ENTRY_NAME_LEN);
        LOGD(TAG, "Adding boot entry #%d: %08X, %08X, %u", dest, dest_entry->id_flags, dest_entry->address, dest_entry->size);

        // Save modified boot sector
        if (!write_boot_sector()) {
            mp_raise_msg(&mp_type_OSError, "Error saving config sector.");
        }
        LOGD(TAG, "Boot entry #%d saved.", dest);
    }
    else {
        mp_stream_close(dest_file);
        mp_raise_msg(&mp_type_OSError, "Error while writing firmware.");
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(mod_ota_fw_fromfile_obj, 3, mod_ota_fw_fromfile);

//--------------------------------------------------
STATIC mp_obj_t mod_ota_setactive(mp_obj_t entry_in)
{
    int entry = mp_obj_get_int(entry_in);
    if ((entry < 0) || (entry > (BOOT_CONFIG_ITEMS-1))) {
        mp_raise_ValueError("Wrong config entry index");
    }

    bool res = config_set_active(entry);
    return mp_obj_new_bool(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_ota_setactive_obj, mod_ota_setactive);

//-------------------------------------------------------------------------
STATIC mp_obj_t mod_ota_setInteractive(size_t n_args, const mp_obj_t *args)
{
    bool enable = mp_obj_is_true(args[0]);
    int pin = -1;
    if (n_args > 1) {
        pin = mp_obj_get_int(args[1]);
        if ((pin < 0) || (pin >= 48) || (pin == 16)) {
            mp_raise_ValueError("Allowed boot pins: 0 ~ 15 & 17 ~47");
        }
    }

    bool res = config_set_interactive(enable, pin);
    return mp_obj_new_bool(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_ota_setInteractive_obj, 1, 2, mod_ota_setInteractive);

//===========================================================
STATIC const mp_map_elem_t ota_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_ota) },

    { MP_ROM_QSTR(MP_QSTR_list),            MP_ROM_PTR(&mod_ota_list_obj) },
    { MP_ROM_QSTR(MP_QSTR_clone),           MP_ROM_PTR(&mod_ota_clone_obj) },
    { MP_ROM_QSTR(MP_QSTR_write),           MP_ROM_PTR(&mod_ota_fw_fromfile_obj) },
    { MP_ROM_QSTR(MP_QSTR_setActive),       MP_ROM_PTR(&mod_ota_setactive_obj) },
    { MP_ROM_QSTR(MP_QSTR_setInteractive),  MP_ROM_PTR(&mod_ota_setInteractive_obj) },
};

//===========================
STATIC MP_DEFINE_CONST_DICT (
    ota_module_globals,
    ota_module_globals_table
);

const mp_obj_module_t ota_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&ota_module_globals,
};

#endif // MICROPY_PY_USE_OTA
