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

#include "sha256_hard.h"

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

#define K210_APP_ID             0xDEADBEEF
#define DEFAULT_APP_ADDRESS     0x00010000
#define BOOT_ENTRY_NAME_LEN     16


typedef struct _ota_entry_t {
    uint32_t id_flags;
    uint32_t address;
    uint32_t size;
    uint32_t crc32;
    char     name[BOOT_ENTRY_NAME_LEN];
} __attribute__((packed, aligned(4))) ota_entry_t;


extern uint8_t config_sector[BOOT_CONFIG_SECTOR_SIZE];
extern uint8_t config_loaded;

bool write_boot_sector();
bool read_config(uint8_t n_conf, bool swap);
bool backup_boot_sector();
bool restore_boot_sector();
int config_get_active();

uint32_t flash2uint32(uint32_t addr);
bool uint32toflash(uint32_t addr, uint32_t val);
uint32_t calc_app_crc32(uint32_t address, uint32_t size);
bool calc_set_app_sha256(uint32_t address);
bool check_app_sha256(uint32_t address);
void calc_app_sha256(uint32_t address, uint8_t *hash, uint8_t *app_hash);
uint32_t get_fw_flash_size(uint32_t address);
