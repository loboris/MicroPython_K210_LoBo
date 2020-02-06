/* Copyright 2019 LoBo
 * 
 * K210 Bootloader stage_1
 * ver. 1.4.1, 01/2020
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * -----------------------------------------------------------------------------
 * KBoot SPI Flash layout:
 * -----------------------------------------------------------------------------
 * From         To          Length  Comment
 * -----------------------------------------------------------------------------
 * 0x00000000   0x00000FFF      4K  bootloader stage_0 code
 * 0x00001000   0x00003FFF     12K  bootloader stage_1 code
 * 0x00004000   0x00004FFF      4K  boot configuration sector (256B used)
 * 0x00005000   0x00005FFF      4K  boot configuration sector BACKUP (256B used)
 * 0x00006000   0x0000FFFF     40K  reserved
 * -----------------------------------------------------------------------------
 * 0x00010000    <DEF_END>          default application, optional, recommended
 *  <DEF_END>    flash_end          user area, app code, file systems, data...
 * -----------------------------------------------------------------------------
 */


#include "spi.h"
#include "sysctl.h"
#include "fpioa.h"
#include "gpiohs.h"
#include "sha256.h"
#include "encoding.h"
#include "config.h"

// Constants
/*
 * Pin used to enter interactive mode and request the user
 * to select which application to load
 * Any valid and not used K210 gpio can be used except gpio#16 (0 ~ 15 & 17 ~ 47)
 */
//#define BOOT_PIN            18
#define GPIO_KEY            2

#define DEFAULT_APP_ADDR        0x00010000  // default application flash address (64K)
#define APP_START_ADDR          0x80000000  // application start address in K210 SRAM
#define BOOT_CONFIG_ADDR        0x00004000  // main boot config sector in flash at 52K
#define BOOT_CONFIG_ITEMS       8           // number of handled config entries

// boot config magic number (bits 4~31)
//   flag: bit0 - active; bit1 - crc flag; bit2 - SHA flag; bit3 - check size
#define MAGIC_ID                0x5AA5D0C0
#define MAGIC_ID_MASK           0xFFFFFFF0
#define MAGIC_ID_FLAG           0xA55A60C0
#define MAGIC_ID_FLAG_MASK      0xFFFFFFC0
#define CFG_APP_FLAG_ACTIVE     0x00000001
#define CFG_APP_FLAG_CRC32      0x00000002
#define CFG_APP_FLAG_SHA256     0x00000004
#define CFG_APP_FLAG_SIZE       0x00000008

// aplications limits
#define MIN_APP_FLASH_ADDR      0x00010000  // minimal application address in Flash: 64K
#define MAX_APP_FLASH_ADDR      0x00800000  // mmaximal application address in Flash: 8M
#define MIN_APP_FLASH_SIZE      0x00004000  // minimal application size in Flash: 16K
#define MAX_APP_FLASH_SIZE      0x00300000  // mmaximal application size in Flash: 3M

// K210 ROM functions
typedef int rom_printf_func(const char * restrict format, ... );
typedef int rom_getchar_func();

rom_printf_func *rom_printf = (rom_printf_func*)0x88001418;     // fixed address in ROM
rom_printf_func *rom_printk = (rom_printf_func*)0x880016b0;     // fixed address in ROM
rom_getchar_func *rom_getchar = (rom_getchar_func*)0x88008bd0;  // fixed address in ROM

// Variables

#if FIRMWARE_SIZE
// variables used to provide mboot info to the loaded application
volatile uint32_t *ld_mbootid = (volatile uint32_t *)(APP_START_ADDR+FIRMWARE_SIZE);
volatile uint32_t *ld_address = (volatile uint32_t *)(APP_START_ADDR+FIRMWARE_SIZE+4);
volatile uint32_t *ld_size = (volatile uint32_t *)(APP_START_ADDR+FIRMWARE_SIZE+8);
#endif

volatile sysctl_t *const sysctl = (volatile sysctl_t *)SYSCTL_BASE_ADDR;

static uint32_t cfg_magic = 0;
static uint32_t cfg_address = 0;
static uint32_t cfg_size = 0;
static uint32_t cfg_crc = 0;
static uint8_t  cfg_info[16] = "App name";
static uint8_t  available_apps[BOOT_CONFIG_ITEMS] = {0};

static uint64_t app_start = APP_START_ADDR;
static uint32_t app_flash_start = DEFAULT_APP_ADDR;
static uint32_t app_size = 0;
static uint8_t *app_flash_ptr = (uint8_t *)SPI3_BASE_ADDR;
static uint8_t *app_sram_ptr = (uint8_t *)APP_START_ADDR;

static uint32_t *cfg_flash_ptr = (uint32_t *)(SPI3_BASE_ADDR+BOOT_CONFIG_ADDR);
static uint8_t *cfg_flash_bptr = (uint8_t *)(SPI3_BASE_ADDR+BOOT_CONFIG_ADDR);

static uint32_t i = 0;
static uint32_t cfg_offset = 0;
static uint32_t offset = 0;
static uint8_t key = 0;
static uint32_t crc32;

static uint8_t app_hash[SHA256_HASH_LEN] = {0};
static uint8_t hash[SHA256_HASH_LEN] = {0};
static uint8_t buffer[1024] = {0};
static sha256_context_t context;

static uint32_t boot_pin = 1;
static int char_in = 0;
static uint32_t print_enabled = 1;

static uint32_t core0_sync = 0;
static uint32_t core1_sync = 0;


// Printing messages if interactive mode is enabled
// K210 ROM code is used for printing
#define LOG(format, ...)                        \
    do                                          \
    {                                           \
        if (print_enabled) {                    \
            rom_printk(format, ##__VA_ARGS__);  \
            usleep(200);                        \
        }                                       \
    } while(0)


// ==== Local functions ===

/*
 * Get uint32_t value from Flash
 * 8-bit Flash pionter is used,
 * so the byte order must be swapped
 */
//-----------------------------------------
static uint32_t flash2uint32(uint32_t addr)
{
    uint32_t val = app_flash_ptr[addr];
    val += app_flash_ptr[addr+1] << 8;
    val += app_flash_ptr[addr+2] << 16;
    val += app_flash_ptr[addr+3] << 24;
    return val;
}

/*
 * Get all configuration parameters
 * for config sector entry at index 'i'
 * into static variables
 */
//--------------------------------
static void get_params(uint32_t i)
{
    offset = (i*8) + (cfg_offset / 4); // 32bit offset in current config sector
    cfg_magic = cfg_flash_ptr[offset + 0];
    cfg_address = cfg_flash_ptr[offset + 1];
    cfg_size = cfg_flash_ptr[offset + 2];
    cfg_crc = cfg_flash_ptr[offset + 3];

    // Get app description
    offset = (i*32) + cfg_offset + 0x10; // 8bit offset in current config sector
    key = cfg_flash_bptr[offset];        // dummy read needed to switch to 8bit XiP read
    for (int n=0; n<16; n++) {
        cfg_info[n] = cfg_flash_bptr[offset + n];
    }
    cfg_info[15] = 0;
}

/*
 * Check the application's SHA256 hash
 * The hash is 32 bytes long and written after the application's code
 * by Kflash or other application
 */
//--------------------------
static uint32_t app_sha256()
{
    int size = cfg_size+5;
    int sz;
    sha256_init(&context, size);
    uint32_t idx = 0;

    while (size > 0) {
        sz = (size >= 1024) ? 1024 : size;
        for (int n=0; n<sz; n++) {
            buffer[n] = app_flash_ptr[cfg_address + idx + n];
        }
        sha256_update(&context, buffer, sz);
        idx += sz;
        size -= sz;
    }

    sha256_final(&context, hash);

    // get the application's SHA256 hash
    offset = cfg_address + cfg_size + 5;
    for (int n=0; n<SHA256_HASH_LEN; n++) {
        app_hash[n] = app_flash_ptr[offset + n];
    }

    for (idx=0; idx<SHA256_HASH_LEN; idx++) {
        if (hash[idx] != app_hash[idx]) {
            LOG("SHA256 error, ");
            return 0;
        }
    }
    return 1;
}

/*
 * Check the current application CRC32 value
 */
//-------------------------
static uint32_t app_crc32()
{
    uint32_t crc = 0xFFFFFFFF;
    uint32_t byte, mask;
    // Read from flash and update CRC32 value
    for (uint32_t n = 5; n < (cfg_size+5); n++) {
        byte = app_flash_ptr[cfg_address + n];
        crc = crc ^ byte;
        for (uint32_t j = 0; j < 8; j++) {
            mask = -(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320 & mask);
        }
    }
    crc32 = ~crc;
    if (crc32 != cfg_crc) {
        LOG("CRC32 error, ");
        return 0;
    }
    return 1;
}

/*
 * Check if the current application parameters
 * points to the valid Kboot application
 */
//-------------------------
static uint32_t app_check()
{
    uint8_t  key = app_flash_ptr[cfg_address];          // must be 0, SHA256 key NOT specified
    uint32_t sz = flash2uint32(cfg_address+1);          // app size
    //uint32_t app_id = flash2uint32(cfg_address+7);      // app ID = MAGIC_ID

    if (key != 0) return 0;
    //if (app_id != MAGIC_ID) return 0;
    if ((cfg_magic & CFG_APP_FLAG_SIZE) && (cfg_size != sz)) return 0;
    else if (cfg_size != sz) {
        LOG("app_size=%u, ", sz);
        cfg_size = sz;
    }

    if (cfg_magic & CFG_APP_FLAG_SHA256) {
        // SHA256 check was requested, check flash data
        return app_sha256();
    }
    if (cfg_magic & CFG_APP_FLAG_CRC32) {
        // CRC check was requested, check flash data
        return app_crc32();
    }
    return 1;
}

/*
 * After the 1st part of the kboot code is executed
 * the execution is continued here.
 * This is the first location containing the hi sram kboot code!
 * -------------------------------------------------------------
 * We come here twice: running on core #0 and running on core #1
 * -------------------------------------------------------------
 */

//============
int main(void)
{
    // Initialize bss data to 0, not needed as no bss data is used
    /*
    extern unsigned int _bss;
    extern unsigned int _ebss;
    unsigned int *dst;
    dst = &_bss;
    while(dst < &_ebss)
        *dst++ = 0;
    */

    // Check the core we are running on
    i = read_csr(mhartid);
    //--------------------------------------------------------
    if (i != 0) {
        // ============================
        // ==== Running on core #1 ====
        // ============================
        core0_sync = 1;
        // wait for synchronization with core #0
        while (core1_sync == 0) {
            asm("nop");
        }
        
        /*
         * core #0 is done, the application code is transfered
         * to sram at 0x80000000, continue the execution there
         * 
         * each core has separate instruction cache
         * we must clear it before continuing
         */
        usleep(1000);
        asm("fence");   // D-Cache; this may not be necessary, but it looks it doesn't hurt if it is executed
        asm("fence.i"); // I-Cache
        //asm("li a0, 1");
        asm ("jr %0" : : "r"(app_start));

        // This should never be reached!
        while (1) {
            asm("nop");
        }
    }
    //--------------------------------------------------------

    // ============================
    // ==== Running on core #0 ====
    // ============================

    // wait for synchronization with core #1
    while (core0_sync == 0) {
        asm("nop");
    }

    // === Printing and Flash XiP mode were initialized in stage_0 ===

    // Check if interractive mode can be enabled
    boot_pin = 1;
    cfg_magic = cfg_flash_ptr[BOOT_CONFIG_ITEMS*8];
    print_enabled = (cfg_magic == MAGIC_ID) ? 0 : 1;
    if (print_enabled) {
        if ((cfg_magic & MAGIC_ID_FLAG_MASK) == MAGIC_ID_FLAG) {
            // use provided pin number as boot pin
            if ((cfg_magic & 0x3F) > 0) {
                fpioa_set_function(cfg_magic & 0x3F, FUNC_GPIOHS2);
                gpiohs_set_drive_mode(GPIO_KEY, GPIO_DM_INPUT_PULL_UP);
                usleep(1000);
                boot_pin = gpiohs_get_pin(GPIO_KEY);
            }
        }
        else {
            // use default pin number as boot pin
            #if ((BOOT_PIN >= 0) && (BOOT_PIN < 48) && (BOOT_PIN != 16))
            fpioa_set_function(BOOT_PIN, FUNC_GPIOHS2);
            gpiohs_set_drive_mode(GPIO_KEY, GPIO_DM_INPUT_PULL_UP);
            usleep(1000);
            boot_pin = gpiohs_get_pin(GPIO_KEY);
            #endif
        }
    }

    LOG("\nK210 bootloader by LoBo v.1.4.3\n\n");

    LOG("* Find applications in MAIN parameters\n");

    // First we check the main config sector
    app_flash_start = 0;
    cfg_offset = 0;

    // === Read boot configuration from config sector ===
check_cfg:
    // Check all boot entries
    for (i = 0; i < BOOT_CONFIG_ITEMS; i++) {
        get_params(i);

        if ((cfg_magic & MAGIC_ID_MASK) == MAGIC_ID) {
            // *** Valid configuration found
            LOG("    %u: '%15s', ", i, cfg_info);
            // Check if the Flash address is in range (512K ~ 8MB)
            if ((cfg_address >= MIN_APP_FLASH_ADDR) && (cfg_address <= MAX_APP_FLASH_ADDR)) {
                // Address valid, check if the size is in range (16K ~ 3MB)
                if ((cfg_size >= MIN_APP_FLASH_SIZE) && (cfg_size <= MAX_APP_FLASH_SIZE)) {
                    // Valid size
                    LOG("@ 0x%08X, size=%u, ", cfg_address, cfg_size);
                    /*
                     * Basic check passed, now we can check the application's vilidity
                     * If in interractive mode, all applications are checked,
                     * otherwize, the application is checked only if flagged as active
                     */
                    if ((cfg_magic & CFG_APP_FLAG_ACTIVE) || (boot_pin == 0)) {
                        // ** Check if valid application
                        if (app_check() == 0) {
                            LOG("App CHECK failed\n");
                            continue;
                        }
                        LOG("App ok, ");
                        available_apps[i] = 1;
                    }
                    else {
                        LOG("not checked, ");
                    }
                    // ** Check if this is an active config (bit #0 set)
                    if ((cfg_magic & CFG_APP_FLAG_ACTIVE) == 0) {
                        LOG("NOT ");
                    }
                    else {
                        if (app_flash_start == 0) {
                            app_size = cfg_size;
                            app_flash_start = cfg_address;
                        }
                        if (boot_pin > 0) {
                            // Active application found and cheched and not in interractive mode
                            LOG("ACTIVE\n");
                            break;
                        }
                    }
                    LOG("ACTIVE\n");
                }
                else {
                    LOG("size Error!\n");
                }
            }
            else {
                LOG("address Error!\n");
            }
        }
    }

    // check if any valid application was found
    for (i = 0; i < BOOT_CONFIG_ITEMS; i++) {
        if (available_apps[i]) break;
    }
    if ((app_flash_start == 0) && (i >= BOOT_CONFIG_ITEMS)) {
        // No valid application found
        if (cfg_offset == 0) {
            // no valid entry found in main config sector, check the backup one
            LOG("\n* Find applications in BACKUP parameters\n");
            cfg_offset = 4096;
            //if (boot_pin == 0) {
            //    cfg_magic = cfg_flash_ptr[4096+BOOT_CONFIG_ITEMS];
            //    boot_pin = (cfg_magic == MAGIC_ID) ? 1 : 0;
            //}
            goto check_cfg;
        }
        else {
            LOG("\n* No app found, loading default\n");
        }
    }
    else if (boot_pin == 0) {
        // === Interractive mode ===
        #if ((BOOT_PIN >= 0) && (BOOT_PIN < 48) && (BOOT_PIN != 16))
        // Check boot pin again
        boot_pin = gpiohs_get_pin(GPIO_KEY);
        #endif
        if (boot_pin == 0) {
            // ** request user to select the application to run
            LOG("\nSelect the application number to load [");
            for (i = 0; i < BOOT_CONFIG_ITEMS; i++) {
                if (available_apps[i]) {
                    LOG(" %u,", i);
                }
            }
            LOG(" d=default ] ? ");
            while (1) {
                char_in = rom_getchar();
                if ((char_in == 'd') || (char_in == 'D')) {
                    // use default application
                    app_flash_start = DEFAULT_APP_ADDR;
                    break;
                }
                else {
                    char_in -= 0x30;
                    if ((char_in >= 0) && (char_in < BOOT_CONFIG_ITEMS)) {
                        if (available_apps[char_in]) {
                            get_params(char_in);
                            // get application's size and address in Flash
                            app_size = flash2uint32(cfg_address+1);
                            app_flash_start = cfg_address;
                            char_in += 0x30;
                            break;
                        }
                    }
                    char_in += 0x30;
                    LOG("%c? ", char_in);
                }
            }
            LOG("%c\n\n", (char)char_in);
        }
    }

    #if FIRMWARE_SIZE
    *ld_address = app_flash_start;
    #endif

    // If default application is selected for load, get its size and check SHA hash
    if (app_flash_start == 0) {
        key = 1;
        app_flash_start = DEFAULT_APP_ADDR;
        app_size = flash2uint32(app_flash_start+1); // get app size
        if ((app_size >= MIN_APP_FLASH_SIZE) && (app_size <= MAX_APP_FLASH_SIZE)) {
            cfg_size = app_size;
            cfg_address = app_flash_start;
            // Check default application
            if (app_sha256()) key = 0;
        }
        if (key) {
            // Check failed
            print_enabled = 1;
            LOG("\n* Default application check failed!\n");
            LOG("* SYSTEM HALTED\n");
            while (1) {
                asm ("nop");
            }
        }
    }

    // === Copy the application code from flash to SRAM ===
    #if FIRMWARE_SIZE
    *ld_mbootid = MAGIC_ID;
    *ld_size = app_size;
    if (*ld_address == 0) {
        *ld_address = app_flash_start | 0x80000000;
    }
    #endif
    LOG("* Loading app from flash at 0x%08X (%u B)\n", app_flash_start, app_size);

    for (i=0; i < app_size; i++) {
        app_sram_ptr[i] = app_flash_ptr[app_flash_start+i+5];
    }

    // === Start the application ===
    LOG("* Starting at 0x%08X ...\n\n", app_start);
    usleep(1000);

    // Disable XIP mode
    sysctl->peri.spi3_xip_en = 0;

    // === Jump to the loaded application start address ===
    /*
     * each core has separate instruction cache
     * we must clear it before continuing
    */

    core1_sync = 1;
    asm("fence");   // D-Cache; this may not be necessary, but it looks it doesn't hurt if it is executed
    asm("fence.i"); // I-Cache
    asm ("jr %0" : : "r"(app_start));

    // This should never be reached!
    LOG("\nERROR, SYSTEM HALTED\n");
    while (1) {
        asm ("nop");
    }
    return 0;
}

