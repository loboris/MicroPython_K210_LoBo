/* Copyright 2019 LoBo
 * 
 * K210 Bootloader stage_0
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
#include "encoding.h"

// Constants

#define BOOTHI_FLASH_ADDR       0x00001005  // stage_1 bootloader code Flash address
#define BOOTHI_SRAM_ADDR        0x805E0000  // stage_1 bootloader code SRAM address
#define BOOTHI_SIZE             0x00003000  // stage_1 bootloader code size (max)

// K210 ROM functions
typedef int rom_print_func(const char * restrict format, ... );
rom_print_func *rom_printf = (rom_print_func*)0x88001418;     // fixed address in ROM
rom_print_func *rom_printk = (rom_print_func*)0x880016b0;     // fixed address in ROM

// Variables
volatile sysctl_t *const sysctl = (volatile sysctl_t *)SYSCTL_BASE_ADDR;
static volatile spi_t *spi_handle = (volatile spi_t *)SPI3_BASE_ADDR;

static uint64_t boothi_start = BOOTHI_SRAM_ADDR;
static uint8_t *boothi_flash_ptr = (uint8_t *)SPI3_BASE_ADDR+BOOTHI_FLASH_ADDR;
static uint8_t *boothi_sram_ptr = (uint8_t *)BOOTHI_SRAM_ADDR;
static uint32_t i = 0;
static uint32_t core0_sync = 0;
static uint32_t core1_sync = 0;

/*
 * After the K210 is reset the 'booltloder_lo.bin' code is loaded from SPI Flash
 * to SRAM address 0x80000000 by ROM code and executed.
 * This is the first location containing the K210 code!
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
    //---------------------------------------------------------
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
         * core #0 is done, the 2nd part of kboot is transfered
         * to the high sram, continue the execution there
         * 
         * each core has separate instruction cache
         * we must clear it before continuing
         */
        asm("fence");   // D-Cache; this may not be necessary, but it looks it doesn't hurt if it is executed
        asm("fence.i"); // I-Cache
        // continue at 2nd part of kboot at high sram
        asm ("jr %0" : : "r"(boothi_start));

        // This should never be reached!
        while (1) {
            asm("nop");
        }
    }
    //---------------------------------------------------------

    // ============================
    // ==== Running on core #0 ====
    // ============================

    // Without this command every character is printed twice (why !?)
    rom_printf(NULL);
    // wait for synchronization with core #1
    while (core0_sync == 0) {
        asm("nop");
    }

    // ----------------------------------------------------------------------------------------------
    // === Initialize SPI Flash driver ===
    // spi clock init (SPI3 clock source is PLL0/2, 390 MHz)
    sysctl->clk_sel0.spi3_clk_sel = 1;
    sysctl->clk_en_peri.spi3_clk_en = 1;
    sysctl->clk_th1.spi3_clk_threshold = 0;

    // spi3 init
    // sets spi clock to 43.333333 MHz
    spi_handle->baudr = 9;
    spi_handle->imr = 0x00;
    spi_handle->dmacr = 0x00;
    spi_handle->dmatdlr = 0x10;
    spi_handle->dmardlr = 0x00;
    spi_handle->ser = 0x00;
    spi_handle->ssienr = 0x00;
    spi_handle->ctrlr0 = (SPI_WORK_MODE_0 << 8) | (SPI_FF_QUAD << 22) | (7 << 0);
    spi_handle->spi_ctrlr0 = 0;
    spi_handle->endian = 0;

    // Enable XIP mode
    spi_handle->xip_ctrl = (0x01 << 29) | (0x02 << 26) | (0x01 << 23) | (0x01 << 22) | (0x04 << 13) |
                (0x01 << 12) | (0x02 << 9) | (0x06 << 4) | (0x01 << 2) | 0x02;
    spi_handle->xip_incr_inst = 0xEB;
    spi_handle->xip_mode_bits = 0x00;
    spi_handle->xip_ser = 0x01;
    spi_handle->ssienr = 0x01;
    sysctl->peri.spi3_xip_en = 1;
    // ----------------------------------------------------------------------------------------------

    // ==========================================================================
    // === Copy the 2nd part of kboot code from flash to SRAM at high address ===
    // ==========================================================================
    for (i = 0; i < BOOTHI_SIZE; i++) {
        *boothi_sram_ptr++ = *boothi_flash_ptr++;
    }

    // === JUMP to stage_1 code start ===
    // Leave XIP mode enabled
    /*
     * each core has separate instruction cache
     * we must clear it before continuing
     * Quote:
        The FENCE.I instruction is used to synchronize the instruction and data streams.
        RISC-V does not guarantee that stores to instruction memory will be made visible to instruction fetches
        on a RISC-V hart until that hart executes a FENCE.I instruction.
        A FENCE.I instruction ensures that a subsequent instruction fetch on a RISC-V hart
        will see any previous data stores already visible to the same RISC-V hart.
        FENCE.I does not ensure that other RISC-V harts’ instruction fetches will
        observe the local hart’s stores in a multiprocessor system.
        To make a store to instruction memory visible to all RISC-V harts,
        the writing hart has to execute a data FENCE before requesting that all remote RISC-V harts execute a FENCE.I.
    */

    core1_sync = 1;
    asm("fence");   // D-Cache; this may not be necessary, but it looks it doesn't hurt if it is executed
    asm("fence.i"); // I-Cache
    asm ("jr %0" : : "r"(boothi_start));

    // This should never be reached!
    while (1) {
        asm("nop");
    }
    return 0;
}
