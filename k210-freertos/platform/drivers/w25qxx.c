/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 LoBo (https://github.com/loboris)
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

/* Copyright 2018 Canaan Inc.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <devices.h>
#include "w25qxx.h"
#include "syslog.h"
#include "sysctl.h"

bool w25qxx_spi_check = false;
uintptr_t spi_adapter;
uintptr_t spi_adapter_wr;
uintptr_t spi_stand;
uint8_t work_trans_mode = 0;
uint32_t w25qxx_flash_speed = 0;
static uint32_t rd_count;
static uint32_t wr_count;
static uint32_t er_count;
static uint64_t op_time;

//--------------------------------------------------------------------------------------------------------------------
static enum w25qxx_status_t w25qxx_receive_data(uint8_t* cmd_buff, uint8_t cmd_len, uint8_t* rx_buff, uint32_t rx_len)
{
    spi_dev_transfer_sequential(spi_stand, (uint8_t *)cmd_buff, cmd_len, (uint8_t *)rx_buff, rx_len);
    return W25QXX_OK;
}

//------------------------------------------------------------------------------------------------------------------------------
static enum w25qxx_status_t w25qxx_receive_data_enhanced(uint32_t* cmd_buff, uint8_t cmd_len, uint8_t* rx_buff, uint32_t rx_len)
{
    memcpy(rx_buff, cmd_buff, cmd_len);
    io_read(spi_adapter, (uint8_t *)rx_buff, rx_len);
    return W25QXX_OK;
}

//---------------------------------------------------------------------------------------------------------------------------------
static enum w25qxx_status_t w25qxx_send_data(uintptr_t file, uint8_t* cmd_buff, uint8_t cmd_len, uint8_t* tx_buff, uint32_t tx_len)
{
    configASSERT(cmd_len);
    uint8_t* tmp_buf = malloc(cmd_len + tx_len);
    memcpy(tmp_buf, cmd_buff, cmd_len);
    if (tx_len)
        memcpy(tmp_buf + cmd_len, tx_buff, tx_len);
    io_write(file, (uint8_t *)tmp_buf, cmd_len + tx_len);
    free(tmp_buf);
    return W25QXX_OK;
}

//---------------------------------------------------
static enum w25qxx_status_t w25qxx_write_enable(void)
{
    uint8_t cmd[1] = {WRITE_ENABLE};

    w25qxx_send_data(spi_stand, cmd, 1, 0, 0);
    return W25QXX_OK;
}

//--------------------------------------------------------------------
static enum w25qxx_status_t w25qxx_read_status_reg1(uint8_t* reg_data)
{
    uint8_t cmd[1] = {READ_REG1};
    uint8_t data[1];

    w25qxx_receive_data(cmd, 1, data, 1);
    *reg_data = data[0];
    return W25QXX_OK;
}

//--------------------------------------------------------------------
static enum w25qxx_status_t w25qxx_read_status_reg2(uint8_t* reg_data)
{
    uint8_t cmd[1] = {READ_REG2};
    uint8_t data[1];

    w25qxx_receive_data(cmd, 1, data, 1);
    *reg_data = data[0];
    return W25QXX_OK;
}

//---------------------------------------------------------------------------------------
static enum w25qxx_status_t w25qxx_write_status_reg(uint8_t reg1_data, uint8_t reg2_data)
{
    uint8_t cmd[3] = {WRITE_REG1, reg1_data, reg2_data};

    w25qxx_write_enable();
    w25qxx_send_data(spi_stand, cmd, 3, 0, 0);
    return W25QXX_OK;
}

//-------------------------------------------------------
static enum w25qxx_status_t w25qxx_enable_quad_mode(void)
{
    uint8_t reg_data;

    w25qxx_read_status_reg2(&reg_data);
    if (!(reg_data & REG2_QUAL_MASK))
    {
        reg_data |= REG2_QUAL_MASK;
        w25qxx_write_status_reg(0x00, reg_data);
        w25qxx_read_status_reg2(&reg_data);
        if (!(reg_data & REG2_QUAL_MASK)) {
            LOGE("w25qxx_mode", "quad mode NOT enabled");
            return W25QXX_ERROR;
        }
    }
    LOGD("w25qxx_mode", "quad mode enabled");
    return W25QXX_OK;
}

//---------------------------------------
enum w25qxx_status_t w25qxx_is_busy(void)
{
    uint8_t status;

    w25qxx_read_status_reg1(&status);
    if (status & REG1_BUSY_MASK)
        return W25QXX_BUSY;
    return W25QXX_OK;
}

//-----------------------------------------------------
enum w25qxx_status_t w25qxx_sector_erase(uint32_t addr)
{
    uint8_t cmd[4] = {SECTOR_ERASE};

    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)(addr);
    w25qxx_write_enable();
    w25qxx_send_data(spi_stand, cmd, 4, 0, 0);
    er_count++;
    return W25QXX_OK;
}

//------------------------------------------------------------------------
enum w25qxx_status_t w25qxx_read_id(uint8_t *manuf_id, uint8_t *device_id)
{
    uint8_t cmd[4] = {READ_ID, 0x00, 0x00, 0x00};
    uint8_t data[2] = {0};

    w25qxx_receive_data(cmd, 4, data, 2);
    *manuf_id = data[0];
    *device_id = data[1];
    return W25QXX_OK;
}

//-------------------------------------------------------------------------------------------------------
static enum w25qxx_status_t w25qxx_read_data_less_64kb(uint32_t addr, uint8_t* data_buf, uint32_t length)
{
    uint32_t cmd[2];

    switch (work_trans_mode)
    {
        case SPI_FF_DUAL:
            *(((uint8_t*)cmd) + 0) = FAST_READ_DUAL_OUTPUT;
            *(((uint8_t*)cmd) + 1) = (uint8_t)(addr >> 0);
            *(((uint8_t*)cmd) + 2) = (uint8_t)(addr >> 8);
            *(((uint8_t*)cmd) + 3) = (uint8_t)(addr >> 16);
            w25qxx_receive_data_enhanced(cmd, 4, data_buf, length);
            break;
        case SPI_FF_QUAD:
            *(((uint8_t*)cmd) + 0) = FAST_READ_QUAL_OUTPUT;
            *(((uint8_t*)cmd) + 1) = (uint8_t)(addr >> 0);
            *(((uint8_t*)cmd) + 2) = (uint8_t)(addr >> 8);
            *(((uint8_t*)cmd) + 3) = (uint8_t)(addr >> 16);
            w25qxx_receive_data_enhanced(cmd, 4, data_buf, length);
            break;
        case SPI_FF_STANDARD:
        default:
            *(((uint8_t*)cmd) + 0) = READ_DATA;
            *(((uint8_t*)cmd) + 1) = (uint8_t)(addr >> 16);
            *(((uint8_t*)cmd) + 2) = (uint8_t)(addr >> 8);
            *(((uint8_t*)cmd) + 3) = (uint8_t)(addr >> 0);
            w25qxx_receive_data((uint8_t*)cmd, 4, data_buf, length);
            break;
    }
    return W25QXX_OK;
}

//----------------------------------------------------------------------------------------------
static enum w25qxx_status_t _w25qxx_read_data(uint32_t addr, uint8_t* data_buf, uint32_t length)
{
    uint32_t len;
    uint64_t ticks_start = (read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000));
    uint64_t ticks;
    while (length) {
        len = length >= 0x010000 ? 0x010000 : length;
        w25qxx_read_data_less_64kb(addr, data_buf, len);
        addr += len;
        data_buf += len;
        length -= len;
    }
    ticks = (read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000));
    rd_count++;
    op_time += (ticks - ticks_start);

    return W25QXX_OK;
}

//--------------------------------------------------------------------------------------
enum w25qxx_status_t w25qxx_read_data(uint32_t addr, uint8_t* data_buf, uint32_t length)
{
    uint8_t *read_buf = NULL;
    int retry = 0;
    if (w25qxx_spi_check) read_buf = malloc(length);
start:
    _w25qxx_read_data(addr, data_buf, length);

    if ((w25qxx_spi_check) && (read_buf)) {
        _w25qxx_read_data(addr, read_buf, length);
        if (memcmp(data_buf, read_buf, length) != 0) {
            retry++;
            if (retry < 3) goto start;
            free(read_buf);
            return W25QXX_ERROR;
        }
    }

    if (read_buf) free(read_buf);
    return W25QXX_OK;
}

//-------------------------------------
enum w25qxx_status_t w25qxx_wait_busy()
{
    uint64_t ticks_start = (read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000));
    uint64_t ticks;
    while (w25qxx_is_busy() == W25QXX_BUSY) {
        ticks = (read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000));
        if ((ticks - ticks_start) > 500000) {
            op_time += (ticks - ticks_start);
            return W25QXX_ERROR;
        }
    }
    ticks = (read_csr64(mcycle) / (uint64_t)(sysctl_clock_get_freq(SYSCTL_CLOCK_CPU)/1000000));
    op_time += (ticks - ticks_start);
    return W25QXX_OK;
}

//------------------------------------------------------------------------------------------------
static enum w25qxx_status_t w25qxx_page_program(uint32_t addr, uint8_t* data_buf, uint32_t length)
{
    uint8_t *read_buf = NULL;
    int retry = 0;
    if (w25qxx_spi_check) read_buf = malloc(length);

    uint32_t cmd[2];
start:
    w25qxx_write_enable();
    if (work_trans_mode == SPI_FF_QUAD)
    {
        *(((uint8_t*)cmd) + 0) = QUAD_PAGE_PROGRAM;
        *(((uint8_t*)cmd) + 1) = (uint8_t)(addr >> 0);
        *(((uint8_t*)cmd) + 2) = (uint8_t)(addr >> 8);
        *(((uint8_t*)cmd) + 3) = (uint8_t)(addr >> 16);
        w25qxx_send_data(spi_adapter_wr, (uint8_t*)cmd, 4, data_buf, length);
    }
    else
    {
        *(((uint8_t*)cmd) + 0) = PAGE_PROGRAM;
        *(((uint8_t*)cmd) + 1) = (uint8_t)(addr >> 16);
        *(((uint8_t*)cmd) + 2) = (uint8_t)(addr >> 8);
        *(((uint8_t*)cmd) + 3) = (uint8_t)(addr >> 0);
        w25qxx_send_data(spi_stand, (uint8_t*)cmd, 4, data_buf, length);
    }
    wr_count++;
    enum w25qxx_status_t res = w25qxx_wait_busy();
    if (res != W25QXX_OK) return res;

    if (w25qxx_spi_check) {
        _w25qxx_read_data(addr, read_buf, length);
        if (memcmp(data_buf, read_buf, length) != 0) {
            retry++;
            if (retry < 3) goto start;
            res = W25QXX_ERROR;
        }
    }
    if (read_buf) free(read_buf);
    return res;
}

//---------------------------------------------------------------------------------
static enum w25qxx_status_t w25qxx_sector_program(uint32_t addr, uint8_t* data_buf)
{
    uint8_t index;

    for (index = 0; index < w25qxx_FLASH_PAGE_NUM_PER_SECTOR; index++)
    {
        enum w25qxx_status_t res = w25qxx_page_program(addr, data_buf, w25qxx_FLASH_PAGE_SIZE);
        if (res != W25QXX_OK) return res;
        addr += w25qxx_FLASH_PAGE_SIZE;
        data_buf += w25qxx_FLASH_PAGE_SIZE;
    }
    return W25QXX_OK;
}

//---------------------------------------------------------------------------------------
enum w25qxx_status_t w25qxx_write_data(uint32_t addr, uint8_t* data_buf, uint32_t length)
{
    uint32_t sector_addr, sector_offset, sector_remain, write_len, index;
    uint8_t swap_buf[w25qxx_FLASH_SECTOR_SIZE];
    uint8_t *pread, *pwrite;
    enum w25qxx_status_t res;

    while (length)
    {
        sector_addr = addr & (~(w25qxx_FLASH_SECTOR_SIZE - 1));
        sector_offset = addr & (w25qxx_FLASH_SECTOR_SIZE - 1);
        sector_remain = w25qxx_FLASH_SECTOR_SIZE - sector_offset;
        write_len = length < sector_remain ? length : sector_remain;
        w25qxx_read_data(sector_addr, swap_buf, w25qxx_FLASH_SECTOR_SIZE);
        pread = swap_buf + sector_offset;
        pwrite = data_buf;
        // Check if some bits should be erased
        for (index = 0; index < write_len; index++) {
            if ((*pwrite) != ((*pwrite) & (*pread))) {
                //LOGD("w25qxx_write", "erase sector %x (%x, %d)", sector_addr, addr, length);
                w25qxx_sector_erase(sector_addr);
                if (w25qxx_wait_busy() != W25QXX_OK) {
                    LOGE("w25qxx_write", "sector erase timeout");
                    return W25QXX_ERROR;
                }
                break;
            }
            pwrite++;
            pread++;
        }
        if (write_len == w25qxx_FLASH_SECTOR_SIZE) {
            res = w25qxx_sector_program(sector_addr, data_buf);
        }
        else  {
            pread = swap_buf + sector_offset;
            pwrite = data_buf;
            for (index = 0; index < write_len; index++)
                *pread++ = *pwrite++;
            res = w25qxx_sector_program(sector_addr, swap_buf);
            if (res != W25QXX_OK) return res;
        }
        length -= write_len;
        addr += write_len;
        data_buf += write_len;
    }
    return W25QXX_OK;
}

//---------------------------------------------------------------------
uint32_t w25qxx_init(uintptr_t spi_in, uint8_t mode, double clock_rate)
{
    configASSERT(mode < 3);
    work_trans_mode = mode;
    uint8_t manuf_id, device_id;

    spi_stand = spi_get_device(spi_in, SPI_MODE_0, SPI_FF_STANDARD, CHIP_SELECT, FRAME_LENGTH);
    w25qxx_flash_speed = (uint32_t)spi_dev_set_clock_rate(spi_stand, clock_rate);

    w25qxx_read_id(&manuf_id, &device_id);
    if ((manuf_id != 0xEF && manuf_id != 0xC8) || (device_id != 0x17 && device_id != 0x16))
    {
        LOGW("w25qxx_init", "manuf_id: 0x%02x, device_id:0x%02x", manuf_id, device_id);
    }
    LOGD("w25qxx_init", "manuf_id:0x%02x, device_id:0x%02x", manuf_id, device_id);
    switch (work_trans_mode)
    {
        case SPI_FF_DUAL:
            spi_adapter = spi_get_device(spi_in, SPI_MODE_0, SPI_FF_DUAL, CHIP_SELECT, FRAME_LENGTH);
            spi_dev_config_non_standard(spi_adapter, INSTRUCTION_LENGTH, ADDRESS_LENGTH, WAIT_CYCLE, SPI_AITM_STANDARD);
            w25qxx_flash_speed = spi_dev_set_clock_rate(spi_adapter, clock_rate);
            break;
        case SPI_FF_QUAD:
            spi_adapter = spi_get_device(spi_in, SPI_MODE_0, SPI_FF_QUAD, CHIP_SELECT, FRAME_LENGTH);
            spi_dev_config_non_standard(spi_adapter, INSTRUCTION_LENGTH, ADDRESS_LENGTH, WAIT_CYCLE, SPI_AITM_STANDARD);
            w25qxx_flash_speed = spi_dev_set_clock_rate(spi_adapter, clock_rate);

            spi_adapter_wr = spi_get_device(spi_in, SPI_MODE_0, SPI_FF_QUAD, CHIP_SELECT, FRAME_LENGTH);
            spi_dev_config_non_standard(spi_adapter_wr, INSTRUCTION_LENGTH, ADDRESS_LENGTH, 0, SPI_AITM_STANDARD);
            spi_dev_set_clock_rate(spi_adapter_wr, clock_rate);

            w25qxx_enable_quad_mode();
            break;
        case SPI_FF_STANDARD:
        default:
            spi_adapter = spi_stand;
            break;
    }
    return w25qxx_flash_speed;
}

//--------------------------
void w25qxx_clear_counters()
{
    rd_count = 0;
    wr_count = 0;
    er_count = 0;
    op_time = 0;
}

//-----------------------------------------------------------------------------
void w25qxx_get_counters(uint32_t *r, uint32_t *w, uint32_t *e, uint64_t *time)
{
    if (r) *r = rd_count;
    if (w) *w = wr_count;
    if (e) *e = er_count;
    if (time) *time = op_time;
}
