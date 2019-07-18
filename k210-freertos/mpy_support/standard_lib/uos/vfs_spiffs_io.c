#include <stdio.h>

#include "py/binary.h"
#include "py/objarray.h"
#include "py/misc.h"
#include "py/runtime.h"
#include "vfs_spiffs.h"

#include "w25qxx.h"
#include "sleep.h"

#if MICROPY_VFS && MICROPY_VFS_SPIFFS

#include "spiffs_config.h"

static const char* TAG = "[VFS_SPIFFS_IO]";

#if SPIFFS_HAL_CALLBACK_EXTRA
//--------------------------------------------------------------
s32_t sys_spiffs_read(spiffs* fs, int addr, int size, char *buf)
#else
//--------------------------------------------------
s32_t sys_spiffs_read(int addr, int size, char *buf)
#endif
{
    int phy_addr=addr;
    enum w25qxx_status_t res = w25qxx_read_data(phy_addr, (uint8_t *)buf, size);
    LOGV(TAG, "flash read addr:%x size:%d buf_head:%x %x", phy_addr,size,buf[0],buf[1]);
    if (res != W25QXX_OK) {
        LOGE(TAG, "spifalsh read err");
        return res;
    }
    return res;
}

#if SPIFFS_HAL_CALLBACK_EXTRA
//---------------------------------------------------------------
s32_t sys_spiffs_write(spiffs* fs, int addr, int size, char *buf)
#else
//---------------------------------------------------
s32_t sys_spiffs_write(int addr, int size, char *buf)
#endif
{
    int phy_addr=addr;
    
    enum w25qxx_status_t res = w25qxx_write_data(phy_addr, (uint8_t *)buf, size);
    LOGV(TAG, "flash write addr:%x size:%d buf_head:%x,%x\n", phy_addr,size,buf[0],buf[1]);
    if (res != W25QXX_OK) {
        LOGE(TAG, "spifalsh write err");
        return res;
    }
    return res;
}

#if SPIFFS_HAL_CALLBACK_EXTRA
//----------------------------------------------------
s32_t sys_spiffs_erase(spiffs* fs, int addr, int size)
#else
//----------------------------------------
s32_t sys_spiffs_erase(int addr, int size)
#endif
{
    // size is always 4096!
    uint8_t rd_buf[w25qxx_FLASH_SECTOR_SIZE];
    uint8_t *pread = rd_buf;
    w25qxx_read_data(addr, rd_buf, w25qxx_FLASH_SECTOR_SIZE);
    for (int index = 0; index < w25qxx_FLASH_SECTOR_SIZE; index++)
    {
        if (*pread != 0xFF) {
            if (w25qxx_sector_erase(addr) != W25QXX_OK) {
                LOGE(TAG, "spifalsh erase err");
                return W25QXX_BUSY;
            }
            break;
        }
        pread++;
    }
    return W25QXX_OK;
}

#endif
