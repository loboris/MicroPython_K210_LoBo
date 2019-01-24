#include <stdio.h>

#include "py/binary.h"
#include "py/objarray.h"
#include "py/misc.h"
#include "py/runtime.h"
#include "vfs_spiffs.h"

#include "w25qxx.h"
#include "sleep.h"

#include "spiffs_config.h"

static const char* TAG = "[VFS_SPIFFS_IO]";

//--------------------------------------------------------------
s32_t sys_spiffs_read(spiffs* fs, int addr, int size, char *buf)
{
    int phy_addr=addr;
    enum w25qxx_status_t res = w25qxx_read_data(phy_addr, (uint8_t *)buf, size);
    LOGV(TAG, "flash read addr:%x size:%d buf_head:%x %x", phy_addr,size,buf[0],buf[1]);
    if (res != W25QXX_OK) {
        LOGV(TAG, "spifalsh read err");
        return res;
    }
    return res;
}

//---------------------------------------------------------------
s32_t sys_spiffs_write(spiffs* fs, int addr, int size, char *buf)
{
    int phy_addr=addr;
    
    enum w25qxx_status_t res = w25qxx_write_data(phy_addr, (uint8_t *)buf, size);
    LOGV(TAG, "flash write addr:%x size:%d buf_head:%x,%x\n", phy_addr,size,buf[0],buf[1]);
    if (res != W25QXX_OK) {
        LOGV(TAG, "spifalsh write err");
        return res;
    }
    return res;
}

//----------------------------------------------------
s32_t sys_spiffs_erase(spiffs* fs, int addr, int size)
{
    // size is always 4096!
    uint8_t rd_buf[w25qxx_FLASH_SECTOR_SIZE];
    uint8_t *pread = rd_buf;
    w25qxx_read_data(addr, rd_buf, w25qxx_FLASH_SECTOR_SIZE);
    for (int index = 0; index < w25qxx_FLASH_SECTOR_SIZE; index++)
    {
        if (*pread != 0xFF) {
            w25qxx_sector_erase(addr);
            while (w25qxx_is_busy() == W25QXX_BUSY)
                ;
            break;
        }
        pread++;
    }
    return W25QXX_OK;
}
