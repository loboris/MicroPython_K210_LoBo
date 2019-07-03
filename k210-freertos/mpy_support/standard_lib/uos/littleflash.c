/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 LoBo (https://github.com/loboris)
 *
 * Adapted from https://github.com/lllucius/esp32_littleflash, see the Copyright and license notice below
 *
 */

// Copyright 2017-2018 Leland Lucius
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mpconfigport.h"

#if MICROPY_VFS_LITTLEFS

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/errno.h>
#include <sys/fcntl.h>
#include <sys/lock.h>
#include <sys/time.h>
#include <time.h>
#include "devices.h"
#include "syslog.h"

#include "py/runtime.h"
#include "py/mperrno.h"
#include "mphalport.h"
#include "littleflash.h"


typedef struct _mp_vfs_littlefs_ilistdir_it_t {
    mp_obj_base_t base;
    mp_fun_1_t iternext;
    bool is_str;
    lfs_dir_t dir;
    lfs_t *lfs;
    char path[LITTLEFS_CFG_MAX_NAME_LEN];
} mp_vfs_littlefs_ilistdir_it_t;


extern handle_t mp_rtc_rtc0;

static const char *TAG = "[LITTLEFS]";

static littleFlash_t littleFlash = {0};

bool force_erase_fs_flash = false;
littlefs_user_mount_t littlefs_user_mount_handle;
littlefs_user_mount_t* vfs_littlefs = &littlefs_user_mount_handle;
void *vfs_flashfs = &littlefs_user_mount_handle;

static char littlefs_current_dir[LITTLEFS_CFG_MAX_NAME_LEN-8] = {'\0'};
static char littlefs_file_path[LITTLEFS_CFG_MAX_NAME_LEN] = {'\0'};

static SemaphoreHandle_t littlefs_mutex = NULL; // FS lock

static uint8_t read_buffer[LITTLEFS_CFG_SECTOR_SIZE] __attribute__((aligned (8)));
static uint8_t prog_buffer[LITTLEFS_CFG_SECTOR_SIZE] __attribute__((aligned (8)));
static uint8_t lookahead_buffer[LITTLEFS_CFG_LOOKAHEAD_SIZE] __attribute__((aligned (8)));

// ============================================================================
// LFS disk interface for internal flash
// ============================================================================

//-------------------------------------------------------------------------------------------------------------------
static int internal_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    if (xSemaphoreTake(littlefs_mutex, 200 / portTICK_PERIOD_MS) != pdTRUE) return LFS_ERR_IO;
    //littleFlash_t *self = (littleFlash_t *) c->context;
    uint32_t phy_addr = LITTLEFS_CFG_START_ADDR + (block * LITTLEFS_CFG_SECTOR_SIZE) + off;
    if (w25qxx_debug) LOGD(TAG, "[READ] bkl=%u, off=%u, sz=%u, adr=%x", block, off, size, phy_addr);

    enum w25qxx_status_t res = w25qxx_read_data(phy_addr, (uint8_t *)buffer, size);
    if (res != W25QXX_OK) {
        if (w25qxx_debug) LOGE(TAG, "read err");
        xSemaphoreGive(littlefs_mutex);
        return LFS_ERR_IO;
    }
    xSemaphoreGive(littlefs_mutex);
    return LFS_ERR_OK;
}

//-------------------------------------------------------------------------------------------------------------------------
static int internal_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    if (xSemaphoreTake(littlefs_mutex, 200 / portTICK_PERIOD_MS) != pdTRUE) return LFS_ERR_IO;
    //littleFlash_t *self = (littleFlash_t *) c->context;
    uint32_t phy_addr = LITTLEFS_CFG_START_ADDR + (block * LITTLEFS_CFG_SECTOR_SIZE) + off;
    if (w25qxx_debug) LOGD(TAG, "[PROG] bkl=%u, off=%u, sz=%u, adr=%x", block, off, size, phy_addr);

    enum w25qxx_status_t res = w25qxx_write_data(phy_addr, (uint8_t *)buffer, size);
    if (res != W25QXX_OK) {
        if (w25qxx_debug) LOGE(TAG, "write err");
        xSemaphoreGive(littlefs_mutex);
        return LFS_ERR_IO;
    }
    xSemaphoreGive(littlefs_mutex);
    return LFS_ERR_OK;
}

//----------------------------------------------------------------------
static int internal_erase(const struct lfs_config *c, lfs_block_t block)
{
    if (xSemaphoreTake(littlefs_mutex, 200 / portTICK_PERIOD_MS) != pdTRUE) return LFS_ERR_IO;
    //if (w25qxx_debug) LOGD(TAG, "[ERASE] bkl=%u", block);
    //littleFlash_t *self = (littleFlash_t *) c->context;
    uint32_t phy_addr = LITTLEFS_CFG_START_ADDR + (block * w25qxx_FLASH_SECTOR_SIZE);

    // erase sector size is 4096!
    uint8_t rd_buf[w25qxx_FLASH_SECTOR_SIZE];
    uint8_t *pread = rd_buf;
    w25qxx_read_data(phy_addr, rd_buf, w25qxx_FLASH_SECTOR_SIZE);
    for (int index = 0; index < w25qxx_FLASH_SECTOR_SIZE; index++)
    {
        if (*pread != 0xFF) {
            //if (w25qxx_debug) LOGD(TAG, "[ERASE] physical erase %x", phy_addr);
            w25qxx_sector_erase(phy_addr);
            enum w25qxx_status_t res = w25qxx_wait_busy();
            if (res != W25QXX_OK) {
                //if (w25qxx_debug) LOGE(TAG, "erase err");
                xSemaphoreGive(littlefs_mutex);
                return res;
            }
            break;
        }
        pread++;
    }
    xSemaphoreGive(littlefs_mutex);
    return LFS_ERR_OK;
}

// Flash driver takes care of erasing the sector when performing program command
// if needed, so we don't need to actually perform the erase before program
// That enables defining the block size smaller than the physical erase sector size
//----------------------------------------------------------------------------
static int internal_dummy_erase(const struct lfs_config *c, lfs_block_t block)
{
    if (w25qxx_debug) LOGD(TAG, "[DUMMY_ERASE] bkl=%u", block);
    return LFS_ERR_OK;
}

//--------------------------------------------------
static int internal_sync(const struct lfs_config *c)
{
    return LFS_ERR_OK;
}


// ============================================================================
// K210 littlefs VFS implementation
// ============================================================================

//-----------------------------------------------
const char *littlefs_local_path(const char *path)
{
    const char *lpath = path;
    if (lpath[0] == '/') {
        lpath++; // absolute path
        int len = strlen(lpath);
        if (lpath[len-1] == '/') {
            strcpy(littlefs_file_path, lpath);
            littlefs_file_path[len-1] = '\0';
            lpath = littlefs_file_path;
        }
    }
    else {
        if (strstr(lpath, "..") == lpath) {
            // parent directory
            lpath += 2;
            if (lpath[0] == '/') lpath++;
            sprintf(littlefs_file_path, "%s", littlefs_current_dir);
            char *ppath = strrchr(littlefs_file_path, '/');
            if (ppath) *ppath = '\0';
            strcat(littlefs_file_path, lpath);
        }
        else {
            if (strstr(lpath, ".") == lpath) lpath += 1;
            if (strstr(lpath, "flash/") == lpath) lpath += 6;
            snprintf(littlefs_file_path, LITTLEFS_CFG_MAX_NAME_LEN, "%s/%s", littlefs_current_dir, lpath);
        }
        int len = strlen(littlefs_file_path);
        while ((len > 0) && (littlefs_file_path[len-1] == '/')) {
            littlefs_file_path[len-1] = '\0';
            len = strlen(littlefs_file_path);
        }
        lpath = littlefs_file_path;
        if (lpath[0] == '/') lpath++;
    }
    if (w25qxx_debug) LOGD(TAG, "LOCAL_PATH [%s]->[%s], currdir=[%s]", path, lpath, littlefs_current_dir);
    return lpath;
}

//------------------------
int map_lfs_error(int err)
{
    if (err == LFS_ERR_OK) return 0;

    switch (err)
    {
        case LFS_ERR_IO:
            errno = MP_EIO;
        break;
        case LFS_ERR_CORRUPT:
            errno = MP_EIO;
        break;
        case LFS_ERR_NOENT:
            errno = MP_ENOENT;
        break;
        case LFS_ERR_EXIST:
            errno = MP_EEXIST;
        break;
        case LFS_ERR_NOTDIR:
            errno = MP_ENOTDIR;
        break;
        case LFS_ERR_ISDIR:
            errno = MP_EISDIR;
        break;
        case LFS_ERR_NOTEMPTY:
            errno = MP_EMFILE;
        break;
        case LFS_ERR_INVAL:
            errno = MP_EINVAL;
        break;
        case LFS_ERR_NOSPC:
            errno = MP_ENOSPC;
        break;
        case LFS_ERR_NOMEM:
            errno = MP_ENOMEM;
        break;
        default:
            errno = MP_EINVAL;
        break;
    }

    return -1;
}


// ============================================================================
// LittleFlash global functions
// ============================================================================

//--------------------------------------------
int set_timestamp(lfs_t *fs, const char *path)
{
    uint32_t timestamp;
    struct tm now;
    rtc_get_datetime(mp_rtc_rtc0, &now);
    timestamp = (uint32_t)mktime(&now);

    return lfs_setattr(fs, path, LITTLEFS_ATTR_MTIME, (const void *)&timestamp, sizeof(uint32_t));
}

//--------------------------------------------
int get_timestamp(lfs_t *fs, const char *path)
{
    uint32_t timestamp;
    int res = lfs_getattr(fs, path, LITTLEFS_ATTR_MTIME, (void *)&timestamp, sizeof(uint32_t));
    if (res != sizeof(uint32_t)) return res;
    return timestamp;
}

/*
//----------------------
void dump_block(int blk)
{
    uint8_t buf[256];
    LOGM(TAG, "=== Block %d ===", blk);
    internal_read((const struct lfs_config *)&littleFlash.lfs.cfg->context, (blk * 256) / 4096, (blk * 256) % 4096, buf, 256);

    printf("\n00: ");
    for (int i=0; i<256; i++) {
        if ((i > 0) && ((i % 32) == 0)) {
            printf("  ");
            for (int n=0;n<32;n++) {
                if ((buf[i-32+n] >= 32) && (buf[i-32+n] < 127)) printf("%c", (char)buf[i-32+n]);
                else printf(".");
            }
            printf("\n%02x: ", i);
        }
        printf("%02x ", buf[i]);
    }
    printf("\n");
}
*/

//======================================
MP_NOINLINE bool init_flash_filesystem()
{
    littlefs_mutex = xSemaphoreCreateMutex();
    configASSERT(littlefs_mutex);

    w25qxx_clear_counters();

    int err;

    vfs_littlefs->flags = SYS_LITTLEFS;
    vfs_littlefs->base.type = &mp_littlefs_vfs_type;
    vfs_littlefs->fs = &littleFlash;

    littleFlash.lfs_cfg.read             = &internal_read;
    littleFlash.lfs_cfg.prog             = &internal_prog;
    littleFlash.lfs_cfg.erase            = &internal_dummy_erase;
    littleFlash.lfs_cfg.sync             = &internal_sync;

    littleFlash.lfs_cfg.read_buffer      = read_buffer;
    littleFlash.lfs_cfg.prog_buffer      = prog_buffer;
    littleFlash.lfs_cfg.lookahead_buffer = lookahead_buffer;

    littleFlash.lfs_cfg.read_size        = LITTLEFS_CFG_RWBLOCK_SIZE;
    littleFlash.lfs_cfg.prog_size        = LITTLEFS_CFG_RWBLOCK_SIZE;
    littleFlash.lfs_cfg.block_size       = LITTLEFS_CFG_SECTOR_SIZE;
    littleFlash.lfs_cfg.block_count      = LITTLEFS_CFG_PHYS_SZ / LITTLEFS_CFG_SECTOR_SIZE;
    littleFlash.lfs_cfg.cache_size       = LITTLEFS_CFG_SECTOR_SIZE;
    littleFlash.lfs_cfg.lookahead_size   = LITTLEFS_CFG_LOOKAHEAD_SIZE;

    littleFlash.lfs_cfg.file_max         = LITTLEFS_CFG_MAX_FILE_SIZE;
    littleFlash.lfs_cfg.name_max         = LITTLEFS_CFG_MAX_NAME_LEN;
    littleFlash.lfs_cfg.block_cycles     = LITTLEFS_CFG_BLOCK_CYCLES;

    littleFlash.lfs_cfg.context          = (void *)&littleFlash;

    if (force_erase_fs_flash) {
        // Erase first 4 blocks (force format)
        for (int i=0; i<4; i++) {
            err = internal_erase(&littleFlash.lfs_cfg, i);
        }
    }
    if (w25qxx_debug) LOGD(TAG, "Littlefs mount.");
    err = lfs_mount(&littleFlash.lfs, &littleFlash.lfs_cfg);
    if (err != LFS_ERR_OK)
    {
        lfs_unmount(&littleFlash.lfs);
        mp_printf(&mp_plat_print, LOG_BOLD(LOG_COLOR_CYAN)"Error mounting Flash file system, select an option\n"LOG_RESET_COLOR);
        mp_printf(&mp_plat_print, LOG_BOLD(LOG_COLOR_BROWN)"A"LOG_RESET_COLOR" - continue without file system (default)\n");
        mp_printf(&mp_plat_print, LOG_BOLD(LOG_COLOR_BROWN)"F"LOG_RESET_COLOR" - format Flash and try to mount again\n");
        mp_printf(&mp_plat_print, LOG_BOLD(LOG_COLOR_BROWN)"E"LOG_RESET_COLOR" - erase Flash, format and try to mount again\n");
        char key = wait_key("", 10000);

        if ((key != 'F') && (key != 'f') && (key != 'E') && (key != 'e')) return false;
        if ((key == 'E') || (key == 'e')) {
            // erase flash
            mp_printf(&mp_plat_print, LOG_BOLD(LOG_COLOR_PURPLE)"Erasing Flash, this can take some time...\n"LOG_RESET_COLOR);
            for (int i=0; i<(LITTLEFS_CFG_PHYS_SZ / w25qxx_FLASH_SECTOR_SIZE) ; i++) {
                internal_erase(&littleFlash.lfs_cfg, i);
            }
            mp_printf(&mp_plat_print, LOG_BOLD(LOG_COLOR_CYAN)"Flash erased\n"LOG_RESET_COLOR);
        }
        else {
            // Erase first 4 blocks
            for (int i=0; i<4; i++) {
                err = internal_erase(&littleFlash.lfs_cfg, i);
                if (err != LFS_ERR_OK) {
                    mp_printf(&mp_plat_print, LOG_BOLD(LOG_COLOR_RED)"Error erasing Flash\n"LOG_RESET_COLOR);
                    goto fail;
                }
            }
        }

        memset(&littleFlash.lfs, 0, sizeof(lfs_t));
        // format
        err = lfs_format(&littleFlash.lfs, &littleFlash.lfs_cfg);
        if (err < 0)
        {
            lfs_unmount(&littleFlash.lfs);
            mp_printf(&mp_plat_print, LOG_BOLD(LOG_COLOR_RED)"Error formating (%d)\n"LOG_RESET_COLOR, err);
            goto fail;
        }
        lfs_unmount(&littleFlash.lfs);

        memset(&littleFlash.lfs, 0, sizeof(lfs_t));
        err = lfs_mount(&littleFlash.lfs, &littleFlash.lfs_cfg);
        if (err < 0)
        {
            lfs_unmount(&littleFlash.lfs);
            mp_printf(&mp_plat_print, LOG_BOLD(LOG_COLOR_RED)"Error mounting after format\n"LOG_RESET_COLOR);
            goto fail;
        }
        mp_printf(&mp_plat_print, LOG_BOLD(LOG_COLOR_CYAN)"Littlefs formated and mounted\n"LOG_RESET_COLOR);
    }
    littleFlash.mounted = true;

    // === Register filesystem to MicroPython VFS
    mp_vfs_mount_t *vfs = m_new_obj_maybe(mp_vfs_mount_t);
    if (vfs == NULL) {
        if (w25qxx_debug) LOGE(TAG, "Cannot create new VFS");
    }
    vfs->str = "/flash";
    vfs->len = 6;
    vfs->obj = MP_OBJ_FROM_PTR(vfs_littlefs);
    vfs->next = NULL;
    mp_vfs_mount_t *vfs_sys = MP_STATE_VM(vfs_mount_table);
    if (vfs_sys == NULL) {
        MP_STATE_VM(vfs_mount_table) = vfs;
    }
    else {
        vfs_sys->next = vfs;
    }
    if (w25qxx_debug) LOGD(TAG, "Flash VFS registered.");
    sprintf(littlefs_current_dir, "%s", "");

    return true;

fail:
    return false;
}

//================================================
void littleFlash_term(const char* partition_label)
{
    if (littleFlash.mounted) {
        lfs_unmount(&littleFlash.lfs);
        littleFlash.mounted = false;
    }
}

// ==============================================================================================================


// Check if path is a directory
//---------------------------------------------
static int _is_dir(lfs_t *fs, const char *path)
{
    if (strlen(path) == 0) return 1;
    struct lfs_info info;

    int res = lfs_stat(fs, path, &info);
    if (res != LFS_ERR_OK) return res;

    if (info.type == LFS_TYPE_DIR) return 1;
    return 0;
}

/*
//---------------------------------------------------
static bool is_dir_empty(lfs_t *fs, const char *path)
{
    lfs_dir_t dir;
    struct lfs_info info;
    bool is_empty = true;

    if (!lfs_dir_open(fs, &dir, path)) return false;

    for (;;) {
        if (lfs_dir_read(fs, &dir, &info) != LFS_ERR_OK) break;
        char *fn = (char *)info.name;
        if (fn[0] == 0) {
            // stop on error or end of dir
            break;
        }

        if (strstr(fn, path) == path) {
            is_empty = false;
            break;
        }
    }
    lfs_dir_close(fs, &dir);
    return is_empty;
}
*/

//--------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t littlefs_vfs_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 1, false);

    // Return existing littlefs vfs object
    for (mp_vfs_mount_t *vfs = MP_STATE_VM(vfs_mount_table); vfs != NULL; vfs = vfs->next) {
        if ((vfs->len == 6) && (strcmp(vfs->str, "/flash") == 0)) {
            return vfs->obj;
        }
    }
    mp_raise_NotImplementedError("System littlefs not found");
    return mp_const_none;
}

//-----------------------------------------------------------------------------
STATIC mp_import_stat_t littlefs_vfs_import_stat(void *vfs_in,const char *path)
{
    littlefs_user_mount_t *vfs = vfs_in;
    struct lfs_info info;
    if (vfs == NULL) return MP_IMPORT_STAT_NO_EXIST;

    const char *lpath = littlefs_local_path(path);

    int res = lfs_stat(&vfs->fs->lfs, lpath, &info);
    if (res == LFS_ERR_OK) {
        if (info.type == LFS_TYPE_DIR) return MP_IMPORT_STAT_DIR;
        else return MP_IMPORT_STAT_FILE;
    }
    return MP_IMPORT_STAT_NO_EXIST;
}

/*
//----------------------------------------------
STATIC mp_obj_t littlefs_vfs_del(mp_obj_t self_in)
{
    mp_raise_NotImplementedError("System littlefs is always mounted");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(littlefs_vfs_del_obj, littlefs_vfs_del);
*/

//-------------------------------------------------
STATIC mp_obj_t littlefs_vfs_mkfs(mp_obj_t self_in)
{
    mp_raise_NotImplementedError("System Flash FS is always in use");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(littlefs_vfs_mkfs_fun_obj, littlefs_vfs_mkfs);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(littlefs_vfs_mkfs_obj, MP_ROM_PTR(&littlefs_vfs_mkfs_fun_obj));

//--------------------------------------------------------------------
STATIC mp_obj_t mp_vfs_littlefs_ilistdir_it_iternext(mp_obj_t self_in)
{
    mp_vfs_littlefs_ilistdir_it_t *self = MP_OBJ_TO_PTR(self_in);
    struct lfs_info info;

    for (;;) {
        int res = lfs_dir_read(self->lfs, &self->dir, &info);
        if (res < LFS_ERR_OK) {
            if (w25qxx_debug) LOGD(TAG, "Read dir error (%d)", res);
            break;
        }
        if (strlen(info.name) == 0) break;
        if (w25qxx_debug) LOGD(TAG, "Read dir: [%s], %u", info.name, info.size);
        if ((strcmp(info.name, ".") == 0) || (strcmp(info.name, "..") == 0)) continue;

        // make 4-tuple with info about this entry
        mp_obj_tuple_t *t = MP_OBJ_TO_PTR(mp_obj_new_tuple(4, NULL));
        if (self->is_str) {
            t->items[0] = mp_obj_new_str(info.name, strlen(info.name));
        } else {
            t->items[0] = mp_obj_new_bytes((const byte*)info.name, strlen(info.name));
        }
        if (info.type == LFS_TYPE_DIR) {
            // dir
            t->items[1] = mp_obj_new_int(MP_S_IFDIR);
        }
        else{
            // file
            t->items[1] = mp_obj_new_int(MP_S_IFREG);
        }
        t->items[2] = mp_obj_new_int(0); // no inode number
        t->items[3] = mp_obj_new_int_from_uint(info.size);

        return MP_OBJ_FROM_PTR(t);
    }

    // ignore error because we may be closing a second time
    lfs_dir_close(self->lfs, &self->dir);

    return MP_OBJ_STOP_ITERATION;
}

//---------------------------------------------------------------------------
STATIC mp_obj_t littlefs_vfs_ilistdir_func(size_t n_args, const mp_obj_t *args)
{
    littlefs_user_mount_t *self = MP_OBJ_TO_PTR(args[0]);
    bool is_str_type = true;
    const char *path;
    char *lpath;
    if (n_args == 2) {
        if (mp_obj_get_type(args[1]) == &mp_type_bytes) {
            is_str_type = false;
        }
        path = mp_obj_str_get_str(args[1]);
        lpath = (char *)littlefs_local_path(path);
    }
    else lpath = littlefs_current_dir;
    if (w25qxx_debug) LOGD(TAG, "LISTDIR [%s]", lpath);

    if (strlen(lpath) > 0) {
        // Check if requested path is a directory
        int res = _is_dir(&(self->fs->lfs), lpath);
        if (res != 1) {
            mp_raise_OSError(map_lfs_error(res));
        }
    }

    // Create a new iterator object to list the directory
    mp_vfs_littlefs_ilistdir_it_t *iter = m_new_obj(mp_vfs_littlefs_ilistdir_it_t);
    iter->base.type = &mp_type_polymorph_iter;
    iter->iternext = mp_vfs_littlefs_ilistdir_it_iternext;
    iter->is_str = is_str_type;
    iter->lfs = &self->fs->lfs;
    strcpy(iter->path, lpath);

    int res = lfs_dir_open(&self->fs->lfs, &iter->dir, lpath);
    if (res != LFS_ERR_OK) {
        if (w25qxx_debug) LOGD(TAG, "Dir open error (%d)", res);
        mp_raise_OSError(map_lfs_error(res));
    }

    return MP_OBJ_FROM_PTR(iter);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(littlefs_vfs_ilistdir_obj, 1, 2, littlefs_vfs_ilistdir_func);

//------------------------------------------------------------------
STATIC mp_obj_t littlefs_vfs_remove(mp_obj_t vfs_in, mp_obj_t path_in)
{
    littlefs_user_mount_t *self = MP_OBJ_TO_PTR(vfs_in);
    const char *path = mp_obj_str_get_str(path_in);
    const char *lpath = littlefs_local_path(path);
    int res;

    // Check if file is directory
    res = _is_dir(&(self->fs->lfs), lpath);
    if (res < 0) {
        mp_raise_OSError(MP_ENOENT);
    }
    if (res == 1) {
        // It is directory, cannot be removed
        mp_raise_OSError(MP_EISDIR);
    }

    res = lfs_remove(&self->fs->lfs, lpath);
    if (res != LFS_ERR_OK) {
        mp_raise_OSError(map_lfs_error(res));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(littlefs_vfs_remove_obj, littlefs_vfs_remove);


//-------------------------------------------------------------------------------------
STATIC mp_obj_t littlefs_vfs_rename(mp_obj_t vfs_in, mp_obj_t path_in, mp_obj_t path_out)
{
    littlefs_user_mount_t *self = MP_OBJ_TO_PTR(vfs_in);
    const char *old_path = mp_obj_str_get_str(path_in);
    const char *new_path = mp_obj_str_get_str(path_out);
    const char *lold_path = littlefs_local_path(old_path);
    const char *lnew_path = littlefs_local_path(new_path);
    int res;

    res = lfs_rename(&self->fs->lfs, lold_path, lnew_path);
    if (res != LFS_ERR_OK) {
        mp_raise_OSError(map_lfs_error(res));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(littlefs_vfs_rename_obj, littlefs_vfs_rename);

//----------------------------------------------------------------
STATIC mp_obj_t littlefs_vfs_mkdir(mp_obj_t vfs_in, mp_obj_t path_o)
{
    littlefs_user_mount_t* self = MP_OBJ_TO_PTR(vfs_in);
    const char *path = mp_obj_str_get_str(path_o);
    const char *lpath = littlefs_local_path(path);
    int res;

    // Check if the directory or file with the same name exists
    res = _is_dir(&(self->fs->lfs), lpath);
    if (res >= 0) {
        // It is directory, cannot be removed
        mp_raise_OSError(MP_EEXIST);
    }

    res = lfs_mkdir(&self->fs->lfs, lpath);
    if (res != LFS_ERR_OK) {
        mp_raise_OSError(map_lfs_error(res));
    }
    res = set_timestamp(&self->fs->lfs, lpath);
    if (res != LFS_ERR_OK) {
        if (w25qxx_debug) LOGD(TAG, "Error setting directory timestamp (%d)", res);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(littlefs_vfs_mkdir_obj, littlefs_vfs_mkdir);

//--------------------------------------------------------------
STATIC mp_obj_t fat_vfs_rmdir(mp_obj_t vfs_in, mp_obj_t path_in)
{
    littlefs_user_mount_t* vfs = MP_OBJ_TO_PTR(vfs_in);
    const char *path = mp_obj_str_get_str(path_in);
    const char *lpath = littlefs_local_path(path);
    int res;

    // Check if path is directory
    res = _is_dir(&vfs->fs->lfs, lpath);
    if (res < 0) {
        mp_raise_OSError(MP_ENOENT);
    }
    if (res != 1) {
        // It is not a directory, cannot be removed
        mp_raise_OSError(MP_ENOTDIR);
    }

    res = lfs_remove(&vfs->fs->lfs, lpath);
    if (res != LFS_ERR_OK) {
        mp_raise_OSError(map_lfs_error(res));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(littlefs_vfs_rmdir_obj, fat_vfs_rmdir);

// Change current directory.
//-----------------------------------------------------------------
STATIC mp_obj_t littlefs_vfs_chdir(mp_obj_t vfs_in, mp_obj_t path_in)
{
    littlefs_user_mount_t* vfs = MP_OBJ_TO_PTR(vfs_in);
    const char *path = mp_obj_str_get_str(path_in);
    int res;

    if ((path[0] != 0) && !(path[0] == '/' && path[1] == 0)) {
        const char *lpath = littlefs_local_path(path);
        // Check if directory
        res = _is_dir(&vfs->fs->lfs, lpath);
        if (res < 0) {
            mp_raise_OSError(res);
        }
        if (res == 0) {
            // Not a directory
            mp_raise_OSError(MP_ENOTDIR);
        }
        sprintf(littlefs_current_dir, "/%s", lpath);
        if (w25qxx_debug) LOGD(TAG, "CHDIR currdir=[%s]", littlefs_current_dir);
    }
    else sprintf(littlefs_current_dir, "%s", "");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(littlefs_vfs_chdir_obj, littlefs_vfs_chdir);

// Get the current directory.
//------------------------------------------------
STATIC mp_obj_t littlefs_vfs_getcwd(mp_obj_t vfs_in)
{
    if (w25qxx_debug) LOGD(TAG, "GETCWD [%s]", littlefs_current_dir);
    return mp_obj_new_str(littlefs_current_dir, strlen(littlefs_current_dir));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(littlefs_vfs_getcwd_obj, littlefs_vfs_getcwd);

// function stat(path)
// Get the status of a file or directory.
//------------------------------------------------------------------
STATIC mp_obj_t littlefs_vfs_stat(mp_obj_t vfs_in, mp_obj_t path_in)
{
    littlefs_user_mount_t *self = MP_OBJ_TO_PTR(vfs_in);
    const char *path = mp_obj_str_get_str(path_in);

    mp_int_t mode = MP_S_IFDIR;
    mp_int_t size = 0;
    mp_int_t time = 0;

    if ((path[0] != 0) && !(path[0] == '/' && path[1] == 0)) {
        const char *lpath = littlefs_local_path(path);
        struct lfs_info info;

        int res = lfs_stat(&self->fs->lfs, lpath, &info);
        if (res != LFS_ERR_OK) {
            if (w25qxx_debug) LOGD(TAG, "STAT error (%d)", res);
            mp_raise_OSError(map_lfs_error(res));
        }

        size = info.size;
        if (info.type == LFS_TYPE_DIR) mode = MP_S_IFDIR;
        else mode = MP_S_IFREG;
        int timestamp = get_timestamp(&self->fs->lfs, lpath);
        if (timestamp >= 0) time = timestamp;
        else {
            if (w25qxx_debug) LOGD(TAG, "GET_ATTR error (%d)", timestamp);
        }
    }

    mp_obj_tuple_t *t = MP_OBJ_TO_PTR(mp_obj_new_tuple(10, NULL));
    t->items[0] = mp_obj_new_int(mode); // st_mode
    t->items[1] = mp_obj_new_int(0); // st_ino
    t->items[2] = mp_obj_new_int(0); // st_dev
    t->items[3] = mp_obj_new_int(0); // st_nlink
    t->items[4] = mp_obj_new_int(0); // st_uid
    t->items[5] = mp_obj_new_int(0); // st_gid
    t->items[6] = mp_obj_new_int_from_uint(size); // st_size
    t->items[7] = mp_obj_new_int(time); // st_atime
    t->items[8] = t->items[7]; // st_mtime
    t->items[9] = t->items[7]; // st_ctime

    return MP_OBJ_FROM_PTR(t);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(littlefs_vfs_stat_obj, littlefs_vfs_stat);

// Get the status of a VFS.
//---------------------------------------------------------------------
STATIC mp_obj_t littlefs_vfs_statvfs(mp_obj_t vfs_in, mp_obj_t path_in)
{
    littlefs_user_mount_t *self = MP_OBJ_TO_PTR(vfs_in);

    int used = lfs_fs_size(&self->fs->lfs);
    if (used < 0) {
        mp_raise_OSError(map_lfs_error(used));
    }
    mp_obj_tuple_t *t = MP_OBJ_TO_PTR(mp_obj_new_tuple(10, NULL));

    t->items[0] = mp_obj_new_int(LITTLEFS_CFG_SECTOR_SIZE);                                 // file system block size
    t->items[1] = t->items[0];                                                              // fragment size
    t->items[2] = mp_obj_new_int(LITTLEFS_CFG_PHYS_SZ/LITTLEFS_CFG_SECTOR_SIZE);            // size of fs in f_frsize units
    t->items[3] = mp_obj_new_int((LITTLEFS_CFG_PHYS_SZ/LITTLEFS_CFG_SECTOR_SIZE) - used);   // f_bfree
    t->items[4] = t->items[3];                                                              // f_bavail
    t->items[5] = mp_obj_new_int(0); // f_files
    t->items[6] = mp_obj_new_int(0); // f_ffree
    t->items[7] = mp_obj_new_int(0); // f_favail
    t->items[8] = mp_obj_new_int(0); // f_flags
    t->items[9] = mp_obj_new_int(LITTLEFS_CFG_MAX_NAME_LEN);                                // f_namemax

    return MP_OBJ_FROM_PTR(t);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(littlefs_vfs_statvfs_obj, littlefs_vfs_statvfs);

//----------------------------------------------------------------------------------
STATIC mp_obj_t vfs_littlefs_mount(mp_obj_t self_in, mp_obj_t readonly, mp_obj_t mkfs)
{
    mp_raise_NotImplementedError("System littlefs is always mounted");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(vfs_littlefs_mount_obj, vfs_littlefs_mount);

//-------------------------------------------------
STATIC mp_obj_t vfs_littlefs_umount(mp_obj_t self_in)
{
    mp_raise_NotImplementedError("Only system littlefs is used");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(littlefs_vfs_umount_obj, vfs_littlefs_umount);


//------------------------------------------------------------------------
STATIC mp_obj_t vfs_littlefs_counters(size_t n_args, const mp_obj_t *args)
{
    uint32_t rd, wr, er;
    uint64_t spitime;
    w25qxx_get_counters(&rd, &wr, &er, &spitime);

    mp_obj_tuple_t *t = MP_OBJ_TO_PTR(mp_obj_new_tuple(4, NULL));
    t->items[0] = mp_obj_new_int(rd);
    t->items[1] = mp_obj_new_int(wr);
    t->items[2] = mp_obj_new_int(er);
    t->items[3] = mp_obj_new_int(spitime);

    if (n_args > 1) {
        if (mp_obj_is_true(args[1])) w25qxx_clear_counters();
    }
    return MP_OBJ_FROM_PTR(t);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(littlefs_vfs_counters_obj, 1, 2, vfs_littlefs_counters);


//===============================================================
STATIC const mp_rom_map_elem_t littlefs_vfs_locals_dict_table[] = {
    #if _FS_REENTRANT
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&littlefs_vfs_del_obj) },
    #endif
    { MP_ROM_QSTR(MP_QSTR_mkfs),        MP_ROM_PTR(&littlefs_vfs_mkfs_obj) },
    { MP_ROM_QSTR(MP_QSTR_open),        MP_ROM_PTR(&littlefs_vfs_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_mount),       MP_ROM_PTR(&vfs_littlefs_mount_obj) },
    { MP_ROM_QSTR(MP_QSTR_ilistdir),    MP_ROM_PTR(&littlefs_vfs_ilistdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_chdir),       MP_ROM_PTR(&littlefs_vfs_chdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_mkdir),       MP_ROM_PTR(&littlefs_vfs_mkdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_rmdir),       MP_ROM_PTR(&littlefs_vfs_rmdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_getcwd),      MP_ROM_PTR(&littlefs_vfs_getcwd_obj) },
    { MP_ROM_QSTR(MP_QSTR_remove),      MP_ROM_PTR(&littlefs_vfs_remove_obj) },
    { MP_ROM_QSTR(MP_QSTR_rename),      MP_ROM_PTR(&littlefs_vfs_rename_obj) },
    { MP_ROM_QSTR(MP_QSTR_stat),        MP_ROM_PTR(&littlefs_vfs_stat_obj) },
    { MP_ROM_QSTR(MP_QSTR_statvfs),     MP_ROM_PTR(&littlefs_vfs_statvfs_obj) },
    { MP_ROM_QSTR(MP_QSTR_umount),      MP_ROM_PTR(&littlefs_vfs_umount_obj) },
    { MP_ROM_QSTR(MP_QSTR_counters),    MP_ROM_PTR(&littlefs_vfs_counters_obj) },
};
STATIC MP_DEFINE_CONST_DICT(littlefs_vfs_locals_dict, littlefs_vfs_locals_dict_table);


STATIC const mp_vfs_proto_t littlefs_vfs_proto = {
    .import_stat = littlefs_vfs_import_stat,
};

const mp_obj_type_t mp_littlefs_vfs_type = {
    { &mp_type_type },
    .name = MP_QSTR_VfsFlashfs,
    .make_new = littlefs_vfs_make_new,
    .protocol = &littlefs_vfs_proto,
    .locals_dict = (mp_obj_dict_t*)&littlefs_vfs_locals_dict,
};

#endif // MICROPY_VFS_LITTLEFS
