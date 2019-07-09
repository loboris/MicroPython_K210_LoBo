/*
 * This file is part of the MicroPython ESP32 project, https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 LoBo (https://github.com/loboris)
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

#include "py/mpconfig.h"
#if MICROPY_VFS_SDCARD

#if !MICROPY_VFS
#error "with MICROPY_VFS_SDCARD enabled, MICROPY_VFS must also be enabled"
#endif

#include <string.h>
#include <sys/stat.h>
#include "pin_cfg.h"

#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/binary.h"
#include "py/objarray.h"

#include "lib/timeutils/timeutils.h"
#include "vfs_sdcard.h"
#include <devices.h>
#include <filesystem.h>
#include <storage/sdcard.h>
#include "syslog.h"
#include "modmachine.h"

static bool sdcard_pins_init = false;
static int sdcard_cs_gpionum = 4;
static mp_fpioa_cfg_item_t sdcard_pin_func[4];

#if FF_MAX_SS == FF_MIN_SS
#define SECSIZE(fs) (FF_MIN_SS)
#else
#define SECSIZE(fs) ((fs)->ssize)
#endif

#define GET_ERR_CODE(res) ((-res)-10000+1)
#define mp_obj_sdcard_vfs_t sdcard_user_mount_t

#define FORMAT_FS_FORCE 1

typedef struct _mp_vfs_sdcard_ilistdir_it_t {
    mp_obj_base_t base;
    mp_fun_1_t iternext;
    bool is_str;
    DIR dir;
} mp_vfs_sdcard_ilistdir_it_t;

const mp_obj_type_t mp_sdcard_vfs_type;

// this table converts from FRESULT to POSIX errno
const byte fresult_to_errno_table[20] = {
    [FR_OK] = 0,
    [FR_DISK_ERR] = MP_EIO,
    [FR_INT_ERR] = MP_EIO,
    [FR_NOT_READY] = MP_EBUSY,
    [FR_NO_FILE] = MP_ENOENT,
    [FR_NO_PATH] = MP_ENOENT,
    [FR_INVALID_NAME] = MP_EINVAL,
    [FR_DENIED] = MP_EACCES,
    [FR_EXIST] = MP_EEXIST,
    [FR_INVALID_OBJECT] = MP_EINVAL,
    [FR_WRITE_PROTECTED] = MP_EROFS,
    [FR_INVALID_DRIVE] = MP_ENODEV,
    [FR_NOT_ENABLED] = MP_ENODEV,
    [FR_NO_FILESYSTEM] = MP_ENODEV,
    [FR_MKFS_ABORTED] = MP_EIO,
    [FR_TIMEOUT] = MP_EIO,
    [FR_LOCKED] = MP_EIO,
    [FR_NOT_ENOUGH_CORE] = MP_ENOMEM,
    [FR_TOO_MANY_OPEN_FILES] = MP_EMFILE,
    [FR_INVALID_PARAMETER] = MP_EINVAL,
};

static const char* TAG = "[VFS_SDCARD]";

static char sdcard_current_dir[FF_MAX_LFN-8] = {'\0'};
static char sdcard_file_path[FF_MAX_LFN] = {'\0'};

//--------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t sdcard_vfs_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 1, false);
    // create new object
    sdcard_user_mount_t *vfs = m_new_obj(sdcard_user_mount_t);
    vfs->base.type = &mp_sdcard_vfs_type;
    vfs->flags = MODULE_SDCARD;

    return MP_OBJ_FROM_PTR(vfs);
}

//----------------------------------------------------------------------------------
STATIC mp_obj_t vfs_sdcard_mount(mp_obj_t self_in, mp_obj_t readonly, mp_obj_t mkfs)
{
    sdcard_user_mount_t *self = MP_OBJ_TO_PTR(self_in);
    handle_t sd_spi = 0, sd_gpio = 0, sdcard = 0;
    bool is_init = false;
    int res;


    // Initialize SDCard
    if (!sdcard_pins_init) {
        sdcard_cs_gpionum = gpiohs_get_free();
        if (sdcard_cs_gpionum < 0) {
            mp_raise_msg(&mp_type_OSError, "Error preparing SDCard pins");
        }

        sdcard_pin_func[0] = (mp_fpioa_cfg_item_t){sdcard_cs_gpionum, 29, GPIO_USEDAS_CS, FUNC_GPIOHS0 + sdcard_cs_gpionum};
        sdcard_pin_func[1] = (mp_fpioa_cfg_item_t){-1, 27, GPIO_USEDAS_CLK, FUNC_SPI1_SCLK};
        sdcard_pin_func[2] = (mp_fpioa_cfg_item_t){-1, 28, GPIO_USEDAS_DATA0, FUNC_SPI1_D0};
        sdcard_pin_func[3] = (mp_fpioa_cfg_item_t){-1, 26, GPIO_USEDAS_DATA1, FUNC_SPI1_D1};

        if (!fpioa_check_pins(4, sdcard_pin_func, GPIO_FUNC_SDCARD)) {
            gpiohs_set_free(sdcard_cs_gpionum);
            mp_raise_msg(&mp_type_OSError, "Error preparing SDCard pins");
        }
        fpioa_setup_pins(4, sdcard_pin_func);
        fpioa_setused_pins(4, sdcard_pin_func, GPIO_FUNC_SDCARD);
        sdcard_pins_init = true;
    }

    sd_spi = io_open("/dev/spi1");
    if (sd_spi) {
        sd_gpio = io_open("/dev/gpio0");
        if (sd_gpio) {
            sdcard = spi_sdcard_driver_install(sd_spi, sd_gpio, sdcard_cs_gpionum);
            if (sdcard) {
                res = filesystem_mount("/fs/0/", sdcard);
                if (res == 0) is_init = true;
                else self->flags |= FSUSER_NO_FILESYSTEM;
            }
        }
    }

    if (sd_spi) io_close(sd_spi);
    if (sd_gpio) io_close(sd_gpio);
    if (sdcard) io_close(sdcard);

    if (!is_init) {
        mp_raise_OSError(MP_EINVAL);
    }
    // check if we need to make the filesystem
    res = (self->flags & FSUSER_NO_FILESYSTEM) ? FR_NO_FILESYSTEM : FR_OK;
    if ((res == FR_NO_FILESYSTEM) && mp_obj_is_true(mkfs)) {
        mp_printf(&mp_plat_print, "Creating file system on SD Card ...");
        uint8_t working_buf[FF_MAX_SS];
        res = f_mkfs("", FM_FAT32, 0, working_buf, sizeof(working_buf));
    }
    if (res != FR_OK) {
        mp_raise_OSError(fresult_to_errno_table[res]);
    }
    self->flags &= ~FSUSER_NO_FILESYSTEM;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(vfs_sdcard_mount_obj, vfs_sdcard_mount);

//-------------------------------------------------
STATIC mp_obj_t vfs_sdcard_umount(mp_obj_t self_in)
{
    //sdcard_user_mount_t *self = MP_OBJ_TO_PTR(self_in);

    f_mount(NULL, "0/", 0);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sdcard_vfs_umount_obj, vfs_sdcard_umount);


//--------------------------------------------------------------------------
const char *sdcard_local_path(const char *path, sdcard_user_mount_t *vfsobj)
{
    const char *lpath = path;
    if (lpath[0] == '/') {
        lpath++; // absolute path
        sprintf(sdcard_file_path, "0/%s", lpath);
        int len = strlen(sdcard_file_path);
        if ((len > 2) && (lpath[len-1] == '/')) {
            sdcard_file_path[len-1] = '\0';
            lpath = sdcard_file_path;
        }
    }
    else {
        if (strstr(lpath, "..") == lpath) {
            // parent directory
            lpath += 2;
            if (lpath[0] == '/') lpath++;
            if (sdcard_current_dir[0] == '/')
                sprintf(sdcard_file_path, "0%s", sdcard_current_dir);
            else
                sprintf(sdcard_file_path, "0/%s", sdcard_current_dir);
            int len = strlen(sdcard_file_path);
            for (int i=len; i>0; i--) {
                if (sdcard_file_path[i] == '/') {
                    if (i == 1) sdcard_file_path[i+1] = '\0';
                    else sdcard_file_path[i] = '\0';
                    break;
                }
            }
            strcat(sdcard_file_path, lpath);
        }
        else {
            const char * mount_point = NULL;
            int mplen = 0;
            for (mp_vfs_mount_t **vfsp = &MP_STATE_VM(vfs_mount_table); *vfsp != NULL; vfsp = &(*vfsp)->next) {
                if ((*vfsp)->obj == (mp_obj_t)vfsobj) {
                    mount_point = (*vfsp)->str;
                    mplen = (*vfsp)->len;
                    break;
                }
            }
            if (strstr(lpath, ".") == lpath) lpath += 1;
            if ((mount_point != NULL) && (mplen > 1)) {
                if (strstr(lpath, mount_point) == lpath) lpath += mplen;
            }
            else {
                LOGE(TAG, "LOCAL_PATH mount point not found");
            }
            if (sdcard_current_dir[0] == '/')
                snprintf(sdcard_file_path, FF_MAX_LFN, "0%s/%s", sdcard_current_dir, lpath);
            else
                snprintf(sdcard_file_path, FF_MAX_LFN, "0/%s/%s", sdcard_current_dir, lpath);
        }
        lpath = sdcard_file_path;
        int len = strlen(lpath);
        while ((len > 2) && (lpath[len-1] == '/')) {
            sdcard_file_path[len-1] = '\0';
            len = strlen(sdcard_file_path);
        }
    }
    LOGD(TAG, "LOCAL_PATH [%s]->[%s], currdir=[%s] %s", path, lpath, sdcard_current_dir, sdcard_file_path);
    return lpath;
}

//---------------------------------------------------------------------------
STATIC mp_import_stat_t sdcard_vfs_import_stat(void *vfs_in,const char *path)
{
    sdcard_user_mount_t *vfs = vfs_in;
    if (vfs == NULL) return MP_IMPORT_STAT_NO_EXIST;
    if (path[0] == 0 || (path[0] == '/' && path[1] == 0)) {
        return MP_IMPORT_STAT_DIR;
    }

    FILINFO fno;
    int res;
    const char *lpath = sdcard_local_path(path, vfs);
    res = f_stat(lpath, &fno);
    if (res != 0) return MP_IMPORT_STAT_NO_EXIST;

    if (fno.fattrib & AM_DIR) return MP_IMPORT_STAT_DIR;

    return MP_IMPORT_STAT_FILE;
}

#if FF_FS_REENTRANT
//----------------------------------------------
STATIC mp_obj_t sdcard_vfs_del(mp_obj_t self_in)
{
    //sdcard_user_mount_t *self = MP_OBJ_TO_PTR(self_in);
    // f_umount only needs to be called to release the sync object
    f_mount(NULL, "0/", 0);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sdcard_vfs_del_obj, sdcard_vfs_del);
#endif

//-----------------------------------------------
STATIC mp_obj_t sdcard_vfs_mkfs(mp_obj_t self_in)
{
    // create new object
    //sdcard_user_mount_t *vfs = MP_OBJ_TO_PTR(sdcard_vfs_make_new(&mp_sdcard_vfs_type, 0, 0, mp_const_none));

    // make the filesystem
    mp_printf(&mp_plat_print, "Creating file system on SD Card ...");
    uint8_t working_buf[FF_MAX_SS];
    FRESULT res = f_mkfs("0/", FM_FAT32, 0, working_buf, sizeof(working_buf));
    if (res != FR_OK) {
        mp_raise_OSError(fresult_to_errno_table[res]);
    }

    //mp_raise_NotImplementedError("mkfs not implemented");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sdcard_vfs_mkfs_fun_obj, sdcard_vfs_mkfs);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(sdcard_vfs_mkfs_obj, MP_ROM_PTR(&sdcard_vfs_mkfs_fun_obj));


//------------------------------------------------------------------
STATIC mp_obj_t mp_vfs_sdcard_ilistdir_it_iternext(mp_obj_t self_in)
{
    mp_vfs_sdcard_ilistdir_it_t *self = MP_OBJ_TO_PTR(self_in);

    FILINFO fno;
    for (;;) {
        int res = f_readdir(&self->dir, &fno);
        if ((res != 0) || (strlen(fno.fname) == 0)) {
            // stop on error or end of dir
            break;
        }

        char *fn = fno.fname;
        // filter . and ..
        if (fn[0] == '.' && ((fn[1] == '.' && fn[2] == 0) || fn[1] == 0)) continue;

        // make 4-tuple with info about this entry
        mp_obj_tuple_t *t = MP_OBJ_TO_PTR(mp_obj_new_tuple(4, NULL));
        if (self->is_str) {
            t->items[0] = mp_obj_new_str(fn, strlen(fn));
        }
        else {
            t->items[0] = mp_obj_new_bytes((const byte*)fn, strlen(fn));
        }
        if (fno.fattrib & AM_DIR) {
            // dir
            t->items[1] = mp_obj_new_int(MP_S_IFDIR);
        } 
		else {
            // file
            t->items[1] = mp_obj_new_int(MP_S_IFREG);
        }
        t->items[2] = mp_obj_new_int(0); // no inode number
        t->items[3] = mp_obj_new_int_from_uint(fno.fsize);

        return MP_OBJ_FROM_PTR(t);
    }

    // ignore error because we may be closing a second time
    f_closedir(&self->dir);

    return MP_OBJ_STOP_ITERATION;
}

//---------------------------------------------------------------------------
STATIC mp_obj_t sdcard_vfs_ilistdir_func(size_t n_args, const mp_obj_t *args)
{
    sdcard_user_mount_t *self = MP_OBJ_TO_PTR(args[0]);
    bool is_str_type = true;

    const char *path;
    const char *lpath;
    if (n_args == 2) {
        if (mp_obj_get_type(args[1]) == &mp_type_bytes) {
            is_str_type = false;
        }
        path = mp_obj_str_get_str(args[1]);
        lpath = sdcard_local_path(path, self);
    }
    else lpath =sdcard_local_path("", self);
    LOGD(TAG, "LISTDIR [%s]", lpath);

    // Create a new iterator object to list the dir
    mp_vfs_sdcard_ilistdir_it_t *iter = m_new_obj(mp_vfs_sdcard_ilistdir_it_t);
    iter->base.type = &mp_type_polymorph_iter;
    iter->iternext = mp_vfs_sdcard_ilistdir_it_iternext;
    iter->is_str = is_str_type;

    int res = f_opendir(&iter->dir, lpath);
    if (res != 0) {
        mp_raise_OSError(res);
        return mp_const_none;
    }

    return MP_OBJ_FROM_PTR(iter);
	
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sdcard_vfs_ilistdir_obj, 1, 2, sdcard_vfs_ilistdir_func);

//------------------------------------------------------------------------------------------
STATIC mp_obj_t sdcard_vfs_remove_internal(mp_obj_t vfs_in, mp_obj_t path_in, mp_int_t attr)
{
    sdcard_user_mount_t *self = MP_OBJ_TO_PTR(vfs_in);
    const char *path = mp_obj_str_get_str(path_in);
    const char *lpath = sdcard_local_path(path, self);

    FILINFO fno;
    FRESULT res = f_stat(lpath, &fno);

    if (res != FR_OK) {
        mp_raise_OSError(fresult_to_errno_table[res]);
    }

    // check if path is a file or directory
    if ((fno.fattrib & AM_DIR) == attr) {
        res = f_unlink(lpath);

        if (res != FR_OK) {
            mp_raise_OSError(fresult_to_errno_table[res]);
        }
        return mp_const_none;
    }
    else {
        mp_raise_OSError(attr ? MP_ENOTDIR : MP_EISDIR);
    }
    return mp_const_none;
}

//------------------------------------------------------------------
STATIC mp_obj_t sdcard_vfs_remove(mp_obj_t vfs_in, mp_obj_t path_in)
{
    return sdcard_vfs_remove_internal(vfs_in, path_in, 0); // 0 == file attribute
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sdcard_vfs_remove_obj, sdcard_vfs_remove);

//-----------------------------------------------------------------
STATIC mp_obj_t sdcard_vfs_rmdir(mp_obj_t vfs_in, mp_obj_t path_in)
{
    return sdcard_vfs_remove_internal(vfs_in, path_in, AM_DIR);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sdcard_vfs_rmdir_obj, sdcard_vfs_rmdir);

//-------------------------------------------------------------------------------------
STATIC mp_obj_t sdcard_vfs_rename(mp_obj_t vfs_in, mp_obj_t path_in, mp_obj_t path_out)
{
    mp_obj_sdcard_vfs_t *self = MP_OBJ_TO_PTR(vfs_in);
    const char *old_path = mp_obj_str_get_str(path_in);
    const char *new_path = mp_obj_str_get_str(path_out);
    const char *local_oldpath = sdcard_local_path(old_path, self);
    const char *local_newpath = sdcard_local_path(new_path, self);

    FRESULT res = f_rename(local_oldpath, local_newpath);
    if (res == FR_EXIST) {
        // if new_path exists then try removing it (but only if it's a file)
        sdcard_vfs_remove_internal(vfs_in, path_out, 0); // 0 == file attribute
        // try to rename again
        res = f_rename(local_oldpath, local_newpath);
    }
    if (res == FR_OK) {
        return mp_const_none;
    } else {
        mp_raise_OSError(fresult_to_errno_table[res]);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(sdcard_vfs_rename_obj, sdcard_vfs_rename);

//----------------------------------------------------------------
STATIC mp_obj_t sdcard_vfs_mkdir(mp_obj_t vfs_in, mp_obj_t path_o)
{
    mp_obj_sdcard_vfs_t *self = MP_OBJ_TO_PTR(vfs_in);
    const char *path = mp_obj_str_get_str(path_o);
    const char *lpath = sdcard_local_path(path, self);

    FRESULT res = f_mkdir(lpath);
    if (res == FR_OK) {
        return mp_const_none;
    }
    else {
        mp_raise_OSError(fresult_to_errno_table[res]);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sdcard_vfs_mkdir_obj, sdcard_vfs_mkdir);

// Change current directory.
//-----------------------------------------------------------------
STATIC mp_obj_t sdcard_vfs_chdir(mp_obj_t vfs_in, mp_obj_t path_in)
{
    mp_obj_sdcard_vfs_t *self = MP_OBJ_TO_PTR(vfs_in);
    const char *path;
    path = mp_obj_str_get_str(path_in);

    if ((path[0] != 0) && !(path[0] == '/' && path[1] == 0)) {
        const char *lpath = sdcard_local_path(path, self);
        // Check if directory
        FILINFO fno;
        int res = f_stat(lpath, &fno);
        if (res != FR_OK) {
            mp_raise_OSError(fresult_to_errno_table[res]);
        }

        if (!(fno.fattrib & AM_DIR)) {
            // Not a directory
            mp_raise_OSError(MP_ENOTDIR);
        }
        sprintf(sdcard_current_dir, "/%s", lpath);
        LOGD(TAG, "CHDIR currdir=[%s]", sdcard_current_dir);
    }
    else sprintf(sdcard_current_dir, "%s", "");

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sdcard_vfs_chdir_obj, sdcard_vfs_chdir);

// Get the current directory.
//------------------------------------------------
STATIC mp_obj_t sdcard_vfs_getcwd(mp_obj_t vfs_in)
{
    /*
    //mp_obj_sdcard_vfs_t *self = MP_OBJ_TO_PTR(vfs_in);
    char buf[MICROPY_ALLOC_PATH_MAX + 1];

    FRESULT res = f_getcwd(buf, sizeof(buf));
    if (res != FR_OK) {
        mp_raise_OSError(fresult_to_errno_table[res]);
    }
    return mp_obj_new_str(buf, strlen(buf));
    */
    LOGD(TAG, "GETCWD [%s]", sdcard_current_dir);
    return mp_obj_new_str(sdcard_current_dir, strlen(sdcard_current_dir));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sdcard_vfs_getcwd_obj, sdcard_vfs_getcwd);

// function stat(path)
// Get the status of a file or directory.
//----------------------------------------------------------------
STATIC mp_obj_t sdcard_vfs_stat(mp_obj_t vfs_in, mp_obj_t path_in)
{
    sdcard_user_mount_t *self = MP_OBJ_TO_PTR(vfs_in);
    const char *path = mp_obj_str_get_str(path_in);
    FILINFO fno;

    if (path[0] == 0 || (path[0] == '/' && path[1] == 0)) {
        // stat root directory
        fno.fsize = 0;
        fno.fdate = 0x2821; // Jan 1, 2000
        fno.ftime = 0;
        fno.fattrib = AM_DIR;
    }
    else {
        int res;
        const char *lpath = sdcard_local_path(path, self);
        res = f_stat(lpath, &fno);
        if (res != 0) {
            mp_raise_OSError(fresult_to_errno_table[res]);
        }
    }
    mp_obj_tuple_t *t = MP_OBJ_TO_PTR(mp_obj_new_tuple(10, NULL));
    mp_int_t mode = 0;
    if (fno.fattrib & AM_DIR) {
        mode |= MP_S_IFDIR;
    } else {
        mode |= MP_S_IFREG;
    }
    mp_int_t seconds = timeutils_seconds_since_2000(
        1980 + ((fno.fdate >> 9) & 0x7f),
        (fno.fdate >> 5) & 0x0f,
        fno.fdate & 0x1f,
        (fno.ftime >> 11) & 0x1f,
        (fno.ftime >> 5) & 0x3f,
        2 * (fno.ftime & 0x1f)
    );
    seconds += 946684800; // Convert to UNIX time
    t->items[0] = mp_obj_new_int(mode); // st_mode
    t->items[1] = mp_obj_new_int(0); // st_ino
    t->items[2] = mp_obj_new_int(0); // st_dev
    t->items[3] = mp_obj_new_int(0); // st_nlink
    t->items[4] = mp_obj_new_int(0); // st_uid
    t->items[5] = mp_obj_new_int(0); // st_gid
    t->items[6] = mp_obj_new_int_from_uint(fno.fsize); // st_size
    t->items[7] = mp_obj_new_int(seconds); // st_atime
    t->items[8] = mp_obj_new_int(seconds); // st_mtime
    t->items[9] = mp_obj_new_int(seconds); // st_ctime

    return MP_OBJ_FROM_PTR(t);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sdcard_vfs_stat_obj, sdcard_vfs_stat);


// Get the status of a VFS.
//-------------------------------------------------------------------
STATIC mp_obj_t sdcard_vfs_statvfs(mp_obj_t vfs_in, mp_obj_t path_in)
{
    sdcard_user_mount_t *self = MP_OBJ_TO_PTR(vfs_in);

    DWORD nclst;
    FRESULT res = f_getfree("", &nclst, &self->fs);
    if (FR_OK != res) {
        mp_raise_OSError(fresult_to_errno_table[res]);
    }

    mp_obj_tuple_t *t = MP_OBJ_TO_PTR(mp_obj_new_tuple(10, NULL));

    t->items[0] = mp_obj_new_int(self->fs->csize * SECSIZE(self->fs)); // f_bsize
    t->items[1] = t->items[0]; // f_frsize
    t->items[2] = mp_obj_new_int((self->fs->n_fatent - 2)); // f_blocks
    t->items[3] = mp_obj_new_int(nclst); // f_bfree
    t->items[4] = t->items[3]; // f_bavail
    t->items[5] = mp_obj_new_int(0); // f_files
    t->items[6] = mp_obj_new_int(0); // f_ffree
    t->items[7] = mp_obj_new_int(0); // f_favail
    t->items[8] = mp_obj_new_int(0); // f_flags
    t->items[9] = mp_obj_new_int(FF_MAX_LFN); // f_namemax

    return MP_OBJ_FROM_PTR(t);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sdcard_vfs_statvfs_obj, sdcard_vfs_statvfs);

STATIC const mp_rom_map_elem_t sdcard_vfs_locals_dict_table[] = {
    #if FF_FS_REENTRANT
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&sdcard_vfs_del_obj) },
    #endif
    { MP_ROM_QSTR(MP_QSTR_mkfs), MP_ROM_PTR(&sdcard_vfs_mkfs_obj) },
    { MP_ROM_QSTR(MP_QSTR_open), MP_ROM_PTR(&sdcard_vfs_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_openex), MP_ROM_PTR(&sdcard_vfs_open_ex_obj) },
    { MP_ROM_QSTR(MP_QSTR_mount), MP_ROM_PTR(&vfs_sdcard_mount_obj) },
    { MP_ROM_QSTR(MP_QSTR_ilistdir), MP_ROM_PTR(&sdcard_vfs_ilistdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_chdir), MP_ROM_PTR(&sdcard_vfs_chdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_mkdir), MP_ROM_PTR(&sdcard_vfs_mkdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_rmdir), MP_ROM_PTR(&sdcard_vfs_rmdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_getcwd), MP_ROM_PTR(&sdcard_vfs_getcwd_obj) },
    { MP_ROM_QSTR(MP_QSTR_remove), MP_ROM_PTR(&sdcard_vfs_remove_obj) },
    { MP_ROM_QSTR(MP_QSTR_rename), MP_ROM_PTR(&sdcard_vfs_rename_obj) },
    { MP_ROM_QSTR(MP_QSTR_stat), MP_ROM_PTR(&sdcard_vfs_stat_obj) },
    { MP_ROM_QSTR(MP_QSTR_statvfs), MP_ROM_PTR(&sdcard_vfs_statvfs_obj) },
    { MP_ROM_QSTR(MP_QSTR_umount), MP_ROM_PTR(&sdcard_vfs_umount_obj) },
};
STATIC MP_DEFINE_CONST_DICT(sdcard_vfs_locals_dict, sdcard_vfs_locals_dict_table);


STATIC const mp_vfs_proto_t sdcard_vfs_proto = {
    .import_stat = sdcard_vfs_import_stat,
};

const mp_obj_type_t mp_sdcard_vfs_type = {
    { &mp_type_type },
    .name = MP_QSTR_VfsSDCard,
    .make_new = sdcard_vfs_make_new,
    .protocol = &sdcard_vfs_proto,
    .locals_dict = (mp_obj_dict_t*)&sdcard_vfs_locals_dict,
};

#endif

