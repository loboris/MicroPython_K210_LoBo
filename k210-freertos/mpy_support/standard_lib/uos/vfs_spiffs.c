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

#include "py/mpconfig.h"
#if MICROPY_VFS_SPIFFS

#if !MICROPY_VFS
#error "with MICROPY_VFS_SPIFFS enabled, MICROPY_VFS must also be enabled"
#endif

#include <string.h>
#include <sys/stat.h>
#include "syslog.h"
#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/binary.h"
#include "py/objarray.h"

#include "lib/timeutils/timeutils.h"

#include "spiffs_nucleus.h"
#include "vfs_spiffs.h"
#include "spiffs_config.h"
#include "w25qxx.h"

int spiffs_dbg_level = 0;

#if _MAX_SS == _MIN_SS
#define SECSIZE(fs) (_MIN_SS)
#else
#define SECSIZE(fs) ((fs)->ssize)
#endif

#define mp_obj_spiffs_vfs_t spiffs_user_mount_t

#define FORMAT_FS_FORCE 0
#define SPIFFS_MAX_OPEN_FILES   4

const mp_obj_type_t mp_spiffs_vfs_type;

static char spiffs_current_dir[SPIFFS_OBJ_NAME_LEN-8] = {'\0'};
static char spiffs_file_path[SPIFFS_OBJ_NAME_LEN] = {'\0'};

static u8_t spiffs_work_buf[SPIFFS_CFG_LOG_PAGE_SZ(fs) * 2];
static u8_t spiffs_fds[sizeof(spiffs_fd) * SPIFFS_MAX_OPEN_FILES + 8] = {0};
#if SPIFFS_CACHE
static u8_t spiffs_cache_buf[SPIFFS_CFG_LOG_PAGE_SZ(fs)*2];
#endif
static SemaphoreHandle_t spiffs_lock = NULL; // FS lock

static const char* TAG = "[VFS_SPIFFS]";

spiffs_user_mount_t spiffs_user_mount_handle;

static int *fs_check = NULL;

typedef struct _mp_vfs_spiffs_ilistdir_it_t {
    mp_obj_base_t base;
    mp_fun_1_t iternext;
    bool is_str;
    spiffs_DIR dir;
    char path[SPIFFS_OBJ_NAME_LEN];
} mp_vfs_spiffs_ilistdir_it_t;


#if SPIFFS_HAL_CALLBACK_EXTRA
//--------------------------------------------------------------------------------------------------------------------
static void mp_spiffs_check_cb(spiffs *fs, spiffs_check_type type, spiffs_check_report report, u32_t arg1, u32_t arg2)
#else
//--------------------------------------------------------------------------------------------------------
static void mp_spiffs_check_cb(spiffs_check_type type, spiffs_check_report report, u32_t arg1, u32_t arg2)
#endif
{
    if (fs_check) {
        if (report == SPIFFS_CHECK_PROGRESS) fs_check[SPIFFS_CHECK_PROGRESS]++;
        else if (report == SPIFFS_CHECK_ERROR) fs_check[SPIFFS_CHECK_ERROR]++;
        else if (report == SPIFFS_CHECK_FIX_INDEX) fs_check[SPIFFS_CHECK_FIX_INDEX]++;
        else if (report == SPIFFS_CHECK_FIX_LOOKUP) fs_check[SPIFFS_CHECK_FIX_LOOKUP]++;
        else if (report == SPIFFS_CHECK_DELETE_ORPHANED_INDEX) fs_check[SPIFFS_CHECK_DELETE_ORPHANED_INDEX]++;
        else if (report == SPIFFS_CHECK_DELETE_PAGE) fs_check[SPIFFS_CHECK_DELETE_PAGE]++;
        else if (report == SPIFFS_CHECK_DELETE_BAD_FILE) fs_check[SPIFFS_CHECK_DELETE_BAD_FILE]++;
    }
    return;
}

//------------------------------
void spiffs_api_lock(spiffs *fs)
{
    xSemaphoreTake(spiffs_lock, portMAX_DELAY);
}

//--------------------------------
void spiffs_api_unlock(spiffs *fs)
{
    xSemaphoreGive(spiffs_lock);
}

// Create a default 'boot.py' file
//----------------------------
void check_boot_py(spiffs *fs)
{
    spiffs_file fd;
    spiffs_stat fno;
    int res = SPIFFS_stat(fs, "boot.py", &fno);
    if (res == SPIFFS_OK) {
        LOGD(TAG, "boot.py already exists");
        return;
    }

    fd = SPIFFS_open(fs, "boot.py", SPIFFS_O_RDWR | SPIFFS_O_CREAT, 0);
    if(fd <= 0) {
        LOGE(TAG, "Error creating boot.py");
    }
    vfs_spiffs_update_meta(fs, fd, SPIFFS_TYPE_FILE);
    char buf[128] = {'\0'};
    sprintf(buf, "# This file is executed on every boot\nimport sys\nsys.path.append('/flash/lib')\n");
    int len = strlen(buf);
    res = SPIFFS_write(fs, fd, buf, len);
    if ((res < 0) || (res != len)){
        SPIFFS_clearerr(fs);
        LOGE(TAG, "Error writing to boot.py");
    }
    res = SPIFFS_close(fs, fd);
    if (res != SPIFFS_OK) {
        SPIFFS_clearerr(fs);
        LOGW(TAG, "Error closing boot.py");
    }
    LOGD(TAG, "boot.py created");
}

// Check if 'main.py' file exists
//----------------------------
bool check_main_py(spiffs *fs)
{
    spiffs_stat fno;
    int res = SPIFFS_stat(fs, "boot.py", &fno);
    if (res == SPIFFS_OK) {
        return true;
    }
    return false;
}

//---------------------------------------------
const char *spiffs_local_path(const char *path)
{
    const char *lpath = path;
    if (lpath[0] == '/') {
        lpath++; // absolute path
        int len = strlen(lpath);
        if (lpath[len-1] == '/') {
            strcpy(spiffs_file_path, lpath);
            spiffs_file_path[len-1] = '\0';
            lpath = spiffs_file_path;
        }
    }
    else {
        if (strstr(lpath, "..") == lpath) {
            // parent directory
            lpath += 2;
            if (lpath[0] == '/') lpath++;
            sprintf(spiffs_file_path, "%s", spiffs_current_dir);
            char *ppath = strrchr(spiffs_file_path, '/');
            if (ppath) *ppath = '\0';
            strcat(spiffs_file_path, lpath);
        }
        else {
            if (strstr(lpath, ".") == lpath) lpath += 1;
            if (strstr(lpath, "flash/") == lpath) lpath += 6;
            snprintf(spiffs_file_path, SPIFFS_OBJ_NAME_LEN, "%s/%s", spiffs_current_dir, lpath);
        }
        int len = strlen(spiffs_file_path);
        while ((len > 0) && (spiffs_file_path[len-1] == '/')) {
            spiffs_file_path[len-1] = '\0';
            len = strlen(spiffs_file_path);
        }
        lpath = spiffs_file_path;
        if (lpath[0] == '/') lpath++;
    }
    LOGD(TAG, "LOCAL_PATH [%s]->[%s], currdir=[%s]", path, lpath, spiffs_current_dir);
    return lpath;
}

//----------------------------------
MP_NOINLINE bool init_flash_spiffs()
{
    spiffs_lock = xSemaphoreCreateMutex();
    configASSERT(spiffs_lock);

    w25qxx_clear_counters();
    spiffs_user_mount_t* vfs_spiffs = &spiffs_user_mount_handle;
    vfs_spiffs->flags = SYS_SPIFFS;
    vfs_spiffs->base.type = &mp_spiffs_vfs_type;
    vfs_spiffs->fs.user_data = vfs_spiffs;
    vfs_spiffs->cfg.hal_read_f = (spiffs_read)sys_spiffs_read;
    vfs_spiffs->cfg.hal_write_f = (spiffs_write)sys_spiffs_write;
    vfs_spiffs->cfg.hal_erase_f = (spiffs_erase)sys_spiffs_erase;
    #if SPIFFS_SINGLETON == 0
    vfs_spiffs->cfg.phys_size = SPIFFS_CFG_PHYS_SZ();               // use all spi flash
    vfs_spiffs->cfg.phys_addr = SPIFFS_CFG_PHYS_ADDR();             // start spiffs at start of spi flash
    vfs_spiffs->cfg.phys_erase_block = SPIFFS_CFG_PHYS_ERASE_SZ();  // according to datasheet
    vfs_spiffs->cfg.log_block_size = SPIFFS_CFG_LOG_BLOCK_SZ();     // let us not complicate things
    vfs_spiffs->cfg.log_page_size = SPIFFS_CFG_LOG_PAGE_SZ();       // as we said
    #endif
    LOGD(TAG, "Spiffs mount.");

    int res = SPIFFS_mount(&vfs_spiffs->fs,
                       &vfs_spiffs->cfg,
                       spiffs_work_buf,
                       spiffs_fds,
                       sizeof(spiffs_fds),
#if SPIFFS_CACHE
                        spiffs_cache_buf,
                        sizeof(spiffs_cache_buf),
#else
                        NULL,
                        0,
#endif
                        (void *)mp_spiffs_check_cb);
    if (FORMAT_FS_FORCE || res != SPIFFS_OK || res==SPIFFS_ERR_NOT_A_FS)
    {
        SPIFFS_unmount(&vfs_spiffs->fs);
        LOGD(TAG, "Spiffs Unmount.");
        mp_printf(&mp_plat_print, "[VFS_SPIFFS] Spiffs formating (this may take some time) ...\n");

        vfs_spiffs->fs.block_count = SPIFFS_CFG_PHYS_SZ(fs) / SPIFFS_CFG_LOG_BLOCK_SZ(fs);
        s32_t format_res = SPIFFS_format(&vfs_spiffs->fs);
        if (format_res) {
            LOGE(TAG, "Format failed");
            return false;
        }
        else {
            LOGD(TAG, "Format successful");
        }

        res = SPIFFS_mount(&vfs_spiffs->fs,
            &vfs_spiffs->cfg,
            spiffs_work_buf,
            spiffs_fds,
            sizeof(spiffs_fds),
#if SPIFFS_CACHE
            spiffs_cache_buf,
            sizeof(spiffs_cache_buf),
#else
            NULL,
            0,
#endif
            0);
        LOGD(TAG, "Mount %s ", res ? "failed":"successful");
        if (res != SPIFFS_OK) return false;
    }
    else {
        LOGD(TAG, "Mount successful");
    }
    mp_vfs_mount_t *vfs = m_new_obj_maybe(mp_vfs_mount_t);
    if (vfs == NULL) {
        LOGE(TAG, "Cannot create new VFS");
    }
    vfs->str = "/flash";
    vfs->len = 6;
    vfs->obj = MP_OBJ_FROM_PTR(vfs_spiffs);
    vfs->next = NULL;
    mp_vfs_mount_t *vfs_sys = MP_STATE_VM(vfs_mount_table);
    if (vfs_sys == NULL) {
        MP_STATE_VM(vfs_mount_table) = vfs;
    }
    else {
        vfs_sys->next = vfs;
    }
    LOGD(TAG, "Flash VFS registered.");
    sprintf(spiffs_current_dir, "%s", "");

    check_boot_py(&vfs_spiffs->fs);

    return true;
}

//--------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t spiffs_vfs_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 1, false);

    // Return existing spiffs vfs object
    for (mp_vfs_mount_t *vfs = MP_STATE_VM(vfs_mount_table); vfs != NULL; vfs = vfs->next) {
        if ((vfs->len == 6) && (strcmp(vfs->str, "/flash") == 0)) {
            return vfs->obj;
        }
    }
    mp_raise_NotImplementedError("System spiffs not found");
    return mp_const_none;
}

//---------------------------------------------------------------------------
STATIC mp_import_stat_t spiffs_vfs_import_stat(void *vfs_in,const char *path)
{
    spiffs_user_mount_t *vfs = vfs_in;
    spiffs_stat  fno;
    if (vfs == NULL) return MP_IMPORT_STAT_NO_EXIST;

    const char *lpath = spiffs_local_path(path);

    int res = SPIFFS_stat(&vfs->fs, lpath, &fno);
    if (res == SPIFFS_OK) {
        vfs_spiffs_meta_t *meta = (vfs_spiffs_meta_t *)&fno.meta;
        if (meta->type == SPIFFS_TYPE_DIR) return MP_IMPORT_STAT_DIR;
        else return MP_IMPORT_STAT_FILE;
    }
    return MP_IMPORT_STAT_NO_EXIST;
}

/*
//----------------------------------------------
STATIC mp_obj_t spiffs_vfs_del(mp_obj_t self_in)
{
    mp_raise_NotImplementedError("System spiffs is always mounted");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(spiffs_vfs_del_obj, spiffs_vfs_del);
*/

//-----------------------------------------------
STATIC mp_obj_t spiffs_vfs_mkfs(mp_obj_t self_in)
{
    mp_raise_NotImplementedError("System Flash FS is always in use");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(spiffs_vfs_mkfs_fun_obj, spiffs_vfs_mkfs);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(spiffs_vfs_mkfs_obj, MP_ROM_PTR(&spiffs_vfs_mkfs_fun_obj));


//------------------------------------------------------------------
STATIC mp_obj_t mp_vfs_spiffs_ilistdir_it_iternext(mp_obj_t self_in)
{
    mp_vfs_spiffs_ilistdir_it_t *self = MP_OBJ_TO_PTR(self_in);
	struct spiffs_dirent de;
	struct spiffs_dirent* de_ret;

	for (;;) {
		de_ret = SPIFFS_readdir(&self->dir, &de);		
        char *fn = (char *)de.name;
        if (de_ret == NULL || fn[0] == 0) {
            // stop on error or end of directory
            break;
        }
        // Note that SPIFFS already filters . and .., so we don't need to

        // Check if the name is in the selected directory
        char *fname = fn;
        char dirname[strlen(fn)+1];
        strcpy(dirname, fn);
        char *dirend = strrchr(dirname, '/');
        if (dirend != NULL) {
            fname += (dirend - dirname) + 1;
            *dirend = '\0';
        }
        else dirname[0] = '\0';
        if (strcmp(self->path, dirname) != 0) continue;

        // make 4-tuple with info about this entry
        mp_obj_tuple_t *t = MP_OBJ_TO_PTR(mp_obj_new_tuple(4, NULL));
        if (self->is_str) {
            t->items[0] = mp_obj_new_str(fname, strlen(fname));
        } else {
            t->items[0] = mp_obj_new_bytes((const byte*)fname, strlen(fname));
        }
        vfs_spiffs_meta_t * meta = (vfs_spiffs_meta_t *)&de.meta;
        if (meta->type == SPIFFS_TYPE_DIR) {
            // dir
            t->items[1] = mp_obj_new_int(MP_S_IFDIR);
        } 
		else{
            // file
            t->items[1] = mp_obj_new_int(MP_S_IFREG);
        }
        t->items[2] = mp_obj_new_int(0); // no inode number
        t->items[3] = mp_obj_new_int_from_uint(de.size);

        return MP_OBJ_FROM_PTR(t);
    }

    // ignore error because we may be closing a second time
    SPIFFS_closedir(&self->dir);

    return MP_OBJ_STOP_ITERATION;
}

//---------------------------------------------------------------------------
STATIC mp_obj_t spiffs_vfs_ilistdir_func(size_t n_args, const mp_obj_t *args)
{
    spiffs_user_mount_t *self = MP_OBJ_TO_PTR(args[0]);
    bool is_str_type = true;
    const char *path;
    char *lpath;
    if (n_args == 2) {
        if (mp_obj_get_type(args[1]) == &mp_type_bytes) {
            is_str_type = false;
        }
        path = mp_obj_str_get_str(args[1]);
        lpath = (char *)spiffs_local_path(path);
    }
    else lpath = spiffs_current_dir;
    LOGD(TAG, "LISTDIR [%s]", lpath);

    if (strlen(lpath) > 0) {
        // Check if requested path is a directory
        spiffs_stat  fno;
        int res = SPIFFS_stat(&(self->fs), lpath, &fno);
        if (res == SPIFFS_OK) {
            vfs_spiffs_meta_t * meta = (vfs_spiffs_meta_t *)&fno.meta;
            if (meta->type != SPIFFS_TYPE_DIR) res = -1;
        }
        if (res != SPIFFS_OK) {
            mp_raise_OSError(MP_ENOTDIR);
        }
    }

    // Create a new iterator object to list the directory
    mp_vfs_spiffs_ilistdir_it_t *iter = m_new_obj(mp_vfs_spiffs_ilistdir_it_t);
    iter->base.type = &mp_type_polymorph_iter;
    iter->iternext = mp_vfs_spiffs_ilistdir_it_iternext;
    iter->is_str = is_str_type;
    strcpy(iter->path, lpath);
	if (!SPIFFS_opendir(&(self->fs), lpath, &iter->dir)) {
        mp_raise_OSError(MP_ENOTDIR);
    }

    return MP_OBJ_FROM_PTR(iter);
	
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(spiffs_vfs_ilistdir_obj, 1, 2, spiffs_vfs_ilistdir_func);

//----------------------------------------------------
static bool is_dir_empty(spiffs *fs, const char *path)
{
    spiffs_DIR dir;
    struct spiffs_dirent de;
    struct spiffs_dirent* de_ret;
    bool is_empty = true;

    if (!SPIFFS_opendir(fs, path, &dir)) return false;

    for (;;) {
        de_ret = SPIFFS_readdir(&dir, &de);
        char *fn = (char *)de.name;
        if (de_ret == NULL || fn[0] == 0) {
            // stop on error or end of dir
            break;
        }
        // Note that FatFS already filters . and .., so we don't need to

        if (strstr(fn, path) == path) {
            is_empty = false;
            break;
        }
    }
    SPIFFS_closedir(&dir);
    return is_empty;
}

//----------------------------------------------
static int _is_dir(spiffs *fs, const char *path)
{
    // Check if file is directory
    spiffs_stat fno;
    int res;

    res = SPIFFS_stat(fs, path, &fno);
    if (res != SPIFFS_OK) {
        SPIFFS_clearerr(fs);
        return res;
    }
    vfs_spiffs_meta_t *meta = (vfs_spiffs_meta_t *)&fno.meta;
    if (meta->type == SPIFFS_TYPE_DIR) return 1;
    return 0;
}

//------------------------------------------------------------------
STATIC mp_obj_t spiffs_vfs_remove(mp_obj_t vfs_in, mp_obj_t path_in)
{
	spiffs_user_mount_t *self = MP_OBJ_TO_PTR(vfs_in);
	const char *path = mp_obj_str_get_str(path_in);
    const char *lpath = spiffs_local_path(path);
    int res;

    // Check if file is directory
    res = _is_dir(&(self->fs), lpath);
    if (res < 0) {
        mp_raise_OSError(MP_ENOENT);
    }
    if (res == 1) {
        // It is directory, cannot be removed
        mp_raise_OSError(MP_EISDIR);
    }

    res = SPIFFS_remove(&(self->fs), lpath);
    if (res != SPIFFS_OK) {
        SPIFFS_clearerr(&(self->fs));
		mp_raise_OSError(SPIFFS_errno_table[GET_ERR_CODE(res)]);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(spiffs_vfs_remove_obj, spiffs_vfs_remove);


//-------------------------------------------------------------------------------------
STATIC mp_obj_t spiffs_vfs_rename(mp_obj_t vfs_in, mp_obj_t path_in, mp_obj_t path_out)
{
    spiffs_user_mount_t *self = MP_OBJ_TO_PTR(vfs_in);
    const char *old_path = mp_obj_str_get_str(path_in);
    const char *new_path = mp_obj_str_get_str(path_out);
    const char *lold_path = spiffs_local_path(old_path);
    const char *lnew_path = spiffs_local_path(new_path);
    int res;

    res = SPIFFS_rename(&(self->fs), lold_path, lnew_path);
    if (res == SPIFFS_ERR_CONFLICTING_NAME){
        // if new_path exists then try removing it (but only if it's a file)
        res = _is_dir(&(self->fs), lnew_path);
        if (res < 0) {
            mp_raise_OSError(SPIFFS_errno_table[GET_ERR_CODE(res)]);
        }
        if (res == 1) {
            // It is directory, cannot be removed
            mp_raise_OSError(MP_EISDIR);
        }
        res = SPIFFS_remove(&(self->fs), lnew_path); //remove file
        if (res < 0) {
            mp_raise_OSError(SPIFFS_errno_table[GET_ERR_CODE(res)]);
        }
        // try to rename again
        res = SPIFFS_rename(&(self->fs), lold_path, lnew_path);
        if (res < 0) {
            mp_raise_OSError(SPIFFS_errno_table[GET_ERR_CODE(res)]);
        }
    }
    else if (res == SPIFFS_OK) {
        return mp_const_none;
    }
    else {
        SPIFFS_clearerr(&(self->fs));
        mp_raise_OSError(SPIFFS_errno_table[GET_ERR_CODE(res)]);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(spiffs_vfs_rename_obj, spiffs_vfs_rename);

//----------------------------------------------------------------
STATIC mp_obj_t spiffs_vfs_mkdir(mp_obj_t vfs_in, mp_obj_t path_o)
{
    spiffs_user_mount_t* self = MP_OBJ_TO_PTR(vfs_in);
    const char *path = mp_obj_str_get_str(path_o);
    const char *lpath = spiffs_local_path(path);
    int res;

    // Check if the directory or file with the same name exists
    res = _is_dir(&(self->fs), lpath);
    if (res >= 0) {
        // It is directory, cannot be removed
        mp_raise_OSError(MP_EEXIST);
    }

    int fd = SPIFFS_open(&(self->fs), lpath, SPIFFS_CREAT | SPIFFS_WRONLY, 0);
    if (fd < 0) {
        SPIFFS_clearerr(&(self->fs));
        mp_raise_OSError(SPIFFS_errno_table[GET_ERR_CODE(fd)]);
    }
    vfs_spiffs_update_meta(&(self->fs), fd, SPIFFS_TYPE_DIR);
    //if (!vfs_spiffs_update_meta(&(self->fs), fd, SPIFFS_TYPE_DIR)) {
    //    mp_raise_OSError(MP_EIO);
    //}

    res = SPIFFS_close(&(self->fs), fd);
    if (res < 0) {
        SPIFFS_clearerr(&(self->fs));
        mp_raise_OSError(SPIFFS_errno_table[GET_ERR_CODE(res)]);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(spiffs_vfs_mkdir_obj, spiffs_vfs_mkdir);

//--------------------------------------------------------------
STATIC mp_obj_t fat_vfs_rmdir(mp_obj_t vfs_in, mp_obj_t path_in)
{
    spiffs_user_mount_t* vfs = MP_OBJ_TO_PTR(vfs_in);
    const char *path = mp_obj_str_get_str(path_in);
    const char *lpath = spiffs_local_path(path);
    int res;

    // Check if directory
    res = _is_dir(&vfs->fs, lpath);
    if (res < 0) {
        mp_raise_OSError(SPIFFS_errno_table[GET_ERR_CODE(res)]);
    }
    if (res == 0) {
        // Not a directory
        mp_raise_OSError(MP_ENOTDIR);
    }
    // Check if  directory is empty
    if (!is_dir_empty(&vfs->fs, lpath)) {
        mp_raise_OSError(MP_EPERM);
    }

    res = SPIFFS_remove(&vfs->fs, lpath);
    if (res < 0) {
        SPIFFS_clearerr(&vfs->fs);
        mp_raise_OSError(SPIFFS_errno_table[GET_ERR_CODE(res)]);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(spiffs_vfs_rmdir_obj, fat_vfs_rmdir);

// Change current directory.
//-----------------------------------------------------------------
STATIC mp_obj_t spiffs_vfs_chdir(mp_obj_t vfs_in, mp_obj_t path_in)
{
    spiffs_user_mount_t* vfs = MP_OBJ_TO_PTR(vfs_in);
    const char *path = mp_obj_str_get_str(path_in);
    int res;

    if ((path[0] != 0) && !(path[0] == '/' && path[1] == 0)) {
        const char *lpath = spiffs_local_path(path);
        // Check if directory
        res = _is_dir(&vfs->fs, lpath);
        if (res < 0) {
            mp_raise_OSError(SPIFFS_errno_table[GET_ERR_CODE(res)]);
        }
        if (res == 0) {
            // Not a directory
            mp_raise_OSError(MP_ENOTDIR);
        }
        sprintf(spiffs_current_dir, "/%s", lpath);
        LOGD(TAG, "CHDIR currdir=[%s]", spiffs_current_dir);
    }
    else sprintf(spiffs_current_dir, "%s", "");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(spiffs_vfs_chdir_obj, spiffs_vfs_chdir);

// Get the current directory.
//------------------------------------------------
STATIC mp_obj_t spiffs_vfs_getcwd(mp_obj_t vfs_in)
{
    LOGD(TAG, "GETCWD [%s]", spiffs_current_dir);
    return mp_obj_new_str(spiffs_current_dir, strlen(spiffs_current_dir));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(spiffs_vfs_getcwd_obj, spiffs_vfs_getcwd);

// function stat(path)
// Get the status of a file or directory.
//----------------------------------------------------------------
STATIC mp_obj_t spiffs_vfs_stat(mp_obj_t vfs_in, mp_obj_t path_in)
{
    spiffs_user_mount_t *self = MP_OBJ_TO_PTR(vfs_in);
    const char *path = mp_obj_str_get_str(path_in);
    int res;

    mp_int_t mode = MP_S_IFDIR;
    mp_int_t size = 0;
    mp_int_t time = 0;

    if ((path[0] != 0) && !(path[0] == '/' && path[1] == 0)) {
        const char *lpath = spiffs_local_path(path);
        spiffs_stat fno;
        res = SPIFFS_stat(&(self->fs), lpath, &fno);
        if (res != SPIFFS_OK) {
            mp_raise_OSError(SPIFFS_errno_table[GET_ERR_CODE(res)]);
        }
        size = fno.size;
        vfs_spiffs_meta_t * meta = (vfs_spiffs_meta_t *)&fno.meta;
        if (meta->type == SPIFFS_TYPE_DIR) mode = MP_S_IFDIR;
        else mode = MP_S_IFREG;
        time = meta->mtime;
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
STATIC MP_DEFINE_CONST_FUN_OBJ_2(spiffs_vfs_stat_obj, spiffs_vfs_stat);

// Get the status of a VFS.
//-------------------------------------------------------------------
STATIC mp_obj_t spiffs_vfs_statvfs(mp_obj_t vfs_in, mp_obj_t path_in)
{
    spiffs_user_mount_t *self = MP_OBJ_TO_PTR(vfs_in);

    spiffs *spif_fs = &(self->fs);
	u32_t total, used;
    int res;

    res = SPIFFS_info(spif_fs, &total,&used);
    if (res != SPIFFS_OK) {
        SPIFFS_clearerr(&(self->fs));
		mp_raise_OSError(SPIFFS_errno_table[GET_ERR_CODE(res)]);
    }
    mp_obj_tuple_t *t = MP_OBJ_TO_PTR(mp_obj_new_tuple(10, NULL));

    t->items[0] = mp_obj_new_int(SPIFFS_CFG_LOG_PAGE_SZ());               // file system block size
    t->items[1] = t->items[0];                                            // fragment size
    t->items[2] = mp_obj_new_int(total/SPIFFS_CFG_LOG_PAGE_SZ());         // size of fs in f_frsize units
    t->items[3] = mp_obj_new_int((total-used)/SPIFFS_CFG_LOG_PAGE_SZ());  // f_bfree
    t->items[4] = t->items[3];                                            // f_bavail
    t->items[5] = mp_obj_new_int(0); // f_files
    t->items[6] = mp_obj_new_int(0); // f_ffree
    t->items[7] = mp_obj_new_int(0); // f_favail
    t->items[8] = mp_obj_new_int(0); // f_flags
    t->items[9] = mp_obj_new_int(SPIFFS_OBJ_NAME_LEN);                    // f_namemax

    return MP_OBJ_FROM_PTR(t);

}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(spiffs_vfs_statvfs_obj, spiffs_vfs_statvfs);

//----------------------------------------------------------------------------------
STATIC mp_obj_t vfs_spiffs_mount(mp_obj_t self_in, mp_obj_t readonly, mp_obj_t mkfs)
{
    mp_raise_NotImplementedError("System spiffs is always mounted");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(vfs_spiffs_mount_obj, vfs_spiffs_mount);

//-------------------------------------------------
STATIC mp_obj_t vfs_spiffs_umount(mp_obj_t self_in)
{
    mp_raise_NotImplementedError("Only system spiffs is used");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(spiffs_vfs_umount_obj, vfs_spiffs_umount);

//-------------------------------------------------
STATIC mp_obj_t vfs_spiffs_check(mp_obj_t self_in)
{
    spiffs_user_mount_t *self = MP_OBJ_TO_PTR(self_in);

    int check[7] = {0};
    fs_check = &check[0];
    mp_printf(&mp_plat_print, "\r\nChecking spiffs, please wait...\r\n");

    SPIFFS_clearerr(&(self->fs));
    SPIFFS_check(&(self->fs));
    fs_check = NULL;
    mp_printf(&mp_plat_print, "Finished.\r\n");
    mp_printf(&mp_plat_print, "--------------\r\n");
    mp_printf(&mp_plat_print, " Checks      : %d\r\n", check[SPIFFS_CHECK_PROGRESS]);
    mp_printf(&mp_plat_print, " Errors      : %d\r\n", check[SPIFFS_CHECK_ERROR]);
    mp_printf(&mp_plat_print, " Fix index   : %d\r\n", check[SPIFFS_CHECK_FIX_INDEX]);
    mp_printf(&mp_plat_print, " Fix look up : %d\r\n", check[SPIFFS_CHECK_FIX_LOOKUP]);
    mp_printf(&mp_plat_print, " Del orph pg : %d\r\n", check[SPIFFS_CHECK_DELETE_ORPHANED_INDEX]);
    mp_printf(&mp_plat_print, " Del pages   : %d\r\n", check[SPIFFS_CHECK_DELETE_PAGE]);
    mp_printf(&mp_plat_print, " Del bad file: %d\r\n", check[SPIFFS_CHECK_DELETE_BAD_FILE]);
    mp_printf(&mp_plat_print, "--------------\r\n");

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(spiffs_vfs_check_obj, vfs_spiffs_check);

//---------------------------------------------------------------
STATIC mp_obj_t vfs_spiffs_gc(size_t n_args, const mp_obj_t *args)
{
    spiffs_user_mount_t *self = MP_OBJ_TO_PTR(args[0]);

    if (SPIFFS_mounted(&(self->fs)) == false) {
        mp_raise_OSError(MP_ENOENT);
    }
    int n = 1;
    if (n_args >= 2) {
        n = mp_obj_get_int(args[1]);
        if (n < 1) n = 1;
        if (n > 64) n = 64;
    }
    int max_fp = 0;
    if (n_args >= 3) {
        max_fp = mp_obj_get_int(args[2]);
        if (max_fp < 0) max_fp = 0;
        if (max_fp > 50) max_fp = 50;
    }

    mp_printf(&mp_plat_print, "Starting spiffs garbage collection, please wait...\r\n");
    int nf = 0;
    SPIFFS_clearerr(&(self->fs));
    for (int i=0; i<n; i++) {
        if (SPIFFS_gc_quick(&(self->fs), max_fp) < 0) break;
        if (SPIFFS_errno(&(self->fs)) == SPIFFS_OK) nf++;
        SPIFFS_clearerr(&(self->fs));
    }
    mp_printf(&mp_plat_print, "Finished: %d %dK block(s) erased.\r\n", nf, SPIFFS_CFG_LOG_BLOCK_SZ(fs)/1024);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(spiffs_vfs_gc_obj, 1, 3, vfs_spiffs_gc);

#if SPIFFS_TEST_VISUALISATION
//----------------------------------------------
STATIC mp_obj_t vfs_spiffs_vis(mp_obj_t self_in)
{
    spiffs_user_mount_t *self = MP_OBJ_TO_PTR(self_in);

    SPIFFS_clearerr(&(self->fs));
    SPIFFS_vis(&(self->fs));

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(spiffs_vfs_vis_obj, vfs_spiffs_vis);
#else
//----------------------------------------------
STATIC mp_obj_t vfs_spiffs_vis(mp_obj_t self_in)
{
    mp_raise_NotImplementedError("Only system spiffs is used");

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(spiffs_vfs_vis_obj, vfs_spiffs_vis);
#endif

//------------------------------------------------------------------------
STATIC mp_obj_t vfs_spiffs_dbg_level(mp_obj_t self_in, mp_obj_t dbglev_in)
{
    //spiffs_user_mount_t *self = MP_OBJ_TO_PTR(self_in);

    int dbg = mp_obj_get_int(dbglev_in);
    if ((dbg >= 0) && (dbg <= 16)) {
        spiffs_dbg_level = dbg;
    }

    return mp_obj_new_int(spiffs_dbg_level);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(spiffs_vfs_dbg_level_obj, vfs_spiffs_dbg_level);

//----------------------------------------------------------------------
STATIC mp_obj_t vfs_spiffs_counters(size_t n_args, const mp_obj_t *args)
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
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(spiffs_vfs_counters_obj, 1, 2, vfs_spiffs_counters);


//===============================================================
STATIC const mp_rom_map_elem_t spiffs_vfs_locals_dict_table[] = {
    #if _FS_REENTRANT
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&spiffs_vfs_del_obj) },
    #endif
    { MP_ROM_QSTR(MP_QSTR_mkfs),        MP_ROM_PTR(&spiffs_vfs_mkfs_obj) },
    { MP_ROM_QSTR(MP_QSTR_open),        MP_ROM_PTR(&spiffs_vfs_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_mount),       MP_ROM_PTR(&vfs_spiffs_mount_obj) },
    { MP_ROM_QSTR(MP_QSTR_ilistdir),    MP_ROM_PTR(&spiffs_vfs_ilistdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_chdir),       MP_ROM_PTR(&spiffs_vfs_chdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_mkdir),       MP_ROM_PTR(&spiffs_vfs_mkdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_rmdir),       MP_ROM_PTR(&spiffs_vfs_rmdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_getcwd),      MP_ROM_PTR(&spiffs_vfs_getcwd_obj) },
    { MP_ROM_QSTR(MP_QSTR_remove),      MP_ROM_PTR(&spiffs_vfs_remove_obj) },
    { MP_ROM_QSTR(MP_QSTR_rename),      MP_ROM_PTR(&spiffs_vfs_rename_obj) },
    { MP_ROM_QSTR(MP_QSTR_stat),        MP_ROM_PTR(&spiffs_vfs_stat_obj) },
    { MP_ROM_QSTR(MP_QSTR_statvfs),     MP_ROM_PTR(&spiffs_vfs_statvfs_obj) },
    { MP_ROM_QSTR(MP_QSTR_umount),      MP_ROM_PTR(&spiffs_vfs_umount_obj) },
    { MP_ROM_QSTR(MP_QSTR_counters),    MP_ROM_PTR(&spiffs_vfs_counters_obj) },
    { MP_ROM_QSTR(MP_QSTR_check),       MP_ROM_PTR(&spiffs_vfs_check_obj) },
    { MP_ROM_QSTR(MP_QSTR_gc),          MP_ROM_PTR(&spiffs_vfs_gc_obj) },
    { MP_ROM_QSTR(MP_QSTR_dbg_level),   MP_ROM_PTR(&spiffs_vfs_dbg_level_obj) },
    #if SPIFFS_TEST_VISUALISATION
    { MP_ROM_QSTR(MP_QSTR_vis),         MP_ROM_PTR(&spiffs_vfs_vis_obj) },
    #endif
    // class constants
    { MP_ROM_QSTR(MP_QSTR_DBG_NONE),    MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_DBG_GEN),     MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_DBG_GC),      MP_ROM_INT(2) },
    { MP_ROM_QSTR(MP_QSTR_DBG_CACHE),   MP_ROM_INT(4) },
    { MP_ROM_QSTR(MP_QSTR_DBG_CHECK),   MP_ROM_INT(8) },
    { MP_ROM_QSTR(MP_QSTR_DBG_API),     MP_ROM_INT(16) },
};
STATIC MP_DEFINE_CONST_DICT(spiffs_vfs_locals_dict, spiffs_vfs_locals_dict_table);


STATIC const mp_vfs_proto_t spiffs_vfs_proto = {
    .import_stat = spiffs_vfs_import_stat,
};

const mp_obj_type_t mp_spiffs_vfs_type = {
    { &mp_type_type },
    .name = MP_QSTR_VfsSpiffs,
    .make_new = spiffs_vfs_make_new,
    .protocol = &spiffs_vfs_proto,
    .locals_dict = (mp_obj_dict_t*)&spiffs_vfs_locals_dict,
};
#endif

