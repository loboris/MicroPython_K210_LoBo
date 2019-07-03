/*
 * This file is part of the MicroPython K210 project, https://github.com/loboris/MicroPython_K210_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 LoBo (https://github.com/loboris)
 *
 * Permission is hereby granted, vPortFree of charge, to any person obtaining a copy
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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "py/objtuple.h"
#include "py/objstr.h"
#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "py/lexer.h"
#include "extmod/misc.h"
#if MICROPY_VFS
#include "extmod/vfs.h"
#include "py/stream.h"
#endif
#include "genhdr/mpversion.h"
#if MICROPY_VFS_SPIFFS
#include "vfs_spiffs.h"
#include "spiffs_config.h"
#endif
#if MICROPY_VFS_LITTLEFS
#include "littleflash.h"
#endif

#if MICROPY_VFS_SDCARD
#include "vfs_sdcard.h"
#endif
#include "mphalport.h"


STATIC const qstr os_uname_info_fields[] = {
    MP_QSTR_sysname, MP_QSTR_nodename,
    MP_QSTR_release, MP_QSTR_version, MP_QSTR_machine
};
STATIC const MP_DEFINE_STR_OBJ(os_uname_info_sysname_obj, MICROPY_PY_SYS_PLATFORM);
STATIC const MP_DEFINE_STR_OBJ(os_uname_info_nodename_obj, MICROPY_PY_SYS_PLATFORM);
STATIC const MP_DEFINE_STR_OBJ(os_uname_info_release_obj, MICROPY_VERSION_STRING);
STATIC const MP_DEFINE_STR_OBJ(os_uname_info_version_obj, MICROPY_GIT_TAG" on "MICROPY_BUILD_DATE);
STATIC const MP_DEFINE_STR_OBJ(os_uname_info_machine_obj, MICROPY_HW_BOARD_NAME " with " MICROPY_HW_MCU_NAME);

STATIC MP_DEFINE_ATTRTUPLE(
    os_uname_info_obj,
    os_uname_info_fields,
    5,
    (mp_obj_t)&os_uname_info_sysname_obj,
    (mp_obj_t)&os_uname_info_nodename_obj,
    (mp_obj_t)&os_uname_info_release_obj,
    (mp_obj_t)&os_uname_info_version_obj,
    (mp_obj_t)&os_uname_info_machine_obj
);

//----------------------------
STATIC mp_obj_t os_uname(void)
{
    return (mp_obj_t)&os_uname_info_obj;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(os_uname_obj, os_uname);

#define SMALL_MAX 65535
#define SMALL_MIN 0
//--------------------------
uint32_t get_random(int num)
{
	unsigned int ran_val;
    srand(num);
    ran_val = rand() % (SMALL_MAX + 1 - SMALL_MIN) + SMALL_MIN;
	return ran_val;
}

//--------------------------------------
STATIC mp_obj_t os_urandom(mp_obj_t num)
{
    mp_int_t n = mp_obj_get_int(num);
    vstr_t vstr;
    vstr_init_len(&vstr, n);
    uint32_t r = 0;
    for (int i = 0; i < n; i++) {
        r = get_random(i); // returns 32-bit hardware random number
        vstr.buf[i] = r;
    }
    return mp_obj_new_str_from_vstr(&mp_type_bytes, &vstr);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(os_urandom_obj, os_urandom);

#if MICROPY_PY_OS_DUPTERM
//--------------------------------------------------
STATIC mp_obj_t os_dupterm_notify(mp_obj_t obj_in) {
    (void)obj_in;
    for (;;) {
        int c = mp_uos_dupterm_rx_chr();
        if (c < 0) {
            break;
        }
        ringbuf_put(&stdin_ringbuf, c);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(os_dupterm_notify_obj, os_dupterm_notify);
#endif

//---------------------------
STATIC mp_obj_t os_sync(void)
{
    #if MICROPY_VFS
    /*
    for (mp_vfs_mount_t *vfs = MP_STATE_VM(vfs_mount_table); vfs != NULL; vfs = vfs->next) {
        // this assumes that vfs->obj is fs_user_mount_t with block device functions
        disk_ioctl(MP_OBJ_TO_PTR(vfs->obj), CTRL_SYNC, NULL);
    }
    */
    #endif
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(mod_os_sync_obj, os_sync);

#define PROXY_MAX_ARGS (2)

//-----------------------------------------------------------------------------------------------------------
STATIC mp_obj_t mp_vfs_proxy_call(mp_vfs_mount_t *vfs, qstr meth_name, size_t n_args, const mp_obj_t *args) {
    assert(n_args <= PROXY_MAX_ARGS);
    if (vfs == MP_VFS_NONE) {
        // mount point not found
        mp_raise_OSError(MP_ENODEV);
    }
    if (vfs == MP_VFS_ROOT) {
        // can't do operation on root dir
        mp_raise_OSError(MP_EPERM);
    }
    mp_obj_t meth[2 + PROXY_MAX_ARGS];
    mp_load_method(vfs->obj, meth_name, meth);
    if (args != NULL) {
        memcpy(meth + 2, args, n_args * sizeof(*args));
    }
    return mp_call_method_n_kw(n_args, 0, meth);
}

/// Get the current drive.
//-------------------------------------
STATIC mp_obj_t os_getdrive() {

    char drive[256] = {'\0'};

    if (MP_STATE_VM(vfs_cur) == MP_VFS_ROOT) {
        sprintf(drive, "/");
    }
    else {
        mp_obj_t cwd_o = mp_vfs_proxy_call(MP_STATE_VM(vfs_cur), MP_QSTR_getcwd, 0, NULL);
        const char *cwd = mp_obj_str_get_str(cwd_o);

        if (MP_STATE_VM(vfs_cur)->len == 1) {
            // don't prepend "/" for vfs mounted at root
            sprintf(drive, "%s", cwd);
        }
        else {
            sprintf(drive, "%s", MP_STATE_VM(vfs_cur)->str);
            if (!(cwd[0] == '/' && cwd[1] == 0)) {
                strcat(drive, cwd);
            }
            char *drvend = strchr(drive+1, '/');
            if (drvend) *drvend = '\0';
        }
    }

    return mp_obj_new_str(drive, strlen(drive));
}
MP_DEFINE_CONST_FUN_OBJ_0(os_getdrive_obj, os_getdrive);

//---------------------------------------------
STATIC mp_obj_t os_checkfile(mp_obj_t fname_in)
{
    const char *fname = mp_obj_str_get_str(fname_in);
    int res = mp_vfs_import_stat(fname);
    return mp_obj_new_int(res);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(os_checkfile_obj, os_checkfile);

//-------------------------------------------------------------
STATIC mp_obj_t os_getfile(mp_obj_t fname_in, mp_obj_t size_in)
{
    int res = 0, remain, blk_size;
    int size = mp_obj_get_int(size_in);
    if ((size < 1) || (size > 1000000)) {
        // wrong file size
        return mp_obj_new_int(-3);
    }

    mp_obj_t args[2];
    args[0] = fname_in;
    args[1] = mp_obj_new_str("wb", 2);
    // Open the file for writing
    mp_obj_t ffd = mp_vfs_open(2, args, (mp_map_t*)&mp_const_empty_map);
    if (!ffd) {
        return mp_obj_new_int(-4);
    }

    bool do_write = true;
    uint8_t buf[1028];
    mp_obj_t fres = mp_const_true;

    // Get the file content to buffer
    mp_hal_delay_ms(20);
    remain = size;
    while (remain > 0) {
        blk_size = (remain > 1024) ? 1024 : remain;
        // Get file block
        res = mp_hal_get_file_block(buf, blk_size);
        if (res < 0) {
            fres = mp_obj_new_int(-5);
            break;
        }
        if (do_write) {
            // save buffer to file
            int wrbytes = mp_stream_posix_write((void *)ffd, (char*)buf, blk_size);
            if (wrbytes != blk_size) {
                fres = mp_obj_new_int(-6);
                do_write = false;
            }
        }
        remain -= blk_size;
    }

    mp_stream_close(ffd);

    return fres;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(os_getfile_obj, os_getfile);

//--------------------------------------------------------------
STATIC mp_obj_t os_sendfile(mp_obj_t fname_in, mp_obj_t size_in)
{
    mp_obj_t args[2];
    args[0] = fname_in;
    args[1] = mp_obj_new_str("rb", 2);
    uint8_t buf[1028];
    int res = 0, remain, blk_size;
    int size = mp_obj_get_int(size_in);
    if ((size < 0) || (size > 1000000)) {
        // wrong file size
        return mp_obj_new_int(-4);
    }

    // Open the file
    mp_obj_t ffd = mp_vfs_open(2, args, (mp_map_t*)&mp_const_empty_map);
    if (ffd) {
        // Get file size
        int fsize = mp_stream_posix_lseek((void *)ffd, 0, SEEK_END);
        int at_start = mp_stream_posix_lseek((void *)ffd, 0, SEEK_SET);
        int send_fsize = 0;
        if (size == 0) {
            size = fsize;
            send_fsize = fsize;
        }

        if ((fsize > 0) && (at_start == 0) && (fsize == size)) {
            remain = size;
            // send all file blocks
            while (remain > 0) {
                blk_size = (remain > 1024) ? 1024 : remain;
                // read to buffer from file
                memset(buf, 0, 1028);
                int rdbytes = mp_stream_posix_read((void *)ffd, (char*)buf, blk_size);
                if (rdbytes != blk_size) {
                    // file read error, request abort
                    res = -4;
                    memset(buf, 0x5A, 1028);
                    mp_hal_send_file_block(buf, blk_size, false, 0);
                    send_fsize = 0;
                }
                else {
                    res = mp_hal_send_file_block(buf, blk_size, true, send_fsize);
                    send_fsize = 0;
                }
                if (res < 0) break;
                remain -= blk_size;
            }
        }
        else res = -4; // wrong file size
        mp_stream_close(ffd);
    }
    else {
        return mp_const_false;
    }

    mp_hal_delay_ms(200);
    if (res != 0) return mp_obj_new_int(res);
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(os_sendfile_obj, os_sendfile);

//--------------------------------------------------------------------
STATIC mp_obj_t os_list_dir_files(size_t n_args, const mp_obj_t *args)
{
    const char *fname;
    char fullname[256], dirname[256] = {'\0'};
    mp_obj_t dir_list = mp_obj_new_list(0, NULL);
    mp_obj_t tuple[4];
    bool names_only = false;

    mp_obj_t largs[1];
    if (n_args > 0) {
        if (mp_obj_is_str(args[0])) {
            largs[0] = args[0];
            strcpy(dirname, mp_obj_str_get_str(args[0]));
            if (strlen(dirname) == 0) sprintf(dirname, "/");
            else if (dirname[strlen(dirname)-1] != '/') dirname[strlen(dirname)] = '/';
            if (n_args > 1) names_only = mp_obj_is_true(args[1]);
        }
        else {
            largs[0] = mp_obj_new_str("/", 1);
            sprintf(dirname, "/");
            names_only = mp_obj_is_true(args[0]);
        }
    }
    else {
        largs[0] = mp_obj_new_str("/", 1);
        sprintf(dirname, "/");
    };
    mp_obj_list_t *flist = MP_OBJ_TO_PTR(mp_vfs_listdir(1, largs));

    if (flist->len > 0) {
        for (int i = 0; i < flist->len; i++) {
            fname = mp_obj_str_get_str(flist->items[i]);
            strcpy(fullname, dirname);
            strcat(fullname, fname);
            mp_obj_tuple_t *tstat = (mp_obj_tuple_t *)mp_vfs_stat(mp_obj_new_str(fullname, strlen(fullname)));
            if (tstat->len == 10) {
                if (names_only) {
                    // list only file names
                    mp_obj_list_append(dir_list, flist->items[i]);
                }
                else {
                    tuple[0] = flist->items[i];
                    tuple[1] = (mp_obj_get_int(tstat->items[0]) & MP_S_IFREG) ? mp_const_false : mp_const_true;
                    tuple[2] = tstat->items[6];
                    tuple[3] = tstat->items[7];
                    mp_obj_list_append(dir_list, mp_obj_new_tuple(4, tuple));
                }
            }
        }
    }
    return dir_list;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(os_list_dir_files_obj, 0, 2, os_list_dir_files);


//==========================================================
STATIC const mp_rom_map_elem_t os_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),        MP_ROM_QSTR(MP_QSTR_uos) },
    { MP_ROM_QSTR(MP_QSTR_uname),           MP_ROM_PTR(&os_uname_obj) },
    { MP_ROM_QSTR(MP_QSTR_urandom),         MP_ROM_PTR(&os_urandom_obj) },
    #if MICROPY_PY_OS_DUPTERM
    { MP_ROM_QSTR(MP_QSTR_dupterm),         MP_ROM_PTR(&mp_uos_dupterm_obj) },
    { MP_ROM_QSTR(MP_QSTR_dupterm_notify),  MP_ROM_PTR(&os_dupterm_notify_obj) },
    #endif
    #if MICROPY_VFS
    { MP_ROM_QSTR(MP_QSTR_ilistdir),        MP_ROM_PTR(&mp_vfs_ilistdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_listdir),         MP_ROM_PTR(&mp_vfs_listdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_mkdir),           MP_ROM_PTR(&mp_vfs_mkdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_rmdir),           MP_ROM_PTR(&mp_vfs_rmdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_chdir),           MP_ROM_PTR(&mp_vfs_chdir_obj) },
    { MP_ROM_QSTR(MP_QSTR_getcwd),          MP_ROM_PTR(&mp_vfs_getcwd_obj) },
    { MP_ROM_QSTR(MP_QSTR_getdrive),        MP_ROM_PTR(&os_getdrive_obj) },
    { MP_ROM_QSTR(MP_QSTR_remove),          MP_ROM_PTR(&mp_vfs_remove_obj) },
    { MP_ROM_QSTR(MP_QSTR_rename),          MP_ROM_PTR(&mp_vfs_rename_obj) },
    { MP_ROM_QSTR(MP_QSTR_stat),            MP_ROM_PTR(&mp_vfs_stat_obj) },
    { MP_ROM_QSTR(MP_QSTR_statvfs),         MP_ROM_PTR(&mp_vfs_statvfs_obj) },
    { MP_ROM_QSTR(MP_QSTR_mount),           MP_ROM_PTR(&mp_vfs_mount_obj) },
    { MP_ROM_QSTR(MP_QSTR_umount),          MP_ROM_PTR(&mp_vfs_umount_obj) },
    { MP_ROM_QSTR(MP_QSTR_sync),            MP_ROM_PTR(&mod_os_sync_obj) },
    { MP_ROM_QSTR(MP_QSTR_check_file),      MP_ROM_PTR(&os_checkfile_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_file),        MP_ROM_PTR(&os_getfile_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_file),       MP_ROM_PTR(&os_sendfile_obj) },
    { MP_ROM_QSTR(MP_QSTR_listdirex),       MP_ROM_PTR(&os_list_dir_files_obj) },
    #endif
	#if MICROPY_VFS_SPIFFS
	{ MP_ROM_QSTR(MP_QSTR_VfsFlashfs),      MP_ROM_PTR(&mp_spiffs_vfs_type) },
    { MP_ROM_QSTR(MP_QSTR__flash),          MP_ROM_PTR(&spiffs_user_mount_handle) },
	#endif
    #if MICROPY_VFS_LITTLEFS
    { MP_ROM_QSTR(MP_QSTR_VfsFlashfs),      MP_ROM_PTR(&mp_littlefs_vfs_type) },
    { MP_ROM_QSTR(MP_QSTR__flash),          MP_ROM_PTR(&littlefs_user_mount_handle) },
    #endif
    #if MICROPY_VFS_SDCARD
    { MP_ROM_QSTR(MP_QSTR_VfsSDCard),       MP_ROM_PTR(&mp_sdcard_vfs_type) },
    #endif
};

STATIC MP_DEFINE_CONST_DICT(os_module_globals, os_module_globals_table);

//==================================
const mp_obj_module_t uos_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&os_module_globals,
};
