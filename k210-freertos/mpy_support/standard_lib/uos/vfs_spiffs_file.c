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

#if MICROPY_VFS && MICROPY_VFS_SPIFFS

#include <stdio.h>
#include <time.h>
#include "syslog.h"
#include "devices.h"
#include "w25qxx.h"

#include "py/runtime.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "vfs_spiffs.h"
#include "mphalport.h"

#define DEBUG_PRINTS    1

spiffs* glb_fs;
const mp_obj_type_t mp_type_vfs_spiffs_textio;

const byte SPIFFS_errno_table[43] = {
        [FS_OK] = 0 ,
        [ERR_NOT_MOUNTED] = MP_ENOEXEC,
        [ERR_FULL] = MP_ENOSPC,
        [ERR_NOT_FOUND] = MP_ENOENT,
        [ERR_END_OF_OBJECT] = MP_EIO,//TODO
        [ERR_DELETED] = MP_EIO,
        [ERR_NOT_FINALIZED] = MP_EIO,//TODO
        [ERR_NOT_INDEX] = MP_EIO,//TODO
        [ERR_OUT_OF_FILE_DESCS] = MP_EMFILE,
        [ERR_FILE_CLOSED] = MP_EPERM,
        [ERR_FILE_DELETED] = MP_EIO,//TODO
        [ERR_BAD_DESCRIPTOR] = MP_EBADF,
        [ERR_IS_INDEX] = MP_EPERM,
        [ERR_IS_FREE] = MP_EIO,//TODO
        [ERR_INDEX_SPAN_MISMATCH] = MP_EIO,//TODO
        [ERR_DATA_SPAN_MISMATCH] = MP_EIO,//TODO
        [ERR_INDEX_REF_FREE] = MP_EIO,//TODO
        [ERR_INDEX_REF_LU] = MP_EIO,//TODO
        [ERR_INDEX_REF_INVALID] = MP_EINVAL,
        [ERR_INDEX_FREE] = MP_EIO,//TODO
        [ERR_INDEX_LU] = MP_EIO,//TODO
        [ERR_INDEX_INVALID] = MP_EINVAL,
        [ERR_NOT_WRITABLE] = MP_EPERM,
        [ERR_NOT_READABLE] = MP_EPERM,
        [ERR_CONFLICTING_NAME] = MP_EEXIST,
        [ERR_NOT_CONFIGURED] = MP_EIO,
        [ERR_NOT_A_FS] = MP_EIO,
        [ERR_MOUNTED] = MP_EIO,
        [ERR_ERASE_FAIL] = MP_EIO,
        [ERR_MAGIC_NOT_POSSIBLE] = MP_EIO,
        [ERR_NO_DELETED_BLOCKS] = MP_EWOULDBLOCK,
        [ERR_FILE_EXISTS] = MP_EEXIST,
        [ERR_NOT_A_FILE] = MP_ENOENT,
        [ERR_RO_NOT_IMPL] = MP_EIO,
        [ERR_RO_ABORTED_OPERATION] = MP_EIO,//TODO
        [ERR_PROBE_TOO_FEW_BLOCKS] = MP_EIO,//TODO
        [ERR_PROBE_NOT_A_FS] = MP_EIO,//TODO
        [ERR_NAME_TOO_LONG] = MP_EINVAL,
        [ERR_IX_MAP_UNMAPPED] = MP_EIO,//TODO,
        [ERR_IX_MAP_MAPPED] = MP_EIO,//TODO
        [ERR_IX_MAP_BAD_RANGE] = MP_EIO,//TODO,
        [ERR_SEEK_BOUNDS] = MP_ESPIPE,//TODO,
};

static const char* TAG = "[SPIFFS_FILE]";
extern handle_t mp_rtc_rtc0;

static int spiffs_res_to_errno(s32_t fr)
{
    switch(fr) {
    case SPIFFS_OK :
        return 0;
    case SPIFFS_ERR_NOT_MOUNTED :
        return ENODEV;
    case SPIFFS_ERR_NOT_A_FS :
        return ENODEV;
    case SPIFFS_ERR_FULL :
        return ENOSPC;
    case SPIFFS_ERR_BAD_DESCRIPTOR :
        return EBADF;
    case SPIFFS_ERR_MOUNTED :
        return EEXIST;
    case SPIFFS_ERR_FILE_EXISTS :
        return EEXIST;
    case SPIFFS_ERR_NOT_FOUND :
        return ENOENT;
    case SPIFFS_ERR_NOT_A_FILE :
        return ENOENT;
    case SPIFFS_ERR_DELETED :
        return ENOENT;
    case SPIFFS_ERR_FILE_DELETED :
        return ENOENT;
    case SPIFFS_ERR_NAME_TOO_LONG :
        return ENAMETOOLONG;
    case SPIFFS_ERR_RO_NOT_IMPL :
        return EROFS;
    case SPIFFS_ERR_RO_ABORTED_OPERATION :
        return EROFS;
    default :
        return EIO;
    }
    return ENOTSUP;
}

//-------------------------------------------------------------------
bool vfs_spiffs_update_meta(spiffs *fs, spiffs_file fd, uint8_t type)
{
    vfs_spiffs_meta_t meta;
    time_t seconds;
    struct tm now;
    rtc_get_datetime(mp_rtc_rtc0, &now);
    seconds = mktime(&now);
    meta.mtime = seconds;
    // Add file type (directory or regular file) to the last byte of metadata
    meta.type = type;
    int ret = SPIFFS_fupdate_meta(fs, fd, (uint8_t *)&meta);
    if (ret != SPIFFS_OK) {
        #if DEBUG_PRINTS
        LOGD(TAG, "Failed to update metadata (%d)", ret);
        #endif
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------------------
STATIC void file_obj_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    (void)kind;
    mp_printf(print, "<io.%s %p>", mp_obj_get_type_str(self_in), MP_OBJ_TO_PTR(self_in));
}

//---------------------------------------------------------------------------------------
STATIC mp_uint_t file_obj_read(mp_obj_t self_in, void *buf, mp_uint_t size, int *errcode)
{
    spiffs_file_obj_t *self = MP_OBJ_TO_PTR(self_in);

    spiffs_FILE fp=self->fp;
	s32_t ret = SPIFFS_read(fp.fs, fp.fd, buf, size);
    if(ret <  0){
        *errcode = MP_EIO;
        SPIFFS_clearerr(fp.fs);
        return MP_STREAM_ERROR;
    }
    return (mp_uint_t)ret;
}

//----------------------------------------------------------------------------------------------
STATIC mp_uint_t file_obj_write(mp_obj_t self_in, const void *buf, mp_uint_t size, int *errcode)
{
    spiffs_file_obj_t *self = MP_OBJ_TO_PTR(self_in);
	spiffs_FILE fp=self->fp;
    int32_t ret = SPIFFS_write(fp.fs, fp.fd, (uint8_t*)buf, size);
    if (ret < 0) {
        *errcode = MP_EIO;
        SPIFFS_clearerr(fp.fs);
        return MP_STREAM_ERROR;
    }
    if (ret != size) {
        *errcode = MP_ENOSPC;
        return MP_STREAM_ERROR;
    }
    return (mp_uint_t)ret;
}

//--------------------------------------------------------------------
STATIC mp_obj_t file_obj___exit__(size_t n_args, const mp_obj_t *args)
{
    (void)n_args;
    return mp_stream_close(args[0]);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(file_obj___exit___obj, 4, 4, file_obj___exit__);

//--------------------------------------------------------------------------------------------
STATIC mp_uint_t file_obj_ioctl(mp_obj_t o_in, mp_uint_t request, uintptr_t arg, int *errcode)
{
    spiffs_file_obj_t *self = MP_OBJ_TO_PTR(o_in);
    spiffs_FILE fp=self->fp;
    if (request == MP_STREAM_SEEK) {
        struct mp_stream_seek_t *s = (struct mp_stream_seek_t*)(uintptr_t)arg;
        s32_t ret = 0;

        switch (s->whence) {
            case 0: // SEEK_SET
                ret = SPIFFS_lseek(fp.fs, fp.fd,s->offset,SPIFFS_SEEK_SET);
                break;

            case 1: // SEEK_CUR
                ret = SPIFFS_lseek(fp.fs, fp.fd,s->offset,SPIFFS_SEEK_CUR);
                break;

            case 2: // SEEK_END
                ret = SPIFFS_lseek(fp.fs, fp.fd,s->offset,SPIFFS_SEEK_END);
                break;
        }

        s->offset = ret;
        return 0;

    } else if (request == MP_STREAM_FLUSH) {
        uint32_t ret = SPIFFS_fflush(fp.fs, fp.fd);
        if (ret != 0) {
            *errcode = MP_EIO;
            SPIFFS_clearerr(fp.fs);
            return MP_STREAM_ERROR;
        }
        return 0;

    } else if (request == MP_STREAM_CLOSE) {
        // if fs==NULL then the file is closed and in that case this method is a no-op
        if (fp.fd > 0) {
            int32_t ret = SPIFFS_close(fp.fs, fp.fd);
            if (ret != 0) {
                *errcode = MP_EIO;
                SPIFFS_clearerr(fp.fs);
                return MP_STREAM_ERROR;
            }
        }
        return 0;

    } else {
        *errcode = MP_EINVAL;
        return MP_STREAM_ERROR;
    }
    return MP_STREAM_ERROR;
}

STATIC const mp_arg_t file_open_args[] = {
    { MP_QSTR_file, MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_rom_obj = MP_ROM_PTR(&mp_const_none_obj)} },
    { MP_QSTR_mode, MP_ARG_OBJ, {.u_obj = MP_OBJ_NEW_QSTR(MP_QSTR_r)} },
    { MP_QSTR_encoding, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_PTR(&mp_const_none_obj)} },
};
#define FILE_OPEN_NUM_ARGS MP_ARRAY_SIZE(file_open_args)

//------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t file_open(spiffs_user_mount_t *vfs, const mp_obj_type_t *type, mp_arg_val_t *args, bool raise)
{
	const char *file_name = mp_obj_str_get_str(args[0].u_obj);
    const char *mode_s = mp_obj_str_get_str(args[1].u_obj);
    if (strlen(file_name) > SPIFFS_OBJ_NAME_LEN) {
        if (raise) mp_raise_ValueError("name too long");
        return mp_const_none;
    }

    const char *lpath = spiffs_local_path(file_name);
    LOGD(TAG, "OPEN [%s]->[%s]", file_name, lpath);

    uint32_t mode = 0;
    while (*mode_s) {
        switch (*mode_s++) {
            case 'r':
                mode |= SPIFFS_O_RDONLY;
                break;
            case 'w':
                mode |= SPIFFS_O_RDWR | SPIFFS_O_CREAT;
                break;
            case 'x':
                mode |= SPIFFS_O_RDWR | SPIFFS_O_CREAT | SPIFFS_O_TRUNC;
                break;
            case 'a':
                mode |= SPIFFS_O_RDWR | SPIFFS_O_APPEND;
                break;
            case '+':
                mode |= SPIFFS_O_RDWR;
                break;
            #if MICROPY_PY_IO_FILEIO
            case 'b':
                type = &mp_type_vfs_spiffs_fileio;
                break;
            #endif
            case 't':
                type = &mp_type_vfs_spiffs_textio;
                break;
            default:
                if (raise) mp_raise_ValueError("not allowed mode character");
                return mp_const_none;
        }
    }

    spiffs_file_obj_t *o = m_new_obj_with_finaliser(spiffs_file_obj_t);
    o->base.type = type;
    spiffs_FILE fp;

    fp.fd = SPIFFS_open(&vfs->fs, lpath, mode, 0);

	fp.fs = &vfs->fs;
    if(fp.fd <= 0) {
        m_del_obj(spiffs_file_obj_t, o);
        if (raise) mp_raise_OSError(SPIFFS_errno_table[SPIFFS_ERR_NOT_FOUND]);
        return mp_const_none;
    }
    o->fp = fp;

    // Check if the opened file is directory
    spiffs_stat s;
    int ret = SPIFFS_fstat(&vfs->fs, fp.fd, &s);
    if (ret == SPIFFS_OK) {
        vfs_spiffs_meta_t * meta = (vfs_spiffs_meta_t *)&s.meta;
        if (meta->type == SPIFFS_TYPE_DIR) {
            // It is directory, cannot be opened
            errno = EISDIR;
            ret = SPIFFS_close(&vfs->fs, fp.fd);
            if (ret < 0) {
                errno = spiffs_res_to_errno(SPIFFS_errno(&vfs->fs));
                SPIFFS_clearerr(&vfs->fs);
            }
            if (raise) mp_raise_OSError(MP_EISDIR);
            return mp_const_none;
        }
    }
    if (mode != SPIFFS_RDONLY) {
        vfs_spiffs_update_meta(&vfs->fs, fp.fd, SPIFFS_TYPE_FILE);
        //if (!vfs_spiffs_update_meta(&vfs->fs, fp.fd, SPIFFS_TYPE_FILE)) {
        //    mp_raise_OSError(SPIFFS_errno_table[SPIFFS_ERR_NOT_FOUND]);
        //}
    }

    return MP_OBJ_FROM_PTR(o);
}

//------------------------------------------------------------------------------------------------------------
STATIC mp_obj_t file_obj_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_val_t arg_vals[FILE_OPEN_NUM_ARGS];
    mp_arg_parse_all_kw_array(n_args, n_kw, args, FILE_OPEN_NUM_ARGS, file_open_args, arg_vals);
    return file_open(NULL, type, arg_vals, true);
}

// TODO gc hook to close the file if not already closed

STATIC const mp_rom_map_elem_t rawfile_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_stream_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_stream_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_readline), MP_ROM_PTR(&mp_stream_unbuffered_readline_obj) },
    { MP_ROM_QSTR(MP_QSTR_readlines), MP_ROM_PTR(&mp_stream_unbuffered_readlines_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&mp_stream_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush), MP_ROM_PTR(&mp_stream_flush_obj) },
    { MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&mp_stream_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_seek), MP_ROM_PTR(&mp_stream_seek_obj) },
    { MP_ROM_QSTR(MP_QSTR_tell), MP_ROM_PTR(&mp_stream_tell_obj) },
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&mp_stream_close_obj) },
    { MP_ROM_QSTR(MP_QSTR___enter__), MP_ROM_PTR(&mp_identity_obj) },
    { MP_ROM_QSTR(MP_QSTR___exit__), MP_ROM_PTR(&file_obj___exit___obj) },
};

STATIC MP_DEFINE_CONST_DICT(rawfile_locals_dict, rawfile_locals_dict_table);
#if MICROPY_PY_IO_FILEIO
STATIC const mp_stream_p_t fileio_stream_p = {
    .read = file_obj_read,
    .write = file_obj_write,
    .ioctl = file_obj_ioctl,
};


const mp_obj_type_t mp_type_vfs_spiffs_fileio = {
    { &mp_type_type },
    .name = MP_QSTR_FileIO,
    .print = file_obj_print,
    .make_new = file_obj_make_new,
    .getiter = mp_identity_getiter,
    .iternext = mp_stream_unbuffered_iter,
    .protocol = &fileio_stream_p,
    .locals_dict = (mp_obj_dict_t*)&rawfile_locals_dict,
};
#endif
STATIC const mp_stream_p_t textio_stream_p = {
    .read = file_obj_read,
    .write = file_obj_write,
    .ioctl = file_obj_ioctl,
    .is_text = true,
};

const mp_obj_type_t mp_type_vfs_spiffs_textio = {
    { &mp_type_type },
    .name = MP_QSTR_TextIOWrapper,
    .print = file_obj_print,
    .make_new = file_obj_make_new,
    .getiter = mp_identity_getiter,
    .iternext = mp_stream_unbuffered_iter,
    .protocol = &textio_stream_p,
    .locals_dict = (mp_obj_dict_t*)&rawfile_locals_dict,
};


// Factory function for I/O stream classes
//--------------------------------------------------------------------------------------
STATIC mp_obj_t spiffs_builtin_open_self(mp_obj_t self_in, mp_obj_t path, mp_obj_t mode)
{
    // TODO: analyze buffering args and instantiate appropriate type
    spiffs_user_mount_t *self = MP_OBJ_TO_PTR(self_in);
    mp_arg_val_t arg_vals[FILE_OPEN_NUM_ARGS];
    arg_vals[0].u_obj = path;
    arg_vals[1].u_obj = mode;
    arg_vals[2].u_obj = mp_const_none;
    return file_open(self, &mp_type_vfs_spiffs_textio, arg_vals, true);
}
MP_DEFINE_CONST_FUN_OBJ_3(spiffs_vfs_open_obj, spiffs_builtin_open_self);

//-----------------------------------------------------------------------------------------
STATIC mp_obj_t spiffs_builtin_open_ex_self(mp_obj_t self_in, mp_obj_t path, mp_obj_t mode)
{
    // TODO: analyze buffering args and instantiate appropriate type
    spiffs_user_mount_t *self = MP_OBJ_TO_PTR(self_in);
    mp_arg_val_t arg_vals[FILE_OPEN_NUM_ARGS];
    arg_vals[0].u_obj = path;
    arg_vals[1].u_obj = mode;
    arg_vals[2].u_obj = mp_const_none;
    return file_open(self, &mp_type_vfs_spiffs_textio, arg_vals, false);
}
MP_DEFINE_CONST_FUN_OBJ_3(spiffs_vfs_open_ex_obj, spiffs_builtin_open_ex_self);

#endif // MICROPY_VFS && MICROPY_VFS_SPIFFS
