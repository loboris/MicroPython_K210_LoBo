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


#if !defined(_LITTLEFLASH_H_)
#define _LITTLEFLASH_H_ 1

#include "mpconfigport.h"

#if MICROPY_VFS_LITTLEFS

#include "w25qxx.h"
#include "lfs.h"
#include "extmod/vfs.h"

// these are the values for fs_user_mount_t.flags
#define MODULE_LITTLEFS      (0x0001) // readblocks[2]/writeblocks[2] contain native func
#define SYS_LITTLEFS         (0x0002) // fs_user_mount_t obj should be freed on umount
#define FSUSER_HAVE_IOCTL    (0x0004) // new protocol with ioctl
#define FSUSER_NO_FILESYSTEM (0x0008) // the block device has no filesystem on it

#define LITTLEFS_CFG_PHYS_SZ          MICRO_PY_FLASHFS_SIZE
#define LITTLEFS_CFG_PHYS_ERASE_SZ    MICRO_PY_FLASH_ERASE_SECTOR_SIZE
#define LITTLEFS_CFG_START_ADDR       MICRO_PY_FLASHFS_START_ADDRESS
#define LITTLEFS_CFG_END_ADDR         (LITTLEFS_CFG_START_ADDR+LITTLEFS_CFG_PHYS_SZ)

// LFS size of the erase sector
#define LITTLEFS_CFG_SECTOR_SIZE      MICRO_PY_LITTLEFS_SECTOR_SIZE
#define LITTLEFS_CFG_RWBLOCK_SIZE     MICRO_PY_LITTLEFS_RWBLOCK_SIZE

// Number of erase cycles before we should move data to another block.
#define LITTLEFS_CFG_BLOCK_CYCLES     (64)

#define LITTLEFS_CFG_MAX_FILES        (6)
#define LITTLEFS_CFG_MAX_FILE_SIZE    (LITTLEFS_CFG_PHYS_SZ / 2)
#define LITTLEFS_CFG_MAX_NAME_LEN     (128)
#define LITTLEFS_CFG_LOOKAHEAD_SIZE   (32)
#define LITTLEFS_CFG_AUTOFORMAT       (1)

#define LITTLEFS_ATTR_MTIME           0x10


typedef struct _littlefs_file_obj_t {
    mp_obj_base_t base;
    lfs_t* fs;
    lfs_file_t fd;
    uint32_t timestamp;
    uint8_t file_buffer[LITTLEFS_CFG_SECTOR_SIZE];
    struct lfs_attr attrs;
    struct lfs_file_config cfg;
} __attribute__((aligned(8))) littlefs_file_obj_t;

typedef struct {
	struct lfs_config lfs_cfg;	// littlefs configuration
	lfs_t lfs;					// The littlefs type
	bool mounted;
} __attribute__((aligned(8))) littleFlash_t;

typedef struct _little_user_mount_t {
    mp_obj_base_t base;
    uint16_t flags;
    littleFlash_t *fs;
    mp_obj_t read_obj[5];
    mp_obj_t write_obj[5];
    mp_obj_t erase_obj[4];
    mp_vfs_proto_t *protocol;
} __attribute__((aligned(8))) littlefs_user_mount_t;


extern littlefs_user_mount_t littlefs_user_mount_handle;
extern void *vfs_flashfs;
extern const mp_obj_type_t mp_littlefs_vfs_type;
extern const mp_obj_type_t mp_type_vfs_littlefs_fileio;
extern const mp_obj_type_t mp_type_vfs_littlefs_textio;
extern bool force_erase_fs_flash;

MP_NOINLINE bool init_flash_filesystem();

const char *littlefs_local_path(const char *path);
int set_timestamp(lfs_t *fs, const char *path);
int get_timestamp(lfs_t *fs, const char *path);
int map_lfs_error(int err);

void littleFlash_term();

MP_DECLARE_CONST_FUN_OBJ_3(littlefs_vfs_open_obj);

#endif // MICROPY_VFS_LITTLEFS

#endif
