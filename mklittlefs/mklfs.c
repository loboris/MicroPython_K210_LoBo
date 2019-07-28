/*
 * Image creator for the littlefs
 *
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

#include "lfs.h"
#include "lfs_util.h"

#include <stdio.h>
#include <time.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <getopt.h>

#define LITTLEFS_ATTR_MTIME         0x10
#define LITTLEFS_IMAGE_SIZE                  (12*1024*1024)

typedef struct _littlefs_file_obj_t {
    lfs_t* fs;
    lfs_file_t fd;
    uint32_t timestamp;
    uint8_t file_buffer[512];
    struct lfs_attr attrs;
    struct lfs_file_config cfg;
} __attribute__((aligned(8))) littlefs_file_obj_t;

static struct lfs_config config = {0};
static lfs_t lfs = {0};
static uint8_t *lfs_image = NULL;

static uint32_t block_size = 512;
static uint32_t block_count = (10*1024*1024) / 512;
static uint32_t lookahead = 32;
static char image_name[256] = {0};
static char image_dir[256] = {0};
static uint32_t fs_offset = 0;

static uint32_t *erase_log = NULL;
static uint32_t *prog_log = NULL;
static uint32_t *read_log = NULL;

static uint8_t *read_buffer __attribute__((aligned (8)));
static uint8_t *prog_buffer __attribute__((aligned (8)));
static uint8_t *lookahead_buffer __attribute__((aligned (8)));

// === Image access functions ===

//-------------------------------------------------------------------------------------------------------------
int lfs_img_read(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    memcpy((uint8_t *)buffer, lfs_image + fs_offset + ((uint32_t)block * cfg->block_size) + (uint32_t)off, (size_t)size);
    if (read_log) read_log[block]++;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
int lfs_img_prog(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    memcpy(lfs_image + fs_offset + ((uint32_t)block * cfg->block_size) + (uint32_t)off, (uint8_t *)buffer, (size_t)size);
    if (size != cfg->block_size) printf("SIZE=%d, BLOCK=%d, OFFSET=%d\r\n", size, block, off);
    if (prog_log) prog_log[block]++;
    return 0;
}

//----------------------------------------------------------------
int lfs_img_erase(const struct lfs_config *cfg, lfs_block_t block)
{
	memset(lfs_image + fs_offset + ((uint32_t)block * cfg->block_size), 0xFF, cfg->block_size);
    if (erase_log) erase_log[block]++;
    return 0;
}

//--------------------------------------------------------------------
int lfs_img_sync(__attribute__((unused)) const struct lfs_config *cfg)
{
    // do nothing
    return 0;
}

// === File functions ===================

//---------------------------------------
int addFile(char* name, const char* path)
{
    FILE* src = fopen(path, "rb");
    if (!src) {
        printf("error: failed to open '%s' for reading\r\n", path);
        return 1;
    }

    int lfs_flags = LFS_O_RDWR | LFS_O_CREAT;

    littlefs_file_obj_t o = {0};
    o.fs = &lfs;
    o.cfg.buffer = &o.file_buffer;

    time_t rawtime;
    struct tm *info;
    time( &rawtime );
    info = localtime( &rawtime );
    o.timestamp = (uint32_t)mktime(info);
    o.attrs.type = LITTLEFS_ATTR_MTIME;
    o.attrs.buffer = &o.timestamp;
    o.attrs.size = sizeof(uint32_t);
    o.cfg.attr_count = 1;
    o.cfg.attrs = &o.attrs;

    int err = lfs_file_opencfg(&lfs, &o.fd, name, lfs_flags, &o.cfg);
    if (err != LFS_ERR_OK) {
        printf("error: failed to open lfs file '%s' for writting (%d)\r\n", name, err);
        return 2;
    }

    // read file size
    fseek(src, 0, SEEK_END);
    size_t size = ftell(src);
    fseek(src, 0, SEEK_SET);

    size_t left = size;
    uint8_t data_byte;
    while (left > 0){
        if (1 != fread(&data_byte, 1, 1, src)) {
            printf("fread error!\r\n");

            fclose(src);
            lfs_file_close(&lfs, &o.fd);
            return 1;
        }
        lfs_ssize_t res = lfs_file_write(&lfs, &o.fd, &data_byte, 1);
        if (res < 0) {
            printf("LittleFS write error (%d)\r\n", res);

            fclose(src);
            lfs_file_close(&lfs, &o.fd);
            return 1;
        }
        left -= 1;
    }

    lfs_file_close(&lfs, &o.fd);

    return 0;
}

//--------------------
int addDir(char* name)
{
    int err = lfs_mkdir(&lfs, name);
    return err;
}

//----------------------------------------------------
int addFiles(const char* dirname, const char* subPath)
{
    DIR *dir;
    struct dirent *ent;
    bool error = false;
    char dirPath[512] = {0};
    char dirpath[512] = {0};
    char newSubPath[512] = {0};
    char filepath[512] = {0};
    char fullpath[512] = {0};

    sprintf(dirPath, "%s", dirname);
    strcat(dirPath, subPath);

    // Open directory
    if ((dir = opendir(dirPath)) != NULL) {
        // Read files from directory.
        while ((ent = readdir (dir)) != NULL) {

            // Ignore directory itself.
            if ((strcmp(ent->d_name, ".") == 0) || (strcmp(ent->d_name, "..") == 0)) {
                continue;
            }

            sprintf(fullpath, "%s", dirPath);
            strcat(fullpath, ent->d_name);
            struct stat path_stat;
            stat(fullpath, &path_stat);

            if (!S_ISREG(path_stat.st_mode)) {
                // Check if path is a directory.
                if (S_ISDIR(path_stat.st_mode)) {
                    sprintf(dirpath, "%s", subPath);
                    strcat(dirpath, ent->d_name);
                    printf("%s [D]\r\n", dirpath);
                    int res = addDir(dirpath);
                    if (res != 0) {
                        printf("error adding directory (open)!\r\n");
                        error = true;
                        break;
                    }
                    // Prepare new sub path.
                    sprintf(newSubPath, "%s", subPath);
                    strcat(newSubPath, ent->d_name);
                    strcat(newSubPath, "/");

                    if (addFiles(dirname, newSubPath) != 0) {
                        printf("Error for adding content from '%s' !\r\n", ent->d_name);
                        error = true;
                        break;
                    }

                    continue;
                }
                else {
                    printf("skipping '%s'\r\n", ent->d_name);
                    continue;
                }
            }

            // File path with directory name as root folder.
            sprintf(filepath, "%s", subPath);
            strcat(filepath, ent->d_name);
            printf("%s\r\n", filepath);

            // Add File to image.
            if (addFile(filepath, fullpath) != 0) {
                printf("error adding file!\r\n");
                error = true;
                break;
            }
        } // end while
        closedir(dir);
    }
    else {
        printf("warning: can't read source directory\r\n");
        return 1;
    }

    return (error) ? 1 : 0;
}


//----------------------------------------
int lfs_img_create(struct lfs_config *cfg)
{
    lfs_image = malloc(LITTLEFS_IMAGE_SIZE);
    if (lfs_image == NULL) return -1;

    memset(lfs_image, 0xFF, LITTLEFS_IMAGE_SIZE);

    read_buffer = malloc(cfg->block_size);
    prog_buffer = malloc(config.prog_size);
    lookahead_buffer = malloc(cfg->lookahead_size);
    
    // setup function pointers
    cfg->read  = lfs_img_read;
    cfg->prog  = lfs_img_prog;
    cfg->erase = lfs_img_erase;
    cfg->sync  = lfs_img_sync;

    cfg->read_buffer      = read_buffer;
    cfg->prog_buffer      = prog_buffer;
    cfg->lookahead_buffer = lookahead_buffer;

    return 0;
}

//----------------------
int lfs_img_format(void)
{
    int err = lfs_img_create(&config);
    if (err) {
        return err;
    }

    err = lfs_format(&lfs, &config);

    return err;
}

//------------------
int save_image(void)
{
    // find image size
    int size = LITTLEFS_IMAGE_SIZE;
    for (size=LITTLEFS_IMAGE_SIZE-1; size>0; size--) {
        if (lfs_image[size] != 0xFF) break;
    }
    int img_size = ((size / 4096) * 4096) + 4096;
    printf("Image size: %d (%d)\r\n", size, img_size);

    printf("Saving image to '%s'\r\n", image_name);
    FILE* img_file = fopen(image_name, "wb");
    if (!img_file) {
        printf("error: failed to open '%s'\r\n", image_name);
        return 1;
    }
    fwrite(lfs_image, 1, img_size, img_file);
    fclose(img_file);

    return 0;
}

//---------------------
int lfs_img_mount(void)
{
    printf("\r\nCreating and mounting LittleFS image...\r\n");
    int err = 0;

    fs_offset = 0;

    config.read_size = block_size;      // LITTLEFS_CFG_RWBLOCK_SIZE
    config.prog_size = block_size;      // LITTLEFS_CFG_RWBLOCK_SIZE
    config.block_size = block_size;     // LITTLEFS_CFG_SECTOR_SIZE
    config.block_count = block_count;   // LITTLEFS_CFG_PHYS_SZ / LITTLEFS_CFG_SECTOR_SIZE
    config.cache_size = block_size;     // LITTLEFS_CFG_SECTOR_SIZE
    config.lookahead_size = lookahead;  // LITTLEFS_CFG_LOOKAHEAD_SIZE

    config.file_max = (block_count * block_size) / 2;
    config.name_max = 128;
    config.block_cycles = 64;

    err = lfs_img_create(&config);
    if (err) {
        printf("  Error creating image (%d)\r\n", err);
        return err;
    }
    printf("  Image file created.\r\n");

    err = lfs_format(&lfs, &config);
    if (err) {
        printf("  Error formating image (%d)\r\n", err);
        return err;
    }
    printf("  Image file formated.\r\n");

    err = lfs_mount(&lfs, &config);
    if (err) {
        printf("Error mounting image (%d)\r\n", err);
        return err;
    }
    printf("  Image file mounted.\r\n");
    return 0;
}

//------------------------
int lfs_create_image(void)
{
    int err = 0;

    err = lfs_img_mount();
    if (err) return err;

    printf("\r\nAdding files from image directory:\r\n");
    printf("  '%s'\r\n", image_dir);
    printf("----------------------------------\r\n\r\n");
    err = addFiles(image_dir, "/");
    printf("\r\n");
    if (err != 0) {
        printf("---------------------------------\r\n");
        printf("Errors occured while adding files\r\n");
        printf("Try to increase the file system size (-c option, block count)\r\n");

        err = lfs_unmount(&lfs);
        if (err) {
            printf("Error unmounting image (%d)\r\n", err);
        }
        return 1;
    }

    err = lfs_unmount(&lfs);
    if (err) {
        printf("Error unmounting image (%d)\r\n", err);
    }

    save_image();

    return 0;
}


//===============================
int main(int argc, char **argv) {
    // parse options
    int c;
    char *cvalue = NULL;
    char *ptr;
    bool help = false;

    printf("\r\n");
    while ( (c = getopt(argc, argv, "b:c:l")) != -1) {
        switch (c) {
        case 'b':
            cvalue = optarg;
            block_size = (uint32_t)strtol(cvalue, &ptr, 10);
            break;
        case 'c':
            cvalue = optarg;
            block_count = (uint32_t)strtol(cvalue, &ptr, 10);
            break;
        case 'l':
            cvalue = optarg;
            lookahead = (uint32_t)strtol(cvalue, &ptr, 10);
            break;
        case 'h':
            help = true;
            break;
        case '?':
            break;
        default:
            printf ("?? getopt returned character code 0%o ??\r\n", c);
        }
    }

    if (argc < 2) help = true;
    if (help) {
        printf("Usage:\r\n");
        printf("  mklfs [-b block_size] [-c block_count] [-l lookahead_size] image_dir image_name\r\n");
        printf("      block_size: default=512   (MICRO_PY_LITTLEFS_SECTOR_SIZE)\r\n");
        printf("     block_count: default=20480 (MICRO_PY_FLASHFS_SIZE / MICRO_PY_LITTLEFS_SECTOR_SIZE)\r\n");
        printf("  lookahead_size: default=32    (LITTLEFS_CFG_LOOKAHEAD_SIZE)\r\n");
        printf("\r\n");
        return 0;
    }

    sprintf(image_dir, "%s", argv[optind]);
    sprintf(image_name, "%s", argv[optind+1]);

    printf("Creating LittleFS image\r\n");
    printf("=======================\r\n");
    printf("Image directory:\r\n  '%s'\r\n", image_dir);
    printf("Image name:\r\n  '%s'\r\n", image_name);
    printf("Block size=%u, Block count=%u, lookahead=%u\r\n", block_size, block_count, lookahead);

    int err = lfs_create_image();
    if (lfs_image) free(lfs_image);
    printf("=======================\r\n");
    printf("\r\n");
    
    return err;
}
