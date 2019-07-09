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


#include "mpconfigport.h"

#if MICROPY_PY_USE_SQLITE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include "sqlite3_k210.h"
#include "syslog.h"
#include "mphalport.h"
#include "modmachine.h"
#include "extmod/vfs.h"
#include "py/stream.h"



sqlite3_vfs  K210Vfs = {
	1,			                // iVersion
	sizeof(K210_file),          // Size of subclassed sqlite3_file
	K210_DEFAULT_MAXNAMESIZE+1, // Maximum file pathname length
	NULL,			            // pNext
	"K210",		                // name
	0,			                // pAppData
	K210_Open,		            // xOpen
	K210_Delete,		        // xDelete
	K210_Access,		        // xAccess
	K210_FullPathname,	        // xFullPathname
	0,		                    // xDlOpen
	0,	                        // xDlError
	0,		                    // xDlSym
	0,          	            // xDlClose
	K210_Randomness,	        // xRandomness
	K210_Sleep,		            // xSleep
	K210_CurrentTime,	        // xCurrentTime
	0,			                // xGetLastError
    0,
    0,
    0,
    0,
};

const sqlite3_io_methods K210IoMethods = {
	1,
	K210_Close,
	K210_Read,
	K210_Write,
	K210_Truncate,
	K210_Sync,
	K210_FileSize,
	K210_Lock,
	K210_Unlock,
	K210_CheckReservedLock,
	K210_FileControl,
	K210_SectorSize,
	K210_DeviceCharacteristics,
	0,
    0,
    0,
    0,
    0,
    0,
};

const sqlite3_io_methods K210MemMethods = {
	1,
	K210mem_Close,
	K210mem_Read,
	K210mem_Write,
	K210_Truncate,
	K210mem_Sync,
	K210mem_FileSize,
	K210_Lock,
	K210_Unlock,
	K210_CheckReservedLock,
	K210_FileControl,
	K210_SectorSize,
	K210_DeviceCharacteristics,
    0,
    0,
    0,
    0,
    0,
    0,
};

#if USER_MEM_ALLOC
const sqlite3_mem_methods K210AllocMethods = {
  K210alloc_Malloc,     // Memory allocation function
  K210alloc_Free,       // Free a prior allocation
  K210alloc_Realloc,    // Resize an allocation
  K210alloc_Size,       // Return the size of an allocation
  K210alloc_Roundup,    // Round up request size to allocation size
  K210alloc_Init,       // Initialize the memory allocator
  K210alloc_Shutdown,   // Deinitialize the memory allocator
  0                     // Argument to xInit() and xShutdown()
};

static int K210alloc_last_size = 0;
#endif

static const char* TAG = "[SQLITE3]";
bool sqlite3_debug = true;
bool sqlite3_alloc_debug = false;

// ==== Memory (file cache ) functions ============================================================

//--------------------------------------------------------------------------------------------------------
static uint32_t linkedlist_store (linkedlist_t **leaf, uint32_t offset, uint32_t len, const uint8_t *data)
{
	const uint8_t blank[CACHEBLOCKSZ] = { 0 };
	uint16_t blockid = offset/CACHEBLOCKSZ;
	linkedlist_t *block;

	if (!memcmp(data, blank, CACHEBLOCKSZ))
		return len;

	block = *leaf;
	if (!block || ( block->blockid != blockid ) ) {
		block = (linkedlist_t *) sqlite3_malloc ( sizeof( linkedlist_t ) );
		if (!block)
			return SQLITE_NOMEM;

		memset (block->data, 0, CACHEBLOCKSZ);
		block->blockid = blockid;
	}

	if (!*leaf) {
		*leaf = block;
		block->next = NULL;
	} else if (block != *leaf) {
		if (block->blockid > (*leaf)->blockid) {
			block->next = (*leaf)->next;
			(*leaf)->next = block;
		} else {
			block->next = (*leaf);
			(*leaf) = block;
		}
	}

	memcpy (block->data + offset%CACHEBLOCKSZ, data, len);

	return len;
}

//-----------------------------------------------------------------------------------------------
static uint32_t filecache_pull (pFileCache_t cache, uint32_t offset, uint32_t len, uint8_t *data)
{
	uint16_t i;
	float blocks;
	uint32_t r = 0;

	blocks = ( offset % CACHEBLOCKSZ + len ) / (float) CACHEBLOCKSZ;
	if (blocks == 0.0)
		return 0;
	if (!cache->list)
		return 0;

	if (( blocks - (int) blocks) > 0.0)
		blocks = blocks + 1.0;

	for (i = 0; i < (uint16_t) blocks; i++) {
		uint16_t round;
		float relablock;
		linkedlist_t *leaf;
		uint32_t relaoffset, relalen;
		uint8_t * reladata = (uint8_t*) data;

		relalen = len - r;

		reladata = reladata + r;
		relaoffset = offset + r;

		round = CACHEBLOCKSZ - relaoffset%CACHEBLOCKSZ;
		if (relalen > round) relalen = round;

		for (leaf = cache->list; leaf && leaf->next; leaf = leaf->next) {
			if ( ( leaf->next->blockid * CACHEBLOCKSZ ) > relaoffset )
				break;
		}

		relablock = relaoffset/((float)CACHEBLOCKSZ) - leaf->blockid;

		if ( ( relablock >= 0 ) && ( relablock < 1 ) )
			memcpy (data + r, leaf->data + (relaoffset % CACHEBLOCKSZ), relalen);

		r = r + relalen;
	}

	return 0;
}

//-----------------------------------------------------------------------------------------------------
static uint32_t filecache_push (pFileCache_t cache, uint32_t offset, uint32_t len, const uint8_t *data)
{
	uint16_t i;
	float blocks;
	uint32_t r = 0;
	uint8_t updateroot = 0x1;

	blocks = ( offset % CACHEBLOCKSZ + len ) / (float) CACHEBLOCKSZ;

	if (blocks == 0.0)
		return 0;

	if (( blocks - (int) blocks) > 0.0)
		blocks = blocks + 1.0;

	for (i = 0; i < (uint16_t) blocks; i++) {
		uint16_t round;
		uint32_t localr;
		linkedlist_t *leaf;
		uint32_t relaoffset, relalen;
		uint8_t * reladata = (uint8_t*) data;

		relalen = len - r;

		reladata = reladata + r;
		relaoffset = offset + r;

		round = CACHEBLOCKSZ - relaoffset%CACHEBLOCKSZ;
		if (relalen > round) relalen = round;

		for (leaf = cache->list; leaf && leaf->next; leaf = leaf->next) {
			if ( ( leaf->next->blockid * CACHEBLOCKSZ ) > relaoffset )
				break;
			updateroot = 0x0;
		}

		localr = linkedlist_store(&leaf, relaoffset, (relalen > CACHEBLOCKSZ) ? CACHEBLOCKSZ : relalen, reladata);
		if (localr == SQLITE_NOMEM)
			return SQLITE_NOMEM;

		r = r + localr;

		if (updateroot & 0x1)
			cache->list = leaf;
	}

	if (offset + len > cache->size)
		cache->size = offset + len;

	return r;
}

//---------------------------------------------
static void filecache_free (pFileCache_t cache)
{
	pLinkedList_t ll = cache->list, next;

	while (ll != NULL) {
		next = ll->next;
		sqlite3_free (ll);
		ll = next;
	}
}

//---------------------------------
int K210mem_Close(sqlite3_file *id)
{
	K210_file *file = (K210_file*) id;

	filecache_free(file->cache);
	sqlite3_free (file->cache);

	if (sqlite3_debug) LOGM(TAG, "K210mem_Close: %s OK", file->name);
	return SQLITE_OK;
}

//--------------------------------------------------------------------------------
int K210mem_Read(sqlite3_file *id, void *buffer, int amount, sqlite3_int64 offset)
{
	int32_t ofst;
	K210_file *file = (K210_file*) id;
	ofst = (int32_t)(offset & 0x7FFFFFFF);

	filecache_pull (file->cache, ofst, amount, (uint8_t *) buffer);

	if (sqlite3_debug) LOGM(TAG, "K210mem_Read: %s [%d] [%d] OK", file->name, ofst, amount);
	return SQLITE_OK;
}

//---------------------------------------------------------------------------------------
int K210mem_Write(sqlite3_file *id, const void *buffer, int amount, sqlite3_int64 offset)
{
	int32_t ofst;
	K210_file *file = (K210_file*) id;

	ofst = (int32_t)(offset & 0x7FFFFFFF);

	filecache_push (file->cache, ofst, amount, (const uint8_t *) buffer);

	if (sqlite3_debug) LOGM(TAG, "K210mem_Write: %s [%d] [%d] OK", file->name, ofst, amount);
	return SQLITE_OK;
}

//-------------------------------------------
int K210mem_Sync(sqlite3_file *id, int flags)
{
	K210_file *file = (K210_file*) id;
	if (sqlite3_debug) LOGM(TAG, "K210mem_Sync: %s OK", file->name);
	return  SQLITE_OK;
}

//---------------------------------------------------------
int K210mem_FileSize(sqlite3_file *id, sqlite3_int64 *size)
{
	K210_file *file = (K210_file*) id;

	*size = 0LL | file->cache->size;
	if (sqlite3_debug) LOGM(TAG, "K210mem_FileSize: %s [%d] OK", file->name, file->cache->size);
	return SQLITE_OK;
}

// ==== File IO functions =========================================================================

#define PROXY_MAX_ARGS (2)

//---------------------------------------------------------------------------------------------------------
static mp_obj_t mp_vfs_proxy_call(mp_vfs_mount_t *vfs, qstr meth_name, size_t n_args, const mp_obj_t *args)
{
    if (vfs == MP_VFS_NONE) return mp_const_none;
    if (vfs == MP_VFS_ROOT)  return mp_const_none;
    mp_obj_t meth[2 + PROXY_MAX_ARGS];
    mp_load_method(vfs->obj, meth_name, meth);
    if (args != NULL) {
        memcpy(meth + 2, args, n_args * sizeof(*args));
    }
    return mp_call_method_n_kw(n_args, 0, meth);
}

/* SQLite guarantees that the zFilename parameter to xOpen is either a NULL pointer
   or string obtained from xFullPathname() with an optional suffix added.
   SQLite will also add one of the following flags to the xOpen() call, depending on the object being opened:
    SQLITE_OPEN_MAIN_DB
    SQLITE_OPEN_MAIN_JOURNAL
    SQLITE_OPEN_TEMP_DB
    SQLITE_OPEN_TEMP_JOURNAL
    SQLITE_OPEN_TRANSIENT_DB
    SQLITE_OPEN_SUBJOURNAL
    SQLITE_OPEN_MASTER_JOURNAL
    SQLITE_OPEN_WAL
*/
int K210_Open( sqlite3_vfs * vfs, const char * path, sqlite3_file * file, int flags, int * outflags )
{
	char mode[5];
    K210_file *p = (K210_file*) file;

	strcpy(mode, "r");
	if (path == NULL) {
	    if (sqlite3_debug) LOGQ(TAG, "K210_Open: NULL file name");
	    return SQLITE_IOERR;
	}

	if (flags & SQLITE_OPEN_READONLY) strcpy(mode, "r");
	if ((flags & SQLITE_OPEN_READWRITE) || (flags & SQLITE_OPEN_MAIN_JOURNAL)) {
		int result;
		if (SQLITE_OK != K210_Access(vfs, path, flags, &result)) {
		    if (sqlite3_debug) LOGQ(TAG, "K210_Open: [1] %s %s, Access Error", path, mode);
		    return SQLITE_CANTOPEN;
		}

		if (result == 1) strcpy(mode, "r+");    // file exists
		else strcpy(mode, "w+");                // file does not exists
	}

	memset (p, 0, sizeof(K210_file));

	if (flags & SQLITE_OPEN_MAIN_JOURNAL) {
	    strncpy (p->name, path, K210_DEFAULT_MAXNAMESIZE);
	    p->name[K210_DEFAULT_MAXNAMESIZE-1] = '\0';
		p->fd = mp_const_none;
		p->cache = (filecache_t *) sqlite3_malloc(sizeof (filecache_t));
		if (! p->cache ) return SQLITE_NOMEM;
		memset (p->cache, 0, sizeof(filecache_t));

		p->base.pMethods = &K210MemMethods;
		if (sqlite3_debug) LOGY(TAG, "K210_Open: [2] %s (MEM Methods) OK", p->name);
		return SQLITE_OK;
	}

    mp_obj_t args[2];
    args[1] = mp_obj_new_str(mode, 2);
    const char *p_out;

    mp_vfs_mount_t *mpvfs = mp_vfs_lookup_path(path, &p_out);
    if (mpvfs != MP_VFS_NONE && mpvfs != MP_VFS_ROOT) {
        strncpy (p->name, p_out, K210_DEFAULT_MAXNAMESIZE);
        p->name[K210_DEFAULT_MAXNAMESIZE-1] = '\0';
        args[0] = mp_obj_new_str(p_out, strlen(p_out));
    }
    else {
        strncpy (p->name, path, K210_DEFAULT_MAXNAMESIZE);
        p->name[K210_DEFAULT_MAXNAMESIZE-1] = '\0';
        args[0] = mp_obj_new_str(path, strlen(path));
    }
    if (sqlite3_debug) LOGM(TAG, "K210_Open: [1] %s %s", p->name, mode);

    p->fd = mp_vfs_proxy_call(mpvfs, MP_QSTR_openex, 2, (mp_obj_t*)&args);
    if (p->fd == mp_const_none) {
        if (sqlite3_debug) LOGQ(TAG, "K210_Open: [3] %s ERROR", p->name);
        return SQLITE_CANTOPEN;
    }

	p->base.pMethods = &K210IoMethods;
	if (sqlite3_debug) LOGM(TAG, "K210_Open: [3] %s OK", p->name);
	return SQLITE_OK;
}

//------------------------------
int K210_Close(sqlite3_file *id)
{
	K210_file *file = (K210_file*) id;
    if (sqlite3_debug) LOGM(TAG, "K210_Close: [1] %s", file->name);

    // mp_stream_close with no exception raised on error
    const mp_stream_p_t *stream_p = (const mp_stream_p_t*)((const mp_obj_base_t*)MP_OBJ_TO_PTR(file->fd))->type->protocol;
    int error;
    mp_uint_t res = stream_p->ioctl(file->fd, MP_STREAM_CLOSE, 0, &error);
    if (res == MP_STREAM_ERROR) {
        if (sqlite3_debug) LOGQ(TAG, "K210_Close: [2] %s ERROR (%d)", file->name, error);
        return SQLITE_IOERR_CLOSE;
    }

	if (sqlite3_debug) LOGM(TAG, "K210_Close: [2] %s OK", file->name);
	return SQLITE_OK;
}

//-----------------------------------------------------------------------------
int K210_Read(sqlite3_file *id, void *buffer, int amount, sqlite3_int64 offset)
{
	int nRead;
	int32_t ofst;
	K210_file *file = (K210_file*) id;
    mp_hal_wdt_reset();

	if (sqlite3_debug) LOGM(TAG, "K210_Read: [1] %s %d %lld", file->name, amount, offset);
	ofst = mp_stream_posix_lseek(file->fd, offset, SEEK_SET);
	if (ofst != offset) {
	    if (sqlite3_debug) LOGQ(TAG, "K210_Read: [2] Seek Error (%d != %lld)", ofst, offset);
		return SQLITE_IOERR_SHORT_READ /* SQLITE_IOERR_SEEK */;
	}

	nRead = mp_stream_posix_read(file->fd, buffer, amount);
	if ( nRead == amount ) {
	    if (sqlite3_debug) LOGM(TAG, "K210_Read: [3] %s OK", file->name);
		return SQLITE_OK;
	}
	else if (nRead < 0) {
	    if (sqlite3_debug) LOGQ(TAG, "K210_Read: [4] %s FAIL", file->name);
	    return SQLITE_IOERR_READ;
	}
    if (sqlite3_debug) LOGQ(TAG, "K210_Read: [3] %s, Read Error (%d <> %d)", file->name, nRead, amount);
    return SQLITE_IOERR_SHORT_READ;
}

//------------------------------------------------------------------------------------
int K210_Write(sqlite3_file *id, const void *buffer, int amount, sqlite3_int64 offset)
{
	int nWrite;
	int32_t ofst;
	K210_file *file = (K210_file*) id;
	mp_hal_wdt_reset();

	if (sqlite3_debug) LOGM(TAG, "K210_Write: [1] %s %d %lld ", file->name, amount, offset);
	ofst = mp_stream_posix_lseek(file->fd, offset, SEEK_SET);
	if (ofst != offset) {
	    if (sqlite3_debug) LOGQ(TAG, "K210_Write: [1] %s Seek Error (%d)", file->name, ofst);
		return SQLITE_IOERR_SEEK;
	}

	nWrite = mp_stream_posix_write(file->fd, buffer, amount);
	if ( nWrite != amount ) {
		if (sqlite3_debug) LOGQ(TAG, "K210_Write: [2] %s, ERROR (%d <> %d)", file->name, nWrite, amount);
		return SQLITE_IOERR_WRITE;
	}

	if (sqlite3_debug) LOGM(TAG, "K210_Write: [3] %s OK", file->name);
	return SQLITE_OK;
}

//------------------------------------------------------
int K210_Truncate(sqlite3_file *id, sqlite3_int64 bytes)
{
	if (sqlite3_debug) LOGM(TAG, "K210_Truncate:");
	return SQLITE_OK;
}

//----------------------------------------------------------------
int K210_Delete(sqlite3_vfs * vfs, const char * path, int syncDir)
{
    if (sqlite3_debug) LOGM(TAG, "K210_Delete: [1] %s", path);
    int rc = mp_vfs_import_stat(path);
    if (rc != MP_IMPORT_STAT_FILE) {
        if (sqlite3_debug) LOGM(TAG, "K210_Delete: [2] %s ERROR", path);
        return SQLITE_IOERR_DELETE;
    }
    mp_vfs_remove(mp_obj_new_str(path, strlen(path)));

	if (sqlite3_debug) LOGM(TAG, "K210_Delete: [2] %s OK", path);
	return SQLITE_OK;
}

//------------------------------------------------------
int K210_FileSize(sqlite3_file *id, sqlite3_int64 *size)
{
	K210_file *file = (K210_file*) id;
    // Get file size
    int curr = mp_stream_posix_lseek(file->fd, 0, SEEK_CUR);
    int fsize = mp_stream_posix_lseek(file->fd, 0, SEEK_END);
    curr = mp_stream_posix_lseek(file->fd, curr, SEEK_SET);
	if (fsize < -1) {
	    if (sqlite3_debug) LOGM(TAG, "K210_FileSize: %s: Error", file->name);
		return SQLITE_IOERR_FSTAT;
	}
    *size = fsize;
	if (sqlite3_debug) LOGM(TAG, "K210_FileSize: %s: %d[%lld]", file->name, fsize, *size);
	return SQLITE_OK;
}

//----------------------------------------
int K210_Sync(sqlite3_file *id, int flags)
{
	K210_file *file = (K210_file*) id;
    if (sqlite3_debug) LOGM(TAG, "K210_Sync: [1] %s", file->name);

	int rc = mp_stream_posix_fsync( file->fd );
	if (sqlite3_debug) LOGM(TAG, "K210_Sync: [2] %d", rc);

	return (rc < 0) ? SQLITE_IOERR_FSYNC : SQLITE_OK;
}

/*
  The flags argument to xAccess() may be SQLITE_ACCESS_EXISTS to test for the existence of a file,
  or SQLITE_ACCESS_READWRITE to test whether a file is readable and writable,
  or SQLITE_ACCESS_READ to test whether a file is at least readable.
  The file can be a directory. (?!)
 */
//----------------------------------------------------------------------------
int K210_Access(sqlite3_vfs * vfs, const char * path, int flags, int * result)
{
    int rc = mp_vfs_import_stat(path);
    *result = (rc != MP_IMPORT_STAT_NO_EXIST);

	if (sqlite3_debug) LOGM(TAG, "K210_Access: %s [%d]", path, *result);
	return SQLITE_OK;
}

//-----------------------------------------------------------------------------------
int K210_FullPathname(sqlite3_vfs * vfs, const char * path, int len, char * fullpath)
{
	// just copy the path
	strncpy( fullpath, path, len );
	fullpath[ len - 1 ] = '\0';

	if (sqlite3_debug) LOGM(TAG, "K210_FullPathname: (%d) %s", len, fullpath);
	return SQLITE_OK;
}

//--------------------------------------------
int K210_Lock(sqlite3_file *id, int lock_type)
{
	//K210_file *file = (K210_file*) id;

	if (sqlite3_debug) LOGM(TAG, "K210_Lock");
	return SQLITE_OK;
}

//----------------------------------------------
int K210_Unlock(sqlite3_file *id, int lock_type)
{
	//K210_file *file = (K210_file*) id;

	if (sqlite3_debug) LOGM(TAG, "K210_Unlock");
	return SQLITE_OK;
}

//-------------------------------------------------------
int K210_CheckReservedLock(sqlite3_file *id, int *result)
{
	//K210_file *file = (K210_file*) id;

	*result = 0;

	if (sqlite3_debug) LOGM(TAG, "K210_CheckReservedLock");
	return SQLITE_OK;
}

//-------------------------------------------------------
int K210_FileControl(sqlite3_file *id, int op, void *arg)
{
	//K210_file *file = (K210_file*) id;

	if (sqlite3_debug) LOGM(TAG, "K210_FileControl: [%04X]", op);
	return SQLITE_OK;
}

//-----------------------------------
int K210_SectorSize(sqlite3_file *id)
{
    //K210_file *file = (K210_file*) id;

	if (sqlite3_debug) LOGM(TAG, "K210_SectorSize: %d", 512);
	return 512;
}

//----------------------------------------------
int K210_DeviceCharacteristics(sqlite3_file *id)
{
	//K210_file *file = (K210_file*) id;
    int dc = SQLITE_IOCAP_UNDELETABLE_WHEN_OPEN | SQLITE_IOCAP_UNDELETABLE_WHEN_OPEN;
	if (sqlite3_debug) LOGM(TAG, "K210_DeviceCharacteristics: %4X", dc);
	return 0;
}

//------------------------------------------------------------
int K210_Randomness(sqlite3_vfs * vfs, int len, char * buffer)
{
	long rdm;
	int sz = 1 + (len / sizeof(long));
	char a_rdm[sz * sizeof(long)];
	while (sz--) {
        rdm = random_at_most(0xFFFFFFFF);
		memcpy(a_rdm + sz * sizeof(long), &rdm, sizeof(long));
	}
	memcpy(buffer, a_rdm, len);
	if (sqlite3_debug) LOGM(TAG, "K210_Randomness");
	return SQLITE_OK;
}

//-------------------------------------------------
int K210_Sleep(sqlite3_vfs * vfs, int microseconds)
{
    mp_hal_delay_us(microseconds);
	if (sqlite3_debug) LOGM(TAG, "K210_Sleep:");
	return SQLITE_OK;
}

//------------------------------------------------------
int K210_CurrentTime(sqlite3_vfs * vfs, double * result)
{
	*result = _get_time(false);
	if (sqlite3_debug) LOGM(TAG, "K210_CurrentTime: %g", *result);
	return SQLITE_OK;
}

//-----------------------
int sqlite3_os_init(void)
{
  sqlite3_vfs_register(&K210Vfs, 1);
  return SQLITE_OK;
}

//----------------------
int sqlite3_os_end(void)
{
  return SQLITE_OK;
}

#if USER_MEM_ALLOC
// ==== Memory allocation functions =====================================================================

//------------------------------
void *K210alloc_Malloc(int size)
{
    void *p = pvPortMalloc((size_t)size);
    //void *p = malloc((size_t)size);
    if (p) K210alloc_last_size = size;
    else K210alloc_last_size = 0;
    if (sqlite3_alloc_debug) LOGQ(TAG, "K210alloc_Malloc: %p [%d]", p, K210alloc_last_size);
    return p;
}

//----------------------------
void K210alloc_Free(void *ptr)
{
    if (sqlite3_alloc_debug) LOGQ(TAG, "K210alloc_Free: %p", ptr);
    vPortFree(ptr);
    //free(ptr);
}

//------------------------------------------
void *K210alloc_Realloc(void *mem, int size)
{
    if (size == 0) {
        vPortFree(mem);
        //free(mem);
        K210alloc_last_size = 0;
        if (sqlite3_alloc_debug) LOGQ(TAG, "K210alloc_Realloc: %p [0]", mem);
        return NULL;
    }

    void *p = pvPortRealloc(mem, (size_t)size);
    //void *p = realloc(mem, (size_t)size);
    if (p) K210alloc_last_size = size;
    else K210alloc_last_size = 0;
    if (sqlite3_alloc_debug) LOGQ(TAG, "K210alloc_Realloc: %p -> %p [%d]", mem, p, K210alloc_last_size);
    return p;
}

//---------------------------
int K210alloc_Size(void *ptr)
{
    if (sqlite3_alloc_debug) LOGQ(TAG, "K210alloc_Size: %d", K210alloc_last_size);
    return K210alloc_last_size;
}

//-----------------------------
int K210alloc_Roundup(int size)
{
    int xWantedSize = size;
    if ((xWantedSize & portBYTE_ALIGNMENT_MASK) != 0x00) {
        // Byte alignment required.
        xWantedSize += (portBYTE_ALIGNMENT - (xWantedSize & portBYTE_ALIGNMENT_MASK));
        configASSERT((xWantedSize & portBYTE_ALIGNMENT_MASK) == 0);
    }
    if (sqlite3_alloc_debug) LOGQ(TAG, "K210alloc_Roundup: %d->%d", size, xWantedSize);
    return xWantedSize;
}

//---------------------------
int K210alloc_Init(void *arg)
{
    if (sqlite3_alloc_debug) LOGQ(TAG, "K210alloc_init: %p", arg);
    return SQLITE_OK;
}

//--------------------------------
void K210alloc_Shutdown(void *arg)
{
    if (sqlite3_alloc_debug) LOGQ(TAG, "K210alloc_Shutdown: %p", arg);
    return;
}
#endif


void errorLogCallback(void *pArg, int iErrCode, const char *zMsg)
{
    if (sqlite3_debug) LOGY("[uSQLITE3_ERR]", "(%d) %s", iErrCode, zMsg);
}

#endif
