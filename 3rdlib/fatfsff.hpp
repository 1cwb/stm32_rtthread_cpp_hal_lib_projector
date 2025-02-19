#pragma once
#include "ff.h"

using mFResult = FRESULT;

class mFatFs
{
public:
    mFatFs(){}
    ~mFatFs(){}
    mFatFs(const mFatFs&) = delete;
    mFatFs(mFatFs&&) = delete;
    mFatFs& operator=(const mFatFs&) = delete;
    mFatFs& operator=(mFatFs&&) = delete;

    static mFResult mkdir (const TCHAR* path)								/* Create a sub directory */
    {
        return f_mkdir(path);
    }
    static mFResult unlink (const TCHAR* path)								/* Delete an existing file or directory */
    {
        return f_unlink(path);
    }
    static mFResult rename (const TCHAR* path_old, const TCHAR* path_new)	/* Rename/Move a file or directory */
    {
        return f_rename(path_old, path_new);
    }
    static mFResult stat (const TCHAR* path, FILINFO* fno)					/* Get file status */
    {
        return f_stat(path, fno);
    }
    static mFResult chmod (const TCHAR* path, BYTE attr, BYTE mask)			/* Change attribute of a file/dir */
    {
        return f_chmod(path, attr, mask);
    }
    static mFResult utime (const TCHAR* path, const FILINFO* fno)			/* Change timestamp of a file/dir */
    {
        return f_utime(path, fno);
    }
    static mFResult chdir (const TCHAR* path)								/* Change current directory */
    {
        return f_chdir(path);
    }
    static mFResult chdrive (const TCHAR* path)								/* Change current drive */
    {
        return f_chdrive(path);
    }
    static mFResult getcwd (TCHAR* buff, UINT len)							/* Get current directory */
    {
        return f_getcwd(buff, len);
    }
    static mFResult getfree (const TCHAR* path, DWORD* nclst, FATFS** fatfs)	/* Get number of free clusters on the drive */
    {
        return f_getfree(path, nclst, fatfs);
    }
    static mFResult getlabel (const TCHAR* path, TCHAR* label, DWORD* vsn)	/* Get volume label */
    {
        return f_getlabel(path, label, vsn);
    }
    static mFResult setlabel (const TCHAR* label)							/* Set volume label */
    {
        return f_setlabel(label);
    }

    mFResult mount (const TCHAR* path, BYTE opt)			/* Mount/Unmount a logical drive */
    {
        return f_mount(&fs, path, opt);
    }
    static mFResult mkfs (const TCHAR* path, const MKFS_PARM* opt, void* work, UINT len)	/* Create a FAT volume */
    {
        return f_mkfs(path, opt, work, len);
    }
    static mFResult fdisk (BYTE pdrv, const LBA_t ptbl[], void* work)		/* Divide a physical drive into some partitions */
    {
        return f_fdisk(pdrv, ptbl, work);
    }
    static mFResult setcp (WORD cp)											/* Set current code page */
    {
        return f_setcp(cp);
    }

    static mFResult rmdir(const TCHAR* path)
    {
        return f_rmdir(path);
    }
    static mFResult unmount(const TCHAR* path)
    {
        return f_unmount(path);
    }
private:
    FATFS fs;
};

class mFile
{
public:
    mFile(){}
    ~mFile(){}
    mFile(const mFile&) = delete;
    mFile(mFile&&) = delete;
    mFile& operator=(const mFile&) = delete;
    mFile& operator=(mFile&&) = delete;

    mFResult open (const TCHAR* path, BYTE mode)				/* Open or create a file */
    {
        return f_open(&fil, path, mode);
    }
    mFResult close ()											/* Close an open file object */
    {
        return f_close(&fil);
    }
    mFResult read (void* buff, UINT btr, UINT* br)			/* Read data from the file */
    {
        return f_read(&fil, buff, btr, br);
    }
    mFResult write (const void* buff, UINT btw, UINT* bw)	/* Write data to the file */
    {
        return f_write(&fil, buff, btw, bw);
    }
    mFResult lseek (FSIZE_t ofs)								/* Move file pointer of the file object */
    {
        return f_lseek(&fil, ofs);
    }
    mFResult forward (UINT(*func)(const BYTE*,UINT), UINT btf, UINT* bf)	/* Forward data to the stream */
    {
        return f_forward(&fil, func, btf, bf);
    }
    mFResult expand (FSIZE_t fsz, BYTE opt)					/* Allocate a contiguous block to the file */
    {
        return f_expand(&fil, fsz, opt);
    }
    mFResult truncate ()										/* Truncate the file */
    {
        return f_truncate(&fil);
    }
    mFResult sync ()											/* Flush cached data of the writing file */
    {
        return f_sync(&fil);
    }
    int putc (TCHAR c)										/* Put a character to the file */
    {
        return f_putc(c, &fil);
    }
    int puts (const TCHAR* str)								/* Put a string to the file */
    {
        return f_puts(str, &fil);
    }
    int printf (const TCHAR* str, ...)						/* Put a formatted string to the file */
    {
        va_list ap;
        va_start(ap, str);
        int ret = f_printf(&fil, str, ap);
        va_end(ap);
        return ret;
    }
    TCHAR* gets (TCHAR* buff, int len)						/* Get a string from the file */
    {
        return f_gets(buff, len, &fil);
    }
    mFResult rewinddir(DIR* dp)
    {
        return f_rewinddir(dp);
    }
    bool eof()
    {
        return f_eof(&fil) != 0;
    }
    int error()
    {
        return f_error(&fil);
    }
    FSIZE_t tell()
    {
        return f_tell(&fil);
    }
    FSIZE_t size()
    {
        return f_size(&fil);
    }
    mFResult rewind()
    {
        return f_rewind(&fil);
    }
private:
    FIL fil;
};

class mDir
{
public:
    mDir(){}
    ~mDir(){}
    mDir(const mDir&) = delete;
    mDir(mDir&&) = delete;
    mDir& operator=(const mDir&) = delete;
    mDir& operator=(mDir&&) = delete;
    mFResult opendir (DIR* dp, const TCHAR* path)						/* Open a directory */
    {
        return f_opendir(dp, path);
    }
    mFResult closedir (DIR* dp)										/* Close an open directory */
    {
        return f_closedir(dp);
    }
    mFResult readdir (DIR* dp, FILINFO* fno)							/* Read a directory item */
    {
        return f_readdir(dp, fno);
    }
    mFResult findfirst (DIR* dp, FILINFO* fno, const TCHAR* path, const TCHAR* pattern)	/* Find first file */
    {
        return f_findfirst(dp, fno, path, pattern);
    }
    mFResult findnext (DIR* dp, FILINFO* fno)							/* Find next file */
    {
        return f_findnext(dp, fno);
    }
private:
    DIR dir;
};