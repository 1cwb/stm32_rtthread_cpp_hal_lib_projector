#include "platform.hpp"
#include "mdev.hpp"

#include <stdlib.h>
#include "mthread.h"
#include "mpoll.hpp"
#if 0
mMutex devmutex("devmtx",IPC_FLAG_FIFO);
mMutex filemutex("filemtx",IPC_FLAG_FIFO);

struct mdev_t 
{
    char* name_{nullptr};
    mdev::mDev* mdev_{nullptr};
    mdev_t(const char* name, mdev::mDev* dev) : mdev_(dev)
    {
        name_ = new char[strlen(name)+1];
        if(name_)
        {
            memcpy(name_, name, strlen(name)+1);
        }
        else
        {
            printf("Error: new sizeof name error\r\n");
        }
    }
    ~mdev_t()
    {
        if(name_)
        {
            free(name_);
            name_ = nullptr;
        }
    }
private:
    mdev_t() = default;
};

#define MAX_DRV_FD_SIZE 512

static mdev_t* devmap[MAX_DRV_FD_SIZE]{};
static mdev::file_t filemap[MAX_DRV_FD_SIZE] {};

class VFile : public mdev::mDev
{
public:
    VFile(const char* fname, mode_t mode) : mdev::mDev(fname){}
    virtual ~VFile() override = default;
    /*ssize_t write(mdev::file_t* phandle, const char* buffer, size_t buflen)override
    {
        pollNotify(POLLIN);
        return buflen;
    }*/
};

static mdev::mDev* getDev(const char* name)
{
    devmutex.mutexTake(0xffff);
    for(const auto& dev : devmap)
    {
        if(dev && (strcmp(dev->name_, name) == 0))
        {
            devmutex.mutexRelease();
            return dev->mdev_;
        }
    }
    devmutex.mutexRelease();
    return nullptr;
}

static mdev::mDev* getFile(int fd)
{
    filemutex.mutexTake(0xffff);
    mdev::mDev* dev = nullptr;

    if(fd < MAX_DRV_FD_SIZE && fd >= 0)
    {
        dev = filemap[fd].dev;
    }
    filemutex.mutexRelease();
    return dev;
}

extern "C"
{
    mResult registerDriver(const char* name, const mdev::fileOperations_t* fops, mdev::mode_t mode, void* data)
    {
        printf("mdev::register_driver %s\r\n",name);
        mResult ret = M_RESULT_ERROR;
        if(name == nullptr || data == nullptr)
        {
            return M_RESULT_EINVAL;
        }
        devmutex.mutexTake(0xffff);
        for(const auto &dev : devmap)
        {
            if(dev && (strcmp(dev->name_, name) == 0))
            {
                devmutex.mutexRelease();
                printf("the device %s already exist\r\n",name);
                return M_RESULT_EXIST;
            }
        }
        for(auto &dev : devmap)
        {
            if(dev == nullptr)
            {
                dev = new mdev_t(name, reinterpret_cast<mdev::mDev*>(data));
                printf("Register Dev %s\r\n",name);
                ret = M_RESULT_EOK;
                break;
            }
        }
        devmutex.mutexRelease();
        if(ret != M_RESULT_EOK)
        {
            printf("Error: No free devmap space, increase devmap size\r\n");
        }
        return ret;
    }
    mResult unregisterDriver(const char* name)
    {
        printf("mdev::unregister_driver %s\r\n",name);
        mResult ret = M_RESULT_ERROR;
        if(name == nullptr)
        {
            return M_RESULT_EINVAL;
        }
        devmutex.mutexTake(0xffff);
        for(auto &dev : devmap)
        {
            if(dev && (strcmp(name, dev->name_) == 0))
            {
                delete dev;
                dev = nullptr;
                ret = M_RESULT_EOK;
                break;
            }
        }
        devmutex.mutexRelease();
        return ret;
    }
    mResult mopen(const char* path, int flags, ...)
    {
        printf("%s()%d\r\n",__FUNCTION__,__LINE__);
        mdev::mDev* dev = getDev(path);
        mResult ret = M_RESULT_ERROR;
        int i;
        mode_t mode;

        if(!dev && (flags & F_WRONLY) != 0 &&
            strncmp(path, "/obj/", 5) != 0 &&
            strncmp(path, "/dev/", 5) != 0)
        {
            va_list p;
            va_start(p, flags);
            mode = va_arg(p, int);
            va_end(p);

            printf("create virtual file %s\r\n", path);
            dev = new VFile(path, mode);
            registerDriver(path, nullptr, 0666, (void*)dev);
        }

        if(dev)
        {
            filemutex.mutexTake(0xffff);
            for(i = 0; i < MAX_DRV_FD_SIZE; ++i)
            {
                if(filemap[i].dev == nullptr)
                {
                    filemap[i] = mdev::file_t(flags, dev);
                    break;
                }
            }
            filemutex.mutexRelease();
            if(i < MAX_DRV_FD_SIZE)
            {
                ret = dev->open(&filemap[i]);
            }
            else
            {
                //const unsigned NAMELEN = 32;
                //char threadName[NAMELEN] {};
                printf("%s() exceeded maximu number of file descriptors, accessing %s\r\n",__FUNCTION__, path);
                ret = M_RESULT_ENOSYS;
            }
        }
        else
        {
            ret = M_RESULT_EINVAL;
        }
        return ret;
    }

    mResult mclose(int fd)
    {
        mResult ret = M_RESULT_ERROR;
        mdev::mDev* dev = getFile(fd);
        if(dev)
        {
            filemutex.mutexTake(0xffff);
            ret = dev->close(&filemap[fd]);
            filemap[fd].dev = nullptr;
            filemutex.mutexRelease();
            printf("close fd = %d\r\n",fd);
        }
        else
        {
            ret = M_RESULT_EINVAL;
        }
        return ret;
    }
    ssize_t mread(int fd, void* buffer, size_t buflen)
    {
        int ret = -1;
        mdev::mDev* dev = getFile(fd);
        if(dev)
        {
            ret = dev->read(&filemap[fd], (char*)buffer, buflen);
        }
        return ret;
    }
    ssize_t mwrite(int fd, const void* buffer, size_t buflen)
    {
        int ret = -1;
        mdev::mDev* dev = getFile(fd);

        if(dev)
        {
            ret = dev->write(&filemap[fd], (const char*)buffer, buflen);
        }
        return ret;
    }
    mResult mioctl(int fd, int cmd, unsigned long arg)
    {
        mResult ret = M_RESULT_ERROR;
        mdev::mDev* dev = getFile(fd);
        if(dev)
        {
            ret = dev->ioctl(&filemap[fd], cmd, arg);
        }
        else
        {
            ret = M_RESULT_EINVAL;
        }
        return ret;
    }
#if 1
    mResult mpoll(struct pollfd_t* fd, unsigned int nfds, int timeout)
    {
        if(nfds == 0)
        {
            return M_RESULT_EINVAL;
        }
        int num;
        
    }
#endif
    bool maccess(const char* pathname)
    {
        mdev::mDev* dev = getDev(pathname);
        return (dev != nullptr);
    }

    void mshowFiles()
    {
        printf("Files:\r\n");
        devmutex.mutexTake(0xffff);
        for(const auto& dev : devmap)
        {
            if(dev)
            {
               printf("  %s\r\n",dev->name_);
            }
        }
        devmutex.mutexRelease();
    }
};
#endif