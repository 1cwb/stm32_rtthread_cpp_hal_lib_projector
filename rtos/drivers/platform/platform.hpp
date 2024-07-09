#pragma once
#include <inttypes.h>
#include <cstring>

#include "mipc.h"
#include "rtoscommon.h"
#include "containers.hpp"

#define  F_RDONLY 1
#define  F_WRONLY 2

namespace mdev
{
    struct fileOperations
    {
        void* op;
    };
    using fileOperations_t = struct fileOperations;
    using mode_t = uint32_t;
    class mDev;

    struct file_t
    {
        int  flags{0};
        void *fPriv{nullptr};
        mDev* dev{nullptr};

        file_t() = default;
        file_t(int f, mDev *c) : flags(f), dev(c) {}
    };
};

extern "C" {
mResult registerDriver(const char* name, const mdev::fileOperations_t* fops, mdev::mode_t mode, void* data);
mResult unregisterDriver(const char* name);
mResult mopen(const char* path, int flags, int* outfd, ...);
mResult mclose(int fd);
ssize_t mread(int fd, void* buffer, size_t buflen);
ssize_t mread(int fd, void* buffer, size_t buflen);
ssize_t mwrite(int fd, const void* buffer, size_t buflen);
mResult mioctl(int fd, int cmd, unsigned long arg);
mResult mpoll(struct pollfd_t* fds, unsigned int nfds, int timeout, int* pollCount);
bool maccess(const char* pathname);
void mshowFiles();
}