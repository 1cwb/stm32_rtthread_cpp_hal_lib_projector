#pragma once
#include "mtimer.h"

#define POLLIN          (0x01)
#define POLLRDNORM      (0x01)
#define POLLRDBAND      (0x01)
#define POLLPRI         (0x01)

#define POLLOUT         (0x02)
#define POLLWRNORM      (0x02)
#define POLLWRBAND      (0x02)

#define POLLERR         (0x04)
#define POLLHUP         (0x08)
#define POLLNVAL        (0x10)

#define POLLMASK_DEFAULT (POLLIN | POLLOUT | POLLRDNORM | POLLWRNORM)

struct pollfd_t
{
    int fd;
    short events;
    short revents;
    mSemaphore_t* sem;
    void* priv;
};
