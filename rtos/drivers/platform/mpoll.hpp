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
#if 0
struct pollfd_t
{
    int fd;
    short events;
    short revents;
};

struct rt_poll_node;

struct rt_poll_table
{
    rt_pollreq_t req;
    rt_uint32_t triggered; /* the waited thread whether triggered */
    rt_thread_t polling_thread;
    struct rt_poll_node *nodes;
};

struct rt_poll_node
{
    struct rt_wqueue_node wqn;
    struct rt_poll_table *pt;
    struct rt_poll_node *next;
};
#endif