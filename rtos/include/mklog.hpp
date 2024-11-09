#pragma once
#include <cstdarg>
#include "mipc.hpp"

enum logLevel
{
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_CRITICAL
};

class mKlog
{
public:
    static mKlog* getInstance()
    {
        return logx;
    }
    mResult LOGD(const char *format, ...);
    mResult LOGI(const char *format, ...);
    mResult LOGW(const char *format, ...);
    mResult LOGE(const char *format, ...);
    mResult LOGC(const char *format, ...);
    void setLevel(logLevel level) {this->_level = level;}
protected:
    void registerSelf(mKlog* logx) {this->logx = logx;}
    virtual mResult send(const uint8_t* data, uint32_t len) {return M_RESULT_EOK;}
protected:
    mKlog() : _level(LOG_LEVEL_INFO) {_sem.init("klogsem", 1, IPC_FLAG_FIFO);}
    virtual ~mKlog() {_sem.detach();}
    mKlog(const mKlog&) = delete;
    mKlog(mKlog&&) = delete;
    mKlog& operator=(const mKlog&) = delete;
    mKlog& operator=(mKlog&&) = delete;
private:
    static constexpr uint16_t LOG_BUFF_SIZE = 256;
    logLevel _level;
    uint8_t _logBuff[LOG_BUFF_SIZE];
    mSemaphore _sem;
    static mKlog* logx;
};

#define KLOGD(fmt, ...) \
{   \
    if(mKlog::getInstance())    \
    {   \
        mKlog::getInstance()->LOGD("D:%s()[%d] " fmt "\r\n",__FUNCTION__,__LINE__, ##__VA_ARGS__);  \
    }   \
}
#define KLOGI(fmt, ...) \
{   \
    if(mKlog::getInstance())    \
    {   \
        mKlog::getInstance()->LOGI("I:%s()[%d] " fmt "\r\n",__FUNCTION__,__LINE__, ##__VA_ARGS__);  \
    }   \
}
#define KLOGW(fmt, ...) \
{   \
    if(mKlog::getInstance())    \
    {   \
        mKlog::getInstance()->LOGW("W:%s()[%d] " fmt "\r\n",__FUNCTION__,__LINE__, ##__VA_ARGS__);  \
    }   \
}
#define KLOGE(fmt, ...) \
{   \
    if(mKlog::getInstance())    \
    {   \
        mKlog::getInstance()->LOGE("E:%s()[%d] " fmt "\r\n",__FUNCTION__,__LINE__, ##__VA_ARGS__);  \
    }   \
}
#define KLOGC(fmt, ...) \
{   \
    if(mKlog::getInstance())    \
    {   \
        mKlog::getInstance()->LOGC("C:%s()[%d] " fmt "\r\n",__FUNCTION__,__LINE__, ##__VA_ARGS__);  \
    }   \
}

#define ALOGD(fmt, ...) \
{   \
    if(mKlog::getInstance())    \
    {   \
        mKlog::getInstance()->LOGD(fmt,##__VA_ARGS__);  \
    }   \
}
#define ALOGI(fmt, ...) \
{   \
    if(mKlog::getInstance())    \
    {   \
        mKlog::getInstance()->LOGI(fmt,##__VA_ARGS__);  \
    }   \
}
#define ALOGW(fmt, ...) \
{   \
    if(mKlog::getInstance())    \
    {   \
        mKlog::getInstance()->LOGW(fmt,##__VA_ARGS__);  \
    }   \
}
#define ALOGE(fmt, ...) \
{   \
    if(mKlog::getInstance())    \
    {   \
        mKlog::getInstance()->LOGE(fmt,##__VA_ARGS__);  \
    }   \
}
#define ALOGC(fmt, ...) \
{   \
    if(mKlog::getInstance())    \
    {   \
        mKlog::getInstance()->LOGC(fmt,##__VA_ARGS__);  \
    }   \
}

#ifndef MASSERT
#define MASSERT(x) \
if(!(x)) \
{        \
    KLOGE("Error happend");\
}
#endif