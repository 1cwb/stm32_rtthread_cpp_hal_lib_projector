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
    
    // 颜色控制相关方法
    void enableColor(bool enable) { _colorEnabled = enable; }
    bool isColorEnabled() const { return _colorEnabled; }
    
    // 设置输出目标（终端或文件）
    void setOutputToFile(bool toFile) { _outputToFile = toFile; }
    bool isOutputToFile() const { return _outputToFile; }
    virtual void saveLogToFile(const std::string& fileName) = 0;
protected:
    void registerSelf(mKlog* logx) {this->logx = logx;}
    virtual mResult send(const uint8_t* data, uint32_t len) = 0;
    
    // 内部格式化方法（处理颜色）
    mResult formatAndSend(logLevel level, const char* colorCode, const char* prefix, const char* format, va_list args);
    
    // 内部颜色获取方法
    const char* getColorDebug() const { return _colorEnabled && !_outputToFile ? "\033[34m" : ""; }
    const char* getColorInfo() const { return _colorEnabled && !_outputToFile ? "\033[32m" : ""; }
    const char* getColorWarning() const { return _colorEnabled && !_outputToFile ? "\033[33m" : ""; }
    const char* getColorError() const { return _colorEnabled && !_outputToFile ? "\033[31m" : ""; }
    const char* getColorCritical() const { return _colorEnabled && !_outputToFile ? "\033[35m" : ""; }
    const char* getColorReset() const { return _colorEnabled && !_outputToFile ? "\033[0m" : ""; }
    
protected:
    mKlog() : _level(LOG_LEVEL_INFO), _colorEnabled(true), _outputToFile(false) 
    {
        _sem.init("klogsem", 1, IPC_FLAG_FIFO);
    }
    virtual ~mKlog() {_sem.detach();}
    mKlog(const mKlog&) = delete;
    mKlog(mKlog&&) = delete;
    mKlog& operator=(const mKlog&) = delete;
    mKlog& operator=(mKlog&&) = delete;
    
private:
    static constexpr uint16_t LOG_BUFF_SIZE = 512; // 增大缓冲区以容纳颜色代码
    logLevel _level;
    bool _colorEnabled;      // 颜色启用标志
    bool _outputToFile;      // 是否输出到文件
    uint8_t _logBuff[LOG_BUFF_SIZE];
    mSemaphore _sem;
    static mKlog* logx;
};

// 恢复原来的宏定义（不包含颜色处理）
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