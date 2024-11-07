#pragma once
#include "mdevice.hpp"
#include "mipc.hpp"
#include <cstdarg>

namespace mDev
{
enum logLevel
{
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_CRITICAL
};

class mLog : public mDevice
{
public:
    mLog(const char* name) : mDevice(name),_level(LOG_LEVEL_INFO) {_sem.init(name, 1, IPC_FLAG_FIFO);}
    virtual ~mLog() {_sem.detach();}
    mResult LOGD(const char *format, ...)
    {
        if(_level > LOG_LEVEL_DEBUG)
        {
            return M_RESULT_EOK;
        }
	    uint32_t length = 0;
	    va_list args;
	    va_start(args, format);
        _sem.semTake(WAITING_FOREVER);
        syncDataByAddr((uint32_t*)_logBuff, LOG_BUFF_SIZE_UINT32);
	    length = vsnprintf((char*)_logBuff, sizeof(_logBuff), (char*)format, args);
        if(send(_logBuff, length) != M_RESULT_EOK)
        {
            _sem.semRelease();
            return M_RESULT_ERROR;
        }
        _sem.semRelease();
        return M_RESULT_EOK;
    }
    mResult LOGI(const char *format, ...)
    {
        if(_level > LOG_LEVEL_INFO)
        {
            return M_RESULT_EOK;
        }
	    uint32_t length = 0;
	    va_list args;
	    va_start(args, format);
        _sem.semTake(WAITING_FOREVER);
        syncDataByAddr((uint32_t*)_logBuff, LOG_BUFF_SIZE_UINT32);
	    length = vsnprintf((char*)_logBuff, sizeof(_logBuff), (char*)format, args);
        if(send(_logBuff, length) != M_RESULT_EOK)
        {
            _sem.semRelease();
            return M_RESULT_ERROR;
        }
        _sem.semRelease();
        return M_RESULT_EOK;
    }
    mResult LOGW(const char *format, ...)
    {
        if(_level > LOG_LEVEL_WARNING)
        {
            return M_RESULT_EOK;
        }
	    uint32_t length = 0;
	    va_list args;
	    va_start(args, format);
        _sem.semTake(WAITING_FOREVER);
        syncDataByAddr((uint32_t*)_logBuff, LOG_BUFF_SIZE_UINT32);
	    length = vsnprintf((char*)_logBuff, sizeof(_logBuff), (char*)format, args);
        if(send(_logBuff, length) != M_RESULT_EOK)
        {
            _sem.semRelease();
            return M_RESULT_ERROR;
        }
        _sem.semRelease();
        return M_RESULT_EOK;
    }
    mResult LOGE(const char *format, ...)
    {
        if(_level > LOG_LEVEL_ERROR)
        {
            return M_RESULT_EOK;
        }
	    uint32_t length = 0;
	    va_list args;
	    va_start(args, format);
        _sem.semTake(WAITING_FOREVER);
        syncDataByAddr((uint32_t*)_logBuff, LOG_BUFF_SIZE_UINT32);
	    length = vsnprintf((char*)_logBuff, sizeof(_logBuff), (char*)format, args);
        if(send(_logBuff, length) != M_RESULT_EOK)
        {
            _sem.semRelease();
            return M_RESULT_ERROR;
        }
        _sem.semRelease();
        return M_RESULT_EOK;
    }
    mResult LOGC(const char *format, ...)
    {
        if(_level > LOG_LEVEL_CRITICAL)
        {
            return M_RESULT_EOK;
        }
	    uint32_t length = 0;
	    va_list args;
	    va_start(args, format);
        _sem.semTake(WAITING_FOREVER);
        syncDataByAddr((uint32_t*)_logBuff, LOG_BUFF_SIZE_UINT32);
	    length = vsnprintf((char*)_logBuff, sizeof(_logBuff), (char*)format, args);
        if(send(_logBuff, length) != M_RESULT_EOK)
        {
            _sem.semRelease();
            return M_RESULT_ERROR;
        }
        _sem.semRelease();
        return M_RESULT_EOK;
    }
    void setLevel(logLevel level) {this->_level = level;}
protected:
    virtual mResult send(const uint8_t* data, uint32_t len) {return M_RESULT_EOK;}
    virtual void syncDataByAddr(uint32_t* addr, uint32_t len) {}
private:
    static constexpr uint16_t LOG_BUFF_SIZE = 128;
    static constexpr uint16_t LOG_BUFF_SIZE_UINT32 = 128/4;
    logLevel _level;
    uint8_t _logBuff[LOG_BUFF_SIZE];
    mSemaphore _sem;
};
}