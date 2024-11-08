#include "mklog.hpp"
mKlog* mKlog::logx = nullptr;
mResult mKlog::LOGD(const char *format, ...)
{
    if(_level > LOG_LEVEL_DEBUG)
    {
        return M_RESULT_EOK;
    }
    uint32_t length = 0;
    va_list args;
    va_start(args, format);
    _sem.semTake(WAITING_FOREVER);
    length = vsnprintf((char*)_logBuff, sizeof(_logBuff), (char*)format, args);
    if(send(_logBuff, length) != M_RESULT_EOK)
    {
        _sem.semRelease();
        return M_RESULT_ERROR;
    }
    _sem.semRelease();
    return M_RESULT_EOK;
}
mResult mKlog::LOGI(const char *format, ...)
{
    if(_level > LOG_LEVEL_INFO)
    {
        return M_RESULT_EOK;
    }
    uint32_t length = 0;
    va_list args;
    va_start(args, format);
    _sem.semTake(WAITING_FOREVER);
    length = vsnprintf((char*)_logBuff, sizeof(_logBuff), (char*)format, args);
    if(send(_logBuff, length) != M_RESULT_EOK)
    {
        _sem.semRelease();
        return M_RESULT_ERROR;
    }
    _sem.semRelease();
    return M_RESULT_EOK;
}
mResult mKlog::LOGW(const char *format, ...)
{
    if(_level > LOG_LEVEL_WARNING)
    {
        return M_RESULT_EOK;
    }
    uint32_t length = 0;
    va_list args;
    va_start(args, format);
    _sem.semTake(WAITING_FOREVER);
    length = vsnprintf((char*)_logBuff, sizeof(_logBuff), (char*)format, args);
    if(send(_logBuff, length) != M_RESULT_EOK)
    {
        _sem.semRelease();
        return M_RESULT_ERROR;
    }
    _sem.semRelease();
    return M_RESULT_EOK;
}
mResult mKlog::LOGE(const char *format, ...)
{
    if(_level > LOG_LEVEL_ERROR)
    {
        return M_RESULT_EOK;
    }
    uint32_t length = 0;
    va_list args;
    va_start(args, format);
    _sem.semTake(WAITING_FOREVER);
    length = vsnprintf((char*)_logBuff, sizeof(_logBuff), (char*)format, args);
    if(send(_logBuff, length) != M_RESULT_EOK)
    {
        _sem.semRelease();
        return M_RESULT_ERROR;
    }
    _sem.semRelease();
    return M_RESULT_EOK;
}
mResult mKlog::LOGC(const char *format, ...)
{
    if(_level > LOG_LEVEL_CRITICAL)
    {
        return M_RESULT_EOK;
    }
    uint32_t length = 0;
    va_list args;
    va_start(args, format);
    _sem.semTake(WAITING_FOREVER);
    length = vsnprintf((char*)_logBuff, sizeof(_logBuff), (char*)format, args);
    if(send(_logBuff, length) != M_RESULT_EOK)
    {
        _sem.semRelease();
        return M_RESULT_ERROR;
    }
    _sem.semRelease();
    return M_RESULT_EOK;
}