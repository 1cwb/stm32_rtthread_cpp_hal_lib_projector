#include "mklog.hpp"
mKlog* mKlog::logx = nullptr;

// 统一的格式化发送方法
mResult mKlog::formatAndSend(logLevel level, const char* colorCode, const char* prefix, const char* format, va_list args)
{
    if (_level > level)
    {
        return M_RESULT_EOK;
    }
    
    _sem.semTake(WAITING_FOREVER);
    
    // 格式化消息
    int length = 0;
    if (prefix) {
        // 带前缀的格式化（KLOG系列）
        length = snprintf((char*)_logBuff, sizeof(_logBuff), "%s%s", colorCode, prefix);
        if (length > 0 && length < (int)sizeof(_logBuff)) {
            length += vsnprintf((char*)_logBuff + length, sizeof(_logBuff) - length, format, args);
            if (length > 0 && length < (int)sizeof(_logBuff)) {
                snprintf((char*)_logBuff + length, sizeof(_logBuff) - length, "%s\r\n", getColorReset());
                length += strlen(getColorReset()) + 2; // +2 for "\r\n"
            }
        }
    } else {
        // 不带前缀的格式化（ALOG系列）
        length = snprintf((char*)_logBuff, sizeof(_logBuff), "%s", colorCode);
        if (length > 0 && length < (int)sizeof(_logBuff)) {
            length += vsnprintf((char*)_logBuff + length, sizeof(_logBuff) - length, format, args);
            if (length > 0 && length < (int)sizeof(_logBuff)) {
                snprintf((char*)_logBuff + length, sizeof(_logBuff) - length, "%s", getColorReset());
                length += strlen(getColorReset());
            }
        }
    }
    
    mResult result = M_RESULT_EOK;
    if (length > 0 && length < (int)sizeof(_logBuff)) {
        result = send(_logBuff, length);
    } else {
        result = M_RESULT_ERROR;
    }
    
    _sem.semRelease();
    return result;
}

mResult mKlog::LOGD(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    mResult result = formatAndSend(LOG_LEVEL_DEBUG, getColorDebug(), nullptr, format, args);
    va_end(args);
    return result;
}

mResult mKlog::LOGI(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    mResult result = formatAndSend(LOG_LEVEL_INFO, getColorInfo(), nullptr, format, args);
    va_end(args);
    return result;
}

mResult mKlog::LOGW(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    mResult result = formatAndSend(LOG_LEVEL_WARNING, getColorWarning(), nullptr, format, args);
    va_end(args);
    return result;
}

mResult mKlog::LOGE(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    mResult result = formatAndSend(LOG_LEVEL_ERROR, getColorError(), nullptr, format, args);
    va_end(args);
    return result;
}

mResult mKlog::LOGC(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    mResult result = formatAndSend(LOG_LEVEL_CRITICAL, getColorCritical(), nullptr, format, args);
    va_end(args);
    return result;
}
