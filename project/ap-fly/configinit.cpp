#include "fatfsff.hpp"
#include "project.hpp"
#include "mbase.hpp"
#include "config.hpp"
#include "mklog.hpp"

int configInit(void)
{
    mConfig* pconfig = mConfig::getInstance();
    pconfig->load();
    /*std::vector<mConfig::ConfigItem> configItems = {
        {"key1", "value1"},
        {"key2", "value2"},
        {"key3", "value3"}
    };
    pconfig->save(configItems);
    pconfig->load();
    printf("key1=%s\r\n", pconfig->getValue("key1").c_str());
    printf("key2=%s\r\n", pconfig->getValue("key2").c_str());
    printf("key3=%s\r\n", pconfig->getValue("key3").c_str());
    */
    ALOGD("ACCXZERO=%s\r\n", pconfig->getValue("ACCXZERO").c_str());
    ALOGD("ACCYZERO=%s\r\n", pconfig->getValue("ACCYZERO").c_str());
    ALOGD("ACCZZERO=%s\r\n", pconfig->getValue("ACCZZERO").c_str());
    ALOGD("GYROXZERO=%s\r\n", pconfig->getValue("GYROXZERO").c_str());
    ALOGD("GYROYZERO=%s\r\n", pconfig->getValue("GYROYZERO").c_str());
    ALOGD("GYROZZERO=%s\r\n", pconfig->getValue("GYROZZERO").c_str());
    ALOGD("MAGXZERO=%s\r\n", pconfig->getValue("MAGXZERO").c_str());
    ALOGD("MAGYZERO=%s\r\n", pconfig->getValue("MAGYZERO").c_str());
    ALOGD("MAGZZERO=%s\r\n", pconfig->getValue("MAGZZERO").c_str());
    return 0;
}
INIT_EXPORT(configInit, "0.4");