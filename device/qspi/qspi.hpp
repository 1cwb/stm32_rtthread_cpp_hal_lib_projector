#pragma once
#include "mqspidrv.hpp"
#include "sys.h"

class Qspi : public mDev::mQspi
{
public:
    Qspi() = delete;
    explicit Qspi(const char* name) : mQspi(name){}
    virtual ~Qspi(){}
    mResult init(const mDev::initCallbackExt& cb ,QSPI_HandleTypeDef* qspihandle);
    mResult deInit();
    inline virtual void csEnable(mDev::mGpio* cspin)override;
    inline virtual void csDisable(mDev::mGpio* cspin)override;
    QSPI_HandleTypeDef* getQspiHandle() {return &hQspi;}
protected:
    virtual mResult _sendCmd(mDev::QSPICommand* cmd) override;
    virtual mResult _sendData(uint8_t *buf) override;
    virtual mResult _receive(uint8_t *buf) override;
    virtual mResult _autoPolling(mDev::QSPICommand* cmd, mDev::QSPIAutoPolling* poll, uint32_t timeout) override;
    virtual mResult _remapCmd(mDev::QSPICommand* cmd, void* dst) override;
    virtual mResult _remapAutoPolling(mDev::QSPIAutoPolling* poll, void* dst) override;
    virtual mResult _remapMemMapCfg(mDev::QSPIMemoryMappedCfg* cfg, void* dst) override;
    virtual mResult _memoryMapped(mDev::QSPICommand* cmd, mDev::QSPIMemoryMappedCfg* cfg) override;
public:
    QSPI_HandleTypeDef hQspi;
private:
};