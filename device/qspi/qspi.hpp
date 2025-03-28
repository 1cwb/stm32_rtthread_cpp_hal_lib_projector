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
    virtual mResult sendCmd(mDev::QSPICommand* cmd) override;
    virtual mResult sendData(uint8_t *buf) override;
    virtual mResult receive(uint8_t *buf) override;
    virtual mResult autoPolling(mDev::QSPICommand* cmd, mDev::QSPIAutoPolling* poll, uint32_t timeout) override;
    virtual mResult remapCmd(mDev::QSPICommand* cmd, void* dst) override;
    virtual mResult remapAutoPolling(mDev::QSPIAutoPolling* poll, void* dst) override;
    virtual mResult remapMemMapCfg(mDev::QSPIMemoryMappedCfg* cfg, void* dst) override;
    virtual mResult memoryMapped(mDev::QSPICommand* cmd, mDev::QSPIMemoryMappedCfg* cfg) override;
    inline virtual void csEnable(mDev::mGpio* cspin)override;
    inline virtual void csDisable(mDev::mGpio* cspin)override;
    QSPI_HandleTypeDef* getQspiHandle() {return &hQspi;}
public:
    QSPI_HandleTypeDef hQspi;
private:
};