#pragma once
#include "mqspidrv.hpp"
#include "sys.h"

class Qspi : public mDev::mQspi
{
public:
    Qspi() = delete;
    explicit Qspi(const char* name) : mQspi(name){}
    virtual ~Qspi(){}
    virtual mResult sendCmd(mDev::QSPICommand* cmd) override;
    virtual mResult sendData(uint8_t *buf) override;
    virtual mResult receive(uint8_t *buf) override;
    virtual mResult autoPolling(mDev::QSPICommand* cmd, mDev::QSPIAutoPolling* poll, uint32_t timeout) override;
    mResult init(const mDev::initCallbackExt& cb ,QSPI_HandleTypeDef* qspihandle);
    mResult deInit();
    
    virtual mResult remapCmd(mDev::QSPICommand* cmd, void* dst) override;
    virtual mResult remapAutoPolling(mDev::QSPIAutoPolling* poll, void* dst) override;
    QSPI_HandleTypeDef* getQspiHandle() {return &hQspi;}
public:
    QSPI_HandleTypeDef hQspi;
private:
};