#pragma once
#include "mdevice.hpp"

namespace mDev
{
class mMspiflash : public mDevice
{
public:
    mMspiflash() = delete;
    explicit mMspiflash(const char* name) : mDevice(name) {}
    virtual ~mMspiflash() {}
    virtual mResult reset(void) {return M_RESULT_EOK;}			
    virtual uint32_t readID(void){return -1;}				
    virtual mResult memoryMappedMode(void){return M_RESULT_EOK;}	
    virtual mResult sectorErase(uint32_t SectorAddress){return M_RESULT_EOK;}	
    virtual mResult blockErase32K (uint32_t SectorAddress){return M_RESULT_EOK;}
    virtual mResult blockErase64K (uint32_t SectorAddress){return M_RESULT_EOK;}
    virtual mResult chipErase (void){return M_RESULT_EOK;}                
    virtual mResult	writePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite){return M_RESULT_EOK;}
    virtual mResult	writeBuffer(uint8_t* pData, uint32_t WriteAddr, uint32_t Size){return M_RESULT_EOK;}
    virtual mResult readBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead){return M_RESULT_EOK;}
private:
};
}