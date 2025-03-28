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
    virtual mResult reset(void);					
    virtual uint32_t readID(void);					
    virtual mResult memoryMappedMode(void);		
    virtual mResult sectorErase(uint32_t SectorAddress);		
    virtual mResult blockErase32K (uint32_t SectorAddress);	
    virtual mResult blockErase64K (uint32_t SectorAddress);	
    virtual mResult chipErase (void);                        
    virtual mResult	writePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);	
    virtual mResult	writeBuffer(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);				
    virtual mResult readBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
private:
};
}