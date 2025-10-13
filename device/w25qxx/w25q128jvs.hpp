#ifndef W25Q128JVS_H
#define W25Q128JVS_H

#include "mspiflashdrv.hpp"
#include "mspidrv.hpp"
#include "mgpiodrv.hpp"
#include <stdint.h>

// W25Q128JVS指令定义
#define W25X_WriteEnable          0x06
#define W25X_WriteDisable         0x04
#define W25X_ReadStatusReg1       0x05
#define W25X_ReadStatusReg2       0x35
#define W25X_ReadStatusReg3       0x15
#define W25X_WriteStatusReg1      0x01
#define W25X_WriteStatusReg2      0x31
#define W25X_WriteStatusReg3      0x11
#define W25X_ReadData             0x03
#define W25X_FastReadData         0x0B
#define W25X_FastReadDual         0x3B
#define W25X_PageProgram          0x02
#define W25X_SectorErase          0x20
#define W25X_BlockErase32K        0x52
#define W25X_BlockErase64K        0xD8
#define W25X_ChipErase            0xC7
#define W25X_PowerDown            0xB9
#define W25X_ReleasePowerDown     0xAB
#define W25X_DeviceID             0xAB
#define W25X_ManufactDeviceID     0x90
#define W25X_JedecDeviceID        0x9F
#define W25X_Enable4ByteAddr      0xB7
#define W25X_Exit4ByteAddr        0xE9
#define W25X_SetReadParam         0xC0
#define W25X_EnterQPIMode         0x38
#define W25X_ExitQPIMode          0xFF
#define W25X_ResetEnable          0x66
#define W25X_Reset                 0x99

// Flash容量参数
#define W25Q128JV_PAGE_SIZE        256
#define W25Q128JV_SECTOR_SIZE      4096
#define W25Q128JV_BLOCK32K_SIZE    32768
#define W25Q128JV_BLOCK64K_SIZE    65536
#define W25Q128JV_TOTAL_SIZE       16777216  // 16MB = 128Mbit

// 状态寄存器位定义
#define W25Q128JV_STATUS_BUSY     0x01
#define W25Q128JV_STATUS_WEL      0x02
#define W25Q128JV_JEDEC_ID        0x00EF4018

class W25Q128JVS : public mDev::mMspiflash {
public:
    W25Q128JVS(const char* name, mDev::mSpi* spi, mDev::mGpio* cs_pin);
    virtual ~W25Q128JVS();
    
    // 从mMspiflash继承的接口实现
    virtual mResult reset(void) override;
    virtual uint32_t readID(void) override;
    virtual mResult memoryMappedMode(void) override;
    virtual mResult sectorErase(uint32_t SectorAddress) override;
    virtual mResult blockErase32K(uint32_t SectorAddress) override;
    virtual mResult blockErase64K(uint32_t SectorAddress) override;
    virtual mResult chipErase(void) override;
    virtual mResult writePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) override;
    virtual mResult writeBuffer(uint8_t* pData, uint32_t WriteAddr, uint32_t Size) override;
    virtual mResult readBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead) override;

    // 额外实用功能
    mResult init(void);
    bool isBusy(void);
    mResult waitForReady(void);
    mResult powerDown(void);
    mResult releasePowerDown(void);
    uint8_t readStatusReg1(void);
    uint8_t readStatusReg2(void);
    
private:
    mDev::mSpi* m_spi;
    mDev::mGpio* m_cs;
    bool m_initialized;
    
    // 私有方法
    mResult writeEnable(void);
    mResult writeDisable(void);
    mResult sendCommand(uint8_t cmd);
    mResult sendCommandWithAddress(uint8_t cmd, uint32_t addr);
    mResult readData(uint8_t* data, uint32_t len);
    mResult writeData(uint8_t* data, uint32_t len);
    
    // SPI传输辅助方法
    mResult transferByte(uint8_t tx_byte, uint8_t* rx_byte = nullptr);
    mResult transferBuffer(uint8_t* tx_buf, uint8_t* rx_buf, uint32_t len);
};

#endif // W25Q128JVS_H