#include "w25q128jvs.hpp"
#include <cstring>

W25Q128JVS::W25Q128JVS(const char* name, mDev::mSpi* spi, mDev::mGpio* cs_pin) 
    : mDev::mMspiflash(name), m_spi(spi), m_cs(cs_pin), m_initialized(false) {
}

W25Q128JVS::~W25Q128JVS() {
}

mResult W25Q128JVS::init(void) {
    if (!m_spi || !m_cs) {
        return M_RESULT_ERROR;
    }
    
    // 初始化CS引脚
    m_cs->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_HIGH);
    
    // 短暂延时确保设备稳定
    for (volatile int i = 0; i < 10000; i++);
    
    // 释放掉电模式
    releasePowerDown();
    
    // 等待设备就绪
    mResult result = waitForReady();
    if (result != M_RESULT_EOK) {
        return result;
    }
    
    // 标记为已初始化，然后读取ID
    m_initialized = true;  // 移动到readID之前
    
    // 可选：读取ID验证设备连接
    uint32_t id = readID();
    printf("W25Q128JVS ID: 0x%08lX\r\n", id);  // 修复格式警告
    
    return M_RESULT_EOK;
}

mResult W25Q128JVS::reset(void) {
    if (!m_spi || !m_cs || !m_initialized) {
        return M_RESULT_ERROR;
    }
    
    // 发送复位使能指令
    sendCommand(W25X_ResetEnable);
    
    // 短暂延时
    for (volatile int i = 0; i < 1000; i++);

    // 发送复位指令
    sendCommand(W25X_Reset);
    
    // 等待复位完成
    for (volatile int i = 0; i < 10000; i++);
    
    return waitForReady();
}

uint32_t W25Q128JVS::readID(void) {
    if (!m_spi || !m_cs || !m_initialized) {
        return 0xFFFFFFFF;
    }
    
    uint8_t tx_buf[4] = {W25X_JedecDeviceID, 0xFF, 0xFF, 0xFF};
    uint8_t rx_buf[4] = {0};
    
    // 使用transfer方法一次性发送命令并读取数据
    mResult result = m_spi->transfer(tx_buf, rx_buf, 4, m_cs);
    
    if (result != M_RESULT_EOK) {
        return 0xFFFFFFFF;
    }
    
    // 返回后3字节作为ID（跳过命令字节）
    return (rx_buf[1] << 16) | (rx_buf[2] << 8) | rx_buf[3];
}

mResult W25Q128JVS::memoryMappedMode(void) {
    // W25Q128JVS不支持传统的内存映射模式
    return M_RESULT_ERROR;
}

mResult W25Q128JVS::sectorErase(uint32_t SectorAddress) {
    if (!m_spi || !m_cs || !m_initialized) {
        return M_RESULT_ERROR;
    }
    
    // 检查地址对齐 (4KB对齐)
    if (SectorAddress & 0xFFF) {
        return M_RESULT_ERROR;
    }
    
    // 等待设备就绪
    mResult result = waitForReady();
    if (result != M_RESULT_EOK) {
        return result;
    }
    
    // 使能写操作
    result = writeEnable();
    if (result != M_RESULT_EOK) {
        return result;
    }
    
    // 发送扇区擦除指令和地址
    result = sendCommandWithAddress(W25X_SectorErase, SectorAddress);
        // 添加：等待擦除完成
    if (result == M_RESULT_EOK) {
        result = waitForReady();
    }
    return result;
}

mResult W25Q128JVS::blockErase32K(uint32_t SectorAddress) {
    if (!m_spi || !m_cs || !m_initialized) {
        return M_RESULT_ERROR;
    }
    
    // 检查地址对齐 (32KB对齐)
    if (SectorAddress & 0x7FFF) {
        return M_RESULT_ERROR;
    }
    
    // 等待设备就绪
    mResult result = waitForReady();
    if (result != M_RESULT_EOK) {
        return result;
    }
    
    // 使能写操作
    result = writeEnable();
    if (result != M_RESULT_EOK) {
        return result;
    }
    
    // 发送32K块擦除指令和地址
    result = sendCommandWithAddress(W25X_BlockErase32K, SectorAddress);
    // 添加：等待擦除完成
    if (result == M_RESULT_EOK) {
        result = waitForReady();
    }
    return result;
}

mResult W25Q128JVS::blockErase64K(uint32_t SectorAddress) {
    if (!m_spi || !m_cs || !m_initialized) {
        return M_RESULT_ERROR;
    }
    
    // 检查地址对齐 (64KB对齐)
    if (SectorAddress & 0xFFFF) {
        return M_RESULT_ERROR;
    }
    
    // 等待设备就绪
    mResult result = waitForReady();
    if (result != M_RESULT_EOK) {
        return result;
    }
    
    // 使能写操作
    result = writeEnable();
    if (result != M_RESULT_EOK) {
        return result;
    }
    
    // 发送64K块擦除指令和地址
    result = sendCommandWithAddress(W25X_BlockErase64K, SectorAddress);
        // 添加：等待擦除完成
    if (result == M_RESULT_EOK) {
        result = waitForReady();
    }
    return result;
}

mResult W25Q128JVS::chipErase(void) {
    if (!m_spi || !m_cs || !m_initialized) {
        printf("ERROR: W25Q128JVS chipErase: not initialized\r\n");
        return M_RESULT_ERROR;
    }
    
    // 等待设备就绪
    mResult result = waitForReady();
    if (result != M_RESULT_EOK) {
        printf("ERROR: W25Q128JVS chipErase: initial waitForReady failed\r\n");
        return result;
    }
    
    // 使能写操作
    result = writeEnable();
    if (result != M_RESULT_EOK) {
        printf("ERROR: W25Q128JVS chipErase: writeEnable failed\r\n");
        return result;
    }
    
    // 发送整片擦除指令
    result = sendCommand(W25X_ChipErase);
    if (result != M_RESULT_EOK) {
        printf("ERROR: W25Q128JVS chipErase: sendCommand failed\r\n");
        return result;
    }
    
    printf("W25Q128JVS chipErase: command sent, waiting for completion (may take 30-60 seconds)...\r\n");
    
    // 整片擦除需要特别长的等待时间
    const uint32_t maxChipEraseTime = 120000000;  // 120秒
    uint32_t waitCount = 0;
    uint32_t printInterval = 5000000;  // 每5秒打印一次
    
    while (isBusy()) {
        if (++waitCount > maxChipEraseTime) {
            uint8_t status = readStatusReg1();
            printf("ERROR: W25Q128JVS chipErase: timeout after %u seconds, status=0x%02X\r\n", 
                   waitCount/1000000, status);
            return M_RESULT_ETIMEOUT;
        }
        
        // 定期打印进度
        if (waitCount % printInterval == 0) {
            uint8_t status = readStatusReg1();
            printf("W25Q128JVS chipErase: erasing... %u seconds, status=0x%02X\r\n", 
                   waitCount/1000000, status);
        }
    }
    
    printf("W25Q128JVS chipErase: completed successfully in %u seconds\r\n", waitCount/1000000);
    return M_RESULT_EOK;
}

mResult W25Q128JVS::writePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
    if (!m_spi || !m_cs || !m_initialized || !pBuffer) {
        return M_RESULT_ERROR;
    }
    
    // 检查写入长度 (最大256字节)
    if (NumByteToWrite > W25Q128JV_PAGE_SIZE) {
        return M_RESULT_ERROR;
    }
    
    // 检查页对齐
    if ((WriteAddr + NumByteToWrite) > ((WriteAddr & ~(W25Q128JV_PAGE_SIZE - 1)) + W25Q128JV_PAGE_SIZE)) {
        return M_RESULT_ERROR;
    }
    
    // 等待设备就绪
    mResult result = waitForReady();
    if (result != M_RESULT_EOK) {
        return result;
    }
    
    // 使能写操作
    result = writeEnable();
    if (result != M_RESULT_EOK) {
        return result;
    }
    
    uint8_t* tx_buf = new uint8_t[4 + NumByteToWrite];
    tx_buf[0] = W25X_PageProgram;
    tx_buf[1] = (WriteAddr >> 16) & 0xFF;
    tx_buf[2] = (WriteAddr >> 8) & 0xFF;
    tx_buf[3] = WriteAddr & 0xFF;
    memcpy(&tx_buf[4], pBuffer, NumByteToWrite);
    
    result = m_spi->write(tx_buf, 4 + NumByteToWrite, m_cs);
    // 添加：等待写入完成
    if (result == M_RESULT_EOK) {
        result = waitForReady();
    }
    delete[] tx_buf;
    return result;
}

mResult W25Q128JVS::writeBuffer(uint8_t* pData, uint32_t WriteAddr, uint32_t Size) {
    if (!m_spi || !m_cs || !m_initialized || !pData) {
        return M_RESULT_ERROR;
    }
    
    uint32_t remaining = Size;
    uint32_t currentAddr = WriteAddr;
    uint8_t* currentData = pData;
    
    while (remaining > 0) {
        // 计算当前页剩余空间
        uint32_t pageOffset = currentAddr % W25Q128JV_PAGE_SIZE;
        uint16_t bytesToWrite = W25Q128JV_PAGE_SIZE - pageOffset;
        
        if (bytesToWrite > remaining) {
            bytesToWrite = remaining;
        }
        
        // 写入当前页
        mResult result = writePage(currentData, currentAddr, bytesToWrite);
        if (result != M_RESULT_EOK) {
            return result;
        }
        
        // 更新指针和剩余字节数
        currentAddr += bytesToWrite;
        currentData += bytesToWrite;
        remaining -= bytesToWrite;
    }
    
    return M_RESULT_EOK;
}

mResult W25Q128JVS::readBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead) {
    if (!m_spi || !m_cs || !m_initialized || !pBuffer) {
        return M_RESULT_ERROR;
    }
    
    // 等待设备就绪
    mResult result = waitForReady();
    if (result != M_RESULT_EOK) {
        return result;
    }
    
    // 使用transfer一次性完成
    uint8_t* tx_buf = new uint8_t[4 + NumByteToRead];
    uint8_t* rx_buf = new uint8_t[4 + NumByteToRead];
    
    tx_buf[0] = W25X_ReadData;
    tx_buf[1] = (ReadAddr >> 16) & 0xFF;
    tx_buf[2] = (ReadAddr >> 8) & 0xFF;
    tx_buf[3] = ReadAddr & 0xFF;
    
    // 填充dummy bytes
    for (uint32_t i = 4; i < 4 + NumByteToRead; i++) {
        tx_buf[i] = 0xFF;
    }
    
    result = m_spi->transfer(tx_buf, rx_buf, 4 + NumByteToRead, m_cs);
    
    if (result == M_RESULT_EOK) {
        // 复制有效数据（跳过前4个字节）
        memcpy(pBuffer, &rx_buf[4], NumByteToRead);
    }
    
    delete[] tx_buf;
    delete[] rx_buf;
    return result;
}

// 私有方法实现
mResult W25Q128JVS::writeEnable(void) {
    mResult result = sendCommand(W25X_WriteEnable);
    if (result != M_RESULT_EOK) {
        printf("ERROR: W25Q128JVS writeEnable: sendCommand failed\r\n");
        return result;
    }
    
    // 验证写使能是否成功
    uint8_t status = readStatusReg1();
    if (status & W25Q128JV_STATUS_WEL) {
        printf("W25Q128JVS writeEnable: success, status=0x%02X\r\n", status);
        return M_RESULT_EOK;
    }
    
    printf("ERROR: W25Q128JVS writeEnable: WEL bit not set, status=0x%02X\r\n", status);
    return M_RESULT_ERROR;
}

mResult W25Q128JVS::writeDisable(void) {
    return sendCommand(W25X_WriteDisable);
}

mResult W25Q128JVS::sendCommand(uint8_t cmd) {
    return m_spi->write(&cmd, 1, m_cs);
}

mResult W25Q128JVS::sendCommandWithAddress(uint8_t cmd, uint32_t addr) {
    uint8_t tx_buf[4] = {
        cmd,
        (uint8_t)((addr >> 16) & 0xFF),
        (uint8_t)((addr >> 8) & 0xFF),
        (uint8_t)(addr & 0xFF)
    };
    return m_spi->write(tx_buf, 4, m_cs);
}

bool W25Q128JVS::isBusy(void) {
    uint8_t status = readStatusReg1();
    bool busy = (status & W25Q128JV_STATUS_BUSY) != 0;
    
    // 如果状态读取失败，假设设备忙
    if (status == 0xFF) {
        printf("WARNING: W25Q128JVS isBusy: failed to read status, assuming busy\r\n");
        return true;
    }
    
    return busy;
}

mResult W25Q128JVS::waitForReady(void) {
    // 最大等待时间：60秒（整片擦除可能需要30-60秒）
    const uint32_t maxWaitCount = 60000000;  // 60秒，假设每次循环约1us
    uint32_t waitCount = 0;
    uint32_t printInterval = 1000000;  // 每1秒打印一次状态
    
    while (isBusy()) {
        if (++waitCount > maxWaitCount) {
            uint8_t status = readStatusReg1();
            printf("ERROR: W25Q128JVS waitForReady: timeout after %u ms, status=0x%02X\r\n", 
                   waitCount/1000, status);
            return M_RESULT_ETIMEOUT;
        }
        
        // 定期打印状态信息
        if (waitCount % printInterval == 0) {
            uint8_t status = readStatusReg1();
            printf("W25Q128JVS waitForReady: waiting... %u ms, status=0x%02X\r\n", 
                   waitCount/1000, status);
        }
    }
    
    if (waitCount > 0) {
        printf("W25Q128JVS waitForReady: completed after %u ms\r\n", waitCount/1000);
    }
    
    return M_RESULT_EOK;
}

uint8_t W25Q128JVS::readStatusReg1(void) {
    uint8_t status = 0;
    uint8_t tx_buf[2] = {W25X_ReadStatusReg1, 0xFF};  // 命令 + dummy byte
    uint8_t rx_buf[2] = {0};
    
    mResult result = m_spi->transfer(tx_buf, rx_buf, 2, m_cs);
    if (result != M_RESULT_EOK) {
        return 0xFF;  // 错误时返回0xFF
    }
    
    return rx_buf[1];  // 返回状态字节（跳过命令字节）
}

uint8_t W25Q128JVS::readStatusReg2(void) {
    uint8_t status = 0;
    uint8_t cmd = W25X_ReadStatusReg2;
    m_spi->readReg(cmd, &status, 1, m_cs);
    return status;
}

mResult W25Q128JVS::powerDown(void) {
    return sendCommand(W25X_PowerDown);
}

mResult W25Q128JVS::releasePowerDown(void) {
    uint8_t tx_buf[4] = {W25X_ReleasePowerDown, 0xFF, 0xFF, 0xFF};
    return m_spi->write(tx_buf, 4, m_cs);
}