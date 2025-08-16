#include "qspi.hpp"
#include "mklog.hpp"

mResult Qspi::init(const mDev::initCallbackExt& cb ,QSPI_HandleTypeDef* qspihandle)
{
    _initcb = cb;
    if(qspihandle)
    {
        memcpy(&hQspi, qspihandle, sizeof(QSPI_HandleTypeDef));
        HAL_QSPI_DeInit(&hQspi);
    }
    if(HAL_QSPI_Init(&hQspi) != HAL_OK)
    {
        ALOGE("%s()%d HAL_QSPI_Init Fail\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult Qspi::deInit()
{
    return M_RESULT_EOK;
}

mResult Qspi::_sendCmd(mDev::QSPICommand* cmd)
{
    QSPI_CommandTypeDef dst;
    if(_remapCmd(cmd, &dst) != M_RESULT_EOK)
    {
        ALOGE("%s()%d remapCmd Fail\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    if(_transferMode == mDev::transferMode::TRANSFER_MODE_NOMAL)
    {
        if (HAL_QSPI_Command(&hQspi, &dst, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) 
        {
            ALOGE("%s()%d HAL_QSPI_Command Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if(_transferMode == mDev::transferMode::TRANSFER_MODE_IT)
    {
        if (HAL_QSPI_Command_IT(&hQspi, &dst)!= HAL_OK)
        {
            ALOGE("%s()%d HAL_QSPI_Command_IT Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if(_transferMode == mDev::transferMode::TRANSFER_MODE_DMA)
    {
        if (HAL_QSPI_Command_IT(&hQspi, &dst)!= HAL_OK)
        {
            ALOGE("%s()%d HAL_QSPI_Command_IT Fail\r\n",__FUNCTION__,__LINE__);
            return M_RESULT_ERROR;
        }
    }
    else
    {
        ALOGE("%s()%d Invalid transfer mode\r\n", __FUNCTION__, __LINE__);
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult Qspi::_sendData(uint8_t *buf)
{
    if (!buf)
    {
        ALOGE("%s()%d Invalid parameters\r\n", __FUNCTION__, __LINE__);
        return M_RESULT_ERROR;
    }

    if (_transferMode == mDev::transferMode::TRANSFER_MODE_NOMAL)
    {
        if (HAL_QSPI_Transmit(&hQspi, buf, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        {
            ALOGE("%s()%d HAL_QSPI_Transmit Fail\r\n", __FUNCTION__, __LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if (_transferMode == mDev::transferMode::TRANSFER_MODE_IT)
    {
        if (HAL_QSPI_Transmit_IT(&hQspi, buf) != HAL_OK)
        {
            ALOGE("%s()%d HAL_QSPI_Transmit_IT Fail\r\n", __FUNCTION__, __LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if (_transferMode == mDev::transferMode::TRANSFER_MODE_DMA)
    {
        if (HAL_QSPI_Transmit_DMA(&hQspi, buf) != HAL_OK)
        {
            ALOGE("%s()%d HAL_QSPI_Transmit_DMA Fail\r\n", __FUNCTION__, __LINE__);
            return M_RESULT_ERROR;
        }
    }
    else
    {
        ALOGE("%s()%d Invalid transfer mode\r\n", __FUNCTION__, __LINE__);
        return M_RESULT_ERROR;
    }

    return M_RESULT_EOK;
}

mResult Qspi::_receive(uint8_t *buf)
{
    if (!buf)
    {
        ALOGE("%s()%d Invalid parameters\r\n", __FUNCTION__, __LINE__);
        return M_RESULT_ERROR;
    }

    if (_recvMode == mDev::recvMode::RECV_MODE_NOMAL)
    {
        if (HAL_QSPI_Receive(&hQspi, buf, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        {
            ALOGE("%s()%d HAL_QSPI_Receive Fail\r\n", __FUNCTION__, __LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_IT)
    {
        if (HAL_QSPI_Receive_IT(&hQspi, buf) != HAL_OK)
        {
            ALOGE("%s()%d HAL_QSPI_Receive_IT Fail\r\n", __FUNCTION__, __LINE__);
            return M_RESULT_ERROR;
        }
    }
    else if (_recvMode == mDev::recvMode::RECV_MODE_DMA)
    {
        if (HAL_QSPI_Receive_DMA(&hQspi, buf) != HAL_OK)
        {
            ALOGE("%s()%d HAL_QSPI_Receive_DMA Fail\r\n", __FUNCTION__, __LINE__);
            return M_RESULT_ERROR;
        }
    }
    else
    {
        ALOGE("%s()%d Invalid transfer mode\r\n", __FUNCTION__, __LINE__);
        return M_RESULT_ERROR;
    }

    return M_RESULT_EOK;
}
mResult Qspi::_autoPolling(mDev::QSPICommand* cmd, mDev::QSPIAutoPolling* poll, uint32_t timeout)
{
    QSPI_CommandTypeDef dst;
    QSPI_AutoPollingTypeDef polldst;
    if(_remapCmd(cmd, &dst) != M_RESULT_EOK)
    {
        ALOGE("%s()%d remapCmd Fail\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    if(_remapAutoPolling(poll, &polldst)!= M_RESULT_EOK)
    {
        ALOGE("%s()%d _remapAutoPolling Fail\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    if(_transferMode == mDev::transferMode::TRANSFER_MODE_NOMAL && _recvMode == mDev::recvMode::RECV_MODE_NOMAL)
    {
        if (HAL_QSPI_AutoPolling(&hQspi, &dst, &polldst, timeout) != HAL_OK)
        {
            return M_RESULT_ETIMEOUT; // 轮询等待无响应
        }
    }
    else
    {
        if (HAL_QSPI_AutoPolling_IT(&hQspi, &dst, &polldst) != HAL_OK)
        {
            return M_RESULT_ETIMEOUT; // 轮询等待无响应
        }
    }
    return M_RESULT_EOK;
}

mResult Qspi::_remapCmd(mDev::QSPICommand* cmd, void* dst)
{
    if(!dst || !cmd)
        return M_RESULT_ERROR;

    QSPI_CommandTypeDef *p = (QSPI_CommandTypeDef*)dst;

    // 指令模式
    switch (cmd->InstructionMode)
    {
    case mDev::E_QSPI_INSTRUCTION_NONE:
        p->InstructionMode = QSPI_INSTRUCTION_NONE;
        break;
    case mDev::E_QSPI_INSTRUCTION_1_LINE:
        p->InstructionMode = QSPI_INSTRUCTION_1_LINE;
        break;
    case mDev::E_QSPI_INSTRUCTION_2_LINES:
        p->InstructionMode = QSPI_INSTRUCTION_2_LINES;
        break;
    case mDev::E_QSPI_INSTRUCTION_4_LINES:
        p->InstructionMode = QSPI_INSTRUCTION_4_LINES;
        break;
    default:
        p->InstructionMode = QSPI_INSTRUCTION_NONE;
        break;
    }

    // 指令
    p->Instruction = cmd->Instruction;

    // 地址模式
    switch (cmd->AddressMode)
    {
    case mDev::E_QSPI_ADDRESS_NONE:
        p->AddressMode = QSPI_ADDRESS_NONE;
        break;
    case mDev::E_QSPI_ADDRESS_1_LINE:
        p->AddressMode = QSPI_ADDRESS_1_LINE;
        break;
    case mDev::E_QSPI_ADDRESS_2_LINES:
        p->AddressMode = QSPI_ADDRESS_2_LINES;
        break;
    case mDev::E_QSPI_ADDRESS_4_LINES:
        p->AddressMode = QSPI_ADDRESS_4_LINES;
        break;
    default:
        p->AddressMode = QSPI_ADDRESS_NONE;
        break;
    }

    // 地址大小
    switch (cmd->AddressSize)
    {
    case mDev::E_QSPI_ADDRESS_8_BITS:
        p->AddressSize = QSPI_ADDRESS_8_BITS;
        break;
    case mDev::E_QSPI_ADDRESS_16_BITS:
        p->AddressSize = QSPI_ADDRESS_16_BITS;
        break;
    case mDev::E_QSPI_ADDRESS_24_BITS:
        p->AddressSize = QSPI_ADDRESS_24_BITS;
        break;
    case mDev::E_QSPI_ADDRESS_32_BITS:
        p->AddressSize = QSPI_ADDRESS_32_BITS;
        break;
    default:
        p->AddressSize = QSPI_ADDRESS_8_BITS;
        break;
    }

    // 地址
    p->Address = cmd->Address;

    // 替代字节模式
    switch (cmd->AlternateByteMode)
    {
    case mDev::E_QSPI_ALTERNATE_BYTES_NONE:
        p->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        break;
    case mDev::E_QSPI_ALTERNATE_BYTES_1_LINE:
        p->AlternateByteMode = QSPI_ALTERNATE_BYTES_1_LINE;
        break;
    case mDev::E_QSPI_ALTERNATE_BYTES_2_LINES:
        p->AlternateByteMode = QSPI_ALTERNATE_BYTES_2_LINES;
        break;
    case mDev::E_QSPI_ALTERNATE_BYTES_4_LINES:
        p->AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
        break;
    default:
        p->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
        break;
    }

    // 替代字节大小
    switch (cmd->AlternateBytesSize)
    {
    case mDev::E_QSPI_ALTERNATE_BYTES_8_BITS:
        p->AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
        break;
    case mDev::E_QSPI_ALTERNATE_BYTES_16_BITS:
        p->AlternateBytesSize = QSPI_ALTERNATE_BYTES_16_BITS;
        break;
    case mDev::E_QSPI_ALTERNATE_BYTES_24_BITS:
        p->AlternateBytesSize = QSPI_ALTERNATE_BYTES_24_BITS;
        break;
    case mDev::E_QSPI_ALTERNATE_BYTES_32_BITS:
        p->AlternateBytesSize = QSPI_ALTERNATE_BYTES_32_BITS;
        break;
    default:
        p->AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
        break;
    }

    // 替代字节
    p->AlternateBytes = cmd->AlternateBytes;

    // 数据模式
    switch (cmd->DataMode)
    {
    case mDev::E_QSPI_DATA_NONE:
        p->DataMode = QSPI_DATA_NONE;
        break;
    case mDev::E_QSPI_DATA_1_LINE:
        p->DataMode = QSPI_DATA_1_LINE;
        break;
    case mDev::E_QSPI_DATA_2_LINES:
        p->DataMode = QSPI_DATA_2_LINES;
        break;
    case mDev::E_QSPI_DATA_4_LINES:
        p->DataMode = QSPI_DATA_4_LINES;
        break;
    default:
        p->DataMode = QSPI_DATA_NONE;
        break;
    }

    // 数据长度
    p->NbData = cmd->NbData;

    // 空指令周期数
    p->DummyCycles = cmd->DummyCycles;

    // 双倍数据速率模式
    p->DdrMode = (cmd->DdrMode == mDev::E_QSPI_DDR_MODE_DISABLE) ? QSPI_DDR_MODE_DISABLE : QSPI_DDR_MODE_ENABLE;

    // 双倍数据速率保持
    p->DdrHoldHalfCycle = (cmd->DdrHoldHalfCycle == mDev::E_QSPI_DDR_HHC_ANALOG_DELAY) ? QSPI_DDR_HHC_ANALOG_DELAY : QSPI_DDR_HHC_HALF_CLK_DELAY;

    // SIO采样延迟
    p->SIOOMode = (cmd->SIOOMode == mDev::E_QSPI_SIOO_INST_EVERY_CMD) ? QSPI_SIOO_INST_EVERY_CMD : QSPI_SIOO_INST_ONLY_FIRST_CMD;

    return M_RESULT_EOK;
}
mResult Qspi::_remapMemMapCfg(mDev::QSPIMemoryMappedCfg* cfg, void* dst)
{
    if (!cfg || !dst)
    {
        ALOGE("%s()%d Invalid parameters\r\n", __FUNCTION__, __LINE__);
        return M_RESULT_ERROR;
    }

    QSPI_MemoryMappedTypeDef* p = (QSPI_MemoryMappedTypeDef*)dst;

    // 超时计数器时钟预分频
    switch (cfg->TimeOutActivation)
    {
    case mDev::E_QSPI_TIMEOUT_COUNTER_DISABLE:
        p->TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
        break;
    case mDev::E_QSPI_TIMEOUT_COUNTER_ENABLE:
        p->TimeOutActivation = QSPI_TIMEOUT_COUNTER_ENABLE;
        break;
    default:
        ALOGE("%s()%d Invalid TimeOutActivation\r\n", __FUNCTION__, __LINE__);
        return M_RESULT_ERROR;
    }

    // 超时周期
    p->TimeOutPeriod = cfg->TimeOutPeriod;

    return M_RESULT_EOK;
}
mResult Qspi::_memoryMapped(mDev::QSPICommand* cmd, mDev::QSPIMemoryMappedCfg* cfg)
{
    if(!cmd ||!cfg)
        return M_RESULT_ERROR;

    QSPI_CommandTypeDef dst;
    if(_remapCmd(cmd, &dst) != M_RESULT_EOK)
    {
        ALOGE("%s()%d remapCmd Fail\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    QSPI_MemoryMappedTypeDef hlcfg;
    if(_remapMemMapCfg(cfg, &hlcfg)!= M_RESULT_EOK)
    {
        ALOGE("%s()%d remapMemMapCfg Fail\r\n",__FUNCTION__,__LINE__);
        return M_RESULT_ERROR;
    }
    if(HAL_QSPI_MemoryMapped(&hQspi, (QSPI_CommandTypeDef*)&dst, (QSPI_MemoryMappedTypeDef *)&hlcfg)!=HAL_OK)
    {
        return M_RESULT_ERROR;
    }
    return M_RESULT_EOK;
}
mResult Qspi::_remapAutoPolling(mDev::QSPIAutoPolling* poll, void* dst)
{
    if(!poll || !dst)
        return M_RESULT_ERROR;

    QSPI_AutoPollingTypeDef *p = (QSPI_AutoPollingTypeDef*)dst;

    // 匹配模式
    switch (poll->MatchMode)
    {
    case mDev::E_QSPI_MATCH_MODE_AND:
        p->MatchMode = QSPI_MATCH_MODE_AND;
        break;
    case mDev::E_QSPI_MATCH_MODE_OR:
        p->MatchMode = QSPI_MATCH_MODE_OR;
        break;
    default:
        return M_RESULT_ERROR;
    }

    // 自动停止模式
    switch (poll->AutomaticStop)
    {
    case mDev::E_QSPI_AUTOMATIC_STOP_DISABLE:
        p->AutomaticStop = QSPI_AUTOMATIC_STOP_DISABLE;
        break;
    case mDev::E_QSPI_AUTOMATIC_STOP_ENABLE:
        p->AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;
        break;
    default:
        p->AutomaticStop = QSPI_AUTOMATIC_STOP_DISABLE;
        break;
    }

    // 状态掩码
    p->Mask = poll->Mask;

    // 匹配值
    p->Match = poll->Match;

    // 轮询间隔
    p->Interval = poll->Interval;

    // 状态字节数
    p->StatusBytesSize = poll->StatusBytesSize;

    return M_RESULT_EOK;
}

void Qspi::csEnable(mDev::mGpio* cspin)
{
    if(cspin)
    {
        cspin->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
    }
    else
    {
        KLOGE("Error: %s()%d CS pin not set\r\n",__FUNCTION__,__LINE__);
    }
}
void Qspi::csDisable(mDev::mGpio* cspin)
{
    if(cspin)
    {
        cspin->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_HIGH);
    }
    else
    {
        KLOGE("Error: %s()%d CS pin not set\r\n",__FUNCTION__,__LINE__);
    }
}

extern "C" void HAL_QSPI_MspInit(QSPI_HandleTypeDef* hqspi)
{
    Qspi* qspi = Qspi::GetObjectFromPrivateMember(hqspi);
    if(hqspi == qspi->getQspiHandle())
    {
        qspi->runInitCallback(true);
    }
}

extern "C" void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef *hqspi)
{
    Qspi* qspi = Qspi::GetObjectFromPrivateMember(hqspi);
    if(hqspi == qspi->getQspiHandle())
    {
        qspi->runInitCallback(false);
    }
}

