#pragma once
#include "mdevice.hpp"
#include "mgpiodrv.hpp"

namespace mDev
{
enum QSPIAddressMode
{
    E_QSPI_ADDRESS_NONE,    
    E_QSPI_ADDRESS_1_LINE,  
    E_QSPI_ADDRESS_2_LINES, 
    E_QSPI_ADDRESS_4_LINES, 
};
enum QSPIInstructionMode
{
    E_QSPI_INSTRUCTION_NONE,
    E_QSPI_INSTRUCTION_1_LINE,
    E_QSPI_INSTRUCTION_2_LINES,
    E_QSPI_INSTRUCTION_4_LINES,
};
enum QSPIAddressSize
{
    E_QSPI_ADDRESS_8_BITS,
    E_QSPI_ADDRESS_16_BITS,
    E_QSPI_ADDRESS_24_BITS,
    E_QSPI_ADDRESS_32_BITS,
};
enum QSPIAlternateBytesMode
{
    E_QSPI_ALTERNATE_BYTES_NONE,
    E_QSPI_ALTERNATE_BYTES_1_LINE,
    E_QSPI_ALTERNATE_BYTES_2_LINES,
    E_QSPI_ALTERNATE_BYTES_4_LINES,
};
enum QSPIDdrMode
{
    E_QSPI_DDR_MODE_DISABLE,
    E_QSPI_DDR_MODE_ENABLE,
};

enum QSPIDdrHoldHalfCycle
{
    E_QSPI_DDR_HHC_ANALOG_DELAY,
    E_QSPI_DDR_HHC_HALF_CLK_DELAY,
};
enum QSPISIOOMode
{
    E_QSPI_SIOO_INST_EVERY_CMD,
    E_QSPI_SIOO_INST_ONLY_FIRST_CMD,
};

enum QSPITimeOutActivation
{
    E_QSPI_TIMEOUT_COUNTER_DISABLE,
    E_QSPI_TIMEOUT_COUNTER_ENABLE, 
};
enum QSPIDataMode
{
    E_QSPI_DATA_NONE,   
    E_QSPI_DATA_1_LINE, 
    E_QSPI_DATA_2_LINES,
    E_QSPI_DATA_4_LINES,
};
enum QSPIAlternateBytesSize
{
    E_QSPI_ALTERNATE_BYTES_8_BITS, 
    E_QSPI_ALTERNATE_BYTES_16_BITS,
    E_QSPI_ALTERNATE_BYTES_24_BITS,
    E_QSPI_ALTERNATE_BYTES_32_BITS,
};

struct QSPICommand
{
    uint32_t Instruction;        /* Specifies the Instruction to be sent
                                    This parameter can be a value (8-bit) between 0x00 and 0xFF */
    uint32_t Address;            /* Specifies the Address to be sent (Size from 1 to 4 bytes according AddressSize)
                                    This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */
    uint32_t AlternateBytes;     /* Specifies the Alternate Bytes to be sent (Size from 1 to 4 bytes according AlternateBytesSize)
                                    This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */
    QSPIAddressSize AddressSize;        /* Specifies the Address Size
                                    This parameter can be a value of @ref E_QSPI_AddressSize */
    QSPIAlternateBytesSize AlternateBytesSize; /* Specifies the Alternate Bytes Size
                                    This parameter can be a value of @ref E_QSPI_AlternateBytesSize */
    uint32_t DummyCycles;        /* Specifies the Number of Dummy Cycles.
                                    This parameter can be a number between 0 and 31 */
    QSPIInstructionMode InstructionMode;    /* Specifies the Instruction Mode
                                    This parameter can be a value of @ref E_QSPI_InstructionMode */
    QSPIAddressMode AddressMode;        /* Specifies the Address Mode
                                    This parameter can be a value of @ref E_QSPI_AddressMode */
    QSPIAlternateBytesMode AlternateByteMode;  /* Specifies the Alternate Bytes Mode
                                    This parameter can be a value of @ref E_QSPI_AlternateBytesMode */
    QSPIDataMode DataMode;           /* Specifies the Data Mode (used for dummy cycles and data phases)
                                    This parameter can be a value of @ref E_QSPI_DataMode */
    uint32_t NbData;             /* Specifies the number of data to transfer. (This is the number of bytes)
                                    This parameter can be any value between 0 and 0xFFFFFFFF (0 means undefined length
                                    until end of memory)*/
    QSPIDdrMode DdrMode;            /* Specifies the double data rate mode for address, alternate byte and data phase
                                    This parameter can be a value of @ref E_QSPI_DdrMode */
    QSPIDdrHoldHalfCycle DdrHoldHalfCycle;   /* Specifies if the DDR hold is enabled. When enabled it delays the data
                                    output by one half of system clock in DDR mode.
                                    This parameter can be a value of @ref E_QSPI_DdrHoldHalfCycle */
    QSPISIOOMode SIOOMode;           /* Specifies the send instruction only once mode
                                    This parameter can be a value of @ref E_QSPI_SIOOMode */
};
enum QSPIMatchMode
{
    E_QSPI_MATCH_MODE_AND, 
    E_QSPI_MATCH_MODE_OR,
};
enum QSPIAutomaticStop
{
    E_QSPI_AUTOMATIC_STOP_DISABLE,
    E_QSPI_AUTOMATIC_STOP_ENABLE, 
};

struct QSPIAutoPolling
{
  uint32_t Match;              /* Specifies the value to be compared with the masked status register to get a match.
                                  This parameter can be any value between 0 and 0xFFFFFFFF */
  uint32_t Mask;               /* Specifies the mask to be applied to the status bytes received.
                                  This parameter can be any value between 0 and 0xFFFFFFFF */
  uint32_t Interval;           /* Specifies the number of clock cycles between two read during automatic polling phases.
                                  This parameter can be any value between 0 and 0xFFFF */
  uint32_t StatusBytesSize;    /* Specifies the size of the status bytes received.
                                  This parameter can be any value between 1 and 4 */
  QSPIMatchMode MatchMode;          /* Specifies the method used for determining a match.
                                  This parameter can be a value of @ref E_QSPI_MatchMode */
  QSPIAutomaticStop AutomaticStop;      /* Specifies if automatic polling is stopped after a match.
                                  This parameter can be a value of @ref E_QSPI_AutomaticStop */
};

struct QSPIMemoryMappedCfg
{
    uint32_t TimeOutPeriod;      /* Specifies the number of clock to wait when the FIFO is full before to release the chip select.
                                    This parameter can be any value between 0 and 0xFFFF */
    QSPITimeOutActivation TimeOutActivation;  /* Specifies if the timeout counter is enabled to release the chip select.
                                    This parameter can be a value of @ref QSPI_TimeOutActivation */
};

class mQspi : public mDevice
{
public:
    mQspi() = delete;
    explicit mQspi(const char* name) : mDevice(name), _transferMode(transferMode::TRANSFER_MODE_NOMAL),_recvMode(recvMode::RECV_MODE_NOMAL){}
    virtual ~mQspi(){}
    virtual mResult sendCmd(QSPICommand* cmd) {return M_RESULT_ERROR;}
    virtual mResult sendData(uint8_t *buf) {return M_RESULT_ERROR;}
    virtual mResult receive(uint8_t *buf) {return M_RESULT_ERROR;}
    virtual mResult autoPolling(QSPICommand* cmd, QSPIAutoPolling* poll, uint32_t timeout) {return M_RESULT_ERROR;}
    inline virtual void csEnable(mDev::mGpio* cspin){}
    inline virtual void csDisable(mDev::mGpio* cspin){}
    virtual mResult memoryMapped(QSPICommand* cmd, QSPIMemoryMappedCfg* cfg){return M_RESULT_EOK;}
    void setTransferMode(transferMode mode) {_transferMode = mode;}
    void setRecvMode(recvMode mode) {_recvMode = mode;}
    recvMode getRecvMode() const {return _recvMode;}
protected:
    virtual mResult remapCmd(QSPICommand* cmd, void* dst) = 0;
    virtual mResult remapAutoPolling(QSPIAutoPolling* poll, void* dst) = 0;
    virtual mResult remapMemMapCfg(QSPIMemoryMappedCfg* cfg, void* dst) = 0;

protected:
    transferMode _transferMode;
    recvMode _recvMode;
};
}