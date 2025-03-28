/****
	***********************************************************************************************************************************************************************************
	*	@file  	qspi_w25q64.c
	*	@version V1.0
	*  @date    2022-7-12
	*	@author  反客科技
	*	@brief   QSPI驱动W25Qxx相关函数，提供的读写函数均使用HAL库函数直接操作
   ************************************************************************************************************************************************************************************
   *  @description
	*
	*	实验平台：反客STM32H750XBH6核心板 （型号：FK750M5-XBH6）
	*	淘宝地址：https://shop212360197.taobao.com
	*	QQ交流群：536665479
	*
>>>>> 文件说明：
	*
	*  1.例程参考于官方驱动文件 stm32h743i_eval_qspi.c
	*	2.例程使用的是 QUADSPI_BK1
	*	3.提供的读写函数均使用HAL库函数直接操作，没有用到DMA和中断
	*	4.默认配置QSPI驱动时钟为120M
	*
>>>>> 重要说明：
	*
	*	1.W25QXX的擦除时间是限定的!!! 手册给出的典型参考值为: 4K-45ms, 32K-120ms ,64K-150ms,整片擦除20S
	*
	*	2.W25QXX的写入时间是限定的!!! 手册给出的典型参考值为: 256字节-0.4ms，也就是 1M字节/s （实测大概在600K字节/s左右）
	*
	*	3.如果使用库函数直接读取，那么是否使用DMA、是否开启Cache、编译器的优化等级以及数据存储区的位置(内部 TCM SRAM 或者 AXI SRAM)都会影响读取的速度
	*
	*	4.如果使用内存映射模式，则读取性能只与QSPI的驱动时钟以及是否开启Cache有关
	*
	*	5.使用库函数进行直接读取，keil版本5.30，编译器AC6.14，编译等级Oz image size，读取速度为 7M字节/S ，数据放在TCM SRAM 或者 AXI SRAM  
	*    都是差不多的结果，因为CPU直接访问外设寄存器的效率很低，直接使用HAL库进行读取的话，速度很慢
	*
	*	6.如果使用MDMA进行读取，可以达到 58M字节/S，使用内存映射模式的话，几乎可以达到驱动时钟的全速，62.14M/s (133MHz时钟下)
	*
	*  7.W25Q64JV 所允许的最高驱动频率为133MHz，750的QSPI最高驱动频率也是133MHz ，但是对于HAL库函数直接读取而言，驱动时钟超过15M已经不会有性能提升
	*
	*	8.对于内存映射模式直接读取而言，驱动时钟超过127.5M已经不会有性能提升，因为QSPI内核时钟最高限定为250M，所以建议实际QSPI驱动时钟不要超过125M，
	*	  具体的时钟配置请参考 SystemClock_Config 函数
	*
	*	9.实际使用中，当数据比较大时，建议使用64K或者32K擦除，擦除时间比4K擦除块	
	*
	**************************************************************************************************************************************************************************************FANKE*****
***/

#include "w25qxx.hpp"
#include "project.hpp"
/*************************************************************************************************
*	函 数 名: QSPI_W25Qxx_Init
*	入口参数: 无
*	返 回 值: M_RESULT_EOK - 初始化成功，W25Qxx_ERROR_INIT - 初始化错误
*	函数功能: 初始化 QSPI 配置，读取W25Q64ID
*	说    明: 无	
*************************************************************************************************/

mResult W25QXX::QSPIW25QxxInit(void)
{
	uint32_t	Device_ID;	                                  // 器件ID
	_qspi = (mDev::mQspi*)(mDev::mPlatform::getInstance()->getDevice(DEV_QSPI));
	if(!_qspi)
	{
		return M_RESULT_ERROR;
	}
	QSPIW25QxxReset();							          // 复位器件
	Device_ID = QSPIW25QxxReadID(); 		                  // 读取器件ID
	
	if( Device_ID == W25Qxx_FLASH_ID )		                  // 进行匹配
	{
		printf ("W25Q64 OK,flash ID:%lX\r\n",Device_ID);	  // 初始化成功
		return M_RESULT_EOK;			                      // 返回成功标志		
	}
	else
	{
		printf ("W25Q64 ERROR!!!!!  ID:%lX\r\n",Device_ID);	  // 初始化失败	
		return M_RESULT_BADF;		                      // 返回错误标志
	}	
}

/*************************************************************************************************
*	函 数 名: QSPIW25QxxAutoPollingMemReady
*	入口参数: 无
*	返 回 值: M_RESULT_EOK - 通信正常结束，W25Qxx_ERROR_AUTOPOLLING - 轮询等待无响应
*	函数功能: 使用自动轮询标志查询，等待通信结束
*	说    明: 每一次通信都应该调用次函数，等待通信结束，避免错误的操作	
**************************************************************************************************/

mResult W25QXX::QSPIW25QxxAutoPollingMemReady(void)
{
	mDev::QSPICommand s_command;
	mDev::QSPIAutoPolling s_config;		// 轮询比较相关配置参数

	s_command.InstructionMode   = mDev::E_QSPI_INSTRUCTION_1_LINE;			// 1线指令模式
	s_command.AddressMode       = mDev::E_QSPI_ADDRESS_NONE;					// 无地址模式
	s_command.AlternateByteMode = mDev::E_QSPI_ALTERNATE_BYTES_NONE;			//	无交替字节 
	s_command.DdrMode           = mDev::E_QSPI_DDR_MODE_DISABLE;	     	 	// 禁止DDR模式
	s_command.DdrHoldHalfCycle  = mDev::E_QSPI_DDR_HHC_ANALOG_DELAY;	   	// DDR模式中数据延迟，这里用不到
	s_command.SIOOMode          = mDev::E_QSPI_SIOO_INST_EVERY_CMD;	   	//	每次传输数据都发送指令	
	s_command.DataMode          = mDev::E_QSPI_DATA_1_LINE;						// 1线数据模式
	s_command.DummyCycles       = 0;											//	空周期个数
	s_command.Instruction       = W25Qxx_CMD_ReadStatus_REG1;	   // 读状态信息寄存器
																					
// 不停的查询 W25Qxx_CMD_ReadStatus_REG1 寄存器，将读取到的状态字节中的 W25Qxx_Status_REG1_BUSY 不停的与0作比较
// 读状态寄存器1的第0位（只读），Busy标志位，当正在擦除/写入数据/写命令时会被置1，空闲或通信结束为0
	
	s_config.Match           = 0;   									//	匹配值
	s_config.MatchMode       = mDev::E_QSPI_MATCH_MODE_AND;	      	//	与运算
	s_config.Interval        = 0x10;	                     	//	轮询间隔
	s_config.AutomaticStop   = mDev::E_QSPI_AUTOMATIC_STOP_ENABLE;	// 自动停止模式
	s_config.StatusBytesSize = 1;	                        	//	状态字节数
	s_config.Mask            = W25Qxx_Status_REG1_BUSY;	   // 对在轮询模式下接收的状态字节进行屏蔽，只比较需要用到的位
		
	// 发送轮询等待命令
	if (_qspi->autoPolling(&s_command, &s_config, 5000U) != M_RESULT_EOK)
	{
		return M_RESULT_ETIMEOUT; // 轮询等待无响应
	}
	return M_RESULT_EOK; // 通信正常结束

}

/*************************************************************************************************
*	函 数 名: QSPI_W25Qxx_Reset
*	入口参数: 无
*	返 回 值: M_RESULT_EOK - 复位成功，W25Qxx_ERROR_INIT - 初始化错误
*	函数功能: 复位器件
*	说    明: 无	
*************************************************************************************************/

mResult W25QXX::QSPIW25QxxReset(void)	
{
	mDev::QSPICommand s_command;	// QSPI传输配置

	s_command.InstructionMode   = mDev::E_QSPI_INSTRUCTION_1_LINE;   	// 1线指令模式
	s_command.AddressMode 		 = mDev::E_QSPI_ADDRESS_NONE;   			// 无地址模式
	s_command.AlternateByteMode = mDev::E_QSPI_ALTERNATE_BYTES_NONE; 	// 无交替字节 
	s_command.DdrMode           = mDev::E_QSPI_DDR_MODE_DISABLE;     	// 禁止DDR模式
	s_command.DdrHoldHalfCycle  = mDev::E_QSPI_DDR_HHC_ANALOG_DELAY; 	// DDR模式中数据延迟，这里用不到
	s_command.SIOOMode          = mDev::E_QSPI_SIOO_INST_EVERY_CMD;	 	// 每次传输数据都发送指令
	s_command.DataMode 			 = mDev::E_QSPI_DATA_NONE;       			// 无数据模式	
	s_command.DummyCycles 		 = 0;                     			// 空周期个数
	s_command.Instruction 		 = W25Qxx_CMD_EnableReset;       // 执行复位使能命令

	// 发送复位使能命令
	if (_qspi->sendCmd(&s_command) != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;			// 如果发送失败，返回错误信息
	}
	// 使用自动轮询标志位，等待通信结束
	if (QSPIW25QxxAutoPollingMemReady() != M_RESULT_EOK)
	{
		return M_RESULT_ETIMEOUT;	// 轮询等待无响应
	}

	s_command.Instruction  = W25Qxx_CMD_ResetDevice;     // 复位器件命令    

	//发送复位器件命令
	if (_qspi->sendCmd(&s_command) != M_RESULT_EOK) 
	{
		return M_RESULT_ERROR;		  // 如果发送失败，返回错误信息
	}
	// 使用自动轮询标志位，等待通信结束
	if (QSPIW25QxxAutoPollingMemReady() != M_RESULT_EOK)
	{
		return M_RESULT_ETIMEOUT;	// 轮询等待无响应
	}	
	return M_RESULT_EOK;	// 复位成功
}

/*************************************************************************************************
*	函 数 名: QSPI_W25Qxx_ReadID
*	入口参数: 无
*	返 回 值: W25Qxx_ID - 读取到的器件ID，W25Qxx_ERROR_INIT - 通信、初始化错误
*	函数功能: 初始化 QSPI 配置，读取器件ID
*	说    明: 无	
**************************************************************************************************/

uint32_t W25QXX::QSPIW25QxxReadID(void)	
{
	mDev::QSPICommand s_command;	// QSPI传输配置
	uint8_t	QSPI_ReceiveBuff[3];		// 存储QSPI读到的数据
	uint32_t	W25Qxx_ID;					// 器件的ID

	s_command.InstructionMode   = mDev::E_QSPI_INSTRUCTION_1_LINE;    // 1线指令模式
	s_command.AddressSize       = mDev::E_QSPI_ADDRESS_24_BITS;     	 // 24位地址
	s_command.AlternateByteMode = mDev::E_QSPI_ALTERNATE_BYTES_NONE;  // 无交替字节 
	s_command.DdrMode           = mDev::E_QSPI_DDR_MODE_DISABLE;      // 禁止DDR模式
	s_command.DdrHoldHalfCycle  = mDev::E_QSPI_DDR_HHC_ANALOG_DELAY;  // DDR模式中数据延迟，这里用不到
	s_command.SIOOMode          = mDev::E_QSPI_SIOO_INST_EVERY_CMD;	 // 每次传输数据都发送指令
	s_command.AddressMode		 = mDev::E_QSPI_ADDRESS_NONE;   		 // 无地址模式
	s_command.DataMode			 = mDev::E_QSPI_DATA_1_LINE;       	 // 1线数据模式
	s_command.DummyCycles 		 = 0;                   		 // 空周期个数
	s_command.NbData 				 = 3;                          // 传输数据的长度
	s_command.Instruction 		 = W25Qxx_CMD_JedecID;         // 执行读器件ID命令

	// 发送指令
	if (_qspi->sendCmd(&s_command) != M_RESULT_EOK) 
	{
		return -1;		// 如果发送失败，返回错误信息
	}
	// 接收数据
	if (_qspi->receive(QSPI_ReceiveBuff) != M_RESULT_EOK) 
	{
		return -2;  // 如果接收失败，返回错误信息
	}
	// 将得到的数据组合成ID
	W25Qxx_ID = (QSPI_ReceiveBuff[0] << 16) | (QSPI_ReceiveBuff[1] << 8 ) | QSPI_ReceiveBuff[2];

	return W25Qxx_ID; // 返回ID
}



/*************************************************************************************************
*	函 数 名: QSPI_W25Qxx_MemoryMappedMode
*	入口参数: 无
*	返 回 值: M_RESULT_EOK - 写使能成功，W25Qxx_ERROR_WriteEnable - 写使能失败
*	函数功能: 将QSPI设置为内存映射模式
*	说    明: 设置为内存映射模式时，只能读，不能写！！！	
**************************************************************************************************/
#if 0
int8_t W25QXX::QSPIW25QxxMemoryMappedMode(void)
{
	mDev::QSPICommand      s_command;				 // QSPI传输配置
	QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;	 // 内存映射访问参数

	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;    		// 1线指令模式
	s_command.AddressSize       = QSPI_ADDRESS_24_BITS;            // 24位地址
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;  		// 无交替字节 
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;     		// 禁止DDR模式
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY; 		// DDR模式中数据延迟，这里用不到
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;			// 每次传输数据都发送指令	
	s_command.AddressMode 		 = QSPI_ADDRESS_4_LINES; 				// 4线地址模式
	s_command.DataMode    		 = QSPI_DATA_4_LINES;    				// 4线数据模式
	s_command.DummyCycles 		 = 6;                    				// 空周期个数
	s_command.Instruction 		 = W25Qxx_CMD_FastReadQuad_IO; 		// 1-4-4模式下(1线指令4线地址4线数据)，快速读取指令
	
	s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE; // 禁用超时计数器, nCS 保持激活状态
	s_mem_mapped_cfg.TimeOutPeriod     = 0;									 // 超时判断周期

	QSPIW25QxxReset();		// 复位W25Qxx
	
	if (HAL_QSPI_MemoryMapped(&hqspi, &s_command, &s_mem_mapped_cfg) != M_RESULT_EOK)	// 进行配置
	{
		return W25Qxx_ERROR_MemoryMapped; 	// 设置内存映射模式错误
	}

	return M_RESULT_EOK; // 配置成功
}
#endif
/*************************************************************************************************
*	函 数 名: QSPIW25QxxWriteEnable
*	入口参数: 无
*	返 回 值: M_RESULT_EOK - 写使能成功，W25Qxx_ERROR_WriteEnable - 写使能失败
*	函数功能: 发送写使能命令
*	说    明: 无	
**************************************************************************************************/

mResult W25QXX::QSPIW25QxxWriteEnable(void)
{
	mDev::QSPICommand     s_command;	   // QSPI传输配置
	mDev::QSPIAutoPolling s_config;		// 轮询比较相关配置参数

	s_command.InstructionMode   	= mDev::E_QSPI_INSTRUCTION_1_LINE;    	// 1线指令模式
	s_command.AddressMode 			= mDev::E_QSPI_ADDRESS_NONE;   		      // 无地址模式
	s_command.AlternateByteMode 	= mDev::E_QSPI_ALTERNATE_BYTES_NONE;  	// 无交替字节 
	s_command.DdrMode           	= mDev::E_QSPI_DDR_MODE_DISABLE;      	// 禁止DDR模式
	s_command.DdrHoldHalfCycle  	= mDev::E_QSPI_DDR_HHC_ANALOG_DELAY;  	// DDR模式中数据延迟，这里用不到
	s_command.SIOOMode          	= mDev::E_QSPI_SIOO_INST_EVERY_CMD;		// 每次传输数据都发送指令	
	s_command.DataMode 				= mDev::E_QSPI_DATA_NONE;       	      // 无数据模式
	s_command.DummyCycles 			= 0;                   	         // 空周期个数
	s_command.Instruction	 		= W25Qxx_CMD_WriteEnable;      	// 发送写使能命令

	// 发送写使能命令
	if (_qspi->sendCmd(&s_command) != M_RESULT_EOK) 
	{
		return M_RESULT_EBUSY;	//
	}
	
// 不停的查询 W25Qxx_CMD_ReadStatus_REG1 寄存器，将读取到的状态字节中的 W25Qxx_Status_REG1_WEL 不停的与 0x02 作比较
// 读状态寄存器1的第1位（只读），WEL写使能标志位，该标志位为1时，代表可以进行写操作
	
	s_config.Match           = 0x02;  								// 匹配值
	s_config.Mask            = W25Qxx_Status_REG1_WEL;	 		// 读状态寄存器1的第1位（只读），WEL写使能标志位，该标志位为1时，代表可以进行写操作
	s_config.MatchMode       = mDev::E_QSPI_MATCH_MODE_AND;			 	// 与运算
	s_config.StatusBytesSize = 1;									 	// 状态字节数
	s_config.Interval        = 0x10;							 		// 轮询间隔
	s_config.AutomaticStop   = mDev::E_QSPI_AUTOMATIC_STOP_ENABLE;	// 自动停止模式

	s_command.Instruction    = W25Qxx_CMD_ReadStatus_REG1;	// 读状态信息寄存器
	s_command.DataMode       = mDev::E_QSPI_DATA_1_LINE;					// 1线数据模式
	s_command.NbData         = 1;										// 数据长度

	// 发送轮询等待命令	
	if (_qspi->autoPolling(&s_command, &s_config, 5000U) != M_RESULT_EOK)
	{
		return M_RESULT_ETIMEOUT; 	// 轮询等待无响应
	}	
	return M_RESULT_EOK;  // 通信正常结束
}

/*************************************************************************************************
*
*	函 数 名: QSPI_W25Qxx_SectorErase
*
*	入口参数: SectorAddress - 要擦除的地址
*
*	返 回 值: M_RESULT_EOK - 擦除成功
*			    M_RESULT_ERROR - 擦除失败
*				 M_RESULT_ERROR - 轮询等待无响应
*
*	函数功能: 进行扇区擦除操作，每次擦除4K字节
*
*	说    明: 1.按照 W25Q64JV 数据手册给出的擦除参考时间，典型值为 45ms，最大值为400ms
*				 2.实际的擦除速度可能大于45ms，也可能小于45ms
*				 3.flash使用的时间越长，擦除所需时间也会越长
*
**************************************************************************************************/

mResult W25QXX::QSPIW25QxxSectorErase(uint32_t SectorAddress)	
{
	mDev::QSPICommand s_command;	// QSPI传输配置
	
	s_command.InstructionMode   	= mDev::E_QSPI_INSTRUCTION_1_LINE;    // 1线指令模式
	s_command.AddressSize       	= mDev::E_QSPI_ADDRESS_24_BITS;       // 24位地址模式
	s_command.AlternateByteMode 	= mDev::E_QSPI_ALTERNATE_BYTES_NONE;  //	无交替字节 
	s_command.DdrMode           	= mDev::E_QSPI_DDR_MODE_DISABLE;      // 禁止DDR模式
	s_command.DdrHoldHalfCycle  	= mDev::E_QSPI_DDR_HHC_ANALOG_DELAY;  // DDR模式中数据延迟，这里用不到
	s_command.SIOOMode          	= mDev::E_QSPI_SIOO_INST_EVERY_CMD;	// 每次传输数据都发送指令
	s_command.AddressMode 			= mDev::E_QSPI_ADDRESS_1_LINE;        // 1线地址模式
	s_command.DataMode 				= mDev::E_QSPI_DATA_NONE;             // 无数据
	s_command.DummyCycles 			= 0;                          // 空周期个数
	s_command.Address           	= SectorAddress;              // 要擦除的地址
	s_command.Instruction	 		= W25Qxx_CMD_SectorErase;     // 扇区擦除命令

	// 发送写使能
	if (QSPIW25QxxWriteEnable() != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;		// 写使能失败
	}
	// 发出擦除命令
	if (_qspi->sendCmd(&s_command) != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;				// 擦除失败
	}
	// 使用自动轮询标志位，等待擦除的结束 
	if (QSPIW25QxxAutoPollingMemReady() != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;		// 轮询等待无响应
	}
	return M_RESULT_EOK; // 擦除成功
}

/*************************************************************************************************
*
*	函 数 名: QSPI_W25Qxx_BlockErase_32K
*
*	入口参数: SectorAddress - 要擦除的地址
*
*	返 回 值: M_RESULT_EOK - 擦除成功
*			    M_RESULT_ERROR - 擦除失败
*				 M_RESULT_ERROR - 轮询等待无响应
*
*	函数功能: 进行块擦除操作，每次擦除32K字节
*
*	说    明: 1.按照 W25Q64JV 数据手册给出的擦除参考时间，典型值为 120ms，最大值为1600ms
*				 2.实际的擦除速度可能大于120ms，也可能小于120ms
*				 3.flash使用的时间越长，擦除所需时间也会越长
*
*************************************************************************************************/

mResult W25QXX::QSPIW25QxxBlockErase32K (uint32_t SectorAddress)	
{
	mDev::QSPICommand s_command;	// QSPI传输配置
	
	s_command.InstructionMode   	= mDev::E_QSPI_INSTRUCTION_1_LINE;    // 1线指令模式
	s_command.AddressSize       	= mDev::E_QSPI_ADDRESS_24_BITS;       // 24位地址模式
	s_command.AlternateByteMode 	= mDev::E_QSPI_ALTERNATE_BYTES_NONE;  //	无交替字节 
	s_command.DdrMode           	= mDev::E_QSPI_DDR_MODE_DISABLE;      // 禁止DDR模式
	s_command.DdrHoldHalfCycle  	= mDev::E_QSPI_DDR_HHC_ANALOG_DELAY;  // DDR模式中数据延迟，这里用不到
	s_command.SIOOMode          	= mDev::E_QSPI_SIOO_INST_EVERY_CMD;	// 每次传输数据都发送指令
	s_command.AddressMode 			= mDev::E_QSPI_ADDRESS_1_LINE;        // 1线地址模式
	s_command.DataMode 				= mDev::E_QSPI_DATA_NONE;             // 无数据
	s_command.DummyCycles 			= 0;                          // 空周期个数
	s_command.Address           	= SectorAddress;              // 要擦除的地址
	s_command.Instruction	 		= W25Qxx_CMD_BlockErase_32K;  // 块擦除命令，每次擦除32K字节

	// 发送写使能	
	if (QSPIW25QxxWriteEnable() != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;		// 写使能失败
	}
	// 发出擦除命令
	if (_qspi->sendCmd(&s_command) != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;				// 擦除失败
	}
	// 使用自动轮询标志位，等待擦除的结束 
	if (QSPIW25QxxAutoPollingMemReady() != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;		// 轮询等待无响应
	}
	return M_RESULT_EOK;	// 擦除成功
}

/*************************************************************************************************
*
*	函 数 名: QSPI_W25Qxx_BlockErase_64K
*
*	入口参数: SectorAddress - 要擦除的地址
*
*	返 回 值: M_RESULT_EOK - 擦除成功
*			    M_RESULT_ERROR - 擦除失败
*				 M_RESULT_ERROR - 轮询等待无响应
*
*	函数功能: 进行块擦除操作，每次擦除64K字节
*
*	说    明: 1.按照 W25Q64JV 数据手册给出的擦除参考时间，典型值为 150ms，最大值为2000ms
*				 2.实际的擦除速度可能大于150ms，也可能小于150ms
*				 3.flash使用的时间越长，擦除所需时间也会越长
*				 4.实际使用建议使用64K擦除，擦除的时间最快
*
**************************************************************************************************/

mResult W25QXX::QSPIW25QxxBlockErase64K (uint32_t SectorAddress)	
{
	mDev::QSPICommand s_command;	// QSPI传输配置
	
	s_command.InstructionMode   	= mDev::E_QSPI_INSTRUCTION_1_LINE;    // 1线指令模式
	s_command.AddressSize       	= mDev::E_QSPI_ADDRESS_24_BITS;       // 24位地址模式
	s_command.AlternateByteMode 	= mDev::E_QSPI_ALTERNATE_BYTES_NONE;  //	无交替字节 
	s_command.DdrMode           	= mDev::E_QSPI_DDR_MODE_DISABLE;      // 禁止DDR模式
	s_command.DdrHoldHalfCycle  	= mDev::E_QSPI_DDR_HHC_ANALOG_DELAY;  // DDR模式中数据延迟，这里用不到
	s_command.SIOOMode          	= mDev::E_QSPI_SIOO_INST_EVERY_CMD;	// 每次传输数据都发送指令
	s_command.AddressMode 			= mDev::E_QSPI_ADDRESS_1_LINE;        // 1线地址模式
	s_command.DataMode 				= mDev::E_QSPI_DATA_NONE;             // 无数据
	s_command.DummyCycles 			= 0;                          // 空周期个数
	s_command.Address           	= SectorAddress;              // 要擦除的地址
	s_command.Instruction	 		= W25Qxx_CMD_BlockErase_64K;  // 块擦除命令，每次擦除64K字节	

	// 发送写使能
	if (QSPIW25QxxWriteEnable() != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;	// 写使能失败
	}
	// 发出擦除命令
	if (_qspi->sendCmd(&s_command) != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;			// 擦除失败
	}
	// 使用自动轮询标志位，等待擦除的结束 
	if (QSPIW25QxxAutoPollingMemReady() != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;	// 轮询等待无响应
	}
	return M_RESULT_EOK;		// 擦除成功
}

/*************************************************************************************************
*
*	函 数 名: QSPI_W25Qxx_ChipErase
*
*	入口参数: 无
*
*	返 回 值: M_RESULT_EOK - 擦除成功
*			    M_RESULT_ERROR - 擦除失败
*				 M_RESULT_ERROR - 轮询等待无响应
*
*	函数功能: 进行整片擦除操作
*
*	说    明: 1.按照 W25Q64JV 数据手册给出的擦除参考时间，典型值为 20s，最大值为100s
*				 2.实际的擦除速度可能大于20s，也可能小于20s
*				 3.flash使用的时间越长，擦除所需时间也会越长
*
*************************************************************************************************/

mResult W25QXX::QSPIW25QxxChipErase (void)	
{
	mDev::QSPICommand s_command;		// QSPI传输配置
	mDev::QSPIAutoPolling s_config;	// 轮询等待配置参数

	s_command.InstructionMode   	= mDev::E_QSPI_INSTRUCTION_1_LINE;    // 1线指令模式
	s_command.AddressSize       	= mDev::E_QSPI_ADDRESS_24_BITS;       // 24位地址模式
	s_command.AlternateByteMode 	= mDev::E_QSPI_ALTERNATE_BYTES_NONE;  //	无交替字节 
	s_command.DdrMode           	= mDev::E_QSPI_DDR_MODE_DISABLE;      // 禁止DDR模式
	s_command.DdrHoldHalfCycle  	= mDev::E_QSPI_DDR_HHC_ANALOG_DELAY;  // DDR模式中数据延迟，这里用不到
	s_command.SIOOMode          	= mDev::E_QSPI_SIOO_INST_EVERY_CMD;	// 每次传输数据都发送指令
	s_command.AddressMode 			= mDev::E_QSPI_ADDRESS_NONE;       	// 无地址
	s_command.DataMode 				= mDev::E_QSPI_DATA_NONE;             // 无数据
	s_command.DummyCycles 			= 0;                          // 空周期个数
	s_command.Instruction	 		= W25Qxx_CMD_ChipErase;       // 擦除命令，进行整片擦除

	// 发送写使能	
	if (QSPIW25QxxWriteEnable() != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;	// 写使能失败
	}
	// 发出擦除命令
	if (_qspi->sendCmd(&s_command) != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;		 // 擦除失败
	}

// 不停的查询 W25Qxx_CMD_ReadStatus_REG1 寄存器，将读取到的状态字节中的 W25Qxx_Status_REG1_BUSY 不停的与0作比较
// 读状态寄存器1的第0位（只读），Busy标志位，当正在擦除/写入数据/写命令时会被置1，空闲或通信结束为0
	
	s_config.Match           = 0;   									//	匹配值
	s_config.MatchMode       = mDev::E_QSPI_MATCH_MODE_AND;	      	//	与运算
	s_config.Interval        = 0x10;	                     	//	轮询间隔
	s_config.AutomaticStop   = mDev::E_QSPI_AUTOMATIC_STOP_ENABLE;	// 自动停止模式
	s_config.StatusBytesSize = 1;	                        	//	状态字节数
	s_config.Mask            = W25Qxx_Status_REG1_BUSY;	   // 对在轮询模式下接收的状态字节进行屏蔽，只比较需要用到的位
	
	s_command.Instruction    = W25Qxx_CMD_ReadStatus_REG1;	// 读状态信息寄存器
	s_command.DataMode       = mDev::E_QSPI_DATA_1_LINE;					// 1线数据模式
	s_command.NbData         = 1;										// 数据长度

	// W25Q64整片擦除的典型参考时间为20s，最大时间为100s，这里的超时等待值 W25Qxx_ChipErase_TIMEOUT_MAX 为 100S
	if (_qspi->autoPolling(&s_command, &s_config, W25Qxx_ChipErase_TIMEOUT_MAX) != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;	 // 轮询等待无响应
	}
	return M_RESULT_EOK;
}

/**********************************************************************************************************
*
*	函 数 名: QSPIW25QxxWritePage
*
*	入口参数: pBuffer 		 - 要写入的数据
*				 WriteAddr 		 - 要写入 W25Qxx 的地址
*				 NumByteToWrite - 数据长度，最大只能256字节
*
*	返 回 值: M_RESULT_EOK 		     - 写数据成功
*			    M_RESULT_ERROR - 写使能失败
*				 M_RESULT_ERROR	  - 传输失败
*				 M_RESULT_ERROR - 轮询等待无响应
*
*	函数功能: 按页写入，最大只能256字节，在数据写入之前，请务必完成擦除操作
*
*	说    明: 1.Flash的写入时间和擦除时间一样，是限定的，并不是说QSPI驱动时钟133M就可以以这个速度进行写入
*				 2.按照 W25Q64JV 数据手册给出的 页(256字节) 写入参考时间，典型值为 0.4ms，最大值为3ms
*				 3.实际的写入速度可能大于0.4ms，也可能小于0.4ms
*				 4.Flash使用的时间越长，写入所需时间也会越长
*				 5.在数据写入之前，请务必完成擦除操作
*
***********************************************************************************************************/

mResult W25QXX::QSPIW25QxxWritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	mDev::QSPICommand s_command;	// QSPI传输配置	
	
	s_command.InstructionMode   = mDev::E_QSPI_INSTRUCTION_1_LINE;    		// 1线指令模式
	s_command.AddressSize       = mDev::E_QSPI_ADDRESS_24_BITS;            // 24位地址
	s_command.AlternateByteMode = mDev::E_QSPI_ALTERNATE_BYTES_NONE;  		// 无交替字节 
	s_command.DdrMode           = mDev::E_QSPI_DDR_MODE_DISABLE;     		// 禁止DDR模式
	s_command.DdrHoldHalfCycle  = mDev::E_QSPI_DDR_HHC_ANALOG_DELAY; 		// DDR模式中数据延迟，这里用不到
	s_command.SIOOMode          = mDev::E_QSPI_SIOO_INST_EVERY_CMD;			// 每次传输数据都发送指令	
	s_command.AddressMode 		 = mDev::E_QSPI_ADDRESS_1_LINE; 				// 1线地址模式
	s_command.DataMode    		 = mDev::E_QSPI_DATA_4_LINES;    				// 4线数据模式
	s_command.DummyCycles 		 = 0;                    				// 空周期个数
	s_command.NbData      		 = NumByteToWrite;      			   // 数据长度，最大只能256字节
	s_command.Address     		 = WriteAddr;         					// 要写入 W25Qxx 的地址
	s_command.Instruction 		 = W25Qxx_CMD_QuadInputPageProgram; // 1-1-4模式下(1线指令1线地址4线数据)，页编程指令
	
	// 写使能
	if (QSPIW25QxxWriteEnable() != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;	// 写使能失败
	}
	// 写命令
	if (_qspi->sendCmd(&s_command) != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;		// 传输数据错误
	}
	// 开始传输数据
	if (_qspi->sendData(pBuffer) != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;		// 传输数据错误
	}
	// 使用自动轮询标志位，等待写入的结束 
	if (QSPIW25QxxAutoPollingMemReady() != M_RESULT_EOK)
	{
		return M_RESULT_ERROR; // 轮询等待无响应
	}
	return M_RESULT_EOK;	// 写数据成功
}

/**********************************************************************************************************
*
*	函 数 名: QSPI_W25Qxx_WriteBuffer
*
*	入口参数: pBuffer 		 - 要写入的数据
*				 WriteAddr 		 - 要写入 W25Qxx 的地址
*				 NumByteToWrite - 数据长度，最大不能超过flash芯片的大小
*
*	返 回 值: M_RESULT_EOK 		     - 写数据成功
*			    M_RESULT_ERROR - 写使能失败
*				 M_RESULT_ERROR	  - 传输失败
*				 M_RESULT_ERROR - 轮询等待无响应
*
*	函数功能: 写入数据，最大不能超过flash芯片的大小，请务必完成擦除操作
*
*	说    明: 1.Flash的写入时间和擦除时间一样，是有限定的，并不是说QSPI驱动时钟133M就可以以这个速度进行写入
*				 2.按照 W25Q64JV 数据手册给出的 页 写入参考时间，典型值为 0.4ms，最大值为3ms
*				 3.实际的写入速度可能大于0.4ms，也可能小于0.4ms
*				 4.Flash使用的时间越长，写入所需时间也会越长
*				 5.在数据写入之前，请务必完成擦除操作
*				 6.该函数移植于 stm32h743i_eval_qspi.c
*
**********************************************************************************************************/

mResult W25QXX::QSPIW25QxxWriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t Size)
{	
	uint32_t end_addr, current_size, current_addr;
	uint8_t *write_data;  // 要写入的数据

	current_size = W25Qxx_PageSize - (WriteAddr % W25Qxx_PageSize); // 计算当前页还剩余的空间

	if (current_size > Size)	// 判断当前页剩余的空间是否足够写入所有数据
	{
		current_size = Size;		// 如果足够，则直接获取当前长度
	}

	current_addr = WriteAddr;		// 获取要写入的地址
	end_addr = WriteAddr + Size;	// 计算结束地址
	write_data = pBuffer;			// 获取要写入的数据

	do
	{
		// 发送写使能
		if (QSPIW25QxxWriteEnable() != M_RESULT_EOK)
		{
			return M_RESULT_ERROR;
		}

		// 按页写入数据
		else if(QSPIW25QxxWritePage(write_data, current_addr, current_size) != M_RESULT_EOK)
		{
			return M_RESULT_ERROR;
		}

		// 使用自动轮询标志位，等待写入的结束 
		else if (QSPIW25QxxAutoPollingMemReady() != M_RESULT_EOK)
		{
			return M_RESULT_ERROR;
		}

		else // 按页写入数据成功，进行下一次写数据的准备工作
		{
			current_addr += current_size;	// 计算下一次要写入的地址
			write_data += current_size;	// 获取下一次要写入的数据存储区地址
			// 计算下一次写数据的长度
			current_size = ((current_addr + W25Qxx_PageSize) > end_addr) ? (end_addr - current_addr) : W25Qxx_PageSize;
		}
	}
	while (current_addr < end_addr) ; // 判断数据是否全部写入完毕

	return M_RESULT_EOK;	// 写入数据成功
}

/**********************************************************************************************************************************
*
*	函 数 名: QSPI_W25Qxx_ReadBuffer
*
*	入口参数: pBuffer 		 - 要读取的数据
*				 ReadAddr 		 - 要读取 W25Qxx 的地址
*				 NumByteToRead  - 数据长度，最大不能超过flash芯片的大小
*
*	返 回 值: M_RESULT_EOK 		     - 读数据成功
*				 M_RESULT_ERROR	  - 传输失败
*				 M_RESULT_ERROR - 轮询等待无响应
*
*	函数功能: 读取数据，最大不能超过flash芯片的大小
*
*	说    明: 1.Flash的读取速度取决于QSPI的通信时钟，最大不能超过133M
*				 2.这里使用的是1-4-4模式下(1线指令4线地址4线数据)，快速读取指令 Fast Read Quad I/O
*				 3.使用快速读取指令是有空周期的，具体参考W25Q64JV的手册  Fast Read Quad I/O  （0xEB）指令
*				 4.实际使用中，是否使用DMA、编译器的优化等级以及数据存储区的位置(内部 TCM SRAM 或者 AXI SRAM)都会影响读取的速度
*			    5.在本例程中，使用的是库函数进行直接读写，keil版本5.30，编译器AC6.14，编译等级Oz image size，读取速度为 7M字节/S ，
*		         数据放在 TCM SRAM 或者 AXI SRAM 都是差不多的结果
*		       6.因为CPU直接访问外设寄存器的效率很低，直接使用HAL库进行读写的话，速度很慢，使用MDMA进行读取，可以达到 58M字节/S
*	          7. W25Q64JV 所允许的最高驱动频率为133MHz，750的QSPI最高驱动频率也是133MHz ，但是对于HAL库函数直接读取而言，
*		          驱动时钟超过15M已经不会对性能有提升，对速度要求高的场合可以用MDMA的方式
*
*****************************************************************************************************************FANKE************/

mResult W25QXX::QSPIW25QxxReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
	mDev::QSPICommand s_command;	// QSPI传输配置
	
	s_command.InstructionMode   = mDev::E_QSPI_INSTRUCTION_1_LINE;    		// 1线指令模式
	s_command.AddressSize       = mDev::E_QSPI_ADDRESS_24_BITS;            // 24位地址
	s_command.AlternateByteMode = mDev::E_QSPI_ALTERNATE_BYTES_NONE;  		// 无交替字节 
	s_command.DdrMode           = mDev::E_QSPI_DDR_MODE_DISABLE;     		// 禁止DDR模式
	s_command.DdrHoldHalfCycle  = mDev::E_QSPI_DDR_HHC_ANALOG_DELAY; 		// DDR模式中数据延迟，这里用不到
	s_command.SIOOMode          = mDev::E_QSPI_SIOO_INST_EVERY_CMD;			// 每次传输数据都发送指令	
	s_command.AddressMode 		 = mDev::E_QSPI_ADDRESS_4_LINES; 				// 4线地址模式
	s_command.DataMode    		 = mDev::E_QSPI_DATA_4_LINES;    				// 4线数据模式
	s_command.DummyCycles 		 = 6;                    				// 空周期个数
	s_command.NbData      		 = NumByteToRead;      			   	// 数据长度，最大不能超过flash芯片的大小
	s_command.Address     		 = ReadAddr;         					// 要读取 W25Qxx 的地址
	s_command.Instruction 		 = W25Qxx_CMD_FastReadQuad_IO; 		// 1-4-4模式下(1线指令4线地址4线数据)，快速读取指令
	
	// 发送读取命令
	if (_qspi->sendCmd(&s_command) != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;		// 传输数据错误
	}

	//	接收数据
	
	if (_qspi->receive(pBuffer) != M_RESULT_EOK)
	{
		return M_RESULT_ERROR;		// 传输数据错误
	}

	// 使用自动轮询标志位，等待接收的结束 
	if (QSPIW25QxxAutoPollingMemReady() != M_RESULT_EOK)
	{
		return M_RESULT_ERROR; // 轮询等待无响应
	}
	return M_RESULT_EOK;	// 读取数据成功
}


//	实验平台：反客STM32H750XBH6核心板 （型号：FK750M4-XBH6）

/********************************************************************************************************************************************************************************************************FANKE**********/