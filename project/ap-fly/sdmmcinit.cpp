#include "sdmmc.hpp"
#include "gpio.hpp"
#include "project.hpp"
#include <stm32h7xx_hal_sd.h>
#include <stm32h750xx.h>

static sdmmc* sd1 = nullptr;

/**
  * @brief Read DMA Buffer 0 Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SDEx_Read_DMADoubleBuffer0CpltCallback(SD_HandleTypeDef *hsd)
{
  //SCB_InvalidateDCache_by_Addr(Buffer0, BUFFER_WORD_SIZE*4);
  //ReadError += Buffercmp(Buffer0, BUFFERSIZE, DATA_PATTERN0 + (RBuff_0 * (uint32_t)0x00010000));
  //RBuff_0++;

}

/**
  * @brief Read DMA Buffer 1 Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SDEx_Read_DMADoubleBuffer1CpltCallback(SD_HandleTypeDef *hsd)
{
  //SCB_InvalidateDCache_by_Addr(Buffer1, BUFFER_WORD_SIZE*4);
  //ReadError += Buffercmp(Buffer1, BUFFERSIZE, DATA_PATTERN1 + (RBuff_1 * (uint32_t)0x00010000));
  //RBuff_1++;
}

/**
  * @brief Write DMA Buffer 0 Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SDEx_Write_DMADoubleBuffer0CpltCallback(SD_HandleTypeDef *hsd)
{
  //WBuff_0++;
  //Fill_Buffer(Buffer0, BUFFERSIZE, DATA_PATTERN0 + (WBuff_0 * (uint32_t)0x00010000));

}

/**
  * @brief Write DMA Buffer 1 Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SDEx_Write_DMADoubleBuffer1CpltCallback(SD_HandleTypeDef *hsd)
{
  //WBuff_1++;
  //Fill_Buffer(Buffer1, BUFFERSIZE, DATA_PATTERN1 + (WBuff_1 * (uint32_t)0x00010000));
}

/**
  * @brief SD Abort callbacks
  * @param hsd: SD handle
  * @retval None
  */
extern "C" void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
    sdmmc* sdx = containerof(hsd, sdmmc, uSdHandle);
    if(hsd == sdx->sdmmcHandle())
    {
        mDev::MSDMMC_IRQ_TYPE type = mDev::MSDMMC_IRQ_TYPE::SDMMC_IRQ_ABORT;
        sdx->runInterruptCb(&type);
    }
}

/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
extern "C" void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    sdmmc* sdx = containerof(hsd, sdmmc, uSdHandle);
    if(hsd == sdx->sdmmcHandle())
    {
        mDev::MSDMMC_IRQ_TYPE type = mDev::MSDMMC_IRQ_TYPE::SDMMC_IRQ_TX_COMPLETE;
        sdx->runInterruptCb(&type);
    }
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
extern "C" void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
    sdmmc* sdx = containerof(hsd, sdmmc, uSdHandle);
    if(hsd == sdx->sdmmcHandle())
    {
        mDev::MSDMMC_IRQ_TYPE type = mDev::MSDMMC_IRQ_TYPE::SDMMC_IRQ_RX_COMPLETE;
        sdx->runInterruptCb(&type);
    }
}

/**
  * @brief Error callbacks
  * @param hsd: SD handle
  * @retval None
  */
extern "C" void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    sdmmc* sdx = containerof(hsd, sdmmc, uSdHandle);
    if(hsd == sdx->sdmmcHandle())
    {
        mDev::MSDMMC_IRQ_TYPE type = mDev::MSDMMC_IRQ_TYPE::SDMMC_IRQ_ERROR;
        sdx->runInterruptCb(&type);
    }
}
  
/**
  * @brief  Enable the SD Transceiver 1.8V Mode Callback.
  */
extern "C" void HAL_SD_DriveTransciver_1_8V_Callback(FlagStatus status)
{
    mDev::MSDMMC_IRQ_TYPE type = status == RESET ? mDev::MSDMMC_IRQ_TYPE::SDMMC_IRQ_TX_1V8_MODE_RESET : mDev::MSDMMC_IRQ_TYPE::SDMMC_IRQ_TX_1V8_MODE_SET;
    sd1->runInterruptCb(&type);
}

extern "C" void SDMMC1_IRQHandler(void)
{
	HAL_SD_IRQHandler(sd1->sdmmcHandle());
}

int sdmmcInit()
{
    SD_HandleTypeDef uSdHandle;
    memset(&uSdHandle, 0, sizeof(SD_HandleTypeDef));
    uSdHandle.Instance = SDMMC1;
    uSdHandle.Init.ClockDiv            = 2;
    uSdHandle.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    uSdHandle.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    uSdHandle.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    uSdHandle.Init.BusWide             = SDMMC_BUS_WIDE_4B;

    sd1 = new sdmmc("sd0");
    sd1->init([](bool enable){
        if(enable)
        {
            /* Enable SDIO clock */
            __HAL_RCC_SDMMC1_CLK_ENABLE();
            __HAL_RCC_GPIOD_CLK_ENABLE();
            /* D0(PC8), D1(PC9), D2(PC10), D3(PC11), CK(PC12), CMD(PD2) */
            gpiox gpioSDMMCD0("sdgpiod0");
            gpioSDMMCD0.init([](bool b){if(b)__HAL_RCC_GPIOC_CLK_ENABLE();}, GPIOC, GPIO_PIN_8, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF12_SDIO1);
            gpiox gpioSDMMCD1("sdgpiod1");
            gpioSDMMCD1.init([](bool b){if(b)__HAL_RCC_GPIOC_CLK_ENABLE();}, GPIOC, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF12_SDIO1);
            gpiox gpioSDMMCD2("sdgpiod2");
            gpioSDMMCD2.init([](bool b){if(b)__HAL_RCC_GPIOC_CLK_ENABLE();}, GPIOC, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF12_SDIO1);
            gpiox gpioSDMMCD3("sdgpiod3");
            gpioSDMMCD3.init([](bool b){if(b)__HAL_RCC_GPIOC_CLK_ENABLE();}, GPIOC, GPIO_PIN_11, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF12_SDIO1);
            gpiox gpioSDMMCCK("sdgpiock");
            gpioSDMMCCK.init([](bool b){if(b)__HAL_RCC_GPIOC_CLK_ENABLE();}, GPIOC, GPIO_PIN_12, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF12_SDIO1);
            gpiox gpioSDMMCCMD("sdgpiocmd");
            gpioSDMMCCMD.init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();}, GPIOD, GPIO_PIN_2, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF12_SDIO1);

            __HAL_RCC_SDMMC1_FORCE_RESET();
            __HAL_RCC_SDMMC1_RELEASE_RESET();

            /* NVIC configuration for SDIO interrupts */
            HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
            printf("tony init sd\r\n");
        }
        else
        {
            HAL_NVIC_DisableIRQ(SDMMC1_IRQn);

            /* DeInit GPIO pins can be done in the application
            (by surcharging this __weak function) */

            /* Disable SDMMC1 clock */
            __HAL_RCC_SDMMC1_CLK_DISABLE();
        }
    }, &uSdHandle);
    mDev::MSDMMC_CARD_INFO CardInfo;
    sd1->setTransferMode(mDev::MSDMMC_TRANSFER_MODE::SDMMC_TRANSFER_MODE_NORMAL);
    sd1->getCardInfo(&CardInfo);
    printf("card info: %d\n", CardInfo.cardType);
    printf("card version: %d\n", CardInfo.cardVersion);
    printf("card cardSpeed: %d\n", CardInfo.cardSpeed);
    if(sd1->erase(0,100) != M_RESULT_EOK)
    {
        printf("erase failed\n");
    }
    //HAL_Delay(1000);
    uint32_t data = 100;
    if(sd1->writeBlocks(&data, 0, 0) != M_RESULT_EOK)
    {
        printf("write failed\n");
    }
    //HAL_Delay(1000);
    data=0;
    if(sd1->readBlocks(&data, 0, 0) != M_RESULT_EOK)
    {
        printf("write failed\n");
    }
    //HAL_Delay(1000);
    printf("read data = %u\r\n",data);
    return 0;
}
INIT_EXPORT(sdmmcInit, "0.4");

