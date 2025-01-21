#include "sdmmc.hpp"
#include "gpio.hpp"
#include "project.hpp"

static sdmmc* sd1 = nullptr;

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
            gpioSDMMCD0.init([](bool b){if(b)__HAL_RCC_GPIOC_CLK_ENABLE();}, GPIOC, GPIO_PIN_8, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF12_SDIO1);
            gpiox gpioSDMMCD1("sdgpiod1");
            gpioSDMMCD1.init([](bool b){if(b)__HAL_RCC_GPIOC_CLK_ENABLE();}, GPIOC, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF12_SDIO1);
            gpiox gpioSDMMCD2("sdgpiod2");
            gpioSDMMCD2.init([](bool b){if(b)__HAL_RCC_GPIOC_CLK_ENABLE();}, GPIOC, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF12_SDIO1);
            gpiox gpioSDMMCD3("sdgpiod3");
            gpioSDMMCD3.init([](bool b){if(b)__HAL_RCC_GPIOC_CLK_ENABLE();}, GPIOC, GPIO_PIN_11, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF12_SDIO1);
            gpiox gpioSDMMCCK("sdgpiock");
            gpioSDMMCCK.init([](bool b){if(b)__HAL_RCC_GPIOC_CLK_ENABLE();}, GPIOC, GPIO_PIN_12, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF12_SDIO1);
            gpiox gpioSDMMCCMD("sdgpiocmd");
            gpioSDMMCCMD.init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();}, GPIOD, GPIO_PIN_2, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF12_SDIO1);

            __HAL_RCC_SDMMC1_FORCE_RESET();
            __HAL_RCC_SDMMC1_RELEASE_RESET();

            /* NVIC configuration for SDIO interrupts */
            HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
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
    return 0;
}
INIT_EXPORT(sdmmcInit, "0.4");

