#include "stm32h7xx_hal_conf.h"
#include "spi.hpp"
#include "led.hpp"
#include "DFRobot_ICM42688.h"
#include "DFRobot_ICM42605.h"
#include "mdevice.hpp"
#include "containers.hpp"
#include "mgpio.hpp"
#include "gpio.hpp"
#include "mplatform.hpp"
#include "timer.hpp"
#include "bmi088.hpp"
#include "delay.h"
#include "mplatform.hpp"
timerx* timer1 = nullptr;
timerx* timer2 = nullptr;
spix* spi1 = nullptr;
spix* spi4 = nullptr;

#if 1
int initAllDevice()
{
    ledx* led0 = new ledx("led0");
    led0->init([](bool benable){ if(benable) __HAL_RCC_GPIOD_CLK_ENABLE(); },GPIOD,GPIO_PIN_15);
    ledx* led1 = new ledx("led1");
    led1->init([](bool benable){ if(benable) __HAL_RCC_GPIOB_CLK_ENABLE(); },GPIOB,GPIO_PIN_15);
    ledx* led2 = new ledx("led2");
    led2->init([](bool benable){ if(benable) __HAL_RCC_GPIOD_CLK_ENABLE(); },GPIOD,GPIO_PIN_11);

    //TIM1 INIT
    TIM_HandleTypeDef timerst;
    memset(&timerst, 0, sizeof(TIM_HandleTypeDef));
    timerst.Instance = TIM1;
    timerst.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    timerst.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timerst.Init.CounterMode = TIM_COUNTERMODE_UP;
    timerst.Init.RepetitionCounter = 0;

    timer1 = new timerx("timer1");
    timer1->calcPeriodAndPrescalerByFreq(&timerst,200);
    timer1->baseTimeInit([](bool b){
        if(b)
        {
            __HAL_RCC_TIM1_CLK_ENABLE();
            HAL_NVIC_SetPriority(TIM1_UP_IRQn, 3, 0);
            HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
        }
    }, &timerst,mDev::TIMESTARTMODE_IT);

    SPI_HandleTypeDef spixHandle;
    spi1 = new spix("spi1");
    memset(&spixHandle, 0, sizeof(SPI_HandleTypeDef));
    spixHandle.Instance = SPI1;
    spixHandle.Init.Mode = SPI_MODE_MASTER;
    spixHandle.Init.Direction = SPI_DIRECTION_2LINES;
    spixHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    spixHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    spixHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spixHandle.Init.NSS = SPI_NSS_SOFT;
    spixHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    spixHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spixHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    spixHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spixHandle.Init.CRCPolynomial = 0x0;
    spixHandle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    spixHandle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    spixHandle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    spixHandle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_15CYCLE;
    spixHandle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_15CYCLE;
    spixHandle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    spixHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    spixHandle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    spi1->init([](bool b){
        if(b)
        {
            RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
            PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
            PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI1CLKSOURCE_CLKP;
            HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
            __HAL_RCC_SPI1_CLK_ENABLE();
            gpiox spi1mosi("spi1sck");
            spi1mosi.init([](bool b){if(b)__HAL_RCC_GPIOA_CLK_ENABLE();},GPIOA, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI1);
            gpiox spi1miso("spi1miso");
            spi1miso.init([](bool b){if(b)__HAL_RCC_GPIOA_CLK_ENABLE();},GPIOA, GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI1);
            gpiox spi1sck("spi1mosi");
            spi1sck.init([](bool b){if(b)__HAL_RCC_GPIOA_CLK_ENABLE();},GPIOA, GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI1);
        }
    },&spixHandle);
    gpiox* imu1acs = new gpiox("imu1acs");
    imu1acs->init([](bool b){if(b){__HAL_RCC_GPIOA_CLK_ENABLE();}},GPIOA,GPIO_PIN_2,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);

    gpiox* imu1gcs = new gpiox("imu1gcs");
    imu1gcs->init([](bool b){if(b){__HAL_RCC_GPIOA_CLK_ENABLE();}},GPIOA,GPIO_PIN_3,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);

    imu1acs->setLevel(mDev::mGpio::LEVEL_HIGH);
    imu1gcs->setLevel(mDev::mGpio::LEVEL_HIGH);

    Bmi088* imu1 = new Bmi088("imu1",*spi1,*imu1acs,*imu1gcs);
    imu1->begin();

    //SPI4 init
    spi4 = new spix("spi4");
    memset(&spixHandle, 0, sizeof(SPI_HandleTypeDef));

    spixHandle.Instance = SPI4;
    spixHandle.Init.Mode = SPI_MODE_MASTER;
    spixHandle.Init.Direction = SPI_DIRECTION_2LINES;
    spixHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    spixHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    spixHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spixHandle.Init.NSS = SPI_NSS_SOFT;
    spixHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    spixHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spixHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    spixHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spixHandle.Init.CRCPolynomial = 0x0;
    spixHandle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    spixHandle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    spixHandle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    spixHandle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_15CYCLE;
    spixHandle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_15CYCLE;
    spixHandle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    spixHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    spixHandle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    spi4->init([](bool b){
        if(b)
        {
            RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
            PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
            PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;
            HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
            __HAL_RCC_SPI4_CLK_ENABLE();
            __HAL_RCC_GPIOE_CLK_ENABLE();

            gpiox spi1mosi("spi4mosi");
            spi1mosi.init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI4);
            gpiox spi1miso("spi4miso");
            spi1miso.init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI4);
            gpiox spi1sck("spi4sck");
            spi1sck.init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_2, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI4);
        }
    },&spixHandle);
    gpiox* imu2acs = new gpiox("imu2acs");
    imu2acs->init([](bool b){if(b){__HAL_RCC_GPIOC_CLK_ENABLE();}},GPIOC,GPIO_PIN_13,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);
    gpiox* imu2gcs = new gpiox("imu2gcs");
    imu2gcs->init([](bool b){if(b){__HAL_RCC_GPIOC_CLK_ENABLE();}},GPIOC,GPIO_PIN_2,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);
    imu2acs->setLevel(mDev::mGpio::LEVEL_HIGH);
    imu2gcs->setLevel(mDev::mGpio::LEVEL_HIGH);

    Bmi088* imu2 = new Bmi088("imu2",*spi4,*imu2acs,*imu2gcs);
    imu2->begin();
    return 0;
}
INIT_EXPORT(initAllDevice, "1");

extern "C" void EXTI0_IRQHandler(void)
{

}
#else
int initAllDevice()
{
    ledx* led0 = new ledx("led0");
    led0->init([](bool benable){ if(benable) __HAL_RCC_GPIOE_CLK_ENABLE(); },GPIOE,GPIO_PIN_3);
    ledx* led1 = new ledx("led1");
    led1->init([](bool benable){ if(benable) __HAL_RCC_GPIOE_CLK_ENABLE(); },GPIOE,GPIO_PIN_4);

    //TIM1 INIT
    TIM_HandleTypeDef timerst;
    memset(&timerst, 0, sizeof(TIM_HandleTypeDef));
    timerst.Instance = TIM1;
    timerst.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    timerst.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timerst.Init.CounterMode = TIM_COUNTERMODE_UP;
    timerst.Init.RepetitionCounter = 0;

    timer1 = new timerx("timer1");
    timer1->calcPeriodAndPrescalerByFreq(&timerst,1000);
    timer1->baseTimeInit([](bool b){
        if(b)
        {
            __HAL_RCC_TIM1_CLK_ENABLE();
            HAL_NVIC_SetPriority(TIM1_UP_IRQn, 3, 0);
            HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
        }
    }, &timerst,mDev::TIMESTARTMODE_IT);
    //TIM2 INIT
    memset(&timerst, 0, sizeof(TIM_HandleTypeDef));
    timerst.Instance = TIM2;
    timerst.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    timerst.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timerst.Init.CounterMode = TIM_COUNTERMODE_UP;
    timerst.Init.RepetitionCounter = 0;
    timer2 = new timerx("timer2");
    timer2->calcPeriodAndPrescalerByFreq(&timerst,1000);
    timer2->pwmTimeInit([](bool b){
        if(b)
        {
            __HAL_RCC_TIM2_CLK_ENABLE();
            gpiox bz("bz");
            bz.init([](bool b){if(b)__HAL_RCC_GPIOA_CLK_ENABLE();}, GPIOA, GPIO_PIN_15,GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF1_TIM2);//TIM2 CH1 AF1
            HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
            HAL_NVIC_EnableIRQ(TIM2_IRQn);
        }
    }, &timerst /*, mDev::TIMESTARTMODE::TIMESTARTMODE_IT*/);

    TIM_OC_InitTypeDef ocinit;
    ocinit.OCFastMode = TIM_OCFAST_DISABLE;
    ocinit.OCIdleState = TIM_OCIDLESTATE_SET;
    ocinit.OCMode = TIM_OCMODE_PWM1;
    ocinit.OCPolarity = TIM_OCPOLARITY_LOW;
    ocinit.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    ocinit.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    ocinit.Pulse = 0;
    
    //pmwtim2ch1 = new pwmx("pwmtim2ch1",timer2);
    timer2->pwmConfig(&ocinit, TIM_CHANNEL_1);

    spi1 = new spix("spi1");
    SPI_HandleTypeDef spixHandle;
    memset(&spixHandle, 0, sizeof(SPI_HandleTypeDef));
    spixHandle.Instance = SPI1;
    spixHandle.Init.Mode = SPI_MODE_MASTER;
    spixHandle.Init.Direction = SPI_DIRECTION_2LINES;
    spixHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    spixHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    spixHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spixHandle.Init.NSS = SPI_NSS_SOFT;
    spixHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    spixHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spixHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    spixHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spixHandle.Init.CRCPolynomial = 0x0;
    spixHandle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    spixHandle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    spixHandle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    spixHandle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_15CYCLE;
    spixHandle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_15CYCLE;
    spixHandle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    spixHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    spixHandle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    spi1->init([](bool b){
        if(b)
        {
            RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
            PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
            PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI1CLKSOURCE_CLKP;
            HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
            __HAL_RCC_SPI1_CLK_ENABLE();
            gpiox spi1mosi("spi1mosi");
            spi1mosi.init([](bool b){if(b)__HAL_RCC_GPIOA_CLK_ENABLE();},GPIOA, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI1);
            gpiox spi1miso("spi1miso");
            spi1miso.init([](bool b){if(b)__HAL_RCC_GPIOA_CLK_ENABLE();},GPIOA, GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI1);
            gpiox spi1sck("spi1sck");
            spi1sck.init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI1);
        }
    },&spixHandle);
    //CS
    gpiox* imu1cs = new gpiox("imu1cs");
    imu1cs->init([](bool b){if(b){__HAL_RCC_GPIOC_CLK_ENABLE();}},GPIOC,GPIO_PIN_15,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);
    mDev::mDevice* df42688 = new DFRobot_ICM42688_SPI("imu1", dynamic_cast<mDev::mGpio*>(imu1cs));
    //ICM42688_FIFO* icm42688x = new ICM42688_FIFO("icm42688",*spi1);
	//icm42688x->enableFifo(true, true, true);

    //SPI4 init
    spi4 = new spix("spi4");
    memset(&spixHandle, 0, sizeof(SPI_HandleTypeDef));
    spixHandle.Instance = SPI4;
    spixHandle.Init.Mode = SPI_MODE_MASTER;
    spixHandle.Init.Direction = SPI_DIRECTION_2LINES;
    spixHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    spixHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    spixHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    spixHandle.Init.NSS = SPI_NSS_SOFT;
    spixHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    spixHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spixHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    spixHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spixHandle.Init.CRCPolynomial = 0x0;
    spixHandle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    spixHandle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    spixHandle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    spixHandle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    spixHandle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_15CYCLE;
    spixHandle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_15CYCLE;
    spixHandle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    spixHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    spixHandle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    spi4->init([](bool b){
        if(b)
        {
            RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
            PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
            PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;
            HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
            __HAL_RCC_SPI4_CLK_ENABLE();
            __HAL_RCC_GPIOC_CLK_ENABLE();
            gpiox spi1mosi("spi4mosi");
            spi1mosi.init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_14, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI4);
            gpiox spi1miso("spi4miso");
            spi1miso.init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_13, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI4);
            gpiox spi1sck("spi4sck");
            spi1sck.init([](bool b){if(b)__HAL_RCC_GPIOE_CLK_ENABLE();},GPIOE, GPIO_PIN_12, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI4);
        }
    },&spixHandle);
    //CS
    gpiox* imu2cs = new gpiox("imu2cs");
    imu2cs->init([](bool b){if(b){__HAL_RCC_GPIOC_CLK_ENABLE();}},GPIOC,GPIO_PIN_13,GPIO_MODE_OUTPUT_PP,GPIO_PULLUP);
    mDev::mDevice* df42605 = new DFRobot_ICM42605_SPI("imu2",dynamic_cast<mDev::mGpio*>(imu2cs));
    //ICM42605_FIFO* icm42605x = new ICM42605_FIFO("icm42605",*spi4);
	//icm42605x->enableFifo(true, true, true);
    return 0;
}
INIT_EXPORT(initAllDevice, "1");
#endif
extern "C" void TIM1_UP_IRQHandler(void)
{
    timerx* timx = timer1;
    if(timx  != nullptr)
    {
        if(__HAL_TIM_GET_FLAG(timx->getTimHandle(), TIM_FLAG_UPDATE))
        {
            __HAL_TIM_CLEAR_FLAG(timx->getTimHandle(), TIM_FLAG_UPDATE);
            timx->runInterruptCb();
        }
    }
}

extern "C" void TIM2_IRQHandler(void)
{
    timerx* timx = timer2;
    if(timx != nullptr)
    {
        if(__HAL_TIM_GET_FLAG(timx->getTimHandle(), TIM_FLAG_UPDATE))
        {
            __HAL_TIM_CLEAR_FLAG(timx->getTimHandle(), TIM_FLAG_UPDATE);
            timx->runInterruptCb();
        }
        if(__HAL_TIM_GET_FLAG(timx->getTimHandle(), TIM_FLAG_CC1))
        {
            __HAL_TIM_CLEAR_FLAG(timx->getTimHandle(), TIM_FLAG_CC1);
        }
    }
}
