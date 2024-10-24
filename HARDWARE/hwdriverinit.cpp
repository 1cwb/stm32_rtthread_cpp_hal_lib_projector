#include "stm32h7xx_hal_conf.h"
#include "spi.hpp"
#include "led.hpp"
#include "DFRobot_ICM42688.h"
#include "DFRobot_ICM42605.h"
#include "mdevice.hpp"
#include "containers.hpp"
//#include "mgpiodrv.hpp"
#include "gpio.hpp"
#include "mplatform.hpp"
#include "timer.hpp"
#include "delay.h"
#include "mplatform.hpp"
#include "i2c.hpp"
#include "qmc5883.hpp"
#include "spl06.hpp"
#include "workqueue.hpp"
#include "workqueuemanager.hpp"
#include "systick.hpp"
#include "bmi088new.hpp"

timerx* timer1 = nullptr;
timerx* timer2 = nullptr;
spix* spi1 = nullptr;
spix* spi4 = nullptr;
gpiox* pd12 = nullptr;
i2cx* i2c4 = nullptr;
QMC5883LCompass* qmc5883l = nullptr;
#if 1
int initAllDevice()
{
    #if 1
    systick* msystick = new systick;
    if(msystick)
    {
        msystick->init();//do no thing
    }
    #endif

    gpiox* pd8 = new gpiox("pd8");
    pd8->init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_8, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    gpiox* pd9 = new gpiox("pd9");
    pd9->init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    pd8->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
    pd9->setLevel(mDev::mGpio::GPIOLEVEL::LEVEL_LOW);
    ledx* led0 = new ledx("led0");
    led0->init([](bool benable){ if(benable) __HAL_RCC_GPIOD_CLK_ENABLE(); },GPIOD,GPIO_PIN_15);
    ledx* led1 = new ledx("led1");
    led1->init([](bool benable){ if(benable) __HAL_RCC_GPIOB_CLK_ENABLE(); },GPIOB,GPIO_PIN_15);
    ledx* led2 = new ledx("led2");
    led2->init([](bool benable){ if(benable) __HAL_RCC_GPIOD_CLK_ENABLE(); },GPIOD,GPIO_PIN_11);

    I2C_HandleTypeDef I2C_Handle = {0};
    memset(&I2C_Handle, 0, sizeof(I2C_Handle));
    /* I2C 配置 */
    I2C_Handle.Instance = I2C4;
    I2C_Handle.Init.Timing           = i2cx::i2cClockTIMINGR(i2cx::getClockFreq(I2C4),2000,0);//0x307075B1;//100KHz
    I2C_Handle.Init.OwnAddress1      = 0;
    I2C_Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2C_Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2C_Handle.Init.OwnAddress2      = 0;
    I2C_Handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    I2C_Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2C_Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    i2c4 = new i2cx("i2c4");
    i2c4->init([&](bool b){
        if(b)
        {
            __HAL_RCC_I2C4_CLK_ENABLE();
            gpiox i2c4scl("i2c4scl");
            i2c4scl.init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_12, GPIO_MODE_AF_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_AF4_I2C4);
            gpiox i2c4sda("i2c4sda");
            i2c4sda.init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_13, GPIO_MODE_AF_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_AF4_I2C4);
            #if 0
            HAL_NVIC_SetPriority(I2C4_EV_IRQn, 3, 0);
            HAL_NVIC_EnableIRQ(I2C4_EV_IRQn);
            HAL_NVIC_SetPriority(I2C4_ER_IRQn, 3, 0);
            HAL_NVIC_EnableIRQ(I2C4_ER_IRQn);
            #endif
        }
    },&I2C_Handle);
    qmc5883l = new QMC5883LCompass("mag1",i2c4);
    qmc5883l->init();

    I2C_Handle.Instance = I2C1;
    I2C_Handle.Init.Timing           = i2cx::i2cClockTIMINGR(i2cx::getClockFreq(I2C1),2000,0);//0x307075B1;//100KHz
    I2C_Handle.Init.OwnAddress1      = 0;
    I2C_Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2C_Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2C_Handle.Init.OwnAddress2      = 0;
    I2C_Handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    I2C_Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2C_Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    i2cx* i2c1 = new i2cx("i2c1");
    i2c1->init([&](bool b){
        if(b)
        {
            __HAL_RCC_I2C1_CLK_ENABLE();
            gpiox i2c4scl("i2c1scl");
            i2c4scl.init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_6, GPIO_MODE_AF_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_AF4_I2C1);
            gpiox i2c4sda("i2c1sda");
            i2c4sda.init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_7, GPIO_MODE_AF_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, GPIO_AF4_I2C1);
            gpiox i22d("i22d");
            i22d.init([](bool b){if(b)__HAL_RCC_GPIOB_CLK_ENABLE();},GPIOB, GPIO_PIN_5, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
            i22d.setLevel(mDev::mGpio::LEVEL_HIGH);
        }
    },&I2C_Handle);

    ArtronShop_SPL06_001* barometor = new ArtronShop_SPL06_001("baro1", i2c1);
    barometor->begin();

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

    bmi088* imu1 = new bmi088("imu1",spi1,imu1acs,imu1gcs);
    imu1->init();

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

    bmi088* imu2 = new bmi088("imu2",spi4,imu2acs,imu2gcs);
    imu2->init();

    return 0;
}
INIT_EXPORT(initAllDevice, "1");

extern "C" void EXTI0_IRQHandler(void)
{

}
extern "C" void EXTI1_IRQHandler(void)
{

}
#if 0
extern "C" void EXTI15_10_IRQHandler(void)
{
    printf("data-ready\r\n");
    if(pd12)
    {
        if(__HAL_GPIO_EXTI_GET_IT(pd12->getPin()) != 0X00)
        {
            __HAL_GPIO_EXTI_CLEAR_IT(pd12->getPin());
            
        }
    }
}
#endif
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
#if 0
extern "C" void I2C4_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(i2c4->i2cxHandle());
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
extern "C" void I2C4_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(i2c4->i2cxHandle());
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}
extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(qmc5883l)
    {
        qmc5883l->runInterruptCb();
    }
}

extern "C" void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    printf("tony xxxxxxxxxxxxxxxxxx\r\n");
}
#endif
