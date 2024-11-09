#include "i2c.hpp"
#include "gpio.hpp"
static  i2cx* i2c4 = nullptr;
static  i2cx* i2c1 = nullptr;


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


int i2cInit()
{
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

    I2C_Handle.Instance = I2C1;
    I2C_Handle.Init.Timing           = i2cx::i2cClockTIMINGR(i2cx::getClockFreq(I2C1),2000,0);//0x307075B1;//100KHz
    I2C_Handle.Init.OwnAddress1      = 0;
    I2C_Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2C_Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2C_Handle.Init.OwnAddress2      = 0;
    I2C_Handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    I2C_Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2C_Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    i2c1 = new i2cx("i2c1");
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

    return 0;
}
INIT_EXPORT(i2cInit, "0.4");