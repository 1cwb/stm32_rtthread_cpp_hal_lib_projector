#include "usartx.hpp"
#include "gpio.hpp"

static usart* uart1 = nullptr;
uint8_t btxcpl = 1;
/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
extern "C" void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  if(uart1)
  {
    HAL_DMA_IRQHandler(uart1->dmaTxHandle());
  }
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
extern "C" void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  if(uart1)
  {
    HAL_DMA_IRQHandler(uart1->dmaRxHandle());
  }
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
extern "C" void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  if(uart1)
  {
        HAL_UART_IRQHandler(uart1->usartHandle());
  }
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    usart* usartx = containerof(huart, usart, _uartHandle);
    if(huart == usartx->usartHandle())
    {
        usartx->setTransferComplete(true);
        usartx->runInterruptCb(nullptr);
        btxcpl = 1;
    }
}


uint8_t usart1_tx_buffer[128]; //串口1的DMA发送缓冲区

//串口1的DMA发送printf
void Debug_printf(const char *format, ...)
{
	uint32_t length = 0;
	va_list args;
	va_start(args, format);

	while(btxcpl == 0);
	length = vsnprintf((char*)usart1_tx_buffer, sizeof(usart1_tx_buffer), (char*)format, args);

	((mDev::mUsart*)mDev::mPlatform::getInstance()->getDevice("usart1"))->sendData(usart1_tx_buffer, length);
	btxcpl = 0;
}

int initUsart()
{
    UART_HandleTypeDef huart1;
    DMA_HandleTypeDef hdma_usart1_tx;
    DMA_HandleTypeDef hdma_usart1_rx;
    uart1 = new usart("usart1");

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Stream0;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Stream1;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    uart1->init([](bool enable){
          /* DMA controller clock enable */
        __HAL_RCC_DMA1_CLK_ENABLE();

        /* DMA interrupt init */
        /* DMA1_Stream0_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
        /* DMA1_Stream1_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
        RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
        __HAL_RCC_USART1_CLK_ENABLE();
        gpiox usart1txpin("usart1tx");
        usart1txpin.init([](bool b){if(b)__HAL_RCC_GPIOA_CLK_ENABLE();},GPIOA, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM, GPIO_AF7_USART1);
        gpiox usart1rxpin("usart1rx");
        usart1rxpin.init([](bool b){if(b)__HAL_RCC_GPIOA_CLK_ENABLE();},GPIOA, GPIO_PIN_10, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM, GPIO_AF7_USART1);
        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    
    },&huart1, &hdma_usart1_tx, &hdma_usart1_rx);
    uart1->setTransferMode(mDev::transferMode::TRANSFER_MODE_DMA);
while (1)
{
  HAL_Delay(200);
 Debug_printf("HELLOW WORLD\r\n");
}

    return 0;
}
INIT_EXPORT(initUsart, "0.2");