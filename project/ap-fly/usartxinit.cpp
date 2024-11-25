#if 1
#include "usartx.hpp"
#include "gpio.hpp"
#include "stdio.h"

static usart* uart2 = nullptr;
/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
extern "C" void DMA1_Stream0_IRQHandler(void)
{
  if(uart2)
  {
    HAL_DMA_IRQHandler(uart2->dmaTxHandle());
  }
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
extern "C" void DMA1_Stream1_IRQHandler(void)
{
  if(uart2)
  {
    HAL_DMA_IRQHandler(uart2->dmaRxHandle());
  }
}

extern "C" void USART2_IRQHandler(void)
{
  if(uart2)
  {
        HAL_UART_IRQHandler(uart2->usartHandle());
  }
}
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    usart* usartx = containerof(huart, usart, _uartHandle);
    if(huart == usartx->usartHandle())
    {
        usartx->setTransferComplete(true);
    }
}
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    usart* usartx = containerof(huart, usart, _uartHandle);
    if(huart == usartx->usartHandle())
    {
        mDev::mUsart::usartData rxdata = {
          .data = usartx->getRxBuff(),
          .len = usart::RX_BUFF_LEN,
        };
        usartx->runInterruptCb(&rxdata);
        usartx->recvData(usartx->getRxBuff(),usart::RX_BUFF_LEN);
    }
}
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    usart* usartx = containerof(huart, usart, _uartHandle);
    if(huart == usartx->usartHandle())
    {
        mDev::mUsart::usartData rxdata = {
          .data = usartx->getRxBuff(),
          .len = Size,
        };
        usartx->runInterruptCb(&rxdata);
        usartx->recvData(usartx->getRxBuff(),usart::RX_BUFF_LEN);
    }
}
#if 0
#define TXBUFF_SZIE 128
#define TX_CACHE_SIZE 32 //(128/4)
uint8_t usart1_tx_buffer[TXBUFF_SZIE]; //串口1的DMA发送缓冲区

//串口1的DMA发送printf
void Debug_printf(const char *format, ...)
{
  if(!uart1)
  {
    return;
  }
	uint32_t length = 0;
	va_list args;
	va_start(args, format);

  while(!uart1->btransferComplete());
  uart1->setTransferComplete(false);
  SCB_CleanDCache_by_Addr((uint32_t *)usart1_tx_buffer,sizeof(usart1_tx_buffer)/4);
	length = vsnprintf((char*)usart1_tx_buffer, sizeof(usart1_tx_buffer), (char*)format, args);
  uart1->sendData(usart1_tx_buffer, length);
}
#endif

int initUsart()
{
    UART_HandleTypeDef huart2;
    DMA_HandleTypeDef hdma_usart2_tx;
    DMA_HandleTypeDef hdma_usart2_rx;
    uart2 = new usart("usart2");

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    hdma_usart2_tx.Instance = DMA1_Stream0;
    hdma_usart2_tx.Init.Request = DMA_REQUEST_USART2_TX;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_usart2_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart2_tx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usart2_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;

    /* USART1_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream1;
    hdma_usart2_rx.Init.Request = DMA_REQUEST_USART2_RX;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_usart2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart2_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usart2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    uart2->init([](bool enable){
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

      PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
      PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
      HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

        /* Peripheral clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();

        __HAL_RCC_USART1_CLK_ENABLE();
        gpiox usart2txpin("usart1tx");
        usart2txpin.init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM, GPIO_AF7_USART2);
        gpiox usart2rxpin("usart1rx");
        usart2rxpin.init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM, GPIO_AF7_USART2);
        uart2->txDmaInit();
        uart2->rxDmaInit();
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    },&huart2, &hdma_usart2_tx, &hdma_usart2_rx);
    uart2->setTransferMode(mDev::transferMode::TRANSFER_MODE_IT_RECV_IDLE);
    uart2->recvData(uart2->getRxBuff(),usart::RX_BUFF_LEN);
    return 0;
}
INIT_EXPORT(initUsart, "0.1");
#endif