#if 1
#include "usartx.hpp"
#include "gpio.hpp"
#include "stdio.h"
/*uart2*/
static usart* uart2 = nullptr;
static usart* uart3 = nullptr;
static usart* uart4 = nullptr;
static usart* uart5 = nullptr;
static usart* uart6 = nullptr;
static usart* uart8 = nullptr;
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
/*****************************************INTERRUPT BEGIN*******************/
extern "C" void USART2_IRQHandler(void)
{
  if(uart2)
  {
        HAL_UART_IRQHandler(uart2->usartHandle());
  }
}
extern "C" void USART3_IRQHandler(void)
{
  if(uart3)
  {
        HAL_UART_IRQHandler(uart3->usartHandle());
  }
}
extern "C" void USART4_IRQHandler(void)
{
  if(uart4)
  {
        HAL_UART_IRQHandler(uart4->usartHandle());
  }
}
extern "C" void USART5_IRQHandler(void)
{
  if(uart5)
  {
        HAL_UART_IRQHandler(uart5->usartHandle());
  }
}
extern "C" void USART6_IRQHandler(void)
{
  if(uart6)
  {
        HAL_UART_IRQHandler(uart6->usartHandle());
  }
}
extern "C" void USART8_IRQHandler(void)
{
  if(uart8)
  {
        HAL_UART_IRQHandler(uart8->usartHandle());
  }
}
/*****************************************INTERRUPT END*******************/
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
    UART_HandleTypeDef huartX;
    DMA_HandleTypeDef hdma_usartx_tx;
    DMA_HandleTypeDef hdma_usartx_rx;
    uart2 = new usart("usart2");
    uart3 = new usart("usart3");
    uart4 = new usart("usart4");
    uart5 = new usart("usart5");
    uart6 = new usart("usart6");
    uart8 = new usart("usart8");

    huartX.Instance = USART2;
    huartX.Init.BaudRate = 115200;
    huartX.Init.WordLength = UART_WORDLENGTH_8B;
    huartX.Init.StopBits = UART_STOPBITS_1;
    huartX.Init.Parity = UART_PARITY_NONE;
    huartX.Init.Mode = UART_MODE_TX_RX;
    huartX.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huartX.Init.OverSampling = UART_OVERSAMPLING_16;
    huartX.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huartX.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huartX.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    hdma_usartx_tx.Instance = DMA1_Stream0;
    hdma_usartx_tx.Init.Request = DMA_REQUEST_USART2_TX;
    hdma_usartx_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usartx_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usartx_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usartx_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usartx_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usartx_tx.Init.Mode = DMA_NORMAL;
    hdma_usartx_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usartx_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_usartx_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_usartx_tx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usartx_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;

    /* USART1_RX Init */
    hdma_usartx_rx.Instance = DMA1_Stream1;
    hdma_usartx_rx.Init.Request = DMA_REQUEST_USART2_RX;
    hdma_usartx_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usartx_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usartx_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usartx_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usartx_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usartx_rx.Init.Mode = DMA_NORMAL;
    hdma_usartx_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usartx_rx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_usartx_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_usartx_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usartx_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;
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
        gpiox usart2txpin("usart2tx");
        usart2txpin.init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM, GPIO_AF7_USART2);
        gpiox usart2rxpin("usart2rx");
        usart2rxpin.init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM, GPIO_AF7_USART2);
        uart2->txDmaInit();
        uart2->rxDmaInit();
        HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    },&huartX, &hdma_usartx_tx, &hdma_usartx_rx);
    uart2->setTransferMode(mDev::transferMode::TRANSFER_MODE_DMA);
    uart2->setRecvMode(mDev::recvMode::RECV_MODE_DMA_RECV_IDLE);
    uart2->recvData(uart2->getRxBuff(),usart::RX_BUFF_LEN);

    huartX.Instance = USART3;
    uart3->init([](bool enable){

        /* Peripheral clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();
        gpiox usart3txpin("usart3tx");
        usart3txpin.init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_8, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM, GPIO_AF7_USART3);
        gpiox usart3rxpin("usart3rx");
        usart3rxpin.init([](bool b){if(b)__HAL_RCC_GPIOD_CLK_ENABLE();},GPIOD, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM, GPIO_AF7_USART3);
        //uart3->txDmaInit();
        //uart3->rxDmaInit();
        HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    },&huartX, nullptr, nullptr);
    uart3->setTransferMode(mDev::transferMode::TRANSFER_MODE_NOMAL);
    uart3->setRecvMode(mDev::recvMode::RECV_MODE_IT_RECV_IDLE);
    uart3->recvData(uart3->getRxBuff(),usart::RX_BUFF_LEN);

    return 0;
}
INIT_EXPORT(initUsart, "0.1");
#endif