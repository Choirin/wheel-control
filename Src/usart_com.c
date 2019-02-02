

/* Includes ------------------------------------------------------------------*/
#include "usart_com.h"

#include "main.h"

#define TXBUFFERSIZE 64
#define RXBUFFERSIZE 64

UART_HandleTypeDef *huart;

uint8_t aTxBuffer[] = " ****UART_TwoBoards_ComIT****  ****UART_TwoBoards_ComIT****  ****UART_TwoBoards_ComIT**** ";
uint8_t aRxBuffer[RXBUFFERSIZE];

__IO ITStatus TxReady = RESET;
__IO ITStatus RxReady = RESET;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
  TxReady = SET;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
  RxReady = SET;
}

 void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
}

void init_usart_com(UART_HandleTypeDef *huart_)
{
  huart = huart_;
}

void sample_loop_back(void)
{
  int i;
  for (i = 0; i < RXBUFFERSIZE; ++i)
  {
    aRxBuffer[i] = 0x00;
  }

  if(HAL_UART_Transmit_IT(huart, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
  {
    Error_Handler();
  }

  if(HAL_UART_Receive_IT(huart, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
  {
    Error_Handler();
  }

  while (RxReady != SET)
  {
  }
  RxReady = RESET;

  printf((char *)aRxBuffer);

}