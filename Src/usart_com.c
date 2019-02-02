#include "usart_com.h"

#include "main.h"

#define TXBUFFERSIZE 64
#define RXBUFFERSIZE 64

UART_HandleTypeDef *huart;

uint8_t aTxBuffer1[] = " ****UART_TwoBoards_ComIT****  ****UART_TwoBoards_ComIT****  *\n\0";
uint8_t aTxBuffer2[] = "asdlgasgioasndclamnwietgnaosgjlasdjfkashgiabnsioldfja;lsjclash\n\0";
uint8_t aRxBuffer[RXBUFFERSIZE];
uint16_t read_pos = 0;

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

void InitializeUsartCom(UART_HandleTypeDef *huart_)
{
  huart = huart_;
  if(HAL_UART_Receive_DMA(huart, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
  {
    Error_Handler();
  }
}

uint16_t ReadBuffer(uint8_t *ptr)
{
  uint16_t size = 0;
  uint16_t write_pos = RXBUFFERSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
  while (write_pos != read_pos)
  {
    *(ptr++) = aRxBuffer[read_pos++];
    read_pos &= (RXBUFFERSIZE - 1);
    size++;
  }
  return size;
}

void sample_loop_back(void)
{
  uint16_t size;
  uint8_t buffer[RXBUFFERSIZE];

  size = ReadBuffer(buffer);
  buffer[size] = '\0';
  printf((char *)buffer);

#if 0
  while (HAL_UART_GetState(huart) != HAL_UART_STATE_READY)
  {
  }
#endif

  static int toggle = 0;
  uint8_t *ptr;
  if (toggle == 0)
  {
    ptr = aTxBuffer1;
    toggle = 1;
  }
  else
  {
    ptr = aTxBuffer2;
    toggle = 0;
  }
  
  if(HAL_UART_Transmit_IT(huart, (uint8_t*)ptr, TXBUFFERSIZE)!= HAL_OK)
  {
    Error_Handler();
  }
  while (TxReady == RESET);
  TxReady = RESET;

  printf((char *)aRxBuffer);

}