#include "usart_com.h"

#include "main.h"

#define TXBUFFERSIZE 128
#define RXBUFFERSIZE 64

UART_HandleTypeDef *huart;

static const uint8_t aTxBuffer[] = "once upon a time, one old programmer lives in a small castle. he writes c program everyday and night. write until 128 chars.  \n\0";
static const uint16_t block_size = 16;
static uint16_t tx_pos = 0;

static uint8_t usart_com_tx_buffer[TXBUFFERSIZE];

uint8_t aRxBuffer[RXBUFFERSIZE];
static uint16_t read_pos = 0;

__IO ITStatus TxReady = SET;
__IO ITStatus RxReady = SET;

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

  tx_pos = 0;
  read_pos = 0;
}


void UartCom_Send(uint8_t *ptr, uint16_t size)
{
#if 0
  HAL_UART_Transmit(huart, (uint8_t *)&packet, (uint16_t)packet.size, 0xFFFF);
#else
  if(HAL_UART_Transmit_IT(huart, ptr, size)!= HAL_OK)
  {
    Error_Handler();
  }
#endif
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

void dump(uint8_t *ptr, uint16_t size)
{
  uint16_t i;
  for (i = 0; i < size; ++i, ++ptr)
  {
    printf("%02x ", *ptr);
    if ((i + 1) % 16 == 0)
      printf("\n");
    else if ((i + 1) % 8 == 0)
      printf(" | ");
  }
}

bool ParseProcess(TWIST *twist_command)
{
  static uint8_t buffer[RXBUFFERSIZE];
  static uint16_t buf_pos = 0;

  buf_pos &= (RXBUFFERSIZE - 1);
  buf_pos += ReadBuffer(&buffer[buf_pos]);
  if (buf_pos < 3)
    return false;
  if ((buffer[0] == MARKER_COMMAND_TWIST_H) && (buffer[1] == MARKER_COMMAND_TWIST_L))
  {
    PACKET_TWIST_COMMAND *packet = (PACKET_TWIST_COMMAND *)buffer;
    if (packet->size == sizeof(PACKET_TWIST_COMMAND))
    {
      *twist_command = packet->twist;
      return true;
    }
  }

  buf_pos = 0;
  return false;
}


void SendTwistCommand(TWIST twist_command)
{
  PACKET_TWIST_COMMAND *packet = (PACKET_TWIST_COMMAND *)usart_com_tx_buffer;
  while (TxReady == RESET);
  TxReady = RESET;

  packet->marker[0] = MARKER_COMMAND_TWIST_H;
  packet->marker[1] = MARKER_COMMAND_TWIST_L;
  packet->size      = sizeof(PACKET_TWIST_COMMAND);
  packet->twist     = twist_command;

  //dump((uint8_t*)&packet, packet->size);

  UartCom_Send((uint8_t *)packet, packet->size);
}


void SendSpeed(float *speed)
{
  PACKET_SPEED *packet = (PACKET_SPEED *)usart_com_tx_buffer;
  while (TxReady == RESET);
  TxReady = RESET;

  packet->marker[0] = MARKER_SPEED_H;
  packet->marker[1] = MARKER_SPEED_L;
  packet->size      = sizeof(PACKET_SPEED);
  packet->speed[0]  = speed[0];
  packet->speed[1]  = speed[1];

  UartCom_Send((uint8_t *)packet, packet->size);
}


void SendSensor(uint16_t *value)
{
  PACKET_SENSOR *packet = (PACKET_SENSOR *)usart_com_tx_buffer;
  uint16_t i;
  while (TxReady == RESET);
  TxReady = RESET;

  packet->marker[0] = MARKER_SENSOR_H;
  packet->marker[1] = MARKER_SENSOR_L;
  packet->size      = sizeof(PACKET_SENSOR);
  for (i = 0; i < 6; ++i)
    packet->value[i]  = value[i];

  UartCom_Send((uint8_t *)packet, packet->size);
}


void sample_loop_back(void)
{
  uint16_t size;
  uint8_t buffer[RXBUFFERSIZE];

  // Reading
  size = ReadBuffer(buffer);
  buffer[size] = '\0';
  printf("\na: %d\n", tx_pos);
  printf((char *)buffer);

  // Sending
  // Wait last sending
  while (TxReady == RESET);
  TxReady = RESET;
  if(HAL_UART_Transmit_IT(huart, (uint8_t*)&aTxBuffer[tx_pos], block_size)!= HAL_OK)
  {
    Error_Handler();
  }

  tx_pos += block_size;
  tx_pos &= (TXBUFFERSIZE - 1);
}