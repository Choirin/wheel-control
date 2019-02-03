#ifndef __INCLUDE_USART_COM_H__
#define __INCLUDE_USART_COM_H__

#include "stm32f4xx_hal.h"

#include "wheel_control.h"

#define MARKER_COMMAND_TWIST_H     0xFF
#define MARKER_COMMAND_TWIST_L     0xAB

#define MARKER_SPEED_H             0xFF
#define MARKER_SPEED_L             0xAC

typedef struct{
  uint8_t  marker[2];           //  2 bytes: 0xFF, 0xAB
  uint8_t  size;                //  1 bytes
  uint8_t  reserved;            //  1 bytes
  TWIST    twist;               //  8 bytes
  uint8_t  sum;                 //  1 bytes
  uint8_t  reserved2[3];        //  3 bytes
}PACKET_TWIST_COMMAND;

typedef struct{
  uint8_t  marker[2];           //  2 bytes: 0xFF, 0xAC
  uint8_t  size;                //  1 bytes
  uint8_t  reserved;            //  1 bytes
  float    speed[2];            //  8 bytes
  uint8_t  sum;                 //  1 bytes
  uint8_t  reserved2[3];        //  3 bytes
}PACKET_SPEED;

void InitializeUsartCom(UART_HandleTypeDef * huart_);
bool ParseProcess(TWIST *twist_command);
void sample_loop_back(void);

void SendTwistCommand(TWIST twist_command);

#endif