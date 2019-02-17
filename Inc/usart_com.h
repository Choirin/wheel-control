#ifndef __INCLUDE_USART_COM_H__
#define __INCLUDE_USART_COM_H__

#include "stm32f4xx_hal.h"

#include "wheel_control.h"

#define MARKER_COMMAND_TWIST_H     0xFF
#define MARKER_COMMAND_TWIST_L     0xAB

typedef struct{
  uint8_t  marker[2];           //  2 bytes: 0xFF, 0xAB
  uint8_t  size;                //  1 bytes
  uint8_t  reserved;            //  1 bytes
  TWIST    twist;               //  8 bytes
  uint8_t  sum;                 //  1 bytes
  uint8_t  reserved2[3];        //  3 bytes
}PACKET_TWIST_COMMAND;


#define MARKER_SPEED_H             0xFF
#define MARKER_SPEED_L             0xAC

typedef struct{
  uint8_t  marker[2];           //  2 bytes: 0xFF, 0xAC
  uint8_t  size;                //  1 bytes
  uint8_t  reserved;            //  1 bytes
  float    speed[2];            //  8 bytes
  uint8_t  sum;                 //  1 bytes
  uint8_t  reserved2[3];        //  3 bytes
}PACKET_SPEED;


#define MARKER_SENSOR_H            0xFF
#define MARKER_SENSOR_L            0xAD

typedef struct{
  uint8_t  marker[2];           //  2 bytes: 0xFF, 0xAD
  uint8_t  size;                //  1 bytes
  uint8_t  reserved;            //  1 bytes
  uint16_t value[6];            // 12 bytes
  uint8_t  sum;                 //  1 bytes
  uint8_t  reserved2[3];        //  3 bytes
}PACKET_SENSOR;


#define MARKER_TWIST_H             0xFF
#define MARKER_TWIST_L             0xAE

typedef struct{
  uint8_t  marker[2];           //  2 bytes: 0xFF, 0xAE
  uint8_t  size;                //  1 bytes
  uint8_t  reserved;            //  1 bytes
  TWIST    twist;               //  8 bytes
  uint8_t  sum;                 //  1 bytes
  uint8_t  reserved2[3];        //  3 bytes
}PACKET_TWIST;


void InitializeUsartCom(UART_HandleTypeDef * huart_);
bool ParseProcess(TWIST *twist_command);
void sample_loop_back(void);

void SendTwistCommand(TWIST twist_command);
void SendSpeed(float *speed);
void SendSensor(uint16_t *value);
void SendTwist(TWIST twist);

#endif