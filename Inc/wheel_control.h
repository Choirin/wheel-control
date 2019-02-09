#ifndef __INCLUDE_WHEEL_CONTROL_H__
#define __INCLUDE_WHEEL_CONTROL_H__
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define M_PI                    3.141592653589
#define MODEL_WHEEL_RADIUS      0.033
#define MODEL_WHEEL_CIRC_LEN    (2.0 * M_PI * MODEL_WHEEL_RADIUS)
#define MODEL_WHEEL_BASE        0.130
#define ENCODER_4_RESOLUTION    32.0
#define ENCODER_1_RESOLUTION    8.0
#define GEAR_RATIO              120.0

#define TIMER_CLOCK_MHZ         84.0
#define PRESCALER               168.0
#define COUNTER_CLOCK_HZ        (TIMER_CLOCK_MHZ * 1000000.0 / PRESCALER)

typedef struct
{
  float linear;
  float angular;
}TWIST;

void InitializeWheelControl(TIM_HandleTypeDef *htim_pwm_, TIM_HandleTypeDef *htim_enc0_, TIM_HandleTypeDef *htim_enc1_);

void SetEmergencyStop(uint8_t emergency_);
void SetTwistCommand(TWIST twist);
void SetTargetSpeed(float *target_);
void GetCurrentSpeed(float *speed_);
void MotorControl(void);

#endif