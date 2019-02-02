#include <stdbool.h>
#include "stm32f4xx_hal.h"

void InitializeWheelControl(TIM_HandleTypeDef *htim_pwm_, TIM_HandleTypeDef *htim_ic0_, TIM_HandleTypeDef *htim_ic1_);

void SetEmergencyStop(uint8_t emergency_);
void SetTargetSpeed(float *target_);
void GetCurrentSpeed(float *speed_);
void MotorControl(void);
