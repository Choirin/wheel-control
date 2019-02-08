#include <stdbool.h>

#include "wheel_control.h"
#include "main.h"

#define saturate(a, limit)       ( \
  ((a) < -(limit))? -(limit): \
  ((limit) < (a))? (limit): (a))

#define MAX_NUMBER_16BIT         0xFFFF
#define MAX_INPUT_CAPTURE        0xFFFF
#define MIN_INPUT_CAPTURE        100
#define MAX_PWM_DUTY             999

#define PID_P_GAIN               50.0
#define PID_I_GAIN               100.0

typedef struct{
  GPIO_TypeDef *port;
  uint16_t pin;
}GPIO_TYPE;

GPIO_TYPE gpio_phase[2] = {
  {GPIOA, GPIO_PIN_11},
  {GPIOA, GPIO_PIN_12}
};
uint32_t timer_channel[2] = {
  TIM_CHANNEL_2,
  TIM_CHANNEL_1
};

bool emergency = false;

int32_t upper_count[2];
uint16_t last_count[2];

bool ccw[2] = {false, false};
uint32_t input_capture[2] = {0, 0};
float speed[2];
float target[2];
float pid_i[2];

TIM_HandleTypeDef *htim_pwm;
TIM_HandleTypeDef *htim_enc[2];
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;


void CalculateSpeed(void);
void CalculatePID(uint8_t num);
void SetPWM(uint32_t channel, unsigned int duty);

void InitializeWheelControl(TIM_HandleTypeDef *htim_pwm_, TIM_HandleTypeDef *htim_enc0_, TIM_HandleTypeDef *htim_enc1_)
{
  htim_pwm = htim_pwm_;
  htim_enc[0] = htim_enc0_;
  htim_enc[1] = htim_enc1_;

  if (HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /*
  HAL_TIM_Base_Start_IT(htim_enc[0]);
  HAL_TIM_IC_Start_IT(htim_enc[0], TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(htim_enc[1]);
  HAL_TIM_IC_Start_IT(htim_enc[1], TIM_CHANNEL_1);
  */

  HAL_TIM_Encoder_Start(htim_enc[0],TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(htim_enc[1],TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
#if 0
  while (1)
  {
    printf("%ld, %ld, %ld, %ld\n", htim_enc[0]->Instance->CNT, htim_enc[1]->Instance->CNT, TIM4->CCR1, TIM5->CCR1);
    HAL_Delay(100);
  }
#endif
}

#if 0
float saturate(float a, float limit)
{
  if (a < -limit)
    a = -limit
  else if(limit < a)
    a = limit
  return a;
}
#endif


void SetTwistCommand(TWIST twist)
{
  target[0] = twist.linear + twist.angular * MODEL_WHEEL_BASE / 2.0;
  target[1] = twist.linear - twist.angular * MODEL_WHEEL_BASE / 2.0;
  //printf("target: %d, %d\n", (int)(target[0] * 1000.0), (int)(target[1] * 1000.0));
}


void SetTargetSpeed(float *target_)
{
  target[0] = target_[0];
  target[1] = target_[1];
}


void GetCurrentSpeed(float *speed_)
{
  speed_[0] = speed[0];
  speed_[1] = speed[1];
}


void SetPWM(uint32_t channel, unsigned int duty)
{
  __HAL_TIM_SET_COMPARE(htim_pwm, channel, duty);
}


void CalculatePID(uint8_t num)
{
  float p;
  float duty;

  p = target[num] - speed[num];
  pid_i[num] = pid_i[num] + p;
  duty = PID_P_GAIN * p + PID_I_GAIN * pid_i[num];
  duty = saturate(duty, MAX_PWM_DUTY);

  if (emergency == true)
  {
    pid_i[num] = 0;
    duty = 0;
  }
  if (duty < 0)
  {
    HAL_GPIO_WritePin(gpio_phase[num].port, gpio_phase[num].pin, SET);
    duty = -duty;
  }
  else
  {
    HAL_GPIO_WritePin(gpio_phase[num].port, gpio_phase[num].pin, RESET);
  }
  SetPWM(timer_channel[num], (int)duty);
}

void SetEmergencyStop(uint8_t emergency_)
{
  emergency = emergency_;
}

void CalculateSpeed(void)
{
  int i;
  //printf("%ld\n", input_capture[1]);
  for (i = 0; i < 2; ++i)
  {
    if (MAX_INPUT_CAPTURE <= input_capture[i])
      speed[i] = 0.0;
    else if (MIN_INPUT_CAPTURE < input_capture[i])
    {
      speed[i] = MODEL_WHEEL_CIRC_LEN * COUNTER_CLOCK_HZ / ENCODER_RESOLUTION / GEAR_RATION / input_capture[i];
    }
    if (ccw[i])
      speed[i] = -speed[i];
  }

#if 0
  static uint16_t count = 0;
  //if (count++ > 5)
  {
    //printf("%ld, %ld, %ld, %ld\n", htim_enc[0]->Instance->CNT, TIM4->CCR1, input_capture[0], (long)(100.0 * speed[0]));
    printf("%ld, %ld, %ld, %ld\n", htim_enc[1]->Instance->CNT, TIM5->CCR1, input_capture[1], (long)(100.0 * speed[1]));
    count = 0;
  }
#endif
}

void MotorControl(void)
{
  CalculateSpeed();
  CalculatePID(0);
  CalculatePID(1);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance==TIM4)
  {
    bool ccw_ = false;
    uint16_t count = htim_enc[0]->Instance->CNT;
    if (last_count[0] < count)
    {
      ccw_ = false;
      if (count - last_count[0] > 32767)
      {
        ccw_ = true;
        upper_count[0]--;
      }
    }else if (last_count[0] > count)
    {
      ccw_ = true;
      if (last_count[0] - count > 32767)
      {
        ccw_ = false;
        upper_count[0]++;
      }
    }
    last_count[0] = count;
    ccw[0] = ccw_;
    input_capture[0] = __HAL_TIM_GetCompare(htim, TIM_CHANNEL_1);    //read TIM2 channel 1 capture value
    __HAL_TIM_SetCounter(htim, 0);    //reset counter after input capture interrupt occurs
  }
 if (htim->Instance==TIM5)
  {
    bool ccw_ = false;
    uint16_t count = htim_enc[1]->Instance->CNT;
    if (last_count[1] < count)
    {
      ccw_ = false;
      if (count - last_count[1] > 32767)
      {
        ccw_ = true;
        upper_count[1]--;
      }
    }else if (last_count[1] > count)
    {
      ccw_ = true;
      if (last_count[1] - count > 32767)
      {
        ccw_ = false;
        upper_count[1]++;
      }
    }
    last_count[1] = count;
    ccw[1] = !ccw_;
    input_capture[1] = __HAL_TIM_GetCompare(htim, TIM_CHANNEL_1);    //read TIM2 channel 1 capture value
    __HAL_TIM_SetCounter(htim, 0);    //reset counter after input capture interrupt occurs
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance==TIM4)
  {
    input_capture[0] = MAX_NUMBER_16BIT;
  }
 if (htim->Instance==TIM5)
  {
    input_capture[1] = MAX_NUMBER_16BIT;
  }
}