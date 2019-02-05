#include "wheel_control.h"

#include "main.h"

#define saturate(a, limit)       ( \
  ((a) < -(limit))? -(limit): \
  ((limit) < (a))? (limit): (a))

#define MAX_NUMBER_16BIT         0xFFFF
#define MAX_INPUT_CAPTURE        10000
#define MIN_INPUT_CAPTURE        50
#define TIMER_PERIOD             16000.0
#define MAX_PWM_DUTY             999

#define PID_P_GAIN               4000.0
#define PID_I_GAIN               1000.0

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

uint32_t input_capture[2];
float speed[2];
float target[2];
float pid_i[2];

TIM_HandleTypeDef *htim_pwm;
TIM_HandleTypeDef *htim_ic[2];


void CalculateSpeed(void);
void CalculatePID(uint8_t num);
void SetPWM(uint32_t channel, unsigned int duty);

void InitializeWheelControl(TIM_HandleTypeDef *htim_pwm_, TIM_HandleTypeDef *htim_ic0_, TIM_HandleTypeDef *htim_ic1_)
{
  htim_pwm = htim_pwm_;
  htim_ic[0] = htim_ic0_;
  htim_ic[1] = htim_ic1_;

  if (HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(htim_pwm, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_Base_Start_IT(htim_ic[0]);
  HAL_TIM_IC_Start_IT(htim_ic[0], TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(htim_ic[1]);
  HAL_TIM_IC_Start_IT(htim_ic[1], TIM_CHANNEL_1);

  HAL_TIM_Encoder_Start(htim_ic[0],TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(htim_ic[1],TIM_CHANNEL_ALL);

  while (1)
  {
    printf("%ld, %ld", htim_ic[0]->Instance->CNT, htim_ic[1]->Instance->CNT);
    HAL_Delay(100);
  }
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
    if (MAX_INPUT_CAPTURE < input_capture[i])
      speed[i] = 0.0;
    else if (MIN_INPUT_CAPTURE < input_capture[i])
    {
      speed[i] = MODEL_WHEEL_CIRC_LEN * COUNTER_CLOCK_HZ / ENCODER_RESOLUTION / GEAR_RATION / input_capture[i];
    }
#if 1
    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(gpio_phase[i].port, gpio_phase[i].pin))
      speed[i] = -speed[i];
#else
    if (target[i] < -0.001)
      speed[i] = -speed[i];
#endif
  }
}

void MotorControl(void)
{
  CalculateSpeed();
  CalculatePID(0);
  CalculatePID(1);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance==TIM2)
  {
    input_capture[0] = __HAL_TIM_GetCompare(htim, TIM_CHANNEL_1);    //read TIM2 channel 1 capture value
    __HAL_TIM_SetCounter(htim, 0);    //reset counter after input capture interrupt occurs
  }
 if (htim->Instance==TIM3)
  {
    input_capture[1] = __HAL_TIM_GetCompare(htim, TIM_CHANNEL_1);    //read TIM2 channel 1 capture value
    __HAL_TIM_SetCounter(htim, 0);    //reset counter after input capture interrupt occurs
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if (htim->Instance==TIM2)
  {
    input_capture[0] = MAX_NUMBER_16BIT;
  }
 if (htim->Instance==TIM3)
  {
    input_capture[1] = MAX_NUMBER_16BIT;
  }
}