#include "time.h"

TIM_HandleTypeDef htim9;

unsigned long int time_uppercount = 0;

void InitializeTime(void)
{
  HAL_TIM_Base_Start_IT(&htim9);
  return;
}

void GetTime(void)
{
  printf("%lu:%d\r\n", time_uppercount, TIM9->CNT);
  return;
}