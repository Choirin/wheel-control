#include "time.h"

TIM_HandleTypeDef htim9;

unsigned long int time_uppercount = 0;
unsigned long int msec = 0;

void InitializeTime(void)
{
  HAL_TIM_Base_Start_IT(&htim9);
  return;
}


void SetTimeRate(unsigned long int msec)
{
  rate_msec = msec;
}

void GetTime(void)
{
  unsigned long int sec = time_uppercount;
  unsigned long int nsec = TIM9->CNT;
  nsec = (sec % 10 * 10000 + nsec) * 10000;
  sec = sec / 10;
  printf("%9lu.%09lu\r\n", sec, nsec);
  return;
}