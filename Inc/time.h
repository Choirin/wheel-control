#ifndef __INCLUDE_TIME_H__
#define __INCLUDE_TIME_H__

#include "stm32f4xx_hal.h"

#include <stdbool.h>

void InitializeTime(void);
void TimeSetRate(uint16_t msec);
bool TimeRateSleep(void);

#endif