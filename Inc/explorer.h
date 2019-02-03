#ifndef __INCLUDE_EXPLORER_H__
#define __INCLUDE_EXPLORER_H__

#include "stm32f4xx_hal.h"

void InitializeExplorer(void);
void ExplorerStateControl(float *speed, float *target, uint16_t *distance, uint16_t *depth);

#endif