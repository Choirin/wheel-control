#ifndef __INCLUDE_EXPLORER_H__
#define __INCLUDE_EXPLORER_H__

#include "stm32f4xx_hal.h"

#include <stdbool.h>

#define FOWARD_SPEED           0.1
#define SPEED_ZERO             0.0001

void InitializeExplorer(void);
void ResetState(void);
bool StateActive(void);
void ExplorerStateControl(float *speed, float *target, uint16_t *distance, uint16_t *depth);

#endif