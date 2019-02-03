#include "explorer.h"

#include "wheel_control.h"

#define PRINT_DEBUG

enum STATE {
  Foward,
  Backward,
  Stop,
  LeftTurn,
  RightTurn,
  Wait,
};

static const int count_limit[] = {150, 600, 250, 200, 565, 360, 100, 80, 800};
int limit = 0;

static enum STATE state = Wait;
static enum STATE next_state = Foward;

static int count = 0;
static int print_count = 0;
static int btn_count = 0;

void InitializeExplorer(void)
{
  state = Wait;
  next_state = Foward;
}

void ExplorerStateControl(float *speed, float *target, uint16_t *distance, uint16_t *depth)
{
  uint16_t min_dist, max_depth;
  int j = 0;

  min_dist = 1000;
  max_depth = 0;
  for (j = 0; j < 3; j++)
  {
    if (max_depth < depth[j])
      max_depth = depth[j];
  }
  for (j = 0; j < 3; j++)
  {
    if (min_dist > distance[j])
      min_dist = distance[j];
  }

  switch (state)
  {
    case Wait:
      if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
        btn_count++;
      else
        btn_count = 0;
      if (btn_count > 10)
        state = Foward;
      target[0] = 0;
      target[1] = 0;
      break;
    case Foward:
      if (100 < max_depth)
      {
        SetEmergencyStop(1);
        next_state = Backward;
        state = Stop;
      }
      if (min_dist < 200)
      {
        next_state = Backward;
        state = Stop;
      }
      if (count_limit[limit % 8] < count)
      {
        if (8 < ++limit) limit = 0;
        next_state = RightTurn;
        state = Stop;
      }
      target[0] = 20;
      target[1] = 20;
      break;
    case Stop:
      if (speed[0] < 0.1 && speed[1] < 0.1)
      {
        SetEmergencyStop(0);
        count = 0;
        if (next_state == Stop)
          next_state = Foward;
        state = next_state;
      }
      target[0] = 0;
      target[1] = 0;
      break;
    case Backward:
      if (200 < min_dist && max_depth < 100)
      {
        next_state = LeftTurn;
        state = Stop;
      }
      target[0] = -10;
      target[1] = -10;
      break;
    case LeftTurn:
      if (100 < max_depth)
      {
        SetEmergencyStop(1);
        next_state = Backward;
        state = Stop;
      }
      if (100 < count && 400 < min_dist)
      {
        next_state = Foward;
        state = Stop;
      }
      target[0] = 10;
      target[1] = -10;
      break;
    case RightTurn:
      if (100 < max_depth)
      {
        SetEmergencyStop(1);
        next_state = Backward;
        state = Stop;
      }
      if (100 < count && 400 < min_dist)
      {
        next_state = Foward;
        state = Stop;
      }
      target[0] = -10;
      target[1] = 10;
      break;
    default:
      state = Foward;
      break;
  }
  count++;

  if ((print_count++) % 50 == 0)
  {
#ifdef PRINT_DEBUG
    printf("max_depth = %d mm, min_dist = %d mm,  state = %d, %d\n", max_depth, min_dist, state, count);
    //printf("input_capture1 = %ld, input_capture2 = %ld\n", input_capture1, input_capture2);
    printf("speed[0] = %ld, speed[1] = %ld\n", (long)speed[0], (long)speed[1]);
#endif
  }
}