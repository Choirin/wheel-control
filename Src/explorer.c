#include "explorer.h"

#include "wheel_control.h"

#define abs(a)                 ((a) < 0? -(a): (a))

#define PRINT_DEBUG

enum STATE {
  Foward,
  Backward,
  Stop,
  LeftTurn,
  RightTurn,
  Wait,
};

static const int count_limit[] = {1500, 6000, 2500, 2000, 500, 3600, 1000, 800, 8000};
int limit = 0;

static enum STATE state = Wait;
static enum STATE next_state = Foward;

static int count = 0;
static int print_count = 0;
static int btn_count = 0;

void InitializeExplorer(void)
{
  count = 0;
  print_count = 0;
  btn_count = 0;
  state = Wait;
  next_state = Foward;
}

void ResetState(void)
{
  state = Wait;
}

bool StateActive(void)
{
  return (state != Wait);
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
      target[0] = 0.0;
      target[1] = 0.0;
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
      target[0] = FOWARD_SPEED;
      target[1] = FOWARD_SPEED;
      break;
    case Stop:
      if (abs(speed[0]) < SPEED_ZERO && abs(speed[1]) < SPEED_ZERO)
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
      target[0] = -FOWARD_SPEED;
      target[1] = -FOWARD_SPEED;
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
      target[0] = FOWARD_SPEED;
      target[1] = -FOWARD_SPEED;
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
      target[0] = -FOWARD_SPEED;
      target[1] = FOWARD_SPEED;
      break;
    default:
      state = Foward;
      break;
  }
  count++;

  if ((print_count++) % 10 == 0)
  {
#if 1
//#ifdef PRINT_DEBUG
    //printf("max_depth = %d mm, min_dist = %d mm,  state = %d, %d\n", max_depth, min_dist, state, count);
    //printf("input_capture1 = %ld, input_capture2 = %ld\n", input_capture1, input_capture2);
#if 0
    printf("speed[0] = %d.%03d, speed[1] = %d.%03d\n",
        (int)speed[0], (int)((speed[0] - (int)speed[0]) * 1000.0),
        (int)speed[1], (int)((speed[1] - (int)speed[1]) * 1000.0));
#endif
    printf("%5d, %5d, %6d, %5d, %5d, %6d\n",
        (int)(target[0] * 1000.0),
        (int)(speed[0] * 1000.0),
        TIM2->CNT,
        (int)(target[1] * 1000.0),
        (int)(speed[1] * 1000.0),
        TIM3->CNT);
#endif
  }
}