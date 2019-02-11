#include "explorer.h"

#include "wheel_control.h"

#define abs(a)                 ((a) < 0? -(a): (a))

#define CLIFF_THRESHOLD_DEPTH  65
#define WALL_THRESHOLD_DIST    200

#define PRINT_DEBUG

enum STATE
{
  Foward = 0,
  Backward,
  Stop,
  LeftTurn,
  RightTurn,
  Wait,
  Setup,
  Safety,
};

static const uint16_t count_limit[] = {1500, 6000, 2500, 2000, 500, 3600, 1000, 800, 8000};
int limit = 0;

static enum STATE state = Wait;
static enum STATE next_state = Foward;

static int duration = 1000;
static int print_count = 0;
static int btn_count = 0;

void InitializeExplorer(void)
{
  duration = 1000;
  print_count = 0;
  btn_count = 0;
  state = Setup;
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

/*
bool Motion(uint16_t *distance, uint16_t *depth, MOTION *motion)
{

}
*/

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

  if (GetSafety())
  {
    SetEmergencyStop(1);
    switch (state)
    {
      case Foward:
        next_state = Backward;
        break;
      case Backward:
        next_state = Foward;
        break;
      case LeftTurn:
        next_state = RightTurn;
        break;
      case RightTurn:
        next_state = LeftTurn;
        break;
      default:
        next_state = Backward;
        break;
    }
    state = Stop;
  }

  switch (state)
  {
    case Wait:
      if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
        btn_count++;
      else
        btn_count = 0;
      if (btn_count > 10)
        state = Setup;
      target[0] = 0.0;
      target[1] = 0.0;
      break;
    case Setup:
      if (8 < ++limit) limit = 0;
      duration = count_limit[limit % 8];
      state = Foward;
      break;
    case Safety:
      duration = 50;
      state = Backward;
      next_state = RightTurn;
      target[0] = 0;
      target[1] = 0;
      break;
    case Foward:
      if (CLIFF_THRESHOLD_DEPTH < max_depth)
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
      if (0 == duration)
      {
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
        duration = 100;
        if (next_state == Stop)
          next_state = Setup;
        state = next_state;
      }
      target[0] = 0;
      target[1] = 0;
      break;
    case Backward:
      if (duration == 0 && 200 < min_dist && max_depth < CLIFF_THRESHOLD_DEPTH)
      {
        duration = 100;
        next_state = LeftTurn;
        state = Stop;
      }
      target[0] = -FOWARD_SPEED;
      target[1] = -FOWARD_SPEED;
      break;
    case LeftTurn:
      if (CLIFF_THRESHOLD_DEPTH < max_depth)
      {
        SetEmergencyStop(1);
        next_state = Backward;
        state = Stop;
      }
      if (duration == 0 && 400 < min_dist)
      {
        next_state = Setup;
        state = Stop;
      }
      target[0] = FOWARD_SPEED;
      target[1] = -FOWARD_SPEED;
      break;
    case RightTurn:
      if (CLIFF_THRESHOLD_DEPTH < max_depth)
      {
        SetEmergencyStop(1);
        next_state = Backward;
        state = Stop;
      }
      if (duration == 0 && 400 < min_dist)
      {
        next_state = Setup;
        state = Stop;
      }
      target[0] = -FOWARD_SPEED;
      target[1] = FOWARD_SPEED;
      break;
    default:
      state = Setup;
      break;
  }
  if (duration != 0) duration--;

  if ((print_count++) % 10 == 0)
  {
#if 0
//#ifdef PRINT_DEBUG
    printf("dep:%5d, dis:%5d, %5d, %5d, %2d, %3d\r\n",
        max_depth,
        min_dist,
        (int)(speed[0] * 1000.0),
        (int)(speed[1] * 1000.0),
        state,
        duration);
    //printf("input_capture1 = %ld, input_capture2 = %ld\n", input_capture1, input_capture2);
#if 0
    printf("%5d, %5d, %6d, %5d, %5d, %6d\n",
        (int)(target[0] * 1000.0),
        (int)(speed[0] * 1000.0),
        TIM2->CNT,
        (int)(target[1] * 1000.0),
        (int)(speed[1] * 1000.0),
        TIM3->CNT);
#endif
#endif
  }
}