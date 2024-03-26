#include "tick.h"

#include <stddef.h>

#define NO_STEP { 0, 0, 0, 0, 0, DIR_MAIN_AXIS_UP }
const MotorSchedule noStep = NO_STEP;

const OutputSchedule noSteps = {
  { NO_STEP, NO_STEP, NO_STEP, NO_STEP, NO_STEP, NO_STEP, NO_STEP, NO_STEP, NO_STEP, NO_STEP },
  0,
  NULL
};
