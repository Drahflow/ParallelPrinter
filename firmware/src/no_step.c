#include "tick.h"

#include <stddef.h>

const MotorSchedule noStep = { 0, 0, 0, 0, 0, DIR_MAIN_AXIS_UP };
const OutputSchedule noSteps = {
  { noStep, noStep, noStep, noStep, noStep, noStep, noStep, noStep, noStep, noStep },
  0,
  NULL
};
