#include "homing.h"

#include "tick.h"
#include "endstop.h"
#include "console.h"

#include <stdbool.h>

OutputSchedule homingStep;
uint32_t homingThreshold;
static int homingState = -1;

#define AXES_TO_HOME 7

#define coroutine } switch(homingState) { case 0:
#define yield do { homingState = __LINE__; return; case __LINE__:; } while (0)

void runHoming() {
  if(motorsMoving() || (endstopScanning() && endstopDuration < homingThreshold)) return;
  bool endstopClear = !endstopScanning(); // otherwise it's over threshold

  {{ coroutine
    endstopScan();
    yield;

    if(!endstopClear) {
      console_send_str("Endstop not clear at begin of homing.\r\n");
      homingStop();
      return;
    }

    static uint32_t homedAxes;
    for(homedAxes = 0; homedAxes ^ ((1 << AXES_TO_HOME) - 1); ) {
      homingStep.dir = DIR_MAIN_AXIS_UP;
      while(endstopClear) {
        for(int axis = 0; axis < AXES_TO_HOME; ++axis) {
          if(homedAxes & (1 << axis)) continue;
          scheduleMotor(axis, &homingStep);
        }
        yield;
      }

      homingStep.dir = DIR_MAIN_AXIS_DOWN;
      static int unblockSteps;
      for(unblockSteps = 0; !endstopClear; ++unblockSteps) {
        for(int axis = 0; axis < AXES_TO_HOME; ++axis) {
          if(homedAxes & (1 << axis)) continue;
          scheduleMotor(axis, &homingStep);
        }
        yield;
      }

      static int axis;
      for(axis = 0; axis < AXES_TO_HOME; ++axis) {
        if(homedAxes & (1 << axis)) continue;

        homingStep.dir = DIR_MAIN_AXIS_UP;
        {
          static int i;
          for(i = 0; i < unblockSteps; ++i) { scheduleMotor(axis, &homingStep); yield; }
        }

        static bool blocked; blocked = !endstopClear;

        homingStep.dir = DIR_MAIN_AXIS_DOWN;
        {
          static int i;
          for(i = 0; i < unblockSteps; ++i) { scheduleMotor(axis, &homingStep); yield; }
        }

        if(blocked) {
          homedAxes |= (1 << axis);

          console_send_str("Axis homed. Now: ");
          console_send_uint32(homedAxes);
          console_send_str("\r\n");
        }
      }
    }
  }}
}

void homingUpwards() {
  endstopOn();

  homingState = 0;
  console_send_str("Homing upwards\r\n");
}

void homingStop() {
  homingState = -1;

  endstopFloat();
  console_send_str("Homing aborted\r\n");
}
