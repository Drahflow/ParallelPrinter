#include "homing.h"

#include "tick.h"
#include "endstop.h"
#include "console.h"

#include <stdbool.h>
#include <string.h>

MotorSchedule homingStep;
MotorSchedule homingClearingStep;
MotorSchedule homingFineStep;
uint32_t homingThresholdInitialRevert;
uint32_t homingThresholdMinimumAxisEffect;
uint32_t homingThresholdFineScan;
uint32_t homingThresholdSingleAxisScan;
uint32_t homingThresholdInitialScan;
uint32_t homingThresholdRescan;

bool homingDebug = false;

uint32_t activeHomingThreshold;
static int homingState = -1;

static OutputSchedule scheduledSteps;

#define coroutine } switch(homingState) { case 0:
#define yield do { homingState = __LINE__; return; case __LINE__:; } while (0)

void runHoming() {
  if(homingState < 0) return;

  uint32_t currentDuration = endstopDuration;

  if(motorsMoving()) return;
  if(endstopInitializing() && endstopWaitDuration > homingThresholdRescan) {
    console_send_str("Endstop wait timeout. Rescheduling.\r\n");
    endstopScan();
    return;
  }

  if(endstopScanning() && currentDuration < activeHomingThreshold) return;
  bool endstopClear = currentDuration < activeHomingThreshold;

  if(homingDebug) {
    console_send_str("Homing state: "); console_send_uint32(homingState); console_send_str("\r\n");
    console_send_str("Duration: "); console_send_uint32(currentDuration); console_send_str("\r\n");
  }

  {{ coroutine
    endstopScan();
    yield;

    if(!endstopClear) {
      console_send_str("Endstop not clear at begin of homing.\r\n");
      homingStop();
      return;
    }

    static uint32_t homedAxes;
    for(homedAxes = 0; homedAxes ^ ((1 << MAIN_AXIS_COUNT) - 1); ) {
      activeHomingThreshold = homingThresholdInitialScan;

      endstopScan();
      yield;
      while(endstopClear) {
        scheduledSteps = noSteps;
        for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
          if(homedAxes & (1 << axis)) continue;

          scheduledSteps.motors[axis] = homingStep;
          scheduledSteps.motors[axis].dir = DIR_MAIN_AXIS_UP;
        }
        scheduleMotors(&scheduledSteps);
        yield;

        endstopScan();
        yield;
      }

      activeHomingThreshold = homingThresholdInitialRevert;

      static int unblockSteps; unblockSteps = 0;

      endstopScan();
      yield;
      while(!endstopClear) {
        scheduledSteps = noSteps;
        for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
          if(homedAxes & (1 << axis)) continue;

          scheduledSteps.motors[axis] = homingStep;
          scheduledSteps.motors[axis].dir = DIR_MAIN_AXIS_DOWN;
        }
        scheduleMotors(&scheduledSteps);
        yield;

        ++unblockSteps;
        endstopScan();
        yield;
      }

      if(homingDebug) {
        console_send_str("Endstop clear after initial homing trigger, unblockSteps: ");
        console_send_uint32(unblockSteps);
        console_send_str("\r\n");
      }

      static int maximumAxis; maximumAxis = -1;
      static uint32_t maximumEndstopDuration; maximumEndstopDuration = homingThresholdMinimumAxisEffect - 1;

      static int axis;
      for(axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
        if(homedAxes & (1 << axis)) continue;

        {
          static int i;
          for(i = 0; i < unblockSteps; ++i) {
            scheduledSteps = noSteps;
            scheduledSteps.motors[axis] = homingStep;
            scheduledSteps.motors[axis].dir = DIR_MAIN_AXIS_UP;
            scheduleMotors(&scheduledSteps);
            yield;
          }
        }

        activeHomingThreshold = homingThresholdSingleAxisScan;
        endstopScan();
        yield;
        static bool blocked; blocked = !endstopClear;
        if(currentDuration > maximumEndstopDuration) {
          maximumAxis = axis;
          maximumEndstopDuration = currentDuration;
        }

        if(homingDebug) {
          console_send_str("Axis scan ");
          console_send_uint8(axis);
          console_send_uint32(currentDuration);
          console_send_str("\r\n");
        }

        {
          static int i;
          for(i = 0; i < unblockSteps; ++i) {
            scheduledSteps = noSteps;
            scheduledSteps.motors[axis] = homingStep;
            scheduledSteps.motors[axis].dir = DIR_MAIN_AXIS_DOWN;
            scheduleMotors(&scheduledSteps);
            yield;
          }
        }

        if(blocked) {
          maximumAxis = -1;
          homedAxes |= (1 << axis);

          console_send_str("Axis homed (endstop triggered). Now: ");
          console_send_uint32(homedAxes);
          console_send_str("\r\n");

          scheduledSteps = noSteps;
          scheduledSteps.motors[axis] = homingClearingStep;
          scheduledSteps.motors[axis].dir = DIR_MAIN_AXIS_DOWN;
          scheduleMotors(&scheduledSteps);
          yield;
        }
      }

      if(maximumAxis >= 0) {
        homedAxes |= (1 << maximumAxis);

        console_send_str("Axis homed (maximum effect). Now: ");
        console_send_uint32(homedAxes);
        console_send_str("\r\n");

        scheduledSteps = noSteps;
        scheduledSteps.motors[maximumAxis] = homingClearingStep;
        scheduledSteps.motors[maximumAxis].dir = DIR_MAIN_AXIS_DOWN;
        scheduleMotors(&scheduledSteps);
        yield;
      }
    }

    console_send_str("Starting precision homing pass.\r\n");

    static int axis;
    for(axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
      console_send_str("Precision homing axis ");
      console_send_uint8(axis);
      console_send_str("\r\n");

      scheduledSteps = noSteps;
      scheduledSteps.motors[axis] = homingClearingStep;
      scheduledSteps.motors[axis].dir = DIR_MAIN_AXIS_UP;
      scheduleMotors(&scheduledSteps);
      yield;

      while(true) {
        activeHomingThreshold = homingThresholdInitialRevert;
        endstopScan();
        yield;

        if(endstopClear) break;

        scheduledSteps = noSteps;
        scheduledSteps.motors[axis] = homingFineStep;
        scheduledSteps.motors[axis].dir = DIR_MAIN_AXIS_DOWN;
        scheduleMotors(&scheduledSteps);
        yield;
      }
        
      while(true) {
        activeHomingThreshold = homingThresholdFineScan;
        endstopScan();
        yield;
        
        if(!endstopClear) break;

        scheduledSteps = noSteps;
        scheduledSteps.motors[axis] = homingFineStep;
        scheduledSteps.motors[axis].dir = DIR_MAIN_AXIS_UP;
        scheduleMotors(&scheduledSteps);
        yield;
      }

      scheduledSteps = noSteps;
      scheduledSteps.motors[axis] = homingClearingStep;
      scheduledSteps.motors[axis].dir = DIR_MAIN_AXIS_DOWN;
      scheduleMotors(&scheduledSteps);
      yield;
    }

    console_send_str("Homing complete.\r\n");

    homingState = -1;
  }}
}

void homingUpwards() {
  endstopOn();

  homingState = 0;
  activeHomingThreshold = homingThresholdInitialScan;
  console_send_str("Homing upwards\r\n");
}

void homingStop() {
  homingState = -1;

  endstopFloat();
  console_send_str("Homing aborted\r\n");
}
