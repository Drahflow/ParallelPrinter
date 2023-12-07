#include "homing.h"

#include "tick.h"
#include "endstop.h"
#include "console.h"

#include <stdbool.h>
#include <string.h>

OutputSchedule homingStep;
OutputSchedule homingClearingStep;
OutputSchedule homingFineStep;
uint32_t homingThresholdInitialRevert;
uint32_t homingThresholdMinimumAxisEffect;
uint32_t homingThresholdFineScan;
uint32_t homingThresholdSingleAxisScan;
uint32_t homingThresholdInitialScan;
uint32_t homingThresholdRescan;

bool homingDebug = false;

uint32_t activeHomingThreshold;
static int homingState = -1;

#define AXES_TO_HOME 7
static OutputSchedule scheduledStep[AXES_TO_HOME];

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
    for(homedAxes = 0; homedAxes ^ ((1 << AXES_TO_HOME) - 1); ) {
      activeHomingThreshold = homingThresholdInitialScan;

      endstopScan();
      yield;
      while(endstopClear) {
        for(int axis = 0; axis < AXES_TO_HOME; ++axis) {
          if(homedAxes & (1 << axis)) continue;

          memcpy(&scheduledStep[axis], &homingStep, sizeof(OutputSchedule));
          scheduledStep[axis].next = NULL;
          scheduledStep[axis].dir = DIR_MAIN_AXIS_UP;
          scheduleMotor(axis, &scheduledStep[axis]);
        }
        yield;

        endstopScan();
        yield;
      }

      activeHomingThreshold = homingThresholdInitialRevert;

      static int unblockSteps; unblockSteps = 0;

      endstopScan();
      yield;
      while(!endstopClear) {
        for(int axis = 0; axis < AXES_TO_HOME; ++axis) {
          if(homedAxes & (1 << axis)) continue;

          memcpy(&scheduledStep[axis], &homingStep, sizeof(OutputSchedule));
          scheduledStep[axis].next = NULL;
          scheduledStep[axis].dir = DIR_MAIN_AXIS_DOWN;
          scheduleMotor(axis, &scheduledStep[axis]);
        }
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
      for(axis = 0; axis < AXES_TO_HOME; ++axis) {
        if(homedAxes & (1 << axis)) continue;

        {
          static int i;
          for(i = 0; i < unblockSteps; ++i) {
            memcpy(&scheduledStep[axis], &homingStep, sizeof(OutputSchedule));
            scheduledStep[axis].next = NULL;
            scheduledStep[axis].dir = DIR_MAIN_AXIS_UP;
            scheduleMotor(axis, &scheduledStep[axis]);
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
            memcpy(&scheduledStep[axis], &homingStep, sizeof(OutputSchedule));
            scheduledStep[axis].next = NULL;
            scheduledStep[axis].dir = DIR_MAIN_AXIS_DOWN;
            scheduleMotor(axis, &scheduledStep[axis]);
            yield;
          }
        }

        if(blocked) {
          maximumAxis = -1;
          homedAxes |= (1 << axis);

          console_send_str("Axis homed (endstop triggered). Now: ");
          console_send_uint32(homedAxes);
          console_send_str("\r\n");

          memcpy(&scheduledStep[axis], &homingClearingStep, sizeof(OutputSchedule));
          scheduledStep[axis].next = NULL;
          scheduledStep[axis].dir = DIR_MAIN_AXIS_DOWN;
          scheduleMotor(axis, &scheduledStep[axis]);
          yield;
        }
      }

      if(maximumAxis >= 0) {
        homedAxes |= (1 << maximumAxis);

        console_send_str("Axis homed (maximum effect). Now: ");
        console_send_uint32(homedAxes);
        console_send_str("\r\n");

        memcpy(&scheduledStep[maximumAxis], &homingClearingStep, sizeof(OutputSchedule));
        scheduledStep[maximumAxis].next = NULL;
        scheduledStep[maximumAxis].dir = DIR_MAIN_AXIS_DOWN;
        scheduleMotor(maximumAxis, &scheduledStep[maximumAxis]);
        yield;
      }
    }

    console_send_str("Starting precision homing pass.\r\n");

    static int axis;
    for(axis = 0; axis < AXES_TO_HOME; ++axis) {
      console_send_str("Precision homing axis ");
      console_send_uint8(axis);
      console_send_str("\r\n");

      memcpy(&scheduledStep[axis], &homingClearingStep, sizeof(OutputSchedule));
      scheduledStep[axis].next = NULL;
      scheduledStep[axis].dir = DIR_MAIN_AXIS_UP;
      scheduleMotor(axis, &scheduledStep[axis]);
      yield;

      while(true) {
        activeHomingThreshold = homingThresholdInitialRevert;
        endstopScan();
        yield;

        if(endstopClear) break;

        memcpy(&scheduledStep[axis], &homingFineStep, sizeof(OutputSchedule));
        scheduledStep[axis].next = NULL;
        scheduledStep[axis].dir = DIR_MAIN_AXIS_DOWN;
        scheduleMotor(axis, &scheduledStep[axis]);
        yield;
      }
        
      while(true) {
        activeHomingThreshold = homingThresholdFineScan;
        endstopScan();
        yield;
        
        if(!endstopClear) break;

        memcpy(&scheduledStep[axis], &homingFineStep, sizeof(OutputSchedule));
        scheduledStep[axis].next = NULL;
        scheduledStep[axis].dir = DIR_MAIN_AXIS_UP;
        scheduleMotor(axis, &scheduledStep[axis]);
        yield;
      }

      memcpy(&scheduledStep[axis], &homingClearingStep, sizeof(OutputSchedule));
      scheduledStep[axis].next = NULL;
      scheduledStep[axis].dir = DIR_MAIN_AXIS_DOWN;
      scheduleMotor(axis, &scheduledStep[axis]);
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
