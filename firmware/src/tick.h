#ifndef H_7A41FB48_DD1E_4611_A2B9_D442D65000C7
#define H_7A41FB48_DD1E_4611_A2B9_D442D65000C7

#include "config.h"

#include <stdint.h>
#include <stdbool.h>

#define DIR_MAIN_AXIS_UP 0
#define DIR_MAIN_AXIS_DOWN 1

typedef struct MotorSchedule {
  uint32_t count;
  uint32_t timer;
  uint32_t dt;
  uint32_t ddt;
  uint32_t dddt;
  uint8_t dir;
} MotorSchedule;

typedef struct OutputSchedule {
  MotorSchedule motors[MOTOR_COUNT];
  uint8_t completed;
  struct OutputSchedule *next;
} OutputSchedule;

extern const MotorSchedule noStep;
extern const OutputSchedule noSteps;

typedef enum {
  ENDSTOP_INIT,
  ENDSTOP_WAIT,
  ENDSTOP_SCAN,
  ENDSTOP_DONE
} EndstopState;

void enableSystick();
void disableSystick();
void SysTick_IRQ_Handler();

void scheduleMotors(OutputSchedule *);
void scheduleEndstopScan();
void stopEndstopScan();
void dumpScheduleStatus();

extern EndstopState endstopState;
extern uint32_t endstopInitDuration;
extern uint32_t endstopWaitDuration;
extern uint32_t endstopDuration;

bool motorsMoving();

#endif
