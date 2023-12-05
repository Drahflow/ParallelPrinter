#ifndef H_7A41FB48_DD1E_4611_A2B9_D442D65000C7
#define H_7A41FB48_DD1E_4611_A2B9_D442D65000C7

#include "config.h"

#include <stdint.h>
#include <stdbool.h>

#define DIR_MAIN_AXIS_UP 0
#define DIR_MAIN_AXIS_DOWN 1

typedef struct OutputSchedule {
  uint32_t count;
  uint32_t timer;
  uint32_t dt;
  uint32_t ddt;
  uint32_t dddt;
  uint8_t dir;
  struct OutputSchedule *next;
} OutputSchedule;

typedef enum {
  ENDSTOP_INIT,
  ENDSTOP_WAIT,
  ENDSTOP_SCAN,
  ENDSTOP_DONE
} EndstopState;

void enableSystick();
void SysTick_IRQ_Handler();

void scheduleMotor(uint32_t index, OutputSchedule *);
void scheduleEndstopScan();
void stopEndstopScan();

extern EndstopState endstopState;
extern uint32_t endstopInitDuration;
extern uint32_t endstopDuration;

bool motorsMoving();

#endif
