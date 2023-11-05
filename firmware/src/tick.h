#ifndef H_7A41FB48_DD1E_4611_A2B9_D442D65000C7
#define H_7A41FB48_DD1E_4611_A2B9_D442D65000C7

#include <stdint.h>

#include "config.h"

typedef struct OutputSchedule {
  uint32_t count;
  uint32_t timer;
  uint32_t dt;
  uint32_t ddt;
  uint32_t dddt;
  struct OutputSchedule *next;
} OutputSchedule;

void enableSystick();
void SysTick_IRQ_Handler();

void scheduleMotor(uint32_t index, OutputSchedule *);


#endif
