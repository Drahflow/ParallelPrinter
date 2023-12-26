#include "tick.h"

#include "stm32h7xx.h"
#include "config.h"
#include "console.h"
#include "endstop.h"

#include <stddef.h>

void enableSystick() {
  SysTick->LOAD = CONFIG_CLOCK_FREQ / CONFIG_TICK_FREQ - 1u;
  SysTick->VAL = 0;
  SysTick->CTRL =
    SysTick_CTRL_CLKSOURCE_Msk |
    SysTick_CTRL_TICKINT_Msk |
    SysTick_CTRL_ENABLE_Msk;
}

void disableSystick() {
  SysTick->LOAD = CONFIG_CLOCK_FREQ / CONFIG_TICK_FREQ - 1u;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
}

static OutputSchedule *volatile motorSchedule = NULL;
uint32_t motorPulses = 0;

static inline void setMotorDirection(MotorSchedule *schedule, GPIO_TypeDef *dir_gpio, uint32_t dir_pin) {
  dir_gpio->BSRR |= schedule->dir? (1u << dir_pin): (1u << (dir_pin + 16));
}

static inline void setMotorDirections(OutputSchedule *schedule) {
  setMotorDirection(schedule->motors + 0, GPIOC, 14);
  setMotorDirection(schedule->motors + 1, GPIOE,  5);
  setMotorDirection(schedule->motors + 2, GPIOE,  0);
  setMotorDirection(schedule->motors + 3, GPIOB,  9);
  setMotorDirection(schedule->motors + 4, GPIOB,  4);
  setMotorDirection(schedule->motors + 5, GPIOB,  3);
  setMotorDirection(schedule->motors + 6, GPIOD,  2);
  setMotorDirection(schedule->motors + 7, GPIOA,  9);
  setMotorDirection(schedule->motors + 8, GPIOC,  7);
  setMotorDirection(schedule->motors + 9, GPIOC,  6);
}

static inline bool executeMotorSchedule(MotorSchedule *schedule, GPIO_TypeDef *step_gpio, uint32_t step_pin, uint32_t pulseMask) {
  if(!schedule->count) return false;

  uint32_t oldTimer = schedule->timer;
  schedule->timer += schedule->dt;
  schedule->dt += schedule->ddt;
  schedule->ddt += schedule->dddt;

  if(schedule->timer < oldTimer) {
    motorPulses ^= pulseMask;
    step_gpio->BSRR |= (motorPulses & pulseMask)? (1u << step_pin): (1u << (step_pin + 16));

    --schedule->count;
  }

  if(!schedule->count) return false;
  return true;
}

EndstopState endstopState;
uint32_t endstopDuration;
uint32_t endstopWaitDuration;
uint32_t endstopInitDuration = 1000;

void SysTick_IRQ_Handler() {
  OutputSchedule *schedule = motorSchedule;
  if(schedule) {
    bool moreSteps =
      executeMotorSchedule(schedule->motors + 0, GPIOC, 13, 1 << 0) |
      executeMotorSchedule(schedule->motors + 1, GPIOE,  4, 1 << 1) |
      executeMotorSchedule(schedule->motors + 2, GPIOE,  1, 1 << 2) |
      executeMotorSchedule(schedule->motors + 3, GPIOB,  8, 1 << 3) |
      executeMotorSchedule(schedule->motors + 4, GPIOB,  5, 1 << 4) |
      executeMotorSchedule(schedule->motors + 5, GPIOG, 15, 1 << 5) |
      executeMotorSchedule(schedule->motors + 6, GPIOD,  3, 1 << 6) |
      executeMotorSchedule(schedule->motors + 7, GPIOA, 10, 1 << 7) |
      executeMotorSchedule(schedule->motors + 8, GPIOA,  8, 1 << 8) |
      executeMotorSchedule(schedule->motors + 9, GPIOG,  6, 1 << 9) |
      0;

    if(!moreSteps) {
      schedule->completed = 1;
      motorSchedule = schedule->next;
    }
  }

  switch(endstopState) {
#define endstop_pin 0
#define endstop_pin_high() (GPIOF->IDR & (1u << endstop_pin))
#define endstop_msk (0x03 << (endstop_pin * 2))
    case ENDSTOP_INIT:
      if(++endstopDuration < endstopInitDuration) break;
      GPIOF->MODER = (GPIOF->MODER & ~endstop_msk) | (0 << (endstop_pin * 2));
      GPIOF->PUPDR = (GPIOF->PUPDR & ~endstop_msk) | (0 << (endstop_pin * 2));
      endstopState = ENDSTOP_WAIT;
      endstopWaitDuration = 0;
      break;

    case ENDSTOP_WAIT:
      ++endstopWaitDuration;

      if(endstop_pin_high()) break;

      endstopState = ENDSTOP_SCAN;
      endstopDuration = 0;
      break;

    case ENDSTOP_SCAN:
      if(!endstop_pin_high()) {
        ++endstopDuration;
        break;
      }

      endstopState = ENDSTOP_DONE;
      break;

    case ENDSTOP_DONE:
      break;
  }
}

void scheduleEndstopScan() {
  endstopState = ENDSTOP_INIT;
  endstopDuration = 0;
  endstopWaitDuration = 0;
}

void stopEndstopScan() {
  endstopState = ENDSTOP_DONE;
}

void scheduleMotors(OutputSchedule *schedule) {
  setMotorDirections(schedule);
  motorSchedule = schedule;
}

bool motorsMoving() {
  return motorSchedule;
}

void dumpScheduleStatus() {
  if(!motorSchedule) {
    console_send_str("No active schedule.\r\n");
    return;
  }

  console_send_str("Output schedule at ");
  console_send_uint32((uint32_t)motorSchedule);
  console_send_str("\r\n");
  for(int i = 0; i < MOTOR_COUNT; ++i) {
    MotorSchedule *sched = motorSchedule->motors + i;

    console_send_uint8(i);
    console_send_str(" t: ");
    console_send_uint32(sched->timer);
    console_send_str(" dt: ");
    console_send_uint32(sched->dt);
    console_send_str(" ddt: ");
    console_send_uint32(sched->ddt);
    console_send_str(" dddt: ");
    console_send_uint32(sched->dddt);
    console_send_str(" dir: ");
    console_send_uint8(sched->dir);
    console_send_str("\r\n");
  }

  if(motorSchedule->completed) {
    console_send_str("Completed.\r\n");
  } else {
    console_send_str("Not completed.\r\n");
  }

  console_send_str("Next ");
  console_send_uint32((uint32_t)motorSchedule->next);
  console_send_str("\r\n");
}
