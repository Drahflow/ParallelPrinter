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

OutputSchedule *m0sched = NULL;
OutputSchedule *m1sched = NULL;
OutputSchedule *m2sched = NULL;
OutputSchedule *m3sched = NULL;
OutputSchedule *m4sched = NULL;
OutputSchedule *m5sched = NULL;
OutputSchedule *m6sched = NULL;
OutputSchedule *m7sched = NULL;
OutputSchedule *m8sched = NULL;
OutputSchedule *m9sched = NULL;
uint32_t m0pulse = 0;
uint32_t m1pulse = 0;
uint32_t m2pulse = 0;
uint32_t m3pulse = 0;
uint32_t m4pulse = 0;
uint32_t m5pulse = 0;
uint32_t m6pulse = 0;
uint32_t m7pulse = 0;
uint32_t m8pulse = 0;
uint32_t m9pulse = 0;

#define MOTOR_DIRECTION(dir_gpio, dir_pin, schedule) \
  do { \
    dir_gpio->BSRR |= schedule->dir? (1u << dir_pin): (1u << (dir_pin + 16)); \
  } while(0)

#define MOTOR_TICK(step_gpio, step_pin, dir_gpio, dir_pin, schedule, pulse) \
  do { \
    if(schedule) { \
      uint32_t oldTimer = schedule->timer; \
      schedule->timer += schedule->dt; \
      schedule->dt += schedule->ddt; \
      schedule->ddt += schedule->dddt; \
   \
      if(schedule->timer < oldTimer) { \
        pulse = !pulse; \
        step_gpio->BSRR |= pulse? (1u << step_pin): (1u << (step_pin + 16)); \
   \
        if(!--schedule->count) { \
          schedule = schedule->next; \
          dir_gpio->BSRR |= schedule->dir? (1u << dir_pin): (1u << (dir_pin + 16)); \
        } \
      } \
    } \
  } while(0)


EndstopState endstopState;
uint32_t endstopDuration;
uint32_t endstopInitDuration = 1000;

void SysTick_IRQ_Handler() {
  MOTOR_TICK(GPIOC, 13, GPIOC, 14, m0sched, m0pulse);
  MOTOR_TICK(GPIOE,  4, GPIOE,  5, m1sched, m1pulse);
  MOTOR_TICK(GPIOE,  1, GPIOE,  0, m2sched, m2pulse);
  MOTOR_TICK(GPIOB,  8, GPIOB,  9, m3sched, m3pulse);
  MOTOR_TICK(GPIOB,  5, GPIOB,  4, m4sched, m4pulse);
  MOTOR_TICK(GPIOG, 15, GPIOB,  3, m5sched, m5pulse);
  MOTOR_TICK(GPIOD,  3, GPIOD,  2, m6sched, m6pulse);
  MOTOR_TICK(GPIOA, 10, GPIOA,  9, m7sched, m7pulse);
  MOTOR_TICK(GPIOA,  8, GPIOC,  7, m8sched, m8pulse);
  MOTOR_TICK(GPIOG,  6, GPIOC,  6, m9sched, m9pulse);

  switch(endstopState) {
#define endstop_pin 0
#define endstop_pin_high() (GPIOF->IDR & (1u << endstop_pin))
#define endstop_msk (0x03 << (endstop_pin * 2))
    case ENDSTOP_INIT:
      if(++endstopDuration < endstopInitDuration) break;
      GPIOF->MODER = (GPIOF->MODER & ~endstop_msk) | (0 << (endstop_pin * 2));
      GPIOF->PUPDR = (GPIOF->PUPDR & ~endstop_msk) | (0 << (endstop_pin * 2));
      endstopState = ENDSTOP_WAIT;
      break;

    case ENDSTOP_WAIT:
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
}

void stopEndstopScan() {
  endstopState = ENDSTOP_DONE;
}

void scheduleMotor(uint32_t index, OutputSchedule *schedule) {
  switch(index) {
    case 0: m0sched = schedule; MOTOR_DIRECTION(GPIOC, 14, m0sched); break;
    case 1: m1sched = schedule; MOTOR_DIRECTION(GPIOE,  5, m1sched); break;
    case 2: m2sched = schedule; MOTOR_DIRECTION(GPIOE,  0, m2sched); break;
    case 3: m3sched = schedule; MOTOR_DIRECTION(GPIOB,  9, m3sched); break;
    case 4: m4sched = schedule; MOTOR_DIRECTION(GPIOB,  4, m4sched); break;
    case 5: m5sched = schedule; MOTOR_DIRECTION(GPIOB,  3, m5sched); break;
    case 6: m6sched = schedule; MOTOR_DIRECTION(GPIOD,  2, m6sched); break;
    case 7: m7sched = schedule; MOTOR_DIRECTION(GPIOA,  9, m7sched); break;
    case 8: m8sched = schedule; MOTOR_DIRECTION(GPIOC,  7, m8sched); break;
    case 9: m9sched = schedule; MOTOR_DIRECTION(GPIOC,  6, m9sched); break;
  }

  // console_send_str("Scheduled motor ticks on motor");
  // console_send_uint8(index);
  // console_send_str("\r\n");
}

bool motorsMoving() {
  return m0sched || m1sched || m2sched || m3sched || m4sched || m5sched || m6sched || m7sched || m8sched || m9sched;
}
