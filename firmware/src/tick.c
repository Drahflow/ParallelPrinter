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

#define MOTOR_TICK(gpio, step_pin, schedule, pulse) \
  do { \
    if(schedule) { \
      uint32_t oldTimer = schedule->timer; \
      schedule->timer += schedule->dt; \
      schedule->dt += schedule->ddt; \
      schedule->ddt += schedule->dddt; \
   \
      if(schedule->timer < oldTimer) { \
        pulse = !pulse; \
        gpio->BSRR |= pulse? (1u << step_pin): (1u << (step_pin + 16)); \
   \
        if(!--schedule->count) { \
          schedule = schedule->next; \
        } \
      } \
    } \
  } while(0)


EndstopState endstopState;
uint32_t endstopDuration;

void SysTick_IRQ_Handler() {
  MOTOR_TICK(GPIOC, 13, m0sched, m0pulse);
  MOTOR_TICK(GPIOE,  4, m1sched, m1pulse);
  MOTOR_TICK(GPIOE,  1, m2sched, m2pulse);
  MOTOR_TICK(GPIOB,  8, m3sched, m3pulse);
  MOTOR_TICK(GPIOB,  5, m4sched, m4pulse);
  MOTOR_TICK(GPIOG, 15, m5sched, m5pulse);
  MOTOR_TICK(GPIOD,  3, m6sched, m6pulse);
  MOTOR_TICK(GPIOA, 10, m7sched, m7pulse);
  MOTOR_TICK(GPIOA,  8, m8sched, m8pulse);
  MOTOR_TICK(GPIOG,  6, m9sched, m9pulse);

  switch(endstopState) {
#define endstop_pin 0
#define endstop_pin_high() (GPIOF->IDR & (1u << endstop_pin))
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
  endstopState = ENDSTOP_WAIT;
  endstopDuration = 0;
}

void stopEndstopScan() {
  endstopState = ENDSTOP_DONE;
}

void scheduleMotor(uint32_t index, OutputSchedule *schedule) {
  switch(index) {
    case 0: m0sched = schedule; break;
    case 1: m1sched = schedule; break;
    case 2: m2sched = schedule; break;
    case 3: m3sched = schedule; break;
    case 4: m4sched = schedule; break;
    case 5: m5sched = schedule; break;
    case 6: m6sched = schedule; break;
    case 7: m7sched = schedule; break;
    case 8: m8sched = schedule; break;
    case 9: m9sched = schedule; break;
  }

  // console_send_str("Scheduled motor ticks on motor");
  // console_send_uint8(index);
  // console_send_str("\r\n");
}
