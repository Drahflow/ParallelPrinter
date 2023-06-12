#include "tick.h"

#include "stm32h7xx.h"
#include "config.h"

#include <stddef.h>

void enableSystick() {
  SysTick->LOAD = CONFIG_CLOCK_FREQ / CONFIG_TICK_FREQ - 1u;
  SysTick->VAL = 0;
  SysTick->CTRL =
    SysTick_CTRL_CLKSOURCE_Msk |
    SysTick_CTRL_TICKINT_Msk |
    SysTick_CTRL_ENABLE_Msk;
}

OutputSchedule *m1sched = NULL;
uint32_t m1pulse = 0;

#define m1_GPIO GPIOC
#define m1_STEP_PIN 13

void SysTick_IRQ_Handler() {
  if(m1sched) {
    uint32_t oldTimer = m1sched->timer;
    m1sched->timer += m1sched->dt;
    m1sched->dt += m1sched->ddt;
    m1sched->ddt += m1sched->dddt;

    if(m1sched->timer < oldTimer) {
      m1pulse = !m1pulse;
      m1_GPIO->BSRR |= m1pulse? (1u << m1_STEP_PIN): (1u << (m1_STEP_PIN + 16));

      if(!--m1sched->count) {
        m1sched = m1sched->next;
      }
    }
  }
}
