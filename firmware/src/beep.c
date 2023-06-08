#include "beep.h"

#include "gpio.h"

uint32_t beep_error = 0;

void beep_synchronous(uint32_t ms, uint32_t freq) {
  struct gpio_out beeper = gpio_out_setup(GPIO('G', 2), 0);

  for(uint32_t t = 0; t < ms * 4000; ++t) {
    gpio_out_write(beeper, ((t * freq / 1000000) & 1));
  }
}
