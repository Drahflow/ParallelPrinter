#include "motor.h"

#include "gpio.h"

static struct gpio_out m1enable;
static struct gpio_out m1step;
static struct gpio_out m1dir;

uint8_t stepPulse = 0;

void wait(uint32_t n) {
  for(volatile uint32_t i = 0; i < n; ++i);
}

void initMotorDrivers() {
  m1enable = gpio_out_setup(GPIO('E', 6), 0);
  m1step = gpio_out_setup(GPIO('C', 13), 0);
  m1dir = gpio_out_setup(GPIO('C', 14), 0);
}

void forward(uint32_t n) {
    gpio_out_write(m1dir, 1);
    wait(7500);
    for(int i = 0; i < n; ++i) {
      gpio_out_write(m1step, 1);
      wait(7500);
      gpio_out_write(m1step, 0);
      wait(7500);
    }
}

void backward(uint32_t n) {
    gpio_out_write(m1dir, 0);
    wait(7500);
    for(int i = 0; i < n; ++i) {
      gpio_out_write(m1step, 1);
      wait(7500);
      gpio_out_write(m1step, 0);
      wait(7500);
    }
}
