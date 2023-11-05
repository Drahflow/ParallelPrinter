#ifndef H_8FA03834_4167_4067_AC46_26B7722B6E99
#define H_8FA03834_4167_4067_AC46_26B7722B6E99

#include <stdint.h>

#include "gpio.h"
#include "config.h"

typedef struct Motor Motor;
struct Motor {
  uint32_t index;

  struct gpio_out enable;
  struct gpio_out step;
  struct gpio_out dir;
  struct gpio_out uart_out;
  struct gpio_in uart_in;
};

extern Motor motors[MOTOR_COUNT];

void initMotorDrivers();
void forward(Motor *);
void backward(Motor *);
void setupMotor(Motor *, uint32_t stepResolution, uint32_t runPower);
void dumpMotorStatus(Motor *);

#endif
