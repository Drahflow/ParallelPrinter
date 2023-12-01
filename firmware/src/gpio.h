#ifndef H_B9662DAD_01B6_403B_9178_34499C2054A2
#define H_B9662DAD_01B6_403B_9178_34499C2054A2

#include <stdint.h>

#include "stm32h723xx.h"

void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup);
void gpio_clock_enable(GPIO_TypeDef *regs);

struct gpio_out {
    void *regs;
    uint32_t bit;
};

struct gpio_out gpio_out_setup(uint32_t pin, uint32_t val);
struct gpio_out gpio_opendrain_out_setup(uint32_t pin, uint32_t val);

void gpio_out_reset(struct gpio_out g, uint32_t val);
void gpio_out_toggle_noirq(struct gpio_out g);
void gpio_out_toggle(struct gpio_out g);
void gpio_out_write(struct gpio_out g, uint32_t val);

struct gpio_in {
    void *regs;
    uint32_t bit;
};
struct gpio_in gpio_in_setup(uint32_t pin, int32_t pull_up);
void gpio_in_reset(struct gpio_in g, int32_t pull_up);
uint8_t gpio_in_read(struct gpio_in g);

struct gpio_in gpio_analog_in_setup(uint32_t pin);

#define GPIO(PORT, NUM) (((PORT)-'A') * 16 + (NUM))
#define GPIO2PORT(PIN) ((PIN) / 16)
#define GPIO2BIT(PIN) (1<<((PIN) % 16))

#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define GPIO_OPEN_DRAIN 0x100
#define GPIO_HIGH_SPEED 0x200
#define GPIO_FUNCTION(fn) (2 | ((fn) << 4))
#define GPIO_ANALOG 3

#endif
