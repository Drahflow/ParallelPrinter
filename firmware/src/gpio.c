// Code to setup gpio on stm32 chip (except for stm32f1)
//
// Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "gpio.h"
#include "main.h"
#include "irq.h"

#include "compiler.h"
#include "stm32h723xx.h"

#include <strings.h>

GPIO_TypeDef * const digital_regs[] = {
    GPIOA, GPIOB, GPIOC, GPIOD,
    GPIOE, GPIOF, GPIOG, GPIOH,
    0,
    GPIOJ, GPIOK,
};

// Convert a register and bit location back to an integer pin identifier
static int
regs_to_pin(GPIO_TypeDef *regs, uint32_t bit)
{
    int i;
    for (i=0; i<ARRAY_SIZE(digital_regs); i++)
        if (digital_regs[i] == regs)
            return GPIO('A' + i, ffs(bit)-1);
    return 0;
}

// Verify that a gpio is a valid pin
static int
gpio_valid(uint32_t pin)
{
    uint32_t port = GPIO2PORT(pin);
    return port < ARRAY_SIZE(digital_regs) && digital_regs[port];
}

struct gpio_out
gpio_out_setup(uint32_t pin, uint32_t val)
{
    if (!gpio_valid(pin))
        shutdown("Not an output pin");
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(pin)];
    gpio_clock_enable(regs);
    struct gpio_out g = { .regs=regs, .bit=GPIO2BIT(pin) };
    gpio_out_reset(g, val);
    return g;
}

void
gpio_out_reset(struct gpio_out g, uint32_t val)
{
    GPIO_TypeDef *regs = g.regs;
    int pin = regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    if (val)
        regs->BSRR = g.bit;
    else
        regs->BSRR = g.bit << 16;
    gpio_peripheral(pin, GPIO_OUTPUT, 0);
    irq_restore(flag);
}

void
gpio_opendrain_out_reset(struct gpio_out g, uint32_t val)
{
    GPIO_TypeDef *regs = g.regs;
    int pin = regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    if (val)
        regs->BSRR = g.bit;
    else
        regs->BSRR = g.bit << 16;
    gpio_peripheral(pin, GPIO_OUTPUT | GPIO_OPEN_DRAIN, 0);
    irq_restore(flag);
}

struct gpio_out
gpio_opendrain_out_setup(uint32_t pin, uint32_t val)
{
    if (!gpio_valid(pin))
        shutdown("Not an output pin");
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(pin)];
    gpio_clock_enable(regs);
    struct gpio_out g = { .regs=regs, .bit=GPIO2BIT(pin) };
    gpio_opendrain_out_reset(g, val);
    return g;
}

void
gpio_out_toggle_noirq(struct gpio_out g)
{
    GPIO_TypeDef *regs = g.regs;
    regs->ODR ^= g.bit;
}

void
gpio_out_toggle(struct gpio_out g)
{
    irqstatus_t flag = irq_save();
    gpio_out_toggle_noirq(g);
    irq_restore(flag);
}

void
gpio_out_write(struct gpio_out g, uint32_t val)
{
    GPIO_TypeDef *regs = g.regs;
    if (val)
        regs->BSRR = g.bit;
    else
        regs->BSRR = g.bit << 16;
}


struct gpio_in
gpio_in_setup(uint32_t pin, int32_t pull_dir)
{
    if (!gpio_valid(pin))
        shutdown("Not a valid input pin");
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(pin)];
    struct gpio_in g = { .regs=regs, .bit=GPIO2BIT(pin) };
    gpio_in_reset(g, pull_dir);
    return g;
}

void
gpio_in_reset(struct gpio_in g, int32_t pull_dir)
{
    GPIO_TypeDef *regs = g.regs;
    int pin = regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    gpio_peripheral(pin, GPIO_INPUT, pull_dir);
    irq_restore(flag);
}

uint8_t
gpio_in_read(struct gpio_in g)
{
    GPIO_TypeDef *regs = g.regs;
    return !!(regs->IDR & g.bit);
}

void
gpio_analog_in_reset(struct gpio_in g)
{
    GPIO_TypeDef *regs = g.regs;
    int pin = regs_to_pin(regs, g.bit);
    irqstatus_t flag = irq_save();
    gpio_peripheral(pin, GPIO_ANALOG, 0);
    irq_restore(flag);
}

struct gpio_in
gpio_analog_in_setup(uint32_t pin)
{
    if (!gpio_valid(pin))
        shutdown("Not a valid input pin");
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(pin)];
    struct gpio_in g = { .regs=regs, .bit=GPIO2BIT(pin) };
    gpio_analog_in_reset(g);
    return g;
}

// Set the mode, extended function and speed of a pin
void
gpio_peripheral(uint32_t gpio, uint32_t mode, int pull_dir)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];

    // Enable GPIO clock
    gpio_clock_enable(regs);

    // Configure GPIO
    uint32_t mode_bits = mode & 0xf;
    uint32_t func = (mode >> 4) & 0xf;
    uint32_t od = (mode >> 8) & 0x1;
    uint32_t hs = (mode >> 9) & 0x1;
    uint32_t pup = pull_dir ? (pull_dir > 0 ? 1 : 2) : 0;
    uint32_t pos = gpio % 16;
    uint32_t af_reg = pos / 8;
    uint32_t af_shift = (pos % 8) * 4;
    uint32_t af_msk = 0x0f << af_shift;
    uint32_t m_shift = pos * 2;
    uint32_t m_msk = 0x03 << m_shift;

    regs->AFR[af_reg] = (regs->AFR[af_reg] & ~af_msk) | (func << af_shift);
    regs->MODER = (regs->MODER & ~m_msk) | (mode_bits << m_shift);
    regs->PUPDR = (regs->PUPDR & ~m_msk) | (pup << m_shift);
    regs->OTYPER = (regs->OTYPER & ~(1 << pos)) | (od << pos);

    // Setup OSPEEDR:
    // stm32f0 is ~10Mhz at 50pF
    // stm32f2 is ~25Mhz at 40pF
    // stm32f4 is ~50Mhz at 40pF
    // stm32f7 is ~50Mhz at 40pF
    // stm32g0 is ~30Mhz at 50pF
    // stm32h7 is ~85Mhz at 50pF
    uint32_t ospeed = hs ? 0x03 : 0x02;
    regs->OSPEEDR = (regs->OSPEEDR & ~m_msk) | (ospeed << m_shift);
}

// Enable a GPIO peripheral clock
void
gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t pos = ((uint32_t)regs - D3_AHB1PERIPH_BASE) / 0x400;
    RCC->AHB4ENR |= (1<<pos);
    RCC->AHB4ENR;
}

