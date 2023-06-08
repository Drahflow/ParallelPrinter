#include "compiler.h"
#include "stm32h723xx.h"
#include "cmsis_compiler.h"
#include "core_cm7.h"

#include "usb.h"
#include "main.h"

#include <stdint.h>

// Symbols created by stm32h723ze.ld linker script
extern uint32_t _data_start, _data_end, _data_flash;
extern uint32_t _bss_start, _bss_end, _stack_start;
extern uint32_t _stack_end;

void ResetHandler(void);
void DefaultHandler(void);

const void *VectorTable[] __visible __section(".vector_table") = {
  &_stack_end,
  ResetHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  // SysTick_Handler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  DefaultHandler,
  [OTG_HS_IRQn] = OTG_FS_IRQHandler,
};

static void __noreturn reset_handler_stage_two(void) {
    int i;

    // Clear all enabled user interrupts and user pending interrupts
    for (i = 0; i < ARRAY_SIZE(NVIC->ICER); i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        __DSB();
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // Reset all user interrupt priorities
    for (i = 0; i < ARRAY_SIZE(NVIC->IP); i++)
        NVIC->IP[i] = 0;

    // Disable SysTick interrupt
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
    __DSB();

    // Clear pending pendsv and systick interrupts
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk;

    // Reset all system interrupt priorities
    for (i = 0; i < ARRAY_SIZE(SCB->SHPR); i++)
        SCB->SHPR[i] = 0;

    __DSB();
    __ISB();
    __enable_irq();

    // Copy global variables from flash to ram
    uint32_t count = (&_data_end - &_data_start) * 4;
    __builtin_memcpy(&_data_start, &_data_flash, count);

    // Clear the bss segment
    __builtin_memset(&_bss_start, 0, (&_bss_end - &_bss_start) * 4);

    barrier();

    // Run the main board specific code
    main();

    // The main() call should not return
    for (;;)
        ;
}

// Initial code entry point - invoked by the processor after a reset
// Reset interrupts and stack to take control from bootloaders
void ResetHandler(void) {
    __disable_irq();

    // Explicitly load the stack pointer, jump to stage two
    asm volatile("mov sp, %0\n bx %1"
                 : : "r"(&_stack_end), "r"(reset_handler_stage_two));
}

void DefaultHandler(void) {
  for (;;)
    ;
}
