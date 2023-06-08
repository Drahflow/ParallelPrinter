#include "compiler.h"
#include "stm32h723xx.h"
#include "cmsis_compiler.h"
#include "core_cm7.h"

#include "io.h"
#include "usb.h"
#include "main.h"
#include "beep.h"
#include "config.h"

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
  [OTG_HS_IRQn + 16] = OTG_FS_IRQHandler,
};

// Check if rebooting into system DFU Bootloader
void
dfu_reboot_check(void)
{
    if (*(uint64_t*)USB_BOOT_FLAG_ADDR != USB_BOOT_FLAG)
        return;
    *(uint64_t*)USB_BOOT_FLAG_ADDR = 0;
    uint32_t *sysbase = (uint32_t*)CONFIG_STM32_DFU_ROM_ADDRESS;
    asm volatile("mov sp, %0\n bx %1"
                 : : "r"(sysbase[0]), "r"(sysbase[1]));
}

#define FREQ_PERIPH (CONFIG_CLOCK_FREQ / 4)

// Main clock and power setup called at chip startup
static void
clock_setup(void)
{
    // Set this despite correct defaults.
    // "The software has to program the supply configuration in PWR control
    // register 3" (pg. 259)
    // Only a single write is allowed (pg. 304)
    PWR->CR3 = (PWR->CR3 | PWR_CR3_LDOEN) & ~(PWR_CR3_BYPASS | PWR_CR3_SCUEN);
    while (!(PWR->CSR1 & PWR_CSR1_ACTVOSRDY))
        ;
    // (HSE 25mhz) /DIVM1(5) (pll_base 5Mhz) *DIVN1(192) (pll_freq 960Mhz)
    // /DIVP1(2) (SYSCLK 480Mhz)
    uint32_t pll_base = 5000000;
    // Only even dividers (DIVP1) are allowed
    uint32_t pll_freq = CONFIG_CLOCK_FREQ * 2;
    if (!CONFIG_STM32_CLOCK_REF_INTERNAL) {
        // Configure PLL from external crystal (HSE)
        RCC->CR |= RCC_CR_HSEON;
        while(!(RCC->CR & RCC_CR_HSERDY))
            ;
        MODIFY_REG(RCC->PLLCKSELR, RCC_PLLCKSELR_PLLSRC_Msk,
                                   RCC_PLLCKSELR_PLLSRC_HSE);
        MODIFY_REG(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM1_Msk,
            (CONFIG_CLOCK_REF_FREQ/pll_base) << RCC_PLLCKSELR_DIVM1_Pos);
    } else {
        // Configure PLL from internal 64Mhz oscillator (HSI)
        // HSI frequency of 64Mhz is integer divisible with 4Mhz
        pll_base = 4000000;
        MODIFY_REG(RCC->PLLCKSELR, RCC_PLLCKSELR_PLLSRC_Msk,
                                   RCC_PLLCKSELR_PLLSRC_HSI);
        MODIFY_REG(RCC->PLLCKSELR, RCC_PLLCKSELR_DIVM1_Msk,
          (64000000/pll_base) << RCC_PLLCKSELR_DIVM1_Pos);
    }
    // Set input frequency range of PLL1 according to pll_base
    // 3 = 8-16Mhz, 2 = 4-8Mhz
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLL1RGE_Msk, RCC_PLLCFGR_PLL1RGE_2);
    // Disable unused PLL1 outputs
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_DIVR1EN_Msk, 0);
    // Enable PLL1Q and set to 100MHz for SPI 1,2,3
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_DIVQ1EN, RCC_PLLCFGR_DIVQ1EN);
    MODIFY_REG(RCC->PLL1DIVR, RCC_PLL1DIVR_Q1,
        (pll_freq / FREQ_PERIPH - 1) << RCC_PLL1DIVR_Q1_Pos);
    // This is necessary, default is not 1!
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_DIVP1EN_Msk, RCC_PLLCFGR_DIVP1EN);
    // Set multiplier DIVN1 and post divider DIVP1
    // 001 = /2, 010 = not allowed, 0011 = /4 ...
    MODIFY_REG(RCC->PLL1DIVR, RCC_PLL1DIVR_N1_Msk,
        (pll_freq/pll_base - 1) << RCC_PLL1DIVR_N1_Pos);
    MODIFY_REG(RCC->PLL1DIVR, RCC_PLL1DIVR_P1_Msk,
        (pll_freq/CONFIG_CLOCK_FREQ - 1) << RCC_PLL1DIVR_P1_Pos);

    // Pwr
    MODIFY_REG(PWR->D3CR, PWR_D3CR_VOS_Msk, PWR_D3CR_VOS);
    while (!(PWR->D3CR & PWR_D3CR_VOSRDY))
        ;

    // Enable VOS0 (overdrive)
    if (CONFIG_CLOCK_FREQ > 400000000) {
        RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
        PWR->CR3 |= PWR_CR3_BYPASS;
        while (!(PWR->D3CR & PWR_D3CR_VOSRDY))
            ;
    }

    SCB_EnableICache();
    SCB_EnableDCache();

    // Set flash latency according to clock frequency (pg.159)
    uint32_t flash_acr_latency = (CONFIG_CLOCK_FREQ > 450000000) ?
        FLASH_ACR_LATENCY_4WS : FLASH_ACR_LATENCY_2WS;
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, flash_acr_latency);
    MODIFY_REG(FLASH->ACR, FLASH_ACR_WRHIGHFREQ_Msk, FLASH_ACR_WRHIGHFREQ_1);
    while (!(FLASH->ACR & flash_acr_latency))
        ;

    // Set HPRE, D1PPRE, D2PPRE, D2PPRE2, D3PPRE dividers
    // 480MHz / 2 = 240MHz rcc_hclk3
    MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_HPRE,    RCC_D1CFGR_HPRE_3);
    // 240MHz / 2 = 120MHz rcc_pclk3
    MODIFY_REG(RCC->D1CFGR, RCC_D1CFGR_D1PPRE,  RCC_D1CFGR_D1PPRE_DIV2);
    // 240MHz / 2 = 120MHz rcc_pclk1
    MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE1, RCC_D2CFGR_D2PPRE1_DIV2);
    // 240MHz / 2 = 120MHz rcc_pclk2
    MODIFY_REG(RCC->D2CFGR, RCC_D2CFGR_D2PPRE2, RCC_D2CFGR_D2PPRE2_DIV2);
    // 240MHz / 2 = 120MHz rcc_pclk4
    MODIFY_REG(RCC->D3CFGR, RCC_D3CFGR_D3PPRE,  RCC_D3CFGR_D3PPRE_DIV2);

    // Switch on PLL1
    RCC->CR |= RCC_CR_PLL1ON;
    while (!(RCC->CR & RCC_CR_PLL1RDY))
        ;

    // Switch system clock source (SYSCLK) to PLL1
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL1);
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL1)
        ;

    // Set the source of FDCAN clock
    MODIFY_REG(RCC->D2CCIP1R, RCC_D2CCIP1R_FDCANSEL, RCC_D2CCIP1R_FDCANSEL_0);

    // Configure HSI48 clock for USB
    SET_BIT(RCC->CR, RCC_CR_HSI48ON);
    while((RCC->CR & RCC_CR_HSI48RDY) == 0);
    SET_BIT(RCC->APB1HENR, RCC_APB1HENR_CRSEN);
    SET_BIT(RCC->APB1HRSTR, RCC_APB1HRSTR_CRSRST);
    CLEAR_BIT(RCC->APB1HRSTR, RCC_APB1HRSTR_CRSRST);
    CLEAR_BIT(CRS->CFGR, CRS_CFGR_SYNCSRC);
    SET_BIT(CRS->CR, CRS_CR_CEN | CRS_CR_AUTOTRIMEN);
    CLEAR_BIT(RCC->D2CCIP2R, RCC_D2CCIP2R_USBSEL);
    SET_BIT(RCC->D2CCIP2R, RCC_D2CCIP2R_USBSEL);
}

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

    // Run SystemInit() and then restore VTOR
    SystemInit();
    RCC->D1CCIPR = 0x00000000;
    RCC->D2CCIP1R = 0x00000000;
    RCC->D2CCIP2R = 0x00000000;
    RCC->D3CCIPR = 0x00000000;
    RCC->APB1LENR = 0x00000000;
    RCC->APB1HENR = 0x00000000;
    RCC->APB2ENR = 0x00000000;
    RCC->APB3ENR = 0x00000000;
    RCC->APB4ENR = 0x00000000;

    SCB->VTOR = (uint32_t)VectorTable;

    dfu_reboot_check();

    clock_setup();

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
  if(beep_error) {
    for(;;) {
      beep_synchronous(500, 200);
      beep_synchronous(1000, 10);

      for(uint32_t j = 0; j < 8; ++j) {
        if(beep_error & (1ul << j)) {
          beep_synchronous(250, 500);
        } else {
          beep_synchronous(250, 400);
        }

        beep_synchronous(100, 10);
      }

      beep_synchronous(1000, 10);
    }
  }

  uint32_t iabr[8];
  for(uint32_t i = 0; i < 8; ++i) iabr[i] = NVIC->IABR[i];

  for (;;) {
    beep_synchronous(100, 200);
    beep_synchronous(100, 300);
    beep_synchronous(100, 200);
    beep_synchronous(100, 300);
    beep_synchronous(1000, 10);

    for(uint32_t i = 0; i < 8; ++i) {
      beep_synchronous(100, 200);
      beep_synchronous(100, 10);

      for(uint32_t j = 0; j < 32; ++j) {
        if(j % 8 == 0) {
          beep_synchronous(100, 300);
          beep_synchronous(100, 10);
        }

        if(iabr[i] & (1ul << j)) {
          beep_synchronous(100, 500);
          beep_synchronous(100, 10);
        } else {
          beep_synchronous(100, 400);
          beep_synchronous(100, 10);
        }
      }
    }

    beep_synchronous(2000, 10);
  }
}
