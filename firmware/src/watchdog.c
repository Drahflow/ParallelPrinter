#include "watchdog.h"

#include "stm32h723xx.h"

void
watchdog_reset(void)
{
    IWDG1->KR = 0xAAAA;
}

void
watchdog_init(void)
{
    IWDG1->KR = 0x5555;
    IWDG1->PR = 0;
    IWDG1->RLR = 0x0FFF; // 410-512ms timeout (depending on stm32 chip)
    IWDG1->KR = 0xCCCC;
}
