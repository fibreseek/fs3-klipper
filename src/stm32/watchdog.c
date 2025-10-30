// Watchdog handler on STM32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_STM32H7
#include "internal.h" // IWDG
#include "sched.h" // DECL_TASK
#include "gpio.h"
#if CONFIG_MACH_STM32H7 // stm32h7 libraries only define IWDG1 and IWDG2
#define IWDG IWDG1
#endif
//struct gpio_out gpio_out_setup(uint32_t pin, uint32_t val);

struct gpio_out dis;
void
watchdog_reset(void)
{
    IWDG->KR = 0xAAAA;
    gpio_out_toggle(dis);
}
DECL_TASK(watchdog_reset);

void
watchdog_init(void)
{
    IWDG->KR = 0x5555;
    IWDG->PR = 0;
    IWDG->RLR = 0x0FFF; // 410-512ms timeout (depending on stm32 chip)
    IWDG->KR = 0xCCCC;
    #if CONFIG_MACH_STM32F4
    dis = gpio_out_setup(GPIO('C', 13), 0);
    #else
    dis = gpio_out_setup(GPIO('B', 11), 1);
    #endif
}
DECL_INIT(watchdog_init);
