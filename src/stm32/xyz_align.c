// Hardware interface to "USB OTG (on the go) controller" on stm32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // NULL
#include "autoconf.h" // CONFIG_MACH_STM32F446
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/io.h" // writel
#include "board/usb_cdc.h" // usb_notify_ep0
#include "board/usb_cdc_ep.h" // USB_CDC_EP_BULK_IN
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // GPIO
#include "sched.h" // DECL_INIT
#include "board/misc.h"

static int32_t offset_z0 = 0;
static int32_t offset_z1 = 0;
static int32_t offset_xy[10] = {0};
static uint8_t offset_index = 0;
extern int32_t get_step_count_x(void);
extern int32_t get_step_count_z(void);
#define PG11_VALUE (GPIOG->IDR & (1 << 11)) ? 1 : 0

void command_align_start(uint32_t *args)
{
    uint8_t res = 0;
    uint8_t oid = args[0];
    uint8_t val = args[1];
    switch(val)
    {
      case 0:
        offset_index = 0;
        //printf_("command_align_start\r\n");
        break;
      case 1:
        if(offset_index == 10)
        {
            res = 1;
        }
        else
        {
            res = 0;
        }
        //sendf("xyz_align_end oid=%c val=%c", oid, val);
        //printf_("xyz_align_end oid=%d val=%d", oid, val);
        //((x1-x0) - (x0-x2))/2
        sendf("xyz_align_end oid=%c res=%c z0=%i z1=%i x0=%i y0=%i x1=%i y1=%i", oid, res, offset_z0, offset_z1, 
             ((offset_xy[0] - offset_xy[2])-(offset_xy[1] - offset_xy[0]))/2, 
             ((offset_xy[0] - offset_xy[4])-(offset_xy[3] - offset_xy[0]))/2, 
             ((offset_xy[5] - offset_xy[7])-(offset_xy[6] - offset_xy[5]))/2, 
             ((offset_xy[5] - offset_xy[9])-(offset_xy[8] - offset_xy[5]))/2
         );
        /*printf_("xyz_align_end res=%u z0=%i z1=%i x0=%i y0=%i x1=%i y1=%i", res, offset_z0, offset_z1, 
            ((offset_xy[0] - offset_xy[2])-(offset_xy[1] - offset_xy[0]))/2, 
            ((offset_xy[0] - offset_xy[4])-(offset_xy[3] - offset_xy[0]))/2, 
            ((offset_xy[5] - offset_xy[7])-(offset_xy[6] - offset_xy[5]))/2, 
            ((offset_xy[5] - offset_xy[9])-(offset_xy[8] - offset_xy[5]))/2
        );*/
        break;

        case 2:
            sendf("xyz_align_end oid=%c res=%c z0=%i z1=%i x0=%i y0=%i x1=%i y1=%i", oid, 2, PG11_VALUE, 0, 0,0,0,0);
        break;

    }
}    //sendf("uart_get_cmd oid=%c val=%c", oid, val);

void EXTI0_IRQHandler(void) {
    if (EXTI->PR1 & (1 << 0)) {  // 检查EXTI0中断标志位
        EXTI->PR1 |= (1 << 0);   // 清除EXTI0中断标志位
        //printf_("PF0 %d %d\r\n",get_step_count_x(),get_step_count_z());
        // 在这里添加中断处理代码
        // 例如，切换一个LED的状态或执行其他任务
        if(offset_index == 0)
        {
            offset_z0 = get_step_count_z();
        }
        else if(offset_index == 5)
        {
            offset_z1 = get_step_count_z();
        }
        if(offset_index < 10)
        {
            offset_xy[offset_index] = get_step_count_x();
        }
        offset_index ++;
    }
}

void gpio_irq_init_pf0(void) //PF0
{
    // 1. 使能 GPIOF 时钟
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOFEN;
    
    // 2. 将 PF0 配置为输入模式 (修改 MODER0)
    GPIOF->MODER &= ~(3UL << (0 * 2)); // 00: 输入模式
    
    // 3. 将 PF0 配置为上拉输入
    GPIOF->PUPDR &= ~(3UL << (0 * 2)); // 清除 PUPDR0
    GPIOF->PUPDR |= (1UL << (0 * 2));  // 01: 上拉模式
    
    // 4. 使能 SYSCFG 时钟
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    
    // 5. 选择 EXTI0 映射到 GPIOF
    SYSCFG->EXTICR[0] &= ~(0xF << (0 * 4)); // 清除 EXTI0 选择
    SYSCFG->EXTICR[0] |= (5 << (0 * 4));    // 选择 GPIOF (5: GPIOF)
    
    // 6. 配置 EXTI0 中断触发条件为上升沿触发
    EXTI->FTSR1 &= ~(1 << 0);  // 清除下降沿触发
    EXTI->RTSR1 |= (1 << 0);   // 设置上升沿触发
    
    // 7. 使能 EXTI0 中断
    EXTI->IMR1 |= (1 << 0);  // 使能中断
    EXTI->EMR1 |= (1 << 0);  // 使能事件
    
    // 8. 配置 NVIC 中断优先级和使能中断
    uint32_t irq_number = 6; // EXTI0 对应 NVIC 中断号

    // 使能 NVIC 中断 EXTI0 在 NVIC ISER[0]中第 6 位
    NVIC->ISER[irq_number / 32] = (1 << (irq_number % 32));

    // 配置 NVIC 中断优先级分组为 4-bit 优先级 EXTI0 在 IPR[0]中设置 8 位优先级
    uint32_t priority_group = 2; // 设置 4-bit 优先级分组
    NVIC->IP[irq_number] = (priority_group << 4) | 7; // 设置 4-bit 优先级分组和优先级
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR1 & (1UL << 11))  // 检查是否是 EXTI10 触发
    {
        EXTI->PR1 = (1UL << 11);  // 清除挂起标志（写 1 清除）
        if(offset_index == 0)
        {
            offset_z0 = get_step_count_z();
        }
        else if(offset_index == 5)
        {
            offset_z1 = get_step_count_z();
        }
        if(offset_index < 10)
        {
            offset_xy[offset_index] = get_step_count_x();
        }
        offset_index ++;
        // 处理逻辑
    }
}

void gpio_irq_init_pg11(void) // PG11
{
    // 1. 使能 GPIOG 时钟
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN;
    
    // 2. 将 PG11 配置为输入模式 (修改 MODER11)
    GPIOG->MODER &= ~(3UL << (11 * 2)); // 00: 输入模式
    
    // 3. 将 PG11 配置为上拉输入
    GPIOG->PUPDR &= ~(3UL << (11 * 2)); // 清除 PUPDR11
    GPIOG->PUPDR |= (1UL << (11 * 2));  // 01: 上拉模式
    
    // 4. 使能 SYSCFG 时钟
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    
    // 5. 选择 EXTI11 映射到 GPIOG
    SYSCFG->EXTICR[2] &= ~(0xF << (3 * 4)); // 清除 EXTI11 选择
    SYSCFG->EXTICR[2] |= (6 << (3 * 4));    // 选择 GPIOG (6: GPIOG)
    
    // 6. 配置 EXTI11 中断触发条件为上升沿触发
    EXTI->FTSR1 &= ~(1 << 11);  // 清除下降沿触发
    EXTI->RTSR1 |= (1 << 11);   // 设置上升沿触发
    
    // 7. 使能 EXTI11 中断
    EXTI->IMR1 |= (1 << 11);  // 使能中断
    EXTI->EMR1 |= (1 << 11);  // 使能事件
    
    // 8. 配置 NVIC 中断优先级和使能中断
    uint32_t irq_number = 40; // EXTI15_10 对应 NVIC 中断号为 40

    // 使能 NVIC 中断 EXTI15_10 在 NVIC ISER[1]中第 8 位（40 % 32 = 8）
    NVIC->ISER[irq_number / 32] = (1 << (irq_number % 32));

    // 配置 NVIC 中断优先级分组为 4-bit 优先级 EXTI15_10 在 IPR[10]中设置 8 位优先级
    uint32_t priority_group = 2; // 设置 4-bit 优先级分组
    NVIC->IP[irq_number] = (priority_group << 4) | 7; // 设置 4-bit 优先级分组和优先级
}


DECL_COMMAND(command_align_start, "align_start oid=%c val=%c");
//DECL_ARMCM_IRQ(EXTI0_IRQHandler, EXTI0_IRQn);
DECL_ARMCM_IRQ(EXTI15_10_IRQHandler, EXTI15_10_IRQn);
DECL_INIT(gpio_irq_init_pg11);