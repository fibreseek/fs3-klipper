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
#include "gpio.h"

static int32_t offset_z[10] = {0};
static int32_t offset_xy[10][5] = {0};
static uint8_t offset_index = 0;
extern int32_t get_step_count_x(void);
extern int32_t get_step_count_z(void);
#define PG10_VALUE (GPIOG->IDR & (1 << 10)) ? 1 : 0
#define ABS(x) ((x) < 0 ? -(x) : (x))
volatile uint8_t debounce = 0; // Debounce counter for interrupt filtering
#define DEBOUNCE_CYCLES 10  // Increase debounce time for better filtering

void sort_array(int arr[], int n) {
    int i, j, temp;
    for (i = 0; i < n - 1; i++) {
        for (j = 0; j < n - 1 - i; j++) {
            if (arr[j] > arr[j + 1]) {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}


void command_align_start(uint32_t *args)
{
    uint8_t res = 0;
    uint8_t oid = args[0];
    uint8_t val = args[1];
    uint8_t i = 0, j = 0;
    switch(val)
    {
      case 0:
        offset_index = 0;
        // printf_("command_align_start\r\n");
        break;
      case 1:
        if(offset_index == 50)
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
        for(i = 0; i < 10; i++)
        {
            sort_array(offset_xy[i], 5);
        }
        sort_array(offset_z, 5);
        sort_array(offset_z+5, 5);

        sendf("xyz_align_end oid=%c res=%c z0=%i z1=%i x0=%i y0=%i x1=%i y1=%i", oid, offset_index, offset_z[2], offset_z[7], 
             (ABS(offset_xy[0][2] - offset_xy[2][2])-ABS(offset_xy[1][2] - offset_xy[0][2])), 
             (ABS(offset_xy[0][2] - offset_xy[4][2])-ABS(offset_xy[3][2] - offset_xy[0][2])), 
             (ABS(offset_xy[5][2] - offset_xy[7][2])-ABS(offset_xy[6][2] - offset_xy[5][2])), 
             (ABS(offset_xy[5][2] - offset_xy[9][2])-ABS(offset_xy[8][2] - offset_xy[5][2]))
         );
         for(i = 0; i < 10; i++)
         {
            printf_("offset_xy[%d]=%d \r\n", i, offset_xy[i][2]);
         }
         printf_("z0=%d z1=%d\r\n", offset_z[2], offset_z[7]);

        break;

    case 2:
        sendf("xyz_align_end oid=%c res=%c z0=%i z1=%i x0=%i y0=%i x1=%i y1=%i", oid, 2, PG10_VALUE, 0, 0,0,0,0);
        // printf_("xyz align result=%i\r\n", PG10_VALUE);
    break;

    case 3:
        offset_index = 0;
        printf_("z:%d, xy: %d %d %d %d %d\r\n",offset_z[0], offset_xy[0][0], offset_xy[1][0], offset_xy[2][0], offset_xy[3][0], offset_xy[4][0]);
    break;

    case 4:
        for(i = 0; i < 10; i++)
        {
            
            for(j = 0; j < 5; j++)
            {
                printf_("%d ", offset_xy[i][j]);
            }
            printf_("%d \r\n",offset_z[i]);
        }
    break;
    }
}    //sendf("uart_get_cmd oid=%c val=%c", oid, val);


void EXTI15_10_IRQHandler(void)
{
    int32_t x,z;
    int32_t index;
    if (EXTI->PR1 & (1UL << 10))  // 检查是否是 EXTI10 触发
    {
        EXTI->PR1 = (1UL << 10);  // 清除挂起标志（写 1 清除）

        if(debounce != 0)         // Check debounce counter
        {
            return;
        }
        debounce = DEBOUNCE_CYCLES;

        x = get_step_count_x();
        z = get_step_count_z();

        if(offset_index %5==0)
        {
            offset_z[offset_index/5] = z;
        }
        
        if(offset_index < 25)
        {
            offset_xy[offset_index%5][offset_index/5] = x;
        }
        else if(offset_index < 50)
        {
            index = offset_index - 25;
            offset_xy[(index%5) + 5][index/5] = x;
        }
        offset_index++;
        // printf_("offset_index=%d\r\n", offset_index);
        // 处理逻辑
    }
}

void gpio_irq_init_pg10(void) // PG10
{
    // 1. 使能 GPIOG 时钟
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN;
    
    // 2. 将 PG10 配置为输入模式 (修改 MODER10)
    GPIOG->MODER &= ~(3UL << (10 * 2)); // 00: 输入模式
    
    // 3. 将 PG10 配置为上拉输入
    GPIOG->PUPDR &= ~(3UL << (10 * 2)); // 清除 PUPDR10
    GPIOG->PUPDR |= (1UL << (10 * 2));  // 01: 上拉模式
    
    // 4. 使能 SYSCFG 时钟
    RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
    
    // 5. 选择 EXTI10 映射到 GPIOG
    SYSCFG->EXTICR[2] &= ~(0xF << (2 * 4)); // 清除 EXTI10 选择
    SYSCFG->EXTICR[2] |= (6 << (2 * 4));    // 选择 GPIOG (6: GPIOG)
    
    // 6. 配置 EXTI10 中断触发条件为上升沿触发
    EXTI->FTSR1 &= ~(1 << 10);  // 清除下降沿触发
    EXTI->RTSR1 |= (1 << 10);   // 设置上升沿触发
    
    // 7. 使能 EXTI10 中断
    EXTI->IMR1 |= (1 << 10);  // 使能中断
    EXTI->EMR1 |= (1 << 10);  // 使能事件
    
    // 8. 配置 NVIC 中断优先级和使能中断
    uint32_t irq_number = 40; // EXTI15_10 对应 NVIC 中断号为 40

    // 使能 NVIC 中断 EXTI15_10 在 NVIC ISER[1]中第 8 位（40 % 32 = 8）
    NVIC->ISER[irq_number / 32] = (1 << (irq_number % 32));

    // 配置 NVIC 中断优先级分组为 4-bit 优先级 EXTI15_10 在 IPR[10]中设置 8 位优先级
    uint32_t priority_group = 2; // 设置 4-bit 优先级分组
    NVIC->IP[irq_number] = (priority_group << 4) | 7; // 设置 4-bit 优先级分组和优先级
}

void xyz_task(void)
{
    if(debounce > 0)
    {
        debounce--;
    }

}

DECL_COMMAND(command_align_start, "align_start oid=%c val=%c");
//DECL_ARMCM_IRQ(EXTI0_IRQHandler, EXTI0_IRQn);
DECL_ARMCM_IRQ(EXTI15_10_IRQHandler, EXTI15_10_IRQn);
DECL_INIT(gpio_irq_init_pg10);
DECL_TASK(xyz_task);
