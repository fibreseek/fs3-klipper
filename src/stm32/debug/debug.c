#include <string.h> // NULL
#include "autoconf.h" // CONFIG_MACH_STM32F446
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/io.h" // writel
#include "board/usb_cdc.h" // usb_notify_ep0
#include "board/usb_cdc_ep.h" // USB_CDC_EP_BULK_IN
#include "command.h" // DECL_CONSTANT_STR
#include "../internal.h" // GPIO
#include "sched.h" // DECL_INIT
#include "printf.h"
#include "board/misc.h"
#include "stm32h723xx.h" 

  #define GPIO_Rx GPIO('B', 15)
  #define GPIO_Tx GPIO('B', 14)
  #define USARTx_FUNCTION GPIO_FUNCTION( \
            (CONFIG_MACH_STM32H7 | CONFIG_MACH_STM32G4) ? 4 : 0)
  #define USARTx USART1
  #define USARTx_IRQn USART1_IRQn


#define CR1_FLAGS (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE   \
                   | USART_CR1_RXNEIE)

void _printf_bin(char * buf, int len)
{
  printf_("dump:");
  for(int i = 0; i < len; i++)
  {
    printf_("0x%x ", buf[i]);
  }
  printf_("\r\n");
}

void _putchar(char ch)
{
  while ((USARTx->ISR & 0X40) == 0);   

  USARTx->TDR = (uint8_t)ch; 
}


int32_t _write(int fd, char data[], int32_t size)
{
    int32_t i = -1;

    if (NULL != data)
    {
        (void)fd; 

        for (i = 0; i < size; i++)
        {
		    while ((USARTx->ISR & 0X40) == 0);  

		    USARTx->TDR = (uint8_t)data[i]; 
        }
    }

    return i ? i : -1;
}

void serial_send(unsigned char * data, uint8_t len)
{
	uint8_t i = 0;
	
	for(i = 0; i < len; i++)
	{
		while ((USARTx->ISR & 0X40) == 0);   
			USARTx->TDR = (uint8_t)data[i]; 
	}

}


void debug_init(void)
{
    enable_pclock((uint32_t)USARTx);

    uint32_t pclk = get_pclock_frequency((uint32_t)USARTx);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, 115200);
    USARTx->BRR = (((div / 16) << USART_BRR_DIV_MANTISSA_Pos)
                   | ((div % 16) << USART_BRR_DIV_FRACTION_Pos));
    USARTx->CR3 = USART_CR3_OVRDIS; // disable the ORE ISR
    USARTx->CR1 = CR1_FLAGS;
    //armcm_enable_irq(USARTx_IRQHandler, USARTx_IRQn, 0);

    gpio_peripheral(GPIO_Rx, USARTx_FUNCTION, 1);
    gpio_peripheral(GPIO_Tx, USARTx_FUNCTION, 0);
}


void serial_task(void)
{
    static uint32_t serial_repeat_until =0;
    //uint32_t now = timer_read_time();
    
    if (serial_repeat_until ++ > 10)
    {
        printf_("123\r\n");
        serial_send((unsigned char *)"123\r\n", 5);
        serial_repeat_until = 0;
    }    
}


//DECL_TASK(serial_task);
DECL_INIT(debug_init);

