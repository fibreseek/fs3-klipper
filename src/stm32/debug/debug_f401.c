#include <string.h> // NULL
#include <string.h> // NULL
#include "autoconf.h" // CONFIG_MACH_STM32F401
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/io.h" // writel
#include "board/usb_cdc.h" // usb_notify_ep0
#include "board/usb_cdc_ep.h" // USB_CDC_EP_BULK_IN
#include "command.h" // DECL_CONSTANT_STR
#include "../internal.h" // GPIO
#include "sched.h" // DECL_INIT
#include "printf.h"
#include "board/misc.h"
#include "stm32f4xx.h"

/*
 * F401 debug serial implementation. Uses USART1 with PA9 = TX, PA10 = RX
 * Matches the public interface used by src/stm32/debug/debug.c
 * Enabled when CONFIG_MACH_STM32F401 is set in autoconf.h
 */

#if CONFIG_MACH_STM32F4

#define GPIO_Rx GPIO('A', 10)
#define GPIO_Tx GPIO('A', 9)
#define USARTx_FUNCTION GPIO_FUNCTION(7) /* AF7 for USART1 on F4 */
#define USARTx USART1
#define USARTx_IRQn USART1_IRQn

#define CR1_FLAGS (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE \
                   | USART_CR1_RXNEIE)

void _printf_bin(char * buf, int len)
{
  printf_("dump:");
  for(int i = 0; i < len; i++)
  {
    printf_("0x%x ", (unsigned char)buf[i]);
  }
  printf_("\r\n");
}

void _putchar(char ch)
{
  while ((USARTx->SR & USART_SR_TXE) == 0);
  USARTx->DR = (uint8_t)ch;
}

int32_t _write(int fd, char data[], int32_t size)
{
    int32_t i = -1;

    if (NULL != data)
    {
        (void)fd;

        for (i = 0; i < size; i++)
        {
            while ((USARTx->SR & USART_SR_TXE) == 0);
            USARTx->DR = (uint8_t)data[i];
        }
    }

    return i ? i : -1;
}

void serial_send(unsigned char * data, uint8_t len)
{
    uint8_t i = 0;

    for(i = 0; i < len; i++)
    {
        while ((USARTx->SR & USART_SR_TXE) == 0);
        USARTx->DR = (uint8_t)data[i];
    }
}

void debug_init(void)
{
    /* Enable APB2 clock for USART1 */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    uint32_t pclk = get_pclock_frequency((uint32_t)USARTx);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, 115200);
    /* For F4 series, baud rate register is USART_BRR: mantissa in high bits */
    uint32_t mant = div / 16;
    uint32_t frac = div % 16;
    USARTx->BRR = (mant << 4) | (frac & 0xF);

    /* Disable overrun interrupt not present on F4's CR3 OVRDIS field; keep default */
    USARTx->CR3 = 0;
    USARTx->CR1 = CR1_FLAGS;

    /* Configure pins for AF7 USART1 */
    gpio_peripheral(GPIO_Rx, USARTx_FUNCTION, 1);
    gpio_peripheral(GPIO_Tx, USARTx_FUNCTION, 0);

    /* Optionally enable IRQ if needed */
    //armcm_enable_irq(USART1_IRQHandler, USARTx_IRQn, 0);
}

void serial_task(void)
{
    static uint32_t serial_repeat_until =0;

    if (serial_repeat_until ++ > 10)
    {
        printf_("123\r\n");
        serial_send((unsigned char *)"123\r\n", 5);
        serial_repeat_until = 0;
    }
}


DECL_INIT(debug_init);
// DECL_TASK(serial_task);

#endif /* CONFIG_MACH_STM32F4 */
