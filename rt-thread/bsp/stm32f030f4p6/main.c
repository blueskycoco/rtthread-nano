/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-11-05     yangjie      First edition
 */

#include <rtthread.h>
#include <stm32f0xx.h>
struct rt_semaphore sem;
rt_uint8_t uart_rcv[512];
rt_uint16_t uart_rcv_len;
void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void enter_at_mode()
{
	rt_hw_console_output1("+++");
	rt_thread_mdelay(100);
	rt_hw_console_output1("a");
	rt_thread_mdelay(100);
}
int main(void)
{

    led_init();
	
	rt_sem_init(&sem, "shrx", 0, 0);
	enter_at_mode();
	while (1)
    {
		rt_memset(uart_rcv, 0, 512);
    	rt_hw_console_output1("AT+H\r\n");
    	rt_sem_take(&sem, RT_WAITING_FOREVER);
    }
}

