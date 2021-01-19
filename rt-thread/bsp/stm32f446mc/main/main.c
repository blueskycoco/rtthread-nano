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
#include <stm32f4xx.h>
#include "md5.h"
#include "ymodem.h"
#include "boot.h"
#include "utils.h"
#include "mcu.h"

struct rym_ctx ctx;
struct rt_semaphore sem;
void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
int main(void)
{
	uint8_t flag = 0;
	//rt_sem_init(&sem, "shrx", 0, 0);
	led_init();

	//rt_sem_take(&sem, RT_WAITING_FOREVER);
	//uart_rx_set();
	//_rym_do_recv(&ctx, RT_WAITING_FOREVER);
    	//rt_hw_interrupt_disable();

	if (warm_boot())
		protocol_init();
	
	verify_and_jump();
	
	protocol_init();

	while (1)
	{
		if (flag == 1) {
			GPIO_ResetBits(GPIOB,GPIO_Pin_0);
			flag = 0;
			rt_thread_mdelay(1000);
		} else {
			GPIO_SetBits(GPIOB,GPIO_Pin_0);
			flag = 1;
			rt_thread_mdelay(1000);
		}
	}
}

