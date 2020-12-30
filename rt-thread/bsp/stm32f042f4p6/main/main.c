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
#include "usbd_custom_hid_core.h"
#include  "usbd_usr.h"

USB_CORE_HANDLE  USB_Device_dev ;
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

int main(void)
{
	//	rt_kprintf("sys clk %d\r\n", SystemCoreClock);
	led_init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

	USBD_Init(&USB_Device_dev,
			&USR_desc, 
			&USBD_HID_cb, 
			&USR_cb);
	while (1)
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);
		rt_thread_mdelay(500);
		GPIO_SetBits(GPIOB,GPIO_Pin_1);
		rt_thread_mdelay(500);
	}
}

