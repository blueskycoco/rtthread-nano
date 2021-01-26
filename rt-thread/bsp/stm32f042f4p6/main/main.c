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
#include "mem_list.h"

#define EVENT_UART2HID	0x01
#define EVENT_HID2UART	0x02
rt_bool_t hid_sent = RT_TRUE;
extern rt_uint8_t uart_tx_buf[64];
static struct rt_event event;
uint32_t sof_timer = 0;
struct rt_semaphore sof_sem;
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

void led_blink()
{
	static rt_bool_t flag = RT_FALSE;
	if (flag) {
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);
		flag = RT_FALSE;
	} else {
		GPIO_SetBits(GPIOB,GPIO_Pin_1);
		flag = RT_TRUE;
	}
}

void sof_int()
{
    rt_sem_release(&sof_sem);
}

int main(void)
{
	rt_uint32_t status;
	rt_uint8_t *hid;
	rt_uint8_t imu[64];

    	rt_sem_init(&sof_sem, "sem", 0, 0);
	led_init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

	USBD_Init(&USB_Device_dev,
			&USR_desc, 
			&USBD_HID_cb, 
			&USR_cb);
	init_imu();
	while (1)
	{
		rt_sem_take(&sof_sem, RT_WAITING_FOREVER);
		dump_imu(imu, sof_timer);
		get_ts(imu+15);
		//if (hid_sent) {
			hid_sent = RT_FALSE;
			USBD_HID_SendReport (&USB_Device_dev, imu, 23);
		//}
		led_blink();
	}
}

