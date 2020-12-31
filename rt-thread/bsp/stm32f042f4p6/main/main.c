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
extern rt_uint8_t uart_tx_buf[64];
static struct rt_event event;
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

void uart_dma_tx_open()
{
	DMA1_Channel4->CNDTR = 64;
	DMA_Cmd(DMA1_Channel4, ENABLE);
}

void uart_dma_rx_open()
{
	DMA1_Channel5->CNDTR = 64;
	DMA_Cmd(DMA1_Channel5, ENABLE);
}

void notify_uart2hid()
{
	rt_event_send(&event, EVENT_UART2HID);
}

void notify_hid2uart()
{
	rt_event_send(&event, EVENT_HID2UART);
}

int main(void)
{
	rt_uint32_t status;
	rt_uint8_t *hid;
	rt_uint8_t *uart;
	rt_bool_t flag = RT_FALSE;

	rt_memlist_init();
    	rt_event_init(&event, "bridge", RT_IPC_FLAG_FIFO);
	led_init();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

	USBD_Init(&USB_Device_dev,
			&USR_desc, 
			&USBD_HID_cb, 
			&USR_cb);
	while (1)
	{
		if (rt_event_recv(&event, EVENT_HID2UART | EVENT_UART2HID,
					RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
			                RT_WAITING_FOREVER, &status) == RT_EOK)
		{
			if (status & EVENT_HID2UART) {
				remove_mem(TYPE_HID2UART, &hid);
				if (hid != RT_NULL) {
					rt_memcpy(uart_tx_buf, hid, 64);
					uart_dma_tx_open();
				}
			}

			if (status & EVENT_UART2HID) {
				remove_mem(TYPE_UART2HID, &uart);
				if (uart != RT_NULL) {
					USBD_HID_SendReport (&USB_Device_dev,
							uart, 64);
				}
			}
		}
		if (flag) {
			GPIO_ResetBits(GPIOB,GPIO_Pin_1);
			flag = RT_FALSE;
		} else {
			GPIO_SetBits(GPIOB,GPIO_Pin_1);
			flag = RT_TRUE;
		}
	}
}

