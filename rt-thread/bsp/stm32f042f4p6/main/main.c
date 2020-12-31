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
#include "ymodem.h"

USB_CORE_HANDLE  USB_Device_dev ;
extern uint8_t uart_rx_buf[64];
extern uint8_t uart_tx_buf[64];
struct rym_ctx ctx;
struct rt_semaphore sem;
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
void uart_tx_set()
{
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA1_Channel4->CNDTR = 64;
	DMA_Cmd(DMA1_Channel4, ENABLE);
}
void uart_rx_set()
{
	DMA_Cmd(DMA1_Channel5, DISABLE);
	DMA1_Channel5->CNDTR = 64;
	DMA_Cmd(DMA1_Channel5, ENABLE);
}
int main(void)
{
	int i = 0;
	uint8_t flag = 0;
	rt_sem_init(&sem, "shrx", 0, 0);
	//	rt_kprintf("sys clk %d\r\n", SystemCoreClock);
	led_init();
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	//SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;

	//USBD_Init(&USB_Device_dev,
	//		&USR_desc, 
	//		&USBD_HID_cb, 
	//		&USR_cb);
#if 1
	rt_sem_take(&sem, RT_WAITING_FOREVER);
	rt_kprintf("got uart data\r\n");
	for (i=0; i<64; i++) {
		rt_kprintf("%c", uart_rx_buf[i]);
	}
	rt_kprintf("\r\n");
	uart_rx_set();
	_rym_do_recv(&ctx, RT_WAITING_FOREVER);
#endif
	while (1)
	{
#if 0
		rt_sem_take(&sem, RT_WAITING_FOREVER);
		rt_kprintf("got uart data\r\n");
		for (i=0; i<64; i++) {
			rt_kprintf("%c", uart_rx_buf[i]);
		}
		rt_kprintf("\r\n");
		uart_rx_set();
		rt_memcpy(uart_tx_buf, uart_rx_buf, 64);
		uart_tx_set();
		uart_recover();
#endif
		if (flag == 1) {
			GPIO_ResetBits(GPIOB,GPIO_Pin_1);
			flag = 0;
		} else {
			GPIO_SetBits(GPIOB,GPIO_Pin_1);
			flag = 1;
		}
	}
}

