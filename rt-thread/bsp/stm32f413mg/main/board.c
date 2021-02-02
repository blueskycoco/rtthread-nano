/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-07-24     Tanek        the first version
 * 2018-11-12     Ernest Chen  modify copyright
 */

#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>
#include <system_stm32f4xx.h>
#include <stm32f4xx.h>
#include "mcu.h"
#include "mcu_cmd.h"
#include "utils.h"
#include "mem_list.h"
#include "usbd_customhid_core.h"

extern struct rt_semaphore ota_sem;
extern rt_bool_t ota_mode;
extern void SystemCoreClockUpdate(void);
extern uint32_t SystemCoreClock;
extern USB_OTG_CORE_HANDLE USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler(USB_OTG_CORE_HANDLE * pdev);
#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 10*1024
static uint32_t rt_heap[RT_HEAP_SIZE];	// heap default size: 6K(1536 * 4)
RT_WEAK void *rt_heap_begin_get(void)
{
	return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
	return rt_heap + RT_HEAP_SIZE;
}
#endif

/**
 * This function will initial your board.
 */
void rt_hw_board_init()
{
	/* System Clock Update */
	SystemCoreClockUpdate();

	/* System Tick Configuration */
	SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

	/* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
	rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
	rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}

void SysTick_Handler(void)
{
	/* enter interrupt */
	rt_interrupt_enter();

	rt_tick_increase();

	/* leave interrupt */
	rt_interrupt_leave();
}

void OTG_FS_IRQHandler(void)
{
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}

static int uart_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	USART_InitTypeDef USART_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_USART1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = 
		USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_Cmd(USART1, ENABLE);
	return 0;
}

INIT_BOARD_EXPORT(uart_init);

void rt_hw_console_output(const char *str)
{   
	rt_size_t i = 0, size = 0;
	char a = '\r';

	size = rt_strlen(str);
	for (i = 0; i < size; i++)
	{
		if (*(str + i) == '\n')
		{
			USART_SendData(USART1, a);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
		}
		USART_SendData(USART1, *(uint8_t *)(str + i));
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
	}
}

char rt_hw_console_getchar(void)
{
	int8_t ch = -1;
	return 0;
	if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
		ch = USART_ReceiveData(USART1) & 0xff;
	return ch;
}
