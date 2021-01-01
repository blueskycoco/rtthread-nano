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
#include <system_stm32f0xx.h>
#include <stm32f0xx.h>
extern void SystemCoreClockUpdate(void);
extern struct rt_semaphore sem;
extern rt_uint8_t uart_rcv[512];
extern rt_uint16_t uart_rcv_len;

// Holds the system core clock, which is the system clock 
// frequency supplied to the SysTick timer and the processor 
// core clock.
extern uint32_t SystemCoreClock;
#define USART1_RDR_Address    0x40013824
#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 1024
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

void USART1_IRQHandler(void)
{
	rt_interrupt_enter();
	if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)
	{
		USART_ClearFlag(USART1,USART_FLAG_ORE);
	}

	if(USART_GetFlagStatus(USART1, USART_FLAG_RTO) != RESET)
	{
		USART_ClearFlag(USART1,USART_FLAG_ORE);
		USART_ReceiverTimeOutCmd(USART1, DISABLE);
		DMA_Cmd(DMA1_Channel3, DISABLE);
		uart_rcv_len = 511 - DMA1_Channel3->CNDTR;
		//rt_sem_release(&sem);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	}

	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
	{
		uart_rcv[0] = USART_ReceiveData(USART1) & 0xff;
		DMA1_Channel3->CNDTR = 511;
		DMA1_Channel3->CMAR = (uint32_t)(uart_rcv+1);
		DMA_Cmd(DMA1_Channel3, ENABLE);
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		USART_ReceiverTimeOutCmd(USART1, ENABLE);
	}
	rt_interrupt_leave();
}

static int uart_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
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

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_InitStructure.DMA_BufferSize = 512;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uart_rcv;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_RDR_Address;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel3, DISABLE);
	DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

	USART_ClearFlag(USART1,USART_FLAG_ORE);
	USART_ClearFlag(USART1,USART_FLAG_RTO);
	USART_ClearFlag(USART1,USART_FLAG_RXNE);
	USART_ITConfig(USART1, USART_IT_ORE, ENABLE);
	USART_ITConfig(USART1, USART_IT_ERR, ENABLE);
	USART_ITConfig(USART1, USART_IT_RTO, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_SetReceiverTimeOut(USART1, 115200/4);
	USART_ReceiverTimeOutCmd(USART1, DISABLE);
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
	/*	if (*(str + i) == '\n')
		{
			USART_SendData(USART1, a);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
		}*/
		USART_SendData(USART1, *(uint8_t *)(str + i));
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
	}
}

char rt_hw_console_getchar(void)
{
	int8_t ch = -1;
	if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
		ch = USART_ReceiveData(USART1) & 0xff;
	return ch;
}
