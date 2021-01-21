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

extern struct rt_semaphore ota_sem;
extern rt_bool_t ota_mode;
extern void SystemCoreClockUpdate(void);
#define USART6_RDR_Address    0x40011404
#define USART6_TDR_Address    0x40011404
// Holds the system core clock, which is the system clock 
// frequency supplied to the SysTick timer and the processor 
// core clock.
extern uint32_t SystemCoreClock;
uint8_t uart_tx_buf[64] = {0};
uint8_t uart_rx_buf[64] = {0};
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

void USART2_IRQHandler(void)
{
	rt_interrupt_enter();

	if (USART_GetFlagStatus(USART6, USART_FLAG_FE) != RESET) 
		USART_ClearFlag(USART6, USART_FLAG_FE);

	if(USART_GetFlagStatus(USART6, USART_FLAG_PE) != RESET)         
		USART_ClearFlag(USART6, USART_FLAG_PE);  			  

	if(USART_GetFlagStatus(USART6, USART_FLAG_ORE) != RESET)
		USART_ClearFlag(USART6, USART_FLAG_ORE);

	rt_interrupt_leave();
}

void DMA2_Stream6_IRQHandler(void)
{
	rt_interrupt_enter();
	if (DMA_GetFlagStatus(DMA2_Stream6, DMA_FLAG_TCIF6)) {
		/* data sending to ov580 finish */
  		DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6);
  		DMA_Cmd(DMA2_Stream6, DISABLE);
		if (ota_mode)
    			rt_sem_release(&ota_sem);
		else
			notify_event(EVENT_ST2OV);
	}
	rt_interrupt_leave();
}

void DMA2_Stream1_IRQHandler(void)
{
	rt_interrupt_enter();
	if (DMA_GetFlagStatus(DMA2_Stream1, DMA_FLAG_TCIF1)){
  		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1);
		DMA_Cmd(DMA2_Stream1, DISABLE);
		/* data from ov580 finish */
		if (ota_mode) {
			insert_mem(TYPE_H2D, uart_rx_buf, 64);
    			rt_sem_release(&ota_sem);
		} else {
			insert_mem(TYPE_H2D, uart_rx_buf, 64);
			notify_event(EVENT_OV2ST);
		}
		uart_rx_set();
	}
	rt_interrupt_leave();
}

void uart_rsp_out(uint8_t *cmd, uint16_t len)
{
	rt_memcpy(uart_tx_buf, cmd, 64);
	uart_tx_set();
}

static void uart_dma_config()
{
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_DeInit(DMA2_Stream1);
	DMA_DeInit(DMA2_Stream6);

	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
	while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE);

	DMA_InitStructure.DMA_Channel = DMA_Channel_5;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USART6_RDR_Address;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart_rx_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = (uint32_t)64;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USART6_TDR_Address;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uart_tx_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);

	USART_DMACmd(USART6, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(DMA2_Stream1, ENABLE);
	DMA_Cmd(DMA2_Stream6, DISABLE);
	
	DMA_ITConfig(DMA2_Stream6, DMA_SxCR_TCIE, ENABLE);
	DMA_ITConfig(DMA2_Stream1, DMA_SxCR_TCIE, ENABLE);
}

static int uart_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	USART_InitTypeDef USART_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = 
		USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 2000000;
	USART_Init(USART6, &USART_InitStructure);

	uart_dma_config();

	USART_ITConfig(USART6, USART_IT_ORE, ENABLE);	
	USART_ITConfig(USART6, USART_IT_ERR, ENABLE);	
	
	USART_Cmd(USART6, ENABLE);
	USART_ClearFlag(USART6, USART_FLAG_TC);
	USART_Cmd(USART3, ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_TC);
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
			USART_SendData(USART3, a);
			while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); 
		}
		USART_SendData(USART3, *(uint8_t *)(str + i));
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); 
	}
}

char rt_hw_console_getchar(void)
{
	int8_t ch = -1;
	return 0;
	if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
		ch = USART_ReceiveData(USART3) & 0xff;
	return ch;
}
