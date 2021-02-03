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
#include "stm32f4xx_hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h" 


extern PCD_HandleTypeDef hpcd;
extern USBD_HandleTypeDef USBD_Device;
extern struct rt_semaphore ota_sem;
extern rt_bool_t ota_mode;
extern void SystemCoreClockUpdate(void);
extern uint32_t SystemCoreClock;
#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 10*1024
static uint32_t rt_heap[RT_HEAP_SIZE];	// heap default size: 6K(1536 * 4)
UART_HandleTypeDef UartHandle;
RT_WEAK void *rt_heap_begin_get(void)
{
	return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
	return rt_heap + RT_HEAP_SIZE;
}
#endif
void SystemClock_Config (void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
	HAL_StatusTypeDef ret = HAL_OK;

	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	/* The voltage scaling allows optimizing the power consumption when the device is 
	   clocked below the maximum system frequency, to update the voltage scaling value 
	   regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 200;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	RCC_OscInitStruct.PLL.PLLR = 2;
	ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);

	if(ret != HAL_OK)
	{
		while(1) {};
	}

	/* Select PLLSAI output as USB clock source */
	PeriphClkInitStruct.PLLI2S.PLLI2SM = 25;
	PeriphClkInitStruct.PLLI2S.PLLI2SQ = 4;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CK48CLKSOURCE_PLLI2SQ;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	   clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
	ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
	if(ret != HAL_OK)
	{
		while(1) {};
	}
}

/**
 * This function will initial your board.
 */
void rt_hw_board_init()
{
	/* System Clock Update */
	HAL_Init();
	SystemClock_Config();
	SystemCoreClockUpdate();
	/* System Tick Configuration */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / RT_TICK_PER_SECOND);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	NVIC_SetPriority(SysTick_IRQn, 0xFF);

	/* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
	rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
	rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}

uint32_t HAL_GetTick(void)
{
	return rt_tick_get() * 1000 / RT_TICK_PER_SECOND;
}

void HAL_Delay(__IO uint32_t Delay)
{
	rt_thread_mdelay(Delay);
}

void SysTick_Handler(void)
{
	/* enter interrupt */
	rt_interrupt_enter();
	HAL_IncTick();
	rt_tick_increase();

	/* leave interrupt */
	rt_interrupt_leave();
}

void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&hpcd);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();

	__HAL_RCC_USART1_CLK_ENABLE();

	GPIO_InitStruct.Pin       = GPIO_PIN_15;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
	__HAL_RCC_USART1_FORCE_RESET();
	__HAL_RCC_USART1_RELEASE_RESET();

	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

}

static int uart_init(void)
{
	UartHandle.Instance        = USART1;

	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&UartHandle);
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
			HAL_UART_Transmit(&UartHandle, (uint8_t *)&a, 1, 0xFFFF);
		}
		HAL_UART_Transmit(&UartHandle, (uint8_t *)(str+i), 1, 0xFFFF);
	}
}

char rt_hw_console_getchar(void)
{
	int8_t ch = -1;
	if (HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 5000) == HAL_OK)
		return ch;
	return -1;
}
