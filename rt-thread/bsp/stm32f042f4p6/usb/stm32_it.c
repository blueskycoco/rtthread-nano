#include <rtthread.h>
#include "stm32_it.h"
#include "mem_list.h"

extern uint8_t uart_rx_buf[64];
extern USB_CORE_HANDLE  USB_Device_dev ;
rt_bool_t hid_sent = RT_TRUE;
void NMI_Handler(void)
{
}

void SVC_Handler(void)
{
}

void USB_IRQHandler(void)
{
	USB_Istr();
}
void USART2_IRQHandler()
{
	if (USART_GetFlagStatus(USART2, USART_FLAG_FE) != RESET)
	{
		USART_ClearFlag(USART2, USART_FLAG_FE);
	}

	if(USART_GetFlagStatus(USART2, USART_FLAG_PE) != RESET)
	{
		USART_ClearFlag(USART2, USART_FLAG_PE);
	}

	if(USART_GetFlagStatus(USART2,USART_FLAG_ORE) != RESET)
	{
		USART_ClearFlag(USART2,USART_FLAG_ORE);
	}

	if(USART_GetFlagStatus(USART2,USART_FLAG_NE) != RESET)
	{
		USART_ClearFlag(USART2,USART_FLAG_NE);
	}
}
void DMA1_Channel4_5_IRQHandler(void)
{
	rt_interrupt_enter();
	if (DMA_GetFlagStatus(DMA1_FLAG_TC4)) {
		DMA_ClearFlag(DMA1_FLAG_TC4);
		DMA_Cmd(DMA1_Channel4, DISABLE);
		notify_hid2uart();
	} else if (DMA_GetFlagStatus(DMA1_FLAG_TC5)){
		DMA_ClearFlag(DMA1_FLAG_TC5);
		if (USB_Device_dev.dev.device_status == USB_CONFIGURED) {
			rt_bool_t insert = insert_mem(TYPE_UART2HID, uart_rx_buf);
			if (insert && hid_sent)
				notify_uart2hid();
		}
		DMA_Cmd(DMA1_Channel5, DISABLE);
		DMA1_Channel5->CNDTR = 64;
		DMA_Cmd(DMA1_Channel5, ENABLE);
	}
	rt_interrupt_leave();
}
