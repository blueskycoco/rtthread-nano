#include <rtthread.h>
#include "stm32_it.h"

extern uint8_t uart_tx_buf[64];
extern uint8_t uart_rx_buf[64];
uint8_t _uart_rx_buf[64];
uint8_t PrevRxDone = 1;
uint8_t PrevTxDone = 1;

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

void DMA1_Channel4_5_IRQHandler(void)
{
	rt_interrupt_enter();
	if (DMA_GetFlagStatus(DMA1_FLAG_TC4)) {
		/* data sending to mcu finish */
		PrevTxDone = 1;
		DMA_ClearFlag(DMA1_FLAG_TC4);
		DMA_Cmd(DMA1_Channel4, DISABLE);
	} else if (DMA_GetFlagStatus(DMA1_FLAG_TC5)){
		DMA_ClearFlag(DMA1_FLAG_TC5);
		/* data from mcu finish */
		if (PrevRxDone &&
				USB_Device_dev.dev.device_status == USB_CONFIGURED) {
			GPIO_SetBits(GPIOB,GPIO_Pin_1);
			DMA_Cmd(DMA1_Channel5, DISABLE);
			PrevRxDone = 0;
			rt_memcpy(_uart_rx_buf, uart_rx_buf, 64);
			USBD_HID_SendReport (&USB_Device_dev, _uart_rx_buf, 64);
		}
	}
	rt_interrupt_leave();
}
