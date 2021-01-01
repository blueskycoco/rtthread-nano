/**
  ******************************************************************************
  * @file    stm32_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-January-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include <rtthread.h>
#include "stm32_it.h"

extern struct rt_semaphore sem;
extern uint8_t uart_tx_buf[64];
extern uint8_t uart_rx_buf[64];
uint8_t _uart_rx_buf[64];
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint8_t Send_Buffer[2];
uint8_t PrevRxDone = 1;
uint8_t PrevTxDone = 1;
extern uint32_t ADC_ConvertedValueX;
extern uint32_t ADC_ConvertedValueX_1;

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}
void EXTI0_1_IRQHandler(void)
{
	extern void imu_isr(void);
	/* enter interrupt */
	rt_interrupt_enter();
	if(EXTI_GetITStatus(EXTI_Line1))
	{	 
		//imu_isr();	
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
	/* leave interrupt */
	rt_interrupt_leave();
}
/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

/**
  * @brief  This function handles USB FS Handler.
  * @param  None
  * @retval None
  */
void USB_IRQHandler(void)
{
  USB_Istr();
}
void USART2_IRQHandler(void)
{

	rt_interrupt_enter();
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
#if 0
	uint8_t data_len;
	if(USART_GetFlagStatus(USART2, USART_FLAG_IDLE)!= RESET) {
		DMA_Cmd(DMA1_Channel5, DISABLE);
		data_len = 64 - DMA_GetCurrDataCounter(DMA1_Channel5);
		if (data_len > 0) {
			if (PrevRxDone &&
				USB_Device_dev.dev.device_status == USB_CONFIGURED) {
				PrevRxDone = 0;
				//rt_memcpy(_uart_rx_buf, uart_rx_buf, data_len);
				USBD_HID_SendReport (&USB_Device_dev, uart_rx_buf, 64);
			}
		}
	}
#endif
	rt_interrupt_leave();
}
void DMA1_Channel4_5_IRQHandler(void)
{
	rt_interrupt_enter();
	if (DMA_GetFlagStatus(DMA1_FLAG_TC4)) {
		/* data sending to mcu finish */
		PrevTxDone = 1;
  		DMA_ClearFlag(DMA1_FLAG_TC4);
  		DMA_Cmd(DMA1_Channel4, DISABLE);
    		rt_sem_release(&sem);
	} else if (DMA_GetFlagStatus(DMA1_FLAG_TC5)){
  		DMA_ClearFlag(DMA1_FLAG_TC5);
		DMA_Cmd(DMA1_Channel5, DISABLE);
		/* data from mcu finish */
		//rt_kprintf("DMA1_FLAG_TC5, PrevRxDone %d, conf %d\r\n",
		//		PrevRxDone, USB_Device_dev.dev.device_status)
    		rt_sem_release(&sem);
#if 0
		if (PrevRxDone &&
			USB_Device_dev.dev.device_status == USB_CONFIGURED) {
			GPIO_SetBits(GPIOB,GPIO_Pin_1);
			DMA_Cmd(DMA1_Channel5, DISABLE);
			PrevRxDone = 0;
			rt_memcpy(_uart_rx_buf, uart_rx_buf, 64);
			USBD_HID_SendReport (&USB_Device_dev, _uart_rx_buf, 64);
			//rt_kprintf("hid sending \r\n");
		}
#endif
	}
	rt_interrupt_leave();
}
/**
  * @brief  This function handles DMA1_Channel1 Handler.
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler(void)
{  
  Send_Buffer[0] = 0x07;
  
  if((ADC_ConvertedValueX >>4) - (ADC_ConvertedValueX_1 >>4) > 4)
  {
    if ((PrevRxDone) && (USB_Device_dev.dev.device_status == USB_CONFIGURED))
    {
      Send_Buffer[1] = (uint8_t)(ADC_ConvertedValueX >>4);
      
      USBD_HID_SendReport (&USB_Device_dev, Send_Buffer, 2);
      
      ADC_ConvertedValueX_1 = ADC_ConvertedValueX;
      PrevRxDone = 0;
    }
  }
    /* Test DMA1 TC flag */
  while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET );
  
  DMA_ClearFlag(DMA1_FLAG_TC1);
}
         
/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f072.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
