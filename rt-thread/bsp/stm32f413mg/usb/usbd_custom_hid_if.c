/**
  ******************************************************************************
  * @file    USB_Device/CustomHID_Standalone/Src/usbd_customhid_if.c
  * @author  MCD Application Team
  * @brief   USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "usbd_customhid_if.h"
#include "stm32f4xx_hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_customhid.h" 
#include "mcu.h"
#include "mcu_cmd.h"
#include "utils.h"
#include "mem_list.h"
//#include "main.h"
uint8_t uart_rx_buf[64] = {0};
uint8_t uart_tx_buf[64] = {0};
extern struct rt_semaphore ota_sem;
extern rt_bool_t ota_mode;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static int8_t CustomHID_Init(void);
static int8_t CustomHID_DeInit(void);
static int8_t CustomHID_OutEvent(uint8_t* event_idx, uint16_t state);

/* Private variables ---------------------------------------------------------*/
uint8_t SendBuffer[2];
extern USBD_HandleTypeDef USBD_Device;

__ALIGN_BEGIN static uint8_t CustomHID_ReportDesc[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
	0x05,0x01,                      /* Usage Page (Generic Desktop) */ 
	0x09,0x02,                      /* Usage (Mouse) */ 
	0xA1,0x01,                      /* Collection (Application) */

	0x09,0x01,                      /* Usage (Pointer) */
	0xA1,0x00,                      /* Collection (Physical) */
	0x85, 0x01,
	0x95, 0x40,
	0x75, 0x08,
	0x09, 0x3a,
	0x15, 0x00,
	0x26, 0xff, 0x00,
	0x81, 0x02,
	0xC0,                           /* End Collection */

	0xC0                            /* End Collection */
};
#if 0
__ALIGN_BEGIN static uint8_t CustomHID_ReportDesc[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  0x06, 0xFF, 0x00,      /* USAGE_PAGE (Vendor Page: 0xFF00) */                       
  0x09, 0x01,            /* USAGE (Demo Kit)               */    
  0xa1, 0x01,            /* COLLECTION (Application)       */            
  /* 6 */
  
  /* LED1 */        
  0x85, LED1_REPORT_ID,  /*     REPORT_ID (1)		     */
  0x09, 0x01,            /*     USAGE (LED 1)	             */
  0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */          
  0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */           
  0x75, 0x08,            /*     REPORT_SIZE (8)            */        
  0x95, LED1_REPORT_COUNT, /*     REPORT_COUNT (1)           */       
  0xB1, 0x82,             /*    FEATURE (Data,Var,Abs,Vol) */     
  
  0x85, LED1_REPORT_ID,  /*     REPORT_ID (1)              */
  0x09, 0x01,            /*     USAGE (LED 1)              */
  0x91, 0x82,            /*     OUTPUT (Data,Var,Abs,Vol)  */
  /* 26 */
  
  /* LED2 */
  0x85, LED2_REPORT_ID,  /*     REPORT_ID 2		     */
  0x09, 0x02,            /*     USAGE (LED 2)	             */
  0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */          
  0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */           
  0x75, 0x08,            /*     REPORT_SIZE (8)            */        
  0x95, LED2_REPORT_COUNT, /*     REPORT_COUNT (1)           */       
  0xB1, 0x82,             /*    FEATURE (Data,Var,Abs,Vol) */     
  
  0x85, LED2_REPORT_ID,  /*     REPORT_ID (2)              */
  0x09, 0x02,            /*     USAGE (LED 2)              */
  0x91, 0x82,            /*     OUTPUT (Data,Var,Abs,Vol)  */
  /* 46 */
  
  /* LED3 */        
  0x85, LED3_REPORT_ID,  /*     REPORT_ID (3)		     */
  0x09, 0x03,            /*     USAGE (LED 3)	             */
  0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */          
  0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */           
  0x75, 0x08,            /*     REPORT_SIZE (8)            */        
  0x95, LED3_REPORT_COUNT, /*     REPORT_COUNT (1)           */       
  0xB1, 0x82,             /*    FEATURE (Data,Var,Abs,Vol) */     
  
  0x85, LED3_REPORT_ID,  /*     REPORT_ID (3)              */
  0x09, 0x03,            /*     USAGE (LED 3)              */
  0x91, 0x82,            /*     OUTPUT (Data,Var,Abs,Vol)  */
  /* 66 */
  
  /* LED4 */
  0x85, LED4_REPORT_ID,  /*     REPORT_ID 4)		     */
  0x09, 0x04,            /*     USAGE (LED 4)	             */
  0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */          
  0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */           
  0x75, 0x08,            /*     REPORT_SIZE (8)            */        
  0x95, LED4_REPORT_COUNT, /*     REPORT_COUNT (1)           */       
  0xB1, 0x82,            /*     FEATURE (Data,Var,Abs,Vol) */     
  
  0x85, LED4_REPORT_ID,  /*     REPORT_ID (4)              */
  0x09, 0x04,            /*     USAGE (LED 4)              */
  0x91, 0x82,            /*     OUTPUT (Data,Var,Abs,Vol)  */
  /* 86 */
  
  /* key Push Button */  
  0x85, KEY_REPORT_ID,   /*     REPORT_ID (5)              */
  0x09, 0x05,            /*     USAGE (Push Button)        */      
  0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */      
  0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */      
  0x75, 0x01,            /*     REPORT_SIZE (1)            */  
  0x81, 0x82,            /*     INPUT (Data,Var,Abs,Vol)   */   
  
  0x09, 0x05,            /*     USAGE (Push Button)        */               
  0x75, 0x01,            /*     REPORT_SIZE (1)            */           
  0xb1, 0x82,            /*     FEATURE (Data,Var,Abs,Vol) */  
  
  0x75, 0x07,            /*     REPORT_SIZE (7)            */           
  0x81, 0x83,            /*     INPUT (Cnst,Var,Abs,Vol)   */                    
  0x85, KEY_REPORT_ID,   /*     REPORT_ID (2)              */         
  
  0x75, 0x07,            /*     REPORT_SIZE (7)            */           
  0xb1, 0x83,            /*     FEATURE (Cnst,Var,Abs,Vol) */                      
  /* 114 */
  
  /* Tamper Push Button */  
  0x85, TAMPER_REPORT_ID,/*     REPORT_ID (6)              */
  0x09, 0x06,            /*     USAGE (Tamper Push Button) */      
  0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */      
  0x25, 0x01,            /*     LOGICAL_MAXIMUM (1)        */      
  0x75, 0x01,            /*     REPORT_SIZE (1)            */  
  0x81, 0x82,            /*     INPUT (Data,Var,Abs,Vol)   */   
  
  0x09, 0x06,            /*     USAGE (Tamper Push Button) */               
  0x75, 0x01,            /*     REPORT_SIZE (1)            */           
  0xb1, 0x82,            /*     FEATURE (Data,Var,Abs,Vol) */  
  
  0x75, 0x07,            /*     REPORT_SIZE (7)            */           
  0x81, 0x83,            /*     INPUT (Cnst,Var,Abs,Vol)   */                    
  0x85, TAMPER_REPORT_ID,/*     REPORT_ID (6)              */         
  
  0x75, 0x07,            /*     REPORT_SIZE (7)            */           
  0xb1, 0x83,            /*     FEATURE (Cnst,Var,Abs,Vol) */  
  /* 142 */
  
  /* ADC IN */
  0x85, ADC_REPORT_ID,   /*     REPORT_ID                 */         
  0x09, 0x07,            /*     USAGE (ADC IN)             */          
  0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */               
  0x26, 0xff, 0x00,      /*     LOGICAL_MAXIMUM (255)      */                 
  0x75, 0x08,            /*     REPORT_SIZE (8)            */           
  0x81, 0x82,            /*     INPUT (Data,Var,Abs,Vol)   */                    
  0x85, ADC_REPORT_ID,   /*     REPORT_ID (7)              */                 
  0x09, 0x07,            /*     USAGE (ADC in)             */                     
  0xb1, 0x82,            /*     FEATURE (Data,Var,Abs,Vol) */                                 
  /* 161 */
  
  0xc0 	                 /*     END_COLLECTION	             */
}; 
#endif
USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops = 
{
  CustomHID_ReportDesc,
  CustomHID_Init,
  CustomHID_DeInit,
  CustomHID_OutEvent,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CustomHID_Init
  *         Initializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CustomHID_Init(void)
{
  return (0);
}

/**
  * @brief  CustomHID_DeInit
  *         DeInitializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CustomHID_DeInit(void)
{
  /*
  Add your de-initialization code here 
  */  
  return (0);
}

extern uint8_t read_state;
/**
  * @brief  CustomHID_OutEvent
  *         Manage the CUSTOM HID class Out Event    
  * @param  event_idx: LED Report Number
  * @param  state: LED states (ON/OFF)
  */
static int8_t CustomHID_OutEvent  (uint8_t* event_idx, uint16_t len)
{
#if 1
	if (ota_mode) {
			insert_mem(TYPE_H2D, event_idx, 64);
			//rt_kprintf("->%d\r\n", HAL_GetTick()); 
    			rt_sem_release(&ota_sem);
	} else {
			insert_mem(TYPE_H2D, event_idx, 64);
			notify_event(EVENT_OV2ST);
	}
#endif
#if 0
	int i;
	for (i=0; i<len; i++) {
		if (i != 0 && (i % 16) ==0)
			rt_kprintf("\r\n");
		rt_kprintf("%02x ", event_idx[i]);
	}
	rt_kprintf("\r\nota_mode = %d\r\n", ota_mode);
#endif
	return (0);
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void hid_out(uint8_t *buf, uint16_t len)
{
    USBD_CUSTOM_HID_SendReport(&USBD_Device, buf, len);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
