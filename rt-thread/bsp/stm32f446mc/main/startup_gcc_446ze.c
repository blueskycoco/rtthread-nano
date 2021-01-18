//*****************************************************************************
//
// startup_gcc.c - Startup code for use with GNU tools.
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include "system_stm32f4xx.h"

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void Reset_Handler(void);
static void NmiSR(void);
static void IntDefaultHandler(void);
static void SVC_Handler(void);
extern void EXTI4_15_IRQHandler(void);
extern void USART1_IRQHandler(void);
extern void USART2_IRQHandler(void);
extern void SysTick_Handler(void);
extern void PendSV_Handler(void);
extern void HardFault_Handler(void);
extern void USB_IRQHandler(void);
extern void EXTI0_1_IRQHandler(void);
extern void DMA1_Channel4_5_IRQHandler(void);
//*****************************************************************************
//
// External declaration for the interrupt handler used by the application.
//
//*****************************************************************************
//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern int entry(void);

//*****************************************************************************
//
// Reserve space for the system stack.
//
//*****************************************************************************
static uint32_t pui32Stack[1024];
#define BootRAM ((void *)(0xF108F85F))
//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
	(void (*)(void))((uint32_t)pui32Stack + sizeof(pui32Stack)),
	// The initial stack pointer
	Reset_Handler,        // 	The reset handler
	NmiSR,                // 	The NMI handler
	HardFault_Handler,    // 	The hard fault handler
	0,                    // 	The MPU fault handler
	0,                    // 	The bus fault handler
	0,                    // 	The usage fault handler
	0,                    // 	Reserved
	0,                    // 	Reserved
	0,                    // 	Reserved
	0,                    // 	Reserved
	SVC_Handler,          // 	SVCall handler
	0,                    // 	Debug monitor handler
	0,                    //        Reserved
	PendSV_Handler,       //	PendSV_Handler
	SysTick_Handler,      //	SysTick_Handler
	IntDefaultHandler,    //	WWDG_IRQHandler
	0,    
	0,    
#if 0
	IntDefaultHandler,    //	RTC_IRQHandler
	IntDefaultHandler,    //	FLASH_IRQHandler
	IntDefaultHandler,    //	RCC_IRQHandler
	IntDefaultHandler,    //	EXTI0_1_IRQHandler
	IntDefaultHandler,    //	EXTI2_3_IRQHandler
	IntDefaultHandler,    //	EXTI4_15_IRQHandler
	IntDefaultHandler,	//	TSC_IRQHandler    
	IntDefaultHandler,    //	DMA1_Channel1_IRQHandler
	IntDefaultHandler,    //	DMA1_Channel2_3_IRQHandler
	IntDefaultHandler,    //	DMA1_Channel4_5_IRQHandler
	IntDefaultHandler,    //	ADC1_IRQHandler
	IntDefaultHandler,    //	TIM1_BRK_IRQHandler
	IntDefaultHandler,    //	TIM1_CC_IRQHandler
	IntDefaultHandler,	//	TIM2_IRQHandler    
	IntDefaultHandler,    //	TIM3_IRQHandler
	0,    
	0,    
	IntDefaultHandler,    //	TIM14_IRQHandler
	0,		    //	TIM15_IRQHandler
	IntDefaultHandler,    //	TIM16_IRQHandler
	IntDefaultHandler,    //	TIM17_IRQHandler
	IntDefaultHandler,    //	I2C1_IRQHandler
	0,		    //	I2C2_IRQHandler
	IntDefaultHandler,    //	SPI1_IRQHandler
	IntDefaultHandler,    //	SPI2_IRQHandler
	IntDefaultHandler,    //	USART1_IRQHandler
	IntDefaultHandler,    //	USART2_IRQHandler	
	0,
	IntDefaultHandler,    //	CEC_CAN
	IntDefaultHandler,	      //	USB
#endif
  IntDefaultHandler, //g    RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */                      
  IntDefaultHandler, //g    FLASH_IRQHandler                  /* FLASH                        */                                          
  IntDefaultHandler, //g    RCC_IRQHandler                    /* RCC                          */                                            
  IntDefaultHandler, //g    EXTI0_IRQHandler                  /* EXTI Line0                   */                        
  IntDefaultHandler, //g    EXTI1_IRQHandler                  /* EXTI Line1                   */                          
  IntDefaultHandler, //g    EXTI2_IRQHandler                  /* EXTI Line2                   */                          
  IntDefaultHandler, //g    EXTI3_IRQHandler                  /* EXTI Line3                   */                          
  IntDefaultHandler, //g    EXTI4_IRQHandler                  /* EXTI Line4                   */                          
  IntDefaultHandler, //g    DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */                  
  IntDefaultHandler, //g    DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */                   
  IntDefaultHandler, //g    DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */                   
  IntDefaultHandler, //g    DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */                   
  IntDefaultHandler, //g    DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */                   
  IntDefaultHandler, //g    DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */                   
  IntDefaultHandler, //g    DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */                   
  IntDefaultHandler, //g    ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */                   
  IntDefaultHandler, //g    CAN1_TX_IRQHandler                /* CAN1 TX                      */                         
  IntDefaultHandler, //g    CAN1_RX0_IRQHandler               /* CAN1 RX0                     */                          
  IntDefaultHandler, //g    CAN1_RX1_IRQHandler               /* CAN1 RX1                     */                          
  IntDefaultHandler, //g    CAN1_SCE_IRQHandler               /* CAN1 SCE                     */                          
  IntDefaultHandler, //g    EXTI9_5_IRQHandler                /* External Line[9:5]s          */                          
  IntDefaultHandler, //g    TIM1_BRK_TIM9_IRQHandler          /* TIM1 Break and TIM9          */         
  IntDefaultHandler, //g    TIM1_UP_TIM10_IRQHandler          /* TIM1 Update and TIM10        */         
  IntDefaultHandler, //g    TIM1_TRG_COM_TIM11_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
  IntDefaultHandler, //g    TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */                          
  IntDefaultHandler, //g    TIM2_IRQHandler                   /* TIM2                         */                   
  IntDefaultHandler, //g    TIM3_IRQHandler                   /* TIM3                         */                   
  IntDefaultHandler, //g    TIM4_IRQHandler                   /* TIM4                         */                   
  IntDefaultHandler, //g    I2C1_EV_IRQHandler                /* I2C1 Event                   */                          
  IntDefaultHandler, //g    I2C1_ER_IRQHandler                /* I2C1 Error                   */                          
  IntDefaultHandler, //g    I2C2_EV_IRQHandler                /* I2C2 Event                   */                          
  IntDefaultHandler, //g    I2C2_ER_IRQHandler                /* I2C2 Error                   */                            
  IntDefaultHandler, //g    SPI1_IRQHandler                   /* SPI1                         */                   
  IntDefaultHandler, //g    SPI2_IRQHandler                   /* SPI2                         */                   
  IntDefaultHandler, //g    USART1_IRQHandler                 /* USART1                       */                   
  IntDefaultHandler, //g    USART2_IRQHandler                 /* USART2                       */                   
  IntDefaultHandler, //g    USART3_IRQHandler                 /* USART3                       */                   
  IntDefaultHandler, //g    EXTI15_10_IRQHandler              /* External Line[15:10]s        */                          
  IntDefaultHandler, //g    RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */                 
  IntDefaultHandler, //g    OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI line */                       
  IntDefaultHandler, //g    TIM8_BRK_TIM12_IRQHandler         /* TIM8 Break and TIM12         */         
  IntDefaultHandler, //g    TIM8_UP_TIM13_IRQHandler          /* TIM8 Update and TIM13        */         
  IntDefaultHandler, //g    TIM8_TRG_COM_TIM14_IRQHandler     /* TIM8 Trigger and Commutation and TIM14 */
  IntDefaultHandler, //g    TIM8_CC_IRQHandler                /* TIM8 Capture Compare         */                          
  IntDefaultHandler, //g    DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */                          
  IntDefaultHandler, //g    FSMC_IRQHandler                   /* FSMC                         */                   
  IntDefaultHandler, //g    SDIO_IRQHandler                   /* SDIO                         */                   
  IntDefaultHandler, //g    TIM5_IRQHandler                   /* TIM5                         */                   
  IntDefaultHandler, //g    SPI3_IRQHandler                   /* SPI3                         */                   
  IntDefaultHandler, //g    UART4_IRQHandler                  /* UART4                        */                   
  IntDefaultHandler, //g    UART5_IRQHandler                  /* UART5                        */                   
  IntDefaultHandler, //g    TIM6_DAC_IRQHandler               /* TIM6 and DAC1&2 underrun errors */                   
  IntDefaultHandler, //g    TIM7_IRQHandler                   /* TIM7                         */
  IntDefaultHandler, //g    DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */                   
  IntDefaultHandler, //g    DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */                   
  IntDefaultHandler, //g    DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */                   
  IntDefaultHandler, //g    DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */                   
  IntDefaultHandler, //g    DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */                   
  IntDefaultHandler, //g    0                                 /* Ethernet                     */                   
  IntDefaultHandler, //g    0                                 /* Ethernet Wakeup through EXTI line */                     
  IntDefaultHandler, //g    CAN2_TX_IRQHandler                /* CAN2 TX                      */                          
  IntDefaultHandler, //g    CAN2_RX0_IRQHandler               /* CAN2 RX0                     */                          
  IntDefaultHandler, //g    CAN2_RX1_IRQHandler               /* CAN2 RX1                     */                          
  IntDefaultHandler, //g    CAN2_SCE_IRQHandler               /* CAN2 SCE                     */                          
  IntDefaultHandler, //g    OTG_FS_IRQHandler                 /* USB OTG FS                   */                   
  IntDefaultHandler, //g    DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */                   
  IntDefaultHandler, //g    DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */                   
  IntDefaultHandler, //g    DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */                   
  IntDefaultHandler, //g    USART6_IRQHandler                 /* USART6                       */                    
  IntDefaultHandler, //g    I2C3_EV_IRQHandler                /* I2C3 event                   */                          
  IntDefaultHandler, //g    I2C3_ER_IRQHandler                /* I2C3 error                   */                          
  IntDefaultHandler, //g    OTG_HS_EP1_OUT_IRQHandler         /* USB OTG HS End Point 1 Out   */                   
  IntDefaultHandler, //g    OTG_HS_EP1_IN_IRQHandler          /* USB OTG HS End Point 1 In    */                   
  IntDefaultHandler, //g    OTG_HS_WKUP_IRQHandler            /* USB OTG HS Wakeup through EXTI */                         
  IntDefaultHandler, //g    OTG_HS_IRQHandler                 /* USB OTG HS                   */                   
  IntDefaultHandler, //g    DCMI_IRQHandler                   /* DCMI                         */                   
  IntDefaultHandler, //g    0                                 /* CRYP crypto                  */                   
  IntDefaultHandler, //g    0                                 /* Hash and Rng                 */
  IntDefaultHandler, //g    FPU_IRQHandler                    /* FPU                          */                         
  IntDefaultHandler, //g    0                                 /* UART7                        */
  IntDefaultHandler, //g    0                                 /* UART8                        */
  IntDefaultHandler, //g    SPI4_IRQHandler                   /* SPI4                         */
  IntDefaultHandler, //g    0                   	      /* SPI5                         */
  IntDefaultHandler, //g    0                   	      /* SPI6                         */
  IntDefaultHandler, //g    SAI1_IRQHandler                   /* SAI1                         */
  IntDefaultHandler, //g    0                   	      /* LTDC                         */
  IntDefaultHandler, //g    0                		      /* LTDC error                   */
  IntDefaultHandler, //g    0                  		      /* DMA2D                        */
  IntDefaultHandler, //g    SAI2_IRQHandler                   /* SAI2                         */
  IntDefaultHandler, //g    QUADSPI_IRQHandler                /* QUADSPI                      */
  IntDefaultHandler, //g    CEC_IRQHandler                    /* CRC                          */
  IntDefaultHandler, //g    SPDIF_RX_IRQHandler               /* SPDIF rx                     */
  IntDefaultHandler, //g    FMPI2C1_Event_IRQHandler          /* FMPI2C1 event                */
  IntDefaultHandler, //g    FMPI2C1_Error_IRQHandler          /* FMPI2C1 error                */
	BootRAM
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
	void
Reset_Handler(void)
{

	uint32_t *pui32Src, *pui32Dest;

	//
	// Copy the data segment initializers from flash to SRAM.
	//
	pui32Src = &_sidata;
	for(pui32Dest = &_sdata; pui32Dest < &_edata; )
	{
		*pui32Dest++ = *pui32Src++;
	}

	//
	// Zero fill the bss segment.
	//
	for(pui32Dest = &_sbss; pui32Dest < &_ebss; )
	{
		*pui32Dest++ = 0;
	}

	//
	// Call the application's entry point.
	//
	SystemInit();
	entry();
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
	static void
NmiSR(void)
{
	//
	// Enter an infinite loop.
	//
	while(1)
	{
	}
}
	static void
SVC_Handler(void)
{
	//
	// Enter an infinite loop.
	//
	while(1)
	{
	}
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
	static void
IntDefaultHandler(void)
{
	//
	// Go into an infinite loop.
	//
	while(1)
	{
	}
}
