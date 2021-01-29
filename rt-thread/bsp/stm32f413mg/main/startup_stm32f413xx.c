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
extern void SysTick_Handler(void);
extern void PendSV_Handler(void);
extern void HardFault_Handler(void);
extern void OTG_FS_IRQHandler(void);
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
	IntDefaultHandler, // RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */                      
	IntDefaultHandler, // FLASH_IRQHandler                  /* FLASH                        */                                          
	IntDefaultHandler, // RCC_IRQHandler                    /* RCC                          */                                            
	IntDefaultHandler, // EXTI0_IRQHandler                  /* EXTI Line0                   */                        
	IntDefaultHandler, // EXTI1_IRQHandler                  /* EXTI Line1                   */                          
	IntDefaultHandler, // EXTI2_IRQHandler                  /* EXTI Line2                   */                          
	IntDefaultHandler, // EXTI3_IRQHandler                  /* EXTI Line3                   */                          
	IntDefaultHandler, // EXTI4_IRQHandler                  /* EXTI Line4                   */                          
	IntDefaultHandler, // DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */                  
	IntDefaultHandler, // DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */                   
	IntDefaultHandler, // DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */                   
	IntDefaultHandler, // DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */                   
	IntDefaultHandler, // DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */                   
	IntDefaultHandler, // DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */                   
	IntDefaultHandler, // DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */                   
	IntDefaultHandler, // ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */                   
	IntDefaultHandler, // CAN1_TX_IRQHandler                /* CAN1 TX                      */                         
	IntDefaultHandler, // CAN1_RX0_IRQHandler               /* CAN1 RX0                     */                          
	IntDefaultHandler, // CAN1_RX1_IRQHandler               /* CAN1 RX1                     */                          
	IntDefaultHandler, // CAN1_SCE_IRQHandler               /* CAN1 SCE                     */                          
	IntDefaultHandler, // EXTI9_5_IRQHandler                /* External Line[9:5]s          */                          
	IntDefaultHandler, // TIM1_BRK_TIM9_IRQHandler          /* TIM1 Break and TIM9          */         
	IntDefaultHandler, // TIM1_UP_TIM10_IRQHandler          /* TIM1 Update and TIM10        */         
	IntDefaultHandler, // TIM1_TRG_COM_TIM11_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
	IntDefaultHandler, // TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */                          
	IntDefaultHandler, // TIM2_IRQHandler                   /* TIM2                         */                   
	IntDefaultHandler, // TIM3_IRQHandler                   /* TIM3                         */                   
	IntDefaultHandler, // TIM4_IRQHandler                   /* TIM4                         */                   
	IntDefaultHandler, // I2C1_EV_IRQHandler                /* I2C1 Event                   */                          
	IntDefaultHandler, // I2C1_ER_IRQHandler                /* I2C1 Error                   */                          
	IntDefaultHandler, // I2C2_EV_IRQHandler                /* I2C2 Event                   */                          
	IntDefaultHandler, // I2C2_ER_IRQHandler                /* I2C2 Error                   */                            
	IntDefaultHandler, // SPI1_IRQHandler                   /* SPI1                         */                   
	IntDefaultHandler, // SPI2_IRQHandler                   /* SPI2                         */                   
	IntDefaultHandler, // USART1_IRQHandler                 /* USART1                       */                   
	IntDefaultHandler, // USART2_IRQHandler                 /* USART2                       */                   
	IntDefaultHandler, // USART3_IRQHandler                 /* USART3                       */                   
	IntDefaultHandler, // EXTI15_10_IRQHandler              /* External Line[15:10]s        */                          
	IntDefaultHandler, // RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */                 
	IntDefaultHandler, // OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI line */                       
	IntDefaultHandler, // TIM8_BRK_TIM12_IRQHandler         /* TIM8 Break and TIM12         */         
	IntDefaultHandler, // TIM8_UP_TIM13_IRQHandler          /* TIM8 Update and TIM13        */         
	IntDefaultHandler, // TIM8_TRG_COM_TIM14_IRQHandler     /* TIM8 Trigger and Commutation and TIM14 */
	IntDefaultHandler, // TIM8_CC_IRQHandler                /* TIM8 Capture Compare         */                          
	IntDefaultHandler, // DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */                          
	IntDefaultHandler, // FSMC_IRQHandler                   /* FSMC                         */                   
	IntDefaultHandler, // SDIO_IRQHandler                   /* SDIO                         */                   
	IntDefaultHandler, // TIM5_IRQHandler                   /* TIM5                         */                   
	IntDefaultHandler, // SPI3_IRQHandler                   /* SPI3                         */                   
	IntDefaultHandler, // UART4_IRQHandler                  /* UART4                        */                   
	IntDefaultHandler, // UART5_IRQHandler                  /* UART5                        */                   
	IntDefaultHandler, // TIM6_DAC_IRQHandler               /* TIM6 and DAC1&2 underrun errors */                   
	IntDefaultHandler, // TIM7_IRQHandler                   /* TIM7                         */
	IntDefaultHandler, // DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */                   
	IntDefaultHandler, // DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */                   
	IntDefaultHandler, // DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */                   
	IntDefaultHandler, // DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */                   
	IntDefaultHandler, // DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */                   
	IntDefaultHandler, // DFSDM1_FLT0_IRQHandler                                 /* Ethernet                     */                   
	IntDefaultHandler, // DFSDM1_FLT1_IRQHandler                                 /* Ethernet Wakeup through EXTI line */                     
	IntDefaultHandler, // CAN2_TX_IRQHandler                /* CAN2 TX                      */                          
	IntDefaultHandler, // CAN2_RX0_IRQHandler               /* CAN2 RX0                     */                          
	IntDefaultHandler, // CAN2_RX1_IRQHandler               /* CAN2 RX1                     */                          
	IntDefaultHandler, // CAN2_SCE_IRQHandler               /* CAN2 SCE                     */                          
	IntDefaultHandler, // OTG_FS_IRQHandler                 /* USB OTG FS                   */                   
	IntDefaultHandler, // DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */                   
	IntDefaultHandler, // DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */                   
	IntDefaultHandler, // DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */                   
	IntDefaultHandler, // USART6_IRQHandler                 /* USART6                       */                    
	IntDefaultHandler, // I2C3_EV_IRQHandler                /* I2C3 event                   */                          
	IntDefaultHandler, // I2C3_ER_IRQHandler                /* I2C3 error                   */                          
	IntDefaultHandler, // CAN3_TX_IRQHandler         /* USB OTG HS End Point 1 Out   */                   
	IntDefaultHandler, // CAN3_RX0_IRQHandler          /* USB OTG HS End Point 1 In    */                   
	IntDefaultHandler, // CAN3_RX1_IRQHandler            /* USB OTG HS Wakeup through EXTI */                         
	IntDefaultHandler, // CAN3_SCE_IRQHandler                 /* USB OTG HS                   */                   
	IntDefaultHandler, // 0                   /* DCMI                         */                   
	IntDefaultHandler, // AES_IRQHandler                                 /* CRYP crypto                  */                   
	IntDefaultHandler, // RNG_IRQHandler                                 /* Hash and Rn              */
	IntDefaultHandler, // FPU_IRQHandler                    /* FPU                          */                         
	IntDefaultHandler, // UART7_IRQHandler                                 /* UART7                        */
	IntDefaultHandler, // UART8_IRQHandler                                 /* UART8                        */
	IntDefaultHandler, // SPI4_IRQHandler                   /* SPI4                         */
	IntDefaultHandler, // SPI5_IRQHandler                   	      /* SPI5                         */
	IntDefaultHandler, // 0                   	      /* SPI6                         */
	IntDefaultHandler, // SAI1_IRQHandler                   /* SAI1                         */
	IntDefaultHandler, // UART9_IRQHandler                   	      /* LTDC                         */
	IntDefaultHandler, // UART10_IRQHandler                		      /* LTDC error                   */
	IntDefaultHandler, // 0                  		      /* DMA2D                        */
	IntDefaultHandler, // 0                   /* SAI2                         */
	IntDefaultHandler, // QUADSPI_IRQHandler                /* QUADSPI                      */
	IntDefaultHandler, // 0                    /* CRC                          */
	IntDefaultHandler, // 0               /* SPDIF rx                     */
	IntDefaultHandler, // FMPI2C1_Event_IRQHandler          /* FMPI2C1 event                */
	IntDefaultHandler, // FMPI2C1_Error_IRQHandler          /* FMPI2C1 error                */
	IntDefaultHandler, // LPTIM1_IRQHandler          /* FMPI2C1 error                */
	IntDefaultHandler, // DFSDM2_FLT0_IRQHandler          /* FMPI2C1 error                */
	IntDefaultHandler, // DFSDM2_FLT1_IRQHandler          /* FMPI2C1 error                */
	IntDefaultHandler, // DFSDM2_FLT2_IRQHandler          /* FMPI2C1 error                */
	IntDefaultHandler, // DFSDM2_FLT3_IRQHandler          /* FMPI2C1 error                */
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
	//for(pui32Dest = &_sbss; pui32Dest < &_ebss; )
	//{
	//	*pui32Dest++ = 0;
	//}

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
