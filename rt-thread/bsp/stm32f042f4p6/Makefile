#******************************************************************************
#
# Makefile - Rules for building the rtthread example.
#
# Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
# 
# Texas Instruments (TI) is supplying this software for use solely and
# exclusively on TI's microcontroller products. The software is owned by
# TI and/or its suppliers, and is protected under applicable copyright
# laws. You may not combine this software with "viral" open-source
# software in order to form a larger program.
# 
# THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
# NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
# NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
# CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES, FOR ANY REASON WHATSOEVER.
# 
# This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
#
#******************************************************************************

#
# The base directory for TivaWare.
#
ROOT=.

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find header files that do not live in the source directory.
#
IPATH = ./
IPATH += ./inc
IPATH += ./Libraries/CMSIS/Include
IPATH += ./Libraries/CMSIS/Device/ST/STM32F0xx/Include
IPATH += ./Libraries/STM32_USB_Device_Driver/inc
IPATH += ./Libraries/STM32_USB_Device_Library/Class/hid/inc
IPATH += ./Libraries/STM32_USB_Device_Library/Core/inc
IPATH += ./Libraries/STM32F0xx_StdPeriph_Driver/inc
IPATH += ../../include
IPATH += ../../components/finsh

VPATH = .
VPATH += ./main
VPATH += ./usb
VPATH += ./Libraries/STM32F0xx_StdPeriph_Driver/src
VPATH += ./Libraries/STM32_USB_Device_Library/Class/hid/src
VPATH += ./Libraries/STM32_USB_Device_Library/Core/src
VPATH += ./Libraries/STM32_USB_Device_Driver/src
VPATH += ./Libraries/CMSIS/Device/ST/STM32F0xx/Source/Templates
VPATH += ./libraries
VPATH += ../../src
VPATH += ../../components/finsh
VPATH += ../../libcpu/arm/cortex-m0
#
# The default rule, which causes the rtthread example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/rtthread.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

install:
	jlink.exe burn.txt
#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the rtthread example.
#
#init
${COMPILER}/rtthread.axf: ${COMPILER}/startup_${COMPILER}_042f4p6.o
${COMPILER}/rtthread.axf: ${COMPILER}/system_stm32f0xx.o
${COMPILER}/rtthread.axf: ${COMPILER}/main.o
#${COMPILER}/rtthread.axf: ${COMPILER}/imu.o
${COMPILER}/rtthread.axf: ${COMPILER}/board.o
${COMPILER}/rtthread.axf: ${COMPILER}/ymodem.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32_it.o
${COMPILER}/rtthread.axf: ${COMPILER}/usb_bsp.o
${COMPILER}/rtthread.axf: ${COMPILER}/usbd_desc.o
${COMPILER}/rtthread.axf: ${COMPILER}/usbd_pwr.o
${COMPILER}/rtthread.axf: ${COMPILER}/usbd_usr.o
#${COMPILER}/rtthread.axf: ${COMPILER}/imu_jump_detection-gyro.o
#${COMPILER}/rtthread.axf: ${COMPILER}/imu_jump_detection.o
#driver
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_gpio.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_misc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_adc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_comp.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_can.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_cec.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_crc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_dac.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_dbgmcu.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_dma.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_exti.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_flash.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_crs.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_i2c.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_iwdg.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_pwr.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_rcc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_rtc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_syscfg.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_spi.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_tim.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_usart.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f0xx_wwdg.o
${COMPILER}/rtthread.axf: ${COMPILER}/usbd_custom_hid_core.o
${COMPILER}/rtthread.axf: ${COMPILER}/usbd_core.o
${COMPILER}/rtthread.axf: ${COMPILER}/usbd_ioreq.o
${COMPILER}/rtthread.axf: ${COMPILER}/usbd_req.o
${COMPILER}/rtthread.axf: ${COMPILER}/usb_core.o
${COMPILER}/rtthread.axf: ${COMPILER}/usb_dcd.o
${COMPILER}/rtthread.axf: ${COMPILER}/usb_dcd_int.o
#rtthread
${COMPILER}/rtthread.axf: ${COMPILER}/clock.o
${COMPILER}/rtthread.axf: ${COMPILER}/components.o
${COMPILER}/rtthread.axf: ${COMPILER}/context_gcc.o
${COMPILER}/rtthread.axf: ${COMPILER}/cpuport.o
${COMPILER}/rtthread.axf: ${COMPILER}/idle.o
${COMPILER}/rtthread.axf: ${COMPILER}/ipc.o
${COMPILER}/rtthread.axf: ${COMPILER}/irq.o
${COMPILER}/rtthread.axf: ${COMPILER}/kservice.o
${COMPILER}/rtthread.axf: ${COMPILER}/mem.o
${COMPILER}/rtthread.axf: ${COMPILER}/memheap.o
${COMPILER}/rtthread.axf: ${COMPILER}/mempool.o
${COMPILER}/rtthread.axf: ${COMPILER}/object.o
${COMPILER}/rtthread.axf: ${COMPILER}/scheduler.o
${COMPILER}/rtthread.axf: ${COMPILER}/thread.o
${COMPILER}/rtthread.axf: ${COMPILER}/timer.o
${COMPILER}/rtthread.axf: ${COMPILER}/cmd.o
${COMPILER}/rtthread.axf: ${COMPILER}/msh.o
${COMPILER}/rtthread.axf: ${COMPILER}/shell.o
${COMPILER}/rtthread.axf: rtthread.ld
SCATTERgcc_rtthread=rtthread.ld
ENTRY_rtthread=Reset_Handler
#CFLAGSgcc=-DUSE_STDPERIPH_DRIVER -DSTM32F10X_HD -DMASTER
CFLAGSgcc=-DUSE_STDPERIPH_DRIVER -DSTM32F042
#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
