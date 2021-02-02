#******************************************************************************
#
# Makefile - Rules for building the ${VER} example.
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
IPATH = ./main
IPATH += ./main/driver
IPATH += ./main/utils
IPATH += ./inc
IPATH += ./Libraries/CMSIS/Include
IPATH += ./Libraries/CMSIS/Device/ST/STM32F4xx/Include
IPATH += ./Libraries/STM32F4xx_StdPeriph_Driver/inc
IPATH += ../../include
IPATH += ../../components/finsh
IPATH += ./Libraries/STM32_USB_Device_Library/Class/customhid/inc
IPATH += ./Libraries/STM32_USB_Device_Library/Core/inc
IPATH += ./Libraries/STM32_USB_OTG_Driver/inc

VPATH = .
VPATH += ./main
VPATH += ./usb
VPATH += ./main/driver
VPATH += ./main/utils
VPATH += ./Libraries/STM32F4xx_StdPeriph_Driver/src
VPATH += ./Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates
VPATH += ./Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc_ride7
VPATH += ./libraries
VPATH += ../../src
VPATH += ../../components/finsh
VPATH += ../../libcpu/arm/cortex-m4
VPATH += ./Libraries/STM32_USB_OTG_Driver/src
VPATH += ./Libraries/STM32_USB_Device_Library/Class/customhid/src
VPATH += ./Libraries/STM32_USB_Device_Library/Core/src
#
# The default rule, which causes the ${VER} example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/${VER}.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

install:
	openocd -f openocd.cfg -c "flash_image ${VER}.bin"
#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the ${VER} example.
#
#init
${COMPILER}/${VER}.axf: ${COMPILER}/startup_stm32f413xx.o
${COMPILER}/${VER}.axf: ${COMPILER}/system_stm32f4xx.o
${COMPILER}/${VER}.axf: ${COMPILER}/main.o
${COMPILER}/${VER}.axf: ${COMPILER}/board.o
${COMPILER}/${VER}.axf: ${COMPILER}/ymodem.o
${COMPILER}/${VER}.axf: ${COMPILER}/md5.o
${COMPILER}/${VER}.axf: ${COMPILER}/boot.o
${COMPILER}/${VER}.axf: ${COMPILER}/mem_list.o
${COMPILER}/${VER}.axf: ${COMPILER}/mcu.o
${COMPILER}/${VER}.axf: ${COMPILER}/rc4.o
${COMPILER}/${VER}.axf: ${COMPILER}/crc.o
${COMPILER}/${VER}.axf: ${COMPILER}/utils.o
${COMPILER}/${VER}.axf: ${COMPILER}/flash_if.o
${COMPILER}/${VER}.axf: ${COMPILER}/usb_bsp.o
${COMPILER}/${VER}.axf: ${COMPILER}/usbd_desc.o
${COMPILER}/${VER}.axf: ${COMPILER}/usbd_usr.o
${COMPILER}/${VER}.axf: ${COMPILER}/usbd_customhid_core.o
${COMPILER}/${VER}.axf: ${COMPILER}/usbd_core.o
${COMPILER}/${VER}.axf: ${COMPILER}/usbd_ioreq.o
${COMPILER}/${VER}.axf: ${COMPILER}/usbd_req.o
${COMPILER}/${VER}.axf: ${COMPILER}/usb_core.o
${COMPILER}/${VER}.axf: ${COMPILER}/usb_dcd.o
${COMPILER}/${VER}.axf: ${COMPILER}/usb_dcd_int.o
#${COMPILER}/${VER}.axf: ${COMPILER}/stm32_it.o
#driver
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_gpio.o
${COMPILER}/${VER}.axf: ${COMPILER}/misc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_adc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_can.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_cec.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_crc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_dac.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_dbgmcu.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_dma.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_exti.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_flash.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_i2c.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_iwdg.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_pwr.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_rcc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_rtc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_syscfg.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_spi.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_tim.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_usart.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_wwdg.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_cryp.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_cryp_aes.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_cryp_des.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_cryp_tdes.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_dcmi.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_dfsdm.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_dma2d.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_dsi.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_flash_ramfunc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_fmpi2c.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hash.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hash_md5.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hash_sha1.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_lptim.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ltdc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_qspi.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_rng.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_sai.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_sdio.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_spdifrx.o
#${VER}
${COMPILER}/${VER}.axf: ${COMPILER}/clock.o
${COMPILER}/${VER}.axf: ${COMPILER}/components.o
${COMPILER}/${VER}.axf: ${COMPILER}/context_gcc.o
${COMPILER}/${VER}.axf: ${COMPILER}/cpuport.o
${COMPILER}/${VER}.axf: ${COMPILER}/mem.o
${COMPILER}/${VER}.axf: ${COMPILER}/idle.o
${COMPILER}/${VER}.axf: ${COMPILER}/ipc.o
${COMPILER}/${VER}.axf: ${COMPILER}/irq.o
${COMPILER}/${VER}.axf: ${COMPILER}/kservice.o
${COMPILER}/${VER}.axf: ${COMPILER}/mem.o
${COMPILER}/${VER}.axf: ${COMPILER}/memheap.o
${COMPILER}/${VER}.axf: ${COMPILER}/mempool.o
${COMPILER}/${VER}.axf: ${COMPILER}/object.o
${COMPILER}/${VER}.axf: ${COMPILER}/scheduler.o
${COMPILER}/${VER}.axf: ${COMPILER}/thread.o
${COMPILER}/${VER}.axf: ${COMPILER}/timer.o
${COMPILER}/${VER}.axf: ${COMPILER}/cmd.o
${COMPILER}/${VER}.axf: ${COMPILER}/msh.o
${COMPILER}/${VER}.axf: ${COMPILER}/shell.o
${COMPILER}/${VER}.axf: rtthread.ld
SCATTERgcc_${VER}=rtthread.ld
ENTRY_${VER}=Reset_Handler
CFLAGSgcc=-DUSE_STDPERIPH_DRIVER -DSTM32F413_423xx -DFW_VERSION=\"${VER}\"
#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
