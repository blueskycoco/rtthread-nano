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
IPATH = ./main
IPATH += ./main/driver
IPATH += ./main/utils
IPATH += ./inc
IPATH += ./Libraries/CMSIS/Include
IPATH += ./Libraries/CMSIS/Device/ST/STM32F4xx/Include
IPATH += ./Libraries/STM32F4xx_StdPeriph_Driver/inc
IPATH += ../../include
IPATH += ../../components/finsh

VPATH = .
VPATH += ./main
VPATH += ./main/driver
VPATH += ./main/utils
VPATH += ./Libraries/STM32F4xx_StdPeriph_Driver/src
VPATH += ./Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates
VPATH += ./Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc_ride7
VPATH += ./libraries
VPATH += ../../src
VPATH += ../../components/finsh
VPATH += ../../libcpu/arm/cortex-m4
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
#${COMPILER}/rtthread.axf: ${COMPILER}/startup_gcc_446ze.o
${COMPILER}/rtthread.axf: ${COMPILER}/startup_stm32f446xx.o
${COMPILER}/rtthread.axf: ${COMPILER}/system_stm32f4xx.o
${COMPILER}/rtthread.axf: ${COMPILER}/main.o
${COMPILER}/rtthread.axf: ${COMPILER}/board.o
${COMPILER}/rtthread.axf: ${COMPILER}/ymodem.o
${COMPILER}/rtthread.axf: ${COMPILER}/md5.o
${COMPILER}/rtthread.axf: ${COMPILER}/boot.o
${COMPILER}/rtthread.axf: ${COMPILER}/mem_list.o
${COMPILER}/rtthread.axf: ${COMPILER}/mcu.o
${COMPILER}/rtthread.axf: ${COMPILER}/rc4.o
${COMPILER}/rtthread.axf: ${COMPILER}/crc.o
${COMPILER}/rtthread.axf: ${COMPILER}/flash_if.o
#${COMPILER}/rtthread.axf: ${COMPILER}/stm32_it.o
#driver
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_gpio.o
${COMPILER}/rtthread.axf: ${COMPILER}/misc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_adc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_can.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_cec.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_crc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_dac.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_dbgmcu.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_dma.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_exti.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_flash.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_i2c.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_iwdg.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_pwr.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_rcc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_rtc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_syscfg.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_spi.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_tim.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_usart.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_wwdg.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_cryp.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_cryp_aes.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_cryp_des.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_cryp_tdes.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_dcmi.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_dfsdm.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_dma2d.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_dsi.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_flash_ramfunc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_fmc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_fmpi2c.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_hash.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_hash_md5.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_hash_sha1.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_lptim.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_ltdc.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_qspi.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_rng.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_sai.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_sdio.o
${COMPILER}/rtthread.axf: ${COMPILER}/stm32f4xx_spdifrx.o
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
CFLAGSgcc=-DUSE_STDPERIPH_DRIVER -DSTM32F446xx -DFW_VERSION=\"${VER}\"
#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
