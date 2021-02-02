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
IPATH += ./hal/CMSIS/Include
IPATH += ./hal/CMSIS/Device/ST/STM32F4xx/Include
IPATH += ./hal/STM32F4xx_HAL_Driver/Inc
IPATH += ./hal/STM32F4xx_HAL_Driver/Inc/Legacy
IPATH += ./inc
IPATH += ../../include
IPATH += ../../components/finsh
IPATH += ./hal/STM32_USB_Device_Library/Class/CustomHID/Inc
IPATH += ./hal/STM32_USB_Device_Library/Class/HID/Inc
IPATH += ./hal/STM32_USB_Device_Library/Core/Inc

VPATH = .
VPATH += ./main
VPATH += ./usb
VPATH += ./main/driver
VPATH += ./main/utils
VPATH += ./hal/STM32F4xx_HAL_Driver/Src
VPATH += ./hal/STM32F4xx_HAL_Driver/Src/Legacy
VPATH += ./hal/CMSIS/Device/ST/STM32F4xx/Source/Templates
VPATH += ./hal/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc_ride7
VPATH += ../../src
VPATH += ../../components/finsh
VPATH += ../../libcpu/arm/cortex-m4
VPATH += ./hal/STM32_USB_Device_Library/Class/CustomHID/Src
VPATH += ./hal/STM32_USB_Device_Library/Class/HID/Src
VPATH += ./hal/STM32_USB_Device_Library/Core/Src
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
${COMPILER}/${VER}.axf: ${COMPILER}/usbd_hid.o
${COMPILER}/${VER}.axf: ${COMPILER}/usbd_ioreq.o
${COMPILER}/${VER}.axf: ${COMPILER}/usbd_ctlreq.o
${COMPILER}/${VER}.axf: ${COMPILER}/usbd_core.o
${COMPILER}/${VER}.axf: ${COMPILER}/usbd_desc.o
${COMPILER}/${VER}.axf: ${COMPILER}/usbd_conf.o
#driver
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_utils.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_usb.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_usart.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_tim.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_spi.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_sdmmc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_rtc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_rng.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_rcc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_pwr.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_lptim.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_i2c.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_gpio.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_fsmc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_fmc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_exti.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_dma2d.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_dma.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_dac.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_crc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_ll_adc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_wwdg.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_usart.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_uart.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_tim_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_tim.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_sram.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_spi.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_spdifrx.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_smbus.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_smartcard.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_sdram.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_sd.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_sai_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_sai.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_rtc_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_rtc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_rng.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_rcc_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_rcc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_qspi.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_pwr_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_pwr.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_pcd_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_pcd.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_pccard.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_nor.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_nand.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_mmc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_ltdc_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_ltdc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_lptim.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_iwdg.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_irda.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_i2s_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_i2s.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_i2c_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_i2c.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_hcd.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_hash_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_hash.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_gpio.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_fmpi2c_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_fmpi2c.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_flash_ramfunc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_flash_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_flash.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_exti.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_eth.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_dsi.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_dma2d.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_dma_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_dma.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_dfsdm.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_dcmi_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_dcmi.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_dac_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_dac.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_cryp_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_cryp.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_crc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_cortex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_cec.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_can.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_adc_ex.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_adc.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal.o
${COMPILER}/${VER}.axf: ${COMPILER}/stm32f4xx_hal_can.o
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
CFLAGSgcc=-DUSE_HAL_DRIVER -DSTM32F413xx -DUSE_USB_FS -DFW_VERSION=\"${VER}\"
#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
