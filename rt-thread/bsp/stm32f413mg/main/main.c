/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-11-05     yangjie      First edition
 */

#include <rtthread.h>
#include "md5.h"
#include "ymodem.h"
#include "boot.h"
#include "utils.h"
#include "mcu.h"
#include "stm32f4xx_hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h" 

USBD_HandleTypeDef USBD_Device;
extern PCD_HandleTypeDef hpcd;

int main(void)
{
	uint8_t flag = 0;

	USBD_Init(&USBD_Device, &HID_Desc, 0);
  	USBD_RegisterClass(&USBD_Device, USBD_HID_CLASS);
  	USBD_Start(&USBD_Device);

	while (1)
	{
		rt_thread_mdelay(1000);
	}
}

