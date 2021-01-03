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
#include <stm32f0xx.h>
struct rt_semaphore sem;
rt_uint8_t uart_rcv[512];
rt_uint16_t uart_rcv_len;
rt_uint8_t state = 0;
rt_uint8_t flag = 0;
enum {
	STATE_INIT,
	STATE_INIT1,
	STATE_TCP_LINK,
	STATE_AP_LINK,
	STATE_SET_SSID,
	STATE_SET_KEY,
	STATE_THROUGH,
	STATE_STA_MODE,
	STATE_TCP_CRT,
	STATE_TCP_ON,
	STATE_SET_MODE,
	STATE_SET_REBOOT,
};
void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void enter_at_mode()
{
	rt_hw_console_output1("+++");
	rt_thread_mdelay(100);
	rt_hw_console_output1("a");
	rt_thread_mdelay(100);
}
void wifi_configure()
{
	switch (state) {
		case STATE_INIT:
			state = STATE_INIT1;
			rt_hw_console_output1("at+tcplk\n");
			break;
		case STATE_INIT1:
			state = STATE_TCP_LINK;
			rt_hw_console_output1("at+tcplk\n");
			break;
		case STATE_TCP_LINK:
			if (rt_strstr(uart_rcv, "on") != RT_NULL) {
				state = STATE_THROUGH;
				rt_hw_console_output1("at+entm\n");
			} else {
				state = STATE_AP_LINK;
				rt_hw_console_output1("at+z\n");
			}
			break;
		case STATE_AP_LINK:
			if (rt_strstr(uart_rcv, "Disconnected") == RT_NULL &&
				rt_strstr(uart_rcv, "RF Off") == RT_NULL) {
				state = STATE_TCP_CRT;
				rt_hw_console_output1("at+tcpdis=off\n");
			} else {
				state = STATE_SET_SSID;
				rt_hw_console_output1("at+wsssid=CMCC-333\n");
			}
			break;
		case STATE_SET_SSID:
    			rt_hw_console_output1("at+wskey=wpa2psk,aes,minfei520tanghua1314\n");
    			state = STATE_SET_MODE;
    			break;
		case STATE_SET_MODE:
    			rt_hw_console_output1("at+wmode=sta\n");
    			state = STATE_SET_REBOOT;
    			break;
		case STATE_SET_REBOOT:
    			rt_hw_console_output1("at+z\n");
    			state = STATE_TCP_LINK;
    			break;
		case STATE_TCP_CRT:
				rt_hw_console_output1("at+netp=TCP,SERVER,8899,192.168.1.2\n");
				state = STATE_TCP_ON;
			break;
		case STATE_TCP_ON:
				state = STATE_INIT1;
				rt_hw_console_output1("at+tcpdis=on\n");
			break;
		default:
			rt_hw_console_output1(uart_rcv);
			break;
	}
}
int main(void)
{

    led_init();
	
	rt_sem_init(&sem, "shrx", 0, 0);
	enter_at_mode();
	while (1)
    {

#if 0
		if (flag == 0 || flag == 1) {
    		flag++;
    		rt_hw_console_output1("at+wsssid=xxx\n");
		} else if (flag == 2) {
    		flag++;
    		rt_hw_console_output1("at+wskey=wpa2psk,aes,yyy\n");
		} else if (flag == 3) {
			flag++;
    		rt_hw_console_output1("at+wslk\n");
		} else if (flag == 4) {
			flag++;
    		rt_hw_console_output1("at+wslq\n");
		}
#else
#if 0
		if (flag == 0 || flag == 1) {
    		flag++;
    		rt_hw_console_output1("at+reld\n");
		} else
    		rt_hw_console_output1("at+z\n");
#else
	wifi_configure();
	rt_memset(uart_rcv, 0, 512);
#endif
#endif
    	rt_sem_take(&sem, RT_WAITING_FOREVER);
    }
}

