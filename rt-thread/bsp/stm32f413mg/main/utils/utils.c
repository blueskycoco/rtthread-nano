#include <rtthread.h>
#include <stm32f4xx.h>
#include <stdint.h>
#include <string.h>
#include "utils.h"
#include "mcu_cmd.h"
#include "mcu.h"
extern uint8_t uart_rx_buf[64];
extern uint8_t uart_tx_buf[64];
rt_bool_t warm_boot()
{
	uint32_t flag = 0;
	HAL_PWR_EnableBkUpAccess();
	uint32_t tmp = (uint32_t)0x40002850;
	flag = *(__IO uint32_t *)tmp;
	HAL_PWR_DisableBkUpAccess();
	rt_kprintf("warm boot: %x\r\n", flag);
	if (flag == TYPE_WARM_BOOT)
		return RT_TRUE;
	return RT_FALSE;
}

void uart_tx_set()
{
//	DMA_Cmd(DMA2_Stream6, DISABLE);
//	DMA_SetCurrDataCounter(DMA2_Stream6, 64);
//	DMA_Cmd(DMA2_Stream6, ENABLE);
}

void uart_rx_set()
{
//	DMA_Cmd(DMA2_Stream1, DISABLE);
//	DMA_SetCurrDataCounter(DMA2_Stream1, 64);
//	DMA_Cmd(DMA2_Stream1, ENABLE);
}

void read_ts_64(uint8_t *ts)
{
	rt_memset(ts, 0, 8);
}

void reboot()
{
	//RTC_WriteBackupRegister(RTC_BKP_DR0, 0x00);
	HAL_PWR_EnableBkUpAccess();
	uint32_t tmp = (uint32_t)0x40002850;
	*(__IO uint32_t *)tmp = (uint32_t)0;
	HAL_PWR_DisableBkUpAccess();
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

void dump_mcu_cmd(uint16_t msg_id, uint16_t cmd_id,
		uint8_t *payload, uint16_t len)
{
	int64_val ts;
	uint16_t i = 0;
	char tmp[64] = {0};
	read_ts_64(ts.m_bytes);
	switch (msg_id) {
		case MSG_ID_HW_VER:
		strcpy(tmp, "MSG_ID_HW_VER: ");
		break;
		
		case MSG_ID_SW_VER:
		strcpy(tmp, "MSG_ID_SW_VER: ");
		break;
		
		case MSG_ID_OTA:
		strcpy(tmp, "MSG_ID_OTA: ");
		break;
		
		case HEART_CMD:
//		strcat(tmp,"HEART_CMD: \r\n");
		break;
		
		case HOST_CMD_GET:
		strcpy(tmp,"HOST_CMD_GET: ");
		break;
		
		case HOST_CMD_SET:
		strcpy(tmp,"HOST_CMD_SET: ");
		break;
		
		case DEVICE_EVENT_PSENSOR:
		strcpy(tmp,"Event Psensor : ");
		break;
		
		case DEVICE_EVENT_KEY:
		strcpy(tmp,"Event Brightness Key : ");
		break;
		
		case NR_EVENT_ENV_LIGHT:
//		strcat(tmp,"Event Env Light : \r\n");
		break;
		
		case DEVICE_EVENT_MAG:
//		strcat(tmp,"Event Mag : \r\n");
		break;
		
		case DEVICE_EVENT_VSYNC:
//		strcat(tmp,"Event Vsync : \r\n");
		break;
		
		case DEVICE_EVENT_TMP_HEAD:
		strcpy(tmp,"Event Temp H : ");
		break;
		
		case DEVICE_EVENT_TMP_TAIL:
		strcpy(tmp,"Event Temp T : ");
		break;
		
		default:
		rt_sprintf(tmp,"UnKnown MsgId(%x): ", msg_id);
		break;
	}

	if (msg_id == HOST_CMD_GET || msg_id == HOST_CMD_SET) {
		switch (cmd_id) {
			case NR_BRIGHTNESS:
				 strcat(tmp,"NR_BRIGHTNESS: ");
				 break;
			case NR_DISPLAY_2D_3D:
				 strcat(tmp,"NR_DISPLAY_2D_3D: ");
				 break;
			case NR_DISPLAY_VERSION:
				 strcat(tmp,"NR_DISPLAY_VERSION: ");
				 break;
			case NR_HOST_ID:
				 strcat(tmp,"NR_HOST_ID: ");
				 break;
			case NR_GLASSID:
				 strcat(tmp,"NR_GLASSID: ");
				 break;
			case NR_PSENSOR_CLOSED:
				 strcat(tmp,"NR_PSENSOR_CLOSED: ");
				 break;
			case NR_PSENSOR_NOCLOSED:
				 strcat(tmp,"NR_PSENSOR_NOCLOSED: ");
				 break;
			case NR_TEMPERATURE:
				 strcat(tmp,"NR_TEMPERATURE: ");
				 break;
			case NR_DISPLAY_DUTY:
				 strcat(tmp,"NR_DISPLAY_DUTY: ");
				 break;
			case NR_POWER_FUCTION:
				 strcat(tmp,"NR_POWER_FUCTION: ");
				 break;
			case NR_MAGNETIC_FUCTION:
				 strcat(tmp,"NR_MAGNETIC_FUCTION: ");
				 break;
			case NR_VSYNC_FUCTION:
				 strcat(tmp,"NR_VSYNC_FUCTION: ");
				 break;
			case NR_ENV_LIGHT:
				 strcat(tmp,"NR_ENV_LIGHT: ");
				 break;
			case NR_WORLD_LED:
				 strcat(tmp,"NR_WORLD_LED: ");
				 break;
			case NR_SLEEP_TIME:
				 strcat(tmp,"NR_SLEEP_TIME: ");
				 break;
			case NR_7211_VERSION:
				 strcat(tmp,"NR_7211_VERSION: ");
				 break;
			case NR_7211_UPDATE:
				 strcat(tmp,"NR_7211_UPDATE: ");
				 break;
			case NR_REBOOT:
				 strcat(tmp,"NR_REBOOT: ");
				 break;
			case NR_BRIGHTNESS_EXT:
				 strcat(tmp,"NR_BRIGHTNESS_EXT: ");
				 break;
			case NR_TEMPERATURE_FUNCTION:
				 strcat(tmp,
				 		 "NR_TEMPERATURE_FUNCTION: ");
				 break;
			case NR_TRY_CTRL_DISPLAY_STATUS:
				 strcat(tmp,
				 	"NR_TRY_CTRL_DISPLAY_STATUS: ");
				 break;
			case NR_TEMPERATURE_EXT:
				 strcat(tmp,"NR_TEMPERATURE_EXT: ");
				 break;
			case NR_BRIGHTNESS_MAX:
				 strcat(tmp,"NR_BRIGHTNESS_MAX: ");
				 break;
			case NR_SPEAKER_LEVEL:
				 strcat(tmp,"NR_SPEAKER_LEVEL: ");
				 break;
			case NR_CPU_INFO:
				 strcat(tmp,"NR_CPU_INFO: ");
				 break;
			case NR_ROM_INFO:
				 strcat(tmp,"NR_ROM_INFO: ");
				 break;
			case NR_RAM_INFO:
				 strcat(tmp,"NR_RAM_INFO: ");
				 break;
			case NR_LEFT_OLED_H_ORBIT:
				 strcat(tmp,
				 		 "NR_LEFT_OLED_H_ORBIT: ");
				 break;
			case NR_LEFT_OLED_V_ORBIT:
				 strcat(tmp,
				 		 "NR_LEFT_OLED_V_ORBIT: ");
				 break;
			case NR_RIGHT_OLED_H_ORBIT:
				 strcat(tmp,
				 		 "NR_RIGHT_OLED_H_ORBIT: ");
				 break;
			case NR_RIGHT_OLED_V_ORBIT:
				 strcat(tmp,
				 		 "NR_RIGHT_OLED_V_ORBIT: ");
				 break;
			case NR_ORBIT_ADJUST:
				 strcat(tmp,"NR_ORBIT_ADJUST: ");
				 break;
			case NR_COLOR:
				 strcat(tmp,"NR_COLOR: ");
				 break;
			case NR_RECOVERY_FACTORY:
				 strcat(tmp,
				 		 "NR_RECOVERY_FACTORY: ");
				 break;
			case NR_GLASS_BRI_TEST:
				 strcat(tmp,"NR_GLASS_BRI_TEST: ");
				 break;
			default:
				 strcat(tmp,"UnKnow CmdId(): ");
				 break;

		}
	}

	rt_kprintf("%d: %s\r\n", msg_id, tmp);
	if (len != 0 &&
		msg_id != DEVICE_EVENT_MAG &&
		msg_id != HEART_CMD &&
		msg_id != DEVICE_EVENT_VSYNC &&
		msg_id != NR_EVENT_ENV_LIGHT) {
	for (i=0; i<len; i++)
		rt_kprintf("0x%x \r\n", payload[i]);
	rt_kprintf("\r\n");
	}
}
