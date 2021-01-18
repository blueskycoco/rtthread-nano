#include <rtthread.h>
#include <stdint.h>
#include <stm32f4xx.h>
typedef  void (*jump_app)(void);
#define APP_ADDRESS 0x08020000

jump_app jump;

void app_boot()
{
	if (((*(__IO uint32_t*)(APP_ADDRESS + 4)) & 0xFF000000 ) == 0x08000000)
	{
		uint32_t JumpAddress = *(__IO uint32_t*) (APP_ADDRESS + 4);
		jump = (jump_app) JumpAddress;
		__set_MSP(*(__IO uint32_t*) APP_ADDRESS);
		//SCB->VTOR = 0x08020000;
		jump();
	}
}
