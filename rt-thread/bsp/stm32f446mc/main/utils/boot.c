#include <rtthread.h>
#include <stdint.h>
#include <stm32f4xx.h>
#include "md5.h"

typedef  void (*jump_app)(void);
#define APP_ADDRESS 0x08020000
#define APP_LEN_ADDR 0x08008000
#define APP_MD5_ADDR 0x08008004

jump_app jump;

static void app_boot()
{
	if (((*(__IO uint32_t*)(APP_ADDRESS + 4)) & 0xFF000000 ) == 0x08000000)
	{
		uint32_t JumpAddress = *(__IO uint32_t*) (APP_ADDRESS + 4);
		jump = (jump_app) JumpAddress;
		__set_MSP(*(__IO uint32_t*) APP_ADDRESS);
		jump();
	}
}

void verify_and_jump()
{
	uint32_t fw_len = *(__IO uint32_t *)(APP_LEN_ADDR);
	uint8_t fw_md5[16];
	int i;
	uint8_t boot = 1;
	MD5_CTX md5; 
	uint8_t decrypt[16] = {0};    

	rt_memcpy((void *)fw_md5,
			(const void *)(__IO uint8_t *)APP_MD5_ADDR, 16);

	rt_kprintf("fw len: %u %x\r\n", fw_len, fw_len);

	if (fw_len > 256*1024)
		return;

	MD5Init(&md5);
	MD5Update(&md5, (uint8_t *)APP_ADDRESS, fw_len);
	MD5Final(&md5, decrypt);

	for (i=0; i<16; i++) {
		if (decrypt[i] == fw_md5[i])
			rt_kprintf("%02x == %02x\r\n", decrypt[i], fw_md5[i]);
		else {
			rt_kprintf("%02x != %02x\r\n", decrypt[i], fw_md5[i]);
			boot = 0;
		}
	}

	if (boot == 1)
		app_boot();
}
