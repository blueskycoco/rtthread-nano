#include <rtthread.h>
#include "utils.h"
#include <stm32f4xx.h>

rt_bool_t warm_boot()
{
	if (RTC_ReadBackupRegister(RTC_BKP_DR0) == TYPE_WARM_BOOT)
		return RT_TRUE;

	return RT_FALSE;
}
