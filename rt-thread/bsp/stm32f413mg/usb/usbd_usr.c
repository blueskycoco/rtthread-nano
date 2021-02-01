#include "usbd_usr.h"

USBD_Usr_cb_TypeDef USR_cb = {
  USBD_USR_Init,
  USBD_USR_DeviceReset,
  USBD_USR_DeviceConfigured,
  USBD_USR_DeviceSuspended,
  USBD_USR_DeviceResumed,

  USBD_USR_DeviceConnected,
  USBD_USR_DeviceDisconnected,
};

void USBD_USR_Init(void)
{
  rt_kprintf("> USB device library started.\n");
  rt_kprintf("     USB Device Library V1.2.1");
}

void USBD_USR_DeviceReset(uint8_t speed)
{
  switch (speed)
  {
  case USB_OTG_SPEED_HIGH:
    rt_kprintf("     USB Device Library V1.2.1 [HS]");
    break;

  case USB_OTG_SPEED_FULL:
    rt_kprintf("     USB Device Library V1.2.1 [FS]");
    break;
  default:
    rt_kprintf("     USB Device Library V1.2.1 [??]");

  }
}

void USBD_USR_DeviceConfigured(void)
{
  rt_kprintf("> CUSTOMHID Interface started.\n");

}

void USBD_USR_DeviceConnected(void)
{
  rt_kprintf("> USB Device Connected.\n");
}

void USBD_USR_DeviceDisconnected(void)
{
  rt_kprintf("> USB Device Disconnected.\n");
}

void USBD_USR_DeviceSuspended(void)
{
  rt_kprintf("> USB Device in Suspend Mode.\n");
}

void USBD_USR_DeviceResumed(void)
{
  rt_kprintf("> USB Device in Idle Mode.\n");
}

