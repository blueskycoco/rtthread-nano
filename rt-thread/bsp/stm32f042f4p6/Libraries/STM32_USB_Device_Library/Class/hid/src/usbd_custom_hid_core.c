#include <rtthread.h>
#include "usbd_custom_hid_core.h"
#include "mem_list.h"

extern uint32_t sof_timer;
extern rt_bool_t hid_sent;
uint8_t  USBD_HID_Init (void  *pdev, uint8_t cfgidx);
uint8_t  USBD_HID_DeInit (void  *pdev, uint8_t cfgidx);
uint8_t  USBD_HID_Setup (void  *pdev, USB_SETUP_REQ *req);
uint8_t  *USBD_HID_GetCfgDesc (uint8_t speed, uint16_t *length);
uint8_t  USBD_HID_DataIn (void  *pdev, uint8_t epnum);
uint8_t  USBD_HID_DataOut (void  *pdev, uint8_t epnum);
uint8_t  USBD_HID_EP0_RxReady (void  *pdev);
uint8_t  USBD_HID_SOF (void  *pdev);

USBD_Class_cb_TypeDef  USBD_HID_cb = 
{
	USBD_HID_Init,
	USBD_HID_DeInit,
	USBD_HID_Setup,
	NULL,			/*EP0_TxSent*/
	USBD_HID_EP0_RxReady,	/*EP0_RxReady*/
	USBD_HID_DataIn,	/*DataIn*/
	USBD_HID_DataOut,	/*DataOut*/
	USBD_HID_SOF,			/*SOF */
	USBD_HID_GetCfgDesc, 
};



uint8_t Report_buf[64];
uint8_t USBD_HID_Report_ID=0;
uint8_t flag = 0;
static uint32_t  USBD_HID_AltSet = 0;
static uint32_t  USBD_HID_Protocol = 0;
static uint32_t  USBD_HID_IdleState = 0;

/* USB HID device Configuration Descriptor */
const uint8_t USBD_HID_CfgDesc[CUSTOMHID_SIZ_CONFIG_DESC] =
{
	0x09, /* bLength: Configuration Descriptor size */
	USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
	CUSTOMHID_SIZ_CONFIG_DESC,
	/* wTotalLength: Bytes returned */
	0x00,
	0x01,         /*bNumInterfaces: 1 interface*/
	0x01,         /*bConfigurationValue: Configuration value*/
	0x00,         /*iConfiguration: Index of string descriptor describing
			the configuration*/
	0xC0,         /*bmAttributes: bus powered and Support Remote Wake-up */
	0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

	/************** Descriptor of Custom HID interface ****************/
	/* 09 */
	0x09,         /*bLength: Interface Descriptor size*/
	USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
	0x00,         /*bInterfaceNumber: Number of Interface*/
	0x00,         /*bAlternateSetting: Alternate setting*/
	0x02,         /*bNumEndpoints*/
	0x03,         /*bInterfaceClass: HID*/
	0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
	0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
	0,            /*iInterface: Index of string descriptor*/
	/******************** Descriptor of Custom HID ********************/
	/* 18 */
	0x09,         /*bLength: HID Descriptor size*/
	HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
	0x11,         /*bcdHID: HID Class Spec release number*/
	0x01,
	0x00,         /*bCountryCode: Hardware target country*/
	0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
	0x22,         /*bDescriptorType*/
	CUSTOMHID_SIZ_REPORT_DESC,/*wItemLength: Total length of Report descriptor*/
	0x00,
	/******************** Descriptor of Custom HID endpoints ***********/
	/* 27 */
	0x07,          /* bLength: Endpoint Descriptor size */
	USB_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType: */

	HID_IN_EP,     /* bEndpointAddress: Endpoint Address (IN) */
	0x03,          /* bmAttributes: Interrupt endpoint */
	HID_IN_PACKET, /* wMaxPacketSize: 2 Bytes max */
	0x00,
	0x00,          /* bInterval: Polling Interval (32 ms) */
	/* 34 */

	0x07,	         /* bLength: Endpoint Descriptor size */
	USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType: */
	/*	Endpoint descriptor type */
	HID_OUT_EP,	/* bEndpointAddress: */
	/*	Endpoint Address (OUT) */
	0x03,	/* bmAttributes: Interrupt endpoint */
	HID_OUT_PACKET,	/* wMaxPacketSize: 2 Bytes max  */
	0x00,
	0x00,	/* bInterval: Polling Interval (20 ms) */
	/* 41 */
} ;
const uint8_t CustomHID_ReportDescriptor1[CUSTOMHID_SIZ_REPORT_DESC] =
{
	0x05,0x01,                      /* Usage Page (Generic Desktop) */ 
	0x09,0x02,                      /* Usage (Mouse) */ 
	0xA1,0x01,                      /* Collection (Application) */

	0x09,0x01,                      /* Usage (Pointer) */
	0xA1,0x00,                      /* Collection (Physical) */
	0x85, 0x01,
	0x95, 0x40,
	0x75, 0x08,
	0x09, 0x3a,
	0x15, 0x00,
	0x26, 0xff, 0x00,
	0x81, 0x02,
	0xC0,                           /* End Collection */

	0xC0                            /* End Collection */
};
const uint8_t CustomHID_ReportDescriptor[CUSTOMHID_SIZ_REPORT_DESC] =
{
	0x06, 0xFF, 0xa0,      /* USAGE_PAGE (Vendor Page: 0xFF00) */
	0x09, 0x01,            /* USAGE (Demo Kit)               */
	0xa1, 0x01,            /* COLLECTION (Application)       */

	0x85, 0xfd,            /*     REPORT_ID (1)		     */
	0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */
	0x26, 0xff, 0x00,      /*     LOGICAL_MAXIMUM (255)      */
	0x75, 0x08,            /*     REPORT_SIZE (8)            */
	0x95, 0x3f,            /*     REPORT_COUNT (63)           */
	0x09, 0x01,            /*     USAGE (LED 1)	             */
	0x81, 0x02,            /*     ??INPUT (Data,Var,Abs,Vol)   */

	0x85, 0xfd,            /*     REPORT_ID 2		     */
	0x15, 0x00,            /*     LOGICAL_MINIMUM (0)        */
	0x26, 0xff, 0x00,      /*     LOGICAL_MAXIMUM (255)      */
	0x95, 0x3f,            /*     REPORT_COUNT (63)           */
	0x75, 0x08,            /*     REPORT_SIZE (8)            */
	0x09, 0x01,            /*     USAGE (LED 1)	             */
	0x91, 0x02,            /*     OUTPUT (Data,Var,Abs,Vol)  */

	0xc0 	          /*     END_COLLECTION	             */
};

uint8_t  USBD_HID_Init (void  *pdev, uint8_t cfgidx)
{
	DCD_PMA_Config(pdev , HID_IN_EP,USB_SNG_BUF,HID_IN_TX_ADDRESS);
	DCD_PMA_Config(pdev , HID_OUT_EP,USB_SNG_BUF,HID_OUT_RX_ADDRESS);

	/* Open EP IN */
	DCD_EP_Open(pdev, HID_IN_EP, HID_IN_PACKET, USB_EP_INT);

	/* Open EP OUT */
	DCD_EP_Open(pdev, HID_OUT_EP, HID_OUT_PACKET, USB_EP_INT);

	/*Receive Data*/
	DCD_EP_PrepareRx(pdev,HID_OUT_EP,Report_buf,64);

	return USBD_OK;
}

uint8_t  USBD_HID_DeInit (void  *pdev, uint8_t cfgidx)
{
	/* Close HID EPs */
	DCD_EP_Close (pdev , HID_IN_EP);
	DCD_EP_Close (pdev , HID_OUT_EP);

	return USBD_OK;
}

uint8_t  USBD_HID_SOF (void  *pdev)
{
	sof_timer++;
	sof_int();
}
/**
 * @brief  USBD_HID_Setup
 *         Handle the HID specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
uint8_t  USBD_HID_Setup (void  *pdev, 
		USB_SETUP_REQ *req)
{
	uint8_t USBD_HID_Report_LENGTH=0;
	uint16_t len = 0;
	uint8_t  *pbuf = NULL;


	switch (req->bmRequest & USB_REQ_TYPE_MASK)
	{
		case USB_REQ_TYPE_CLASS :  
			switch (req->bRequest)
			{
				case HID_REQ_SET_PROTOCOL:
					USBD_HID_Protocol =
						(uint8_t)(req->wValue);
					break;

				case HID_REQ_GET_PROTOCOL:
					USBD_CtlSendData (pdev, 
						(uint8_t *)&USBD_HID_Protocol,
						1);    
					break;

				case HID_REQ_SET_IDLE:
					USBD_HID_IdleState =
						(uint8_t)(req->wValue >> 8);
					break;

				case HID_REQ_GET_IDLE:
					USBD_CtlSendData (pdev, 
						(uint8_t *)&USBD_HID_IdleState,
						1);        
					break;

				case HID_REQ_SET_REPORT:
					flag = 1;
					USBD_HID_Report_ID =
						(uint8_t)(req->wValue);
					USBD_HID_Report_LENGTH =
						(uint8_t)(req->wLength);
					USBD_CtlPrepareRx (pdev, Report_buf,
							USBD_HID_Report_LENGTH);

					break;

				default:
					USBD_CtlError (pdev, req);
					return USBD_FAIL; 
			}
			break;

		case USB_REQ_TYPE_STANDARD:
			switch (req->bRequest)
			{
				case USB_REQ_GET_DESCRIPTOR: 
					if( req->wValue >> 8 == HID_REPORT_DESC)
					{
						len =
						MIN(CUSTOMHID_SIZ_REPORT_DESC ,
								req->wLength);
						pbuf =
					(uint8_t*)CustomHID_ReportDescriptor;
					}
					else if( req->wValue >> 8 ==
							HID_DESCRIPTOR_TYPE)
					{
						pbuf =
					(uint8_t*)USBD_HID_CfgDesc + 0x12;
						len =
					MIN(USB_HID_DESC_SIZ , req->wLength);
					}

					USBD_CtlSendData (pdev, 
							pbuf,
							len);

					break;

				case USB_REQ_GET_INTERFACE :
					USBD_CtlSendData (pdev,
						(uint8_t *)&USBD_HID_AltSet,
						1);
					break;

				case USB_REQ_SET_INTERFACE :
					USBD_HID_AltSet =
						(uint8_t)(req->wValue);
					break;
			}
	}
	return USBD_OK;
}

/**
 * @brief  USBD_HID_SendReport 
 *         Send HID Report
 * @param  pdev: device instance
 * @param  buff: pointer to report
 * @retval status
 */
uint8_t USBD_HID_SendReport(USB_CORE_HANDLE  *pdev, uint8_t *report,
		uint16_t len)
{
	/* Check if USB is configured */
	if (pdev->dev.device_status == USB_CONFIGURED )
	{
		DCD_EP_Tx (pdev, HID_IN_EP, report, len);
	}
	return USBD_OK;
}

/**
 * @brief  USBD_HID_GetCfgDesc 
 *         return configuration descriptor
 * @param  speed : current device speed
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
uint8_t  *USBD_HID_GetCfgDesc (uint8_t speed, uint16_t *length)
{
	*length = sizeof (USBD_HID_CfgDesc);
	return (uint8_t*)USBD_HID_CfgDesc;
}
#if 0
void uart_recover(void)
{

	if (USART_GetFlagStatus(USART2, USART_FLAG_FE) != RESET) 
	{
		USART_ReceiveData(USART2); 
		USART_ClearFlag(USART2, USART_FLAG_FE);
	}

	if(USART_GetFlagStatus(USART2, USART_FLAG_PE) != RESET)         
	{        
		USART_ReceiveData(USART2);  
		USART_ClearFlag(USART2, USART_FLAG_PE);  			  
	}																						 
	if(USART_GetFlagStatus(USART2,USART_FLAG_ORE) != RESET)
	{
		USART_ReceiveData(USART2);
		USART_ClearFlag(USART2,USART_FLAG_ORE);
	}
}
#endif
/**
 * @brief  USBD_HID_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
uint8_t  USBD_HID_DataIn (void  *pdev, 
		uint8_t epnum)
{
	if (epnum == 1) {
		hid_sent = RT_TRUE;
		//notify_uart2hid();
	}
	return USBD_OK;
}

/**
 * @brief  USBD_HID_DataOut
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
uint8_t  USBD_HID_DataOut (void  *pdev, 
		uint8_t epnum)
{
	if (epnum == 0x01) {
		//if (insert_mem(TYPE_HID2UART, Report_buf))
		//	notify_hid2uart();
	
		DCD_EP_PrepareRx(pdev,HID_IN_EP,Report_buf, 64);
	}
	return USBD_OK;
}

/**
 * @brief  USBD_HID_EP0_RxReady
 *         Handles control request data.
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */

uint8_t USBD_HID_EP0_RxReady(void *pdev)
{
	return USBD_OK;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
