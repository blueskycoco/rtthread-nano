#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>
#include <system_stm32f0xx.h>
#include <stm32f0xx.h>

#define USART_BUF_LEN 128
uint8_t RecvBuf[USART_BUF_LEN];
uint8_t RecvBufBak[USART_BUF_LEN];
uint8_t RecvLen = 0;
struct rt_semaphore rx_sem;

void USART1_IRQHandler(void)
{
	rt_base_t level;

	level = rt_hw_interrupt_disable();

	if (USART_GetITStatus (USART1, USART_IT_RTO) != RESET) {
		USART_ClearITPendingBit (USART1, USART_IT_RTO);
		DMA_Cmd (DMA1_Channel3, DISABLE);
		RecvLen = USART_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Channel3);
		rt_memcpy(RecvBufBak, RecvBuf, RecvLen);
		rt_sem_release(&rx_sem);
		DMA_SetCurrDataCounter(DMA1_Channel3, USART_BUF_LEN);
		DMA_Cmd(DMA1_Channel3, ENABLE);
	}

	rt_hw_interrupt_enable(level);
}

static int uart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->RDR);
	DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)RecvBuf;
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize         = USART_BUF_LEN;
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3,&DMA_InitStructure);
	DMA_Cmd(DMA1_Channel3,ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = 
	USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_SetReceiverTimeOut(USART1, 40);
    USART_ReceiverTimeOutCmd(USART1, ENABLE);

    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART1, USART_IT_TC, DISABLE);
    USART_ITConfig(USART1, USART_IT_RTO, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    
    USART_Cmd(USART1, ENABLE);

    rt_sem_init(&rx_sem, "wifi_rx", 0, 0);
    return 0;
}

INIT_BOARD_EXPORT(uart_init);

uint8_t wifi_rx(uint8_t *data)
{
	rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
	rt_memcpy(data, RecvBufBak, RecvLen);
	return RecvLen;
}
uint8_t wifi_tx(uint8_t *data, uint8_t len)
{
	int i;
	for (i = 0; i < len; i++) {
		while (!(USART1->ISR & USART_FLAG_TXE));
		USART1->TDR = data[i];
	}

	return len;
}
void rt_hw_console_output(const char *str)
{   
    rt_size_t i = 0, size = 0;
    char a = '\r';

    size = rt_strlen(str);
    for (i = 0; i < size; i++)
    {
	if (*(str + i) == '\n')
	{
	    USART_SendData(USART1, a);
	    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
	}
	USART_SendData(USART1, *(uint8_t *)(str + i));
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
    }
}

char rt_hw_console_getchar(void)
{
    int8_t ch = -1;
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
	ch = USART_ReceiveData(USART1) & 0xff;
    return ch;
}
