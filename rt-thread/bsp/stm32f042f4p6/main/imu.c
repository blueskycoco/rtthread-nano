#include <stm32f0xx.h>
#include <rtthread.h>

SPI_InitTypeDef  SPI_InitStructure;
    struct rt_semaphore imu_sem;
void init_spi()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable the SPI periph */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	/* Enable SCK, MOSI, MISO and NSS GPIO clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);//sck
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);//miso
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);//mosi
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_0);//cs

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;

	/* SPI SCK pin configuration */
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_SetBits(GPIOA,GPIO_Pin_7);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	//GPIO_Init(GPIOF, &GPIO_InitStructure);

	/* SPI NSS pin configuration */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	/* Configure the SPI interrupt priority 
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);*/

	/* Initializes the SPI communication */
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI1, &SPI_InitStructure);

	/* Initialize the FIFO threshold */
	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);

	/* Enable the Rx buffer not empty interrupt */
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
	/* Enable the SPI Error interrupt */
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_ERR, ENABLE);
	/* Data transfer is performed in the SPI interrupt routine */
	SPI_SSOutputCmd(SPI1,ENABLE);
	//SPI_NSSPulseModeCmd(SPI1,ENABLE);
	/* Enable the SPI peripheral */
	SPI_Cmd(SPI1, ENABLE);
}
void init_irq()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;


	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	//GPIO_Init(GPIOF, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);
	//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line1);
}
void imu_isr()
{
    rt_sem_release(&imu_sem);
}
void spi_rw(uint8_t *data, uint16_t len, uint8_t *rsp, uint16_t rsp_len)
{
	uint16_t i,j;
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	for (i=0; i<len; i++) {
		j=0;
		SPI_SendData8(SPI1, data[i]);		
		while (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) != SET && j < 50) j++;
		//if (j == 50)
		//	rt_kprintf("sending timeout\r\n");
		if (rsp != RT_NULL && rsp_len != 0) {
			j=0;
			while (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) != SET && j < 50) j++;
			rsp[i] = SPI_ReceiveData8(SPI1);
			//if (j == 50)
			//	rt_kprintf("recving timeout\r\n");
		}
	}
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}
void inv_serif_write_1B(uint8_t reg_addr, uint8_t reg_val)
{
	uint8_t tmp_reg_addr[2];
	tmp_reg_addr[0] = reg_addr & 0x7F;
	tmp_reg_addr[1] = reg_val;
	spi_rw(tmp_reg_addr, 2, RT_NULL, 0);
}

uint8_t inv_serif_read_1B(uint8_t reg_addr)
{
	uint8_t tmp_reg_addr[2]={0}, tmp_reg_val[2] = {0};
	tmp_reg_addr[0] = reg_addr & 0x7F;
	tmp_reg_addr[0] |= 0x80;
	spi_rw(tmp_reg_addr, 2, tmp_reg_val, 2);
	return tmp_reg_val[1];
}
void ICM4x6xx_init(uint8_t *chip_id)
{
	*chip_id = inv_serif_read_1B(0x75);
	//inv_serif_write_1B(0x6b, 0x00);	
	//rt_thread_mdelay(100);
	inv_serif_write_1B(0x6c, 0x00);
	uint8_t gyro = inv_serif_read_1B(0x1b);
	gyro = gyro & 0xe7;
	gyro = gyro | (3 << 3);
	inv_serif_write_1B(0x1b, gyro);
	uint8_t accel = inv_serif_read_1B(0x1c);
	accel = accel & 0xe7;
	accel = accel | (3 << 3);
	inv_serif_write_1B(0x1c, accel);
	inv_serif_write_1B(0x1d, 0x07);//0x06
	inv_serif_write_1B(0x19, 0);
	inv_serif_write_1B(0x1a, 0x05);
	inv_serif_write_1B(0x38, 0x01);
	
}
void init_imu()
{
	uint8_t chip;
    	rt_sem_init(&imu_sem, "shrx", 0, 0);
    	imu_jump_detection_init(1.0f);
	init_spi();
	init_irq();
	ICM4x6xx_init(&chip);
	rt_kprintf("imu id %x\r\n", chip);
}
uint8_t get_imu_data()
{
	uint8_t accel_reg[15] = {0x00};
	uint8_t accel[15];
	rt_int16_t data[3];
	accel_reg[0] = 0xbb;
	rt_sem_take(&imu_sem, RT_WAITING_FOREVER);
	spi_rw(accel_reg, 15, accel, 15);
	data[0] = (accel[1] << 8) + accel[2];
	data[1] = (accel[3] << 8) + accel[4];
	data[2] = (accel[5] << 8) + accel[6];
	if (imu_jump_detect_int(data)) {		
		//rt_kprintf("find jump\r\n");
		return 1;
	}
#if 0
	rt_kprintf("accel %06d %06d %06d\r\n",
			(accel[1] << 8) + accel[2], 
			(accel[3] << 8) + accel[4],
			(accel[5] << 8) + accel[6]);
	rt_kprintf("gyro  %06d %06d %06d\r\n",
			(accel[9] << 8) + accel[10], 
			(accel[11] << 8) + accel[12],
			(accel[13] << 8) + accel[14]);
	rt_kprintf("temp  %d\r\n",
			(accel[7] << 8) + accel[8]); 
#endif
	return 0;
}
