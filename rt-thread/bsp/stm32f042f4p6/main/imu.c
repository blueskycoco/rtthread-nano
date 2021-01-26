#include <stm32f0xx.h>
#include <rtthread.h>

static uint8_t recv_data, send_data;
struct rt_semaphore spi_sem;
SPI_InitTypeDef  SPI_InitStructure;
void init_spi()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);//sck
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);//miso
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);//mosi

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;

	GPIO_SetBits(GPIOA,GPIO_Pin_5);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA,GPIO_Pin_7);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
	
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	//SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_ERR, ENABLE);

	SPI_SSOutputCmd(SPI1,ENABLE);
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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);
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

void spi_rw(uint8_t *data, uint16_t len, uint8_t *rsp, uint16_t rsp_len)
{
	uint16_t i,j;
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	for (i=0; i<len; i++) {
		j=0;
		SPI_SendData8(SPI1, data[i]);		
		while (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) != SET && j < 50) j++;
		
		if (rsp != RT_NULL && rsp_len != 0) {
			j=0;
			while (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) != SET && j < 50) j++;
			rsp[i] = SPI_ReceiveData8(SPI1);
		}
	}
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void SPI1_IRQHandler(void)
{

  /* SPI in Master Tramitter mode--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET)
  {

      SPI_SendData8(SPI1, send_data);
      SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);

  }

  /* SPI in Master Receiver mode--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
  {
    recv_data=SPI_ReceiveData8(SPI1);
  }

  /* SPI Error interrupt--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_OVR) == SET)
  {
    SPI_ReceiveData8(SPI1);
    SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_OVR);
  }
}

void write_spi(uint8_t data)
{
	send_data=data;
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);
	while (SPI_GetTransmissionFIFOStatus(SPI1) != SPI_TransmissionFIFOStatus_Empty);
}

uint8_t read_spi()
{
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
	while (SPI_GetReceptionFIFOStatus(SPI1) != SPI_ReceptionFIFOStatus_Empty);
	return recv_data;
}

void inv_serif_write_1B(uint8_t reg_addr, uint8_t reg_val)
{
	uint8_t tmp_reg_addr[2];
	tmp_reg_addr[0] = reg_addr & 0x7F;
	tmp_reg_addr[1] = reg_val;
	spi_rw(tmp_reg_addr, 2, RT_NULL, 0);
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	write_spi(tmp_reg_addr[0]);
	write_spi(tmp_reg_addr[1]);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

uint8_t inv_serif_read_1B(uint8_t reg_addr)
{
	uint8_t tmp_reg_addr[2]={0}, tmp_reg_val[2] = {0};
	tmp_reg_addr[0] = reg_addr & 0x7F;
	tmp_reg_addr[0] |= 0x80;
	//spi_rw(tmp_reg_addr, 2, tmp_reg_val, 2);
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	write_spi(tmp_reg_addr[0]);
	read_spi();
	write_spi(0xff);
	tmp_reg_val[0] = read_spi();
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	if (reg_addr != 0x2d)
	rt_kprintf("read %x %x\r\n", reg_addr, tmp_reg_val[0]);
	return tmp_reg_val[0];
}
void inv_serif_read_burst(uint8_t reg_addr, uint8_t *buf, uint16_t len)
{
	int i;
	uint8_t tmp_reg_addr[2]={0}, tmp_reg_val[2] = {0};
	tmp_reg_addr[0] = reg_addr & 0x7F;
	tmp_reg_addr[0] |= 0x80;
	//spi_rw(tmp_reg_addr, 2, tmp_reg_val, 2);
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
	write_spi(tmp_reg_addr[0]);
	read_spi();
	for (i=0; i<len; i++)
	{
		write_spi(0xff);
		buf[i] = read_spi();
		//rt_kprintf("%x ", buf[i]);
	}
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	//rt_kprintf("\r\n");
	//rt_kprintf("read %x %x\r\n", reg_addr, tmp_reg_val[0]);
}
void ICM4x6xx_init(uint8_t *chip_id)
{
#if 0
	*chip_id = inv_serif_read_1B(0x75);
	inv_serif_write_1B(0x6b, 0x00);	
	rt_thread_mdelay(100);
	inv_serif_write_1B(0x6c, 0x00);
	uint8_t gyro = inv_serif_read_1B(0x1b);
	gyro = gyro & 0xe7;
	gyro = gyro | (3 << 3);
	inv_serif_write_1B(0x1b, gyro);
	uint8_t accel = inv_serif_read_1B(0x1c);
	accel = accel & 0xe7;
	accel = accel | (3 << 3);
	inv_serif_write_1B(0x1c, accel);
	inv_serif_write_1B(0x1d, 0x06);//0x06
	inv_serif_write_1B(0x19, 0);
	inv_serif_write_1B(0x1a, 0x06);
	inv_serif_write_1B(0x38, 0x01);
#else
	uint8_t data;

	inv_serif_write_1B(0x76, 0x00);
	inv_serif_write_1B(0x11, 0x01);
	rt_thread_mdelay(50);
	*chip_id = inv_serif_read_1B(0x75);
	/* gyro */
	data = inv_serif_read_1B(0x4e);
	data = data | 0x0f;
	inv_serif_write_1B(0x4e, data);
	data = inv_serif_read_1B(0x4e);
#if 0
	/* acc odr */
	data = inv_serif_read_1B(0x50);
	data = (data & 0x18) | 0x06;
	inv_serif_write_1B(0x50, data);
	data = inv_serif_read_1B(0x50);
	/* gyro odr */
	data = inv_serif_read_1B(0x4f);
	data = (data & 0x18) | 0x06;
	inv_serif_write_1B(0x4f, data);
	data = inv_serif_read_1B(0x4f);
#endif
#endif
}
#define ACC 0
void init_imu()
{
	uint8_t chip;
    	rt_sem_init(&spi_sem, "spi_sem", 0, 0);
#if 0
#if ACC 
    	imu_jump_detection_init(1.0f);
#else
    	imu_gyro_jump_detection_init(0.051f);
#endif
#endif
	init_spi();
	//init_irq();
	ICM4x6xx_init(&chip);
	rt_kprintf("imu id %x\r\n", chip);
}
void dump_imu(uint8_t *imu, uint32_t ts)
{
#if 0
	uint8_t accel_reg[15] = {0x00};
	uint8_t accel[15];
	rt_int16_t data[3];
#if ACC 
	accel_reg[0] = 0xbb;//0xbb;
#else
	accel_reg[0] = 0xc3;//0xbb;
#endif
	spi_rw(accel_reg, 3, accel, 3);
	data[0] = (accel[1] << 8) + accel[2];
#if ACC
	accel_reg[0] = 0xbd;
#else
	accel_reg[0] = 0xc5;//0xbb;
#endif
	spi_rw(accel_reg, 3, accel, 3);
	data[1] = (accel[1] << 8) + accel[2];
#if ACC
	accel_reg[0] = 0xbf;
#else
	accel_reg[0] = 0xc7;//0xbb;
#endif
	spi_rw(accel_reg, 3, accel, 3);
	data[2] = (accel[1] << 8) + accel[2];
#if 0
	rt_kprintf("gyro/accel %d %d %d\r\n",
			data[0], data[1], data[2]);
#endif
#if ACC
	if (imu_jump_detect_int(data)) {		
#else
	if (imu_gyro_jump_detect_int(data)) {		
#endif
//		rt_kprintf("find  jump\r\n");

		return 1;
	}
#else
	uint8_t reg[3] = {0x00};
	uint8_t data[3] = {0x00};
	do {
		data[0] = inv_serif_read_1B(0x2d);
	} while (!(data[0] & 0x08));
	//rt_kprintf("data[0] %x\r\n", data[0]);
	imu[0] = 0xfd;
#if 0
	/* acc */
	imu[1] = inv_serif_read_1B(0x1f);
	imu[2] = inv_serif_read_1B(0x20);
	imu[3] = inv_serif_read_1B(0x21);
	imu[4] = inv_serif_read_1B(0x22);
	imu[5] = inv_serif_read_1B(0x23);
	imu[6] = inv_serif_read_1B(0x24);
	/* gyro */
	imu[7] = inv_serif_read_1B(0x25);
	imu[8] = inv_serif_read_1B(0x26);
	imu[9] = inv_serif_read_1B(0x27);
	imu[10] = inv_serif_read_1B(0x28);
	imu[11] = inv_serif_read_1B(0x29);
	imu[12] = inv_serif_read_1B(0x2a);
#else
	inv_serif_read_burst(0x1d, imu+1, 14);
#endif
	/* ts */
	imu[15] = (ts >> 24) & 0xff;
	imu[16] = (ts >> 16) & 0xff;
	imu[17] = (ts >> 8) & 0xff;
	imu[18] = (ts >> 0) & 0xff;
#endif
	return 0;
}
