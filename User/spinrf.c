#include "spinrf.h"
#include "systeminit.h"
#include "system_config.h"
#include "delay.h"

static uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01���յ�������
static uint8_t NRF24L01_TXDATA[RX_PLOAD_WIDTH];//nrf24l01��Ҫ���͵�����

//�޸ĸý��պͷ��͵�ַ�����Թ������������ͬһ������У����ݲ��ܸ���
static uint8_t  RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0xc3,0x10,0x10,0x11};	//������ַ	
static uint8_t  TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0xc3,0x10,0x10,0x00};	//���յ�ַ

uint8_t spi2NrfInit(void)
{
	SPI_InitTypeDef SPI_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO, ENABLE );        //spi2�˿�ʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE ); 
	
	//RCC_MCOConfig(RCC_MCO_HSE);
	/* Configure SPI2 pins: SCK, MISO and MOSI -------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//NRF_CSN--PB12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//NRF_CE--PB11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//NRF_IRQ->PA8    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;     
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //���� 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
		
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
		
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;      
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);  
	
	SPI_CSN_H();
		
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //˫��ȫ˫�� 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //��ģʽ 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //���ݴ�С8λ 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //ʱ�Ӽ��ԣ�����ʱΪ�� 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //��1��������Ч��������Ϊ����ʱ�� 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS�ź���������� 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //8��Ƶ��9MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //��λ��ǰ 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI_X, &SPI_InitStructure); 

	SPI_Cmd(SPI_X, ENABLE);
		
	return 1;
}
	
uint8_t NRF_RX_FINISH_state;
/*void EXTI9_5_IRQHandler(void)//line5-9���ж�
{
	uint8_t state;
	if ( EXTI_GetITStatus(EXTI_Line8) != RESET )//�ж���line8�����ж� 
	{
		SPI_CE_L();
		state = NRF_Read_Reg(NRFRegSTATUS); 	//��ȡ״̬�Ĵ�����ֵ    	 
		NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS,state); //����жϱ�־
		if(state & RX_DR)//���յ�����
		{
			NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);//��ȡ����
			NRF_Write_Reg(FLUSH_RX,NOP);//���RX FIFO�Ĵ���  
			NRF_RX_FINISH_state = 0;
		}	   		
		else NRF_RX_FINISH_state = 1;
		SPI_CE_H();
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
}*/

uint8_t SPI_RW(uint8_t dat) 
{ 
    while (SPI_I2S_GetFlagStatus(SPI_X, SPI_I2S_FLAG_TXE) == RESET); 
    SPI_I2S_SendData(SPI_X, dat); 
    while (SPI_I2S_GetFlagStatus(SPI_X, SPI_I2S_FLAG_RXNE) == RESET);  
    return SPI_I2S_ReceiveData(SPI_X); 
}

//д������
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
    uint8_t i;
    uint8_t status;
    SPI_CSN_L();				        /* ѡͨ���� */
    status = SPI_RW(reg);	/* д�Ĵ�����ַ */
    for(i=0; i<uchars; i++)
    {
        SPI_RW(pBuf[i]);		/* д���� */
    }
    SPI_CSN_H();						/* ��ֹ������ */
    return 	status;	
}


//��������
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
  uint8_t i;
  uint8_t status;
	SPI_CE_L();
  SPI_CSN_L();						/* ѡͨ���� */
  status = SPI_RW(reg);	/* д�Ĵ�����ַ */
  for(i=0; i<uchars; i++)
		{
     pBuf[i] = SPI_RW(0); /* ��ȡ�������� */ 	
		}
  SPI_CSN_H();						/* ��ֹ������ */
  return status;
}

uint8_t NRF24L01_Check(void) 
{ 
   u8 buf[5]={0xC2,0xC2,0xC2,0xC2,0xC2}; 
   u8 buf1[5]; 
   u8 i; 
    
   /*д��5 ���ֽڵĵ�ַ.  */ 
   NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5); 
     
   /*����д��ĵ�ַ */ 
   NRF_Read_Buf(TX_ADDR,buf1,5); 
   
    /*�Ƚ�*/ 
   for (i=0;i<5;i++) 
   { 
      if (buf1[i]!=0xC2) 
      break; 
   } 
   if (i==5)  
		return 1;
   else         
		return 0;  //MCU��NRF����������    
} 

//д�Ĵ���
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    SPI_CSN_L();					  
    status = SPI_RW(reg);  
    SPI_RW(value);		  /* д���� */
    SPI_CSN_H();					  /* ��ֹ������ */
    return 	status;
}


//���Ĵ���
uint8_t NRF_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    SPI_CSN_L();					 
    SPI_RW(reg);			  
    reg_val = SPI_RW(NOP);	  /* ��ȡ�üĴ����������� */
    SPI_CSN_H();	
 
    return 	reg_val;
}

void nrf_RXMode(void)
{	
	SPI_CE_L();
	//NRF_Write_Reg(FLUSH_RX,0xff);//���TX FIFO�Ĵ���	
	NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0 ,RX_ADDRESS ,RX_ADR_WIDTH);	//дRX�ڵ��ַ 
	
	NRF_Write_Reg(NRF_WRITE_REG + EN_AA , 	0x01);	//01ʹ��ͨ��0���Զ�Ӧ��
	NRF_Write_Reg(NRF_WRITE_REG + EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ
	NRF_Write_Reg(NRF_WRITE_REG + RF_CH,		40);
	
	NRF_Write_Reg(NRF_WRITE_REG + RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
	NRF_Write_Reg(NRF_WRITE_REG + RF_SETUP,0x0f);//����TX�������,0db����,2Mbps,���������濪��   
  NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ  IRQ���ж�
  SPI_CE_H();
	delay_us(150);
}

void nrf_TXMode(void)
{
  SPI_CE_L();
  NRF_Write_Reg(FLUSH_TX,0xff);										//���TX FIFO�Ĵ���		  
  NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);		//дTX�ڵ��ַ 
  NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); 	//����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
  NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  NRF_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
  NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  NRF_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
  SPI_CE_H();
}

void nrfSlaveMode(void)
{
	nrf_RXMode();
	
}

uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;		
	SPI_CE_H();
	while(NRF_Read_IRQ()!=0); 
	SPI_CE_L();
	state = NRF_Read_Reg(NRFRegSTATUS); 	//��ȡ״̬�Ĵ�����ֵ 	 
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS,state); //����жϱ�־
	if(state&RX_DR)//���յ�����
	{
		NRF_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF_Write_Reg(FLUSH_RX,NOP);//���RX FIFO�Ĵ��� 
		return 0; 
	}	   		
	else
	return 1;//û�յ��κ�����
}		

uint8_t NRF_TxPacket(uint8_t * tx_buf, uint8_t len)
{	
	uint8_t state;  
	SPI_CE_L();
  NRF_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);
 	SPI_CE_H();                         
	while(NRF_Read_IRQ()!=0); 	                            
	state = NRF_Read_Reg(NRFRegSTATUS);                 
	NRF_Write_Reg(NRF_WRITE_REG+NRFRegSTATUS,state); 	
	NRF_Write_Reg(FLUSH_TX,NOP); 
	if(state&MAX_RT)                    	//�ﵽ����ط�����
			return MAX_RT; 
	else if(state&TX_DS)                  //�������
					return TX_DS;
				else						  
					return ERROR;               	//����ԭ����ʧ��
}

	
