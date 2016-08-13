#include "spinrf.h"
#include "systeminit.h"
#include "system_config.h"
#include "delay.h"

uint8_t spi2NrfInit(void)
{
	SPI_InitTypeDef SPI_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO, ENABLE );        //spi2端口时钟使能
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //输入 
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
		
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //双线全双工 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //主模式 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //数据大小8位 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //时钟极性，空闲时为低 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //第1个边沿有效，上升沿为采样时刻 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS信号由软件产生 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //8分频，9MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //高位在前 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI_X, &SPI_InitStructure); 

	SPI_Cmd(SPI_X, ENABLE);
		
	return 1;
}
	
uint8_t NRF_RX_FINISH_state;
/*void EXTI9_5_IRQHandler(void)//line5-9的中断
{
	uint8_t state;
	if ( EXTI_GetITStatus(EXTI_Line8) != RESET )//判断是line8产生中断 
	{
		SPI_CE_L();
		state = NRF_Read_Reg(NRFRegSTATUS); 	//读取状态寄存器的值    	 
		NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS,state); //清除中断标志
		if(state & RX_DR)//接收到数据
		{
			NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);//读取数据
			NRF_Write_Reg(FLUSH_RX,NOP);//清除RX FIFO寄存器  
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

//写缓冲区
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
    uint8_t i;
    uint8_t status;
    SPI_CSN_L();				        /* 选通器件 */
    status = SPI_RW(reg);	/* 写寄存器地址 */
    for(i=0; i<uchars; i++)
    {
        SPI_RW(pBuf[i]);		/* 写数据 */
    }
    SPI_CSN_H();						/* 禁止该器件 */
    return 	status;	
}


//读缓冲区
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
    uint8_t i;
    uint8_t status;
	SPI_CE_L();
    SPI_CSN_L();						/* 选通器件 */
    status = SPI_RW(reg);	/* 写寄存器地址 */
    for(i=0; i<uchars; i++)
	{
        pBuf[i] = SPI_RW(0); /* 读取返回数据 */ 	
	}
    SPI_CSN_H();						/* 禁止该器件 */
    return status;
}

uint8_t NRF24L01_Check(void) 
{ 
    uint8_t buf[5]={0xC2,0xC2,0xC2,0xC2,0xC2}; 
    uint8_t buf1[5]; 
    uint8_t i; 
    
    NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR, buf, 5); 
    NRF_Read_Buf(TX_ADDR, buf1, 5); 
    for(i=0;i<5;i++) 
    { 
        if(buf1[i]!=0xC2) 
	    {
	        break;
	    }		   
    } 
    if(i==5)  
	{
		return 1;
	}
    else
	{		
		return 0;
	}		
} 

//写寄存器
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    SPI_CSN_L();					  
    status = SPI_RW(reg);  
    SPI_RW(value);		  /* 写数据 */
    SPI_CSN_H();					  /* 禁止该器件 */
    return 	status;
}


//读寄存器
uint8_t NRF_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    SPI_CSN_L();					 
    SPI_RW(reg);			  
    reg_val = SPI_RW(NOP);	  /* 读取该寄存器返回数据 */
    SPI_CSN_H();	
 
    return 	reg_val;
}

