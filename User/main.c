#include "stm32f10x.h"
#include "systeminit.h"
#include "system_config.h"
#include "system_loop.h"

#include "spinrf.h"
#include "delay.h"

#include "pwm.h"
#include "mpu9250.h"
#include "adc.h"
#include "ms5611.h"

#ifdef _USBDEBUG_
#include "debug.h"
#endif 

uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01接收到的数据
uint8_t NRF24L01_TXDATA[TX_PLOAD_WIDTH];//nrf24l01需要发送的数据
uint8_t NrfState;

uint8_t lostPacket;

#define RT_TIMES 15.0

uint8_t  RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0xc3,0x10,0x10,0x11};	//本机地址	
uint8_t  TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0xc3,0x10,0x10,0x00};	//接收地址

#define TX_MODE 0
#define RX_MODE 1
#define TRANS_TO_TX 2
#define TRANS_TO_RX 3

#define TIMEOUT 1429
uint32_t timecount;

int main(void)
{
	uint8_t i;
		
    sysclockInit();
    systickInit();
    ledInit();
    spi2NrfInit();
    if(NRF24L01_Check() == 1)
	{
        BBSYSTEM.NRFONLINE = 1;
	}
	adcInit();
	i2cInit();
	mpu9250Init();
	if(MPU9250_Check() == TRUE)		
	{
		BBSYSTEM.MPU9250ONLINE = 1;
	}
	ms5611Init();
	pwmInit();	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	/*
    sysclockInit();
    systickInit();
    ledInit();
	spi2NrfInit();
	
	LedA_on;
	
	NrfState = TRANS_TO_TX;
	while(1)
		{		
			switch(NrfState)
			{
				case TRANS_TO_TX: SPI_CE_L();  
					              NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x4A);     // Disable Rx interrupt, turn to Tx mode
								  NRF_Write_Buf(NRF_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
								  NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // For auto axknowledge
								
							      NRF_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);      // Enabled pipe0 auto ack 
								  NRF_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);  // Enabled pipe0 RX address
								  NRF_Write_Reg(NRF_WRITE_REG + SETUP_AW, 0x05);   // Setup of address widths
								  NRF_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1f); // Setup of automatic retransmission: 500us, 10times
								  NRF_Write_Reg(NRF_WRITE_REG + RF_CH, 0x5e);      // Set the frequency channel 0x28
								  NRF_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x26);   // Set data rate and output power:0db, 250kbps
								
								  //i = USB_RxRead(NRF24L01_TXDATA, TX_PLOAD_WIDTH);
								  //if(i == 0)
								  //{
								  //    for(i=0; i<TX_PLOAD_WIDTH; i++)
								  //    {
								  //      NRF24L01_TXDATA[i] = 0x00;
								  //    }
								  //}
								  //else
								  //{
								  //    USB_TxWrite(NRF24L01_TXDATA, TX_PLOAD_WIDTH); // debug use, must be deleted
								  //}
								  
								  NRF_Write_Buf(WR_TX_PLOAD, NRF24L01_TXDATA, TX_PLOAD_WIDTH);
								
								  NrfState = TX_MODE;
								
								  SPI_CE_H(); // A high pulse on CE starts the transmission
								  break;
									  
				case TRANS_TO_RX: SPI_CE_L();
					              NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x3B);
								  NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH);
				
								  NRF_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);
								  NRF_Write_Reg(NRF_WRITE_REG + EN_RXADDR,0x01);
								  NRF_Write_Reg(NRF_WRITE_REG + SETUP_AW, 0x05);
								  NRF_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1f);
								  NRF_Write_Reg(NRF_WRITE_REG + RF_CH, 0x5e);
								  NRF_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x26);  
								  
								  NRF_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);
								  
								  SPI_CE_H();
								  
								  NrfState = RX_MODE;
								  break;
												  
				case TX_MODE: timecount = 0;
							  while((NrfState == TX_MODE) && (timecount < TIMEOUT*1500))
							  {
								  timecount ++;
								  
							  }
							  NrfState = TRANS_TO_RX;
							  break;
				case RX_MODE: timecount = 0;
							  while((NrfState == RX_MODE) && (timecount < TIMEOUT*1500))
							  {
								  timecount ++;
							  }
							  NrfState = TRANS_TO_TX; // Time out  
							  LedB_on;
							  delay_noInt_ms(100);
							  LedB_off;
					          break;

			
			
			
			
			
		}
	}*/
}

void nrfTransmitResult(uint8_t res, uint8_t linkQuality)
{
	uint8_t temp[5];
	if(res == 0)
	{
		LedC_on;
		delay_noInt_ms(100);
		LedC_off;
	}
	lostPacket = linkQuality;
	//sprintf(temp, "%d\n", linkQuality);
	//USB_TxWrite(temp, 5);
	//return
}

void nrfReceiveResult(uint8_t res)
{
	uint16_t sum;
	uint8_t i;
	if(res == 1)
	{
		for(i=0; i<RX_PLOAD_WIDTH; i++)
		{
			sum += NRF24L01_RXDATA[i];
		}
		if(sum != 0)
		{
			for(i=0; i<TX_PLOAD_WIDTH; i++)
			{
				NRF24L01_TXDATA[i] = NRF24L01_RXDATA[i] + 1;  // plus 1 to test
			}
		}
		NRF24L01_TXDATA[31] =lostPacket;
				
		//if(sum != 0)
		//{
		//	USB_TxWrite(NRF24L01_RXDATA, RX_PLOAD_WIDTH);
		//}
	}
	//return
}


void EXTI9_5_IRQHandler(void)//line5-9的中断
{
	uint8_t state;
	uint8_t linkQuality;
	if ( EXTI_GetITStatus(EXTI_Line8) != RESET )//判断是line8产生中断 
	{
		switch(NrfState)
		{
			case TX_MODE:   SPI_CE_L();
							
							state = NRF_Read_Reg(NRF_STATUS);  
							linkQuality = (NRF_Read_Reg(OBSERVE_TX) &0xf0 >>4) /RT_TIMES *100.0;
							NRF_Write_Reg(NRF_WRITE_REG +NRF_STATUS,state); 	
							NRF_Write_Reg(FLUSH_TX,NOP);
			
							if(state &MAX_RT)
							{
								nrfTransmitResult(0, linkQuality); 
							}
							else if(state &TX_DS)
								 {	
									 nrfTransmitResult(1, linkQuality); 
								 }
							
							NrfState = TRANS_TO_RX;	
							EXTI_ClearITPendingBit(EXTI_Line8);

							break;
			
			case RX_MODE:   SPI_CE_L();
							state = NRF_Read_Reg(NRF_STATUS); 		 
							NRF_Write_Reg(NRF_WRITE_REG + NRF_STATUS,state); //清除中断标志
							if(state &RX_DR)
							{
								NRF_Read_Buf(RD_RX_PLOAD, NRF24L01_RXDATA, RX_PLOAD_WIDTH);//读取数据
								
								NRF_Write_Reg(FLUSH_RX,NOP);//清除RX FIFO寄存器  
								nrfReceiveResult(1);
							}	
							NrfState = TRANS_TO_TX;
							EXTI_ClearITPendingBit(EXTI_Line8);

							break;
		}			
	}
}

