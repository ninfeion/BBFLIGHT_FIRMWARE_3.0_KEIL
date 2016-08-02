#include "stm32f10x.h"
#include "systeminit.h"
#include "system_config.h"

#include "spinrf.h"
#include "delay.h"

#include "pwm.h"
#include "mpu9250.h"
#include "adc.h"
#include "ms5611.h"
#include "system_loop.h"

#define MAGREADDELAY 7
#define PRESREADDELAY 10

enum reloadStatus
{
	ready,
	reloading,
	reloadfinish
}magStatus = ready,presStatus = ready;

typedef struct 
{
	uint8_t head[2];
	uint16_t thrust;
	float roll;
	float pitch;
	float yaw;
		
	uint8_t motor1;
	uint8_t motor2;
	uint8_t motor3;
	uint8_t motor4;
	uint16_t battery;
	uint8_t linkQuality;
	uint8_t resevered[1];
		
	float broVal;
	uint8_t haveBro;
	uint8_t connect;
	uint8_t tail[2];
}RespondMess;
	
RespondMess BBMess;

#define TX_MODE            0
#define RX_MODE            1
#define TRANS_TO_TX        2
#define TRANS_TO_RX        3
#define RT_TIMES           15.0
#define TIMEOUT            1429
#define LOSTCONTROLTRIGGER 5
uint32_t timeCount = 0;
uint8_t txFailCount = 0;
uint8_t NrfState = TRANS_TO_TX;
uint8_t NRF_RXBUFFER[RX_PLOAD_WIDTH];//nrf24l01接收到的数据
uint8_t NRF_TXBUFFER[TX_PLOAD_WIDTH];//nrf24l01需要发送的数据
uint8_t  RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0xc3,0x10,0x10,0x11};	//本机地址	
uint8_t  TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0xc3,0x10,0x10,0x00};	//接收地址

//////////////////////////
uint8_t lostControlFlag = 0;

//////////////////////////
int16_t ACCELDATA[3];
int16_t GYRODATA[3];
int16_t MAGDATA[3];
	
float ACCELDATA_F[3];
float GYRODATA_F[3];
float MAGDATA_F[3];

static uint16_t MS5611_PROM[7];
static float MS5611_TEMP_NoOffset[2],MS5611_TaPAfterOffset[2],PRESSUREDATA;

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
	
	systemLoopTim1Init();
	LedA_on;
	
	while(1)
	{
		if ( radioFlag == 1)
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
											
								  NRF_Write_Buf(WR_TX_PLOAD, NRF_TXBUFFER, TX_PLOAD_WIDTH);
											
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
															  
				//case TX_MODE: timeCount = 0;
				//			  while((NrfState == TX_MODE) && (timeCount < TIMEOUT*1500))
				//			  {
				//				  timeCount ++;	  
				//			  }
				//			  NrfState = TRANS_TO_RX;
				//			  break;
				//case RX_MODE: timeCount = 0;
				//			  LedB_on;
				//			  while((NrfState == RX_MODE) && (timeCount < TIMEOUT*1500))
				//			  {
				//				  timeCount ++;
				//			  }
				//			  NrfState = TRANS_TO_TX; // Time out  
				//			  LedB_off;
				//			  break;
			}
			radioPeriodCount = 0;
			radioFlag = 0;
		}				
		
		if(attitudeUpdateFlag == 1)
		{
			if(BBSYSTEM.MPU9250ONLINE == 1)
			{
				READ_MPU9250_ACCEL_RAW(ACCELDATA);
				READ_MPU9250_GYRO_RAW(GYRODATA);
				//READ_MPU9250_Bypass_MAG_RAW(MAGDATA);
				
				ACCELDATA_F[0]=ACCELDATA[0]/ACCELLSB;
				ACCELDATA_F[1]=ACCELDATA[1]/ACCELLSB;
				ACCELDATA_F[2]=ACCELDATA[2]/ACCELLSB;	
				GYRODATA_F[0]=GYRODATA[0]/GYROLSB;
				GYRODATA_F[1]=GYRODATA[1]/GYROLSB;
				GYRODATA_F[2]=GYRODATA[2]/GYROLSB;
				//MAGDATA_F[0]=MAGDATA[0]*MAGLSB;
				//MAGDATA_F[1]=MAGDATA[1]*MAGLSB;
				//MAGDATA_F[2]=MAGDATA[2]*MAGLSB;	
			}
			
			#ifdef BROENABLED
			MS5611_PROM_READ(MS5611_PROM);
			MS5611_GetTempture(CMD_MS5611_D2_OSR_4096, MS5611_PROM, MS5611_TEMP_NoOffset);
			MS5611_GetPressure(CMD_MS5611_D1_OSR_4096, MS5611_PROM, MS5611_TEMP_NoOffset, MS5611_TaPAfterOffset);
			PRESSUREDATA = MS5611_TaPAfterOffset[1] * 0.01f;
			#endif
			
			attitudeUpdatePeriodCount = 0;
			attitudeUpdateFlag = 0;
		}
	}
}


void nrfTransmitResult(uint8_t res, uint8_t linkQuality)
{
	if(res == 0)
	{
		txFailCount ++;
	}
	else
	{
		txFailCount = 0;
	}
	if(txFailCount > LOSTCONTROLTRIGGER)
	{
		lostControlFlag = 1;
	}
	BBMess.linkQuality = linkQuality;
}

void nrfReceiveResult(uint8_t res)
{
	uint16_t sum;
	uint8_t i;
	if(res == 1)
	{
		for(i=0; i<RX_PLOAD_WIDTH; i++)
		{
			sum += NRF_RXBUFFER[i];
		}
		if(sum != 0)
		{
			for(i=0; i<TX_PLOAD_WIDTH; i++)
			{
				NRF_TXBUFFER[i] = NRF_RXBUFFER[i] + 1;  // plus 1 to test
			}
		}
		///////RECIEVE ANALYZE
		///////TRANSMIT CREATE
	}
}


void EXTI9_5_IRQHandler(void)
{
	uint8_t state;
	uint8_t linkQuality;
	if ( EXTI_GetITStatus(EXTI_Line8) != RESET )
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
							NRF_Write_Reg(NRF_WRITE_REG + NRF_STATUS,state);
							if(state &RX_DR)
							{
								NRF_Read_Buf(RD_RX_PLOAD, NRF_RXBUFFER,RX_PLOAD_WIDTH);
								
								NRF_Write_Reg(FLUSH_RX,NOP);
								nrfReceiveResult(1);
							}	
							NrfState = TRANS_TO_TX;
							EXTI_ClearITPendingBit(EXTI_Line8);

							break;
		}			
	}
}

