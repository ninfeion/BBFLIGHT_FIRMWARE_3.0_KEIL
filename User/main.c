#include "stm32f10x.h"
#include "systeminit.h"
#include "system_config.h"
#include "string.h"
#include "spinrf.h"
#include "delay.h"
#include "pwm.h"
#include "mpu9250.h"
#include "adc.h"
#include "ms5611.h"
#include "system_loop.h"
#include "imucal.h"
#include "control.h"
#include "filter.h"
#include "usb2com.h"

#define MAGREADDELAY  7
#define PRESREADDELAY 10

enum reloadStatus
{
	ready,
	reloading,
	reloadfinish
}magStatus = ready,presStatus = ready;

#define TX_MODE            0
#define RX_MODE            1
#define TRANS_TO_TX        2
#define TRANS_TO_RX        3
#define RT_TIMES           15.0
#define TIMEOUT            1429
#define LOSTCONTROLTRIGGER 5

uint8_t txFailCount = 0;
uint8_t rxFailCount = 0;
uint8_t NrfState = TRANS_TO_TX;
uint8_t NRF_RXBUFFER[RX_PLOAD_WIDTH];
uint8_t NRF_TXBUFFER[TX_PLOAD_WIDTH];
uint8_t RX_ADDRESS[RX_ADR_WIDTH]= {0x35,0xc3,0x10,0x10,0x11};	//Slave Address	
uint8_t TX_ADDRESS[TX_ADR_WIDTH]= {0x35,0xc3,0x10,0x10,0x00};	//Master Address

//////////////////////////
uint8_t lostControlFlag = 0;
uint8_t timeOutMaxFlag = 0;
//////////////////////////
int16_t ACCELDATA[3];
int16_t GYRODATA[3];
int16_t MAGDATA[3];

static uint16_t MS5611_PROM[7];
static float MS5611_TEMP_NoOffset[2],MS5611_TaPAfterOffset[2],PRESSUREDATA;

/////////////////////////
void nrfTransmitResult(uint8_t res, uint8_t linkQuality);
void nrfReceiveResult(uint8_t res);


int main(void)
{
	uint32_t i;
	uint32_t whileStart, timeTemp;
	
    sysclockInit();
	cycleCounterInit();
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
	
	#ifdef USB_DEBUG
		usbInit();
	#endif
	
	memset (&BBImu, 0, sizeof(ImuData));
	
	accelAndGyroOffset(&BBImu);
	
	#ifdef USE_LPF_FILTER
		LPF2pSetCutoffFreq_1(SAMPLINGFREQ, LPFCUTOFFFREQ);	
		LPF2pSetCutoffFreq_2(SAMPLINGFREQ, LPFCUTOFFFREQ);
		LPF2pSetCutoffFreq_3(SAMPLINGFREQ, LPFCUTOFFFREQ);
		LPF2pSetCutoffFreq_4(SAMPLINGFREQ, LPFCUTOFFFREQ);
		LPF2pSetCutoffFreq_5(SAMPLINGFREQ, LPFCUTOFFFREQ);
		LPF2pSetCutoffFreq_6(SAMPLINGFREQ, LPFCUTOFFFREQ);
	#endif
	
	BBImu.pidPitch.pidP    = 0.0;   // 7.1;   //  5.0
	BBImu.pidPitch.pidD    = 0.0;   // 7.8;   // 11.0; // 166.0
	BBImu.pidPitch.pidI    = 0.0;
	// BBImu.pidPitch.pidIout = 0.0;
	
	BBImu.pidRoll.pidP    = 7.2;   // 4.5
	BBImu.pidRoll.pidD    = 5.0;   // 11.0; // 167.0
	BBImu.pidRoll.pidI    = 0.0;
	// BBImu.pidRoll.pidIout = 0.0;
	
	BBImu.pidYaw.pidP    = 0.0;
	BBImu.pidYaw.pidD    = 0.0;    // 12.0;  // 139.0
	BBImu.pidYaw.pidI    = 0.0;
	// BBImu.pidYaw.pidPout = 0.0;
	// BBImu.pidYaw.pidIout = 0.0;
	
	// BBImu.actualYaw.newData = 0;
	
	whileStart = currentTime();
	while(1)
	{	
		LedA_off;
		if ( radioFlag == 1)
		{
			timeTemp = currentTime();
			LedA_on;
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
                                  NrfState = RX_MODE;
								  SPI_CE_H();
								  rxTimeOutFlag = 0;
								  rxTimeOutCount = 0;
								  break;

				case RX_MODE: if(rxTimeOutFlag ==1)
							  {
								  NrfState = TRANS_TO_TX;
								  rxTimeOutFlag = 0;
								  rxTimeOutCount = 0;
								  nrfReceiveResult(0);
							  }							
			}

			radioFlag = 0;
			BBSYSTEM.nrfExecPrd = currentTime() - timeTemp;
			BBSYSTEM.idlePrd = currentTime() - BBSYSTEM.nrfExecPrd - whileStart;
			continue;
		}				
		LedD_off;
		if(attitudeUpdateFlag == 1)
		{
			timeTemp = currentTime();
			LedD_on;
			if(BBSYSTEM.MPU9250ONLINE == 1)
			{
				READ_MPU9250_ACCEL_RAW(ACCELDATA);
				READ_MPU9250_GYRO_RAW(GYRODATA);
				#ifdef USE_MAG_PASSMODE                                                                  // mpu9250.h
					READ_MPU9250_Bypass_MAG_RAW(MAGDATA);
				#endif				
				
				for(i=0; i<3; i++)
				{
					BBImu.accelRaw[i]         = ACCELDATA[i] - BBImu.accelOffset[i];       // /ACCELLSB; // Binary
					BBImu.gyroRaw.lastData[i] = BBImu.gyroRaw.newData[i];
					BBImu.gyroRaw.newData[i]  = (GYRODATA[i] - BBImu.gyroOffset[i]) /GYROLSB;            // Degree
					#ifdef USE_MAG_PASSMODE                                                              // mpu9250.h
						BBImu.magRaw[i]       = MAGDATA[i] *MAGLSB;
					#endif
				}
				
				#ifdef USE_LPF_FILTER
					BBImu.accelRaw[0] = LPF2pApply_1(BBImu.accelRaw[0]);
					BBImu.accelRaw[1] = LPF2pApply_2(BBImu.accelRaw[1]);
					BBImu.accelRaw[2] = LPF2pApply_3(BBImu.accelRaw[2]);
					
					BBImu.gyroRaw.newData[0] = LPF2pApply_4(BBImu.gyroRaw.newData[0]);
					BBImu.gyroRaw.newData[1] = LPF2pApply_5(BBImu.gyroRaw.newData[1]);
					BBImu.gyroRaw.newData[2] = LPF2pApply_6(BBImu.gyroRaw.newData[2]);
				#endif				
				
				// imuUpdate(&BBImu);
				IMUSO3Thread(&BBImu);
			}
		
			#ifdef BROENABLED
				MS5611_PROM_READ(MS5611_PROM);
				MS5611_GetTempture(CMD_MS5611_D2_OSR_4096, MS5611_PROM, MS5611_TEMP_NoOffset);
				MS5611_GetPressure(CMD_MS5611_D1_OSR_4096, MS5611_PROM, MS5611_TEMP_NoOffset, MS5611_TaPAfterOffset);
				PRESSUREDATA = MS5611_TaPAfterOffset[1] * 0.01f;
			#endif

			attitudeUpdateFlag = 0;
			BBSYSTEM.imuExecPrd = currentTime() - timeTemp;
			BBSYSTEM.idlePrd = currentTime() - BBSYSTEM.imuExecPrd - whileStart;
			continue;
		}		
		if(motorUpdateFlag == 1)
		{
			pidControl(&BBImu);
			motorUpdate(&BBImu, &BBMess);	
			
			motorUpdateFlag = 0;
			continue;
		}
		if(batteryCheckFlag == 1)
		{
			BBMess.battery = adcBatteryConversion();
			
			batteryCheckFlag = 0;
			continue;
		}
	}		
}


void nrfTransmitResult(uint8_t res, uint8_t linkQuality)
{
	if(res == 0)
	{
		txFailCount ++;
		if(txFailCount > LOSTCONTROLTRIGGER)
		{
			lostControlFlag = 1;
		}
		BBMess.linkQuality = 0;
	}
	else
	{
		txFailCount = 0;
		BBMess.linkQuality = linkQuality;
	}		
}


void nrfReceiveResult(uint8_t res)
{
	if(res == 1)
	{
		rxFailCount = 0;
		if((NRF_RXBUFFER[0] == 0xaa) && (NRF_RXBUFFER[31] == 0xff))
		{
			memcpy(&BBCom, NRF_RXBUFFER, RX_PLOAD_WIDTH);
			BBImu.targetThrust = BBCom.thrust;
			BBImu.targetPitch = BBCom.pitch;
			BBImu.targetRoll = BBCom.roll;
			BBImu.targetYaw = BBCom.yaw;
			switch(BBCom.pidType)
			{
				case 1: BBImu.pidPitch.pidP = (float)BBCom.pidValue;
						break;
				case 2: BBImu.pidPitch.pidI = (float)BBCom.pidValue;
						break;
				case 3: BBImu.pidPitch.pidD = (float)BBCom.pidValue;
						break;
				case 4: BBImu.pidRoll.pidP = (float)BBCom.pidValue;
						break;
				case 5: BBImu.pidRoll.pidI = (float)BBCom.pidValue;
						break;
				case 6: BBImu.pidRoll.pidD = (float)BBCom.pidValue;
						break;
				case 7: BBImu.pidYaw.pidP = (float)BBCom.pidValue;
						break;
				case 8: BBImu.pidYaw.pidI = (float)BBCom.pidValue;
						break;
				case 9: BBImu.pidYaw.pidD = (float)BBCom.pidValue;
						break;
			}
			
			// BBMess.thrust = BBImu.actualThrust;
			BBMess.pitch = BBImu.actualPitch.newData;
			BBMess.roll = BBImu.actualRoll.newData;
			BBMess.yaw = BBImu.actualYaw.newData;
			memcpy(NRF_TXBUFFER, &BBMess, TX_PLOAD_WIDTH);

		}		
		//else
		//{
		//	for(i=0; i<TX_PLOAD_WIDTH; i++)
		//	{
		//		NRF_TXBUFFER[i] = 0;  
		//	}	
		//}
		//should send a affective frame data to trigge the pc clients transmit
	}
	else
	{
		rxFailCount ++;
		if(rxFailCount > LOSTCONTROLTRIGGER)
		{
			timeOutMaxFlag = 1;
		}
	}
}


void EXTI9_5_IRQHandler(void)
{
	uint8_t state;
	uint8_t linkQuality;

	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
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



