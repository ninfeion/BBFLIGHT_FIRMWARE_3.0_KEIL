#ifndef _MS5611_H_
#define _MS5611_H_
#include "stm32f10x.h"

#define MS5611_ADDRESS		       0XEE //111011CX C取决于CSB的补码

#define CMD_MS5611_RESET  		   0x1E
#define CMD_MS5611_ADC_RD    	   0x00
#define	CMD_MS5611_PROM_RD 	 	   0xA0
#define CMD_MS5611_PROM_CRC  	   0xAE

//D1 -> peressure
#define	CMD_MS5611_D1_OSR_256      0x40
#define	CMD_MS5611_D1_OSR_512      0x42
#define	CMD_MS5611_D1_OSR_1024     0x44
#define	CMD_MS5611_D1_OSR_2048     0x46
#define	CMD_MS5611_D1_OSR_4096     0x48

//D2 -> tempturature
#define	CMD_MS5611_D2_OSR_256      0x50
#define	CMD_MS5611_D2_OSR_512      0x52
#define	CMD_MS5611_D2_OSR_1024     0x54
#define	CMD_MS5611_D2_OSR_2048     0x56
#define	CMD_MS5611_D2_OSR_4096     0x58

//PROM constants
//A0 16 bit reserved for manufacturer
//A2 C1 PRESSURE SENSITIVITY
//A4 C2 PRESSURE OFFSET
//A6 C3 TEMPERATURE COEFFICIENT OF PRESSURE SENSITIVITY
//A8 C4 TEMPERATURE COEFFICIENT OF PRESSURE OFFSET
//AA C5 REFERENCE REMPERATURE
//AC C6 TEMPERATURE COEFFICIENT OF THE TEMPERATURE

// OSR (Over Sampling Ratio) constants
//MS5611_OSR_256 	Conversion time 0.6ms  Resolution 0.065mbar
//MS5611_OSR_512 	Conversion time 1.2ms  Resolution 0.042mbar
//MS5611_OSR_1024 	Conversion time 2.3ms  Resolution 0.027mbar
//MS5611_OSR_2048 	Conversion time 4.6ms  Resolution 0.018mbar
//MS5611_OSR_4096 	Conversion time 9.1ms  Resolution 0.012mbar

#define ConversionTime 10000
#define DefaultPresInitFilterTime 50

//D1测气压，D2测温度

void ms5611Init(void);
void ms5611Reset(void);
void ms5611PromRead(uint16_t *PROM_C);

void ms5611StartConversion(uint8_t OSR_Cmd_SelectTempOrPres);
uint32_t ms5611GetConversion(void);
void ms5611FinalCalculation(uint32_t rawPress, uint32_t tempCache, uint16_t *PROM_C, float *outPutData);

/*
uint8_t MS5611_GetAltitude(float *AfterOffset_TempPresDATA, float *Altitude_DATA);//return 0 还不可以输出正确高度
*/

#endif


