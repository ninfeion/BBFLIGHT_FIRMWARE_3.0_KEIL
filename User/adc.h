#ifndef _ADC_H_
#define _ADC_H_
#include "stm32f10x.h"

typedef struct 
{
	uint16_t  BatteryAD;             //电压AD值
	uint16_t  BatteryVal;            //电压实际值
	uint16_t  BatReal;               //电池的实际电压，用万用表测
	uint16_t  ADRef;                 //AD参考源电压，这里是单片机供电电压，一般在3.3V左右，要实测
	uint16_t  ADinput;               //AD采样输入电压--->R15和R17相连的焊盘电压
	uint16_t  Bat_K;                 //计算电压值系数，用于电压校准
	uint16_t  BatteryADmin;          //电压门限
	char   alarm;									//报警位
	char   chargeSta;							//充电状态
}BATTERY_TypeDef;

typedef struct
{
	uint16_t CurrentAD;
	float gain;                   //运放增益
	uint16_t ADRef;                  //参考电压 mv
	uint16_t Rom;										//采样电阻大小
	uint16_t CURRENTVal;
	
}CURRENT_TypeDef;

extern BATTERY_TypeDef BATTERY_0;
extern CURRENT_TypeDef CURRENT_0;

void adcInit(void);
uint16_t adcBatteryConversion(void);
uint16_t adcCurrentConversion(void);
uint16_t adcGetChannelValue(uint8_t channel);

#endif
