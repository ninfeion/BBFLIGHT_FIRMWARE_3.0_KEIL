#ifndef _PWM_H_
#define _PWM_H_
#include "stm32f10x.h"

  //MOTO_0->PB9
  //MOTO_2->PB0
  //MOTO_1->PB8
  //MOTO_3->PB1
	
//enum TimType
//{
//	TIM1T,
//	TIM2T,
//	TIM3T,
//	TIM4T
//};

typedef struct
{
	uint32_t Gpiorcc_Periph;
	//enum TimType TimT;
	uint32_t Gpiorcc_Tim_Periph;
	uint32_t Gpio_PIN;
	//uint8_t Gpio_Speed;
	GPIO_TypeDef *Gpiox;
	
	TIM_TypeDef *TIMX;
	uint32_t TimPeriod;
	uint16_t TimPrescaler;
	uint16_t TimPolarity;
	uint16_t TimPulse;
	void (*ocInit)(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
	
}MOTO_DEF;
/*
const MOTO_DEF MOTO_A =
{
	.Gpiorcc_Periph = RCC_APB2Periph_GPIOB,
	//.TimT = TIM3T,
	.Gpiorcc_Tim_Periph = RCC_APB1Periph_TIM4,
	.Gpio_PIN = GPIO_Pin_9,
	//.Gpio_Speed = GPIO_Speed_2MHz,
	.Gpiox = GPIOB,
	
	.TIMX = TIM4,
	.TimPeriod = 64000 - 1 ,        //TIM频率=TIM1CLK/(TIM1_Period+1)
	.TimPrescaler = 0 ,     //TIM时钟频率的预分频值
	.TimPolarity = TIM_OCPolarity_High ,      //TIM_OCPolarity_Low
	.TimPulse =  32000,
	
	.ocInit = TIM_OC4Init,
};*/

#define Moto_PwmMax 64000

const static MOTO_DEF MOTO_A =
{
	RCC_APB2Periph_GPIOB,
	RCC_APB1Periph_TIM4,
	GPIO_Pin_9,
	
	//GPIO_Speed_2MHz,
	GPIOB,
	
	TIM4,
	64000 -1 ,   //1125hz     //TIM频率=TIM1CLK/(TIM1_Period+1)
	0 ,     //TIM时钟频率的预分频值
	TIM_OCPolarity_High ,      //TIM_OCPolarity_Low
	0,//占空比
	
	TIM_OC4Init,
};

const static MOTO_DEF MOTO_B =
{
	RCC_APB2Periph_GPIOB,
	RCC_APB1Periph_TIM3,
	GPIO_Pin_0,
	//GPIO_Speed_2MHz,
	GPIOB,
	
	TIM3,
	64000 -1 ,
	0 ,
	TIM_OCPolarity_High ,
	0 ,
	
	TIM_OC3Init,
};

const static MOTO_DEF MOTO_C =
{
	RCC_APB2Periph_GPIOB,
	RCC_APB1Periph_TIM4,
	GPIO_Pin_8,
	//GPIO_Speed_2MHz,
	GPIOB,
	
	TIM4,
	64000 -1,
	0 ,
	TIM_OCPolarity_High ,
	0 ,
	
	TIM_OC3Init,
};

const static MOTO_DEF MOTO_D =
{
	RCC_APB2Periph_GPIOB,
	RCC_APB1Periph_TIM3,
	GPIO_Pin_1,
	//GPIO_Speed_2MHz,
	GPIOB,
	
	TIM3,
	64000 -1,
	0 ,
	TIM_OCPolarity_High ,
	0 ,
	
	TIM_OC4Init,
};

uint8_t pwmInit(void);
uint8_t MOTO_Init(const MOTO_DEF *MOTO_MAP);
void MotorPwmFlash(int32_t MOTO1_PWM,int32_t MOTO2_PWM,int32_t MOTO3_PWM,int32_t MOTO4_PWM);

#endif
