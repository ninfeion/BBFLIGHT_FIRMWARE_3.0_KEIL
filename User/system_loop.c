#include "system_loop.h"
#include "spinrf.h"
#include "delay.h"
#include "system_config.h"
#define MAGREADDELAY 7
#define PRESREADDELAY 10

uint8_t radioFlag, attitudeUpdateFlag;
volatile uint16_t radioPeriodCount,attitudeUpdatePeriodCount;
uint8_t USB_DEBUG_flag; 
volatile uint16_t USB_DEBUG_flag_count;

enum reloadStatus
{
	ready,
	reloading,
	reloadfinish
}magStatus = ready,presStatus = ready;
			
void TIM1_UP_IRQHandler(void)
{
	if(radioPeriodCount ++ == 1)
	{
		radioFlag = 1;
		radioPeriodCount = 0;
	}
	if(attitudeUpdatePeriodCount ++ == 10)
	{
		attitudeUpdateFlag = 1;
		attitudeUpdatePeriodCount = 0;
	}
	if(USB_DEBUG_flag_count ++ == 1000) //1hz
	{
		USB_DEBUG_flag=1;
		USB_DEBUG_flag_count = 0;
	}
	TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update); 
}


void systemLoopInit(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;	
	NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  //打开时钟
    
    TIM_DeInit(TIM1);

    TIM_TimeBaseStructure.TIM_Period = 7200;//定时1ms
    TIM_TimeBaseStructure.TIM_Prescaler = 10-1;//预分频 72/(10-1 +1)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
    /* NVIC_PriorityGroup 2 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//飞控主循环基准定时器，优先级高于串口打印
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
		
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
    TIM_ClearFlag(TIM1,TIM_FLAG_Update);		
    TIM_Cmd(TIM1,ENABLE);
}


//void systemLoop(void)
//{
	//if(radioFlag == 1)
	//{
		//radioFlag = 0;

		
		
		
		
		
	//}
	/*
	switch(magStatus)
	{
		case ready: magdelaycount = MAGREADDELAY * 1000 ;magStatus = reloading;break;
		case reloading: if(MAGREADDELAY == 0) magStatus = reloadfinish;break;
		case reloadfinish: magStatus = ready;break;
	}
	
	switch(presStatus)
	{
		case ready: presdelaycount = PRESREADDELAY * 1000 ;presStatus = reloading;break;
		case reloading: if(PRESREADDELAY == 0) presStatus = reloadfinish;break;
		case reloadfinish: presStatus = ready;break;
	}
*/
//}
	
	
	
