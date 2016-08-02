#include "adc.h"
#include "stm32f10x_dma.h"


//TCAD->PA0->ADC12_IN0
//AD_VBAT->PA1->ADC12_IN1

BATTERY_TypeDef BATTERY_0;
CURRENT_TypeDef CURRENT_0;

#define BATTERYCHANNEL ADC_Channel_1
#define CURRENTCHANNEL ADC_Channel_0
#define ADC_MEAN_SIZE 10

void adcInit(void)
{
	ADC_InitTypeDef ADC_InitStructure;//定义ADC初始化结构体变量
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE ); 
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);    
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入引脚 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	                              
  ADC_DeInit(ADC1); 
	
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC1和ADC2工作在独立模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; //使能扫描
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//ADC转换工作在连续模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//有软件控制转换
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//转换数据右对齐
  ADC_InitStructure.ADC_NbrOfChannel = 1;//转换通道数目为2
  ADC_Init(ADC1, &ADC_InitStructure); //初始化ADC
	

  ADC_Cmd(ADC1, ENABLE);//使能ADC1
  ADC_ResetCalibration(ADC1); //重置ADC1校准寄存器
	
  while(ADC_GetResetCalibrationStatus(ADC1));//等待ADC1校准重置完成
	
  ADC_StartCalibration(ADC1);//开始ADC1校准
  
	while(ADC_GetCalibrationStatus(ADC1));//等待ADC1校准完成
  
	//ADC_SoftwareStartConvCmd(ADC1, ENABLE); //使能ADC1软件开始转换
	
	BATTERY_0.BatReal = 3950;//单位为mv 电池实际电压  校准电压时修改
  BATTERY_0.ADinput = 1980;//单位为mv R15和R17连接处电压 校准电压时修改
  BATTERY_0.ADRef   = 3000;//单位为mv 单片机供电电压   校准电压时修改
  BATTERY_0.Bat_K   = 2;//计算电压计算系数
  BATTERY_0.BatteryADmin = 2000;//电压门限AD值
	
	CURRENT_0.ADRef = 3000;
	CURRENT_0.gain = 100.0;
	CURRENT_0.Rom = 10;
}



uint16_t adcBatteryConversion(void)
{
	uint16_t ADCTEMP;
	uint8_t i;
	for(i=0;i< ADC_MEAN_SIZE;i++)
	{
		ADCTEMP += adcGetChannelValue( BATTERYCHANNEL);

	}
	BATTERY_0.BatteryAD = ADCTEMP / ADC_MEAN_SIZE;  
	BATTERY_0.BatteryVal = BATTERY_0.Bat_K * BATTERY_0.BatteryAD * BATTERY_0.ADRef / 4096;//实际电压 值计算	
	
	return BATTERY_0.BatteryVal;
}
	
uint16_t adcCurrentConversion(void)
{
	uint16_t ADCTEMP;
	uint8_t i;
		
	for(i=0;i< ADC_MEAN_SIZE;i++)
	{
		ADCTEMP += adcGetChannelValue( CURRENTCHANNEL);
	}
	CURRENT_0.CurrentAD =  ADCTEMP / ADC_MEAN_SIZE; 
	CURRENT_0.CURRENTVal = CURRENT_0.CurrentAD * CURRENT_0.ADRef / 4096 / CURRENT_0.Rom / CURRENT_0.gain ;
	
	return CURRENT_0.CURRENTVal;
}

uint16_t adcGetChannelValue(uint8_t channel)
{
	  //设置指定ADC的规则组通道，一个序列，采样时间
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_239Cycles5 );    //ADC1,ADC通道,采样时间为239.5周期                   
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);        //使能指定的ADC1的软件转换启动功能         
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束
    return ADC_GetConversionValue(ADC1);    //返回最近一次ADC1规则组的转换结果
}
