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
	ADC_InitTypeDef ADC_InitStructure;//����ADC��ʼ���ṹ�����
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE ); 
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);    
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //ģ���������� 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	                              
  ADC_DeInit(ADC1); 
	
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC1��ADC2�����ڶ���ģʽ
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; //ʹ��ɨ��
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//ADCת������������ģʽ
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//���������ת��
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ת�������Ҷ���
  ADC_InitStructure.ADC_NbrOfChannel = 1;//ת��ͨ����ĿΪ2
  ADC_Init(ADC1, &ADC_InitStructure); //��ʼ��ADC
	

  ADC_Cmd(ADC1, ENABLE);//ʹ��ADC1
  ADC_ResetCalibration(ADC1); //����ADC1У׼�Ĵ���
	
  while(ADC_GetResetCalibrationStatus(ADC1));//�ȴ�ADC1У׼�������
	
  ADC_StartCalibration(ADC1);//��ʼADC1У׼
  
	while(ADC_GetCalibrationStatus(ADC1));//�ȴ�ADC1У׼���
  
	//ADC_SoftwareStartConvCmd(ADC1, ENABLE); //ʹ��ADC1�����ʼת��
	
	BATTERY_0.BatReal = 3950;//��λΪmv ���ʵ�ʵ�ѹ  У׼��ѹʱ�޸�
  BATTERY_0.ADinput = 1980;//��λΪmv R15��R17���Ӵ���ѹ У׼��ѹʱ�޸�
  BATTERY_0.ADRef   = 3000;//��λΪmv ��Ƭ�������ѹ   У׼��ѹʱ�޸�
  BATTERY_0.Bat_K   = 2;//�����ѹ����ϵ��
  BATTERY_0.BatteryADmin = 2000;//��ѹ����ADֵ
	
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
	BATTERY_0.BatteryVal = BATTERY_0.Bat_K * BATTERY_0.BatteryAD * BATTERY_0.ADRef / 4096;//ʵ�ʵ�ѹ ֵ����	
	
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
	  //����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_239Cycles5 );    //ADC1,ADCͨ��,����ʱ��Ϊ239.5����                   
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);        //ʹ��ָ����ADC1�����ת����������         
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������
    return ADC_GetConversionValue(ADC1);    //�������һ��ADC1�������ת�����
}
