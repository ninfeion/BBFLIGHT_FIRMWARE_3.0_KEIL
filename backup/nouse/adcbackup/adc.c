#include "adc.h"
#include "stm32f10x_dma.h"


//TCAD->PA0->ADC12_IN0
//AD_VBAT->PA1->ADC12_IN1



uint16_t ADC_DMAValue[BAT_filter_times][ChannelNum];
//uint16_t ADC_DMAValue[BAT_filter_times];

uint8_t ADCInit(void)
{
	ADC_InitTypeDef ADC_InitStructure;//����ADC��ʼ���ṹ�����
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //ģ���������� 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE ); 
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);                                       //72M/6=12,ADC���ʱ�䲻�ܳ���14M
        
  ADC_DeInit(ADC1); 
	
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC1��ADC2�����ڶ���ģʽ
  ADC_InitStructure.ADC_ScanConvMode = DISABLE; //ʹ��ɨ��
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//ADCת������������ģʽ
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//���������ת��
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ת�������Ҷ���
  ADC_InitStructure.ADC_NbrOfChannel = 1;//ת��ͨ����ĿΪ2
  ADC_Init(ADC1, &ADC_InitStructure); //��ʼ��ADC
	
  //����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	//ADC1,ADCͨ��x,�������˳��ֵΪy,����ʱ��Ϊ239.5����  
	
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 ); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5 );
	
	
  //ADC_DMACmd(ADC1, ENABLE);//ʹ��ADC1ģ��DMA  // ����ADC��DMA֧�֣�Ҫʵ��DMA���ܣ������������DMAͨ���Ȳ�����
	
	//ADC_TempSensorVrefintCmd(ENABLE); //�����ڲ��¶ȴ���������:
  ADC_Cmd(ADC1, ENABLE);//ʹ��ADC1
  ADC_ResetCalibration(ADC1); //����ADC1У׼�Ĵ���
  while(ADC_GetResetCalibrationStatus(ADC1));//�ȴ�ADC1У׼�������
  ADC_StartCalibration(ADC1);//��ʼADC1У׼
  while(ADC_GetCalibrationStatus(ADC1));//�ȴ�ADC1У׼���
  ADC_SoftwareStartConvCmd(ADC1, ENABLE); //ʹ��ADC1�����ʼת��
	
	BATTERY_0.BatReal = 3950;//��λΪmv ���ʵ�ʵ�ѹ  У׼��ѹʱ�޸�
  BATTERY_0.ADinput = 1980;//��λΪmv R15��R17���Ӵ���ѹ У׼��ѹʱ�޸�
  BATTERY_0.ADRef   = 3000;//��λΪmv ��Ƭ�������ѹ   У׼��ѹʱ�޸�
  BATTERY_0.Bat_K   = 2;//�����ѹ����ϵ��
  BATTERY_0.BatteryADmin = 2000;//��ѹ����ADֵ
	
	CURRENT_0.ADRef = 3000;
	CURRENT_0.gain = 100.0;
	CURRENT_0.Rom = 10;
	
	//ADC_DMA_Configuration();
	
	return 1;
}

void ADC_DMA_Configuration(void) 
{   
	DMA_InitTypeDef DMA_InitStructure;  
	
	DMA_DeInit(DMA1_Channel1); //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //DMA����ADC����ַ 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_DMAValue; //DMA�ڴ����ַ  
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //�ڴ���Ϊ���ݴ����Ŀ�ĵ� 
	DMA_InitStructure.DMA_BufferSize = ChannelNum * BAT_filter_times;// //DMAͨ����DMA����Ĵ�С
																													//����ͨ�� һ���ɼ��˴�
  
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�Ĵ������� 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //�ڴ��ַ�Ĵ������� 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //���ݿ��Ϊ16λ  
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //���ݿ��Ϊ16λ  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //������ѭ������ģʽ  
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMAͨ�� xӵ�и����ȼ�  
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��  
	DMA_Init(DMA1_Channel1, &DMA_InitStructure); //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��  
	
	DMA_Cmd(DMA1_Channel1, ENABLE);
} 

uint16_t ADC_CONV_BATTERY(void)
{
	uint16_t ADCTEMP;
	uint8_t i;
	for(i=0;i<BAT_filter_times;i++)
	{
		//ADCTEMP += ADC_DMAValue[i][BATTERYCHANNEL];
		ADC1->SQR3&=0XFFFFFFE0;//��������1 
		ADC1->SQR3|=BATTERYCHANNEL;//ͨ��2		  			    
		ADC1->CR2|=1<<22;       //��������ת��ͨ�� 
		while(!(ADC1->SR&1<<1));//�ȴ�ת������	 	   
		ADCTEMP += ADC1->DR;		    //����adcֵ	
	}
	BATTERY_0.BatteryAD = ADCTEMP / BAT_filter_times;  
	BATTERY_0.BatteryVal = BATTERY_0.Bat_K * BATTERY_0.BatteryAD * BATTERY_0.ADRef / 4096;//ʵ�ʵ�ѹ ֵ����	
	
	return BATTERY_0.BatteryVal;
}
	
uint16_t ADC_CONV_CURRENT(void)
{
	uint16_t ADCTEMP;
	uint8_t i;
		
	for(i=0;i<BAT_filter_times;i++)
	{
		ADCTEMP += ADC_DMAValue[i][CURRENTCHANNEL];
	}
	CURRENT_0.CurrentAD =  ADCTEMP / BAT_filter_times;  //����3λ=����8
	CURRENT_0.CURRENTVal = CURRENT_0.CurrentAD * CURRENT_0.ADRef / 4096 / CURRENT_0.Rom / CURRENT_0.gain ;
	
	return CURRENT_0.CURRENTVal;
}
