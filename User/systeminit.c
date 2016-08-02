#include "systeminit.h"
#include "spinrf.h"
#include "pwm.h"
#include "adc.h"
#include "system_loop.h"
#include "i2c.h"
#ifdef _USE6050_
	#include "mpu6050.h"
#else
	#include "mpu9250.h"
#endif
#include "MS5611.h"
#include "delay.h"
#ifdef _USBDEBUG_
	#include "usb2com.h"
	#include "debug.h"
#else
	#include "usart.h"
#endif

struct SYSTEM_STATE BBSYSTEM;

/*
uint8_t sysInit(void)
{
  sysclockInit();
  systickInit();
  ledInit();
	spi2NrfInit();
	if(NRF24L01_Check() == 1)			BBSYSTEM.NRFONLINE = 1;
	//NRF check��ce���ǵ͵�ƽ��������ģʽҪ�ں���
	#ifdef _SLAVEMODE_
		nrfSlaveMode();
	#endif

	adcInit();
	i2cInit();
	mpu9250Init();
	if(MPU9250_Check() == TRUE)		BBSYSTEM.MPU9250ONLINE = 1;
	ms5611Init();
	pwmInit();	
	systemLoopInit();
	#ifdef _USBDEBUG_
		USBVCOM_Init();
		//DEBUG_TIM2_Init();
	#else
		//USARTInit(9600);
	#endif	
	
	//enableNrfIqr();
  return 1;
}
*/

uint8_t sysclockInit(void)
{
    ErrorStatus HSEStartUpStatus;
	
    RCC_DeInit();/* RCC���� */
    RCC_HSEConfig(RCC_HSE_ON); /*ʹ��HSE*/
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)   //�ⲿ����ʹ�ܳɹ�
    {
        RCC_HCLKConfig(RCC_SYSCLK_Div1); /* ����HCLK = SYSCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1); /* ����PCLK2 = HCLK */
        RCC_PCLK1Config(RCC_HCLK_Div2); /* ����PCLK1 = HCLK/2 */

        FLASH_SetLatency(FLASH_Latency_2);
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);  /* RCC_PLLSource_HSE_Div1Ϊ���þ���ķ�Ƶϵ��;RCC_PLLMul_9Ϊ��Ƶ�� */
        RCC_PLLCmd(ENABLE);
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //�ȴ�pll����
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);   /* ѡ��PLLΪϵͳ��ʱ�� */
        while(RCC_GetSYSCLKSource() != 0x08); //�ж�pll�Ƿ�Ϊϵͳʱ��
		return 1;

	}
	else
		return 0;
}

uint8_t systickInit(void)
{
	/*
	��SysTick�Ĳ���������������1��������  
	void SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource); 
	���ܣ�����SysTickʱ��Դ  
	������
	SysTick_CLKSource��  
	SysTick_CLKSource_HCLK_Div8:����ʱ�ӳ���8ѡΪSysTickʱ��Դ��
	SysTick_CLKSource_HCLK:����SysTickʱ��ѡΪʱ��Դ�� 

	uint32_t SysTick_Config(uint32_t ticks);  
	���ܣ�����SysTick��װ����ֵ��ʹ���жϣ���������
	������  ticks��24λ���ڵ���װֵ 
	���أ�1ʧ��,0�ɹ� */
	return SysTick_Config(72); //��1000����1us
}

uint8_t ledInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;//����GPIO��ʼ���ṹ��

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //ʹ��GPOIA,GPIOB����ʱ��	   
   	

    /*
    RCC_APB2Periph_AFIO ���ܸ���IOʱ�� 
    RCC_APB2Periph_GPIOA GPIOAʱ�� 
    RCC_APB2Periph_GPIOB GPIOBʱ�� 
    RCC_APB2Periph_GPIOC GPIOCʱ�� 
    RCC_APB2Periph_GPIOD GPIODʱ�� 
    RCC_APB2Periph_GPIOE GPIOEʱ�� 
    RCC_APB2Periph_ADC1 ADC1ʱ�� 
    RCC_APB2Periph_ADC2 ADC2ʱ�� 
    RCC_APB2Periph_TIM1 TIM1ʱ�� 
    RCC_APB2Periph_SPI1 SPI1ʱ�� 
    RCC_APB2Periph_USART1 USART1ʱ�� 
    RCC_APB2Periph_ALL ȫ��APB2����ʱ��
    */
   	
    //LED GPIO��ʼ��
    //LED_0->D3->PA4
    //LED_3->D4->PA7
    //LED_2->D5->PA6
    //LED_1->D6->PA5
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
    LedA_off;
    LedB_off;
    LedC_off;
    LedD_off;
  
    /*
	GPIO_Mode_AIN = 0x0,     //ģ������   
	GPIO_Mode_IN_FLOATING = 0x04, //��������   
	GPIO_Mode_IPD = 0x28,    //��������   
	GPIO_Mode_IPU = 0x48,    //��������   
	GPIO_Mode_Out_OD = 0x14, //��©���   
	GPIO_Mode_Out_PP = 0x10,  //�������   
	GPIO_Mode_AF_OD = 0x1C,   //��©����   
	GPIO_Mode_AF_PP = 0x18    //���츴�� 

    1����������GPIO_IN_FLOATING �����������룬������KEYʶ��RX1
    2������������GPIO_IPU����IO�ڲ�������������
    3������������GPIO_IPD���� IO�ڲ�������������
    4��ģ������GPIO_AIN ����Ӧ��ADCģ�����룬���ߵ͹�����ʡ��
    5����©���GPIO_OUT_OD ����IO���0��GND��IO���1�����գ���Ҫ����������裬����ʵ������ߵ�ƽ�������Ϊ1ʱ��IO�ڵ�״̬�������������ߵ�ƽ���������ǿ�©���ģʽ������IO��Ҳ�Ϳ������ⲿ��·�ı�Ϊ�͵�ƽ�򲻱䡣���Զ�IO�����ƽ�仯��ʵ��C51��IO˫����
    6���������GPIO_OUT_PP ����IO���0-��GND�� IO���1 -��VCC��������ֵ��δ֪��
    7�����ù��ܵ��������GPIO_AF_PP ����Ƭ�����蹦�ܣ�I2C��SCL,SDA��
    8�����ù��ܵĿ�©���GPIO_AF_OD����Ƭ�����蹦�ܣ�TX1,MOSI,MISO.SCK.SS��

    ���ڴ��ڣ������������ֻ��115.2k����ô��2M��GPIO�������ٶȾ͹��ˣ���ʡ��Ҳ����С��
    ����I2C�ӿڣ�����ʹ��400k�����ʣ��������������Щ����ô��2M��GPIO�������ٶȻ���������ʱ����ѡ��10M��GPIO�����ٶȡ�
    ����SPI�ӿڣ�����ʹ��18M��9M�����ʣ���10M��GPIO�������ٶ���Ȼ�����ˣ���Ҫѡ��50M��GPIO�������ٶȡ�

    IIC �Ƽ�10MHZ
    SPI �Ƽ�50MHZ
    */
    return 1;
}
  




  
