#include "nrfapp.h"
#include "system_config.h"

uint8_t i;

void TIM2_IRQHandler(void)
{			
	if(i)
	{
		LedD_on;
		delay_noInt_ms(1000);
		i=0;
	}
	else
	{
		LedD_off;
		delay_noInt_ms(1000);
		i=1;
	}
	/*LedD_on;
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
								
						  NRF_Write_Buf(WR_TX_PLOAD, NRF24L01_TXDATA, TX_PLOAD_WIDTH);
								
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
												  
		case TX_MODE: timecount = 0;
					  while((NrfState == TX_MODE) && (timecount < TIMEOUT*1500))
					  {
						  timecount ++;
							  
					  }
					  NrfState = TRANS_TO_RX;
					  break;
		case RX_MODE: timecount = 0;
					  while((NrfState == RX_MODE) && (timecount < TIMEOUT*1500))
					  {
						  timecount ++;
					  }
					  NrfState = TRANS_TO_TX; // Time out  
					  LedB_on;
					  delay_noInt_ms(100);
					  LedB_off;
			          break;
	}	
	LedD_off;
	delay_noInt_ms(100);*/
	TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
	
}

void nrfTim2Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  //打开时钟
    
    TIM_DeInit(TIM2);

    TIM_TimeBaseStructure.TIM_Period = 7200;//定时1ms
    TIM_TimeBaseStructure.TIM_Prescaler = 10-1;//预分频 72/(10-1 +1)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
    // NVIC_PriorityGroup 2
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//串口打印定时器，优先级低于姿态解算
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    TIM_Cmd(TIM2,ENABLE);
}



