#ifndef _DEBUG_H_
#define _DEBUG_H_
#include "stm32f10x.h"
#include "system_loop.h"

#define USB_PRINTF_BUFFER_SIZE 128

void USART_printf(uint8_t *TarStr, ...);
void USB_printf(uint8_t *TarStr, ...);
void DEBUG_TIM2_Init(void);
void usbDebugLoop(void);
void USB_DEBUG_RADIO(void);

#endif


