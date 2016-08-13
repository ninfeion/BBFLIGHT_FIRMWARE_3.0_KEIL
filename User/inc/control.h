#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "stm32f10x.h"
#include "systeminit.h"

void pidControl(ImuData *tarData);
void motorUpdate(ImuData *tarData, RespondMess *tarMessData);

#endif

