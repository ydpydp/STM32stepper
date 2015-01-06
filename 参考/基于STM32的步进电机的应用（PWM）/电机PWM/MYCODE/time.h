#ifndef TIME_H
#define TIME_H

#include "stm32f10x.h"


 
#define START_TIME  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);TIM_Cmd(TIM4, ENABLE)
#define STOP_TIME  TIM_Cmd(TIM4, DISABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , DISABLE)

void TIM4_NVIC_Configuration(void);
void TIM4_Configuration(void);

#endif	
