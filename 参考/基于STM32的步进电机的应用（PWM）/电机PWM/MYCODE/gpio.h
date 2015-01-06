#ifndef __GPIO_H
#define	__GPIO_H

#include "stm32f10x.h"

#define    PB0_1    GPIO_SetBits(GPIOB, GPIO_Pin_0)
#define	   PB0_0	GPIO_ResetBits(GPIOB, GPIO_Pin_0)
#define    PB1_1    GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define	   PB1_0	GPIO_ResetBits(GPIOB, GPIO_Pin_1)
#define    PB6_1    GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define	   PB6_0	GPIO_ResetBits(GPIOB, GPIO_Pin_6)
#define    PB5_1    GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define	   PB5_0	GPIO_ResetBits(GPIOB, GPIO_Pin_5)

#define    LED_ON   GPIO_SetBits(GPIOC, GPIO_Pin_0)
#define    LED_OFF	GPIO_ResetBits(GPIOC, GPIO_Pin_0)

void GPIO_Config(void);


#endif
