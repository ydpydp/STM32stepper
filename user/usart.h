/*********************************************************************************************************
*
* File                : usart.h
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/

#ifndef _USART_H
#define _USART_H

#include <stdio.h>
#include <stm32f10x.h>


void Rcc_Configuration(void);
void UsartGPIO_Configuration(void);
void USART_Configuration(void);
void UsartGPIO_CTRT_Configuration(void);
void USART_CTRT_Configuartion(void);

#endif /*_USART_H*/
