#ifndef __1602_H
#define __1602_H
#include "stm32f10x.h"

void write1602_com(u8 com);
void write1602_dat(u8 dat);
void lcd1602_init(void);
void write1602_Achar(u8 hang,u8 numadd,u8 x);
void write1602_string(u8 hang,u8 numadd,u8 *p);

#endif /* __STM32F10x_IT_H */



