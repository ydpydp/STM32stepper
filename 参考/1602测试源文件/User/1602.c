#include"stm32f10x.h"
#include"1602.h"

//sbit rs=P2^5;//gpioa_0
//sbit rw=P2^6;//gpioa_1
//sbit en=P2^7;//gpioa_2

void Delayms(unsigned short time)//nms
{
    unsigned short i, j;
	
	for(; time > 0; time--){
	    for(j = 0; j < 10; j++){
		    for(i = 0; i < 1000; i++);
		}
	}

} 					  

void write1602_com(u8 com)
		{
			GPIOD->ODR&=0xff00;//??
		    GPIOA->ODR&=0xfffb;//GPIOA->ODR&=0<<2;//********* en=0
			GPIOA->ODR&=0xfffe;  //*******rs=0;
			GPIOD->ODR|=com;
			Delayms(1);	  //延时太少不行
			GPIOA->ODR|=1<<2;//******en=1
		   	Delayms(1);
			GPIOA->ODR&=0xfffb;//GPIOA->ODR&=0<<2;//******en=0	
		}
void write1602_dat(u8 dat)
		{
			GPIOD->ODR&=0xff00;
			GPIOA->ODR&=0xfffb;//GPIOA->ODR&=0xfd;//0<<2;//********en=0
			GPIOA->ODR|=1;    //*******rs=1;
			GPIOD->ODR|=dat;  //*****P0=dat;
			Delayms(1);
			GPIOA->ODR|=1<<2; //*******en=1;
		   	Delayms(1);
		    GPIOA->ODR&=0xfffb;//GPIOA->ODR&=0xfd;//GPIOA->ODR&=0<<2;  //*******en=0;		
		}
void lcd1602_init()
	{

	    //RCC->APB2ENR|=1<<2;    //使能PORTA时钟	   	 
    	//RCC->APB2ENR|=1<<5;    //使能PORTD时钟
		GPIOA->CRL&=0XFFFFF000; 
	    GPIOA->CRL|=0X00000333;//PA0-2推挽输出   	 
        									  
	    GPIOD->CRL&=0X00000000;
	    GPIOD->CRL|=0X33333333;//PD推挽输出

		GPIOA->ODR|=0xfff7;//en=0
		GPIOA->ODR&=0xfffd;//GPIOA->ODR&=0<<1; //********rw=0;	
		GPIOA->ODR&=0xfffb;//->ODR&=0<<2; //********en=0; 
		GPIOD->ODR|=0xffff;
		Delayms(30);
		write1602_com(0x38);
		write1602_com(0x0c);
		write1602_com(0x06);
		write1602_com(0x01);
	}
//numadd 0-0x0f（可显示）-0x27共40个，可以写17代0x10
void write1602_Achar(u8 hang,u8 numadd,u8 x)

		{
			if(hang==1)
				hang=0x80;
			else
				hang=0x80+0x40;
			numadd=numadd+hang;
			write1602_com(numadd);
			write1602_dat(x+0x30);
		}
void write1602_string(u8 hang,u8 numadd,u8 *p)
		{
		
		   if(hang==1)
				hang=0x80;
			else
				hang=0x80+0x40;
			numadd=numadd+hang;	
			write1602_com(numadd);
			while(*p!='\0')
			write1602_dat(*p++);

		}

