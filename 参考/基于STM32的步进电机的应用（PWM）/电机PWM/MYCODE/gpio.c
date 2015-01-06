#include "GPIO.h"

void GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	/*开启对应时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);
	/*按键上拉*/			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_9;			 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		   //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/*PB0、PB1、PB4、PB5推挽输出*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_5 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;	   //推挽
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //设置IO速度为50M
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*LED推挽输出（测试用）*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	PB0_0;PB1_1;PB6_1;PB5_0;
	
}
