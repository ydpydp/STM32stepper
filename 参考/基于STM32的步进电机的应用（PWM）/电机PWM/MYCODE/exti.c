#include"EXTI.h"
#include"NVIC.h"

#include<stdarg.h>





void EXTI_Config(void)
{	
	/*定义一个EXTI_InitTypeDef 类型的结构体，名字叫EXTI_InitStructure*/
	EXTI_InitTypeDef EXTI_InitStructure;

	/*初始化中断向量函数*/
	NVIC_Config();	

	/* 连接IO口到中断线 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource9);
	/*中断线*/
	EXTI_InitStructure.EXTI_Line = EXTI_Line5|EXTI_Line6|EXTI_Line7|EXTI_Line9; 
	/*触发模式*/
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

	/*触发信号*/
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发

	/*使能中断线*/
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	/*调用库函数，初始化EXTI*/
	EXTI_Init(&EXTI_InitStructure);

}

void EXTI9_5_IRQHandler(void)			   //按键中断函数
{   
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line9);

		Delay();						 									 						
	}
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line9);

		Delay();						 									 						
	}
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line9);

		Delay();						 									 						
	}
	if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line9);

	
		Delay();						 									 						
	}	
}

void Delay(void)
{	u32 i,j;
	for(i=0;i<2000;i++)
	for(j=0;j<3000;j++);
}
