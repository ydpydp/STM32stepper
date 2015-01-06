#include "NVIC.h"

void NVIC_Config(void)
{	
	/*定义一个NVIC_InitTypeDef 类型的结构体，名字叫NVIC_InitStructure*/ 
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 配置中断使用组合0*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
		
	/*配置选中的中断向量*/
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;

	/*配置抢占优先级*/
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;

	/*配置响应优先级*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	/*使能中断向量*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	/*调用库函数，初始化中断向量*/
	NVIC_Init(&NVIC_InitStructure);
					 
	
}
