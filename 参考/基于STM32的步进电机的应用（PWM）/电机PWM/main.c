#include"stm32f10x.h"

#include"time.h"
#include"pwm.h"

#define  GPIOB_SZ	 (*(volatile unsigned long *)0x40010c0c)



u32 time;


int main()
{
	time=8000;
	TIM4_NVIC_Configuration();
	TIM4_Configuration();
	TIM3_PWM_Init();
	TIM2_PWM_Init();
	TIM_Cmd(TIM3, ENABLE);
	START_TIME;
//	TIM_Cmd(TIM2, ENABLE);
	while(1)
	{
	
	}								
}


void TIM4_IRQHandler(void)			//定时器中断程序
{
	if ( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
	{	
		TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);
		TIM_Cmd(TIM2, ENABLE);    
  		STOP_TIME; 
	}		 	
}

