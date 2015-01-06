/**
  ******************************************************************************
  * @file    Project/Template/main.c 
  * @author  MCD Application Team
  * @version V3.0.0
  * @date    04/06/2009
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_conf.h"
#include "stdio.h"
#include "stdlib.h"


/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSCLK_FREQ_HSE
#define TIMx_PRE_EMPTION_PRIORITY 1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


#define  CONTROL_PORT    GPIOE
#define  AXIS_PULSE_1    GPIO_Pin_0
#define  AXIS_PULSE_2    GPIO_Pin_2
#define  AXIS_PULSE_3    GPIO_Pin_4
#define  AXIS_PULSE_4    GPIO_Pin_6
#define  AXIS_DIR_1    GPIO_Pin_1
#define  AXIS_DIR_2    GPIO_Pin_3
#define  AXIS_DIR_3    GPIO_Pin_5
#define  AXIS_DIR_4    GPIO_Pin_7
#define  AXIS_LMTPOS_1    GPIO_Pin_8
#define  AXIS_LMTNEG_1    GPIO_Pin_9
#define  AXIS_LMTPOS_2    GPIO_Pin_10
#define  AXIS_LMTNEG_2    GPIO_Pin_11
#define  AXIS_LMTPOS_3    GPIO_Pin_12
#define  AXIS_LMTNEG_3    GPIO_Pin_13
#define  AXIS_LMTPOS_4    GPIO_Pin_14
#define  AXIS_LMTNEG_4    GPIO_Pin_15
#define  AXIS_HOME_1    GPIO_Pin_0
#define  AXIS_HOME_2    GPIO_Pin_1
#define  AXIS_HOME_3    GPIO_Pin_2
#define  AXIS_HOME_4    GPIO_Pin_3


#define POSITIVE	1
#define NEGATIVE	-1

#define CW  0
#define CCW 1

typedef struct {
  //! What part of the speed ramp we are in.
  u8 run_state ;
  //! Direction stepper motor should move.
  u8 dir ;
  //! Peroid of next timer delay. At start this value set the accelration rate.
  s32 step_delay;
  //! What step_pos to start decelaration
  u32 decel_start;
  //! Sets deceleration rate.
  s32 decel_val;
  //! Minimum time delay (max speed)
  s32 min_delay;
  //! Counter used when accelerateing/decelerateing to calculate step_delay.
  s32 accel_count;
} speedRampData;

GPIO_InitTypeDef GPIO_InitStructure;

#define T1_FREQ 1000000
#define SPR 1600
// Maths constants. To simplify maths when calculating in AxisMoveRel().
#define ALPHA (2*3.14159/SPR)                    // 2*pi/spr
#define A_T_x100 ((long)(ALPHA*T1_FREQ*100))     // (ALPHA / T1_FREQ)*100
#define T1_FREQ_148 ((int)((T1_FREQ*0.676)/100)) // divided by 100 and scaled by 0.676
#define A_SQ (long)(ALPHA*2*100000*100000)         // 
#define A_x20000 (int)(ALPHA*20000)              // ALPHA*20000
// Speed ramp states
#define STOP  0
#define ACCEL 1
#define DECEL 2
#define RUN   3


 s32 steps[4]={150000, 150000, 150000, 150000};//运动距离
 u32 accel[4]={10000,10000, 10000, 10000};//加速度
 u32 decel[4]={10000, 10000, 10000, 10000};//减速度
 u32 speed[4]={10000, 10000, 10000, 10000};//速度
 s32 stpdecel[4]={65000, 65000, 65000, 65000};//急停减速度
 u8 addr485=0;
 u8 LmtSnsPos[4]={1,1,1,1};//正限位电平
 u8 LmtSnsNeg[4]={1,1,1,1};//负限位电平
 u8 HomeSns[4]={1,1,1,1};//回零电平
 s32 position[4]={0,0,0,0};//当前位置
 s32 HomePos[4]={0,0,0,0};//
 s32 SoftLmtPos[4]={0x7fffffff, 0x7fffffff, 0x7fffffff, 0x7fffffff};
 s32 SoftLmtNeg[4]={0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff};
 

 u16 AxisPulsePin[4]={AXIS_PULSE_1, AXIS_PULSE_2, AXIS_PULSE_3, AXIS_PULSE_4};
 u16 AxisDirPin[4]={AXIS_DIR_1, AXIS_DIR_2, AXIS_DIR_3, AXIS_DIR_4};
 u16 LmtPosPin[4]={AXIS_LMTPOS_1, AXIS_LMTPOS_2, AXIS_LMTPOS_3, AXIS_LMTPOS_4};
 u16 LmtNegPin[4]={AXIS_LMTNEG_1, AXIS_LMTNEG_2, AXIS_LMTNEG_3, AXIS_LMTNEG_4};
 u16 HomePin[4]={AXIS_HOME_1, AXIS_HOME_2, AXIS_HOME_3, AXIS_HOME_4};
 u16 AddrPin[4]={GPIO_Pin_7, GPIO_Pin_6, GPIO_Pin_15, GPIO_Pin_14};
 GPIO_TypeDef *AxisLmtPosPort[4]={GPIOE, GPIOE, GPIOE, GPIOE};
 GPIO_TypeDef *AxisLmtNegPort[4]={GPIOE, GPIOE, GPIOE, GPIOE};
 GPIO_TypeDef *AxisPulsePort[4]={GPIOE, GPIOE, GPIOE, GPIOE};
 GPIO_TypeDef *AxisDirPort[4]={GPIOE, GPIOE, GPIOE, GPIOE};
 GPIO_TypeDef *HomePort[4]={GPIOC, GPIOC, GPIOC, GPIOC};
 GPIO_TypeDef *AddrPort[4]={GPIOC, GPIOC, GPIOB, GPIOB};
 uint32_t AxisEXTILine[4]={EXTI_Line0, EXTI_Line1, EXTI_Line2, EXTI_Line3};
 uint8_t AxisEXTIPinSource[4]={GPIO_PinSource0, GPIO_PinSource1, GPIO_PinSource2, GPIO_PinSource3};
 
 bool bLmtPos[4]={FALSE, FALSE, FALSE, FALSE};
 bool bLmtNeg[4]={FALSE, FALSE, FALSE, FALSE};
 bool bStopCmd[4]={FALSE, FALSE, FALSE, FALSE};
 bool bEmgStopping[4]={FALSE, FALSE, FALSE, FALSE}; //是否碰限位急停
 bool bEnableSoftLmt[4]={FALSE, FALSE, FALSE, FALSE};
 
 bool bHomeOK[4]={FALSE, FALSE, FALSE, FALSE};
 bool bZeroCapture[4]={FALSE, FALSE, FALSE, FALSE};
 s32 ZeroDir[4]={POSITIVE, POSITIVE, POSITIVE, POSITIVE};	  //回零方向
 s32 ZeroOffset[4]={1000, 1000, 1000, 1000}; //零点偏移
 u32 ZeroBackDistance[4]={6400, 6400, 6400, 6400}; //回零回退距离 
 u8 ZeroStep[4]={0, 0, 0, 0};	
 u32 HomeFastSpeed[4]={10000, 10000, 10000, 10000};//回零快搜速度
 u32 HomeSlowSpeed[4]={2000, 2000, 2000, 2000};//回零慢搜速度
 
	speedRampData srd[4];
 
 enum
 {
 	IDEL,
 	FASTSEEK,
	FASTSEEKSTOP,
	FASTSEEKBACK,
	SLOWSEEK,
	SLOWSEEKSTOP,
	MOVETOZERO
 };
 
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void NVIC_Configuration(void);
void Timer_Init(void);
void EXTILine_Config(void);
void GPIO_Configuration(void);
int fputc(int ch, FILE *f);
 
 
u8 MotionStatus[4]={0, 0, 0, 0};//是否在运动？0：停止，1：运动

bool bDataOK;
USART_InitTypeDef USART_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;

uint8_t RxBuffer[12];
uint8_t RxCounter;

void AxisMoveRel(u32 axis, s32 step, u32 accel, u32 decel, u32 speed);
void AxisMoveAbs(u32 axis, s32 step, u32 accel, u32 decel, u32 speed);
u32 sqrt(u32 x);
void LimitDetect(u8 axis);
void USART_Configuration(void);
void AxisHome(u8 axis);

#define EnableHomeCapture1	EXTI->IMR |= GPIO_Pin_0
#define DisableHomeCapture1	EXTI->IMR &=~GPIO_Pin_0
#define EnableHomeCapture2	EXTI->IMR |= GPIO_Pin_1
#define DisableHomeCapture2	EXTI->IMR &=~GPIO_Pin_1  
#define EnableHomeCapture3	EXTI->IMR |= GPIO_Pin_2
#define DisableHomeCapture3	EXTI->IMR &=~GPIO_Pin_2  
#define EnableHomeCapture4	EXTI->IMR |= GPIO_Pin_3
#define DisableHomeCapture4	EXTI->IMR &=~GPIO_Pin_3  

void EnableHomeCapture(u8 axis) {EXTI->IMR |=HomePin[axis];}
void DisableHomeCapture(u8 axis)	{EXTI->IMR &=~HomePin[axis];}

/**
  * @brief  Main program.
  * @param  None
  * @retval : None
  */
int main(void)
{
  /* Setup STM32 system (clock, PLL and Flash configuration) */

	u8 axis=0;


	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);


	for(axis=0; axis<4; axis++)
	{
		GPIO_InitStructure.GPIO_Pin = AxisPulsePin[axis];
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(AxisPulsePort[axis], &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = AxisDirPin[axis];
		GPIO_Init(AxisDirPort[axis], &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin=LmtPosPin[axis];
		GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;

		GPIO_Init(AxisLmtPosPort[axis], &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin=LmtNegPin[axis];
		GPIO_Init(AxisLmtNegPort[axis], &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin=HomePin[axis];
		GPIO_Init(HomePort[axis], &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin=AddrPin[axis];
		GPIO_Init(AddrPort[axis], &GPIO_InitStructure);
	}
	

	
	Timer_Init();

	EXTILine_Config();
  NVIC_Configuration();
	
//    AxisMoveRel(1, 550000, 25000, 25000, 50000);
//    AxisMoveRel(2, 550000, 32000, 32000, 40000);
//    AxisMoveRel(3, 550000, 20000, 20000, 60000);
//  	AxisMoveRel(4, 550000, 30000, 30000, 45000);
	

	


	
  while (1)																
  {
		LimitDetect(1);
		LimitDetect(2);
		LimitDetect(3);
		LimitDetect(4);
		AxisHome(1);
		AxisHome(2);
		AxisHome(3);
		AxisHome(4);
  }
}

int fputc(int ch, FILE *f)
{

  USART_SendData(USART1, (uint8_t) ch); /*发送一个字符函数*/ 

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)/*等待发送完成*/
  {
  
  }
  return ch;

}

void GPIO_Configuration(void)
{
//   GPIO_InitTypeDef GPIO_InitStructure;

//   /* GPIOA Configuration:TIM2 Channel1, 2, 3 and 4 as alternate function push-pull */
//   GPIO_InitStructure.GPIO_Pin =AXIS_PULSE_1 | AXIS_PULSE_2 |AXIS_PULSE_3 | AXIS_PULSE_4 | AXIS_DIR_1 | AXIS_DIR_2 | AXIS_DIR_3 | AXIS_DIR_4;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

//   GPIO_Init(CONTROL_PORT, &GPIO_InitStructure);

//    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
//    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
//    GPIO_Init(MACHINPUT_PORT, &GPIO_InitStructure);


}


void Timer_Init(void)//定时器配置
{ 
	uint16_t PrescalerValue;
	
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

  TIM_TimeBaseStructure.TIM_Period =65535;
  TIM_TimeBaseStructure.TIM_Prescaler =0;    //1 MHz 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	PrescalerValue=(uint16_t) ((SystemFrequency_SysClk / 2) / 1000000) - 1;
	TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);
	TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);
	TIM_PrescalerConfig(TIM4, PrescalerValue, TIM_PSCReloadMode_Immediate);
	TIM_PrescalerConfig(TIM5, PrescalerValue, TIM_PSCReloadMode_Immediate);
  /* Output Compare Active Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Active;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 10;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);

  /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
}

void NVIC_Configuration(void)//中断向量配置
{
	  NVIC_InitTypeDef NVIC_InitStructure;
	  
	
	   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	   //设置向量表的位置和偏移
	#ifdef  VECT_TAB_RAM  
		/* Set the Vector Table base location at 0x20000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 		//向量表位于RAM
	#else  /* VECT_TAB_FLASH  */
		/* Set the Vector Table base location at 0x08000000 */ 
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   //向量表位于FLASH
	#endif

	  /* Enable the TIM2 Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_Init(&NVIC_InitStructure);


	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_Init(&NVIC_InitStructure);



     /* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//使能按键所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	//先占优先级4位,共16级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;					//先占优先级0位,从优先级4位
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	

	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	
}

void EXTILine_Config(void)//外部中断配置
{
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, AxisEXTIPinSource[0]);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, AxisEXTIPinSource[0]);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, AxisEXTIPinSource[0]);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, AxisEXTIPinSource[0]);

	EXTI_InitStructure.EXTI_Line = AxisEXTILine[0] | AxisEXTILine[1] | AxisEXTILine[2] | AxisEXTILine[3];
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	
}

void LimitDetect(u8 axis)//限位开关检测，axis参数是轴号
{
		if(GPIO_ReadInputDataBit(AxisLmtPosPort[axis-1], LmtPosPin[axis-1]))
		{
			if(LmtSnsPos[axis-1]==0)
			{
				bLmtPos[axis-1]=FALSE;
			}
			else
			{
				bLmtPos[axis-1]=TRUE;
			}		
		}
		else
		{
			if(LmtSnsPos[axis-1]==0)
			{
				bLmtPos[axis-1]=TRUE;
			}
			else
			{
				bLmtPos[axis-1]=FALSE;
			}
		}
		if(GPIO_ReadInputDataBit(AxisLmtNegPort[axis-1], LmtNegPin[axis-1]))
		{
			if(LmtSnsNeg[axis-1]==0)
			{
				bLmtNeg[axis-1]=FALSE;
			}
			else
			{
				bLmtNeg[axis-1]=TRUE;
			}		
		}
		else
		{
			if(LmtSnsNeg[axis-1]==0)
			{
				bLmtNeg[axis-1]=TRUE;
			}
			else
			{
				bLmtNeg[axis-1]=FALSE;
			}
		}			
}

void AxisHome(u8 axis)//轴回零程序，axis参数是轴号
{
	axis--;
	switch(ZeroStep[axis])
	{
		case IDEL:
			break;
		case FASTSEEK:
			if(!MotionStatus[axis])
			{
					if(bZeroCapture[axis])
					{
						ZeroStep[axis]=FASTSEEKBACK;
						bZeroCapture[axis]=FALSE;
						AxisMoveRel(axis+1, (position[axis]-HomePos[axis]+ZeroBackDistance[axis])*ZeroDir[axis]*-1, accel[axis], decel[axis], HomeFastSpeed[axis]);
					}
					else
					{
						ZeroStep[axis]=IDEL;
					}
					
			}
			break;
		case FASTSEEKSTOP:
			break;
		case FASTSEEKBACK:
			if(!MotionStatus[axis])
			{
				switch(axis)
				{
					case 0:
						EnableHomeCapture1;
						break;
					case 1:
						EnableHomeCapture2;
						break;
					case 2:
						EnableHomeCapture3;
						break;
					case 3:
						EnableHomeCapture4;
						break;
				}
				AxisMoveRel(axis+1, 0x7FFFFFFF*ZeroDir[axis], accel[axis], decel[axis], HomeSlowSpeed[axis]);	
				ZeroStep[axis]=SLOWSEEK;
			}
			break;
		case SLOWSEEK:
			if(!MotionStatus[axis])
			{
				if(bZeroCapture[axis])
				{
					ZeroStep[axis]=MOVETOZERO;
					bZeroCapture[axis]=FALSE;
					AxisMoveRel(axis+1, (position[axis]-HomePos[axis]+ZeroOffset[axis])*ZeroDir[axis]*-1, accel[axis], decel[axis], HomeFastSpeed[axis]);
				}
				else
				{
					ZeroStep[axis]=IDEL;
				}
			}
			break;
		case SLOWSEEKSTOP:
			break;
		case MOVETOZERO:
			if(!MotionStatus[axis])
			{
				ZeroStep[axis]=IDEL;
				position[axis]=0;
			}
			break;
	}
}

void AxisMoveAbs(u32 axis, s32 step, u32 accel, u32 decel, u32 speed)//绝对运动，axis参数是轴号，step是目标位置，accel是加速度，decel是减速度，speed是速度
{
  //! Number of steps before we hit max speed.
  u32 max_s_lim;
  //! Number of steps before we must start deceleration (if accel does not hit max speed).
  u32 accel_lim;
	float ftemp=0.0;

	step=step-position[axis-1];
   if(step <0)
    {
			if(GPIO_ReadInputDataBit(AxisLmtNegPort[axis-1], LmtNegPin[axis-1]))
			{
				if(LmtSnsNeg[axis-1]==0)
				{
					bLmtNeg[axis-1]=FALSE;
				}
				else
				{
					bLmtNeg[axis-1]=TRUE;
					return;
				}
						
			}
			else
			{
				if(LmtSnsNeg[axis-1]==0)
				{
					bLmtNeg[axis-1]=TRUE;
					return;
				}
				else
				{
					bLmtNeg[axis-1]=FALSE;	
				}		 	
			}
      srd[axis-1].dir = CCW;
      GPIO_ResetBits(AxisDirPort[axis-1], AxisDirPin[axis-1]);
			step =-step;
    }
  else
   {
		if(GPIO_ReadInputDataBit(AxisLmtPosPort[axis-1], LmtPosPin[axis-1]))
		{
			if(LmtSnsPos[axis-1]==0)
			{
				bLmtPos[axis-1]=FALSE;
			}
			else
			{
				bLmtPos[axis-1]=TRUE;
				return;
			}					
		}
		else
		{
			if(LmtSnsPos[axis-1]==0)
			{
				bLmtPos[axis-1]=TRUE;
				return;
			}
			else
			{
				bLmtPos[axis-1]=FALSE;
			}		 	
		}
    srd[axis-1].dir = CW;
	  GPIO_SetBits(AxisDirPort[axis-1], AxisDirPin[axis-1]);
   }

  if(step == 1)
  {
    // Move one step...
    srd[axis-1].accel_count = -1;
    // ...in DECEL state.
    srd[axis-1].run_state = DECEL;
    // Just a short delay so main() can act on 'running'.
    srd[axis-1].step_delay = 1000;
		switch(axis)
			{
			case 1:
				 TIM2->CCR1=10;
				 TIM2->ARR=10;
				 break;
			case 2:
				 TIM3->CCR1=10;
				 TIM3->ARR=10;
				 break;
			case 3:
				 TIM4->CCR1=10;
				 TIM4->ARR=10;
				 break;
			case 4:
				 TIM5->CCR1=10;
				 TIM5->ARR=10;
				 break;
			}
    
		MotionStatus[axis-1] = 1;
		switch(axis)
			{
			case 1:
				 TIM_Cmd(TIM2, ENABLE);
			 break;
			case 2:
			 TIM_Cmd(TIM3, ENABLE);
			 break;
			case 3:
				 TIM_Cmd(TIM4, ENABLE);
			 break;
			case 4:
				 TIM_Cmd(TIM5, ENABLE);
			 break;
			}
	
  }
  // Only move if number of steps to move is not zero.
  else if(step != 0)
  {
    // Refer to documentation for detailed information about these calculations.

    // Set max speed limit, by calc min_delay to use in timer.
    // min_delay = (alpha / tt)/ w
    srd[axis-1].min_delay = T1_FREQ/speed/2;

    // Set accelration by calc the first (c0) step delay .
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
    srd[axis-1].step_delay = ((long)T1_FREQ*0.676* sqrt(2000000 / accel))/1000/2;
    // Find out after how many steps does the speed hit the max speed limit.
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = speed*speed/(2*accel);
    // If we hit max speed limit before 0,5 step it will round to 0.
    // But in practice we need to move atleast 1 step to get any speed at all.
    if(max_s_lim == 0){
      max_s_lim = 1;
    }

    // Find out after how many steps we must start deceleration.
    // n1 = (n1+n2)decel / (accel + decel)
    if((accel+decel)>step)
		{
//			accel_lim = step*decel/(accel+decel);
			ftemp=(float)decel/(float)(accel+decel);
			accel_lim = (float)step*ftemp;
		}
		else
		{
			accel_lim = step/(accel+decel)*decel;
		}
    // We must accelrate at least 1 step before we can start deceleration.
    if(accel_lim == 0){
      accel_lim = 1;
    }

    // Use the limit we hit first to calc decel.
    if(accel_lim <= max_s_lim){
      srd[axis-1].decel_val = accel_lim - step;
    }
    else{
      srd[axis-1].decel_val =-(s32)(max_s_lim*accel/decel);
    }
    // We must decelrate at least 1 step to stop.
    if(srd[axis-1].decel_val == 0){
      srd[axis-1].decel_val = -1;
    }

    // Find step to start decleration.
    srd[axis-1].decel_start = step + srd[axis-1].decel_val;

    // If the maximum speed is so low that we dont need to go via accelration state.
    if(srd[axis-1].step_delay <= srd[axis-1].min_delay)
	{
      srd[axis-1].step_delay = srd[axis-1].min_delay;
      srd[axis-1].run_state = RUN;
    }
    else{
      srd[axis-1].run_state = ACCEL;
    }

    // Reset counter.
    srd[axis-1].accel_count = 0;
    MotionStatus[axis-1] = 1;
    //OCR1A = 10;
	switch(axis)
	  {
	  case 1:
	  	 TIM2->CCR1=10;
		 TIM2->ARR=10;
		 break;
	  case 2:
		 TIM3->CCR1=10;
		 TIM3->ARR=10;
		 break;
	  case 3:
	     TIM4->CCR1=10;
		 TIM4->ARR=10;
		 break;
	  case 4:
	     TIM5->CCR1=10;
		 TIM5->ARR=10;
		 break;
	  }
	
    // Set Timer/Counter to divide clock by 8
    //TCCR1B |= ((0<<CS12)|(1<<CS11)|(0<<CS10));
	switch(axis)
	  {
	  case 1:
	  	 TIM_Cmd(TIM2, ENABLE);
		 break;
	  case 2:
		 TIM_Cmd(TIM3, ENABLE);
		 break;
	  case 3:
	     TIM_Cmd(TIM4, ENABLE);
		 break;
	  case 4:
	     TIM_Cmd(TIM5, ENABLE);
		 break;
	  }
  }
}

void AxisMoveRel(u32 axis, s32 step, u32 accel, u32 decel, u32 speed)//相对运动
{
  //! Number of steps before we hit max speed.
  u32 max_s_lim;
  //! Number of steps before we must start deceleration (if accel does not hit max speed).
  u32 accel_lim;
	float ftemp=0.0;

   if(step <0)
    {
			if(GPIO_ReadInputDataBit(AxisLmtNegPort[axis-1], LmtNegPin[axis-1]))
			{
				if(LmtSnsNeg[axis-1]==0)
				{
					bLmtNeg[axis-1]=FALSE;
				}
				else
				{
					bLmtNeg[axis-1]=TRUE;
					return;
				}
						
			}
			else
			{
				if(LmtSnsNeg[axis-1]==0)
				{
					bLmtNeg[axis-1]=TRUE;
					return;
				}
				else
				{
					bLmtNeg[axis-1]=FALSE;	
				}		 	
			}
      srd[axis-1].dir = CCW;
      GPIO_ResetBits(AxisDirPort[axis-1], AxisDirPin[axis-1]);
			step =-step;
    }
  else
   {
		if(GPIO_ReadInputDataBit(AxisLmtPosPort[axis-1], LmtPosPin[axis-1]))
		{
			if(LmtSnsPos[axis-1]==0)
			{
				bLmtPos[axis-1]=FALSE;
			}
			else
			{
				bLmtPos[axis-1]=TRUE;
				return;
			}					
		}
		else
		{
			if(LmtSnsPos[axis-1]==0)
			{
				bLmtPos[axis-1]=TRUE;
				return;
			}
			else
			{
				bLmtPos[axis-1]=FALSE;
			}		 	
		}
    srd[axis-1].dir = CW;
	  GPIO_SetBits(AxisDirPort[axis-1], AxisDirPin[axis-1]);
   }

  if(step == 1)
  {
    // Move one step...
    srd[axis-1].accel_count = -1;
    // ...in DECEL state.
    srd[axis-1].run_state = DECEL;
    // Just a short delay so main() can act on 'running'.
    srd[axis-1].step_delay = 1000;
		switch(axis)
			{
			case 1:
				 TIM2->CCR1=10;
				 TIM2->ARR=10;
				 break;
			case 2:
				 TIM3->CCR1=10;
				 TIM3->ARR=10;
				 break;
			case 3:
				 TIM4->CCR1=10;
				 TIM4->ARR=10;
				 break;
			case 4:
				 TIM5->CCR1=10;
				 TIM5->ARR=10;
				 break;
			}
    
		MotionStatus[axis-1] = 1;
		switch(axis)
			{
			case 1:
				 TIM_Cmd(TIM2, ENABLE);
			 break;
			case 2:
			 TIM_Cmd(TIM3, ENABLE);
			 break;
			case 3:
				 TIM_Cmd(TIM4, ENABLE);
			 break;
			case 4:
				 TIM_Cmd(TIM5, ENABLE);
			 break;
			}
	
  }
  // Only move if number of steps to move is not zero.
  else if(step != 0)
  {
    // Refer to documentation for detailed information about these calculations.

    // Set max speed limit, by calc min_delay to use in timer.
    // min_delay = (alpha / tt)/ w
    srd[axis-1].min_delay = T1_FREQ/speed/2;

    // Set accelration by calc the first (c0) step delay .
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/100 )*100 * sqrt( (2*alpha*10000000000) / (accel*100) )/10000
    srd[axis-1].step_delay = ((long)T1_FREQ*0.676* sqrt(2000000 / accel))/1000/2;
    // Find out after how many steps does the speed hit the max speed limit.
    // max_s_lim = speed^2 / (2*alpha*accel)
    max_s_lim = speed*speed/(2*accel);
    // If we hit max speed limit before 0,5 step it will round to 0.
    // But in practice we need to move atleast 1 step to get any speed at all.
    if(max_s_lim == 0){
      max_s_lim = 1;
    }

    // Find out after how many steps we must start deceleration.
    // n1 = (n1+n2)decel / (accel + decel)
    if((accel+decel)>step)
		{
//			accel_lim = step*decel/(accel+decel);
			ftemp=(float)decel/(float)(accel+decel);
			accel_lim = (float)step*ftemp;
		}
		else
		{
			accel_lim = step/(accel+decel)*decel;
		}
    // We must accelrate at least 1 step before we can start deceleration.
    if(accel_lim == 0){
      accel_lim = 1;
    }

    // Use the limit we hit first to calc decel.
    if(accel_lim <= max_s_lim){
      srd[axis-1].decel_val = accel_lim - step;
    }
    else{
      srd[axis-1].decel_val =-(s32)(max_s_lim*accel/decel);
    }
    // We must decelrate at least 1 step to stop.
    if(srd[axis-1].decel_val == 0){
      srd[axis-1].decel_val = -1;
    }

    // Find step to start decleration.
    srd[axis-1].decel_start = step + srd[axis-1].decel_val;

    // If the maximum speed is so low that we dont need to go via accelration state.
    if(srd[axis-1].step_delay <= srd[axis-1].min_delay)
	{
      srd[axis-1].step_delay = srd[axis-1].min_delay;
      srd[axis-1].run_state = RUN;
    }
    else{
      srd[axis-1].run_state = ACCEL;
    }

    // Reset counter.
    srd[axis-1].accel_count = 0;
    MotionStatus[axis-1] = 1;
    //OCR1A = 10;
	switch(axis)
	  {
	  case 1:
	  	 TIM2->CCR1=10;
		 TIM2->ARR=10;
		 break;
	  case 2:
		 TIM3->CCR1=10;
		 TIM3->ARR=10;
		 break;
	  case 3:
	     TIM4->CCR1=10;
		 TIM4->ARR=10;
		 break;
	  case 4:
	     TIM5->CCR1=10;
		 TIM5->ARR=10;
		 break;
	  }
	
    // Set Timer/Counter to divide clock by 8
    //TCCR1B |= ((0<<CS12)|(1<<CS11)|(0<<CS10));
	switch(axis)
	  {
	  case 1:
	  	 TIM_Cmd(TIM2, ENABLE);
		 break;
	  case 2:
		 TIM_Cmd(TIM3, ENABLE);
		 break;
	  case 3:
	     TIM_Cmd(TIM4, ENABLE);
		 break;
	  case 4:
	     TIM_Cmd(TIM5, ENABLE);
		 break;
	  }
  }
}


void TIM2_IRQHandler(void)//1轴定时器中断处理
{ 
  // Holds next delay period.
  u16 new_step_delay;
  // Remember the last step delay used when accelrating.
  static u16 last_accel_delay;
  // Counting steps when moving.
  static u32 step_count = 0;
  // Keep track of remainder from new_step-delay calculation to incrase accurancy
  static s32 rest = 0;
  static u8 i=0;

  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
   {
    /* Clear TIM2 Capture Compare1 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
   }
  	TIM2->CCR1=srd[0].step_delay;
	TIM2->ARR=srd[0].step_delay;
 // OCR1A = srd.step_delay;	//
  if(srd[0].run_state)
    {
	  if(GPIO_ReadOutputDataBit(CONTROL_PORT, AXIS_PULSE_1)==1)
		 {
		   GPIO_ResetBits(CONTROL_PORT, AXIS_PULSE_1);
		 }
	   else
	     {
			GPIO_SetBits(CONTROL_PORT, AXIS_PULSE_1);
	     }


	}
	//PORTD^=BIT(5);		   
  i++;
  if(i==2)
 {
	 i=0;
  switch(srd[0].run_state) 
  {
    case STOP:
      step_count = 0;
      rest = 0;
			TIM_Cmd(TIM2, DISABLE);			
      MotionStatus[0] = 0;
			bEmgStopping[0]=FALSE;
      break;

    case ACCEL:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
			if(srd[0].dir==CW)
			{	  	
				position[0]++;
// 	 			if(bLmtPos[0])
// 	 			{
// 	 				srd[0].run_state = STOP;
// 	 				break;	
// 	 			}
			}
			else
			{
				position[0]--;
// 	 			if(bLmtNeg[0])
// 	 			{
// 	 				srd[0].run_state = STOP;
// 	 				break;	
// 	 			}
			}
      srd[0].accel_count++;
      new_step_delay = srd[0].step_delay - (((2 * (long)srd[0].step_delay) + rest)/(4 * srd[0].accel_count + 1));
      rest = ((2 * (long)srd[0].step_delay)+rest)%(4 * srd[0].accel_count + 1);
			if(step_count >= srd[0].decel_start  || bStopCmd[0] || (bLmtPos[0] && srd[0].dir==CW) || (bLmtNeg[0] && srd[0].dir==CCW))
			{
				if(bStopCmd[0])
				{
					bStopCmd[0]=FALSE;
					srd[0].accel_count = T1_FREQ/2/stpdecel[0]*T1_FREQ/srd[0].step_delay/srd[0].step_delay*-1;
				}
				else if((bLmtPos[0] && srd[0].dir==CW) || (bLmtNeg[0] && srd[0].dir==CCW))
				{
					srd[0].accel_count = T1_FREQ/2/stpdecel[0]*T1_FREQ/srd[0].step_delay/srd[0].step_delay*-1;
					bEmgStopping[0]=TRUE;
				}
				else
				{
					srd[0].accel_count = srd[0].decel_val;
				}
        
        srd[0].run_state = DECEL;
      }
      else if(new_step_delay <= srd[0].min_delay)
			{
        last_accel_delay = new_step_delay;
        new_step_delay = srd[0].min_delay;
        rest = 0;
        srd[0].run_state = RUN;
      }
      break;

    case RUN:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
			if(srd[0].dir==CW)
			{	  	
				position[0]++;
// 				if(bLmtPos[0])
// 				{
// 					srd[0].run_state = STOP;
// 					break;	
// 				}
			}
			else
			{
				position[0]--;
// 				if(bLmtNeg[0])
// 				{
// 					srd[0].run_state = STOP;
// 					break;	
// 				}
			}
      new_step_delay = srd[0].min_delay;
      if(step_count >= srd[0].decel_start || bStopCmd[0] || (bLmtPos[0] && srd[0].dir==CW) || (bLmtNeg[0] && srd[0].dir==CCW))
			{
				if(bStopCmd[0])
				{
					bStopCmd[0]=FALSE;
					srd[0].accel_count = T1_FREQ/2/stpdecel[0]*T1_FREQ/srd[0].step_delay/srd[0].step_delay*-1;
				}
				else if((bLmtPos[0] && srd[0].dir==CW) || (bLmtNeg[0] && srd[0].dir==CCW))
				{
					srd[0].accel_count = T1_FREQ/2/stpdecel[0]*T1_FREQ/srd[0].step_delay/srd[0].step_delay*-1;
					bEmgStopping[0]=TRUE;
				}
				else
				{
					srd[0].accel_count = srd[0].decel_val;
				}
        // Start decelration with same delay as accel ended with.
        new_step_delay = last_accel_delay;
        srd[0].run_state = DECEL;
      }
      break;

    case DECEL:
      step_count++;
			if(srd[0].dir==CW)
			{	  	
				position[0]++;
// 				if(bLmtPos[0])
// 				{
// 					srd[0].run_state = STOP;
// 					break;	
// 				}
			}
			else
			{
				position[0]--;
// 				if(bLmtNeg[0])
// 				{
// 					srd[0].run_state = STOP;
// 					break;	
// 				}
			}
      srd[0].accel_count++;
      new_step_delay = srd[0].step_delay - (((2 * (long)srd[0].step_delay) + rest)/(4 * srd[0].accel_count + 1));
      rest = ((2 * (long)srd[0].step_delay)+rest)%(4 * srd[0].accel_count + 1);
       if(bStopCmd[0])
			{
				bStopCmd[0]=FALSE;
				srd[0].accel_count = T1_FREQ/2/stpdecel[0]*T1_FREQ/srd[0].step_delay/srd[0].step_delay*-1;
			}
			else if(!bEmgStopping[0] && ((bLmtPos[0] && srd[0].dir==CW) || (bLmtNeg[0] && srd[0].dir==CCW)))
			{
				srd[0].accel_count = T1_FREQ/2/stpdecel[0]*T1_FREQ/srd[0].step_delay/srd[0].step_delay*-1;
				bEmgStopping[0]=TRUE;
			}
			if(srd[0].accel_count >= 0)
			{
        srd[0].run_state = STOP;
      }
      break;
   }
  srd[0].step_delay = new_step_delay;
  
  }
}



void TIM3_IRQHandler(void)//2轴定时器中断处理
{ 
  // Holds next delay period.
  u16 new_step_delay;
  // Remember the last step delay used when accelrating.
  static u16 last_accel_delay;
  // Counting steps when moving.
  static u32 step_count = 0;
  // Keep track of remainder from new_step-delay calculation to incrase accurancy
  static s32 rest = 0;
	
  static u8 i=0;

  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
   {
    /* Clear TIM2 Capture Compare1 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
   }
  	TIM3->CCR1=srd[1].step_delay;
	TIM3->ARR=srd[1].step_delay;
 // OCR1A = srd.step_delay;	//
  if(srd[1].run_state)
    {
	  if(GPIO_ReadOutputDataBit(CONTROL_PORT, AXIS_PULSE_2)==1)
		 {
		   GPIO_ResetBits(CONTROL_PORT, AXIS_PULSE_2);
		 }
	   else
	     {
			GPIO_SetBits(CONTROL_PORT, AXIS_PULSE_2);
	     }


	}
		   
  i++;
  if(i==2)
 {
	 i=0;
  switch(srd[1].run_state) 
  {
    case STOP:
      step_count = 0;
      rest = 0;
			TIM_Cmd(TIM3, DISABLE);
      MotionStatus[1] = 0;
			bEmgStopping[1]=FALSE;
      break;

    case ACCEL:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
			if(srd[1].dir==CW)
			{	  	
				position[1]++;
// 				if(bLmtPos[1])
// 				{
// 					srd[1].run_state = STOP;
// 					break;	
// 				}
			}
			else
			{
				position[1]--;
// 				if(bLmtNeg[1])
// 				{
// 					srd[1].run_state = STOP;
// 					break;	
// 				}
			}
      srd[1].accel_count++;
      new_step_delay = srd[1].step_delay - (((2 * (long)srd[1].step_delay) + rest)/(4 * srd[1].accel_count + 1));
      rest = ((2 * (long)srd[1].step_delay)+rest)%(4 * srd[1].accel_count + 1);
			if(step_count >= srd[1].decel_start  || bStopCmd[1] || (bLmtPos[1] && srd[1].dir==CW) || (bLmtNeg[1] && srd[1].dir==CCW))
			{
				if(bStopCmd[1])
				{
					bStopCmd[1]=FALSE;
					srd[1].accel_count = T1_FREQ/2/stpdecel[1]*T1_FREQ/srd[1].step_delay/srd[1].step_delay*-1;
				}
				else if((bLmtPos[1] && srd[1].dir==CW) || (bLmtNeg[1] && srd[1].dir==CCW))
				{
					srd[1].accel_count = T1_FREQ/2/stpdecel[1]*T1_FREQ/srd[1].step_delay/srd[1].step_delay*-1;
					bEmgStopping[1]=TRUE;
				}
				else
				{
					srd[1].accel_count = srd[1].decel_val;
				}
        
        srd[1].run_state = DECEL;
      }
      else if(new_step_delay <= srd[1].min_delay)
			{
        last_accel_delay = new_step_delay;
        new_step_delay = srd[1].min_delay;
        rest = 0;
        srd[1].run_state = RUN;
      }
      break;

    case RUN:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
			if(srd[1].dir==CW)
			{	  	
				position[1]++;
// 			if(bLmtPos[1])
// 			{
// 				srd[1].run_state = STOP;
// 				break;	
// 			}
			}
			else
			{
				position[1]--;
// 			if(bLmtNeg[1])
// 			{
// 				srd[1].run_state = STOP;
// 				break;	
// 			}
			}
      new_step_delay = srd[1].min_delay;
      // Chech if we should start decelration.
      if(step_count >= srd[1].decel_start || bStopCmd[1] || (bLmtPos[1] && srd[1].dir==CW) || (bLmtNeg[1] && srd[1].dir==CCW))
			{
				if(bStopCmd[1])
				{
					bStopCmd[1]=FALSE;
					srd[1].accel_count = T1_FREQ/2/stpdecel[1]*T1_FREQ/srd[1].step_delay/srd[1].step_delay*-1;
				}
				else if((bLmtPos[1] && srd[1].dir==CW) || (bLmtNeg[1] && srd[1].dir==CCW))
				{
					srd[1].accel_count = T1_FREQ/2/stpdecel[1]*T1_FREQ/srd[1].step_delay/srd[1].step_delay*-1;
					bEmgStopping[1]=TRUE;
				}
				else
				{
					srd[1].accel_count = srd[1].decel_val;
				}
        // Start decelration with same delay as accel ended with.
        new_step_delay = last_accel_delay;
        srd[1].run_state = DECEL;
      }
      break;

    case DECEL:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
			if(srd[1].dir==CW)
			{	  	
				position[1]++;
// 				if(bLmtPos[1])
// 				{
// 					srd[1].run_state = STOP;
// 					break;	
// 				}
			}
			else
			{
				position[1]--;
// 				if(bLmtNeg[1])
// 				{
// 					srd[1].run_state = STOP;
// 					break;	
// 				}
			}
      srd[1].accel_count++;
      new_step_delay = srd[1].step_delay - (((2 * (long)srd[1].step_delay) + rest)/(4 * srd[1].accel_count + 1));
      rest = ((2 * (long)srd[1].step_delay)+rest)%(4 * srd[1].accel_count + 1);
      if(bStopCmd[1])
			{
				bStopCmd[1]=FALSE;
				srd[1].accel_count = T1_FREQ/2/stpdecel[1]*T1_FREQ/srd[1].step_delay/srd[1].step_delay*-1;
			}
			else if(!bEmgStopping[1] && ((bLmtPos[1] && srd[1].dir==CW) || (bLmtNeg[1] && srd[1].dir==CCW)))
			{
				srd[1].accel_count = T1_FREQ/2/stpdecel[1]*T1_FREQ/srd[1].step_delay/srd[1].step_delay*-1;
				bEmgStopping[1]=TRUE;
			}
			if(srd[1].accel_count >= 0)
			{
        srd[1].run_state = STOP;
      }
      break;
   }
  srd[1].step_delay = new_step_delay;
  
  }
}


void TIM4_IRQHandler(void)//3轴定时器中断处理
{ 
  // Holds next delay period.
  u16 new_step_delay;
  // Remember the last step delay used when accelrating.
  static u16 last_accel_delay;
  // Counting steps when moving.
  static u32 step_count = 0;
  // Keep track of remainder from new_step-delay calculation to incrase accurancy
  static s32 rest = 0;
  static u8 i=0;

  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
   {
    /* Clear TIM2 Capture Compare1 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
   }
  TIM4->CCR1=srd[2].step_delay;
	TIM4->ARR=srd[2].step_delay;
 // OCR1A = srd.step_delay;	//
  if(srd[2].run_state)
    {
	  if(GPIO_ReadOutputDataBit(CONTROL_PORT, AXIS_PULSE_3)==1)
		 {
		   GPIO_ResetBits(CONTROL_PORT, AXIS_PULSE_3);
		 }
	   else
	   {
			GPIO_SetBits(CONTROL_PORT, AXIS_PULSE_3);
	   }
	}
	   
  i++;
 if(i==2)
 {
	 i=0;
  switch(srd[2].run_state) 
  {
    case STOP:
      step_count = 0;
      rest = 0;
			TIM_Cmd(TIM4, DISABLE);
      MotionStatus[2] = 0;
			bEmgStopping[2]=FALSE;
      break;

    case ACCEL:

      step_count++;
			if(srd[2].dir==CW)
			{	  	
				position[2]++;
	// 			if(bLmtPos[2])
	// 			{
	// 				srd[2].run_state = STOP;
	// 				break;	
	// 			}
			}
			else
			{
				position[2]--;
	// 			if(bLmtNeg[2])
	// 			{
	// 				srd[2].run_state = STOP;
	// 				break;	
	// 			}
			}
			srd[2].accel_count++;
			new_step_delay = srd[2].step_delay - (((2 * (long)srd[2].step_delay) + rest)/(4 * srd[2].accel_count + 1));
			rest = ((2 * (long)srd[2].step_delay)+rest)%(4 * srd[2].accel_count + 1);
			if(step_count >= srd[2].decel_start  || bStopCmd[2] || (bLmtPos[2] && srd[2].dir==CW) || (bLmtNeg[2] && srd[2].dir==CCW))
			{
				if(bStopCmd[2])
				{
					bStopCmd[2]=FALSE;
					srd[2].accel_count = T1_FREQ/2/stpdecel[2]*T1_FREQ/srd[2].step_delay/srd[2].step_delay*-1;
				}
				else if((bLmtPos[2] && srd[2].dir==CW) || (bLmtNeg[2] && srd[2].dir==CCW))
				{
					srd[2].accel_count = T1_FREQ/2/stpdecel[2]*T1_FREQ/srd[2].step_delay/srd[2].step_delay*-1;
					bEmgStopping[2]=TRUE;
				}
				else
				{
					srd[2].accel_count = srd[2].decel_val;
				}
        
        srd[2].run_state = DECEL;
      }
      else if(new_step_delay <= srd[2].min_delay)
			{
        last_accel_delay = new_step_delay;
        new_step_delay = srd[2].min_delay;
        rest = 0;
        srd[2].run_state = RUN;
      }
      break;

    case RUN:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
			if(srd[2].dir==CW)
			{	  	
				position[2]++;
// 				if(bLmtPos[2])
// 				{
// 					srd[2].run_state = STOP;
// 					break;	
// 				}
			}
			else
			{
				position[2]--;
// 				if(bLmtNeg[2])
// 				{
// 					srd[2].run_state = STOP;
// 					break;	
// 				}
			}
      new_step_delay = srd[2].min_delay;
      // Chech if we should start decelration.
      if(step_count >= srd[2].decel_start || bStopCmd[2] || (bLmtPos[2] && srd[2].dir==CW) || (bLmtNeg[2] && srd[2].dir==CCW))
			{
				if(bStopCmd[2])
				{
					bStopCmd[2]=FALSE;
					srd[2].accel_count = T1_FREQ/2/stpdecel[2]*T1_FREQ/srd[2].step_delay/srd[2].step_delay*-1;
				}
				else if((bLmtPos[2] && srd[2].dir==CW) || (bLmtNeg[2] && srd[2].dir==CCW))
				{
					srd[2].accel_count = T1_FREQ/2/stpdecel[2]*T1_FREQ/srd[2].step_delay/srd[2].step_delay*-1;
					bEmgStopping[2]=TRUE;
				}
				else
				{
					srd[2].accel_count = srd[2].decel_val;
				}
        // Start decelration with same delay as accel ended with.
        new_step_delay = last_accel_delay;
        srd[2].run_state = DECEL;
      }
      break;

    case DECEL:
      step_count++;
			if(srd[2].dir==CW)
			{	  	
				position[2]++;
// 				if(bLmtPos[2])
// 				{
// 					srd[2].run_state = STOP;
// 					break;	
// 				}
			}
			else
			{
				position[2]--;
// 				if(bLmtNeg[2])
// 				{
// 					srd[2].run_state = STOP;
// 					break;	
// 				}
			}
      srd[2].accel_count++;
      new_step_delay = srd[2].step_delay - (2*srd[2].step_delay+rest)/(4*srd[2].accel_count+1);
      rest = (2 * srd[2].step_delay+rest)%(4 * srd[2].accel_count + 1);
      if(bStopCmd[2])
			{
				bStopCmd[2]=FALSE;
				srd[2].accel_count = T1_FREQ/2/stpdecel[2]*T1_FREQ/srd[2].step_delay/srd[2].step_delay*-1;
			}
			else if(!bEmgStopping[2] && ((bLmtPos[2] && srd[2].dir==CW) || (bLmtNeg[2] && srd[2].dir==CCW)))
			{
				srd[2].accel_count = T1_FREQ/2/stpdecel[2]*T1_FREQ/srd[2].step_delay/srd[2].step_delay*-1;
				bEmgStopping[2]=TRUE;
			}
			if(srd[2].accel_count >= 0)
			{
        srd[2].run_state = STOP;
      }
      break;
   }
		srd[2].step_delay = new_step_delay;
  
  }
}



void TIM5_IRQHandler(void)//4轴定时器中断处理
{ 

  u16 new_step_delay;

  static u16 last_accel_delay;
  // Counting steps when moving.
  static u32 step_count = 0;

  static s32 rest = 0;
  static u8 i=0;

  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
   {

    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
   }
  	TIM5->CCR1=srd[3].step_delay;
		TIM5->ARR=srd[3].step_delay;

  if(srd[3].run_state)
    {
	  if(GPIO_ReadOutputDataBit(CONTROL_PORT, AXIS_PULSE_4)==1)
		 {
		   GPIO_ResetBits(CONTROL_PORT, AXIS_PULSE_4);
		 }
	   else
	     {
			GPIO_SetBits(CONTROL_PORT, AXIS_PULSE_4);
	     }


	}
	//PORTD^=BIT(5);		   
  i++;
  if(i==2)
 {i=0;
  switch(srd[3].run_state) 
  {
    case STOP:
      step_count = 0;
      rest = 0;
			TIM_Cmd(TIM5, DISABLE);
      MotionStatus[3] = 0;
			bEmgStopping[3]=FALSE;
      break;

    case ACCEL:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
			if(srd[3].dir==CW)
			{	  	
				position[3]++;
// 				if(bLmtPos[3])
// 				{
// 					srd[3].run_state = STOP;
// 					break;	
// 				}
			}
			else
			{
				position[3]--;
// 				if(bLmtNeg[3])
// 				{
// 					srd[3].run_state = STOP;
// 					break;	
// 				}
			}
      srd[3].accel_count++;
      new_step_delay = srd[3].step_delay - (((2 * (long)srd[3].step_delay) + rest)/(4 * srd[3].accel_count + 1));
      rest = ((2 * (long)srd[3].step_delay)+rest)%(4 * srd[3].accel_count + 1);
			if(step_count >= srd[3].decel_start  || bStopCmd[3] || (bLmtPos[3] && srd[3].dir==CW) || (bLmtNeg[3] && srd[3].dir==CCW))
			{
				if(bStopCmd[3])
				{
					bStopCmd[3]=FALSE;
					srd[3].accel_count = T1_FREQ/2/stpdecel[3]*T1_FREQ/srd[3].step_delay/srd[3].step_delay*-1;
				}
				else if((bLmtPos[3] && srd[3].dir==CW) || (bLmtNeg[3] && srd[3].dir==CCW))
				{
					srd[3].accel_count = T1_FREQ/2/stpdecel[3]*T1_FREQ/srd[3].step_delay/srd[3].step_delay*-1;
					bEmgStopping[3]=TRUE;
				}
				else
				{
					srd[3].accel_count = srd[3].decel_val;
				}
        
        srd[3].run_state = DECEL;
      }
      else if(new_step_delay <= srd[3].min_delay)
			{
        last_accel_delay = new_step_delay;
        new_step_delay = srd[3].min_delay;
        rest = 0;
        srd[3].run_state = RUN;
      }
      break;

    case RUN:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
			if(srd[3].dir==CW)
			{	  	
				position[3]++;
// 				if(bLmtPos[3])
// 				{
// 					srd[3].run_state = STOP;
// 					break;	
// 				}
			}
			else
			{
				position[3]--;
// 				if(bLmtNeg[3])
// 				{
// 					srd[3].run_state = STOP;
// 					break;	
// 				}
			}
      new_step_delay = srd[3].min_delay;
      if(step_count >= srd[3].decel_start || bStopCmd[3] || (bLmtPos[3] && srd[3].dir==CW) || (bLmtNeg[3] && srd[3].dir==CCW))
			{
				if(bStopCmd[3])
				{
					bStopCmd[3]=FALSE;
					srd[3].accel_count = T1_FREQ/2/stpdecel[3]*T1_FREQ/srd[3].step_delay/srd[3].step_delay*-1;
				}
				else if((bLmtPos[3] && srd[3].dir==CW) || (bLmtNeg[3] && srd[3].dir==CCW))
				{
					srd[3].accel_count = T1_FREQ/2/stpdecel[3]*T1_FREQ/srd[3].step_delay/srd[3].step_delay*-1;
					bEmgStopping[3]=TRUE;
				}
				else
				{
					srd[3].accel_count = srd[3].decel_val;
				}
        // Start decelration with same delay as accel ended with.
        new_step_delay = last_accel_delay;
        srd[3].run_state = DECEL;
      }
      break;

    case DECEL:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
			if(srd[3].dir==CW)
			{	  	
				position[3]++;
// 				if(bLmtPos[3])
// 				{
// 					srd[3].run_state = STOP;
// 					break;	
// 				}
			}
			else
			{
				position[3]--;
// 				if(bLmtNeg[3])
// 				{
// 					srd[3].run_state = STOP;
// 					break;	
// 				}
			}
      srd[3].accel_count++;
      new_step_delay = srd[3].step_delay - (((2 * (long)srd[3].step_delay) + rest)/(4 * srd[3].accel_count + 1));
      rest = ((2 * (long)srd[3].step_delay)+rest)%(4 * srd[3].accel_count + 1);
      if(bStopCmd[3])
			{
				bStopCmd[3]=FALSE;
				srd[3].accel_count = T1_FREQ/2/stpdecel[3]*T1_FREQ/srd[3].step_delay/srd[3].step_delay*-1;
			}
			else if(!bEmgStopping[3] && ((bLmtPos[3] && srd[3].dir==CW) || (bLmtNeg[3] && srd[3].dir==CCW)))
			{
				srd[3].accel_count = T1_FREQ/2/stpdecel[3]*T1_FREQ/srd[3].step_delay/srd[3].step_delay*-1;
				bEmgStopping[3]=TRUE;
			}
			if(srd[3].accel_count >= 0)
			{
        srd[3].run_state = STOP;
      }
      break;
   }
  srd[3].step_delay = new_step_delay;
  
  }
}

void EXTI0_IRQHandler(void)//1轴回零开关触发，记录触发时的位置值
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    HomePos[0]=position[0];
		bZeroCapture[0]=TRUE;
		if(MotionStatus[0])
		{
			bStopCmd[0]=TRUE;
		}
    EXTI_ClearITPendingBit(EXTI_Line0);
		DisableHomeCapture1;
  }
}

void EXTI1_IRQHandler(void)//2轴回零开关触发，记录触发时的位置值
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
    HomePos[1]=position[1];
		bZeroCapture[1]=TRUE;
		if(MotionStatus[1])
		{
			bStopCmd[1]=TRUE;
		}
    EXTI_ClearITPendingBit(EXTI_Line1);
		DisableHomeCapture2;
  }
}

void EXTI2_IRQHandler(void)//3轴回零开关触发，记录触发时的位置值
{
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {
    HomePos[2]=position[2];
		bZeroCapture[2]=TRUE;
		if(MotionStatus[2])
		{
			bStopCmd[2]=TRUE;
		}
    EXTI_ClearITPendingBit(EXTI_Line2);
		DisableHomeCapture3;
  }
}

void EXTI3_IRQHandler(void)//4轴回零开关触发，记录触发时的位置值
{
  if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
    HomePos[3]=position[3];
		bZeroCapture[3]=TRUE;
		if(MotionStatus[3])
		{
			bStopCmd[3]=TRUE;
		}
    EXTI_ClearITPendingBit(EXTI_Line3);
		DisableHomeCapture4;
  }
}

u32 sqrt(u32 x)//开方
{
  register u32 xr;  // result register
  register u32 q2;  // scan-bit register
  register u8 f;   // flag (one bit)

  xr = 0;                     // clear result
  q2 = 0x40000000L;           // higest possible result bit
  do
  {
    if((xr + q2) <= x)
    {
      x -= xr + q2;
      f = 1;                  // set flag
    }
    else{
      f = 0;                  // clear flag
    }
    xr >>= 1;
    if(f){
      xr += q2;               // test flag
    }
  } while(q2 >>= 2);          // shift twice
  if(xr < x){
    return xr +1;             // add for rounding
  }
  else{
    return xr;
	  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
