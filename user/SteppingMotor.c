#include "stm32f10x.h"
#include "SteppingMotor.h"

static uint32_t KeyAndJostickValue;	

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void KyeAndJoystickScan(void)
{
	/*
	if(!GPIO_ReadInputDataBit( IO_KEY_ARRAY, IO_KEY1))KeyAndJostickValue = KEY1_VALUE;
	if(!GPIO_ReadInputDataBit( IO_KEY_ARRAY, IO_KEY2))KeyAndJostickValue = KEY2_VALUE;
	if(!GPIO_ReadInputDataBit( IO_KEY_ARRAY, IO_KEY3))KeyAndJostickValue = KEY3_VALUE;
	*/
	
	if(!GPIO_ReadInputDataBit( IO_JOYSTICK_ARRAY, IO_JOYSTICK_A))KeyAndJostickValue = JOYSTICK_A_VALUE;
	if(!GPIO_ReadInputDataBit( IO_JOYSTICK_ARRAY, IO_JOYSTICK_B))KeyAndJostickValue = JOYSTICK_B_VALUE;
	if(!GPIO_ReadInputDataBit( IO_JOYSTICK_ARRAY, IO_JOYSTICK_C))KeyAndJostickValue = JOYSTICK_C_VALUE;
	if(!GPIO_ReadInputDataBit( IO_JOYSTICK_ARRAY, IO_JOYSTICK_D))KeyAndJostickValue = JOYSTICK_D_VALUE;
	if(!GPIO_ReadInputDataBit( IO_JOYSTICK_ARRAY, IO_JOYSTICK_P))KeyAndJostickValue = JOYSTICK_P_VALUE;
}
void JoystickGpioConfiguration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_IO_JOYSTICK, ENABLE);

  GPIO_InitStructure.GPIO_Pin = IO_JOYSTICK_A | IO_JOYSTICK_B | IO_JOYSTICK_C | IO_JOYSTICK_D | IO_JOYSTICK_P;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init( IO_JOYSTICK_ARRAY, &GPIO_InitStructure);
}

void SteppingMotor_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = IO_PWM_CH2 | IO_PWM_CH3 | IO_PWM_CH4; //TIM2的CH2,CH3,CH4输出PWM
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//设置GPIO推挽复用模式
		//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//设置GPIO开漏复用模式需要接上拉电阻
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IO_PWM_ARRAY,&GPIO_InitStructure);

	/*
	GPIO_Mode_AIN = 0x0,     //模拟输入
  GPIO_Mode_IN_FLOATING = 0x04, //悬空输入
  GPIO_Mode_IPD = 0x28,    //下拉输入
  GPIO_Mode_IPU = 0x48,    //上拉输入
  GPIO_Mode_Out_OD = 0x14, //开漏输出
  GPIO_Mode_Out_PP = 0x10,  //推挽输出
  GPIO_Mode_AF_OD = 0x1C,   //开漏复用
  GPIO_Mode_AF_PP = 0x18    //推挽复用
	*/

    GPIO_InitStructure.GPIO_Pin = IO_DirectionX | IO_DirectionY | IO_DirectionZ; //控制步进电机正反转
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IO_Direction_ARRAY,&GPIO_InitStructure);
		GPIO_ResetBits(IO_Direction_ARRAY,IO_DirectionX);
		GPIO_ResetBits(IO_Direction_ARRAY,IO_DirectionY);
		GPIO_ResetBits(IO_Direction_ARRAY,IO_DirectionZ);

    //定义PWM频率
    TIM_TimeBaseStructure.TIM_Prescaler =719;                    //预分频值
    TIM_TimeBaseStructure.TIM_Period =99;                   //重装值，这里pwm输出频率1KHZ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//设置TIM的驱动时钟源不分频，为72MHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);          //时基单元配置
                                                                      
    //设定占空比
    TIM_OCStructInit(& TIM_OCInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //允许输出PWM
    TIM_OCInitStructure.TIM_Pulse =25;                      //占空比50%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;    //设置高电平为PWM有效输出电平
		
		//TIM_ARRPreloadConfig(TIM2,ENABLE);//使能TIMx 在ARR预装载寄存器
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);	
		TIM_OC3Init(TIM2, &TIM_OCInitStructure);	
		TIM_OC4Init(TIM2, &TIM_OCInitStructure);	
    //TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);//CH2预装载使能
		

    //TIM_CtrlPWMOutputs(TIM2, ENABLE); 
    //TIM_Cmd(TIM2, ENABLE);	
		
}

void Run(void)
{
		if(KeyAndJostickValue == JOYSTICK_A_VALUE)
    {
        GPIO_ResetBits(IO_Direction_ARRAY,IO_DirectionX);     //反转			
    }
    else if(KeyAndJostickValue == JOYSTICK_D_VALUE)
    {
        GPIO_SetBits(IO_Direction_ARRAY,IO_DirectionX);         //正传
    }
		else if(KeyAndJostickValue == JOYSTICK_B_VALUE)
    {
        TIM_Cmd(TIM2, ENABLE); 
    }
		else if(KeyAndJostickValue == JOYSTICK_C_VALUE)
    {
        TIM_Cmd(TIM2, DISABLE); 
    }
		else
		{
			;
		}
}






/*
void EnableSteppingMotor(void)
{
    GPIO_ResetBits(GPIOA,GPIO_Pin_2);
}

void DisableSteppingMotor(void)
{
    GPIO_SetBits(GPIOA,GPIO_Pin_2);
}
*/
