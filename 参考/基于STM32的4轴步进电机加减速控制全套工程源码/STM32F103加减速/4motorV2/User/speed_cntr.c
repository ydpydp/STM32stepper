/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Linear speed ramp controller.
 *
 * Stepper motor driver, increment/decrement the position and outputs the
 * correct signals to stepper motor.
 *
 * - File:               speed_cntr.c
 * - Compiler:           IAR EWAAVR 4.11A
 * - Supported devices:  All devices with a 16 bit timer can be used.
 *                       The example is written for ATmega48
 * - AppNote:            AVR446 - Linear speed control of stepper motor
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support email: avr@atmel.com
 *
 * $Name: RELEASE_1_0 $
 * $Revision: 1.2 $
 * $RCSfile: speed_cntr.c,v $
 * $Date: 2006/05/08 12:25:58 $
 *****************************************************************************/
#include "stm32f10x.h"
#include "speed_cntr.h"
#include "stm32f10x_conf.h"

u8 i;   //因为是在每次中断里取反，所以必须等到第二次取反后才去计算下一步的定时器比较值；
speedRampData srd[4];
extern uint8_t RxBuffer[6];
extern uint8_t RxCounter;
extern bool bDataOK;
extern s32 position;
extern bool bLmtPos;
extern bool bLmtNeg; 
extern bool bStopCmd;
/*! \brief Move the stepper motor a given number of steps.
 *
 *  Makes the stepper motor move the given number of steps.
 *  It accelrate with given accelration up to maximum speed and decelerate
 *  with given deceleration so it stops at the given step.
 *  If accel/decel is to small and steps to move is to few, speed might not
 *  reach the max speed limit before deceleration starts.
 *
 *  \param step  Number of steps to move (pos - CW, neg - CCW).
 *  \param accel  Accelration to use, in 0.01*rad/sec^2.
 *  \param decel  Decelration to use, in 0.01*rad/sec^2.
 *  \param speed  Max speed, in 0.01*rad/sec.
 */
 /*
 1、首先根据程序的步数判断运动方向，然后输出控制方向的引脚电平； 
2、根据希望的速度值计算出最小步进时间间隔： speed_para.min_delay = A_T_x100 / speed; (结果为16位定时器的比较匹配值) 
3、根据加速度计算初始步延时：speed_para.step_delay = (T1_FREQ_148 * sqrt(A_SQ / accel))/100; 
4、计算出加速到希望速度所需要的步数accel_lim = ((long)step*decel) / (accel+decel);； 
5、再根据减速度计算出必须开始减速时刻的步数；para_speed.decel_val = -((long)max_s_lim*accel)/decel; 
6、比较第4、5步骤计算的大小关系判断运动曲线是梯形（到达希望速度值匀速运行一段时间再开始减速）还是三角形（加速未完成就必须开始减速）； 
程序中的速度单位0.01*rad/s,加速度、减速度单位是00.1rad/s^2。因此程序中速度speed=1000时实际上是1000*0.01rad/s，=10rad/s；换算为转/分=10/（2π）*60=95.5r/min。 
参数范围：加减速为71~32000；速度的话一般步进电机空载9000就丢步了，所以最好是12~6000； 

 */
void speed_cntr_Move(unsigned int axis, signed int step, unsigned int accel, unsigned int decel, unsigned int speed)
{
  //! Number of steps before we hit max speed.
  u32 max_s_lim;
  //! Number of steps before we must start deceleration (if accel does not hit max speed).
  u32 accel_lim;

  // Set direction from sign on step value.
   if(step <0)
    {
//		if(GPIO_ReadInputDataBit(GPIOC, MOTOR_LMTNEG))
//		{
//			bLmtNeg=FALSE;		
//		}
//		else
//		{
//		 	bLmtNeg=TRUE;
//			return;
//		}
      srd[axis-1].dir = CCW;
	  switch(axis)
	  {
	  case 1:
	  	 GPIO_ResetBits(GPIOA, AXIS_DIR_1);
		 break;
	  case 2:
		 GPIO_ResetBits(GPIOA, AXIS_DIR_2);
		 break;
	  case 3:
	     GPIO_ResetBits(GPIOA, AXIS_DIR_3);
		 break;
	  case 4:
	     GPIO_ResetBits(GPIOA, AXIS_DIR_4);
		 break;
	  }
//      GPIO_ResetBits(MOTOR_COTR_PORT, MOTOR_COTR_DIR);
	  step =-step;
    }
  else
   {
//		if(GPIO_ReadInputDataBit(GPIOC, MOTOR_LMTPOS))
//		{
//			bLmtPos=FALSE;		
//		}
//		else
//		{
//		 	bLmtPos=TRUE;
//			return;
//		}
      srd[axis-1].dir = CW;
//	  GPIO_SetBits(MOTOR_COTR_PORT, MOTOR_COTR_DIR);
	  switch(axis)
	  {
	  case 1:
	  	 GPIO_SetBits(GPIOA, AXIS_DIR_1);
		 break;
	  case 2:
		 GPIO_SetBits(GPIOA, AXIS_DIR_2);
		 break;
	  case 3:
	     GPIO_SetBits(GPIOA, AXIS_DIR_3);
		 break;
	  case 4:
	     GPIO_SetBits(GPIOA, AXIS_DIR_4);
		 break;
	  }
	   
   }
  // If moving only 1 step.
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
    
	status = TRUE;
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
    accel_lim = step/(accel+decel)*decel;
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
    status = TRUE;
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

void TIM4_IRQHandler(void)
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
	//PORTD^=BIT(5);		   
  i++;
  if(i==2)
 {i=0;
  switch(srd[2].run_state) 
  {
    case STOP:
      step_count = 0;
      rest = 0;
      // Stop Timer/Counter 1.
	  TIM_Cmd(TIM4, DISABLE);
      //TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));
      status = FALSE;
      break;

    case ACCEL:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
	  if(srd[2].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[2].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[2].run_state = STOP;
			break;	
		}
	  }
      srd[2].accel_count++;
      new_step_delay = srd[2].step_delay - (((2 * (long)srd[2].step_delay) + rest)/(4 * srd[2].accel_count + 1));
      rest = ((2 * (long)srd[2].step_delay)+rest)%(4 * srd[2].accel_count + 1);
      // Chech if we should start decelration.
      if(step_count >= srd[2].decel_start  || bStopCmd)
	  {
	  	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[2].accel_count = srd[2].decel_val;
        srd[2].run_state = DECEL;
      }
      // Chech if we hitted max speed.
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
	  	position++;
		if(bLmtPos)
		{
			srd[2].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[2].run_state = STOP;
			break;	
		}
	  }
      new_step_delay = srd[2].min_delay;
      // Chech if we should start decelration.
      if(step_count >= srd[2].decel_start || bStopCmd)
	   {
	   	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[2].accel_count = srd[2].decel_val;
        // Start decelration with same delay as accel ended with.
        new_step_delay = last_accel_delay;
        srd[2].run_state = DECEL;
      }
      break;

    case DECEL:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
	  if(srd[2].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[2].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[2].run_state = STOP;
			break;	
		}
	  }
      srd[2].accel_count++;
      new_step_delay = srd[2].step_delay - (2*srd[2].step_delay+rest)/(4*srd[2].accel_count+1);
      rest = (2 * srd[2].step_delay+rest)%(4 * srd[2].accel_count + 1);
      // Check if we at last step
      if(srd[2].accel_count >= 0)
	  {
        srd[2].run_state = STOP;
      }
	  if(bStopCmd)
	  {
	  	bStopCmd=FALSE;
	  }
      break;
   }
  srd[2].step_delay = new_step_delay;
  
  }
}



 void TIM3_IRQHandler(void)
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
	//PORTD^=BIT(5);		   
  i++;
  if(i==2)
 {i=0;
  switch(srd[1].run_state) 
  {
    case STOP:
      step_count = 0;
      rest = 0;
      // Stop Timer/Counter 1.
	  TIM_Cmd(TIM3, DISABLE);
      //TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));
      status = FALSE;
      break;

    case ACCEL:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
	  if(srd[1].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[1].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[1].run_state = STOP;
			break;	
		}
	  }
      srd[1].accel_count++;
      new_step_delay = srd[1].step_delay - (((2 * (long)srd[1].step_delay) + rest)/(4 * srd[1].accel_count + 1));
      rest = ((2 * (long)srd[1].step_delay)+rest)%(4 * srd[1].accel_count + 1);
      // Chech if we should start decelration.
      if(step_count >= srd[1].decel_start  || bStopCmd)
	  {
	  	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[1].accel_count = srd[1].decel_val;
        srd[1].run_state = DECEL;
      }
      // Chech if we hitted max speed.
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
	  	position++;
		if(bLmtPos)
		{
			srd[1].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[1].run_state = STOP;
			break;	
		}
	  }
      new_step_delay = srd[1].min_delay;
      // Chech if we should start decelration.
      if(step_count >= srd[1].decel_start || bStopCmd)
	   {
	   	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[1].accel_count = srd[1].decel_val;
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
	  	position++;
		if(bLmtPos)
		{
			srd[1].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[1].run_state = STOP;
			break;	
		}
	  }
      srd[1].accel_count++;
      new_step_delay = srd[1].step_delay - (((2 * (long)srd[1].step_delay) + rest)/(4 * srd[1].accel_count + 1));
      rest = ((2 * (long)srd[1].step_delay)+rest)%(4 * srd[1].accel_count + 1);
      // Check if we at last step
      if(srd[1].accel_count >= 0)
	  {
        srd[1].run_state = STOP;
      }
	  if(bStopCmd)
	  {
	  	bStopCmd=FALSE;
	  }
      break;
   }
  srd[1].step_delay = new_step_delay;
  
  }
}

 void TIM2_IRQHandler(void)
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
 {i=0;
  switch(srd[0].run_state) 
  {
    case STOP:
      step_count = 0;
      rest = 0;
      // Stop Timer/Counter 1.
	  TIM_Cmd(TIM2, DISABLE);
      //TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));
      status = FALSE;
      break;

    case ACCEL:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
	  if(srd[0].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[0].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[0].run_state = STOP;
			break;	
		}
	  }
      srd[0].accel_count++;
      new_step_delay = srd[0].step_delay - (((2 * (long)srd[0].step_delay) + rest)/(4 * srd[0].accel_count + 1));
      rest = ((2 * (long)srd[0].step_delay)+rest)%(4 * srd[0].accel_count + 1);
      // Chech if we should start decelration.
      if(step_count >= srd[0].decel_start  || bStopCmd)
	  {
	  	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[0].accel_count = srd[0].decel_val;
        srd[0].run_state = DECEL;
      }
      // Chech if we hitted max speed.
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
	  	position++;
		if(bLmtPos)
		{
			srd[0].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[0].run_state = STOP;
			break;	
		}
	  }
      new_step_delay = srd[0].min_delay;
      // Chech if we should start decelration.
      if(step_count >= srd[0].decel_start || bStopCmd)
	   {
	   	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[0].accel_count = srd[0].decel_val;
        // Start decelration with same delay as accel ended with.
        new_step_delay = last_accel_delay;
        srd[0].run_state = DECEL;
      }
      break;

    case DECEL:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
	  if(srd[0].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[0].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[0].run_state = STOP;
			break;	
		}
	  }
      srd[0].accel_count++;
      new_step_delay = srd[0].step_delay - (((2 * (long)srd[0].step_delay) + rest)/(4 * srd[0].accel_count + 1));
      rest = ((2 * (long)srd[0].step_delay)+rest)%(4 * srd[0].accel_count + 1);
      // Check if we at last step
      if(srd[0].accel_count >= 0)
	  {
        srd[0].run_state = STOP;
      }
	  if(bStopCmd)
	  {
	  	bStopCmd=FALSE;
	  }
      break;
   }
  srd[0].step_delay = new_step_delay;
  
  }
}



 void TIM5_IRQHandler(void)
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

  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
   {
    /* Clear TIM2 Capture Compare1 interrupt pending bit*/
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
   }
  	TIM5->CCR1=srd[3].step_delay;
	TIM5->ARR=srd[3].step_delay;
 // OCR1A = srd.step_delay;	//
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
      // Stop Timer/Counter 1.
	  TIM_Cmd(TIM5, DISABLE);
      //TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));
      status = FALSE;
      break;

    case ACCEL:
      //sm_driver_StepCounter(srd.dir);
      step_count++;
	  if(srd[3].dir==CW)
	  {	  	
	  	position++;
		if(bLmtPos)
		{
			srd[3].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[3].run_state = STOP;
			break;	
		}
	  }
      srd[3].accel_count++;
      new_step_delay = srd[3].step_delay - (((2 * (long)srd[3].step_delay) + rest)/(4 * srd[3].accel_count + 1));
      rest = ((2 * (long)srd[3].step_delay)+rest)%(4 * srd[3].accel_count + 1);
      // Chech if we should start decelration.
      if(step_count >= srd[3].decel_start  || bStopCmd)
	  {
	  	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[3].accel_count = srd[3].decel_val;
        srd[3].run_state = DECEL;
      }
      // Chech if we hitted max speed.
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
	  	position++;
		if(bLmtPos)
		{
			srd[3].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[3].run_state = STOP;
			break;	
		}
	  }
      new_step_delay = srd[3].min_delay;
      // Chech if we should start decelration.
      if(step_count >= srd[3].decel_start || bStopCmd)
	   {
	   	if(bStopCmd)
		{
			bStopCmd=FALSE;
		}
        srd[3].accel_count = srd[3].decel_val;
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
	  	position++;
		if(bLmtPos)
		{
			srd[3].run_state = STOP;
			break;	
		}
	  }
	  else
	  {
	  	position--;
		if(bLmtNeg)
		{
			srd[3].run_state = STOP;
			break;	
		}
	  }
      srd[3].accel_count++;
      new_step_delay = srd[3].step_delay - (((2 * (long)srd[3].step_delay) + rest)/(4 * srd[3].accel_count + 1));
      rest = ((2 * (long)srd[3].step_delay)+rest)%(4 * srd[3].accel_count + 1);
      // Check if we at last step
      if(srd[3].accel_count >= 0)
	  {
        srd[3].run_state = STOP;
      }
	  if(bStopCmd)
	  {
	  	bStopCmd=FALSE;
	  }
      break;
   }
  srd[3].step_delay = new_step_delay;
  
  }
}

/*! \brief Square root routine.
 *
 * sqrt routine 'grupe', from comp.sys.ibm.pc.programmer
 * Subject: Summary: SQRT(int) algorithm (with profiling)
 *    From: warwick@cs.uq.oz.au (Warwick Allison)
 *    Date: Tue Oct 8 09:16:35 1991
 *
 *  \param x  Value to find square root of.
 *  \return  Square root of x.
 */
static u32 sqrt(u32 x)
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

/*! \brief Find minimum value.
 *
 *  Returns the smallest value.
 *
 *  \return  Min(x,y).
 */
unsigned int min(unsigned int x, unsigned int y)
{
  if(x < y)
  {
    return x;
  }
  else
  {
    return y;
  }
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  	{
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
//		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		RxBuffer[RxCounter] = USART_ReceiveData(USART1);
//		USART_ClearFlag(USART1,USART_FLAG_TC); 
//		USART_SendData(USART1, RxBuffer[RxCounter]);
		if(RxBuffer[RxCounter]=='#')
		{
			RxBuffer[RxCounter]=0;
			RxCounter=0;
			bDataOK=TRUE;
		}
		else
		{
			RxCounter++;
			if(RxCounter==12)
			{
				RxCounter=0;
			}
			
		}
		
		
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	}

//	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
//  	{
//		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
//	}
}

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		
		printf("%d\r\n", position);
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		printf("%d\r\n", position);
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}
