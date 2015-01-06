/**
  ******************************************************************************
  * @file    Project/STM32F103CBT6/user/main.c 
  * @author  Rocyang
  * @version V0.7
  * @date    18-December-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "SteppingMotor.h"
#include "RTC_Time.h" 
#include "usart.h"
#include <stdio.h>

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

	
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     

	SystemInit();
	USART_Configuration();

  printf("\r\n****************************************************************\r\n");
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB , ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	//LedGpioConfiguration();
	//KeyGpioConfiguration();
	JoystickGpioConfiguration();
	SteppingMotor_Config();
	RTC_Init();
	
	while(1)
	{
		KyeAndJoystickScan();
		Run();

	}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */

}

/******************* (C) COPYRIGHT 2014 Rocyang(ydpydp@hotmail.com) *****END OF FILE****/
