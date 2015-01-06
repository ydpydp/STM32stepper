/**
  ******************************************************************************
  * @file    GPIO/IOToggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm324xg_eval.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup IOToggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */

  /* GPIOG Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

  /* Configure PG6 and PG8 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

  /* To achieve GPIO toggling maximum frequency, the following  sequence is mandatory. 
     You can monitor PG6 or PG8 on the scope to measure the output signal. 
     If you need to fine tune this frequency, you can add more GPIO set/reset 
     cycles to minimize more the infinite loop timing.
     This code needs to be compiled with high speed optimization option.  */
  while (1)
  {
    /* Set PG6 and PG8 */
    GPIOG->BSRRL = GPIO_Pin_6 | GPIO_Pin_8;
    /* Reset PG6 and PG8 */
    GPIOG->BSRRH = GPIO_Pin_6 | GPIO_Pin_8;

    /* Set PG6 and PG8 */
    GPIOG->BSRRL = GPIO_Pin_6 | GPIO_Pin_8;
    /* Reset PG6 and PG8 */
    GPIOG->BSRRH = GPIO_Pin_6 | GPIO_Pin_8;

    /* Set PG6 and PG8 */
    GPIOG->BSRRL = GPIO_Pin_6 | GPIO_Pin_8;
    /* Reset PG6 and PG8 */
    GPIOG->BSRRH = GPIO_Pin_6 | GPIO_Pin_8;

    /* Set PG6 and PG8 */
    GPIOG->BSRRL = GPIO_Pin_6 | GPIO_Pin_8;
    /* Reset PG6 and PG8 */
    GPIOG->BSRRH = GPIO_Pin_6 | GPIO_Pin_8;

    /* Set PG6 and PG8 */
    GPIOG->BSRRL = GPIO_Pin_6 | GPIO_Pin_8;
    /* Reset PG6 and PG8 */
    GPIOG->BSRRH = GPIO_Pin_6 | GPIO_Pin_8;

    /* Set PG6 and PG8 */
    GPIOG->BSRRL = GPIO_Pin_6 | GPIO_Pin_8;
    /* Reset PG6 and PG8 */
    GPIOG->BSRRH = GPIO_Pin_6 | GPIO_Pin_8;

    /* Set PG6 and PG8 */
    GPIOG->BSRRL = GPIO_Pin_6 | GPIO_Pin_8;
    /* Reset PG6 and PG8 */
    GPIOG->BSRRH = GPIO_Pin_6 | GPIO_Pin_8;

    /* Set PG6 and PG8 */
    GPIOG->BSRRL = GPIO_Pin_6 | GPIO_Pin_8;
    /* Reset PG6 and PG8 */
    GPIOG->BSRRH = GPIO_Pin_6 | GPIO_Pin_8;

    /* Set PG6 and PG8 */
    GPIOG->BSRRL = GPIO_Pin_6 | GPIO_Pin_8;
    /* Reset PG6 and PG8 */
    GPIOG->BSRRH = GPIO_Pin_6 | GPIO_Pin_8;

    /* Set PG6 and PG8 */
    GPIOG->BSRRL = GPIO_Pin_6 | GPIO_Pin_8;
    /* Reset PG6 and PG8 */
    GPIOG->BSRRH = GPIO_Pin_6 | GPIO_Pin_8;
  }
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
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

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
