#include "stm32f10x.h"
#include "LedAndKey.h"

static uint32_t KeyAndJostickValue;

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void KyeAndJoystickScan(void)
{
	if(!GPIO_ReadInputDataBit( IO_KEY_ARRAY, IO_KEY1))KeyAndJostickValue = KEY1_VALUE;
	if(!GPIO_ReadInputDataBit( IO_KEY_ARRAY, IO_KEY2))KeyAndJostickValue = KEY2_VALUE;
	if(!GPIO_ReadInputDataBit( IO_KEY_ARRAY, IO_KEY3))KeyAndJostickValue = KEY3_VALUE;
	
	if(!GPIO_ReadInputDataBit( IO_JOYSTICK_ARRAY, IO_JOYSTICK_A))KeyAndJostickValue = JOYSTICK_A_VALUE;
	if(!GPIO_ReadInputDataBit( IO_JOYSTICK_ARRAY, IO_JOYSTICK_B))KeyAndJostickValue = JOYSTICK_B_VALUE;
	if(!GPIO_ReadInputDataBit( IO_JOYSTICK_ARRAY, IO_JOYSTICK_C))KeyAndJostickValue = JOYSTICK_C_VALUE;
	if(!GPIO_ReadInputDataBit( IO_JOYSTICK_ARRAY, IO_JOYSTICK_D))KeyAndJostickValue = JOYSTICK_D_VALUE;
	if(!GPIO_ReadInputDataBit( IO_JOYSTICK_ARRAY, IO_JOYSTICK_P))KeyAndJostickValue = JOYSTICK_P_VALUE;
}

void LedShow(void)
{
	if(KeyAndJostickValue == KEY1_VALUE)GPIO_WriteBit( IO_LED_ARRAY, IO_LED1 | IO_LED2,Bit_SET);
	if(KeyAndJostickValue == KEY2_VALUE)GPIO_WriteBit( IO_LED_ARRAY, IO_LED2 | IO_LED3,Bit_SET);
	if(KeyAndJostickValue == KEY3_VALUE)GPIO_WriteBit( IO_LED_ARRAY, IO_LED3 | IO_LED4,Bit_SET);
	
	if(KeyAndJostickValue == JOYSTICK_B_VALUE)GPIO_WriteBit( IO_LED_ARRAY, IO_LED1,Bit_SET);
	if(KeyAndJostickValue == JOYSTICK_C_VALUE)GPIO_WriteBit( IO_LED_ARRAY, IO_LED4,Bit_SET);
	if(KeyAndJostickValue == JOYSTICK_A_VALUE)GPIO_WriteBit( IO_LED_ARRAY, IO_LED2,Bit_SET);
	if(KeyAndJostickValue == JOYSTICK_D_VALUE)GPIO_WriteBit( IO_LED_ARRAY, IO_LED3,Bit_SET);
	if(KeyAndJostickValue == JOYSTICK_P_VALUE)GPIO_WriteBit( IO_LED_ARRAY, IO_LED1 | IO_LED2 | IO_LED3 | IO_LED4,Bit_SET);

	Delay(0x5ffff);
	GPIO_WriteBit( IO_LED_ARRAY, IO_LED1 | IO_LED2 | IO_LED3 | IO_LED4,Bit_RESET);
	Delay(0x5ffff);
}

void LedGpioConfiguration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_IO_LED, ENABLE);

  GPIO_InitStructure.GPIO_Pin = IO_LED1 | IO_LED2 | IO_LED3 | IO_LED4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init( IO_LED_ARRAY, &GPIO_InitStructure);
}

void KeyGpioConfiguration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_IO_KEY, ENABLE);

  GPIO_InitStructure.GPIO_Pin = IO_KEY1 | IO_KEY2 | IO_KEY3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init( IO_KEY_ARRAY, &GPIO_InitStructure);
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
