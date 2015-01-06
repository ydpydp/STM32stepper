#ifndef __STEPPINGMOTOR_H
#define __STEPPINGMOTOR_H

//PWM_OUT IO define					
#define IO_PWM_ARRAY GPIOA
#define RCC_IO_PWM RCC_APB2Periph_GPIOA
#define IO_PWM_CH2 GPIO_Pin_1
#define IO_PWM_CH3 GPIO_Pin_2
#define IO_PWM_CH4 GPIO_Pin_3

//SteppingMotor Direction
#define IO_Direction_ARRAY GPIOA
#define RCC_IO_Direction RCC_ABP2Periph_GPIOA
#define IO_DirectionX GPIO_Pin_4
#define IO_DirectionY GPIO_Pin_5
#define IO_DirectionZ GPIO_Pin_6

//KEY IO define					
/*
#define IO_KEY_ARRAY GPIOA
#define RCC_IO_KEY RCC_APB2Periph_GPIOA
#define IO_KEY1 GPIO_Pin_0
#define IO_KEY2 GPIO_Pin_0
#define IO_KEY3 GPIO_Pin_0
*/

//JOYSTICK IO define					
#define IO_JOYSTICK_ARRAY GPIOB
#define RCC_IO_JOYSTICK RCC_APB2Periph_GPIOB
#define IO_JOYSTICK_A GPIO_Pin_0
#define IO_JOYSTICK_B GPIO_Pin_1
#define IO_JOYSTICK_C GPIO_Pin_3
#define IO_JOYSTICK_D GPIO_Pin_8
#define IO_JOYSTICK_P GPIO_Pin_9

//define Key and Joystick Value
/*
#define KEY1_VALUE 1
#define KEY2_VALUE 2
#define KEY3_VALUE 3
*/
#define JOYSTICK_A_VALUE 4
#define JOYSTICK_B_VALUE 5
#define JOYSTICK_C_VALUE 6
#define JOYSTICK_D_VALUE 7
#define JOYSTICK_P_VALUE 8

//define extern function
extern void KyeAndJoystickScan(void);
//extern void LedShow(void);
//extern void LedGpioConfiguration(void);
//extern void KeyGpioConfiguration(void);
extern void JoystickGpioConfiguration(void);
extern void SteppingMotor_Config(void);
extern void Run(void);

#endif
