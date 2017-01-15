#ifndef __GPIO_H

#define __GPIO_H

#include "stm32f4xx_gpio.h"


#define KEYSWITCHLEFT				   	(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0))
#define KEYSWITCHRIGHT 					(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1))
#define KEYSWITCHBACK				  	(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10))
#define PHOTOSENSORLEFTFRONT 		(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9))
#define PHOTOSENSORRIGHTFRONT 	(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8))
#define PHOTOSENSORLEFTUP 			(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_10))
#define PHOTOSENSORRIGHTUP 			(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7))
#define BEEP_ON          				GPIO_SetBits(GPIOE,GPIO_Pin_2)
#define BEEP_OFF         				GPIO_ResetBits(GPIOE,GPIO_Pin_2)
 
void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void KeyInit(void);
void BeepInit(void);
void PhotoelectricityInit(void);
#endif
