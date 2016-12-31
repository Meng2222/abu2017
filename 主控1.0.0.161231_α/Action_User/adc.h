#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx.h"

#define LEFT               0
#define RIGHT              1
#define BASIC_LEFT         2
#define BASIC_RIGHT        2
#define POST               3

#define TOTALLASERNUM      4

#define LEFTGET  (GetLaserValue(LEFT)<2000&&GetLaserValue(LEFT)>70)
#define LEFTLOSE (GetLaserValue(LEFT)<70||GetLaserValue(LEFT)>2000)

#define RIGHTGET  (GetLaserValue(RIGHT)<=2000&&GetLaserValue(RIGHT)>70)
#define RIGHTLOSE (GetLaserValue(RIGHT)<70||GetLaserValue(RIGHT)>2000)

void ADC1mixed_DMA_Config(void);
void AverageValue(void);
uint16_t GetLaserValue(u8 num);

#endif
