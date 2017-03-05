/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

#include "hardware_init.h"
#include "GUI.h"

extern GPIO_InitTypeDef gpioInit;
extern ADC_ChannelConfTypeDef adcChannel1;
extern ADC_ChannelConfTypeDef adcChannel4;
//extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef g_AdcHandle;

void ConfigureADC(void);
void InitTouch(void);
void DeInitTouhch(void);
float MeasurmentChannel4();
float MeasurmentChannel1();


#define GUI_TOUCH_AD_TOP     877
#define GUI_TOUCH_AD_BOTTOM  273
#define GUI_TOUCH_AD_LEFT    232
#define GUI_TOUCH_AD_RIGHT   918

#endif

