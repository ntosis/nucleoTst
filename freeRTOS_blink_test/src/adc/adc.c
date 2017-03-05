#include "adc.h"

GPIO_InitTypeDef gpioAnalog;
ADC_ChannelConfTypeDef adcChannel1;
ADC_ChannelConfTypeDef adcChannel4;
ADC_HandleTypeDef g_AdcHandle;
//UART_HandleTypeDef huart2;


float MeasurmentChannel4() {

    volatile float ADCVoltage;
        uint32_t ADCValue;




    ADCValue =HAL_ADC_GetValue(&g_AdcHandle);
    ADCVoltage = ((float)ADCValue)/((float)4096.0f)*((float)3.3f);
   // HAL_ADC_Stop(&g_AdcHandle);
    return ADCVoltage;
}
float MeasurmentChannel1() {
    volatile float ADCVoltage;
        uint32_t ADCValue;




    ADCValue =HAL_ADC_GetValue(&g_AdcHandle);
    ADCVoltage = ((float)ADCValue)/((float)4096.0f)*((float)3.3f);

    //HAL_ADC_Stop(&g_AdcHandle);

    return ADCVoltage;
}

void InitTouch(void){
    HAL_GPIO_Init(GPIOA, &gpioAnalog);
}
void DeInitTouhch(void){
    HAL_GPIO_DeInit(GPIOA, &gpioAnalog);
}
void ConfigureADC()
{


    //__GPIOC_CLK_ENABLE();
    __ADC1_CLK_ENABLE();

    gpioAnalog.Pin = GPIO_PIN_1|GPIO_PIN_4;
    gpioAnalog.Mode = GPIO_MODE_ANALOG;
    gpioAnalog.Pull = GPIO_NOPULL;
    //HAL_GPIO_Init(GPIOA, &gpioAnalog);

    //HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(ADC_IRQn);


    g_AdcHandle.Instance = ADC1;

    g_AdcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4 ;
    g_AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
    g_AdcHandle.Init.ScanConvMode = DISABLE;//DISABLE ;
    g_AdcHandle.Init.ContinuousConvMode =DISABLE ;
    g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    g_AdcHandle.Init.NbrOfDiscConversion = 2;
    g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_AdcHandle.Init.NbrOfConversion = 2;//1
    g_AdcHandle.Init.DMAContinuousRequests = DISABLE;
    g_AdcHandle.Init.EOCSelection = ENABLE;

    HAL_ADC_Init(&g_AdcHandle);

    adcChannel1.Channel = ADC_CHANNEL_1;
    adcChannel1.Rank = 1;
    adcChannel1.SamplingTime = ADC_SAMPLETIME_192CYCLES ; //ADC_SAMPLETIME_16CYCLES;


    /*if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel1) != HAL_OK)

    	{

    	asm("bkpt 255");

    	}*/


}
int readTouchX(void) {

    GPIO_InitTypeDef GPIO_InitStruct,gpioAnalog;
    adcChannel1.Channel = ADC_CHANNEL_1;
    adcChannel1.Rank = 1;
    adcChannel1.SamplingTime = ADC_SAMPLETIME_384CYCLES; //ADC_SAMPLETIME_16CYCLES;
    //adcChannel.Offset = 0;
    if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel1) != HAL_OK)
      	{
          	asm("bkpt 255");
      	}
    //YP A1 PA1--SET AS ANALOG INPUT
    gpioAnalog.Pin = GPIO_PIN_1;
    gpioAnalog.Mode = GPIO_MODE_ANALOG;
    gpioAnalog.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpioAnalog);

    //YM D7LCD D6NUCL_PB10--SET AS INPUT (TRI-State)
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //XP D6LCD D14NUCL_PB9 --SET AS OUTPUT
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //XM A2LCD A2NUCL_PA4 --SET AS OUTPUT
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //RD PIN A0LCD A0NUCL_PA0 --SET AS OUTPUT
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    //LCD_RD HIGH to set the IC as A Ports input
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,SET);
    //XP HIGH
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,SET);
    //XM LOW
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,RESET);

    //READ ANALOG
    uint32_t ADCValue=0;
    HAL_ADC_Start(&g_AdcHandle);
    while(1){
	if(__HAL_ADC_GET_FLAG(&g_AdcHandle, ADC_FLAG_EOC)) {
	    //char msg[20];
	    ADCValue += HAL_ADC_GetValue(&g_AdcHandle); //A2
	    //sprintf(msg,"ADC: %d \r\n",ADCValue);
	    //HAL_UART_Transmit(&huart2,(uint8_t*)msg, strlen(msg), 0xFFFF);

	    break;
	}

    }
    HAL_ADC_Stop(&g_AdcHandle);

   // if (HAL_ADC_PollForConversion(&g_AdcHandle, 100) == HAL_OK)
       	//ADCValue =HAL_ADC_GetValue(&g_AdcHandle); //A1

    //DeInit ALL
    HAL_GPIO_DeInit(GPIOB,GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOB,GPIO_PIN_9);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_4);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_1);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_0);
    //int X = (((ADCValue >>4)-520)*(128))/(3386-520); dieresh me to 16 (num>>4)
    return (int)(ADCValue);

}
int readTouchY(void) {

    GPIO_InitTypeDef GPIO_InitStruct,gpioAnalog;
    adcChannel4.Channel = ADC_CHANNEL_4;
    adcChannel4.Rank = 1;
    adcChannel4.SamplingTime = ADC_SAMPLETIME_384CYCLES; //ADC_SAMPLETIME_16CYCLES;
    //adcChannel.Offset = 0;
    if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel4) != HAL_OK)
      	{
           	asm("bkpt 255");
      	}
    //YP A1 PA1--SET AS OUTPUT
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    //YM D7LCD D6NUCL_PB10--SET AS OUTPUT
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //XP D6LCD D14NUCL_PB9 --SET AS INPUT(Hi-Z)
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //XM A2LCD A2NUCL_PA4 --SET AS INPUT
    gpioAnalog.Pin = GPIO_PIN_4;
    gpioAnalog.Mode = GPIO_MODE_ANALOG;
    gpioAnalog.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpioAnalog);
    //RD PIN A0LCD A0NUCL_PA0 --SET AS OUTPUT
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    //LCD_RD HIGH to set the IC as A Ports input
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,SET);

    //YP HIGH
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,SET);
    //YM LOW
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,RESET);

    //READ ANALOG
    HAL_ADC_Start(&g_AdcHandle);
    volatile uint32_t ADCValue=0;
    while(1){
   	if(__HAL_ADC_GET_FLAG(&g_AdcHandle, ADC_FLAG_EOC)) {
   	    ADCValue =HAL_ADC_GetValue(&g_AdcHandle); //A2
   	    break;
   	}

       }
   // if (HAL_ADC_PollForConversion(&g_AdcHandle, 100) == HAL_OK)
	//ADCValue =HAL_ADC_GetValue(&g_AdcHandle); //A1
    //ADCValue =HAL_ADC_GetValue(&g_AdcHandle); //A2

    HAL_ADC_Stop(&g_AdcHandle);

    //DeInit ALL
    HAL_GPIO_DeInit(GPIOB,GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOB,GPIO_PIN_9);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_4);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_1);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_0);
    //int Y = (((ADCValue)-590)*(160))/(3693-590);
    return (int)(ADCValue);
    //return (int)ADCValue;

}

int readPressure(void) {
    GPIO_InitTypeDef GPIO_InitStruct,gpioAnalog;



    //XP D6LCD D14NUCL_PB9 --SET AS OUTPUT
     GPIO_InitStruct.Pin = GPIO_PIN_9;
     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
     GPIO_InitStruct.Pull = GPIO_PULLUP;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


     //YM D7LCD D6NUCL_PB10--SET AS OUTPUT
     GPIO_InitStruct.Pin = GPIO_PIN_10;
     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
     GPIO_InitStruct.Pull = GPIO_PULLUP;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


     //XM A2LCD A2NUCL_PA4 --SET AS INPUT
     gpioAnalog.Pin = GPIO_PIN_4;
     gpioAnalog.Mode = GPIO_MODE_ANALOG;
     gpioAnalog.Pull = GPIO_NOPULL;
     HAL_GPIO_Init(GPIOA, &gpioAnalog);

     //YP A1 PA1--SET AS ANALOG INPUT
     gpioAnalog.Pin = GPIO_PIN_1;
     gpioAnalog.Mode = GPIO_MODE_ANALOG;
     gpioAnalog.Pull = GPIO_NOPULL;
     HAL_GPIO_Init(GPIOA, &gpioAnalog);

     //LCD_RD HIGH to set the IC as A Ports input
     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,SET);
     //set X+ to ground
     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,RESET);
     // Set Y- to VCC
     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,SET);

     //READ ANALOG CH4
     if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel4) != HAL_OK)
          {
            	asm("bkpt 255");
          }
         HAL_ADC_Start(&g_AdcHandle);
         volatile uint32_t ADCValue=0;
         while(1){
        	if(__HAL_ADC_GET_FLAG(&g_AdcHandle, ADC_FLAG_EOC)) {
        	    ADCValue =HAL_ADC_GetValue(&g_AdcHandle); //A2
        	    break;
        	}

            }
        // if (HAL_ADC_PollForConversion(&g_AdcHandle, 100) == HAL_OK)
     	//ADCValue =HAL_ADC_GetValue(&g_AdcHandle); //A1

         HAL_ADC_Stop(&g_AdcHandle);

         //READ ANALOG CH1
              if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel1) != HAL_OK)
                   {
                     	asm("bkpt 255");
                   }
                  HAL_ADC_Start(&g_AdcHandle);
                  volatile uint32_t ADCValue2=0;
                  while(1){
                 	if(__HAL_ADC_GET_FLAG(&g_AdcHandle, ADC_FLAG_EOC)) {
                 	    ADCValue2 =HAL_ADC_GetValue(&g_AdcHandle); //A2
                 	    break;
                 	}

                     }
                 HAL_ADC_Stop(&g_AdcHandle);

       //DeInit ALL
      HAL_GPIO_DeInit(GPIOB,GPIO_PIN_10);
      HAL_GPIO_DeInit(GPIOB,GPIO_PIN_9);
      HAL_GPIO_DeInit(GPIOA,GPIO_PIN_4);
      HAL_GPIO_DeInit(GPIOA,GPIO_PIN_1);
      HAL_GPIO_DeInit(GPIOA,GPIO_PIN_0);

      return abs(ADCValue2-ADCValue);
}
void MX_USART2_UART_Init(void)
{
	/*huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	HAL_UART_Init(&huart2); */
}


void LCD_X_Config(void) {
  //
  // Initialize display driver
  //
  //
  // Set orientation of touch screen (only required when using
  //
  unsigned int TouchOrientation = (GUI_MIRROR_X * LCD_GetMirrorX()) |
                     (GUI_MIRROR_Y * LCD_GetMirrorY()) |
                     (GUI_SWAP_XY  * LCD_GetSwapXY()) ;
  GUI_TOUCH_SetOrientation(TouchOrientation);
  //
  // Calibrate touch screen
  //
  GUI_TOUCH_Calibrate(GUI_COORD_X, 0, 240, GUI_TOUCH_AD_TOP , GUI_TOUCH_AD_BOTTOM);
  GUI_TOUCH_Calibrate(GUI_COORD_Y, 0, 320, GUI_TOUCH_AD_LEFT, GUI_TOUCH_AD_RIGHT);
}

void GUI_TOUCH_X_ActivateX(void) {} //empty
void GUI_TOUCH_X_ActivateY(void) {} //empty
int GUI_TOUCH_X_MeasureX(void) {
    return readTouchX();
}
int GUI_TOUCH_X_MeasureY(void) {
    return readTouchY();
}
void GUI_X_Config(void){}
void GUI_X_Init  (void){}
