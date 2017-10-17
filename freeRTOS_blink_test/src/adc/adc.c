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
    GPIO_TypeDef tempA; // save the state of the PortA
    GPIO_TypeDef tempB; // save the state of the PortB
    memcpy((void *)&tempA,GPIOA,sizeof(GPIO_TypeDef));
    memcpy((void *)&tempB,GPIOB,sizeof(GPIO_TypeDef));

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
    /*
    HAL_GPIO_DeInit(GPIOB,GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOB,GPIO_PIN_9);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_4);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_1);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_0);
    */
    memcpy(GPIOA,(void *)&tempA,sizeof(GPIO_TypeDef));
    memcpy(GPIOB,(void *)&tempB,sizeof(GPIO_TypeDef));

    //int X = (((ADCValue >>4)-520)*(128))/(3386-520); dieresh me to 16 (num>>4)
    return (int)(ADCValue);

}
int readTouchY(void) {

    GPIO_TypeDef tempA; // save the state of the PortA
    GPIO_TypeDef tempB; // save the state of the PortB
    memcpy((void *)&tempA,GPIOA,sizeof(GPIO_TypeDef));
    memcpy((void *)&tempB,GPIOB,sizeof(GPIO_TypeDef));

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

    /*//DeInit ALL
    HAL_GPIO_DeInit(GPIOB,GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOB,GPIO_PIN_9);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_4);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_1);
    HAL_GPIO_DeInit(GPIOA,GPIO_PIN_0); */
    //int Y = (((ADCValue)-590)*(160))/(3693-590);
    memcpy(GPIOA,(void *)&tempA,sizeof(GPIO_TypeDef));
    memcpy(GPIOB,(void *)&tempB,sizeof(GPIO_TypeDef));
    return (int)(ADCValue);
    //return (int)ADCValue;

}

int readPressure(void) {
    GPIO_InitTypeDef GPIO_InitStruct,gpioAnalog;

    GPIO_TypeDef tempA; // save the state of the PortA
    GPIO_TypeDef tempB; // save the state of the PortB
    memcpy((void *)&tempA,GPIOA,sizeof(GPIO_TypeDef));
    memcpy((void *)&tempB,GPIOB,sizeof(GPIO_TypeDef));

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
     /*
      HAL_GPIO_DeInit(GPIOB,GPIO_PIN_10);
      HAL_GPIO_DeInit(GPIOB,GPIO_PIN_9);
      HAL_GPIO_DeInit(GPIOA,GPIO_PIN_4);
      HAL_GPIO_DeInit(GPIOA,GPIO_PIN_1);
      HAL_GPIO_DeInit(GPIOA,GPIO_PIN_0);
      */

      memcpy(GPIOA,(void *)&tempA,sizeof(GPIO_TypeDef));
      memcpy(GPIOB,(void *)&tempB,sizeof(GPIO_TypeDef));
      return abs(ADCValue2-ADCValue);
}



void GUI_TOUCH_X_ActivateX(void) {} //empty
void GUI_TOUCH_X_ActivateY(void) {} //empty
int GUI_TOUCH_X_MeasureX(void) {

    /* Frame of the command for XPT2046
    BIT7(MSB) BIT6 BIT5 BIT4 BIT3 BIT2 BIT1 BIT0(LSB)
    S A2 A1 A0 MODE (SER/DFR) PD1 PD0
    S always high, 001=xp read, 1=8bit ADC, 0=DFR, 00=power down mode */
   /* 1 001 0 0 01
    1 101 0 0 01
    1 011 0 0 01
    1 100 0 0 01
    1 001 0 0 00*/
    /* local variables */
    uint8_t frame_read_XP = 0b11011001;
    uint8_t power_down_xpt2046 = 0b10011000;
    uint8_t tmp_buffer=0;
    uint8_t uart_buffer[50]={0};

    /* check if spi is busy for LCD */
    if(spi_TFT_busy_flag)
    {

	return -1;

    }

    /* check if touch pin is active */
    else if(__READ(TOUCH_INT))
    {
	//return -1;

    }

    /* set spi busy flag high*/
    spi_TFT_busy_flag = 1;


    /* set CS to low */
    GPIOC->ODR &=  ~((1<<6));

    /* send the command */
    spi_send_read_U8(&frame_read_XP,&tmp_buffer);
    /* read the raw value */
    spi_send_read_U8(0,&tmp_buffer);
    /* power down xpt2046 */
    spi_send_U8(power_down_xpt2046);

    /* set CS to high */
    GPIOC->ODR |=  ((1<<6));

    /* set spi busy flag low */
    spi_TFT_busy_flag = 0;
    sprintf(uart_buffer,"X: %d\r\n",tmp_buffer);
    HAL_UART_Transmit(&huart2,uart_buffer,sizeof(uart_buffer),5);
    /* return value */
    return tmp_buffer;

}
int GUI_TOUCH_X_MeasureY(void) {

    /* Frame of the command for XPT2046
    BIT7(MSB) BIT6 BIT5 BIT4 BIT3 BIT2 BIT1 BIT0(LSB)
    S A2 A1 A0 MODE (SER/DFR) PD1 PD0
    S always high, 101=yp read, 1=8bit ADC, 0=DFR, 00=power down mode */
    /* local variables */
    uint8_t frame_read_YP = 0b10011001;
    uint8_t power_down_xpt2046 = 0b10011000;
    uint8_t tmp_buffer=0;
        uint8_t uart_buffer[50]={0};


      /* check if spi is busy for LCD */
    if(spi_TFT_busy_flag)
    {

	return -1;

    }

    /* check if touch pin is active (low) */
    else if(__READ(TOUCH_INT))
    {
	//return -1;

    }

    /* set spi busy flag high*/
    spi_TFT_busy_flag = 1;

    /* set CS to low */
    GPIOC->ODR &=  ~((1<<6));

    /* send the command */
    spi_send_read_U8(&frame_read_YP,&tmp_buffer);
    /* read the raw value */
    spi_send_read_U8(0,&tmp_buffer);
    /* power down xpt2046 */
    spi_send_U8(power_down_xpt2046);

    /* set CS to high */
    GPIOC->ODR |=  ((1<<6));

    /* set spi busy flag low*/
    spi_TFT_busy_flag = 0;
    sprintf(uart_buffer,"Y: %d ",tmp_buffer);
    HAL_UART_Transmit(&huart2,uart_buffer,sizeof(uart_buffer),5);
    /* return value */
    return tmp_buffer;
}
