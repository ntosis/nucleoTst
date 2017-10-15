#include "temperatureSens.h"

uint8_t Temperature=0;
extern SPI_HandleTypeDef SpiHandle;

void initTempSens() {

	spi_init();
}

float actualTemperature() {

    float RoomTemperature,ADC_Temp_Code_dec;
    int TempVal =0;
    uint8_t SensorData[2]= {0x00,0x00};
    uint8_t dammy=0;
    __LOW(TempSensor_SS);
     /* set CS to low */

    //GPIOC->ODR &=  ~((1<<6));
    //_delay_us(10);
    osDelay(5);
    //Read the temperature data 2 bytes
    //HAL_SPI_TransmitReceive(&SpiHandle,SensorData,SensorData,2,5);
    //spi_transfer_sync (SensorData,SensorData,2);
    SensorData[0]=spi_send_read_U8(&dammy,&dammy);
    dammy=0;
    SensorData[1]=spi_send_read_U8(&dammy,&dammy);
    //_delay_us(10);
    __HIGH(TempSensor_SS);
    //GPIOC->ODR |=  ((1<<6));


    // convert the temperature to celsius and display it:
    TempVal = (SensorData[0] << 8) + SensorData[1];
    ADC_Temp_Code_dec = (float)TempVal ;
    Temperature = TempVal / 32.0;
    return (RoomTemperature = TempVal / 32.0);
}
