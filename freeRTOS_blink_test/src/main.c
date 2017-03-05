/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "hardware_init.h"
#include "temperatureSens.h"
#include "eeprom_calib.h"
#include "pid.h"
#include "rtc.h"
#include <stdio.h>
#include "adc.h"
#include "ili9163.h"
#include "FRAMEWIN.h"
//#include "rtc.h"
//#include "stm32l1xx_hal_msp.c"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

osThreadId defaultTaskHandle;
osThreadId blinkTaskHandle;
osThreadId TaskHandle_10ms;
osThreadId TaskHandle_500ms;
osThreadId TaskHandle_300ms;
osSemaphoreId semHandle;

extern uint8_t Temperature;
extern SPI_HandleTypeDef SpiHandle;
extern pidData_t pidData_Htng;
extern pidData_t pidData_Coolg;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void StartDefaultTask(void const * argument);

void Task_10ms(void const *argument);
void Task_100ms(void const *argument);
void Task_300ms(void const *argument);
void Task_500ms(void const *argument);



/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /*##-Configure the RTC peripheral #######################################*/
  Configure_RTC_Clock();

  /*##-Check if Data stored in BackUp register1: No Need to reconfigure RTC#*/
  /* Read the Back Up Register 1 Data */
  if (LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR1) != RTC_BKP_DATE_TIME_UPDTATED)
  {
    /*##-Configure the RTC peripheral #######################################*/
    Configure_RTC();

    /* Configure RTC Calendar */
    Configure_RTC_Calendar();
  }

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  initTempSens();
  HAL_MspInit();
  initRtrEncoder();
  initLEDs();
  ConfigureADC();
  GUI_Init();
  pid_Init(K_P_Htng*SCALING_FACTOR,K_I_Htng*SCALING_FACTOR,K_D_Htng*SCALING_FACTOR,&pidData_Htng);
  pid_Init(K_P_Coolg*SCALING_FACTOR,K_I_Coolg*SCALING_FACTOR,K_D_Coolg*SCALING_FACTOR,&pidData_Coolg);
  //HAL_SPI_MspInit(&SpiHandle);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  //name, thread, priority, instances, stacksz
  /* Create the threads and semaphore */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(task_300ms, Task_300ms, osPriorityNormal, 0, 128);
  TaskHandle_300ms = osThreadCreate(osThread(task_300ms), NULL);

 osThreadDef(task_10ms, Task_10ms, osPriorityRealtime, 0, 128);
  TaskHandle_10ms = osThreadCreate(osThread(task_10ms), NULL);

  osThreadDef(task_500ms, Task_500ms, osPriorityNormal, 0, 128);
  TaskHandle_500ms = osThreadCreate(osThread(task_500ms), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}




/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
    while(1) {
   	  //if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
	if(1){

   	  }
     }
  /* USER CODE END 5 */ 
}

void Task_300ms(void const *argument)
{
    portTickType xLastWakeTime;
    const portTickType xDelay = 100 / portTICK_RATE_MS;
    // Initialise the xLastWakeTime variable with the current time.
         xLastWakeTime = xTaskGetTickCount ();
		while(1) {
		        //actualTemperature();
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		        readButton(xTaskGetTickCount ());
			  // Wait for the next cycle.
			  vTaskDelayUntil( &xLastWakeTime, xDelay );
		}
}

void Task_10ms(void const *argument)
    {
        portTickType xLastWakeTime;
        const portTickType xDelay = 10 / portTICK_RATE_MS;
        // Initialise the xLastWakeTime variable with the current time.
             xLastWakeTime = xTaskGetTickCount ();
    		while(1) {


    			  readEncoder();

    			  // Wait for the next cycle.
    			vTaskDelayUntil( &xLastWakeTime, xDelay );
    		}
    	}

void Task_500ms(void const *argument)
    {
        portTickType xLastWakeTime;
        const portTickType xDelay = 500 / portTICK_RATE_MS;
        uint8_t internCounter=0;
        // Initialise the xLastWakeTime variable with the current time.
             xLastWakeTime = xTaskGetTickCount ();
    		while(1) {

    			 checkStruct();
    			 updateSollTemperature();
    			 LEDfunction();
    			 volatile CAL_PARAM *gp = &CALinEE;
    			 volatile uint8_t ii =  oneLevelSystem_C;

    			 //run every 1 second
    			  if(internCounter==2) {

    				      Ctrl_Subsystem_step();
    				      /*##-3- Display the updated Time and Date ################################*/
    				      LED_Blinking((__LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC)))*10);

    				      internCounter=0;

    				       }

    			internCounter++;

    			  // Wait for the next cycle.
    			vTaskDelayUntil( &xLastWakeTime, xDelay );
    		}
    	}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
