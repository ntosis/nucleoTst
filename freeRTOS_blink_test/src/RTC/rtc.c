/*
 * rtc.c
 *
 *  Created on: Sep 20, 2016
 *      Author: unix
 */

#include "rtc.h"

RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

void rtc_init(void) {

    /**Initialize RTC and set the Time and Date
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv= 127;
  hrtc.Init.SynchPrediv= 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 9;
  sTime.Minutes = 30;
  sTime.Seconds = 0;
  sTime.SubSeconds = 0;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_AUGUST;
  sDate.Date = 24;
  sDate.Year = 15;
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);


}
