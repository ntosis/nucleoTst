/*
 * rtc.h
 *
 *  Created on: Sep 20, 2016
 *      Author: unix
 */

#ifndef RTC_H_
#define RTC_H_

#include <stm32l1xx_hal_rtc.h>

extern RTC_HandleTypeDef hrtc;
void rtc_init(void);

#endif /* RTC_H_ */
