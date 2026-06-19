#ifndef __RTC_H
#define __RTC_H
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common

/* Defines related to Clock configuration */
#define RTC_ASYNCH_PREDIV  0x7F   /* LSE as RTC clock */
#define RTC_SYNCH_PREDIV   0x00FF /* LSE as RTC clock */
int Init_ThRTC (void);

#endif
