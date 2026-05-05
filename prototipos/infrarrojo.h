#ifndef infrarrojo_h
#define infrarrojo_h

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"    
#include "Driver_I2C.h"
#define	 SENSOR_INT_PIN GPIO_PIN_12
int Init_ThIR(void);
#endif
