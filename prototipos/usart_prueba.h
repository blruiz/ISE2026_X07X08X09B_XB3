#ifndef __USART_PRUEBA_H
#define __USART_PRUEBA_H

#include "cmsis_os2.h"
#include "Driver_USART.h"
#include "stm32f4xx_hal.h"

#define START_BYTE 0xAA
#define END_BYTE   0x55

typedef struct {
  uint8_t data[4];
} FO_MSG_t;

void Init_FO(void);
void FO_SendCmd(uint8_t command, uint8_t value);

#endif
