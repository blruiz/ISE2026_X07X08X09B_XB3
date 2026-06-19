#ifndef __USART_H
#define __USART_H

#include "cmsis_os2.h"
#include "Driver_USART.h"
#include "stm32f4xx_hal.h"

#define START_BYTE 0xAA
#define END_BYTE   0x55

#define ACCION   0x77
#define PALMADA  0x11
#define MOVIMIENTO  0x22
#define GESTO  0x33

#define PUNTOS   0xFF

#define CONSUMO   0x99

#define INI_JUEGO    0x44
#define DERROTA    0x88
#define PAUSA        0x10

typedef struct {
  uint8_t data[4];
} FO_MSG_t;

 extern void Init_FO(void);

void enviarDatos(uint8_t funcion, uint8_t valor);

#endif
