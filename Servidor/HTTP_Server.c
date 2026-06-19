/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::Network
 * Copyright (c) 2004-2019 Arm Limited (or its affiliates). All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    HTTP_Server.c
 * Purpose: HTTP Server example
 *----------------------------------------------------------------------------*/

#include "main.h"
#include "rl_net.h"                     // Keil.MDK-Pro::Network:CORE
#include "string.h"
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "memoria.h"
#include "usart.h"

// --- Configuración de Atributos del Hilo Principal (app_main) ---
// El stack debe ser múltiplo de 8 bytes por alineación de hardware en ARM Cortex-M
#define APP_MAIN_STK_SZ (1024U)

#define FLAG_FORMATEO 0x20
#define FLAG_ACTUALIZAR 0X40

extern char player_name_global[21];
extern uint16_t puntos;
extern bool ranking_actualizado;

uint64_t app_main_stk[APP_MAIN_STK_SZ / 8];
const osThreadAttr_t app_main_attr = {
  .stack_mem  = &app_main_stk[0],
  .stack_size = sizeof(app_main_stk)
};

// --- Declaraciones Externas ---
//extern uint16_t AD_in          (uint32_t ch);
extern osMessageQueueId_t mid_MsgQueueLCD; // Cola definida en lcd.c
extern void     netDHCP_Notify (uint32_t if_num, uint8_t option, const uint8_t *val, uint32_t len);


/* Thread IDs */
 osThreadId_t tid_formateo;
static void Thread_Formateo (void *arg);
/* Thread declarations */
//static void BlinkLed (void *arg);
//static void Display  (void *arg);

/*Configuracion stack hilo formateo pila*/
//static const osThreadAttr_t atributo_hilo = {
//  .stack_size = 256
//};

__NO_RETURN void app_main (void *arg);
//PRUEBA
osThreadId_t tid_prueba;
void Thread_prueba (void *arg);

/* IP address change notification */
void netDHCP_Notify (uint32_t if_num, uint8_t option, const uint8_t *val, uint32_t len) {

  (void)if_num;
  (void)val;
  (void)len;

  if (option == NET_DHCP_OPTION_IP_ADDRESS) {
    /* IP address change, trigger LCD update */
//    osThreadFlagsSet (TID_Display, 0x01);
  }
}

/*----------------------------------------------------------------------------
  Main Thread 'main': Run Network
 *---------------------------------------------------------------------------*/
__NO_RETURN void app_main (void *arg) {
  (void)arg;
	// Inicialización de la pila de red (Network Stack)
	netInitialize ();
  MemoriaInitialize();         // Inicializa el periférico I2C/SPI de la EEPROM
  tid_formateo = osThreadNew (Thread_Formateo,  NULL, NULL);
//  tid_prueba = osThreadNew (Thread_prueba,  NULL, NULL);
  osThreadExit();// El hilo app_main ya no es necesario, se cierra para liberar recursos
}

void Thread_Formateo (void *arg){
  uint32_t flags;
	while(1){
			flags = osThreadFlagsWait(FLAG_FORMATEO | FLAG_ACTUALIZAR, osFlagsWaitAny, osWaitForever);
      if( flags == FLAG_FORMATEO){
        FormatearEEPROM_Ranking();
        CargarRankingDesdeEEPROM();}
      else if(flags == FLAG_ACTUALIZAR){
//        if (!ranking_actualizado) {
          ActualizarRanking(puntos, player_name_global);
//          ranking_actualizado = true; // Echamos el candado
//          } 
//          else {
//            // Si la acción es cualquier otra (PALMADA, GESTO, etc.), 
//            // significa que estamos en una partida nueva o activa. Rearmamos el candado.
//            ranking_actualizado = false; 
//          }
      }
      osDelay(10);
  }
}

