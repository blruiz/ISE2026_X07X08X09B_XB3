#include "usart.h"
#include <string.h>
#include <stdio.h>

osThreadId_t tid_Th_FO;
osMessageQueueId_t mid_MsgQueueFO;
 //uint8_t trama_envio[4];
 FO_MSG_t objetoLocal; // Usamos la estructura 
 uint8_t buffer_rx[4];
extern ARM_DRIVER_USART Driver_USART7; 
static ARM_DRIVER_USART *USARTdrv = &Driver_USART7;

static const osThreadAttr_t threadUART_attr = {
  .stack_size = 512
};

void enviarDatos(uint8_t funcion, uint8_t valor);
void init_UART(void);

int Init_MsgQueue(void);
int Init_Th(void);

void Init_FO(void) {
  init_UART();
  Init_MsgQueue();
  Init_Th();

}
extern char accion_go[21];
extern uint16_t puntos;
extern uint8_t consumo;
FO_MSG_t mensaje; 

// Callback
void myUSART_callback(uint32_t event) {
  if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
        osThreadFlagsSet(tid_Th_FO, 0x01); // Flag de Recepción
    }
    if (event & ARM_USART_EVENT_SEND_COMPLETE) {
        osThreadFlagsSet(tid_Th_FO, 0x02); // Flag de Transmisión
    }
  
  if (event & ARM_USART_EVENT_RX_TIMEOUT) {}

  if (event & (ARM_USART_EVENT_RX_OVERFLOW | ARM_USART_EVENT_TX_UNDERFLOW)) {}
}

// Función para enviar a la cola
//void FO_Send(uint8_t command, uint8_t value) {
//  FO_MSG_t msg;
//  msg.cmd = command;
// // msg.data = value;
//  osMessageQueuePut(mid_MsgQueueFO, &msg, 0U, 0U);
//}

void Thread_FO(void *argument) {
 uint32_t flags;
    
    // 1. Iniciamos la recepción ASÍNCRONA por primera vez (solo una vez fuera del while)
    USARTdrv->Receive(buffer_rx, 4);

    while (1) {
        // 2. ENVÍO: Revisamos si hay algo en la cola para enviar
        // Usamos un timeout de 0 para no bloquear aquí si no hay mensajes
        if (osMessageQueueGet(mid_MsgQueueFO, &objetoLocal, NULL, 0) == osOK) {   //obtiene los mensajes de la cola
            USARTdrv->Send(objetoLocal.data, 4);																	//envia los mensajes en la cola
            
            // Esperamos específicamente el flag de TRANSMISIÓN (0x02)
            // Esto es crucial para no saturar el driver
            //osThreadFlagsWait(0x02, osFlagsWaitAny, osWaitForever); 
					                       
						if (!(osThreadFlagsWait(0x02, osFlagsWaitAny, 100))) {     // Esperamos TX con timeout             
//							printf("Error TX UART\n");            
						}
        }

        // 3. RECEPCIÓN: Esperamos el flag de RECEPCIÓN (0x01) con un timeout pequeño
        // El timeout permite que el bucle siga girando para revisar la cola de envío
        flags = osThreadFlagsWait(0x01, osFlagsWaitAny, 10);

        if (flags & 0x01) {
            // Procesamos lo recibido
            if (buffer_rx[0] == START_BYTE && buffer_rx[3] == END_BYTE) { //comprueba que llega un mensaje completo
                
              
              if (buffer_rx[1] == ACCION ) { //comprueba que le llega el mensaje para iniciar juego
                  switch(buffer_rx[2]){
                    case PALMADA:
                        strcpy(accion_go, "PALMADA");
                        break;
                   case MOVIMIENTO:
                        strcpy(accion_go, "GOLPEA");
                        break;
                   case GESTO:
                        strcpy(accion_go, "GESTO");
                        break;
                  }
              }
              else if(buffer_rx[1] == PUNTOS ) {
                  puntos = buffer_rx[2];

              }
              else if(buffer_rx[1] == CONSUMO ) {
                  consumo = buffer_rx[2];
              }
              else if(buffer_rx[1] == DERROTA ) {
                  strcpy(accion_go, "DERROTA");
              }
              else if(buffer_rx[1] == PAUSA ) {
                  strcpy(accion_go, "PAUSA");
              }
            } else {
                // Si el frame es basura o está desfasado, abortamos y limpiamos
                USARTdrv->Control(ARM_USART_ABORT_RECEIVE, 0);
            }
            memset(buffer_rx, 0, 4); //borrar buffer recepción
            // 4. RE-ARMAMOS la recepción después de procesar (solo si recibimos algo)
            USARTdrv->Receive(buffer_rx, 4);
        }
        
        // Pequeño retardo para estabilidad del RTOS
        osDelay(1);
    }
}

void init_UART(void){
  /*Initialize the USART driver */
  USARTdrv->Initialize(myUSART_callback);
  /*Power up the USART peripheral */
  USARTdrv->PowerControl(ARM_POWER_FULL);
  /*Configure the USART to 9600 Bits/sec */
  USARTdrv->Control(ARM_USART_MODE_ASYNCHRONOUS |
                    ARM_USART_DATA_BITS_8 |
                    ARM_USART_PARITY_NONE |
                    ARM_USART_STOP_BITS_1 |
                    ARM_USART_FLOW_CONTROL_NONE, 9600);
     
    /* Enable Receiver and Transmitter lines */
  USARTdrv->Control (ARM_USART_CONTROL_TX, 1);
  USARTdrv->Control (ARM_USART_CONTROL_RX, 1);
}

int Init_Th(void) {

	tid_Th_FO = osThreadNew(Thread_FO, NULL, &threadUART_attr);
  if ( tid_Th_FO == NULL) {
    return (-1);
  }
  return (0);
}

int Init_MsgQueue(void){ 
 mid_MsgQueueFO = osMessageQueueNew(8, sizeof(FO_MSG_t), NULL);
  if (mid_MsgQueueFO == NULL) {
    return(-1);
  }
  return(0);
}

void enviarDatos(uint8_t funcion, uint8_t valor){ //crea el mensaje para enviar
  mensaje.data[0] = 0xAA;
  mensaje.data[1] = funcion;
  mensaje.data[2] = valor;
  mensaje.data[3] = 0x55;
     
	if (osMessageQueuePut(mid_MsgQueueFO, &mensaje, 0U, 0U) != osOK) {        
//		printf("Cola FO llena\n");    
	}
}
