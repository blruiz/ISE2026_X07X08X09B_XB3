#include "usart_prueba.h"
#include <string.h>

osThreadId_t tid_Th_FO;
osMessageQueueId_t mid_MsgQueueFO;
 //uint8_t trama_envio[4];
 FO_MSG_t objetoLocal; // Usamos la estructura 
 uint8_t buffer_rx[4];
extern ARM_DRIVER_USART Driver_USART7; 
static ARM_DRIVER_USART *USARTdrv = &Driver_USART7;

void init_UART(void);
//ejemplo boton
osThreadId_t tid_Th_Boton; // ID del hilo del botón

// Prototipos
void Thread_Boton(void *argument);
int Init_Th(void);

void Init_LEDs(void);
void Init_UserButton(void);

void Init_FO(void) {
  init_UART();
  Init_MsgQueue();
	Init_Th();
	Init_LEDs();
}

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
        if (osMessageQueueGet(mid_MsgQueueFO, &objetoLocal, NULL, 0) == osOK) {
            USARTdrv->Send(objetoLocal.data, 4);
            
            // Esperamos específicamente el flag de TRANSMISIÓN (0x02)
            // Esto es crucial para no saturar el driver
            osThreadFlagsWait(0x02, osFlagsWaitAny, osWaitForever); 
        }

        // 3. RECEPCIÓN: Esperamos el flag de RECEPCIÓN (0x01) con un timeout pequeño
        // El timeout permite que el bucle siga girando para revisar la cola de envío
        flags = osThreadFlagsWait(0x01, osFlagsWaitAny, 10);

        if (flags == 0x01) {
            // Procesamos lo recibido
            if (buffer_rx[0] == START_BYTE && buffer_rx[3] == END_BYTE) {
                if (buffer_rx[1] == 0x04 && buffer_rx[2] == 0x08) {
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
                    osDelay(1000);
                    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
                }
            } else {
                // Si el frame es basura o está desfasado, abortamos y limpiamos
                USARTdrv->Control(ARM_USART_ABORT_RECEIVE, 0);
            }
            memset(buffer_rx, 0, 4);
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
	Init_UserButton();
	tid_Th_FO = osThreadNew(Thread_FO, NULL, NULL);
  tid_Th_Boton = osThreadNew(Thread_Boton, NULL, NULL);
  if (tid_Th_Boton == NULL || tid_Th_FO == NULL) {
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

void Thread_Boton(void *argument) {
  FO_MSG_t mensaje; 
  
  while (1) {
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
      // Rellenamos el array dentro de la estructura
      mensaje.data[0] = 0xAA;
      mensaje.data[1] = 0x04;
      mensaje.data[2] = 0x08;
      mensaje.data[3] = 0x55;
      
      osMessageQueuePut(mid_MsgQueueFO, &mensaje, 0U, 0U);
      
      osDelay(200); // Antirrebote
      while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) osDelay(10);
    }
    osDelay(50);
  }
}

static void Init_UserButton(void){

  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct_Button = {0};

  GPIO_InitStruct_Button.Pin = GPIO_PIN_13;

  GPIO_InitStruct_Button.Mode = GPIO_MODE_IT_RISING;

  GPIO_InitStruct_Button.Pull = GPIO_PULLDOWN;

  HAL_GPIO_Init(GPIOC,&GPIO_InitStruct_Button);

}



static void Init_LEDs(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7; // LED Azul
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
