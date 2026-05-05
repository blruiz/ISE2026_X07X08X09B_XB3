/**
 * @file infrarrojo.c
 * @brief Driver para el sensor de proximidad APDS9960 integrado con CMSIS-RTOS2.
 * 
 * Este módulo gestiona la detección de objetos por infrarrojos.
 * Utiliza interrupciones físicas para minimizar el uso de CPU y un Timer del SO
 * para controlar los tiempos de respuesta del jugador.
 */

#include "infrarrojo.h"
#include "stdio.h"

/* --- Configuración de Hardware --- */
#define APDS9960_I2C_ADDR       0x39  //Direccion I2C de 7 bits
#define SENSOR_INT_PIN GPIO_PIN_12

/* --- Mapa de Registros del APDS9960 --- */
#define REG_ENABLE     0x80  // Control de encendido y activacion de funciones (Power, proximidad, etc.)
#define REG_PILT       0x89  // Umbral bajo de proximidad (dispara interrupción si PDATA < PILT). Determina el nivel para detectar que se ha ido
#define REG_PIHT       0x8B  // Umbral alto de proximidad (dispara interrupción si PDATA > PIHT)
#define REG_PDATA      0x9C  // Lectura directa de la cantidad de luz IR reflejada (0-255)
#define REG_PICLEAR    0xE5  // Limpiar interrupción de proximidad
#define REG_WAIT_TIME  0x83  // Tiempo de reposo entre ciclos de medida (ahorro de energía)
#define REG_PERSIST    0x8C	 // Filtro de ruido: número de lecturas seguidas fuera de umbral para disparar INT
#define REG_CONTROL    0x8F  // Configuración analógica: Ganancia y potencia del LED IR
#define REG_CONFIG2    0x90  // Configuración avanzada del LED (Boost)
#define REG_PPULSE     0x8E  // Configuración de los pulsos IR (cantidad y duración)

/* --- Máscaras de Bits (Registro ENABLE) --- */
#define ENABLE_PON              (1 << 0) // Encendido general del chip (Power ON)(bit 0 a '1')
#define ENABLE_PEN              (1 << 2) // Activación del motor de proximidad (bit 2 a '1')
#define WAIT_ENABLE							(1 << 3) // Activacion del temporizador interno entre medidas (bit 3 a '1')
#define ENABLE_PIEN             (1 << 5) // Permite que el sensor baje el pin INT al detectar algo (bit 5 a '1')

/* --- Flags de Comunicación entre Hilos (Eventos) --- */
#define FLAG_MOV_DETECT 			0x01	// El sensor ha detectado movimiento
#define FLAG_I2C_DONE         0x02  // La transferencia I2C ha finalizado (Callback)
#define FLAG_START_DETECTION  0x04  // Flag que se manda desde el juego
#define FLAG_TIMEOUT          0x08  // Flag que enviará el Timer si no hay respuesta

/* --- Variables de Control de RTOS --- */
extern ARM_DRIVER_I2C Driver_I2C1;	// Driver CMSIS-Driver para I2C
osThreadId_t tid_ThIR;							// Identificador del hilo del sensor
volatile uint32_t I2C_Event = 0;		// Almacena el resultado de la última operación I2C

/*Funciones principales*/
int IR_Init(void); 										//Inicializacion del sensor IR
void IR_Thread(void *argument);				//Hilo con la funcionalidad principal del sensor IR
int Init_ThIR(void);									//Inicialización del hilo del sensor IR
void Init_INT_signal(void);						//Inicializacion de la interrupcion fisica del sensor
void I2C_SignalEvent(uint32_t event);	//Callback de la ocurrencia de cualquier evento I2C

/*Timer*/
osTimerId_t timer_id; // ID del Timer
void Timer_Callback(void *argument);

/* Funciones para manejar la interrupción del botón- QUITAR LUEGO*/
void HAL_GPIO_EXTI_Callback(uint16_t pin);
void EXTI15_10_IRQHandler(void);
static void Init_UserButton(void);


/**

 * @brief Inicializa el sensor APDS9960 para detección por interrupción.
 * @return 0: Éxito, -1: Fallo de hardware o ID incorrecto.

 */

int IR_Init(void) {
    uint8_t id_reg = 0x92; // Registro de ID del dispositivo
    uint8_t chip_id = 0;

    // 1. Configuración del Driver I2C (Capa de abstracción de hardware)
    Driver_I2C1.Initialize(I2C_SignalEvent); 
    Driver_I2C1.PowerControl(ARM_POWER_FULL);
    Driver_I2C1.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
		osDelay(10);

    // 2. Verificación de Identidad: Comprobamos si el sensor está en el bus
   int32_t status =  Driver_I2C1.MasterTransmit(APDS9960_I2C_ADDR, &id_reg, 1, true);
		if (status != ARM_DRIVER_OK) {
				return -1;
		}

    osThreadFlagsWait(FLAG_I2C_DONE, osFlagsWaitAny, 100); 
    Driver_I2C1.MasterReceive(APDS9960_I2C_ADDR, &chip_id, 1, false);
    osThreadFlagsWait(FLAG_I2C_DONE, osFlagsWaitAny, 100);

    if (chip_id != 0xAB) { //el id del chip es AB (Datasheet)
        return -1; // Error: No se encuentra el APDS9960
    }

    // 3. Configuración de registros
    uint8_t config_data[][2] = {
    {REG_WAIT_TIME, 0xAF},	// Espera ~200ms: Reduce consumo y evita detecciones fantasmas
    {REG_PILT, 0x0}, 				// Umbral mínimo: 0
    {REG_PIHT, 0x32},				// Umbral máximo: 50 (se dispara al superar este valor)
    {REG_PERSIST, 0x11},		// Requiere 2 lecturas consecutivas OK para evitar ruido
    {REG_CONTROL, 0x48}, 		// Potencia LED 50mA y Ganancia 4x: Rango medio de detección
    {REG_CONFIG2, 0x11}, 		// LED Boost 100%: Mejora la potencia de los pulsos
    {REG_PPULSE, 0x83},  		// 4 pulsos de 16us
    // ENABLE: PON(1), PEN(4), PIEN(32), WEN(8) -> 0x2D
    {REG_ENABLE, ENABLE_PON | ENABLE_PEN | ENABLE_PIEN | WAIT_ENABLE}
    };


    for(int i=0; i<8; i++) {
			I2C_Event = 0; // Limpiar variable volátil
			status = Driver_I2C1.MasterTransmit(APDS9960_I2C_ADDR, config_data[i], 2, false);
			if (status != ARM_DRIVER_OK) return -1;
			osThreadFlagsWait(FLAG_I2C_DONE, osFlagsWaitAny, 100); 
    }

   // 4. Limpieza inicial: El pin INT se queda bajo hasta que se limpie manualmente
    uint8_t clear_cmd = REG_PICLEAR;
    Driver_I2C1.MasterTransmit(APDS9960_I2C_ADDR, &clear_cmd, 1, false);
    osThreadFlagsWait(FLAG_I2C_DONE, osFlagsWaitAny, 50);
		
    return 0;
}


/**
 * @brief Inicializa el hilo del sensor
 */
int Init_ThIR(void){
	tid_ThIR = osThreadNew(IR_Thread, NULL, NULL);
	if (tid_ThIR == NULL) {
			return(-1);
		}
		return(0);
}


/**
 * @brief Hilo prinicipal del sensor
 */

void IR_Thread(void *argument) {
    uint8_t prox_data;
	  uint32_t flags;
    uint8_t reg_pdata = REG_PDATA;
    uint8_t clear_cmd = REG_PICLEAR;

    // Inicializamos el hardware
    if (IR_Init() != 0) {
        printf("Error: Sensor no encontrado\n");
        osThreadTerminate(osThreadGetId()); // O manejo de error
    }
		
		Init_INT_signal(); // Configura PF12 y EXTI
		Init_UserButton();
		// Creamos el Timer: Tipo one-shot
    timer_id = osTimerNew(Timer_Callback, osTimerOnce, NULL, NULL);
		
    while(1) {
				// 1. ESPERAR ORDEN DE INICIO DEL JUEGO
        printf("Esperando FLAG_START_DETECTION...\n");
        osThreadFlagsWait(FLAG_START_DETECTION, osFlagsWaitAny, osWaitForever);
				osDelay(100);
				// 2. INICIAR CRONÓMETRO
        osTimerStart(timer_id, 1000U); 
        printf("Busqueda de objeto iniciada...\n");
			
        // 3. ESPERAR DETECCIÓN O TIMEOUT
        // Esperamos tanto el flag físico INT (F12) como el del timer
        flags = osThreadFlagsWait(FLAG_MOV_DETECT | FLAG_TIMEOUT, osFlagsWaitAny, osWaitForever);

        if (flags & FLAG_MOV_DETECT) {
            osTimerStop(timer_id); // Detenemos el reloj porque ya encontramos algo
            
					 // Leemos el valor de proximidad real para confirmar la distancia
            Driver_I2C1.MasterTransmit(APDS9960_I2C_ADDR, &reg_pdata, 1, true);
            osThreadFlagsWait(FLAG_I2C_DONE, osFlagsWaitAny, 20);
            Driver_I2C1.MasterReceive(APDS9960_I2C_ADDR, &prox_data, 1, false);
            osThreadFlagsWait(FLAG_I2C_DONE, osFlagsWaitAny, 20);

            printf("Exito! Objeto detectado. Valor: %d\n", prox_data);
						osDelay(100);
						
            // Aquí enviamos un flag o mensaje a la Lógica de Juego
        } 
        else if (flags & FLAG_TIMEOUT) {
            printf("Timeout: No se detecto nada en el tiempo previsto.\n");
            osDelay(100);
            // Aquí enviamos el flag a la Lógica de Juego 
        }

        // LIMPIAR INTERRUPCIÓN: Para que INT vuelva a subir a 3.3V
        Driver_I2C1.MasterTransmit(APDS9960_I2C_ADDR, &clear_cmd, 1, false);
        osThreadFlagsWait(FLAG_I2C_DONE, osFlagsWaitAny, 20);
        osDelay(100);
				osThreadFlagsClear(FLAG_START_DETECTION); // Limpieza manual por seguridad
    }
}


// --- GESTIÓN DE INTERRUPCIONES ---
/**

 * @brief Configuramos la interrupción físisca del sensor

 */
static void Init_INT_signal(void){
  __HAL_RCC_GPIOF_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct_INT = {0};
  GPIO_InitStruct_INT.Pin = GPIO_PIN_12;
  GPIO_InitStruct_INT.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct_INT.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF,&GPIO_InitStruct_INT);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0); // Prioridad segura para RTOS
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


/**

 * @brief Callback de interrupción GPIO (Llamado por HAL_GPIO_EXTI_Callback)

 */
volatile uint32_t debug_count = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == SENSOR_INT_PIN) {
      __HAL_GPIO_EXTI_CLEAR_IT(SENSOR_INT_PIN);  //¿esto hace falta o lo hace el halder?
			osThreadFlagsSet(tid_ThIR, FLAG_MOV_DETECT);
    }
		
		//Callback del botón- QUITAR LUEGO
		if(GPIO_Pin==GPIO_PIN_13){
			__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
			debug_count++; // Si esto sube solo, es ruido físico
    // Enviamos el FLAG para despertar al hilo del timer
    osThreadFlagsSet(tid_ThIR, FLAG_START_DETECTION); 	
  }
}


/**
 * @brief Callback del Driver I2C.
 * Avisa al hilo de que los datos ya están en el bus o la transmisión terminó.
 */
void I2C_SignalEvent(uint32_t event) {
    I2C_Event = event;
    osThreadFlagsSet(tid_ThIR, FLAG_I2C_DONE);

}

void Timer_Callback(void *argument) {
    // Si el tiempo se agota, avisamos al hilo del IR
    osThreadFlagsSet(tid_ThIR, FLAG_TIMEOUT);
}

// --- GESTIÓN DE INTERRUPCIONES BOTON- SOLO PARA PRUEBAS---
static void Init_UserButton(void){
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct_Button = {0};
  GPIO_InitStruct_Button.Pin = GPIO_PIN_13;
  GPIO_InitStruct_Button.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct_Button.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC,&GPIO_InitStruct_Button);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0); // Prioridad segura para RTOS
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/*Esta función está definid en el archivo stm32f4xx_it.c*/
//void EXTI15_10_IRQHandler(void){
//  // Esta función de la HAL limpia los flags de hardware por ti
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
//}
