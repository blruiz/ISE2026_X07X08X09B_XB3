#include "acelerometro.h"
#include <stdio.h>
#include <stdlib.h>

/* Definiciones de pines para el MPU6500 */
#define MPU6500_CS_PORT  GPIOD
#define MPU6500_CS_PIN   GPIO_PIN_14

#define MPU6500_MISO_PORT  GPIOA
#define MPU6500_MISO_PIN  GPIO_PIN_6

#define MPU6500_MOSI_PORT  GPIOB
#define MPU6500_MOSI_PIN  GPIO_PIN_5

/* Registros importantes del MPU6500 */
#define REG_READ_BIT     0x80 // MSB = 1 para lectura
#define REG_WHO_AM_I     0x75
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_XOUT_H 0x3B

#define S_TRANS_DONE_SPI  0x01 // Flag de evento para sincronizar la comunicación SPI con el hilo
#define FLAG_BUTTON        0x04 //Flag para despertar hilo del sensor con la pulsación del boton
#define FLAG_MOVIMIENTO_DETECTADO 0x08  

/*Definición de funciones*/
void MPU_reset(void); //Configuración inicial del hardware SPI y pines del MPU
void MPU_Init(void);  //Inicialización del sensor
void MPU_WriteReg(uint8_t reg, uint8_t value); //Enviar datos al esclavo
uint8_t MPU_ReadReg(uint8_t reg); //Leer datos del esclavo
int Init_ThMPU (void); //Inicialización del hilo del sensor
void SPI_Callback (uint32_t event); //

static void MPU_Read_burst(int16_t *x, int16_t *y, int16_t *z);

//static void initPINMPUReset(uint16_t GPIO_Pin);
static void initPINMPUcs(uint16_t GPIO_Pin);
void initTIMER7(void);
void delay(volatile uint32_t n_microsegundos);

/* Funciones para manejar la interrupción*/
void HAL_GPIO_EXTI_Callback(uint16_t pin);
void EXTI15_10_IRQHandler(void);
static void Init_UserButton(void);

/* Identificadores del RTOS */
osThreadId_t tid_ThMPU;                     // ID del hilo encargado del MPU

float gX,gY,gZ;
int16_t refX, refY, refZ;

// --- GESTIÓN DE BAJO NIVEL (SPI y HARDWARE) ---
// --- Variables de Control de Periféricos ---
extern ARM_DRIVER_SPI Driver_SPI1;	//Driver CMSIS para el bus SPI1
static ARM_DRIVER_SPI* SPIdrv = &Driver_SPI1;
static TIM_HandleTypeDef htim7; 		//Timer 7 para retardos precisos (microsegundos)

//Configuración inicial del hardware SPI y pines del MPU
void MPU_reset(void){
	/* Initialize the SPI driver */
	SPIdrv->Initialize(SPI_Callback);
	/* Power up the SPI peripheral */
	SPIdrv->PowerControl(ARM_POWER_FULL);
	/* Configure the SPI to Master, 8-bit mode @20000 kBits/sec */
	SPIdrv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_DATA_BITS(8), 1000000);
	/* SS line = INACTIVE = HIGH */
	SPIdrv->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	
	//INICIALIZACIÓN DE PINES
	initPINMPUcs(MPU6500_CS_PIN);
	HAL_GPIO_WritePin(GPIOD, MPU6500_CS_PIN, GPIO_PIN_SET); // IDLE: CS en alto
	delay(1000);

}
//Inicialiización del sensor
void MPU_Init(void) {
    // 1. Configurar el SPI y los Pines
    // (Asegúrate de que el SPI esté en Modo 3: CPOL=1, CPHA=1 como tenías en el LCD)
    MPU_reset(); 
    printf("\r\n--- MPU6500 Debug Start ---\r\n");
    // 2. Comprobar conexión
    uint8_t id = MPU_ReadReg(REG_WHO_AM_I);
	
    if (id == 0x70) {
			  printf("Conexion exitosa con el sensor.\r\n");
        // 3. Despertar el sensor (Sale de Sleep Mode)
        // Escribimos 0x01 en PWR_MGMT_1 para usar el reloj del giróscopo (más estable)
        MPU_WriteReg(REG_PWR_MGMT_1, 0x01);
				osDelay(2000); // Esperar a que el PLL se estabilice
			
				printf("Calibrando reposo MPU6500...\r\n");
        //Tomamos una medida inicial como referencia
        MPU_Read_burst(&refX, &refY, &refZ); 
			  printf("Valores de calibración: Acc X: %.2f g Acc Y: %.2f gAcc Z: %.2f g\r\n", (float)refX/16384.0f, (float)refY/16384.0f, (float)refZ/16384.0f);
    }else {
        printf("ERROR: No se detecta el sensor. Revisa el cableado.\r\n");
        while(1) osThreadYield(); // Detener si falla
    }
}
// Función para escribir en un registro
void MPU_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = { reg, value }; // MSB = 0 para escritura
    
    HAL_GPIO_WritePin(GPIOD, MPU6500_CS_PIN, GPIO_PIN_RESET); // CS Low
    SPIdrv->Send(data, 2);
    osThreadFlagsWait(S_TRANS_DONE_SPI, osFlagsWaitAny, osWaitForever);
    HAL_GPIO_WritePin(GPIOD, MPU6500_CS_PIN, GPIO_PIN_SET); // CS High
}

// Función para leer un registro
uint8_t MPU_ReadReg(uint8_t reg) {
    uint8_t tx[2] = { reg | REG_READ_BIT, 0x00 }; 
    uint8_t rx[2] = { 0, 0 };
    
    HAL_GPIO_WritePin(GPIOD, MPU6500_CS_PIN, GPIO_PIN_RESET); // CS Low
		// Enviamos dirección y recibimos dato en una sola operación full-duplex
    SPIdrv->Transfer(tx, rx, 2); 
    osThreadFlagsWait(S_TRANS_DONE_SPI, osFlagsWaitAny, osWaitForever);
    HAL_GPIO_WritePin(GPIOD, MPU6500_CS_PIN, GPIO_PIN_SET); // CS High
    
    return rx[1]; // El dato real viene en el segundo byte
}

void Accelerometer_Thread (void *argument) {
    MPU_Init();
		Init_UserButton();
    int16_t accX, accY, accZ;
	  float threshold = 0.40f;  // Umbral de 0.25g para detectar movimiento
	  uint32_t flags;
	
    while (1) {
				printf("Sistema en espera. Pulsa el BOTON AZUL para medir...\r\n");
        
        // El hilo se duerme hasta que pulses el botón
        flags = osThreadFlagsWait(0x04, osFlagsWaitAny, osWaitForever);
			  
			  if (flags&0x04){
					MPU_Read_burst(&accX, &accY, &accZ);
					// 2. Bucle de detección de movimiento (ej: durante 5 seg)
					int i = 0;
					uint32_t mov_detectado = false;
					while( (i < 10)&!(mov_detectado) ) { 
						MPU_Read_burst(&accX, &accY, &accZ);

						// Calculamos la diferencia en "g"
						float diffX = (float)abs(accX - refX) / 16384.0f;
						float diffY = (float)abs(accY - refY) / 16384.0f;
						float diffZ = (float)abs(accZ - refZ) / 16384.0f;

						if (diffX > threshold || diffY > threshold || diffZ > threshold) {
							printf("¡MOVIMIENTO DETECTADO! Gx:%.2f Gy:%.2f Gz:%.2f\r\n", diffX, diffY, diffZ);
							osThreadFlagsSet(tid_ThMPU, FLAG_MOVIMIENTO_DETECTADO);
							mov_detectado = true;
						 }
						 i++;
						osDelay(500); // Muestreo cada 500ms
					}
					if (!mov_detectado) printf("Vigilancia terminada: No hubo movimiento.\r\n");
				}
				osDelay(500);
		}
}

//Inicializa y crea el hilo del MPU
int Init_ThMPU (void) {  
	// Lanzamos el hilo que gestionará la pantalla
	tid_ThMPU = osThreadNew(Accelerometer_Thread, NULL, NULL);
  
	if (tid_ThMPU == NULL) {
    return(-1);
  }
 
  return(0);
}

void SPI_Callback (uint32_t event){
    switch (event)
    {
    case ARM_SPI_EVENT_TRANSFER_COMPLETE:
        /* Success: Wakeup Thread */
        osThreadFlagsSet(tid_ThMPU, S_TRANS_DONE_SPI); 
        break;
    
    case ARM_SPI_EVENT_DATA_LOST:
        /*  Occurs in slave mode when data is requested/sent by master
            but send/receive/transfer operation has not been started
            and indicates that data is lost. Occurs also in master mode
            when driver cannot transfer data fast enough. */
        break;
    
    case ARM_SPI_EVENT_MODE_FAULT:
        /*  Occurs in master mode when Slave Select is deactivated and
            indicates Master Mode Fault. */
        break;
    }
}

void MPU_Read_burst(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer_rx[6];
    uint8_t reg_addr = REG_ACCEL_XOUT_H | 0x80;
	
		// Lectura en ráfaga (Burst read) de 6 bytes (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
    HAL_GPIO_WritePin(GPIOD, MPU6500_CS_PIN, GPIO_PIN_RESET);
    SPIdrv->Send(&reg_addr, 1); // Enviamos dirección inicial
    osThreadFlagsWait(S_TRANS_DONE_SPI, osFlagsWaitAny, osWaitForever);
     SPIdrv->Receive(buffer_rx, 6); // Recibimos los 6 bytes de golpe
    osThreadFlagsWait(S_TRANS_DONE_SPI, osFlagsWaitAny, osWaitForever);
    HAL_GPIO_WritePin(GPIOD, MPU6500_CS_PIN, GPIO_PIN_SET);

    // Reconstrucción de los datos (Unir High byte y Low byte)
    *x = (int16_t)((buffer_rx[0] << 8) | buffer_rx[1]);
    *y = (int16_t)((buffer_rx[2] << 8) | buffer_rx[3]);
    *z = (int16_t)((buffer_rx[4] << 8) | buffer_rx[5]);

//		// Con sensibilidad por defecto (+/- 2g), 16384 LSB = 1g
//    gX = (float)*x / 16384.0f;
//		gY = (float)*y / 16384.0f;
//		gZ = (float)*z / 16384.0f;
//	 printf("Acc X: %.2f g Acc Y: %.2f gAcc Z: %.2f g\r\n", gX, gY, gZ);
}
	
// --- FUNCIONES DE CONTROL DE PINES (GPIO) ---
//Configuración del pin de RESET del LCD (PA6).
//static void initPINMPUReset(uint16_t GPIO_Pin){
//		GPIO_InitTypeDef GPIO_InitStruct = {0};
//		__HAL_RCC_GPIOA_CLK_ENABLE();
//    //Configuracion de los pines
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;// Salida digital estándar
//    GPIO_InitStruct.Pin = GPIO_Pin;
//    //Inicializacion de los pines
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//}

static void initPINMPUcs(uint16_t GPIO_Pin){
	
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		
		__HAL_RCC_GPIOD_CLK_ENABLE();
    //Configuracion de los pines
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pin = GPIO_Pin;

    //Inicializaci?n de los pines
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

//DELAY
/**
  * @brief  Configura el delay
	* @param  n_microsegundos -> tiempo establecido para realizar la pausa del delay; su valor se acota entre: 0-65534
  * @retval None
  */
void delay(volatile uint32_t n_microsegundos)
{
	initTIMER7();
	
	//Iniciamos el timer 7
  HAL_TIM_Base_Start(&htim7);
 
	//Esperamos a que se consuma el tiempo que hemos establecido en segundos
	while( __HAL_TIM_GET_COUNTER(&htim7) < n_microsegundos ){
		if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE)) {
            __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
    } 
  }
	//Paramos el timer 7
	HAL_TIM_Base_Stop(&htim7);
	//Reiniciamos el contador del timer 7
	__HAL_TIM_SET_COUNTER(&htim7, 0);
}

/**
 * @brief Inicializa el Timer 7 para contar microsegundos.
 * Configuración: Bus a 84MHz / Prescaler 83 = 1 tick por microsegundo.
 */
void initTIMER7(void){
	__HAL_RCC_TIM7_CLK_ENABLE();								// Habilitar reloj del periférico
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;									// Frecuencia = 84MHz / (83(valor prescaler) + 1) = 1 MHz
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0xFFFF;									// Valor máximo de cuenta (16 bits)
  HAL_TIM_Base_Init(&htim7);
}

// --- GESTIÓN DE INTERRUPCIONES ---
static void Init_UserButton(void){
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct_Button = {0};
  GPIO_InitStruct_Button.Pin = GPIO_PIN_13;
  GPIO_InitStruct_Button.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct_Button.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC,&GPIO_InitStruct_Button);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0); // Prioridad segura para RTOS
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void EXTI15_10_IRQHandler(void){
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin){
  if(pin==GPIO_PIN_13){
    // Enviamos el FLAG para despertar al hilo del sensor
    osThreadFlagsSet(tid_ThMPU, 0x04); // Usamos el bit 2 (0x04) para el botón
  }
}
