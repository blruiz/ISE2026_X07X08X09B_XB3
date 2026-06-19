
#include "RTC.h"
#include "SNTP.h"

#define FLAG_ALARM_RTC 0x01 // Flag para avisar de la alarma

osThreadId_t tid_ThRTC;                     // ID del hilo encargado del RTC
osThreadId_t tid_ThLED;                  // ID del hilo encargado del LED de la alarma

/* RTC handler declaration */
RTC_HandleTypeDef RtcHandle = {0};

/* Buffers used for displaying Time and Date */
uint8_t aShowTime[20] = {0};
uint8_t aShowDate[20] = {0};
//static MSGQUEUE_OBJ_tLCD LCDmsg; // Objeto para enviar mensajes a la cola del LCD


/* Private function prototypes -----------------------------------------------*/
static void RTC_CalendarConfig(void);
static void RTC_CalendarShow(uint8_t *showtime, uint8_t *showdate);
//static void LED_Initialize(void);
//void RTC_AlarmConfig(void);

static void Error_Handler(void);
void init_RTC(void);
//int Init_ThLCD (void);
void Thread_RTC (void *argument);
//void Thread_LEDs(void *argument);

//Inicializa  y crea el hilo del RTC
int Init_ThRTC (void) {
  
  // Lanzamos el hilo que gestionará la pantalla
	tid_ThRTC = osThreadNew(Thread_RTC, NULL, NULL);
//  tid_ThLED = osThreadNew(Thread_LEDs, NULL, NULL);/// Hilo de los LEDs
	if (tid_ThRTC== NULL || tid_ThLED == NULL) {
    return(-1);
  }
 
  return(0);
}

//Hilo principal de gestión del LCD
void Thread_RTC (void *argument) {
  init_RTC();
  Init_ThSNTP ();
//  LED_Initialize();
  while(1){
    RTC_CalendarShow(aShowTime, aShowDate);
  }
}
  
void init_RTC(){
  /* CONFIGURACIÓN ESPECÍFICA DEL RELOJ PARA EL RTC */
   /* Desbloqueo del Dominio de Backup (Según tu manual HAL) --- */
  // 1. Activar el reloj del Power Controller (PWR)
  __HAL_RCC_PWR_CLK_ENABLE();

  // 2. Habilitar el acceso a los registros del RTC y Backup
  HAL_PWR_EnableBkUpAccess();

  // 3. SELECCIONAR EL RELOJ 
  // Vamos a forzar el uso del LSI (oscilador interno) porque el LSE (externo) 
  // puede no estar presente o tardar mucho en estabilizarse.
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler(); // Si falla aquí, el chip tiene un problema de reloj interno
  }

  // 4. Conectar el reloj seleccionado al RTC
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }

  // 5. Finalmente, activar el reloj del periférico RTC
  __HAL_RCC_RTC_ENABLE();
  
  /* --- Configuración del Handle --- */
  RtcHandle.Instance = RTC;
    __HAL_RTC_RESET_HANDLE_STATE(&RtcHandle); // 1. Limpiamos el estado para asegurar que MspInit se ejecute

  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;//Format 24
  // Valores específicos para LSI (32 KHz aprox) para conseguir 1Hz de base
  // Si no pones estos valores, el reloj irá muy rápido o muy lento
  RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;//Value according to source clock
  RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;//Value according to source clock
  
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;//Output Disable
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;//High Polarity  
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;//Open Drain 
  
    
  /* Inicialización --- */
  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
 
  /*##-2- Check if Data stored in BackUp register1: No Need to reconfigure RTC#*/
  /* Read the Back Up Register 1 Data */
  if (HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR1) != 0x32F2)
  {
    /* Configure RTC Calendar */
    RTC_CalendarConfig();
  }
//  RTC_AlarmConfig();//activamos la alarma
}

/**
  * @brief  Configure the current time and date.
  * @param  None
  * @retval None
  */
static void RTC_CalendarConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /*##-1- Configure the Date #################################################*/
  /* Set Date: Wednesday March 4th 2026 */
  sdatestructure.Year = 0x14;
  sdatestructure.Month = RTC_MONTH_MARCH;
  sdatestructure.Date = 0x12;
  sdatestructure.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the Time #################################################*/
  /* Set Time: 16:00:00 */
  stimestructure.Hours = 0x16;
  stimestructure.Minutes = 0x00;
  stimestructure.Seconds = 0x45;
  stimestructure.TimeFormat = RTC_HOURFORMAT_24;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
    if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BCD) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
    
  /*##-3- Writes a data in a RTC Backup data Register1 #######################*/
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, 0x32F2);
}

/**
  * @brief  Display the current time and date.
  * @param  showtime : pointer to buffer
  * @param  showdate : pointer to buffer
  * @retval None
  */
static void RTC_CalendarShow(uint8_t *showtime, uint8_t *showdate)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
    sprintf((char *)showtime, "%02d:%02d:%02d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
//    strcpy(LCDmsg.text, (char *)showtime);
//    LCDmsg.line = 0;
//    osMessageQueuePut(mid_MsgQueueLCD, &LCDmsg, 0, 0);
//  /* Display date Format : mm-dd-yy */
    sprintf((char *)showdate, "%02d-%02d-%02d", sdatestructureget.Month, sdatestructureget.Date, 2000 + sdatestructureget.Year);
//    strcpy(LCDmsg.text, (char *)showdate);
//    LCDmsg.line = 1;
//    osMessageQueuePut(mid_MsgQueueLCD, &LCDmsg, 0, 0);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED3 on */
//  strcpy(LCDmsg.text, "ERROR");
//    LCDmsg.line = 1;
//    osMessageQueuePut(mid_MsgQueueLCD, &LCDmsg, 0, 0);
//  while (1)
  {
  }
}

/**
  * @brief  Configura la alarma para que salte cada minuto
  */
//static void RTC_AlarmConfig(void) {
//  RTC_AlarmTypeDef salarmstructure;

//  // Se activa cuando los segundos son 00 (cada minuto)
//  salarmstructure.AlarmTime.Seconds = 0x00;
//  salarmstructure.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  salarmstructure.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  salarmstructure.AlarmMask = RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES | RTC_ALARMMASK_DATEWEEKDAY;//enmascaramos en resto de campos para que solo tenga en cuenta los segundos
//  salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
//  salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
//  salarmstructure.Alarm = RTC_ALARM_A; //Seleccionamos la alarma A del RTC
//  
//  //HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 5, 0);
//  HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
//  
//  HAL_RTC_SetAlarm_IT(&RtcHandle, &salarmstructure, RTC_FORMAT_BCD); // Enable the alarm interrupt
//  
//}
//// Callback de la alarma
//void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
//    osThreadFlagsSet(tid_ThLED, FLAG_ALARM_RTC); // Enviamos una señal al hilo que controla los LEDs 
//}

//void LED_Initialize(void){
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  GPIO_InitTypeDef GPIO_InitStruct = {0};

//  // Configuración del LED
//  GPIO_InitStruct.Pin = GPIO_PIN_0;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//}

//void Thread_LEDs(void *argument) {
//    while(1) {
//      // 1. El hilo se queda dormido aquí hasta que el RTC mande el Flag 0x01
//      osThreadFlagsWait(FLAG_ALARM_RTC, osFlagsWaitAny, osWaitForever);
//      
//      // 2. Al recibir el flag, parpadeamos el LED VERDE (LD1 - PB0) durante 5 segundos
////      uint32_t start_time = osKernelGetTickCount();
////      
////      // Bucle de 5000ms (5 segundos)
//      for (int i = 0; i< 10; i++){
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Conmutar LED Verde
//        osDelay(250);
//         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//        osDelay(250);
//      }
//     // 3. Aseguramos que el LED quede apagado al terminar los 5s
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//   }
//}

