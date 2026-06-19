#include "rtc.h"
#include "rl_net.h"
#include "cmsis_os2.h"

#define DNS_SERVER_SNTP "time.google.com"

extern RTC_HandleTypeDef RtcHandle;
osThreadId_t tid_Th_SNTP;

static const int mdays[12]={31,28,31,30,31,30,31,31,30,31,30,31};

static void rtc_set_local_from_epoch(uint32_t epoch_utc);
static void epoch_to_ymdhms(uint32_t t, int *Y,int *M,int *D,int *h,int *m,int *s);
static bool is_leap(int y);
int dow(int y, int m, int d);
int is_dst_eu(int Y, int M, int D, int h);
//static void Init_UserButton(void);
//void EXTI15_10_IRQHandler(void);
//void HAL_GPIO_EXTI_Callback(uint16_t pin);

void Thread_Ntp(void *argument);
//static void LED_Rojo_Parapadeo(void);
/*Configuracion stack hilo sntp*/
//static const osThreadAttr_t atributo_hilo1 = {
//  .stack_size = 256
//};

int Init_ThSNTP (void) {
  
  tid_Th_SNTP = osThreadNew(Thread_Ntp,  NULL, NULL); //Hilo del SNTP
  if (tid_Th_SNTP== NULL ) {
    return(-1);
  }
 
  return(0);
}

/* ---------- Aplicar zona horaria Madrid al epoch UTC ---------- */
static void rtc_set_local_from_epoch(uint32_t epoch_utc){
  int Y,M,D,h,m,s;
  epoch_to_ymdhms(epoch_utc,&Y,&M,&D,&h,&m,&s);

  int dst=is_dst_eu(Y,M,D,h);
  int offset = dst?2:1;

  epoch_to_ymdhms(epoch_utc + offset*3600,&Y,&M,&D,&h,&m,&s);

  RTC_TimeTypeDef t={0};
  RTC_DateTypeDef d={0};

  t.Hours=h; t.Minutes=m; t.Seconds=s;
  d.Year = (Y>=2000)?(Y-2000):(Y-1900);
  d.Month=M; d.Date=D;
  d.WeekDay=dow(Y,M,D);

  HAL_RTC_SetTime(&RtcHandle,&t,RTC_FORMAT_BIN);
  HAL_RTC_SetDate(&RtcHandle,&d,RTC_FORMAT_BIN);
}


static void epoch_to_ymdhms(uint32_t t, int *Y,int *M,int *D,int *h,int *m,int *s){
  uint32_t secs=t;
  *s=secs%60; secs/=60;
  *m=secs%60; secs/=60;
  *h=secs%24;
  uint32_t days=secs/24;

  int year=1970;
  while(1){
    uint32_t dy=is_leap(year)?366:365;
    if(days>=dy){ days-=dy; year++; }
    else break;
  }
  int month=0;
  for(int i=0;i<12;i++){
    int dm = mdays[i] + ((i==1 && is_leap(year))?1:0);
    if(days>=dm){ days-=dm; month++; }
    else break;
  }
  *Y=year; *M=month+1; *D=days+1;
}

// Año bisiesto
bool is_leap(int y) {
  return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}

// Día de la semana 0..6 (Dom..Sáb) — Sakamoto
int dow(int y, int m, int d) {
  static int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
  if (m < 3) y -= 1;
  return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

// Regla DST UE (CET/CEST) — desde último domingo de marzo 01:00 UTC hasta
// último domingo de octubre 01:00 UTC.
int is_dst_eu(int Y, int M, int D, int h) {
  if (M > 3 && M < 10) return 1;
  if (M < 3 || M > 10) return 0;

  // Último domingo de marzo
  if (M == 3) {
    int wd = dow(Y, 3, 31);
    int last_sun = 31 - ((wd + 6) % 7);
    if ((D > last_sun) || (D == last_sun && h >= 1)) return 1;
    return 0;
  }
  // Último domingo de octubre
  if (M == 10) {
    int wd = dow(Y, 10, 31);
    int last_sun = 31 - ((wd + 6) % 7);
    if ((D < last_sun) || (D == last_sun && h < 1)) return 1;
    return 0;
  }
  return 0;
}

void Thread_Ntp(void *argument){
//    Init_UserButton();
//    osDelay(5000);
  while (1) {
    uint32_t sec=0, frac=0;
    netStatus st = netSNTPc_GetTimeX(DNS_SERVER_SNTP, &sec, &frac);  // Google SNTP
    
    if(st==netOK && sec!=0){
      rtc_set_local_from_epoch(sec);
//      LED_Rojo_Parapadeo();
    }
    osDelay(176000);   // 3 minutos - los 4 segundos de parpadeo del LED rojo
  }
}
