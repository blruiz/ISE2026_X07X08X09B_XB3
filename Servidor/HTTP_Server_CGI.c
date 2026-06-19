/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::Network:Service
 * Copyright (c) 2004-2018 ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    HTTP_Server_CGI.c
 * Purpose: HTTP Server CGI Module
 * Rev.:    V6.0.0
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "rl_net.h"                     // Keil.MDK-Pro::Network:CORE
#include "memoria.h"
#include "usart.h"
//#include "Board_LED.h"                  // ::Board Support:LED

#if      defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma  clang diagnostic push
#pragma  clang diagnostic ignored "-Wformat-nonliteral"
#endif

// http_server.c
/* Referencias externas a funciones y variables de otros módulos */
//extern void LED_SetOut (uint8_t val);
//extern bool LEDrun;

//extern uint16_t AD_in (uint32_t ch);
extern uint8_t aShowTime[20] ;
extern uint8_t aShowDate[20] ;

//extern char lcd_text[2][20+1]; // Buffer compartido para el texto del LCD
//extern osThreadId_t TID_Display;
//extern uint8_t  get_button (void);

// Local variables.
//static uint8_t P2; // Variable para almacenar el estado de los LEDs (8 bits)
static uint8_t ip_addr[NET_ADDR_IP6_LEN];
static char    ip_string[40];

// My structure of CGI status variable.
typedef struct {
  uint8_t idx;
  uint8_t unused[3];
} MY_BUF;
#define MYBUF(p)        ((MY_BUF *)p)

/*JUEGO*/
char player_name_global[21];
char accion_go[21];
//char estado_juego[21];
uint16_t puntos;

/* --- INTEGRACIÓN CON EEPROM --- */
//static const uint8_t num_scores = 10;
bool ranking_actualizado = false; 
extern osThreadId_t tid_formateo;
#define FLAG_FORMATEO 0x20
#define FLAG_ACTUALIZAR 0X40

void leerScore(uint8_t index, char *nombre, uint8_t *puntos_ret, char *fecha);
/* RTC+CONSUMO */
uint8_t consumo;

//PRUEBA
//extern osThreadId_t tid_prueba;
//extern void Thread_prueba (void *arg);
//#define FLAG_PRUEBA 0x80

// --- RECEPCIÓN DE DATOS (Del Navegador al STM32) ---/**

/**
 * @brief Procesa las peticiones GET para configurar parámetros de red
 * @param qstr: Cadena de consulta (query string) recibida por el servidor.
 */
void netCGI_ProcessQuery (const char *qstr) {
  netIF_Option opt = netIF_OptionMAC_Address;
  int16_t      typ = 0;
  char var[40];

  do {
    // Loop through all the parameters
    qstr = netCGI_GetEnvVar (qstr, var, sizeof (var));//extrae los parámetros de la URL uno a uno
    // Check return string, 'qstr' now points to the next parameter
		
    switch (var[0]) {		// Identificación del parámetro (i=IP, m=Mask, g=Gateway, p/s=DNS)
      case 'i': // Local IP address
        if (var[1] == '4') { opt = netIF_OptionIP4_Address;       }
        else               { opt = netIF_OptionIP6_StaticAddress; }
        break;

      case 'm': // Local network mask
        if (var[1] == '4') { opt = netIF_OptionIP4_SubnetMask; }
        break;

      case 'g': // Default gateway IP address
        if (var[1] == '4') { opt = netIF_OptionIP6_DefaultGateway; }
        else               { opt = netIF_OptionIP6_DefaultGateway; }
        break;

      case 'p': // Primary DNS server IP address
        if (var[1] == '4') { opt = netIF_OptionIP4_PrimaryDNS; }
        else               { opt = netIF_OptionIP6_PrimaryDNS; }
        break;

      case 's': // Secondary DNS server IP address
        if (var[1] == '4') { opt = netIF_OptionIP4_SecondaryDNS; }
        else               { opt = netIF_OptionIP6_SecondaryDNS; }
        break;
      
      default: var[0] = '\0'; break;
    }

    switch (var[1]) { // Identificación de la familia de direcciones (IPv4 o IPv6)
      case '4': typ = NET_ADDR_IP4; break;
      case '6': typ = NET_ADDR_IP6; break;

      default: var[0] = '\0'; break;
    }
		
		// Si el formato es correcto (ej: "i4=192.168.0.1"), aplicamos el cambio
    if ((var[0] != '\0') && (var[2] == '=')) {
      netIP_aton (&var[3], typ, ip_addr); // Convierte string ASCII a binario IP
      // Set required option
      netIF_SetOption (NET_IF_CLASS_ETH, opt, ip_addr, sizeof(ip_addr)); // Aplica al hardware
    }
  } while (qstr);
}

// Process data received by POST request.
// Type code: - 0 = www-url-encoded form data.
//            - 1 = filename for file upload (null-terminated string).
//            - 2 = file upload raw data.
//            - 3 = end of file upload (file close requested).
//            - 4 = any XML encoded POST data (single or last stream).
//            - 5 = the same as 4, but with more XML data to follow.
void netCGI_ProcessData (uint8_t code, const char *data, uint32_t len) {
  char var[40],passw[12];

  if (code != 0) { // Solo procesamos datos tipo url-encoded
    // Ignore all other codes
    return;
  }
  passw[0] = 1;
  do {
    // Parse all parameters
    data = netCGI_GetEnvVar (data, var, sizeof (var));
    if (var[0] != 0) { // First character is non-null, string exists
			// Actualización del texto del Nombre
      if (strncmp(var, "player_name=", 12) == 0) {
				strcpy(player_name_global, var + 12);   // Guardar nombre
				// Aquí puedes lanzar un evento, cambiar pantalla, etc.
				//    netCGI_SetResponse("juego.htm");   // Cambiar a la pantalla del juego
			}
      else if (strcmp(var, "play=Jugar") == 0) {
                     strcpy(accion_go, "");
          enviarDatos(INI_JUEGO, 0x00);
//           printf("INICIO JUEGO");
//          osThreadFlagsSet(tid_prueba, FLAG_PRUEBA);
          ranking_actualizado = false;
        }
      else if (strcmp(var, "rst=Reset") == 0) { 
          osThreadFlagsSet(tid_formateo, FLAG_FORMATEO);
          ranking_actualizado = false; 
        }
      }
  } while (data);
//  LED_SetOut (P2); // Aplicamos el estado de los LEDs al hardware
}
uint32_t step;
// Generate dynamic web data from a script line.
// Sustituye etiquetas especiales en el HTML por datos en tiempo real.
uint32_t netCGI_Script (const char *env, char *buf, uint32_t buflen, uint32_t *pcgi) {
//  int32_t socket;
//  netTCP_State state;
//  NET_ADDR r_client;
  uint32_t len = 0U;
//  uint8_t id;
  netIF_Option opt = netIF_OptionMAC_Address;
  int16_t      typ = 0;

  switch (env[0]) {
    // Analyze a 'c' script line starting position 2
    case 'a' :
      // Network parameters from 'network.cgi'
      switch (env[3]) {
        case '4': typ = NET_ADDR_IP4; break;
        case '6': typ = NET_ADDR_IP6; break;

        default: return (0);
      }
      
      switch (env[2]) {
        case 'l':
          // Link-local address
          if (env[3] == '4') { return (0);                             }
          else               { opt = netIF_OptionIP6_LinkLocalAddress; }
          break;

        case 'i':
          // Write local IP address (IPv4 or IPv6)
          if (env[3] == '4') { opt = netIF_OptionIP4_Address;       }
          else               { opt = netIF_OptionIP6_StaticAddress; }
          break;

        case 'm':
          // Write local network mask
          if (env[3] == '4') { opt = netIF_OptionIP4_SubnetMask; }
          else               { return (0);                       }
          break;

        case 'g':
          // Write default gateway IP address
          if (env[3] == '4') { opt = netIF_OptionIP4_DefaultGateway; }
          else               { opt = netIF_OptionIP6_DefaultGateway; }
          break;

        case 'p':
          // Write primary DNS server IP address
          if (env[3] == '4') { opt = netIF_OptionIP4_PrimaryDNS; }
          else               { opt = netIF_OptionIP6_PrimaryDNS; }
          break;

        case 's':
          // Write secondary DNS server IP address
          if (env[3] == '4') { opt = netIF_OptionIP4_SecondaryDNS; }
          else               { opt = netIF_OptionIP6_SecondaryDNS; }
          break;
      }

      netIF_GetOption (NET_IF_CLASS_ETH, opt, ip_addr, sizeof(ip_addr));
      netIP_ntoa (typ, ip_addr, ip_string, sizeof(ip_string));
      len = (uint32_t)sprintf (buf, &env[5], ip_string);
      break;
    case 'c': {
        // Usamos la estructura MY_BUF que ya tienes declarada al inicio del archivo
        // El campo 'idx' mantendrá de forma persistente el jugador actual (de 0 a 9)
        // Mientras queden al menos 120 bytes libres en el buffer de red, procesamos la siguiente fila
        while ((uint32_t)(len + 120) < buflen) {
            
            // Incrementamos el índice de jugador. 
            // Ojo: Como MYBUF(pcgi)->idx empieza en 0, en la primera vuelta socket valdrá 0 si usamos un post-incremento o inicialización limpia.
            // Para controlar el array de 0 a 9 de forma segura:
            uint8_t jugador_actual = MYBUF(pcgi)->idx;

            // Si ya hemos procesado los 10 jugadores (0 al 9), terminamos por completo
            if (jugador_actual >= 10) {
                return (len); // Devolvemos la longitud acumulada sin el bit de repetición
            }

            // Buffers locales seguros para evitar datos basura en RAM (el error del número gigante)
            char nombre_seguro[12];
            char fecha_segura[21];

            // Filtro de seguridad para el Nombre (11 bytes según memoria.h)
            memset(nombre_seguro, 0, sizeof(nombre_seguro));
            if (rankingActual[jugador_actual].nombre[0] == '\0') {
                strcpy(nombre_seguro, "---");
            } else {
                strncpy(nombre_seguro, rankingActual[jugador_actual].nombre, 11);
            }

            // Filtro de seguridad para la Fecha/Hora (20 bytes según memoria.h)
            memset(fecha_segura, 0, sizeof(fecha_segura));
            if (rankingActual[jugador_actual].fecha_hora[0] == '\0') {
                strcpy(fecha_segura, "--/--/---- --:--:--");
            } else {
                strncpy(fecha_segura, rankingActual[jugador_actual].fecha_hora, 20);
            }

            // Extraemos los puntos de forma aislada
            unsigned int puntos_jugador = (unsigned int)rankingActual[jugador_actual].puntos;

            // Formateamos e inyectamos la fila HTML concatenando en el buffer (buf + len)
            len += (uint32_t)sprintf(buf + len, 
                "<tr align=\"center\"><td>%s</td><td>%u</td><td>%s</td></tr>\r\n", 
                nombre_seguro, 
                puntos_jugador, 
                fecha_segura);

            // Avanzamos al siguiente jugador para la siguiente iteración del while
            MYBUF(pcgi)->idx++;
        }

        // Si salimos del while porque el buffer de red se llenó, pero aún quedan jugadores (idx < 10),
        // activamos el bit de repetición para que Keil nos vuelva a llamar.
        if (MYBUF(pcgi)->idx < 10) {
            len |= (1u << 31);
        }
    }
    break;

    case 'g': // CGX de la PARTIDA
      // Variable estática: recuerda si ya guardamos ESTE Game Over

      switch (env[2]) {
        case '1': // ACCIÓN Y DERROTA
          len = (uint32_t)sprintf (buf, &env[4], accion_go);
          
          if (strcmp (accion_go, "DERROTA" ) == 0 ) {
            // SÓLO si no se ha actualizado todavía en esta ráfaga
          if (!ranking_actualizado) {
            osThreadFlagsSet(tid_formateo, FLAG_ACTUALIZAR);
              ranking_actualizado = true; // Echamos el candado

            }
          } 
          else {
            // Si la acción es cualquier otra (PALMADA, GESTO, etc.), 
            // significa que estamos en una partida nueva o activa. Rearmamos el candado.
            ranking_actualizado = false; 
          }
          break;
        case '2': // PUNTOS
          len = (uint32_t)sprintf (buf, &env[4], puntos);
          break;
        case '3':
						len = (uint32_t)sprintf(buf, &env[4], aShowTime);
						break;
				case '4':
						len = (uint32_t)sprintf(buf, &env[4], aShowDate);
						break;
				case '5':
          if (strcmp (accion_go, "PAUSA" ) == 0 ) {
            consumo = 130;
          }
                   len = (uint32_t)sprintf (buf, &env[4], consumo);
//                  len = (uint32_t)sprintf (buf, &env[4], consumo*9);
          break;
      }
      break;
			
		 case 'e': //cgi consumo y RTC
			switch (env[2]) {
        case '1':
          len = (uint32_t)sprintf(buf, &env[4], aShowTime);
          break;
        case '2':
          len = (uint32_t)sprintf(buf, &env[4], aShowDate);
          break;
        case '3':
          len = (uint32_t)sprintf (buf, &env[4], consumo);
//                  len = (uint32_t)sprintf (buf, &env[4], consumo*9);

          break;
      }
			break;

    case 'y': //cgx consumo y RTC
			switch (env[2]) { 
				case '1':
						len = (uint32_t)sprintf(buf, &env[4], aShowTime);
						break;
				case '2':
						len = (uint32_t)sprintf(buf, &env[4], aShowDate);
						break;
				case '3':
						         len = (uint32_t)sprintf (buf, &env[4], consumo);
//                  len = (uint32_t)sprintf (buf, &env[4], consumo*9);
						break;
				}
      break;
  }
  return (len);
}

//void Thread_prueba (void *arg){
//	while(1){
//			osThreadFlagsWait(FLAG_PRUEBA, osFlagsWaitAny, osWaitForever);
//			printf("iniciando prueba\n");
//      puntos = 0;
//      strcpy(accion_go, "PALMADA");
//      osDelay(3000);
//      puntos = 1;
//      osDelay(10);
//      strcpy(accion_go, "GOLPEA");
//      osDelay(3000);
//      strcpy(accion_go, "PAUSA");
//      osDelay(5000);
//      puntos = 2;
//      osDelay(10);
//      strcpy(accion_go, "GESTO");
//      osDelay(3000);
//      puntos = 3;
//       osDelay(10);
//      strcpy(accion_go, "DERROTA");
//      osDelay(3000);
//	}
//}

#if      defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma  clang diagnostic pop
#endif
