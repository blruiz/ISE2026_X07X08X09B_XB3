#ifndef __MEMORIA_H
#define __MEMORIA_H

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

/* --- ESTRUCTURAS DE DATOS --- */

#pragma pack(push, 1) // OBLIGATORIO: Fuerza al compilador a mantener los 32 bytes exactos
typedef struct {
    uint8_t puntos;      // 1 byte  (Rango: 0 a 255)
    char nombre[11];     // 11 bytes (10 caracteres + '\0')
    char fecha_hora[20]; // 20 bytes (Espacio para "DD-MM-AAAA HH:MM:SS\0")
} Jugador_t;             // Tamaño total real: 32 bytes exactos
#pragma pack(pop)

//		typedef struct {
//		uint8_t tipoPeticion; // 0 = Escribir, 1 = Leer
//		uint16_t addr_raw;    // Dirección lineal real en la EEPROM (0 a 16383) | Dirección lineal en la EEPROM (5 << 6)
//		uint8_t *pBufferRAM;  // Puntero directo a la posición de la RAM original
//		uint16_t longitud;    // Cuántos bytes transferir en total (320 bytes para todo el ranking)
//		} MemoriaMsg_t;           // Consolida la entrada de forma limpia y eficiente

typedef struct {
    char dato[320];       // CORREGIDO: Ajustado a 320 bytes para soportar el bloque completo (10 * 32)
} MemoriaOutMsg_t;

/* --- PROTOTIPOS DE LA API DE ALTO NIVEL --- */
void MemoriaInitialize(void);
void CargarRankingDesdeEEPROM(void);
void ActualizarRanking(uint8_t nuevosPuntos, char* nuevoNombre);
void GuardarRankingEnEEPROM(void);
void FormatearEEPROM_Ranking(void);

/* --- PROTOTIPOS DE ACCESO LINEAL GENERAL --- */
// Te permitirán usar la EEPROM para guardar configuraciones del servidor en el futuro
void leerMemoriaLinear(uint16_t addrLinear, char *pDestinoRAM, uint16_t longitud);
void escribirMemoriaLinear(uint16_t addrLinear, char *pOrigenRAM, uint16_t longitud);

/* --- VARIABLES GLOBALES --- */
// Permite que HTTP_Server_CGI.c y el Main lean el ranking directamente desde la RAM
extern Jugador_t rankingActual[10];

#endif /* __MEMORIA_H */
