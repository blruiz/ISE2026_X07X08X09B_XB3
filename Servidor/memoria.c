#include "Memoria.h"
#include "Driver_I2C.h"
#include <string.h>
#include <stdio.h>

#define SLAVE_ADDR 0x50 

#define I2C_EVENT_SIGNAL_OK       0x01
#define I2C_EVENT_SIGNAL_ERROR    0x02

/* --- GLOBALES DEL RTOS --- */
osThreadId_t        e_memoriaThreadId;
osMessageQueueId_t  e_memoriaRxMessageId; 

/* --- DRIVER I2C --- */
extern ARM_DRIVER_I2C Driver_I2C2;
static ARM_DRIVER_I2C *I2Cdrv = &Driver_I2C2;

extern uint8_t aShowTime[20];
extern uint8_t aShowDate[20];

Jugador_t rankingActual[10];

/* Estructura de mensaje interna corregida */
typedef struct {
    uint8_t tipoPeticion;    // 0 = Escribir, 1 = Leer
    uint16_t addr_raw;       // Dirección lineal en la EEPROM
    uint8_t *pBufferRAM;     // Puntero directo a la RAM de origen/destino
    uint16_t longitud;       // Cantidad de bytes
    osThreadId_t thLlamante; // CORRECCIÓN: Guardamos qué hilo hizo la petición para despertarlo
} MemoriaMsg_t;

void I2C_Callback(uint32_t event) {
    if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        osThreadFlagsSet(e_memoriaThreadId, I2C_EVENT_SIGNAL_OK);
    } else if (event & (ARM_I2C_EVENT_TRANSFER_INCOMPLETE | ARM_I2C_EVENT_ADDRESS_NACK | ARM_I2C_EVENT_BUS_ERROR)) {
        osThreadFlagsSet(e_memoriaThreadId, I2C_EVENT_SIGNAL_ERROR);
    }
}

/* --- TAREA PRINCIPAL REPARADA --- */
void Run(void *argument) {
    MemoriaMsg_t msg;
    uint8_t cmdBuffer[3];

    I2Cdrv->Initialize(I2C_Callback);
    I2Cdrv->PowerControl(ARM_POWER_FULL);
    I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
    I2Cdrv->Control(ARM_I2C_BUS_CLEAR, 0);

    while (1) {
        if (osMessageQueueGet(e_memoriaRxMessageId, &msg, NULL, osWaitForever) == osOK) {
            
            if (msg.tipoPeticion == 0) { // ESCRIBIR BLOQUE (Byte a byte secuencial seguro)
                for (uint16_t i = 0; i < msg.longitud; i++) {
                    uint16_t currentAddr = msg.addr_raw + i;
                    
                    cmdBuffer[0] = (uint8_t)(currentAddr >> 8);   
                    cmdBuffer[1] = (uint8_t)(currentAddr & 0xFF); 
                    cmdBuffer[2] = msg.pBufferRAM[i];             
                    
                    I2Cdrv->MasterTransmit(SLAVE_ADDR, cmdBuffer, 3, false);
                    osThreadFlagsWait(I2C_EVENT_SIGNAL_OK | I2C_EVENT_SIGNAL_ERROR, osFlagsWaitAny, osWaitForever);
                    
                    osDelay(10); // tWR necesario de la EEPROM
                }
            } 
            else if (msg.tipoPeticion == 1) { // LEER EN RÁFAGA (Directo a RAM)
                cmdBuffer[0] = (uint8_t)(msg.addr_raw >> 8);
                cmdBuffer[1] = (uint8_t)(msg.addr_raw & 0xFF);
                
                I2Cdrv->MasterTransmit(SLAVE_ADDR, cmdBuffer, 2, true);
                osThreadFlagsWait(I2C_EVENT_SIGNAL_OK | I2C_EVENT_SIGNAL_ERROR, osFlagsWaitAny, osWaitForever);
                
                I2Cdrv->MasterReceive(SLAVE_ADDR, msg.pBufferRAM, msg.longitud, false);
                osThreadFlagsWait(I2C_EVENT_SIGNAL_OK | I2C_EVENT_SIGNAL_ERROR, osFlagsWaitAny, osWaitForever);
            }
            
            // CORRECCIÓN: Despertamos de forma segura al hilo específico que invocó la función
            if (msg.thLlamante != NULL) {
                osThreadFlagsSet(msg.thLlamante, 0x01); 
            }
        }
    }
}

/* --- API DE ALTO NIVEL --- */

void MemoriaInitialize(void) {
    e_memoriaRxMessageId = osMessageQueueNew(5, sizeof(MemoriaMsg_t), NULL);
    e_memoriaThreadId = osThreadNew(Run, NULL, NULL);
    
    osDelay(100); 
    CargarRankingDesdeEEPROM();
}

void leerMemoriaLinear(uint16_t addrLinear, char *pDestinoRAM, uint16_t longitud) {
    MemoriaMsg_t msg;
    msg.tipoPeticion = 1;
    msg.addr_raw = addrLinear;
    msg.pBufferRAM = (uint8_t*)pDestinoRAM;
    msg.longitud = longitud;
    msg.thLlamante = osThreadGetId(); // Guardamos el ID del hilo de pruebas (o del main) actual

    osMessageQueuePut(e_memoriaRxMessageId, &msg, 1, osWaitForever);
    
    // El hilo llamante se duerme de forma segura liberando CPU hasta que 'Run' termine
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
}

void escribirMemoriaLinear(uint16_t addrLinear, char *pOrigenRAM, uint16_t longitud) {
    MemoriaMsg_t msg;
    msg.tipoPeticion = 0;
    msg.addr_raw = addrLinear;
    msg.pBufferRAM = (uint8_t*)pOrigenRAM;
    msg.longitud = longitud;
    msg.thLlamante = osThreadGetId(); // Guardamos el ID del hilo que escribe

    osMessageQueuePut(e_memoriaRxMessageId, &msg, 1, osWaitForever);
    
    // Esperamos pacientemente a que termine toda la ráfaga física de escritura
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
}

/* --- IMPLEMENTACIÓN DEL RANKING --- */

#define RANKING_EEPROM_ADDR   320 //tengo que explicar por qué empezamos en la pagina 5 (64*5)

void CargarRankingDesdeEEPROM(void) {
    leerMemoriaLinear(RANKING_EEPROM_ADDR, (char*)rankingActual, sizeof(rankingActual));
    
    for (int i = 0; i < 10; i++) {
        if (rankingActual[i].puntos == 0xFF) {
            rankingActual[i].puntos = 0;
            memset(rankingActual[i].nombre, 0, sizeof(rankingActual[i].nombre));
            strcpy(rankingActual[i].fecha_hora, "00-00-0000 00:00:00");
        }
    }
}

void GuardarRankingEnEEPROM(void) {
    escribirMemoriaLinear(RANKING_EEPROM_ADDR, (char*)rankingActual, sizeof(rankingActual));
}

void FormatearEEPROM_Ranking(void) {
//    printf("[MEMORIA] Iniciando formateo fisico a 32 bytes por ranura...\n");

    for(int i = 0; i < 10; i++) {
        rankingActual[i].puntos = 0;
        memset(rankingActual[i].nombre, 0, sizeof(rankingActual[i].nombre));
        strcpy(rankingActual[i].fecha_hora, "00-00-0000 00:00:00");
    }

    GuardarRankingEnEEPROM();
//    printf("[MEMORIA] Formateo completado con exito.\n");
}

void ActualizarRanking(uint8_t nuevosPuntos, char* nuevoNombre) {
    // Entra si supera los puntos O si el puesto 10 está vacío de fábrica
    if ((nuevosPuntos > rankingActual[9].puntos) || (rankingActual[9].nombre[0] == '\0')) {
        
        // Sobrescribimos el último puesto temporalmente
        rankingActual[9].puntos = nuevosPuntos;
        
        memset(rankingActual[9].nombre, 0, sizeof(rankingActual[9].nombre));
        memset(rankingActual[9].fecha_hora, 0, sizeof(rankingActual[9].fecha_hora));
        
        strncpy(rankingActual[9].nombre, nuevoNombre, 10);
        snprintf(rankingActual[9].fecha_hora, sizeof(rankingActual[9].fecha_hora), 
                 "%s %s", (char*)aShowDate, (char*)aShowTime);

        // Ordenar por algoritmo de la Burbuja
        // NOTA: Para que el orden sea estable, si dos jugadores tienen 0 puntos,
        // el que tiene un nombre real debe escalar por encima del que está vacío ('\0').
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 9 - i; j++) {
                // Condición de intercambio: 
                // Caso A: El de abajo tiene estrictamente más puntos.
                // Caso B: Tienen los mismos puntos pero el de abajo tiene un nombre real y el de arriba está vacío.
                if ((rankingActual[j].puntos < rankingActual[j+1].puntos) || 
                    ((rankingActual[j].puntos == rankingActual[j+1].puntos) && 
                     (rankingActual[j].nombre[0] == '\0' && rankingActual[j+1].nombre[0] != '\0'))) {
                    
                    Jugador_t temp = rankingActual[j];
                    rankingActual[j] = rankingActual[j+1];
                    rankingActual[j+1] = temp;
                }
            }
        }
        
        // Guardamos el ranking corregido en la EEPROM física
        GuardarRankingEnEEPROM();
    }
}
