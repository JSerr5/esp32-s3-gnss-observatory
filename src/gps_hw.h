/*
 * gps_hw.h
 * ---------------------------------------------------------------------------
 * Agente 1 - Arquitecto de Hardware, FreeRTOS y Driver UART en ESP-IDF.
 *
 * RESTRICCION ESTRICTA DE PINES (Seccion 2):
 *   - GPIO 4 (IO4): Salida UART1 TX -> RX del modulo GPS NEO-6M
 *   - GPIO 5 (IO5): Entrada UART1 RX <- TX del modulo GPS NEO-6M (pull-up)
 *   - GPIO 6 (IO6): Entrada opcional de interrupcion TIMEPULSE / PPS (1 Hz)
 *   Queda estrictamente prohibido utilizar cualquier otro pin GPIO.
 * ---------------------------------------------------------------------------
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

/* Estadisticas del pulso fisico PPS (TIMEPULSE) en GPIO 6 */
typedef struct {
    bool     active;          /* true si se recibio pulso en los ultimos 2.5 s */
    uint32_t pulse_count;     /* total de flancos de subida detectados         */
    int64_t  last_edge_us;    /* marca esp_timer_get_time() del ultimo pulso   */
    int64_t  last_period_us;  /* ultimo periodo medido (~1000000 us)           */
    int64_t  last_jitter_us;  /* desviacion respecto a 1000000 us              */
    int64_t  max_jitter_us;   /* peor jitter observado en valor absoluto       */
    int64_t  avg_jitter_us;   /* media movil simple del jitter absoluto        */
    int      pin_level;       /* nivel logico instantaneo de GPIO 6            */
} gps_pps_stat_t;

/* Inicializa UART1 en GPIO4/GPIO5 y la interrupcion opcional PPS en GPIO6 */
esp_err_t gps_hw_init(uint32_t initial_baud);

/* Cambia la velocidad de baudios de UART1 en caliente y limpia buffers */
esp_err_t gps_hw_set_baud(uint32_t baud);
uint32_t  gps_hw_get_baud(void);
void      gps_hw_flush_rx(void);

/* Envio de tramas por UART1 (GPIO 4) */
int gps_hw_send_raw(const uint8_t *data, size_t len);
int gps_hw_send_ubx(uint8_t cls, uint8_t id, const void *payload, uint16_t len);

/* Lectura de estado electrico de los pines permitidos */
int  gps_hw_rx_pin_level(void);   /* nivel logico de GPIO 5 (1 = pull-up idle) */
void gps_hw_get_pps(gps_pps_stat_t *out);

/* Tarea 1 (Core 1, prioridad alta): adquisicion UART1 y parseo UBX + NMEA */
void gps_rx_parser_task(void *arg);
