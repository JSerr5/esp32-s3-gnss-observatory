/*
 * gps_wifi.h
 * ---------------------------------------------------------------------------
 * Modulo de Conectividad WiFi (Solo STA directo), mDNS ("esp32-gps.local"),
 * Servidor HTTP REST (GET /api/gps con CORS *), Server-Sent Events (GET /events)
 * y WebSocket RFC 6455 (GET /ws) aislado en el Core 0 con politica estricta
 * de publicacion de telemetria JSON cada 10,000 ms (10 segundos).
 *
 * RESTRICCION DE PINES: No utiliza ningun pin GPIO fisico (solo radio WiFi
 * interna de la ESP32-S3). El Core 1 queda 100% dedicado a UART1 (GPIO4/5).
 * ---------------------------------------------------------------------------
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "gps_data.h"

/* Inicializa pila TCP/IP, WiFi STA (WPA3/WPA2), servidor HTTP/WS y lanza la
 * tarea de telemetria de 10 segundos en el Core 0.                        */
esp_err_t gps_wifi_init_and_start(void);

/* Obtiene una copia del estado de red actual (IP STA)                     */
void gps_wifi_get_status_str(char *out, size_t cap);

/* Imprime en el monitor serial la IP asignada, mDNS y endpoints activos   */
void gps_wifi_print_network_banner(void);

/* Imprime en el monitor serial el ultimo JSON de 10s generado             */
void gps_wifi_print_last_json(void);
