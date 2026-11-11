/*
 * gps_config.h
 * ---------------------------------------------------------------------------
 * Configuracion global del Auditor NEO-6M (Agente 1 - Hardware / FreeRTOS).
 *
 * RESTRICCION ABSOLUTA DE PINES: este firmware solo toca GPIO4, GPIO5, GPIO6.
 * Ningun otro GPIO es referenciado en todo el codigo fuente.
 * (La consola usa la UART0 por defecto del sistema / puente USB, que no se
 *  configura en este codigo.)
 * ---------------------------------------------------------------------------
 */
#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"

#ifndef GPS_FW_NAME
#define GPS_FW_NAME "NEO6M-AUDITOR"
#endif
#ifndef GPS_FW_VERSION
#define GPS_FW_VERSION "2.0.0"
#endif

/* ----------------------------- Pines ----------------------------------- */
#define GPS_PIN_TX            GPIO_NUM_4   /* ESP32-S3 TX -> RX NEO-6M      */
#define GPS_PIN_RX            GPIO_NUM_5   /* ESP32-S3 RX <- TX NEO-6M      */
#define GPS_PIN_PPS           GPIO_NUM_6   /* TIMEPULSE opcional (pad LED)  */

/* ----------------------------- UART ------------------------------------ */
#define GPS_UART_NUM          UART_NUM_1
#define GPS_UART_RX_BUF       8192
#define GPS_UART_TX_BUF       2048
#define GPS_UART_EVT_QUEUE    32
#define GPS_UART_RX_CHUNK     512

#define GPS_TARGET_BAUD       115200u      /* velocidad negociada objetivo  */
#define GPS_DEFAULT_ROM_BAUD  9600u        /* velocidad por defecto ROM 7.03 */

/* ------------------------- Tiempos (ms) -------------------------------- */
#define GPS_NO_DATA_WARN_MS        3000    /* Etapa 1: silencio inicial     */
#define GPS_WAIT_PRINT_PERIOD_MS   2000    /* aviso periodico de espera     */
#define GPS_AUTOBAUD_LISTEN_MS     1200    /* escucha por velocidad         */
#define GPS_CMD_TIMEOUT_MS          500    /* espera ACK/NAK/respuesta      */
#define GPS_BAUD_SWITCH_DELAY_MS    100    /* tras CFG-PRT                  */
#define GPS_LINK_LOST_MS           3000    /* sin bytes -> desconexion      */
#define GPS_NEG_TEST_LISTEN_MS     1500    /* escucha de datos en pruebas - */

/* ------------------------- Consola ------------------------------------- */
#define CONSOLE_BAUD                  115200
#define CONSOLE_PERIOD_WARMUP_MS      2000  /* panel de calentamiento       */
#define CONSOLE_PERIOD_MASTER_MS      3000  /* panel maestro confiable      */
#define CONSOLE_AUDIT_HOLD_MS         8000  /* tiempo que se mantiene la
                                               auditoria antes del panel   */
#define CONSOLE_USE_ANSI_DEFAULT      1     /* 1 = redibuja en sitio (ANSI) */

/* ------------------- Calentamiento / confianza ------------------------- */
#define TRUST_REQUIRED_EPOCHS         5     /* epocas consecutivas validas  */
#define TRUST_LOSS_EPOCHS             3     /* epocas malas para degradar   */
#define TRUST_MAX_TACC_NS             1000u
#define TRUST_MAX_PDOP_X100           1000u /* PDOP < 10.0                  */
#define TRUST_PREF_PDOP_X100           600u /* PDOP < 6.0 (preferido)       */
#define TRUST_MAX_HACC_MM             50000u/* 50 m                         */
#define TRUST_PREF_HACC_MM            10000u/* 10 m (cielo abierto)         */
#define TRUST_MIN_SV_3D               4
#define TRUST_MIN_SV_2D               3
#define TRUST_NMEA_FRESH_MS           2500  /* frescura de GGA/RMC          */
#define WARMUP_INDOOR_TIMEOUT_S       900   /* 15 min sin fix -> aviso      */
#define WARMUP_EPH_CNO_THRESHOLD      27    /* dB-Hz para decodificar efem. */
#define HEADING_MIN_GSPEED_CMS        10    /* < 0.10 m/s rumbo sin sentido */

/* ------------------------ Hora local ----------------------------------- */
/* Desfase de la hora local respecto a UTC en minutos (Colombia/Peru/Ecuador
 * = -300). Ajustar segun la zona del usuario.                              */
#define GPS_LOCAL_UTC_OFFSET_MIN      (-300)

/* ------------------------ Politica SBAS -------------------------------- */
/* 1 = si la posicion esta fuera de la huella de WAAS/EGNOS/MSAS/GAGAN, se
 *     deshabilita el uso de SBAS (CFG-SBAS) para que el receptor use el
 *     modelo ionosferico Klobuchar difundido por GPS (AID-HUI).           */
#define GPS_SBAS_AUTO_POLICY          1

/* ------------------------ Tasa de navegacion --------------------------- */
/* 1 = tras certificar datos confiables, pasar a 5 Hz (CFG-RATE 200 ms)
 *     reescalando las tasas de mensajes voluminosos. 0 = permanecer 1 Hz. */
#define GPS_FAST_NAV_AFTER_TRUST      0

/* ------------------------ Prueba de memoria ---------------------------- */
/* 1 = ejecutar UBX-CFG-CFG (save) en BBR y EEPROM 24C32 durante auditoria */
#define GPS_RUN_NVM_TEST              1

/* ------------------------ FreeRTOS ------------------------------------- */
#define TASK_RX_STACK        8192
#define TASK_RX_PRIO         (configMAX_PRIORITIES - 3)
#define TASK_RX_CORE         1

#define TASK_MGR_STACK       8192
#define TASK_MGR_PRIO        5
#define TASK_MGR_CORE        0

#define TASK_CON_STACK       10240
#define TASK_CON_PRIO        2
#define TASK_CON_CORE        0

/* ------------------- WiFi STA + Telemetria Web (Core 0) ---------------- */
#define GPS_WIFI_STA_SSID         "TU_SSID_WIFI"
#define GPS_WIFI_STA_PASS         "TU_PASSWORD_WIFI"
#define GPS_MDNS_HOSTNAME         "esp32-gps"
#define GPS_HTTP_PORT             80
#define GPS_JSON_PERIOD_MS        1000u     /* Emision en tiempo real por epoca */

#define TASK_WIFI_STACK           6144
#define TASK_WIFI_PRIO            3
#define TASK_WIFI_CORE            0         /* Core 0: aislado de UART1 (Core 1) */

