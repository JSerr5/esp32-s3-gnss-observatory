/*
 * gps_console.h
 * ---------------------------------------------------------------------------
 * Agente 5 - Integrador de Telemetria Serial y Consola Estructurada (Seccion 8).
 * Presenta en el monitor serial a 115200 bps los tres paneles progresivos:
 *   1. Reporte Inicial de Auditoria y Descubrimiento del Hardware
 *   2. Panel de Calentamiento y Diagnostico en Tiempo Real (cada 2 s)
 *   3. Panel Maestro de Datos Confiables Verificados (Fase E certificada)
 * ---------------------------------------------------------------------------
 */
#pragma once
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "gps_data.h"

/* Imprime el banner de arranque y el resultado del autodiagnostico UBX */
void gps_console_print_boot_banner(void);
void gps_console_selftest_cb(const char *name, bool ok, const uint8_t *frame, size_t len);

/* Imprime los 3 paneles estructurados a partir de una instantanea del repo */
void gps_console_print_audit_report(const gps_repo_t *r);
void gps_console_print_warmup_panel(const gps_repo_t *r);
void gps_console_print_master_panel(const gps_repo_t *r);

/* Tarea 3 (Core 0, prioridad baja, stack 10240): consola estructurada */
void gps_telemetry_console_task(void *arg);
