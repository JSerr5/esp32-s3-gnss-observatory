/*
 * gps_warmup.h
 * ---------------------------------------------------------------------------
 * Agente 4 - Gestor de Calentamiento (Cold Start / Warm-Up) y Motor de
 * Validacion Cientifica de Confianza (Seccion 5).
 * ---------------------------------------------------------------------------
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "gps_data.h"

/* Actualiza los contadores de canales, efemerides y almanaques en el repo */
void gps_warmup_refresh_channels(gps_repo_t *r);

/* Evaluado al cerrarse cada epoca PVT sintetizada (enlace bidireccional UBX) */
void gps_warmup_on_epoch(gps_repo_t *r, uint32_t now_ms);

/* Evaluado en modo degradado de solo escucha NMEA (cuando llega GPRMC) */
void gps_warmup_on_nmea_epoch(gps_repo_t *r, uint32_t now_ms);

/* Devuelve la descripcion y valores actuales del criterio de confianza i (0..8) */
bool gps_warmup_crit_desc(int idx, const gps_repo_t *r, char *buf, size_t cap);

/* Descripcion textual de qualityInd (0..7) de UBX-NAV-SVINFO */
const char *gps_warmup_quality_str(uint8_t q);
