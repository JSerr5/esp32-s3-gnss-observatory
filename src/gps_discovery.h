/*
 * gps_discovery.h
 * ---------------------------------------------------------------------------
 * Agente 2 - Motor de Autoconciencia, Interrogacion Activa, Auto-Bauding y
 * Auditoria de Clones (Secciones 3 y 4).
 * ---------------------------------------------------------------------------
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "gps_data.h"

/* Envia un comando UBX y espera de forma determinista ACK-ACK, ACK-NAK o
 * trama de respuesta (hasta timeout_ms).                                   */
gps_res_t gps_cmd_Transact(uint8_t cls, uint8_t id,
                           const void *payload, uint16_t len,
                           uint8_t resp_cls, uint8_t resp_id,
                           bool want_resp, uint32_t timeout_ms);

/* Configura la tasa de emision de un mensaje en UART1 mediante UBX-CFG-MSG */
gps_res_t gps_cfg_set_msg_rate(uint8_t msg_cls, uint8_t msg_id, uint8_t rate_uart1);

/* Nombre legible de la fuente de arranque cfgSource de UBX-MON-HW2 */
const char *gps_cfg_source_str(uint8_t src);

/* Nombre legible del veredicto de autenticidad de silicio */
const char *gps_auth_str(gps_auth_t a);

/* Tarea 2 (Core 0, prioridad media): Auto-Bauding, descubrimiento y gestion */
void gps_discovery_and_manager_task(void *arg);
