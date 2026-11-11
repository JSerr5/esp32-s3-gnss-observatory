/*
 * ubx_protocol.h
 * ---------------------------------------------------------------------------
 * Agente 3 - Nucleo binario UBX: checksum Fletcher-8 (CK_A, CK_B), constructor
 * de tramas y maquina de estados de decodificacion byte a byte, sin bloqueos
 * y con cero asignaciones dinamicas de memoria.
 * ---------------------------------------------------------------------------
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define UBX_MAX_PAYLOAD   1024   /* NAV-SVINFO 50 canales = 608 bytes */
#define UBX_FRAME_OVERHEAD 8     /* sync(2)+cls+id+len(2)+ck(2) */

typedef enum {
    UBX_FEED_NONE = 0,     /* byte consumido, sin trama completa         */
    UBX_FEED_FRAME,        /* trama completa con checksum valido         */
    UBX_FEED_CKERR,        /* trama completa con checksum invalido       */
    UBX_FEED_OVERSIZE      /* longitud declarada mayor al buffer         */
} ubx_feed_t;

typedef struct {
    uint8_t  state;
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint16_t idx;
    uint8_t  ck_a;
    uint8_t  ck_b;
    uint8_t  rx_ck_a;
    uint8_t  payload[UBX_MAX_PAYLOAD];
    /* estadisticas */
    uint32_t frames_ok;
    uint32_t frames_ckerr;
    uint32_t frames_oversize;
} ubx_parser_t;

/* Fletcher-8 segun GPS.G6-SW-10018-F seccion "UBX Checksum" */
void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b);

/* Construye una trama completa. Devuelve la longitud total o 0 si no cabe */
size_t ubx_build_frame(uint8_t *out, size_t cap, uint8_t cls, uint8_t id,
                       const void *payload, uint16_t len);

void       ubx_parser_init(ubx_parser_t *p);
ubx_feed_t ubx_parser_feed(ubx_parser_t *p, uint8_t byte);

/* Autoprueba: compara el constructor con las tramas de referencia
 * documentadas (MON-VER, NAV-PVT, AID-HUI, CFG-RATE, CFG-PRT, CFG-CFG).
 * Devuelve el numero de fallos (0 = todo correcto).                       */
int  ubx_selftest(void (*report)(const char *name, bool ok, const uint8_t *frame, size_t len));

/* Lectura Little-Endian segura desde buffers no alineados */
static inline uint16_t ubx_u2(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline int16_t  ubx_i2(const uint8_t *p) { return (int16_t)ubx_u2(p); }
static inline uint32_t ubx_u4(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t  ubx_i4(const uint8_t *p) { return (int32_t)ubx_u4(p); }
