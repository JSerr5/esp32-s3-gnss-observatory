/*
 * gps_discovery.c
 * ---------------------------------------------------------------------------
 * Agente 2 - Implementacion de las 5 etapas de reconocimiento consciente:
 *   Etapa 1: Vigilancia de conexion fisica y espera activa en GPIO 4 / GPIO 5
 *   Etapa 2: Auto-Bauding (9600..4800) y negociacion a 115200 bps (CFG-PRT)
 *   Etapa 3: Interrogacion de identidad (MON-VER, MON-HW, MON-HW2), auditoria
 *            de autenticidad u-blox 6 vs clon y prueba de BBR + EEPROM 24C32
 *   Etapa 4: Matriz de Autodescubrimiento (ACK-ACK vs ACK-NAK)
 *   Etapa 5: Configuracion optima sin desbordamiento de TXBUF y sondeo
 *            periodico en segundo plano + politica regional SBAS/Klobuchar.
 * ---------------------------------------------------------------------------
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "gps_config.h"
#include "gps_hw.h"
#include "gps_data.h"
#include "gps_warmup.h"
#include "gps_discovery.h"
#include "ubx_protocol.h"

/* Tramas exactas documentadas en la especificacion (Seccion 3 y Seccion 4) */
static const uint8_t k_ubx_mon_ver_poll[8] = {
    0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34
};
static const uint8_t k_ubx_nav_pvt_poll[8] = {
    0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19
};
static const uint8_t k_ubx_aid_hui_poll[8] = {
    0xB5, 0x62, 0x0B, 0x02, 0x00, 0x00, 0x0D, 0x32
};
static const uint8_t k_ubx_cfg_prt_115200[28] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
    0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
    0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E
};
static const uint8_t k_ubx_cfg_cfg_save_both[21] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x1F, 0x06, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x05, 0x46, 0x05
};
static const uint8_t k_ubx_cfg_rate_5hz[14] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A
};

static const uint32_t k_baud_sweep[] = {
    9600u, 115200u, 38400u, 57600u, 19200u, 4800u
};
#define NUM_BAUDS (sizeof(k_baud_sweep) / sizeof(k_baud_sweep[0]))

const char *gps_cfg_source_str(uint8_t src)
{
    switch (src) {
    case 114: return "114 (ROM - arranque limpio desde mascara de silicio)";
    case 111: return "111 (OTP - memoria programable unica)";
    case 112: return "112 (Configuracion por pines de hardware)";
    case 102: return "102 (Memoria no volatil BBR / EEPROM I2C / Flash)";
    default:  return "Desconocido";
    }
}

const char *gps_auth_str(gps_auth_t a)
{
    switch (a) {
    case AUTH_GENUINE:      return "GENUINO u-blox NEO-6M-0-001 (ROM 7.03 / UBX 13.03)";
    case AUTH_UBLOX_OTHER:  return "CHIP u-blox AUTENTICO (revision distinta a ROM 7.03)";
    case AUTH_SUSPECT:      return "SOSPECHOSO (respuestas UBX parciales o incoherentes)";
    case AUTH_CLONE:        return "POSIBLE CLON FALSIFICADO (no responde UBX / talker ajeno)";
    case AUTH_UNVERIFIABLE: return "NO VERIFICABLE POR UBX (enlace unidireccional solo RX)";
    default:                return "En evaluacion...";
    }
}

static void set_status_line(const char *fmt, const char *arg, uint32_t val)
{
    gps_repo_t *r = gps_data_lock();
    snprintf(r->status_line, sizeof(r->status_line), fmt, arg ? arg : "", (unsigned long)val);
    gps_data_unlock();
}

/* -------------------------------------------------------------------------
 * Transaccion determinista UBX con espera de ACK-ACK / ACK-NAK / Respuesta
 * ------------------------------------------------------------------------- */
gps_res_t gps_cmd_Transact(uint8_t cls, uint8_t id,
                           const void *payload, uint16_t len,
                           uint8_t resp_cls, uint8_t resp_id,
                           bool want_resp, uint32_t timeout_ms)
{
    EventGroupHandle_t ev = gps_data_events();
    gps_pending_set(cls, id, resp_cls, resp_id, want_resp);

    if (gps_hw_send_ubx(cls, id, payload, len) < 0) {
        gps_pending_clear();
        return RES_TIMEOUT;
    }

    EventBits_t wait_bits = EVT_ACK | EVT_NAK | (want_resp ? EVT_RESP : 0);
    EventBits_t got = xEventGroupWaitBits(
        ev, wait_bits, pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout_ms)
    );

    /* Si llego primero la respuesta de un comando CFG, esperar brevemente su ACK */
    if ((got & EVT_RESP) && cls == UBX_CLS_CFG && !(got & (EVT_ACK | EVT_NAK))) {
        EventBits_t got2 = xEventGroupWaitBits(
            ev, EVT_ACK | EVT_NAK, pdTRUE, pdFALSE, pdMS_TO_TICKS(120)
        );
        got |= got2;
    }

    gps_pending_clear();

    if (got & EVT_NAK) {
        return RES_NAK;
    }
    if (want_resp && (got & EVT_RESP)) {
        return RES_DATA;
    }
    if (got & EVT_ACK) {
        return RES_ACK;
    }
    return RES_TIMEOUT;
}

gps_res_t gps_cfg_set_msg_rate(uint8_t msg_cls, uint8_t msg_id, uint8_t rate_uart1)
{
    /* Formato UBX-CFG-MSG de 8 bytes: tasa por cada uno de los 6 puertos
     * (DDC=0, UART1=1, UART2=2, USB=3, SPI=4, Res=5).                      */
    ubx_cfg_msg8_t m = {
        .msgClass = msg_cls,
        .msgID    = msg_id,
        .rate     = { 0, rate_uart1, 0, 0, 0, 0 }
    };
    return gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_MSG, &m, sizeof(m),
                            0, 0, false, GPS_CMD_TIMEOUT_MS);
}

/* -------------------------------------------------------------------------
 * Etapa 1 y Etapa 2: Vigilancia de Conexion Fisica, Auto-Bauding y CFG-PRT
 * ------------------------------------------------------------------------- */
static bool stage12_autobaud_and_link(void)
{
    EventGroupHandle_t ev = gps_data_events();
    uint32_t start_wait_ms = gps_now_ms();
    uint32_t last_warn_ms  = start_wait_ms;

    {
        gps_repo_t *r = gps_data_lock();
        r->sys_state = SYS_WAIT_LINK;
        r->link      = LINK_SEARCHING;
        gps_data_unlock();
    }

    for (;;) {
        for (size_t b = 0; b < NUM_BAUDS; b++) {
            uint32_t baud = k_baud_sweep[b];
            set_status_line("Auto-Bauding: probando UART1 a %s%lu bps...", "", baud);

            gps_hw_set_baud(baud);

            uint32_t u_ok0, n_ok0, bytes0, ferr0;
            {
                gps_repo_t *r = gps_data_lock();
                u_ok0  = r->ubx_ok;
                n_ok0  = r->nmea_ok;
                bytes0 = r->bytes_rx;
                ferr0  = r->uart_frame_err;
                gps_data_unlock();
            }

            /* Sondeo activo UBX-MON-VER: B5 62 0A 04 00 00 0E 34 */
            xEventGroupClearBits(ev, EVT_ANY_UBX | EVT_ANY_NMEA | EVT_ANY_BYTE | EVT_FRAME_ERR);
            gps_hw_send_raw(k_ubx_mon_ver_poll, sizeof(k_ubx_mon_ver_poll));

            /* Escuchar durante 1200 ms en dos mitades de 600 ms */
            vTaskDelay(pdMS_TO_TICKS(GPS_AUTOBAUD_LISTEN_MS / 2));
            gps_hw_send_raw(k_ubx_mon_ver_poll, sizeof(k_ubx_mon_ver_poll));
            vTaskDelay(pdMS_TO_TICKS(GPS_AUTOBAUD_LISTEN_MS / 2));

            uint32_t u_ok1, n_ok1, bytes1, ferr1;
            {
                gps_repo_t *r = gps_data_lock();
                u_ok1  = r->ubx_ok;
                n_ok1  = r->nmea_ok;
                bytes1 = r->bytes_rx;
                ferr1  = r->uart_frame_err;
                gps_data_unlock();
            }

            if ((u_ok1 > u_ok0) || (n_ok1 > n_ok0)) {
                /* Velocidad de baudios detectada con tramas validas */
                bool got_ubx = (u_ok1 > u_ok0);

                gps_repo_t *r = gps_data_lock();
                r->detected_baud = baud;
                r->link          = LINK_DETECTED;
                r->link_events++;
                gps_data_unlock();

                /* Si aun no recibimos UBX, intentar un sondeo determinista MON-VER */
                if (!got_ubx) {
                    gps_res_t vr = gps_cmd_Transact(UBX_CLS_MON, UBX_MON_VER, NULL, 0,
                                                    UBX_CLS_MON, UBX_MON_VER, true,
                                                    GPS_CMD_TIMEOUT_MS);
                    if (vr == RES_DATA || vr == RES_ACK) {
                        got_ubx = true;
                    }
                }

                /* Negociar cambio a 115200 bps mediante la trama exacta UBX-CFG-PRT */
                uint32_t final_baud = baud;
                bool switched_ok = false;

                if (baud != GPS_TARGET_BAUD) {
                    set_status_line("Negociando cambio de velocidad a %s%lu bps (UBX-CFG-PRT)...",
                                    "", GPS_TARGET_BAUD);
                    gps_hw_send_raw(k_ubx_cfg_prt_115200, sizeof(k_ubx_cfg_prt_115200));
                    vTaskDelay(pdMS_TO_TICKS(GPS_BAUD_SWITCH_DELAY_MS));
                    gps_hw_set_baud(GPS_TARGET_BAUD);
                    vTaskDelay(pdMS_TO_TICKS(150));

                    /* Verificar comunicacion bidireccional a 115200 bps */
                    gps_res_t vr = gps_cmd_Transact(UBX_CLS_MON, UBX_MON_VER, NULL, 0,
                                                    UBX_CLS_MON, UBX_MON_VER, true,
                                                    GPS_CMD_TIMEOUT_MS);
                    if (vr == RES_DATA || vr == RES_ACK) {
                        switched_ok = true;
                        final_baud  = GPS_TARGET_BAUD;
                        got_ubx     = true;
                    } else {
                        /* Comprobar si llegan sentencias NMEA validas a 115200 bps */
                        uint32_t n0;
                        r = gps_data_lock();
                        n0 = r->nmea_ok;
                        gps_data_unlock();
                        vTaskDelay(pdMS_TO_TICKS(900));
                        r = gps_data_lock();
                        bool nmea_at_115k = (r->nmea_ok > n0);
                        gps_data_unlock();

                        if (nmea_at_115k) {
                            switched_ok = true;
                            final_baud  = GPS_TARGET_BAUD;
                        } else {
                            /* El pin RX del GPS no recibio CFG-PRT: volver al baudrate original */
                            gps_hw_set_baud(baud);
                            vTaskDelay(pdMS_TO_TICKS(150));
                            final_baud  = baud;
                            switched_ok = false;
                        }
                    }
                } else {
                    /* Ya operaba a 115200 bps: asegurar mascara in/out UBX+NMEA */
                    gps_hw_send_raw(k_ubx_cfg_prt_115200, sizeof(k_ubx_cfg_prt_115200));
                    vTaskDelay(pdMS_TO_TICKS(GPS_BAUD_SWITCH_DELAY_MS));
                    switched_ok = true;
                }

                /* Confirmar bidireccionalidad con un sondeo MON-VER o CFG-PRT */
                if (!got_ubx) {
                    gps_res_t pr = gps_cmd_Transact(UBX_CLS_MON, UBX_MON_VER, NULL, 0,
                                                    UBX_CLS_MON, UBX_MON_VER, true,
                                                    GPS_CMD_TIMEOUT_MS);
                    if (pr == RES_DATA || pr == RES_ACK) {
                        got_ubx = true;
                    }
                }

                r = gps_data_lock();
                r->final_baud     = final_baud;
                r->baud_switch_ok = switched_ok;
                r->bidir          = got_ubx;
                r->nmea_only      = !got_ubx;
                r->lite_profile   = (final_baud <= 19200u);
                r->link           = got_ubx ? LINK_BIDIR : LINK_RX_ONLY;
                gps_data_unlock();
                return true;
            }

            if (bytes1 > bytes0 || ferr1 > ferr0) {
                gps_repo_t *r = gps_data_lock();
                r->link = LINK_ACTIVITY;
                gps_data_unlock();
            }

            /* Etapa 1: Si han pasado mas de 3 s sin bytes, emitir aviso cada 2 s */
            uint32_t now = gps_now_ms();
            gps_repo_t *r = gps_data_lock();
            uint32_t total_rx = r->bytes_rx;
            gps_data_unlock();

            if (total_rx == 0 && (now - start_wait_ms) >= GPS_NO_DATA_WARN_MS &&
                (now - last_warn_ms) >= GPS_WAIT_PRINT_PERIOD_MS) {
                last_warn_ms = now;
                int rx_lvl = gps_hw_rx_pin_level();
                printf("\r\n[ETAPA 1 - ESPERA ACTIVA t=%lus] Sin flujo de bytes en GPIO 5 (RX=%d). "
                       "Esperando conexion fisica o alimentacion del modulo GPS en GPIO 4 (TX) y GPIO 5 (RX)...\r\n",
                       (unsigned long)((now - start_wait_ms) / 1000u), rx_lvl);
            }
        }

        gps_repo_t *r = gps_data_lock();
        r->autobaud_cycles++;
        gps_data_unlock();
    }
}

/* -------------------------------------------------------------------------
 * Etapa 3: Identidad de Silicio, Auditoria de Clones y Prueba BBR + EEPROM
 * ------------------------------------------------------------------------- */
static void stage3_identity_and_nvm_audit(void)
{
    set_status_line("Etapa 3: Interrogando identidad de silicio (UBX-MON-VER / HW / HW2)...", "", 0);

    bool bidir;
    {
        gps_repo_t *r = gps_data_lock();
        bidir = r->bidir;
        gps_data_unlock();
    }

    if (!bidir) {
        gps_repo_t *r = gps_data_lock();
        if (r->talker_mask & ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 4))) {
            r->auth = AUTH_CLONE;
            snprintf(r->auth_reason, sizeof(r->auth_reason),
                     "No responde UBX y emite talkers multi-GNSS (GN/GL/GA/BD) imposibles en u-blox 6");
        } else {
            r->auth = AUTH_UNVERIFIABLE;
            snprintf(r->auth_reason, sizeof(r->auth_reason),
                     "Solo escucha en GPIO 5: verifique el cable de GPIO 4 (TX ESP32-S3) al pin RX del GPS");
        }
        r->nvm_bbr    = RES_NA;
        r->nvm_eeprom = RES_NA;
        r->nvm_both   = RES_NA;
        gps_data_unlock();
        return;
    }

    /* 1. Sondeo UBX-MON-VER (0x0A 0x04) */
    int idx_ver = gps_msg_index(UBX_CLS_MON, UBX_MON_VER);
    gps_res_t r_ver = gps_cmd_Transact(UBX_CLS_MON, UBX_MON_VER, NULL, 0,
                                       UBX_CLS_MON, UBX_MON_VER, true,
                                       GPS_CMD_TIMEOUT_MS);
    if (r_ver == RES_TIMEOUT) {
        /* Segundo intento por seguridad */
        r_ver = gps_cmd_Transact(UBX_CLS_MON, UBX_MON_VER, NULL, 0,
                                 UBX_CLS_MON, UBX_MON_VER, true,
                                 GPS_CMD_TIMEOUT_MS);
    }

    /* 2. Sondeo UBX-MON-HW (0x0A 0x09) y UBX-MON-HW2 (0x0A 0x0B) */
    int idx_hw  = gps_msg_index(UBX_CLS_MON, UBX_MON_HW);
    int idx_hw2 = gps_msg_index(UBX_CLS_MON, UBX_MON_HW2);
    gps_res_t r_hw  = gps_cmd_Transact(UBX_CLS_MON, UBX_MON_HW, NULL, 0,
                                       UBX_CLS_MON, UBX_MON_HW, true,
                                       GPS_CMD_TIMEOUT_MS);
    gps_res_t r_hw2 = gps_cmd_Transact(UBX_CLS_MON, UBX_MON_HW2, NULL, 0,
                                       UBX_CLS_MON, UBX_MON_HW2, true,
                                       GPS_CMD_TIMEOUT_MS);

    /* 3. Prueba no destructiva de BBR y EEPROM I2C 24C32 (UBX-CFG-CFG) */
    gps_res_t r_bbr = RES_NA, r_eep = RES_NA, r_both = RES_NA;
#if GPS_RUN_NVM_TEST
    set_status_line("Etapa 3: Probando memoria BBR y EEPROM I2C 24C32 (UBX-CFG-CFG)...", "", 0);
    /* Guardar subconjunto msgConf+infMsg (0x061F) en BBR (devMask=0x01) */
    ubx_cfg_cfg_t cfg_bbr = {
        .clearMask  = 0x00000000u,
        .saveMask   = 0x0000061Fu,
        .loadMask   = 0x00000000u,
        .deviceMask = 0x01u
    };
    r_bbr = gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_CFG, &cfg_bbr, sizeof(cfg_bbr),
                             0, 0, false, GPS_CMD_TIMEOUT_MS);

    /* Guardar en EEPROM I2C externa 24C32 (devMask=0x04) */
    ubx_cfg_cfg_t cfg_eep = cfg_bbr;
    cfg_eep.deviceMask = 0x04u;
    r_eep = gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_CFG, &cfg_eep, sizeof(cfg_eep),
                             0, 0, false, GPS_CMD_TIMEOUT_MS);

    /* Ejecutar la trama hexadecimal exacta especificada para BBR + EEPROM (devMask=0x05):
     * B5 62 06 09 0D 00 00 00 00 00 1F 06 00 00 00 00 00 00 05 46 05 */
    EventGroupHandle_t ev = gps_data_events();
    gps_pending_set(UBX_CLS_CFG, UBX_CFG_CFG, 0, 0, false);
    gps_hw_send_raw(k_ubx_cfg_cfg_save_both, sizeof(k_ubx_cfg_cfg_save_both));
    EventBits_t got = xEventGroupWaitBits(ev, EVT_ACK | EVT_NAK, pdTRUE, pdFALSE,
                                          pdMS_TO_TICKS(GPS_CMD_TIMEOUT_MS));
    gps_pending_clear();
    if (got & EVT_ACK)      r_both = RES_ACK;
    else if (got & EVT_NAK) r_both = RES_NAK;
    else                    r_both = RES_TIMEOUT;
#endif

    /* Evaluar autenticidad del silicio u-blox 6 vs clon */
    gps_repo_t *r = gps_data_lock();
    if (idx_ver >= 0) r->mstat[idx_ver].poll = r_ver;
    if (idx_hw  >= 0) r->mstat[idx_hw].poll  = r_hw;
    if (idx_hw2 >= 0) r->mstat[idx_hw2].poll = r_hw2;
    r->nvm_bbr    = r_bbr;
    r->nvm_eeprom = r_eep;
    r->nvm_both   = r_both;

    bool sw_703  = (strstr(r->swVersion, "7.03") != NULL);
    bool sw_rom  = (strstr(r->swVersion, "ROM CORE") != NULL) || (strstr(r->swVersion, "EXT CORE") != NULL);
    bool hw_g60  = (strstr(r->hwVersion, "00040007") != NULL);
    bool bad_tlk = (r->talker_mask & ((1u << 1) | (1u << 2) | (1u << 3) | (1u << 4))) != 0;

    if (!r->ver_valid) {
        r->auth = AUTH_CLONE;
        snprintf(r->auth_reason, sizeof(r->auth_reason),
                 "No devolvio UBX-MON-VER valido; posible clon (AT6558 / CASIC) remarcado");
    } else if (bad_tlk) {
        r->auth = AUTH_CLONE;
        snprintf(r->auth_reason, sizeof(r->auth_reason),
                 "MON-VER respondio pero emite sentencias multi-constelacion ajenas a u-blox 6");
    } else if (sw_703 && hw_g60) {
        if (!r->protver_verified) {
            /* ROM 7.03 (45969) corresponde exactamente al protocolo 13.03 */
            snprintf(r->protver, sizeof(r->protver), "13.03");
        }
        r->auth = AUTH_GENUINE;
        snprintf(r->auth_reason, sizeof(r->auth_reason),
                 "sw='%s' hw='%s' prot=%s coherentes con silicio u-blox 6 NEO-6M-0-001",
                 r->swVersion, r->hwVersion, r->protver);
    } else if (sw_rom || hw_g60) {
        r->auth = AUTH_UBLOX_OTHER;
        snprintf(r->auth_reason, sizeof(r->auth_reason),
                 "Silicio u-blox autentico con version sw='%s' hw='%s'",
                 r->swVersion, r->hwVersion);
    } else {
        r->auth = AUTH_SUSPECT;
        snprintf(r->auth_reason, sizeof(r->auth_reason),
                 "Cadenas MON-VER atipicas: sw='%s' hw='%s'",
                 r->swVersion, r->hwVersion);
    }
    gps_data_unlock();
}

/* -------------------------------------------------------------------------
 * Etapa 4: Matriz de Autodescubrimiento de Mensajes Soportados vs Rechazados
 * ------------------------------------------------------------------------- */
static void stage4_capability_matrix(void)
{
    bool bidir, lite;
    {
        gps_repo_t *r = gps_data_lock();
        bidir = r->bidir;
        lite  = r->lite_profile;
        gps_data_unlock();
    }
    if (!bidir) {
        return;
    }

    for (int i = 0; i < g_msg_table_len; i++) {
        const gps_msg_desc_t *d = &g_msg_table[i];
        char info[48];
        snprintf(info, sizeof(info), "%.24s (0x%02X 0x%02X)", d->name, d->cls, d->id);
        set_status_line("Etapa 4: Auditando soporte de %s...", info, 0);

        gps_res_t poll_res   = RES_NA;
        gps_res_t cfgmsg_res = RES_NA;
        uint8_t   rate_final = 0;

        /* 1) Sondeo directo (payload 0 bytes) si el mensaje es sondeable o CFG */
        if (d->flags & (MF_POLL | MF_CFGREAD)) {
            if (lite && (d->flags & MF_HEAVY)) {
                poll_res = RES_NA;
            } else if (d->cls == UBX_CLS_NAV && d->id == UBX_NAV_PVT) {
                /* Sondeo explicito de UBX-NAV-PVT con trama exacta B5 62 01 07 00 00 08 19 */
                EventGroupHandle_t ev = gps_data_events();
                gps_pending_set(UBX_CLS_NAV, UBX_NAV_PVT, UBX_CLS_NAV, UBX_NAV_PVT, true);
                gps_hw_send_raw(k_ubx_nav_pvt_poll, sizeof(k_ubx_nav_pvt_poll));
                EventBits_t got = xEventGroupWaitBits(
                    ev, EVT_ACK | EVT_NAK | EVT_RESP, pdTRUE, pdFALSE,
                    pdMS_TO_TICKS(GPS_CMD_TIMEOUT_MS)
                );
                gps_pending_clear();
                if (got & EVT_NAK)       poll_res = RES_NAK;
                else if (got & EVT_RESP) poll_res = RES_DATA;
                else if (got & EVT_ACK)  poll_res = RES_ACK;
                else                     poll_res = RES_TIMEOUT;
            } else if (d->cls == UBX_CLS_AID && d->id == UBX_AID_HUI) {
                /* Sondeo explicito de UBX-AID-HUI con trama exacta B5 62 0B 02 00 00 0D 32 */
                EventGroupHandle_t ev = gps_data_events();
                gps_pending_set(UBX_CLS_AID, UBX_AID_HUI, UBX_CLS_AID, UBX_AID_HUI, true);
                gps_hw_send_raw(k_ubx_aid_hui_poll, sizeof(k_ubx_aid_hui_poll));
                EventBits_t got = xEventGroupWaitBits(
                    ev, EVT_ACK | EVT_NAK | EVT_RESP, pdTRUE, pdFALSE,
                    pdMS_TO_TICKS(GPS_CMD_TIMEOUT_MS)
                );
                gps_pending_clear();
                if (got & EVT_NAK)       poll_res = RES_NAK;
                else if (got & EVT_RESP) poll_res = RES_DATA;
                else if (got & EVT_ACK)  poll_res = RES_ACK;
                else                     poll_res = RES_TIMEOUT;
            } else if (d->cls == UBX_CLS_CFG && d->id == UBX_CFG_PRT) {
                /* CFG-PRT poll para UART1 requiere 1 byte (portID = 1) */
                uint8_t port_id = 1;
                poll_res = gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_PRT, &port_id, 1,
                                            UBX_CLS_CFG, UBX_CFG_PRT, true,
                                            GPS_CMD_TIMEOUT_MS);
            } else if ((d->cls == UBX_CLS_AID || d->cls == UBX_CLS_RXM) &&
                       (d->id == UBX_AID_ALM || d->id == UBX_AID_EPH || d->id == UBX_AID_AOP)) {
                /* Sondear SV 1 (1 byte svid=1) para no inundar con 32 paquetes seguidos */
                uint8_t svid = 1;
                poll_res = gps_cmd_Transact(d->cls, d->id, &svid, 1,
                                            d->cls, d->id, true,
                                            GPS_CMD_TIMEOUT_MS);
            } else {
                poll_res = gps_cmd_Transact(d->cls, d->id, NULL, 0,
                                            d->cls, d->id, true,
                                            GPS_CMD_TIMEOUT_MS);
            }
        }

        /* 2) Prueba de activacion con UBX-CFG-MSG (0x06 0x01) */
        if (d->flags & MF_PERIODIC) {
            if (d->flags & MF_NEG) {
                /* Prueba negativa (NAV-PVT, RXM-RAW, RXM-SFRB, TIM-SVIN, etc.):
                 * intentar habilitar a 1 Hz para demostrar que el NEO-6M devuelve ACK-NAK */
                cfgmsg_res = gps_cfg_set_msg_rate(d->cls, d->id, 1);
                if (cfgmsg_res == RES_ACK) {
                    /* Si aceptara CFG-MSG, apagar de nuevo */
                    gps_cfg_set_msg_rate(d->cls, d->id, 0);
                }
                rate_final = 0;
            } else {
                uint8_t target_rate = lite ? d->rate_lite : d->rate_norm;
                /* Si target_rate es 0, probamos primero con 0 (que igual produce ACK-ACK
                 * si el ID existe en la tabla del firmware ROM 7.03) */
                cfgmsg_res = gps_cfg_set_msg_rate(d->cls, d->id, target_rate);
                if (cfgmsg_res == RES_ACK) {
                    rate_final = target_rate;
                }
            }
        }

        gps_repo_t *r = gps_data_lock();
        if (r->mstat[i].poll == RES_UNTESTED || poll_res != RES_NA) {
            r->mstat[i].poll = poll_res;
        }
        r->mstat[i].cfgmsg   = cfgmsg_res;
        r->mstat[i].rate_set = rate_final;
        gps_data_unlock();
    }
}

/* -------------------------------------------------------------------------
 * Etapa 5: Configuracion Optima del Receptor (CFG-RATE, CFG-ITFM, CFG-NMEA,
 * CFG-INF) sin desbordamiento de UBX-MON-TXBUF.
 * ------------------------------------------------------------------------- */
static void stage5_optimal_configuration(void)
{
    bool bidir;
    {
        gps_repo_t *r = gps_data_lock();
        bidir = r->bidir;
        gps_data_unlock();
    }
    if (!bidir) {
        return;
    }

    set_status_line("Etapa 5: Configurando receptor (RATE 1Hz, ITFM, NMEA v2.3 extendido)...", "", 0);

    /* 1. Tasa de navegacion a 1 Hz durante calentamiento y auditoria (1000 ms) */
    ubx_cfg_rate_t rate_1hz = {
        .measRate = 1000u,
        .navRate  = 1u,
        .timeRef  = 1u /* GPS time */
    };
    if (gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_RATE, &rate_1hz, sizeof(rate_1hz),
                         0, 0, false, GPS_CMD_TIMEOUT_MS) == RES_ACK) {
        gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_RATE, NULL, 0,
                         UBX_CLS_CFG, UBX_CFG_RATE, true, GPS_CMD_TIMEOUT_MS);
    }

    /* 2. Activar monitor de interferencias CW / Jamming UBX-CFG-ITFM (8 bytes):
     *    bbThreshold = 3 (b0..3), cwThreshold = 15 (b4..8),
     *    algorithmBits = 0x16B156 (b9..30), enable = 1 (b31).
     *    config2: generalBits = 0x31E (b0..11), antSetting = 0 (b12..13). */
    ubx_cfg_itfm_t itfm = {
        .config  = (1u << 31) | (0x16B156u << 9) | (15u << 4) | 3u,
        .config2 = 0x0000031Eu
    };
    if (gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_ITFM, &itfm, sizeof(itfm),
                         0, 0, false, GPS_CMD_TIMEOUT_MS) == RES_ACK) {
        gps_repo_t *r = gps_data_lock();
        r->itfm_enabled_by_fw = true;
        gps_data_unlock();
        gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_ITFM, NULL, 0,
                         UBX_CLS_CFG, UBX_CFG_ITFM, true, GPS_CMD_TIMEOUT_MS);
    }

    /* 3. Activar NMEA modo extendido v2.3 sin limite de 12 satelites (numSV = 0) */
    ubx_cfg_nmea_t nmea_cfg = {
        .filter  = 0x00u,
        .version = 0x23u,
        .numSV   = 0x00u, /* 0 = ilimitado */
        .flags   = 0x02u  /* consider mode activo */
    };
    if (gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_NMEA, &nmea_cfg, sizeof(nmea_cfg),
                         0, 0, false, GPS_CMD_TIMEOUT_MS) == RES_ACK) {
        gps_repo_t *r = gps_data_lock();
        r->nmea_ext_by_fw = true;
        gps_data_unlock();
        gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_NMEA, NULL, 0,
                         UBX_CLS_CFG, UBX_CFG_NMEA, true, GPS_CMD_TIMEOUT_MS);
    }

    /* 4. Habilitar mensajes INF (ERROR, WARNING, NOTICE) para UBX y NMEA en UART1 */
    uint8_t inf_ubx[10] = {
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x07, 0x00, 0x00, 0x00, 0x00 /* UART1: ERROR+WARNING+NOTICE */
    };
    gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_INF, inf_ubx, sizeof(inf_ubx),
                     0, 0, false, GPS_CMD_TIMEOUT_MS);
    uint8_t inf_nmea[10] = {
        0x01, 0x00, 0x00, 0x00,
        0x00, 0x87, 0x00, 0x00, 0x00, 0x00 /* UART1 NMEA: USER+NOTICE+WARN+ERR */
    };
    gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_INF, inf_nmea, sizeof(inf_nmea),
                     0, 0, false, GPS_CMD_TIMEOUT_MS);

    /* 5. Sondeo inicial de contadores de puertos y buffers */
    gps_cmd_Transact(UBX_CLS_MON, UBX_MON_IO,    NULL, 0, UBX_CLS_MON, UBX_MON_IO,    true, 300);
    gps_cmd_Transact(UBX_CLS_MON, UBX_MON_MSGPP, NULL, 0, UBX_CLS_MON, UBX_MON_MSGPP, true, 300);
    gps_cmd_Transact(UBX_CLS_MON, UBX_MON_RXBUF, NULL, 0, UBX_CLS_MON, UBX_MON_RXBUF, true, 300);
    gps_cmd_Transact(UBX_CLS_MON, UBX_MON_TXBUF, NULL, 0, UBX_CLS_MON, UBX_MON_TXBUF, true, 300);
}

/* -------------------------------------------------------------------------
 * Politica regional SBAS vs Klobuchar (Seccion 3.4.d):
 * En regiones ecuatoriales o Sudamerica (lat < 15 N y lon entre -95 y -30),
 * fuera de la malla ionosferica de WAAS/EGNOS, las correcciones regionales
 * pueden degradar la posicion; se informa y opcionalmente se desactivan las
 * correcciones ionosfericas SBAS para priorizar Klobuchar (UBX-AID-HUI).
 * ------------------------------------------------------------------------- */
static void evaluate_sbas_regional_policy(void)
{
    bool do_disable = false;
    double lat_deg = 0.0, lon_deg = 0.0;

    {
        gps_repo_t *r = gps_data_lock();
        if (r->sbas_policy[0] == '\0' && r->posllh.valid &&
            (r->pvt.gpsFix_sol == 2 || r->pvt.gpsFix_sol == 3 || r->pvt.gpsFix_sol == 4)) {
            lat_deg = (double)r->posllh.d.lat * 1e-7;
            lon_deg = (double)r->posllh.d.lon * 1e-7;

            bool out_of_waas_egnos = (lat_deg < 15.0 && lon_deg > -100.0 && lon_deg < -30.0);
            if (out_of_waas_egnos) {
#if GPS_SBAS_AUTO_POLICY
                if (r->bidir && !r->sbas_disabled_by_fw) {
                    do_disable = true;
                }
                snprintf(r->sbas_policy, sizeof(r->sbas_policy),
                         "Region ecuatorial/Sudamerica (%.2f, %.2f): priorizando Klobuchar GPS (AID-HUI) sobre malla WAAS",
                         lat_deg, lon_deg);
#else
                snprintf(r->sbas_policy, sizeof(r->sbas_policy),
                         "Aviso regional (%.2f, %.2f): fuera de huella ionosferica WAAS/EGNOS; verifique Klobuchar AID-HUI",
                         lat_deg, lon_deg);
#endif
            } else {
                snprintf(r->sbas_policy, sizeof(r->sbas_policy),
                         "Posicion (%.2f, %.2f) dentro de cobertura nominal SBAS/Klobuchar",
                         lat_deg, lon_deg);
            }
        }
        gps_data_unlock();
    }

    if (do_disable) {
        /* Mantener recepcion SBAS en modo Ranging+Integrity pero sin aplicar diffCorr
         * ionosferica fuera de huella (usage = 0x05 -> range+integrity, sin diffCorr) */
        ubx_cfg_sbas_t sbas = {
            .mode      = 0x01u,
            .usage     = 0x05u,
            .maxSBAS   = 3u,
            .scanmode2 = 0u,
            .scanmode1 = 0u
        };
        if (gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_SBAS, &sbas, sizeof(sbas),
                             0, 0, false, GPS_CMD_TIMEOUT_MS) == RES_ACK) {
            gps_repo_t *r = gps_data_lock();
            r->sbas_disabled_by_fw = true;
            gps_data_unlock();
            gps_cmd_Transact(UBX_CLS_CFG, UBX_CFG_SBAS, NULL, 0,
                             UBX_CLS_CFG, UBX_CFG_SBAS, true, 300);
        }
    }
}

/* -------------------------------------------------------------------------
 * Control automatico de carga del buffer TX del NEO-6M (UBX-MON-TXBUF < 80%)
 * ------------------------------------------------------------------------- */
static void guard_txbuf_occupancy(void)
{
    bool throttle = false;
    {
        gps_repo_t *r = gps_data_lock();
        if (r->txbuf.valid && (r->txbuf.d.tUsage > 80 || r->txbuf.d.usage[1] > 80)) {
            throttle = true;
        }
        gps_data_unlock();
    }
    if (throttle) {
        /* Reducir frecuencia de los mensajes mas voluminosos para proteger el TXBUF */
        gps_cfg_set_msg_rate(UBX_CLS_NAV,  UBX_NAV_SVINFO, 5);
        gps_cfg_set_msg_rate(UBX_CLS_RXM,  UBX_RXM_SVSI,   10);
        gps_cfg_set_msg_rate(NMEA_CLS_STD, NMEA_ID_GSV,    10);
        gps_cfg_set_msg_rate(NMEA_CLS_PUBX, PUBX_ID_03,    5);
    }
}

/* -------------------------------------------------------------------------
 * Tarea 2: gps_discovery_and_manager_task (Core 0, prioridad media)
 * ------------------------------------------------------------------------- */
void gps_discovery_and_manager_task(void *arg)
{
    (void)arg;
    uint8_t bg_step = 0;
    uint8_t sv_poll = 1;

    for (;;) {
        /* Etapas 1 y 2: Vigilancia fisica y Auto-Bauding */
        stage12_autobaud_and_link();

        {
            gps_repo_t *r = gps_data_lock();
            r->sys_state = SYS_DISCOVERY;
            gps_data_unlock();
        }

        /* Etapa 3: Identidad, autenticidad y prueba de memoria BBR/EEPROM */
        stage3_identity_and_nvm_audit();

        /* Etapa 4: Matriz de capacidades soportadas vs rechazadas */
        stage4_capability_matrix();

        /* Etapa 5: Configuracion optima del receptor */
        stage5_optimal_configuration();

        {
            gps_repo_t *r = gps_data_lock();
            r->discovery_ms = gps_now_ms();
            r->sys_state    = SYS_OPERATING;
            if (r->phase == PH_NONE) {
                r->phase          = PH_A_BLIND;
                r->phase_since_ms = r->discovery_ms;
                r->warm_start_ms  = r->discovery_ms;
            }
            snprintf(r->status_line, sizeof(r->status_line),
                     "Operacion continua activa (%lu bps, %s)",
                     (unsigned long)r->final_baud,
                     r->bidir ? "Bidireccional UBX+NMEA" : "Solo Escucha NMEA");
            gps_data_unlock();
        }

        /* Bucle de supervision y sondeo en segundo plano durante la operacion */
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));

            uint32_t now = gps_now_ms();
            uint32_t last_b;
            bool     bidir, trusted, fast_active;
            {
                gps_repo_t *r = gps_data_lock();
                last_b      = r->last_byte_ms;
                bidir       = r->bidir;
                trusted     = (r->phase == PH_E_TRUSTED);
                fast_active = r->fast_nav_active;
                gps_warmup_refresh_channels(r);
                gps_data_unlock();
            }

            /* Detectar desconexion fisica en vivo (> 3 s sin bytes en GPIO 5) */
            if ((now - last_b) > GPS_LINK_LOST_MS) {
                gps_repo_t *r = gps_data_lock();
                r->link      = LINK_LOST;
                r->sys_state = SYS_WAIT_LINK;
                snprintf(r->status_line, sizeof(r->status_line),
                         "Enlace perdido en GPIO 5 (> %u ms sin datos): relanzando Auto-Bauding...",
                         (unsigned)GPS_LINK_LOST_MS);
                gps_data_unlock();
                break; /* vuelve a stage12_autobaud_and_link() */
            }

            if (!bidir) {
                continue;
            }

#if GPS_FAST_NAV_AFTER_TRUST
            /* Conmutacion opcional a 5 Hz (UBX-CFG-RATE: B5 62 06 08 06 00 C8 00 01 00 01 00 DE 6A) */
            if (trusted && !fast_active) {
                for (int i = 0; i < g_msg_table_len; i++) {
                    if ((g_msg_table[i].flags & MF_PERIODIC) && !(g_msg_table[i].flags & MF_NEG)) {
                        gps_cfg_set_msg_rate(g_msg_table[i].cls, g_msg_table[i].id,
                                             g_msg_table[i].rate_fast);
                    }
                }
                gps_hw_send_raw(k_ubx_cfg_rate_5hz, sizeof(k_ubx_cfg_rate_5hz));
                gps_repo_t *r = gps_data_lock();
                r->fast_nav_active = true;
                gps_data_unlock();
            }
#else
            (void)trusted;
            (void)fast_active;
            (void)k_ubx_cfg_rate_5hz;
#endif

            /* Sondeo escalonado en segundo plano (1 mensaje por segundo) */
            switch (bg_step % 8) {
            case 0:
                /* Sondeo de UBX-AID-HUI con su trama exacta B5 62 0B 02 00 00 0D 32 */
                gps_hw_send_raw(k_ubx_aid_hui_poll, sizeof(k_ubx_aid_hui_poll));
                break;
            case 1:
                gps_hw_send_ubx(UBX_CLS_MON, UBX_MON_IO, NULL, 0);
                break;
            case 2:
                gps_hw_send_ubx(UBX_CLS_MON, UBX_MON_RXBUF, NULL, 0);
                gps_hw_send_ubx(UBX_CLS_MON, UBX_MON_TXBUF, NULL, 0);
                break;
            case 3:
                gps_hw_send_ubx(UBX_CLS_MON, UBX_MON_MSGPP, NULL, 0);
                break;
            case 4:
                gps_hw_send_ubx(UBX_CLS_AID, UBX_AID_INI, NULL, 0);
                break;
            case 5:
                gps_hw_send_ubx(UBX_CLS_TIM, UBX_TIM_VRFY, NULL, 0);
                break;
            case 6:
                /* Sondear efemerides y almanaque de un PRN rotativo (1..32) */
                gps_hw_send_ubx(UBX_CLS_AID, UBX_AID_EPH, &sv_poll, 1);
                gps_hw_send_ubx(UBX_CLS_AID, UBX_AID_ALM, &sv_poll, 1);
                break;
            case 7:
                gps_hw_send_ubx(UBX_CLS_AID, UBX_AID_AOP, &sv_poll, 1);
                sv_poll = (uint8_t)((sv_poll % 32u) + 1u);
                break;
            }
            bg_step++;

            guard_txbuf_occupancy();
            evaluate_sbas_regional_policy();
        }
    }
}
