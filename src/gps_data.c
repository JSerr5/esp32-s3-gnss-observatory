/*
 * gps_data.c
 * ---------------------------------------------------------------------------
 * Repositorio central, tabla de capacidades, decodificacion semantica de UBX
 * y NMEA hacia el repositorio, sintetizador PVT y espera ACK/NAK.
 * ---------------------------------------------------------------------------
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_timer.h"
#include "driver/uart.h"
#include "gps_data.h"
#include "gps_warmup.h"
#include "ubx_protocol.h"

/* ======================================================================= */
/*                 TABLA DE MENSAJES (Matriz de Capacidades)               */
/* ======================================================================= */
#define P  MF_PERIODIC
#define Q  MF_POLL
#define N  MF_NEG
#define C  MF_CFGREAD
#define S  MF_PASSIVE
#define H  MF_HEAVY

const gps_msg_desc_t g_msg_table[] = {
    /* cls   id                  nombre            grupo     flags      norm lite fast  nota */
    { 0x01, UBX_NAV_POSECEF,   "NAV-POSECEF",   GRP_NAV, P|Q,        1, 0,  5, "ECEF X/Y/Z + pAcc" },
    { 0x01, UBX_NAV_POSLLH,    "NAV-POSLLH",    GRP_NAV, P|Q,        1, 1,  1, "Lat/Lon/Alt + hAcc/vAcc" },
    { 0x01, UBX_NAV_STATUS,    "NAV-STATUS",    GRP_NAV, P|Q,        1, 1,  1, "Fix, TTFF, msss" },
    { 0x01, UBX_NAV_DOP,       "NAV-DOP",       GRP_NAV, P|Q,        1, 1,  1, "7 DOP" },
    { 0x01, UBX_NAV_SOL,       "NAV-SOL",       GRP_NAV, P|Q,        1, 1,  1, "Solucion ECEF, numSV" },
    { 0x01, UBX_NAV_PVT,       "NAV-PVT",       GRP_NAV, P|Q|N,      0, 0,  0, "Prot>=14: se sintetiza" },
    { 0x01, UBX_NAV_VELECEF,   "NAV-VELECEF",   GRP_NAV, P|Q,        1, 0,  5, "Velocidad ECEF" },
    { 0x01, UBX_NAV_VELNED,    "NAV-VELNED",    GRP_NAV, P|Q,        1, 1,  1, "Velocidad NED, rumbo" },
    { 0x01, UBX_NAV_TIMEGPS,   "NAV-TIMEGPS",   GRP_NAV, P|Q,        1, 1,  5, "Semana, leapS" },
    { 0x01, UBX_NAV_TIMEUTC,   "NAV-TIMEUTC",   GRP_NAV, P|Q,        1, 1,  1, "Fecha/hora UTC" },
    { 0x01, UBX_NAV_CLOCK,     "NAV-CLOCK",     GRP_NAV, P|Q,        1, 1,  5, "Sesgo/deriva cristal" },
    { 0x01, UBX_NAV_SVINFO,    "NAV-SVINFO",    GRP_NAV, P|Q|H,      2, 5, 10, "Canales, C/N0, prRes" },
    { 0x01, UBX_NAV_DGPS,      "NAV-DGPS",      GRP_NAV, P|Q,        5, 0, 25, "Correcciones DGPS" },
    { 0x01, UBX_NAV_SBAS,      "NAV-SBAS",      GRP_NAV, P|Q,        5, 10, 25, "Estado SBAS" },
    { 0x01, UBX_NAV_AOPSTATUS, "NAV-AOPSTATUS", GRP_NAV, P|Q,       10, 0, 50, "AssistNow Autonomous" },
    { 0x01, UBX_NAV_EKFSTATUS, "NAV-EKFSTATUS", GRP_NAV, P|Q|N,      0, 0,  0, "Solo LEA-6R (DR)" },
    { 0x01, UBX_NAV_ODO,       "NAV-ODO",       GRP_NAV, P|Q|N,      0, 0,  0, "No existe en u-blox 6" },
    { 0x01, UBX_NAV_GEOFENCE,  "NAV-GEOFENCE",  GRP_NAV, P|Q|N,      0, 0,  0, "No existe en u-blox 6" },

    { 0x02, UBX_RXM_RAW,       "RXM-RAW",       GRP_RXM, P|Q|N,      0, 0,  0, "Licencia NEO-6P/6T" },
    { 0x02, UBX_RXM_SFRB,      "RXM-SFRB",      GRP_RXM, P|Q|N,      0, 0,  0, "Licencia NEO-6P/6T" },
    { 0x02, UBX_RXM_SVSI,      "RXM-SVSI",      GRP_RXM, P|Q,        5, 10, 25, "ephVal/almVal por SV" },
    { 0x02, UBX_RXM_ALM,       "RXM-ALM",       GRP_RXM, P|Q|H,      0, 0,  0, "Almanaque por SV" },
    { 0x02, UBX_RXM_EPH,       "RXM-EPH",       GRP_RXM, P|Q|H,      0, 0,  0, "Efemerides por SV" },

    { 0x0A, UBX_MON_IO,        "MON-IO",        GRP_MON, P|Q,        0, 0,  0, "Bytes/errores puertos" },
    { 0x0A, UBX_MON_VER,       "MON-VER",       GRP_MON, Q,          0, 0,  0, "Identidad SW/HW" },
    { 0x0A, UBX_MON_MSGPP,     "MON-MSGPP",     GRP_MON, P|Q,        0, 0,  0, "Mensajes por protocolo" },
    { 0x0A, UBX_MON_RXBUF,     "MON-RXBUF",     GRP_MON, P|Q,        0, 0,  0, "Buffer RX del GPS" },
    { 0x0A, UBX_MON_TXBUF,     "MON-TXBUF",     GRP_MON, P|Q,        5, 10, 25, "Buffer TX del GPS" },
    { 0x0A, UBX_MON_HW,        "MON-HW",        GRP_MON, P|Q,        2, 5, 10, "Antena, AGC, ruido, jam" },
    { 0x0A, UBX_MON_HW2,       "MON-HW2",       GRP_MON, P|Q,        5, 0, 25, "I/Q, cfgSource, POST" },
    { 0x0A, UBX_MON_RXR,       "MON-RXR",       GRP_MON, P|Q,        1, 0,  1, "Receptor despierto" },

    { 0x0D, UBX_TIM_TP,        "TIM-TP",        GRP_TIM, P|Q,        1, 0,  5, "Proximo PPS, qErr" },
    { 0x0D, UBX_TIM_TM2,       "TIM-TM2",       GRP_TIM, P|Q,        1, 0,  5, "Marcas EXTINT0" },
    { 0x0D, UBX_TIM_VRFY,      "TIM-VRFY",      GRP_TIM, P|Q,        0, 0,  0, "Verificacion de tiempo" },
    { 0x0D, UBX_TIM_SVIN,      "TIM-SVIN",      GRP_TIM, P|Q|N,      0, 0,  0, "Exclusivo NEO-6T" },

    { 0x0B, UBX_AID_INI,       "AID-INI",       GRP_AID, Q,          0, 0,  0, "Inicializacion pos/tiempo" },
    { 0x0B, UBX_AID_HUI,       "AID-HUI",       GRP_AID, Q,          0, 0,  0, "Salud, UTC, Klobuchar" },
    { 0x0B, UBX_AID_ALM,       "AID-ALM",       GRP_AID, Q|H,        0, 0,  0, "Almanaques" },
    { 0x0B, UBX_AID_EPH,       "AID-EPH",       GRP_AID, Q|H,        0, 0,  0, "Efemerides" },
    { 0x0B, UBX_AID_AOP,       "AID-AOP",       GRP_AID, Q|H,        0, 0,  0, "Orbitas AssistNow Aut." },

    { 0x06, UBX_CFG_PRT,       "CFG-PRT",       GRP_CFG, C,          0, 0,  0, "Puerto UART1" },
    { 0x06, UBX_CFG_RATE,      "CFG-RATE",      GRP_CFG, C,          0, 0,  0, "Tasa de navegacion" },
    { 0x06, UBX_CFG_NAV5,      "CFG-NAV5",      GRP_CFG, C,          0, 0,  0, "Motor de navegacion" },
    { 0x06, UBX_CFG_NAVX5,     "CFG-NAVX5",     GRP_CFG, C,          0, 0,  0, "Nav. expertos" },
    { 0x06, UBX_CFG_SBAS,      "CFG-SBAS",      GRP_CFG, C,          0, 0,  0, "SBAS" },
    { 0x06, UBX_CFG_NMEA,      "CFG-NMEA",      GRP_CFG, C,          0, 0,  0, "Protocolo NMEA" },
    { 0x06, UBX_CFG_TP5,       "CFG-TP5",       GRP_CFG, C,          0, 0,  0, "TIMEPULSE" },
    { 0x06, UBX_CFG_ITFM,      "CFG-ITFM",      GRP_CFG, C,          0, 0,  0, "Monitor jamming" },
    { 0x06, UBX_CFG_ANT,       "CFG-ANT",       GRP_CFG, C,          0, 0,  0, "Supervisor antena" },
    { 0x06, UBX_CFG_RXM,       "CFG-RXM",       GRP_CFG, C,          0, 0,  0, "Modo de energia" },
    { 0x06, UBX_CFG_PM2,       "CFG-PM2",       GRP_CFG, C,          0, 0,  0, "Ahorro de energia" },
    { 0x06, UBX_CFG_DAT,       "CFG-DAT",       GRP_CFG, C,          0, 0,  0, "Datum" },
    { 0x06, UBX_CFG_GNSS,      "CFG-GNSS",      GRP_CFG, C|N,        0, 0,  0, "No existe en ROM 7.03" },

    { 0xF0, NMEA_ID_GGA,       "GPGGA",         GRP_NMEA, P,         1, 1,  5, "Fix, calidad, altitud" },
    { 0xF0, NMEA_ID_GLL,       "GPGLL",         GRP_NMEA, P,         1, 0,  5, "Lat/Lon, estado" },
    { 0xF0, NMEA_ID_GSA,       "GPGSA",         GRP_NMEA, P,         1, 5,  5, "PRN usados, DOP" },
    { 0xF0, NMEA_ID_GSV,       "GPGSV",         GRP_NMEA, P|H,       5, 10, 25, "SV en vista" },
    { 0xF0, NMEA_ID_RMC,       "GPRMC",         GRP_NMEA, P,         1, 1,  1, "Minimo recomendado" },
    { 0xF0, NMEA_ID_VTG,       "GPVTG",         GRP_NMEA, P,         1, 0,  5, "Rumbo/velocidad" },
    { 0xF0, NMEA_ID_GRS,       "GPGRS",         GRP_NMEA, P,         1, 0,  5, "Residuales de rango" },
    { 0xF0, NMEA_ID_GST,       "GPGST",         GRP_NMEA, P,         1, 0,  5, "Errores std" },
    { 0xF0, NMEA_ID_ZDA,       "GPZDA",         GRP_NMEA, P,         1, 0,  5, "Fecha/hora" },
    { 0xF0, NMEA_ID_GBS,       "GPGBS",         GRP_NMEA, P,         1, 0,  5, "RAIM" },
    { 0xF0, NMEA_ID_DTM,       "GPDTM",         GRP_NMEA, P,         5, 0, 25, "Datum" },
    { 0xF0, 0x41,              "GPTXT",         GRP_NMEA, P|S,       1, 1,  1, "Texto (CFG-INF)" },
    { 0xF1, PUBX_ID_00,        "PUBX,00",       GRP_PUBX, P,         1, 0,  5, "Posicion u-blox" },
    { 0xF1, PUBX_ID_03,        "PUBX,03",       GRP_PUBX, P|H,       2, 0, 10, "Satelites u-blox" },
    { 0xF1, PUBX_ID_04,        "PUBX,04",       GRP_PUBX, P,         1, 5,  5, "Reloj u-blox" },

    { 0x04, UBX_INF_ERROR,     "INF-ERROR",     GRP_INF, S,          0, 0,  0, "Texto error" },
    { 0x04, UBX_INF_WARNING,   "INF-WARNING",   GRP_INF, S,          0, 0,  0, "Texto aviso" },
    { 0x04, UBX_INF_NOTICE,    "INF-NOTICE",    GRP_INF, S,          0, 0,  0, "Texto noticia" },
    { 0x04, UBX_INF_TEST,      "INF-TEST",      GRP_INF, S,          0, 0,  0, "Texto prueba" },
    { 0x04, UBX_INF_DEBUG,     "INF-DEBUG",     GRP_INF, S,          0, 0,  0, "Texto depuracion" },

    { 0x21, 0x08,              "LOG-INFO",      GRP_NAV, Q|N,        0, 0,  0, "Clase LOG inexistente" },
};
const int g_msg_table_len = (int)(sizeof(g_msg_table) / sizeof(g_msg_table[0]));

_Static_assert(sizeof(g_msg_table) / sizeof(g_msg_table[0]) <= 96, "mstat[] demasiado pequeno");

#undef P
#undef Q
#undef N
#undef C
#undef S
#undef H

int gps_msg_index(uint8_t cls, uint8_t id)
{
    for (int i = 0; i < g_msg_table_len; i++) {
        if (g_msg_table[i].cls == cls && g_msg_table[i].id == id) return i;
    }
    return -1;
}

/* ======================================================================= */
/*                         ESTADO DEL MODULO                               */
/* ======================================================================= */
static gps_repo_t          s_repo;
static SemaphoreHandle_t   s_mtx;
static StaticSemaphore_t   s_mtx_buf;
static EventGroupHandle_t  s_evt;
static StaticEventGroup_t  s_evt_buf;

/* espera determinista */
static volatile bool    s_pend_active;
static volatile uint8_t s_pend_cls, s_pend_id, s_pend_rcls, s_pend_rid;
static volatile bool    s_pend_want_resp;
static uint8_t          s_resp_buf[GPS_RESP_BUF];
static volatile uint16_t s_resp_len;

/* PVT en construccion */
static gps_pvt_t s_work;
static bool      s_work_open;

/* agregador GSV */
static gps_gsv_sat_t s_gsv_tmp[GPS_MAX_GSV];
static int           s_gsv_tmp_n;
static int           s_gsv_expect_next;

uint32_t gps_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void gps_data_init(void)
{
    memset(&s_repo, 0, sizeof(s_repo));
    s_mtx = xSemaphoreCreateMutexStatic(&s_mtx_buf);
    s_evt = xEventGroupCreateStatic(&s_evt_buf);
    s_repo.boot_ms = gps_now_ms();
    s_repo.sys_state = SYS_BOOT;
    s_repo.link = LINK_SEARCHING;
    s_repo.phase = PH_NONE;
    strcpy(s_repo.antstatus, "?");
    strcpy(s_repo.protver, "?");
    s_repo.hacc_min = UINT32_MAX;
    s_repo.vacc_min = UINT32_MAX;
    s_repo.sacc_min = UINT32_MAX;
    s_repo.pdop_min = UINT16_MAX;
    s_repo.hdop_min = UINT16_MAX;
    s_repo.vdop_min = UINT16_MAX;
    s_work_open = false;
}

gps_repo_t *gps_data_lock(void)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    return &s_repo;
}

void gps_data_unlock(void)
{
    xSemaphoreGive(s_mtx);
}

void gps_data_snapshot(gps_repo_t *dst)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    memcpy(dst, &s_repo, sizeof(*dst));
    xSemaphoreGive(s_mtx);
}

EventGroupHandle_t gps_data_events(void)
{
    return s_evt;
}

/* ---------------------------- pendientes ------------------------------- */
void gps_pending_set(uint8_t cls, uint8_t id, uint8_t rcls, uint8_t rid, bool want_resp)
{
    xEventGroupClearBits(s_evt, EVT_ACK | EVT_NAK | EVT_RESP);
    s_resp_len = 0;
    s_pend_cls = cls;
    s_pend_id = id;
    s_pend_rcls = rcls;
    s_pend_rid = rid;
    s_pend_want_resp = want_resp;
    s_pend_active = true;
}

void gps_pending_clear(void)
{
    s_pend_active = false;
}

uint16_t gps_pending_resp(uint8_t *buf, uint16_t cap)
{
    uint16_t n = s_resp_len;
    if (n > cap) n = cap;
    if (buf && n) memcpy(buf, s_resp_buf, n);
    return n;
}

/* ------------------------------ textos --------------------------------- */
void gps_data_log_txt(gps_repo_t *r, const char *src, const char *text)
{
    gps_txt_entry_t *e = &r->txt[r->txt_head];
    snprintf(e->src, sizeof(e->src), "%s", src);
    snprintf(e->text, sizeof(e->text), "%s", text);
    e->ms = gps_now_ms();
    r->txt_head = (uint8_t)((r->txt_head + 1) % GPS_TXT_LOG);
    if (r->txt_count < GPS_TXT_LOG) r->txt_count++;
}

const char *gps_res_str(gps_res_t r)
{
    switch (r) {
    case RES_ACK:     return "ACK";
    case RES_NAK:     return "NAK";
    case RES_TIMEOUT: return "T.OUT";
    case RES_DATA:    return "DATOS";
    case RES_NA:      return "-";
    default:          return "...";
    }
}

const char *gps_group_name(uint8_t g)
{
    static const char *n[GRP_COUNT] = { "NAV", "RXM", "MON", "TIM", "AID", "CFG", "NMEA", "PUBX", "INF" };
    return (g < GRP_COUNT) ? n[g] : "?";
}

const char *gps_phase_name(gps_phase_t p)
{
    switch (p) {
    case PH_A_BLIND:    return "FASE A - Busqueda ciega / estabilizacion HW";
    case PH_B_SIGNALS:  return "FASE B - Portadoras detectadas / descarga de efemerides";
    case PH_C_TIME:     return "FASE C - Tiempo GPS sincronizado (sin posicion)";
    case PH_D_CONVERGE: return "FASE D - Primer fix: convergencia del filtro Kalman";
    case PH_E_TRUSTED:  return "FASE E - Regimen estacionario CONFIABLE";
    default:            return "Sin fase (enlace no establecido)";
    }
}

const char *gps_fix_name(uint8_t fix)
{
    switch (fix) {
    case 0:  return "Sin Fix";
    case 1:  return "Solo Dead Reckoning";
    case 2:  return "Fix 2D";
    case 3:  return "Fix 3D";
    case 4:  return "GPS + DR";
    case 5:  return "Solo Tiempo";
    default: return "Desconocido";
    }
}

/* ======================================================================= */
/*                    SINTETIZADOR PVT (NAV-PVT virtual)                    */
/* ======================================================================= */
static void pvt_close(gps_repo_t *r)
{
    if (!s_work_open) return;
    uint64_t now_us = (uint64_t)esp_timer_get_time();
    s_work.closed_ms = (uint32_t)(now_us / 1000ULL);
    if (r->epoch_start_us > 0 && now_us >= r->epoch_start_us) {
        r->epoch_ubx_build_us   = (uint32_t)(now_us - r->epoch_start_us);
        r->epoch_burst_total_us = r->epoch_ubx_build_us;
    }
    r->epoch_frames_cnt = r->epoch_work_frames;
    r->epoch_bytes_cnt  = r->bytes_rx - r->epoch_start_bytes;
    r->pvt = s_work;
    r->pvt_valid = true;
    r->pvt_epochs++;
    s_work_open = false;
    gps_warmup_on_epoch(r, s_work.closed_ms);
    xEventGroupSetBits(s_evt, EVT_EPOCH_READY);
}

static void pvt_begin(gps_repo_t *r, uint32_t itow)
{
    if (s_work_open && s_work.iTOW != itow) {
        pvt_close(r);
    }
    if (!s_work_open) {
        /* arrastra los ultimos valores conocidos; la mascara indica frescura */
        s_work = r->pvt;
        s_work.iTOW = itow;
        s_work.mask = 0;
        s_work_open = true;
        r->epoch_start_us      = (uint64_t)esp_timer_get_time();
        r->epoch_last_frame_us = r->epoch_start_us;
        r->epoch_start_bytes   = r->bytes_rx;
        r->epoch_work_frames   = 0;
    }
}

static void pvt_end(gps_repo_t *r)
{
    if (s_work_open && s_work.mask == PVT_ALL) {
        pvt_close(r);
    }
}

/* ======================================================================= */
/*                         DECODIFICACION UBX                              */
/* ======================================================================= */
#define COPY_SLOT(slot, p, len)                                 \
    do {                                                        \
        if ((len) == sizeof((slot).d)) {                        \
            memcpy(&(slot).d, (p), sizeof((slot).d));           \
            (slot).valid = true;                                \
            (slot).ms = now;                                    \
        }                                                       \
    } while (0)

static void copy_str(char *dst, size_t cap, const uint8_t *src, size_t n)
{
    size_t i = 0;
    for (; i < n && i + 1 < cap; i++) {
        char c = (char)src[i];
        if (c == '\0') break;
        dst[i] = (c >= 0x20 && c <= 0x7E) ? c : '.';
    }
    dst[i] = '\0';
    /* recorta espacios finales */
    while (i > 0 && dst[i - 1] == ' ') dst[--i] = '\0';
}

static void decode_mon_ver(gps_repo_t *r, const uint8_t *p, uint16_t len)
{
    if (len < 40) return;
    copy_str(r->swVersion, sizeof(r->swVersion), p, 30);
    copy_str(r->hwVersion, sizeof(r->hwVersion), p + 30, 10);
    r->n_ext = 0;
    for (uint16_t off = 40; off + 30 <= len && r->n_ext < GPS_MAX_VER_EXT; off += 30) {
        copy_str(r->ext[r->n_ext], sizeof(r->ext[0]), p + off, 30);
        const char *pv = strstr(r->ext[r->n_ext], "PROTVER");
        if (pv) {
            pv += 7;
            while (*pv == ' ' || *pv == '=' || *pv == ':') pv++;
            snprintf(r->protver, sizeof(r->protver), "%.7s", pv);
            r->protver_verified = true;
        }
        r->n_ext++;
    }
    r->ver_valid = true;
}

static void decode_svinfo(gps_repo_t *r, const uint8_t *p, uint16_t len, uint32_t now)
{
    if (len < 8) return;
    memcpy(&r->svinfo.h, p, 8);
    uint16_t n = r->svinfo.h.numCh;
    if ((uint32_t)len < 8u + 12u * n) n = (uint16_t)((len - 8) / 12);
    if (n > GPS_MAX_SVINFO_CH) n = GPS_MAX_SVINFO_CH;
    memcpy(r->svinfo.b, p + 8, (size_t)n * 12);
    r->svinfo.n = (uint8_t)n;
    r->svinfo.valid = true;
    r->svinfo.ms = now;
}

static void decode_svsi(gps_repo_t *r, const uint8_t *p, uint16_t len, uint32_t now)
{
    if (len < 8) return;
    memcpy(&r->svsi.h, p, 8);
    uint16_t n = r->svsi.h.numSV;
    if ((uint32_t)len < 8u + 6u * n) n = (uint16_t)((len - 8) / 6);
    if (n > GPS_MAX_SVSI) n = GPS_MAX_SVSI;
    memcpy(r->svsi.b, p + 8, (size_t)n * 6);
    r->svsi.n = (uint8_t)n;
    r->svsi.valid = true;
    r->svsi.ms = now;
}

static void decode_dgps(gps_repo_t *r, const uint8_t *p, uint16_t len, uint32_t now)
{
    if (len < 16) return;
    memcpy(&r->dgps.h, p, 16);
    uint16_t n = r->dgps.h.numCh;
    if ((uint32_t)len < 16u + 12u * n) n = (uint16_t)((len - 16) / 12);
    if (n > GPS_MAX_DGPS_CH) n = GPS_MAX_DGPS_CH;
    memcpy(r->dgps.b, p + 16, (size_t)n * 12);
    r->dgps.n = (uint8_t)n;
    r->dgps.valid = true;
    r->dgps.ms = now;
}

static void decode_sbas(gps_repo_t *r, const uint8_t *p, uint16_t len, uint32_t now)
{
    if (len < 12) return;
    memcpy(&r->sbas.h, p, 12);
    uint16_t n = r->sbas.h.cnt;
    if ((uint32_t)len < 12u + 12u * n) n = (uint16_t)((len - 12) / 12);
    if (n > GPS_MAX_SBAS_SV) n = GPS_MAX_SBAS_SV;
    memcpy(r->sbas.b, p + 12, (size_t)n * 12);
    r->sbas.n = (uint8_t)n;
    r->sbas.valid = true;
    r->sbas.ms = now;
}

static void decode_inf(gps_repo_t *r, uint8_t id, const uint8_t *p, uint16_t len)
{
    static const char *src[5] = { "INF-ERR", "INF-WARN", "INF-NOTE", "INF-TEST", "INF-DBG" };
    char txt[NMEA_TXT_LEN];
    copy_str(txt, sizeof(txt), p, len);
    if (id < 5) {
        r->inf_count[id]++;
        gps_data_log_txt(r, src[id], txt);
    }
}

static void decode_ubx(gps_repo_t *r, uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len, uint32_t now)
{
    switch (cls) {
    /* ------------------------------- NAV ------------------------------- */
    case UBX_CLS_NAV:
        switch (id) {
        case UBX_NAV_POSECEF:
            COPY_SLOT(r->posecef, p, len);
            if (len == 20) {
                pvt_begin(r, r->posecef.d.iTOW);
                s_work.ecefX = r->posecef.d.ecefX;
                s_work.ecefY = r->posecef.d.ecefY;
                s_work.ecefZ = r->posecef.d.ecefZ;
                s_work.pAcc  = r->posecef.d.pAcc;
            }
            break;
        case UBX_NAV_POSLLH:
            COPY_SLOT(r->posllh, p, len);
            if (len == 28) {
                const ubx_nav_posllh_t *d = &r->posllh.d;
                pvt_begin(r, d->iTOW);
                s_work.lat = d->lat; s_work.lon = d->lon;
                s_work.height = d->height; s_work.hMSL = d->hMSL;
                s_work.hAcc = d->hAcc; s_work.vAcc = d->vAcc;
                s_work.mask |= PVT_POSLLH;
                pvt_end(r);
            }
            break;
        case UBX_NAV_STATUS:
            COPY_SLOT(r->status, p, len);
            if (len == 16) {
                const ubx_nav_status_t *d = &r->status.d;
                pvt_begin(r, d->iTOW);
                s_work.gpsFix_st = d->gpsFix; s_work.flags_st = d->flags;
                s_work.fixStat = d->fixStat; s_work.flags2 = d->flags2;
                s_work.ttff = d->ttff; s_work.msss = d->msss;
                s_work.mask |= PVT_STATUS;
                pvt_end(r);
            }
            break;
        case UBX_NAV_DOP:
            COPY_SLOT(r->dop, p, len);
            if (len == 18) {
                const ubx_nav_dop_t *d = &r->dop.d;
                pvt_begin(r, d->iTOW);
                s_work.gDOP = d->gDOP; s_work.pDOP = d->pDOP; s_work.tDOP = d->tDOP;
                s_work.vDOP = d->vDOP; s_work.hDOP = d->hDOP; s_work.nDOP = d->nDOP;
                s_work.eDOP = d->eDOP;
                s_work.mask |= PVT_DOP;
                pvt_end(r);
            }
            break;
        case UBX_NAV_SOL:
            COPY_SLOT(r->sol, p, len);
            if (len == 52) {
                const ubx_nav_sol_t *d = &r->sol.d;
                pvt_begin(r, d->iTOW);
                s_work.fTOW = d->fTOW; s_work.week = d->week;
                s_work.gpsFix_sol = d->gpsFix; s_work.flags_sol = d->flags;
                s_work.ecefX = d->ecefX; s_work.ecefY = d->ecefY; s_work.ecefZ = d->ecefZ;
                s_work.pAcc = d->pAcc;
                s_work.ecefVX = d->ecefVX; s_work.ecefVY = d->ecefVY; s_work.ecefVZ = d->ecefVZ;
                s_work.sAcc = d->sAcc;
                s_work.pDOP_sol = d->pDOP;
                s_work.numSV = d->numSV;
                s_work.mask |= PVT_SOL;
                pvt_end(r);
            }
            break;
        case UBX_NAV_VELECEF:
            COPY_SLOT(r->velecef, p, len);
            break;
        case UBX_NAV_VELNED:
            COPY_SLOT(r->velned, p, len);
            if (len == 36) {
                const ubx_nav_velned_t *d = &r->velned.d;
                pvt_begin(r, d->iTOW);
                s_work.velN = d->velN; s_work.velE = d->velE; s_work.velD = d->velD;
                s_work.speed = d->speed; s_work.gSpeed = d->gSpeed;
                s_work.heading = d->heading;
                s_work.sAcc = d->sAcc; s_work.cAcc = d->cAcc;
                s_work.mask |= PVT_VELNED;
                pvt_end(r);
            }
            break;
        case UBX_NAV_TIMEGPS:
            COPY_SLOT(r->timegps, p, len);
            if (len == 16) {
                const ubx_nav_timegps_t *d = &r->timegps.d;
                pvt_begin(r, d->iTOW);
                s_work.leapS = d->leapS; s_work.tgps_valid = d->valid;
                s_work.tgps_tAcc = d->tAcc;
                s_work.week = d->week; s_work.fTOW = d->fTOW;
                s_work.mask |= PVT_TIMEGPS;
                pvt_end(r);
            }
            break;
        case UBX_NAV_TIMEUTC:
            COPY_SLOT(r->timeutc, p, len);
            if (len == 20) {
                const ubx_nav_timeutc_t *d = &r->timeutc.d;
                pvt_begin(r, d->iTOW);
                s_work.year = d->year; s_work.month = d->month; s_work.day = d->day;
                s_work.hour = d->hour; s_work.min = d->min; s_work.sec = d->sec;
                s_work.nano = d->nano; s_work.utc_valid = d->valid;
                s_work.utc_tAcc = d->tAcc;
                s_work.mask |= PVT_TIMEUTC;
                pvt_end(r);
            }
            break;
        case UBX_NAV_CLOCK:
            COPY_SLOT(r->clock, p, len);
            if (len == 20) {
                const ubx_nav_clock_t *d = &r->clock.d;
                pvt_begin(r, d->iTOW);
                s_work.clkB = d->clkB; s_work.clkD = d->clkD;
                s_work.clk_tAcc = d->tAcc; s_work.fAcc = d->fAcc;
                s_work.mask |= PVT_CLOCK;
                pvt_end(r);
            }
            break;
        case UBX_NAV_SVINFO:   decode_svinfo(r, p, len, now); break;
        case UBX_NAV_DGPS:     decode_dgps(r, p, len, now);   break;
        case UBX_NAV_SBAS:     decode_sbas(r, p, len, now);   break;
        case UBX_NAV_AOPSTATUS: COPY_SLOT(r->aopstatus, p, len); break;
        default: break;
        }
        break;

    /* ------------------------------- RXM ------------------------------- */
    case UBX_CLS_RXM:
        if (id == UBX_RXM_SVSI) {
            decode_svsi(r, p, len, now);
        } else if (id == UBX_RXM_ALM && len >= 8) {
            uint32_t sv = ubx_u4(p);
            if (sv >= 1 && sv <= 32) r->alm_week[sv] = (len >= 40) ? ubx_u4(p + 4) : 0;
            r->alm_ms = now;
            r->alm_resp++;
        } else if (id == UBX_RXM_EPH && len >= 8) {
            uint32_t sv = ubx_u4(p);
            if (sv >= 1 && sv <= 32) r->eph_how[sv] = (len >= 104) ? ubx_u4(p + 4) : 0;
            r->eph_ms = now;
            r->eph_resp++;
        }
        break;

    /* ------------------------------- INF ------------------------------- */
    case UBX_CLS_INF:
        decode_inf(r, id, p, len);
        break;

    /* ------------------------------- MON ------------------------------- */
    case UBX_CLS_MON:
        switch (id) {
        case UBX_MON_VER:   decode_mon_ver(r, p, len); break;
        case UBX_MON_HW:    COPY_SLOT(r->hw, p, len); break;
        case UBX_MON_HW2:   COPY_SLOT(r->hw2, p, len); break;
        case UBX_MON_MSGPP: COPY_SLOT(r->msgpp, p, len); break;
        case UBX_MON_RXBUF: COPY_SLOT(r->rxbuf, p, len); break;
        case UBX_MON_TXBUF:
            COPY_SLOT(r->txbuf, p, len);
            if (len == 28 && r->txbuf.d.tUsage > r->txbuf_peak_seen) {
                r->txbuf_peak_seen = r->txbuf.d.tUsage;
            }
            break;
        case UBX_MON_RXR:   COPY_SLOT(r->rxr, p, len); break;
        case UBX_MON_IO: {
            uint8_t n = (uint8_t)(len / 20);
            if (n > GPS_MAX_IO_PORTS) n = GPS_MAX_IO_PORTS;
            memcpy(r->io.p, p, (size_t)n * 20);
            r->io.n = n;
            r->io.valid = (n > 0);
            r->io.ms = now;
            break;
        }
        default: break;
        }
        break;

    /* ------------------------------- TIM ------------------------------- */
    case UBX_CLS_TIM:
        if (id == UBX_TIM_TP)   COPY_SLOT(r->tp, p, len);
        if (id == UBX_TIM_TM2)  COPY_SLOT(r->tm2, p, len);
        if (id == UBX_TIM_VRFY) COPY_SLOT(r->vrfy, p, len);
        break;

    /* ------------------------------- AID ------------------------------- */
    case UBX_CLS_AID:
        switch (id) {
        case UBX_AID_HUI:
            COPY_SLOT(r->hui, p, len);
            if (len == 72) r->klob_valid = (r->hui.d.flags & 0x04) != 0;
            break;
        case UBX_AID_INI:
            COPY_SLOT(r->ini, p, len);
            break;
        case UBX_AID_ALM:
            if (len >= 8) {
                uint32_t sv = ubx_u4(p);
                if (sv >= 1 && sv <= 32) r->alm_week[sv] = (len >= 40) ? ubx_u4(p + 4) : 0;
                r->alm_ms = now;
                r->alm_resp++;
            }
            break;
        case UBX_AID_EPH:
            if (len >= 8) {
                uint32_t sv = ubx_u4(p);
                if (sv >= 1 && sv <= 32) r->eph_how[sv] = (len >= 104) ? ubx_u4(p + 4) : 0;
                r->eph_ms = now;
                r->eph_resp++;
            }
            break;
        case UBX_AID_AOP:
            if (len >= 1) {
                uint8_t sv = p[0];
                if (sv >= 1 && sv <= 32) {
                    if (len > 8) r->aop_mask |= (1u << (sv - 1));
                    else         r->aop_mask &= ~(1u << (sv - 1));
                }
                r->aop_resp++;
                r->aop_ms = now;
            }
            break;
        default: break;
        }
        break;

    /* ------------------------------- CFG ------------------------------- */
    case UBX_CLS_CFG:
        switch (id) {
        case UBX_CFG_PRT:   if (len >= 20 && p[0] == 1) COPY_SLOT(r->cfg_prt, p, 20); break;
        case UBX_CFG_RATE:  COPY_SLOT(r->cfg_rate, p, len);  break;
        case UBX_CFG_NAV5:  COPY_SLOT(r->cfg_nav5, p, len);  break;
        case UBX_CFG_NAVX5: COPY_SLOT(r->cfg_navx5, p, len); break;
        case UBX_CFG_SBAS:  COPY_SLOT(r->cfg_sbas, p, len);  break;
        case UBX_CFG_NMEA:  COPY_SLOT(r->cfg_nmea, p, len);  break;
        case UBX_CFG_TP5:   COPY_SLOT(r->cfg_tp5, p, len);   break;
        case UBX_CFG_ITFM:  COPY_SLOT(r->cfg_itfm, p, len);  break;
        case UBX_CFG_ANT:   COPY_SLOT(r->cfg_ant, p, len);   break;
        case UBX_CFG_RXM:   COPY_SLOT(r->cfg_rxm, p, len);   break;
        case UBX_CFG_PM2:   COPY_SLOT(r->cfg_pm2, p, len);   break;
        case UBX_CFG_DAT:   COPY_SLOT(r->cfg_dat, p, len);   break;
        default: break;
        }
        break;

    default:
        break;
    }
}

void gps_data_on_ubx(uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len)
{
    uint32_t now = gps_now_ms();
    EventBits_t bits = EVT_ANY_UBX;

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    gps_repo_t *r = &s_repo;
    r->last_frame_ms = now;
    r->epoch_work_frames++;
    r->epoch_last_frame_us = (uint64_t)esp_timer_get_time();
    if (!s_work_open && r->epoch_start_us > 0 && r->epoch_last_frame_us >= r->epoch_start_us) {
        uint32_t dt = (uint32_t)(r->epoch_last_frame_us - r->epoch_start_us);
        if (dt < 500000u) {
            r->epoch_burst_total_us = dt;
            r->epoch_frames_cnt     = r->epoch_work_frames;
            r->epoch_bytes_cnt      = r->bytes_rx - r->epoch_start_bytes;
        }
    }

    if (cls == UBX_CLS_ACK && len >= 2) {
        if (s_pend_active && p[0] == s_pend_cls && p[1] == s_pend_id) {
            bits |= (id == UBX_ACK_ACK) ? EVT_ACK : EVT_NAK;
        }
    } else {
        int idx = gps_msg_index(cls, id);
        if (idx >= 0) {
            r->mstat[idx].rx_count++;
            r->mstat[idx].last_rx_ms = now;
            r->mstat[idx].last_len = len;
        }
        decode_ubx(r, cls, id, p, len, now);
        if (s_pend_active && s_pend_want_resp && cls == s_pend_rcls && id == s_pend_rid) {
            uint16_t n = (len > GPS_RESP_BUF) ? GPS_RESP_BUF : len;
            memcpy(s_resp_buf, p, n);
            s_resp_len = n;
            bits |= EVT_RESP;
        }
    }
    xSemaphoreGive(s_mtx);
    xEventGroupSetBits(s_evt, bits);
}

/* ======================================================================= */
/*                         DECODIFICACION NMEA                             */
/* ======================================================================= */
static uint8_t nmea_cfg_id(nmea_sentence_t s, uint8_t *cls)
{
    *cls = NMEA_CLS_STD;
    switch (s) {
    case NMEA_S_GGA: return NMEA_ID_GGA;
    case NMEA_S_GLL: return NMEA_ID_GLL;
    case NMEA_S_GSA: return NMEA_ID_GSA;
    case NMEA_S_GSV: return NMEA_ID_GSV;
    case NMEA_S_RMC: return NMEA_ID_RMC;
    case NMEA_S_VTG: return NMEA_ID_VTG;
    case NMEA_S_GRS: return NMEA_ID_GRS;
    case NMEA_S_GST: return NMEA_ID_GST;
    case NMEA_S_ZDA: return NMEA_ID_ZDA;
    case NMEA_S_GBS: return NMEA_ID_GBS;
    case NMEA_S_DTM: return NMEA_ID_DTM;
    case NMEA_S_TXT: return 0x41;
    case NMEA_S_PUBX00: *cls = NMEA_CLS_PUBX; return PUBX_ID_00;
    case NMEA_S_PUBX03: *cls = NMEA_CLS_PUBX; return PUBX_ID_03;
    case NMEA_S_PUBX04: *cls = NMEA_CLS_PUBX; return PUBX_ID_04;
    default: *cls = 0; return 0xFF;
    }
}

static uint16_t talker_bit(const char t[3])
{
    if (t[0] == 'G' && t[1] == 'P') return 1u << 0;
    if (t[0] == 'G' && t[1] == 'N') return 1u << 1;
    if (t[0] == 'G' && t[1] == 'L') return 1u << 2;
    if (t[0] == 'G' && t[1] == 'A') return 1u << 3;
    if ((t[0] == 'B' && t[1] == 'D') || (t[0] == 'G' && t[1] == 'B')) return 1u << 4;
    if (t[0] == 'Q' && t[1] == 'Z') return 1u << 5;
    if (t[0] == 'P' && t[1] == 'U') return 0;
    return 1u << 6;
}

static void handle_txt(gps_repo_t *r, const nmea_txt_t *t)
{
    gps_data_log_txt(r, "GPTXT", t->text);
    const char *s;
    if (strstr(t->text, "u-blox")) r->txt_ublox = true;
    if ((s = strstr(t->text, "HW ")) != NULL || (s = strstr(t->text, "HW\t")) != NULL) {
        snprintf(r->txt_hw, sizeof(r->txt_hw), "%s", s);
    }
    if ((s = strstr(t->text, "ROM CORE")) != NULL) {
        snprintf(r->txt_rom, sizeof(r->txt_rom), "%s", s);
    }
    if ((s = strstr(t->text, "PROTVER")) != NULL) {
        s += 7;
        while (*s == ' ' || *s == '=') s++;
        snprintf(r->protver, sizeof(r->protver), "%.7s", s);
        r->protver_verified = true;
    }
    if ((s = strstr(t->text, "ANTSTATUS=")) != NULL) {
        snprintf(r->antstatus, sizeof(r->antstatus), "%s", s + 10);
    }
    if ((s = strstr(t->text, "ANTSUPERV=")) != NULL) {
        snprintf(r->antsuperv, sizeof(r->antsuperv), "%s", s + 10);
    }
}

void gps_data_on_nmea(nmea_parser_t *np)
{
    static char *fields[NMEA_MAX_FIELDS];
    /* estructuras temporales estaticas: solo la tarea RX llama aqui */
    static union {
        nmea_gga_t g; nmea_gll_t l; nmea_gsa_t a; nmea_gsv_t v; nmea_rmc_t m;
        nmea_vtg_t t; nmea_grs_t rs; nmea_gst_t st; nmea_zda_t z; nmea_gbs_t b;
        nmea_dtm_t d; nmea_txt_t x; nmea_pubx00_t p0; nmea_pubx03_t p3; nmea_pubx04_t p4;
    } u;

    int n = nmea_tokenize(np, fields, NMEA_MAX_FIELDS);
    char talker[3];
    nmea_sentence_t s = nmea_identify(fields, n, talker);
    uint32_t now = gps_now_ms();
    bool ok = false;

    /* decodificacion fuera del mutex */
    switch (s) {
    case NMEA_S_GGA:    ok = nmea_parse_gga(fields, n, &u.g);  break;
    case NMEA_S_GLL:    ok = nmea_parse_gll(fields, n, &u.l);  break;
    case NMEA_S_GSA:    ok = nmea_parse_gsa(fields, n, &u.a);  break;
    case NMEA_S_GSV:    ok = nmea_parse_gsv(fields, n, &u.v);  break;
    case NMEA_S_RMC:    ok = nmea_parse_rmc(fields, n, &u.m);  break;
    case NMEA_S_VTG:    ok = nmea_parse_vtg(fields, n, &u.t);  break;
    case NMEA_S_GRS:    ok = nmea_parse_grs(fields, n, &u.rs); break;
    case NMEA_S_GST:    ok = nmea_parse_gst(fields, n, &u.st); break;
    case NMEA_S_ZDA:    ok = nmea_parse_zda(fields, n, &u.z);  break;
    case NMEA_S_GBS:    ok = nmea_parse_gbs(fields, n, &u.b);  break;
    case NMEA_S_DTM:    ok = nmea_parse_dtm(fields, n, &u.d);  break;
    case NMEA_S_TXT:    ok = nmea_parse_txt(fields, n, &u.x);  break;
    case NMEA_S_PUBX00: ok = nmea_parse_pubx00(fields, n, &u.p0); break;
    case NMEA_S_PUBX03: ok = nmea_parse_pubx03(fields, n, &u.p3); break;
    case NMEA_S_PUBX04: ok = nmea_parse_pubx04(fields, n, &u.p4); break;
    default: break;
    }

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    gps_repo_t *r = &s_repo;
    r->last_frame_ms = now;
    r->epoch_work_frames++;
    r->epoch_last_frame_us = (uint64_t)esp_timer_get_time();
    if (!s_work_open && r->epoch_start_us > 0 && r->epoch_last_frame_us >= r->epoch_start_us) {
        uint32_t dt = (uint32_t)(r->epoch_last_frame_us - r->epoch_start_us);
        if (dt < 500000u) {
            r->epoch_burst_total_us = dt;
            r->epoch_frames_cnt     = r->epoch_work_frames;
            r->epoch_bytes_cnt      = r->bytes_rx - r->epoch_start_bytes;
        }
    }
    r->talker_mask |= talker_bit(talker);
    if (s < NMEA_S_COUNT) r->nmea_cnt[s]++;

    uint8_t ccls;
    uint8_t cid = nmea_cfg_id(s, &ccls);
    if (ccls) {
        int idx = gps_msg_index(ccls, cid);
        if (idx >= 0) {
            r->mstat[idx].rx_count++;
            r->mstat[idx].last_rx_ms = now;
            r->mstat[idx].last_len = np->len;
        }
    }

#define STORE(slot, src) do { (slot).d = (src); (slot).valid = true; (slot).ms = now; } while (0)
    if (ok) {
        switch (s) {
        case NMEA_S_GGA: STORE(r->gga, u.g); break;
        case NMEA_S_GLL: STORE(r->gll, u.l); break;
        case NMEA_S_GSA: STORE(r->gsa, u.a); break;
        case NMEA_S_RMC:
            STORE(r->rmc, u.m);
            if (u.m.magvar_empty && u.m.magdir_empty) r->rmc_magvar_empty++;
            else r->rmc_magvar_present++;
            if (r->nmea_only) gps_warmup_on_nmea_epoch(r, now);
            break;
        case NMEA_S_VTG:
            STORE(r->vtg, u.t);
            if (u.t.cog_m_empty) r->vtg_mag_empty++; else r->vtg_mag_present++;
            break;
        case NMEA_S_GRS: STORE(r->grs, u.rs); break;
        case NMEA_S_GST:
            STORE(r->gst, u.st);
            if (u.st.major_empty && u.st.minor_empty && u.st.orient_empty) r->gst_ellipse_empty++;
            else r->gst_ellipse_present++;
            break;
        case NMEA_S_ZDA: STORE(r->zda, u.z); break;
        case NMEA_S_GBS: STORE(r->gbs, u.b); break;
        case NMEA_S_DTM: STORE(r->dtm, u.d); break;
        case NMEA_S_TXT: handle_txt(r, &u.x); break;
        case NMEA_S_PUBX00: STORE(r->pubx00, u.p0); break;
        case NMEA_S_PUBX03: STORE(r->pubx03, u.p3); break;
        case NMEA_S_PUBX04:
            STORE(r->pubx04, u.p4);
            r->leap_from_space = !u.p4.leap_default;
            break;
        case NMEA_S_GSV: {
            const nmea_gsv_t *g = &u.v;
            if (g->msg_num == 1) {
                s_gsv_tmp_n = 0;
                s_gsv_expect_next = 1;
            }
            if (g->msg_num == s_gsv_expect_next) {
                for (int i = 0; i < g->nblk && s_gsv_tmp_n < GPS_MAX_GSV; i++) {
                    s_gsv_tmp[s_gsv_tmp_n].prn  = g->blk[i].prn;
                    s_gsv_tmp[s_gsv_tmp_n].elev = g->blk[i].elev;
                    s_gsv_tmp[s_gsv_tmp_n].azim = g->blk[i].azim;
                    s_gsv_tmp[s_gsv_tmp_n].cno  = g->blk[i].cno;
                    s_gsv_tmp_n++;
                }
                s_gsv_expect_next++;
                if (g->msg_num >= g->total_msgs) {
                    memcpy(r->gsv.s, s_gsv_tmp, sizeof(gps_gsv_sat_t) * (size_t)s_gsv_tmp_n);
                    r->gsv.n = s_gsv_tmp_n;
                    r->gsv.in_view = g->sats_in_view;
                    r->gsv.valid = true;
                    r->gsv.ms = now;
                    s_gsv_expect_next = 0;
                }
            } else {
                s_gsv_expect_next = 0;   /* secuencia rota: descartar ciclo */
            }
            break;
        }
        default: break;
        }
    }
#undef STORE
    xSemaphoreGive(s_mtx);
    xEventGroupSetBits(s_evt, EVT_ANY_NMEA);
}

/* ======================================================================= */
/*                         EVENTOS DE BAJO NIVEL                           */
/* ======================================================================= */
void gps_data_on_bytes(uint32_t n)
{
    uint32_t now = gps_now_ms();
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    if (s_repo.bytes_rx == 0) s_repo.first_byte_ms = now;
    s_repo.bytes_rx += n;
    s_repo.last_byte_ms = now;
    xSemaphoreGive(s_mtx);
    xEventGroupSetBits(s_evt, EVT_ANY_BYTE);
}

void gps_data_on_uart_error(int type)
{
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    switch (type) {
    case UART_FIFO_OVF:    s_repo.uart_fifo_ovf++;   break;
    case UART_BUFFER_FULL: s_repo.uart_buf_full++;   break;
    case UART_FRAME_ERR:   s_repo.uart_frame_err++;  s_repo.last_byte_ms = gps_now_ms(); break;
    case UART_PARITY_ERR:  s_repo.uart_parity_err++; break;
    case UART_BREAK:       s_repo.uart_break++;      break;
    default: break;
    }
    xSemaphoreGive(s_mtx);
    if (type == UART_FRAME_ERR || type == UART_BREAK) {
        xEventGroupSetBits(s_evt, EVT_FRAME_ERR);
    }
}
