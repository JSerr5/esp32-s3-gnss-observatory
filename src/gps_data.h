/*
 * gps_data.h
 * ---------------------------------------------------------------------------
 * Repositorio central de telemetria (protegido por mutex FreeRTOS), tabla de
 * descriptores de mensajes para la Matriz de Capacidades, sintetizador PVT
 * (NAV-PVT no existe en protocolo 13.03) y mecanismo de espera determinista
 * de ACK-ACK / ACK-NAK / respuestas de sondeo.
 * ---------------------------------------------------------------------------
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "ubx_defs.h"
#include "nmea_parser.h"

/* ------------------------------ Limites -------------------------------- */
#define GPS_MAX_SVINFO_CH   64
#define GPS_MAX_SVSI        64
#define GPS_MAX_SBAS_SV     24
#define GPS_MAX_DGPS_CH     16
#define GPS_MAX_GSV         64
#define GPS_MAX_IO_PORTS    6
#define GPS_MAX_VER_EXT     6
#define GPS_TXT_LOG         10
#define GPS_RESP_BUF        1024

/* ------------------------------ Eventos -------------------------------- */
#define EVT_ACK        (1u << 0)
#define EVT_NAK        (1u << 1)
#define EVT_RESP       (1u << 2)
#define EVT_ANY_UBX    (1u << 3)
#define EVT_ANY_NMEA   (1u << 4)
#define EVT_ANY_BYTE     (1u << 5)
#define EVT_FRAME_ERR    (1u << 6)
#define EVT_EPOCH_READY  (1u << 7)

/* ------------------------- Resultado de pruebas ------------------------ */
typedef enum {
    RES_UNTESTED = 0,
    RES_ACK,          /* ACK-ACK (0x05 0x01)                    */
    RES_NAK,          /* ACK-NAK (0x05 0x00)                    */
    RES_TIMEOUT,      /* silencio                               */
    RES_DATA,         /* respuesta con datos                    */
    RES_NA            /* no aplica                              */
} gps_res_t;

/* ------------------------- Descriptor de mensaje ----------------------- */
typedef enum {
    GRP_NAV = 0, GRP_RXM, GRP_MON, GRP_TIM, GRP_AID, GRP_CFG, GRP_NMEA,
    GRP_PUBX, GRP_INF, GRP_COUNT
} gps_group_t;

#define MF_PERIODIC  0x01   /* configurable con CFG-MSG                  */
#define MF_POLL      0x02   /* sondeable con payload de longitud 0       */
#define MF_NEG       0x04   /* prueba negativa: se espera NO soportado   */
#define MF_CFGREAD   0x08   /* configuracion CFG leida por sondeo        */
#define MF_PASSIVE   0x10   /* solo se observa (INF, TXT)                */
#define MF_HEAVY     0x20   /* voluminoso: no sondear en enlace lento    */

typedef struct {
    uint8_t     cls;
    uint8_t     id;
    const char *name;
    uint8_t     group;
    uint8_t     flags;
    uint8_t     rate_norm;   /* tasa CFG-MSG a 1 Hz, enlace rapido       */
    uint8_t     rate_lite;   /* tasa CFG-MSG en enlace lento (<=19200)   */
    uint8_t     rate_fast;   /* tasa CFG-MSG en navegacion a 5 Hz        */
    const char *note;
} gps_msg_desc_t;

typedef struct {
    gps_res_t cfgmsg;        /* resultado de CFG-MSG                     */
    gps_res_t poll;          /* resultado del sondeo directo             */
    bool      neg_data_seen; /* prueba negativa: datos tras habilitar    */
    uint8_t   rate_set;      /* tasa finalmente configurada              */
    uint32_t  rx_count;
    uint32_t  last_rx_ms;
    uint16_t  last_len;
} gps_msg_stat_t;

extern const gps_msg_desc_t g_msg_table[];
extern const int            g_msg_table_len;
int gps_msg_index(uint8_t cls, uint8_t id);

/* --------------------------- Estados globales -------------------------- */
typedef enum {
    LINK_SEARCHING = 0,  /* sin bytes / barrido de baudios         */
    LINK_ACTIVITY,       /* bytes presentes, baudios no resueltos  */
    LINK_DETECTED,       /* baudios detectados                     */
    LINK_BIDIR,          /* enlace bidireccional verificado        */
    LINK_RX_ONLY,        /* solo escucha (TX GPIO4 no responde)    */
    LINK_LOST
} gps_link_t;

typedef enum {
    SYS_BOOT = 0,
    SYS_WAIT_LINK,
    SYS_DISCOVERY,
    SYS_OPERATING
} gps_sys_state_t;

typedef enum {
    PH_NONE = 0,
    PH_A_BLIND,          /* busqueda ciega / estabilizacion HW     */
    PH_B_SIGNALS,        /* portadoras detectadas, sin efemerides  */
    PH_C_TIME,           /* tiempo GPS conocido                     */
    PH_D_CONVERGE,       /* primer fix, convergencia Kalman        */
    PH_E_TRUSTED         /* regimen estacionario confiable          */
} gps_phase_t;

typedef enum {
    AUTH_UNKNOWN = 0,
    AUTH_GENUINE,        /* u-blox 6 ROM 7.03 coherente            */
    AUTH_UBLOX_OTHER,    /* u-blox pero firmware/hw distinto        */
    AUTH_SUSPECT,        /* respuestas incoherentes                 */
    AUTH_CLONE,          /* no habla UBX / talkers no u-blox 6      */
    AUTH_UNVERIFIABLE    /* enlace solo escucha                     */
} gps_auth_t;

/* Criterios del Filtro Cientifico de Confianza */
#define CRIT_FIX_TYPE    (1u << 0)  /* SOL y STATUS gpsFix 2/3/4             */
#define CRIT_FIX_OK      (1u << 1)  /* gpsFixOk = 1 en SOL y STATUS          */
#define CRIT_GGA_Q       (1u << 2)  /* GGA calidad >= 1 (fresco)             */
#define CRIT_RMC_A       (1u << 3)  /* RMC estado 'A' (fresco)               */
#define CRIT_UTC_VALID   (1u << 4)  /* TIMEUTC valid & 0x07 == 0x07          */
#define CRIT_TACC        (1u << 5)  /* tAcc < 1000 ns                        */
#define CRIT_NUMSV       (1u << 6)  /* numSV >= 4 (3D) / >= 3 (2D)           */
#define CRIT_PDOP        (1u << 7)  /* PDOP < 10                             */
#define CRIT_HACC        (1u << 8)  /* hAcc < 50 m                           */
#define CRIT_ALL_MASK    0x01FFu
#define CRIT_COUNT       9

/* Contribuciones al PVT sintetico */
#define PVT_POSLLH   (1u << 0)
#define PVT_VELNED   (1u << 1)
#define PVT_SOL      (1u << 2)
#define PVT_DOP      (1u << 3)
#define PVT_STATUS   (1u << 4)
#define PVT_TIMEGPS  (1u << 5)
#define PVT_TIMEUTC  (1u << 6)
#define PVT_CLOCK    (1u << 7)
#define PVT_ALL      0xFFu

typedef struct {
    uint32_t iTOW;
    uint16_t mask;
    uint32_t closed_ms;
    /* posicion */
    int32_t  lat, lon, height, hMSL;
    uint32_t hAcc, vAcc;
    int32_t  ecefX, ecefY, ecefZ;
    uint32_t pAcc;
    /* velocidad */
    int32_t  velN, velE, velD;
    uint32_t speed, gSpeed;
    int32_t  heading;
    uint32_t sAcc, cAcc;
    int32_t  ecefVX, ecefVY, ecefVZ;
    /* solucion */
    uint8_t  gpsFix_sol, flags_sol, numSV;
    uint8_t  gpsFix_st, flags_st, fixStat, flags2;
    int16_t  week;
    int32_t  fTOW;
    uint16_t gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP;
    uint16_t pDOP_sol;
    uint32_t ttff, msss;
    /* tiempo */
    uint16_t year;
    uint8_t  month, day, hour, min, sec;
    int32_t  nano;
    uint8_t  utc_valid;
    uint32_t utc_tAcc;
    int8_t   leapS;
    uint8_t  tgps_valid;
    uint32_t tgps_tAcc;
    /* reloj */
    int32_t  clkB, clkD;
    uint32_t clk_tAcc, fAcc;
} gps_pvt_t;

/* Ranura generica con validez y marca de tiempo local */
#define GPS_SLOT(T) struct { bool valid; uint32_t ms; T d; }

typedef struct {
    int     prn;
    int     elev, azim, cno;
} gps_gsv_sat_t;

typedef struct {
    char     src[10];
    char     text[NMEA_TXT_LEN];
    uint32_t ms;
} gps_txt_entry_t;

typedef struct {
    /* ---------------- sistema / enlace ---------------- */
    gps_sys_state_t sys_state;
    gps_link_t      link;
    uint32_t        boot_ms;
    uint32_t        detected_baud;
    uint32_t        final_baud;
    uint32_t        current_baud;
    bool            baud_switch_ok;
    bool            bidir;
    bool            lite_profile;
    uint32_t        autobaud_cycles;
    uint32_t        link_events;        /* conexiones detectadas */
    uint32_t        bytes_rx;
    uint32_t        bytes_tx;
    uint32_t        first_byte_ms;
    uint32_t        last_byte_ms;
    uint32_t        last_frame_ms;
    uint32_t        uart_fifo_ovf, uart_buf_full, uart_frame_err;
    uint32_t        uart_parity_err, uart_break;
    uint32_t        ubx_ok, ubx_ckerr, ubx_oversize;
    uint32_t        nmea_ok, nmea_ckerr, nmea_overflow;
    uint32_t        nmea_cnt[NMEA_S_COUNT];
    uint16_t        talker_mask;        /* bit0 GP,1 GN,2 GL,3 GA,4 BD/GB,5 QZ,6 otro */
    uint32_t        discovery_ms;
    char            status_line[160];   /* estado del gestor */

    /* ---------------- identidad ---------------- */
    bool            ver_valid;
    char            swVersion[31];
    char            hwVersion[11];
    char            ext[GPS_MAX_VER_EXT][31];
    uint8_t         n_ext;
    char            protver[8];
    bool            protver_verified;   /* leido de MON-VER/GPTXT */
    char            txt_hw[48];
    char            txt_rom[64];
    bool            txt_ublox;          /* "u-blox ag" visto en GPTXT */
    gps_auth_t      auth;
    char            auth_reason[160];
    gps_res_t       nvm_bbr, nvm_eeprom, nvm_both;
    char            antsuperv[32];
    char            antstatus[16];
    gps_txt_entry_t txt[GPS_TXT_LOG];
    uint8_t         txt_head, txt_count;
    uint32_t        inf_count[5];

    /* ---------------- matriz de capacidades ---------------- */
    gps_msg_stat_t  mstat[96];

    /* ---------------- UBX NAV ---------------- */
    GPS_SLOT(ubx_nav_posecef_t)   posecef;
    GPS_SLOT(ubx_nav_posllh_t)    posllh;
    GPS_SLOT(ubx_nav_status_t)    status;
    GPS_SLOT(ubx_nav_dop_t)       dop;
    GPS_SLOT(ubx_nav_sol_t)       sol;
    GPS_SLOT(ubx_nav_velecef_t)   velecef;
    GPS_SLOT(ubx_nav_velned_t)    velned;
    GPS_SLOT(ubx_nav_timegps_t)   timegps;
    GPS_SLOT(ubx_nav_timeutc_t)   timeutc;
    GPS_SLOT(ubx_nav_clock_t)     clock;
    GPS_SLOT(ubx_nav_aopstatus_t) aopstatus;
    struct { bool valid; uint32_t ms; ubx_nav_svinfo_hdr_t h; uint8_t n;
             ubx_nav_svinfo_blk_t b[GPS_MAX_SVINFO_CH]; } svinfo;
    struct { bool valid; uint32_t ms; ubx_nav_dgps_hdr_t h; uint8_t n;
             ubx_nav_dgps_blk_t b[GPS_MAX_DGPS_CH]; } dgps;
    struct { bool valid; uint32_t ms; ubx_nav_sbas_hdr_t h; uint8_t n;
             ubx_nav_sbas_blk_t b[GPS_MAX_SBAS_SV]; } sbas;

    /* ---------------- UBX RXM ---------------- */
    struct { bool valid; uint32_t ms; ubx_rxm_svsi_hdr_t h; uint8_t n;
             ubx_rxm_svsi_blk_t b[GPS_MAX_SVSI]; } svsi;
    uint32_t  alm_week[33];      /* indice = PRN 1..32 (AID/RXM-ALM)   */
    uint32_t  eph_how[33];       /* indice = PRN 1..32 (AID/RXM-EPH)   */
    uint32_t  alm_ms, eph_ms;
    uint8_t   alm_resp, eph_resp;
    uint32_t  aop_mask;          /* AID-AOP con datos por PRN          */
    uint8_t   aop_resp;
    uint32_t  aop_ms;

    /* ---------------- UBX MON ---------------- */
    GPS_SLOT(ubx_mon_hw_t)     hw;
    GPS_SLOT(ubx_mon_hw2_t)    hw2;
    GPS_SLOT(ubx_mon_msgpp_t)  msgpp;
    GPS_SLOT(ubx_mon_rxbuf_t)  rxbuf;
    GPS_SLOT(ubx_mon_txbuf_t)  txbuf;
    GPS_SLOT(ubx_mon_rxr_t)    rxr;
    struct { bool valid; uint32_t ms; uint8_t n;
             ubx_mon_io_port_t p[GPS_MAX_IO_PORTS]; } io;
    uint8_t   txbuf_peak_seen;   /* maximo tUsage observado (%)        */

    /* ---------------- UBX TIM / AID ---------------- */
    GPS_SLOT(ubx_tim_tp_t)     tp;
    GPS_SLOT(ubx_tim_tm2_t)    tm2;
    GPS_SLOT(ubx_tim_vrfy_t)   vrfy;
    GPS_SLOT(ubx_aid_hui_t)    hui;
    GPS_SLOT(ubx_aid_ini_t)    ini;

    /* ---------------- UBX CFG (lecturas) ---------------- */
    GPS_SLOT(ubx_cfg_prt_t)    cfg_prt;
    GPS_SLOT(ubx_cfg_rate_t)   cfg_rate;
    GPS_SLOT(ubx_cfg_nav5_t)   cfg_nav5;
    GPS_SLOT(ubx_cfg_navx5_t)  cfg_navx5;
    GPS_SLOT(ubx_cfg_sbas_t)   cfg_sbas;
    GPS_SLOT(ubx_cfg_nmea_t)   cfg_nmea;
    GPS_SLOT(ubx_cfg_tp5_t)    cfg_tp5;
    GPS_SLOT(ubx_cfg_itfm_t)   cfg_itfm;
    GPS_SLOT(ubx_cfg_ant_t)    cfg_ant;
    GPS_SLOT(ubx_cfg_rxm_t)    cfg_rxm;
    GPS_SLOT(ubx_cfg_pm2_t)    cfg_pm2;
    GPS_SLOT(ubx_cfg_dat_t)    cfg_dat;
    bool      itfm_enabled_by_fw;
    bool      nmea_ext_by_fw;
    bool      sbas_disabled_by_fw;
    char      sbas_policy[128];
    bool      fast_nav_active;

    /* ---------------- NMEA ---------------- */
    GPS_SLOT(nmea_gga_t)    gga;
    GPS_SLOT(nmea_gll_t)    gll;
    GPS_SLOT(nmea_gsa_t)    gsa;
    GPS_SLOT(nmea_rmc_t)    rmc;
    GPS_SLOT(nmea_vtg_t)    vtg;
    GPS_SLOT(nmea_grs_t)    grs;
    GPS_SLOT(nmea_gst_t)    gst;
    GPS_SLOT(nmea_zda_t)    zda;
    GPS_SLOT(nmea_gbs_t)    gbs;
    GPS_SLOT(nmea_dtm_t)    dtm;
    GPS_SLOT(nmea_pubx00_t) pubx00;
    GPS_SLOT(nmea_pubx03_t) pubx03;
    GPS_SLOT(nmea_pubx04_t) pubx04;
    struct { bool valid; uint32_t ms; int in_view; int n;
             gps_gsv_sat_t s[GPS_MAX_GSV]; } gsv;
    /* transparencia de campos vacios por hardware */
    uint32_t  rmc_magvar_empty, rmc_magvar_present;
    uint32_t  vtg_mag_empty, vtg_mag_present;
    uint32_t  gst_ellipse_empty, gst_ellipse_present;

    /* ---------------- PVT sintetico y cronometria de trama ---------------- */
    gps_pvt_t pvt;               /* ultima epoca cerrada              */
    bool      pvt_valid;
    uint32_t  pvt_epochs;
    uint64_t  epoch_start_us;      /* t0 (us) del primer frame del iTOW actual */
    uint64_t  epoch_last_frame_us; /* t1 (us) del ultimo frame recibido        */
    uint32_t  epoch_ubx_build_us;  /* duracion (us) en armar PVT UBX completo  */
    uint32_t  epoch_burst_total_us;/* duracion (us) de toda la rafaga UBX+NMEA */
    uint32_t  epoch_frames_cnt;    /* tramas UBX+NMEA de la epoca cerrada      */
    uint32_t  epoch_bytes_cnt;     /* bytes UART de la epoca cerrada           */
    uint32_t  epoch_start_bytes;   /* bytes_rx al abrir el iTOW actual         */
    uint32_t  epoch_work_frames;   /* tramas acumuladas en el iTOW actual      */

    /* ---------------- calentamiento / confianza ---------------- */
    gps_phase_t phase;
    uint32_t    phase_since_ms;
    uint32_t    warm_start_ms;   /* ESP ms del inicio del calentamiento */
    bool        ttff_frozen;
    uint32_t    ttff_ms;         /* NAV-STATUS ttff congelado          */
    uint32_t    ttff_local_ms;   /* segun reloj ESP32-S3               */
    uint16_t    crit;            /* criterios cumplidos en la ultima epoca */
    uint8_t     good_epochs;
    uint8_t     bad_epochs;
    uint32_t    certified_ms;
    uint32_t    cert_count;
    bool        nmea_only;       /* enlace solo escucha: filtro degradado */
    bool        indoor_timeout;
    /* convergencia */
    uint32_t    hacc_first, hacc_min, vacc_first, vacc_min;
    uint32_t    sacc_first, sacc_min;
    uint16_t    pdop_first, pdop_min, hdop_min, vdop_min;
    /* contadores de canales */
    uint8_t     ch_total, ch_idle, ch_search, ch_acq, ch_detect;
    uint8_t     ch_code, ch_carrier, sv_cno_pos, sv_strong, sv_used;
    uint8_t     sv_eph, sv_alm, sv_healthy, sv_vis, sv_sbas;
    bool        time_known;
    bool        leap_from_space;
    bool        klob_valid;
    /* filtro fisico de rumbo */
    bool        heading_have;
    int32_t     heading_last_valid;  /* 1e-5 deg */
    bool        heading_static;
} gps_repo_t;

/* ------------------------------ API ------------------------------------ */
void        gps_data_init(void);
gps_repo_t *gps_data_lock(void);           /* toma el mutex y devuelve el repo */
void        gps_data_unlock(void);
void        gps_data_snapshot(gps_repo_t *dst);
EventGroupHandle_t gps_data_events(void);
uint32_t    gps_now_ms(void);

/* llamados por la tarea RX */
void gps_data_on_ubx(uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len);
void gps_data_on_nmea(nmea_parser_t *np);
void gps_data_on_bytes(uint32_t n);
void gps_data_on_uart_error(int type);

/* mecanismo de espera determinista */
void     gps_pending_set(uint8_t cls, uint8_t id, uint8_t rcls, uint8_t rid, bool want_resp);
void     gps_pending_clear(void);
uint16_t gps_pending_resp(uint8_t *buf, uint16_t cap);

/* utilidades */
void        gps_data_log_txt(gps_repo_t *r, const char *src, const char *text);
const char *gps_res_str(gps_res_t r);
const char *gps_group_name(uint8_t g);
const char *gps_phase_name(gps_phase_t p);
const char *gps_fix_name(uint8_t fix);
