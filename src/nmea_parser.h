/*
 * nmea_parser.h
 * ---------------------------------------------------------------------------
 * Agente 3 - Decodificador ASCII NMEA 0183 (v2.3/v2.1) y propietarias PUBX
 * del u-blox 6.  Ensamblador de lineas byte a byte, verificacion XOR tras '*',
 * tokenizador propio que CONSERVA campos vacios consecutivos (",,") y
 * decodificadores de cada sentencia.  Cero asignaciones dinamicas.
 * ---------------------------------------------------------------------------
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define NMEA_MAX_LINE     1024   /* PUBX,03 con muchos satelites es largo  */
#define NMEA_MAX_FIELDS   220
#define NMEA_MAX_SATS     64
#define NMEA_TXT_LEN      80

typedef enum {
    NMEA_FEED_NONE = 0,
    NMEA_FEED_SENTENCE,      /* sentencia completa y checksum valido      */
    NMEA_FEED_CKERR,         /* checksum invalido                          */
    NMEA_FEED_OVERFLOW       /* linea demasiado larga / caracter invalido  */
} nmea_feed_t;

typedef enum {
    NMEA_S_UNKNOWN = 0,
    NMEA_S_GGA, NMEA_S_GLL, NMEA_S_GSA, NMEA_S_GSV, NMEA_S_RMC, NMEA_S_VTG,
    NMEA_S_GRS, NMEA_S_GST, NMEA_S_ZDA, NMEA_S_GBS, NMEA_S_DTM, NMEA_S_TXT,
    NMEA_S_PUBX00, NMEA_S_PUBX03, NMEA_S_PUBX04, NMEA_S_PUBX_OTHER,
    NMEA_S_COUNT
} nmea_sentence_t;

typedef struct {
    uint8_t  state;
    uint16_t len;
    uint8_t  xor_calc;
    uint8_t  xor_rx;
    uint8_t  hexcnt;
    char     line[NMEA_MAX_LINE];      /* contenido entre '$' y '*'         */
    char     work[NMEA_MAX_LINE];      /* copia tokenizada                  */
    /* estadisticas */
    uint32_t ok;
    uint32_t ckerr;
    uint32_t overflow;
} nmea_parser_t;

/* ---------------------------- Tipos comunes ---------------------------- */
typedef struct {
    bool    valid;
    uint8_t hh, mm;
    float   ss;
} nmea_time_t;

typedef struct {
    bool    valid;
    uint8_t day, month;
    uint16_t year;
} nmea_date_t;

/* --------------------------- Sentencias -------------------------------- */
typedef struct {
    char        talker[3];
    nmea_time_t time;
    bool        pos_valid;
    double      lat, lon;           /* grados decimales con signo */
    int         quality;            /* 0 inv, 1 SPS, 2 DGPS/SBAS, 6 DR */
    int         numSV;
    bool        hdop_valid;  float hdop;
    bool        alt_valid;   float alt_msl;
    bool        sep_valid;   float geoid_sep;
    bool        dgps_age_valid; float dgps_age;
    bool        dgps_sta_valid; int dgps_station;
} nmea_gga_t;

typedef struct {
    nmea_time_t time;
    bool        pos_valid;
    double      lat, lon;
    char        status;             /* A / V */
    char        mode;               /* N A D E */
} nmea_gll_t;

typedef struct {
    char   opmode;                  /* M / A */
    int    fixmode;                 /* 1 no fix, 2 2D, 3 3D */
    int    nprn;
    int    prn[12];
    bool   pdop_valid; float pdop;
    bool   hdop_valid; float hdop;
    bool   vdop_valid; float vdop;
} nmea_gsa_t;

typedef struct {
    int    prn;
    int    elev;                    /* -1 = vacio */
    int    azim;                    /* -1 = vacio */
    int    cno;                     /* -1 = vacio (no rastreado) */
} nmea_gsv_sat_t;

typedef struct {                    /* una sentencia GSV individual */
    int            total_msgs;
    int            msg_num;
    int            sats_in_view;
    int            nblk;
    nmea_gsv_sat_t blk[4];
} nmea_gsv_t;

typedef struct {
    nmea_time_t time;
    char        status;             /* A / V */
    bool        pos_valid;
    double      lat, lon;
    bool        sog_valid;  float sog_kn;
    bool        cog_valid;  float cog;
    nmea_date_t date;
    bool        magvar_empty;       /* campo 10 vacio (sin magnetometro) */
    bool        magdir_empty;       /* campo 11 vacio                    */
    float       magvar;
    char        magdir;
    char        mode;
} nmea_rmc_t;

typedef struct {
    bool  cog_t_valid;  float cog_true;
    bool  cog_m_empty;  float cog_mag;
    bool  sog_kn_valid; float sog_kn;
    bool  sog_kmh_valid; float sog_kmh;
    char  mode;
} nmea_vtg_t;

typedef struct {
    nmea_time_t time;
    int    mode;
    int    nres;
    bool   res_valid[12];
    float  res[12];                 /* m */
} nmea_grs_t;

typedef struct {
    nmea_time_t time;
    bool  rms_valid;   float rangeRms;
    bool  major_empty; float stdMajor;
    bool  minor_empty; float stdMinor;
    bool  orient_empty; float orient;
    bool  lat_valid;   float stdLat;
    bool  lon_valid;   float stdLon;
    bool  alt_valid;   float stdAlt;
} nmea_gst_t;

typedef struct {
    nmea_time_t time;
    nmea_date_t date;
    bool  ltz_valid;
    int   ltz_h, ltz_m;
} nmea_zda_t;

typedef struct {
    nmea_time_t time;
    bool  errLat_valid; float errLat;
    bool  errLon_valid; float errLon;
    bool  errAlt_valid; float errAlt;
    bool  svid_valid;   int   svid;
    bool  prob_valid;   float prob;
    bool  bias_valid;   float bias;
    bool  stddev_valid; float stddev;
} nmea_gbs_t;

typedef struct {
    char  datum[6];
    char  subdatum[6];
    float lat_ofs_min;              /* con signo */
    float lon_ofs_min;
    float alt_ofs_m;
    char  ref_datum[6];
} nmea_dtm_t;

typedef struct {
    int  num_msg;
    int  msg_num;
    int  msg_type;                  /* 0 err 1 warn 2 notice 7 user */
    char text[NMEA_TXT_LEN];
} nmea_txt_t;

typedef struct {
    nmea_time_t time;
    bool   pos_valid;
    double lat, lon;
    float  altRef;
    char   navStat[3];
    float  hAcc, vAcc;
    float  sog_kmh, cog, vVel;
    bool   diffAge_valid; float diffAge;
    float  hdop, vdop, tdop;
    int    gu, ru, dr;
} nmea_pubx00_t;

typedef struct {
    int   sv;
    char  status;                   /* '-' no usado, 'U' usado, 'e' eph */
    int   azim;
    int   elev;
    int   cno;
    int   lck;                      /* s, 0..64 */
} nmea_pubx03_sat_t;

typedef struct {
    int               n;
    int               nblk;
    nmea_pubx03_sat_t sat[NMEA_MAX_SATS];
} nmea_pubx03_t;

typedef struct {
    nmea_time_t time;
    nmea_date_t date;
    double  utcTow;
    int     utcWk;
    int     leapSec;
    bool    leap_default;           /* sufijo 'D' = valor por defecto ROM */
    long    clkBias;                /* ns */
    float   clkDrift;               /* ns/s */
    int     tpGran;                 /* ns */
} nmea_pubx04_t;

/* ------------------------------ API ------------------------------------ */
void        nmea_parser_init(nmea_parser_t *p);
nmea_feed_t nmea_parser_feed(nmea_parser_t *p, char c);

/* Tokeniza la ultima sentencia valida. Devuelve numero de campos.
 * fields[0] = direccion (p.ej. "GPGGA" o "PUBX").                         */
int  nmea_tokenize(nmea_parser_t *p, char **fields, int max_fields);

/* Identifica la sentencia y devuelve el talker ID (2 chars) */
nmea_sentence_t nmea_identify(char **f, int n, char talker[3]);
const char     *nmea_sentence_name(nmea_sentence_t s);

bool nmea_parse_gga(char **f, int n, nmea_gga_t *o);
bool nmea_parse_gll(char **f, int n, nmea_gll_t *o);
bool nmea_parse_gsa(char **f, int n, nmea_gsa_t *o);
bool nmea_parse_gsv(char **f, int n, nmea_gsv_t *o);
bool nmea_parse_rmc(char **f, int n, nmea_rmc_t *o);
bool nmea_parse_vtg(char **f, int n, nmea_vtg_t *o);
bool nmea_parse_grs(char **f, int n, nmea_grs_t *o);
bool nmea_parse_gst(char **f, int n, nmea_gst_t *o);
bool nmea_parse_zda(char **f, int n, nmea_zda_t *o);
bool nmea_parse_gbs(char **f, int n, nmea_gbs_t *o);
bool nmea_parse_dtm(char **f, int n, nmea_dtm_t *o);
bool nmea_parse_txt(char **f, int n, nmea_txt_t *o);
bool nmea_parse_pubx00(char **f, int n, nmea_pubx00_t *o);
bool nmea_parse_pubx03(char **f, int n, nmea_pubx03_t *o);
bool nmea_parse_pubx04(char **f, int n, nmea_pubx04_t *o);
