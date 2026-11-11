/*
 * ubx_defs.h
 * ---------------------------------------------------------------------------
 * Agente 3 - Definiciones binarias UBX para u-blox 6, protocolo 13.03
 * (GPS.G6-SW-10018-F).  Todas las estructuras son empaquetadas (pack 1) y
 * Little-Endian, nativas de la ESP32-S3.  Cada tamano se verifica en tiempo
 * de compilacion con _Static_assert.
 * ---------------------------------------------------------------------------
 */
#pragma once
#include <stdint.h>

/* ------------------------------ Sincronia ------------------------------ */
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

/* ------------------------------ Clases --------------------------------- */
#define UBX_CLS_NAV  0x01
#define UBX_CLS_RXM  0x02
#define UBX_CLS_INF  0x04
#define UBX_CLS_ACK  0x05
#define UBX_CLS_CFG  0x06
#define UBX_CLS_MON  0x0A
#define UBX_CLS_AID  0x0B
#define UBX_CLS_TIM  0x0D
#define UBX_CLS_LOG  0x21
#define NMEA_CLS_STD 0xF0
#define NMEA_CLS_PUBX 0xF1

/* ------------------------------ NAV ------------------------------------ */
#define UBX_NAV_POSECEF    0x01
#define UBX_NAV_POSLLH     0x02
#define UBX_NAV_STATUS     0x03
#define UBX_NAV_DOP        0x04
#define UBX_NAV_SOL        0x06
#define UBX_NAV_PVT        0x07   /* NO existe en prot 13.03 (prueba -) */
#define UBX_NAV_VELECEF    0x11
#define UBX_NAV_VELNED     0x12
#define UBX_NAV_TIMEGPS    0x20
#define UBX_NAV_TIMEUTC    0x21
#define UBX_NAV_CLOCK      0x22
#define UBX_NAV_SVINFO     0x30
#define UBX_NAV_DGPS       0x31
#define UBX_NAV_SBAS       0x32
#define UBX_NAV_EKFSTATUS  0x40   /* solo LEA-6R (prueba de soporte) */
#define UBX_NAV_AOPSTATUS  0x60
#define UBX_NAV_ODO        0x09   /* no existe en u-blox 6 */
#define UBX_NAV_GEOFENCE   0x39   /* no existe en u-blox 6 */

/* ------------------------------ RXM ------------------------------------ */
#define UBX_RXM_RAW   0x10   /* bloqueado por licencia en NEO-6M */
#define UBX_RXM_SFRB  0x11   /* bloqueado por licencia en NEO-6M */
#define UBX_RXM_SVSI  0x20
#define UBX_RXM_ALM   0x30
#define UBX_RXM_EPH   0x31
#define UBX_RXM_PMREQ 0x41

/* ------------------------------ INF ------------------------------------ */
#define UBX_INF_ERROR    0x00
#define UBX_INF_WARNING  0x01
#define UBX_INF_NOTICE   0x02
#define UBX_INF_TEST     0x03
#define UBX_INF_DEBUG    0x04

/* ------------------------------ ACK ------------------------------------ */
#define UBX_ACK_NAK 0x00
#define UBX_ACK_ACK 0x01

/* ------------------------------ CFG ------------------------------------ */
#define UBX_CFG_PRT    0x00
#define UBX_CFG_MSG    0x01
#define UBX_CFG_INF    0x02
#define UBX_CFG_RST    0x04
#define UBX_CFG_DAT    0x06
#define UBX_CFG_RATE   0x08
#define UBX_CFG_CFG    0x09
#define UBX_CFG_RXM    0x11
#define UBX_CFG_ANT    0x13
#define UBX_CFG_SBAS   0x16
#define UBX_CFG_NMEA   0x17
#define UBX_CFG_NAVX5  0x23
#define UBX_CFG_NAV5   0x24
#define UBX_CFG_TP5    0x31
#define UBX_CFG_ITFM   0x39
#define UBX_CFG_PM2    0x3B
#define UBX_CFG_GNSS   0x3E   /* NO existe en ROM 7.03 */

/* ------------------------------ MON ------------------------------------ */
#define UBX_MON_IO     0x02
#define UBX_MON_VER    0x04
#define UBX_MON_MSGPP  0x06
#define UBX_MON_RXBUF  0x07
#define UBX_MON_TXBUF  0x08
#define UBX_MON_HW     0x09
#define UBX_MON_HW2    0x0B
#define UBX_MON_RXR    0x21

/* ------------------------------ AID ------------------------------------ */
#define UBX_AID_INI    0x01
#define UBX_AID_HUI    0x02
#define UBX_AID_ALM    0x30
#define UBX_AID_EPH    0x31
#define UBX_AID_AOP    0x33

/* ------------------------------ TIM ------------------------------------ */
#define UBX_TIM_TP     0x01
#define UBX_TIM_TM2    0x03
#define UBX_TIM_SVIN   0x04   /* exclusivo NEO-6T */
#define UBX_TIM_VRFY   0x06

/* ------------------------------ NMEA (CFG-MSG) ------------------------- */
#define NMEA_ID_GGA 0x00
#define NMEA_ID_GLL 0x01
#define NMEA_ID_GSA 0x02
#define NMEA_ID_GSV 0x03
#define NMEA_ID_RMC 0x04
#define NMEA_ID_VTG 0x05
#define NMEA_ID_GRS 0x06
#define NMEA_ID_GST 0x07
#define NMEA_ID_ZDA 0x08
#define NMEA_ID_GBS 0x09
#define NMEA_ID_DTM 0x0A
#define PUBX_ID_00  0x00
#define PUBX_ID_03  0x03
#define PUBX_ID_04  0x04

/* ======================================================================= */
/*                     ESTRUCTURAS EMPAQUETADAS                            */
/* ======================================================================= */
#pragma pack(push, 1)

/* ---------------------------- NAV -------------------------------------- */
typedef struct {            /* UBX-NAV-POSECEF 0x01 0x01 (20) */
    uint32_t iTOW;          /* ms */
    int32_t  ecefX;         /* cm */
    int32_t  ecefY;         /* cm */
    int32_t  ecefZ;         /* cm */
    uint32_t pAcc;          /* cm */
} ubx_nav_posecef_t;

typedef struct {            /* UBX-NAV-POSLLH 0x01 0x02 (28) */
    uint32_t iTOW;          /* ms */
    int32_t  lon;           /* 1e-7 deg */
    int32_t  lat;           /* 1e-7 deg */
    int32_t  height;        /* mm, elipsoide */
    int32_t  hMSL;          /* mm, nivel medio del mar */
    uint32_t hAcc;          /* mm */
    uint32_t vAcc;          /* mm */
} ubx_nav_posllh_t;

typedef struct {            /* UBX-NAV-STATUS 0x01 0x03 (16) */
    uint32_t iTOW;
    uint8_t  gpsFix;        /* 0..5 */
    uint8_t  flags;         /* b0 gpsFixOk, b1 diffSoln, b2 wknSet, b3 towSet */
    uint8_t  fixStat;       /* b0 dgpsIStat, b6..7 mapMatching */
    uint8_t  flags2;        /* b0..1 psmState */
    uint32_t ttff;          /* ms */
    uint32_t msss;          /* ms */
} ubx_nav_status_t;

typedef struct {            /* UBX-NAV-DOP 0x01 0x04 (18) */
    uint32_t iTOW;
    uint16_t gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP;  /* x0.01 */
} ubx_nav_dop_t;

typedef struct {            /* UBX-NAV-SOL 0x01 0x06 (52) */
    uint32_t iTOW;
    int32_t  fTOW;          /* ns */
    int16_t  week;
    uint8_t  gpsFix;
    uint8_t  flags;         /* b0 GPSfixOK, b1 DiffSoln, b2 WKNSET, b3 TOWSET */
    int32_t  ecefX, ecefY, ecefZ;     /* cm */
    uint32_t pAcc;                    /* cm */
    int32_t  ecefVX, ecefVY, ecefVZ;  /* cm/s */
    uint32_t sAcc;                    /* cm/s */
    uint16_t pDOP;                    /* x0.01 */
    uint8_t  reserved1;
    uint8_t  numSV;
    uint32_t reserved2;
} ubx_nav_sol_t;

typedef struct {            /* UBX-NAV-VELECEF 0x01 0x11 (20) */
    uint32_t iTOW;
    int32_t  ecefVX, ecefVY, ecefVZ;  /* cm/s */
    uint32_t sAcc;                    /* cm/s */
} ubx_nav_velecef_t;

typedef struct {            /* UBX-NAV-VELNED 0x01 0x12 (36) */
    uint32_t iTOW;
    int32_t  velN, velE, velD;        /* cm/s */
    uint32_t speed;                   /* cm/s 3D */
    uint32_t gSpeed;                  /* cm/s 2D */
    int32_t  heading;                 /* 1e-5 deg */
    uint32_t sAcc;                    /* cm/s */
    uint32_t cAcc;                    /* 1e-5 deg */
} ubx_nav_velned_t;

typedef struct {            /* UBX-NAV-TIMEGPS 0x01 0x20 (16) */
    uint32_t iTOW;
    int32_t  fTOW;          /* ns */
    int16_t  week;
    int8_t   leapS;         /* s */
    uint8_t  valid;         /* b0 towValid, b1 weekValid, b2 leapSValid */
    uint32_t tAcc;          /* ns */
} ubx_nav_timegps_t;

typedef struct {            /* UBX-NAV-TIMEUTC 0x01 0x21 (20) */
    uint32_t iTOW;
    uint32_t tAcc;          /* ns */
    int32_t  nano;          /* ns */
    uint16_t year;
    uint8_t  month, day, hour, min, sec;
    uint8_t  valid;         /* b0 validTOW, b1 validWKN, b2 validUTC, b4..7 utcStd */
} ubx_nav_timeutc_t;

typedef struct {            /* UBX-NAV-CLOCK 0x01 0x22 (20) */
    uint32_t iTOW;
    int32_t  clkB;          /* ns */
    int32_t  clkD;          /* ns/s */
    uint32_t tAcc;          /* ns */
    uint32_t fAcc;          /* ps/s */
} ubx_nav_clock_t;

typedef struct {            /* UBX-NAV-SVINFO cabecera (8) */
    uint32_t iTOW;
    uint8_t  numCh;
    uint8_t  globalFlags;   /* b0..2 chipGen (2 = u-blox 6) */
    uint16_t reserved2;
} ubx_nav_svinfo_hdr_t;

typedef struct {            /* UBX-NAV-SVINFO bloque (12) */
    uint8_t  chn;
    uint8_t  svid;
    uint8_t  flags;         /* b0 svUsed b1 diffCorr b2 orbitAvail b3 orbitEph
                               b4 unhealthy b5 orbitAlm b6 orbitAop b7 smoothed */
    uint8_t  quality;       /* b0..3 qualityInd 0..7 */
    uint8_t  cno;           /* dB-Hz */
    int8_t   elev;          /* deg */
    int16_t  azim;          /* deg */
    int32_t  prRes;         /* cm */
} ubx_nav_svinfo_blk_t;

typedef struct {            /* UBX-NAV-DGPS cabecera (16) */
    uint32_t iTOW;
    int32_t  age;           /* ms */
    int16_t  baseId;
    int16_t  baseHealth;
    uint8_t  numCh;
    uint8_t  status;        /* 0 none, 1 PR+PRR */
    uint16_t reserved1;
} ubx_nav_dgps_hdr_t;

typedef struct {            /* UBX-NAV-DGPS bloque (12) */
    uint8_t  svid;
    uint8_t  flags;         /* b0..3 canal, b4 dgps usado */
    uint16_t ageC;          /* ms */
    float    prc;           /* m */
    float    prrc;          /* m/s */
} ubx_nav_dgps_blk_t;

typedef struct {            /* UBX-NAV-SBAS cabecera (12) */
    uint32_t iTOW;
    uint8_t  geo;
    uint8_t  mode;          /* 0 dis, 1 en integ, 3 en test */
    int8_t   sys;           /* -1 unk, 0 WAAS, 1 EGNOS, 2 MSAS, 3 GAGAN, 16 GPS */
    uint8_t  service;       /* b0 ranging b1 corr b2 integ b3 test */
    uint8_t  cnt;
    uint8_t  reserved0[3];
} ubx_nav_sbas_hdr_t;

typedef struct {            /* UBX-NAV-SBAS bloque (12) */
    uint8_t  svid;
    uint8_t  flags;
    uint8_t  udre;
    uint8_t  svSys;
    uint8_t  svService;
    uint8_t  reserved1;
    int16_t  prc;           /* cm */
    uint16_t reserved2;
    int16_t  ic;            /* cm */
} ubx_nav_sbas_blk_t;

typedef struct {            /* UBX-NAV-AOPSTATUS 0x01 0x60 (20) */
    uint32_t iTOW;
    uint8_t  aopCfg;        /* b0 useAOP */
    uint8_t  status;        /* 0 idle */
    uint8_t  reserved0;
    uint8_t  reserved1;
    uint32_t availGPS;      /* mascara PRN1..32 */
    uint32_t reserved2;
    uint32_t reserved3;
} ubx_nav_aopstatus_t;

/* ---------------------------- RXM -------------------------------------- */
typedef struct {            /* UBX-RXM-SVSI cabecera (8) */
    int32_t  iTOW;
    int16_t  week;
    uint8_t  numVis;
    uint8_t  numSV;
} ubx_rxm_svsi_hdr_t;

typedef struct {            /* UBX-RXM-SVSI bloque (6) */
    uint8_t  svid;
    uint8_t  svFlag;        /* b0..3 ura, b4 healthy, b5 ephVal, b6 almVal, b7 notAvail */
    int16_t  azim;
    int8_t   elev;
    uint8_t  age;           /* b0..3 almAge, b4..7 ephAge */
} ubx_rxm_svsi_blk_t;

typedef struct {            /* UBX-RXM-ALM / AID-ALM cabecera (8) */
    uint32_t svid;
    uint32_t week;          /* 0 = sin almanaque */
} ubx_alm_hdr_t;

typedef struct {            /* UBX-RXM-EPH / AID-EPH cabecera (8) */
    uint32_t svid;
    uint32_t how;           /* 0 = sin efemeride */
} ubx_eph_hdr_t;

/* ---------------------------- MON -------------------------------------- */
typedef struct {            /* UBX-MON-HW 0x0A 0x09 (68) */
    uint32_t pinSel;
    uint32_t pinBank;
    uint32_t pinDir;
    uint32_t pinVal;
    uint16_t noisePerMS;
    uint16_t agcCnt;        /* 0..8191 */
    uint8_t  aStatus;       /* 0 INIT 1 DONTKNOW 2 OK 3 SHORT 4 OPEN */
    uint8_t  aPower;        /* 0 OFF 1 ON 2 DONTKNOW */
    uint8_t  flags;         /* b0 rtcCalib b1 safeBoot b2..3 jammingState */
    uint8_t  reserved1;
    uint32_t usedMask;
    uint8_t  VP[25];
    uint8_t  jamInd;        /* 0..255 */
    uint16_t reserved3;
    uint32_t pinIrq;
    uint32_t pullH;
    uint32_t pullL;
} ubx_mon_hw_t;

typedef struct {            /* UBX-MON-HW2 0x0A 0x0B (28) */
    int8_t   ofsI;
    uint8_t  magI;
    int8_t   ofsQ;
    uint8_t  magQ;
    uint8_t  cfgSource;     /* 114 ROM, 111 OTP, 112 pins, 102 Flash/BBR/EEPROM */
    uint8_t  reserved0[3];
    uint32_t lowLevCfg;
    uint32_t reserved1[2];
    uint32_t postStatus;
    uint32_t reserved2;
} ubx_mon_hw2_t;

typedef struct {            /* UBX-MON-IO bloque por puerto (20) */
    uint32_t rxBytes;
    uint32_t txBytes;
    uint16_t parityErrs;
    uint16_t framingErrs;
    uint16_t overrunErrs;
    uint16_t breakCond;
    uint8_t  rxBusy;
    uint8_t  txBusy;
    uint16_t reserved1;
} ubx_mon_io_port_t;

typedef struct {            /* UBX-MON-MSGPP 0x0A 0x06 (120) */
    uint16_t msg[6][8];     /* [puerto][protocolo] */
    uint32_t skipped[6];
} ubx_mon_msgpp_t;

typedef struct {            /* UBX-MON-RXBUF 0x0A 0x07 (24) */
    uint16_t pending[6];
    uint8_t  usage[6];
    uint8_t  peakUsage[6];
} ubx_mon_rxbuf_t;

typedef struct {            /* UBX-MON-TXBUF 0x0A 0x08 (28) */
    uint16_t pending[6];
    uint8_t  usage[6];
    uint8_t  peakUsage[6];
    uint8_t  tUsage;
    uint8_t  tPeakUsage;
    uint8_t  errors;        /* b0..5 limite por puerto, b6 mem, b7 alloc */
    uint8_t  reserved1;
} ubx_mon_txbuf_t;

typedef struct {            /* UBX-MON-RXR 0x0A 0x21 (1) */
    uint8_t  flags;         /* b0 awake */
} ubx_mon_rxr_t;

/* ---------------------------- TIM -------------------------------------- */
typedef struct {            /* UBX-TIM-TP 0x0D 0x01 (16) */
    uint32_t towMS;
    uint32_t towSubMS;      /* 2^-32 ms */
    int32_t  qErr;          /* ps */
    uint16_t week;
    uint8_t  flags;         /* b0 timeBase (0 GPS, 1 UTC), b1 utc disp. */
    uint8_t  reserved1;
} ubx_tim_tp_t;

typedef struct {            /* UBX-TIM-TM2 0x0D 0x03 (28) */
    uint8_t  ch;
    uint8_t  flags;
    uint16_t count;
    uint16_t wnR;
    uint16_t wnF;
    uint32_t towMsR;
    uint32_t towSubMsR;
    uint32_t towMsF;
    uint32_t towSubMsF;
    uint32_t accEst;
} ubx_tim_tm2_t;

typedef struct {            /* UBX-TIM-VRFY 0x0D 0x06 (20) */
    int32_t  itow;
    int32_t  frac;
    int32_t  deltaMs;
    int32_t  deltaNs;
    uint16_t wno;
    uint8_t  flags;         /* b0..2 src */
    uint8_t  reserved1;
} ubx_tim_vrfy_t;

/* ---------------------------- AID -------------------------------------- */
typedef struct {            /* UBX-AID-HUI 0x0B 0x02 (72) */
    uint32_t health;
    double   utcA0;
    double   utcA1;
    int32_t  utcTOW;
    int16_t  utcWNT;
    int16_t  utcLS;
    int16_t  utcWNF;
    int16_t  utcDN;
    int16_t  utcLSF;
    int16_t  utcSpare;
    float    klobA0, klobA1, klobA2, klobA3;
    float    klobB0, klobB1, klobB2, klobB3;
    uint32_t flags;         /* b0 healthValid b1 utcValid b2 klobValid */
} ubx_aid_hui_t;

typedef struct {            /* UBX-AID-INI 0x0B 0x01 (48) */
    int32_t  ecefXOrLat;
    int32_t  ecefYOrLon;
    int32_t  ecefZOrAlt;
    uint32_t posAcc;
    uint16_t tmCfg;
    uint16_t wn;
    uint32_t tow;
    int32_t  towNs;
    uint32_t tAccMs;
    uint32_t tAccNs;
    int32_t  clkDOrFreq;
    uint32_t clkDAccOrFreqAcc;
    uint32_t flags;
} ubx_aid_ini_t;

/* ---------------------------- CFG -------------------------------------- */
typedef struct {            /* UBX-CFG-PRT (UART) (20) */
    uint8_t  portID;
    uint8_t  reserved0;
    uint16_t txReady;
    uint32_t mode;
    uint32_t baudRate;
    uint16_t inProtoMask;
    uint16_t outProtoMask;
    uint16_t flags;
    uint16_t reserved5;
} ubx_cfg_prt_t;

typedef struct {            /* UBX-CFG-MSG (8 bytes: rate por puerto) */
    uint8_t  msgClass;
    uint8_t  msgID;
    uint8_t  rate[6];
} ubx_cfg_msg8_t;

typedef struct {            /* UBX-CFG-RATE (6) */
    uint16_t measRate;      /* ms */
    uint16_t navRate;       /* ciclos */
    uint16_t timeRef;       /* 0 UTC, 1 GPS */
} ubx_cfg_rate_t;

typedef struct {            /* UBX-CFG-CFG (13) */
    uint32_t clearMask;
    uint32_t saveMask;
    uint32_t loadMask;
    uint8_t  deviceMask;    /* b0 BBR, b1 Flash, b2 EEPROM, b4 SPI Flash */
} ubx_cfg_cfg_t;

typedef struct {            /* UBX-CFG-NAV5 (36) */
    uint16_t mask;
    uint8_t  dynModel;
    uint8_t  fixMode;
    int32_t  fixedAlt;      /* cm */
    uint32_t fixedAltVar;   /* 0.0001 m^2 */
    int8_t   minElev;
    uint8_t  drLimit;
    uint16_t pDop;          /* x0.1 */
    uint16_t tDop;          /* x0.1 */
    uint16_t pAcc;          /* m */
    uint16_t tAcc;          /* m */
    uint8_t  staticHoldThresh; /* cm/s */
    uint8_t  dgpsTimeOut;   /* s */
    uint32_t reserved2;
    uint32_t reserved3;
    uint32_t reserved4;
} ubx_cfg_nav5_t;

typedef struct {            /* UBX-CFG-NAVX5 (40) prot 13 */
    uint16_t version;
    uint16_t mask1;
    uint32_t reserved0;
    uint8_t  reserved1;
    uint8_t  reserved2;
    uint8_t  minSVs;
    uint8_t  maxSVs;
    uint8_t  minCNO;
    uint8_t  reserved5;
    uint8_t  iniFix3D;
    uint8_t  reserved6;
    uint8_t  reserved7;
    uint8_t  reserved8;
    uint16_t wknRollover;
    uint32_t reserved9;
    uint8_t  reserved10;
    uint8_t  reserved11;
    uint8_t  usePPP;
    uint8_t  useAOP;
    uint8_t  reserved12;
    uint8_t  reserved13;
    uint16_t aopOrbMaxErr;
    uint32_t reserved3;
    uint32_t reserved4;
} ubx_cfg_navx5_t;

typedef struct {            /* UBX-CFG-SBAS (8) */
    uint8_t  mode;          /* b0 enabled b1 test */
    uint8_t  usage;         /* b0 range b1 diffCorr b2 integrity */
    uint8_t  maxSBAS;
    uint8_t  scanmode2;
    uint32_t scanmode1;
} ubx_cfg_sbas_t;

typedef struct {            /* UBX-CFG-NMEA (4) prot 13 */
    uint8_t  filter;
    uint8_t  version;       /* 0x23 = 2.3, 0x21 = 2.1 */
    uint8_t  numSV;         /* 0 = ilimitado */
    uint8_t  flags;         /* b0 compat, b1 consider */
} ubx_cfg_nmea_t;

typedef struct {            /* UBX-CFG-TP5 (32) */
    uint8_t  tpIdx;
    uint8_t  reserved0;
    uint16_t reserved1;
    int16_t  antCableDelay; /* ns */
    int16_t  rfGroupDelay;  /* ns */
    uint32_t freqPeriod;
    uint32_t freqPeriodLock;
    uint32_t pulseLenRatio;
    uint32_t pulseLenRatioLock;
    int32_t  userConfigDelay;
    uint32_t flags;
} ubx_cfg_tp5_t;

typedef struct {            /* UBX-CFG-ITFM (8) */
    uint32_t config;        /* b0..3 bbThr, b4..8 cwThr, b9..30 res, b31 enable */
    uint32_t config2;       /* b0..11 res(0x31E), b12..13 antSetting */
} ubx_cfg_itfm_t;

typedef struct {            /* UBX-CFG-ANT (4) */
    uint16_t flags;         /* b0 svcs b1 scd b2 ocd b3 pdwnOnSCD b4 recovery */
    uint16_t pins;
} ubx_cfg_ant_t;

typedef struct {            /* UBX-CFG-RXM (2) */
    uint8_t  reserved1;
    uint8_t  lpMode;        /* 0 max perf, 1 power save, 4 eco */
} ubx_cfg_rxm_t;

typedef struct {            /* UBX-CFG-PM2 (44) prot 13 */
    uint8_t  version;
    uint8_t  reserved1;
    uint8_t  reserved2;
    uint8_t  reserved3;
    uint32_t flags;
    uint32_t updatePeriod;
    uint32_t searchPeriod;
    uint32_t gridOffset;
    uint16_t onTime;
    uint16_t minAcqTime;
    uint16_t reserved4;
    uint16_t reserved5;
    uint32_t reserved6;
    uint32_t reserved7;
    uint8_t  reserved8;
    uint8_t  reserved9;
    uint16_t reserved10;
    uint32_t reserved11;
} ubx_cfg_pm2_t;

typedef struct {            /* UBX-CFG-DAT respuesta al sondeo (52) */
    uint16_t datumNum;
    char     datumName[6];
    double   majA;
    double   flat;
    float    dX, dY, dZ;
    float    rotX, rotY, rotZ;
    float    scale;
} ubx_cfg_dat_t;

#pragma pack(pop)

/* --------------------- Verificacion de tamanos ------------------------- */
_Static_assert(sizeof(ubx_nav_posecef_t)   == 20, "NAV-POSECEF");
_Static_assert(sizeof(ubx_nav_posllh_t)    == 28, "NAV-POSLLH");
_Static_assert(sizeof(ubx_nav_status_t)    == 16, "NAV-STATUS");
_Static_assert(sizeof(ubx_nav_dop_t)       == 18, "NAV-DOP");
_Static_assert(sizeof(ubx_nav_sol_t)       == 52, "NAV-SOL");
_Static_assert(sizeof(ubx_nav_velecef_t)   == 20, "NAV-VELECEF");
_Static_assert(sizeof(ubx_nav_velned_t)    == 36, "NAV-VELNED");
_Static_assert(sizeof(ubx_nav_timegps_t)   == 16, "NAV-TIMEGPS");
_Static_assert(sizeof(ubx_nav_timeutc_t)   == 20, "NAV-TIMEUTC");
_Static_assert(sizeof(ubx_nav_clock_t)     == 20, "NAV-CLOCK");
_Static_assert(sizeof(ubx_nav_svinfo_hdr_t) == 8, "NAV-SVINFO hdr");
_Static_assert(sizeof(ubx_nav_svinfo_blk_t) == 12, "NAV-SVINFO blk");
_Static_assert(sizeof(ubx_nav_dgps_hdr_t)  == 16, "NAV-DGPS hdr");
_Static_assert(sizeof(ubx_nav_dgps_blk_t)  == 12, "NAV-DGPS blk");
_Static_assert(sizeof(ubx_nav_sbas_hdr_t)  == 12, "NAV-SBAS hdr");
_Static_assert(sizeof(ubx_nav_sbas_blk_t)  == 12, "NAV-SBAS blk");
_Static_assert(sizeof(ubx_nav_aopstatus_t) == 20, "NAV-AOPSTATUS");
_Static_assert(sizeof(ubx_rxm_svsi_hdr_t)  == 8,  "RXM-SVSI hdr");
_Static_assert(sizeof(ubx_rxm_svsi_blk_t)  == 6,  "RXM-SVSI blk");
_Static_assert(sizeof(ubx_mon_hw_t)        == 68, "MON-HW");
_Static_assert(sizeof(ubx_mon_hw2_t)       == 28, "MON-HW2");
_Static_assert(sizeof(ubx_mon_io_port_t)   == 20, "MON-IO");
_Static_assert(sizeof(ubx_mon_msgpp_t)     == 120, "MON-MSGPP");
_Static_assert(sizeof(ubx_mon_rxbuf_t)     == 24, "MON-RXBUF");
_Static_assert(sizeof(ubx_mon_txbuf_t)     == 28, "MON-TXBUF");
_Static_assert(sizeof(ubx_tim_tp_t)        == 16, "TIM-TP");
_Static_assert(sizeof(ubx_tim_tm2_t)       == 28, "TIM-TM2");
_Static_assert(sizeof(ubx_tim_vrfy_t)      == 20, "TIM-VRFY");
_Static_assert(sizeof(ubx_aid_hui_t)       == 72, "AID-HUI");
_Static_assert(sizeof(ubx_aid_ini_t)       == 48, "AID-INI");
_Static_assert(sizeof(ubx_cfg_prt_t)       == 20, "CFG-PRT");
_Static_assert(sizeof(ubx_cfg_rate_t)      == 6,  "CFG-RATE");
_Static_assert(sizeof(ubx_cfg_cfg_t)       == 13, "CFG-CFG");
_Static_assert(sizeof(ubx_cfg_nav5_t)      == 36, "CFG-NAV5");
_Static_assert(sizeof(ubx_cfg_navx5_t)     == 40, "CFG-NAVX5");
_Static_assert(sizeof(ubx_cfg_sbas_t)      == 8,  "CFG-SBAS");
_Static_assert(sizeof(ubx_cfg_nmea_t)      == 4,  "CFG-NMEA");
_Static_assert(sizeof(ubx_cfg_tp5_t)       == 32, "CFG-TP5");
_Static_assert(sizeof(ubx_cfg_itfm_t)      == 8,  "CFG-ITFM");
_Static_assert(sizeof(ubx_cfg_ant_t)       == 4,  "CFG-ANT");
_Static_assert(sizeof(ubx_cfg_rxm_t)       == 2,  "CFG-RXM");
_Static_assert(sizeof(ubx_cfg_pm2_t)       == 44, "CFG-PM2");
_Static_assert(sizeof(ubx_cfg_dat_t)       == 52, "CFG-DAT");
