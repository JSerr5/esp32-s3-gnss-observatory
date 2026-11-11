/*
 * gps_console.c
 * ---------------------------------------------------------------------------
 * Agente 5 - Implementacion de los reportes estructurados en espanol para el
 * monitor serial a 115200 baudios:
 *   1) Reporte Inicial de Auditoria y Descubrimiento del Hardware
 *   2) Panel de Calentamiento y Diagnostico en Tiempo Real (Fases A..D)
 *   3) Panel Maestro de Datos Confiables Verificados (Fase E)
 * ---------------------------------------------------------------------------
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gps_config.h"
#include "gps_hw.h"
#include "gps_data.h"
#include "gps_warmup.h"
#include "gps_discovery.h"
#include "gps_console.h"
#include "gps_wifi.h"

/* Instantanea estatica en BSS para no consumir pila en la tarea de consola */
static gps_repo_t s_snap;

/* -------------------------- Helpers de formato -------------------------- */
static const char *ant_status_str(uint8_t a)
{
    switch (a) {
    case 0:  return "0:INIT";
    case 1:  return "1:DONTKNOW";
    case 2:  return "2:OK";
    case 3:  return "3:SHORT (Cortocircuito)";
    case 4:  return "4:OPEN (Circuito abierto)";
    default: return "?:Desconocido";
    }
}

static const char *ant_power_str(uint8_t p)
{
    switch (p) {
    case 0:  return "0:OFF";
    case 1:  return "1:ON";
    case 2:  return "2:DONTKNOW";
    default: return "?";
    }
}

static const char *jam_state_str(uint8_t flags)
{
    switch ((flags >> 2) & 0x03u) {
    case 0:  return "0:Desconocido/Inactivo";
    case 1:  return "1:OK (Sin interferencia)";
    case 2:  return "2:WARNING (Interferencia moderada)";
    case 3:  return "3:CRITICAL (Interferencia severa bloqueante)";
    default: return "?";
    }
}

static const char *psm_state_str(uint8_t flags2)
{
    switch (flags2 & 0x03u) {
    case 0:  return "0:Acquisition";
    case 1:  return "1:Tracking";
    case 2:  return "2:Power Optimized Tracking";
    case 3:  return "3:Inactive";
    default: return "?";
    }
}

static const char *sbas_sys_str(int8_t sys)
{
    switch (sys) {
    case -1: return "Desconocido";
    case 0:  return "WAAS";
    case 1:  return "EGNOS";
    case 2:  return "MSAS";
    case 3:  return "GAGAN";
    case 16: return "GPS";
    default: return "Otro";
    }
}

static const char *utc_std_str(uint8_t valid)
{
    switch ((valid >> 4) & 0x0Fu) {
    case 0:  return "Info no disponible";
    case 1:  return "CRL (Communications Research Lab)";
    case 2:  return "NIST";
    case 3:  return "USNO (U.S. Naval Observatory)";
    case 4:  return "BIPM";
    case 5:  return "Laboratorios Europeos";
    case 6:  return "SU (GLONASS)";
    default: return "Reservado";
    }
}

/* Convierte grados en escala 1e-7 a Grados, Minutos y Segundos */
static void format_dms(int32_t deg_1e7, char pos_hemi, char neg_hemi, char *out, size_t cap)
{
    char hemi = (deg_1e7 < 0) ? neg_hemi : pos_hemi;
    int64_t abs_v = (deg_1e7 < 0) ? -(int64_t)deg_1e7 : (int64_t)deg_1e7;
    int deg = (int)(abs_v / 10000000LL);
    double rem_deg = (double)(abs_v % 10000000LL) * 1e-7;
    double total_min = rem_deg * 60.0;
    int min = (int)total_min;
    double sec = (total_min - (double)min) * 60.0;
    snprintf(out, cap, "%3d deg %02d' %06.3f\" %c", deg, min, sec, hemi);
}

/* Busca el tiempo de bloqueo de portadora lck (0..64 s) en PUBX,03 */
static int find_pubx03_lck(const gps_repo_t *r, uint8_t svid, char *status_out)
{
    if (status_out) *status_out = '-';
    if (!r->pubx03.valid) return -1;
    for (int i = 0; i < r->pubx03.d.nblk; i++) {
        if (r->pubx03.d.sat[i].sv == (int)svid) {
            if (status_out) *status_out = r->pubx03.d.sat[i].status;
            return r->pubx03.d.sat[i].lck;
        }
    }
    return -1;
}

/* Busca svFlag y age de RXM-SVSI para un PRN dado */
static bool find_svsi_info(const gps_repo_t *r, uint8_t svid, uint8_t *svFlag, uint8_t *age)
{
    if (!r->svsi.valid) return false;
    for (uint8_t i = 0; i < r->svsi.n; i++) {
        if (r->svsi.b[i].svid == svid) {
            if (svFlag) *svFlag = r->svsi.b[i].svFlag;
            if (age)    *age    = r->svsi.b[i].age;
            return true;
        }
    }
    return false;
}

/* Busca correccion rapida prc e ionosferica ic en NAV-SBAS */
static bool find_sbas_corr(const gps_repo_t *r, uint8_t svid, int16_t *prc, int16_t *ic)
{
    if (!r->sbas.valid) return false;
    for (uint8_t i = 0; i < r->sbas.n; i++) {
        if (r->sbas.b[i].svid == svid) {
            if (prc) *prc = r->sbas.b[i].prc;
            if (ic)  *ic  = r->sbas.b[i].ic;
            return true;
        }
    }
    return false;
}

/* Busca residual NMEA GPGRS asociado al orden de PRNs de GPGSA */
static bool find_grs_residual(const gps_repo_t *r, uint8_t svid, float *res_m)
{
    if (!r->gsa.valid || !r->grs.valid) return false;
    for (int i = 0; i < r->gsa.d.nprn && i < 12; i++) {
        if (r->gsa.d.prn[i] == (int)svid && r->grs.d.res_valid[i]) {
            if (res_m) *res_m = r->grs.d.res[i];
            return true;
        }
    }
    return false;
}

/* Calcula hora local a partir de UTC aplicando un desfase en minutos */
static void compute_local_time(uint16_t y, uint8_t mo, uint8_t d,
                               uint8_t h, uint8_t mi, int offset_min,
                               uint16_t *oy, uint8_t *omo, uint8_t *od,
                               uint8_t *oh, uint8_t *omi)
{
    static const uint8_t mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    int total_m = (int)h * 60 + (int)mi + offset_min;
    int day_delta = 0;
    while (total_m < 0)    { total_m += 1440; day_delta--; }
    while (total_m >= 1440){ total_m -= 1440; day_delta++; }

    *oh  = (uint8_t)(total_m / 60);
    *omi = (uint8_t)(total_m % 60);

    int ny = (int)y, nmo = (int)mo, nd = (int)d + day_delta;
    bool leap = ((ny % 4 == 0 && ny % 100 != 0) || (ny % 400 == 0));
    if (nmo >= 1 && nmo <= 12) {
        int dim = mdays[nmo - 1] + ((nmo == 2 && leap) ? 1 : 0);
        if (nd < 1) {
            nmo--;
            if (nmo < 1) { nmo = 12; ny--; }
            nd = mdays[nmo - 1] + ((nmo == 2 && leap) ? 1 : 0);
        } else if (nd > dim) {
            nd = 1;
            nmo++;
            if (nmo > 12) { nmo = 1; ny++; }
        }
    }
    *oy  = (uint16_t)ny;
    *omo = (uint8_t)nmo;
    *od  = (uint8_t)nd;
}

/* =========================================================================
 * BANNER DE ARRANQUE Y AUTODIAGNOSTICO CRIPTOGRAFICO / CHECKSUM UBX
 * ========================================================================= */
void gps_console_print_boot_banner(void)
{
    printf("\r\n");
    printf("====================================================================================================\r\n");
    printf("  INSTRUMENTO DE AUDITORIA E INTERROGACION CONSCIENTE GNSS u-blox NEO-6M-0-001 (ROM 7.03 / UBX 13.03)\r\n");
    printf("  MCU: ESP32-S3 (ESP-IDF / FreeRTOS) | Firmware: %s v%s\r\n", GPS_FW_NAME, GPS_FW_VERSION);
    printf("----------------------------------------------------------------------------------------------------\r\n");
    printf("  RESTRICCION ESTRICTA DE HARDWARE (UNICAMENTE PINES IO4 AL IO6):\r\n");
    printf("    * GPIO 4 (IO4) -> UART1 TX (ESP32-S3) hacia pin RX del modulo GY-NEO6MV2\r\n");
    printf("    * GPIO 5 (IO5) <- UART1 RX (ESP32-S3, pull-up activo) desde pin TX del modulo GY-NEO6MV2\r\n");
    printf("    * GPIO 6 (IO6) <- Entrada opcional TIMEPULSE / PPS 1 Hz (pull-down seguro si no se suelda)\r\n");
    printf("====================================================================================================\r\n");
}

void gps_console_selftest_cb(const char *name, bool ok, const uint8_t *frame, size_t len)
{
    printf("  [AUTOTEST FLETCHER-8] %-28s : %s (", name, ok ? "EXACTO" : "FALLO");
    for (size_t i = 0; i < len; i++) {
        printf("%02X%s", frame[i], (i + 1 < len) ? " " : "");
    }
    printf(")\r\n");
}

/* =========================================================================
 * PANEL 1: REPORTE INICIAL DE AUDITORIA Y DESCUBRIMIENTO DEL HARDWARE
 * ========================================================================= */
void gps_console_print_audit_report(const gps_repo_t *r)
{
    gps_pps_stat_t pps;
    gps_hw_get_pps(&pps);
    int rx_lvl = gps_hw_rx_pin_level();

    printf("\r\n");
    printf("####################################################################################################\r\n");
    printf("# 1. REPORTE INICIAL DE AUDITORIA Y DESCUBRIMIENTO DEL HARDWARE (u-blox NEO-6M-0-001)              #\r\n");
    printf("####################################################################################################\r\n");

    /* 1.a) Estado del Enlace Fisico y Pines IO4..IO6 */
    printf("\r\n[1.A] ENLACE SERIE Y ESTADO FISICO DE PINES (GPIO 4, GPIO 5, GPIO 6):\r\n");
    printf("  * Baudrate original detectado : %lu bps (barrido Auto-Bauding completado en %lu ciclo(s))\r\n",
           (unsigned long)r->detected_baud, (unsigned long)(r->autobaud_cycles + 1u));
    printf("  * Baudrate final negociado    : %lu bps (%s)\r\n",
           (unsigned long)r->final_baud,
           r->baud_switch_ok ? "Conmutado con exito mediante UBX-CFG-PRT"
                             : "Mantenido en velocidad original de respaldo");
    printf("  * Linea GPIO 4 (TX ESP32-S3)  : %s (%lu bytes enviados)\r\n",
           r->bidir ? "ACTIVA Y VERIFICADA (El receptor responde a comandos UBX)"
                    : "SIN RESPUESTA UBX (Modo solo escucha; revise cable GPIO 4 -> RX GPS)",
           (unsigned long)r->bytes_tx);
    printf("  * Linea GPIO 5 (RX ESP32-S3)  : ACTIVA (Nivel actual=%d, %lu bytes recibidos, UBX_OK=%lu, NMEA_OK=%lu)\r\n",
           rx_lvl, (unsigned long)r->bytes_rx,
           (unsigned long)r->ubx_ok, (unsigned long)r->nmea_ok);
    if (pps.pulse_count > 0) {
        printf("  * Pulso PPS en GPIO 6 (IO6)   : DETECTADO (%lu pulsos, periodo=%lld us, jitter=%+lld us, max=%lld us)\r\n",
               (unsigned long)pps.pulse_count,
               (long long)pps.last_period_us,
               (long long)pps.last_jitter_us,
               (long long)pps.max_jitter_us);
    } else {
        printf("  * Pulso PPS en GPIO 6 (IO6)   : No cableado (Operando 100%% por UART)\r\n");
    }

    /* 1.b) Identidad de Silicio, Autenticidad y Memoria No Volatil */
    printf("\r\n[1.B] IDENTIDAD EXTRAIDA DEL SILICIO, AUTENTICIDAD Y MEMORIA NO VOLATIL:\r\n");
    printf("  * UBX-MON-VER swVersion       : %s\r\n", r->ver_valid ? r->swVersion : "(Sin respuesta UBX-MON-VER)");
    printf("  * UBX-MON-VER hwVersion       : %s (Esperado u-blox 6: 00040007)\r\n",
           r->ver_valid ? r->hwVersion : "(Sin respuesta)");
    printf("  * Protocolo UBX verificado    : %s (Especificacion objetivo: 13.03)\r\n", r->protver);
    for (uint8_t i = 0; i < r->n_ext; i++) {
        printf("  * Extension MON-VER [%u]       : %s\r\n", i, r->ext[i]);
    }
    if (r->txt_rom[0] || r->txt_hw[0]) {
        printf("  * Arranque NMEA GPTXT         : HW='%s' | ROM='%s' | ANT='%s'\r\n",
               r->txt_hw[0] ? r->txt_hw : "-",
               r->txt_rom[0] ? r->txt_rom : "-",
               r->antstatus);
    }
    if (r->hw2.valid) {
        printf("  * Fuente de arranque cfgSource: %s\r\n", gps_cfg_source_str(r->hw2.d.cfgSource));
        printf("  * Autodiagnostico POST Status : 0x%08lX | lowLevCfg=0x%08lX\r\n",
               (unsigned long)r->hw2.d.postStatus, (unsigned long)r->hw2.d.lowLevCfg);
    }
    if (r->hw.valid) {
        printf("  * Supervisor de Antena        : aStatus=%s | aPower=%s | rtcCalib=%u | safeBoot=%u\r\n",
               ant_status_str(r->hw.d.aStatus),
               ant_power_str(r->hw.d.aPower),
               (unsigned)(r->hw.d.flags & 0x01u),
               (unsigned)((r->hw.d.flags >> 1) & 0x01u));
    }
    printf("  * Prueba Memoria BBR (bit 0)  : %s\r\n", gps_res_str(r->nvm_bbr));
    printf("  * Prueba EEPROM I2C 24C32(b2) : %s %s\r\n",
           gps_res_str(r->nvm_eeprom),
           (r->nvm_eeprom == RES_ACK) ? "(EEPROM externa en bus DDC/I2C operativa)"
                                      : "(Sin ACK de EEPROM externa)");
    printf("  * Prueba combinada (dev=0x05) : %s (Trama B5 62 06 09 ... 05 46 05)\r\n",
           gps_res_str(r->nvm_both));
    printf("  * VEREDICTO DE AUTENTICIDAD   : %s\r\n", gps_auth_str(r->auth));
    printf("  * Detalle de auditoria        : %s\r\n", r->auth_reason);

    /* 1.c) Matriz Completa de Capacidades Soportadas vs Rechazadas */
    printf("\r\n[1.C] MATRIZ DE AUTODESCUBRIMIENTO DE MENSAJES (SOPORTADOS vs RECHAZADOS EN ROM 7.03):\r\n");
    printf("  +--------+----------------+-------------+--------+--------+------+--------+--------------------------------+\r\n");
    printf("  | Grupo  | Mensaje        | Cls/ID      | CFGMSG | Sondeo | Tasa | RxCnt  | Estado Empirico / Nota         |\r\n");
    printf("  +--------+----------------+-------------+--------+--------+------+--------+--------------------------------+\r\n");

    for (int i = 0; i < g_msg_table_len; i++) {
        const gps_msg_desc_t *d = &g_msg_table[i];
        const gps_msg_stat_t *s = &r->mstat[i];
        const char *verdict = "EN OBSERVACION";

        if (d->flags & MF_NEG) {
            if (s->cfgmsg == RES_NAK || s->poll == RES_NAK) {
                verdict = "NO SOPORTADO (ACK-NAK Confirmado)";
            } else if (s->cfgmsg == RES_TIMEOUT && s->poll == RES_TIMEOUT) {
                verdict = "NO SOPORTADO (Silencio/Timeout)";
            } else {
                verdict = "INESPERADO (Aceptado)";
            }
        } else if (s->cfgmsg == RES_ACK || s->poll == RES_DATA || s->poll == RES_ACK || s->rx_count > 0) {
            verdict = "SOPORTADO Y ACTIVO";
        } else if (s->cfgmsg == RES_NAK && s->poll == RES_NAK) {
            verdict = "NO SOPORTADO (ACK-NAK)";
        }

        printf("  | %-6s | %-14s | 0x%02X 0x%02X   | %-6s | %-6s | %3u  | %6lu | %-30s |\r\n",
               gps_group_name(d->group),
               d->name,
               d->cls, d->id,
               gps_res_str(s->cfgmsg),
               gps_res_str(s->poll),
               (unsigned)s->rate_set,
               (unsigned long)s->rx_count,
               verdict);
    }
    printf("  +--------+----------------+-------------+--------+--------+------+--------+--------------------------------+\r\n");

    /* 1.d) Resumen de Configuracion Actual Clase CFG Leida por Sondeo */
    printf("\r\n[1.D] CONFIGURACIONES DE HARDWARE Y NAVEGACION LEIDAS DEL CHIP (CLASE CFG 0x06):\r\n");
    if (r->cfg_prt.valid) {
        printf("  * CFG-PRT (UART1)  : baud=%lu bps | mode=0x%08lX | inProto=0x%04X | outProto=0x%04X\r\n",
               (unsigned long)r->cfg_prt.d.baudRate,
               (unsigned long)r->cfg_prt.d.mode,
               r->cfg_prt.d.inProtoMask,
               r->cfg_prt.d.outProtoMask);
    }
    if (r->cfg_rate.valid) {
        printf("  * CFG-RATE         : measRate=%u ms | navRate=%u ciclos | timeRef=%u (%s)\r\n",
               r->cfg_rate.d.measRate, r->cfg_rate.d.navRate, r->cfg_rate.d.timeRef,
               (r->cfg_rate.d.timeRef == 1) ? "GPS" : "UTC");
    }
    if (r->cfg_nav5.valid) {
        printf("  * CFG-NAV5         : dynModel=%u | fixMode=%u | minElev=%d deg | pDop=%u.%u | staticHold=%u cm/s\r\n",
               r->cfg_nav5.d.dynModel, r->cfg_nav5.d.fixMode, r->cfg_nav5.d.minElev,
               r->cfg_nav5.d.pDop / 10u, r->cfg_nav5.d.pDop % 10u,
               r->cfg_nav5.d.staticHoldThresh);
    }
    if (r->cfg_navx5.valid) {
        printf("  * CFG-NAVX5        : minSVs=%u | maxSVs=%u | minCNO=%u dB-Hz | usePPP=%u | useAOP=%u | wknRoll=%u\r\n",
               r->cfg_navx5.d.minSVs, r->cfg_navx5.d.maxSVs, r->cfg_navx5.d.minCNO,
               r->cfg_navx5.d.usePPP, r->cfg_navx5.d.useAOP, r->cfg_navx5.d.wknRollover);
    }
    if (r->cfg_sbas.valid) {
        printf("  * CFG-SBAS         : mode=0x%02X | usage=0x%02X | maxSBAS=%u | scanmode1=0x%08lX\r\n",
               r->cfg_sbas.d.mode, r->cfg_sbas.d.usage, r->cfg_sbas.d.maxSBAS,
               (unsigned long)r->cfg_sbas.d.scanmode1);
    }
    if (r->cfg_nmea.valid) {
        printf("  * CFG-NMEA         : version=0x%02X | numSV=%u (0=Ilimitado) | filter=0x%02X | flags=0x%02X\r\n",
               r->cfg_nmea.d.version, r->cfg_nmea.d.numSV,
               r->cfg_nmea.d.filter, r->cfg_nmea.d.flags);
    }
    if (r->cfg_tp5.valid) {
        printf("  * CFG-TP5 (PPS)    : freqPeriod=%lu | Lock=%lu | lenRatio=%lu | antDelay=%d ns | flags=0x%08lX\r\n",
               (unsigned long)r->cfg_tp5.d.freqPeriod,
               (unsigned long)r->cfg_tp5.d.freqPeriodLock,
               (unsigned long)r->cfg_tp5.d.pulseLenRatio,
               r->cfg_tp5.d.antCableDelay,
               (unsigned long)r->cfg_tp5.d.flags);
    }
    if (r->cfg_itfm.valid) {
        uint32_t c = r->cfg_itfm.d.config;
        printf("  * CFG-ITFM (Jam)   : enable=%u | bbThreshold=%u | cwThreshold=%u | config=0x%08lX\r\n",
               (unsigned)((c >> 31) & 1u), (unsigned)(c & 0x0Fu), (unsigned)((c >> 4) & 0x1Fu),
               (unsigned long)c);
    }
    if (r->cfg_ant.valid) {
        printf("  * CFG-ANT          : flags=0x%04X | pins=0x%04X\r\n",
               r->cfg_ant.d.flags, r->cfg_ant.d.pins);
    }
    if (r->cfg_rxm.valid) {
        printf("  * CFG-RXM          : lpMode=%u (0=MaxPerf, 1=PowerSave, 4=Eco)\r\n",
               r->cfg_rxm.d.lpMode);
    }
    if (r->cfg_pm2.valid) {
        printf("  * CFG-PM2          : updatePeriod=%lu ms | searchPeriod=%lu ms | onTime=%u s | flags=0x%08lX\r\n",
               (unsigned long)r->cfg_pm2.d.updatePeriod,
               (unsigned long)r->cfg_pm2.d.searchPeriod,
               r->cfg_pm2.d.onTime,
               (unsigned long)r->cfg_pm2.d.flags);
    }
    if (r->cfg_dat.valid) {
        char dn[7];
        memcpy(dn, r->cfg_dat.d.datumName, 6);
        dn[6] = '\0';
        printf("  * CFG-DAT (Datum)  : num=%u (%s) | majA=%.3f m | flat=1/%.6f\r\n",
               r->cfg_dat.d.datumNum, dn, r->cfg_dat.d.majA,
               (r->cfg_dat.d.flat != 0.0) ? (1.0 / r->cfg_dat.d.flat) : 0.0);
    }
    printf("####################################################################################################\r\n\r\n");
}

/* =========================================================================
 * PANEL 2: PANEL DE CALENTAMIENTO Y DIAGNOSTICO EN TIEMPO REAL (CADA 2 S)
 * ========================================================================= */
void gps_console_print_warmup_panel(const gps_repo_t *r)
{
    uint32_t now_ms = gps_now_ms();
    uint32_t esp_s  = (now_ms - r->boot_ms) / 1000u;
    uint32_t msss_s = r->status.valid ? (r->status.d.msss / 1000u) : 0;

    printf("\r\n");
    printf("====================================================================================================\r\n");
    printf(" [PANEL DE CALENTAMIENTO Y ADQUISICION EN FRIO]  ESP32-S3: %4lus | Chip msss: %4lus (%lu ms)\r\n",
           (unsigned long)esp_s,
           (unsigned long)msss_s,
           (unsigned long)(r->status.valid ? r->status.d.msss : 0u));
    printf(" Estado actual : %s\r\n", gps_phase_name(r->phase));
    printf(" Posicion      : EN ESPERA DE CIELO ABIERTO / VALIDACION CIENTIFICA (%u/%u epocas validas seguidas)\r\n",
           r->good_epochs, (unsigned)TRUST_REQUIRED_EPOCHS);
    printf("----------------------------------------------------------------------------------------------------\r\n");

    /* 2.b) Salud electrica, RF, reloj y buffers */
    printf(" [2.A] SALUD ELECTRICA, RADIOFRECUENCIA, CRISTAL Y BUFFERS INTERNOS:\r\n");
    if (r->hw.valid) {
        double agc_pct = (double)r->hw.d.agcCnt / 81.91;
        double jam_pct = (double)r->hw.d.jamInd / 2.55;
        printf("  * Antena / RF (MON-HW) : aStatus=%s | aPower=%s | Ruido=%u cnt/ms | AGC=%u/8191 (%.1f%%)\r\n",
               ant_status_str(r->hw.d.aStatus),
               ant_power_str(r->hw.d.aPower),
               r->hw.d.noisePerMS,
               r->hw.d.agcCnt, agc_pct);
        printf("  * Interferencia Jamming: CW jamInd=%u/255 (%.1f%%) | Monitor ITFM=%s\r\n",
               r->hw.d.jamInd, jam_pct, jam_state_str(r->hw.d.flags));
    } else {
        printf("  * Antena / RF (MON-HW) : Esperando lectura de UBX-MON-HW...\r\n");
    }

    if (r->hw2.valid) {
        printf("  * Conversor I/Q (HW2)  : ofsI=%+d magI=%u | ofsQ=%+d magQ=%u | cfgSource=%u\r\n",
               r->hw2.d.ofsI, r->hw2.d.magI, r->hw2.d.ofsQ, r->hw2.d.magQ, r->hw2.d.cfgSource);
    }

    if (r->clock.valid) {
        printf("  * Oscilador Cristal    : Sesgo clkB=%+ld ns | Deriva clkD=%+ld ns/s (ppb) | tAcc=%lu ns | fAcc=%lu ps/s\r\n",
               (long)r->clock.d.clkB, (long)r->clock.d.clkD,
               (unsigned long)r->clock.d.tAcc, (unsigned long)r->clock.d.fAcc);
    }

    if (r->txbuf.valid || r->rxbuf.valid) {
        printf("  * Buffers GPS (UART1)  : TX usage=%u%% (pico=%u%%, totalPeak=%u%%, err=0x%02X) | RX usage=%u%% (pend=%u B)\r\n",
               r->txbuf.valid ? r->txbuf.d.usage[1] : 0,
               r->txbuf.valid ? r->txbuf.d.peakUsage[1] : 0,
               r->txbuf_peak_seen,
               r->txbuf.valid ? r->txbuf.d.errors : 0,
               r->rxbuf.valid ? r->rxbuf.d.usage[1] : 0,
               r->rxbuf.valid ? r->rxbuf.d.pending[1] : 0);
    }

    /* 2.c) Progreso de descarga orbital y tiempo atomico */
    printf("\r\n [2.B] PROGRESO DE ENGANCHE DE CANALES Y DESCARGA DE EFEMERIDES / ALMANAQUE (50 bps):\r\n");
    printf("  * Canales correladores : Total=%u | Inactivos=%u | Buscando(1)=%u | Adq(2)=%u | Detect(3)=%u\r\n",
           r->ch_total, r->ch_idle, r->ch_search, r->ch_acq, r->ch_detect);
    printf("  * Enganche de senal    : Code Lock(4)=%u | Carrier Lock(5..7)=%u | Con C/N0>0=%u | C/N0>=27 dB-Hz=%u\r\n",
           r->ch_code, r->ch_carrier, r->sv_cno_pos, r->sv_strong);
    printf("  * Datos orbitales      : Efemerides validas(ephVal)=%u | Almanaque(almVal)=%u | Usados(svUsed)=%u | SBAS=%u\r\n",
           r->sv_eph, r->sv_alm, r->sv_used, r->sv_sbas);

    const char *leap_origin = "Pendiente";
    int leap_val = 0;
    if (r->timegps.valid && (r->timegps.d.valid & 0x04)) {
        leap_val = r->timegps.d.leapS;
        leap_origin = "VERIFICADO DESDE SATELITE (NAV-TIMEGPS)";
    } else if (r->pubx04.valid) {
        leap_val = r->pubx04.d.leapSec;
        leap_origin = r->pubx04.d.leap_default ? "POR DEFECTO DE ROM (sufijo 'D' en PUBX,04)"
                                               : "ACTUALIZADO DESDE EL ESPACIO (PUBX,04)";
    }
    printf("  * Segundo intercalar   : %d s [%s]\r\n", leap_val, leap_origin);
    printf("  * Ionosfera Klobuchar  : %s (AID-HUI flags=0x%02lX)\r\n",
           r->klob_valid ? "COEFICIENTES VALIDOS RECIBIDOS" : "Esperando subtrama 4 pagina 18...",
           (unsigned long)(r->hui.valid ? r->hui.d.flags : 0u));

    if (r->timeutc.valid && r->time_known) {
        printf("  * Hora GPS/UTC prelim. : %04u-%02u-%02u %02u:%02u:%02u.%09ld UTC (valid=0x%02X, tAcc=%lu ns)\r\n",
               r->timeutc.d.year, r->timeutc.d.month, r->timeutc.d.day,
               r->timeutc.d.hour, r->timeutc.d.min, r->timeutc.d.sec,
               (long)r->timeutc.d.nano, r->timeutc.d.valid,
               (unsigned long)r->timeutc.d.tAcc);
    }

    if (r->ttff_frozen) {
        printf("  * Convergencia Kalman  : TTFF=%lu ms | hAcc=%.2f m (ini=%.2f, min=%.2f) | vAcc=%.2f m | PDOP=%.2f\r\n",
               (unsigned long)r->ttff_ms,
               (double)r->pvt.hAcc / 1000.0,
               (double)r->hacc_first / 1000.0,
               (r->hacc_min < UINT32_MAX) ? ((double)r->hacc_min / 1000.0) : 0.0,
               (double)r->pvt.vAcc / 1000.0,
               (double)((r->pvt.pDOP > 0) ? r->pvt.pDOP : r->pvt.pDOP_sol) / 100.0);
    }

    /* 2.d) Tabla en vivo de satelites visibles / activos */
    printf("\r\n [2.C] TABLA EN VIVO DE CANALES / SATELITES EN SEGUIMIENTO O VISTA:\r\n");
    printf("  +-----+-----+------+------+------+------+-----------------------+------+---------+-----------+\r\n");
    printf("  | CHN | PRN | Tipo | C/N0 | Elev | Azim | Estado qualityInd     | LckS |Orb/Uso  | prRes(cm) |\r\n");
    printf("  +-----+-----+------+------+------+------+-----------------------+------+---------+-----------+\r\n");

    int shown = 0;
    if (r->svinfo.valid) {
        for (uint8_t i = 0; i < r->svinfo.n; i++) {
            const ubx_nav_svinfo_blk_t *b = &r->svinfo.b[i];
            uint8_t q = (uint8_t)(b->quality & 0x0Fu);
            /* Mostrar todos los canales con senal, bloqueo o satelite en vista */
            if (b->cno == 0 && q <= 1 && b->elev < 0) {
                continue;
            }
            const char *type = (b->svid >= 120 && b->svid <= 158) ? "SBAS" : "GPS ";
            int lck = find_pubx03_lck(r, b->svid, NULL);
            char flags_s[8];
            snprintf(flags_s, sizeof(flags_s), "%c%c%c%c%c",
                     (b->flags & 0x01) ? 'U' : '-',
                     (b->flags & 0x08) ? 'E' : '-',
                     (b->flags & 0x20) ? 'A' : '-',
                     (b->flags & 0x02) ? 'D' : '-',
                     (b->flags & 0x10) ? '!' : 'H');
            printf("  | %3u | %3u | %s | %4u | %4d | %4d | %-21s | %4d | %-7s | %+9ld |\r\n",
                   b->chn, b->svid, type, b->cno, b->elev, b->azim,
                   gps_warmup_quality_str(q), lck, flags_s, (long)b->prRes);
            shown++;
        }
    } else if (r->gsv.valid) {
        for (int i = 0; i < r->gsv.n; i++) {
            const gps_gsv_sat_t *s = &r->gsv.s[i];
            const char *type = (s->prn >= 120 && s->prn <= 158) ? "SBAS" : "GPS ";
            printf("  |  -  | %3d | %s | %4d | %4d | %4d | (Solo NMEA GPGSV)     |   -  |    -    |         - |\r\n",
                   s->prn, type, s->cno, s->elev, s->azim);
            shown++;
        }
    }
    if (shown == 0) {
        printf("  |  -  |  -  |  -   |   0  |   -  |   -  | Buscando portadoras L1 (1575.42 MHz)...                  |\r\n");
    }
    printf("  +-----+-----+------+------+------+------+-----------------------+------+---------+-----------+\r\n");
    printf("  Leyenda Orb/Uso: U=Usado en fix, E=Efemeride OK, A=Almanaque OK, D=DGPS, H=Sano (!=Unhealthy)\r\n");

    /* 2.e) Lista explicita de los 9 criterios del Filtro Cientifico de Confianza */
    printf("\r\n [2.D] ESTADO DEL FILTRO CIENTIFICO DE CONFIANZA (9 CONDICIONES OBLIGATORIAS):\r\n");
    for (int c = 0; c < CRIT_COUNT; c++) {
        char desc[128];
        bool ok = gps_warmup_crit_desc(c, r, desc, sizeof(desc));
        printf("  [%s] %d. %s\r\n", ok ? "CUMPLIDO " : "PENDIENTE", c + 1, desc);
    }
    if (r->indoor_timeout) {
        printf("  [AVISO INTERIOR] Han transcurrido > %u s sin fijacion geometrica: acerque la antena ceramica a cielo abierto.\r\n",
               (unsigned)WARMUP_INDOOR_TIMEOUT_S);
    }
    printf("====================================================================================================\r\n");
}

/* =========================================================================
 * PANEL 3: PANEL MAESTRO DE DATOS CONFIABLES VERIFICADOS (FASE E)
 * ========================================================================= */
void gps_console_print_master_panel(const gps_repo_t *r)
{
    const gps_pvt_t *p = &r->pvt;
    gps_pps_stat_t pps;
    gps_hw_get_pps(&pps);

    double lat_deg    = (double)p->lat * 1e-7;
    double lon_deg    = (double)p->lon * 1e-7;
    double h_elip_m   = (double)p->height / 1000.0;
    double h_msl_m    = (double)p->hMSL / 1000.0;
    double geoid_und  = h_elip_m - h_msl_m;
    double hacc_m     = (double)p->hAcc / 1000.0;
    double vacc_m     = (double)p->vAcc / 1000.0;
    double pacc_m     = (double)p->pAcc / 100.0;
    double ecefx_m    = (double)p->ecefX / 100.0;
    double ecefy_m    = (double)p->ecefY / 100.0;
    double ecefz_m    = (double)p->ecefZ / 100.0;

    char lat_dms[40], lon_dms[40];
    format_dms(p->lat, 'N', 'S', lat_dms, sizeof(lat_dms));
    format_dms(p->lon, 'E', 'W', lon_dms, sizeof(lon_dms));

    printf("\r\n");
    printf("****************************************************************************************************\r\n");
    printf("* PANEL MAESTRO DE DATOS CONFIABLES VERIFICADOS - u-blox NEO-6M-0-001 (SOLUCION CERTIFICADA)       *\r\n");
    printf("* Epoca iTOW: %10lu ms | Semana GPS: %4d | Epocas certificadas: %lu (racha=%u)             *\r\n",
           (unsigned long)p->iTOW, p->week, (unsigned long)r->pvt_epochs, r->good_epochs);
    printf("****************************************************************************************************\r\n");

    /* a) Posicion Geodesica y Cartesiana Confiable */
    printf("\r\n[A] POSICION GEODESICA Y CARTESIANA CONFIABLE (WGS-84):\r\n");
    printf("  * Latitud Geodesica (NAV-POSLLH) : %+12.7f deg  (%s)\r\n", lat_deg, lat_dms);
    printf("  * Longitud Geodesica (NAV-POSLLH): %+12.7f deg  (%s)\r\n", lon_deg, lon_dms);
    printf("  * Altitud Ortometrica (hMSL)     : %+10.3f m sobre el nivel medio del mar\r\n", h_msl_m);
    printf("  * Altura Elipsoidal (height)     : %+10.3f m sobre el elipsoide WGS-84\r\n", h_elip_m);
    printf("  * Ondulacion Geoidal (height-MSL): %+10.3f m (GPGGA sep=%.3f m)\r\n",
           geoid_und, r->gga.valid ? (double)r->gga.d.geoid_sep : 0.0);
    printf("  * Coordenadas ECEF (X, Y, Z)     : X=%+13.2f m | Y=%+13.2f m | Z=%+13.2f m\r\n",
           ecefx_m, ecefy_m, ecefz_m);
    printf("  * Datum Activo (GPDTM / CFG-DAT) : Local='%s' Ref='%s' (dLat=%+.4f' dLon=%+.4f' dAlt=%+.2f m)\r\n",
           r->dtm.valid ? r->dtm.d.datum : "W84",
           r->dtm.valid ? r->dtm.d.ref_datum : "W84",
           r->dtm.valid ? (double)r->dtm.d.lat_ofs_min : 0.0,
           r->dtm.valid ? (double)r->dtm.d.lon_ofs_min : 0.0,
           r->dtm.valid ? (double)r->dtm.d.alt_ofs_m : 0.0);
    printf("  * Incertidumbres UBX             : hAcc=%.3f m | vAcc=%.3f m | pAcc(3D)=%.3f m\r\n",
           hacc_m, vacc_m, pacc_m);
    if (r->gst.valid) {
        printf("  * Estadisticos NMEA GPGST        : rangeRms=%.3f m | stdLat=%.3f m | stdLon=%.3f m | stdAlt=%.3f m\r\n",
               (double)r->gst.d.rangeRms, (double)r->gst.d.stdLat,
               (double)r->gst.d.stdLon,   (double)r->gst.d.stdAlt);
    }
    if (r->pubx00.valid) {
        printf("  * Cruce Propietario PUBX,00      : navStat='%s' | altRef=%.2f m | hAcc=%.2f m | vAcc=%.2f m\r\n",
               r->pubx00.d.navStat, (double)r->pubx00.d.altRef,
               (double)r->pubx00.d.hAcc, (double)r->pubx00.d.vAcc);
    }
    if (r->gll.valid) {
        printf("  * Cruce NMEA GPGLL               : Lat=%+.6f | Lon=%+.6f | Estado='%c' | Modo='%c'\r\n",
               r->gll.d.lat, r->gll.d.lon,
               r->gll.d.status ? r->gll.d.status : '-',
               r->gll.d.mode   ? r->gll.d.mode   : '-');
    }

    /* b) Cinematica y Vectores de Velocidad Confiables */
    double gspeed_ms  = (double)p->gSpeed / 100.0;
    double gspeed_kmh = gspeed_ms * 3.6;
    double gspeed_kn  = gspeed_ms * 1.943844;
    double speed3d_ms = (double)p->speed / 100.0;
    double veln_ms    = (double)p->velN / 100.0;
    double vele_ms    = (double)p->velE / 100.0;
    double veld_ms    = (double)p->velD / 100.0;
    double climb_ms   = -veld_ms; /* invertir signo de velD */
    double vx_ms      = (double)p->ecefVX / 100.0;
    double vy_ms      = (double)p->ecefVY / 100.0;
    double vz_ms      = (double)p->ecefVZ / 100.0;
    double sacc_ms    = (double)p->sAcc / 100.0;
    double head_deg   = (double)p->heading * 1e-5;
    double cacc_deg   = (double)p->cAcc * 1e-5;

    printf("\r\n[B] CINEMATICA Y VECTORES DE VELOCIDAD CONFIABLES:\r\n");
    printf("  * Velocidad Horizontal 2D (gSpeed): %7.3f m/s | %7.3f km/h | %7.3f nudos\r\n",
           gspeed_ms, gspeed_kmh, gspeed_kn);
    printf("  * Velocidad Total 3D (speed)      : %7.3f m/s | %7.3f km/h (Incertidumbre sAcc=%.3f m/s)\r\n",
           speed3d_ms, speed3d_ms * 3.6, sacc_ms);
    printf("  * Vector Local NED (NAV-VELNED)   : Norte=%+7.3f m/s | Este=%+7.3f m/s | Abajo=%+7.3f m/s\r\n",
           veln_ms, vele_ms, veld_ms);
    printf("  * Tasa de Ascenso (Climb Rate)    : %+7.3f m/s (-velD)\r\n", climb_ms);
    printf("  * Vector Cartesiano ECEF (VX,Y,Z) : VX=%+8.3f m/s | VY=%+8.3f m/s | VZ=%+8.3f m/s\r\n",
           vx_ms, vy_ms, vz_ms);
    if (r->heading_static) {
        if (r->heading_have) {
            printf("  * Rumbo Verdadero (COG / heading) : INDETERMINADO POR REPOSO (gSpeed < 0.10 m/s) | Ultimo valido=%.2f deg\r\n",
                   (double)r->heading_last_valid * 1e-5);
        } else {
            printf("  * Rumbo Verdadero (COG / heading) : INDETERMINADO POR REPOSO (gSpeed < 0.10 m/s; sin movimiento previo)\r\n");
        }
    } else {
        printf("  * Rumbo Verdadero (COG / heading) : %7.2f deg (Incertidumbre de rumbo cAcc=%.2f deg)\r\n",
               head_deg, cacc_deg);
    }
    if (r->vtg.valid) {
        printf("  * Cruce NMEA GPVTG               : COG=%.2f deg (T) | SOG=%.3f nudos | %.3f km/h | Modo='%c'\r\n",
               (double)r->vtg.d.cog_true, (double)r->vtg.d.sog_kn,
               (double)r->vtg.d.sog_kmh, r->vtg.d.mode ? r->vtg.d.mode : '-');
    }

    /* c) Geometria Satelital, Calidad de Fijacion e Integridad RAIM */
    printf("\r\n[C] GEOMETRIA SATELITAL, CALIDAD DE FIJACION, SBAS/DGPS E INTEGRIDAD RAIM:\r\n");
    printf("  * Tipo de Fijacion                : %u (%s) | DGPS/SBAS diffSoln=%s | Estado PSM=%s\r\n",
           p->gpsFix_sol, gps_fix_name(p->gpsFix_sol),
           (p->flags_sol & 0x02) ? "ACTIVO" : "Inactivo",
           psm_state_str(p->flags2));
    printf("  * Tiempo al Primer Fix (TTFF)     : %.3f s (%lu ms chip | %lu ms reloj ESP32-S3)\r\n",
           (double)r->ttff_ms / 1000.0, (unsigned long)r->ttff_ms, (unsigned long)r->ttff_local_ms);
    printf("  * Satelites Usados vs En Vista    : numSV=%u usados en solucion (sin limite NMEA) | %u en vista | %u con efem.\r\n",
           p->numSV, r->sv_vis, r->sv_eph);
    if (r->gsa.valid) {
        printf("  * PRNs en GPGSA (%2d satelites)    : ", r->gsa.d.nprn);
        for (int i = 0; i < r->gsa.d.nprn; i++) {
            printf("%d%s", r->gsa.d.prn[i], (i + 1 < r->gsa.d.nprn) ? "," : "");
        }
        printf("\r\n");
    }
    printf("  * 7 Diluciones de Precision (DOP) : GDOP=%.2f | PDOP=%.2f | TDOP=%.2f | VDOP=%.2f | HDOP=%.2f | NDOP=%.2f | EDOP=%.2f\r\n",
           (double)p->gDOP / 100.0, (double)p->pDOP / 100.0, (double)p->tDOP / 100.0,
           (double)p->vDOP / 100.0, (double)p->hDOP / 100.0, (double)p->nDOP / 100.0,
           (double)p->eDOP / 100.0);
    if (r->gbs.valid) {
        printf("  * Integridad Autonoma RAIM (GPGBS): errLat=%.2f m | errLon=%.2f m | errAlt=%.2f m | svFallido=%d | bias=%.2f m\r\n",
               (double)r->gbs.d.errLat, (double)r->gbs.d.errLon, (double)r->gbs.d.errAlt,
               r->gbs.d.svid_valid ? r->gbs.d.svid : 0, (double)r->gbs.d.bias);
    }
    if (r->sbas.valid) {
        printf("  * Estado SBAS (UBX-NAV-SBAS)      : PRN Geo=%u | Sistema=%s | Modo=%u | Servicio=0x%02X | SVs=%u\r\n",
               r->sbas.h.geo, sbas_sys_str(r->sbas.h.sys),
               r->sbas.h.mode, r->sbas.h.service, r->sbas.n);
    }
    if (r->dgps.valid) {
        printf("  * Estado DGPS (UBX-NAV-DGPS)      : age=%ld ms | baseId=%d | baseHealth=%d | numCh=%u | status=%u\r\n",
               (long)r->dgps.h.age, r->dgps.h.baseId, r->dgps.h.baseHealth,
               r->dgps.h.numCh, r->dgps.h.status);
    }
    if (r->sbas_policy[0]) {
        printf("  * Politica Ionosferica Regional   : %s\r\n", r->sbas_policy);
    }

    /* d) Tiempo Atomico, Reloj del Cristal y Metrologia PPS */
    uint16_t ly; uint8_t lmo, ld, lh, lmi;
    compute_local_time(p->year, p->month, p->day, p->hour, p->min,
                       GPS_LOCAL_UTC_OFFSET_MIN, &ly, &lmo, &ld, &lh, &lmi);

    printf("\r\n[D] TIEMPO ATOMICO, RELOJ DEL CRISTAL Y METROLOGIA TIMEPULSE (PPS):\r\n");
    printf("  * Fecha y Hora UTC Certificada    : %04u-%02u-%02u %02u:%02u:%02u (nano=%+ld ns) UTC (Est. %s)\r\n",
           p->year, p->month, p->day, p->hour, p->min, p->sec, (long)p->nano,
           utc_std_str(p->utc_valid));
    printf("  * Fecha y Hora Local (UTC%+d min) : %04u-%02u-%02u %02u:%02u:%02u\r\n",
           GPS_LOCAL_UTC_OFFSET_MIN, ly, lmo, ld, lh, lmi, p->sec);
    printf("  * Semana GPS y Tiempo de Semana   : week=%d | iTOW=%lu ms | fTOW=%+ld ns | leapS=%d s (valid=0x%02X)\r\n",
           p->week, (unsigned long)p->iTOW, (long)p->fTOW, p->leapS, p->tgps_valid);
    printf("  * Incertidumbre Temporal (tAcc)   : UTC tAcc=%lu ns | GPS tAcc=%lu ns\r\n",
           (unsigned long)p->utc_tAcc, (unsigned long)p->tgps_tAcc);
    printf("  * Reloj del Cristal (NAV-CLOCK)   : Sesgo clkB=%+ld ns | Deriva clkD=%+ld ns/s (ppb) | fAcc=%lu ps/s\r\n",
           (long)p->clkB, (long)p->clkD, (unsigned long)p->fAcc);
    if (r->pubx04.valid) {
        printf("  * Reloj Propietario (PUBX,04)     : clkBias=%ld ns | clkDrift=%.2f ns/s | tpGran=%d ns | leap=%d%s\r\n",
               r->pubx04.d.clkBias, (double)r->pubx04.d.clkDrift,
               r->pubx04.d.tpGran, r->pubx04.d.leapSec,
               r->pubx04.d.leap_default ? "D (ROM)" : " (Satelite)");
    }
    if (r->tp.valid) {
        printf("  * Metrologia TIM-TP (Proximo PPS) : towMS=%lu ms | qErr=%+ld ps (error de cuantizacion) | week=%u | flags=0x%02X\r\n",
               (unsigned long)r->tp.d.towMS, (long)r->tp.d.qErr,
               r->tp.d.week, r->tp.d.flags);
    }
    if (r->vrfy.valid) {
        printf("  * Verificacion Tiempo (TIM-VRFY)  : deltaMs=%+ld ms | deltaNs=%+ld ns | wno=%u | flags=0x%02X\r\n",
               (long)r->vrfy.d.deltaMs, (long)r->vrfy.d.deltaNs,
               r->vrfy.d.wno, r->vrfy.d.flags);
    }
    if (r->tm2.valid) {
        printf("  * Marca EXTINT0 (UBX-TIM-TM2)     : ch=%u | count=%u | towMsR=%lu | accEst=%lu ns | flags=0x%02X\r\n",
               r->tm2.d.ch, r->tm2.d.count,
               (unsigned long)r->tm2.d.towMsR, (unsigned long)r->tm2.d.accEst, r->tm2.d.flags);
    }
    if (r->zda.valid && r->zda.d.date.valid) {
        printf("  * Cruce NMEA GPZDA                : %04u-%02u-%02u %02u:%02u:%05.2f UTC (LTZ=%+03d:%02d)\r\n",
               r->zda.d.date.year, r->zda.d.date.month, r->zda.d.date.day,
               r->zda.d.time.hh, r->zda.d.time.mm, (double)r->zda.d.time.ss,
               r->zda.d.ltz_h, r->zda.d.ltz_m);
    }
    if (r->ini.valid) {
        printf("  * Asistencia Inicial (UBX-AID-INI): wn=%u | tow=%lu ms | posAcc=%lu cm | tAccMs=%lu | flags=0x%08lX\r\n",
               r->ini.d.wn, (unsigned long)r->ini.d.tow,
               (unsigned long)r->ini.d.posAcc, (unsigned long)r->ini.d.tAccMs,
               (unsigned long)r->ini.d.flags);
    }
    if (pps.pulse_count > 0) {
        printf("  * Medicion Fisica PPS en GPIO 6   : %lu pulsos | Periodo=%lld us | Jitter=%+lld us (prom=%lld us, max=%lld us)\r\n",
               (unsigned long)pps.pulse_count,
               (long long)pps.last_period_us,
               (long long)pps.last_jitter_us,
               (long long)pps.avg_jitter_us,
               (long long)pps.max_jitter_us);
    } else {
        printf("  * Medicion Fisica PPS en GPIO 6   : No cableado (Operando 100%% por UART)\r\n");
    }

    /* e) Modelo Ionosferico y Salud de la Constelacion */
    printf("\r\n[E] MODELO IONOSFERICO DE KLOBUCHAR, PARAMETROS UTC Y SALUD DE CONSTELACION:\r\n");
    if (r->hui.valid) {
        const ubx_aid_hui_t *h = &r->hui.d;
        printf("  * Mascara Salud GPS (PRN 1..32)   : 0x%08lX (flags=0x%02lX: health=%u, utc=%u, klob=%u)\r\n",
               (unsigned long)h->health, (unsigned long)h->flags,
               (unsigned)(h->flags & 1u), (unsigned)((h->flags >> 1) & 1u), (unsigned)((h->flags >> 2) & 1u));
        printf("  * Klobuchar Alfa (A0..A3)         : A0=%+.4e s | A1=%+.4e s/sc | A2=%+.4e s/sc^2 | A3=%+.4e s/sc^3\r\n",
               (double)h->klobA0, (double)h->klobA1, (double)h->klobA2, (double)h->klobA3);
        printf("  * Klobuchar Beta (B0..B3)         : B0=%+.4e s | B1=%+.4e s/sc | B2=%+.4e s/sc^2 | B3=%+.4e s/sc^3\r\n",
               (double)h->klobB0, (double)h->klobB1, (double)h->klobB2, (double)h->klobB3);
        printf("  * Polinomio UTC (A0, A1, LS)      : A0=%+.4e s | A1=%+.4e s/s | TOW=%ld s | WNT=%d | LS=%d s -> LSF=%d s\r\n",
               h->utcA0, h->utcA1, (long)h->utcTOW, h->utcWNT, h->utcLS, h->utcLSF);
    } else {
        printf("  * UBX-AID-HUI                     : Esperando respuesta de sondeo...\r\n");
    }
    if (r->aopstatus.valid) {
        printf("  * AssistNow Autonomous (AOPSTATUS): useAOP=%u | status=%u (%s) | availGPS=0x%08lX (AID-AOP=0x%08lX)\r\n",
               (unsigned)(r->aopstatus.d.aopCfg & 1u),
               r->aopstatus.d.status,
               (r->aopstatus.d.status == 0) ? "Idle" : "Calculando orbitas",
               (unsigned long)r->aopstatus.d.availGPS,
               (unsigned long)r->aop_mask);
    }

    /* f) Tabla Completa por Satelite y Residuales Orbitales */
    printf("\r\n[F] TABLA COMPLETA POR SATELITE, RESIDUALES DE PSEUDORANGO Y ESTADO ORBITAL:\r\n");
    printf("  +-----+-----+------+-----+------+------+------+-----+---------+----------+---------+---------+---------+\r\n");
    printf("  | CHN | PRN | Tipo | Uso | C/N0 | Elev | Azim | QId | Lck(s)  |prRes(cm) | GRS(m)  |SBAS ic  |Age E/A  |\r\n");
    printf("  +-----+-----+------+-----+------+------+------+-----+---------+----------+---------+---------+---------+\r\n");

    if (r->svinfo.valid) {
        for (uint8_t i = 0; i < r->svinfo.n; i++) {
            const ubx_nav_svinfo_blk_t *b = &r->svinfo.b[i];
            uint8_t q = (uint8_t)(b->quality & 0x0Fu);
            if (b->cno == 0 && !(b->flags & 0x01) && b->elev < 0) {
                continue;
            }
            const char *type = (b->svid >= 120 && b->svid <= 158) ? "SBAS" : "GPS ";
            int lck = find_pubx03_lck(r, b->svid, NULL);
            float grs_m = 0.0f;
            bool has_grs = find_grs_residual(r, b->svid, &grs_m);
            int16_t sbas_prc = 0, sbas_ic = 0;
            bool has_sbas = find_sbas_corr(r, b->svid, &sbas_prc, &sbas_ic);
            uint8_t svsi_fl = 0, svsi_age = 0;
            bool has_svsi = find_svsi_info(r, b->svid, &svsi_fl, &svsi_age);

            char lck_s[16], grs_s[16], ic_s[16], age_s[16];
            if (lck >= 0) snprintf(lck_s, sizeof(lck_s), "%7d", lck);
            else          snprintf(lck_s, sizeof(lck_s), "      -");
            if (has_grs)  snprintf(grs_s, sizeof(grs_s), "%+7.2f", (double)grs_m);
            else          snprintf(grs_s, sizeof(grs_s), "      -");
            if (has_sbas) snprintf(ic_s, sizeof(ic_s), "%+5dcm", sbas_ic);
            else          snprintf(ic_s, sizeof(ic_s), "      -");
            if (has_svsi) snprintf(age_s, sizeof(age_s), "%2u/%2u", (unsigned)(svsi_age >> 4), (unsigned)(svsi_age & 0x0Fu));
            else          snprintf(age_s, sizeof(age_s), "  -/- ");

            printf("  | %3u | %3u | %s | %-3s | %4u | %4d | %4d |  %u  | %7s | %+8ld | %7s | %7s | %7s |\r\n",
                   b->chn, b->svid, type,
                   (b->flags & 0x01) ? "SI" : "NO",
                   b->cno, b->elev, b->azim, q, lck_s,
                   (long)b->prRes, grs_s, ic_s, age_s);
        }
    }
    printf("  +-----+-----+------+-----+------+------+------+-----+---------+----------+---------+---------+---------+\r\n");

    /* g) Seccion de Transparencia de Datos Descartados o Vacios por Hardware */
    printf("\r\n[G] TRANSPARENCIA DE CAMPOS DESCARTADOS O VACIOS POR DISENO DE HARDWARE (NEO-6M-0-001):\r\n");
    printf("  * Variacion Magnetica en GPRMC (campos 10 y 11) : %lu tramas vacias / %lu con dato -> EXCLUIDO (Sin magnetometro)\r\n",
           (unsigned long)r->rmc_magvar_empty, (unsigned long)r->rmc_magvar_present);
    printf("  * Rumbo Magnetico en GPVTG (campo 3)            : %lu tramas vacias / %lu con dato -> EXCLUIDO (Sin magnetometro)\r\n",
           (unsigned long)r->vtg_mag_empty, (unsigned long)r->vtg_mag_present);
    printf("  * Elipse de Error en GPGST (campos 3, 4 y 5)    : %lu tramas vacias / %lu con dato -> EXCLUIDO (ROM 7.03 solo llena stdLat/Lon/Alt)\r\n",
           (unsigned long)r->gst_ellipse_empty, (unsigned long)r->gst_ellipse_present);
    printf("  * UBX-NAV-PVT (0x01 0x07) en Protocolo 13.03    : RECHAZADO POR HARDWARE -> Sintetizado sin perdida por epoca iTOW\r\n");
    if (r->io.valid && r->io.n > 1) {
        const ubx_mon_io_port_t *u1 = &r->io.p[1];
        printf("  * Salud Electrica Puerto UART1 del GPS (MON-IO) : rx=%lu B | tx=%lu B | parityErr=%u | framingErr=%u | overrunErr=%u\r\n",
               (unsigned long)u1->rxBytes, (unsigned long)u1->txBytes,
               u1->parityErrs, u1->framingErrs, u1->overrunErrs);
    }
    if (r->msgpp.valid) {
        printf("  * Mensajes Parseados por GPS en UART1(MON-MSGPP): UBX=%u | NMEA=%u | Descartados(skipped)=%lu B\r\n",
               r->msgpp.d.msg[1][0], r->msgpp.d.msg[1][1],
               (unsigned long)r->msgpp.d.skipped[1]);
    }
    if (r->rxr.valid) {
        printf("  * Estado de Energia del Receptor (UBX-MON-RXR)  : awake=%u (%s)\r\n",
               (unsigned)(r->rxr.d.flags & 1u),
               (r->rxr.d.flags & 1u) ? "Receptor Despierto" : "Modo Ahorro/Sleep");
    }
    if (r->txt_count > 0) {
        printf("  * Mensajes de Sistema GPTXT / UBX-INF recientes : (%u registrados)\r\n", r->txt_count);
        for (uint8_t i = 0; i < r->txt_count; i++) {
            printf("      - [%s] %s\r\n", r->txt[i].src, r->txt[i].text);
        }
    }
    char net_st[192];
    gps_wifi_get_status_str(net_st, sizeof(net_st));
    printf("  * Servidor Web / WiFi 10s (Core 0, CORS *)      : %s\r\n", net_st);
    printf("****************************************************************************************************\r\n");
}

/* Lee sin bloqueo comandos por consola (UART0):
 *   ip   -> muestra IP asignada y endpoints HTTP/WS
 *   json -> imprime el ultimo JSON de 10s                                  */
static void poll_console_commands(void)
{
    static char cmd_buf[32];
    static size_t cmd_len = 0;
    int ch;
    while ((ch = fgetc(stdin)) != EOF) {
        if (ch == '\r' || ch == '\n') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                if (strcmp(cmd_buf, "ip") == 0) {
                    gps_wifi_print_network_banner();
                } else if (strcmp(cmd_buf, "json") == 0) {
                    gps_wifi_print_last_json();
                }
                cmd_len = 0;
            }
        } else if (cmd_len + 1 < sizeof(cmd_buf) && ch >= 0x20 && ch <= 0x7E) {
            cmd_buf[cmd_len++] = (char)ch;
        }
    }
}

/* =========================================================================
 * TAREA 3: gps_telemetry_console_task (Core 0, prioridad baja, stack 10240)
 * ========================================================================= */
void gps_telemetry_console_task(void *arg)
{
    (void)arg;
    uint32_t last_seen_link_event = 0;
    bool     audit_printed        = false;
    uint32_t last_panel_ms        = 0;
    uint32_t last_disc_log_ms     = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));
        poll_console_commands();

        gps_data_snapshot(&s_snap);
        uint32_t now = gps_now_ms();

        /* Si aun estamos buscando el modulo o interrogando la matriz */
        if (s_snap.sys_state == SYS_WAIT_LINK || s_snap.sys_state == SYS_DISCOVERY) {
            audit_printed = false;
            if ((now - last_disc_log_ms) >= 2000u && s_snap.status_line[0] != '\0') {
                last_disc_log_ms = now;
                printf("[DESCUBRIMIENTO] %s\r\n", s_snap.status_line);
            }
            continue;
        }

        /* En cuanto concluye el descubrimiento de un nuevo enlace, imprimir el
         * Reporte Inicial de Auditoria y Descubrimiento del Hardware una vez */
        if (!audit_printed || s_snap.link_events != last_seen_link_event) {
            gps_console_print_audit_report(&s_snap);
            audit_printed        = true;
            last_seen_link_event = s_snap.link_events;
            last_panel_ms        = now;
            continue;
        }

        /* En operacion continua: alternar entre Panel de Calentamiento (cada 2 s)
         * y Panel Maestro de Datos Confiables Verificados (cada 3 s en Fase E) */
        if (s_snap.phase == PH_E_TRUSTED) {
            if ((now - last_panel_ms) >= CONSOLE_PERIOD_MASTER_MS) {
                last_panel_ms = now;
                gps_console_print_master_panel(&s_snap);
            }
        } else {
            if ((now - last_panel_ms) >= CONSOLE_PERIOD_WARMUP_MS) {
                last_panel_ms = now;
                gps_console_print_warmup_panel(&s_snap);
            }
        }
    }
}
