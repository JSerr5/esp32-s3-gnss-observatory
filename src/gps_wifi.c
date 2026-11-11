/*
 * gps_wifi.c
 * ---------------------------------------------------------------------------
 * Implementacion ligera en el Core 0 de:
 *   1) Pila WiFi en modo exclusivamente estacion (WIFI_MODE_STA, WPA3/WPA2)
 *      con credenciales fijas definidas en gps_config.h (sin Access Point).
 *   2) Respondedor mDNS UDP (224.0.0.251:5353) para resolver "esp32-gps.local".
 *   3) Serializador JSON determinista cada 10,000 ms (10 s) protegido por
 *      SemaphoreHandle_t (Mutex) con todas las secciones [A]..[G].
 *   4) Servidor HTTP con cabeceras CORS completas (Access-Control-Allow-Origin: *):
 *        - GET     /api/gps  -> Ultimo snapshot JSON (actualizado cada 10 s)
 *        - OPTIONS /api/gps  -> Preflight CORS
 *        - GET     /events   -> Stream Server-Sent Events (push cada 10 s)
 *        - GET     /ws       -> WebSocket RFC 6455 (push cada 10 s)
 * ---------------------------------------------------------------------------
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
#include "gps_config.h"
#include "gps_data.h"
#include "gps_hw.h"
#include "gps_wifi.h"

#define JSON_BUF_CAPACITY    6144
#define MAX_STREAM_CLIENTS   4

/* Mutex y buffer del snapshot JSON actualizado cada 10 segundos */
static SemaphoreHandle_t s_json_mtx = NULL;
static StaticSemaphore_t s_json_mtx_buf;
static char              s_json_buf[JSON_BUF_CAPACITY];
static size_t            s_json_len = 0;
static uint32_t          s_json_seq = 0;
static uint32_t          s_json_updated_ms = 0;
static uint32_t          s_last_json_us = 0;

/* Instantanea privada en BSS para serializar en Core 0 sin gastar pila */
static gps_repo_t        s_wifi_snap;

/* Estado de red STA */
static esp_netif_t      *s_netif_sta     = NULL;
static httpd_handle_t    s_httpd         = NULL;
static bool              s_sta_connected = false;
static char              s_sta_ip[20]    = "0.0.0.0";
static uint32_t          s_sta_ip_be     = 0; /* en orden de red para mDNS */

/* Clientes conectados por WebSocket (/ws) y Server-Sent Events (/events) */
static int               s_ws_fds[MAX_STREAM_CLIENTS];
static int               s_sse_fds[MAX_STREAM_CLIENTS];

/* =========================================================================
 * SHA-1 (RFC 3174) + Base64 autocontenidos para el handshake WebSocket
 * RFC 6455 (cero dependencias de versiones de MbedTLS).
 * ========================================================================= */
static uint32_t rol32(uint32_t v, unsigned int b)
{
    return (v << b) | (v >> (32u - b));
}

static void sha1_compute(const uint8_t *msg, size_t len, uint8_t digest[20])
{
    uint32_t h0 = 0x67452301u;
    uint32_t h1 = 0xEFCDAB89u;
    uint32_t h2 = 0x98BADCFEu;
    uint32_t h3 = 0x10325476u;
    uint32_t h4 = 0xC3D2E1F0u;

    uint8_t block[128];
    size_t rem = len;
    size_t off = 0;

    while (rem >= 64) {
        uint32_t w[80];
        for (int i = 0; i < 16; i++) {
            const uint8_t *p = msg + off + (size_t)(i * 4);
            w[i] = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                   ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
        }
        for (int i = 16; i < 80; i++) {
            w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        }
        uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
        for (int i = 0; i < 80; i++) {
            uint32_t f, k;
            if (i < 20)      { f = (b & c) | ((~b) & d); k = 0x5A827999u; }
            else if (i < 40) { f = b ^ c ^ d;            k = 0x6ED9EBA1u; }
            else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDCu; }
            else             { f = b ^ c ^ d;            k = 0xCA62C1D6u; }
            uint32_t tmp = rol32(a, 5) + f + e + k + w[i];
            e = d; d = c; c = rol32(b, 30); b = a; a = tmp;
        }
        h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
        off += 64;
        rem -= 64;
    }

    memset(block, 0, sizeof(block));
    if (rem > 0) {
        memcpy(block, msg + off, rem);
    }
    block[rem] = 0x80u;
    size_t total_blocks = (rem >= 56) ? 2u : 1u;
    uint64_t bit_len = (uint64_t)len * 8ULL;
    size_t len_pos = total_blocks * 64u - 8u;
    for (int i = 0; i < 8; i++) {
        block[len_pos + (size_t)i] = (uint8_t)(bit_len >> (56 - i * 8));
    }

    for (size_t blk = 0; blk < total_blocks; blk++) {
        uint32_t w[80];
        for (int i = 0; i < 16; i++) {
            const uint8_t *p = block + blk * 64u + (size_t)(i * 4);
            w[i] = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                   ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
        }
        for (int i = 16; i < 80; i++) {
            w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        }
        uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
        for (int i = 0; i < 80; i++) {
            uint32_t f, k;
            if (i < 20)      { f = (b & c) | ((~b) & d); k = 0x5A827999u; }
            else if (i < 40) { f = b ^ c ^ d;            k = 0x6ED9EBA1u; }
            else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDCu; }
            else             { f = b ^ c ^ d;            k = 0xCA62C1D6u; }
            uint32_t tmp = rol32(a, 5) + f + e + k + w[i];
            e = d; d = c; c = rol32(b, 30); b = a; a = tmp;
        }
        h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
    }

    uint32_t hs[5] = { h0, h1, h2, h3, h4 };
    for (int i = 0; i < 5; i++) {
        digest[i * 4 + 0] = (uint8_t)(hs[i] >> 24);
        digest[i * 4 + 1] = (uint8_t)(hs[i] >> 16);
        digest[i * 4 + 2] = (uint8_t)(hs[i] >> 8);
        digest[i * 4 + 3] = (uint8_t)(hs[i]);
    }
}

static void base64_encode_20(const uint8_t in[20], char out[29])
{
    static const char tbl[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int j = 0;
    for (int i = 0; i < 18; i += 3) {
        uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i + 1] << 8) | in[i + 2];
        out[j++] = tbl[(v >> 18) & 0x3Fu];
        out[j++] = tbl[(v >> 12) & 0x3Fu];
        out[j++] = tbl[(v >> 6)  & 0x3Fu];
        out[j++] = tbl[v & 0x3Fu];
    }
    uint32_t v = ((uint32_t)in[18] << 16) | ((uint32_t)in[19] << 8);
    out[j++] = tbl[(v >> 18) & 0x3Fu];
    out[j++] = tbl[(v >> 12) & 0x3Fu];
    out[j++] = tbl[(v >> 6)  & 0x3Fu];
    out[j++] = '=';
    out[j]   = '\0';
}

/* =========================================================================
 * Helpers para cruzar datos de satelites al serializar la tabla [F]
 * ========================================================================= */
static int wifi_find_pubx03_lck(const gps_repo_t *r, uint8_t svid)
{
    if (!r->pubx03.valid) return 0;
    for (int i = 0; i < r->pubx03.d.nblk; i++) {
        if (r->pubx03.d.sat[i].sv == (int)svid) {
            return r->pubx03.d.sat[i].lck;
        }
    }
    return 0;
}

static bool wifi_find_grs(const gps_repo_t *r, uint8_t svid, float *res_m)
{
    if (!r->gsa.valid || !r->grs.valid) return false;
    for (int i = 0; i < r->gsa.d.nprn && i < 12; i++) {
        if (r->gsa.d.prn[i] == (int)svid && r->grs.d.res_valid[i]) {
            *res_m = r->grs.d.res[i];
            return true;
        }
    }
    return false;
}

static void wifi_find_svsi_age(const gps_repo_t *r, uint8_t svid, int *age_e, int *age_a)
{
    *age_e = 15;
    *age_a = 15;
    if (!r->svsi.valid) return;
    for (uint8_t i = 0; i < r->svsi.n; i++) {
        if (r->svsi.b[i].svid == svid) {
            *age_e = (int)(r->svsi.b[i].age >> 4);
            *age_a = (int)(r->svsi.b[i].age & 0x0Fu);
            return;
        }
    }
}

static void wifi_local_time(uint16_t y, uint8_t mo, uint8_t d,
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

static const char *wifi_ant_status_short(const gps_repo_t *r)
{
    if (r->hw.valid) {
        switch (r->hw.d.aStatus) {
        case 0: return "INIT";
        case 1: return "DONTKNOW";
        case 2: return "OK";
        case 3: return "SHORT";
        case 4: return "OPEN";
        default: break;
        }
    }
    return r->antstatus[0] ? r->antstatus : "OK";
}

/* =========================================================================
 * Serializador JSON Completo (Secciones [A] a [G])
 * Se ejecuta exclusivamente cada 10,000 ms en el Core 0.
 * ========================================================================= */
#define JAPPEND(...)                                                          \
    do {                                                                      \
        if (pos < cap) {                                                      \
            int _n = snprintf(buf + pos, cap - pos, __VA_ARGS__);             \
            if (_n > 0) {                                                     \
                pos += ((size_t)_n < (cap - pos)) ? (size_t)_n : (cap - pos - 1); \
            }                                                                 \
        }                                                                     \
    } while (0)

static size_t serialize_repo_to_json(const gps_repo_t *r, char *buf, size_t cap, uint32_t seq)
{
    size_t pos = 0;
    const gps_pvt_t *p = &r->pvt;
    gps_pps_stat_t pps;
    gps_hw_get_pps(&pps);

    /* [A] Posicion */
    double lat       = (double)p->lat * 1e-7;
    double lon       = (double)p->lon * 1e-7;
    double hMSL      = (double)p->hMSL / 1000.0;
    double height    = (double)p->height / 1000.0;
    double geoidSep  = height - hMSL;
    double ecefX     = (double)p->ecefX / 100.0;
    double ecefY     = (double)p->ecefY / 100.0;
    double ecefZ     = (double)p->ecefZ / 100.0;
    double hAcc      = (double)p->hAcc / 1000.0;
    double vAcc      = (double)p->vAcc / 1000.0;
    double pAcc      = (double)p->pAcc / 100.0;
    double stdLat    = r->gst.valid ? (double)r->gst.d.stdLat   : (hAcc * 0.65);
    double stdLon    = r->gst.valid ? (double)r->gst.d.stdLon   : (hAcc * 0.75);
    double stdAlt    = r->gst.valid ? (double)r->gst.d.stdAlt   : vAcc;
    double rangeRms  = r->gst.valid ? (double)r->gst.d.rangeRms : 0.0;

    /* [B] Cinematica */
    double gSpeed    = (double)p->gSpeed / 100.0;
    double speed3D   = (double)p->speed / 100.0;
    double sAcc      = (double)p->sAcc / 100.0;
    double velN      = (double)p->velN / 100.0;
    double velE      = (double)p->velE / 100.0;
    double velD      = (double)p->velD / 100.0;
    double climbRate = -velD;
    double ecefVX    = (double)p->ecefVX / 100.0;
    double ecefVY    = (double)p->ecefVY / 100.0;
    double ecefVZ    = (double)p->ecefVZ / 100.0;
    double heading   = r->heading_static
                       ? (r->heading_have ? ((double)r->heading_last_valid * 1e-5) : 0.0)
                       : ((double)p->heading * 1e-5);
    double cAcc      = (double)p->cAcc * 1e-5;

    /* [C] Geometria e Integridad */
    int    fixType      = (int)p->gpsFix_sol;
    uint32_t ttff       = r->ttff_ms;
    int    numSVUsed    = (int)p->numSV;
    int    numSVView    = (int)r->sv_vis;
    int    numSVEph     = (int)r->sv_eph;
    double gdop         = (double)p->gDOP / 100.0;
    double pdop         = (double)((p->pDOP > 0) ? p->pDOP : p->pDOP_sol) / 100.0;
    double tdop         = (double)p->tDOP / 100.0;
    double vdop         = (double)p->vDOP / 100.0;
    double hdop         = (double)p->hDOP / 100.0;
    double ndop         = (double)p->nDOP / 100.0;
    double edop         = (double)p->eDOP / 100.0;
    double raimErrLat   = r->gbs.valid ? (double)r->gbs.d.errLat : stdLat;
    double raimErrLon   = r->gbs.valid ? (double)r->gbs.d.errLon : stdLon;
    double raimErrAlt   = r->gbs.valid ? (double)r->gbs.d.errAlt : stdAlt;
    int    raimFailedSV = (r->gbs.valid && r->gbs.d.svid_valid) ? r->gbs.d.svid : 0;

    /* [D] Tiempo y Reloj */
    uint16_t yr = (p->year >= 2020) ? p->year : 2026;
    uint8_t  mo = (p->month >= 1 && p->month <= 12) ? p->month : 11;
    uint8_t  dy = (p->day >= 1 && p->day <= 31) ? p->day : 9;
    uint8_t  hr = p->hour;
    uint8_t  mi = p->min;
    uint8_t  sc = p->sec;

    uint16_t ly; uint8_t lmo, ld, lh, lmi;
    wifi_local_time(yr, mo, dy, hr, mi, GPS_LOCAL_UTC_OFFSET_MIN,
                    &ly, &lmo, &ld, &lh, &lmi);

    char utcIso[32], localIso[32];
    snprintf(utcIso, sizeof(utcIso), "%04u-%02u-%02uT%02u:%02u:%02uZ",
             yr, mo, dy, hr, mi, sc);
    snprintf(localIso, sizeof(localIso), "%04u-%02u-%02uT%02u:%02u:%02u",
             ly, lmo, ld, lh, lmi, sc);

    int32_t qErrPs = r->tp.valid ? r->tp.d.qErr : 0;
    int     tpGran = r->pubx04.valid ? r->pubx04.d.tpGran : 21;

    /* [E] Ionosfera */
    uint32_t hmask = r->hui.valid ? r->hui.d.health : 0xFFFFFFFFu;
    double ka0 = r->hui.valid ? (double)r->hui.d.klobA0 : 0.0;
    double ka1 = r->hui.valid ? (double)r->hui.d.klobA1 : 0.0;
    double ka2 = r->hui.valid ? (double)r->hui.d.klobA2 : 0.0;
    double ka3 = r->hui.valid ? (double)r->hui.d.klobA3 : 0.0;
    double kb0 = r->hui.valid ? (double)r->hui.d.klobB0 : 0.0;
    double kb1 = r->hui.valid ? (double)r->hui.d.klobB1 : 0.0;
    double kb2 = r->hui.valid ? (double)r->hui.d.klobB2 : 0.0;
    double kb3 = r->hui.valid ? (double)r->hui.d.klobB3 : 0.0;

    /* [G] Salud UART/Hardware */
    uint32_t mon_rx = (r->io.valid && r->io.n > 1) ? r->io.p[1].rxBytes : r->bytes_tx;
    uint32_t mon_tx = (r->io.valid && r->io.n > 1) ? r->io.p[1].txBytes : r->bytes_rx;
    uint32_t uart_errs = r->uart_fifo_ovf + r->uart_buf_full +
                         r->uart_frame_err + r->uart_parity_err;
    if (r->io.valid && r->io.n > 1) {
        uart_errs += (uint32_t)r->io.p[1].parityErrs +
                     (uint32_t)r->io.p[1].framingErrs +
                     (uint32_t)r->io.p[1].overrunErrs;
    }
    const char *antStatus = wifi_ant_status_short(r);
    double agcPct = r->hw.valid ? ((double)r->hw.d.agcCnt / 81.91) : 0.0;
    double jamPct = r->hw.valid ? ((double)r->hw.d.jamInd / 2.55)  : 0.0;

    double epochUbxMs   = (double)r->epoch_ubx_build_us / 1000.0;
    double epochBurstMs = (double)r->epoch_burst_total_us / 1000.0;

    JAPPEND("{");
    JAPPEND("\"seq\":%lu,\"uptimeMs\":%lu,\"intervalMs\":%u,",
            (unsigned long)seq, (unsigned long)gps_now_ms(),
            (unsigned)(r->cfg_rate.valid && r->cfg_rate.d.measRate > 0 ? r->cfg_rate.d.measRate : 1000u));
    JAPPEND("\"epochUbxMs\":%.2f,\"epochBurstMs\":%.2f,\"epochFrames\":%lu,\"epochBytes\":%lu,\"jsonBuildUs\":%lu,",
            epochUbxMs, epochBurstMs,
            (unsigned long)r->epoch_frames_cnt, (unsigned long)r->epoch_bytes_cnt,
            (unsigned long)s_last_json_us);
    JAPPEND("\"hostname\":\"%s.local\",\"ip\":\"%s\",",
            GPS_MDNS_HOSTNAME, s_sta_ip);
    JAPPEND("\"phase\":%d,\"phaseName\":\"%s\",\"trusted\":%s,",
            (int)r->phase, gps_phase_name(r->phase),
            (r->phase == PH_E_TRUSTED) ? "true" : "false");
    JAPPEND("\"certifiedEpochs\":%lu,\"streak\":%u,",
            (unsigned long)r->pvt_epochs, (unsigned)r->good_epochs);

    /* Campos planos en raiz [A]..[E],[G] */
    JAPPEND("\"lat\":%.7f,\"lon\":%.7f,\"hMSL\":%.3f,\"height\":%.3f,\"geoidSep\":%.3f,",
            lat, lon, hMSL, height, geoidSep);
    JAPPEND("\"ecefX\":%.2f,\"ecefY\":%.2f,\"ecefZ\":%.2f,",
            ecefX, ecefY, ecefZ);
    JAPPEND("\"hAcc\":%.3f,\"vAcc\":%.3f,\"pAcc\":%.3f,",
            hAcc, vAcc, pAcc);
    JAPPEND("\"stdLat\":%.2f,\"stdLon\":%.2f,\"stdAlt\":%.2f,\"rangeRms\":%.2f,",
            stdLat, stdLon, stdAlt, rangeRms);

    JAPPEND("\"gSpeed\":%.3f,\"speed3D\":%.3f,\"sAcc\":%.3f,",
            gSpeed, speed3D, sAcc);
    JAPPEND("\"velN\":%.3f,\"velE\":%.3f,\"velD\":%.3f,\"climbRate\":%.3f,",
            velN, velE, velD, climbRate);
    JAPPEND("\"ecefVX\":%.3f,\"ecefVY\":%.3f,\"ecefVZ\":%.3f,",
            ecefVX, ecefVY, ecefVZ);
    JAPPEND("\"heading\":%.2f,\"cAcc\":%.2f,\"headingStatic\":%s,",
            heading, cAcc, r->heading_static ? "true" : "false");

    JAPPEND("\"fixType\":%d,\"fixName\":\"%s\",\"ttff\":%lu,",
            fixType, gps_fix_name((uint8_t)fixType), (unsigned long)ttff);
    JAPPEND("\"numSVUsed\":%d,\"numSVView\":%d,\"numSVEph\":%d,",
            numSVUsed, numSVView, numSVEph);
    JAPPEND("\"gdop\":%.2f,\"pdop\":%.2f,\"tdop\":%.2f,\"vdop\":%.2f,\"hdop\":%.2f,\"ndop\":%.2f,\"edop\":%.2f,",
            gdop, pdop, tdop, vdop, hdop, ndop, edop);
    JAPPEND("\"raimErrLat\":%.2f,\"raimErrLon\":%.2f,\"raimErrAlt\":%.2f,\"raimFailedSV\":%d,",
            raimErrLat, raimErrLon, raimErrAlt, raimFailedSV);

    JAPPEND("\"utcIso\":\"%s\",\"localIso\":\"%s\",", utcIso, localIso);
    JAPPEND("\"gpsWeek\":%d,\"iTOW\":%lu,\"fTOW\":%ld,\"leapS\":%d,",
            (int)p->week, (unsigned long)p->iTOW, (long)p->fTOW, (int)p->leapS);
    JAPPEND("\"tAccNs\":%lu,\"clkBiasNs\":%ld,\"clkDriftPpb\":%ld,\"fAccPsS\":%lu,\"tpGranNs\":%d,\"qErrPs\":%ld,",
            (unsigned long)p->utc_tAcc, (long)p->clkB, (long)p->clkD,
            (unsigned long)p->fAcc, tpGran, (long)qErrPs);

    JAPPEND("\"healthMask\":\"0x%08lX\",\"klobValid\":%s,",
            (unsigned long)hmask, r->klob_valid ? "true" : "false");
    JAPPEND("\"klobAlpha\":[%.4e,%.4e,%.4e,%.4e],", ka0, ka1, ka2, ka3);
    JAPPEND("\"klobBeta\":[%.4e,%.4e,%.4e,%.4e],", kb0, kb1, kb2, kb3);

    JAPPEND("\"rxBytes\":%lu,\"txBytes\":%lu,\"uartErrors\":%lu,\"antStatus\":\"%s\",",
            (unsigned long)mon_rx, (unsigned long)mon_tx,
            (unsigned long)uart_errs, antStatus);
    JAPPEND("\"noisePerMS\":%u,\"agcPct\":%.1f,\"jamPct\":%.1f,",
            r->hw.valid ? r->hw.d.noisePerMS : 0u, agcPct, jamPct);

    /* [F] Lista de satelites */
    JAPPEND("\"satellites\":[");
    int sat_emitted = 0;
    if (r->svinfo.valid) {
        for (uint8_t i = 0; i < r->svinfo.n; i++) {
            const ubx_nav_svinfo_blk_t *b = &r->svinfo.b[i];
            uint8_t q = (uint8_t)(b->quality & 0x0Fu);
            if (b->cno == 0 && !(b->flags & 0x01) && b->elev < 0) {
                continue;
            }
            const char *stype = (b->svid >= 120 && b->svid <= 158) ? "SBAS" : "GPS";
            bool used = (b->flags & 0x01) != 0;
            int lck = wifi_find_pubx03_lck(r, b->svid);
            float grs_m = 0.0f;
            bool has_grs = wifi_find_grs(r, b->svid, &grs_m);
            int age_e = 15, age_a = 15;
            wifi_find_svsi_age(r, b->svid, &age_e, &age_a);

            if (sat_emitted > 0) JAPPEND(",");
            JAPPEND("{\"chn\":%u,\"prn\":%u,\"type\":\"%s\",\"used\":%s,"
                    "\"cno\":%u,\"elev\":%d,\"azim\":%d,\"qid\":%u,\"lck\":%d,"
                    "\"prResCm\":%ld,\"grsM\":",
                    b->chn, b->svid, stype, used ? "true" : "false",
                    b->cno, b->elev, b->azim, q, lck, (long)b->prRes);
            if (has_grs) {
                JAPPEND("%.2f", (double)grs_m);
            } else {
                JAPPEND("null");
            }
            JAPPEND(",\"ageEph\":%d,\"ageAlm\":%d}", age_e, age_a);
            sat_emitted++;
        }
    }
    JAPPEND("]");
    JAPPEND("}");
    return pos;
}
#undef JAPPEND

/* =========================================================================
 * Gestion de clientes de streaming (WebSocket /ws y SSE /events)
 * ========================================================================= */
static void stream_lists_init(void)
{
    for (int i = 0; i < MAX_STREAM_CLIENTS; i++) {
        s_ws_fds[i]  = -1;
        s_sse_fds[i] = -1;
    }
}

static void stream_add_fd(int *list, int fd)
{
    for (int i = 0; i < MAX_STREAM_CLIENTS; i++) {
        if (list[i] == fd) return;
    }
    for (int i = 0; i < MAX_STREAM_CLIENTS; i++) {
        if (list[i] < 0) {
            list[i] = fd;
            return;
        }
    }
    /* Si esta lleno, reemplaza el slot 0 */
    if (s_httpd && list[0] >= 0) {
        httpd_sess_trigger_close(s_httpd, list[0]);
    }
    list[0] = fd;
}

static int ws_send_text_frame(int fd, const char *payload, size_t len)
{
    if (s_httpd == NULL || fd < 0) return -1;
    uint8_t hdr[10];
    size_t hdr_len = 0;
    hdr[0] = 0x81u; /* FIN=1, opcode=1 (text) */
    if (len < 126) {
        hdr[1] = (uint8_t)len;
        hdr_len = 2;
    } else if (len <= 65535) {
        hdr[1] = 126u;
        hdr[2] = (uint8_t)((len >> 8) & 0xFFu);
        hdr[3] = (uint8_t)(len & 0xFFu);
        hdr_len = 4;
    } else {
        return -1;
    }
    if (httpd_socket_send(s_httpd, fd, (const char *)hdr, hdr_len, 0) < 0) {
        return -1;
    }
    if (httpd_socket_send(s_httpd, fd, payload, len, 0) < 0) {
        return -1;
    }
    return 0;
}

static int sse_send_event(int fd, const char *payload, size_t len)
{
    if (s_httpd == NULL || fd < 0) return -1;
    if (httpd_socket_send(s_httpd, fd, "data: ", 6, 0) < 0) return -1;
    if (httpd_socket_send(s_httpd, fd, payload, len, 0) < 0) return -1;
    if (httpd_socket_send(s_httpd, fd, "\n\n", 2, 0) < 0) return -1;
    return 0;
}

static void broadcast_json_to_streams(const char *json, size_t len)
{
    for (int i = 0; i < MAX_STREAM_CLIENTS; i++) {
        if (s_ws_fds[i] >= 0) {
            if (ws_send_text_frame(s_ws_fds[i], json, len) < 0) {
                httpd_sess_trigger_close(s_httpd, s_ws_fds[i]);
                s_ws_fds[i] = -1;
            }
        }
        if (s_sse_fds[i] >= 0) {
            if (sse_send_event(s_sse_fds[i], json, len) < 0) {
                httpd_sess_trigger_close(s_httpd, s_sse_fds[i]);
                s_sse_fds[i] = -1;
            }
        }
    }
}

/* =========================================================================
 * Handlers HTTP con CORS obligatorio (Access-Control-Allow-Origin: *)
 * ========================================================================= */
static void set_cors_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Accept, Cache-Control");
    httpd_resp_set_hdr(req, "Access-Control-Max-Age", "86400");
}

static esp_err_t handle_options_cors(httpd_req_t *req)
{
    set_cors_headers(req);
    httpd_resp_set_status(req, "200 OK");
    return httpd_resp_send(req, NULL, 0);
}

/* GET /api/gps -> Devuelve el snapshot JSON de 10 segundos */
static esp_err_t handle_get_api_gps(httpd_req_t *req)
{
    set_cors_headers(req);
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");

    xSemaphoreTake(s_json_mtx, portMAX_DELAY);
    esp_err_t err = httpd_resp_send(req, s_json_buf, (ssize_t)s_json_len);
    xSemaphoreGive(s_json_mtx);
    return err;
}

/* GET /events -> Server-Sent Events (SSE) con push cada 10 s */
static esp_err_t handle_get_sse(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);
    if (fd < 0) return ESP_FAIL;

    const char *hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/event-stream; charset=utf-8\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Access-Control-Allow-Methods: GET, OPTIONS\r\n"
        "Access-Control-Allow-Headers: Content-Type\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: keep-alive\r\n\r\n";

    if (httpd_socket_send(s_httpd, fd, hdr, strlen(hdr), 0) < 0) {
        return ESP_FAIL;
    }

    xSemaphoreTake(s_json_mtx, portMAX_DELAY);
    int rc = sse_send_event(fd, s_json_buf, s_json_len);
    xSemaphoreGive(s_json_mtx);

    if (rc == 0) {
        stream_add_fd(s_sse_fds, fd);
    }
    return ESP_OK;
}

/* GET /ws -> Upgrade WebSocket RFC 6455 con push cada 10 s */
static esp_err_t handle_get_ws(httpd_req_t *req)
{
    char ws_key[64];
    if (httpd_req_get_hdr_value_str(req, "Sec-WebSocket-Key", ws_key, sizeof(ws_key)) != ESP_OK) {
        /* Si se abre /ws con un GET normal sin upgrade, devolver el JSON */
        return handle_get_api_gps(req);
    }

    char concat[128];
    snprintf(concat, sizeof(concat), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", ws_key);
    uint8_t sha1_out[20];
    char    accept_b64[29];
    sha1_compute((const uint8_t *)concat, strlen(concat), sha1_out);
    base64_encode_20(sha1_out, accept_b64);

    int fd = httpd_req_to_sockfd(req);
    if (fd < 0) return ESP_FAIL;

    char resp[256];
    int n = snprintf(resp, sizeof(resp),
                     "HTTP/1.1 101 Switching Protocols\r\n"
                     "Upgrade: websocket\r\n"
                     "Connection: Upgrade\r\n"
                     "Sec-WebSocket-Accept: %s\r\n"
                     "Access-Control-Allow-Origin: *\r\n\r\n",
                     accept_b64);
    if (httpd_socket_send(s_httpd, fd, resp, (size_t)n, 0) < 0) {
        return ESP_FAIL;
    }

    xSemaphoreTake(s_json_mtx, portMAX_DELAY);
    int rc = ws_send_text_frame(fd, s_json_buf, s_json_len);
    xSemaphoreGive(s_json_mtx);

    if (rc == 0) {
        stream_add_fd(s_ws_fds, fd);
    }
    return ESP_OK;
}

static void start_http_server(void)
{
    if (s_httpd != NULL) return;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = GPS_HTTP_PORT;
    cfg.core_id          = TASK_WIFI_CORE; /* Core 0: aislado de UART1 (Core 1) */
    cfg.max_open_sockets = 5;
    cfg.max_uri_handlers = 6;
    cfg.lru_purge_enable = true;

    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        return;
    }

    const httpd_uri_t u_gps_get = {
        .uri = "/api/gps", .method = HTTP_GET, .handler = handle_get_api_gps
    };
    const httpd_uri_t u_gps_opt = {
        .uri = "/api/gps", .method = HTTP_OPTIONS, .handler = handle_options_cors
    };
    const httpd_uri_t u_sse_get = {
        .uri = "/events", .method = HTTP_GET, .handler = handle_get_sse
    };
    const httpd_uri_t u_sse_opt = {
        .uri = "/events", .method = HTTP_OPTIONS, .handler = handle_options_cors
    };
    const httpd_uri_t u_ws_get = {
        .uri = "/ws", .method = HTTP_GET, .handler = handle_get_ws
    };
    const httpd_uri_t u_ws_opt = {
        .uri = "/ws", .method = HTTP_OPTIONS, .handler = handle_options_cors
    };

    httpd_register_uri_handler(s_httpd, &u_gps_get);
    httpd_register_uri_handler(s_httpd, &u_gps_opt);
    httpd_register_uri_handler(s_httpd, &u_sse_get);
    httpd_register_uri_handler(s_httpd, &u_sse_opt);
    httpd_register_uri_handler(s_httpd, &u_ws_get);
    httpd_register_uri_handler(s_httpd, &u_ws_opt);
}

/* =========================================================================
 * Respondedor mDNS ligero RFC 6762 para "esp32-gps.local" (UDP 224.0.0.251:5353)
 * ========================================================================= */
static int mdns_open_socket(void)
{
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) return -1;

    int reuse = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct timeval tv = { .tv_sec = 0, .tv_usec = 15000 }; /* 15 ms para no bloquear tiempo real */
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(5353);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(s);
        return -1;
    }

    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr("224.0.0.251");
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
    return s;
}

static void mdns_poll_and_reply(int sock)
{
    if (sock < 0) return;
    uint8_t pkt[256];
    struct sockaddr_in from;
    socklen_t flen = sizeof(from);
    int n = recvfrom(sock, pkt, sizeof(pkt), 0, (struct sockaddr *)&from, &flen);
    if (n < 18) return;

    /* Solo procesar consultas (QR bit 7 de flags == 0) */
    if ((pkt[2] & 0x80u) != 0) return;

    /* Buscar la etiqueta "\x09esp32-gps\x05local\x00" (case-insensitive) */
    const char *host = GPS_MDNS_HOSTNAME;
    size_t hlen = strlen(host);
    bool match = false;
    for (int i = 12; i + (int)hlen + 7 < n; i++) {
        if (pkt[i] == (uint8_t)hlen &&
            strncasecmp((const char *)&pkt[i + 1], host, hlen) == 0 &&
            pkt[i + 1 + (int)hlen] == 5 &&
            strncasecmp((const char *)&pkt[i + 2 + (int)hlen], "local", 5) == 0) {
            match = true;
            break;
        }
    }
    if (!match || !s_sta_connected || s_sta_ip_be == 0) return;
    uint32_t ip_be = s_sta_ip_be;

    uint8_t resp[128];
    size_t p = 0;
    resp[p++] = pkt[0]; resp[p++] = pkt[1]; /* ID */
    resp[p++] = 0x84;   resp[p++] = 0x00;   /* QR=1, AA=1 */
    resp[p++] = 0x00;   resp[p++] = 0x00;   /* QDCOUNT=0 */
    resp[p++] = 0x00;   resp[p++] = 0x01;   /* ANCOUNT=1 */
    resp[p++] = 0x00;   resp[p++] = 0x00;   /* NSCOUNT=0 */
    resp[p++] = 0x00;   resp[p++] = 0x00;   /* ARCOUNT=0 */
    resp[p++] = (uint8_t)hlen;
    memcpy(&resp[p], host, hlen); p += hlen;
    resp[p++] = 5;
    memcpy(&resp[p], "local", 5); p += 5;
    resp[p++] = 0x00;
    resp[p++] = 0x00; resp[p++] = 0x01;     /* TYPE A */
    resp[p++] = 0x80; resp[p++] = 0x01;     /* CLASS IN + cache-flush */
    resp[p++] = 0x00; resp[p++] = 0x00; resp[p++] = 0x00; resp[p++] = 0x78; /* TTL 120s */
    resp[p++] = 0x00; resp[p++] = 0x04;     /* RDLENGTH 4 */
    memcpy(&resp[p], &ip_be, 4); p += 4;

    struct sockaddr_in mcast;
    memset(&mcast, 0, sizeof(mcast));
    mcast.sin_family      = AF_INET;
    mcast.sin_port        = htons(5353);
    mcast.sin_addr.s_addr = inet_addr("224.0.0.251");
    sendto(sock, resp, p, 0, (struct sockaddr *)&mcast, sizeof(mcast));
    sendto(sock, resp, p, 0, (struct sockaddr *)&from, flen);
}

/* =========================================================================
 * Eventos WiFi STA / IP (Modo Estacion puro con soporte WPA3-Personal)
 * ========================================================================= */
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_sta_connected = false;
        snprintf(s_sta_ip, sizeof(s_sta_ip), "0.0.0.0");
        s_sta_ip_be = 0;
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(s_sta_ip, sizeof(s_sta_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        s_sta_ip_be     = ev->ip_info.ip.addr;
        s_sta_connected = true;
        printf("\r\n====================================================================================================\r\n");
        printf("  [WIFI STA CONECTADO] Red: '%s' | IP asignada: %s\r\n", GPS_WIFI_STA_SSID, s_sta_ip);
        printf("  * REST JSON (CORS *) : http://%s/api/gps   | http://%s.local/api/gps\r\n", s_sta_ip, GPS_MDNS_HOSTNAME);
        printf("  * WebSocket 10s      : ws://%s/ws          | ws://%s.local/ws\r\n", s_sta_ip, GPS_MDNS_HOSTNAME);
        printf("  * SSE Stream 10s     : http://%s/events    | http://%s.local/events\r\n", s_sta_ip, GPS_MDNS_HOSTNAME);
        printf("====================================================================================================\r\n\r\n");
    }
}

void gps_wifi_get_status_str(char *out, size_t cap)
{
    if (s_sta_connected) {
        snprintf(out, cap, "CONECTADO a '%s' | IP=%s | http://%s/api/gps | http://%s.local/api/gps",
                 GPS_WIFI_STA_SSID, s_sta_ip, s_sta_ip, GPS_MDNS_HOSTNAME);
    } else {
        snprintf(out, cap, "Conectando a '%s' (WPA3/WPA2)...", GPS_WIFI_STA_SSID);
    }
}

void gps_wifi_print_network_banner(void)
{
    char st[192];
    gps_wifi_get_status_str(st, sizeof(st));
    printf("\r\n[RED WIFI STA / SERVIDOR TELEMETRIA 10s EN CORE 0]\r\n");
    printf("  * Estado WiFi        : %s\r\n", st);
    printf("  * Hostname mDNS      : http://%s.local/api/gps  |  ws://%s.local/ws\r\n\r\n",
           GPS_MDNS_HOSTNAME, GPS_MDNS_HOSTNAME);
}

void gps_wifi_print_last_json(void)
{
    if (s_json_mtx == NULL) return;
    xSemaphoreTake(s_json_mtx, portMAX_DELAY);
    printf("\r\n[SNAPSHOT JSON #%lu (%u bytes)]:\r\n%s\r\n\r\n",
            (unsigned long)s_json_seq, (unsigned)s_json_len, s_json_buf);
    xSemaphoreGive(s_json_mtx);
}

/* =========================================================================
 * Tarea FreeRTOS en Core 0: Publica el JSON inmediatamente cada vez que se
 * completa una trama/epoca en el ESP32-S3 (EVT_EPOCH_READY) y atiende mDNS.
 * ========================================================================= */
static void gps_wifi_telemetry_task(void *arg)
{
    (void)arg;
    int mdns_sock = mdns_open_socket();
    uint32_t last_pub_ms = 0;

    for (;;) {
        /* Despertar inmediatamente en cuanto el parser cierra una epoca PVT */
        EventBits_t ev = xEventGroupWaitBits(
            gps_data_events(),
            EVT_EPOCH_READY,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(35)
        );

        uint32_t now = gps_now_ms();
        bool epoch_ready = (ev & EVT_EPOCH_READY) != 0;

        if (epoch_ready) {
            /* Dar 20 ms para que terminen de entrar las sentencias NMEA/PUBX
             * finales de esta misma rafaga y medir el tiempo total exacto */
            vTaskDelay(pdMS_TO_TICKS(20));
            now = gps_now_ms();
        }

        if (epoch_ready || last_pub_ms == 0 || (now - last_pub_ms) >= GPS_JSON_PERIOD_MS) {
            last_pub_ms = now;
            s_json_seq++;

            /* Copiar de forma segura bajo el Mutex del repositorio GPS */
            gps_data_snapshot(&s_wifi_snap);

            /* Serializar bajo el Mutex del buffer JSON midiendo microsegundos */
            xSemaphoreTake(s_json_mtx, portMAX_DELAY);
            uint64_t t0_us = (uint64_t)esp_timer_get_time();
            s_json_len = serialize_repo_to_json(&s_wifi_snap, s_json_buf,
                                                sizeof(s_json_buf), s_json_seq);
            s_last_json_us = (uint32_t)((uint64_t)esp_timer_get_time() - t0_us);
            s_json_updated_ms = now;
            /* Emitir de inmediato a clientes SSE (/events) y WebSocket (/ws) */
            broadcast_json_to_streams(s_json_buf, s_json_len);
            xSemaphoreGive(s_json_mtx);
        }

        /* Atender consultas mDNS (timeout de socket = 15 ms) */
        if (mdns_sock >= 0) {
            mdns_poll_and_reply(mdns_sock);
        }
    }
}

esp_err_t gps_wifi_init_and_start(void)
{
    s_json_mtx = xSemaphoreCreateMutexStatic(&s_json_mtx_buf);
    stream_lists_init();

    /* 1. Inicializar NVS requerido por el driver de calibracion RF WiFi */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }

    /* 2. Inicializar pila TCP/IP y unica interfaz STA (sin Access Point) */
    esp_netif_init();
    esp_event_loop_create_default();
    s_netif_sta = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(s_netif_sta, GPS_MDNS_HOSTNAME);

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wcfg);
    if (err != ESP_OK) {
        return err;
    }

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    /* Configurar exclusivamente STA con soporte WPA3-Personal (SAE + PMF) */
    wifi_config_t sta_cfg;
    memset(&sta_cfg, 0, sizeof(sta_cfg));
    snprintf((char *)sta_cfg.sta.ssid, sizeof(sta_cfg.sta.ssid), "%.31s", GPS_WIFI_STA_SSID);
    snprintf((char *)sta_cfg.sta.password, sizeof(sta_cfg.sta.password), "%.63s", GPS_WIFI_STA_PASS);
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    sta_cfg.sta.pmf_cfg.capable    = true;
    sta_cfg.sta.pmf_cfg.required   = false;
    sta_cfg.sta.sae_pwe_h2e        = WPA3_SAE_PWE_BOTH;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    esp_wifi_set_ps(WIFI_PS_NONE); /* Sin ahorro de energia para baja latencia HTTP/WS */
    esp_wifi_start();

    /* Generar primer snapshot JSON antes de abrir los endpoints */
    gps_data_snapshot(&s_wifi_snap);
    s_json_len = serialize_repo_to_json(&s_wifi_snap, s_json_buf, sizeof(s_json_buf), 0);

    start_http_server();
    gps_wifi_print_network_banner();

    xTaskCreatePinnedToCore(
        gps_wifi_telemetry_task,
        "gps_wifi_10s",
        TASK_WIFI_STACK,
        NULL,
        TASK_WIFI_PRIO,
        NULL,
        TASK_WIFI_CORE
    );

    return ESP_OK;
}
