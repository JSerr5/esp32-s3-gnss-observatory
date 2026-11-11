/*
 * nmea_parser.c
 * ---------------------------------------------------------------------------
 * Agente 3 - Implementacion del decodificador NMEA 0183 / PUBX.
 * ---------------------------------------------------------------------------
 */
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "nmea_parser.h"

enum { NS_IDLE = 0, NS_BODY, NS_CK };

void nmea_parser_init(nmea_parser_t *p)
{
    memset(p, 0, sizeof(*p));
}

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

nmea_feed_t nmea_parser_feed(nmea_parser_t *p, char c)
{
    if (c == '$') {                      /* inicio (o reinicio) de sentencia */
        p->state = NS_BODY;
        p->len = 0;
        p->xor_calc = 0;
        return NMEA_FEED_NONE;
    }
    switch (p->state) {
    case NS_IDLE:
        break;

    case NS_BODY:
        if (c == '*') {
            p->line[p->len] = '\0';
            p->state = NS_CK;
            p->hexcnt = 0;
            p->xor_rx = 0;
        } else if ((unsigned char)c < 0x20 || (unsigned char)c > 0x7E) {
            /* CR/LF sin checksum o byte binario: sentencia descartada */
            p->state = NS_IDLE;
            p->overflow++;
            return NMEA_FEED_OVERFLOW;
        } else {
            if (p->len >= NMEA_MAX_LINE - 1) {
                p->state = NS_IDLE;
                p->overflow++;
                return NMEA_FEED_OVERFLOW;
            }
            p->line[p->len++] = c;
            p->xor_calc ^= (uint8_t)c;
        }
        break;

    case NS_CK: {
        int v = hexval(c);
        if (v < 0) {
            p->state = NS_IDLE;
            p->ckerr++;
            return NMEA_FEED_CKERR;
        }
        p->xor_rx = (uint8_t)((p->xor_rx << 4) | (uint8_t)v);
        if (++p->hexcnt == 2) {
            p->state = NS_IDLE;
            if (p->xor_rx == p->xor_calc) {
                p->ok++;
                return NMEA_FEED_SENTENCE;
            }
            p->ckerr++;
            return NMEA_FEED_CKERR;
        }
        break;
    }
    default:
        p->state = NS_IDLE;
        break;
    }
    return NMEA_FEED_NONE;
}

/* Tokenizador seguro: a diferencia de strtok, ",," produce un campo vacio */
int nmea_tokenize(nmea_parser_t *p, char **fields, int max_fields)
{
    memcpy(p->work, p->line, (size_t)p->len + 1);
    int n = 0;
    char *s = p->work;
    fields[n++] = s;
    while (*s && n < max_fields) {
        if (*s == ',') {
            *s = '\0';
            fields[n++] = s + 1;
        }
        s++;
    }
    return n;
}

/* ------------------------- utilidades de campo ------------------------- */
static inline bool fe(const char *s) { return s == NULL || s[0] == '\0'; }

static bool fld_d(const char *s, double *out)
{
    if (fe(s)) return false;
    char *end = NULL;
    double v = strtod(s, &end);
    if (end == s) return false;
    *out = v;
    return true;
}

static bool fld_f(const char *s, float *out)
{
    double d;
    if (!fld_d(s, &d)) return false;
    *out = (float)d;
    return true;
}

static bool fld_i(const char *s, int *out)
{
    if (fe(s)) return false;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s) return false;
    *out = (int)v;
    return true;
}

static int fld_i_or(const char *s, int def)
{
    int v;
    return fld_i(s, &v) ? v : def;
}

static char fld_c(const char *s)
{
    return fe(s) ? '\0' : s[0];
}

/* hhmmss.sss */
static void fld_time(const char *s, nmea_time_t *t)
{
    t->valid = false;
    if (fe(s) || strlen(s) < 6) return;
    for (int i = 0; i < 6; i++) {
        if (!isdigit((unsigned char)s[i])) return;
    }
    t->hh = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
    t->mm = (uint8_t)((s[2] - '0') * 10 + (s[3] - '0'));
    t->ss = (float)strtod(s + 4, NULL);
    t->valid = (t->hh < 24 && t->mm < 60 && t->ss < 61.0f);
}

/* ddmmyy */
static void fld_date(const char *s, nmea_date_t *d)
{
    d->valid = false;
    if (fe(s) || strlen(s) < 6) return;
    for (int i = 0; i < 6; i++) {
        if (!isdigit((unsigned char)s[i])) return;
    }
    d->day   = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
    d->month = (uint8_t)((s[2] - '0') * 10 + (s[3] - '0'));
    d->year  = (uint16_t)(2000 + (s[4] - '0') * 10 + (s[5] - '0'));
    d->valid = (d->day >= 1 && d->day <= 31 && d->month >= 1 && d->month <= 12);
}

/* (d)ddmm.mmmmm + hemisferio -> grados decimales con signo */
static bool fld_latlon(const char *v, const char *hemi, double *out)
{
    double raw;
    if (!fld_d(v, &raw) || fe(hemi)) return false;
    double deg = (double)((long)(raw / 100.0));
    double min = raw - deg * 100.0;
    double res = deg + min / 60.0;
    if (hemi[0] == 'S' || hemi[0] == 'W') res = -res;
    *out = res;
    return true;
}

static const char *F(char **f, int n, int i)
{
    return (i < n) ? f[i] : "";
}

/* ------------------------- identificacion ------------------------------ */
static const char *s_names[NMEA_S_COUNT] = {
    "???", "GGA", "GLL", "GSA", "GSV", "RMC", "VTG", "GRS", "GST", "ZDA",
    "GBS", "DTM", "TXT", "PUBX,00", "PUBX,03", "PUBX,04", "PUBX,??"
};

const char *nmea_sentence_name(nmea_sentence_t s)
{
    return (s < NMEA_S_COUNT) ? s_names[s] : "???";
}

nmea_sentence_t nmea_identify(char **f, int n, char talker[3])
{
    talker[0] = talker[1] = talker[2] = '\0';
    if (n < 1) return NMEA_S_UNKNOWN;
    const char *a = f[0];
    if (strcmp(a, "PUBX") == 0) {
        talker[0] = 'P'; talker[1] = 'U';
        int id = fld_i_or(F(f, n, 1), -1);
        if (id == 0) return NMEA_S_PUBX00;
        if (id == 3) return NMEA_S_PUBX03;
        if (id == 4) return NMEA_S_PUBX04;
        return NMEA_S_PUBX_OTHER;
    }
    size_t L = strlen(a);
    if (L != 5) return NMEA_S_UNKNOWN;
    talker[0] = a[0];
    talker[1] = a[1];
    const char *t = a + 2;
    if (!strcmp(t, "GGA")) return NMEA_S_GGA;
    if (!strcmp(t, "GLL")) return NMEA_S_GLL;
    if (!strcmp(t, "GSA")) return NMEA_S_GSA;
    if (!strcmp(t, "GSV")) return NMEA_S_GSV;
    if (!strcmp(t, "RMC")) return NMEA_S_RMC;
    if (!strcmp(t, "VTG")) return NMEA_S_VTG;
    if (!strcmp(t, "GRS")) return NMEA_S_GRS;
    if (!strcmp(t, "GST")) return NMEA_S_GST;
    if (!strcmp(t, "ZDA")) return NMEA_S_ZDA;
    if (!strcmp(t, "GBS")) return NMEA_S_GBS;
    if (!strcmp(t, "DTM")) return NMEA_S_DTM;
    if (!strcmp(t, "TXT")) return NMEA_S_TXT;
    return NMEA_S_UNKNOWN;
}

/* ------------------------------- GGA ----------------------------------- */
bool nmea_parse_gga(char **f, int n, nmea_gga_t *o)
{
    if (n < 15) return false;
    memset(o, 0, sizeof(*o));
    o->talker[0] = f[0][0];
    o->talker[1] = f[0][1];
    fld_time(f[1], &o->time);
    double la, lo;
    o->pos_valid = fld_latlon(f[2], f[3], &la) && fld_latlon(f[4], f[5], &lo);
    if (o->pos_valid) { o->lat = la; o->lon = lo; }
    o->quality = fld_i_or(f[6], 0);
    o->numSV   = fld_i_or(f[7], 0);
    o->hdop_valid = fld_f(f[8], &o->hdop);
    o->alt_valid  = fld_f(f[9], &o->alt_msl);
    o->sep_valid  = fld_f(f[11], &o->geoid_sep);
    o->dgps_age_valid = fld_f(f[13], &o->dgps_age);
    o->dgps_sta_valid = fld_i(f[14], &o->dgps_station);
    return true;
}

/* ------------------------------- GLL ----------------------------------- */
bool nmea_parse_gll(char **f, int n, nmea_gll_t *o)
{
    if (n < 7) return false;
    memset(o, 0, sizeof(*o));
    double la, lo;
    o->pos_valid = fld_latlon(f[1], f[2], &la) && fld_latlon(f[3], f[4], &lo);
    if (o->pos_valid) { o->lat = la; o->lon = lo; }
    fld_time(f[5], &o->time);
    o->status = fld_c(f[6]);
    o->mode   = fld_c(F(f, n, 7));
    return true;
}

/* ------------------------------- GSA ----------------------------------- */
bool nmea_parse_gsa(char **f, int n, nmea_gsa_t *o)
{
    if (n < 18) return false;
    memset(o, 0, sizeof(*o));
    o->opmode  = fld_c(f[1]);
    o->fixmode = fld_i_or(f[2], 1);
    for (int i = 0; i < 12; i++) {
        int prn;
        if (fld_i(f[3 + i], &prn) && prn > 0) {
            o->prn[o->nprn++] = prn;
        }
    }
    o->pdop_valid = fld_f(f[15], &o->pdop);
    o->hdop_valid = fld_f(f[16], &o->hdop);
    o->vdop_valid = fld_f(f[17], &o->vdop);
    return true;
}

/* ------------------------------- GSV ----------------------------------- */
bool nmea_parse_gsv(char **f, int n, nmea_gsv_t *o)
{
    if (n < 4) return false;
    memset(o, 0, sizeof(*o));
    o->total_msgs   = fld_i_or(f[1], 0);
    o->msg_num      = fld_i_or(f[2], 0);
    o->sats_in_view = fld_i_or(f[3], 0);
    for (int b = 0; b < 4; b++) {
        int base = 4 + b * 4;
        if (base >= n) break;
        int prn;
        if (!fld_i(f[base], &prn)) continue;
        nmea_gsv_sat_t *s = &o->blk[o->nblk++];
        s->prn  = prn;
        s->elev = fld_i_or(F(f, n, base + 1), -1);
        s->azim = fld_i_or(F(f, n, base + 2), -1);
        s->cno  = fld_i_or(F(f, n, base + 3), -1);
    }
    return true;
}

/* ------------------------------- RMC ----------------------------------- */
bool nmea_parse_rmc(char **f, int n, nmea_rmc_t *o)
{
    if (n < 12) return false;
    memset(o, 0, sizeof(*o));
    fld_time(f[1], &o->time);
    o->status = fld_c(f[2]);
    double la, lo;
    o->pos_valid = fld_latlon(f[3], f[4], &la) && fld_latlon(f[5], f[6], &lo);
    if (o->pos_valid) { o->lat = la; o->lon = lo; }
    o->sog_valid = fld_f(f[7], &o->sog_kn);
    o->cog_valid = fld_f(f[8], &o->cog);
    fld_date(f[9], &o->date);
    o->magvar_empty = fe(f[10]);
    if (!o->magvar_empty) fld_f(f[10], &o->magvar);
    o->magdir_empty = fe(f[11]);
    o->magdir = fld_c(f[11]);
    o->mode   = fld_c(F(f, n, 12));
    return true;
}

/* ------------------------------- VTG ----------------------------------- */
bool nmea_parse_vtg(char **f, int n, nmea_vtg_t *o)
{
    if (n < 9) return false;
    memset(o, 0, sizeof(*o));
    o->cog_t_valid  = fld_f(f[1], &o->cog_true);
    o->cog_m_empty  = fe(f[3]);
    if (!o->cog_m_empty) fld_f(f[3], &o->cog_mag);
    o->sog_kn_valid  = fld_f(f[5], &o->sog_kn);
    o->sog_kmh_valid = fld_f(f[7], &o->sog_kmh);
    o->mode = fld_c(F(f, n, 9));
    return true;
}

/* ------------------------------- GRS ----------------------------------- */
bool nmea_parse_grs(char **f, int n, nmea_grs_t *o)
{
    if (n < 3) return false;
    memset(o, 0, sizeof(*o));
    fld_time(f[1], &o->time);
    o->mode = fld_i_or(f[2], 0);
    for (int i = 0; i < 12 && (3 + i) < n; i++) {
        o->res_valid[i] = fld_f(f[3 + i], &o->res[i]);
        if (o->res_valid[i]) o->nres = i + 1;
    }
    return true;
}

/* ------------------------------- GST ----------------------------------- */
bool nmea_parse_gst(char **f, int n, nmea_gst_t *o)
{
    if (n < 9) return false;
    memset(o, 0, sizeof(*o));
    fld_time(f[1], &o->time);
    o->rms_valid    = fld_f(f[2], &o->rangeRms);
    o->major_empty  = !fld_f(f[3], &o->stdMajor);
    o->minor_empty  = !fld_f(f[4], &o->stdMinor);
    o->orient_empty = !fld_f(f[5], &o->orient);
    o->lat_valid    = fld_f(f[6], &o->stdLat);
    o->lon_valid    = fld_f(f[7], &o->stdLon);
    o->alt_valid    = fld_f(f[8], &o->stdAlt);
    return true;
}

/* ------------------------------- ZDA ----------------------------------- */
bool nmea_parse_zda(char **f, int n, nmea_zda_t *o)
{
    if (n < 5) return false;
    memset(o, 0, sizeof(*o));
    fld_time(f[1], &o->time);
    int d, m, y;
    if (fld_i(f[2], &d) && fld_i(f[3], &m) && fld_i(f[4], &y)) {
        o->date.day = (uint8_t)d;
        o->date.month = (uint8_t)m;
        o->date.year = (uint16_t)y;
        o->date.valid = (d >= 1 && d <= 31 && m >= 1 && m <= 12 && y >= 1980);
    }
    int lh, lm;
    if (fld_i(F(f, n, 5), &lh) && fld_i(F(f, n, 6), &lm)) {
        o->ltz_valid = true;
        o->ltz_h = lh;
        o->ltz_m = lm;
    }
    return true;
}

/* ------------------------------- GBS ----------------------------------- */
bool nmea_parse_gbs(char **f, int n, nmea_gbs_t *o)
{
    if (n < 9) return false;
    memset(o, 0, sizeof(*o));
    fld_time(f[1], &o->time);
    o->errLat_valid = fld_f(f[2], &o->errLat);
    o->errLon_valid = fld_f(f[3], &o->errLon);
    o->errAlt_valid = fld_f(f[4], &o->errAlt);
    o->svid_valid   = fld_i(f[5], &o->svid);
    o->prob_valid   = fld_f(f[6], &o->prob);
    o->bias_valid   = fld_f(f[7], &o->bias);
    o->stddev_valid = fld_f(f[8], &o->stddev);
    return true;
}

/* ------------------------------- DTM ----------------------------------- */
static void copy_tok(char *dst, size_t cap, const char *src)
{
    size_t i = 0;
    if (src) {
        for (; i + 1 < cap && src[i]; i++) dst[i] = src[i];
    }
    dst[i] = '\0';
}

bool nmea_parse_dtm(char **f, int n, nmea_dtm_t *o)
{
    if (n < 9) return false;
    memset(o, 0, sizeof(*o));
    copy_tok(o->datum, sizeof(o->datum), f[1]);
    copy_tok(o->subdatum, sizeof(o->subdatum), f[2]);
    float v;
    if (fld_f(f[3], &v)) o->lat_ofs_min = (fld_c(f[4]) == 'S') ? -v : v;
    if (fld_f(f[5], &v)) o->lon_ofs_min = (fld_c(f[6]) == 'W') ? -v : v;
    if (fld_f(f[7], &v)) o->alt_ofs_m = v;
    copy_tok(o->ref_datum, sizeof(o->ref_datum), f[8]);
    return true;
}

/* ------------------------------- TXT ----------------------------------- */
bool nmea_parse_txt(char **f, int n, nmea_txt_t *o)
{
    if (n < 5) return false;
    memset(o, 0, sizeof(*o));
    o->num_msg  = fld_i_or(f[1], 0);
    o->msg_num  = fld_i_or(f[2], 0);
    o->msg_type = fld_i_or(f[3], 0);
    /* el texto puede contener comas: reunir el resto de campos */
    size_t pos = 0;
    for (int i = 4; i < n && pos + 1 < sizeof(o->text); i++) {
        if (i > 4 && pos + 1 < sizeof(o->text)) o->text[pos++] = ',';
        for (const char *s = f[i]; *s && pos + 1 < sizeof(o->text); s++) {
            o->text[pos++] = *s;
        }
    }
    o->text[pos] = '\0';
    return true;
}

/* ----------------------------- PUBX,00 --------------------------------- */
bool nmea_parse_pubx00(char **f, int n, nmea_pubx00_t *o)
{
    if (n < 21) return false;
    memset(o, 0, sizeof(*o));
    fld_time(f[2], &o->time);
    double la, lo;
    o->pos_valid = fld_latlon(f[3], f[4], &la) && fld_latlon(f[5], f[6], &lo);
    if (o->pos_valid) { o->lat = la; o->lon = lo; }
    fld_f(f[7], &o->altRef);
    copy_tok(o->navStat, sizeof(o->navStat), f[8]);
    fld_f(f[9], &o->hAcc);
    fld_f(f[10], &o->vAcc);
    fld_f(f[11], &o->sog_kmh);
    fld_f(f[12], &o->cog);
    fld_f(f[13], &o->vVel);
    o->diffAge_valid = fld_f(f[14], &o->diffAge);
    fld_f(f[15], &o->hdop);
    fld_f(f[16], &o->vdop);
    fld_f(f[17], &o->tdop);
    o->gu = fld_i_or(f[18], 0);
    o->ru = fld_i_or(f[19], 0);
    o->dr = fld_i_or(f[20], 0);
    return true;
}

/* ----------------------------- PUBX,03 --------------------------------- */
bool nmea_parse_pubx03(char **f, int n, nmea_pubx03_t *o)
{
    if (n < 3) return false;
    memset(o, 0, sizeof(*o));
    o->n = fld_i_or(f[2], 0);
    for (int i = 0; i < o->n && o->nblk < NMEA_MAX_SATS; i++) {
        int base = 3 + i * 6;
        if (base + 5 >= n) break;          /* bloque incompleto */
        nmea_pubx03_sat_t *s = &o->sat[o->nblk++];
        s->sv     = fld_i_or(f[base], 0);
        s->status = fld_c(f[base + 1]);
        s->azim   = fld_i_or(f[base + 2], -1);
        s->elev   = fld_i_or(f[base + 3], -1);
        s->cno    = fld_i_or(f[base + 4], 0);
        s->lck    = fld_i_or(f[base + 5], 0);
    }
    return true;
}

/* ----------------------------- PUBX,04 --------------------------------- */
bool nmea_parse_pubx04(char **f, int n, nmea_pubx04_t *o)
{
    if (n < 10) return false;
    memset(o, 0, sizeof(*o));
    fld_time(f[2], &o->time);
    fld_date(f[3], &o->date);
    fld_d(f[4], &o->utcTow);
    o->utcWk = fld_i_or(f[5], 0);
    /* leapSec: "15D" = valor por defecto de ROM; "18" = verificado */
    const char *ls = f[6];
    o->leapSec = fld_i_or(ls, 0);
    o->leap_default = (!fe(ls) && strchr(ls, 'D') != NULL);
    int cb = 0;
    if (fld_i(f[7], &cb)) o->clkBias = cb;
    fld_f(f[8], &o->clkDrift);
    o->tpGran = fld_i_or(f[9], 0);
    return true;
}
